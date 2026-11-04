"""Tip offset: optional, before Punch: the punch tip touches one fixed point from several tilts, giving its offset."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import viser

from ....shared.guide.step import fails
from ....ui.style import indented
from ....world.checks import GOOD, PENDING, WARN, Check
from .base import ACCEPTED, NOT_IN_USE, PASSES, CalibrationStep, punch_tool_check

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext


class TipOffset(CalibrationStep):
    """The tip fit, built from the tip poses since the last remount; Done commits it as the TCP offset; needs Setup.

    Shown again with poses, it asks whether the punch was remounted since: the monitor cannot see a remount.
    """

    name = "Tip offset"
    instructions = ("Optional, for Punch. Stop the monitor, swap the tool body for the punch, start it with the bare "
                    "arm, choose this run and tick Tool on flange again. Fix the target on the robot itself, where the "
                    "arm reaches it. For 4–6 poses: push the tip near the target while holding Free drive (hold), put "
                    "it on the target with the tool pad, then press Record tip pose. Tilt the punch 30° or more each "
                    "way about two axes. Done then makes the fit Punch's TCP offset.")
    goes = cancels = moves_by_hand = moves_robot = True
    #: The target is on the robot: the poses are in the robot's frame, from the joints alone.
    uses_mocap = False
    needs = ("Setup",)
    on_laptop = ""

    def __init__(self, cal):
        """Not asking about a remount."""
        super().__init__(cal)
        #: Whether the step asks if the punch was remounted since the poses shown (`enter`).
        self._asking = False

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Free drive, the tool pad, Record tip pose, Punch remounted, Clear tip poses; the readout."""
        self.build_movers(ctx, gui)
        self.build_robot_buttons(ctx, gui)
        clear = gui.add_button("Clear tip poses", icon=viser.Icon.TRASH,
                               hint="Start the tip offset again; Punch keeps the offset it has.")
        # * The readout last: its lines come and go, and must not move the buttons.
        self._html = gui.add_html("")
        clear.on_click(ctx.defer("clear tip poses", self._clear))

    def build_robot_buttons(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Record tip pose and Punch remounted: used next to the robot, on both pages."""
        record = gui.add_button("Record tip pose", icon=viser.Icon.PLAYER_RECORD,
                                hint="Once the tip touches the target and the arm stands still.")
        remounted = gui.add_button("Punch remounted", icon=viser.Icon.REFRESH,
                                   hint="The punch was taken off or moved on the flange since these poses: they and "
                                        "the punch takes go out of date.")
        record.on_click(ctx.defer("record tip pose", lambda: self.start(ctx, "tip pose", self._record(ctx))))
        remounted.on_click(ctx.defer("punch remounted", self._remounted))

    def enter(self, ctx: PluginContext) -> None:
        """Move by hand; with current poses, ask whether the punch was remounted since."""
        super().enter(ctx)
        self._asking = bool(self.run.current_tip())

    def leave(self, ctx: PluginContext) -> None:
        """Stop asking."""
        self._asking = False

    async def _record(self, ctx: PluginContext) -> None:
        """Wait until the arm stands still, and keep the pose: the punch was not remounted, then."""
        run = self.run
        # * The target is on the robot: the base may rock, the pose holds in the robot's frame.
        _base, joints = await self.cal.motion.still_arm(ctx, run, "tip pose", with_base=False)
        self._asking = False
        run.add_tip_pose(joints)
        run.save(start=True)
        pivot = run.tip_pivot()
        ctx.log_info(f"tip pose {len(run.tip_joints)}" + (
            f": tip offset {np.round(pivot.offset * 1e3, 2).tolist()} mm" if pivot is not None else ""))

    def done(self, ctx: PluginContext) -> None:
        """Commit the tip fit as Punch's TCP offset if it passes or is accepted."""
        run = self.run
        self._asking = False
        if not fails(run.tip_checks()) or run.accepted(self.name):
            run.commit_tip_offset()

    def _remounted(self) -> None:
        if self.busy("remount not noted"):
            return
        self._asking = False
        self.run.remounted()
        self.run.save()

    def _clear(self) -> None:
        if self.busy("tip poses not cleared"):
            return
        self._asking = False
        self.run.clear_tip()
        self.run.save()

    def none_yet(self, ctx: PluginContext) -> str | None:
        """No tip pose yet."""
        return "no tip pose yet" if not self.run.tip_joints else None

    def out_of_date(self, ctx: PluginContext) -> str | None:
        """Every pose taken before the punch was remounted."""
        return "the punch was remounted since these poses" if not self.run.current_tip() else None

    def state(self, ctx: PluginContext) -> Check:
        """As any result's; a fit that passes or is accepted but is not Punch's TCP offset, not in use (amber)."""
        state = super().state(ctx)
        if state.label in (PASSES, ACCEPTED) and self.run.tip_pivot() is not None and not self.run.tip_offset_used():
            return Check(NOT_IN_USE, WARN, "Punch uses another TCP offset: Done makes this fit Punch's")
        return state

    def question(self, ctx: PluginContext) -> str:
        """Shown again with poses: whether the punch was remounted since; Done, or a new pose, answers no."""
        if not self._asking:
            return ""
        return (f"Punch remounted since these {len(self.run.current_tip())} poses? If so, press Punch remounted: "
                f"they go out of date. If not, record more poses or press Done.")

    def checks(self, ctx: PluginContext) -> list[Check]:
        """Enough poses, tilted about two axes, tips together, the base still, a plausible offset; Punch using it."""
        run = self.run
        if run.tip_pivot() is None:
            used = Check("in use", PENDING, "no fit yet")
        elif run.tip_offset_used():
            used = Check("in use", GOOD, "Punch uses this offset")
        else:
            used = Check("in use", WARN, "Punch uses another TCP offset: Done makes this fit Punch's")
        return run.tip_checks() + [used]

    def ready(self, ctx: PluginContext) -> list[Check]:
        """Setup's checklist and the safety boxes, the arm bare in the model as for the punch, a zero TCP."""
        return [*self.needs_checks(ctx), punch_tool_check(self.run), self.cal.tcp_check(ctx, self.run)]

    def draw(self, ctx: PluginContext) -> None:
        """How many poses; the fit and each tip's distance from the fitted point: always 3 lines."""
        run = self.run
        pivot, current = run.tip_pivot(), len(run.current_tip())
        old = len(run.tip_joints) - current
        lines = [f"{current} poses" + (f" ({old} more before the punch was remounted: out of date)" if old else ""),
                 "tip offset —", "tips from the fitted point —"]
        if pivot is not None:
            lines[1] = (f"tip offset {', '.join(f'{v:.2f}' for v in pivot.offset * 1e3)} mm "
                        f"({np.linalg.norm(pivot.offset) * 1e3:.1f} mm from tool0)")
            lines[2] = "tips from the fitted point: " + ", ".join(f"{d:.2f}" for d in pivot.residuals * 1e3) + " mm"
        self._html.content = indented(*lines)
