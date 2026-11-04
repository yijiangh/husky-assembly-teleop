"""Punch: optional, the old procedure's last check: a punch tip onto one fixed target from several base spots."""

from __future__ import annotations

from dataclasses import replace
from typing import TYPE_CHECKING

import numpy as np
import viser

from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.kinematics import link_pose

from ....shared.guide.both_pages import on_both
from ....ui.phone import phone_text
from ....ui.style import check_chip, hover, indented
from ....world.checks import BAD, PENDING, WARN, Check
from ..analysis.checks import (MARKER_ERROR_LIMIT, MARKER_ERROR_WARN, marker_error_level, punch_as_taken_check,
                               punch_checks, sample_marker_error, tcp_offset_check)
from ..analysis.old_outputs import write_punch_outputs
from ..robot.sweeps import named
from ..storage.session_files import punch_take
from ..views.takes_view import chip_row, marker_chip
from .base import CalibrationStep, lab_frame_check, punch_tool_check, relay_sha, relay_up_check

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext

#: What puts punch takes out of date (`Run.punch_current`).
OUT_OF_DATE_BY = "a base refine, a remount, a target move or a lab frame move"


class Punch(CalibrationStep):
    """The punch verdict, built from the punch takes judged under the current solution and TCP offset; needs Setup.

    Takes are current while the base body's definition had no data change, the punch not remounted, the target not
    moved and the lab frame the same (`Run.punch_current`): new solutions, offsets and Apply keep them. It measures, so
    it needs no marker precondition: the base marker error shows live, and a take made while it was red is flagged in
    the verdict.
    """

    name = "Punch"
    instructions = ("Optional, the old procedure's last check, with the TCP offset from Tip offset (or entered in "
                    "mm), onto the target fixed in Tip offset. Per take: drive to a new spot, push the tip near the "
                    "target while holding Free drive (hold) (or with Keep free drive on), put it on the target with "
                    "the tool pad, then Record punch take. Target moved if it did.")
    drives = goes = cancels = moves_by_hand = moves_robot = True
    needs = ("Setup",)
    on_laptop = "The TCP offset is entered on the laptop."

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """The TCP offset, the D-pad, Free drive, the tool pad, Record, Target moved, Clear; the readout below."""
        self._offset = gui.add_vector3("TCP offset (mm)", initial_value=(0.0, 0.0, 0.0), step=0.01,
                                       hint="tool0 to the punch tip, in mm: from Tip offset, or entered here.")
        self.build_movers(ctx, gui)
        self.build_robot_buttons(ctx, gui)
        moved = gui.add_button("Target moved", icon=viser.Icon.ARROWS_MOVE,
                               hint="The target moved since the takes: they go out of date; the next take starts "
                                    "a new file.")
        clear = gui.add_button("Clear punch takes", icon=viser.Icon.TRASH,
                               hint="The next take starts a new file; this one goes to archive/.")
        # * The readout last: its lines come and go, and must not move the buttons. The base body's current marker
        # * error has a handle of its own, after the chips: it changes at mocap rate, which would drop their hover.
        self._html = gui.add_html("")
        self._live_html = gui.add_html("")
        self._offset.on_update(ctx.defer_value("punch offset", self._on_offset, operator_only=True))
        moved.on_click(ctx.defer("target moved", self._target_moved))
        clear.on_click(ctx.defer("clear punch", self._clear))

    def build_robot_buttons(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Record punch take: used next to the robot, on both pages."""
        record = gui.add_button("Record punch take", icon=viser.Icon.PLAYER_RECORD,
                                hint="Once the tip touches the target and the base stands still.")
        record.on_click(ctx.defer("record punch", lambda: self.start(ctx, "punch take", self._record(ctx))))

    def build_phone(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """As every step on the phone, and the base body's current marker error below the readout."""
        super().build_phone(ctx, gui)
        self._live_html = on_both(self._live_html, gui.add_html(""), phone_text)

    def run_changed(self, ctx: PluginContext) -> None:
        """Show the run's TCP offset."""
        self._show_offset()

    def enter(self, ctx: PluginContext) -> None:
        """Drive and move by hand; show the TCP offset, which Tip offset may have changed."""
        super().enter(ctx)
        self._show_offset()

    def _show_offset(self) -> None:
        self._offset.value = tuple(float(v) * 1e3 for v in self.run.punch_offset)

    def _on_offset(self, value) -> None:
        self.run.punch_offset = np.array(value, dtype=float) / 1e3
        self.run.save()

    def _arm_name(self) -> tuple[str, int]:
        """The arm as the old monitor named it in punch files: "single", or "left"/"right"; and its index."""
        run = self.run
        names = [arm.name for arm in run.robot.config.arms]
        index = names.index(run.arm)
        return ("single" if len(names) == 1 else "left" if run.arm.startswith("left") else "right"), index

    async def _record(self, ctx: PluginContext) -> None:
        """Wait until the base and joints stand still, average them, and keep a take with the base body's own pose."""
        run = self.run
        offset = run.punch_offset
        footprint, here = await self.cal.motion.still_arm(ctx, run, "punch take")
        tool0 = f"{run.arm}_tool0"
        tip = compose(link_pose(run.urdf, footprint, named(run.arm, here), tool0), Pose(tuple(offset)))
        name, index = self._arm_name()
        take = punch_take(name, index, tool0, here, footprint, tip, offset, run.applied_calibration() or Pose(),
                          relay_sha(run) or "", run.robot.base.state.marker_error, run.lab_frame())
        file = run.add_punch_take(take, name)
        ctx.log_info(f"punch take {len(run.punch_takes)}: tip at {np.round(tip.position, 4).tolist()} in {file}")
        run.save(start=True)
        ctx.spawn("punch outputs", self.cal.results.write_and_show(ctx, run, write_punch_outputs, run.session, index),
                  survives_soft_stop=True)

    def _target_moved(self) -> None:
        if self.busy("target move not noted"):
            return
        self.run.target_moved()
        self.run.save()

    def _clear(self) -> None:
        if self.busy("punch takes not cleared"):
            return
        self.run.clear_punch()
        self.run.save()

    def none_yet(self, ctx: PluginContext) -> str | None:
        """No punch take yet."""
        return "no punch take yet" if not self.run.punch_takes else None

    def out_of_date(self, ctx: PluginContext) -> str | None:
        """Every take from before a base refine, a remount, a target move or a lab frame move."""
        return (f"taken before {OUT_OF_DATE_BY}" if not self.run.current_punch_takes() else None)

    def judged_under(self, ctx: PluginContext) -> Pose:
        """The base_footprint the verdict is judged under: the current solution's, else the relay's calibration."""
        return self.run.footprint()

    def checks(self, ctx: PluginContext) -> list[Check]:
        """The current takes under the current solution and TCP offset: enough, tips together, turned; and as taken."""
        run = self.run
        takes = run.current_punch_takes()
        checks = punch_checks(*run.punch_tips(run.footprint()))
        detail = checks[0].detail
        if len(takes) < len(run.punch_takes):
            detail += f"; {len(run.punch_takes) - len(takes)} older out of date (the next take starts a new file)"
        red = sum(marker_error_level(sample_marker_error(take.get("marker_errors"))) == BAD for take in takes)
        if red:
            detail += f"; {red} taken with the base marker error over {MARKER_ERROR_LIMIT * 1e3:g} mm (see markers)"
        checks[0] = replace(checks[0], detail=hover(detail, "the markers row"))
        as_taken = np.array([take["world_from_punch_tip"]["position"] for take in takes]).reshape(-1, 3)
        return checks + [punch_as_taken_check(as_taken)]

    def ready(self, ctx: PluginContext) -> list[Check]:
        """Setup's checklist, the safety boxes, a TCP offset, the relay up, the lab frame, a bare arm, a zero TCP.

        The lab frame because every take's tip is compared with the others in it.
        """
        run = self.run
        frame = lab_frame_check(run)
        if run.lab_frame() is None:
            # * Red here, grey in Setup: a take stores the frame it was made in, and one without would stay current
            # * once the frame moves.
            frame = replace(frame, level=BAD, detail=f"{frame.detail} Each punch take stores the frame its tip is in.")
        return [*self.needs_checks(ctx), tcp_offset_check(run.punch_offset), relay_up_check(run, "relay applies"),
                frame, punch_tool_check(run), self.cal.tcp_check(ctx, run)]

    def live_markers(self) -> Check:
        """The base body's marker error now: red over MARKER_ERROR_LIMIT; it only warns, a take is still recorded.

        No number: the chip shares its handle with the take chips; the number has a handle of its own (`draw`).
        """
        error = self.run.robot.base.state.marker_error
        if error is None:
            return Check("base markers now", PENDING, "no marker error from the relay")
        level = marker_error_level(error)
        detail = {BAD: f"over {MARKER_ERROR_LIMIT * 1e3:g} mm: a take now is flagged; look at the base markers",
                  WARN: f"over {MARKER_ERROR_WARN * 1e3:g} mm: a take now still counts"}.get(
                      level, f"within {MARKER_ERROR_WARN * 1e3:g} mm")
        return Check("base markers now", level, f"the base body's marker error now: {detail}")

    def draw(self, ctx: PluginContext) -> None:
        """The base marker error now and each take's; how many takes, the tips from their mean now and as taken."""
        run = self.run
        under = "the current solution" if run.current_solved() is not None else "the calibration the relay applies"
        tips, _bases = run.punch_tips(run.footprint())
        taken = np.array([take["world_from_punch_tip"]["position"] for take in run.current_punch_takes()])
        lines = [f"{len(tips)} takes" + (f" in {run.punch_file.name}" if run.punch_file else "")]
        for what, points in ((f"under {under} and the TCP offset", tips), ("as taken", taken)):
            distances = np.linalg.norm(points - points.mean(axis=0), axis=1) * 1e3 if len(points) else []
            lines.append(f"tips from their mean {what}: " + (", ".join(f"{d:.2f}" for d in distances) + " mm"
                                                             if len(points) else "—"))
        markers = "".join(marker_chip(f"take {number}", [take.get("marker_errors")])
                          for number, take in enumerate(run.current_punch_takes(), start=1))
        self._html.content = (chip_row("markers", check_chip(self.live_markers()) + markers)
                              + indented(*lines))
        error = run.robot.base.state.marker_error
        self._live_html.content = indented(f"base marker error now: {error * 1e3:.2f} mm" if error is not None else
                                           "base marker error now: -")
