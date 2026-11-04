"""Sweeps: the arm planner checks every predefined take for collisions where the robot stands."""

from __future__ import annotations

from dataclasses import replace
from typing import TYPE_CHECKING

import numpy as np
import viser

from ....shared.arm_planner_contract import ArmPlannerContract
from ....shared.guide.panel import set_changed
from ....world.checks import GOOD, PENDING, WARN, Check
from ....ui.style import few, indented
from ..analysis.checks import take_count_check
from ....world.clearance import PLAN_DISTANCE
from ..views.takes_view import take_index, take_label
from .base import CalibrationStep

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext

#: Why Record cannot go on: every take collides where the base stands.
NONE_CLEAR = "no take clear here"


class Sweeps(CalibrationStep):
    """Precondition: the takes' collision checks, holding while the base stands where they were checked; needs nothing.

    A colliding take is skipped. They are checked only when Check collisions is pressed.
    """

    name = "Sweeps"
    precondition = True
    watches = ("base",)
    instructions = "Press Check collisions, then leave the robot where it stands until every take is recorded."
    on_laptop = "The hits and the take preview show in 3D on the laptop."

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Check collisions, and the take and waypoint to preview; the take chips below them."""
        self.build_robot_buttons(ctx, gui)
        self._preview_take = gui.add_dropdown("Preview take", ["-"])
        self._preview_point = gui.add_slider("Waypoint", min=1, max=2, step=1, initial_value=1)
        # * The readout last: its lines come and go, and must not move the buttons.
        self._html = gui.add_html("")

    def build_robot_buttons(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Check collisions: used next to the robot after moving it, on both pages."""
        check = gui.add_button("Check collisions", icon=viser.Icon.SHIELD_CHECK)
        check.on_click(ctx.defer("check collisions", lambda: self.start(ctx, "check", self._check_collisions(ctx))))

    def run_changed(self, ctx: PluginContext) -> None:
        """Offer the run's takes to preview."""
        labels = [take_label(index, sweep) for index, sweep in enumerate(self.run.sweeps.sweeps)]
        keep = self._preview_take.value if self._preview_take.value in labels else labels[0]
        self._preview_take.options, self._preview_take.value = labels, keep

    def leave(self, ctx: PluginContext) -> None:
        """Hide the takes."""
        self.cal.takes.hide(ctx)

    async def _check_collisions(self, ctx: PluginContext) -> None:
        """Ask the arm planner to check every sweep, in the world as it is now; list the hits in the collisions tab.

        The base pose is kept only once every sweep is checked: a check cancelled half-way holds for nothing.
        """
        planner: ArmPlannerContract = ctx.require("arm_planner")
        run, procedure = self.run, self.cal.procedure
        base = ctx.kinematics.base_pose(run.serial)
        run.collisions, run.collision_base = {}, None
        hits = {}
        for index, sweep in enumerate(run.sweeps.sweeps):
            procedure.doing = f"checking take {index + 1} of {len(run.sweeps.sweeps)}"
            try:
                hit = await planner.check_path(run.serial, run.arm, sweep.waypoints(), PLAN_DISTANCE)
            except (ValueError, RuntimeError) as error:
                run.collisions[index] = str(error)
            else:
                run.collisions[index] = "" if hit is None else hit.message
                if hit is not None:
                    hits[f"take {index + 1}"] = hit
            if run.collisions[index]:
                ctx.log_info(f"take {index + 1} skipped: {run.collisions[index]}")
        run.collision_base = base
        self.cal.hits.replace(ctx.name, hits)
        run.save()

    def checks(self, ctx: PluginContext) -> list[Check]:
        """Enough takes clear per joint, grey until every take is checked; the take chips below name the colliding."""
        run = self.run
        if len(run.collisions) < len(run.sweeps.sweeps):
            return [Check("takes", PENDING, "not checked yet: press Check collisions")]
        colliding = [str(index + 1) for index, why in sorted(run.collisions.items()) if why]
        takes = take_count_check([len(run.usable(joint)) for joint in (0, 1)], "clear")
        return [replace(takes, detail=f"{takes.detail}; takes {few(colliding, 5)} collide: skipped")
                if colliding else takes]

    def none_yet(self, ctx: PluginContext) -> str | None:
        """Not every take checked yet (none, or a check cancelled half-way): press Check collisions."""
        return "press Check collisions" if len(self.run.collisions) < len(self.run.sweeps.sweeps) else None

    def ready(self, ctx: PluginContext) -> list[Check]:
        """Whether the collision checks hold where the base stands (`expired`): amber, as Record refuses."""
        why = self.expired(ctx)
        then = "drive elsewhere in Marker error, then" if why == NONE_CLEAR else ""
        return [Check("checked here", WARN, f"{why}: {then} press Check collisions".replace(":  ", ": ")) if why else
                Check("checked here", GOOD, "the base stands where the collisions were checked")]

    def expired(self, ctx: PluginContext) -> str | None:
        """Why the collision checks have expired for Record: not all checked, the base moved since, or none clear."""
        run = self.run
        return run.collisions_expired(ctx.kinematics.base_pose(run.serial)) or (
            None if run.usable() else NONE_CLEAR)

    def preview_joints(self, ctx: PluginContext) -> np.ndarray | None:
        """The previewed take at the previewed waypoint."""
        index = take_index(str(self._preview_take.value))
        if index is None:
            return None
        sweep = self.run.sweeps.sweeps[index]
        return sweep.waypoints()[min(int(self._preview_point.value), sweep.steps) - 1]

    def draw(self, ctx: PluginContext) -> None:
        """The take chips, the previewed take, and the takes coloured by their collision check."""
        run = self.run
        index = take_index(str(self._preview_take.value))
        sweep = run.sweeps.sweeps[index] if index is not None else None
        if sweep is not None:
            set_changed(self._preview_point, max=sweep.steps)
        detail = (f"take {index + 1}: J{sweep.joint} {sweep.start:+.2f} → {sweep.end:+.2f} rad, {sweep.steps} waypoints"
                  if sweep else "")
        self._html.content = (self.cal.takes.chips(run, range(len(run.sweeps.sweeps)), by_collisions=True)
                              + indented(detail))
        self.cal.takes.draw(ctx, run, by_collisions=True)
