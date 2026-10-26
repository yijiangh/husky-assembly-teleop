"""
The assembly_steps part of the Assembly tab: the step, Live IK, Plan, the preview, Exec, the settings, the plots.

The process panel's slider and ◀ mv / mv ▶ move through the movements; the run follows them, and this panel the run.

! Never hide a row or a button, and keep every changing line one line tall, so nothing moves under the cursor.
"""

from __future__ import annotations

import math
from html import escape
from typing import TYPE_CHECKING

from ...plugin_api.context import PluginContext
from ...shared.movement_exec_contract import (BASE_PART, BY_HAND_PART, GRASP_PART, HANDOFF_PART, INSERT_PART,
                                              MAX_TRAJ_TIME, MOTION_PART, OPEN_PART, RETREAT_PART, WITH_MOTION_PART,
                                              PartPlan)
from ...ui.assembly_tab import EXEC_BAND, EXEC_ICON, GOAL_ICON, PLAN_ICON, PLOTS_BAND, SETTINGS_BAND, STEP_BAND
from ...ui.phone import slim_labels_css
from ...ui.style import BUSY, NONE, OK, SECTION_CTRL, SECTION_TOOL, block, chip, chip_line, section, values
from .plan_plots import PlanPlots
from .step_panel_base import SLIDER_STEP

if TYPE_CHECKING:
    from ...shared.planner.timed_path import TimedPath
    from .plugin import AssemblyStepsPlugin


class StepsPanel:
    """The panel's widgets, built once, and what `draw` writes into them. Main thread."""

    def __init__(self, ctx: PluginContext, plugin: AssemblyStepsPlugin):
        """Build the panel in the Assembly tab; every click becomes one of the plugin's commands.

        Args:
            ctx: The plugin's context.
            plugin: The plugin the widgets call.
        """
        # The motion time the slider was last set to, and the plan the plots were last filled from.
        self._shown_time: float | None = None
        self._plotted: TimedPath | None = None
        with ctx.view.ui(band=STEP_BAND) as gui:
            self._status = gui.add_html("")
            self._step = gui.add_html("")
            self._moves = gui.add_html("")
            self._check = gui.add_button("Live IK", color="teal", icon=GOAL_ICON,
                                         hint="Solve where the next movement starts (the insert start after a "
                                              "transfer, the linear move's start after an approach) by IK at the live "
                                              "base, and show it as the target ghost; nothing moves. Only a transfer "
                                              "or an approach has such a goal")
            plan = gui.add_button("Plan", color="teal", icon=PLAN_ICON,
                                  hint="Plan the step's motion from where the robot is now")
            path = gui.add_button_group("Path", ["Play", "Clear"],
                                        hint="Play the preview; remove the plan (or stop planning)")
            gui.add_html(section("preview", SECTION_CTRL))
            # * The base's commands read the slider as `plugin._slider`.
            plugin._slider = gui.add_slider("t", min=0.0, max=1.0, step=SLIDER_STEP, initial_value=0.0,
                                            hint="Scrub through the planned path, seconds")
            self._time = gui.add_slider("s", min=1.0, max=MAX_TRAJ_TIME, step=0.5, initial_value=10.0,
                                        hint="Motion time, seconds: how long the motion takes; the arms' limits may "
                                             "need longer")
            self._minimum = gui.add_html("")
            # * Draws nothing: both sliders across the row.
            gui.add_html(f"<style>{slim_labels_css([plugin._slider._impl.uuid, self._time._impl.uuid])}</style>")
        with ctx.view.ui(band=EXEC_BAND) as gui:
            commit = gui.add_button("Exec", color="orange", icon=EXEC_ICON,
                                    hint="Run the step's parts on the robot, each after Confirm below or on the phone")
            # ! Keep changing text BELOW the buttons.
            self._details = gui.add_html("")
        with ctx.view.ui(band=SETTINGS_BAND) as gui:
            gui.add_html(section("settings", SECTION_TOOL))
            self._ignore = gui.add_checkbox("Ignore built bars", False,
                                            hint="Plan as if the built bars were not there: for tests in the air")
        with ctx.view.ui(band=PLOTS_BAND) as gui:
            gui.add_html(section("plan", SECTION_CTRL))
            self._plots = PlanPlots(gui)
        self._check.on_click(ctx.defer("live IK", lambda: plugin.check_goal(ctx)))
        plan.on_click(ctx.defer("plan", lambda: plugin._on_path(ctx, "Plan")))
        path.on_click(ctx.defer_value("path button", lambda label: plugin._on_path(ctx, label)))
        plugin._slider.on_update(ctx.defer_value("scrub", lambda value: plugin._scrub(float(value))))
        commit.on_click(ctx.defer("exec", lambda: plugin._commit(ctx)))
        self._time.on_update(ctx.defer_value("motion time", lambda value: plugin.set_traj_time(float(value))))
        self._ignore.on_update(ctx.defer_value("ignore built bars", lambda value: plugin.set_ignore_built(bool(value))))

    def draw(self, ctx: PluginContext, plugin: AssemblyStepsPlugin) -> None:
        """Write the current step, its plan and its parts into the widgets, and the plan into the plots.

        Args:
            ctx: The plugin's context.
            plugin: The plugin.
        """
        current = plugin.current()
        executor = ctx.require("movement_exec")
        if current is None:
            self._status.content = chip_line(chip("no current step", NONE) + plugin._plan_chip())
            self._step.content = block(values("—"))
            self._moves.content = block(values("Exec: —"))
        else:
            view, action, step, primary = current
            number, count = plugin.position
            chips = (chip(escape(action), SECTION_CTRL)
                     + chip(escape(f"{plugin._robot_name} ({plugin.serial or 'not configured'})"), OK)
                     + chip(f"step {number}/{count}", NONE) + plugin._plan_chip())
            if executor.active:
                chips += chip(escape(f"exec: {executor.active}"), BUSY)
            self._status.content = chip_line(chips)
            movements = view.process.process.actions[action].movements
            routine = plugin.routine()
            text = f"{primary.label} ({routine.value if routine is not None else 'unclassified'})"
            if step.with_motion:
                text += " · with: " + ", ".join(movements[index].id for index in step.with_motion)
            if step.after_motion:
                text += " · then: " + ", ".join(movements[index].id for index in step.after_motion)
            if step.skipped:
                text += " · skipped: " + ", ".join(movements[index].id for index in step.skipped)
            self._step.content = block(values(_cut(escape(text))))
            self._moves.content = block(values(_cut(escape(exec_line(f"{plugin._robot_name} ({plugin.serial})",
                                                                     plugin.parts())))))
        self._check.label = plugin.goal_label()
        if self._shown_time != plugin.traj_time:
            self._shown_time = self._time.value = plugin.traj_time
        minimum = "—" if math.isnan(plugin.minimum) else f"{plugin.minimum:.1f}"
        self._minimum.content = block(values(f"min {minimum} s by the arms' limits"))
        self._ignore.value = plugin.ignore_built
        self._details.content = block(plugin._message_html())
        if plugin.path is not self._plotted:
            # * A new plan, a retime after a motion-time change, or none (Clear, Exec, a new step).
            self._plotted = plugin.path
            route = plugin.route
            self._plots.show(plugin.path, route.result.trajectory if route is not None else None)


def exec_line(robot: str, parts: tuple[PartPlan, ...]) -> str:
    """What Exec runs, one line: e.g. "Exec: cindy (a200-0806) moves left_ur_arm, right_ur_arm; then by hand …".

    Args:
        robot: The robot and its serial.
        parts: The step's parts not done yet.
    """
    if not parts:
        return "Exec: nothing left in this step"
    words = []
    for part in parts:
        tools = ", ".join(part.tools)
        insert = f"inserts, the joint screw of {tools} tightening" if part.tools else "inserts rigid (ground bar)"
        words.append({MOTION_PART: f"moves {tools}", INSERT_PART: insert,
                      RETREAT_PART: f"retreats {tools}, compliant first", OPEN_PART: f"opens the Robotiq on {tools}",
                      HANDOFF_PART: f"closes the Robotiq on {tools}, the arm compliant",
                      GRASP_PART: f"tightens the scaffolding gripper screws on {tools}",
                      WITH_MOTION_PART: f"opens the gripper screws on {tools} in the retreat",
                      BY_HAND_PART: f"waits for {part.movement} by hand",
                      BASE_PART: f"waits for {part.movement} (base by hand)"}.get(
            part.kind, f"refuses {part.movement}: {part.text}"))
    return f"Exec: {robot} " + "; then ".join(words)


def _cut(html: str) -> str:
    """One line, cut with "…" when too wide; the whole line on hover."""
    return (f'<span title="{html}" style="display:inline-block;max-width:100%;white-space:nowrap;overflow:hidden;'
            f'text-overflow:ellipsis;vertical-align:bottom">{html}</span>')
