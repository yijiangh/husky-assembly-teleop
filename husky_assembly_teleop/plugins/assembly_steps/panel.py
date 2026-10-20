"""
The assembly_steps panel: the current step, Prev and Next, Check goal, the path, Exec, the motion time and Mark done.

! Never hide a row or a button, and keep every changing line one line tall, so nothing moves under the cursor.
"""

from __future__ import annotations

import math
from html import escape
from typing import TYPE_CHECKING

from ...plugin_api.context import PluginContext
from ...shared.movement_exec import (BASE_PART, BY_HAND_PART, CLOSE_PART, GRASP_PART, MAX_TRAJ_TIME, MOTION_PART,
                                     OPEN_PART, PartPlan)
from ...ui.style import BUSY, NONE, OK, SECTION_CTRL, SECTION_TOOL, block, chip, section, values

if TYPE_CHECKING:
    from .plugin import AssemblyStepsPlugin

#: The step buttons.
PREV, NEXT = "◀ Prev", "Next ▶"


class StepsPanel:
    """The panel's widgets, built once, and what `draw` writes into them. Main thread."""

    def __init__(self, ctx: PluginContext, plugin: AssemblyStepsPlugin):
        """Build the panel; every click becomes one of the plugin's commands.

        Args:
            ctx: The plugin's context.
            plugin: The plugin the widgets call.
        """
        # The workaround and motion time the widgets were last set to.
        self._shown_workaround: bool | None = None
        self._shown_time: float | None = None
        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            self._step = gui.add_html("")
            steps = gui.add_button_group("Step", [PREV, NEXT],
                                         hint="Make the previous or next step of this action current")
            check = gui.add_button("Check goal", hint="Solve a transfer's or an approach's goal by IK where the robot "
                                                      "stands now, and show it; nothing moves")
            plugin._add_path_controls(ctx, gui, plan_hint="Plan the step's motion from where the robot is now; play "
                                                          "the preview; remove the plan (or stop planning)",
                                      commit_hint="Run the step's parts on the robot, each after Confirm on the "
                                                  "movement_exec panel or the phone",
                                      commit_label="Exec")
            self._moves = gui.add_html("")
            self._time = gui.add_slider("Motion s", min=1.0, max=MAX_TRAJ_TIME, step=0.5, initial_value=10.0,
                                        hint="How long the motion takes; the arms' limits may need longer")
            self._minimum = gui.add_html("")
            gui.add_html(section("settings", SECTION_TOOL))
            self._workaround = gui.add_checkbox("Transfer workaround", True,
                                                hint="The run's setting: the transfer runs as a free move without "
                                                     "the bar; the operator mounts it at the insertion start")
            self._ignore = gui.add_checkbox("Ignore built bars", False,
                                            hint="Plan as if the built bars were not there: for tests in the air")
            self._mark = gui.add_button("Mark done", hint="Mark the action done in the run, once its last step ran")
            # ! Keep changing text BELOW the buttons.
            self._details = gui.add_html("")
        steps.on_click(ctx.defer_value("step button", lambda label: plugin.go(ctx, -1 if label == PREV else 1)))
        check.on_click(ctx.defer("check goal", lambda: plugin.check_goal(ctx)))
        self._time.on_update(ctx.defer_value("motion time", lambda value: plugin.set_traj_time(float(value))))
        self._workaround.on_update(ctx.defer_value("transfer workaround",
                                                   lambda value: plugin.set_workaround(ctx, bool(value))))
        self._ignore.on_update(ctx.defer_value("ignore built bars", lambda value: plugin.set_ignore_built(bool(value))))
        self._mark.on_click(ctx.defer("mark done", lambda: plugin.mark_done(ctx)))

    def draw(self, ctx: PluginContext, plugin: AssemblyStepsPlugin) -> None:
        """Write the current step, its plan and its parts into the widgets.

        Args:
            ctx: The plugin's context.
            plugin: The plugin.
        """
        current = plugin.current()
        executor = ctx.require("movement_exec")
        if current is None:
            self._status.content = block(chip("no current step", NONE) + plugin._plan_chip())
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
            self._status.content = block(chips)
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
        if self._shown_time != plugin.traj_time:
            self._shown_time = self._time.value = plugin.traj_time
        minimum = "—" if math.isnan(plugin.minimum) else f"{plugin.minimum:.1f}"
        self._minimum.content = block(values(f"min {minimum} s by the arms' limits"))
        workaround = ctx.require("assembly_run").view.settings.transfer_as_free
        if workaround != self._shown_workaround:
            self._shown_workaround = self._workaround.value = workaround
        self._ignore.value = plugin.ignore_built
        self._mark.label = f"Mark {plugin.offer} done" if plugin.offer else "Mark done (after the last step)"
        self._details.content = block(plugin._message_html())


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
        words.append({MOTION_PART: f"moves {tools}", OPEN_PART: f"opens the Robotiq on {tools}",
                      CLOSE_PART: f"closes the Robotiq on {tools} (stiff)",
                      GRASP_PART: f"tightens the scaffolding gripper screws on {tools}",
                      BY_HAND_PART: f"waits for {part.movement} by hand",
                      BASE_PART: f"waits for {part.movement} (base by hand)"}.get(
            part.kind, f"refuses {part.movement}: {part.text}"))
    return f"Exec: {robot} " + "; then ".join(words)


def _cut(html: str) -> str:
    """One line, cut with "…" when too wide; the whole line on hover."""
    return (f'<span title="{html}" style="display:inline-block;max-width:100%;white-space:nowrap;overflow:hidden;'
            f'text-overflow:ellipsis;vertical-align:bottom">{html}</span>')
