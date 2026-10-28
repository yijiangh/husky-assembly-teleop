"""
The assembly_steps part of the Assembly tab: the step, Live IK, Plan, the preview, Exec, the settings.

The Plan plots window at the bottom left is this panel's too; it opens by itself when a plan arrives.
The process panel's slider and ◀ mv / mv ▶ move through the movements; the run follows them, and this panel the run.

! Never hide a row or a button, and keep every changing line one line tall, so nothing moves under the cursor.
"""

from __future__ import annotations

import math
from html import escape
from typing import TYPE_CHECKING

import viser

from ...plugin_api.context import PluginContext
from ...shared.movement_exec_contract import (BASE_PART, BY_HAND_PART, HANDOFF_PART, INSERT_PART, MAX_TRAJ_TIME,
                                              MOTION_PART, OPEN_PART, RETREAT_PART, SERVO_PART, WITH_MOTION_PART,
                                              PartPlan)
from ...shared.runs.runs import (PLAN_ITERATIONS_RANGE, PLAN_RESOLUTION_RANGE, PLAN_TIME_RANGE, PLAN_TRIES_RANGE,
                                 RunSettings)
from ...ui.assembly_tab import (CHECK_ICON, EXEC_BAND, EXEC_ICON, GOAL_ICON, PLAN_ICON, PLAN_PLOTS_X,
                                PLAN_SETTINGS_BAND, PLOTS_HEIGHT, PLOTS_WIDTH, PLOTS_Y, SETTINGS_BAND, STEP_BAND)
from ...ui.style import BUSY, NONE, OK, SECTION_CTRL, SECTION_TOOL, block, chip, chip_line, section, values
from .plan_plots import PlanPlots
from .step_panel_base import SLIDER_STEP

if TYPE_CHECKING:
    from ...shared.assembly_planning.route_result import RouteResult
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
        # The motion time the slider was last set to, the run settings the Advanced planning folder last showed, and
        # the path and the plan the plots were last filled from.
        self._shown_time: float | None = None
        self._shown_settings: RunSettings | None = None
        self._plotted: TimedPath | None = None
        self._plotted_route: RouteResult | None = None
        with ctx.view.ui(band=STEP_BAND) as gui:
            self._status = gui.add_html("")
            self._step = gui.add_html("")
            self._moves = gui.add_html("")
            self._check = gui.add_button("Live IK", color="teal", icon=GOAL_ICON,
                                         hint="Solve where the next movement starts (the insert start after a "
                                              "transfer, the linear move's start after an approach) by IK at the live "
                                              "base, and show it as the target ghost; nothing moves. Only a transfer "
                                              "or an approach has such a goal")
            check_tool0 = gui.add_button("Check tool0", color="teal", icon=CHECK_ICON,
                                         hint="Draw each flange's live tool0 (mocap base + FK, red) against the "
                                              "step's authored target (blue) and say the gap; nothing moves")
            plan = gui.add_button("Plan", color="teal", icon=PLAN_ICON,
                                  hint="Plan the step's motion from where the robot is now")
            path = gui.add_button_group("Path", ["Play", "Clear"],
                                        hint="Play the preview; remove the plan (or stop planning)")
            gui.add_html(section("preview", SECTION_CTRL))
            # * The base's commands read the slider as `plugin._slider`.
            plugin._slider = gui.add_slider("Preview time", min=0.0, max=1.0, step=SLIDER_STEP, initial_value=0.0,
                                            hint="Scrub through the planned path: seconds from its start")
            self._time = gui.add_slider("Motion time", min=1.0, max=MAX_TRAJ_TIME, step=0.5, initial_value=10.0,
                                        hint="How long the motion takes, seconds; the arms' limits may need longer")
            self._minimum = gui.add_html("")
        with ctx.view.ui(band=EXEC_BAND) as gui:
            commit = gui.add_button("Exec", color="orange", icon=EXEC_ICON,
                                    hint="Run the step's parts on the robot: a motion at once; the bar mount, a "
                                         "Robotiq step and the servo's first pass after Confirm below or on the phone")
            # ! Keep changing text BELOW the buttons.
            self._details = gui.add_html("")
        with ctx.view.ui(band=SETTINGS_BAND) as gui:
            gui.add_html(section("settings", SECTION_TOOL))
            self._ignore = gui.add_checkbox("Ignore built bars", False,
                                            hint="Plan as if the built bars were not there: for tests in the air")
            self._diagnosis = gui.add_checkbox("Visual diagnosis", False,
                                               hint="The real robots see-through, and after each insert or retreat "
                                                    "plan its tool0 path with the Rhino and planned start and end "
                                                    "frames; the log says the gaps")
        # * The free searches' settings, saved with the run (`RunSettings.plan_*`); folded away until wanted.
        default = RunSettings()
        with ctx.view.ui(band=PLAN_SETTINGS_BAND) as gui, gui.add_folder("Advanced planning", expand_by_default=False):
            self._plan_time = gui.add_slider("Max time s", min=PLAN_TIME_RANGE[0], max=PLAN_TIME_RANGE[1], step=5.0,
                                             initial_value=default.plan_max_time,
                                             hint="Seconds each try of the free dual-arm search may take")
            self._plan_iterations = gui.add_slider("Max iterations", min=PLAN_ITERATIONS_RANGE[0],
                                                   max=PLAN_ITERATIONS_RANGE[1], step=1,
                                                   initial_value=default.plan_max_iterations,
                                                   hint="Iterations each try of the free dual-arm search may run")
            self._plan_resolution = gui.add_slider("Joint resolution rad", min=PLAN_RESOLUTION_RANGE[0],
                                                   max=PLAN_RESOLUTION_RANGE[1], step=0.01,
                                                   initial_value=default.plan_joint_resolution,
                                                   hint="Joint step of the free search's collision checks: smaller "
                                                        "misses less between checks, and searches slower")
            self._plan_tries = gui.add_slider("Tries", min=PLAN_TRIES_RANGE[0], max=PLAN_TRIES_RANGE[1], step=1,
                                              initial_value=default.plan_tries,
                                              hint="Tries of a free or one-arm search before it gives up, each a new "
                                                   "random search")
            self._plan_swept = gui.add_checkbox("Swept check", default.plan_swept_check,
                                                hint="Check a found path between its waypoints, 0.01 rad apart, "
                                                     "before taking it; off, the path is taken as the search found it")
        # * The plan's plots in a window of their own, minimized to its bar until a plan arrives (`draw`).
        self._plots_panel = ctx.view.panel()
        with self._plots_panel.add_tab("Plan plots", viser.Icon.CHART_LINE):
            self._plots = PlanPlots(ctx.view.gui)
        self._plots_panel.float(x=PLAN_PLOTS_X, y=PLOTS_Y, width=PLOTS_WIDTH, height=PLOTS_HEIGHT)
        self._plots_panel.minimize()
        self._check.on_click(ctx.defer("live IK", lambda: plugin.check_goal(ctx)))
        check_tool0.on_click(ctx.defer("check tool0", lambda: plugin.check_tool0(ctx)))
        plan.on_click(ctx.defer("plan", lambda: plugin._on_path(ctx, "Plan")))
        path.on_click(ctx.defer_value("path button", lambda label: plugin._on_path(ctx, label)))
        # ! Operator drags only: draw writes the slider every tick, and viser calls back for that too, a tick or two
        #   late, which would set the preview back (it lagged, and could stop short of the end).
        plugin._slider.on_update(ctx.defer_value("scrub", lambda value: plugin._scrub(float(value)),
                                                 operator_only=True))
        commit.on_click(ctx.defer("exec", lambda: plugin._commit(ctx)))
        self._time.on_update(ctx.defer_value("motion time", lambda value: plugin.set_traj_time(float(value))))
        self._ignore.on_update(ctx.defer_value("ignore built bars", lambda value: plugin.set_ignore_built(bool(value))))
        # ! Operator clicks only: draw writes the box back, and viser calls back for that too, which could undo a
        #   click queued in the same tick.
        self._diagnosis.on_update(ctx.defer_value("visual diagnosis",
                                                  lambda value: plugin.set_visual_diagnosis(ctx, bool(value)),
                                                  operator_only=True))
        # The iterations and tries go as numbers: the run refuses one that is not whole, then makes it an int.
        settings = ((self._plan_time, "plan_max_time", float), (self._plan_iterations, "plan_max_iterations", float),
                    (self._plan_resolution, "plan_joint_resolution", float), (self._plan_tries, "plan_tries", float),
                    (self._plan_swept, "plan_swept_check", bool))
        for widget, setting, kind in settings:
            widget.on_update(ctx.defer_value(setting, lambda value, setting=setting, kind=kind: ctx.require(
                "assembly_run").request_settings(**{setting: kind(value)}), operator_only=True))

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
        self._diagnosis.value = plugin.visual_diagnosis
        settings = ctx.require("assembly_run").view.settings
        if settings != self._shown_settings:
            # * Only when the run's settings change; the panel's own writes never come back as requests
            #   (`operator_only` above).
            self._shown_settings = settings
            self._plan_time.value, self._plan_iterations.value = settings.plan_max_time, settings.plan_max_iterations
            self._plan_resolution.value, self._plan_tries.value = settings.plan_joint_resolution, settings.plan_tries
            self._plan_swept.value = settings.plan_swept_check
        self._details.content = block(plugin._message_html())
        if plugin.path is not self._plotted:
            # * A new plan, a retime after a motion-time change, or none (Clear, Exec, a new step).
            route = plugin.route
            if route is not None and route is not self._plotted_route:
                # * A new plan brings the window forward, as the old monitor's joint plot opened; only the operator
                #   closes it.
                self._plots_panel.expand()
            self._plotted, self._plotted_route = plugin.path, route
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
                      WITH_MOTION_PART: f"opens the gripper screws on {tools} in the retreat",
                      BY_HAND_PART: f"waits for {part.movement} by hand",
                      BASE_PART: f"waits for {part.movement} (base by hand)",
                      SERVO_PART: f"servos {tools} onto {part.text.split(' onto ')[-1]}"}.get(
            part.kind, f"refuses {part.movement}: {part.text}"))
    return f"Exec: {robot} " + "; then ".join(words)


def _cut(html: str) -> str:
    """One line, cut with "…" when too wide; the whole line on hover."""
    return (f'<span title="{html}" style="display:inline-block;max-width:100%;white-space:nowrap;overflow:hidden;'
            f'text-overflow:ellipsis;vertical-align:bottom">{html}</span>')
