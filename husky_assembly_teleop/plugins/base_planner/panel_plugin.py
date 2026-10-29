"""
The base planner's panel: pick a target for a husky base, ask `base_planner` for a path, scrub through it, commit it.

The flow is `shared.planner.planner_panel.PlannerPanel`; this adds the planar target, the choice of algorithm
(`algorithms`) with its settings and debug view, the experiment's task and recording inputs, and the floor drawing.
The last three sit in a collapsed Advanced folder, so the panel shows only target, plan and commit.

- The floor shows the plan as the follower will cut it, the sent path, and the robot's trail until the next Commit.
- A task of the experiment's standard set (`tasks`) puts its virtual obstacles into the scene, under this panel's
  name; Load sets the target to its start (to drive there) or its target.
- A plan refused for a start or target in collision shows the hit in the collisions panel until a newer plan.
! Drive the planner through its public API (set target, plan, cancel, results) from the panel wherever you can,
  so those paths stay ready for other users such as an executor; only debug drawing and internal state reach inside.
"""

from __future__ import annotations

import math
from dataclasses import asdict
from html import escape

import viser
from crl_husky.follower_path import FollowerPath

from ...drive import drive_folder
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...shared.base_planner_contract import BasePlanRequest
from ...shared.collisions_contract import CollisionsContract
from ...shared.planner.planner_panel import PlannerPanel, unknown_robots_chip
from ...shared.planner.plan import Plan
from ...ui.base_markers import Markers
from ...ui.ghost import RobotGhost
from ...ui.phone import group_buttons
from ...ui.pose_input import PlanarPoseInput, yaw_from_xyzw
from ...ui.pybullet_window import add_pybullet_window_toggle
from ...ui.style import (BUSY, FAIL, OK, SECTION_TOOL, block, check_chip, chip, chip_line, section,
                         values)
from ...ui.visualization import PLAN_TAB
from ...robot_interface.base import live_report
from ...world.follower import path_check
from ...world.kinematics import STOW_TOLERANCE, from_stow
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from .algorithms import ALGORITHMS, Algorithm
from .algorithms.base import add_margin_setting
from .debug_view import DebugView
from .planner_plugin import DEFAULT_EXPERIMENT, BasePlannerPlugin, differs, floor_pose
from .recording import RECORDING_FOLDER
from .tasks import BY_NAME, TASKS

#: The Task dropdown's entry for no task.
NO_TASK = "(none)"


@register
class BasePlannerPanel(PlannerPanel):
    """Sets a target for a base, plans and commits a path to it with `base_planner`, and draws both."""

    name = "base_planner_panel"
    experimental = True
    planner = "base_planner"
    tab_path = (PLAN_TAB, "Base")
    icon = viser.Icon.CAR
    requires = ("base_planner", "process", "collisions")
    #: Orange, so the target never passes for the process panel's blue robot in the same view.
    target_kind = "base_target"

    def __init__(self):
        """Start with no robot chosen and no target."""
        super().__init__()
        #: The plan whose collision, if any, the collisions panel shows.
        self._reported: Plan | None = None
        # The settings' widgets; the planner keeps its own algorithms for planning.
        self._algorithms: list[Algorithm] = [algorithm() for algorithm in ALGORITHMS]
        # The plan's path as the follower will cut it, for drawing: (plan id, its cut).
        self._preview: tuple[int, FollowerPath] | None = None
        self._planner: BasePlannerPlugin | None = None

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, and a target ghost and a path ghost per robot."""
        self._connect(ctx)
        # * The planner itself, only for what this panel alone shows: the debug record, the sent path and its run,
        #   and the PyBullet window. Everything else goes through `self._api`.
        self._planner = ctx.require(self.planner)
        self._collisions: CollisionsContract = ctx.require("collisions")
        serials = [robot.serial for robot in ctx.config.robots]
        if not serials:
            self._set_up_idle(ctx, "no robots configured: nothing to plan for")
            return
        self.serial = serials[0]
        root = ctx.view.scene_root

        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            self._robot = gui.add_dropdown("Robot", options=serials, initial_value=self.serial)

            gui.add_html(section("target", SECTION_TOOL))
            self._target = PlanarPoseInput(ctx, gui, f"{root}/target_gizmo", label="Target",
                                           on_change=self._target_changed)
            load = gui.add_button_group("Load", ["Robot", "Park", "Step", "Task start", "Task target"],
                                        hint="Set the target to where the robot is now, to its parked pose, to its "
                                             "base pose in the step selected in the process plugin, or to the task's "
                                             "start or target")
            # * Step in the target section's orange: the one pressed at every assembly step, among the plain ones.
            gui.add_html(f"<style>{group_buttons(load._impl.uuid, 2)} {{ color: {SECTION_TOOL} !important; "
                         f"border-color: {SECTION_TOOL} !important; }}</style>")
            # * One margin for every algorithm, in plain sight: the first thing to lower when a start or target
            #   near a bar counts as a collision.
            self._margin = add_margin_setting(gui)

            self._add_path_controls(ctx, gui,
                                    plan_hint="Plan from the robot's pose to the target around the others; Play "
                                              "previews; Clear drops the plan. A sent path keeps driving: Stop all "
                                              "stops it.",
                                    commit_hint="Send the path to the onboard follower; it drives at once if Vel "
                                                "runs. With an arm out of its stow pose, press it twice too.")
            # ! Keep changing content BELOW the buttons, so nothing moves under the cursor.
            self._details = gui.add_html("")

            # Everything beyond plan and commit, folded away until wanted.
            with gui.add_folder("Advanced", expand_by_default=False):
                gui.add_html(section("experiment", SECTION_TOOL))
                self._experiment = gui.add_text("Experiment", initial_value=DEFAULT_EXPERIMENT,
                                                hint="Name of this data collection: recorded attempts are saved "
                                                     "under it")
                self._task = gui.add_dropdown("Task", options=[NO_TASK] + [task.name for task in TASKS],
                                              initial_value=NO_TASK,
                                              hint="A task of the standard set: puts its virtual obstacles into the "
                                                   "scene and sets the target; Load its start to drive there first")
                try:
                    drive_folder(ctx.config.drive_root, RECORDING_FOLDER)
                    problem = ""
                except FileNotFoundError as error:
                    problem = str(error)
                self._recording = gui.add_checkbox("Record", initial_value=False, disabled=bool(problem),
                                                   hint=problem or f"Save every plan attempt and each committed run "
                                                                   f"to the Drive's {RECORDING_FOLDER}/<experiment>/")

                gui.add_html(section("algorithm", SECTION_TOOL))
                labels = [algorithm.label for algorithm in self._algorithms]
                self._algorithm = gui.add_dropdown("Algorithm", options=labels, initial_value=labels[0],
                                                   hint="How the next plan is searched; its settings are below")
                # Each algorithm's settings in a folder of their own, shown while it is chosen.
                self._settings = []
                for algorithm in self._algorithms:
                    with gui.add_folder(f"{algorithm.label} settings", expand_by_default=False) as folder:
                        algorithm.add_settings(gui)
                    folder.visible = algorithm is self._algorithms[0]
                    self._settings.append(folder)

                # * Watch the planning world, and the search moving the robot around in it.
                gui.add_html(section("debug", SECTION_TOOL))
                add_pybullet_window_toggle(ctx, gui, self._planner._executor, self._planner._world.set_gui)
                self._debug_on = gui.add_checkbox("Debug view", initial_value=False,
                                                  hint="Record how the next search works, and draw it on the floor")
                self._debug_legend = gui.add_html("")
                self._debug_layers = gui.add_folder("Debug layers")
                self._debug_notes = gui.add_html("")

        self._robot.on_update(ctx.defer_value("choose robot", lambda serial: self._choose_robot(str(serial))))
        self._task.on_update(ctx.defer_value("choose task", lambda name: self._choose_task(ctx, str(name))))
        load.on_click(ctx.defer_value("load target", lambda label: self._load_target(ctx, label)))
        self._algorithm.on_update(ctx.defer_value("choose algorithm", lambda label: self._choose_algorithm(str(label))))

        self._markers = Markers(ctx)
        self._debug_view = DebugView(ctx, self._debug_legend, self._debug_layers, self._debug_notes)
        self._add_ghosts(ctx, ctx.config.robots)

    # --- --- --- --- --- COMMANDS (intents, on the main thread) --- --- --- --- ---

    def _choose_robot(self, serial: str) -> None:
        """Plan for another robot. The old plan no longer applies."""
        self._used.touch()
        if serial != self.serial:
            self.serial = serial
            self._api.clear()

    def _choose_algorithm(self, label: str) -> None:
        """Show the chosen algorithm's settings; the next plan uses it."""
        self._used.touch()
        for algorithm, folder in zip(self._algorithms, self._settings):
            folder.visible = algorithm.label == label

    def _choose_task(self, ctx: PluginContext, name: str) -> None:
        """Put the task's virtual obstacles into the scene, in place of the last task's, and target its target."""
        self._used.touch()
        ctx.scene.remove_prefix(f"{self.name}/task/")
        task = BY_NAME.get(name)
        if task is None:
            self._say("no task: virtual obstacles removed")
            return
        if task.virtual:
            ctx.scene.put_many(task.bodies(f"{self.name}/task/"))
        self._target.set(*task.target_in_lab())
        self._has_target = True
        where = "" if task.virtual else f"; select step {task.process_step} in the process plugin"
        self._say(f"task {task.name}: target set; Load Task start to drive to its start{where}")

    def _target_changed(self) -> None:
        """The operator edited the target."""
        self._used.touch()
        self._has_target = True

    def _load_target(self, ctx: PluginContext, label: str) -> None:
        """Set the target from the robot's current or parked pose, the process plugin's selected step, or the task.

        Args:
            ctx: This plugin's context.
            label: "Robot", "Park", "Step", "Task start" or "Task target".
        """
        self._used.touch()
        if label.startswith("Task"):
            task = BY_NAME.get(self._task.value)
            if task is None:
                self._say("choose a task first", FAIL)
                return
            self._target.set(*(task.start_in_lab() if label == "Task start" else task.target_in_lab()))
            self._has_target = True
            drive = ": plan and commit to drive there" if label == "Task start" else ""
            self._say(f"target set to the {label.lower()}{drive}")
            return
        if label == "Robot":
            self._target.set(*floor_pose(ctx, self.serial))
            self._has_target = True
            self._say(f"target set to where {self.serial} is now")
            return
        if label == "Park":
            park = next(robot for robot in ctx.config.robots if robot.serial == self.serial).park_pose
            if park is None:
                self._say(f"{self.serial} has no parked pose configured", FAIL)
                return
            self._target.set(*park)
            self._has_target = True
            self._say(f"target set to the parked pose of {self.serial}")
            return

        found = self._process_step(ctx)
        if found is None:
            return
        selection, serial, robot_name = found
        label = selection.movement.id
        # * The movement says which robot it is for; plan for that one.
        self._choose_robot(serial)
        # ? The process's base pose is the URDF root (base_footprint), as in mocap and PyBullet.
        base = selection.movement.start.robots[selection.action.robot].base
        if base is None:
            self._say(f"{label} leaves {robot_name}'s base to the planner: no target to load", FAIL)
            return
        self._target.set(float(base.position[0]), float(base.position[1]), yaw_from_xyzw(base.orientation))
        self._has_target = True
        self._say(f"target from {label} ({robot_name})")

    # --- --- --- --- --- PLANNER PANEL --- --- --- --- ---

    def make_request(self, ctx: PluginContext) -> BasePlanRequest:
        """The chosen robot, target, algorithm and its settings (margin from the panel), task and recording inputs."""
        algorithm = next(algorithm for algorithm in self._algorithms if algorithm.label == self._algorithm.value)
        task = str(self._task.value)
        settings = {**asdict(algorithm.settings()), "margin": float(self._margin.value)}
        return BasePlanRequest(self.serial, tuple(float(value) for value in self._target.pose), algorithm.label,
                               tuple(settings.items()), debug=bool(self._debug_on.value),
                               task=None if task == NO_TASK else task, record=bool(self._recording.value),
                               experiment=str(self._experiment.value))

    def target_change(self, ctx: PluginContext, plan: Plan[BasePlanRequest]) -> str:
        """Whether the target moved away from the plan's goal."""
        return "the target changed since planning" if differs(self._target.pose, plan.request.goal) else ""

    def commit_warnings(self, ctx: PluginContext, plan: Plan[BasePlanRequest]) -> list[str]:
        """An arm out of its stow pose: the base would drive with it out."""
        out = self._out_of_stow(ctx, plan.request.serial)
        return [out] if out else []

    def _out_of_stow(self, ctx: PluginContext, serial: str) -> str:
        """How far the robot's measured arms are from their stow pose, e.g. "the arm 40° from stow"; "" if stowed."""
        config = next((robot for robot in ctx.config.robots if robot.serial == serial), None)
        robot = ctx.scene.snapshot.robots.get(robot_id(serial))
        if config is None or robot is None:
            return ""
        away = from_stow(robot, config)
        return f"the arm {math.degrees(away):.0f}° from stow" if away > STOW_TOLERANCE else ""

    def show_target(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """The robot at the target pose, arms as they are; hidden once the robot stands there, so it never covers it."""
        if differs(floor_pose(ctx, self.serial), self._target.pose):
            self._show_at(ctx, ghost, self.serial, self._target.pose)
        else:
            ghost.hide()

    def show_path(self, ctx: PluginContext, ghost: RobotGhost, plan: Plan[BasePlanRequest]) -> None:
        """The robot on the path at the slider time, arms as they are."""
        self._show_at(ctx, ghost, plan.request.serial, plan.path.sample(self._t))

    def _show_at(self, ctx: PluginContext, ghost: RobotGhost, serial: str, pose) -> None:
        """Show `ghost` at floor pose (x, y, yaw), height and arms from the live robot, as planned."""
        x, y, yaw = (float(v) for v in pose)
        z = float(ctx.kinematics.base_pose(serial).position[2])
        ghost.show(Pose((x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))), ctx.kinematics.joints(serial))

    # --- --- --- --- --- TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Take the planner's news and advance playback; show a refused plan's collision, or take it away."""
        super().update(ctx)
        if self.idle:
            return
        plan = self._api.plan
        if plan is not self._reported:
            # * A plan refused for a collision shows it at once; any newer plan takes it away.
            self._reported = plan
            if plan is not None and plan.hit is not None:
                label = f"{plan.request.serial} {plan.hit.where}"
                self._collisions.replace(self.name, {label: plan.hit}, shown=(label,))
            else:
                self._collisions.clear(self.name)

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def draw(self, ctx: PluginContext) -> None:
        """Pose the ghosts, redraw the path line if needed, and fill the panel."""
        if self.idle:
            return
        self._status.content = self._status_html(ctx)
        # Loading a target from the step can switch the robot; show that here.
        self._robot.value = self.serial
        self._details.content = self._details_html()
        self._draw_slider()
        self._draw_floor(ctx)
        self._debug_view.show(self._planner._debug if self._debug_on.value else None)
        self._draw_ghosts(ctx)

    def _draw_floor(self, ctx: PluginContext) -> None:
        """The plan in blue until committed, the sent path in green, the robot's trail, the follower's points."""
        planner = self._planner
        self._markers.paths(self._preview_of(self._api.plan), planner._sent)
        if planner._trace is None:
            self._markers.trail(None)
        else:
            self._markers.trail(planner._trace.values(f"{planner._sent_serial}_floor_pose")[:, :2])
        follower = None if planner._sent_serial is None else \
            live_report(ctx.world.robots[planner._sent_serial].base.state.follower, ctx.now())
        self._markers.follower(follower if follower is not None and follower.path_id == planner._sent_id else None)

    def _preview_of(self, plan: Plan[BasePlanRequest] | None) -> FollowerPath | None:
        """The plan's path as the follower will cut it, until it is sent; cut once per plan."""
        if plan is None or plan.path is None or plan.state == "sent":
            return None
        if self._preview is None or self._preview[0] != plan.id:
            self._preview = plan.id, FollowerPath.from_poses(*plan.path.points.T)
        return self._preview[1]

    def _status_html(self, ctx: PluginContext) -> str:
        """One line of chips: the robot and whether it is tracked, the plan's state and the last run's."""
        tracked = ctx.scene.snapshot.robots[robot_id(self.serial)].base_tracked
        chips = chip(escape(f"{self.serial} {'tracked' if tracked else 'not tracked'}"), OK if tracked else BUSY,
                     "mocap tracks the base: plans start from its live pose" if tracked
                     else "plans from where it was last seen; a commit needs the base tracked by mocap")
        if out := self._out_of_stow(ctx, self.serial):
            chips += chip("arm not stowed", BUSY, f"{out}: the base would drive with it out, so Commit asks twice.\n"
                                                  "Stow it in Plan › Arm (Load Stow).")
        # * The run's chip replaces "sent", and stays while the next plan is made.
        plan, planner = self._api.plan, self._planner
        if plan is None or plan.state != "sent":
            chips += self._plan_chip()
        if planner._sent_id is not None:
            follower = ctx.world.robots[planner._sent_serial].base.state.follower
            chips += check_chip(path_check(follower, planner._sent_id, planner._sent_at, ctx.now()))
        return chip_line(chips + unknown_robots_chip(ctx.scene.snapshot, [robot_id(self.serial)]))

    def _details_html(self) -> str:
        """Numbers of the target and the plan, and the last message, in a fixed number of rows."""
        x, y, yaw = self._target.pose
        target = f"target  {x:+7.3f} {y:+7.3f} {math.degrees(yaw):+7.1f}°" if self._has_target else "target  —"
        plan = self._api.plan
        path = None if plan is None else plan.path
        if path is None:
            line = "path    —"
            at = "at t    —"
        else:
            px, py, pyaw = path.sample(self._t)
            line = f"path    {path.length:6.2f} m  {path.duration:5.1f} s  {len(path.points)} waypoints"
            at = f"at t    {px:+7.3f} {py:+7.3f} {math.degrees(pyaw):+7.1f}°"
        return block(values(target, line, at) + self._message_html())
