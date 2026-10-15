"""
Pick a target for a husky base, plan a path to it, scrub through the path, and commit it.

The flow is `plugins.planning.panel.PlannerPlugin`; this adds the planar target, the choice of algorithm (`algorithms`)
with its settings and debug view, the floor drawing and the PyBullet world.

- Commit sends the path to the robot's onboard path follower (crl_husky `pure_pursuit`), geometric, capped at the
  planning speeds. The floor shows the path as the follower cuts it, and the robot's trail until the next Commit.
- A task of the experiment's standard set (`tasks`) puts its virtual obstacles into the scene; Load sets the target to
  its start (to drive there) or its target. An attempt counts for the task when it starts at the task's start.
- With Record ticked, every plan attempt is saved under the experiment's name, found or not, and a committed plan's
  run is added to it once the follower is done (`recording`, doc/base_planner_manual.md).

! EXPERIMENTAL: the turn-drive-turn path model will change.
"""

from __future__ import annotations

import asyncio
import math
import threading
import time
from dataclasses import asdict
from html import escape

from crl_husky.follower_path import FollowerPath

from ...drive import drive_folder
from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.trace import Trace
from ...robot_interface.base import BaseInterface
from ...plugin_api.plugin import register
from ..planning.panel import WAIT_MARGIN, PlannerPlugin, Search, with_guesses
from ..planning.search import TIME_LIMIT, PlanResult
from ...ui.base_markers import Markers
from ...ui.ghost import RobotGhost
from ...ui.pose_input import PlanarPoseInput, yaw_from_xyzw
from ...ui.pybullet_window import add_pybullet_window_toggle
from ...ui.style import BUSY, OK, SECTION_CTRL, SECTION_TOOL, block, check_chip, chip, section, values
from ...world import signals
from ...world.checks import WARN, Check
from ...world.follower import live_report, path_check
from ...world.kinematics import stow_joints
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from .path import MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED, BasePath
from .algorithms import ALGORITHMS, Algorithm, Debug
from .debug_view import DebugView
from .recording import RECORDING_FOLDER, attempt_folder, load_description, save_plan, write_description
from .tasks import BY_NAME, TASK_ORIGIN, TASKS
from .world import PlanningWorld, stowed

#: The plan goes stale once the base moves this far from its planned start.
STALE_POSITION = 0.05              # m
STALE_YAW = math.radians(3.0)      # rad

#: A run ends after the planned duration times RUN_FACTOR plus RUN_MARGIN seconds, done or not; the recording then
#: goes on for SETTLE seconds, while the robot comes to a stop.
RUN_FACTOR = 2.0
RUN_MARGIN = 10.0
SETTLE = 2.0

#: The Task dropdown's entry for no task, and the experiment name to start with.
NO_TASK = "(none)"
DEFAULT_EXPERIMENT = "default"
#: An attempt counts for its task when it starts this close to the task's start and aims at the task's target.
TASK_POSITION = 0.10           # m
TASK_YAW = math.radians(10.0)  # rad
#: The base plans with the arms stowed; an arm further than this from its stow pose is warned about.
STOW_TOLERANCE = math.radians(5.0)  # rad


def send_to_onboard_follower(base: BaseInterface, path: BasePath) -> int:
    """Hand a path to a robot's onboard path follower, replacing the one it follows.

    ? Geometric: the follower sets its own pace, capped at the planning speeds so it is never faster than the preview.

    Args:
        base: The robot's base.
        path: The path to follow.

    Returns:
        int: The path's id, as in the follower's reports.
    """
    return base.send_path(*path.points.T, None, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)


@register
class BasePlannerPlugin(PlannerPlugin):
    """Plans, shows and commits a base path from the current pose to a target."""

    name = "base_planner"
    experimental = True
    worker_name = "base-plan"

    def __init__(self):
        """Start with no robot chosen, no target and no plan."""
        super().__init__()
        self.path: BasePath | None = None
        # ! Worker thread only (sync, search, close).
        self._world = PlanningWorld()
        # The plan as the follower will cut it, for drawing.
        self._preview: FollowerPath | None = None
        # The path last sent: robot, follower's cut, id, ROS time it went out; the run following it, and its samples.
        self._sent_serial: str | None = None
        self._sent: FollowerPath | None = None
        self._sent_id: int | None = None
        self._sent_at = 0.0
        self._run_task: asyncio.Task | None = None
        self._trace: Trace | None = None
        self._algorithms: list[Algorithm] = [algorithm() for algorithm in ALGORITHMS]
        # The last search's debug record, once it returned.
        self._debug: Debug | None = None
        # The ready plan's planner, settings, time and recorded folder name (None if not recorded), for its run.
        self._plan_record: dict | None = None

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, and a target ghost and a path ghost per robot."""
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
            self._experiment = gui.add_text("Experiment", initial_value=DEFAULT_EXPERIMENT,
                                            hint="Name of this data collection: recorded attempts are saved under it")
            self._task = gui.add_dropdown("Task", options=[NO_TASK] + [task.name for task in TASKS],
                                          initial_value=NO_TASK,
                                          hint="A task of the standard set: puts its virtual obstacles into the "
                                               "scene and sets the target; Load its start to drive there first")
            load = gui.add_button_group("Load", ["Robot", "Park", "Step", "Task start", "Task target"],
                                        hint="Set the target to where the robot is now, to its parked pose, to its "
                                             "base pose in the step selected in the process plugin, or to the task's "
                                             "start or target")

            self._add_path_controls(ctx, gui,
                                    plan_hint="Plan around the other robots from the robot's current pose to "
                                              "the target; play or pause the preview; remove the plan (or stop "
                                              "planning)",
                                    commit_hint="Send the path to the robot's onboard path follower; it drives at once "
                                                "if the velocity controller is active")
            try:
                self._recordings, problem = drive_folder(ctx.config.drive_root, RECORDING_FOLDER), ""
            except FileNotFoundError as error:
                self._recordings, problem = None, str(error)
                ctx.log_warn(f"base planner: runs cannot be recorded: {problem}")
            self._recording = gui.add_checkbox("Record", initial_value=False, disabled=bool(problem),
                                               hint=problem or f"Save every plan attempt, found or not, and the run of "
                                                               f"each committed plan to {self._recordings}")
            # * Debugging: watch the planning world, and the search moving the robot around in it.
            add_pybullet_window_toggle(ctx, gui, self._executor, self._world.set_gui)
            # ! Keep changing content BELOW the buttons, so nothing moves under the cursor.
            self._details = gui.add_html("")

            gui.add_html(section("algorithm", SECTION_TOOL))
            labels = [algorithm.label for algorithm in self._algorithms]
            self._algorithm = gui.add_dropdown("Algorithm", options=labels, initial_value=labels[0],
                                               hint="How the next plan is searched; its settings are below")
            # Each algorithm's settings in a folder of their own, shown while it is chosen.
            self._settings = []
            for algorithm in self._algorithms:
                with gui.add_folder(f"{algorithm.label} settings") as folder:
                    algorithm.add_settings(gui)
                folder.visible = algorithm is self._algorithms[0]
                self._settings.append(folder)
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
            self._clear()

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
                self._say("choose a task first", failed=True)
                return
            self._target.set(*(task.start_in_lab() if label == "Task start" else task.target_in_lab()))
            self._has_target = True
            drive = ": plan and commit to drive there" if label == "Task start" else ""
            self._say(f"target set to the {label.lower()}{drive}")
            return
        if label == "Robot":
            x, y, yaw = self._current_pose(ctx)
            self._target.set(x, y, yaw)
            self._has_target = True
            self._say(f"target set to where {self.serial} is now")
            return
        if label == "Park":
            park = next(robot for robot in ctx.config.robots if robot.serial == self.serial).park_pose
            if park is None:
                self._say(f"{self.serial} has no parked pose configured", failed=True)
                return
            self._target.set(*park)
            self._has_target = True
            self._say(f"target set to the parked pose of {self.serial}")
            return

        found = self._process_step(ctx)
        if found is None:
            return
        step, serial, robot_name = found
        # * The step says which robot it is for; plan for that one.
        self._choose_robot(serial)
        # ? The process's base pose is the URDF root (base_footprint), as in mocap and PyBullet.
        base = step.movement.start.robots[step.action.robot].base
        if base is None:
            self._say(f"{step.label} leaves {robot_name}'s base to the planner: no target to load", failed=True)
            return
        self._target.set(float(base.position[0]), float(base.position[1]), yaw_from_xyzw(base.orientation))
        self._has_target = True
        self._say(f"target from {step.label} ({robot_name})")

    # --- --- --- --- --- PLANNER PLUGIN --- --- --- --- ---

    def make_search(self, ctx: PluginContext, abort: threading.Event) -> Search:
        """Search from the robot's current pose to the target, in the tick's snapshot."""
        serial, start, goal = self.serial, self._current_pose(ctx), self._target.pose
        algorithm = next(algorithm for algorithm in self._algorithms if algorithm.label == self._algorithm.value)
        settings, debug = algorithm.settings(), Debug(on=self._debug_on.value)
        # * Take the snapshot here, on the main thread; the worker syncs from it.
        snapshot, guessed = with_guesses(ctx.scene.snapshot, robot_id(serial))
        config = next(robot for robot in ctx.config.robots if robot.serial == serial)
        snapshot, away = stowed(snapshot, robot_id(serial), config)
        if away > STOW_TOLERANCE:
            guessed = guessed + [f"arm planned as stowed, {math.degrees(away):.0f}° away from it: stow it first"]
        guess = f" (guessed: {'; '.join(guessed)})" if guessed else ""
        if guessed:
            ctx.log_warn(f"planning for {serial} from a guess: {'; '.join(guessed)}")
        # * With Record ticked, the attempt gets its folder now; the worker saves it, found or not.
        environment, task = self._environment(ctx, serial), self._task_of(ctx, start, goal)
        setup = {"experiment": self._experiment_name(), "task": task, "robot": serial, "environment": environment,
                 "started": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                 "conditions": {"task_origin": list(TASK_ORIGIN), "process_step": self._process_index(ctx),
                                "arm_from_stow": math.degrees(away),
                                "speed_caps": {"linear": MAX_LINEAR_SPEED, "angular": MAX_ANGULAR_SPEED}},
                 "algorithm": algorithm.label, "settings": asdict(settings), "start": list(start),
                 "goal": list(goal), "guessed": guessed}
        folder = attempt_folder(self._recordings, setup["experiment"], serial, task, environment) \
            if self._recording.value and self._recordings is not None else None
        # The worker's error saving the attempt, for the main thread to log.
        not_saved: list[str] = []

        def work() -> PlanResult:
            # * Only an absent robot is left with problems; an untracked base or unknown joints were guessed.
            problems = snapshot.robots[robot_id(serial)].acting_problems()
            if problems:
                result, world = PlanResult(None, f"{serial} can't be planned for: {'; '.join(problems)}", 0.0), None
            else:
                self._world.sync(snapshot)
                result, world = algorithm.plan(self._world, robot_id(serial), start, goal, abort, settings,
                                               debug), self._world
            if folder is not None:
                try:
                    save_plan(folder, world, robot_id(serial), setup, result, debug.notes)
                except OSError as error:
                    not_saved.append(str(error))
            return result

        def finished() -> None:
            self._debug = debug
            if not_saved:
                ctx.log_error(f"base planner: plan not saved: {not_saved[0]}")

        def accepted(result: PlanResult) -> tuple[str, bool]:
            self._plan_record = {"folder": folder, "setup": setup, "seconds": result.seconds}
            self._preview = FollowerPath.from_poses(*self.path.points.T)
            how = algorithm.label + (" (straight)" if result.direct else "")
            return (f"{how}: {self.path.length:.2f} m, {self.path.duration:.1f} s, "
                    f"found in {result.seconds:.1f} s{guess}"), bool(guessed)

        return Search(serial=serial, label=f"plan {serial}", message=f"planning for {serial}…{guess}", work=work,
                      time_limit=TIME_LIMIT + WAIT_MARGIN, accepted=accepted,
                      finished=finished)

    def stale_reason(self, ctx: PluginContext) -> str:
        """Stale once the base moves away from the plan's start, or the target from its goal."""
        moved, turned = _offset(self._current_pose(ctx), self.path.start)
        if moved > STALE_POSITION or turned > STALE_YAW:
            return (f"the robot moved {moved * 100:.1f} cm, {math.degrees(turned):.1f}° since planning (replans beyond "
                    f"{STALE_POSITION * 100:.0f} cm or {math.degrees(STALE_YAW):.0f}°)")
        if _differs(self._target.pose, self.path.goal):
            return "the target changed since planning"
        return ""

    def send_plan(self, ctx: PluginContext) -> tuple[bool, str]:
        """Send the plan to the onboard follower; refused while untracked, e-stopped, or with no follower running."""
        serial = self._path_serial
        base = ctx.world.robots[serial].base
        if not base.state.tracked:
            return False, f"not committed: {serial} is not tracked by mocap"
        if base.state.estopped:
            return False, f"not committed: {serial} is e-stopped"
        if live_report(base.state.follower, ctx.now()) is None:
            return False, f"not committed: no follower on {serial}; start crl_husky pure_pursuit"
        if self._run_task is not None:
            self._run_task.cancel()  # the old run is saved as replaced
        self._sent_serial, self._sent = serial, self._preview
        self._sent_id, self._sent_at = send_to_onboard_follower(base, self.path), ctx.now()
        self._run_task = ctx.spawn(f"follow {serial}", self._follow(ctx, serial, self.path, self._plan_record))
        ctx.log_info(f"base path {self._sent_id} sent to {serial}: {self.path.length:.2f} m, "
                     f"{len(self.path.points)} waypoints")
        return True, f"sent to {serial}'s follower (path {self._sent_id})"

    def forget_plan(self) -> None:
        """The preview and debug record go with the plan; a sent path keeps driving (soft stop stops it), drawn."""
        self._preview = None
        self._debug = None
        self._plan_record = None

    def show_target(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """The robot at the target pose, arms stowed."""
        self._show_at(ctx, ghost, self.serial, self._target.pose)

    def show_path(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """The robot on the path at the slider time, arms stowed."""
        self._show_at(ctx, ghost, self._path_serial, self.path.sample(self._t))

    def _show_at(self, ctx: PluginContext, ghost: RobotGhost, serial: str, pose) -> None:
        """Show `ghost` at floor pose (x, y, yaw), height from the live robot, arms stowed as planned."""
        x, y, yaw = (float(v) for v in pose)
        z = float(ctx.kinematics.base_pose(serial).position[2])
        config = next(robot for robot in ctx.config.robots if robot.serial == serial)
        joints = ctx.kinematics.joints(serial)
        ghost.show(Pose((x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))),
                   {**joints, **stow_joints(config, joints)})

    # --- --- --- --- --- RUN (task) --- --- --- --- ---

    async def _follow(self, ctx: PluginContext, serial: str, path: BasePath, plan: dict | None) -> None:
        """Record the robot from Commit until the follower is done with the path, then save it if Record is ticked.

        `plan` is the plan's `_plan_record`: its setup, planning time and recorded folder (None if not recorded).
        """
        robot = ctx.world.robots[serial]
        sent_id, sent_at, started = self._sent_id, self._sent_at, time.strftime("%Y-%m-%dT%H:%M:%S%z")
        outcome, rec = "stopped", None
        try:
            with ctx.record(signals.base_floor_pose(robot), signals.follower_position_error(robot),
                            signals.follower_yaw_error(robot), signals.follower_command(robot),
                            signals.follower_progress(robot)) as rec:
                self._trace = rec
                try:
                    # * Under way is WARN (sent or following); done, aborted, replaced or silent ends the run.
                    await ctx.wait_until(lambda: self._path_check(ctx, serial, sent_id, sent_at).level != WARN,
                                         timeout_s=path.duration * RUN_FACTOR + RUN_MARGIN,
                                         description="the follower to finish")
                    check = self._path_check(ctx, serial, sent_id, sent_at)
                    outcome = check.label + (f": {check.detail}" if check.detail else "")
                except WaitTimeout:
                    outcome = "timed out: still following"
                await ctx.sleep(SETTLE)
        except asyncio.CancelledError:
            outcome = "replaced by a new commit" if self._sent_id != sent_id else "stopped"
            raise
        finally:
            ctx.log_info(f"base path {sent_id} on {serial}: {outcome}")
            if self._recording.value and self._recordings is not None and rec is not None and len(rec):
                self._save(ctx, serial, path, rec, outcome, started, sent_at, plan or {})

    def _save(self, ctx: PluginContext, serial: str, path: BasePath, rec: Trace, outcome: str, started: str,
              sent_at: float, plan: dict) -> None:
        """Add a run to its plan's attempt: recording.npz (samples, path, follower and wheel logs), and the run.

        The run goes into the attempt's experiment.json.

        A plan not recorded, or committed a second time, gets a folder of its own with what is known of the plan.
        """
        base = ctx.world.robots[serial].base
        follower = base.state.follower
        run = {"started": started, "outcome": outcome, "sent_at": sent_at,
               "controller": follower.controller if follower is not None else "unknown"}
        folder = plan.get("folder")
        description = load_description(folder) if folder is not None and folder.is_dir() else None
        try:
            if description is None or description.get("run") is not None:
                setup = plan.get("setup") or {"experiment": self._experiment_name(), "task": None, "robot": serial,
                                              "environment": self._environment(ctx, serial)}
                known = description["plan"] if description is not None else {
                    "found": True, "seconds": plan.get("seconds"), "length": path.length, "duration": path.duration}
                description = {**setup, "plan": known, "plan_folder": folder.name if folder is not None else None}
                folder = attempt_folder(self._recordings, setup["experiment"], serial, setup["task"],
                                        setup["environment"])
                folder.mkdir(parents=True)
            extra = {"path_poses": path.points, "path_t": path.times, "outcome": outcome,
                     "commands": base.follower_history(sent_at), "wheel_odometry": base.odometry_history(sent_at)}
            rec.save(folder / "recording.npz", extra)
            write_description(folder, {**description, "run": run})
        except OSError as error:
            ctx.log_error(f"base planner: run not saved: {error}")
            return
        ctx.log_info(f"base planner: run saved to {folder}")

    # --- --- --- --- --- READING --- --- --- --- ---

    def _path_check(self, ctx: PluginContext, serial: str, sent_id: int, sent_at: float) -> Check:
        """How `serial`'s follower does with path `sent_id`, sent at ROS time `sent_at`."""
        return path_check(ctx.world.robots[serial].base.state.follower, sent_id, sent_at, ctx.now())

    def _experiment_name(self) -> str:
        """The experiment's name as typed, made safe for a folder name; DEFAULT_EXPERIMENT if empty."""
        name = "".join(char if char.isalnum() or char in "-_" else "_" for char in str(self._experiment.value).strip())
        return name or DEFAULT_EXPERIMENT

    def _task_of(self, ctx: PluginContext, start, goal) -> str | None:
        """The chosen task's name if an attempt from `start` to `goal` counts for it, else None.

        It counts when it starts at the task's start, aims at its target, and a process task's step is selected.
        """
        task = BY_NAME.get(self._task.value)
        if task is None:
            return None
        moved, turned = _offset(start, task.start_in_lab())
        aimed, aimed_turn = _offset(goal, task.target_in_lab())
        if moved > TASK_POSITION or turned > TASK_YAW or aimed > 0.01 or aimed_turn > math.radians(1.0):
            return None
        if not task.virtual and self._process_index(ctx) != task.process_step:
            return None
        return task.name

    def _process_index(self, ctx: PluginContext) -> int | None:
        """The step selected in the process plugin; None if no process is loaded."""
        process = ctx.require("process")
        return process.index if process.loaded is not None else None

    def _environment(self, ctx: PluginContext, serial: str) -> str:
        """Whether `serial` is the simulator ("sim") or a robot ("real")."""
        return "sim" if "husky_sim" in ctx.world.robots[serial].base.node_names() else "real"

    def _current_pose(self, ctx: PluginContext) -> tuple[float, float, float]:
        """Where the chosen robot's base is now: x, y in metres and yaw in radians, world frame.

        ? From kinematics (last mocap fix, else the default pose), for drawing and as the plan's start.
        """
        base = ctx.kinematics.base_pose(self.serial)
        return float(base.position[0]), float(base.position[1]), yaw_from_xyzw(base.orientation)

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
        self._debug_view.show(self._debug if self._debug_on.value else None)
        self._draw_ghosts(ctx)

    def _draw_floor(self, ctx: PluginContext) -> None:
        """The plan in blue until committed, the sent path in green, the robot's trail, the follower's points."""
        self._markers.paths(None if self.plan_state == "sent" else self._preview, self._sent)
        if self._trace is None:
            self._markers.trail(None)
        else:
            self._markers.trail(self._trace.values(f"{self._sent_serial}_floor_pose")[:, :2])
        follower = None if self._sent_serial is None else \
            live_report(ctx.world.robots[self._sent_serial].base.state.follower, ctx.now())
        self._markers.follower(follower if follower is not None and follower.path_id == self._sent_id else None)

    def _status_html(self, ctx: PluginContext) -> str:
        """One line of chips: the robot, whether it is tracked, the plan's state and the last run's."""
        tracked = ctx.world.robots[self.serial].base.state.tracked
        chips = chip(escape(self.serial), SECTION_CTRL)
        chips += chip("tracked" if tracked else "not tracked", OK if tracked else BUSY,
                      "" if tracked else "planning needs the base tracked by mocap")
        # * The run's chip replaces "sent", and stays while the next plan is made.
        if self.plan_state != "sent":
            chips += self._plan_chip()
        if self._sent_id is not None:
            chips += check_chip(self._path_check(ctx, self._sent_serial, self._sent_id, self._sent_at))
        return block(chips)

    def _details_html(self) -> str:
        """Numbers of the target and the plan, and the last message, in a fixed number of rows."""
        x, y, yaw = self._target.pose
        target = f"target  {x:+7.3f} {y:+7.3f} {math.degrees(yaw):+7.1f}°" if self._has_target else "target  —"
        if self.path is None:
            path = "path    —"
            at = "at t    —"
        else:
            px, py, pyaw = self.path.sample(self._t)
            path = f"path    {self.path.length:6.2f} m  {self.path.duration:5.1f} s  {len(self.path.points)} waypoints"
            at = f"at t    {px:+7.3f} {py:+7.3f} {math.degrees(pyaw):+7.1f}°"
        return block(values(target, path, at) + self._message_html())


def _offset(pose, reference) -> tuple[float, float]:
    """How far apart two (x, y, yaw) floor poses are: metres, and radians of turn (at most pi)."""
    dx, dy = pose[0] - reference[0], pose[1] - reference[1]
    return math.hypot(dx, dy), abs(math.atan2(math.sin(pose[2] - reference[2]), math.cos(pose[2] - reference[2])))


def _differs(pose, reference) -> bool:
    """Whether two (x, y, yaw) floor poses differ by more than STALE_POSITION or STALE_YAW."""
    moved, turned = _offset(pose, reference)
    return moved > STALE_POSITION or turned > STALE_YAW
