"""
The headless base planner: plans a base path on request, sends it to the onboard follower, and records both.

Its panel is `panel.BasePlannerPanel`; other plugins use it as `BasePlannerContract`.

- Commit sends the path to the robot's onboard path follower (crl_husky `pure_pursuit`), geometric, capped at the
  planning speeds, and follows the run until the follower is done.
- With `record` in the request, every plan attempt is saved under the experiment's name, found or not, and a
  committed plan's run is added to it once the follower is done (`recording`, doc/base_planner_manual.md). An attempt
  counts for the request's task when it starts at the task's start and aims at its target.
"""

from __future__ import annotations

import asyncio
import math
import threading
from dataclasses import asdict, replace
from typing import TYPE_CHECKING, Any

from crl_husky.follower_path import FollowerPath

from ...drive import drive_folder, folder_name, started_at
from ...plugin_api import signals
from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...plugin_api.trace import Trace
from ...robot_interface.base import BaseInterface
from ...shared.base_planner_contract import BasePlanRequest
from ...shared.planner.planner_search import PlanResult, TIME_LIMIT
from ...shared.planner.plan import Plan
from ...shared.planner.planner_search import Search, WAIT_MARGIN, with_guesses
from ...shared.planner.planner_service import PlannerService
from ...ui.pose_input import yaw_from_xyzw
from ...world.checks import WARN, Check
from ...world.follower import live_report, path_check
from ...world.mocap import pose_lost
from scaffolding_assembly_core.ids import robot_id
from .path import MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED, BasePath
from .algorithms import ALGORITHMS, Algorithm, Debug
from .recording import RECORDING_FOLDER, attempt_folder, load_description, save_plan, write_description
from .tasks import BY_NAME, TASK_ORIGIN
from .world import PlanningWorld, stowed

if TYPE_CHECKING:
    from ...shared.process_contract import ProcessContract

#: The plan goes stale once the base moves this far from its planned start.
STALE_POSITION = 0.05              # m
STALE_YAW = math.radians(3.0)      # rad

#: A run ends after the planned duration times RUN_FACTOR plus RUN_MARGIN seconds, done or not; the recording then
#: goes on for SETTLE seconds, while the robot comes to a stop.
RUN_FACTOR = 2.0
RUN_MARGIN = 10.0
SETTLE = 2.0

#: The experiment name for an empty one.
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

    Raises:
        ValueError: If the path holds a value that is not finite; nothing is sent.
    """
    return base.send_path(*path.points.T, None, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)


@register
class BasePlannerPlugin(PlannerService[BasePlanRequest]):
    """Plans base paths in a private PyBullet world, sends them to the onboard follower, and records them."""

    name = "base_planner"
    experimental = True
    worker_name = "base-plan"
    # * For the selected process step, recorded with each attempt and needed for a process task to count.
    requires = ("process",)

    def __init__(self):
        """No plan and nothing sent yet."""
        super().__init__()
        # ! Worker thread only (sync, search, close).
        self._world = PlanningWorld()
        self._algorithms: list[Algorithm] = [algorithm() for algorithm in ALGORITHMS]
        # The path last sent: robot, follower's cut, id, ROS time it went out; the run following it, and its samples.
        self._sent_serial: str | None = None
        self._sent: FollowerPath | None = None
        self._sent_id: int | None = None
        self._sent_at = 0.0
        self._run_task: asyncio.Task | None = None
        self._trace: Trace | None = None
        # The last search's debug record, once it returned.
        self._debug: Debug | None = None
        # The ready plan's setup, planning time and recorded folder (None if not recorded), for its run.
        self._plan_record: dict | None = None
        # Where attempts are recorded; None if the Drive folder is missing.
        self._recordings = None

    def setup(self, ctx: PluginContext) -> None:
        """Find the folder attempts are recorded in."""
        super().setup(ctx)
        try:
            self._recordings = drive_folder(ctx.config.drive_root, RECORDING_FOLDER)
        except FileNotFoundError as error:
            ctx.log_warn(f"base planner: runs cannot be recorded: {error}")

    # --- --- --- --- --- PLANNER SERVICE --- --- --- --- ---

    def make_search(self, ctx: PluginContext, request: BasePlanRequest, abort: threading.Event) -> Search:
        """Search from the robot's current pose to the request's goal, in the tick's snapshot."""
        serial, start, goal = request.serial, floor_pose(ctx, request.serial), tuple(request.goal)
        try:
            algorithm, settings = self._algorithm(request)
        except ValueError as error:
            reason = str(error)
            return Search(label=f"plan {serial}", message=f"planning for {serial}…",
                          work=lambda: PlanResult(None, reason, 0.0), time_limit=WAIT_MARGIN,
                          accepted=lambda _result: ("", False))
        debug = Debug(on=request.debug)
        # * Take the snapshot here, on the main thread; the worker syncs from it.
        snapshot, guessed = with_guesses(ctx.scene.snapshot, robot_id(serial))
        config = next(robot for robot in ctx.config.robots if robot.serial == serial)
        snapshot, away = stowed(snapshot, robot_id(serial), config)
        if away > STOW_TOLERANCE:
            guessed = guessed + [f"arm planned as stowed, {math.degrees(away):.0f}° away from it: stow it first"]
        guess = f" (guessed: {'; '.join(guessed)})" if guessed else ""
        if guessed:
            ctx.log_warn(f"planning for {serial} from a guess: {'; '.join(guessed)}")
        # * With `record`, the attempt gets its folder now; the worker saves it, found or not.
        environment, task = self._environment(ctx, serial), self._task_of(ctx, request.task, start, goal)
        setup = {"experiment": folder_name(request.experiment, DEFAULT_EXPERIMENT), "task": task, "robot": serial,
                 "environment": environment, "started": started_at(),
                 "conditions": {"task_origin": list(TASK_ORIGIN), "process_step": self._process_index(ctx),
                                "arm_from_stow": math.degrees(away),
                                "speed_caps": {"linear": MAX_LINEAR_SPEED, "angular": MAX_ANGULAR_SPEED}},
                 "algorithm": algorithm.label, "settings": asdict(settings), "start": list(start),
                 "goal": list(goal), "guessed": guessed}
        folder = attempt_folder(self._recordings, setup["experiment"], serial, task, environment) \
            if request.record and self._recordings is not None else None
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
            path, how = result.path, algorithm.label + (" (straight)" if result.direct else "")
            return (f"{how}: {path.length:.2f} m, {path.duration:.1f} s, "
                    f"found in {result.seconds:.1f} s{guess}"), bool(guessed)

        return Search(label=f"plan {serial}", message=f"planning for {serial}…{guess}", work=work,
                      time_limit=TIME_LIMIT + WAIT_MARGIN, accepted=accepted, finished=finished)

    def stale_reason(self, ctx: PluginContext, plan: Plan[BasePlanRequest]) -> str:
        """Stale once the base moves away from the plan's start."""
        moved, turned = _offset(floor_pose(ctx, plan.request.serial), plan.path.start)
        if moved > STALE_POSITION or turned > STALE_YAW:
            return (f"the robot moved {moved * 100:.1f} cm, {math.degrees(turned):.1f}° since planning (replans beyond "
                    f"{STALE_POSITION * 100:.0f} cm or {math.degrees(STALE_YAW):.0f}°)")
        return ""

    def send(self, ctx: PluginContext, plan: Plan[BasePlanRequest]) -> tuple[bool, str]:
        """Send the plan to the follower and follow its run; refused without mocap pose or follower, or e-stopped."""
        serial, path = plan.request.serial, plan.path
        base = ctx.world.robots[serial].base
        if lost := pose_lost(base.mocap_id, base.state, ctx.now()):
            return False, f"not committed: {serial} has no mocap pose: {lost}"
        if base.state.estopped:
            return False, f"not committed: {serial} is e-stopped"
        if live_report(base.state.follower, ctx.now()) is None:
            return False, f"not committed: no follower on {serial}; start crl_husky pure_pursuit"
        try:
            sent_id = send_to_onboard_follower(base, path)
        except ValueError as error:
            return False, f"not committed: {error}"
        if self._run_task is not None:
            self._run_task.cancel()  # the old run is saved as replaced
        self._sent_serial, self._sent = serial, FollowerPath.from_poses(*path.points.T)
        self._sent_id, self._sent_at = sent_id, ctx.now()
        self._run_task = ctx.spawn(f"follow {serial}", self._follow(ctx, plan.request, path, self._plan_record))
        ctx.log_info(f"base path {sent_id} sent to {serial}: {path.length:.2f} m, {len(path.points)} waypoints")
        return True, f"sent to {serial}'s follower (path {sent_id})"

    def forget(self, plan: Plan[BasePlanRequest]) -> None:
        """The debug record goes with the plan; a sent path keeps driving (soft stop stops it), and stays drawn."""
        self._debug = None
        self._plan_record = None

    # --- --- --- --- --- RUN (task) --- --- --- --- ---

    async def _follow(self, ctx: PluginContext, request: BasePlanRequest, path: BasePath, plan: dict | None) -> None:
        """Record the robot from Commit until the follower is done with the path, then save it if `request.record`.

        `plan` is the plan's `_plan_record`: its setup, planning time and recorded folder (None if not recorded).
        """
        serial = request.serial
        robot = ctx.world.robots[serial]
        sent_id, sent_at, started = self._sent_id, self._sent_at, started_at()
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
            if request.record and self._recordings is not None and rec is not None and len(rec):
                self._save(ctx, request, path, rec, outcome, started, sent_at, plan or {})

    def _save(self, ctx: PluginContext, request: BasePlanRequest, path: BasePath, rec: Trace, outcome: str,
              started: str, sent_at: float, plan: dict) -> None:
        """Add a run to its plan's attempt: recording.npz (samples, path, follower and wheel logs), and the run.

        The run goes into the attempt's experiment.json.

        A plan not recorded, or committed a second time, gets a folder of its own with what is known of the plan.
        """
        serial = request.serial
        base = ctx.world.robots[serial].base
        follower = base.state.follower
        run = {"started": started, "outcome": outcome, "sent_at": sent_at,
               "controller": follower.controller if follower is not None else "unknown"}
        folder = plan.get("folder")
        description = load_description(folder) if folder is not None and folder.is_dir() else None
        try:
            if description is None or description.get("run") is not None:
                setup = plan.get("setup") or {"experiment": folder_name(request.experiment, DEFAULT_EXPERIMENT),
                                              "task": None, "robot": serial,
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

    def _algorithm(self, request: BasePlanRequest) -> tuple[Algorithm, Any]:
        """The request's algorithm (the default for "") and its default settings with the request's values.

        Raises:
            ValueError: If the algorithm or a setting is unknown.
        """
        label = request.algorithm or self._algorithms[0].label
        algorithm = next((algorithm for algorithm in self._algorithms if algorithm.label == label), None)
        if algorithm is None:
            raise ValueError(f"unknown algorithm {label!r}")
        try:
            return algorithm, replace(algorithm.settings(), **dict(request.settings))
        except TypeError as error:
            raise ValueError(f"bad settings for {label}: {error}") from None

    def _path_check(self, ctx: PluginContext, serial: str, sent_id: int, sent_at: float) -> Check:
        """How `serial`'s follower does with path `sent_id`, sent at ROS time `sent_at`."""
        return path_check(ctx.world.robots[serial].base.state.follower, sent_id, sent_at, ctx.now())

    def _task_of(self, ctx: PluginContext, name: str | None, start, goal) -> str | None:
        """Task `name` if an attempt from `start` to `goal` counts for it, else None.

        It counts when it starts at the task's start, aims at its target, and a process task's step is selected.
        """
        task = BY_NAME.get(name)
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
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        return selection.index if selection is not None else None

    def _environment(self, ctx: PluginContext, serial: str) -> str:
        """Whether `serial` is the simulator ("sim") or a robot ("real")."""
        return "sim" if "husky_sim" in ctx.world.robots[serial].base.node_names() else "real"


def floor_pose(ctx: PluginContext, serial: str) -> tuple[float, float, float]:
    """Where a robot's base is now: x, y in metres and yaw in radians, world frame.

    ? From kinematics (last mocap fix, else the default pose), for drawing and as the plan's start.
    """
    base = ctx.kinematics.base_pose(serial)
    return float(base.position[0]), float(base.position[1]), yaw_from_xyzw(base.orientation)


def differs(pose, reference) -> bool:
    """Whether two (x, y, yaw) floor poses differ by more than STALE_POSITION or STALE_YAW."""
    moved, turned = _offset(pose, reference)
    return moved > STALE_POSITION or turned > STALE_YAW


def _offset(pose, reference) -> tuple[float, float]:
    """How far apart two (x, y, yaw) floor poses are: metres, and radians of turn (at most pi)."""
    dx, dy = pose[0] - reference[0], pose[1] - reference[1]
    return math.hypot(dx, dy), abs(math.atan2(math.sin(pose[2] - reference[2]), math.cos(pose[2] - reference[2])))
