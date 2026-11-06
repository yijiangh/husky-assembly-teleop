"""
The headless base planner: plans a base path on request, sends it to the onboard follower, and records both.

Its panel is `panel.BasePlannerPanel`; other plugins use it as `BasePlannerContract`.

- Commit sends the path to the robot's onboard path follower (crl_husky `pure_pursuit`), geometric, capped at the
  planning speeds, and follows the run until the follower is done.
- With `record` in the request, every plan attempt is saved under the experiment's name, found or not, and a
  committed plan's run is added to it once the follower is done (`recording`,
  doc/experiments/base_planner_manual.md). An attempt counts for the request's task when it starts at the task's
  start and aims at its target.
"""

from __future__ import annotations

import asyncio
import math
import threading
from dataclasses import asdict, dataclass, replace
from typing import TYPE_CHECKING, Any, Callable

import numpy as np
from crl_husky.follower_path import FollowerPath

from ...config import short_serial
from ...drive import drive_folder, folder_name, started_at
from ...plugin_api import signals
from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...plugin_api.trace import Trace
from ...robot_interface.base import BaseInterface
from ...robot_interface.soft_stop import MotionRefused
from ...shared.base_planner_contract import BasePlanRequest
from ...shared.planner.planner_search import PlanResult, TIME_LIMIT
from ...shared.planner.plan import Plan
from ...shared.planner.planner_search import Search, WAIT_MARGIN, unmeasured, with_guesses
from ...shared.planner.planner_service import PlannerService
from ...ui.pose_input import yaw_from_xyzw
from ...ui.style import hover
from ...world.checks import BAD, GOOD, WARN, Check
from ...world.clearance import ALL_ARMS, BASE_DRIVE, BASE_PLAN_DISTANCE
from ...world.follower import path_check
from ...world.kinematics import STOW_TOLERANCE, from_stow
from ...world.stand_ins import with_stand_ins
from scaffolding_assembly_core.ids import robot_id
from crl_husky.base_model import load_model
from .algorithms.base import FOLLOWER_MODEL
from .follow import followed, thinned, xicr_range
from .path import MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED, BasePath, densify, slowed, speed_scales
from .algorithms import ALGORITHMS, Algorithm, Debug
from ...shared.recordings.runs import load_description, save_recording, write_description
from .recording import RECORDING_FOLDER, attempt_folder, save_plan
from .tasks import BY_NAME, TASK_ORIGIN
from .world import CLEARANCE_STEP, PlanningWorld

if TYPE_CHECKING:
    from ...shared.process_contract import ProcessContract

#: The plan goes stale once the base moves this far from its planned start.
STALE_POSITION = 0.05              # m
STALE_YAW = math.radians(3.0)      # rad
#: The plan is for the arms as they are: it goes stale once a joint moves this far from where it was planned.
STALE_JOINT = math.radians(2.0)    # rad

#: Narrow passages: clearance along the path, as planned and as followed, is measured every NARROW_STEP metres (and
#: radians). A traced pose counts for the waypoint nearest it in position and heading (TRACE_TURN metres per radian)
#: within TRACE_AHEAD of progress (metres driven plus TRACE_TURN per radian turned) beyond the last one matched.
NARROW_STEP = 0.05
TRACE_TURN = 0.3   # m/rad
TRACE_AHEAD = 0.5  # m
#: A target with less room than the margin may move up to GOAL_SHIFT metres and GOAL_TURN radians, in GOAL_STEP and
#: GOAL_TURN_STEP steps, to the nearest pose with nearly the most room there (within GOAL_GIVE of it): the arms'
#: live IK takes up a few centimetres, and the guide counts a drive within 6 cm and 6 degrees as arrived.
GOAL_SHIFT, GOAL_STEP = 0.04, 0.02            # m
GOAL_TURN, GOAL_TURN_STEP = math.radians(4.0), math.radians(2.0)
GOAL_GIVE = 0.01                              # m

#: With a narrow margin, what the start or target is closer to than it keeps ARRIVAL_SLACK less than the end has (room
#: for the follower to arrive a little off), but never less than half of it.
ARRIVAL_SLACK = 0.01  # m

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
    return base.send_path(*path.points.T, None, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED, path.speed_scale)


@dataclass(eq=False)
class Run:
    """The last commit to one robot, for its panel: the path sent, the robot's samples, and how it ended.

    Nothing sent (refused): no path, and `ended` says why.
    """

    #: The follower's cut of the path sent, its id, and the ROS time it went out.
    path: FollowerPath | None = None
    path_id: int | None = None
    sent_at: float = 0.0
    trace: Trace | None = None
    #: How the run ended, once it did; until then the follower's reports tell (`path_check`).
    ended: Check | None = None
    #: The ROS time it ended, once it did.
    ended_at: float | None = None


@register
class BasePlannerPlugin(PlannerService[BasePlanRequest]):
    """Plans base paths in a private PyBullet world, sends them to the onboard follower, and records them."""

    name = "base_planner"
    experimental = True
    worker_name = "base-plan"
    # * The follower takes a new path at any time: a new commit to a driving robot replaces its path.
    replaces_running = True
    kind = BASE_DRIVE
    # * For the selected process step, recorded with each attempt and needed for a process task to count.
    requires = ("process",)

    def __init__(self):
        """No plan and nothing sent yet."""
        super().__init__()
        # ! Worker thread only (sync, search, close).
        self._world = PlanningWorld()
        self._algorithms: list[Algorithm] = [algorithm() for algorithm in ALGORITHMS]
        #: The last commit to each robot, by serial, for its run chip and floor drawing; none before a commit.
        self._runs: dict[str, Run] = {}
        # The last search's debug record, once it returned.
        self._debug: Debug | None = None
        # The ready plan's setup, planning time and recorded folder (None if not recorded), for its run.
        self._plan_record: dict | None = None
        # The ready plan's path and the joints it was planned with, for `stale_reason`; `forget` drops them.
        self._start_joints: tuple[BasePath, dict[str, float]] | None = None
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
            return Search(label=f"plan {short_serial(serial)}", message=f"planning for {short_serial(serial)}…",
                          work=lambda: PlanResult(None, reason, 0.0), time_limit=WAIT_MARGIN,
                          accepted=lambda _result: ("", False))
        debug = Debug(on=request.debug)
        # * Take the snapshot here, on the main thread; the worker syncs from it.
        # * The robot is planned as it is: its arms, and what it holds.
        snapshot, guessed = with_guesses(ctx.scene.snapshot, robot_id(serial))
        config = next(robot for robot in ctx.config.robots if robot.serial == serial)
        # * Without a follower model chosen, plan with the one the robot's follower assumes.
        settings = replace(settings, follower_model=settings.follower_model or config.base_model)
        away = from_stow(snapshot.robots[robot_id(serial)], config)
        if request.record and away > STOW_TOLERANCE:
            guessed = guessed + [f"arm {math.degrees(away):.0f}° from its stow pose, where experiments hold it"]
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
        folder = None
        if request.record and self._recordings is not None:
            try:
                folder = attempt_folder(self._recordings, setup["experiment"], serial, task, environment)
            except OSError as error:
                ctx.log_error(f"base planner: attempt not recorded: {error}")
        # The worker's error saving the attempt, for the main thread to log.
        not_saved: list[str] = []
        # The other robots not measured and their stand-ins, from the worker (`with_stand_ins`).
        others: list[str] = []
        # How the target moved for room, from the worker (`roomier_goal`); empty if it did not.
        moved: list[str] = []

        def work() -> PlanResult:
            # * Only an absent robot is left with problems; an untracked base or unknown joints were guessed.
            problems = snapshot.robots[robot_id(serial)].acting_problems()
            if problems:
                why = f"{short_serial(serial)} can't be planned for: {'; '.join(problems)}"
                result, world = PlanResult(None, why, 0.0), None
            else:
                stand_ins = with_stand_ins(snapshot, {robot_id(serial)})
                others[:] = setup["stand_ins"] = list(stand_ins.warnings)
                self._world.sync(stand_ins.scene)
                own, world = robot_id(serial), self._world
                # * A target with little room moves a little to where there is more (`roomier_goal`).
                goal_here, note = roomier_goal(world, own, goal, settings.margin)
                moved[:] = [note] if note else []
                setup["goal_planned"] = list(goal_here)
                # * Narrow passages may come as close as the narrow margin (lattice); only what the start or target
                #   is already closer to keeps less (`end_margins`).
                narrow = min(settings.margin, getattr(settings, "narrow_margin", settings.margin))
                close = end_margins(world.close_objects(own, (start, goal_here), narrow), narrow < settings.margin)
                setup["close"] = {world.snapshot.label(object_id): margin for object_id, margin in close.items()}
                with world.keeping(own, narrow, close):
                    result = algorithm.plan(world, own, start, goal_here, abort, settings, debug)
                if result.path is not None:
                    kept = {world.snapshot.label(object_id): margin for object_id, margin in close.items()}
                    model = load_model(settings.follower_model or FOLLOWER_MODEL)
                    slowed_path = narrowed(world, own, result.path, settings.margin, kept, model, settings.x_icr_spread,
                                           abort)
                    result = (replace(result, path=slowed_path) if slowed_path is not None
                              else PlanResult(None, "cancelled", result.seconds))
                if result.path is None and others:
                    result = replace(result, reason=f"{result.reason} ({unmeasured(others)})")
            if folder is not None:
                try:
                    save_plan(folder, world, robot_id(serial), setup, result, debug.notes)
                except OSError as error:
                    not_saved.append(str(error))
            return result

        def finished() -> None:
            self._debug = debug
            for line in others:
                ctx.log_warn(f"planning for {serial}: {line}")
            if not_saved:
                ctx.log_error(f"base planner: plan not saved: {not_saved[0]}")

        def accepted(result: PlanResult) -> tuple[str, bool]:
            self._plan_record = {"folder": folder, "setup": setup, "seconds": result.seconds}
            self._start_joints = (result.path, dict(snapshot.robots[robot_id(serial)].joints))
            path, how = result.path, algorithm.label + (" (straight)" if result.direct else "")
            stand_ins = f" ({unmeasured(others)})" if others else ""
            shift = f"; {moved[0]}" if moved else ""
            return (f"{how}: {path.length:.2f} m, {path.duration:.1f} s, found in {result.seconds:.1f} s{guess}"
                    f"{stand_ins}{shift}{close_note(path, settings.margin)}"), \
                bool(guessed or others or path.close or moved)

        return Search(label=f"plan {serial}", message=f"planning for {short_serial(serial)}…{guess}", work=work,
                      # * The lattice searches both ways, each up to TIME_LIMIT.
                      time_limit=2.0 * TIME_LIMIT + WAIT_MARGIN, accepted=accepted, finished=finished)

    def stale_reason(self, ctx: PluginContext, plan: Plan[BasePlanRequest]) -> str:
        """Stale once the base moves away from the plan's start, or an arm moves from where it was planned."""
        moved, turned = _offset(floor_pose(ctx, plan.request.serial), plan.path.start)
        if moved > STALE_POSITION or turned > STALE_YAW:
            return (f"the robot moved {moved * 100:.1f} cm, {math.degrees(turned):.1f}° since planning (replans beyond "
                    f"{STALE_POSITION * 100:.0f} cm or {math.degrees(STALE_YAW):.0f}°)")
        if self._start_joints is not None and self._start_joints[0] is plan.path:
            now = ctx.kinematics.joints(plan.request.serial)
            if any(abs(math.remainder(now[name] - value, math.tau)) > STALE_JOINT
                   for name, value in self._start_joints[1].items() if name in now):
                return f"the arm moved since planning (replans beyond {math.degrees(STALE_JOINT):.0f}°)"
        return ""

    def forget(self, plan: Plan[BasePlanRequest]) -> None:
        """The debug record goes with the plan; a sent path keeps driving (Stop all stops it), and stays drawn."""
        self._debug = None
        self._plan_record = None
        self._start_joints = None

    def commit_refused(self, plan: Plan[BasePlanRequest], message: str) -> None:
        """A commit refused before `execute` ran (e.g. another plugin holds the lease): its run chip says refused."""
        self._refused(self._ctx, plan.request.serial, message)

    def run_check(self, ctx: PluginContext, serial: str) -> Check | None:
        """The run chip of `serial`'s last commit: how it ended, else how the follower does; None before a commit."""
        run = self._runs.get(serial)
        if run is None:
            return None
        return run.ended or self._path_check(ctx, serial, run.path_id, run.sent_at)

    # --- --- --- --- --- RUN (task) --- --- --- --- ---

    async def execute(self, ctx: PluginContext, plan: Plan[BasePlanRequest],
                      answer: Callable[[bool, str], None]) -> None:
        """Send the plan to the follower, then record the robot until the follower is done; save it if recorded.

        Recording and, committed with Clearance ticked, a clearance start before the send (`commit_clearance`): a
        soft stop within STOP_DISTANCE, or half the plan's margin when that is below BASE_PLAN_DISTANCE, so a path
        followed a little off does not trip it; a start already that close refuses it. Not checked against itself: the
        arms stand still. Refused while the base may not follow a path.
        With the robot itself in `unknown_allowed` (Commit's second press), its unknown arms are checked where last
        seen, else stowed.

        ? The margin is no stopping distance: at the planning speed the robot rolls ~10 cm after a trip. Revisit
          with the speed-aware clearance (tasks/2026-10-21_collision_view.md, follow-ups).
        """
        request, path, record = plan.request, plan.path, self._plan_record
        serial = request.serial
        robot, allowed = ctx.world.robots[serial], self.unknown_allowed(plan)
        # * The last run's chip goes: every way out below sets this run's, or none for a replaced commit.
        self._runs.pop(serial, None)
        if problem := robot.base.follow_problem():
            answer(False, f"not committed: {short_serial(serial)}: {problem}")
            self._refused(ctx, serial, f"not committed: {short_serial(serial)}: {problem}")
            return
        sent_id, sent_at, started = None, ctx.now(), started_at()
        outcome, rec, clearance, run = "stopped", None, None, None
        try:
            with ctx.record(signals.base_floor_pose(robot), signals.follower_position_error(robot),
                            signals.follower_yaw_error(robot), signals.follower_command(robot),
                            signals.follower_progress(robot), signals.follower_command(robot, every_message=True),
                            signals.follower_state(robot, every_message=True),
                            signals.base_floor_pose(robot, every_message=True),
                            signals.wheel_odometry(robot, every_message=True)) as rec, \
                    self.commit_clearance(ctx, plan, unknown_arms=ALL_ARMS if robot_id(serial) in allowed else (),
                                          check_self=False) as clearance:
                await self.first_check(ctx, clearance)
                try:
                    sent_id = send_to_onboard_follower(robot.base, path)
                except ValueError as error:
                    answer(False, f"not committed: {error}")
                    self._refused(ctx, serial, f"not committed: {short_serial(serial)}: {error}")
                    return
                sent_at = ctx.now()
                run = self._runs[serial] = Run(FollowerPath.from_poses(*path.points.T, None, path.speed_scale), sent_id,
                                               sent_at, rec)
                scaled_as = getattr(robot.base, "scaled_as", "")
                line = (f"base path {sent_id} sent to {serial}: {path.length:.2f} m, {len(path.points)} waypoints"
                        + (f", slowed {scaled_as}" if scaled_as else ""))
                # * An old follower drives the whole path at the slowest scale: it takes that long.
                whole = path.speed_scale is not None and scaled_as.startswith("whole path")
                (ctx.log_warn if whole else ctx.log_info)(line)
                answer(True, f"sent to {short_serial(serial)}'s follower (path {sent_id})")
                duration = (slowed(path, np.full(len(path.points), path.speed_scale.min())).duration if whole
                            else path.duration)
                limit = duration * RUN_FACTOR + RUN_MARGIN
                try:
                    # * Under way is WARN (sent or following); done, aborted, replaced or silent ends the run.
                    await ctx.wait_until(lambda: self._path_check(ctx, serial, sent_id, sent_at).level != WARN,
                                         timeout_s=limit, description="the follower to finish")
                    check = self._path_check(ctx, serial, sent_id, sent_at)
                    # * A finished run's outcome is "done" alone: the analysis counts it by that word.
                    outcome = check.label if check.level == GOOD else f"{check.label}: {check.detail}"
                except WaitTimeout:
                    outcome = "timed out: still following"
                    check = Check("timed out", BAD, f"{short_serial(serial)} still followed the path after "
                                                    f"{limit:.0f} s; its path was stopped")
                    robot.base.stop_path()
                self._end(ctx, serial, run, check)
                await ctx.sleep(SETTLE)
        except asyncio.CancelledError:
            if refusal := self.start_refusal(clearance):
                answer(False, refusal)
                ctx.log_warn(f"{serial}: {refusal}")
            # ! Stop the path, unless a new commit replaces this run: its path replaces this one on the follower.
            # * Cancelled while settling, the run had ended: its outcome and chip stay.
            if run is None or run.ended is None:
                outcome = "replaced by a new commit" if self.replaced() else "stopped"
                if clearance is not None and clearance.problem:
                    outcome = f"{'refused' if clearance.refused else 'soft stopped'}: {clearance.problem}"
                if not self.replaced():
                    self._end(ctx, serial, run, self.cancelled_check(ctx, serial, serial, clearance,
                                                                     "the commit was cancelled; the follower stopped"))
            if sent_id is not None and not self.replaced():
                robot.base.stop_path()
            raise
        except Exception as error:
            if run is None and isinstance(error, MotionRefused):  # e.g. the clearance cannot check the robot
                self._refused(ctx, serial, f"nothing sent: {error}")
            else:
                self._end(ctx, serial, run, Check("failed", BAD, hover(f"{short_serial(serial)}: {error}")))
            raise
        finally:
            if sent_id is not None:
                ctx.log_info(f"base path {sent_id} on {serial}: {outcome}")
                if request.record and self._recordings is not None and rec is not None and len(rec):
                    self._save(ctx, request, path, rec, outcome, started, sent_at, record or {})

    def _save(self, ctx: PluginContext, request: BasePlanRequest, path: BasePath, rec: Trace, outcome: str,
              started: str, sent_at: float, plan: dict) -> None:
        """Add a run to its plan's attempt: recording.npz (per tick and per message, and the path), and the run.

        The run goes into the attempt's experiment.json.

        A plan not recorded, or committed a second time, gets a folder of its own with what is known of the plan.
        """
        serial = request.serial
        follower = ctx.world.robots[serial].base.state.follower
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
            save_recording(folder, rec, serial, {"path_poses": path.points, "path_t": path.times},
                           ctx.config.recording_format)
            write_description(folder, {**description, "run": run})
        except OSError as error:
            ctx.log_error(f"base planner: run not saved: {error}")
            return
        ctx.log_info(f"base planner: run saved to {folder}")

    def _refused(self, ctx: PluginContext, serial: str, message: str) -> None:
        """Show a commit to `serial` that sent nothing on its run chip; logged, as the hover may be cut."""
        ctx.log_warn(message)
        self._runs[serial] = Run(ended=Check("refused", BAD, hover(message)))

    def _end(self, ctx: PluginContext, serial: str, run: Run | None, check: Check) -> None:
        """End `run` (None: nothing sent) with `check`, unless a newer commit to `serial` took its place."""
        if run is None:
            self._runs[serial] = Run(ended=check, ended_at=ctx.now())
        elif self._runs.get(serial) is run:
            run.ended, run.ended_at = check, ctx.now()

    # --- --- --- --- --- READING --- --- --- --- ---

    def plan_margin(self, plan: Plan[BasePlanRequest]) -> float | None:
        """How far the path keeps from everything, metres; None for a request with bad settings.

        The smallest margin it kept (`BasePath.margin`), else the request's.
        """
        if plan.path is not None and plan.path.margin is not None:
            return plan.path.margin
        try:
            return self._algorithm(plan.request)[1].margin
        except ValueError:  # * its plan failed for that reason already
            return None

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
            # * Without a margin, the one the panel starts at: every caller plans the same way.
            return algorithm, replace(algorithm.settings(), **{"margin": BASE_PLAN_DISTANCE, **dict(request.settings)})
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
        """Whether `serial`'s base is simulated ("sim": its simulator announces itself) or real ("real")."""
        return "sim" if ctx.world.robots[serial].base.simulation is not None else "real"


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


def kept_margin(distance: float) -> float:
    """The margin a path keeps from something its start or target is `distance` from: just below, to CLEARANCE_STEP."""
    return max(0.0, math.floor((distance - 1e-6) / CLEARANCE_STEP) * CLEARANCE_STEP)


def close_note(path: BasePath, margin: float) -> str:
    """The plan line's note on a path keeping less from some things; "" if it keeps the margin.

    E.g. "; keeps only 3.5 cm from bars/B25 (the start or target is that close, or a narrow passage); 10 cm from
    everything else; at 25 % speed over 1.2 m".
    """
    if not path.close:
        return ""
    label, kept = path.close[0]
    more = f" (and {len(path.close) - 1} more)" if len(path.close) > 1 else ""
    slow = ""
    if path.speed_scale is not None:
        legs = np.hypot(*np.diff(path.points[:, :2], axis=0).T)
        slow_m = float(legs[np.minimum(path.speed_scale[:-1], path.speed_scale[1:]) < 1.0].sum())
        slow = f"; at {path.speed_scale.min() * 100:.0f} % speed or more over {slow_m:.1f} m"
    return (f"; keeps only {kept * 100:g} cm from {label}{more} (the start or target is that close, or a narrow "
            f"passage); {margin * 100:g} cm from everything else{slow}")


def trace_indices(points: np.ndarray, poses) -> list[int]:
    """For each pose of a trace of the path through `points`, in order, the index of the waypoint it is at.

    By progress, not position alone: the nearest waypoint in position and heading (TRACE_TURN metres per radian), among
    those within TRACE_AHEAD of progress beyond the last one matched, the later one on a tie. So a turn on the spot,
    whose waypoints share a spot, and a path that passes a spot twice keep their order.
    """
    points = np.asarray(points, dtype=float)
    steps = np.hypot(*np.diff(points[:, :2], axis=0).T) + TRACE_TURN * np.abs(np.diff(points[:, 2]))
    progress = np.concatenate([[0.0], np.cumsum(steps)])
    out, index = [], 0
    for pose in poses:
        last = int(np.searchsorted(progress, progress[index] + TRACE_AHEAD, side="right"))
        # * At least the next waypoint too: a sparse path (RRT) turns on the spot in one leg, more than TRACE_AHEAD.
        near = points[index:max(last, index + 2)]
        turn = np.abs(np.angle(np.exp(1j * (near[:, 2] - pose[2]))))
        cost = np.hypot(*(near[:, :2] - np.asarray(pose[:2])).T) + TRACE_TURN * turn
        index += int(len(cost) - 1 - np.argmin(cost[::-1]))
        out.append(index)
    return out


def end_margins(distances: dict[str, float], narrow: bool) -> dict[str, float]:
    """The margins kept from what the start or target is already closer to than the plan's margin, by object.

    As much as the end has (`kept_margin`); with a narrow margin (`narrow`), ARRIVAL_SLACK less, but at least half.
    """
    out = {}
    for object_id, distance in distances.items():
        kept = kept_margin(distance)
        out[object_id] = max(kept - ARRIVAL_SLACK, kept / 2.0) if narrow else kept
    return out


def roomier_goal(world: PlanningWorld, own: str, goal, margin: float) -> tuple[tuple[float, float, float], str]:
    """The target to plan to: `goal`, or the nearest pose near it with nearly the most room there.

    Only a target with less room than `margin` moves, within GOAL_SHIFT and GOAL_TURN, and only to a pose with more
    room; of the poses within GOAL_GIVE of the most room found, the nearest (a degree counts like 5 mm). True
    distances, worker thread, world synced.

    Returns:
        tuple: The pose (x, y, yaw), and how it moved for the plan's line ("" if it did not).
    """
    goal = tuple(float(value) for value in goal)
    here, _ = world.nearest(own, goal, margin)
    if here >= margin - 1e-9:
        return goal, ""
    shifts = np.arange(-GOAL_SHIFT, GOAL_SHIFT + 1e-9, GOAL_STEP)
    turns = np.arange(-GOAL_TURN, GOAL_TURN + 1e-9, GOAL_TURN_STEP)
    room = [(world.nearest(own, (goal[0] + dx, goal[1] + dy, goal[2] + turn), margin)[0], dx, dy, turn)
            for dx in shifts for dy in shifts if math.hypot(dx, dy) <= GOAL_SHIFT + 1e-9 for turn in turns]
    most = max(clear for clear, *_ in room)
    if most <= here + 1e-9:
        return goal, ""
    clear, dx, dy, turn = min((entry for entry in room if entry[0] >= most - GOAL_GIVE - 1e-9),
                              key=lambda entry: math.hypot(entry[1], entry[2]) + 0.005 * abs(math.degrees(entry[3])))
    if math.hypot(dx, dy) < 1e-9 and abs(turn) < 1e-9:
        return goal, ""
    return ((goal[0] + dx, goal[1] + dy, goal[2] + turn),
            f"target moved {math.hypot(dx, dy) * 100:.0f} cm, {math.degrees(turn):+.0f}° for room ({here * 100:.0f} "
            f"cm there, {clear * 100:.0f} cm here)")


def narrowed(world: PlanningWorld, own: str, path: BasePath, margin: float, kept: dict[str, float], model: dict,
             spread: float, abort: threading.Event | None = None) -> BasePath | None:
    """`path` slowed where it comes closer than `margin`, with the smallest margin and the close objects it keeps.

    Measures the true clearance up to `margin` along each leg (every NARROW_STEP), and along the path as the follower
    drives it at the model's xICR and both ends of the range `spread` (`follow.followed`), each traced pose counting for
    the nearest waypoint: where the robot really drifts, it slows down too. Worker thread, world synced.

    Args:
        world: The planning world, synced to the plan's snapshot.
        own: The robot planned for, by id.
        path: The planned path.
        margin: The request's margin, metres.
        kept: Labels the start or target is already closer to, with the margins kept from them.
        model: The follower model the path was planned with (`crl_husky.base_model.load_model`).
        spread: How far the true xICR may be from the model's, either way, metres.
        abort: Ends the measuring early, returning None; None: never.

    Returns:
        BasePath | None: The path slowed, with its margin and close list; None if aborted.
    """
    points = path.points
    at_points = np.full(len(points), margin)
    close = dict(kept)

    def note(where) -> None:
        least, label = world.nearest(own, where, margin)
        if label is not None:
            close[label] = min(close.get(label, margin), kept_margin(least))

    for i in range(len(points) - 1):
        if abort is not None and abort.is_set():
            return None
        poses = [tuple(points[i])] + densify([points[i], points[i + 1]], NARROW_STEP, NARROW_STEP)
        least, where = world.clearance(own, poses, margin)
        if where is not None:
            at_points[i:i + 2] = np.minimum(at_points[i:i + 2], least)
            note(where)
    x_model = model["xICR"]
    for icr in sorted({*xicr_range(x_model, spread), x_model}):
        trace, _ = followed(points, icr, model, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)
        poses = thinned(trace, NARROW_STEP, NARROW_STEP)
        for pose, index in zip(poses, trace_indices(points, poses)):
            if abort is not None and abort.is_set():
                return None
            least, where = world.clearance(own, [tuple(pose)], margin)
            if where is not None:
                at_points[index] = min(at_points[index], least)
                note(where)
    nearest = tuple(sorted(close.items(), key=lambda item: item[1]))
    least = min([margin, *close.values()])
    return replace(slowed(path, speed_scales(points, at_points, margin)), margin=least, close=nearest)
