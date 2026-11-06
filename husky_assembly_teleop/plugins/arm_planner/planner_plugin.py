"""
The headless arm planner: plans a collision-free path for one arm to a joint goal and sends it as one trajectory.

Others use it through `ArmPlannerContract`; its panel is `arm_planner_panel`. The first plan for a robot loads its
models in a compas_fab world (several seconds).

! EXPERIMENTAL, not yet tested on a robot: Commit sends the path as one joint trajectory (`send_motion`), under an
  ARM_MOVE clearance on the robot (`commit_clearance`) when committed with Clearance ticked, so the base
  pose must be known to send. Paths keep PLAN_DISTANCE; only a pair start or target is closer for keeps what they have
  (`endpoint_margins`), and the clearance is then half the smallest (`ARM_MOVE`).
"""

from __future__ import annotations

import asyncio
import math
import threading
from dataclasses import replace
from typing import Callable, Sequence

import numpy as np

from ...config import short_serial
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...robot_interface.mocap_state import pose_lost
from ...robot_interface.soft_stop import MotionRefused
from ...shared.arm_planner_contract import MAX_SLOWDOWN, ArmPlanRequest
from ...shared.assembly_planning.ik import solve_ik
from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.assembly_planning.mirrors import MirrorPool
from ...shared.execution.send import ARRIVED, HOLD_NOT_SENT, INTERRUPTED, REFUSED, STOPPED, TIMEOUT, ArmMotion, \
    MotionCommand, send_motion
from ...world.collision_hit import CollisionHit
from ...shared.planner.planner_search import PlanResult
from ...shared.planner.plan import Plan
from ...shared.planner.planner_search import Search, WAIT_MARGIN, unmeasured, with_guesses
from ...shared.planner.planner_service import PlannerService
from ...ui.pose_input import yaw_from_xyzw
from ...ui.style import hover
from ...world.checks import BAD, GOOD, WARN, Check
from ...world.clearance import ARM_MOVE, PLAN_DISTANCE, Clearance, clearance_problem
from ...world.stand_ins import with_stand_ins
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from .planner import TIME_LIMIT, ArmPath, margin_note, path_hit, plan_arms

#: The plan goes stale once a joint moves this far from the planned start, or the base this far; the panel also
#: refuses a target this far from the planned goal.
STALE_JOINT = math.radians(2.0)    # rad
STALE_POSITION = 0.05              # m
STALE_YAW = math.radians(3.0)      # rad; 5 cm at 1 m reach, as STALE_POSITION
#: The run chip's label for each way a committed motion ends (`send_motion`).
MOTION_LABELS = {ARRIVED: "done", REFUSED: "refused", TIMEOUT: "timed out", STOPPED: "stopped short",
                 INTERRUPTED: "interrupted", HOLD_NOT_SENT: "not held"}
#: Extra time for the first plan of a robot, which loads the models, seconds.
LOAD_ALLOWANCE = 60.0


def arm_command(serial: str, arm_names: str | Sequence[str], path: ArmPath) -> MotionCommand:
    """A path as a command for its arms, each in the driver's joint order, timed as planned and previewed.

    Args:
        serial: Which robot.
        arm_names: Which arm, or arms (in the robot's arm order): one motion each, sent together.
        path: The path to follow; its first point must be where the arms are now (`START_TOLERANCE`), else an arm
            refuses it.
    """
    motions, travel = [], 0.0
    for arm_name in ([arm_names] if isinstance(arm_names, str) else arm_names):
        order = [path.joint_names.index(name) for name in arm_joint_names(arm_name)]
        points = path.points[:, order]
        travel = max(travel, float(np.max(np.sum(np.abs(np.diff(points, axis=0)), axis=0))))
        motions.append(ArmMotion(arm_name, points, path.velocities[:, order]))
    return MotionCommand(serial, tuple(motions), path.duration, path.fastest, path.duration, travel)


@register
class ArmPlannerPlugin(PlannerService[ArmPlanRequest]):
    """Plans a collision-free joint path for one arm and sends it to the arm."""

    name = "arm_planner"
    experimental = True
    worker_name = "arm-plan"
    kind = ARM_MOVE

    def __init__(self):
        """Start with no plan; the models load with the first plan of each robot."""
        super().__init__()
        #: Robots whose models the worker has loaded, so later plans need no load allowance.
        self._loaded: set[str] = set()
        #: The latest plan's path, the base's floor pose (x, y, yaw) it was planned from, and whether mocap tracked
        #: the base then; `stale_reason` compares the base.
        self._start_base: tuple[ArmPath, tuple[float, float, float], bool] | None = None
        #: How the last committed motion of each (serial, arm) goes, for the panel's run chip; none before a commit.
        self._motions: dict[tuple[str, str], Check] = {}
        # ! Worker thread only (sync, search, close).
        self._world = MirrorPool()

    def plan_margin(self, plan: Plan[ArmPlanRequest]) -> float | None:
        """How close the path comes to anything, as planned (`ArmPath.margin`); None without a path."""
        return None if plan.path is None else plan.path.margin

    def setup(self, ctx: PluginContext) -> None:
        """Let the mirrors log through the plugin."""
        super().setup(ctx)
        self._world.log = ctx.log_info  # ? the ROS logger is safe to call from the worker

    def make_search(self, ctx: PluginContext, request: ArmPlanRequest, abort: threading.Event) -> Search:
        """Search from where the arm is in the tick's snapshot to the request's goal."""
        serial, arm, goal = request.serial, request.arm, np.array(request.goals)
        arms, what = request.arms, " + ".join(request.arms)
        # * Take the snapshot here, on the main thread; the arm starts where it is in it.
        live = ctx.scene.snapshot
        snapshot, guessed = with_guesses(live, robot_id(serial))
        notes = f" (guessed: {'; '.join(guessed)})" if guessed else ""
        if guessed:
            ctx.log_warn(f"planning for {serial} {arm} from a guess: {'; '.join(guessed)}")
        tracked = live.robots[robot_id(serial)].base_tracked
        base = floor_pose(snapshot.robots[robot_id(serial)].base)
        loading = serial not in self._loaded
        # The other robots not measured and their stand-ins, from the worker (`with_stand_ins`).
        others: list[str] = []

        def work() -> PlanResult:
            # * Only an absent robot is left with problems; an untracked base or unknown joints were guessed.
            problems = snapshot.robots[robot_id(serial)].acting_problems()
            if problems:
                return PlanResult(None, f"{short_serial(serial)} can't be planned for: {'; '.join(problems)}", 0.0)
            stand_ins = with_stand_ins(snapshot, {robot_id(serial)})
            others[:] = stand_ins.warnings
            result = plan_arms(self._world.sync(stand_ins.scene, serial), stand_ins.scene, arms, goal, abort)
            if result.path is None and others:
                result = replace(result, reason=f"{result.reason} ({unmeasured(others)})")
            return result

        def finished() -> None:
            self._loaded.add(serial)
            for line in others:
                ctx.log_warn(f"planning for {serial} {arm}: {line}")

        def accepted(result: PlanResult) -> tuple[str, bool]:
            self._start_base = (result.path, base, tracked)
            how = "straight" if result.direct else "RRT"
            stand_ins = f" ({unmeasured(others)})" if others else ""
            message = (f"{how}: {result.path.duration:.1f} s, {result.path.rests} rests on the way, "
                       f"{margin_note(result.path.margin, result.path.close)}, "
                       f"found in {result.seconds:.1f} s{notes}{stand_ins}")
            # * A path keeping less than usual warns: its commit's clearance is smaller too (`close_note`).
            return message, bool(guessed or others or result.path.margin < PLAN_DISTANCE)

        return Search(label=f"plan {serial} {what}",
                      message=f"planning for {short_serial(serial)} {what}…{notes}"
                      + (" (loading the models first)" if loading else ""),
                      work=work, time_limit=TIME_LIMIT + WAIT_MARGIN + (LOAD_ALLOWANCE if loading else 0.0),
                      accepted=accepted, finished=finished)

    def set_duration(self, plan_id: int, seconds: float) -> None:
        """Re-time the latest plan to take `seconds`, without replanning (`ArmPlannerContract`)."""
        plan = self._plan
        if plan is None or plan.id != plan_id or plan.state != "ready":
            return
        path = plan.path
        slowed = path.slowed(min(max(seconds / path.fastest, 1.0), MAX_SLOWDOWN))
        # * The base kept for `stale_reason` belongs to the path by identity.
        if self._start_base is not None and self._start_base[0] is path:
            self._start_base = (slowed, *self._start_base[1:])
        self._plan = replace(plan, path=slowed)

    def check_path(self, serial: str, arm: str,
                   waypoints: Sequence[Sequence[float]], margin: float = 0.0) -> asyncio.Future[CollisionHit | None]:
        """Check a given joint path for one arm for collisions, in the world as it is now (`ArmPlannerContract`)."""
        future = asyncio.get_running_loop().create_future()
        path = np.array(waypoints, dtype=float)
        self._ctx.submit(f"check path {serial}", lambda: self._start_check(serial, arm, path, margin, future))
        return future

    def solve_goal(self, serial: str, arm: str, tool0: Pose,
                   reference: Sequence[float] | None = None) -> asyncio.Future[tuple[tuple[float, ...] | None, str]]:
        """Joints for `arm` that put its tool0 at `tool0`, by IK at the live base (`ArmPlannerContract`)."""
        names = arm_joint_names(arm)
        solved = self.solve_goals(serial, {arm: tool0}, dict(zip(names, reference)) if reference is not None else None)
        future = asyncio.get_running_loop().create_future()

        def done(answer: asyncio.Future) -> None:
            joints, line = answer.result()
            future.set_result((tuple(joints[name] for name in names) if joints is not None else None, line))

        solved.add_done_callback(done)
        return future

    def solve_goals(self, serial: str, targets, reference=None) -> asyncio.Future:
        """Joints that put each arm's tool0 at its pose, the arms together, by IK at the live base.

        `ArmPlannerContract.solve_goals`.
        """
        future = asyncio.get_running_loop().create_future()
        self._ctx.submit(f"solve goal {serial}", lambda: self._start_goal(serial, dict(targets), reference, future))
        return future

    def _start_goal(self, serial: str, targets: dict, reference, future: asyncio.Future) -> None:
        """Snapshot the world and solve the goal on the worker, after any search before it. Main thread."""
        snapshot, _guessed = with_guesses(self._ctx.scene.snapshot, robot_id(serial))
        names = [name for arm in targets for name in arm_joint_names(arm)]

        def work() -> tuple[dict[str, float] | None, str]:
            robot = snapshot.robots[robot_id(serial)]
            problems = robot.acting_problems()
            if problems:
                return None, f"{short_serial(serial)} can't be solved for: {'; '.join(problems)}"
            scene = with_stand_ins(snapshot, {robot_id(serial)}).scene
            mirror = self._world.sync(scene, serial)
            self._loaded.add(serial)
            result = solve_ik(mirror, scene.robots[robot_id(serial)], targets, reference, log=self._ctx.log_info)
            if result.joints is None:
                return None, f"no goal at the live base: {result.reason}"
            joints = {name: float(result.joints[name]) for name in names}
            return joints, (f"goal by IK at the live base ({result.backend}, rank {result.rank} of "
                            f"{result.combinations}, {math.degrees(result.max_change):.1f}° from the arms now)")

        async def run() -> None:
            try:
                answer = await asyncio.get_running_loop().run_in_executor(self._executor, work)
            except BaseException as error:
                if not future.done():
                    future.set_result((None, f"IK failed: {error!r}"))
                raise
            if not future.done():
                future.set_result(answer)

        self._ctx.spawn(f"solve goal {serial} {' + '.join(targets)}", run())

    def _start_check(self, serial: str, arm: str, waypoints: np.ndarray, margin: float,
                     future: asyncio.Future[CollisionHit | None]) -> None:
        """Snapshot the world and check the path on the worker, after any search before it. Main thread."""
        snapshot, _guessed = with_guesses(self._ctx.scene.snapshot, robot_id(serial))

        def work() -> CollisionHit | None:
            problems = snapshot.robots[robot_id(serial)].acting_problems()
            if problems:
                raise ValueError(f"{short_serial(serial)} can't be checked: {'; '.join(problems)}")
            scene = with_stand_ins(snapshot, {robot_id(serial)}).scene
            result = path_hit(self._world.sync(scene, serial), scene, arm, waypoints, margin)
            self._loaded.add(serial)
            return result

        async def run() -> None:
            try:
                result = await asyncio.get_running_loop().run_in_executor(self._executor, work)
            except ValueError as error:
                if not future.done():
                    future.set_exception(error)
                return
            except BaseException as error:
                if not future.done():
                    future.set_exception(RuntimeError(f"the check failed: {error!r}"))
                raise
            if not future.done():
                future.set_result(result)

        self._ctx.spawn(f"check path {serial} {arm}", run())

    def stale_reason(self, ctx: PluginContext, plan: Plan[ArmPlanRequest]) -> str:
        """Stale once the arm or base moves away from the plan's start, or mocap loses a base it tracked then."""
        request = plan.request
        joints = ctx.kinematics.joints(request.serial)
        here = np.array([joints.get(name, 0.0) for arm in request.arms for name in arm_joint_names(arm)])
        if np.max(np.abs(here - plan.path.start)) > STALE_JOINT:
            return "the arm moved since planning"
        # * The base is kept for the latest plan only; `forget` drops it when the plan is replaced or cleared.
        if self._start_base is not None and self._start_base[0] is plan.path:
            _path, start, tracked = self._start_base
            # ! Untracked, the base pose is the last fix: a base moved meanwhile would never look moved.
            base = ctx.world.robots[request.serial].base
            if tracked and (lost := pose_lost(base.mocap_id, base.state, ctx.now())):
                return f"mocap lost the base since planning: {lost}"
            x, y, yaw = floor_pose(ctx.kinematics.base_pose(request.serial))
            if (math.hypot(x - start[0], y - start[1]) > STALE_POSITION
                    or abs(math.remainder(yaw - start[2], math.tau)) > STALE_YAW):
                return "the base moved since planning"
        return ""

    async def execute(self, ctx: PluginContext, plan: Plan[ArmPlanRequest],
                      answer: Callable[[bool, str], None]) -> None:
        """Send the plan to the arm and watch it until it is at the end; else hold it (`send_motion`).

        - Committed with Clearance ticked, an ARM_MOVE clearance starts first (`commit_clearance`): one
          that cannot check, or finds the robot too close, refuses the commit (nothing sent); a trip soft stops the
          robot and cancels this.
        - ! The robot's base and every arm must be measured, Clearance on or off: an arm moves beside the other.
        - ! Arrival is judged on the joints, not `is_executing`, which clears during a slow move (the pendant's speed
          slider): the lease would go back mid-motion.
        """
        serial, arm_name, path = plan.request.serial, plan.request.arm, plan.path
        key, what = (serial, arm_name), f"{serial} {arm_name}"
        # * The last run's chip goes: every way out below sets this run's, or none for a replaced commit.
        self._motions.pop(key, None)
        if problem := clearance_problem(ctx.scene.snapshot.robots.get(robot_id(serial))):
            answer(False, f"nothing sent: {short_serial(serial)}: {problem}")
            ctx.log_warn(f"{what}: nothing sent: {problem}")  # * the run chip's hover may be cut
            self._motions[key] = Check("refused", BAD, hover(f"{what}: nothing sent: {problem}"))
            return

        def sent() -> None:
            answer(True, f"sent to {what}")
            # * Not Hold: it stops the arm but not this commit, which then ends "stopped short".
            self._motions[key] = Check("moving", WARN, f"{what} follows the path; Cancel or Stop all stops it")

        clearance: Clearance | None = None
        try:
            with self.commit_clearance(ctx, plan) as clearance:
                await self.first_check(ctx, clearance)
                ctx.log_info(f"{what}: sending a path of {path.duration:.1f} s, {path.rests} rests on the way")
                arms = [arm for arm in ctx.world.robots[serial].arms if arm in plan.request.arms]
                outcome = await send_motion(ctx, ctx.world.robots[serial], arm_command(serial, arms, path),
                                            on_sent=sent)
        except asyncio.CancelledError:
            if refusal := self.start_refusal(clearance):
                answer(False, refusal)
                ctx.log_warn(f"{what}: {refusal}")
            if self.replaced():
                self._motions.pop(key, None)
            else:
                self._motions[key] = self.cancelled_check(ctx, serial, what, clearance,
                                                          "the commit was cancelled; the arm holds where it was")
            raise
        except MotionRefused as refused:  # e.g. the clearance cannot check the robot: nothing sent
            ctx.log_warn(f"{what}: nothing sent: {refused}")
            self._motions[key] = Check("refused", BAD, hover(f"{what}: nothing sent: {refused}"))
            raise
        except Exception as error:
            self._motions[key] = Check("failed", BAD, hover(f"{what}: {error}"))
            raise
        if outcome.status == REFUSED:
            answer(False, outcome.reason)
        # * Logged, refusals too: the run chip's hover may be cut.
        (ctx.log_info if outcome.ok else ctx.log_warn)(f"{what}: {outcome.reason}")
        self._motions[key] = Check(MOTION_LABELS.get(outcome.status, outcome.status.lower()),
                                   GOOD if outcome.ok else BAD, hover(f"{what}: {outcome.reason}"))

    def commit_refused(self, plan: Plan[ArmPlanRequest], message: str) -> None:
        """A commit refused before `execute` ran (e.g. another plugin holds the lease): its run chip says refused."""
        key = (plan.request.serial, plan.request.arm)
        what = f"{key[0]} {key[1]}"
        self._ctx.log_warn(f"{what}: {message}")  # * the run chip's hover may be cut
        self._motions[key] = Check("refused", BAD, hover(f"{what}: {message}"))

    def forget(self, plan: Plan[ArmPlanRequest]) -> None:
        """Drop the base position the plan was for."""
        self._start_base = None


def floor_pose(pose) -> tuple[float, float, float]:
    """A world pose as x, y in metres and yaw in radians."""
    return float(pose.position[0]), float(pose.position[1]), yaw_from_xyzw(pose.orientation)
