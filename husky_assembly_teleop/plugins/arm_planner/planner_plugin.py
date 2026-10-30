"""
The headless arm planner: plans a collision-free path for one arm to a joint goal and sends it as one trajectory.

Others use it through `ArmPlannerContract`; its panel is `arm_planner_panel`. The first plan for a robot loads its
models in a compas_fab world (several seconds).

! EXPERIMENTAL, not yet tested on a robot: Commit sends the path as one joint trajectory (`send_motion`), under an
  ARM_STOP_DISTANCE clearance on the robot (`ctx.clearance`) while `clearance` is on, so the base pose must be known
  to send.
"""

from __future__ import annotations

import asyncio
import math
import threading
from contextlib import nullcontext
from dataclasses import replace
from typing import Callable, Sequence

import numpy as np

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...robot_interface.mocap_state import pose_lost
from ...shared.arm_planner_contract import MAX_SLOWDOWN, ArmPlanRequest
from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.assembly_planning.mirrors import MirrorPool
from ...shared.execution.send import REFUSED, ArmMotion, MotionCommand, send_motion
from ...world.collision_hit import CollisionHit
from ...shared.planner.planner_search import PlanResult, TIME_LIMIT
from ...shared.planner.plan import Plan
from ...shared.planner.planner_search import Search, WAIT_MARGIN, unmeasured, with_guesses
from ...shared.planner.planner_service import PlannerService
from ...ui.pose_input import yaw_from_xyzw
from ...world.clearance import ARM_STOP_DISTANCE, clearance_problem
from ...world.stand_ins import with_stand_ins
from scaffolding_assembly_core.ids import robot_id
from .planner import ArmPath, path_hit, plan_arm

#: The plan goes stale once a joint moves this far from the planned start, or the base this far; the panel also
#: refuses a target this far from the planned goal.
STALE_JOINT = math.radians(2.0)    # rad
STALE_POSITION = 0.05              # m
STALE_YAW = math.radians(3.0)      # rad; 5 cm at 1 m reach, as STALE_POSITION
#: Extra time for the first plan of a robot, which loads the models, seconds.
LOAD_ALLOWANCE = 60.0


def arm_command(serial: str, arm_name: str, path: ArmPath) -> MotionCommand:
    """A path as a command for one arm, in the driver's joint order, timed as planned and previewed.

    Args:
        serial: Which robot.
        arm_name: Which arm.
        path: The path to follow; its first point must be where the arm is now (`START_TOLERANCE`), else the arm
            refuses it.
    """
    order = [path.joint_names.index(name) for name in arm_joint_names(arm_name)]
    points = path.points[:, order]
    travel = float(np.max(np.sum(np.abs(np.diff(points, axis=0)), axis=0)))
    return MotionCommand(serial, (ArmMotion(arm_name, points, path.velocities[:, order]),), path.duration,
                         path.fastest, path.duration, travel)


@register
class ArmPlannerPlugin(PlannerService[ArmPlanRequest]):
    """Plans a collision-free joint path for one arm and sends it to the arm."""

    name = "arm_planner"
    experimental = True
    worker_name = "arm-plan"

    def __init__(self):
        """Start with no plan; the models load with the first plan of each robot."""
        super().__init__()
        #: Whether commits run under a clearance (`ArmPlannerContract.clearance`); the panel's Clearance box sets it.
        self.clearance = True
        #: Robots whose models the worker has loaded, so later plans need no load allowance.
        self._loaded: set[str] = set()
        #: The latest plan's path, the base's floor pose (x, y, yaw) it was planned from, and whether mocap tracked
        #: the base then; `stale_reason` compares the base.
        self._start_base: tuple[ArmPath, tuple[float, float, float], bool] | None = None
        # ! Worker thread only (sync, search, close).
        self._world = MirrorPool()

    def setup(self, ctx: PluginContext) -> None:
        """Let the mirrors log through the plugin."""
        super().setup(ctx)
        self._world.log = ctx.log_info  # ? the ROS logger is safe to call from the worker

    def make_search(self, ctx: PluginContext, request: ArmPlanRequest, abort: threading.Event) -> Search:
        """Search from where the arm is in the tick's snapshot to the request's goal."""
        serial, arm, goal = request.serial, request.arm, np.array(request.goal)
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
                return PlanResult(None, f"{serial} can't be planned for: {'; '.join(problems)}", 0.0)
            stand_ins = with_stand_ins(snapshot, {robot_id(serial)})
            others[:] = stand_ins.warnings
            result = plan_arm(self._world.sync(stand_ins.scene, serial), stand_ins.scene, arm, goal, abort)
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
            return (f"{how}: {result.path.duration:.1f} s, {result.path.rests} rests on the way, "
                    f"found in {result.seconds:.1f} s{notes}{stand_ins}"), bool(guessed or others)

        return Search(label=f"plan {serial} {arm}",
                      message=f"planning for {serial} {arm}…{notes}"
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
                   waypoints: Sequence[Sequence[float]]) -> asyncio.Future[CollisionHit | None]:
        """Check a given joint path for one arm for collisions, in the world as it is now (`ArmPlannerContract`)."""
        future = asyncio.get_running_loop().create_future()
        path = np.array(waypoints, dtype=float)
        self._ctx.submit(f"check path {serial}", lambda: self._start_check(serial, arm, path, future))
        return future

    def _start_check(self, serial: str, arm: str, waypoints: np.ndarray,
                     future: asyncio.Future[CollisionHit | None]) -> None:
        """Snapshot the world and check the path on the worker, after any search before it. Main thread."""
        snapshot, _guessed = with_guesses(self._ctx.scene.snapshot, robot_id(serial))

        def work() -> CollisionHit | None:
            problems = snapshot.robots[robot_id(serial)].acting_problems()
            if problems:
                raise ValueError(f"{serial} can't be checked: {'; '.join(problems)}")
            scene = with_stand_ins(snapshot, {robot_id(serial)}).scene
            result = path_hit(self._world.sync(scene, serial), scene, arm, waypoints)
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
        here = np.array([joints.get(name, 0.0) for name in arm_joint_names(request.arm)])
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

        - While `clearance` is on, an ARM_STOP_DISTANCE clearance starts first: one that cannot check refuses the
          commit (nothing sent), a trip soft stops the robot and cancels this. The other robots unknown at the commit
          may be unknown (`unknown_allowed`).
        - ! The robot's base and every arm must be measured, Clearance on or off: an arm moves beside the other.
        - ! Arrival is judged on the joints, not `is_executing`, which clears during a slow move (the pendant's speed
          slider): the lease would go back mid-motion.
        """
        serial, arm_name, path = plan.request.serial, plan.request.arm, plan.path
        if problem := clearance_problem(ctx.scene.snapshot.robots.get(robot_id(serial))):
            answer(False, f"nothing sent: {serial}: {problem}")
            return
        guard = (ctx.clearance([serial], ARM_STOP_DISTANCE, unknown_robots=self.unknown_allowed(plan))
                 if self.clearance else nullcontext())
        with guard:
            ctx.log_info(f"{serial} {arm_name}: sending a path of {path.duration:.1f} s, {path.rests} rests on the way")
            outcome = await send_motion(ctx, ctx.world.robots[serial], arm_command(serial, arm_name, path),
                                        on_sent=lambda: answer(True, f"sent to {serial} {arm_name}"))
        if outcome.status == REFUSED:
            answer(False, outcome.reason)
        else:
            (ctx.log_info if outcome.ok else ctx.log_warn)(f"{serial} {arm_name}: {outcome.reason}")

    def forget(self, plan: Plan[ArmPlanRequest]) -> None:
        """Drop the base position the plan was for."""
        self._start_base = None


def floor_pose(pose) -> tuple[float, float, float]:
    """A world pose as x, y in metres and yaw in radians."""
    return float(pose.position[0]), float(pose.position[1]), yaw_from_xyzw(pose.orientation)
