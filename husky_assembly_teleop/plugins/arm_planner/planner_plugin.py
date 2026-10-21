"""
The headless arm planner: plans a collision-free path for one arm to a joint goal and sends it as one trajectory.

Others use it through `ArmPlannerContract`; its panel is `arm_planner_panel`. The first plan for a robot loads its
models in a compas_fab world (several seconds).

! EXPERIMENTAL, not yet tested on a robot: Commit sends the path as one joint trajectory (`send_to_arm`).
"""

from __future__ import annotations

import asyncio
import math
import threading
from typing import Callable, Sequence

import numpy as np

from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...robot_interface.mocap_state import pose_lost
from ...shared.arm_planner_contract import ArmPlanRequest
from ...world.collision_hit import CollisionHit
from ...shared.planner.planner_search import PlanResult, TIME_LIMIT
from ...shared.planner.plan import Plan
from ...shared.planner.planner_search import Search, WAIT_MARGIN, with_guesses
from ...shared.planner.planner_service import PlannerService
from ...ui.pose_input import yaw_from_xyzw
from scaffolding_assembly_core.ids import robot_id
from .planner import ArmPath, ArmPlanningWorld, arm_joint_names, path_hit, plan_arm

#: The plan goes stale once a joint moves this far from the planned start, or the base this far; the panel also
#: refuses a target this far from the planned goal.
STALE_JOINT = math.radians(2.0)    # rad
STALE_POSITION = 0.05              # m
STALE_YAW = math.radians(3.0)      # rad; 5 cm at 1 m reach, as STALE_POSITION
#: Extra time for the first plan of a robot, which loads the models, seconds.
LOAD_ALLOWANCE = 60.0
#: Seconds a sent path may take beyond its duration before the arm is held.
MOTION_MARGIN = 5.0


def send_to_arm(ctx: PluginContext, serial: str, arm_name: str, path: ArmPath) -> float | None:
    """Send a path to an arm as one joint trajectory, timed as planned and previewed.

    Args:
        ctx: The plugin's context, for the world and logging.
        serial: Which robot.
        arm_name: Which arm.
        path: The path to follow; its first point must be where the arm is now (`START_TOLERANCE`), else the arm
            refuses it.

    Returns:
        float | None: The path's duration in seconds, or None if the arm did not take it (the log says why).
    """
    order = [path.joint_names.index(name) for name in arm_joint_names(arm_name)]
    if not ctx.world.robots[serial].arms[arm_name].send_joint_trajectory(path.points[:, order], path.duration,
                                                                         path.velocities[:, order]):
        return None
    ctx.log_info(f"{serial} {arm_name}: sent a path of {path.duration:.1f} s, {path.rests} rests on the way")
    return path.duration


@register
class ArmPlannerPlugin(PlannerService[ArmPlanRequest]):
    """Plans a collision-free joint path for one arm and sends it to the arm."""

    name = "arm_planner"
    experimental = True
    worker_name = "arm-plan"

    def __init__(self):
        """Start with no plan; the models load with the first plan of each robot."""
        super().__init__()
        #: Robots whose models the worker has loaded, so later plans need no load allowance.
        self._loaded: set[str] = set()
        #: The latest plan's path, the base's floor pose (x, y, yaw) it was planned from, and whether mocap tracked
        #: the base then; `stale_reason` compares the base.
        self._start_base: tuple[ArmPath, tuple[float, float, float], bool] | None = None
        # ! Worker thread only (sync, search, close).
        self._world = ArmPlanningWorld()

    def setup(self, ctx: PluginContext) -> None:
        """Let the planning world log through the plugin."""
        super().setup(ctx)
        self._world.log = ctx.log_info  # ? the ROS logger is safe to call from the worker

    def make_search(self, ctx: PluginContext, request: ArmPlanRequest, abort: threading.Event) -> Search:
        """Search from where the arm is in the tick's snapshot to the request's goal."""
        serial, arm, goal = request.serial, request.arm, np.array(request.goal)
        # * Take the snapshot here, on the main thread; the arm starts where it is in it.
        live = ctx.scene.snapshot
        snapshot, guessed = with_guesses(live, robot_id(serial))
        guess = f" (guessed: {'; '.join(guessed)})" if guessed else ""
        if guessed:
            ctx.log_warn(f"planning for {serial} {arm} from a guess: {'; '.join(guessed)}")
        tracked = live.robots[robot_id(serial)].base_tracked
        base = floor_pose(snapshot.robots[robot_id(serial)].base)
        loading = serial not in self._loaded

        def work() -> PlanResult:
            # * Only an absent robot is left with problems; an untracked base or unknown joints were guessed.
            problems = snapshot.robots[robot_id(serial)].acting_problems()
            if problems:
                return PlanResult(None, f"{serial} can't be planned for: {'; '.join(problems)}", 0.0)
            return plan_arm(self._world, self._world.sync(snapshot, serial), arm, goal, abort)

        def accepted(result: PlanResult) -> tuple[str, bool]:
            self._start_base = (result.path, base, tracked)
            how = "straight" if result.direct else "RRT"
            return (f"{how}: {result.path.duration:.1f} s, {result.path.rests} rests on the way, "
                    f"found in {result.seconds:.1f} s{guess}"), bool(guessed)

        return Search(label=f"plan {serial} {arm}",
                      message=f"planning for {serial} {arm}…{guess}"
                      + (" (loading the models first)" if loading else ""),
                      work=work, time_limit=TIME_LIMIT + WAIT_MARGIN + (LOAD_ALLOWANCE if loading else 0.0),
                      accepted=accepted, finished=lambda: self._loaded.add(serial))

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
            result = path_hit(self._world, self._world.sync(snapshot, serial), arm, waypoints)
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
        """Send the plan to the arm, and wait until it stands still; Hold it if cancelled or late."""
        serial, arm_name = plan.request.serial, plan.request.arm
        arm = ctx.world.robots[serial].arms[arm_name]
        duration = send_to_arm(ctx, serial, arm_name, plan.path)
        if duration is None:
            answer(False, f"{serial} {arm_name} refused the path (the log says why)")
            return
        answer(True, f"sent to {serial} {arm_name}")
        try:
            await ctx.wait_until(lambda: not arm.state.is_executing, timeout_s=duration + MOTION_MARGIN,
                                 description=f"{serial} {arm_name} to finish the path")
        except WaitTimeout as late:
            ctx.log_warn(f"{late}; holding the arm")
            arm.hold()
        except asyncio.CancelledError:
            arm.hold()  # ! the lease is given back next: nothing would stop the arm otherwise
            raise

    def forget(self, plan: Plan[ArmPlanRequest]) -> None:
        """Drop the base position the plan was for."""
        self._start_base = None


def floor_pose(pose) -> tuple[float, float, float]:
    """A world pose as x, y in metres and yaw in radians."""
    return float(pose.position[0]), float(pose.position[1]), yaw_from_xyzw(pose.orientation)
