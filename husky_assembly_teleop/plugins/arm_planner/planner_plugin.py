"""
The headless arm planner: plans a collision-free path for one arm to a joint goal and sends it as one trajectory.

Others use it through `ArmPlannerContract`; its panel is `arm_planner_panel`. The first plan for a robot loads its
models in a compas_fab world (several seconds).

! EXPERIMENTAL, not yet tested on a robot: Commit sends the path as one joint trajectory (`send_to_arm`).
"""

from __future__ import annotations

import math
import threading

import numpy as np

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import register
from ...robot_interface.arm import joint_path_move
from ...robot_interface.mocap_state import pose_lost
from ...shared.arm_planner_contract import ArmPlanRequest
from ...shared.planner.planner_search import PlanResult, TIME_LIMIT
from ...shared.planner.plan import Plan
from ...shared.planner.planner_search import Search, WAIT_MARGIN, with_guesses
from ...shared.planner.planner_service import PlannerService
from ...ui.pose_input import yaw_from_xyzw
from scaffolding_assembly_core.ids import robot_id
from .planner import ArmPath, ArmPlanningWorld, arm_joint_names, plan_arm

#: The plan goes stale once a joint moves this far from the planned start, or the base this far; the panel also
#: refuses a target this far from the planned goal.
STALE_JOINT = math.radians(2.0)    # rad
STALE_POSITION = 0.05              # m
STALE_YAW = math.radians(3.0)      # rad; 5 cm at 1 m reach, as STALE_POSITION
#: Extra time for the first plan of a robot, which loads the models, seconds.
LOAD_ALLOWANCE = 60.0


def send_to_arm(ctx: PluginContext, serial: str, arm_name: str, path: ArmPath) -> bool:
    """Send a path to an arm as one joint trajectory that stops at every waypoint (`joint_path_move`).

    Args:
        ctx: The plugin's context, for the world and logging.
        serial: Which robot.
        arm_name: Which arm.
        path: The path to follow; its first waypoint must be where the arm is now (`START_TOLERANCE`), else the arm
            refuses it.

    Returns:
        bool: Whether the arm took the path; if not, the log says why.
    """
    order = [path.joint_names.index(name) for name in arm_joint_names(arm_name)]
    try:
        positions, velocities, duration = joint_path_move(path.points[:, order])
    except ValueError as error:
        ctx.log_warn(f"{serial} {arm_name}: nothing sent: {error}")
        return False
    if not ctx.world.robots[serial].arms[arm_name].send_joint_trajectory(positions, duration, velocities):
        return False
    ctx.log_info(f"{serial} {arm_name}: sent a {len(path.points)}-waypoint path, {duration:.1f} s")
    return True


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
            return (f"{how}: {len(result.path.points)} waypoints, {result.path.duration:.1f} s, "
                    f"found in {result.seconds:.1f} s{guess}"), bool(guessed)

        return Search(label=f"plan {serial} {arm}",
                      message=f"planning for {serial} {arm}…{guess}"
                      + (" (loading the models first)" if loading else ""),
                      work=work, time_limit=TIME_LIMIT + WAIT_MARGIN + (LOAD_ALLOWANCE if loading else 0.0),
                      accepted=accepted, finished=lambda: self._loaded.add(serial))

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

    def send(self, ctx: PluginContext, plan: Plan[ArmPlanRequest]) -> tuple[bool, str]:
        """Send the plan to the arm."""
        serial, arm = plan.request.serial, plan.request.arm
        if send_to_arm(ctx, serial, arm, plan.path):
            return True, f"sent to {serial} {arm}"
        return False, f"{serial} {arm} refused the path (the log says why)"

    def forget(self, plan: Plan[ArmPlanRequest]) -> None:
        """Drop the base position the plan was for."""
        self._start_base = None


def floor_pose(pose) -> tuple[float, float, float]:
    """A world pose as x, y in metres and yaw in radians."""
    return float(pose.position[0]), float(pose.position[1]), yaw_from_xyzw(pose.orientation)
