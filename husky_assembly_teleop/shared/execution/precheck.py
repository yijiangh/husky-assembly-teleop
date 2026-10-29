"""
The checks every compliant part (insert, retreat, handoff) runs before it sends anything: one function, one wording.

! Main thread, from a plugin task.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Collection, Mapping

from ...robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER
from ...robot_interface.mocap_state import pose_lost
from .stream import FK_TCP_TOL, fk_tcp_gap
from .tools import status_problem

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.end_effectors import ScaffoldingV3
    from ...robot_interface.robot import HuskyRobotInterface


def part_problem(ctx: PluginContext, robot: HuskyRobotInterface, arms: Collection[str],
                 tools: Mapping[str, ScaffoldingV3], *, frames: bool = True) -> str:
    """Why no compliant part may start on `arms` now; "" when one may.

    Every arm runs the joint controller with no switch pending, keeps the stock URDF frames, is not in Cartesian test
    mode and has joints and a TCP; with `frames`, its measured TCP is within FK_TCP_TOL of tool0 by the URDF. The
    base pose is live, and every tool reported lately (`status_problem`).

    Args:
        ctx: The plugin's context, for its clock.
        robot: The robot.
        arms: The names of the arms the part moves.
        tools: Arm name -> the scaffolding tools the part drives; empty for none.
        frames: Refuse an arm whose measured TCP is over FK_TCP_TOL from tool0; the handoff checks its anchor later
            instead (it closes stiff then).

    Returns:
        str: The refusal, or "".
    """
    joint = SCALED_JOINT_TRAJECTORY_CONTROLLER
    for name in arms:
        arm = robot.arms[name]
        pending, running = arm.state.controllers.switch_in_flight, arm.state.controllers.active
        if pending is not None:
            return f"{name} is still switching to {pending}"
        if not arm.controllers.is_active(joint):
            return f"{name} is not under {joint} (it runs {running}); switch it in robot_control"
        if arm.urdf_problem is not None:
            return f"{name}: the URDF breaks the stock UR frames ({arm.urdf_problem})"
        if arm.config.cartesian_test_mode:
            return f"{name} is in Cartesian test mode: its targets would not be sent"
        gap = fk_tcp_gap(robot, name)
        if gap is None:
            return f"{name}: no joint state or TCP yet"
        if frames and (gap[0] > FK_TCP_TOL[0] or gap[1] > FK_TCP_TOL[1]):
            return (f"{name}: the measured TCP is {gap[0] * 1e3:.1f} mm, {math.degrees(gap[1]):.1f}° from tool0 by "
                    f"the URDF (limit {FK_TCP_TOL[0] * 1e3:.0f} mm, {math.degrees(FK_TCP_TOL[1]):.0f}°): a frame is "
                    f"wrong")
    lost = pose_lost(robot.base.mocap_id, robot.base.state, ctx.now())
    if lost:
        return f"base pose lost: {lost}"
    if robot.husky_in_world() is None:
        return "no base pose from mocap"
    return status_problem(ctx, tools)
