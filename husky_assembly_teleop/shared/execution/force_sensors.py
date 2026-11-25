"""
The arms' force readings: |F| of a wrench, and zeroing the force sensors before a compliant part.

! `zero_force_sensors`: main thread, from a plugin task.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Collection, Sequence

import numpy as np

from ...config import short_serial
from ...robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER
from .moving import robot_moving

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.arm import ArmInterface
    from ...robot_interface.robot import HuskyRobotInterface

#: Seconds the force sensors get after their zero, before the arms go compliant.
FT_SETTLE = 0.5


def force_magnitude(wrench: Sequence[float] | None) -> float | None:
    """|F| of a wrench (force first), newtons; None without one."""
    return None if wrench is None else float(np.linalg.norm(np.asarray(wrench, dtype=float)[:3]))


def newtons(force: float | None) -> str:
    """A force for the log."""
    return "no reading" if force is None else f"{force:.1f} N"


def zero_refusal(serial: str, robot: HuskyRobotInterface, now: float) -> str:
    """Why a robot's force sensors may not be zeroed now, or "".

    Refused without arms, with an arm or the base moving, or with an arm not under the joint controller: a zero in
    motion is wrong, and a compliant arm moves when its reading jumps.
    """
    if not robot.arms:
        return f"{short_serial(serial)} has no connected arms"
    moving = robot_moving(serial, robot, now)
    if moving:
        return moving
    compliant = [name for name, arm in robot.arms.items()
                 if not arm.controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER)]
    return f"{', '.join(compliant)} not under the joint controller" if compliant else ""


async def zero_force_sensors(ctx: PluginContext, arms: Collection[ArmInterface], label: str) -> dict[str, dict]:
    """Zero every arm's force sensor and give it FT_SETTLE; logs |F| before and after, warns for a zero not sent.

    Returns:
        dict[str, dict]: Arm name -> {"sent": the zero went out, "force_before_N", "force_after_N", "wrench_before"
        (the reading the zero tares away, force then torque in tool0, or None)}.
    """
    before = {arm.config.name: force_magnitude(arm.state.wrench) for arm in arms}
    wrenches = {arm.config.name: None if arm.state.wrench is None else np.round(arm.state.wrench, 3).tolist()
                for arm in arms}
    sent = {arm.config.name: arm.zero_ft_sensor() for arm in arms}
    for name, zeroed in sent.items():
        if not zeroed:
            ctx.log_warn(f"{label}: {name}'s force sensor NOT zeroed (service not available); the compliance reacts "
                         f"to its reading as it is")
    await ctx.sleep(FT_SETTLE)
    after = {arm.config.name: force_magnitude(arm.state.wrench) for arm in arms}
    ctx.log_info(f"{label}: force sensors zeroed: |F| " + ", ".join(
        f"{name} {newtons(before[name])} -> {newtons(after[name])}" for name in before))
    return {name: {"sent": sent[name], "force_before_N": before[name], "force_after_N": after[name],
                   "wrench_before": wrenches[name]} for name in before}
