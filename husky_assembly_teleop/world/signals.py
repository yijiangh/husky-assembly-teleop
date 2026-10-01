"""
Ready-made signals of the measured world, for `ctx.trace` and `ctx.record`.

Each is unknown (NaN) until its source has a value. Rename one with `dataclasses.replace(signal, name=...)`.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from ..plugin_api.trace import Signal
from ..robot_interface.arm import UR_JOINT_NAMES

if TYPE_CHECKING:
    from ..robot_interface.arm import ArmInterface
    from ..robot_interface.robot import HuskyRobotInterface
    from .kinematics import Kinematics
    from .measured import TrackedObject

XYZ = ("x", "y", "z")
QUATERNION = ("qx", "qy", "qz", "qw")


def joints(arm: ArmInterface) -> Signal:
    """The arm's six joint angles, radians, UR driver order."""
    return Signal(f"{arm.config.name}_joints", arm.joint_vector,
                  tuple(name.removesuffix("_joint") for name in UR_JOINT_NAMES), "rad")


def tcp_position(arm: ArmInterface) -> Signal:
    """The measured TCP position in the arm's `base_link`, the frame commands use, metres."""
    return Signal(f"{arm.config.name}_tcp", lambda: arm.state.tcp_position, XYZ, "m")


def tcp_error(arm: ArmInterface) -> Signal:
    """Last Cartesian target sent minus the measured TCP, metres, and its length; unknown before a target."""
    def read():
        state = arm.state
        if state.tcp_position is None or state.sent_target_position is None:
            return None
        error = state.sent_target_position - state.tcp_position
        return (*error, np.linalg.norm(error))

    return Signal(f"{arm.config.name}_tcp_error", read, (*XYZ, "|d|"), "m")


def force(arm: ArmInterface) -> Signal:
    """The force/torque sensor's force, newtons, raw: not zeroed, so it includes the tool's weight."""
    return Signal(f"{arm.config.name}_force", lambda: _part(arm.state.wrench, 0), XYZ, "N")


def torque(arm: ArmInterface) -> Signal:
    """The force/torque sensor's torque, newton-metres, raw like `force`."""
    return Signal(f"{arm.config.name}_torque", lambda: _part(arm.state.wrench, 3), XYZ, "Nm")


def link_position(kinematics: Kinematics, serial: str, link: str) -> Signal:
    """A URDF link's world position from this tick's forward kinematics, metres, e.g. link "arm_0_tool0"."""
    return Signal(f"{serial}_{link}", lambda: kinematics.link_pose(serial, link).position, XYZ, "m")


def link_pose(kinematics: Kinematics, serial: str, link: str) -> Signal:
    """A link's world pose as position (m) then quaternion (x, y, z, w), for saving; plot `link_position`."""
    def read():
        pose = kinematics.link_pose(serial, link)
        return (*pose.position, *pose.orientation)

    return Signal(f"{serial}_{link}_pose", read, (*XYZ, *QUATERNION), "m, quat")


def base_position(robot: HuskyRobotInterface) -> Signal:
    """The robot's last valid mocap position in the world, metres."""
    return Signal(f"{robot.config.serial}_base", lambda: robot.base.state.position, XYZ, "m")


def object_position(obj: TrackedObject) -> Signal:
    """A tracked object's last valid mocap position in the world, metres."""
    return Signal(obj.name, lambda: obj.position, XYZ, "m")


def _part(wrench: np.ndarray | None, start: int) -> np.ndarray | None:
    """Three values of a wrench from `start`, or None while there is none."""
    return None if wrench is None else wrench[start:start + 3]
