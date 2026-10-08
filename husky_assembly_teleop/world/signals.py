"""
Ready-made signals of the measured world, for `ctx.trace` and `ctx.record`.

Each is unknown (NaN) until its source has a value. Rename one with `dataclasses.replace(signal, name=...)`.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from ..plugin_api.trace import Signal
from ..robot_interface.arm import UR_JOINT_NAMES
from ..robot_interface.base import FOLLOWER_STATES

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


def base_floor_pose(robot: HuskyRobotInterface) -> Signal:
    """The robot's last valid mocap pose on the floor: x, y in metres and yaw in radians."""
    def read():
        state = robot.base.state
        if state.position is None:
            return None
        qx, qy, qz, qw = state.orientation
        return (*state.position[:2], math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))

    return Signal(f"{robot.config.serial}_floor_pose", read, ("x", "y", "yaw"), "m, rad")


def follower_position_error(robot: HuskyRobotInterface) -> Signal:
    """The onboard path follower's cross-track and along-track (timed paths) errors, metres."""
    return Signal(f"{robot.config.serial}_follower_position_error",
                  lambda: _follower(robot, lambda f: (f.cross_track_error, f.along_track_error)),
                  ("cross", "along"), "m")


def follower_yaw_error(robot: HuskyRobotInterface) -> Signal:
    """The onboard path follower's yaw error, reference minus measured, radians."""
    return Signal(f"{robot.config.serial}_follower_yaw_error", lambda: _follower(robot, lambda f: f.yaw_error),
                  unit="rad")


def follower_command(robot: HuskyRobotInterface) -> Signal:
    """The velocity the onboard path follower sent: linear in m/s, angular in rad/s."""
    return Signal(f"{robot.config.serial}_follower_command", lambda: _follower(robot, lambda f: f.command),
                  ("v", "w"), "m/s, rad/s")


def follower_progress(robot: HuskyRobotInterface) -> Signal:
    """The onboard path follower's progress (0 to 1), current piece, and state code (0 idle to 3 aborted)."""
    return Signal(f"{robot.config.serial}_follower_progress",
                  lambda: _follower(robot, lambda f: (f.progress, f.piece, FOLLOWER_STATES.index(f.state)
                                                      if f.state in FOLLOWER_STATES else -1)),
                  ("progress", "piece", "state"))


def object_position(obj: TrackedObject) -> Signal:
    """A tracked object's last valid mocap position in the world, metres."""
    return Signal(obj.name, lambda: obj.position, XYZ, "m")


def _part(wrench: np.ndarray | None, start: int) -> np.ndarray | None:
    """Three values of a wrench from `start`, or None while there is none."""
    return None if wrench is None else wrench[start:start + 3]


def _follower(robot: HuskyRobotInterface, pick):
    """`pick(report)` of the robot's path follower report, or None before the first."""
    follower = robot.base.state.follower
    return None if follower is None else pick(follower)
