"""
Ready-made signals of the measured world, for `ctx.trace` and `ctx.record`.

Each is unknown (NaN) until its source has a value. Rename one with `dataclasses.replace(signal, name=...)`.
The base's mocap pose, follower and wheel odometry come per tick (the newest message) or, with `every_message`, per
message; a message signal is named after its value alone, e.g. "command", since its event's name comes first in its
key. What the monitor sends (`*_sent`) comes on every send: the commanded next to the measured.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Callable

import numpy as np
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

from ..robot_interface.base import FOLLOWER_STATES
from .trace import Signal

if TYPE_CHECKING:
    from ..robot_interface.arm import ArmInterface
    from ..robot_interface.event import Event
    from ..robot_interface.robot import HuskyRobotInterface
    from ..world.kinematics import Kinematics
    from ..world.measured import TrackedObject

XYZ = ("x", "y", "z")
QUATERNION = ("qx", "qy", "qz", "qw")
_JOINT_LABELS = tuple(name.removesuffix("_joint") for name in UR_JOINT_NAMES)


@dataclass(frozen=True)
class _Field:
    """One value of an interface's record, written once: `latest` makes it a tick signal, `on` a message signal."""

    name: str
    pick: Callable[[Any], Any]
    labels: tuple[str, ...] = ()
    unit: str | tuple[str, ...] = ""

    def latest(self, record: Callable[[], Any], name: str) -> Signal:
        """A tick signal called `name` of the newest record, `record()`; unknown while that is None."""
        def read():
            current = record()
            return None if current is None else self.pick(current)

        return Signal(name, read, self.labels, self.unit)

    def on(self, event: Event) -> Signal:
        """A message signal: the value of every record `event` fires."""
        return Signal(self.name, self.pick, self.labels, self.unit, event)


#: The base's pose on the floor, from its state (last valid pose) or a `MocapSample` (NaN when invalid).
_FLOOR_POSE = _Field("floor_pose", lambda body: _floor_pose(body.position, body.orientation), ("x", "y", "yaw"),
                     ("m", "m", "rad"))
#: Values of the follower's reports (`FollowerState`) and of the wheel odometry (`WheelOdometry`).
_FOLLOWER_POSITION_ERROR = _Field("position_error", lambda f: (f.cross_track_error, f.along_track_error),
                                  ("cross", "along"), "m")
_FOLLOWER_YAW_ERROR = _Field("yaw_error", lambda f: f.yaw_error, unit="rad")
_FOLLOWER_COMMAND = _Field("command", lambda f: f.command, ("v", "w"), ("m/s", "rad/s"))
_FOLLOWER_PROGRESS = _Field("progress", lambda f: (f.progress, f.piece, _state_code(f)), ("progress", "piece", "state"))
_FOLLOWER_STATE = _Field("state", lambda f: _state_code(f))
_WHEEL_VELOCITY = _Field("velocity", lambda o: o.velocity, ("v", "w"), ("m/s", "rad/s"))


def joints(arm: ArmInterface) -> Signal:
    """The arm's six joint angles, radians, UR driver order."""
    return Signal(f"{arm.config.name}_joints", arm.joint_vector, _JOINT_LABELS, "rad")


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

    return Signal(f"{serial}_{link}_pose", read, (*XYZ, *QUATERNION), ("m",) * 3 + ("",) * 4)


def base_position(robot: HuskyRobotInterface) -> Signal:
    """The robot's last valid mocap position in the world, metres."""
    return Signal(f"{robot.config.serial}_base", lambda: robot.base.state.position, XYZ, "m")


def base_floor_pose(robot: HuskyRobotInterface, every_message: bool = False) -> Signal:
    """The robot's mocap pose on the floor: x, y in metres and yaw in radians.

    Per tick the last valid pose; with `every_message` every relay sample, NaN while invalid, at its capture `stamp`.
    """
    base = robot.base
    if every_message:
        return _FLOOR_POSE.on(base.mocap_messages)
    return _FLOOR_POSE.latest(lambda: base.state, f"{robot.config.serial}_floor_pose")


def follower_position_error(robot: HuskyRobotInterface, every_message: bool = False) -> Signal:
    """The onboard path follower's cross-track and along-track (timed paths) errors, metres."""
    return _follower(robot, _FOLLOWER_POSITION_ERROR, every_message)


def follower_yaw_error(robot: HuskyRobotInterface, every_message: bool = False) -> Signal:
    """The onboard path follower's yaw error, reference minus measured, radians."""
    return _follower(robot, _FOLLOWER_YAW_ERROR, every_message)


def follower_command(robot: HuskyRobotInterface, every_message: bool = False) -> Signal:
    """The velocity the onboard path follower sent: linear in m/s, angular in rad/s."""
    return _follower(robot, _FOLLOWER_COMMAND, every_message)


def follower_progress(robot: HuskyRobotInterface, every_message: bool = False) -> Signal:
    """The onboard path follower's progress (0 to 1), current piece, and state code (0 idle to 3 aborted)."""
    return _follower(robot, _FOLLOWER_PROGRESS, every_message)


def follower_state(robot: HuskyRobotInterface, every_message: bool = False) -> Signal:
    """The onboard path follower's state code: 0 idle, 1 following, 2 done, 3 aborted (`FOLLOWER_STATES`)."""
    return _follower(robot, _FOLLOWER_STATE, every_message)


def wheel_odometry(robot: HuskyRobotInterface, every_message: bool = False) -> Signal:
    """The platform's velocity from its wheels, in the base frame: linear in m/s, angular in rad/s. None in sim."""
    base = robot.base
    if every_message:
        return _WHEEL_VELOCITY.on(base.odometry_messages)
    return _WHEEL_VELOCITY.latest(lambda: base.state.odometry, f"{robot.config.serial}_wheel_odometry")


def twist_sent(robot: HuskyRobotInterface) -> Signal:
    """Every velocity command the monitor sent the base, a stop's zero included: linear in m/s, angular in rad/s."""
    return Signal("command", lambda twist: twist.command, ("v", "w"), ("m/s", "rad/s"), robot.base.twists_sent)


def path_sent(robot: HuskyRobotInterface) -> Signal:
    """Every path sent to the onboard follower: its id, waypoints (0: a stop), timed (1) or not, and speed caps."""
    return Signal("path", lambda path: (path.path_id, path.waypoints, path.timed, *path.speed_caps),
                  ("id", "waypoints", "timed", "linear_cap", "angular_cap"), ("", "", "", "m/s", "rad/s"),
                  robot.base.paths_sent)


def trajectory_sent(arm: ArmInterface) -> Signal:
    """Every joint trajectory sent, holds included: the joints it ends at (rad), its duration (s), hold (1) or not."""
    return Signal("end", lambda trajectory: (*trajectory.target, trajectory.duration, trajectory.hold),
                  (*_JOINT_LABELS, "duration", "hold"), ("rad",) * len(_JOINT_LABELS) + ("s", ""),
                  arm.trajectories_sent)


def target_sent(arm: ArmInterface) -> Signal:
    """Every TCP target sent to the compliance controller, in the arm's base_link: position (m), quaternion, hold."""
    return Signal("pose", lambda target: (*target.position, *target.orientation, target.hold),
                  (*XYZ, *QUATERNION, "hold"), ("m",) * 3 + ("",) * 5, arm.targets_sent)


def wrench_sent(arm: ArmInterface) -> Signal:
    """Every target wrench sent, in tool0, a hold's zero included: force (N) and torque (N m)."""
    return Signal("wrench", lambda wrench: (*wrench.force, *wrench.torque), ("fx", "fy", "fz", "tx", "ty", "tz"),
                  ("N",) * 3 + ("N m",) * 3, arm.wrenches_sent)


def object_position(obj: TrackedObject) -> Signal:
    """A tracked object's last valid mocap position in the world, metres."""
    return Signal(obj.name, lambda: obj.position, XYZ, "m")


def _part(wrench: np.ndarray | None, start: int) -> np.ndarray | None:
    """Three values of a wrench from `start`, or None while there is none."""
    return None if wrench is None else wrench[start:start + 3]


def _follower(robot: HuskyRobotInterface, value: _Field, every_message: bool) -> Signal:
    """A value of the follower's reports: of every report, or of the newest each tick as "<serial>_follower_<name>"."""
    base = robot.base
    if every_message:
        return value.on(base.follower_reports)
    return value.latest(lambda: base.state.follower, f"{robot.config.serial}_follower_{value.name}")


def _floor_pose(position: np.ndarray | None, orientation: np.ndarray | None) -> tuple | None:
    """x, y and yaw of a pose, or None without a position."""
    if position is None:
        return None
    qx, qy, qz, qw = orientation
    return (*position[:2], math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)))


def _state_code(follower) -> int:
    """A report's state as its code (the index in FOLLOWER_STATES), -1 if unknown."""
    return FOLLOWER_STATES.index(follower.state) if follower.state in FOLLOWER_STATES else -1
