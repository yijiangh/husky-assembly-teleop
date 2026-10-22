"""
Judging the TCP an arm reports against tool0 by the robot's kinematics.

Off means the pendant's active TCP is not the one the monitor expects (`ArmConfig.pendant_tcp`, tool0), or the URDF or
its calibration is off.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.robot import FLANGE_SUFFIX

from ..robot_interface.arm import STILL_AFTER, ArmInterface
from .checks import GOOD, STALE_AFTER, WARN, Check

if TYPE_CHECKING:
    from .kinematics import Kinematics

#: Largest offset, metres, and turn, radians, between reported and computed tool0 that counts as agreeing.
#: ? Guesses: a calibrated URDF should agree with the UR's own kinematics to about a millimetre.
TCP_POSITION_TOLERANCE = 2e-3
TCP_ANGLE_TOLERANCE = math.radians(0.5)


def tool0_by_kinematics(kinematics: Kinematics, serial: str, arm: str) -> Pose:
    """An arm's tool0 in its base_link, by forward kinematics of this tick's joints: what `tcp_check` expects."""
    return compose(invert(kinematics.link_pose(serial, f"{arm}_base_link")),
                   kinematics.link_pose(serial, f"{arm}{FLANGE_SUFFIX}"))


def tcp_check(arm: ArmInterface, expected: Pose, now: float) -> Check | None:
    """Whether the TCP the arm reports, less the pendant's TCP, is tool0 where the URDF's kinematics put it.

    Off means the pendant's active TCP is not `ArmConfig.pendant_tcp` (tool0), or the URDF or its calibration is off.
    ! Judged only while the arm has been still for STILL_AFTER: joints and TCP arrive in separate messages.

    Args:
        arm: The arm.
        expected: tool0 in the arm's base_link, by forward kinematics of this tick's joints.
        now: Current ROS time, seconds.

    Returns:
        Check | None: The verdict, or None while the arm moves (keep the last one).
    """
    state = arm.state
    if state.tcp_position is None:
        return Check("tcp", WARN, "no tcp_pose yet")
    if now - state.tcp_update_time > STALE_AFTER:
        return Check("tcp", WARN, f"no tcp_pose for over {STALE_AFTER:g}s")
    if state.last_motion_time is not None and now - state.last_motion_time < STILL_AFTER:
        return None
    distance = float(np.linalg.norm(np.subtract(state.tcp_position, expected.position)))
    angle = float((Rotation.from_quat(state.tcp_orientation).inv() * Rotation.from_quat(expected.orientation))
                  .magnitude())
    limits = f"{TCP_POSITION_TOLERANCE * 1e3:g} mm / {np.degrees(TCP_ANGLE_TOLERANCE):g}°"
    if distance > TCP_POSITION_TOLERANCE or angle > TCP_ANGLE_TOLERANCE:
        # ? Whole millimetres and degrees: the tooltip text changes only when the offset does.
        return Check("TCP", WARN, f"reported tool0 {distance * 1e3:.0f} mm / {np.degrees(angle):.0f}° from the "
                                  f"kinematics (over {limits}): is the pendant's active TCP "
                                  f"{_tcp_text(arm.pendant_tcp)}? Else the URDF or its calibration is off")
    return Check("tcp", GOOD, f"reported tool0 within {limits} of the kinematics")


def _tcp_text(tcp: Pose) -> str:
    """A TCP as the pendant takes it: position in mm and rotation vector in degrees; "zero" for tool0."""
    if np.allclose(tcp.position, 0.0) and np.allclose(np.abs(tcp.orientation), (0.0, 0.0, 0.0, 1.0)):
        return "zero, i.e. tool0"
    position = np.round(np.asarray(tcp.position) * 1e3, 1)
    rotation = np.round(np.degrees(Rotation.from_quat(tcp.orientation).as_rotvec()), 1)
    return f"xyz {position.tolist()} mm, rotation vector {rotation.tolist()}°"
