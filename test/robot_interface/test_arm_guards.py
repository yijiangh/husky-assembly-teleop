"""The arm's own guards: every trajectory, Hold, Cartesian target and wrench is checked before it goes out."""

from __future__ import annotations

import numpy as np
import pytest
from robot_fakes import TCP_ORIENTATION, TCP_POSITION, fake_arm
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER, CARTESIAN_MAX_STEP,
                                                       CARTESIAN_STEP_WINDOW, CONTROLLER_STATE_MAX_AGE,
                                                       HOLD_DURATION, JOINT_STATE_MAX_AGE, MAX_TARGET_FORCE,
                                                       MAX_TARGET_TORQUE, MOVING_WITHIN, START_TOLERANCE,
                                                       SYNC_STATUS_MAX_AGE, TRAJECTORY_MAX_DURATION,
                                                       TRAJECTORY_MAX_JOINT_SPEED, TRAJECTORY_MAX_WAYPOINTS,
                                                       UR_JOINT_LIMITS, joint_move)

pytestmark = pytest.mark.safety

#: A slow move: 10 degrees on three joints in 10 s.
DELTA = np.radians([10.0, -10.0, 10.0, 0.0, 0.0, 0.0])


def line(arm, delta=DELTA, duration: float = 10.0, count: int = 11):
    """Evenly spaced waypoints from where the arm is to `delta` away, and the duration."""
    start = arm.joint_vector()
    return np.linspace(start, start + np.asarray(delta), count), duration


def refused(arm, positions, duration, velocities=None) -> str:
    """Send the trajectory, check nothing went out, and return what the log says."""
    assert not arm.send_joint_trajectory(positions, duration, velocities)
    assert not arm._trajectory.sent
    return arm._node.log.text("error")


# --- --- --- --- --- trajectories --- --- --- --- ---

def test_a_slow_trajectory_from_the_arm_goes_out():
    """Waypoints spread evenly over the duration, and the arm counts as executing."""
    arm = fake_arm()
    assert arm.send_joint_trajectory(*line(arm))
    trajectory, = arm._trajectory.sent
    assert len(trajectory.points) == 11 and trajectory.points[-1].time_from_start.sec == 10
    assert arm.state.is_executing and arm.state.last_motion_time == arm._node.now


@pytest.mark.parametrize("big", [np.radians([170.0, -90.0, 150.0, 200.0, -300.0, 10.0]), np.full(6, 1e-4)])
def test_joint_moves_of_any_size_pass_the_checks(big):
    """`joint_move` keeps within the speed and acceleration limits, from a huge move to a tiny one."""
    arm = fake_arm()
    positions, velocities, duration = joint_move(arm.joint_vector(), big)
    assert arm.send_joint_trajectory(positions, duration, velocities)


def _not_active(arm):
    arm.state.controllers.active = CARTESIAN_COMPLIANCE_CONTROLLER
    return line(arm)


def _controllers_silent(arm):
    arm.state.controllers.last_update_time -= CONTROLLER_STATE_MAX_AGE + 0.1
    return line(arm)


def _not_operational(arm):
    arm.state.operational, arm.state.problem = False, "protective stop"
    return line(arm)


def _no_joint_state(arm):
    arm.state.joint_positions.pop("ur_arm_wrist_3_joint")
    return np.zeros((2, 6)), 10.0


def _old_joint_state(arm):
    arm.state.last_update_time -= JOINT_STATE_MAX_AGE + 0.1
    return line(arm)


def _still_moving(arm):
    arm.state.last_motion_time = arm._node.now - MOVING_WITHIN / 2
    return line(arm)


def _away_from_the_start(arm):
    positions, duration = line(arm)
    return positions + START_TOLERANCE * 1.5, duration


def _past_the_elbow_limit(arm):
    elbow = UR_JOINT_LIMITS[2] + 0.01
    arm.state.joint_positions["ur_arm_elbow_joint"] = elbow - 0.05
    return line(arm, delta=[0.0, 0.0, 0.05, 0.0, 0.0, 0.0])


@pytest.mark.parametrize("make, why", [
    (_not_active, "is not active"),
    (_controllers_silent, "has not answered"),
    (_not_operational, "not operational: protective stop"),
    (lambda arm: (np.zeros((2, 5)), 10.0), "at least two waypoints"),
    (lambda arm: (np.zeros((1, 6)), 10.0), "at least two waypoints"),
    (lambda arm: ([[0.0] * 6, [0.0] * 5], 10.0), "not a list of equal-length lists"),
    (lambda arm: line(arm, duration=0.0), "duration must be positive"),
    (lambda arm: line(arm, duration=-1.0), "duration must be positive"),
    (lambda arm: line(arm, duration=np.nan), "duration must be positive"),
    (lambda arm: line(arm, duration=TRAJECTORY_MAX_DURATION + 1.0), "duration must be positive"),
    (lambda arm: line(arm, count=TRAJECTORY_MAX_WAYPOINTS + 1, duration=TRAJECTORY_MAX_DURATION),
     "over the limit of"),
    (lambda arm: (np.vstack([np.zeros(6), np.full(6, np.nan)]), 10.0), "NaN or infinity"),
    (_past_the_elbow_limit, "elbow_joint past its limit"),
    (lambda arm: line(arm, duration=np.radians(10.0) / TRAJECTORY_MAX_JOINT_SPEED / 2), "deg/s on some joint"),
    (lambda arm: (np.array([np.zeros(6), np.full(6, 0.05), np.zeros(6)]), 0.2), "deg/s^2 on some joint"),
    (_no_joint_state, "no joint state yet"),
    (_old_joint_state, "joint state is"),
    (_still_moving, "still moving"),
    (_away_from_the_start, "not at the first waypoint"),
], ids=lambda value: value if isinstance(value, str) else "")
def test_trajectory_refused(make, why):
    """Each check refuses on its own, says why, and nothing is published."""
    arm = fake_arm()
    assert why in refused(arm, *make(arm))


@pytest.mark.parametrize("velocities, why", [
    (np.zeros((3, 6)), "shaped like the waypoints"),
    (np.vstack([np.zeros((10, 6)), np.full((1, 6), np.inf)]), "must be finite"),
    (np.vstack([np.zeros((10, 6)), np.full((1, 6), 0.01)]), "must end at rest"),
    (np.vstack([np.zeros((5, 6)), np.full((6, 6), TRAJECTORY_MAX_JOINT_SPEED * 1.1)]), "deg/s on some joint"),
])
def test_trajectory_velocities_refused(velocities, why):
    """Given velocities must match the waypoints, stay finite and within the speed limit, and end at zero."""
    arm = fake_arm()
    assert why in refused(arm, *line(arm), velocities)


def test_trajectory_without_safety_state_goes_out_with_a_warning():
    """No status from the safety sync (fake hardware): sent, but the log says safety was not checked."""
    arm = fake_arm()
    arm.state.status_update_time -= SYNC_STATUS_MAX_AGE + 0.1
    assert arm.send_joint_trajectory(*line(arm))
    assert "safety not checked" in arm._node.log.text("warning")


# --- --- --- --- --- hold --- --- --- --- ---

def test_hold_under_the_trajectory_controller_stops_at_the_measured_joints():
    """One point at the measured joints, at rest, HOLD_DURATION from now."""
    joints = np.radians([10.0, -80.0, 90.0, -100.0, -90.0, 30.0])
    arm = fake_arm(joints=joints)
    assert arm.hold()
    point, = arm._trajectory.sent[0].points
    np.testing.assert_allclose(point.positions, joints)
    assert list(point.velocities) == [0.0] * 6
    assert point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 == pytest.approx(HOLD_DURATION)


@pytest.mark.parametrize("controller, stale, why", [
    ("free_drive_controller", False, "neither motion controller is active"),
    (None, False, "neither motion controller is active"),
    ("scaled_joint_trajectory_controller", True, "HOLD NOT SENT"),
    (CARTESIAN_COMPLIANCE_CONTROLLER, True, "HOLD NOT SENT"),
])
def test_hold_refused(controller, stale, why):
    """Without a motion controller, or on a stale measurement, Hold sends nothing and says so."""
    arm = fake_arm(controller)
    if stale:
        arm.state.last_update_time -= JOINT_STATE_MAX_AGE + 0.1
        arm.state.tcp_update_time -= JOINT_STATE_MAX_AGE + 0.1
    assert not arm.hold()
    assert not arm._trajectory.sent and not arm._target_frame.sent
    assert why in arm._node.log.text()


def test_hold_under_compliance_targets_the_tcp():
    """The target is the measured tool0, even with a URDF that breaks the stock frames."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    arm.urdf_problem = "turned base"
    assert arm.hold()
    target, = arm._target_frame.sent
    np.testing.assert_allclose([target.pose.position.x, target.pose.position.y, target.pose.position.z],
                               TCP_POSITION)


@pytest.mark.xfail(strict=True, reason="Hold runs the step check against the last target (review: Fix first)")
def test_hold_under_compliance_right_after_a_target_far_from_the_tcp():
    """The arm lags a target sent just now by more than a step (contact): Hold must still go out."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    arm.state.sent_target_position = TCP_POSITION + [2 * CARTESIAN_MAX_STEP, 0.0, 0.0]
    arm.state.sent_target_orientation, arm.state.sent_target_time = TCP_ORIENTATION, arm._node.now
    assert arm.hold()


# --- --- --- --- --- cartesian targets --- --- --- --- ---

def turned(degrees: float) -> np.ndarray:
    """TCP_ORIENTATION turned about z."""
    return (Rotation.from_quat(TCP_ORIENTATION) * Rotation.from_euler("z", degrees, degrees=True)).as_quat()


def test_a_close_target_goes_out_and_is_remembered():
    """A target 5 mm and 1 degree from the TCP is published in base_link and kept as the last one sent."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    position = TCP_POSITION + [0.005, 0.0, 0.0]
    assert arm.send_cartesian_target(position, turned(1.0))
    message, = arm._target_frame.sent
    assert message.header.frame_id == "base_link"
    np.testing.assert_allclose(arm.state.sent_target_position, position)
    assert arm.state.sent_target_time == arm._node.now


def test_test_mode_checks_and_records_but_never_sends():
    """In cartesian_test_mode a good target returns False, is not published, but is kept for the 3D view."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER, cartesian_test_mode=True)
    assert not arm.send_cartesian_target(TCP_POSITION, TCP_ORIENTATION)
    assert not arm._target_frame.sent and arm.state.sent_target_time == arm._node.now
    assert "TEST MODE" in arm._node.log.text("warning")


def _far_tcp(arm):
    arm.state.tcp_position = np.array([0.0, 0.9, 0.163])
    return arm.state.tcp_position, TCP_ORIENTATION


def _stale_tcp(arm):
    arm.state.tcp_update_time -= JOINT_STATE_MAX_AGE + 0.1
    return TCP_POSITION, TCP_ORIENTATION


def _no_tcp(arm):
    arm.state.tcp_position = None
    return TCP_POSITION, TCP_ORIENTATION


def _broken_urdf(arm):
    arm.urdf_problem = "turned base"
    return TCP_POSITION, TCP_ORIENTATION


def _big_step(arm):
    arm.state.sent_target_position, arm.state.sent_target_orientation = TCP_POSITION, TCP_ORIENTATION
    arm.state.sent_target_time = arm._node.now - CARTESIAN_STEP_WINDOW / 2
    return TCP_POSITION + [0.0, 1.5 * CARTESIAN_MAX_STEP, 0.0], TCP_ORIENTATION


def _big_turn_step(arm):
    _big_step(arm)
    return TCP_POSITION, turned(4.0)


def _wrong_controller(arm):
    arm.state.controllers.active = "scaled_joint_trajectory_controller"
    return TCP_POSITION, TCP_ORIENTATION


@pytest.mark.parametrize("make, why", [
    (_wrong_controller, "is not active"),
    (_broken_urdf, "breaks the stock UR frames"),
    (lambda arm: (TCP_POSITION[:2], TCP_ORIENTATION), "3 finite numbers"),
    (lambda arm: ([np.nan, 0.0, 0.4], TCP_ORIENTATION), "3 finite numbers"),
    (lambda arm: (TCP_POSITION, [0.0, 0.0, 0.0, 2.0]), "not a unit quaternion"),
    (_no_tcp, "no TCP pose yet"),
    (_stale_tcp, "TCP pose is"),
    (lambda arm: (TCP_POSITION + [0.06, 0.0, 0.0], TCP_ORIENTATION), "too far from the TCP"),
    (lambda arm: (TCP_POSITION, turned(6.0)), "is turned"),
    (_far_tcp, "out of reach"),
    (_big_step, "jumps"),
    (_big_turn_step, "jumps"),
], ids=lambda value: value if isinstance(value, str) else "")
def test_cartesian_target_refused(make, why):
    """Each check refuses on its own, says why, and nothing is published."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    assert not arm.send_cartesian_target(*make(arm))
    assert not arm._target_frame.sent
    assert why in arm._node.log.text("error")


def test_steps_are_free_again_after_the_window():
    """A target sent longer ago than CARTESIAN_STEP_WINDOW no longer limits the next step."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    _big_step(arm)
    arm.state.sent_target_time = arm._node.now - CARTESIAN_STEP_WINDOW - 0.1
    assert arm.send_cartesian_target(TCP_POSITION + [0.0, 1.5 * CARTESIAN_MAX_STEP, 0.0], TCP_ORIENTATION)


# --- --- --- --- --- wrenches --- --- --- --- ---

def test_a_wrench_within_the_limits_goes_out_in_tool0():
    """Published in the tool frame."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    assert arm.send_target_wrench([0.0, 0.0, MAX_TARGET_FORCE * 0.9], [0.0, 0.0, MAX_TARGET_TORQUE * 0.9])
    message, = arm._target_wrench.sent
    assert message.header.frame_id == "tool0" and message.wrench.force.z == pytest.approx(MAX_TARGET_FORCE * 0.9)


@pytest.mark.parametrize("controller, force, torque, why", [
    ("scaled_joint_trajectory_controller", (0.0, 0.0, 1.0), (0.0, 0.0, 0.0), "is not active"),
    (CARTESIAN_COMPLIANCE_CONTROLLER, (MAX_TARGET_FORCE, 1.0, 0.0), (0.0, 0.0, 0.0), "over the limit"),
    (CARTESIAN_COMPLIANCE_CONTROLLER, (0.0, 0.0, 0.0), (MAX_TARGET_TORQUE, 0.0, 1.0), "over the limit"),
    (CARTESIAN_COMPLIANCE_CONTROLLER, (np.nan, 0.0, 0.0), (0.0, 0.0, 0.0), "3 finite numbers"),
    (CARTESIAN_COMPLIANCE_CONTROLLER, (0.0, 0.0), (0.0, 0.0, 0.0), "3 finite numbers"),
])
def test_wrench_refused(controller, force, torque, why):
    """Wrong controller, over the force or torque limit, or not finite: nothing goes out."""
    arm = fake_arm(controller)
    assert not arm.send_target_wrench(force, torque)
    assert not arm._target_wrench.sent and why in arm._node.log.text("error")


def test_test_mode_never_sends_a_wrench():
    """A wrench that passes is not published in cartesian_test_mode."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER, cartesian_test_mode=True)
    assert not arm.send_target_wrench([0.0, 0.0, 5.0])
    assert not arm._target_wrench.sent
