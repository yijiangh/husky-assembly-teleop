"""The arm's own guards: every trajectory, Hold, Cartesian target and wrench is checked before it goes out."""

from __future__ import annotations

import numpy as np
import pytest
from robot_fakes import TCP_ORIENTATION, TCP_POSITION, Publisher, fake_arm
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER, CARTESIAN_MAX_STEP,
                                                       CARTESIAN_STEP_WINDOW, FREE_DRIVE_CONTROLLER,
                                                       FREE_DRIVE_REQUEST_TTL, HOLD_DURATION,
                                                       JOINT_STATE_MAX_AGE, SCALED_JOINT_TRAJECTORY_CONTROLLER,
                                                       TRAJECTORY_MAX_JOINT_ACCELERATION, UR_JOINT_NAMES,
                                                       MAX_TARGET_FORCE, MAX_TARGET_TORQUE, MOVING_WITHIN,
                                                       START_TOLERANCE, SYNC_STATUS_MAX_AGE, TRAJECTORY_MAX_DURATION,
                                                       TRAJECTORY_MAX_JOINT_SPEED, TRAJECTORY_MAX_WAYPOINTS,
                                                       UR_JOINT_LIMITS, JOG_DISTANCE, JOG_LIMIT_MARGIN,
                                                       JOG_MAX_SPEED, joint_jog, joint_move, joint_path_move)
from husky_assembly_teleop.robot_interface.tool_step import TOOL_JOG_LEAD, TOOL_JOG_MAX_JOINT_SPEED
from husky_assembly_teleop.robot_interface.controller_manager import CONTROLLER_STATE_MAX_AGE
from husky_assembly_teleop.robot_interface.simulated_parts import SimulatedPart
from husky_assembly_teleop.robot_interface.robot_lease import Lease, NotLeased
from husky_assembly_teleop.robot_interface.soft_stop import RobotStopped

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
    """Waypoints spread evenly over the duration, the arm counts as executing, and the trajectory is fired as sent."""
    arm = fake_arm()
    heard = []
    arm.trajectories_sent.attach(heard.append)
    assert arm.send_joint_trajectory(*line(arm))
    trajectory, = arm._trajectory.sent
    assert len(trajectory.points) == 11 and trajectory.points[-1].time_from_start.sec == 10
    assert arm.state.is_executing and arm.state.last_motion_time == arm._node.now
    sent, = heard
    np.testing.assert_allclose(sent.target, trajectory.points[-1].positions)
    assert (sent.duration, sent.waypoints, sent.hold, sent.update_time) == (10.0, 11, 0, arm._node.now)


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


def _program_stopped(arm):
    arm.state.program_playing = False
    return line(arm)


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
    (_program_stopped, "program is stopped"),
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


@pytest.mark.parametrize("unknown", [
    lambda arm: setattr(arm.state, "status_update_time", arm.state.status_update_time - SYNC_STATUS_MAX_AGE - 0.1),
    lambda arm: setattr(arm.state, "dashboard_up", False),
])
def test_a_real_arm_whose_safety_is_unknown_is_refused(unknown):
    """No recent safety-sync status, or no dashboard: a real arm does not move."""
    arm = fake_arm()
    unknown(arm)
    assert "safety is not known" in refused(arm, *line(arm))


def test_fake_hardware_moves_without_a_dashboard_but_not_without_the_sync():
    """A simulated arm has no dashboard, which is fine; it still needs the safety sync's recent, operational status."""
    simulated = SimulatedPart("ur5e", "ur5e/controller_manager", "ur fake hardware", 100.0)
    arm = fake_arm(simulated=simulated)
    arm.state.dashboard_up = False
    assert arm.send_joint_trajectory(*line(arm))
    silent = fake_arm(simulated=simulated)
    silent.state.status_update_time -= SYNC_STATUS_MAX_AGE + 0.1
    assert "safety is not known" in refused(silent, *line(silent))


@pytest.mark.parametrize("unsafe", [
    lambda arm: setattr(arm.state, "status_update_time", arm.state.status_update_time - SYNC_STATUS_MAX_AGE - 0.1),
    lambda arm: setattr(arm.state, "operational", False),
    lambda arm: setattr(arm.state, "program_playing", False),
    lambda arm: setattr(arm._latch, "stopped", True),
])
def test_free_drive_needs_the_safety_sync(unsafe):
    """Not started, and its heartbeat stops (free drive ends in 1 s), while unsafe for the sync or soft stopped."""
    arm = fake_arm(FREE_DRIVE_CONTROLLER)
    arm.state.controllers.requested, arm._free_drive = FREE_DRIVE_CONTROLLER, Publisher()
    arm.request_free_drive()
    arm._keep_free_drive()
    assert len(arm._free_drive.sent) == 1 and arm._switch_refusal(FREE_DRIVE_CONTROLLER) is None
    unsafe(arm)
    arm.request_free_drive()
    arm._keep_free_drive()
    assert len(arm._free_drive.sent) == 1 and "ending free drive" in arm._node.log.text("error")
    assert arm._switch_refusal(FREE_DRIVE_CONTROLLER)
    assert arm._switch_refusal(SCALED_JOINT_TRAJECTORY_CONTROLLER) is None


def test_free_drive_runs_only_while_requested():
    """Running the controller alone keeps the arm stiff; a request frees it for FREE_DRIVE_REQUEST_TTL."""
    arm = fake_arm(FREE_DRIVE_CONTROLLER)
    arm.state.controllers.requested, arm._free_drive = FREE_DRIVE_CONTROLLER, Publisher()
    arm._keep_free_drive()
    assert not arm._free_drive.sent
    arm.request_free_drive()
    arm._keep_free_drive()
    arm._node.now += FREE_DRIVE_REQUEST_TTL + 0.01
    arm.state.status_update_time = arm.state.controllers.last_update_time = arm._node.now
    arm._keep_free_drive()
    assert len(arm._free_drive.sent) == 1


# --- --- --- --- --- paths --- --- --- --- ---

def test_a_path_planned_a_degree_away_from_the_arm_is_refused():
    """Its first point runs at time 0, so the arm would jump the gap: the path must start where the arm is."""
    arm = fake_arm()
    here = arm.joint_vector()
    positions, velocities, duration = joint_path_move([here, here + DELTA])
    assert arm.send_joint_trajectory(positions, duration, velocities)
    away = fake_arm()
    positions, velocities, duration = joint_path_move([here + np.radians(1.0), here + DELTA])
    assert "not at the first waypoint" in refused(away, positions, duration, velocities)


@pytest.mark.parametrize("speed", [JOG_MAX_SPEED, 0.05 * JOG_MAX_SPEED])
def test_a_jog_passes_every_check_and_rests_jog_distance_on(speed):
    """Only the one joint moves, never faster than asked, and it comes to rest JOG_DISTANCE away."""
    arm = fake_arm()
    positions, velocities, duration = joint_jog(arm.joint_vector(), 1, -1.0, speed)
    assert arm.send_joint_trajectory(positions, duration, velocities)
    assert np.max(np.abs(velocities[:, 1])) == pytest.approx(speed)
    np.testing.assert_allclose(positions[-1], [0.0, -JOG_DISTANCE, 0.0, 0.0, 0.0, 0.0], atol=1e-12)
    assert not np.any(velocities[-1]) and not np.any(np.delete(positions, 1, axis=1))


def test_a_jog_stops_short_of_the_joint_limit_and_refuses_at_it():
    """Near the elbow's limit the jog ends JOG_LIMIT_MARGIN short of it; at the limit there is nothing to jog."""
    near = UR_JOINT_LIMITS[2] - JOG_LIMIT_MARGIN - np.radians(5.0)
    positions, velocities, duration = joint_jog([0.0, 0.0, near, 0.0, 0.0, 0.0], 2, 1.0)
    assert positions[-1, 2] == pytest.approx(UR_JOINT_LIMITS[2] - JOG_LIMIT_MARGIN)
    assert fake_arm(joints=positions[0]).send_joint_trajectory(positions, duration, velocities)
    with pytest.raises(ValueError, match="elbow"):
        joint_jog(positions[-1], 2, 1.0)


def test_a_jog_point_goes_out_as_one_point_a_lead_ahead():
    """One point, positions only, reaching in TOOL_JOG_LEAD, and the arm counts as executing."""
    arm = fake_arm()
    target = arm.joint_vector() + np.radians([1.0, 0, 0, 0, 0, 0])
    assert arm.send_jog_point(target)
    trajectory, = arm._trajectory.sent
    point, = trajectory.points
    np.testing.assert_allclose(point.positions, target)
    assert not point.velocities and point.time_from_start.nanosec == pytest.approx(TOOL_JOG_LEAD * 1e9, abs=1)
    assert arm.state.is_executing and arm.state.last_motion_time == arm._node.now


def _too_fast(arm):
    """A point one joint would need to reach faster than TOOL_JOG_MAX_JOINT_SPEED."""
    return arm.joint_vector() + np.array([TOOL_JOG_MAX_JOINT_SPEED * 1.01 * TOOL_JOG_LEAD, 0, 0, 0, 0, 0])


def _stale(arm):
    """The arm's own joints, but measured too long ago."""
    arm.state.last_update_time = arm._node.now - JOINT_STATE_MAX_AGE - 0.1
    return arm.joint_vector()


def _under_compliance(arm):
    """The arm's own joints, with the compliance controller running."""
    arm.state.controllers.active = CARTESIAN_COMPLIANCE_CONTROLLER
    return arm.joint_vector()


@pytest.mark.parametrize("make, why", [
    (_too_fast, "°/s"),
    (lambda arm: np.array([0, 0, UR_JOINT_LIMITS[2] + 0.01, 0, 0, 0]), "joint limit"),
    (lambda arm: np.full(6, np.nan), "finite"),
    (_stale, "stale"),
    (_under_compliance, "is not active"),
])
def test_jog_point_refused(make, why):
    """A point too fast, past a limit, not finite, on a stale joint state or under another controller stays in."""
    arm = fake_arm()
    target = make(arm)
    assert not arm.send_jog_point(target)
    assert not arm._trajectory.sent and why in arm.last_refusal.reason and arm.last_refusal.label == "jog refused"


# --- --- --- --- --- hold --- --- --- --- ---

def _stop(arm) -> tuple[np.ndarray, float]:
    """The one point of the hold trajectory sent: positions and time from start."""
    point, = arm._trajectory.sent[-1].points
    assert list(point.velocities) == [0.0] * 6
    return np.array(point.positions), point.time_from_start.sec + point.time_from_start.nanosec * 1e-9


def test_hold_at_rest_stays_at_the_measured_joints():
    """At rest, the hold point is where the arm is, HOLD_DURATION from now, and the target wrench is zeroed."""
    joints = np.radians([10.0, -80.0, 90.0, -100.0, -90.0, 30.0])
    arm = fake_arm(joints=joints)
    trajectories, wrenches = [], []
    arm.trajectories_sent.attach(trajectories.append)
    arm.wrenches_sent.attach(wrenches.append)
    assert arm.hold()
    assert trajectories[0].hold == 1 and np.allclose(trajectories[0].target, joints)
    assert wrenches[0].force.tolist() == [0.0, 0.0, 0.0] and wrenches[0].torque.tolist() == [0.0, 0.0, 0.0]
    positions, duration = _stop(arm)
    np.testing.assert_allclose(positions, joints)
    assert duration == pytest.approx(HOLD_DURATION)
    wrench, = arm._target_wrench.sent
    assert (wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z) == (0.0, 0.0, 0.0)


def test_hold_of_a_moving_arm_brakes_evenly_ahead_of_it_never_back():
    """From speed v, measured `age` ago, the arm rests v (age + T / 2) on, T at the trajectory deceleration."""
    arm = fake_arm()
    speed = np.radians([60.0, -20.0, 0.0, 0.0, 0.0, 10.0])
    arm.state.joint_velocities = {f"ur_arm_{name}": float(v) for name, v in zip(UR_JOINT_NAMES, speed)}
    arm.state.last_update_time -= 0.05
    assert arm.hold()
    positions, duration = _stop(arm)
    assert duration == pytest.approx(np.radians(60.0) / TRAJECTORY_MAX_JOINT_ACCELERATION)
    np.testing.assert_allclose(positions, speed * (0.05 + duration / 2))


@pytest.mark.parametrize("unknown", [
    lambda arm: setattr(arm.state, "status_update_time", None),
    lambda arm: setattr(arm.state, "program_playing", False),
    lambda arm: setattr(arm.state.controllers, "last_update_time", arm._node.now - 60.0),
    lambda arm: setattr(arm.state.controllers, "active", None),
])
def test_hold_is_sent_whatever_the_sync_program_or_controller_state(unknown):
    """Stopping is always allowed: no sync status, a stopped program, an old or unknown controller state."""
    arm = fake_arm()
    unknown(arm)
    assert arm.hold() and arm._trajectory.sent


def test_hold_in_free_drive_switches_to_the_trajectory_controller():
    """The trajectory controller holds the arm where it is once it runs."""
    arm = fake_arm(FREE_DRIVE_CONTROLLER)
    switched = []
    arm.controllers.switch = lambda name, unguarded=False: switched.append(name) or True
    assert arm.hold() and switched == [SCALED_JOINT_TRAJECTORY_CONTROLLER] and not arm._trajectory.sent


@pytest.mark.parametrize("controller, broken", [
    (SCALED_JOINT_TRAJECTORY_CONTROLLER, "stale"),
    (SCALED_JOINT_TRAJECTORY_CONTROLLER, "nan"),
    (CARTESIAN_COMPLIANCE_CONTROLLER, "stale"),
])
def test_hold_refused_on_a_stale_or_broken_measurement(controller, broken):
    """Only then: the target could be far from the arm. Nothing is sent and the log says so."""
    arm = fake_arm(controller)
    if broken == "stale":
        arm.state.last_update_time -= JOINT_STATE_MAX_AGE + 0.1
        arm.state.tcp_update_time -= JOINT_STATE_MAX_AGE + 0.1
    else:
        arm.state.joint_positions["ur_arm_elbow_joint"] = np.nan
    assert not arm.hold()
    assert not arm._trajectory.sent and not arm._target_frame.sent
    assert "HOLD NOT SENT" in arm._node.log.text()


def test_hold_under_compliance_targets_the_tcp():
    """The target is the measured tool0, even with a URDF that breaks the stock frames, and the wrench is zeroed."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    arm.urdf_problem = "turned base"
    heard = []
    arm.targets_sent.attach(heard.append)
    assert arm.hold()
    assert heard[0].hold == 1 and np.allclose(heard[0].position, TCP_POSITION)
    target, = arm._target_frame.sent
    np.testing.assert_allclose([target.pose.position.x, target.pose.position.y, target.pose.position.z],
                               TCP_POSITION)
    assert arm._target_wrench.sent[-1].wrench.force.z == 0.0


def test_hold_under_compliance_right_after_a_target_far_from_the_tcp():
    """The arm lags a target sent just now by more than a step (contact): Hold still goes out."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    arm.state.sent_target_position = TCP_POSITION + [2 * CARTESIAN_MAX_STEP, 0.0, 0.0]
    arm.state.sent_target_orientation, arm.state.sent_target_time = TCP_ORIENTATION, arm._node.now
    arm.state.program_playing = False
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
    """In cartesian_test_mode a good target returns False, is not published or fired, but is kept for the 3D view."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER, cartesian_test_mode=True)
    heard = []
    arm.targets_sent.attach(heard.append)
    assert not arm.send_cartesian_target(TCP_POSITION, TCP_ORIENTATION)
    assert not arm._target_frame.sent and arm.state.sent_target_time == arm._node.now and not heard
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


def test_a_soft_stopped_arm_refuses_every_motion_but_a_hold():
    """Trajectories, joint moves, Cartesian targets and wrenches raise RobotStopped, sending nothing; holds go out."""
    arm, compliance = fake_arm(), fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    arm._latch.stopped = compliance._latch.stopped = True
    for command in (lambda: arm.send_joint_trajectory(*line(arm)), lambda: arm.send_joint_move(np.full(6, 0.1)),
                    lambda: compliance.send_cartesian_target(TCP_POSITION, TCP_ORIENTATION),
                    lambda: compliance.send_target_wrench([0.0, 0.0, 5.0])):
        with pytest.raises(RobotStopped, match="soft stopped"):
            command()
    assert not arm._trajectory.sent and not compliance._target_frame.sent and not compliance._target_wrench.sent
    assert arm.hold() and len(arm._trajectory.sent) == 1


@pytest.mark.parametrize("holder", [None, "a planner"])
def test_without_its_lease_an_arm_refuses_every_motion_and_switch_but_a_hold(holder):
    """Free or leased to another, every motion and controller switch raises or is refused; a hold goes out."""
    arm, compliance = fake_arm(), fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    arm._lease_slot.lease = compliance._lease_slot.lease = None
    lease = None if holder is None else Lease(holder, (arm._lease_slot, compliance._lease_slot))
    why = "robot 0804 is not leased" if holder is None else "robot 0804 is leased by a planner"
    for command in (lambda: arm.send_joint_trajectory(*line(arm)), lambda: arm.send_joint_move(np.full(6, 0.1)),
                    lambda: compliance.send_cartesian_target(TCP_POSITION, TCP_ORIENTATION),
                    lambda: compliance.send_target_wrench([0.0, 0.0, 5.0])):
        with pytest.raises(NotLeased, match=why):
            command()
    assert not arm._trajectory.sent and not compliance._target_frame.sent and not compliance._target_wrench.sent
    assert why in arm._switch_refusal(FREE_DRIVE_CONTROLLER)
    assert why in arm._switch_refusal(SCALED_JOINT_TRAJECTORY_CONTROLLER)
    if lease is not None:
        with lease.use():
            assert arm._switch_refusal(FREE_DRIVE_CONTROLLER) is None
            assert arm.send_joint_trajectory(*line(arm)) and compliance.send_target_wrench([0.0, 0.0, 5.0])
    arm.state.last_motion_time = None
    assert arm.hold() and len(arm._trajectory.sent) == (1 if lease is None else 2)


def test_free_drive_stays_on_only_while_requested_under_the_lease():
    """Requested under the robot's lease, free drive stays on; requested without it, it ends."""
    arm = fake_arm(FREE_DRIVE_CONTROLLER)
    arm.state.controllers.requested, arm._free_drive = FREE_DRIVE_CONTROLLER, Publisher()
    lease = Lease("calibration", (arm._lease_slot,))
    with lease.use():
        arm.request_free_drive()
    arm._keep_free_drive()
    assert len(arm._free_drive.sent) == 1
    arm.request_free_drive()
    arm._keep_free_drive()
    assert len(arm._free_drive.sent) == 1 and "leased by calibration" in arm._node.log.text("error")
