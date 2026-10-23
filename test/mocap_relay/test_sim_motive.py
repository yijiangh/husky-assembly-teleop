"""Motive in a simulation: the raw rigid bodies sim_motive gives the relay."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from repo_paths import DATA
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.mocap_relay.node import apply_base_calibration
from husky_assembly_teleop.mocap_relay.sim_motive import (drifted, lost, marker_pose, noisy, observed, offset_joints,
                                                          offset_pose, rigid_body)
from husky_assembly_teleop.world.calibration_sessions import Calibration
from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.kinematics import link_pose

pytestmark = pytest.mark.safety

#: A pose that is not the identity, and the 0805's arm in a bent pose.
POSE = offset_pose([1.2, -0.4, 0.3, 10.0, -20.0, 75.0])
JOINTS = {f"ur_arm_{name}": value for name, value in zip(
    ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"),
    (0.3, -1.2, 1.4, -1.5, 1.6, 0.2))}


def angle(a: Pose, b: Pose) -> float:
    """The angle between two poses' orientations, degrees."""
    return float(np.degrees((Rotation.from_quat(a.orientation).inv() * Rotation.from_quat(b.orientation)).magnitude()))


def test_the_relay_turns_the_raw_body_back_into_the_true_footprint():
    """Calibrated with the true calibration, the relay publishes exactly the simulator's base_footprint."""
    footprint = Pose((1.7, 1.35, 0.0), tuple(Rotation.from_euler("z", 90, degrees=True).as_quat()))
    truth = offset_pose([0.056, 0.011, -0.479, 0.3, -0.2, 90.4])
    raw = rigid_body(footprint, truth)
    calibration = Calibration(1850, "a200-0805", truth.position, truth.orientation, "test", "", ())
    position, quaternion = apply_base_calibration(calibration, raw.position, raw.orientation)
    np.testing.assert_allclose(position, footprint.position, atol=1e-12)
    turn = Rotation.from_quat(quaternion).inv() * Rotation.from_quat(footprint.orientation)
    assert turn.magnitude() < 1e-9


def test_the_marker_sits_at_its_offset_on_tool0():
    """The marker body is tool0 by forward kinematics, moved by the offset in tool0's frame."""
    urdf = robot_config_from_serial("0805", DATA).model.urdf
    footprint = Pose((1.0, 2.0, 0.0))
    joints = {f"ur_arm_{name}": value for name, value in zip(
        ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
         "wrist_3_joint"), (0.3, -1.2, 1.4, -1.5, 1.6, 0.2))}
    offset = offset_pose([0.01, -0.005, 0.025, 0.0, 0.0, 15.0])
    marker = marker_pose(urdf, footprint, joints, "ur_arm", offset)
    tool0 = link_pose(urdf, footprint, joints, "ur_arm_tool0")
    assert marker == compose(tool0, offset)
    assert np.linalg.norm(np.subtract(marker.position, tool0.position)) == pytest.approx(np.linalg.norm([0.01, -0.005,
                                                                                                         0.025]))


def test_zero_settings_report_the_true_pose_exactly():
    """Without noise, drift or joint offsets a body is reported exactly where it is, as before they existed."""
    rng = np.random.default_rng(0)
    assert observed(POSE, [0.0] * 6, 12.0, rng, [0.0, 0.0]) is POSE
    assert offset_joints(JOINTS, "ur_arm", [0.0] * 6) == JOINTS
    assert lost(rng, 0.0, 0.0, 5.0, -np.inf) == (False, -np.inf)


def test_noise_has_the_asked_spread_and_no_bias():
    """Position noise per axis and the turn's rotation vector per axis have the given standard deviations."""
    rng = np.random.default_rng(3)
    samples = [noisy(POSE, rng, 1e-4, 0.05) for _ in range(20000)]
    shifts = np.array([s.position for s in samples]) - POSE.position
    turns = np.array([(Rotation.from_quat(POSE.orientation).inv() * Rotation.from_quat(s.orientation)).as_rotvec(
        degrees=True) for s in samples])
    np.testing.assert_allclose(shifts.std(axis=0), 1e-4, rtol=0.05)
    np.testing.assert_allclose(turns.std(axis=0), 0.05, rtol=0.05)
    assert np.abs(shifts.mean(axis=0)).max() < 5e-6 and np.abs(turns.mean(axis=0)).max() < 2e-3


def test_noise_repeats_with_its_seed():
    """The same seed gives the same samples, so a simulated run can be repeated."""
    samples = [noisy(POSE, np.random.default_rng(7), 1e-3, 0.1) for _ in range(2)]
    assert samples[0] == samples[1] and samples[0] != POSE


def test_drift_grows_linearly_with_time_along_a_fixed_direction():
    """A drift's offset in the body's frame is its rate times the minutes, in distance, direction and angle."""
    rate = [1e-3, -2e-3, 0.5e-3, 0.0, 0.0, 0.2]
    for minutes in (1.0, 2.5, 10.0):
        shift = compose(invert(POSE), drifted(POSE, rate, minutes))
        np.testing.assert_allclose(shift.position, np.multiply(rate[:3], minutes), atol=1e-12)
        assert angle(POSE, drifted(POSE, rate, minutes)) == pytest.approx(0.2 * minutes)


def test_joint_offsets_move_the_marker_as_the_offset_arm_would():
    """The marker follows forward kinematics of the joints plus the offsets; the given joints stay unchanged."""
    urdf = robot_config_from_serial("0805", DATA).model.urdf
    footprint, offset = Pose((1.0, 2.0, 0.0)), offset_pose([0.01, -0.005, 0.025, 0.0, 0.0, 15.0])
    offsets = [0.2, -0.1, 0.15, 0.0, 0.3, -0.5]
    before = dict(JOINTS)
    marker = marker_pose(urdf, footprint, offset_joints(JOINTS, "ur_arm", offsets), "ur_arm", offset)
    turned = {name: value + np.radians(o) for (name, value), o in zip(JOINTS.items(), offsets)}
    assert marker == compose(link_pose(urdf, footprint, turned, "ur_arm_tool0"), offset)
    assert JOINTS == before
    true = marker_pose(urdf, footprint, JOINTS, "ur_arm", offset)
    moved = np.linalg.norm(np.subtract(marker.position, true.position))
    assert 1e-3 < moved < 2e-2


def test_dropouts_lose_samples_at_the_given_rate():
    """Single-sample losses come at about the given rate, and none without a chance."""
    rng = np.random.default_rng(11)
    gone = [lost(rng, 0.05, 0.0, i / 120.0, -np.inf)[0] for i in range(20000)]
    assert np.mean(gone) == pytest.approx(0.05, abs=0.005)


def test_a_dropout_burst_lasts_its_seconds():
    """Once a loss starts, the body stays lost for the burst at any chance, then comes back."""
    rng = np.random.default_rng(5)
    until, states = -np.inf, []
    for i in range(1200):
        gone, until = lost(rng, 1.0 if i == 0 else 0.0, 0.25, i / 120.0, until)
        states.append(gone)
    assert sum(states) == 30 and all(states[:30])
