"""Motive in a simulation: what the fake Motive makes of a body's true pose, and how the relay takes it back."""

from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
from robot_fakes import Log, Publisher
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.mocap_relay.node import apply_base_calibration, remove_base_calibration
from husky_assembly_teleop.mocap_relay.sim_motive import (LAB_CAMERAS, MARKER_COUNT, MARKER_SPACING, REFINE_MOVE,
                                                          SimMotive, drifted, fake_definition, lost, noisy, observed,
                                                          offset_pose)
from husky_assembly_teleop.robot_interface.mocap_definitions import compare, read_cameras, read_definitions
from husky_assembly_teleop.world.mocap_frame import fit_cameras
from husky_assembly_teleop.robot_interface.simulated_parts import body_values
from husky_assembly_teleop.world.calibration_sessions import Calibration
from scaffolding_assembly_core.geometry import Pose, compose, invert

pytestmark = pytest.mark.safety

#: A pose that is not the identity.
POSE = offset_pose([1.2, -0.4, 0.3, 10.0, -20.0, 75.0])


def angle(a: Pose, b: Pose) -> float:
    """The angle between two poses' orientations, degrees."""
    return float(np.degrees((Rotation.from_quat(a.orientation).inv() * Rotation.from_quat(b.orientation)).magnitude()))


def calibration(pose: Pose) -> Calibration:
    """Belle's base calibration as `pose`."""
    return Calibration(1850, "a200-0805", pose.position, pose.orientation, "test", "", ())


def test_an_offset_base_is_found_by_its_true_calibration():
    """With the base off by an error, the default moved by that error gives the true footprint back."""
    footprint = Pose((1.7, 1.35, 0.0), tuple(Rotation.from_euler("z", 90, degrees=True).as_quat()))
    default, error = offset_pose([0.056, 0.011, -0.479, 0.3, -0.2, 90.4]), [0.004, -0.003, 0.002, 0.0, 0.0, 0.5]
    published = observed(footprint, error, [0.0] * 6, 0.0, np.random.default_rng(0), [0.0, 0.0])
    body = remove_base_calibration(calibration(default), published.position, published.orientation)
    position, quaternion = apply_base_calibration(calibration(compose(default, offset_pose(error))), *body)
    np.testing.assert_allclose(position, footprint.position, atol=1e-12)
    assert angle(Pose(position, quaternion), footprint) < 1e-7


def test_zero_settings_report_the_true_pose_exactly():
    """Without offset, noise or drift a body is reported exactly where it is."""
    rng = np.random.default_rng(0)
    assert observed(POSE, [0.0] * 6, [0.0] * 6, 12.0, rng, [0.0, 0.0]) is POSE
    assert lost(rng, 0.0, 0.0, 5.0, -np.inf) == (False, -np.inf)


def test_an_offset_puts_the_true_pose_where_a_calibration_would():
    """The offset is the true pose in Motive's body frame: the body composed with it is the truth again."""
    offset = [0.01, -0.005, 0.025, 0.0, 0.0, 15.0]
    body = observed(POSE, offset, [0.0] * 6, 0.0, np.random.default_rng(0), [0.0, 0.0])
    again = compose(body, offset_pose(offset))
    np.testing.assert_allclose(again.position, POSE.position, atol=1e-12)
    assert angle(again, POSE) < 1e-7 and angle(body, POSE) == pytest.approx(15.0)


@pytest.mark.parametrize("text, body, values", [
    ("", 7, [0.0, 0.0]),
    ("0.1,0.2", 7, [0.1, 0.2]),
    ("0.1,0.2 7:0.3,0.4", 7, [0.3, 0.4]),
    ("7:0,0 0.1,0.2", 7, [0.0, 0.0]),
    ("0.1,0.2 7:0.3,0.4", 8, [0.1, 0.2]),
    ("8:0.3,0.4", 7, [0.0, 0.0]),
])
def test_per_body_values_take_a_bodys_own_entry_over_the_one_for_all(text, body, values):
    """Values for every body, `<id>:<values>` for one; a body's own entry wins, also when it is zero."""
    assert body_values(text, body, 2) == values


def test_per_body_values_need_their_count():
    """An entry with too few or too many numbers is refused."""
    with pytest.raises(ValueError, match="2 numbers expected"):
        body_values("7:0.1", 7, 2)


@pytest.mark.slow
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


class Elapsed:
    """A ROS time that is always one second after the start."""

    def __sub__(self, _other):
        """One second."""
        return SimpleNamespace(nanoseconds=1_000_000_000)


def test_a_body_reaches_the_relay_after_the_delay_stamped_with_when_it_was_true():
    """No delay sends each body at once; Motive's latency holds it back; the stamp stays the truth's either way."""
    for delay, sent in ((0.0, 1), (10.0, 0)):
        node = SimMotive.__new__(SimMotive)
        node._delay, node._dropout, node._dropout_burst_s = delay, 0.0, 0.0
        node._rng, node._start, node._seen_from = np.random.default_rng(0), SimpleNamespace(), Pose()
        node.get_clock = lambda: SimpleNamespace(now=lambda: Elapsed())
        stamp, publisher = Time(sec=12, nanosec=34), Publisher()
        node._bodies = {1850: {"truth": POSE, "stamp": stamp, "offset": [0.0] * 6, "drift": [0.0] * 6,
                               "noise": [0.0, 0.0], "marker_error": 0.0003, "held": None, "lost_until": -np.inf,
                               "queue": deque(), "publisher": publisher}}
        node._publish()
        assert len(publisher.sent) == sent
        if sent:
            assert publisher.sent[0].header.stamp == stamp and publisher.sent[0].pose_valid


# * What the fake Motive's model descriptions give the relay: body definitions and the lab's cameras.

def test_a_faked_body_has_the_same_spread_out_markers_every_time_and_others_differ():
    """Seeded by its id: five markers at least 2 cm apart; another id another body."""
    first = fake_definition(1850)
    assert fake_definition(1850) == first and len(first.offsets) == MARKER_COUNT
    points = first.array()
    assert np.linalg.norm(points[:, None] - points[None, :], axis=2)[np.triu_indices(5, 1)].min() >= MARKER_SPACING
    assert compare(first, fake_definition(1866)).kind == "other body"


@pytest.mark.parametrize("refines", [1, 2])
def test_a_refine_nudges_the_markers_as_motives_refine_does(refines):
    """Refined, a data change; each refine its own nudge; none gives the first definition back."""
    change = compare(fake_definition(1850), fake_definition(1850, refines))
    assert change.kind == "refined" and change.data_change and change.markers_moved < 2 * REFINE_MOVE
    assert fake_definition(1850, 1) != fake_definition(1850, 2) and fake_definition(1850, 0) == fake_definition(1850)


def test_the_lab_cameras_file_holds_its_ten_cameras():
    """As the relay announces them, so a fake Motive can announce them as they are."""
    cameras = read_cameras(LAB_CAMERAS.read_text())
    assert len(cameras) == 10 and "PrimeX 22 #72318" in cameras


def faking(bodies) -> SimMotive:
    """A fake Motive without ROS faking these rigid bodies."""
    node = SimMotive.__new__(SimMotive)
    node._bodies, node._refines, node._definitions_publisher = {body: {} for body in bodies}, "", Publisher()
    node._cameras, node._seen_from, node._cameras_publisher = read_cameras(LAB_CAMERAS.read_text()), Pose(), Publisher()
    node.get_logger = lambda: Log()
    return node


def test_refines_set_while_running_announce_the_refined_definitions_at_once():
    """`ros2 param set /sim_motive refines "1850:1"`: 1850 refined, 1866 as it was."""
    node = faking([1850, 1866])
    result = node._on_parameters([SimpleNamespace(name="refines", value="1850:1")])
    announced = read_definitions(node._definitions_publisher.sent[-1].data)
    assert result.successful and announced == {1850: fake_definition(1850, 1), 1866: fake_definition(1866)}


def test_refines_that_do_not_read_are_refused():
    """Nothing announced, and why."""
    node = faking([1850])
    result = node._on_parameters([SimpleNamespace(name="refines", value="1850:a")])
    assert not result.successful and node._definitions_publisher.sent == []


def test_a_recalibration_moves_every_camera_as_the_relays_fit_recovers_it():
    """`ros2 param set /sim_motive recalibration "0.02,-0.01,0,0,0,0.5"`: the truss cameras give that motion back."""
    node = faking([1850])
    result = node._on_parameters([SimpleNamespace(name="recalibration", value="0.02,-0.01,0,0,0,0.5")])
    fit = fit_cameras(read_cameras(LAB_CAMERAS.read_text()), read_cameras(node._cameras_publisher.sent[-1].data))
    assert result.successful and fit.trusted and not fit.bumped
    np.testing.assert_allclose(fit.raw_cal_from_raw_now.position, (0.02, -0.01, 0.0), atol=1e-6)
    assert np.degrees(fit.moved[1]) == pytest.approx(0.5, abs=1e-6)
    assert not node._on_parameters([SimpleNamespace(name="recalibration", value="1,2")]).successful
