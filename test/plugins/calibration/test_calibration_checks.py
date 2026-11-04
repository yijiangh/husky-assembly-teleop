"""The calibration's checks and sampling: what lets the operator go on, and what a sample records."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.plugins.calibration.analysis.fit import Circle, Take, fit_circle
from husky_assembly_teleop.plugins.calibration.robot.sampling import (SAMPLE_TICKS, STILL_TICKS, Reading, Sampler,
                                                                      mean_pose)
from husky_assembly_teleop.plugins.calibration.analysis.checks import (DRIVE_SECONDS, base_spots, frame_check,
                                                                       left_out_check, marker_checks,
                                                                       marker_error_detail, marker_error_level,
                                                                       pose_change, punch_checks,
                                                                       sample_marker_error, spot_groups, take_checks,
                                                                       take_count_check, tcp_offset_check, tool_check,
                                                                       validation_checks, validation_level, yaw_range)
from husky_assembly_teleop.robot_interface.mocap_relay import (FRAME_ERROR, FRAME_HELD, FRAME_NONE, FRAME_REFIT_PENDING,
                                                               FRAME_UNCHECKED, FRAME_UNTRUSTED, FloorCalibrationState)
from husky_assembly_teleop.shared.guide.step import fails
from husky_assembly_teleop.ui.style import HOVER_MAX
from husky_assembly_teleop.world.checks import BAD, GOOD, PENDING, WARN
from scaffolding_assembly_core.geometry import Pose, compose

pytestmark = pytest.mark.safety

#: A base calibration, and the base rigid body's pose in the world.
CALIBRATION = Pose((0.05, 0.01, -0.48), tuple(Rotation.from_euler("z", 91, degrees=True).as_quat()))
BODY = Pose((1.0, -2.0, 0.47), tuple(Rotation.from_euler("z", 30, degrees=True).as_quat()))


def test_marker_errors_need_driving_then_both_bodies_low():
    """Too little driving blocks; then each body's largest error: amber over 1 mm, red over 2 mm, once is enough.

    The same three chips from the start: the bodies pending until measured.
    """
    assert fails(marker_checks((0.5e-3, 0.5e-3), DRIVE_SECONDS / 2))
    unmeasured = marker_checks(None, 0.0)
    assert fails(unmeasured) and [c.level for c in unmeasured] == [BAD, PENDING, PENDING]
    checks = marker_checks((0.5e-3, 0.5e-3), DRIVE_SECONDS)
    assert not fails(checks) and [c.level for c in checks] == [GOOD, GOOD, GOOD]
    assert [c.label for c in checks] == [c.label for c in unmeasured] == ["drive", "base markers", "tool markers"]
    amber = marker_checks((0.5e-3, 1.5e-3), DRIVE_SECONDS)
    assert not fails(amber) and [c.level for c in amber] == [GOOD, GOOD, WARN]
    tool_high = marker_checks((0.5e-3, 3e-3), DRIVE_SECONDS)
    assert fails(tool_high) and "Refine the tool body" in tool_high[2].detail
    # * The details stay the same while the largest error and the seconds rise within a level: the hover stays readable.
    later = marker_checks((0.9e-3, 1.9e-3), DRIVE_SECONDS * 2)
    assert [c.detail for c in later] == [c.detail for c in amber]


def test_a_samples_marker_error_is_its_worst_body_and_unknown_without():
    """Red over 2 mm, amber over 1 mm, grey for a sample that stored none (older punch files)."""
    errors = {"base": 0.4e-3, "tool": 2.5e-3}
    assert sample_marker_error(errors) == 2.5e-3 and marker_error_level(2.5e-3) == BAD
    assert marker_error_level(1.5e-3) == WARN and marker_error_level(0.4e-3) == GOOD
    assert sample_marker_error(None) is None and sample_marker_error({"base": None}) is None
    assert marker_error_level(None) == PENDING and marker_error_detail(None) == "marker error unknown"
    assert marker_error_detail(errors) == "base 0.40 mm, tool 2.50 mm"


def test_expired_marker_errors_are_amber_in_their_own_step():
    """A full measurement too old to count turns drive amber, never red: Record and Validate refuse instead."""
    expired = marker_checks((0.5e-3, 0.5e-3), DRIVE_SECONDS, "measured 10:02, 75 min ago (over 60 min)")
    assert not fails(expired) and [c.level for c in expired] == [WARN, GOOD, GOOD]
    assert expired[0].detail.startswith("measured 10:02") and "Recheck" in expired[0].detail


def test_the_arm_must_carry_the_marker_model():
    """The chosen arm's tool must be calibration_marker; otherwise the check names the tools parameter to start with."""
    assert tool_check("a200-0805", {"ur_arm": "calibration_marker"}, "ur_arm").level == GOOD
    check = tool_check("a200-0806", {"left_ur_arm": "scaffolding_v3", "right_ur_arm": None}, "right_ur_arm")
    assert check.level == BAD and "['0806:scaffolding_v3,calibration_marker']" in check.detail


def test_a_take_must_fit_its_circle_and_sweep_one_way():
    """A clean circle passes; a noisy one, or one swept back and forth, is amber (left out of Solve), never red."""
    angles = np.radians(np.arange(0, 360, 12))
    circle = np.column_stack([0.5 * np.cos(angles), 0.5 * np.sin(angles), np.zeros_like(angles)])
    assert {c.level for c in take_checks("t", fit_circle(Take(0, circle, angles)))} == {GOOD}
    noisy = circle + np.random.default_rng(0).normal(0, 2e-3, circle.shape)
    assert {c.level for c in take_checks("t", fit_circle(Take(0, noisy, angles)))} == {WARN}
    shuffled = np.random.default_rng(1).permutation(len(angles))
    assert WARN in {c.level for c in take_checks("t", fit_circle(Take(0, circle[shuffled], angles)))}


def test_the_takes_left_out_are_named_in_short():
    """The left out chip names each take left out and why in a few words, the first two and how many more."""
    angles = np.radians(np.arange(0, 360, 12))
    circle = np.column_stack([0.5 * np.cos(angles), 0.5 * np.sin(angles), np.zeros_like(angles)])
    noisy = fit_circle(Take(0, circle + np.random.default_rng(0).normal(0, 2e-3, circle.shape), angles))
    check = left_out_check([(1, fit_circle(Take(0, circle, angles)))] + [(number, noisy) for number in range(2, 32)])
    assert check.value == "30" and len(check.detail) <= HOVER_MAX
    assert check.detail.startswith("to record again: take 2 (fit ") and " +28; limits 1.5 mm off" in check.detail
    # * Far off and unswept takes with long numbers: the hover is cut at a word and points at the take chips.
    wild = Circle(np.zeros(3), np.array([0.0, 0.0, 1.0]), 0.5, np.array([12.0]), 0.1)
    cut = left_out_check([(number, wild) for number in range(1000, 1012)])
    assert cut.value == "12" and cut.detail.endswith("… full text in the take chips")


def test_two_takes_per_joint_are_enough_and_four_wanted():
    """Under two takes for a joint is red, two or three amber, four or more green."""
    assert [take_count_check(counts, "clear").level for counts in ((1, 6), (2, 6), (6, 3), (4, 4))] == \
        [BAD, WARN, WARN, GOOD]


def test_validation_samples_are_judged_as_the_position_check():
    """Within 2 mm is green, within 5 mm amber, beyond red."""
    assert [validation_level(d) for d in (1e-3, 3e-3, 6e-3)] == [GOOD, WARN, BAD]


def test_a_pose_change_reads_in_millimetres_and_degrees():
    """Two millimetres and half a degree apart read as such."""
    turn = tuple(Rotation.from_euler("z", 0.5, degrees=True).as_quat())
    solved = compose(CALIBRATION, Pose((0.002, 0.0, 0.0), turn))
    assert pose_change(CALIBRATION, solved) == pytest.approx((0.002, 0.5))


def _at(x: float, yaw: float) -> Pose:
    return Pose((x, 0.0, 0.47), tuple(Rotation.from_euler("z", yaw, degrees=True).as_quat()))


#: Three base spots, six samples each: far apart and turned.
SPOTS = [_at(x, yaw) for x, yaw in ((0.0, 0.0), (2.0, 120.0), (0.0, -120.0)) for _ in range(6)]


def test_spots_and_headings_are_counted():
    """A spot is new only if far enough or turned enough from every earlier one; headings wrap around."""
    assert base_spots(SPOTS) == 3
    assert base_spots([_at(0.0, 0.0), _at(0.2, 10.0), _at(0.0, 45.0)]) == 2
    assert yaw_range([_at(0.0, 170.0), _at(0.0, -170.0)]) == pytest.approx(20.0)
    assert yaw_range(SPOTS) == pytest.approx(240.0)


def test_validation_needs_samples_at_spots_and_judges_the_spread():
    """18 samples at 3 spots within 2 mm pass; few samples or one spot block; 2–5 mm or a tilted axis only warn."""
    good = validation_checks(np.full(18, 1e-3), np.full((18, 3), 0.1), SPOTS)
    assert [c.level for c in good] == [GOOD, GOOD, GOOD, GOOD]
    assert fails(validation_checks(np.full(6, 1e-3), np.full((6, 3), 0.1), SPOTS[:6]))
    assert fails(validation_checks(np.full(18, 1e-3), np.zeros((18, 3)), [SPOTS[0]] * 18))
    loose = validation_checks(np.full(18, 3e-3), np.full((18, 3), 0.5), SPOTS)
    assert not fails(loose) and [c.level for c in loose[2:]] == [WARN, WARN]
    assert fails(validation_checks(np.full(18, 6e-3), np.zeros((18, 3)), SPOTS))


def test_punch_checks_the_offset_the_takes_and_the_tip_spread():
    """The offset must be a plausible length; 4 takes are needed; the tips' spread and headings judged from two."""
    offset = np.array([0.0006, 0.0016, 0.1105])
    assert tcp_offset_check(offset).level == GOOD and tcp_offset_check(offset * 10).level == BAD
    tips = np.array([[1.0, 2.0, 0.5]] * 4) + np.array([[0, 0, 0], [0.001, 0, 0], [0, 0.001, 0], [0, 0, 0.001]])
    bases = [_at(0.0, yaw) for yaw in (0.0, 45.0, 90.0, 135.0)]
    levels = {c.label: c.level for c in punch_checks(tips, bases)}
    assert levels == {"takes": GOOD, "tip spread": GOOD, "headings": GOOD}
    none = punch_checks(tips[:0], [])
    assert list(levels) == [c.label for c in none] and [c.level for c in none] == [BAD, PENDING, PENDING]
    assert fails(punch_checks(tips[:2], bases[:2]))
    far = tips + np.array([[0.01, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]])
    assert {c.label: c.level for c in punch_checks(far, bases)}["tip spread"] == BAD


def _reading(tool_offset: float = 0.0) -> Reading:
    footprint = compose(BODY, CALIBRATION)
    tool = compose(footprint, Pose((0.4 + tool_offset, 0.1, 0.9)))
    return Reading(footprint, tool, np.arange(6.0), 1e-3, 2e-3)


def test_sampler_waits_for_stillness_then_averages():
    """Moving readings are not still; once still, the sample averages them."""
    sampler = Sampler()
    for i in range(STILL_TICKS):
        sampler.add(_reading(0.001 * i))
    assert not sampler.still()
    sampler.clear()
    for _ in range(SAMPLE_TICKS):
        sampler.add(_reading())
    assert sampler.still()
    sample = sampler.sample(CALIBRATION, "abc", 5.0)
    assert sample.spread == pytest.approx(0.0, abs=1e-9) and sample.joints == pytest.approx(np.arange(6.0))


def test_a_sample_records_the_base_rigid_body_without_the_calibration():
    """The record's base pose is the rigid body's own, so the old scripts read it as they always did."""
    sampler = Sampler()
    sampler.add(_reading())
    record = sampler.sample(CALIBRATION, "abc", 5.0).record()
    assert record["base_mocap_pose"][0] == pytest.approx(BODY.position)
    assert np.abs(np.dot(record["base_mocap_pose"][1], BODY.orientation)) == pytest.approx(1.0)
    assert record["applied_calibration"]["sha256"] == "abc" and len(record["joint_conf"]) == 6


def test_mean_pose_averages_positions_and_rotations():
    """The mean of two poses a degree apart in yaw sits half a degree from each."""
    a = Pose((0.0, 0.0, 0.0), tuple(Rotation.from_euler("z", 0, degrees=True).as_quat()))
    b = Pose((0.002, 0.0, 0.0), tuple(Rotation.from_euler("z", 1, degrees=True).as_quat()))
    mean = mean_pose([a, b])
    assert mean.position == pytest.approx((0.001, 0.0, 0.0))
    assert Rotation.from_quat(mean.orientation).as_euler("xyz", degrees=True)[2] == pytest.approx(0.5)


def test_the_punch_needs_a_bare_arm_model():
    """With the punch on, the arm must be modelled bare; the marker model is red and names the tools to start with."""
    assert tool_check("a200-0805", {"ur_arm": None}, "ur_arm", wanted=None).level == GOOD
    check = tool_check("a200-0805", {"ur_arm": "calibration_marker"}, "ur_arm", wanted=None)
    assert check.level == BAD and "['0805:none']" in check.detail


@pytest.mark.parametrize("status, level", [(FRAME_HELD, GOOD), (FRAME_UNCHECKED, WARN), (FRAME_REFIT_PENDING, WARN),
                                           (FRAME_UNTRUSTED, BAD), (FRAME_ERROR, BAD), (FRAME_NONE, PENDING)])
def test_the_frame_chip_follows_the_relays_frame_state(status, level):
    """Held is green; unchecked or a refit pending amber; untrusted or error red; none or not announced grey.

    Its hover names the missing anchors, at most three.
    """
    check = frame_check(FloorCalibrationState(status, "floor.json", pending="3.1 mm off", error="no file"), ("A2",))
    assert (check.label, check.level, check.value) == ("frame", level, status)
    assert check.detail.endswith("anchors missing: A2")
    assert frame_check(None).level == PENDING and "announced no floor calibration" in frame_check(None).detail
    many = frame_check(FloorCalibrationState(FRAME_HELD, "floor.json"), [f"A{n}" for n in range(1, 9)])
    assert many.detail.endswith("anchors missing: A1, A2, A3 +5")
    held = frame_check(FloorCalibrationState(FRAME_HELD, "floor.json", refit="2.1 mm", decides="anchors"))
    assert held.detail == "held by the anchors: floor.json, refit used: 2.1 mm"


def test_samples_are_grouped_by_spot_also_when_the_base_drives_back_to_one():
    """Two spots 1 m apart: a sample back at the first joins it; a small move stays in the spot."""
    here, there = Pose((0.0, 0.0, 0.0)), Pose((1.0, 0.0, 0.0))
    nudged = Pose((0.05, 0.0, 0.0))
    assert spot_groups([here, there, nudged, there]) == [[0, 2], [1, 3]] and base_spots([here, there, nudged]) == 2
