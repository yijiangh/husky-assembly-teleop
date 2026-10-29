"""Checks for the bar pose from MoCap markers: the matcher, the layouts from the process, and the servo's bar math.

Most checks build markers in memory, on a made-up T20 bar and on the real B20 table, both in the process frame
(origin at the bar's MIDDLE). The checks on the purged process, on the old 2ModulesDemo export and on the Oct-1 take
of the old marker rig need the Drive and are skipped without it.
"""

from __future__ import annotations

import json
from dataclasses import replace
from pathlib import Path
from time import perf_counter
from types import SimpleNamespace

import numpy as np
import pytest

from husky_assembly_teleop.drive import drive_root
from husky_assembly_teleop.robot_interface.mocap_state import RawMarkers
from husky_assembly_teleop.world.bar_fit import match_and_fit
from husky_assembly_teleop.world.bar_pose import (
    apply_authored_move, bar_ends, bar_servo_correction, bar_slide_roll, bar_start, kabsch, pose_deviation,
    transform_points)
from husky_assembly_teleop.world.marker_layouts import (
    BarLayout, Sensed, bar_asset_id, bar_layouts, fake_sensed_markers, fit_bar, sensed_from_raw)
from planning_fixtures import purged_process
from process_fixtures import build_process

#: Length of B20 and of the made-up T20 bar, metres: the process frame sits L/2 up the bar from the old origin.
BAR_LENGTH = 1.8

#: The real B20 of the 2ModulesDemo: four T20_MoCap halves, two balls each, ball centres in the process bar frame
#: (metres), in `bar_layouts` order (by half id, then label).
B20_MARKERS = (
    ("joints/J20-169_mocap/M1", (0.02727, 0.02214, 0.2453)),
    ("joints/J20-169_mocap/M2", (0.02358, -0.02602, 0.2453)),
    ("joints/J20-21_mocap/M1", (-0.0255, 0.02415, -0.48)),
    ("joints/J20-21_mocap/M2", (-0.0255, -0.02415, -0.48)),
    ("joints/J20-22_mocap/M1", (-0.02735, -0.02203, -0.8151)),
    ("joints/J20-22_mocap/M2", (-0.02348, 0.02612, -0.8151)),
    ("joints/J20-25_mocap/M1", (-0.0255, -0.02415, 0.78)),
    ("joints/J20-25_mocap/M2", (-0.0255, 0.02415, 0.78)),
)

# The T20_MoCap plate: its two balls in the joint block frame (metres).
T20_BLOCK_M1 = (-0.010, 0.02415, 0.0255)
T20_BLOCK_M2 = (-0.010, -0.02415, 0.0255)

#: Under the Drive root: the old Rhino export B20's layout came from, and an Oct-1 take of the OLD 4-marker rig.
OLD_EXPORT = "data_design_study/261016_2ModulesDemo_YH/MoCapMarkers.json"
OLD_RIG_TAKE = "data_experiment/bar_holding_acc_data/20261001/bar_holding_acc_20261001_1514.json"

#: A sample's ROS time in these checks, seconds.
NOW = 10.0


# * ------------------------------------------------- fixtures and helpers

def drive_file(relative: str) -> Path:
    """A file under the Drive root; skips the test without it."""
    root = drive_root()
    if root is None or not (root / relative).is_file():
        pytest.skip(f"needs HUSKY_DRIVE_ROOT with {relative}")
    return root / relative


def t20_design(joint_z: tuple = (0.075, 0.430, 1.135, 1.670), rolls_deg: tuple = (0, 90, 180, 45)) -> np.ndarray:
    """A made-up T20 bar: one T20_MoCap plate per joint, joint frame = slide along z, then roll.

    Args:
        joint_z: Joint positions along the bar from its START, metres.
        rolls_deg: Joint rolls about the bar, degrees.

    Returns:
        np.ndarray: (2 * joints, 3) ball centres in the process bar frame, ``[J1 M1, J1 M2, J2 M1, ...]``.
    """
    block = np.array([T20_BLOCK_M1, T20_BLOCK_M2])
    return np.vstack([transform_points(bar_slide_roll(z - BAR_LENGTH / 2.0, roll), block)
                      for z, roll in zip(joint_z, rolls_deg)])


def b20_design() -> np.ndarray:
    """The real B20 ball centres in the process bar frame, (8, 3) metres."""
    return np.array([point for _name, point in B20_MARKERS])


def b20_layout() -> BarLayout:
    """The real B20 as `bar_layouts` builds it, without the Drive."""
    return BarLayout("bars/B20", tuple(name for name, _point in B20_MARKERS), b20_design(), BAR_LENGTH)


def random_rigid(seed: int) -> np.ndarray:
    """A random bar pose somewhere in a 6 m room, as a 4x4 proper rigid transform."""
    rng = np.random.default_rng(seed)
    q, _r = np.linalg.qr(rng.normal(size=(3, 3)))
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1.0
    T = np.eye(4)
    T[:3, :3] = q
    T[:3, 3] = rng.uniform(-3.0, 3.0, 3)
    return T


def small_rigid(offset_mm: float, angle_deg: float, seed: int) -> np.ndarray:
    """A small rigid move: ``offset_mm`` along a random direction, ``angle_deg`` about a random axis."""
    rng = np.random.default_rng(seed)
    axis, direction = rng.normal(size=3), rng.normal(size=3)
    axis, direction = axis / np.linalg.norm(axis), direction / np.linalg.norm(direction)
    # Rodrigues: R = I + sin(a) K + (1 - cos(a)) K^2, K the cross-product matrix of the axis.
    K = np.array([[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]])
    angle = np.radians(angle_deg)
    T = np.eye(4)
    T[:3, :3] = np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * K @ K
    T[:3, 3] = direction * offset_mm / 1000.0
    return T


def fit(sensed: np.ndarray, design: np.ndarray, **kwargs):
    """`match_and_fit`, plus the check every fit must pass: never a mirror."""
    result = match_and_fit(sensed, design, **kwargs)
    if result.T_world_from_bar is not None:
        assert np.linalg.det(result.T_world_from_bar[:3, :3]) > 0
    return result


def raw_markers(points: np.ndarray, *, measured=None, residuals=0.0008, sizes=0.0188, capture_time=NOW):
    """A relay sample of these points with ids ``1..N``, shaped like `RawMarkers` with M7's extra fields.

    Built as a namespace, so the checks hold with or without residuals, sizes and a capture time on the class.

    Args:
        points: (N, 3) marker positions in world, metres.
        measured: Per marker, or None for all measured.
        residuals: Per marker or one for all, metres; None = the relay sent none.
        sizes: Per marker or one for all, metres; None = the relay sent none.
        capture_time: ROS time of the sample, seconds; None = the relay sent none.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    n = len(points)
    return SimpleNamespace(ids=np.arange(1, n + 1), positions=points,
                           measured=np.ones(n, dtype=bool) if measured is None else np.asarray(measured, dtype=bool),
                           residuals=None if residuals is None else np.broadcast_to(residuals, n).astype(float),
                           sizes=None if sizes is None else np.broadcast_to(sizes, n).astype(float),
                           capture_time=capture_time)


def sensed_of(points: np.ndarray, **fields) -> Sensed:
    """These points as a sample from the relay, every marker measured and trusted unless `fields` say otherwise."""
    return sensed_from_raw(raw_markers(points, **fields), NOW)


@pytest.fixture(params=["t20", "B20"])
def design(request) -> np.ndarray:
    """The made-up T20 bar and the real B20 bar, for the checks run on both."""
    return t20_design() if request.param == "t20" else b20_design()


# * ------------------------------------------------- matching (B5 cases 1-9, 13)

def test_exact_recovery(design):
    """Case 1 (and 13 on B20): shuffled exact markers give back the pose and the shuffle."""
    T_true = random_rigid(1)
    perm = np.random.default_rng(1).permutation(len(design))
    result = fit(transform_points(T_true, design)[perm], design)
    assert result.ok, result.reason
    assert result.n_matched == len(design)
    assert np.linalg.norm(result.T_world_from_bar - T_true) < 1e-9
    # Sensed row k is design row perm[k].
    assert result.correspondences == {k: int(d) for k, d in enumerate(perm)}
    assert result.unmatched_sensed == [] and result.missing_design == []
    assert result.rms_m < 1e-9


def test_noise_one_mm():
    """Case 2: 1 mm Gaussian noise on every coordinate still fits, at the noise level.

    ! The roll about the bar is the weak direction: the balls sit only ~26 mm off the axis, so 1 mm of noise moves
      the roll by ~0.8 deg (one sigma). The tilt of the bar axis (balls spread over 1.6 m) stays far below 0.2 deg.
    """
    design = t20_design()
    T_true = random_rigid(2)
    sensed = transform_points(T_true, design) + np.random.default_rng(2).normal(0.0, 0.001, design.shape)
    result = fit(sensed, design)
    assert result.ok, result.reason
    assert result.n_matched >= 7
    assert result.rms_m < 2.5e-3
    dev = pose_deviation(result.T_world_from_bar, T_true)
    assert dev.dpos_mm < 2.5
    assert np.hypot(*dev.rotvec_bar_deg[:2]) < 0.2  # tilt of the bar axis
    assert abs(dev.rotvec_bar_deg[2]) < 3.0  # roll about the bar


def test_one_marker_dropped(design):
    """Case 3 (and 13 on B20): one occluded ball -> 7 of 8, still ok."""
    sensed = np.delete(transform_points(random_rigid(3), design), 3, axis=0)
    result = fit(sensed, design)
    assert result.ok, result.reason
    assert result.n_matched == 7
    assert result.missing_design == [3]


def test_ghost_marker(design):
    """Case 4 (and 13 on B20): a stray marker 0.3 m away ends up unmatched."""
    T_true = random_rigid(4)
    moved = transform_points(T_true, design)
    ghost = moved.mean(axis=0) + 0.3 * T_true[:3, 0]
    result = fit(np.vstack([moved, ghost]), design)
    assert result.ok, result.reason
    assert result.n_matched == len(design)
    assert result.unmatched_sensed == [len(design)]


def test_three_markers_dropped():
    """Case 5: three occluded balls -> 5 of 8 is not enough."""
    design = t20_design()
    sensed = np.delete(transform_points(random_rigid(5), design), [0, 3, 6], axis=0)
    result = fit(sensed, design)
    assert result.ok is False
    assert "5 of 8" in result.reason


def test_mirror_is_refused():
    """Case 6: a mirrored marker set (e.g. a wrong axis convention) does not fit."""
    design = t20_design()
    result = fit(design * np.array([1.0, 1.0, -1.0]), design)
    assert result.ok is False
    assert np.linalg.det(result.T_world_from_bar[:3, :3]) > 0


def test_symmetric_layout_is_ambiguous():
    """Case 7: joints placed symmetric about the middle, all facing the same way."""
    design = t20_design(joint_z=(0.1, 0.5, 1.3, 1.7), rolls_deg=(0, 0, 0, 0))
    result = fit(transform_points(random_rigid(7), design), design)
    assert result.ambiguous is True
    assert result.second_best["n_matched"] == 8
    assert result.ok is False
    assert "equally well" in result.reason


def test_m1_m2_swap_resolved(design):
    """Case 8 (and 13 on B20): the M1/M2-swapped guesses lose on the full-set score."""
    T_true = random_rigid(8)
    result = fit(transform_points(T_true, design), design)
    assert result.ok, result.reason
    assert result.correspondences == {k: k for k in range(len(design))}
    # Some other assignment was tried and scored lower.
    assert result.second_best is not None
    assert result.second_best["n_matched"] < result.n_matched


def test_collinear_design(tmp_path):
    """Case 9: balls on one line -> the fit says why it fails, and the bar's layout from the process warns."""
    collinear = np.array([[0.0255, 0.0, z] for z in (-0.8, -0.4, 0.3, 0.8)])
    result = fit(transform_points(random_rigid(9), collinear), collinear)
    assert result.ok is False
    assert "collinear" in result.reason

    # * The same balls on B1's female half (0.85 m up the bar): the layout carries the warning.
    process = build_process(tmp_path)
    female = replace(process.bodies["joints/J1_female"],
                     markers={f"m{k}": (0.0255, 0.0, z) for k, z in enumerate((0.0, 0.1, 0.2, 0.3))})
    process = replace(process, bodies={**process.bodies, female.id: female})
    layout = bar_layouts(process)["bars/B1"]
    assert len(layout.warnings) == 1
    assert "bars/B1" in layout.warnings[0] and "collinear" in layout.warnings[0]
    assert fit_bar(sensed_of(transform_points(random_rigid(9), layout.points)), layout).ok is False


def test_two_bars_in_view():
    """A second bar's markers do not fit this bar's joint spacing and stay unmatched."""
    design = b20_design()
    other = t20_design(joint_z=(0.2, 0.6, 0.9, 1.5))
    sensed = np.vstack([transform_points(random_rigid(10), design), transform_points(random_rigid(11), other)])
    result = fit(sensed, design)
    assert result.ok, result.reason
    assert result.correspondences == {k: k for k in range(8)}
    assert result.unmatched_sensed == list(range(8, 16))


def test_too_few_or_too_many_markers():
    """Fewer than 3 or more than ``max_sensed`` markers fail with a plain reason."""
    design = t20_design()
    assert "only 2 markers" in fit(design[:2], design).reason
    many = np.vstack([design + [0.0, 0.0, 2.0 * k] for k in range(4)])
    assert "too many markers" in fit(many, design).reason


def test_kabsch_never_mirrors():
    """Kabsch on a mirrored set returns a proper rotation, on a clean set the exact pose."""
    pts = t20_design()
    T_true = random_rigid(12)
    assert np.allclose(kabsch(pts, transform_points(T_true, pts)), T_true)
    assert np.linalg.det(kabsch(pts, pts * [1.0, 1.0, -1.0])[:3, :3]) > 0


# * ------------------------------------------------- the layouts from the process (case 10, 16)

def test_bar_layouts_from_the_purged_process():
    """The purged process gives B20, B21, B34 and B35 with 8 named markers each; B20 is 1.8 m and the table above."""
    layouts = bar_layouts(purged_process().process)
    assert {bar: len(layout.names) for bar, layout in layouts.items()} == {
        "bars/B20": 8, "bars/B21": 8, "bars/B34": 8, "bars/B35": 8}
    assert {bar: bar_asset_id(bar) for bar in layouts} == {
        "bars/B20": 1020, "bars/B21": 1021, "bars/B34": 1034, "bars/B35": 1035}
    b20 = layouts["bars/B20"]
    assert b20.bar == "bars/B20" and b20.length == BAR_LENGTH and b20.warnings == ()
    assert b20.names == b20_layout().names
    assert np.abs(b20.points - b20_design()).max() < 5e-5


def test_bar_layouts_match_the_old_export():
    """Case 16: the process B20 layout is the old ``MoCapMarkers.json`` B20 table shifted by L/2 (< 0.05 mm)."""
    with open(drive_file(OLD_EXPORT), "r") as f:
        old = json.load(f)
    assert old["schema_version"] == 2 and old["units"] == "m"
    table = {marker["name"]: np.array(marker["bar_m"]) for marker in old["bars"]["bar_B20"]["markers"]}
    assert len(table) == 8

    b20 = bar_layouts(purged_process().process)["bars/B20"]
    for name, point in zip(b20.names, b20.points):
        # "joints/J20-21_mocap/M1" was "J20-21_mocap_M1" in the export.
        half, label = name.rsplit("/", 1)
        old_point = table[f"{half.split('/')[-1]}_{label}"] - [0.0, 0.0, b20.length / 2.0]
        assert np.abs(old_point - point).max() < 5e-5, name


def test_bars_with_too_few_markers_are_left_out(tmp_path):
    """A bar whose halves carry fewer than 3 markers has no layout; with 3 it has, with its cylinder's length."""
    process = build_process(tmp_path)
    assert bar_layouts(process) == {}
    female = replace(process.bodies["joints/J1_female"],
                     markers={"a": (0.01, 0.0, 0.02), "b": (-0.01, 0.0, 0.02), "c": (0.0, 0.03, 0.0)})
    process = replace(process, bodies={**process.bodies, female.id: female})
    layout = bar_layouts(process)["bars/B1"]
    assert layout.names == ("joints/J1_female/a", "joints/J1_female/b", "joints/J1_female/c")
    assert layout.length == 0.9 and layout.warnings == ()


def test_bar_asset_id():
    """Bxx streams as 10xx, with or without the "bars/" prefix; anything else has no asset."""
    assert bar_asset_id("bars/B20") == 1020
    assert bar_asset_id("B7") == 1007
    assert bar_asset_id("bars/X1") is None and bar_asset_id("bars/B") is None


# * ------------------------------------------------- the sensed markers (cases 11, 14, 15)

def test_old_rig_take_does_not_fit_a_t20_bar():
    """Case 11: every take of the old 82 mm rig is refused against a T20 bar, without an exception."""
    with open(drive_file(OLD_RIG_TAKE), "r") as f:
        takes = json.load(f)["raw_data"]
    for take in takes:
        markers = list(take["bar_rig"].values())
        sample = sensed_of([marker["pos"] for marker in markers],
                           residuals=[marker["error"] for marker in markers],
                           sizes=[np.ravel(marker["size"])[0] for marker in markers])
        for layout in (BarLayout("bars/T20", (), t20_design(), BAR_LENGTH), b20_layout()):
            result = fit_bar(sample, layout)
            assert result.ok is False, result.reason


def test_fake_markers_round_trip():
    """Case 14: the fake sample fits 30 mm along and 2 deg about the bar, the stray unmatched."""
    layout = b20_layout()
    T_design = random_rigid(14)
    # With the default 0.5 mm noise the roll about the bar wanders ~0.3 deg (one sigma).
    for noise_m, tol_mm, tol_deg in ((0.0, 1e-6, 1e-6), (0.0005, 1.5, 1.5)):
        sample = fake_sensed_markers(T_design, layout.points, noise_m=noise_m)
        assert sample.source == "fake" and sorted(sample.ids) == list(range(1, 10))
        result = fit_bar(sample, layout)
        assert result.ok, result.reason
        assert result.bar_name == "bars/B20"
        assert result.n_matched == 8
        assert len(result.unmatched_sensed) == 1
        # The stray is the one marker far from every design ball.
        placed = transform_points(T_design, layout.points)
        assert np.linalg.norm(placed - sample.points[result.unmatched_sensed[0]], axis=1).min() > 0.2
        dev = pose_deviation(result.T_world_from_bar, T_design)
        assert dev.dpos_bar_mm == pytest.approx([0.0, 0.0, 30.0], abs=tol_mm)
        assert dev.dang_deg == pytest.approx(2.0, abs=tol_deg)
        json.dumps(result.to_json())
        json.dumps(dev.to_json())
    # Exact markers come back exactly at the documented pose.
    exact = fit_bar(fake_sensed_markers(T_design, layout.points, noise_m=0.0), layout)
    assert np.allclose(exact.T_world_from_bar, T_design @ bar_slide_roll(0.03, 2.0))


def test_sensed_from_raw_filters():
    """Case 15: trusted markers fit with their ids; a blob and a bad residual are dropped, an old sample refused."""
    layout = BarLayout("bars/T20", (), t20_design(), BAR_LENGTH)
    T_true = random_rigid(15)
    moved = transform_points(T_true, layout.points)
    result = fit_bar(sensed_of(moved), layout)
    assert result.ok, result.reason
    assert np.allclose(result.T_world_from_bar, T_true)
    assert {result.sensed_ids[s] for s in result.correspondences} == set(range(1, 9))

    # A tiny blob and a marker Motive does not trust are dropped before matching.
    sizes, residuals = np.full(8, 0.0188), np.full(8, 0.0008)
    sizes[0], residuals[1] = 0.005, 0.01
    sample = sensed_of(moved, sizes=sizes, residuals=residuals)
    assert sample.dropped == 2 and sample.occluded == 0 and sample.notes == ()
    result = fit_bar(sample, layout)
    assert result.ok, result.reason
    assert result.n_matched == 6
    assert 1 not in result.sensed_ids and 2 not in result.sensed_ids

    old = sensed_of(moved, capture_time=NOW - 2.0)
    assert old.age == pytest.approx(2.0)
    refused = fit_bar(old, layout, max_age=0.5)
    assert refused.ok is False and "old" in refused.reason and refused.bar_name == "bars/T20"
    assert fit_bar(old, layout, max_age=None).ok


def test_unmeasured_markers_are_dropped():
    """A hidden marker of a Motive asset still streams, at Motive's guess, not measured: never fitted."""
    layout = BarLayout("bars/T20", (), t20_design(), BAR_LENGTH)
    moved = transform_points(random_rigid(16), layout.points)
    measured = np.ones(8, dtype=bool)
    # One unmeasured marker (Motive's guess sits exactly on the design spot): it is still missing.
    measured[0] = False
    result = fit_bar(sensed_of(moved, measured=measured), layout)
    assert result.ok and result.n_matched == 7 and 1 not in result.sensed_ids
    # Three unmeasured: 5 of 8 left, and the reason says why.
    measured[1:3] = False
    sample = sensed_of(moved, measured=measured)
    assert sample.occluded == 3 and sample.dropped == 0
    result = fit_bar(sample, layout)
    assert result.ok is False and "5 of 8" in result.reason and "3 marker(s) occluded" in result.reason


def test_sensed_from_raw_drops_unmeasured_and_noisy_markers():
    """Unmeasured, a 3.1 mm residual and 11 or 27 mm sizes are dropped; 3.0 mm, 12 and 26 mm are kept."""
    points = transform_points(random_rigid(17), t20_design())
    measured = np.array([False, True, True, True, True, True, True, True])
    residuals = np.array([0.0008, 0.0031, 0.0008, 0.0008, 0.0008, 0.0008, 0.003, 0.0008])
    sizes = np.array([0.019, 0.019, 0.011, 0.027, 0.012, 0.026, 0.019, 0.019])
    sample = sensed_from_raw(raw_markers(points, measured=measured, residuals=residuals, sizes=sizes,
                                         capture_time=NOW - 0.1), NOW)
    assert sample.ids == (5, 6, 7, 8)
    np.testing.assert_array_equal(sample.points, points[4:])
    np.testing.assert_array_equal(sample.residuals, residuals[4:])
    assert sample.occluded == 1 and sample.dropped == 3
    assert sample.age == pytest.approx(0.1) and sample.source == "asset" and sample.notes == ()


def test_markers_without_sizes_skip_the_size_check():
    """A relay that sends no sizes: every measured marker is kept, and a failed fit's reason says so once."""
    layout = BarLayout("bars/T20", (), t20_design(), BAR_LENGTH)
    points = transform_points(random_rigid(18), layout.points)
    raw = RawMarkers(ids=np.arange(1, 9), positions=points, measured=np.ones(8, dtype=bool))
    sample = sensed_from_raw(raw, NOW)
    assert len(sample.points) == 8 and sample.dropped == 0 and sample.age == 0.0
    note = "marker sizes not relayed: size check skipped"
    assert note in sample.notes
    # An ok fit says nothing; a failed one names the skipped check once.
    assert fit_bar(sample, layout).ok and fit_bar(sample, layout).reason == ""
    failed = fit_bar(sensed_from_raw(RawMarkers(np.arange(1, 3), points[:2], np.ones(2, dtype=bool)), NOW), layout)
    assert failed.ok is False and "only 2 markers" in failed.reason
    assert failed.reason.count(note) == 1


# * ------------------------------------------------- following live: the match around the last pose

# * Since the hint (the last fit), the bar slid this far along itself and rolled this much.
TRACK_SLIDE_M = 0.015
TRACK_ROLL_DEG = 1.0
# The tracked fit for 8 markers must be quick: the live mode re-fits every bar at 10 Hz. A generous bound.
TRACK_MAX_MEDIAN_S = 0.005


def _moved_since_hint(seed: int) -> tuple:
    """A B20 pose (the hint) and the B20 markers after the bar moved a bit from it.

    Returns:
        tuple: ``(T_hint, T_now, sample)``: the old pose, the pose now (slid 15 mm, rolled 1 deg), and the markers
        at the pose now as a relay sample.
    """
    T_hint = random_rigid(seed)
    T_now = T_hint @ bar_slide_roll(TRACK_SLIDE_M, TRACK_ROLL_DEG)
    points = transform_points(T_now, b20_design())
    points = points + np.random.default_rng(seed).normal(0.0, 0.0005, points.shape)
    return T_hint, T_now, sensed_of(points)


def test_tracked_fit_matches_the_full_search():
    """A hint near the truth: the quick match is used and gives the full search's pose."""
    layout = b20_layout()
    T_hint, T_now, sample = _moved_since_hint(20)
    full = fit_bar(sample, layout)
    tracked = fit_bar(sample, layout, hint=T_hint)
    assert full.ok and not full.tracked
    assert tracked.ok and tracked.tracked, tracked.reason
    assert tracked.n_matched == 8 and tracked.correspondences == full.correspondences
    assert np.abs(tracked.T_world_from_bar - full.T_world_from_bar).max() < 1e-6
    assert pose_deviation(tracked.T_world_from_bar, T_now).dpos_mm < 1.5
    assert tracked.ambiguous is False and tracked.second_best is None and tracked.n_hypotheses == 0
    assert tracked.bar_name == "bars/B20" and tracked.reason == ""
    assert json.loads(json.dumps(tracked.to_json()))["tracked"] is True


def test_tracked_fit_falls_back_when_the_hint_is_far():
    """A hint 100 mm off: no marker within 20 mm, so the full search runs (still ok, not tracked)."""
    layout = b20_layout()
    _T_hint, T_now, sample = _moved_since_hint(21)
    far = np.eye(4)
    far[:3, 3] = [0.1, 0.0, 0.0]
    result = fit_bar(sample, layout, hint=far @ T_now)
    assert result.ok, result.reason
    assert result.tracked is False and result.n_hypotheses > 0
    assert pose_deviation(result.T_world_from_bar, T_now).dpos_mm < 1.5


def test_tracked_fit_with_an_occluded_marker():
    """A hidden ball (not measured): never fitted, and the full search runs (7 of 8, not tracked)."""
    layout = b20_layout()
    T_hint, _T_now, sample = _moved_since_hint(22)
    measured = np.ones(8, dtype=bool)
    measured[3] = False
    sample = sensed_of(sample.points, measured=measured)
    result = fit_bar(sample, layout, hint=T_hint)
    assert result.ok and not result.tracked, result.reason
    assert result.n_matched == 7 and result.missing_design == [3]
    assert 4 not in result.sensed_ids


def test_tracked_fit_never_stays_on_a_partial_match():
    """A hint on a wrong pose that matches only some balls: the full search takes over and finds the truth."""
    layout = b20_layout()
    _T_hint, T_now, sample = _moved_since_hint(24)
    # The hint turned half a turn about the bar's middle: at most part of the balls line up.
    T_wrong = T_now @ bar_slide_roll(0.0, 180.0)
    result = fit_bar(sample, layout, hint=T_wrong)
    assert result.ok and result.n_matched == 8, result.reason
    assert pose_deviation(result.T_world_from_bar, T_now).dpos_mm < 1.5


def test_tracked_fit_is_quick():
    """8 markers around a hint: the median of 20 tracked fits stays under 5 ms."""
    layout = b20_layout()
    T_hint, _T_now, sample = _moved_since_hint(23)
    seconds = []
    for _ in range(20):
        start = perf_counter()
        result = fit_bar(sample, layout, hint=T_hint)
        seconds.append(perf_counter() - start)
    assert result.tracked
    assert np.median(seconds) < TRACK_MAX_MEDIAN_S, f"median {np.median(seconds) * 1000:.2f} ms"


# * ------------------------------------------------- pose deviation (case 12), and the bar's start and ends

def test_pose_deviation():
    """Case 12: 5 mm along the bar reads (0, 0, 5) in the bar frame; a 1 deg roll reads 1 deg."""
    T_design = random_rigid(12)
    slid = pose_deviation(T_design @ bar_slide_roll(0.005, 0.0), T_design)
    assert slid.dpos_bar_mm == pytest.approx([0.0, 0.0, 5.0], abs=1e-9)
    assert slid.dpos_world_mm == pytest.approx(T_design[:3, 2] * 5.0, abs=1e-9)
    assert slid.dpos_mm == pytest.approx(5.0) and slid.dang_deg == pytest.approx(0.0, abs=1e-9)

    rolled = pose_deviation(T_design @ bar_slide_roll(0.0, 1.0), T_design)
    assert rolled.dang_deg == pytest.approx(1.0)
    assert rolled.rotvec_bar_deg == pytest.approx([0.0, 0.0, 1.0], abs=1e-9)
    assert rolled.dpos_mm == pytest.approx(0.0, abs=1e-9)

    # A bar flipped end for end (wrong z sign) reads 180 deg.
    flipped = pose_deviation(T_design @ np.diag([1.0, -1.0, -1.0, 1.0]), T_design)
    assert flipped.dang_deg == pytest.approx(180.0)
    json.dumps(flipped.to_json())


def test_bar_ends():
    """The drawn bar line runs L/2 each way from the bar's middle along its z."""
    T = random_rigid(13)
    ends = bar_ends(T, BAR_LENGTH)
    assert ends[0] == pytest.approx(T[:3, 3] - 0.9 * T[:3, 2])
    assert ends[1] == pytest.approx(T[:3, 3] + 0.9 * T[:3, 2])


def test_the_old_origin_is_the_bar_start():
    """Two process-frame poses deviate at `bar_start` exactly as the old-frame poses (origin at the start) did."""
    X_old, Y_old = random_rigid(25), random_rigid(26)
    up = np.eye(4)
    up[2, 3] = BAR_LENGTH / 2.0
    X, Y = X_old @ up, Y_old @ up  # the same bars, their frames moved to the middle
    assert bar_start(X, BAR_LENGTH) == pytest.approx(X_old, abs=1e-12)
    old, new = pose_deviation(X_old, Y_old), pose_deviation(bar_start(X, BAR_LENGTH), bar_start(Y, BAR_LENGTH))
    assert new.dpos_mm == pytest.approx(old.dpos_mm, abs=1e-9)
    assert new.dang_deg == pytest.approx(old.dang_deg, abs=1e-9)
    assert new.dpos_world_mm == pytest.approx(old.dpos_world_mm, abs=1e-9)
    assert new.dpos_bar_mm == pytest.approx(old.dpos_bar_mm, abs=1e-9)
    assert new.rotvec_bar_deg == pytest.approx(old.rotvec_bar_deg, abs=1e-9)
    # At the middle the same tilt moves the origin: the two frames do not agree there.
    assert pose_deviation(X, Y).dpos_mm != pytest.approx(old.dpos_mm, abs=1e-6)


# * ------------------------------------------------- servo on the MoCap bar

def test_bar_servo_correction_carries_the_measured_bar_onto_the_design():
    """``T_corr @ T_mocap`` is the design pose."""
    T_mocap, T_design = random_rigid(21), random_rigid(22)
    T_corr = bar_servo_correction(T_mocap, T_design)
    assert T_corr @ T_mocap == pytest.approx(T_design, abs=1e-12)


def test_apply_authored_move():
    """From the authored start it lands on the authored end; a pure translation stays one."""
    T_start, T_end = random_rigid(31), random_rigid(32)
    assert apply_authored_move(T_start, T_end, T_start) == pytest.approx(T_end, abs=1e-12)

    # The insert: 15 mm straight down in world, from a flange 3 mm / 0.1 deg off the authored start.
    down = np.eye(4)
    down[2, 3] = -0.015
    T_actual = small_rigid(3.0, 0.1, 33) @ T_start
    moved = apply_authored_move(T_start, down @ T_start, T_actual)
    assert moved[:3, :3] == pytest.approx(T_actual[:3, :3], abs=1e-12)
    assert moved[:3, 3] - T_actual[:3, 3] == pytest.approx([0.0, 0.0, -0.015], abs=1e-12)


def test_one_servo_step_closes_on_the_bar():
    """One step: flanges moved by ``T_corr`` put the held bar on its design pose, the grasp kept.

    The model (FK) is off by an unknown flange error E per arm (tool mount, grasp): the real flange is ``T_fk @ E``,
    so MoCap reads the bar at ``T_fk_left @ E_left @ A``. The IK puts the model flanges on ``T_corr @ T_fk``; the
    real bar then lands on the design.
    """
    T_design = random_rigid(41)
    A = random_rigid(42)                                      # left tool0 -> bar (the grasp)
    E_left, E_right = small_rigid(3.0, 0.1, 43), small_rigid(2.0, 0.2, 44)
    # Where the model puts the flanges now: 20 mm / 1 deg off the design, the right one 1.4 m away.
    T_fk_left = small_rigid(20.0, 1.0, 45) @ T_design @ np.linalg.inv(A)
    T_fk_right = T_fk_left @ random_rigid(46)
    T_mocap = T_fk_left @ E_left @ A
    assert pose_deviation(T_mocap, T_design).dpos_mm > 5.0

    T_corr = bar_servo_correction(T_mocap, T_design)
    new_left, new_right = T_corr @ T_fk_left, T_corr @ T_fk_right
    after = pose_deviation(new_left @ E_left @ A, T_design)
    assert after.dpos_mm == pytest.approx(0.0, abs=1e-9) and after.dang_deg == pytest.approx(0.0, abs=1e-6)
    # The real right flange seen from the real left one: unchanged (the bar is not strained).
    held_before = np.linalg.inv(T_fk_left @ E_left) @ (T_fk_right @ E_right)
    held_after = np.linalg.inv(new_left @ E_left) @ (new_right @ E_right)
    assert held_after == pytest.approx(held_before, abs=1e-9)
