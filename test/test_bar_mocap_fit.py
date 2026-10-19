"""Checks for the bar pose from MoCap markers (numpy + pytest only: no ROS, no pybullet).

Most checks build markers in memory, on a made-up T20 bar and on the real B20
table of ``261016_2ModulesDemo_YH``. Two read real files and are skipped where
the shared drive is not on this machine: an Oct-1 take of the OLD 4-marker rig
(it must NOT fit a T20 bar) and the 2ModulesDemo ``MoCapMarkers.json`` (the last
part of that check also needs compas to read a BarAction).
"""
import json
import os
from time import perf_counter

import numpy as np
import pytest

from husky_assembly_teleop import DESIGN_DATA_DIRECTORY, EXPERIMENT_DATA_DIRECTORY
from husky_assembly_teleop.bar_mocap_fit import (
    MOCAP_MARKERS_FILENAME, BarMarkers, DesignMarker, MoCapMarkersError, apply_authored_move, bar_servo_correction,
    bar_slide_roll, design_bar_axis_endpoints, fake_sensed_markers, fit_bar_from_cache, kabsch, load_mocap_markers,
    match_and_fit, pose_deviation, transform_points,
)

# * The real B20 (1.8 m) of 261016_2ModulesDemo_YH: four T20_MoCap joints, two
# * balls each, ball centres in the bar frame (metres), as in MoCapMarkers.json.
B20_MARKERS = [
    ('J20-22_mocap_M1', [-0.02735, -0.02203, 0.0849]),
    ('J20-22_mocap_M2', [-0.02348, 0.02612, 0.0849]),
    ('J20-21_mocap_M1', [-0.0255, 0.02415, 0.42]),
    ('J20-21_mocap_M2', [-0.0255, -0.02415, 0.42]),
    ('J20-169_mocap_M1', [0.02727, 0.02214, 1.1453]),
    ('J20-169_mocap_M2', [0.02358, -0.02602, 1.1453]),
    ('J20-25_mocap_M1', [-0.0255, -0.02415, 1.68]),
    ('J20-25_mocap_M2', [-0.0255, 0.02415, 1.68]),
]

# The T20_MoCap plate: its two balls in the joint block frame (metres).
T20_BLOCK_M1 = [-0.010, 0.02415, 0.0255]
T20_BLOCK_M2 = [-0.010, -0.02415, 0.0255]

# The Oct-1 take of the old 4-marker cross rig (82 mm pairs), 'rhino' axis convention.
OLD_RIG_TAKE = os.path.join(EXPERIMENT_DATA_DIRECTORY,
                            'bar_holding_acc_data/20261001/bar_holding_acc_20261001_1514.json')
DEMO_PROBLEM = os.path.join(DESIGN_DATA_DIRECTORY, '261016_2ModulesDemo_YH')


# * ------------------------------------------------- fixtures and helpers

def t20_design(joint_z: tuple = (0.075, 0.430, 1.135, 1.670), rolls_deg: tuple = (0, 90, 180, 45)) -> np.ndarray:
    """A made-up T20 bar: one T20_MoCap plate per joint, joint frame = slide along z, then roll.

    Args:
        joint_z (tuple): Joint positions along the bar, metres.
        rolls_deg (tuple): Joint rolls about the bar, degrees.

    Returns:
        np.ndarray: (2 * joints, 3) ball centres in the bar frame, ``[J1 M1, J1 M2, J2 M1, ...]``.
    """
    block = np.array([T20_BLOCK_M1, T20_BLOCK_M2])
    return np.vstack([transform_points(bar_slide_roll(z, roll), block) for z, roll in zip(joint_z, rolls_deg)])


def b20_design() -> np.ndarray:
    """The real B20 ball centres in the bar frame.

    Returns:
        np.ndarray: (8, 3) metres.
    """
    return np.array([point for _name, point in B20_MARKERS])


def b20_bar_markers() -> BarMarkers:
    """The real B20 as the loader would build it.

    Returns:
        BarMarkers: ``bar_B20`` with its 8 named balls.
    """
    # 'J20-22_mocap_M1' sits on joint 'joint_J20-22_mocap' with label 'M1'.
    return BarMarkers('bar_B20', 1, [DesignMarker(name, 'joint_' + name.rsplit('_', 1)[0], name[-2:],
                                                  tuple(point), 'T20_MoCap') for name, point in B20_MARKERS])


def random_rigid(seed: int) -> np.ndarray:
    """A random bar pose somewhere in a 6 m room.

    Args:
        seed (int): Random seed.

    Returns:
        np.ndarray: 4x4 proper rigid transform.
    """
    rng = np.random.default_rng(seed)
    q, _r = np.linalg.qr(rng.normal(size=(3, 3)))
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1.0
    T = np.eye(4)
    T[:3, :3] = q
    T[:3, 3] = rng.uniform(-3.0, 3.0, 3)
    return T


def fit(sensed: np.ndarray, design: np.ndarray, **kwargs):
    """`match_and_fit`, plus the check every fit must pass: never a mirror.

    Args:
        sensed (np.ndarray): (N, 3) sensed points.
        design (np.ndarray): (M, 3) design points.
        **kwargs: Passed on to `match_and_fit`.

    Returns:
        BarFit: The fit.
    """
    result = match_and_fit(sensed, design, **kwargs)
    if result.T_world_from_bar is not None:
        assert np.linalg.det(result.T_world_from_bar[:3, :3]) > 0
    return result


@pytest.fixture(params=['t20', 'B20'])
def design(request) -> np.ndarray:
    """The made-up T20 bar and the real B20 bar, for the checks run on both."""
    return t20_design() if request.param == 't20' else b20_design()


def _sidecar(bars: dict, schema_version: int = 2, units: str = 'm') -> dict:
    """A schema-2 ``MoCapMarkers.json`` dict.

    Args:
        bars (dict): ``{bar body name: (n, 3) points}``.
        schema_version (int): Written as is.
        units (str): Written as is.

    Returns:
        dict: The file content.
    """
    return {
        'schema_version': schema_version, 'source': 'test.3dm', 'units': units, 'cell_file': 'RobotCell.json',
        'bar_frame': 'origin at the bar start, z along the bar',
        'marker_blocks': {'T20_MoCap': {'marker_diameter_m': 0.019,
                                        'points_block_m': {'M1': T20_BLOCK_M1, 'M2': T20_BLOCK_M2}}},
        'bars': {name: {'bar_id': name[len('bar_'):], 'assembly_seq': k + 1,
                        'markers': [{'name': f'J{k}-{j // 2}_mocap_M{j % 2 + 1}', 'joint': f'joint_J{k}-{j // 2}_mocap',
                                     'label': f'M{j % 2 + 1}', 'block': 'T20_MoCap', 'bar_m': list(p)}
                                    for j, p in enumerate(np.asarray(points).tolist())]}
                 for k, (name, points) in enumerate(bars.items())},
        'bars_without_markers': ['bar_B22', 'bar_B23'],
    }


def _write(folder, data: dict) -> str:
    """Write a ``MoCapMarkers.json`` into a folder.

    Args:
        folder (pathlib.Path): The problem folder.
        data (dict): The file content.

    Returns:
        str: The folder, as a string.
    """
    (folder / MOCAP_MARKERS_FILENAME).write_text(json.dumps(data))
    return str(folder)


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

    ! The roll about the bar is the weak direction: the balls sit only ~26 mm
    ! off the axis, so 1 mm of noise moves the roll by ~0.8 deg (one sigma).
    ! The tilt of the bar axis (balls spread over 1.6 m) stays far below 0.2 deg.
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
    assert '5 of 8' in result.reason


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
    assert result.second_best['n_matched'] == 8
    assert result.ok is False
    assert 'equally well' in result.reason


def test_m1_m2_swap_resolved(design):
    """Case 8 (and 13 on B20): the M1/M2-swapped guesses lose on the full-set score."""
    T_true = random_rigid(8)
    result = fit(transform_points(T_true, design), design)
    assert result.ok, result.reason
    assert result.correspondences == {k: k for k in range(len(design))}
    # Some other assignment was tried and scored lower.
    assert result.second_best is not None
    assert result.second_best['n_matched'] < result.n_matched


def test_collinear_design(tmp_path):
    """Case 9: balls on one line -> the loader warns, the fit says why it fails."""
    collinear = np.array([[0.0255, 0.0, z] for z in (0.1, 0.5, 1.2, 1.7)])
    result = fit(transform_points(random_rigid(9), collinear), collinear)
    assert result.ok is False
    assert 'collinear' in result.reason

    markers = load_mocap_markers(_write(tmp_path, _sidecar({'bar_B9': collinear, 'bar_B20': b20_design()})))
    assert len(markers.warnings) == 1
    assert 'bar_B9' in markers.warnings[0] and 'collinear' in markers.warnings[0]


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
    assert 'only 2 markers' in fit(design[:2], design).reason
    many = np.vstack([design + [0.0, 0.0, 2.0 * k] for k in range(4)])
    assert 'too many markers' in fit(many, design).reason


def test_kabsch_never_mirrors():
    """Kabsch on a mirrored set returns a proper rotation, on a clean set the exact pose."""
    pts = t20_design()
    T_true = random_rigid(12)
    assert np.allclose(kabsch(pts, transform_points(T_true, pts)), T_true)
    assert np.linalg.det(kabsch(pts, pts * [1.0, 1.0, -1.0])[:3, :3]) > 0


# * ------------------------------------------------- loader (case 10)

def test_loader_round_trip(tmp_path):
    """A schema-2 file loads into the data classes."""
    markers = load_mocap_markers(_write(tmp_path, _sidecar({'bar_B20': b20_design()})))
    assert markers.schema_version == 2 and markers.units == 'm'
    assert markers.path == str(tmp_path / MOCAP_MARKERS_FILENAME)
    assert markers.bars_without_markers == ['bar_B22', 'bar_B23']
    assert markers.blocks['T20_MoCap']['points_block_m']['M1'] == T20_BLOCK_M1
    assert markers.warnings == []
    bar = markers.bars['bar_B20']
    assert isinstance(bar, BarMarkers) and bar.assembly_seq == 1
    assert np.allclose(bar.points(), b20_design())
    assert bar.names()[:2] == ['J0-0_mocap_M1', 'J0-0_mocap_M2']
    assert bar.markers[0].joint == 'joint_J0-0_mocap' and bar.markers[0].block == 'T20_MoCap'


def test_loader_refuses_schema_1(tmp_path):
    """A file in the old schema-1 shape (mm, a list of bars) names its version."""
    schema_1 = {
        'schema_version': 1, 'source': 'test.3dm', 'units': 'mm', 'world': 'rhino',
        'bar_frame': 'origin at the bar start, z along the bar (as RobotCell bar bodies)',
        'marker_name': '<joint object name>_<label>, e.g. J20-21_mocap_M1',
        'bars': [{'bar_id': 'B20', 'assembly_seq': 1, 'start_mm': [0, 0, 0], 'end_mm': [0, 0, 1800],
                  'length_mm': 1800.0, 'joints': [],
                  'markers': [{'index': 1, 'name': 'J20-22_mocap_M1', 'joint_id': 'J20-22', 'label': 'M1',
                               'world_mm': [1.0, 2.0, 3.0], 'bar_mm': [-27.35, -22.03, 84.9]}]}],
        'warnings': [],
    }
    with pytest.raises(MoCapMarkersError, match='schema_version 1'):
        load_mocap_markers(_write(tmp_path, schema_1))


def test_loader_missing_file(tmp_path):
    """No file -> an error that names the full path."""
    with pytest.raises(MoCapMarkersError, match=MOCAP_MARKERS_FILENAME):
        load_mocap_markers(str(tmp_path))


@pytest.mark.parametrize('broken, match', [
    ({'units': 'mm'}, 'units'),
    ({'two_markers': True}, 'at least 3'),
    ({'bad_bar_m': True}, 'malformed'),
])
def test_loader_refuses_broken_files(tmp_path, broken, match):
    """Millimetres, a bar with 2 markers and a malformed ``bar_m`` are refused."""
    data = _sidecar({'bar_B20': b20_design()}, units=broken.get('units', 'm'))
    markers = data['bars']['bar_B20']['markers']
    if broken.get('two_markers'):
        del markers[2:]
    if broken.get('bad_bar_m'):
        markers[0]['bar_m'] = [0.1, 0.2]
    with pytest.raises(MoCapMarkersError, match=match):
        load_mocap_markers(_write(tmp_path, data))


# * ------------------------------------------------- monitor cache (cases 11, 14, 15)

@pytest.mark.skipif(not os.path.isfile(OLD_RIG_TAKE), reason='the Oct-1 bar-holding take is not on this machine')
def test_old_rig_take_does_not_fit_a_t20_bar():
    """Case 11: every take of the old 82 mm rig is refused against a T20 bar, without an exception."""
    with open(OLD_RIG_TAKE, 'r') as f:
        takes = json.load(f)['raw_data']
    for take in takes:
        cache = take['bar_rig']
        assert all(isinstance(marker_id, str) for marker_id in cache)
        for design in (t20_design(), b20_design()):
            result = fit_bar_from_cache(cache, design)
            assert result.ok is False, result.reason


def test_fake_markers_round_trip():
    """Case 14: the fake snapshot fits 30 mm along and 2 deg about the bar, the stray unmatched."""
    design = b20_bar_markers()
    T_design = random_rigid(14)
    # With the default 0.5 mm noise the roll about the bar wanders ~0.3 deg (one sigma).
    for noise_m, tol_mm, tol_deg in ((0.0, 1e-6, 1e-6), (0.0005, 1.5, 1.5)):
        cache = fake_sensed_markers(T_design, design.points(), noise_m=noise_m)
        assert sorted(cache) == list(range(1, 10))
        result = fit_bar_from_cache(cache, design)
        assert result.ok, result.reason
        assert result.bar_name == 'bar_B20'
        assert result.n_matched == 8
        assert len(result.unmatched_sensed) == 1
        stray_id = result.sensed_ids[result.unmatched_sensed[0]]
        # The stray is the one marker far from every design ball.
        placed = transform_points(T_design, design.points())
        assert np.linalg.norm(placed - cache[stray_id]['pos'], axis=1).min() > 0.2
        dev = pose_deviation(result.T_world_from_bar, T_design)
        assert dev.dpos_bar_mm == pytest.approx([0.0, 0.0, 30.0], abs=tol_mm)
        assert dev.dang_deg == pytest.approx(2.0, abs=tol_deg)
        json.dumps(result.to_json())
        json.dumps(dev.to_json())
    # Exact markers come back exactly at the documented pose.
    exact = fit_bar_from_cache(fake_sensed_markers(T_design, design.points(), noise_m=0.0), design)
    assert np.allclose(exact.T_world_from_bar, T_design @ bar_slide_roll(0.03, 2.0))


def test_cache_size_forms_and_filters():
    """Case 15: ``size`` as a number or ``[0.0188]``; string ids; bad and old markers dropped."""
    design = t20_design()
    T_true = random_rigid(15)
    moved = transform_points(T_true, design)
    for size in (0.0188, [0.0188], (0.0188,)):
        cache = {str(k + 1): {'pos': p.tolist(), 'size': size, 'error': 0.0008} for k, p in enumerate(moved)}
        result = fit_bar_from_cache(cache, design)
        assert result.ok, result.reason
        assert np.allclose(result.T_world_from_bar, T_true)
        assert {result.sensed_ids[s] for s in result.correspondences} == set(cache)

    # A tiny blob and a marker Motive does not trust are dropped before matching.
    cache = {k + 1: {'pos': p.tolist(), 'size': [0.0188], 'error': 0.0008} for k, p in enumerate(moved)}
    cache[1]['size'] = [0.005]
    cache[2]['error'] = 0.01
    result = fit_bar_from_cache(cache, design)
    assert result.ok, result.reason
    assert result.n_matched == 6
    assert 1 not in result.sensed_ids and 2 not in result.sensed_ids

    old = fit_bar_from_cache(cache, design, age_s=2.0, max_age_s=0.5)
    assert old.ok is False and 'old' in old.reason


def test_cache_occluded_markers_dropped():
    """A hidden marker of a Motive asset still streams, flagged occluded, at Motive's guess: never fitted."""
    design = t20_design()
    T_true = random_rigid(16)
    moved = transform_points(T_true, design)
    cache = {k + 1: {'pos': p.tolist(), 'size': [0.0188], 'error': 0.0008, 'flags': 0} for k, p in enumerate(moved)}
    # One occluded marker (Motive's guess sits exactly on the design spot): it is still missing.
    cache[1]['flags'] = 0x01 | 0x04
    result = fit_bar_from_cache(cache, design)
    assert result.ok and result.n_matched == 7 and 1 not in result.sensed_ids
    # Three occluded: 5 of 8 left, and the reason says why.
    for k in (2, 3):
        cache[k]['flags'] = 0x01
    result = fit_bar_from_cache(cache, design)
    assert result.ok is False and '5 of 8' in result.reason and '3 marker(s) occluded' in result.reason


# * ------------------------------------------------- following live: the match around the last pose

# * Since the hint (the last fit), the bar slid this far along itself and rolled this much.
TRACK_SLIDE_M = 0.015
TRACK_ROLL_DEG = 1.0
# The tracked fit for 8 markers must be quick: the live mode re-fits every bar at 10 Hz.
TRACK_MAX_MEDIAN_S = 0.005


def _cache_of(points: np.ndarray) -> dict:
    """A marker cache snapshot (int ids from 1) holding these points.

    Args:
        points (np.ndarray): (N, 3) marker positions in world, metres.

    Returns:
        dict: ``{id: {'pos', 'size', 'error', 'flags'}}``.
    """
    return {k + 1: {'pos': p.tolist(), 'size': [0.0188], 'error': 0.0008, 'flags': 0} for k, p in enumerate(points)}


def _moved_since_hint(seed: int) -> tuple:
    """A B20 pose (the hint) and the B20 markers after the bar moved a bit from it.

    Args:
        seed (int): Random seed of the hint pose and the 0.5 mm noise.

    Returns:
        tuple: ``(T_hint, T_now, cache)``: the old pose, the pose now (slid 15 mm,
        rolled 1 deg), and the markers at the pose now as a cache snapshot.
    """
    T_hint = random_rigid(seed)
    T_now = T_hint @ bar_slide_roll(TRACK_SLIDE_M, TRACK_ROLL_DEG)
    points = transform_points(T_now, b20_design())
    points = points + np.random.default_rng(seed).normal(0.0, 0.0005, points.shape)
    return T_hint, T_now, _cache_of(points)


def test_tracked_fit_matches_the_full_search():
    """A hint near the truth: the quick match is used and gives the full search's pose."""
    design = b20_bar_markers()
    T_hint, T_now, cache = _moved_since_hint(20)
    full = fit_bar_from_cache(cache, design)
    tracked = fit_bar_from_cache(cache, design, T_hint=T_hint)
    assert full.ok and not full.tracked
    assert tracked.ok and tracked.tracked, tracked.reason
    assert tracked.n_matched == 8 and tracked.correspondences == full.correspondences
    assert np.abs(tracked.T_world_from_bar - full.T_world_from_bar).max() < 1e-6
    assert pose_deviation(tracked.T_world_from_bar, T_now).dpos_mm < 1.5
    assert tracked.ambiguous is False and tracked.second_best is None and tracked.n_hypotheses == 0
    assert tracked.bar_name == 'bar_B20' and tracked.reason == ''
    assert json.loads(json.dumps(tracked.to_json()))['tracked'] is True


def test_tracked_fit_falls_back_when_the_hint_is_far():
    """A hint 100 mm off: no marker within 20 mm, so the full search runs (still ok, not tracked)."""
    design = b20_bar_markers()
    _T_hint, T_now, cache = _moved_since_hint(21)
    far = np.eye(4)
    far[:3, 3] = [0.1, 0.0, 0.0]
    result = fit_bar_from_cache(cache, design, T_hint=far @ T_now)
    assert result.ok, result.reason
    assert result.tracked is False and result.n_hypotheses > 0
    assert pose_deviation(result.T_world_from_bar, T_now).dpos_mm < 1.5


def test_tracked_fit_with_an_occluded_marker():
    """A hidden ball (Motive's 'occluded' flag): never fitted, and the full search runs (7 of 8, not tracked)."""
    design = b20_bar_markers()
    T_hint, _T_now, cache = _moved_since_hint(22)
    cache[4]['flags'] = 0x01
    result = fit_bar_from_cache(cache, design, T_hint=T_hint)
    assert result.ok and not result.tracked, result.reason
    assert result.n_matched == 7 and result.missing_design == [3]
    assert 4 not in result.sensed_ids


def test_tracked_fit_never_stays_on_a_partial_match():
    """A hint on a wrong pose that matches only some balls: the full search takes over and finds the truth."""
    design = b20_bar_markers()
    T_hint, T_now, cache = _moved_since_hint(24)
    # The hint turned half a turn about the bar's middle: at most part of the balls line up.
    T_wrong = T_now @ bar_slide_roll(0.88, 180.0) @ bar_slide_roll(-0.88, 0.0)
    result = fit_bar_from_cache(cache, design, T_hint=T_wrong)
    assert result.ok and result.n_matched == 8, result.reason
    assert pose_deviation(result.T_world_from_bar, T_now).dpos_mm < 1.5


def test_tracked_fit_is_quick():
    """8 markers around a hint: the median of 50 tracked fits stays under 5 ms."""
    design = b20_bar_markers()
    T_hint, _T_now, cache = _moved_since_hint(23)
    seconds = []
    for _ in range(50):
        start = perf_counter()
        result = fit_bar_from_cache(cache, design, T_hint=T_hint)
        seconds.append(perf_counter() - start)
    assert result.tracked
    assert np.median(seconds) < TRACK_MAX_MEDIAN_S, f"median {np.median(seconds) * 1000:.2f} ms"


# * ------------------------------------------------- pose deviation (case 12)

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


def test_axis_endpoints():
    """The drawn bar line runs from the bar origin along its z."""
    T = random_rigid(13)
    ends = design_bar_axis_endpoints(T, 1.8)
    assert ends[0] == pytest.approx(T[:3, 3])
    assert ends[1] == pytest.approx(T[:3, 3] + 1.8 * T[:3, 2])


# * ------------------------------------------------- servo on the MoCap bar

def small_rigid(offset_mm: float, angle_deg: float, seed: int) -> np.ndarray:
    """A small rigid move: ``offset_mm`` along a random direction, ``angle_deg`` about a random axis.

    Args:
        offset_mm (float): Length of the translation, mm.
        angle_deg (float): Rotation angle, degrees.
        seed (int): Random seed.

    Returns:
        np.ndarray: 4x4 proper rigid transform.
    """
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

    The model (FK) is off by an unknown flange error E per arm (tool mount, grasp):
    the real flange is ``T_fk @ E``, so MoCap reads the bar at ``T_fk_left @ E_left @ A``.
    The IK puts the model flanges on ``T_corr @ T_fk``; the real bar then lands on the design.
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


# * ------------------------------------------------- the real 2ModulesDemo sidecar (case 16)

@pytest.mark.skipif(not os.path.isfile(os.path.join(DEMO_PROBLEM, MOCAP_MARKERS_FILENAME)),
                    reason='the 261016_2ModulesDemo_YH export is not on this machine')
def test_demo_sidecar_matches_bar_actions():
    """Case 16: the sidecar loads, and B20__R's joint frames x block balls land on ``bar_m``."""
    markers = load_mocap_markers(DEMO_PROBLEM)
    assert markers.schema_version == 2
    assert {name: len(bar.markers) for name, bar in markers.bars.items()} == {'bar_B20': 8, 'bar_B21': 8}
    assert markers.warnings == []

    # ? compas (and the BarAction classes) only for this last part, so the rest runs anywhere.
    pytest.importorskip('compas')
    pytest.importorskip('rs_data_structure')
    from compas.data import json_load
    from compas.geometry import Transformation

    action = json_load(os.path.join(DEMO_PROBLEM, 'BarActions', 'B20__R.json'))
    rb_states = action.movements[0].start_state.rigid_body_states
    bar_from_world = np.linalg.inv(np.array(Transformation.from_frame(rb_states['bar_B20'].frame).matrix))
    for marker in markers.bars['bar_B20'].markers:
        world_from_joint = np.array(Transformation.from_frame(rb_states[marker.joint].frame).matrix)
        in_block = markers.blocks[marker.block]['points_block_m'][marker.label]
        in_bar = transform_points(bar_from_world @ world_from_joint, in_block)
        assert np.linalg.norm(in_bar - marker.bar_m) < 5e-5, marker.name
