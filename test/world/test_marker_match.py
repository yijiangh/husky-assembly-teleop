"""Checks for the marker matcher: which sensed marker is which design marker, the fitted pose, and the sensed sample.

The checks build markers in memory, on a made-up T20 bar and on the real B20 table (`bar_fixtures`).
"""

from __future__ import annotations

import numpy as np
import pytest
from bar_fixtures import NOW, b20_design, random_rigid, raw_markers, t20_design

from husky_assembly_teleop.shared.bars.pose import pose_deviation
from husky_assembly_teleop.world.marker_match import kabsch, match_and_fit, sensed_from_raw, transform_points


# * ------------------------------------------------- fixtures

def fit(sensed: np.ndarray, design: np.ndarray, **kwargs):
    """`match_and_fit`, plus the check every fit must pass: never a mirror."""
    result = match_and_fit(sensed, design, **kwargs)
    if result.T_world_from_bar is not None:
        assert np.linalg.det(result.T_world_from_bar[:3, :3]) > 0
    return result


@pytest.fixture(params=["t20", "B20"])
def design(request) -> np.ndarray:
    """The made-up T20 bar and the real B20 bar, for the checks run on both."""
    return t20_design() if request.param == "t20" else b20_design()


# * ------------------------------------------------- matching

def test_exact_recovery(design):
    """Shuffled exact markers give back the pose and the shuffle."""
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
    """1 mm Gaussian noise on every coordinate still fits, at the noise level.

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
    """One occluded ball -> 7 of 8, still ok."""
    sensed = np.delete(transform_points(random_rigid(3), design), 3, axis=0)
    result = fit(sensed, design)
    assert result.ok, result.reason
    assert result.n_matched == 7
    assert result.missing_design == [3]


def test_ghost_marker(design):
    """A stray marker 0.3 m away ends up unmatched."""
    T_true = random_rigid(4)
    moved = transform_points(T_true, design)
    ghost = moved.mean(axis=0) + 0.3 * T_true[:3, 0]
    result = fit(np.vstack([moved, ghost]), design)
    assert result.ok, result.reason
    assert result.n_matched == len(design)
    assert result.unmatched_sensed == [len(design)]


def test_three_markers_dropped():
    """Three occluded balls -> 5 of 8 is not enough."""
    design = t20_design()
    sensed = np.delete(transform_points(random_rigid(5), design), [0, 3, 6], axis=0)
    result = fit(sensed, design)
    assert result.ok is False
    assert "5 of 8" in result.reason


def test_mirror_is_refused():
    """A mirrored marker set (e.g. a wrong axis convention) does not fit."""
    design = t20_design()
    result = fit(design * np.array([1.0, 1.0, -1.0]), design)
    assert result.ok is False
    assert np.linalg.det(result.T_world_from_bar[:3, :3]) > 0


def test_symmetric_layout_is_ambiguous():
    """Joints placed symmetric about the middle, all facing the same way."""
    design = t20_design(joint_z=(0.1, 0.5, 1.3, 1.7), rolls_deg=(0, 0, 0, 0))
    result = fit(transform_points(random_rigid(7), design), design)
    assert result.ambiguous is True
    assert result.second_best["n_matched"] == 8
    assert result.ok is False
    assert "equally well" in result.reason


def test_m1_m2_swap_resolved(design):
    """The M1/M2-swapped guesses lose on the full-set score."""
    T_true = random_rigid(8)
    result = fit(transform_points(T_true, design), design)
    assert result.ok, result.reason
    assert result.correspondences == {k: k for k in range(len(design))}
    # Some other assignment was tried and scored lower.
    assert result.second_best is not None
    assert result.second_best["n_matched"] < result.n_matched


def test_collinear_design():
    """Balls on one line -> the fit says why it fails."""
    collinear = np.array([[0.0255, 0.0, z] for z in (-0.8, -0.4, 0.3, 0.8)])
    result = fit(transform_points(random_rigid(9), collinear), collinear)
    assert result.ok is False
    assert "collinear" in result.reason


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


# * ------------------------------------------------- the sensed markers

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
