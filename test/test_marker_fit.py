"""Tests for fitting a bar's CAD markers to raw mocap markers."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.world.marker_fit import bar_markers_from_prefab, fit_markers

#: Bar B20 of 260916_2ModulesDemo_prefab.json: its MoCap joints, markers in the bar frame, mm.
PREFAB = {"bars": [
    {"bar_id": "B20", "joints": [
        {"joint_id": "J20-22", "mocap": {"markers_mm": {"M1": [-27.35, -22.03, 84.9], "M2": [-23.48, 26.12, 84.9]}}},
        {"joint_id": "G20-T20-0"},
        {"joint_id": "J20-21", "mocap": {"markers_mm": {"M1": [-25.5, 24.15, 420.0], "M2": [-25.5, -24.15, 420.0]}}},
        {"joint_id": "J20-169", "mocap": {"markers_mm": {"M1": [27.27, 22.14, 1145.3], "M2": [23.58, -26.02, 1145.3]}}},
        {"joint_id": "J20-25", "mocap": {"markers_mm": {"M1": [-25.5, -24.15, 1680.0], "M2": [-25.5, 24.15, 1680.0]}}},
    ]},
    {"bar_id": "B22", "joints": [{"joint_id": "J20-22"}]},
]}


@pytest.fixture
def bar() -> np.ndarray:
    """B20's markers, metres."""
    return bar_markers_from_prefab(PREFAB)["B20"]


def test_prefab_keeps_only_bars_with_markers(bar):
    """Bars without mocap joints are left out; markers come in metres."""
    assert list(bar_markers_from_prefab(PREFAB)) == ["B20"]
    assert bar.shape == (8, 3)
    assert bar[0] == pytest.approx([-0.02735, -0.02203, 0.0849])


@pytest.mark.parametrize("seed", range(5))
def test_fit_recovers_pose_from_shuffled_noisy_subset(bar, seed):
    """The fit finds the matching and the pose from a shuffled, noisy subset of the markers."""
    rng = np.random.default_rng(seed)
    rotation = Rotation.random(random_state=seed).as_matrix()
    translation = rng.uniform(-3, 3, 3)
    # * Two markers hidden, the rest in a random order with 0.5 mm noise.
    shown = rng.permutation(8)[:6]
    measured = bar[shown] @ rotation.T + translation + rng.normal(0, 5e-4, (6, 3))
    fit = fit_markers(bar, measured)
    assert list(fit.model_index) == list(shown)
    assert fit.rms < 2e-3
    # ? Not the rotation itself: markers 35 mm off the bar axis leave its roll uncertain by a few tenths of a degree.
    assert fit.transform(bar) == pytest.approx(bar @ rotation.T + translation, abs=2e-3)


def test_fit_needs_three_markers_that_match(bar):
    """No fit from two markers, or from markers that match none of the bar's distances."""
    assert fit_markers(bar, bar[:2]) is None
    # * Spread far wider than any of the bar's distances.
    assert fit_markers(bar, np.array([[0, 0, 0], [5, 0, 0], [0, 7, 0]], dtype=float)) is None
