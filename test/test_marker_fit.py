"""Tests for a bar's marker layout from the process, and fitting it to raw mocap markers."""

from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.world.marker_fit import bar_deviation, bar_markers, fit_markers
from scaffolding_assembly_core.geometry import Pose
from process_fixtures import build_process

#: Bar B20 of 260916_2ModulesDemo: the markers of its four MoCap halves in the bar frame, metres.
B20 = np.array([[-27.35, -22.03, 84.9], [-23.48, 26.12, 84.9], [-25.5, 24.15, 420.0], [-25.5, -24.15, 420.0],
                [27.27, 22.14, 1145.3], [23.58, -26.02, 1145.3], [-25.5, -24.15, 1680.0],
                [-25.5, 24.15, 1680.0]]) * 1e-3


@pytest.fixture
def bar() -> np.ndarray:
    """B20's markers, metres."""
    return B20


def test_bar_layout_comes_from_its_mounted_halves(tmp_path):
    """Each marker of a half lands where the half sits on its bar; bars with under 3 markers are left out."""
    process = build_process(tmp_path)
    # * Three markers on B1's female half, which sits 0.85 m up the bar; B2's half gets two, too few to fit.
    female = replace(process.bodies["joints/J1_female"],
                     markers={"a": (0.01, 0.0, 0.02), "b": (-0.01, 0.0, 0.02), "c": (0.0, 0.03, 0.0)})
    male = replace(process.bodies["joints/J1_male"], markers={"a": (0.0, 0.0, 0.0), "b": (0.0, 0.0, 0.1)})
    process = replace(process, bodies={**process.bodies, female.id: female, male.id: male})
    layouts = bar_markers(process)
    assert list(layouts) == ["bars/B1"]
    names, points = layouts["bars/B1"]
    assert names == ("joints/J1_female/a", "joints/J1_female/b", "joints/J1_female/c")
    np.testing.assert_allclose(points, [[0.01, 0.0, 0.87], [-0.01, 0.0, 0.87], [0.0, 0.03, 0.85]], atol=1e-6)


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
    np.testing.assert_allclose(fit.pose.position, fit.world_from_model[:3, 3])


def test_fit_needs_three_markers_that_match(bar):
    """No fit from two markers, or from markers that match none of the bar's distances."""
    assert fit_markers(bar, bar[:2]) is None
    # * Spread far wider than any of the bar's distances.
    assert fit_markers(bar, np.array([[0, 0, 0], [5, 0, 0], [0, 7, 0]], dtype=float)) is None


def test_bar_deviation_splits_along_and_across_the_axis():
    """3 mm along the planned axis, 4 mm across it, and a 2° tilt, from a planned bar lying along world y."""
    planned = Pose((1.0, 2.0, 0.5), tuple(Rotation.from_euler("x", -90, degrees=True).as_quat()))
    axis, side = planned.matrix()[:3, 2], planned.matrix()[:3, 0]
    tilted = Rotation.from_euler("x", -88, degrees=True).as_quat()
    fitted = Pose(tuple(np.array(planned.position) + 0.003 * axis + 0.004 * side), tuple(tilted))
    deviation = bar_deviation(fitted, planned)
    assert deviation.along == pytest.approx(0.003, abs=1e-9)
    assert deviation.across == pytest.approx(0.004, abs=1e-9)
    assert np.degrees(deviation.tilt) == pytest.approx(2.0, abs=1e-6)
