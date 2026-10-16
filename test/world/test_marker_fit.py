"""Tests for a bar's marker layout from the process, and fitting it to raw mocap markers."""

from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.world.marker_fit import bar_deviation, bar_markers, fit_near
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


def _transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """A 4x4 transform from a rotation matrix and a translation."""
    matrix = np.eye(4)
    matrix[:3, :3], matrix[:3, 3] = rotation, translation
    return matrix


@pytest.mark.parametrize("seed", range(5))
def test_fit_finds_the_bar_among_strays_near_a_rough_prior(bar, seed):
    """From a prior 3 cm and 3° off, the fit picks the bar's markers out of strays, in any order and with noise."""
    rng = np.random.default_rng(seed)
    rotation = Rotation.random(random_state=seed).as_matrix()
    translation = rng.uniform(-3, 3, 3)
    true = bar @ rotation.T + translation
    # * Two markers hidden, the rest with 0.5 mm noise.
    shown = rng.permutation(8)[:6]
    # * Strays: the mating half's four markers 5 cm off, random points near the bar but not on a marker, and a
    #   whole second bar 25 cm away.
    mate = true[:2] + rotation @ [0.05, 0.0, 0.0]
    mate = np.concatenate([mate, true[:2] + rotation @ [0.05, 0.0, 0.03]])
    near = true[rng.integers(0, 8, 40)] + rng.uniform(-0.06, 0.06, (40, 3))
    near = near[np.linalg.norm(near[:, None] - true[None], axis=-1).min(axis=1) > 0.025]
    other = true + rotation @ [0.25, 0.0, 0.0]
    cloud = np.concatenate([true[shown] + rng.normal(0, 5e-4, (6, 3)), mate, near, other])
    order = rng.permutation(len(cloud))
    prior = _transform(Rotation.from_rotvec(rng.normal(size=3) * 0.03).as_matrix() @ rotation,
                       translation + rng.normal(size=3) / np.sqrt(3) * 0.03)
    fit = fit_near(bar, cloud[order], prior)
    assert sorted(fit.model_index) == sorted(shown)
    assert fit.rms < 2e-3
    # ? Not the rotation itself: markers 35 mm off the bar axis leave its roll uncertain by a few tenths of a degree.
    assert fit.transform(bar) == pytest.approx(true, abs=2e-3)
    np.testing.assert_allclose(fit.points, true[fit.model_index], atol=3e-3)


def test_fit_needs_three_markers_near_the_prior(bar):
    """No fit from two markers, or from a whole bar out of reach of the prior."""
    assert fit_near(bar, bar[:2], np.eye(4)) is None
    assert fit_near(bar, bar + [0.5, 0.0, 0.0], np.eye(4)) is None
    assert len(fit_near(bar, bar + [0.03, 0.0, 0.0], np.eye(4)).residuals) == 8


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
