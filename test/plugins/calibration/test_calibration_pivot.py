"""The tip offset: solved from tool0 poses that touch one point, and judged before Punch uses it."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.plugins.calibration.analysis.checks import TILT, TIP_POSES, TIP_TILT, tip_checks
from husky_assembly_teleop.plugins.calibration.analysis.pivot import pivot_fit
from husky_assembly_teleop.shared.guide.step import fails
from husky_assembly_teleop.ui.style import HOVER_MAX
from husky_assembly_teleop.world.checks import BAD, GOOD, PENDING, WARN
from scaffolding_assembly_core.geometry import Pose

pytestmark = pytest.mark.safety

#: The punch tip in tool0, and the target it touches, metres.
TIP = np.array([0.001, -0.002, 0.11])
TARGET = np.array([0.6, 0.2, 0.3])
#: Tool0 pointing down, as when punching a target on a table.
DOWN = Rotation.from_euler("x", 180, degrees=True)


def touching(tilts, tip=TIP, noise: float = 0.0, seed: int = 0) -> list[Pose]:
    """Tool0 poses putting `tip` on TARGET, each tilted from DOWN by (axis, degrees); `noise` metres off at random."""
    random = np.random.default_rng(seed)
    poses = []
    for axis, degrees in tilts:
        rotation = DOWN * Rotation.from_euler(axis, degrees, degrees=True)
        poses.append(Pose.from_arrays(TARGET - rotation.apply(tip) + random.normal(0.0, noise, 3), rotation.as_quat()))
    return poses


#: Tilted ±30° about tool0's x and y: about 18° apart (RMS) about the axis turned least.
TWO_AXES = [("x", 0), ("x", 30), ("x", -30), ("y", 30), ("y", -30)]


def test_the_fit_finds_the_tip_and_the_target():
    """Exact touches give the tip and target back, with no residuals."""
    pivot = pivot_fit(touching(TWO_AXES))
    np.testing.assert_allclose(pivot.offset, TIP, atol=1e-9)
    np.testing.assert_allclose(pivot.point, TARGET, atol=1e-9)
    assert pivot.residuals.max() < 1e-9 and TIP_TILT < np.degrees(pivot.tilt) < 25.0


def test_noisy_touches_put_the_tip_within_their_noise():
    """Touches 0.1 mm off put the tip within about the noise over the tilt, and show residuals of that size."""
    pivot = pivot_fit(touching(TWO_AXES, noise=1e-4))
    assert np.linalg.norm(pivot.offset - TIP) < 1e-3 and 2e-5 < pivot.residuals.max() < 3e-4


def test_tilting_about_one_axis_leaves_the_tip_unknown_along_it():
    """Turns about x alone: the tilt is zero, which the checks show red."""
    pivot = pivot_fit(touching([("x", 0), ("x", 30), ("x", -30), ("x", 15)]))
    assert pivot.tilt < 1e-6
    assert {c.label: c.level for c in tip_checks(4, pivot, [Pose()] * 4)}[TILT] == BAD


def test_three_poses_are_needed():
    """Two cannot solve the tip."""
    with pytest.raises(ValueError, match="needs 3"):
        pivot_fit(touching(TWO_AXES[:2]))


def test_tip_checks_from_nothing_to_a_good_fit():
    """The same chips throughout: pending without a fit; green with enough tilted, close poses and a still base."""
    none = tip_checks(0, None, [])
    assert [c.label for c in none] == ["poses", TILT, "fit", "base", "TCP offset"]
    assert [c.level for c in none] == [BAD, PENDING, PENDING, GOOD, PENDING]
    good = tip_checks(len(TWO_AXES), pivot_fit(touching(TWO_AXES)), [Pose()] * len(TWO_AXES))
    assert [c.label for c in good] == [c.label for c in none] and [c.level for c in good] == [GOOD] * 5
    assert max(len(c.detail) for c in good) <= HOVER_MAX
    assert len(TWO_AXES) >= TIP_POSES


def test_tip_checks_warn_on_small_tilts_and_block_on_spread_tips_or_a_moved_base():
    """Tilts of ±10° only warn; tips 2 mm apart or a base moved 2 mm between poses block."""
    small = [("x", 0), ("x", 10), ("x", -10), ("y", 10), ("y", -10)]
    levels = {c.label: c.level for c in tip_checks(5, pivot_fit(touching(small)), [Pose()] * 5)}
    assert levels[TILT] == WARN and levels["fit"] == GOOD
    assert fails(tip_checks(5, pivot_fit(touching(TWO_AXES, noise=2e-3)), [Pose()] * 5))
    moved = [Pose()] * 4 + [Pose((0.002, 0.0, 0.0))]
    assert {c.label: c.level for c in tip_checks(5, pivot_fit(touching(TWO_AXES)), moved)}["base"] == BAD
