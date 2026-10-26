"""Tests for a bar's marker layout from the process, and a fitted bar's deviation from its planned pose."""

from dataclasses import replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.world.marker_fit import bar_deviation, bar_markers
from scaffolding_assembly_core.geometry import Pose
from process_fixtures import build_process


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
