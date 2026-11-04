"""The plugin's own analysis of a real session's solution: the summary lines and the analysis folder."""

import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from repo_paths import DATA
from husky_assembly_teleop.plugins.calibration.analysis.fit import Take, solve, tool_points
from husky_assembly_teleop.plugins.calibration.analysis.report import summary_lines, write_analysis
from scaffolding_assembly_core.geometry import Pose, compose

#: Cindy's left arm, 20261016: six J0 and five J1 takes (see test_calibration_fit.py).
SESSION = Path(__file__).parent / "data" / "20261016.npz"


def _takes(session, batch: str) -> list[Take]:
    joint = int(batch[1])
    poses = [[Pose.from_arrays(row[:3], row[3:]) for row in session[f"{batch}_{body}"]] for body in ("base", "tool")]
    points, index = tool_points(*poses), session[f"{batch}_take"]
    return [Take(joint, points[index == k], session[f"{batch}_joints"][index == k, joint], f"{batch} take {k}")
            for k in range(index.max() + 1)]


@pytest.fixture(scope="module")
def solved():
    """A real solution, its takes, and an applied calibration 2 mm and 0.2° off the solved one."""
    session = np.load(SESSION)
    takes = _takes(session, "j0") + _takes(session, "j1")
    solution = solve([t for t in takes if t.joint == 0], [t for t in takes if t.joint == 1],
                     DATA / "husky_urdf" / str(session["urdf"]), str(session["arm"]))
    applied = compose(solution.footprint, Pose((0.002, 0.0, 0.0),
                                               tuple(Rotation.from_euler("z", 0.2, degrees=True).as_quat())))
    return solution, takes, applied


def test_summary_lines_give_the_headline_numbers(solved, tmp_path):
    """The change vs applied, per-joint fits, mount, validation and how many figures where, one line each."""
    solution, _, applied = solved
    lines = summary_lines(solution, applied, solution.footprint, [tmp_path / "a.png", tmp_path / "b.png"],
                          (np.array([1e-3, 2e-3]), np.array([[0.1, 0.2, 0.05]])),
                          Pose((0.0, 0.001, 0.0), (0.0, 0.0, 0.0, 1.0)))
    text = "\n".join(lines)
    assert lines[0] == "change vs applied: 2.00 mm, 0.200° from the calibration the relay applies now"
    assert summary_lines(solution, None, solution.footprint)[0] == \
        "change vs applied: the relay applies no calibration to compare with"
    assert "J0: 6 takes" in text and "J1: 5 takes" in text
    assert "mount 1.00 mm, 0.000° from the URDF's" in text
    assert "axes within 0.200°" in text
    assert lines[-1] == f"2 figures in {tmp_path}"


def test_the_analysis_goes_to_its_own_folder(solved, tmp_path):
    """solution.json and summary.txt in analysis/<digits>_<arm>/, nothing else in the session."""
    solution, takes, applied = solved
    files = [tmp_path / f"j{take.joint}" / f"calibration_take{k}.json" for k, take in enumerate(takes)]
    folder = write_analysis(tmp_path, "0806", "left_ur_arm", solution, files, applied)
    assert folder == tmp_path / "analysis" / "0806_left_ur_arm"
    assert sorted(path.name for path in tmp_path.rglob("*") if path.is_file()) == ["solution.json", "summary.txt"]
    data = json.loads((folder / "solution.json").read_text())
    assert data["base_mocap_from_base_footprint"][0] == pytest.approx(list(solution.footprint.position))
    assert [take["joint"] for take in data["takes"]] == [0] * 6 + [1] * 5
    assert data["takes"][0]["file"] == "j0/calibration_take0.json" and "validation" not in data
    assert (folder / "summary.txt").read_text().startswith("change vs applied: 2.00 mm")
