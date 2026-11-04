"""The old pipeline's files written from a real session's solution: names, formats and keys as the old scripts'."""

import json
from pathlib import Path

import numpy as np
import pytest
import yaml
from matplotlib import projections, pyplot

from repo_paths import DATA
from husky_assembly_teleop.plugins.calibration.analysis.fit import solve
from husky_assembly_teleop.plugins.calibration.analysis.old_outputs import (traj_label, write_punch_outputs,
                                                                            write_solve_outputs,
                                                                            write_validation_outputs)
from husky_assembly_teleop.plugins.calibration.storage.session_files import (punch_take, read_take, write_punch,
                                                                             write_solution, write_take)
from scaffolding_assembly_core.geometry import Pose

pytestmark = pytest.mark.slow

#: Belle's 20261013 session (see test_calibration_fit.py).
SESSION = np.load(Path(__file__).parent / "data" / "20261013.npz")
URDF = DATA / "husky_urdf" / str(SESSION["urdf"])
ARM = str(SESSION["arm"])
PNG = b"\x89PNG\r\n\x1a\n"
#: Keys of the old scripts' files in Belle's 20261013 session on the Drive.
OLD_ANALYSIS_TAKE_KEYS = {"file_name", "raw_data", "center", "normal"}
OLD_BASE_FRAME_KEYS = {"base_mocap_from_ur_arm_base_link_inertia", "j0_line", "j1_line", "base_offset"}
OLD_RHINO_KEYS = {"base_mocap_from_ur_arm_base_link_inertia", "base_mocap_from_base_footprint"}
OLD_CONFIG_KEYS = {"data_batches", "validation_data_batch", "robot_name", "arm", "export", "use_gui",
                   "punch_validation", "punch_tool"}


def _samples(batch: str, rows: np.ndarray) -> list[dict]:
    return [{"joint_conf": joints.tolist(), "base_mocap_pose": [base[:3].tolist(), base[3:].tolist()],
             "flange_mocap_pose": [tool[:3].tolist(), tool[3:].tolist()]}
            for joints, base, tool in zip(SESSION[f"{batch}_joints"][rows], SESSION[f"{batch}_base"][rows],
                                          SESSION[f"{batch}_tool"][rows])]


@pytest.fixture(scope="module")
def session(tmp_path_factory):
    """Belle's takes filed as the plugin files them, solved, with the old scripts' outputs written."""
    folder = tmp_path_factory.mktemp("20261013")
    files = []
    for batch in ("j0", "j1"):
        (folder / batch).mkdir()
        index = SESSION[f"{batch}_take"]
        files += [write_take(folder / batch, f"101500_0805_{ARM}_J{batch[1]}_take{k + 1}", _samples(batch, index == k))
                  for k in range(index.max() + 1)]
    takes = [read_take(path, int(path.parent.name[1]))[0] for path in files]
    solution = solve(takes[:6], takes[6:], URDF, ARM)
    write_solution(folder, "0805", ARM, 2, solution)
    solved = write_solve_outputs(folder, "0805", ARM, URDF, solution, files, punch_offset=[0.00058, 0.00158, 0.11054])
    (folder / "validation").mkdir()
    rows = np.zeros(len(SESSION["j0_take"]), bool)
    rows[::31] = True
    validation = [write_take(folder / "validation", f"120000_0805_{ARM}_validation", _samples("j0", rows))]
    validated = write_validation_outputs(folder, "0805", ARM, URDF, solution.footprint, validation)
    return folder, solved, validated


def _assert_written(folder: Path, files: list[Path], names: list[str]) -> None:
    assert [str(file.relative_to(folder)) for file in files] == names
    for file in files:
        if file.suffix == ".png":
            assert file.read_bytes()[:8] == PNG and file.stat().st_size > 10_000
    assert pyplot.get_fignums() == []


def test_solve_outputs_have_the_old_names(session):
    """Scripts 0 to 2's logs, analysis files and figures, under their old names and folders."""
    folder, solved, _ = session
    _assert_written(folder, solved, [
        "j0/circle_fitting_log_j0.txt", "j0/j0_analysis.json", "j0/j0_all_circles.png", "j0/j0_projection_errors.png",
        "j1/circle_fitting_log_j1.txt", "j1/j1_analysis.json", "j1/j1_all_circles.png", "j1/j1_projection_errors.png",
        "calibration_analysis_log.txt", "j0_base_mocap_pos_deviation.png", "j0_base_mocap_axis_deviation.png",
        "j1_base_mocap_pos_deviation.png", "j1_base_mocap_axis_deviation.png", "j0_circle_centers.png",
        "j1_circle_centers.png", "calibration_analysis_result.png", "compute_tf_log.txt", "config.yaml"])


def test_solve_files_have_the_old_keys(session):
    """The old files' keys in analysis, base frame, relay and config files; the relay's names body and environment."""
    folder, _, _ = session
    for batch in ("j0", "j1"):
        analysis = json.loads((folder / batch / f"{batch}_analysis.json").read_text())
        assert set(analysis) == {"takes"} and len(analysis["takes"]) == 6
        assert all(set(take) == OLD_ANALYSIS_TAKE_KEYS for take in analysis["takes"])
    frame = json.loads((folder / "base_frame_calibration.json").read_text())
    assert set(frame) == OLD_BASE_FRAME_KEYS and set(frame["j0_line"]) == {"point", "direction"}
    assert set(json.loads((folder / "calibrated_transformation_0805_rhino.json").read_text())) == \
        OLD_RHINO_KEYS | {"mocap_id", "environment", "ros_domain"}
    config = yaml.safe_load((folder / "config.yaml").read_text())
    assert set(config) == OLD_CONFIG_KEYS
    assert config["robot_name"] == "0805" and config["arm"] == "left" and config["data_batches"] == ["j0", "j1"]
    assert config["punch_tool"]["left"]["offset_xyz"] == [0.00058, 0.00158, 0.11054]


def test_logs_read_as_the_old_ones(session):
    """Old line format, take labels from the plugin's file names, and the solved numbers."""
    folder, _, _ = session
    fitting = (folder / "j0" / "circle_fitting_log_j0.txt").read_text().splitlines()
    assert fitting[1].endswith(" - INFO - Processing data batch: j0")
    assert fitting[-1].endswith(" - INFO - Total takes processed: 6")
    analysis = (folder / "calibration_analysis_log.txt").read_text()
    assert "Base offset: 0.16259148781817487 m (162.5915 mm)" in analysis
    assert "Origin: [  49.4416" in analysis
    assert "Position difference magnitude: 0.000 mm" in (folder / "compute_tf_log.txt").read_text()
    assert traj_label("calibration_101500_0805_ur_arm_J1_take7.json") == "J1_T7"
    assert traj_label("calibration_20261014_1007_ext_calib_0804_J0_traj0_JointTrajectory.json") == "J0_T0"


def test_validation_outputs_have_the_old_names(session):
    """Script 3's log and five figures in validation/."""
    folder, _, validated = session
    _assert_written(folder, validated, [
        "validation/verification_log.txt", "validation/debug_tool0_vs_flange_validation.png",
        "validation/debug_orientation_validation.png", "validation/verification_validation.png",
        "validation/debug_offset_positions_validation.png", "validation/diversity_validation.png"])
    log = (folder / "validation" / "verification_log.txt").read_text()
    assert "Processed 6 samples" in log and log.rstrip().endswith("=" * 60)


def _punch(folder: Path, name: str, arm_index: int, count: int, offset=(0.0, 0.0, 0.11)) -> None:
    takes = [punch_take("single", arm_index, "ur_arm_tool0", np.zeros(6), Pose((k * 0.5, 0.0, 0.0)),
                        Pose((0.3 + 1e-3 * k, -1.5, 0.47)), np.array(offset)) for k in range(count)]
    write_punch(folder / "punch_validation" / f"punch_validation_single_{name}.json", takes)


def test_punch_outputs_have_the_old_names(tmp_path):
    """Script 4's three figures, from every punch file of the arm, named by the arm as the files name it."""
    _punch(tmp_path, "20261014_1833", 0, 1)
    assert write_punch_outputs(tmp_path) == []
    _punch(tmp_path, "20261014_1840", 0, 3)
    _assert_written(tmp_path, write_punch_outputs(tmp_path, 0), [
        "punch_validation/punch_validation_position_single.png", "punch_validation/punch_validation_3d_single.png",
        "punch_validation/punch_validation_diversity_single.png"])


def test_punch_takes_of_several_tools_or_arms_are_refused(tmp_path):
    """Mixed tool offsets in one arm, or two arms with none chosen, as the old script."""
    _punch(tmp_path, "a", 0, 2)
    _punch(tmp_path, "b", 0, 2, offset=(0.0, 0.0, 0.12))
    with pytest.raises(ValueError, match="tool0_from_punch_tip"):
        write_punch_outputs(tmp_path)
    _punch(tmp_path, "b", 1, 2)
    with pytest.raises(ValueError, match="several arms"):
        write_punch_outputs(tmp_path)


def test_3d_axes_are_registered():
    """The 3D projection works although ROS's path shadows matplotlib's own toolkits."""
    assert "3d" in projections.get_projection_names()
