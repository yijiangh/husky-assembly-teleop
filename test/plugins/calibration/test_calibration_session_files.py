"""Calibration session files: takes read back by the joint they turn, and outputs the old scripts and the relay read."""

import json
import shutil
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from repo_paths import DATA
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.world.calibration_sessions import SESSIONS_FOLDER, newest_session_file, read_session_file
from husky_assembly_teleop.plugins.calibration.analysis.fit import solve
from husky_assembly_teleop.plugins.calibration.storage.session_files import (approve_calibration, archive,
                                                                             calibration_file, punch_take, read_punch,
                                                                             read_takes, write_punch, write_solution,
                                                                             write_take)
from scaffolding_assembly_core.geometry import Pose

pytestmark = pytest.mark.safety

#: Belle's 20261013 session (see test_calibration_fit.py).
SESSION = np.load(Path(__file__).parent / "data" / "20261013.npz")


def _write_session(session: Path) -> None:
    """File Belle's takes as the old monitor did, plus a J1 take misfiled under j0/ and a duplicate file."""
    for batch in ("j0", "j1"):
        (session / batch).mkdir(parents=True)
        index = SESSION[f"{batch}_take"]
        for k in range(index.max() + 1):
            rows = index == k
            samples = [{"joint_conf": joints.tolist(), "base_mocap_pose": [base[:3].tolist(), base[3:].tolist()],
                        "flange_mocap_pose": [tool[:3].tolist(), tool[3:].tolist()], "robot_id": 0}
                       for joints, base, tool in zip(SESSION[f"{batch}_joints"][rows], SESSION[f"{batch}_base"][rows],
                                                     SESSION[f"{batch}_tool"][rows])]
            write_take(session / batch, f"{batch}_take{k}", samples)
    shutil.move(session / "j1" / "calibration_j1_take0.json", session / "j0" / "calibration_j1_misfiled.json")
    shutil.copy(session / "j0" / "calibration_j0_take2.json", session / "j0" / "calibration_j0_copy.json")


def test_takes_are_filed_by_the_joint_they_turn(tmp_path):
    """A J1 take filed under j0/ counts as J1, a copied file once; the solution is the old one."""
    session = tmp_path / "20261013"
    _write_session(session)
    takes, skipped = read_takes(session)
    assert [take.joint for take in takes] == [0] * 6 + [1] * 6
    assert "j0/calibration_j1_misfiled.json" in [take.name for take in takes if take.joint == 1]
    assert len(skipped) == 1 and "the same samples as an earlier file" in skipped[0]
    robot = robot_config_from_serial("0805", DATA)
    solution = solve([t for t in takes if t.joint == 0], [t for t in takes if t.joint == 1], robot.model.urdf, "ur_arm")
    assert solution.footprint.position == pytest.approx(SESSION["footprint"][:3], abs=1e-6)


def test_write_take_never_overwrites(tmp_path):
    """A second take of the same name is refused, so a session's raw data is never lost."""
    write_take(tmp_path, "a", [])
    with pytest.raises(FileExistsError):
        write_take(tmp_path, "a", [])


def test_the_relay_reads_the_written_calibration(tmp_path):
    """The written calibration file is the one the relay picks for the robot, with the solved transform."""
    root = tmp_path
    session = root / SESSIONS_FOLDER / "20991231"
    _write_session(session)
    takes, _ = read_takes(session)
    robot = robot_config_from_serial("0805", DATA)
    solution = solve([t for t in takes if t.joint == 0], [t for t in takes if t.joint == 1], robot.model.urdf, "ur_arm")
    path = write_solution(session, "0805", "ur_arm", robot.mocap_id, solution)
    assert path == calibration_file(session, "0805")
    calibration = read_session_file(path, root, robot.mocap_id, "a200-0805")
    assert calibration.position == pytest.approx(solution.footprint.position)
    turn = Rotation.from_quat(calibration.orientation).inv() * Rotation.from_quat(solution.footprint.orientation)
    assert turn.magnitude() < 1e-9
    assert calibration.measured == ("base_mocap_from_ur_arm_base_link_inertia",)
    frame = json.loads((session / "base_frame_calibration.json").read_text())
    assert np.array(frame["base_mocap_from_ur_arm_base_link_inertia"]) == pytest.approx(SESSION["arm_base"], abs=1e-6)


def test_a_punch_file_reads_back_in_the_old_format(tmp_path):
    """The old 4_punch_validation.py's keys: arm and offset at the top, then the takes; rewriting replaces it.

    Each take also keeps the base rigid body's own pose, the applied calibration taken off, and that calibration.
    """
    offset = np.array([0.00058, 0.00158, 0.11054])
    take = punch_take("single", 0, "ur_arm_tool0", np.zeros(6), Pose((1.0, 2.0, 0.0)), Pose((0.4, -1.5, 0.47)), offset,
                      Pose((0.0, 0.0, 0.3)), "abc", 1.4e-3)
    assert take["base_mocap_pose"][0] == pytest.approx([1.0, 2.0, -0.3])
    assert take["applied_calibration"]["sha256"] == "abc" and take["marker_errors"] == {"base": 1.4e-3}
    path = tmp_path / "punch_validation" / "punch_validation_single_20261020_1200.json"
    write_punch(path, [take])
    write_punch(path, [take, take])
    data = json.loads(path.read_text())
    assert {"arm_index", "arm_name", "tool0_link_name", "tool0_from_punch_tip", "takes"} <= data.keys()
    assert data["tool0_from_punch_tip"]["position"] == pytest.approx(list(offset))
    assert read_punch(path)[1]["world_from_punch_tip"]["position"] == pytest.approx([0.4, -1.5, 0.47])
    assert [p.name for p in path.parent.iterdir()] == [path.name]


def test_a_calibration_in_progress_is_never_applied_until_approved(tmp_path):
    """Solve's result in progress leaves the relay on the older session; approving it archives the session's own."""
    root = tmp_path
    older, session = root / SESSIONS_FOLDER / "20991230", root / SESSIONS_FOLDER / "20991231"
    _write_session(session)
    takes, _ = read_takes(session)
    robot = robot_config_from_serial("0805", DATA)
    solution = solve([t for t in takes if t.joint == 0], [t for t in takes if t.joint == 1], robot.model.urdf, "ur_arm")
    older.mkdir()
    write_solution(older, "0805", "ur_arm", robot.mocap_id, solution)
    write_solution(session, "0805", "ur_arm", robot.mocap_id, solution)
    pending = write_solution(session, "0805", "ur_arm", robot.mocap_id, solution, approved=False)
    assert pending == calibration_file(session, "0805", approved=False)

    (session / calibration_file(session, "0805").name).unlink()
    assert newest_session_file(root, "a200-0805") == calibration_file(older, "0805")

    write_solution(session, "0805", "ur_arm", robot.mocap_id, solution)
    approved = approve_calibration(session, "0805")
    assert approved == calibration_file(session, "0805") and not pending.exists()
    assert newest_session_file(root, "a200-0805") == approved
    assert len(list((session / "archive").glob("calibrated_transformation_0805_rhino_*.json"))) == 1


def test_archived_takes_are_not_read(tmp_path):
    """A take moved to its folder's archive/ no longer counts in an offline solve; archiving twice changes nothing."""
    session = tmp_path / "20261013"
    _write_session(session)
    moved = archive(session / "j0" / "calibration_j0_take0.json")
    assert moved == session / "j0" / "archive" / "calibration_j0_take0.json" and archive(moved) == moved
    takes, _ = read_takes(session)
    assert "j0/calibration_j0_take0.json" not in [take.name for take in takes]
