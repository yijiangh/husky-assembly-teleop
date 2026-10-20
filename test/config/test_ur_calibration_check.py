"""The monitor applies each arm's UR calibration from crl-husky to its URDF: nothing if up to date, loudly if not."""

from __future__ import annotations

import shutil
from pathlib import Path

import numpy as np
import pytest
import yaml
from repo_paths import DATA
from crl_husky import config_resolver
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.config import _ROBOTS_BY_SERIAL, robot_config_from_serial
from husky_assembly_teleop.tool_urdfs import absolute_mesh_urdf
from scaffolding_assembly_core.urdf import UR_CALIBRATION_JOINTS, load_urdf

pytestmark = pytest.mark.safety

#: crl-husky's robot configs, copied for each test so calibration files can be added.
ROBOTS = Path(config_resolver.__file__).resolve().parent.parent / "config" / "robots"


@pytest.fixture
def robots_dir(tmp_path, monkeypatch):
    """A copy of crl-husky's config/robots that the resolver reads instead."""
    folder = shutil.copytree(ROBOTS, tmp_path / "robots")
    monkeypatch.setenv("CRL_HUSKY_ROBOTS_CONFIG_DIR", str(folder))
    config_resolver.reload_robot_configs()
    yield folder
    monkeypatch.delenv("CRL_HUSKY_ROBOTS_CONFIG_DIR")
    config_resolver.reload_robot_configs()


def _calibration_of(urdf: Path, arm: str) -> dict:
    """The UR calibration a URDF's arm joints encode, as the driver's calibration.yaml holds it."""
    joints = {joint.name: joint for joint in load_urdf(urdf).robot.joints}
    kinematics = {}
    for section, suffix in UR_CALIBRATION_JOINTS.items():
        origin = joints[f"{arm}_{suffix}"].origin
        rpy = Rotation.from_matrix(origin[:3, :3]).as_euler("xyz")
        kinematics[section] = dict(zip(("x", "y", "z", "roll", "pitch", "yaw"), map(float, [*origin[:3, 3], *rpy])))
    return {"kinematics": {**kinematics, "hash": "calib_test"}}


def test_the_urdfs_have_the_robots_calibrations():
    """Every configured robot's URDF already has the UR calibrations in crl-husky: the monitor changes nothing."""
    for serial in _ROBOTS_BY_SERIAL:
        checks = robot_config_from_serial(serial, DATA).ur_calibration
        assert checks and all(c.file is not None for c in checks), f"{serial}: calibration files missing in crl-husky"
        assert not any(c.out_of_date for c in checks), "\n".join(c.describe() for c in checks if c.out_of_date)


def test_without_calibration_files_nothing_is_checked(robots_dir):
    """A robot whose files are not in crl-husky reports every arm unchecked and uses the URDF as it is."""
    for file in (robots_dir / "0806").glob("ur_calibration*.yaml"):
        file.unlink()
    config = robot_config_from_serial("0806", DATA)
    assert [(c.arm, c.file, c.changes) for c in config.ur_calibration] == [("left_ur_arm", None, ()),
                                                                           ("right_ur_arm", None, ())]
    assert "not checked" in config.ur_calibration[0].describe()


def test_the_calibration_the_urdf_has_changes_nothing(robots_dir):
    """The URDF's own calibration, as the robot's file: a no-op, reported as matching."""
    urdf = DATA / "husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf"
    for side in ("left", "right"):
        (robots_dir / "0806" / f"ur_calibration_{side}.yaml").write_text(
            yaml.safe_dump(_calibration_of(urdf, f"{side}_ur_arm")))
    config = robot_config_from_serial("0806", DATA)
    assert all(c.file is not None and not c.out_of_date for c in config.ur_calibration)
    # * Not rewritten: the copy is exactly the plain mesh-path copy.
    assert config.model.urdf.read_text() == absolute_mesh_urdf(urdf, robots_dir / "plain.urdf").read_text()


def test_an_out_of_date_urdf_is_reported_and_the_calibration_used(robots_dir):
    """A robot whose calibration moved the elbow by 2 mm: the arm says so, and the model has the robot's values."""
    urdf = DATA / "husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Alice_Calibrated.urdf"
    calibration = _calibration_of(urdf, "ur_arm")
    calibration["kinematics"]["forearm"]["x"] -= 0.002
    (robots_dir / "0804" / "ur_calibration.yaml").write_text(yaml.safe_dump(calibration))
    config = robot_config_from_serial("0804", DATA)
    (check,) = config.ur_calibration
    assert check.out_of_date and [c.joint for c in check.changes] == ["ur_arm_elbow_joint"]
    assert check.changes[0].distance == pytest.approx(0.002)
    assert "OUT OF DATE" in check.describe() and "2.00 mm" in check.describe()
    elbow = next(j for j in load_urdf(config.model.urdf).robot.joints if j.name == "ur_arm_elbow_joint")
    np.testing.assert_allclose(elbow.origin[0, 3], calibration["kinematics"]["forearm"]["x"])
