"""husky_sim's tool truth agrees with the monitor's robot model: the arms' mounts, and the arm turned by offsets."""

import numpy as np
import pytest
from crl_husky.config_resolver import get_robot_model
from crl_husky.sim.tool_truth import ARM_CHAIN, DRIVER_BASE, from_quaternion, mount_from_urdf, true_arm
from repo_paths import DATA
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.config import robot_config_from_serial
from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

pytestmark = pytest.mark.safety


@pytest.mark.parametrize("serial", ["0804", "0805", "0806"])
def test_husky_sims_mounts_match_the_monitors_model(serial):
    """Each arm's base link in base_footprint, from crl-husky's model.yaml URDF, as the monitor's model puts it."""
    config = robot_config_from_serial(serial, DATA)
    urdf = DATA / "husky_urdf" / get_robot_model(serial)["urdf"]
    for arm in config.arms:
        link = f"{arm.name}_base_link"
        model = compose(invert(link_pose(config.model.urdf, Pose(), {}, "base_footprint")),
                        link_pose(config.model.urdf, Pose(), {}, link))
        mount = mount_from_urdf(urdf, link)
        np.testing.assert_allclose(mount[:3, 3], model.position, atol=1e-9)
        np.testing.assert_allclose(mount[:3, :3], Rotation.from_quat(model.orientation).as_matrix(), atol=1e-9)


def _between(urdf, joints: dict, parent: str, child: str) -> np.ndarray:
    """`child` in `parent` as a 4x4 transform, by the model's forward kinematics."""
    pose = compose(invert(link_pose(urdf, Pose(), joints, parent)), link_pose(urdf, Pose(), joints, child))
    return from_quaternion(pose.position, pose.orientation)


def test_encoder_offsets_put_tool0_where_the_offset_joints_would():
    """The driver's links at the reported joints, turned by the offsets, give tool0 at the joints plus the offsets."""
    config = robot_config_from_serial("0805", DATA)
    prefix = f"{config.arms[0].name}_"
    reported = dict(zip((prefix + name for name in UR_JOINT_NAMES), (0.3, -1.2, 1.4, -1.5, 1.6, 0.2)))
    offsets = [0.2, -0.1, 0.15, 0.0, 0.3, -0.5]
    frames = [DRIVER_BASE] + [frame for frame, _ in ARM_CHAIN]
    links = [_between(config.model.urdf, reported, prefix + parent, prefix + child)
             for parent, child in zip(frames, frames[1:])]
    turned = {name: value + np.radians(offset) for (name, value), offset in zip(reported.items(), offsets)}
    expected = _between(config.model.urdf, turned, prefix + DRIVER_BASE, prefix + frames[-1])
    np.testing.assert_allclose(true_arm(links, offsets), expected, atol=1e-9)
    assert np.linalg.norm(true_arm(links, [0.0] * 6)[:3, 3] - expected[:3, 3]) > 1e-3
