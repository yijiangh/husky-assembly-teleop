"""An arm's reach sphere holds it whatever its joints; robots whose only unknown arms are that far need no asking."""

import numpy as np
import pytest
from repo_paths import DATA

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.world.arm_reach import REACH_MARGIN, arm_sphere, out_of_reach
from scaffolding_assembly_core.geometry import Pose, box_geometry, shape_mesh
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Attachment, Body, Scene
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

pytestmark = pytest.mark.safety

ARM = "ur_arm"
JOINTS = [f"{ARM}_{name}" for name in UR_JOINT_NAMES]


@pytest.fixture(scope="module")
def model():
    """Alice's model, with her tool."""
    return robot_config_from_serial("0804", DATA).model


def robot(model, key, x, **measured) -> RobotObject:
    """A robot at (x, 0), its arm at zero; `measured` as RobotObject takes it (`base_tracked`, `unmeasured`)."""
    return RobotObject(key, model, Pose((x, 0.0, 0.0)), dict.fromkeys(JOINTS, 0.0), **measured)


def test_the_sphere_holds_every_link_and_the_tool_at_any_joints(model):
    """Random joints: every collision point of the arm's links and of its tool lies inside the sphere."""
    alice = robot(model, "robots/alice", 0.0)
    centre, radius = arm_sphere(alice, ARM)
    links = [link for link in model.link_geometry if link.startswith(f"{ARM}_")]
    for joints in np.random.default_rng(0).uniform(-np.pi, np.pi, size=(30, 6)):
        values = dict(zip(JOINTS, joints))
        placed = [(link_pose(model.urdf, alice.base, values, link), model.link_geometry[link]) for link in links]
        placed += [(link_pose(model.urdf, alice.base, values, flange), tool.geometry)
                   for flange, tool in model.tools.items()]
        for pose, geometry in placed:
            matrix = pose.matrix()
            for shape in geometry.collision:
                points = shape_mesh(shape).vertices @ matrix[:3, :3].T + matrix[:3, 3]
                assert np.max(np.linalg.norm(points - centre, axis=1)) <= radius + 1e-9


def test_a_body_held_on_the_flange_grows_the_sphere(model):
    """A 1 m bar held on tool0 reaches further than the arm alone."""
    alice = robot(model, "robots/alice", 0.0)
    bar = Body("bars/b", box_geometry((0.03, 0.03, 1.0)), Attachment(alice.id, f"{ARM}_tool0", Pose((0, 0, 0.5))))
    assert arm_sphere(alice, ARM, {bar.id: bar})[1] > arm_sphere(alice, ARM)[1] + 0.5


def test_only_a_tracked_robot_whose_unknown_arms_are_beyond_reach_needs_no_asking(model):
    """Far with its base tracked: out of reach; closer, or its base lost, or nothing unknown: not."""
    _centre, radius = arm_sphere(robot(model, "robots/alice", 0.0), ARM)
    beyond = 2 * radius + REACH_MARGIN + 0.01
    arms = frozenset(JOINTS)

    def far(**belle) -> frozenset[str]:
        scene = Scene(robots={"robots/alice": robot(model, "robots/alice", 0.0),
                              "robots/belle": robot(model, "robots/belle", **belle)})
        return out_of_reach(scene, "robots/alice", [ARM])

    assert far(x=beyond, unmeasured=arms) == {"robots/belle"}
    assert far(x=beyond - 0.05, unmeasured=arms) == frozenset()
    assert far(x=10.0, unmeasured=arms, base_tracked=False) == frozenset()
    assert far(x=1.0) == frozenset(), "measured: not unknown at all"
