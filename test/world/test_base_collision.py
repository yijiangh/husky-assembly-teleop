"""Tests for the shared base collision world: what a robot holds moves with it and is checked with it."""

import numpy as np
import pybullet as p
import pytest

from repo_paths import DATA
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.world.base_collision import BaseCollisionWorld, held_bodies
from scaffolding_assembly_core.geometry import Pose, box_geometry
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Attachment, Body, Scene, world_poses

#: A post the robot passes 1 m to its left, clear of it but not of a 2.4 m bar held across it.
POST = Body("test/post", box_geometry((0.1, 0.1, 2.0)), Pose((1.0, 1.1, 1.0)), label="post")


def held_bar(own: str, z: float = 1.0) -> Body:
    """A 2.4 m bar across the robot, held at its base, `z` metres up; not allowed to touch it."""
    return Body("test/bar", box_geometry((0.1, 2.4, 0.1)), Attachment(own, None, Pose((0.0, 0.0, z))), label="bar")


def scene(config, bodies=()) -> Scene:
    """Alice at the origin, arm at zero, with `bodies`; attached ones posed by her kinematics."""
    own = robot_id(config.serial)
    robots = {own: RobotObject(own, config.model, Pose(), {})}
    bodies = {body.id: body for body in bodies}
    return Scene(bodies=bodies, robots=robots, world_poses=world_poses(
        bodies, robots, lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link)))


@pytest.fixture(scope="module")
def config():
    """Alice (0804)."""
    return robot_config_from_serial("0804", DATA)


@pytest.fixture(scope="module")
def world():
    """One world for the module; each test syncs the snapshot it needs."""
    world = BaseCollisionWorld()
    yield world
    world.close()


def test_held_bodies_follow_attachments_to_the_robot(config):
    """Attached to the robot, or to a body it holds; not what is attached to something else."""
    own = robot_id(config.serial)
    on_bar = Body("test/on_bar", box_geometry((0.1, 0.1, 0.1)), Attachment("test/bar", None, Pose()))
    elsewhere = Body("test/elsewhere", box_geometry((0.1, 0.1, 0.1)), Attachment("test/post", None, Pose()))
    assert held_bodies(scene(config, [POST, held_bar(own), on_bar, elsewhere]), own) == ["test/bar", "test/on_bar"]


def test_held_bar_moves_with_the_robot(world, config):
    """The robot alone passes the post; holding the bar it hits it there, and the bar is back in place after a sync."""
    own = robot_id(config.serial)
    world.sync(scene(config, [POST]))
    assert world.hit_by(own, (1.0, 0.0, 0.0), 0.0) is None
    holding = scene(config, [POST, held_bar(own)])
    world.sync(holding)
    assert world.hit_by(own, (1.0, 0.0, 0.0), 0.0) == "post"
    assert world.hit_by(own, (1.0, 0.0, np.pi / 2), 0.0) is None
    world.sync(holding)
    bar = world.mirror.body_ids("test/bar")[0]
    position = p.getBasePositionAndOrientation(bar, physicsClientId=world.mirror.client_id)[0]
    np.testing.assert_allclose(position, holding.world_poses["test/bar"].position, atol=1e-9)


def test_contact_with_what_the_robot_holds_is_left_out(world, config):
    """A bar held through the chassis never blocks its own robot: driving never changes that contact."""
    own = robot_id(config.serial)
    world.sync(scene(config, [held_bar(own, z=0.3)]))
    assert world.hit_by(own, (0.0, 0.0, 0.0), 0.0) is None


def test_a_hit_at_a_pose_has_the_robot_and_what_it_holds_there(world, config):
    """Holding the bar 1 m on, she hits the post: the hit's scene has her and the bar there, the bar's side first."""
    own = robot_id(config.serial)
    world.sync(scene(config, [POST, held_bar(own)]))
    hit = world.collision_hit(own, 0.0, "target", (1.0, 0.0, 0.0))
    assert hit.where == "target" and hit.pairs == (("test/bar", "test/post"),)
    np.testing.assert_allclose(hit.scene.robots[own].base.position, (1.0, 0.0, 0.0), atol=1e-9)
    np.testing.assert_allclose(hit.scene.world_poses["test/bar"].position, (1.0, 0.0, 1.0), atol=1e-9)
    assert world.snapshot.robots[own].base.position == (0.0, 0.0, 0.0)
    assert world.collision_hit(own, 0.0, "start", (0.0, 0.0, 0.0)) is None
