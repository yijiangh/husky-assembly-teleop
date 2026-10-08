"""Tests for the base planner's RRT, with a second robot and a scene body in the way."""

import threading
from pathlib import Path

import numpy as np
import pytest

from husky_assembly_teleop.config import robot_config_from_serial
from bar_assembly_core.design_io.geometry import box_geometry
from husky_assembly_teleop.plugins.base_planner.planner import PlanningWorld, plan_birrt
from bar_assembly_core.robot import RobotObject
from bar_assembly_core.scene import Body, SceneSnapshot, robot_id
from bar_assembly_core.design_io.pose import Pose

DATA = Path(__file__).resolve().parent.parent / "data"

# ? 2 m wide: at 3 m the detour barely fits the sampled area (SAMPLE_PADDING) next to Cindy; half the searches failed.
# ? 2 m wide: at 3 m the detour barely fits the sampled area (SAMPLE_PADDING) next to Cindy; half the searches failed.
WALL = Body("test/wall", box_geometry((2.0, 0.3, 1.0)), Pose((0.0, -2.5, 0.5)), label="wall")


def _snapshot(robots, bodies=()) -> SceneSnapshot:
    """Alice at the origin, Cindy parked 2 m ahead of her, plus `bodies`."""
    alice, cindy = robots
    bases = {alice.serial: Pose(), cindy.serial: Pose((2.0, 0.0, 0.0))}
    entries = {robot_id(config.serial): RobotObject(robot_id(config.serial), config.model, bases[config.serial], {})
               for config in robots}
    return SceneSnapshot(bodies={body.id: body for body in bodies},
                         world_poses={body.id: body.placement for body in bodies}, robots=entries)


@pytest.fixture(scope="module")
def robots():
    """Alice (0804) and Cindy (0806)."""
    return tuple(robot_config_from_serial(serial, DATA) for serial in ("0804", "0806"))


@pytest.fixture(scope="module")
def alice(robots):
    """Alice's robot id, the robot every test plans for."""
    return robot_id(robots[0].serial)


@pytest.fixture(scope="module")
def world(robots):
    """One planning world for the module; each test syncs the snapshot it needs."""
    world = PlanningWorld()
    yield world
    world.close()


@pytest.mark.slow
def test_goes_around_a_robot_in_the_way(world, robots, alice):
    """The straight line runs through Cindy, so the path must detour, and never touch her."""
    world.sync(_snapshot(robots))
    result = plan_birrt(world, alice, (0.0, 0.0, 0.0), (4.0, 0.0, 0.0), threading.Event())
    assert result.path is not None, result.reason
    assert not result.direct
    for t in np.linspace(0.0, result.path.duration, 1000):
        assert world.hit_by(alice, result.path.sample(t)) is None
    np.testing.assert_allclose(result.path.goal[:2], (4.0, 0.0), atol=1e-6)


def test_free_line_is_direct(world, robots, alice):
    """Nothing in the way: the straight move, no search."""
    world.sync(_snapshot(robots))
    result = plan_birrt(world, alice, (0.0, 0.0, 0.0), (0.0, -2.0, 1.0), threading.Event())
    assert result.path is not None and result.direct


def test_target_inside_a_robot_is_refused_with_a_reason(world, robots, alice):
    """A target in collision is reported as such, without searching."""
    world.sync(_snapshot(robots))
    result = plan_birrt(world, alice, (0.0, 0.0, 0.0), (2.0, 0.2, 0.0), threading.Event())
    assert result.path is None and "target" in result.reason


@pytest.mark.slow
def test_goes_around_a_body(world, robots, alice):
    """A wall across the straight line: the path detours, never touches it, and a blocked target names it."""
    world.sync(_snapshot(robots, [WALL]))
    result = plan_birrt(world, alice, (0.0, -1.0, -1.57), (0.0, -4.0, -1.57), threading.Event())
    assert result.path is not None, result.reason
    assert not result.direct
    for t in np.linspace(0.0, result.path.duration, 1000):
        assert world.hit_by(alice, result.path.sample(t)) is None
    blocked = plan_birrt(world, alice, (0.0, -1.0, 0.0), (0.0, -2.5, 0.0), threading.Event())
    assert blocked.path is None and "wall" in blocked.reason


def test_body_allowed_to_touch_is_ignored(world, robots, alice):
    """A body that lists the robot in `touches` doesn't block it (e.g. the ground under the wheels)."""
    pad = Body("test/pad", box_geometry((1.5, 1.5, 0.02)), Pose((0.0, 0.0, 0.0)), touches=(alice,))
    world.sync(_snapshot(robots, [pad]))
    assert world.hit_by(alice, (0.0, 0.0, 0.0)) is None


def test_sync_puts_the_searched_robot_back(world, robots, alice):
    """A search moves the robot around; the next sync must restore its snapshot pose."""
    world.sync(_snapshot(robots))
    assert world.hit_by(alice, (2.0, 0.0, 0.0)) is not None  # moved into Cindy
    world.sync(_snapshot(robots))
    assert world.mirror.collisions(alice, 0.0) == []
