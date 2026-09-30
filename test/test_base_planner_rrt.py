"""Tests for the base planner's RRT, with a second robot in the way."""

import threading
from pathlib import Path

import numpy as np
import pybullet as p
import pytest

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugins.base_planner.planner import PlanningWorld, plan_birrt
from husky_assembly_teleop.plugins.obstacles import Box

DATA = Path(__file__).resolve().parent.parent / "data"


@pytest.fixture(scope="module")
def world():
    """Alice at the origin, Cindy parked 2 m ahead of her."""
    robots = tuple(robot_config_from_serial(serial, DATA) for serial in ("0804", "0806"))
    world = PlanningWorld(robots)
    p.resetBasePositionAndOrientation(world.robots[robots[1].serial], (2.0, 0.0, 0.0), (0, 0, 0, 1),
                                      physicsClientId=world.client_id)
    yield world, robots[0].serial
    world.close()


def test_goes_around_a_robot_in_the_way(world):
    """The straight line runs through Cindy, so the path must detour, and never touch her."""
    world, alice = world
    result = plan_birrt(world, alice, (0.0, 0.0, 0.0), (4.0, 0.0, 0.0), threading.Event())
    assert result.path is not None, result.reason
    assert not result.direct
    for t in np.linspace(0.0, result.path.duration, 1000):
        assert world.hit_by(alice, result.path.sample(t)) is None
    np.testing.assert_allclose(result.path.goal[:2], (4.0, 0.0), atol=1e-6)


def test_free_line_is_direct(world):
    """Nothing in the way: the straight move, no search."""
    world, alice = world
    result = plan_birrt(world, alice, (0.0, 0.0, 0.0), (0.0, -2.0, 1.0), threading.Event())
    assert result.path is not None and result.direct


def test_target_inside_a_robot_is_refused_with_a_reason(world):
    """A target in collision is reported as such, without searching."""
    world, alice = world
    result = plan_birrt(world, alice, (0.0, 0.0, 0.0), (2.0, 0.2, 0.0), threading.Event())
    assert result.path is None and "target" in result.reason


def test_goes_around_a_box(world):
    """A wall of a box across the straight line: the path detours and never touches it."""
    world, alice = world
    world.set_boxes((Box(name="wall", center=(0.0, -2.5, 0.5), size=(3.0, 0.3, 1.0)),))
    try:
        result = plan_birrt(world, alice, (0.0, -1.0, -1.57), (0.0, -4.0, -1.57), threading.Event())
        assert result.path is not None, result.reason
        assert not result.direct
        for t in np.linspace(0.0, result.path.duration, 1000):
            assert world.hit_by(alice, result.path.sample(t)) is None
        blocked = plan_birrt(world, alice, (0.0, -1.0, 0.0), (0.0, -2.5, 0.0), threading.Event())
        assert blocked.path is None and "wall" in blocked.reason
    finally:
        world.set_boxes(())
