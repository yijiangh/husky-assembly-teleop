"""Tests for the arm planner's search (plugins/arm_planner/planner.py) in a compas_fab world."""

from __future__ import annotations

import threading
from pathlib import Path

import numpy as np
import pybullet as p
import pytest

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugins.arm_planner.planner import (ArmPath, ArmPlanningWorld, _extend, arm_joint_names,
                                                               plan_arm)
from husky_assembly_teleop.world.geometry import box_geometry
from husky_assembly_teleop.world.scene import Body, Pose, RobotEntry, SceneSnapshot

DATA = Path(__file__).resolve().parent.parent / "data"
ALICE = "0804"
ARM = "ur_arm"
NAMES = arm_joint_names(ARM)


@pytest.fixture(scope="module")
def config():
    """Alice's stitched config, loaded once."""
    return robot_config_from_serial(ALICE, DATA)


@pytest.fixture(scope="module")
def world():
    """One planning world shared by the tests: loading the models takes seconds."""
    world = ArmPlanningWorld()
    yield world
    world.close()


def snapshot(config, start, bodies=()) -> SceneSnapshot:
    """Alice at the origin with her arm at `start`, and some bodies."""
    robot = RobotEntry(config, Pose(), True, dict(zip(NAMES, start)), frozenset(), None, None)
    return SceneSnapshot(bodies={body.id: body for body in bodies},
                         world_poses={body.id: body.placement for body in bodies}, robots={ALICE: robot})


def start_and_goal(config):
    """Stowed, and the same with the shoulder turned by half a turn."""
    start = np.array(config.arms[0].stow_joints)
    goal = start.copy()
    goal[0] -= np.pi
    return start, goal


def tool_at(world, config, joints) -> Pose:
    """Where tool0 is with the arm at `joints` (checked in the planning world)."""
    mirror = world.sync(snapshot(config, joints), ALICE)
    client = mirror.client
    state = p.getLinkState(client.robot_puid, client.robot_link_puids["ur_arm_tool0"],
                           physicsClientId=client.client_id)
    return Pose.from_arrays(state[4], (0.0, 0.0, 0.0, 1.0))


def test_path_goes_around_a_box(world, config):
    """A box where the straight swing passes is avoided; every step of the path is clear by compas_fab's check."""
    start, goal = start_and_goal(config)
    middle = (start + goal) / 2
    box = Body("t/box", box_geometry((0.15, 0.15, 0.15)), tool_at(world, config, middle), label="the box")
    mirror = world.sync(snapshot(config, start, (box,)), ALICE)
    result = plan_arm(world, mirror, ARM, goal, threading.Event())

    assert result.path is not None, result.reason
    assert not result.direct
    path = result.path
    np.testing.assert_allclose(path.start, start)
    np.testing.assert_allclose(path.goal, goal)
    steps = [q for q1, q2 in zip(path.waypoints[:-1], path.waypoints[1:]) for q in _extend(q1, q2)]
    for q in steps[::3]:
        assert mirror.collisions(dict(zip(NAMES, q))) == [], f"collides at {q}"


def test_target_in_collision_is_refused_with_a_reason(world, config):
    """A goal with the tool inside a box is refused, naming the box by its label."""
    start, goal = start_and_goal(config)
    box = Body("t/box", box_geometry((0.15, 0.15, 0.15)), tool_at(world, config, goal), label="the box")
    mirror = world.sync(snapshot(config, start, (box,)), ALICE)
    result = plan_arm(world, mirror, ARM, goal, threading.Event())
    assert result.path is None and result.reason.startswith("target is in collision") and "the box" in result.reason


def test_start_in_collision_is_refused(world, config):
    """A start with the tool inside a box is refused before searching."""
    start, goal = start_and_goal(config)
    box = Body("t/box", box_geometry((0.15, 0.15, 0.15)), tool_at(world, config, start))
    mirror = world.sync(snapshot(config, start, (box,)), ALICE)
    result = plan_arm(world, mirror, ARM, goal, threading.Event())
    assert result.path is None and result.reason.startswith("start is in collision")


def test_path_timing():
    """Segments are timed by their largest joint change; sample interpolates and clamps."""
    path = ArmPath(NAMES, np.array([[0.0] * 6, [1.0, 0.5, 0, 0, 0, 0], [1.0, 0.5, 0, 0, 0, 2.0]]))
    np.testing.assert_allclose(np.diff(path.times) * np.radians(20.0), [1.0, 2.0])
    np.testing.assert_allclose(path.sample(path.times[1] / 2)[:2], [0.5, 0.25])
    np.testing.assert_allclose(path.sample(1e9), path.goal)


def test_window_before_any_plan_builds_the_world(config, monkeypatch):
    """Ticking the window box first builds the robot's world from the snapshot; without a display it says why."""
    monkeypatch.delenv("DISPLAY", raising=False)
    world = ArmPlanningWorld()
    try:
        start, _ = start_and_goal(config)
        is_open, problem = world.set_gui(True, snapshot(config, start), ALICE)
        assert not is_open and "no display" in problem
        assert world.serial == ALICE and world._mirrors[ALICE].state is not None
        assert world.set_gui(False, snapshot(config, start), ALICE) == (False, "")
    finally:
        world.close()
