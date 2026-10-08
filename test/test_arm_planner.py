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
from bar_assembly_core.geometry import box_geometry
from bar_assembly_core.robot import RobotObject
from bar_assembly_core.scene import Attachment, Body, Scene
from bar_assembly_core.ids import robot_id
from bar_assembly_core.geometry import Pose

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


def snapshot(config, start, bodies=()) -> Scene:
    """Alice at the origin with her arm at `start`, and some bodies."""
    robot = RobotObject(robot_id(ALICE), config.model, Pose(), dict(zip(NAMES, start)))
    poses = {body.id: body.placement if isinstance(body.placement, Pose) else body.placement.offset
             for body in bodies}
    return Scene(bodies={body.id: body for body in bodies}, world_poses=poses, robots={robot.id: robot})


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


@pytest.mark.slow
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
    steps = [q for q1, q2 in zip(path.points[:-1], path.points[1:]) for q in _extend(q1, q2)]
    for q in steps[::3]:
        assert mirror.collisions(dict(zip(NAMES, q))) == [], f"collides at {q}"


def test_target_in_collision_is_refused_with_a_reason(world, config):
    """A goal with the tool inside a box is refused, naming the box by its label."""
    start, goal = start_and_goal(config)
    box = Body("t/box", box_geometry((0.15, 0.15, 0.15)), tool_at(world, config, goal), label="the box")
    mirror = world.sync(snapshot(config, start, (box,)), ALICE)
    result = plan_arm(world, mirror, ARM, goal, threading.Event())
    assert result.path is None and result.reason.startswith("target is in collision") and "the box" in result.reason


def test_held_body_counts(world, config):
    """A body held on tool0 that would hit a box at the target is refused, though the arm may touch the box."""
    start, goal = start_and_goal(config)
    held = Body("t/held", box_geometry((0.05, 0.05, 0.05)), Attachment("robots/0804", "ur_arm_tool0", Pose()),
                touches=("robots/0804",), label="the held part")
    box = Body("t/box", box_geometry((0.05, 0.05, 0.05)), tool_at(world, config, goal), touches=("robots/0804",),
               label="the box")
    mirror = world.sync(snapshot(config, start, (held, box)), ALICE)
    result = plan_arm(world, mirror, ARM, goal, threading.Event())
    assert result.path is None and result.reason == "target is in collision: the held part with the box"


def test_start_in_collision_is_refused(world, config):
    """A start with the tool inside a box is refused before searching."""
    start, goal = start_and_goal(config)
    box = Body("t/box", box_geometry((0.15, 0.15, 0.15)), tool_at(world, config, start))
    mirror = world.sync(snapshot(config, start, (box,)), ALICE)
    result = plan_arm(world, mirror, ARM, goal, threading.Event())
    assert result.path is None and result.reason.startswith("start is in collision")


def test_path_timing():
    """Segments are timed by their largest joint change; sample interpolates and clamps."""
    path = ArmPath.at_preview_speed(NAMES, np.array([[0.0] * 6, [1.0, 0.5, 0, 0, 0, 0], [1.0, 0.5, 0, 0, 0, 2.0]]))
    np.testing.assert_allclose(np.diff(path.times) * np.radians(20.0), [1.0, 2.0])
    np.testing.assert_allclose(path.sample(path.times[1] / 2)[:2], [0.5, 0.25])
    np.testing.assert_allclose(path.sample(1e9), path.goal)


def test_window_without_display_raises(config, monkeypatch):
    """Without an X display, asking for the window raises and leaves it closed; closing it is always fine."""
    monkeypatch.delenv("DISPLAY", raising=False)
    world = ArmPlanningWorld()
    try:
        start, _ = start_and_goal(config)
        with pytest.raises(RuntimeError, match="display"):
            world.set_gui(True, snapshot(config, start), ALICE)
        assert not world.gui
        world.set_gui(False, snapshot(config, start), ALICE)
    finally:
        world.close()
