"""Tests for the arm planner's search (plugins/arm_planner/planner.py) in a compas_fab world."""

from __future__ import annotations

import threading

import numpy as np
import pybullet as p
import pytest

from repo_paths import DATA
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugins.arm_planner.planner import (ArmPath, ArmPlanningWorld, _extend, arm_joint_names,
                                                               path_hit, plan_arm)
from scaffolding_assembly_core.geometry import box_geometry
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Attachment, Body, Scene
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.geometry import Pose

ALICE = "0804"
ARM = "ur_arm"
NAMES = arm_joint_names(ARM)


@pytest.fixture(scope="module")
def config():
    """Alice's config, loaded once."""
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


@pytest.mark.slow
def test_a_given_path_is_checked_at_its_waypoints_and_between_them(world, config):
    """Clear without a box; a box on the swing hits the move to the goal, a box at the goal the goal itself."""
    start, goal = start_and_goal(config)
    assert path_hit(world, world.sync(snapshot(config, start), ALICE), ARM, np.array([start, goal])) is None
    for joints, path, where in (((start + goal) / 2, [start, goal], "the move to waypoint 2"),
                                (goal, [goal], "waypoint 1")):
        box = Body("t/box", box_geometry((0.15, 0.15, 0.15)), tool_at(world, config, joints), label="the box")
        hit = path_hit(world, world.sync(snapshot(config, start, (box,)), ALICE), ARM, np.array(path))
        assert hit is not None and hit.message.startswith(where) and "the box" in hit.message, hit


@pytest.mark.slow
def test_a_hit_lists_every_pair_that_blocks_it_with_its_contact(world, config):
    """A box at the goal: the arm side comes first and the contact overlaps; a box the arm never moves is not listed."""
    start, goal = start_and_goal(config)
    tool = tool_at(world, config, goal)
    box = Body("t/box", box_geometry((0.15, 0.15, 0.15)), tool, label="the box")
    static = Body("t/static", box_geometry((0.1, 0.1, 0.1)), Pose.from_arrays((0.0, 0.0, 0.25), (0, 0, 0, 1)))
    hit = path_hit(world, world.sync(snapshot(config, start, (box, static)), ALICE), ARM, np.array([goal]))

    assert hit.where == "waypoint 1" and hit.robot == robot_id(ALICE)
    assert all(b == "t/box" and a.startswith(f"{robot_id(ALICE)}/") or a.startswith("tools/") for a, b in hit.pairs)
    assert len(hit.contacts) == len(hit.pairs) and all(contact is not None for contact in hit.contacts)
    first = hit.contacts[0]
    assert first.pair == hit.pairs[0] and first.depth < 0.0
    assert np.linalg.norm(np.subtract(first.point_b, tool.position)) < 0.15
    assert hit.scene.robots[robot_id(ALICE)].joints[NAMES[0]] == pytest.approx(goal[0])


@pytest.mark.slow
def test_a_start_in_collision_lists_every_pair(world, config):
    """The start is checked in full: a box at the chassis blocks it though the arm never moves it."""
    start, goal = start_and_goal(config)
    static = Body("t/static", box_geometry((0.1, 0.1, 0.1)), Pose.from_arrays((0.0, 0.0, 0.25), (0, 0, 0, 1)))
    result = plan_arm(world, world.sync(snapshot(config, start, (static,)), ALICE), ARM, goal, threading.Event())
    assert result.hit is not None and result.hit.where == "start" and result.reason == result.hit.message
    assert any("t/static" in pair for pair in result.hit.pairs)


@pytest.mark.slow
def test_a_hit_moves_the_held_body_with_the_arm(world, config):
    """In the hit's scene, a body held on tool0 sits at the tool at the target, not where the snapshot had it."""
    start, goal = start_and_goal(config)
    held = Body("t/held", box_geometry((0.05, 0.05, 0.05)), Attachment(robot_id(ALICE), "ur_arm_tool0", Pose()),
                touches=(robot_id(ALICE),), label="the held part")
    box = Body("t/box", box_geometry((0.05, 0.05, 0.05)), tool_at(world, config, goal), touches=(robot_id(ALICE),))
    scene = snapshot(config, start, (held, box))
    result = plan_arm(world, world.sync(scene, ALICE), ARM, goal, threading.Event())
    assert result.hit.pairs == (("t/held", "t/box"),)
    moved = result.hit.scene.world_poses["t/held"].position
    np.testing.assert_allclose(moved, tool_at(world, config, goal).position, atol=1e-6)
    assert scene.robots[robot_id(ALICE)].joints[NAMES[0]] == pytest.approx(start[0])


@pytest.mark.slow
def test_a_hit_on_another_robot_names_the_link_touched(world, config):
    """Another robot is one body to the check; the contact narrows it to the link the arm touched."""
    start, goal = start_and_goal(config)
    belle = robot_config_from_serial("0805", DATA)
    # * Her chassis sits about 0.26 to 0.36 m above her root: lowered so that it surrounds Alice's tool.
    root = Pose.from_arrays(np.subtract(tool_at(world, config, goal).position, (0.0, 0.0, 0.3)), (0, 0, 0, 1))
    other = RobotObject(robot_id("0805"), belle.model, root, dict(zip(NAMES, belle.arms[0].stow_joints)))
    scene = snapshot(config, start)
    scene.robots[other.id] = other
    hit = path_hit(world, world.sync(scene, ALICE), ARM, np.array([goal]))
    assert hit is not None and hit.pairs[0][1] == other.id
    touched = hit.contacts[0].pair[1]
    assert touched.startswith(f"{other.id}/") or touched in {tool.id for tool in belle.model.tools.values()}, touched
