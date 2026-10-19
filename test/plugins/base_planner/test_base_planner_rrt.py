"""Tests for the base planner's RRT (algorithms.rrt_arcs), with a second robot and a scene body in the way."""

import threading

import numpy as np
import pytest

from repo_paths import DATA
from husky_assembly_teleop.config import robot_config_from_serial
from scaffolding_assembly_core.geometry import box_geometry
from husky_assembly_teleop.plugins.base_planner.algorithms import ALGORITHMS, Debug
from husky_assembly_teleop.plugins.base_planner.algorithms.base import LEAD
from husky_assembly_teleop.plugins.base_planner.algorithms.rrt_arcs import plan
from husky_assembly_teleop.plugins.base_planner.world import PlanningWorld, stowed
from husky_assembly_teleop.shared.planning.panel import with_guesses
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Body, Scene
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES
from scaffolding_assembly_core.geometry import Pose


# ? 2 m wide: at 3 m the detour barely fits the sampled area (SAMPLE_PADDING) next to Cindy; half the searches failed.
# ? 2 m wide: at 3 m the detour barely fits the sampled area (SAMPLE_PADDING) next to Cindy; half the searches failed.
WALL = Body("test/wall", box_geometry((2.0, 0.3, 1.0)), Pose((0.0, -2.5, 0.5)), label="wall")


def _snapshot(robots, bodies=()) -> Scene:
    """Alice at the origin, Cindy parked 2 m ahead of her, plus `bodies`."""
    alice, cindy = robots
    bases = {alice.serial: Pose(), cindy.serial: Pose((2.0, 0.0, 0.0))}
    entries = {robot_id(config.serial): RobotObject(robot_id(config.serial), config.model, bases[config.serial], {})
               for config in robots}
    return Scene(bodies={body.id: body for body in bodies},
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
    result = plan(world, alice, (0.0, 0.0, 0.0), (4.0, 0.0, 0.0), threading.Event())
    assert result.path is not None, result.reason
    assert not result.direct
    for t in np.linspace(0.0, result.path.duration, 1000):
        assert world.hit_by(alice, result.path.sample(t)) is None
    np.testing.assert_allclose(result.path.goal[:2], (4.0, 0.0), atol=1e-6)


def test_free_line_is_direct(world, robots, alice):
    """Nothing in the way: the straight move, no search."""
    world.sync(_snapshot(robots))
    result = plan(world, alice, (0.0, 0.0, 0.0), (0.0, -2.0, 1.0), threading.Event())
    assert result.path is not None and result.direct


def test_target_inside_a_robot_is_refused_with_a_reason(world, robots, alice):
    """A target in collision is reported as such, without searching."""
    world.sync(_snapshot(robots))
    result = plan(world, alice, (0.0, 0.0, 0.0), (2.0, 0.2, 0.0), threading.Event())
    assert result.path is None and "target" in result.reason


@pytest.mark.slow
def test_goes_around_a_body(world, robots, alice):
    """A wall across the straight line: the path detours, never touches it, and a blocked target names it."""
    world.sync(_snapshot(robots, [WALL]))
    result = plan(world, alice, (0.0, -1.0, -1.57), (0.0, -4.0, -1.57), threading.Event())
    assert result.path is not None, result.reason
    assert not result.direct
    for t in np.linspace(0.0, result.path.duration, 1000):
        assert world.hit_by(alice, result.path.sample(t)) is None
    blocked = plan(world, alice, (0.0, -1.0, 0.0), (0.0, -2.5, 0.0), threading.Event())
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


def test_untracked_robot_is_planned_from_a_guess(world, robots, alice):
    """An untracked base and unknown joints are guessed, with the problems named; the shared snapshot is untouched."""
    snapshot = _snapshot(robots)
    robot = snapshot.robots[alice]
    robot.base_tracked, robot.unmeasured = False, frozenset({"left_ur_arm_elbow_joint"})
    guessed_snapshot, guessed = with_guesses(snapshot, alice)
    assert len(guessed) == 2 and snapshot.robots[alice].acting_problems() == guessed
    world.sync(guessed_snapshot)
    assert plan(world, alice, (0.0, 0.0, 0.0), (0.0, -2.0, 1.0), threading.Event()).path is not None
    snapshot.robots[alice].enabled = False
    assert with_guesses(snapshot, alice) == (snapshot, [])


def test_floor_map_sees_a_bar_thinner_than_its_cells(world, robots, alice):
    """A 2 cm bar lying between two rows of cell centres: one ray per cell misses it, several per cell do not."""
    bar = Body("test/bar", box_geometry((1.0, 0.02, 0.02)), Pose((0.0, -1.975, 0.15)), label="bar")
    world.sync(_snapshot(robots, [bar]))
    heights = world.floor_heights(alice, (-1.0, -2.5), 0.05, (41, 21), 0.05)
    xs = -1.0 + 0.05 * np.arange(41)
    along = (xs > -0.45) & (xs < 0.45)
    seen = np.isfinite(heights[along]).any(axis=1)
    assert seen.all()


def test_stowed_puts_the_arm_at_its_stow_pose_and_says_how_far_it_was(robots, alice):
    """Measured joints count towards the distance, unmeasured ones do not; the shared snapshot is untouched."""
    snapshot = _snapshot(robots)
    robot = snapshot.robots[alice]
    stow = {f"ur_arm_{name}": value for name, value in zip(UR_JOINT_NAMES, robots[0].arms[0].stow_joints)}
    live = {**stow, "ur_arm_elbow_joint": 0.5, "ur_arm_wrist_1_joint": 0.0}
    robot.joints, robot.unmeasured = dict(live), frozenset({"ur_arm_wrist_1_joint"})
    stowed_snapshot, away = stowed(snapshot, alice, robots[0])
    assert away == pytest.approx(abs(stow["ur_arm_elbow_joint"] - 0.5))
    assert stowed_snapshot.robots[alice].joints == {**live, **stow}
    assert not stowed_snapshot.robots[alice].unmeasured and robot.joints == live


@pytest.mark.slow
def test_target_beside_a_body_is_reached_driving_straight(world, robots, alice):
    """Facing the wall 5 cm short of it there is no room to turn on the spot: the last LEAD metres drive straight in."""
    world.sync(_snapshot(robots, [WALL]))
    goal = (0.0, -1.5, -np.pi / 2)
    result = plan(world, alice, (-2.0, 0.0, 0.0), goal, threading.Event())
    assert result.path is not None, result.reason
    np.testing.assert_allclose(result.path.goal[:2], goal[:2], atol=1e-6)
    # Distance driven from each waypoint to the target, along the path.
    steps = np.linalg.norm(np.diff(result.path.points[:, :2], axis=0), axis=1)
    to_go = np.concatenate([np.cumsum(steps[::-1])[::-1], [0.0]])
    np.testing.assert_allclose(np.cos(result.path.points[to_go < LEAD - 1e-6, 2] - goal[2]), 1.0, atol=1e-9)


def test_every_algorithm_plans_a_free_line(world, robots, alice):
    """Each algorithm in the dropdown, with its default settings, plans the free straight move; labels differ.

    The path starts exactly where the robot is (else the panel calls the plan stale at once) and ends on the target.
    """
    world.sync(_snapshot(robots))
    assert len({algorithm.label for algorithm in ALGORITHMS}) == len(ALGORITHMS)
    for kind in ALGORITHMS:
        algorithm = kind()
        result = algorithm.plan(world, alice, (0.0, 0.0, 0.0), (0.0, -2.0, 1.0), threading.Event(),
                                algorithm.settings(), Debug())
        assert result.path is not None, f"{algorithm.label}: {result.reason}"
        np.testing.assert_allclose(result.path.start, (0.0, 0.0, 0.0), atol=1e-9, err_msg=algorithm.label)
        np.testing.assert_allclose(result.path.goal[:2], (0.0, -2.0), atol=1e-9, err_msg=algorithm.label)


@pytest.mark.slow
def test_debug_records_the_search(world, robots, alice):
    """With debug on, the detour around Cindy leaves its samples, moves, corners and driven paths, and notes."""
    world.sync(_snapshot(robots))
    debug = Debug(on=True)
    result = plan(world, alice, (0.0, 0.0, 0.0), (4.0, 0.0, 0.0), threading.Event(), debug=debug)
    assert result.path is not None, result.reason
    for name in ("samples", "moves tried, as far as clear", "search's corners", "path as driven, per xICR"):
        assert debug.layers[name].segments or debug.layers[name].points, name
    assert any(text.startswith("search 1") for text in debug.notes)
    off = Debug()
    plan(world, alice, (0.0, 0.0, 0.0), (4.0, 0.0, 0.0), threading.Event(), debug=off)
    assert not off.layers and off.notes, "off, nothing is drawn but the notes are kept"
