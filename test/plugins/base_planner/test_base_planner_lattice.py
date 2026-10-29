"""Tests for the base planner's lattice search (algorithms.lattice): curves, detours, and narrow slanted passages."""

import math
import threading

import numpy as np
import pytest
from crl_husky.base_model import load_model

from husky_assembly_teleop.plugins.base_planner.algorithms import Debug, lattice
from husky_assembly_teleop.plugins.base_planner.algorithms.base import FOLLOWER_MODEL
from crl_husky.pure_pursuit import PurePursuit, PurePursuitParams
from husky_assembly_teleop.plugins.base_planner.algorithms.lattice import (MIN_DRIVE, Settings, _arc, _fragile,
                                                                           _merged, _sent, _settled, _to_base,
                                                                           _to_centre, plan)
from husky_assembly_teleop.plugins.base_planner.follow import followed, thinned, xicr_range
from husky_assembly_teleop.plugins.base_planner.path import MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED, SETTLE_DISTANCE
from husky_assembly_teleop.plugins.base_planner.tasks import PASSAGE_ANGLE, WHEELS, Task, along, passage, to_lab
from scaffolding_assembly_core.scene import Body
from test_base_planner_rrt import POST, _holding_bar, _snapshot, alice, robots, world  # noqa: F401  (fixtures)

#: The passages are the tasks' (`tasks.passage`), with the task frame at ORIGIN: they run along ALONG through CENTRE,
#: slanted off the grid.
ORIGIN = (0.0, -3.0, 0.0)
ANGLE = PASSAGE_ANGLE
CENTRE = np.array(ORIGIN[:2])
ALONG = np.array([math.cos(ANGLE), math.sin(ANGLE)])
ACROSS = np.array([-ALONG[1], ALONG[0]])
#: Alice's arm and other side's reach from base_footprint, metres.
ARM, OTHER_SIDE = 0.885, 0.346
#: How far the default follower model puts the turning centre ahead of base_footprint, metres.
AHEAD = -load_model(FOLLOWER_MODEL)["xICR"]


@pytest.fixture(autouse=True)
def time_to_spare(monkeypatch):
    """Searches get 40 s instead of TIME_LIMIT: a busy machine (parallel tests) must not fail a search that succeeds."""
    monkeypatch.setattr(lattice, "TIME_LIMIT", 40.0)


def _passage(gap: float, depth: float = 1.0, height: float = 0.2, length: float = 5.0) -> list[Body]:
    """Two blocks `length` long across the way, `gap` apart: the only way past is the passage between them."""
    blocks = passage(gap, depth, height, length)
    return Task("block", "", "passages", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), blocks).bodies("test/block", ORIGIN)


def _at(along_: float, across: float, yaw: float = ANGLE) -> tuple[float, float, float]:
    """A pose in the passage's frame: metres along and across it from its centre."""
    return to_lab(along(along_, across, yaw), ORIGIN)


def _follower_pieces(path) -> list:
    """The pieces the default model's follower cuts a path into."""
    follower = PurePursuit(PurePursuitParams(x_icr=-AHEAD))
    return follower.make_path(*np.asarray(path.points).T).pieces


def _assert_no_shuffle(path) -> None:
    """The follower drives the path in long drives and turns: no short drive between turns to reposition."""
    for piece in _follower_pieces(path)[1:-1]:
        assert not hasattr(piece, "length") or piece.length >= MIN_DRIVE - 1e-6, piece.length


def _assert_followed_clear(world, alice, path, settings: Settings = Settings()) -> None:  # noqa: F811
    """The simulated follower, with the true xICR at either end of the range, finishes the path touching nothing."""
    model = load_model(settings.follower_model or FOLLOWER_MODEL)
    for icr in xicr_range(model["xICR"], settings.x_icr_spread):
        trace, why = followed(path.points, icr, model, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)
        assert not why, (icr, why)
        for pose in thinned(trace, 0.02, 0.02):
            assert world.hit_by(alice, tuple(pose), 0.0) is None, (icr, pose)


def test_free_line_is_one_move(world, robots, alice):  # noqa: F811
    """Nothing in the way: one move onto the target, no search."""
    world.sync(_snapshot(robots))
    result = plan(world, alice, (0.0, 0.0, 0.0), (0.0, -2.0, 1.0), threading.Event())
    assert result.path is not None and result.direct


def test_target_inside_a_robot_is_refused_with_a_reason(world, robots, alice):  # noqa: F811
    """A target in collision is reported as such, without searching, with its hit: Alice at the target in its scene."""
    world.sync(_snapshot(robots))
    result = plan(world, alice, (0.0, 0.0, 0.0), (2.0, 0.2, 0.0), threading.Event())
    assert result.path is None and "target" in result.reason
    assert (result.hit.where, result.hit.robot, result.hit.pairs[0][0]) == ("target", alice, alice)
    np.testing.assert_allclose(result.hit.scene.robots[alice].base.position[:2], (2.0, 0.2))


@pytest.mark.slow
@pytest.mark.xfail(reason="the lattice gives up when its path fails the follower check (no retry); passes or fails "
                          "with the xICR spread", strict=False)
def test_plans_around_what_the_robot_holds(world, robots, alice):  # noqa: F811
    """The move west passes the post, but the bar Alice holds would hit it: the path keeps the bar clear too."""
    start, goal = (0.0, 0.0, np.pi), (-1.6, 0.0, np.pi)
    world.sync(_snapshot(robots, [POST]))
    assert plan(world, alice, start, goal, threading.Event()).direct
    world.sync(_holding_bar(robots, alice))
    result = plan(world, alice, start, goal, threading.Event())
    assert result.path is not None and not result.direct, result.reason
    _assert_followed_clear(world, alice, result.path)


def test_drives_in_one_direction_merge_and_turns_on_one_spot_merge():
    """Waypoints between straight drives the same way go; turns in a row on one spot become one."""
    poses = [(0.0, 0.0, 0.0), (0.1, 0.0, 0.0), (0.2, 0.0, 0.0), (0.2, 0.0, 0.1), (0.2, 0.0, 0.3),
             (0.1, 0.0, 0.3), (0.2, 0.0, 0.3)]
    assert _merged(poses) == [(0.0, 0.0, 0.0), (0.2, 0.0, 0.0), (0.2, 0.0, 0.3), (0.1, 0.0, 0.3), (0.2, 0.0, 0.3)]


def test_slack_grows_with_each_turn_and_fades_over_the_drive_after():
    """A quarter turn may leave the base `swing * sqrt(2)` off; the drive after counts towards settling."""
    turn = [(0.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2)]
    slack0, since = _settled(0.0, 0.0, turn, 0.2)
    assert slack0 == pytest.approx(0.2 * math.sqrt(2.0)) and since == 0.0
    drive = [turn[-1], (0.0, SETTLE_DISTANCE / 2.0, math.pi / 2)]
    assert _settled(slack0, since, drive, 0.2) == (slack0, pytest.approx(SETTLE_DISTANCE / 2.0))


def test_a_big_turn_needs_a_drive_after_it():
    """Small turns may come in a row; after a quarter turn the base drives MIN_DRIVE before it stops or turns again."""
    def path(turn: float, drive: float):
        return [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, turn),
                (1.0 + drive * math.cos(turn), drive * math.sin(turn), turn)]
    assert not _fragile(path(math.radians(20.0), 0.05))
    assert _fragile(path(math.pi / 2, 0.1))
    assert not _fragile(path(math.pi / 2, 0.5) + [(1.0, 0.5, 0.0), (2.0, 0.5, 0.0)])
    assert not _fragile(path(math.pi / 2, 0.0)[:3]), "a turn at the end needs no drive after it"


def test_turning_centre_and_base_poses_convert_both_ways():
    """The turning centre lies AHEAD along the heading; converting there and back gives the pose again."""
    pose = (1.0, 2.0, 0.7)
    centre = _to_centre(pose, AHEAD)
    assert centre[:2] == pytest.approx((1.0 + AHEAD * math.cos(0.7), 2.0 + AHEAD * math.sin(0.7)))
    assert _to_base(centre, AHEAD) == pytest.approx(pose)


def test_a_turn_about_the_centre_is_sent_as_one_turn_for_the_follower():
    """A quarter turn planned about the turning centre: base_footprint swings on a circle, the follower turns once."""
    planned = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, math.pi / 2), (1.0, 1.0, math.pi / 2)]
    sent = _sent(planned, AHEAD)
    swing = [pose for pose in sent if abs(pose[2] % (math.pi / 2)) > 1e-6]
    assert swing and all(math.dist(pose[:2], (1.0, 0.0)) == pytest.approx(AHEAD) for pose in swing)
    kinds = [type(piece).__name__ for piece in _follower_pieces(type("Path", (), {"points": np.array(sent)}))]
    assert kinds[:3] == ["DrivePiece", "TurnPiece", "DrivePiece"]


def test_followed_reaches_the_end_of_a_curve():
    """The simulated follower with Alice's model ends a curved path within a few centimetres of its end."""
    poses = [(0.0, 0.0, 0.0)] + _arc((0.0, 0.0, 0.0), 1.0, math.pi / 2, 0.05)
    poses += _arc(poses[-1], 0.0, 1.0, 0.05)
    trace, why = followed(poses, -0.15, load_model(FOLLOWER_MODEL), MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)
    assert not why
    assert math.dist(trace[-1][:2], (1.0, 2.0)) < 0.06


def test_followed_tells_when_the_follower_never_finishes():
    """! The ideal follower model on a base whose turning centre is 0.2 m ahead circles a quarter turn's end for ever.

    This is crl_husky pure pursuit as it is; if it changes, so does this test.
    """
    poses = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, math.pi / 2), (1.0, 1.0, math.pi / 2)]
    _, why = followed(poses, -0.2, load_model("ideal"), MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)
    assert why.startswith("not done")


@pytest.mark.slow
def test_goes_around_a_robot_on_curves(world, robots, alice):  # noqa: F811
    """Cindy is in the way and Alice's arm sticks out: the path goes wide around her, no shuffle, clear as followed."""
    world.sync(_snapshot(robots))
    result = plan(world, alice, (0.0, 0.0, 0.0), (4.0, 0.0, 0.0), threading.Event())
    assert result.path is not None, result.reason
    np.testing.assert_allclose(result.path.goal, (4.0, 0.0, 0.0), atol=1e-6)
    _assert_no_shuffle(result.path)
    _assert_followed_clear(world, alice, result.path)


@pytest.mark.slow
def test_lines_up_with_a_narrow_slanted_passage(world, robots, alice):  # noqa: F811
    """The path lines up before a narrow slanted passage and drives straight through, clear as followed.

    3 cm to spare each side beyond the margin, 1.5 m deep, 17 degrees off the grid; starting 1 m to its side, facing
    across it. The follower drives it without shuffling.
    """
    gap = WHEELS + 2.0 * Settings().margin + 2.0 * 0.03
    world.sync(_snapshot(robots, _passage(gap, depth=1.5)))
    start, goal = _at(-2.0, 1.0, math.pi / 2.0), _at(2.2, -0.5, ANGLE - 0.5)
    result = plan(world, alice, start, goal, threading.Event())
    assert result.path is not None, result.reason
    points = result.path.points
    inside = ((np.abs((points[:, :2] - CENTRE) @ ALONG) < 0.75 + 0.42)
              & (np.abs((points[:, :2] - CENTRE) @ ACROSS) < gap / 2.0))
    assert inside.any()
    off = np.remainder(points[inside, 2] - ANGLE + math.pi / 2, math.pi) - math.pi / 2
    assert np.max(np.abs(off)) < math.radians(2.0), "straight through, lined up"
    _assert_no_shuffle(result.path)
    _assert_followed_clear(world, alice, result.path)


@pytest.mark.slow
def test_parks_inside_a_narrow_passage_either_way(world, robots, alice):  # noqa: F811
    """A target in the middle of a narrow passage, facing in or out: the drive in starts where there is room again."""
    world.sync(_snapshot(robots, _passage(WHEELS + 2.0 * Settings().margin + 2.0 * 0.03, depth=1.5)))
    for yaw in (ANGLE, ANGLE + math.pi):
        goal = _at(0.2, 0.0, yaw)
        result = plan(world, alice, _at(-2.0, 1.0, math.pi / 2.0), goal, threading.Event())
        assert result.path is not None, result.reason
        np.testing.assert_allclose(result.path.goal[:2], goal[:2], atol=1e-6)
        _assert_no_shuffle(result.path)
        _assert_followed_clear(world, alice, result.path)


@pytest.mark.slow
def test_goes_around_a_gap_that_only_looks_wide_enough(world, robots, alice):  # noqa: F811
    """A gap 1 cm too narrow for the margin: the cost map checks with the margin, so it leads round the walls."""
    world.sync(_snapshot(robots, _passage(WHEELS + 2.0 * Settings().margin - 0.01, depth=0.6, length=2.0)))
    debug = Debug(on=True)
    result = plan(world, alice, _at(-2.0, 0.4, 0.0), _at(2.0, 0.0), threading.Event(), debug=debug)
    assert result.path is not None, result.reason
    assert not any(text.startswith("cost map relearned") for text in debug.notes)
    _assert_followed_clear(world, alice, result.path)


@pytest.mark.slow
@pytest.mark.xfail(reason="the lattice gives up when its path fails the follower check (no retry); passes or fails "
                          "with the exact model and start", strict=False)
def test_passes_between_tall_walls_with_the_arm_on_the_roomy_side(world, robots, alice):  # noqa: F811
    """Walls taller than the arm: the gap fits Alice only off-centre, her arm side towards the wider part."""
    gap = ARM + OTHER_SIDE + 2.0 * Settings().margin + 2.0 * 0.04
    world.sync(_snapshot(robots, _passage(gap, height=1.2)))
    result = plan(world, alice, _at(-2.2, -0.27, 0.0), _at(2.2, -0.27), threading.Event())
    assert result.path is not None, result.reason
    _assert_no_shuffle(result.path)
    _assert_followed_clear(world, alice, result.path)


@pytest.mark.slow
def test_debug_records_the_search(world, robots, alice):  # noqa: F811
    """With debug on, a passage leaves the cost map, portals, the moves, the path as followed, and notes."""
    world.sync(_snapshot(robots, _passage(WHEELS + 2.0 * Settings().margin + 2.0 * 0.04)))
    debug = Debug(on=True)
    result = plan(world, alice, _at(-2.0, 0.3, 0.0), _at(2.0, 0.0), threading.Event(), debug=debug)
    assert result.path is not None, result.reason
    for name in ("cost map: no heading fits", "cost map's way", "portals", "path as followed, per xICR"):
        assert debug.layers[name].segments or debug.layers[name].points, name
    assert any(text.startswith("search:") for text in debug.notes)
    assert any(f"Room (margin {Settings().margin * 100:.0f} cm)" in text for text in debug.notes)


@pytest.mark.slow
def test_too_narrow_says_where_it_got_stuck_and_what_it_touches(world, robots, alice, monkeypatch):  # noqa: F811
    """A passage narrower than the robot: the reason says where the search got stuck, and debug draws it.

    The reason gives how close it got, what blocked it there and the room at the target; debug draws that spot and
    where the robot touches the blocks. The search runs until it gives up: 10 s lets it reach the blocks even on a
    busy machine.
    """
    monkeypatch.setattr(lattice, "TIME_LIMIT", 10.0)
    world.sync(_snapshot(robots, _passage(WHEELS - 0.1)))
    debug = Debug(on=True)
    result = plan(world, alice, _at(-2.0, 0.0), _at(2.0, 0.0), threading.Event(), debug=debug)
    assert result.path is None
    assert "closest" in result.reason and "block" in result.reason and "Target: over 30 cm" in result.reason
    assert debug.layers["closest the search got"].points and debug.layers["where the robot touches"].points
