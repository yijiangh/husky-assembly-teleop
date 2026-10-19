"""Tests for the base planner's steer (turn, drive, turn), its timing and sampling, arcs, and where a base drives."""

import math

import numpy as np
import pytest
from crl_husky.follower_path import FollowerPath

from husky_assembly_teleop.plugins.base_planner.path import (ARC_STEP, MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED,
                                                             SETTLE_DISTANCE, densify, move_margins, round_corners,
                                                             steer, steer_cost, steer_points, timed_path,
                                                             driven)

ICR = 0.15
#: Drive 1 m along x, turn left on the spot, drive 1 m along y.
RIGHT_ANGLE = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, math.pi / 2), (1.0, 1.0, math.pi / 2)]


def plan_straight_line(start, goal):
    """One steer from start to goal, timed: the path the RRT makes of a free straight move."""
    return timed_path([start] + steer(start, goal))


def test_ahead_is_drive_only():
    """A goal straight ahead, same yaw: one straight leg, timed by the drive speed."""
    path = plan_straight_line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert len(path.points) == 2
    assert math.isclose(path.duration, 1.0 / MAX_LINEAR_SPEED)
    assert math.isclose(path.length, 1.0)
    np.testing.assert_allclose(path.sample(path.duration / 2), (0.5, 0.0, 0.0))


def test_turn_drive_turn():
    """A goal to the left, facing back: turn to it, drive, turn to its yaw."""
    path = plan_straight_line((0.0, 0.0, 0.0), (0.0, 1.0, math.pi / 2 + 0.5))
    np.testing.assert_allclose(path.points[1], (0.0, 0.0, math.pi / 2))
    np.testing.assert_allclose(path.goal[:2], (0.0, 1.0))
    assert math.isclose(path.goal[2], math.pi / 2 + 0.5)


def test_behind_is_reversed():
    """A goal just behind, same yaw: reverse, never turn."""
    path = plan_straight_line((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
    assert np.allclose(path.points[:, 2], 0.0)
    assert len(path.points) == 2


def test_turn_on_the_spot_takes_short_way():
    """From 170° to -170° is a 20° turn, not 340°."""
    path = plan_straight_line((0.0, 0.0, math.radians(170)), (0.0, 0.0, math.radians(-170)))
    assert math.isclose(path.duration, math.radians(20) / MAX_ANGULAR_SPEED)


def test_already_there():
    """Nothing to do: a single waypoint, zero duration, sampling still works."""
    path = plan_straight_line((1.0, 2.0, 0.3), (1.0, 2.0, 0.3))
    assert path.duration == 0.0
    np.testing.assert_allclose(path.sample(5.0), (1.0, 2.0, 0.3))


def test_steer_points_end_exactly_on_target_and_step_finely():
    """Collision-check steps are small, and the last one is the target as given."""
    q2 = (1.0, 1.0, 3.0)
    points = steer_points((0.0, 0.0, 0.0), q2, position_step=0.05, yaw_step=0.05)
    assert points[-1] == q2
    steps = np.diff(np.array([(0.0, 0.0, 0.0)] + points[:-1]), axis=0)
    assert np.all(np.hypot(steps[:, 0], steps[:, 1]) <= 0.05 + 1e-9)
    assert np.all(np.abs(steps[:, 2]) <= 0.05 + 1e-9)


def test_steer_cost_counts_drive_and_turns():
    """1 m straight ahead costs 1; a quarter turn on the spot costs weight * pi/2."""
    assert math.isclose(steer_cost((0, 0, 0), (1, 0, 0)), 1.0)
    assert math.isclose(steer_cost((0, 0, 0), (0, 0, math.pi / 2), rotation_weight=0.3), 0.3 * math.pi / 2)


def test_corner_becomes_the_widest_arc_that_fits():
    """A 2 m arc needs 2 m of drive either side; 1 m legs take the 0.5 m one, ending exactly on the goal."""
    poses = round_corners(RIGHT_ANGLE, (2.0, 0.5), lambda _: True)
    points = np.array(poses)
    on_arc = points[1:-1]
    np.testing.assert_allclose(np.hypot(on_arc[:, 0] - 0.5, on_arc[:, 1] - 0.5), 0.5, atol=1e-9)
    np.testing.assert_allclose(points[-1], RIGHT_ANGLE[-1], atol=1e-9)
    assert np.all(np.diff(points[:, 2]) >= 0.0)
    steps = np.hypot(*np.diff(on_arc[:, :2], axis=0).T)
    assert np.all(steps <= ARC_STEP + 1e-9)


def test_corner_stays_a_turn_when_no_arc_is_clear_or_it_reverses():
    """Nothing clear, or a switch from forwards to backwards: the turn on the spot stays."""
    assert round_corners(RIGHT_ANGLE, (0.5,), lambda _: False) == RIGHT_ANGLE
    cusp = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, -math.pi / 2), (1.0, 1.0, -math.pi / 2)]
    assert round_corners(cusp, (0.5,), lambda _: True) == cusp


def test_follower_drives_a_rounded_corner_without_stopping():
    """The follower makes one drive piece of drive, arc and drive, and keeps to the planned arc."""
    poses = round_corners(RIGHT_ANGLE, (0.5,), lambda _: True)
    path = timed_path([poses[0]] + densify(poses, ARC_STEP, math.inf))
    follower = FollowerPath.from_poses(*path.points.T)
    drives = [piece for piece in follower.pieces if not piece.turning]
    assert len(drives) == 1
    planned = np.array(densify(path.points, 0.002, math.inf))[:, :2]
    off = max(np.min(np.linalg.norm(planned - point, axis=1)) for point in drives[0].points)
    assert off < 0.005


def test_straight_drive_goes_where_planned():
    """Driving straight, the turning centre does not matter, whatever the follower assumes."""
    poses = [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    for model in (0.0, -ICR):
        np.testing.assert_allclose(driven(poses, -ICR, model, 0.05, 0.05), densify(poses, 0.05, 0.05), atol=1e-12)


def test_turn_on_the_spot_swings_the_base_until_the_drive_after_brings_it_back():
    """Ideal follower: a quarter left turn about a centre 0.15 m ahead puts the base (0.15, -0.15) off, then fades."""
    poses = [(0.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2), (0.0, 2.0, math.pi / 2)]
    points = densify(poses, 0.05, 0.05)
    real = np.array(driven(poses, -ICR, 0.0, 0.05, 0.05))
    turn_end = max(i for i, point in enumerate(points) if point[1] == 0.0)
    np.testing.assert_allclose(real[turn_end], (ICR, -ICR, math.pi / 2), atol=1e-12)
    np.testing.assert_allclose(real[turn_end + 1, :2], (ICR, 0.05 - ICR) + np.array([-ICR, ICR]) * 0.05
                               / SETTLE_DISTANCE, atol=1e-12)
    np.testing.assert_allclose(real[-1], points[-1], atol=1e-12)


def test_follower_with_the_right_model_ends_each_move_on_the_plan():
    """Follower model = true xICR: its turn, drive, turn puts the base back on the plan, and arcs need no correction."""
    poses = [(0.0, 0.0, 0.0), (0.0, 0.0, math.pi / 2), (0.0, 2.0, math.pi / 2)]
    real = np.array(driven(poses, -ICR, -ICR, 0.05, 0.05))
    np.testing.assert_allclose(real[-1], poses[-1], atol=1e-9)
    arc = round_corners(RIGHT_ANGLE, (0.5,), lambda _: True)
    np.testing.assert_allclose(np.array(driven(arc, -ICR, -ICR, 0.05, 0.05))[-1], arc[-1], atol=1e-9)


def test_follower_with_a_model_turns_far_past_a_small_turn():
    """A follower with a model swings about 90 degrees out to reposition after a 10 degree turn; plans see that.

    ! crl_husky `turning_centre_poses` does this; if that changes, so does this test.
    """
    poses = [(0.0, 0.0, 0.0), (0.0, 0.0, math.radians(10)), (1.0, math.tan(math.radians(10)), math.radians(10))]
    yaws = np.array(driven(poses, -ICR, -ICR, 0.05, 0.05))[:, 2]
    assert yaws.max() > math.radians(80)


def test_arc_turns_the_body_off_the_path():
    """Ideal follower: on a 0.5 m left arc, a turning centre 0.15 m ahead makes the body point atan(0.15 / 0.5) left."""
    poses = round_corners(RIGHT_ANGLE, (0.5,), lambda _: True)
    points = densify(poses, 0.05, 0.05)
    off = np.array(driven(poses, -ICR, 0.0, 0.05, 0.05))[:, 2] - np.array(points)[:, 2]
    assert off.max() == pytest.approx(math.atan(ICR / 0.5), rel=1e-3)  # steps are chords, not arcs
    assert off.min() == pytest.approx(0.0, abs=1e-12)


def test_a_move_assumes_full_turns_at_both_ends():
    """The search does not know the moves joined to one: full swing at each end, none mid-way along 2 m."""
    points = steer_points((0.0, 0.0, 0.0), (2.0, 0.0, 0.0), 0.05, 0.05)
    margins = move_margins((0.0, 0.0, 0.0), points, ICR)
    assert margins[0] == margins[-1] == pytest.approx(2.0 * ICR)
    assert margins[1] == pytest.approx(2.0 * ICR * (1.0 - 0.05 / SETTLE_DISTANCE))
    assert margins[len(margins) // 2] == 0.0
