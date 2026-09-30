"""Tests for the base planner's steer (turn, drive, turn), its timing and path sampling."""

import math

import numpy as np

from husky_assembly_teleop.plugins.base_planner.path import (MAX_ANGULAR_SPEED, MAX_LINEAR_SPEED,
                                                             plan_straight_line, steer_cost, steer_points)


def test_ahead_is_drive_only():
    """A goal straight ahead, same yaw: one straight leg, timed by the drive speed."""
    path = plan_straight_line((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert len(path.poses) == 2
    assert math.isclose(path.duration, 1.0 / MAX_LINEAR_SPEED)
    assert math.isclose(path.length, 1.0)
    np.testing.assert_allclose(path.sample(path.duration / 2), (0.5, 0.0, 0.0))


def test_turn_drive_turn():
    """A goal to the left, facing back: turn to it, drive, turn to its yaw."""
    path = plan_straight_line((0.0, 0.0, 0.0), (0.0, 1.0, math.pi / 2 + 0.5))
    np.testing.assert_allclose(path.poses[1], (0.0, 0.0, math.pi / 2))
    np.testing.assert_allclose(path.goal[:2], (0.0, 1.0))
    assert math.isclose(path.goal[2], math.pi / 2 + 0.5)


def test_behind_is_reversed():
    """A goal just behind, same yaw: reverse, never turn."""
    path = plan_straight_line((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0))
    assert np.allclose(path.poses[:, 2], 0.0)
    assert len(path.poses) == 2


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
