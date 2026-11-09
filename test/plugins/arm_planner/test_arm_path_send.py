"""Sending a planned arm path: one joint trajectory through rounded corners, resting at sharp ones, within limits."""

from __future__ import annotations

import numpy as np
import pytest

from husky_assembly_teleop.plugins.arm_planner.planner import BLEND_LENGTH, ArmPath, round_corners
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.plugins.arm_planner.planner_plugin import arm_command
from husky_assembly_teleop.robot_interface.arm import (JOINT_MOVE_MAX_SPEED, JOINT_MOVE_WAYPOINT_SPACING,
                                                       JOINT_PATH_MAX_ACCELERATION, joint_path_move)
from robot_fakes import fake_arm

pytestmark = pytest.mark.safety

WAYPOINTS = np.array([[0.0, -1.5, 1.5, 0.0, 0.0, 0.0],
                      [0.4, -1.5, 1.5, 0.0, 0.0, 0.0],
                      [0.4, -1.5, 1.5, 0.0, 0.0, 0.0],      # a repeat, skipped
                      [0.4, -1.2, 1.0, 0.1, 0.0, 0.0],
                      [0.41, -1.2, 1.0, 0.1, 0.0, 0.0]])    # a tiny last step, limited by acceleration


def test_path_move_stops_on_every_waypoint_and_ends_at_rest():
    """Every waypoint is sampled exactly with zero velocity; the move starts and ends on the path."""
    positions, velocities, duration = joint_path_move(WAYPOINTS)
    assert np.allclose(positions[0], WAYPOINTS[0]) and np.allclose(positions[-1], WAYPOINTS[-1])
    assert duration == pytest.approx((len(positions) - 1) * JOINT_MOVE_WAYPOINT_SPACING)
    for waypoint in WAYPOINTS:
        hits = np.flatnonzero(np.all(np.isclose(positions, waypoint), axis=1))
        assert len(hits) and np.allclose(velocities[hits], 0.0, atol=1e-9)


def test_path_move_stays_on_the_segments():
    """Between two waypoints every sample lies on the straight segment joining them: corners are never cut."""
    positions, _, _ = joint_path_move(WAYPOINTS)
    segments = [(a, b) for a, b in zip(WAYPOINTS[:-1], WAYPOINTS[1:]) if not np.allclose(a, b)]
    for point in positions:
        assert any(_off_segment(point, a, b) < 1e-9 for a, b in segments)


def _off_segment(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    """Distance from `point` to the segment a-b."""
    t = np.clip(np.dot(point - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0)
    return float(np.linalg.norm(point - (a + t * (b - a))))


def _never(_q) -> bool:
    return False


def _rounded() -> tuple[np.ndarray, list[int]]:
    """WAYPOINTS with every corner rounded, nothing in the way."""
    return round_corners(WAYPOINTS, _never)


@pytest.mark.parametrize("stops", [None, []], ids=["resting at every point", "through rounded corners"])
def test_path_move_keeps_to_the_speed_and_acceleration_limits(stops):
    """The sampled speeds and accelerations, as the arm checks them, stay within the limits."""
    positions, velocities, _ = joint_path_move(WAYPOINTS if stops is None else _rounded()[0], stops)
    step = JOINT_MOVE_WAYPOINT_SPACING
    assert np.max(np.abs(np.diff(positions, axis=0))) / step <= JOINT_MOVE_MAX_SPEED + 1e-9
    assert np.max(np.abs(velocities)) <= JOINT_MOVE_MAX_SPEED + 1e-9
    assert np.max(np.abs(np.diff(positions, n=2, axis=0))) / step ** 2 <= JOINT_PATH_MAX_ACCELERATION + 1e-9
    assert np.max(np.abs(np.diff(velocities, axis=0))) / step <= JOINT_PATH_MAX_ACCELERATION + 1e-9


def test_rounded_corners_are_run_through_without_resting():
    """Only the ends rest; the arm accepts the trajectory, and it starts and ends on the path."""
    points, sharp = _rounded()
    assert sharp == []
    positions, velocities, duration = joint_path_move(points, sharp)
    assert np.allclose(positions[0], WAYPOINTS[0]) and np.allclose(positions[-1], WAYPOINTS[-1])
    assert np.all(np.max(np.abs(velocities[1:-1]), axis=1) > 0.0) and not np.any(velocities[[0, -1]])
    assert fake_arm(joints=WAYPOINTS[0]).send_joint_trajectory(positions, duration, velocities)


def test_a_rounding_stays_near_its_corner_and_joins_the_sides():
    """Each rounding leaves the path on one side of its corner and rejoins it on the other, never far from it."""
    points, _ = _rounded()
    segments = [(a, b) for a, b in zip(WAYPOINTS[:-1], WAYPOINTS[1:]) if not np.allclose(a, b)]
    assert max(min(_off_segment(point, a, b) for a, b in segments) for point in points) <= BLEND_LENGTH / 2


def test_a_corner_whose_rounding_collides_is_rested_at():
    """Something inside the second corner blocks every rounding of it: the arm comes to rest exactly there."""
    corner = WAYPOINTS[3]

    def inside_corner(q) -> bool:
        return bool(np.linalg.norm(np.subtract(q, corner)) > 1e-9 and q[0] > 0.4 + 1e-9)

    points, sharp = round_corners(WAYPOINTS, inside_corner)
    assert [tuple(points[i]) for i in sharp] == [tuple(corner)]
    positions, velocities, _ = joint_path_move(points, sharp)
    rest = np.flatnonzero(np.all(np.isclose(positions, corner), axis=1))
    assert len(rest) and np.allclose(velocities[rest], 0.0)


def test_path_move_refuses_a_path_that_does_not_move():
    """A path of one pose repeated is no move."""
    with pytest.raises(ValueError):
        joint_path_move(WAYPOINTS[[0, 0]])


def test_arm_command_is_the_planned_trajectory_in_driver_order():
    """The path's samples, timing and velocities go out as planned, in the UR driver's joint order."""
    names = arm_joint_names("left_ur_arm")
    shuffled = [names[i] for i in (5, 4, 3, 2, 1, 0)]
    path = ArmPath.along(shuffled, WAYPOINTS[:, ::-1])
    command = arm_command("0806", "left_ur_arm", path)
    (motion,) = command.arms
    expected, expected_velocities, expected_duration = joint_path_move(WAYPOINTS)
    assert (command.serial, motion.arm) == ("0806", "left_ur_arm")
    np.testing.assert_allclose(motion.positions, expected)
    np.testing.assert_allclose(motion.velocities, expected_velocities)
    assert command.duration == pytest.approx(expected_duration)


def test_a_path_that_does_not_move_is_no_path():
    """Planning refuses it before there is anything to send."""
    with pytest.raises(ValueError):
        ArmPath.along(arm_joint_names("left_ur_arm"), WAYPOINTS[[0, 0]])


def test_timing_the_waypoints_again_glides_where_timing_the_samples_stops():
    """Timed again (as Exec does), the waypoints glide through the curves; the 0.2 s samples would stop on them.

    The samples meet at an angle on every curve, where the curve itself never turns that sharply.
    """
    from husky_assembly_teleop.shared.execution.retime import retime

    corners = np.array([[0.2 * k, -1.5 + (0.04 if k % 2 else 0.0), 1.5, 0.0, 0.0, 0.0] for k in range(8)])

    def tight(q) -> bool:
        """Only within 3 cm-like 0.03 rad of the zigzag is clear: every curve is halved to fit, as in a tight spot."""
        return min(_off_segment(np.asarray(q), a, b) for a, b in zip(corners[:-1], corners[1:])) > 0.03

    shape, sharp = round_corners(corners, tight)
    path = ArmPath.along([f"j{i}" for i in range(6)], shape, sharp)
    assert sharp == [] and np.array_equal(path.waypoints, shape) and len(path.points) != len(shape)
    assert len(retime(path.points).stops) > 2, "the samples, timed again, stop on the curves"
    assert len(retime(path.waypoints).stops) == 2, "the waypoints, timed again, glide through: no stop between the ends"
    assert np.array_equal(path.slowed(2.0).waypoints, shape), "slowing it keeps its shape"
