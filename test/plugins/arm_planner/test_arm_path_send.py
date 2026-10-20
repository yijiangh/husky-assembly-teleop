"""Sending a planned arm path: one joint trajectory that stops at every waypoint, within the arm's limits."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from husky_assembly_teleop.plugins.arm_planner.planner import ArmPath, arm_joint_names
from husky_assembly_teleop.plugins.arm_planner.planner_plugin import send_to_arm
from husky_assembly_teleop.robot_interface.arm import (JOINT_MOVE_MAX_SPEED, JOINT_MOVE_WAYPOINT_SPACING,
                                                       JOINT_PATH_MAX_ACCELERATION, joint_path_move)

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


def test_path_move_keeps_to_the_speed_and_acceleration_limits():
    """The sampled speeds and accelerations, as the arm checks them, stay within the limits."""
    positions, velocities, _ = joint_path_move(WAYPOINTS)
    step = JOINT_MOVE_WAYPOINT_SPACING
    assert np.max(np.abs(np.diff(positions, axis=0))) / step <= JOINT_MOVE_MAX_SPEED + 1e-9
    assert np.max(np.abs(velocities)) <= JOINT_MOVE_MAX_SPEED + 1e-9
    assert np.max(np.abs(np.diff(positions, n=2, axis=0))) / step ** 2 <= JOINT_PATH_MAX_ACCELERATION + 1e-9
    assert np.max(np.abs(np.diff(velocities, axis=0))) / step <= JOINT_PATH_MAX_ACCELERATION + 1e-9


def test_path_move_refuses_a_path_that_does_not_move():
    """A path of one pose repeated is no move."""
    with pytest.raises(ValueError):
        joint_path_move(WAYPOINTS[[0, 0]])


class _FakeArm:
    """Records the trajectory it is sent; refuses when told to."""

    def __init__(self, accept: bool):
        self.accept, self.sent = accept, None

    def send_joint_trajectory(self, positions, duration, velocities=None) -> bool:
        self.sent = (positions, duration, velocities)
        return self.accept


def _ctx(arm: _FakeArm) -> SimpleNamespace:
    return SimpleNamespace(world=SimpleNamespace(robots={"0806": SimpleNamespace(arms={"left_ur_arm": arm})}),
                           log_info=lambda _text: None, log_warn=lambda _text: None)


def test_send_to_arm_sends_the_waypoints_in_driver_order():
    """The path's joints are put into the UR driver's order before sending; a refusal is reported."""
    names = arm_joint_names("left_ur_arm")
    shuffled = [names[i] for i in (5, 4, 3, 2, 1, 0)]
    path = ArmPath.at_preview_speed(shuffled, WAYPOINTS[:, ::-1])
    arm = _FakeArm(accept=True)
    assert send_to_arm(_ctx(arm), "0806", "left_ur_arm", path)
    assert np.allclose(arm.sent[0][0], WAYPOINTS[0]) and np.allclose(arm.sent[0][-1], WAYPOINTS[-1])
    assert not send_to_arm(_ctx(_FakeArm(accept=False)), "0806", "left_ur_arm", path)
    assert not send_to_arm(_ctx(_FakeArm(accept=True)), "0806", "left_ur_arm",
                           ArmPath.at_preview_speed(names, WAYPOINTS[[0, 0]]))
