"""
A timed base path, and the "steer" move a differential-drive base can make.

A path is a list of timed floor poses; between two poses the base moves at
constant speed. A steer is turn to face the next pose, drive straight, turn to
its yaw. The RRT (planner.py) uses it as its local planner.

! Pure math: no viser, no PyBullet, no ROS, so it is safe on any thread.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

#: Planning speeds; low, since the onboard follower sets its own speed.
MAX_LINEAR_SPEED = 0.2   # m/s
MAX_ANGULAR_SPEED = 0.3  # rad/s

#: Closer than this counts as already there; the move only turns.
POSITION_TOLERANCE = 0.005  # m


@dataclass(frozen=True)
class BasePath:
    """A timed path of the base on the floor.

    Attributes:
        times: Seconds from the start at which each waypoint is reached, rising
            from 0. Shape (N,).
        poses: Waypoints (x, y, yaw), metres and radians, world frame. Shape
            (N, 3). Yaw is unwrapped so blending never turns the long way.
    """

    times: np.ndarray
    poses: np.ndarray

    @property
    def duration(self) -> float:
        """float: Seconds from start to end."""
        return float(self.times[-1])

    @property
    def length(self) -> float:
        """float: Distance driven, metres, turns not counted."""
        return float(np.sum(np.linalg.norm(np.diff(self.poses[:, :2], axis=0), axis=1)))

    @property
    def start(self) -> np.ndarray:
        """np.ndarray: First pose, (x, y, yaw)."""
        return self.poses[0]

    @property
    def goal(self) -> np.ndarray:
        """np.ndarray: Last pose, (x, y, yaw)."""
        return self.poses[-1]

    def sample(self, t: float) -> np.ndarray:
        """The pose at time `t`, clamped to the path's start and end.

        Args:
            t: Seconds from the start.

        Returns:
            np.ndarray: (x, y, yaw) at that time.
        """
        return np.array([np.interp(t, self.times, self.poses[:, axis]) for axis in range(3)])

    def floor_points(self, samples_per_metre: float = 20.0) -> np.ndarray:
        """Points along the path on the floor, for drawing it as a line.

        Args:
            samples_per_metre: Density of the points along straight parts.

        Returns:
            np.ndarray: (M, 2) x, y points, including every waypoint.
        """
        points = [self.poses[0, :2]]
        for a, b in zip(self.poses[:-1, :2], self.poses[1:, :2]):
            count = max(int(np.linalg.norm(b - a) * samples_per_metre), 1)
            points.extend(a + (b - a) * (i / count) for i in range(1, count + 1))
        return np.array(points)


def steer(q1, q2) -> list[tuple[float, float, float]]:
    """The corner poses of the move from `q1` to `q2`: turn, drive straight, turn.

    ? Drives forwards or backwards, whichever needs less turning.

    ! Yaws continue from `q1` and may lie outside (-pi, pi]; the last pose
      matches `q2`'s heading, not necessarily its exact yaw number.

    Args:
        q1: (x, y, yaw) to start from, metres and radians.
        q2: (x, y, yaw) to end at.

    Returns:
        list[tuple[float, float, float]]: One to three poses after `q1`, ending at `q2`.
    """
    x0, y0, yaw0 = (float(value) for value in q1)
    x1, y1, yaw1 = (float(value) for value in q2)
    corners = []
    yaw = yaw0
    if math.hypot(x1 - x0, y1 - y0) > POSITION_TOLERANCE:
        heading = math.atan2(y1 - y0, x1 - x0)
        forward = abs(_angle(heading - yaw0)) + abs(_angle(yaw1 - heading))
        reverse = abs(_angle(heading + math.pi - yaw0)) + abs(_angle(yaw1 - heading - math.pi))
        yaw = yaw0 + _angle((heading if forward <= reverse else heading + math.pi) - yaw0)
        corners += [(x0, y0, yaw), (x1, y1, yaw)]
    corners.append((x1, y1, yaw + _angle(yaw1 - yaw)))
    return corners


def steer_cost(q1, q2, rotation_weight: float = 0.3) -> float:
    """What the move from `q1` to `q2` costs: metres driven plus weighted radians turned.

    Args:
        q1: (x, y, yaw) start.
        q2: (x, y, yaw) end.
        rotation_weight: Metres one radian of turning is worth.

    Returns:
        float: The cost.
    """
    cost, last = 0.0, tuple(float(value) for value in q1)
    for corner in steer(q1, q2):
        cost += math.hypot(corner[0] - last[0], corner[1] - last[1]) + rotation_weight * abs(corner[2] - last[2])
        last = corner
    return cost


def steer_points(q1, q2, position_step: float = 0.05, yaw_step: float = 0.05) -> list[tuple[float, float, float]]:
    """The move from `q1` to `q2` in small steps, for collision checking.

    Args:
        q1: (x, y, yaw) start; not included.
        q2: (x, y, yaw) end; included, exactly as given.
        position_step: Largest step while driving, metres.
        yaw_step: Largest step while turning, radians.

    Returns:
        list[tuple[float, float, float]]: Poses after `q1`, ending with `q2`.
    """
    points, last = [], np.array(q1, dtype=float)
    for corner in steer(q1, q2):
        corner = np.array(corner)
        count = max(int(math.ceil(max(np.linalg.norm(corner[:2] - last[:2]) / position_step,
                                      abs(corner[2] - last[2]) / yaw_step))), 1)
        points += [tuple(last + (corner - last) * (i / count)) for i in range(1, count + 1)]
        last = corner
    # * End exactly on `q2` so the RRT's connected nodes compare equal.
    points[-1] = tuple(float(value) for value in q2)
    return points


def timed_path(poses, linear_speed: float = MAX_LINEAR_SPEED, angular_speed: float = MAX_ANGULAR_SPEED) -> BasePath:
    """Time a list of poses, each leg by its slower part (drive or turn); legs that go nowhere are dropped.

    Args:
        poses: (x, y, yaw) waypoints, the first being where the base starts.
        linear_speed: Driving speed, m/s.
        angular_speed: Turning speed, rad/s.

    Returns:
        BasePath: The timed path.
    """
    poses = np.array(poses, dtype=float)
    poses[:, 2] = np.unwrap(poses[:, 2])
    times, kept = [0.0], [poses[0]]
    for pose in poses[1:]:
        last = kept[-1]
        seconds = max(math.hypot(*(pose[:2] - last[:2])) / linear_speed, abs(pose[2] - last[2]) / angular_speed)
        if seconds > 1e-6:
            times.append(times[-1] + seconds)
            kept.append(pose)
    return BasePath(times=np.array(times), poses=np.array(kept))


def plan_straight_line(start: tuple[float, float, float], goal: tuple[float, float, float],
                       linear_speed: float = MAX_LINEAR_SPEED,
                       angular_speed: float = MAX_ANGULAR_SPEED) -> BasePath:
    """One steer from start to goal, with no collision check.

    Args:
        start: (x, y, yaw) where the base is now, metres and radians.
        goal: (x, y, yaw) where it should end up.
        linear_speed: Driving speed, m/s.
        angular_speed: Turning speed, rad/s.

    Returns:
        BasePath: Up to four waypoints: start, facing the goal, at the goal, at its yaw.
    """
    return timed_path([tuple(start)] + steer(start, goal), linear_speed, angular_speed)


def _angle(angle: float) -> float:
    """An angle folded into (-pi, pi]: the shortest turn to the same heading."""
    return math.atan2(math.sin(angle), math.cos(angle))
