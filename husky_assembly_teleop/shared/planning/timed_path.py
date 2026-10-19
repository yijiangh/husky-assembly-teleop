"""A timed path: states (a base pose, arm joints, ...) and when each is reached. Pure math, safe on any thread."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TimedPath:
    """States over time; between two, every value moves straight at constant speed.

    Attributes:
        times: Seconds from the start at which each state is reached, rising from 0. Shape (N,).
        points: The states, one row each; the first is the start. Shape (N, D).
    """

    times: np.ndarray
    points: np.ndarray

    @property
    def duration(self) -> float:
        """float: Seconds from start to end."""
        return float(self.times[-1])

    @property
    def start(self) -> np.ndarray:
        """np.ndarray: The first state."""
        return self.points[0]

    @property
    def goal(self) -> np.ndarray:
        """np.ndarray: The last state."""
        return self.points[-1]

    def sample(self, t: float) -> np.ndarray:
        """The state at time `t`, straight between states; clamped to the ends.

        Args:
            t: Seconds from the start.

        Returns:
            np.ndarray: One state, shape (D,).
        """
        return np.array([np.interp(t, self.times, column) for column in self.points.T])


def timing(poses: np.ndarray, linear_speed: float, angular_speed: float) -> np.ndarray:
    """Time of each pose when every step takes as long as its slower part, driving or turning.

    Args:
        poses: Poses (n x 3).
        linear_speed: m/s.
        angular_speed: rad/s.

    Returns:
        np.ndarray: Seconds from the first pose.
    """
    steps = np.diff(poses, axis=0)
    seconds = np.maximum(np.hypot(steps[:, 0], steps[:, 1]) / linear_speed, np.abs(steps[:, 2]) / angular_speed)
    return np.concatenate([[0.0], np.cumsum(seconds)])
