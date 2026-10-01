"""
A timed path: states (a base pose, arm joints, ...) and when each is reached.

! Pure math: no viser, no PyBullet, no ROS, so it is safe on any thread.
"""

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
