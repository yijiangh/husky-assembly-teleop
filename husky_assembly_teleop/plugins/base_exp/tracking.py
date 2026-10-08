"""
Tracking errors measured in the monitor: the robot's pose against the path sent, the same for every controller.

The reference is the path as `FollowerPath` cuts it: dense poses along each drive, and along each turn on the spot
its sweep of yaws at one spot. Each sample is matched to the reference pose nearest in position and heading
(HEADING_WEIGHT metres per radian), searched in a window ahead of the last match, so loops, crossings and the poses
of a turn resolve to the right place. From the match:

- position error: metres; signed while driving (robot left of the path positive), the distance to the spot on a turn.
- heading error: reference yaw minus the robot's, wrapped; inside a turn's sweep it is zero, as the path allows it.
- along-track error (timed paths): progress due now minus progress reached, metres (turns count TURN_UNIT per radian).
- context: turning or driving, the path's curvature, progress 0 to 1, and the robot's speed and turn rate.

! The robot's speed comes from consecutive samples' times: give each sample the time its pose was measured.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from crl_husky.follower_path import TURN_UNIT, FollowerPath, wrap

#: Metres of position that weigh like one radian of heading when matching.
HEADING_WEIGHT = 0.1
#: Spacing of a turn's reference yaws, radians.
TURN_STEP = math.radians(1.0)
#: Match window around the last match, in progress units (metres, turns at TURN_UNIT per radian).
BEHIND, AHEAD = 0.2, 0.6
#: Smoothing of the measured speed: weight of the newest sample.
SPEED_SMOOTHING = 0.3


@dataclass
class Sample:
    """The tracking error of one pose; see the module docstring.

    Attributes:
        position: Position error, metres.
        heading: Heading error, radians.
        along: Along-track error, metres; NaN for a geometric path.
        turning: Whether the match is on a turn on the spot.
        curvature: The path's curvature at the match, 1/m; NaN on a turn.
        progress: Progress along the path at the match, 0 to 1.
        speed: The robot's speed over ground, m/s, smoothed.
        turn_rate: The robot's turn rate, rad/s, smoothed.
    """

    position: float
    heading: float
    along: float
    turning: bool
    curvature: float
    progress: float
    speed: float
    turn_rate: float


class Tracker:
    """Matches successive robot poses to one path and measures the errors."""

    def __init__(self, path: FollowerPath):
        """Lay out the reference poses of `path`."""
        xy, yaw, units, times, turning, curvature, tangent = [], [], [], [], [], [], []
        done = 0.0
        for piece in path.pieces:
            if piece.turning:
                span = piece.yaw1 - piece.yaw0
                angles = np.linspace(0.0, span, max(2, int(math.ceil(abs(span) / TURN_STEP)) + 1))
                xy.append(np.repeat(piece.xy[None, :], len(angles), axis=0))
                yaw.append(piece.yaw0 + angles)
                units.append(done + np.abs(angles) * TURN_UNIT)
                if piece.waypoint_t is not None:
                    times.append(np.interp(np.abs(angles), np.abs(piece.waypoint_yaw - piece.yaw0), piece.waypoint_t))
                turning.append(np.ones(len(angles), dtype=bool))
                curvature.append(np.full(len(angles), np.nan))
                tangent.append(np.zeros((len(angles), 2)))
                done += abs(span) * TURN_UNIT
            else:
                heading = np.array([piece.heading(arc) for arc in piece.arc])
                direction = np.diff(piece.points, axis=0)
                direction = np.vstack([direction, direction[-1:]])
                direction /= np.maximum(np.linalg.norm(direction, axis=1, keepdims=True), 1e-12)
                xy.append(piece.points)
                yaw.append(heading)
                units.append(done + piece.arc)
                if piece.waypoint_t is not None:
                    times.append(np.interp(piece.arc, piece.waypoint_arc, piece.waypoint_t))
                turning.append(np.zeros(len(piece.arc), dtype=bool))
                curvature.append(_curvature(piece.points, piece.arc))
                tangent.append(direction)
                done += piece.length
        self._xy = np.vstack(xy)
        self._yaw = np.concatenate(yaw)
        self._units = np.concatenate(units)
        self._times = np.concatenate(times) if path.timed else None
        self._turning = np.concatenate(turning)
        self._curvature = np.concatenate(curvature)
        self._tangent = np.vstack(tangent)
        self._total = max(float(self._units[-1]), 1e-9)
        self._index: int | None = None
        self._last: tuple[float, float, float, float] | None = None
        self._speed = 0.0
        self._turn_rate = 0.0

    @property
    def end(self) -> np.ndarray:
        """The path's last pose (x, y, yaw), yaw unwrapped."""
        return np.array([*self._xy[-1], self._yaw[-1]])

    def update(self, x: float, y: float, yaw: float, t: float, elapsed: float) -> Sample:
        """Match a measured pose and measure its errors.

        Args:
            x: Position, metres, world frame.
            y: Position, metres, world frame.
            yaw: Yaw, radians.
            t: Time the pose was measured, seconds; for the speed.
            elapsed: Seconds since the path was sent; for the along-track error of a timed path.

        Returns:
            Sample: The errors and their context.
        """
        if self._index is None:
            low, high = 0, np.searchsorted(self._units, AHEAD, side="right")
        else:
            here = self._units[self._index]
            low = np.searchsorted(self._units, here - BEHIND)
            high = np.searchsorted(self._units, here + AHEAD, side="right")
        high = max(high, low + 1)
        dx = self._xy[low:high, 0] - x
        dy = self._xy[low:high, 1] - y
        dyaw = np.remainder(self._yaw[low:high] - yaw + math.pi, 2 * math.pi) - math.pi
        i = low + int(np.argmin(dx * dx + dy * dy + (HEADING_WEIGHT * dyaw) ** 2))
        self._index = i

        offset = np.array([x, y]) - self._xy[i]
        if self._turning[i]:
            position = float(np.hypot(*offset))
        else:
            tx, ty = self._tangent[i]
            position = float(tx * offset[1] - ty * offset[0])
        along = math.nan
        if self._times is not None:
            along = float(np.interp(elapsed, self._times, self._units) - self._units[i])
        self._measure_speed(x, y, yaw, t)
        return Sample(position=position, heading=wrap(float(self._yaw[i]) - yaw), along=along,
                      turning=bool(self._turning[i]), curvature=float(self._curvature[i]),
                      progress=float(self._units[i] / self._total), speed=self._speed, turn_rate=self._turn_rate)

    def _measure_speed(self, x: float, y: float, yaw: float, t: float) -> None:
        """Update the smoothed speed and turn rate from the last sample."""
        if self._last is not None and t > self._last[3]:
            dt = t - self._last[3]
            speed = math.hypot(x - self._last[0], y - self._last[1]) / dt
            turn_rate = wrap(yaw - self._last[2]) / dt
            self._speed += SPEED_SMOOTHING * (speed - self._speed)
            self._turn_rate += SPEED_SMOOTHING * (turn_rate - self._turn_rate)
        if self._last is None or t > self._last[3]:
            self._last = (x, y, yaw, t)


def _curvature(points: np.ndarray, arc: np.ndarray) -> np.ndarray:
    """Signed curvature along a polyline, 1/m: heading change per metre, smoothed over a few centimetres."""
    if len(points) < 3:
        return np.zeros(len(points))
    heading = np.unwrap(np.arctan2(*np.diff(points, axis=0).T[::-1]))
    heading = np.append(heading, heading[-1])
    # * Central difference over about 5 cm, so the 1 cm grid's rounding does not show.
    step = 5
    padded = np.pad(heading, step, mode="edge")
    arc_padded = np.pad(arc, step, mode="edge")
    with np.errstate(invalid="ignore", divide="ignore"):
        curvature = (padded[2 * step:] - padded[:-2 * step]) / (arc_padded[2 * step:] - arc_padded[:-2 * step])
    return np.nan_to_num(curvature)
