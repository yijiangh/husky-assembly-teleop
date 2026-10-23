"""
Time a planned joint path for the arm's trajectory controller: a cosine profile per run, sampled evenly in time.

A run is the stretch of the path between two stops. The arm glides through gentle bends at speed and stops only at a
bend it can't glide through: tamp's free paths are dense, so stopping at every waypoint (as `joint_path_move` does
with `stops=None`) would be stop-and-go.

What the controller runs is the cubic through each sample's position and velocity (the velocities are the samples'
own finite differences, zero at the stops). A bend gets a stop when that cubic, next to it, needs more than
SPLIT_ACCELERATION or leaves the planned polyline by more than CORNER_CUT (the swept check checked the polyline).
A run still over SPLIT_ACCELERATION with no bend left to stop at (a short move: a few samples) gets one step
longer at a time until it isn't.

- Columns are UR arms' joints in UR_JOINT_NAMES order, arm after arm (6 or 12 columns).
- Every sample lies on the planned polyline, and the first and last rows are exactly the path's ends.
- Pure: any thread.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from ...robot_interface.arm import (JOINT_MOVE_MAX_SPEED, JOINT_MOVE_WAYPOINT_SPACING, JOINT_PATH_MAX_ACCELERATION,
                                    TRAJECTORY_MAX_DURATION, TRAJECTORY_MAX_JOINT_ACCELERATION,
                                    TRAJECTORY_MAX_JOINT_SPEED, TRAJECTORY_MAX_WAYPOINTS, UR_JOINT_LIMITS)

#: The most the controller's cubic may accelerate a joint, rad/s^2: 1.5 times the design acceleration, still below the
#: arm's own limit (TRAJECTORY_MAX_JOINT_ACCELERATION). A bend whose cubic needs more gets a stop.
SPLIT_ACCELERATION = 1.5 * JOINT_PATH_MAX_ACCELERATION
#: The furthest any joint of the controller's cubic may be from the planned polyline, radians (max over joints).
CORNER_CUT = np.radians(0.05)
#: A bend is split at this share of CORNER_CUT (the check samples the cubic; the margin covers between samples).
_CUT_SPLIT = 0.8
#: Points per segment the cubic is measured at, inside the segment.
_CUT_POINTS = 15
#: Direction changes below this, max-norm, are straight (collinear waypoints).
_STRAIGHT = 1e-9


class RetimeError(ValueError):
    """A path that can't be timed within the arm's limits; the message says why."""


@dataclass(frozen=True)
class Retimed:
    """A path timed for the arm: rows evenly spaced in time, from rest to rest.

    Attributes:
        positions: Joint values per sample (M x D), radians.
        velocities: Joint velocities per sample (M x D), rad/s: finite differences, zero at stops; the last row 0.
        duration: Seconds from the first sample to the last.
        minimum: The shortest duration the arm's limits allow for this path, seconds.
        stops: The path's rows (repeated rows dropped) the arm stops at, its ends included.
    """

    positions: np.ndarray
    velocities: np.ndarray
    duration: float
    minimum: float
    stops: tuple[int, ...]

    @property
    def times(self) -> np.ndarray:
        """np.ndarray: Seconds from the start of each sample (M,)."""
        return np.linspace(0.0, self.duration, len(self.positions))


@dataclass(frozen=True)
class _Polyline:
    """The planned path without repeated rows: its segments' directions, arc lengths and bends.

    Attributes:
        rows: Waypoints (N x D).
        directions: Each segment's direction, largest joint change 1 ((N - 1) x D).
        arc: Arc length (sum of each segment's largest joint change) at each waypoint (N,).
        bends: Each waypoint's change of direction, max-norm; 0 at the ends (N,).
    """

    rows: np.ndarray
    directions: np.ndarray
    arc: np.ndarray
    bends: np.ndarray


@dataclass(frozen=True)
class _Samples:
    """A path sampled for the controller.

    Attributes:
        counts: Samples per run (after its first).
        positions: Joint values per sample (M x D).
        velocities: Their velocities (M x D).
        where: Arc length of each sample (M,).
        boundaries: Sample index of each stop.
    """

    counts: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    where: np.ndarray
    boundaries: np.ndarray


def retime(path: Sequence[Sequence[float]], duration: float = 0.0, *, max_speed: float = JOINT_MOVE_MAX_SPEED,
           max_acceleration: float = JOINT_PATH_MAX_ACCELERATION,
           spacing: float = JOINT_MOVE_WAYPOINT_SPACING) -> Retimed:
    """Time a joint path: as fast as the limits allow, or slower to last `duration`.

    Each run follows s(t) = L (1 - cos(pi t / T)) / 2 along its arc length L (the largest joint change of each
    segment, so no joint is faster than s'), T a whole number of `spacing` steps. A bend whose cubic breaks
    SPLIT_ACCELERATION or CORNER_CUT gets a stop, until none does; a run over SPLIT_ACCELERATION without a bend to
    stop at gets longer.

    Args:
        path: Joint rows (N x D), radians; the first is where the arm is.
        duration: Seconds the operator asks for; only longer than the minimum slows the move.
        max_speed: Peak speed of any joint, rad/s.
        max_acceleration: Peak acceleration along a run, rad/s^2.
        spacing: Seconds between samples.

    Returns:
        Retimed: The timed path.

    Raises:
        RetimeError: If the path has fewer than two distinct rows, is not finite, leaves the UR joint limits, or the
            result would break the arm's limits, its trajectory checks or CORNER_CUT (e.g. over 300 s).
    """
    line = _polyline(_distinct(path))
    stops = [0, len(line.rows) - 1]
    samples, stops = _fit(line, stops, 1.0, max_speed, max_acceleration, spacing)
    minimum = float(samples.counts.sum() * spacing)
    if duration > minimum:
        # * Only slower: every run's step count grows by the same factor, rounded up.
        samples, stops = _fit(line, stops, duration / minimum, max_speed, max_acceleration, spacing)
    timed = Retimed(samples.positions, samples.velocities, float(samples.counts.sum() * spacing), minimum,
                    tuple(stops))
    _check(timed, line, samples, spacing)
    return timed


def sampled_limits(positions: np.ndarray, duration: float, velocities: np.ndarray | None) -> tuple[float, float]:
    """The largest speed and acceleration of a trajectory, measured exactly as the arm checks it (`_check_trajectory`).

    Args:
        positions: Waypoints (M x D), evenly spread over `duration`.
        duration: Seconds.
        velocities: Waypoint velocities (M x D), or None.

    Returns:
        tuple[float, float]: Speed (rad/s) and acceleration (rad/s^2).
    """
    positions = np.asarray(positions, dtype=float)
    step = duration / (len(positions) - 1)
    speed = float(np.max(np.abs(np.diff(positions, axis=0)))) / step
    if velocities is not None:
        speed = max(speed, float(np.max(np.abs(velocities))))
    acceleration = 0.0
    if velocities is not None:
        acceleration = float(np.max(np.abs(np.diff(velocities, axis=0)))) / step
    if len(positions) >= 3:
        acceleration = max(acceleration, float(np.max(np.abs(np.diff(positions, n=2, axis=0)))) / step ** 2)
    return speed, acceleration


def executed_limits(positions: np.ndarray, velocities: np.ndarray, step: float) -> tuple[np.ndarray, float]:
    """What the controller's cubic between samples needs: each segment's largest acceleration, and the largest speed.

    The cubic through (p0, v0) and (p1, v1) over `step` seconds accelerates linearly, so its ends bound it.

    Args:
        positions: Samples (M x D).
        velocities: Their velocities (M x D).
        step: Seconds between samples.

    Returns:
        tuple[np.ndarray, float]: Per segment the largest acceleration of any joint (M - 1,), rad/s^2; the largest
            speed of any joint anywhere, rad/s.
    """
    p, v, h = positions, velocities, step
    dp = p[1:] - p[:-1]
    start = (6 * dp - h * (4 * v[:-1] + 2 * v[1:])) / h ** 2
    end = (-6 * dp + h * (2 * v[:-1] + 4 * v[1:])) / h ** 2
    acceleration = np.maximum(np.max(np.abs(start), axis=1), np.max(np.abs(end), axis=1))
    t = np.linspace(0.0, 1.0, 9)[:, None, None]
    speed = v[:-1] * (1 - 4 * t + 3 * t ** 2) + v[1:] * (3 * t ** 2 - 2 * t) + dp / h * (6 * t - 6 * t ** 2)
    return acceleration, float(np.max(np.abs(speed)))


# --- --- --- --- --- STEPS --- --- --- --- ---

def _distinct(path: Sequence[Sequence[float]]) -> np.ndarray:
    """The path without repeated rows, checked: at least two rows, finite, within the UR joint limits.

    Raises:
        RetimeError: Naming what is wrong.
    """
    try:
        rows = np.array(path, dtype=float)
    except (TypeError, ValueError):
        raise RetimeError("the path is not a table of numbers") from None
    if rows.ndim != 2 or rows.shape[1] == 0:
        raise RetimeError(f"the path must be rows of joint values, got shape {rows.shape}")
    if not np.all(np.isfinite(rows)):
        raise RetimeError("the path holds NaN or infinity")
    outside = np.argwhere(np.abs(rows) > np.resize(UR_JOINT_LIMITS, rows.shape[1]))
    if len(outside):
        row, column = outside[0]
        raise RetimeError(f"row {row} puts joint {column} at {rows[row, column]:.3f} rad, past the UR limits")
    keep = np.concatenate([[True], np.any(np.diff(rows, axis=0) != 0.0, axis=1)])
    rows = rows[keep]
    if len(rows) < 2:
        raise RetimeError("the path does not move")
    return rows


def _polyline(rows: np.ndarray) -> _Polyline:
    """The path's directions, arc lengths and bends."""
    steps = np.diff(rows, axis=0)
    lengths = np.max(np.abs(steps), axis=1)
    directions = steps / lengths[:, None]
    bends = np.zeros(len(rows))
    bends[1:-1] = np.max(np.abs(np.diff(directions, axis=0)), axis=1)
    return _Polyline(rows, directions, np.concatenate([[0.0], np.cumsum(lengths)]), bends)


def _fit(line: _Polyline, stops: list[int], scale: float, max_speed: float, max_acceleration: float,
         spacing: float) -> tuple[_Samples, list[int]]:
    """Sample the runs, `scale` times slower than the fastest, adding stops and steps until the cubic keeps the limits.

    Returns:
        tuple[_Samples, list[int]]: The samples, and the stops (sorted).
    """
    #: Steps added to a run, by its (first, last) stop.
    extra: dict[tuple[int, int], int] = {}
    while True:
        counts = np.ceil(_step_counts(line.arc, stops, max_speed, max_acceleration, spacing) * scale - 1e-9)
        counts = counts.astype(int) + [extra.get(run, 0) for run in zip(stops[:-1], stops[1:])]
        samples = _sample(line, stops, counts, spacing)
        added, stretched = _stops_to_add(line, samples, stops, spacing)
        if not added and not stretched:
            return samples, stops
        stops = sorted(set(stops) | added)
        for run in stretched:
            extra[run] = extra.get(run, 0) + 1


def _step_counts(arc: np.ndarray, stops: list[int], max_speed: float, max_acceleration: float,
                 spacing: float) -> np.ndarray:
    """Samples per run: the cosine profile's time over its arc length, rounded up to whole steps (arm.py:210-211)."""
    lengths = np.diff(arc[stops])
    seconds = np.maximum(np.pi / 2 * lengths / max_speed, np.pi * np.sqrt(lengths / (2 * max_acceleration)))
    return np.maximum(np.ceil(seconds / spacing - 1e-9).astype(int), 1)


def _sample(line: _Polyline, stops: list[int], counts: np.ndarray, spacing: float) -> _Samples:
    """Sample every run on its cosine profile; runs share their boundary sample, which is the stop row exactly.

    Velocities are the samples' central differences, zero at each stop: the cubic between samples then follows them.
    """
    rows, directions, arc = line.rows, line.directions, line.arc
    run = np.repeat(np.arange(len(counts)), counts)                  # the run of every sample after the first
    index = np.concatenate([np.arange(1, count + 1) for count in counts])
    starts, ends = np.asarray(stops[:-1]), np.asarray(stops[1:])
    length = arc[ends] - arc[starts]
    phase = np.pi * index / counts[run]
    where = np.concatenate([[0.0], arc[starts][run] + length[run] * (1.0 - np.cos(phase)) / 2.0])
    run_of = np.concatenate([[0], run])
    first, last = np.asarray(stops)[run_of], np.asarray(stops)[run_of + 1] - 1
    segment = np.clip(np.searchsorted(arc, where, side="right") - 1, first, last)
    positions = rows[segment] + (where - arc[segment])[:, None] * directions[segment]
    boundaries = np.concatenate([[0], np.cumsum(counts)])
    positions[boundaries] = rows[stops]
    velocities = np.zeros_like(positions)
    velocities[1:-1] = (positions[2:] - positions[:-2]) / (2.0 * spacing)
    velocities[boundaries] = 0.0
    return _Samples(counts, positions, velocities, where, boundaries)


def _stops_to_add(line: _Polyline, samples: _Samples, stops: list[int],
                  spacing: float) -> tuple[set[int], set[tuple[int, int]]]:
    """The bends to stop at next: for each segment over SPLIT_ACCELERATION or cutting a corner, its sharpest bend.

    A segment's cubic depends on the samples one before and one after it: its bend is looked for in that window.
    A segment over SPLIT_ACCELERATION with no bend there (or only stops) can't be helped by a stop: its run is to
    get one step longer, which lowers the cubic's acceleration (e.g. 97°/s^2 on 3 samples for a 1.9° move).

    Returns:
        tuple[set[int], set[tuple[int, int]]]: Row indices to stop at, and the runs to lengthen by their
            (first, last) stop; both empty when every segment keeps the limits.
    """
    acceleration, _speed = executed_limits(samples.positions, samples.velocities, spacing)
    over = set(np.flatnonzero(acceleration > SPLIT_ACCELERATION).tolist())
    over |= {k for k in _segments_near_bends(line, samples) if _corner_cut(line, samples, k, spacing)
             > _CUT_SPLIT * CORNER_CUT}
    bends = line.bends.copy()
    bends[stops] = -1.0                                             # already stops: never chosen again
    added, stretched = set(), set()
    for k in over:
        low, high = _window(samples, k)
        inside = np.arange(np.searchsorted(line.arc, low, side="left"), np.searchsorted(line.arc, high, side="right"))
        if len(inside) and np.max(bends[inside]) > _STRAIGHT:
            added.add(int(inside[np.argmax(bends[inside])]))
        elif acceleration[k] > SPLIT_ACCELERATION:
            run = int(np.searchsorted(samples.boundaries, k, side="right")) - 1
            stretched.add((stops[run], stops[run + 1]))
    return added, stretched


def _window(samples: _Samples, k: int) -> tuple[float, float]:
    """The arc lengths segment k's cubic depends on: from the sample before it to the one after it."""
    where = samples.where
    return where[max(k - 1, 0)], where[min(k + 2, len(where) - 1)]


def _segments_near_bends(line: _Polyline, samples: _Samples) -> list[int]:
    """The segments whose window holds a bend; elsewhere the samples are collinear and the cubic stays on the path."""
    bent = line.arc[line.bends > _STRAIGHT]
    where = samples.where
    segments = np.arange(len(where) - 1)
    low, high = where[np.maximum(segments - 1, 0)], where[np.minimum(segments + 2, len(where) - 1)]
    near = np.searchsorted(bent, high, side="right") > np.searchsorted(bent, low, side="left")
    return segments[near].tolist()


def _corner_cut(line: _Polyline, samples: _Samples, k: int, spacing: float) -> float:
    """How far segment k's cubic gets from the planned polyline, radians, the largest joint (an upper bound)."""
    p, v = samples.positions, samples.velocities
    t = np.linspace(0.0, 1.0, _CUT_POINTS + 2)[1:-1, None]
    points = (p[k] * (2 * t ** 3 - 3 * t ** 2 + 1) + spacing * v[k] * (t ** 3 - 2 * t ** 2 + t)
              + p[k + 1] * (3 * t ** 2 - 2 * t ** 3) + spacing * v[k + 1] * (t ** 3 - t ** 2))
    low, high = _window(samples, k)
    first = max(int(np.searchsorted(line.arc, low, side="right")) - 2, 0)
    last = min(int(np.searchsorted(line.arc, high, side="left")) + 1, len(line.rows) - 2)
    a, b = line.rows[first:last + 1][None], line.rows[first + 1:last + 2][None]
    q = points[:, None]
    along = np.clip(np.sum((q - a) * (b - a), axis=2) / np.sum((b - a) ** 2, axis=2), 0.0, 1.0)
    off = np.max(np.abs(q - (a + along[..., None] * (b - a))), axis=2)
    return float(np.max(np.min(off, axis=1)))


def _check(timed: Retimed, line: _Polyline, samples: _Samples, spacing: float) -> None:
    """The backstop: the arm's limits at the samples and on the cubic, the corner cut, rest at the end, exact ends.

    Raises:
        RetimeError: Naming the broken limit.
    """
    count = len(timed.positions)
    if not 0.0 < timed.duration <= TRAJECTORY_MAX_DURATION:
        raise RetimeError(f"the move would take {timed.duration:.0f} s, over the {TRAJECTORY_MAX_DURATION:.0f} s "
                          f"a trajectory may last")
    if count > TRAJECTORY_MAX_WAYPOINTS:
        raise RetimeError(f"{count} samples, over the {TRAJECTORY_MAX_WAYPOINTS} a trajectory may have")
    speed, acceleration = sampled_limits(timed.positions, timed.duration, timed.velocities)
    cubic, cubic_speed = executed_limits(timed.positions, timed.velocities, spacing)
    speed, acceleration = max(speed, cubic_speed), max(acceleration, float(np.max(cubic)))
    if speed > TRAJECTORY_MAX_JOINT_SPEED or acceleration > TRAJECTORY_MAX_JOINT_ACCELERATION:
        raise RetimeError(f"the timed path needs {np.degrees(speed):.0f} deg/s and {np.degrees(acceleration):.0f} "
                          f"deg/s^2, over the arm's limits")
    cut = max((_corner_cut(line, samples, k, spacing) for k in _segments_near_bends(line, samples)), default=0.0)
    if cut > CORNER_CUT:
        raise RetimeError(f"the timed path leaves the planned one by {np.degrees(cut):.3f}°, over "
                          f"{np.degrees(CORNER_CUT):.2f}°")
    if np.any(timed.velocities[-1] != 0.0):
        raise RetimeError("the timed path does not end at rest")
    if not (np.array_equal(timed.positions[0], line.rows[0]) and np.array_equal(timed.positions[-1], line.rows[-1])):
        raise RetimeError("the timed path does not start and end exactly at the path's ends")
