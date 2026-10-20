"""Timing planned paths: within the arm's own checks, on the planned polyline, exactly at its ends, ending at rest."""

from __future__ import annotations

import json
import threading
from functools import lru_cache
from pathlib import Path

import numpy as np
import pytest
from plugin_fakes import FakeContext, fake_arm, fake_node

from husky_assembly_teleop.robot_interface.arm import (JOINT_MOVE_MAX_SPEED, TRAJECTORY_MAX_JOINT_ACCELERATION,
                                                       TRAJECTORY_MAX_JOINT_SPEED)
from husky_assembly_teleop.shared.execution.retime import (CORNER_CUT, SPLIT_ACCELERATION, RetimeError, retime,
                                                           sampled_limits)
from husky_assembly_teleop.shared.planning.search import connect

#: Paths per (dimensions, densified) case: 4 cases, 100 paths.
SEEDS = range(25)


@lru_cache(maxsize=None)
def random_path(seed: int, dimensions: int) -> np.ndarray:
    """An RRT-Connect path between random joint values around a few random balls, one across the straight line."""
    rng = np.random.default_rng(seed)
    start, goal = rng.uniform(-2.5, 2.5, (2, dimensions))
    centres = np.vstack([(start + goal) / 2, rng.uniform(-2.5, 2.5, (3, dimensions))])
    radii = np.concatenate([[0.4 * np.linalg.norm(goal - start)], rng.uniform(0.3, 0.8, 3)])
    clear = [np.linalg.norm(centres - end, axis=1) > radii + 0.1 for end in (start, goal)]
    keep = clear[0] & clear[1]
    centres, radii = centres[keep], radii[keep]

    def collides(q) -> bool:
        return bool(np.any(np.linalg.norm(centres - np.asarray(q), axis=1) < radii))

    def extend(a, b):
        a, b = np.asarray(a), np.asarray(b)
        count = max(1, int(np.ceil(np.max(np.abs(b - a)) / 0.1)))
        return [tuple(a + (b - a) * k / count) for k in range(1, count + 1)]

    corners = connect(tuple(start), tuple(goal), lambda a, b: float(np.linalg.norm(np.subtract(a, b))),
                      lambda: tuple(rng.uniform(-np.pi, np.pi, dimensions)), extend, collides, threading.Event(),
                      time_limit=5.0)
    assert corners.states is not None, corners.reason
    return np.array(corners.states)


def densified(path: np.ndarray, step: float = 0.1) -> np.ndarray:
    """The path with points added so no joint moves more than `step` between two rows."""
    rows = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        count = max(1, int(np.ceil(np.max(np.abs(b - a)) / step)))
        rows += [a + (b - a) * k / count for k in range(1, count + 1)]
    return np.array(rows)


def off_polyline(points: np.ndarray, path: np.ndarray) -> np.ndarray:
    """Each point's distance to the nearest segment of the path."""
    a, b = path[:-1][None], path[1:][None]
    p = points[:, None]
    t = np.clip(np.sum((p - a) * (b - a), axis=2) / np.sum((b - a) ** 2, axis=2), 0.0, 1.0)
    return np.min(np.linalg.norm(p - (a + t[..., None] * (b - a)), axis=2), axis=1)


@pytest.mark.parametrize("dimensions", [6, 12])
@pytest.mark.parametrize("dense", [False, True])
def test_random_paths_pass_the_arms_own_checks(dimensions, dense):
    """Every arm accepts its columns; samples on the polyline, exact ends, rest at the end, the time asked or more."""
    ctx = FakeContext()
    node = fake_node(ctx.now, ctx.logs)
    for seed in SEEDS:
        path = random_path(seed, dimensions)
        if dense:
            path = densified(path)
        asked = float(np.random.default_rng(seed + 1000).uniform(1.0, 30.0))
        timed = retime(path, asked)
        assert timed.duration >= asked - 1e-9 and timed.duration >= timed.minimum
        assert np.array_equal(timed.positions[0], path[0]) and np.array_equal(timed.positions[-1], path[-1])
        assert not np.any(timed.velocities[-1])
        assert np.max(off_polyline(timed.positions, path)) < 1e-9
        for first in range(0, dimensions, 6):
            arm = fake_arm(node, f"arm{first}", path[0, first:first + 6])
            columns = slice(first, first + 6)
            assert arm._check_trajectory(timed.positions[:, columns], timed.duration,
                                         timed.velocities[:, columns]) is None, (seed, ctx.lines("error"))


def test_a_gentle_curve_has_no_stop_inside():
    """A quarter circle in 50 waypoints is one run, at full speed, from rest to rest."""
    angles = np.linspace(0.0, np.pi / 2, 50)
    path = np.zeros((50, 6))
    path[:, 0], path[:, 1] = np.cos(angles), np.sin(angles)
    timed = retime(path)
    assert timed.stops == (0, 49) and timed.duration == timed.minimum
    speed, _ = sampled_limits(timed.positions, timed.duration, timed.velocities)
    assert JOINT_MOVE_MAX_SPEED * 0.9 < speed <= JOINT_MOVE_MAX_SPEED + 1e-9


def test_a_sharp_corner_is_a_stop():
    """An L in joint space stops at its corner; the arm is at rest there."""
    path = np.zeros((3, 6))
    path[1, 0] = path[2, 0] = path[2, 1] = 1.0
    timed = retime(path)
    assert timed.stops == (0, 1, 2)
    corner = np.flatnonzero(np.all(timed.positions == path[1], axis=1))
    assert len(corner) == 1 and not np.any(timed.velocities[corner])


def test_repeated_rows_are_dropped():
    """A path with repeated rows times exactly as the path without them."""
    path = np.array([[0.0] * 6, [0.5] + [0.0] * 5, [0.5, 0.5] + [0.0] * 4])
    repeated = path[[0, 0, 1, 1, 1, 2, 2]]
    plain, timed = retime(path, 12.0), retime(repeated, 12.0)
    assert np.array_equal(plain.positions, timed.positions) and plain.stops == timed.stops


def test_a_slower_time_only_slows():
    """Asking for less than the minimum gives the minimum; asking for more stretches every run."""
    path = np.array([[0.0] * 6, [1.0] + [0.0] * 5])
    fastest = retime(path)
    assert retime(path, fastest.minimum / 2).duration == fastest.minimum
    slow = retime(path, 3 * fastest.minimum)
    assert slow.duration >= 3 * fastest.minimum - 1e-9 and slow.minimum == fastest.minimum


@pytest.mark.parametrize("path, duration, why", [
    ([[0.1] * 6, [0.1] * 6], 0.0, "does not move"),
    ([[0.1] * 6], 0.0, "does not move"),
    ([[0.0] * 6, [np.nan] * 6], 0.0, "NaN"),
    ([[0.0] * 6, [0.0, 0.0, 3.5, 0.0, 0.0, 0.0]], 0.0, "past the UR limits"),
    ([[0.0] * 6, [1.0] + [0.0] * 5], 400.0, "over the 300 s"),
])
def test_paths_that_cannot_be_timed_raise(path, duration, why):
    """A path that does not move, is not finite, leaves the limits, or would last over 300 s is refused."""
    with pytest.raises(RetimeError, match=why):
        retime(path, duration)


# --- --- --- --- --- what the controller runs: the cubic between samples (review M4) --- --- --- --- ---

#: The real B20_R_home plan of the headless chain (tamp, purged process): 76 rows of Cindy's 12 joints.
B20_R_HOME = Path(__file__).resolve().parent / "b20_r_home_plan.json"


def executed(timed, points: int = 21) -> tuple[float, float, np.ndarray]:
    """What the controller runs between samples: the cubic Hermite through each sample's position and velocity.

    Returns its largest acceleration (exact: linear in a segment), its largest speed, and dense points on it.
    """
    p, v = timed.positions, timed.velocities
    h = timed.duration / (len(p) - 1)
    dp = p[1:] - p[:-1]
    a0 = (6 * dp - h * (4 * v[:-1] + 2 * v[1:])) / h ** 2
    a1 = (-6 * dp + h * (2 * v[:-1] + 4 * v[1:])) / h ** 2
    t = np.linspace(0.0, 1.0, points)[:, None, None]
    speed = (v[:-1] * (1 - 4 * t + 3 * t ** 2) + v[1:] * (3 * t ** 2 - 2 * t) + dp / h * (6 * t - 6 * t ** 2))
    position = (p[:-1] * (2 * t ** 3 - 3 * t ** 2 + 1) + h * v[:-1] * (t ** 3 - 2 * t ** 2 + t)
                + p[1:] * (3 * t ** 2 - 2 * t ** 3) + h * v[1:] * (t ** 3 - t ** 2))
    return (float(max(np.max(np.abs(a0)), np.max(np.abs(a1)))), float(np.max(np.abs(speed))),
            position.reshape(-1, p.shape[1]))


def max_norm_off(points: np.ndarray, path: np.ndarray) -> float:
    """The largest max-norm distance of the points from the path's polyline (each projected on a nearest segment)."""
    a, b = path[:-1][None], path[1:][None]
    q = points[:, None]
    t = np.clip(np.sum((q - a) * (b - a), axis=2) / np.sum((b - a) ** 2, axis=2), 0.0, 1.0)
    return float(np.max(np.min(np.max(np.abs(q - (a + t[..., None] * (b - a))), axis=2), axis=1)))


def assert_runs_within_limits(path: np.ndarray, timed) -> None:
    """The cubic the controller runs keeps the split acceleration, the speed limit, and CORNER_CUT off the path."""
    acceleration, speed, points = executed(timed)
    assert acceleration <= SPLIT_ACCELERATION + 1e-9 and acceleration <= TRAJECTORY_MAX_JOINT_ACCELERATION
    assert speed <= TRAJECTORY_MAX_JOINT_SPEED
    assert max_norm_off(points, path) <= CORNER_CUT


@pytest.mark.parametrize("dimensions", [6, 12])
@pytest.mark.parametrize("dense", [False, True])
def test_the_executed_cubic_of_random_paths_keeps_the_limits(dimensions, dense):
    """At full speed and slower, the cubic between samples of 25 seeded paths keeps the limits and the path."""
    for seed in SEEDS:
        path = random_path(seed, dimensions)
        if dense:
            path = densified(path)
        for asked in (0.0, float(np.random.default_rng(seed + 2000).uniform(1.0, 60.0))):
            assert_runs_within_limits(path, retime(path, asked))


def test_the_executed_cubic_of_the_real_way_home_keeps_the_limits():
    """The real B20_R_home plan, at full speed and at 10 s asked: the cubic keeps the limits and the planned path."""
    rows = np.array(json.loads(B20_R_HOME.read_text())["rows"])
    for asked in (0.0, 10.0, 40.0):
        timed = retime(rows, asked)
        assert_runs_within_limits(rows, timed)
        assert np.array_equal(timed.positions[0], rows[0]) and np.array_equal(timed.positions[-1], rows[-1])


def test_at_a_fine_spacing_the_acceleration_stops_a_gentle_bend():
    """Sampled every 0.02 s, a 5.7° bend cuts no corner but would need a fast turn of the velocity: it gets a stop."""
    path = np.zeros((3, 6))
    path[1, 0], path[2, 0], path[2, 1] = 1.0, 2.0, 0.1
    timed = retime(path, spacing=0.02)
    assert timed.stops == (0, 1, 2)
    acceleration, _speed, _points = executed(timed)
    assert acceleration <= SPLIT_ACCELERATION + 1e-9


def test_the_hard_check_measures_the_cubic_not_only_the_samples():
    """Samples standing still with a velocity: every sampled limit holds, yet the cubic swings: refused."""
    from husky_assembly_teleop.shared.execution.retime import Retimed, _check, _polyline, _Samples
    rows = np.zeros((2, 6))
    rows[1, 0] = 0.01
    count, spacing = 11, 0.2
    positions = np.linspace(rows[0], rows[1], count)
    velocities = np.zeros_like(positions)
    velocities[1:-1, 0] = 0.4
    timed = Retimed(positions, velocities, (count - 1) * spacing, (count - 1) * spacing, (0, 1))
    speed, acceleration = sampled_limits(positions, timed.duration, velocities)
    assert speed <= TRAJECTORY_MAX_JOINT_SPEED and acceleration <= TRAJECTORY_MAX_JOINT_ACCELERATION
    samples = _Samples(np.array([count - 1]), positions, velocities, positions[:, 0].copy(), np.array([0, count - 1]))
    with pytest.raises(RetimeError, match="over the arm's limits"):
        _check(timed, _polyline(rows), samples, spacing)


def test_a_short_move_is_stretched_until_its_cubic_keeps_the_split_acceleration():
    """A 1.94° single-joint move needs 97°/s^2 on 3 samples: it gets a 4th. Short straight moves up to 40° keep it."""
    path = np.zeros((2, 6))
    path[1, 0] = np.radians(1.94)
    timed = retime(path)
    assert len(timed.positions) == 4 and timed.stops == (0, 1)
    for degrees in np.arange(0.05, 40.0, 0.05):
        for shape in ([1, 0, 0, 0, 0, 0], [1, 0.5, 0, 0, 0, 0]):
            path[1] = np.radians(degrees) * np.array(shape)
            for asked in (0.0, 1.0):
                assert_runs_within_limits(path, retime(path, asked))
