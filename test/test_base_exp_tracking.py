"""The monitor's own tracking errors: zero on the path, signed sideways, drift on a turn, lag on a timed path."""

import math

import numpy as np
import pytest
from crl_husky.follower_path import FollowerPath

from husky_assembly_teleop.plugins.base_exp.templates import arc, drive_turn_drive, timing, turn
from husky_assembly_teleop.plugins.base_exp.tracking import Tracker


def _follow(tracker: Tracker, poses, offset=(0.0, 0.0), dt=0.05):
    """Feed poses (n x 3), shifted by `offset`, at `dt` apart; return the samples."""
    return [tracker.update(x + offset[0], y + offset[1], yaw, i * dt, i * dt) for i, (x, y, yaw) in enumerate(poses)]


def test_on_the_path_every_error_is_zero():
    """Poses on the path, a drive, a turn and a drive, give no error, and the turn is recognised."""
    poses = drive_turn_drive(1.0, math.pi / 2, 1.0)
    path = FollowerPath.from_poses(*poses.T)
    samples = _follow(Tracker(path), path.polyline())
    assert max(abs(s.position) for s in samples) < 1e-6
    assert max(abs(s.heading) for s in samples) < 1e-6
    assert any(s.turning for s in samples) and not samples[0].turning
    assert samples[-1].progress == pytest.approx(1.0)


def test_sideways_offset_is_signed_and_curvature_known():
    """5 cm to the left of a left arc of radius 1 m reads +5 cm, at curvature 1/m."""
    path = FollowerPath.from_poses(*arc(1.0, math.pi / 2).T)
    poses = path.polyline()
    left = np.column_stack([-np.sin(poses[:, 2]), np.cos(poses[:, 2])]) * 0.05
    tracker = Tracker(path)
    samples = [tracker.update(x + lx, y + ly, yaw, i * 0.05, i * 0.05)
               for i, ((x, y, yaw), (lx, ly)) in enumerate(zip(poses, left))]
    middle = samples[len(samples) // 2]
    assert middle.position == pytest.approx(0.05, abs=2e-3)
    assert middle.curvature == pytest.approx(1.0, rel=0.05)


def test_drift_on_a_turn_is_the_distance_to_the_spot():
    """Turning 3 cm away from the spot reads 3 cm, whichever way."""
    path = FollowerPath.from_poses(*turn(math.pi).T)
    samples = _follow(Tracker(path), path.polyline(), offset=(0.0, -0.03))
    assert all(s.turning for s in samples)
    assert samples[len(samples) // 2].position == pytest.approx(0.03, abs=1e-6)


def test_timed_lag_is_the_along_track_error():
    """Standing still at the start of a timed straight lags by speed times elapsed time."""
    poses = np.column_stack([np.linspace(0, 1, 21), np.zeros(21), np.zeros(21)])
    path = FollowerPath.from_poses(*poses.T, timing(poses, 0.2, 0.5))
    tracker = Tracker(path)
    sample = tracker.update(0.0, 0.0, 0.0, 2.0, elapsed=2.0)
    assert sample.along == pytest.approx(0.4, abs=0.01)
