"""The mocap probe's coverage map: every sample kept, smoothed near them, coloured by error, exported and loaded."""

from __future__ import annotations

import json

import numpy as np
import pytest

from husky_assembly_teleop.world.coverage_map import (COVERAGE_BIN, COVERAGE_SIZE, ERROR_RED, UNCOVERED_COLOR,
                                                      UNTRACKED_COLOR, CoverageMap, coverage_color, error_color,
                                                      map_colors)
from husky_assembly_teleop.world.mocap import MARKER_ERROR_LIMIT, MARKER_ERROR_WARN


def walk(coverage: CoverageMap, start: tuple, end: tuple, error: float | None, samples: int = 100) -> None:
    """Add `samples` evenly spaced along a straight walk at 1 m height, all with `error`."""
    for t, (x, y) in enumerate(np.linspace(start, end, samples)):
        coverage.add((x, y, 1.0), error, float(t))


def test_keeps_every_sample():
    """Inside the map or not, tracked or not; the rate and error over all of them."""
    coverage = CoverageMap()
    coverage.add((0.1, 0.1, 1.0), 1e-3, 1.0)
    coverage.add((0.2, 0.3, 1.0), None, 2.0)
    coverage.add((9.0, 9.0, 1.0), 3e-3, 3.0)
    assert coverage.count == 3 and coverage.rows[1, 4] == 2.0 and np.isnan(coverage.rows[1, 3])
    assert coverage.tracked_rate() == pytest.approx(2 / 3)
    assert coverage.total_error() == (pytest.approx(2e-3), pytest.approx(1e-3))


def test_grows_past_its_buffer():
    """Thousands of samples, all kept in order."""
    coverage = CoverageMap()
    walk(coverage, (-1.0, 0.0), (1.0, 0.0), 5e-4, samples=5000)
    assert coverage.count == 5000 and coverage.rows[-1, 4] == 4999.0


def test_map_shows_near_samples_only():
    """A walk is shown along it with its error, not a metre away, nor beyond the map's edge."""
    coverage = CoverageMap()
    walk(coverage, (-2.0, 0.0), (2.0, 0.0), 5e-4)
    walk(coverage, (COVERAGE_SIZE[0], 0.0), (COVERAGE_SIZE[0] + 1, 0.0), 5e-4)
    smoothed = coverage.smoothed()
    tracked, error = smoothed.at(0.0, 0.1)
    assert tracked == pytest.approx(1.0) and error == pytest.approx(5e-4)
    assert smoothed.at(0.0, 1.0) is None and smoothed.at(-3.0, 0.0) is None
    assert smoothed.seen.sum() * COVERAGE_BIN ** 2 < 4.0 * 1.5


def test_lost_samples_turn_the_map_grey():
    """Where the probe was never tracked the map is grey; half lost is half way to grey."""
    coverage = CoverageMap()
    walk(coverage, (-3.0, 0.0), (-1.0, 0.0), None)
    walk(coverage, (1.0, 0.0), (3.0, 0.0), 5e-4)
    walk(coverage, (1.0, 0.0), (3.0, 0.0), None)
    smoothed = coverage.smoothed()
    lost, half = smoothed.at(-2.0, 0.0), smoothed.at(2.0, 0.0)
    assert lost[0] == 0.0 and np.isnan(lost[1])
    assert half[0] == pytest.approx(0.5) and half[1] == pytest.approx(5e-4)
    colors = map_colors(smoothed)
    assert tuple(colors[smoothed.seen & (smoothed.tracked == 0.0)][0]) == UNTRACKED_COLOR
    assert tuple(colors[~smoothed.seen][0]) == UNCOVERED_COLOR


def test_standing_still_does_not_drown_out_a_walk_past():
    """A thousand bad samples in one spot next to ten good ones in the next bin: the good bin stays nearer good."""
    coverage = CoverageMap()
    for t in range(1000):
        coverage.add((0.1, 0.1, 1.0), 3e-3, float(t))
    for t in range(10):
        coverage.add((0.1 + COVERAGE_BIN, 0.1, 1.0), 5e-4, float(t))
    assert coverage.smoothed().at(0.1 + COVERAGE_BIN, 0.1)[1] < 2e-3


def test_export_and_load_round_trip():
    """A loaded map has the same samples, lost ones included."""
    coverage = CoverageMap()
    coverage.add((1.3, -2.1, 0.9), 1e-3, 10.5)
    coverage.add((1.4, -2.2, 0.9), None, 11.0)
    loaded = CoverageMap.from_json(json.loads(json.dumps(coverage.to_json())))
    assert np.allclose(loaded.rows, coverage.rows, equal_nan=True)


def test_load_refuses_other_files():
    """A probe points file, or a coverage file from before the samples were kept, is not loaded."""
    with pytest.raises(ValueError):
        CoverageMap.from_json({"samples": []})
    with pytest.raises(ValueError, match="older"):
        CoverageMap.from_json({"kind": "mocap_coverage", "cells": []})


def test_error_colors():
    """Deeper green the lower the error, amber past the chip's threshold, red past the limit, grey without a fix."""
    best, near, over, worse = (error_color(e) for e in (0.0, MARKER_ERROR_WARN, MARKER_ERROR_WARN * 1.01,
                                                        MARKER_ERROR_LIMIT))
    assert best[1] > best[0] and near[1] > near[0] and sum(best) < sum(near)
    assert over[0] > over[1] > over[2] and worse[0] > worse[1] and worse[1] < over[1] - 50
    assert error_color(MARKER_ERROR_LIMIT * 1.01) == ERROR_RED
    assert error_color(None) == UNTRACKED_COLOR


def test_colors():
    """The views: red for a low fraction of cameras, green for all, grey for none."""
    red, green = coverage_color(0.0), coverage_color(1.0)
    assert red[0] > red[1] and green[1] > green[0]
    assert coverage_color(None) == UNCOVERED_COLOR
