"""The mocap probe's coverage grid and points files survive an export and load."""

from __future__ import annotations

import json
from dataclasses import asdict

import pytest

from husky_assembly_teleop.plugins.mocap_probe import (COVERAGE_SIZE, UNCOVERED_COLOR, CoverageGrid, Sample,
                                                       coverage_color, samples_from_json)


def test_cells_cover_the_grid():
    """Centred on the mocap origin, lower edges inside, upper edges outside."""
    grid = CoverageGrid.default()
    (nx, ny), cell = grid.shape, grid.cell_size
    assert (nx * cell, ny * cell) == COVERAGE_SIZE
    assert grid.cell(-COVERAGE_SIZE[0] / 2, -COVERAGE_SIZE[1] / 2) == (0, 0)
    assert grid.cell(COVERAGE_SIZE[0] / 2 - 0.01, COVERAGE_SIZE[1] / 2 - 0.01) == (nx - 1, ny - 1)
    assert grid.cell(COVERAGE_SIZE[0] / 2, 0.0) is None
    centers = [grid.center(i, j) for i, j in ((0, 0), (nx - 1, ny - 1))]
    assert centers[0] == pytest.approx(tuple(-c for c in centers[1]))


def test_rate_counts_tracked_messages():
    """The rate is tracked / samples; outside the grid nothing is counted."""
    grid = CoverageGrid.default()
    grid.add(0.1, 0.1, error=1e-3)
    grid.add(0.2, 0.3, error=None)
    assert grid.add(9.0, 9.0, error=1e-3) is None
    assert grid.rate(*grid.cell(0.1, 0.1)) == pytest.approx(0.5)
    assert grid.rate(0, 0) is None  # a corner cell, never visited
    assert grid.samples.sum() == 2


def test_export_and_load_round_trip():
    """A loaded map has the same grid and counts as the exported one."""
    grid = CoverageGrid.default()
    grid.add(1.3, -2.1, error=1e-3)
    grid.add(1.4, -2.2, error=3e-3)
    loaded = CoverageGrid.from_json(json.loads(json.dumps(grid.to_json())))
    assert (loaded.origin, loaded.cell_size, loaded.shape) == (grid.origin, grid.cell_size, grid.shape)
    for name in ("samples", "tracked", "good", "error_sum", "error_sq_sum"):
        assert getattr(loaded, name) == pytest.approx(getattr(grid, name))


def test_error_mean_and_std():
    """Over the tracked messages only; good counts those under the warning threshold."""
    grid = CoverageGrid.default()
    for error in (1e-3, 3e-3, None):
        grid.add(0.5, 0.5, error=error)
    mean, std = grid.error(*grid.cell(0.5, 0.5))
    assert (mean, std) == (pytest.approx(2e-3), pytest.approx(1e-3))
    assert grid.good[grid.cell(0.5, 0.5)] == 1
    assert grid.error(0, 0) is None
    assert grid.total_error() == (pytest.approx(2e-3), pytest.approx(1e-3))


def test_load_refuses_other_files():
    """A probe points file is not a coverage map."""
    with pytest.raises(ValueError):
        CoverageGrid.from_json({"samples": []})


def test_colors():
    """Red when never tracked, green when always, grey without samples."""
    red, green = coverage_color(0.0), coverage_color(1.0)
    assert red[0] > red[1] and green[1] > green[0]
    assert coverage_color(None) == UNCOVERED_COLOR


def test_points_round_trip():
    """Samples read back from an export equal the exported ones, apart from the id."""
    sample = Sample(id=7, name="table_2", label="table", position=[1.0, 2.0, 0.5], orientation=[0.0, 0.0, 0.0, 1.0],
                    mocap_id=1863, marker_error=4e-4, stamp=12.5)
    data = json.loads(json.dumps({"samples": [asdict(sample)]}))
    assert asdict(samples_from_json(data)[0]) == {**asdict(sample), "id": 0}


def test_points_refuse_other_files():
    """A coverage map is not a points file, and a broken sample is reported."""
    with pytest.raises(ValueError):
        samples_from_json(CoverageGrid.default().to_json())
    with pytest.raises(ValueError):
        samples_from_json({"samples": [{"name": "p_1"}]})
