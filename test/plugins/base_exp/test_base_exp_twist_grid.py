"""Constant commands: the command grid, its chooser, and the planned arc of a run."""

import random

import numpy as np

from husky_assembly_teleop.plugins.base_exp.twist import (MAX_WHEEL_SPEED, SEGMENT, WHEEL_BASE, TwistGrid, cell_of,
                                                          grid, settings_of, twist_plan)


def test_grid_keeps_the_wheel_limit_and_both_directions():
    """Every cell within the wheel speed limit; every turning cell also the other way round."""
    cells = grid()
    assert all(abs(v) + abs(w) * WHEEL_BASE / 2 <= MAX_WHEEL_SPEED + 1e-9 for v, w in cells)
    assert all((v, -w) in cells for v, w in cells)
    assert cell_of(settings_of(0.2, -0.3)) == (0.2, -0.3)


def test_chooser_repeats_the_grid_evenly():
    """The least attempted cell comes first, so a second round starts only once the first is complete."""
    chooser = TwistGrid([], random.Random(0))
    seen = []
    for _ in range(len(grid()) + 3):
        settings = next(chooser.candidates())
        seen.append(cell_of(settings))
        chooser.count(settings, True)
    assert len(set(seen[:len(grid())])) == len(grid())
    assert chooser.summary.startswith("round 2: 3/")


def test_plan_predicts_the_ideal_arc():
    """The drawn run is the ideal robot's: on an arc of radius v / ω."""
    plan = twist_plan("a", 0.2, 0.5, (0.0, 0.0, 0.0))
    centre = np.array([0.0, 0.2 / 0.5])
    assert np.allclose(np.hypot(*(plan.poses[:, :2] - centre).T), 0.4, atol=1e-2)
    assert plan.expected == SEGMENT
