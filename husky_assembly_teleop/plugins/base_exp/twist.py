"""
Constant commands: how the robot's response changes with speed and turn rate, driven open loop on a command grid.

- A constant-command run holds one command (v, ω), a cell of the grid, for SEGMENT seconds from standstill, then
  stops; the robot drives back near the series' start pose between runs. Its steady part gives that cell's speed and
  steering efficiency and xICR; its start gives the step response (`analysis/commands.py`).
- `TwistGrid` hands out the cells (`grid`), the least attempted first, ties at random, so every round comes in a new
  order; it resumes from the experiment's earlier runs with the same setup.
- `twist_plan` places a run at the robot as a Plan (the ideal model's prediction), for the clearance check, the
  preview line and the record. Plain math, no ROS.
"""

from __future__ import annotations

import math
import random
from collections import Counter
from typing import Iterable, Iterator

import numpy as np
from crl_husky.base_model import BaseModel
from crl_husky.follower_path import FollowerPath

from .plan import Plan

#: The template name constant-command runs are recorded under (experiment.json "template").
TWIST = "Twist"
#: Seconds each command is held: the platform's ramp and delay, then a steady part long enough to average.
SEGMENT = 4.0
#: Commanded speeds (m/s) and turn rates (rad/s) of the grid; every pair of them, both turn directions.
SPEEDS = (0.0, 0.1, 0.2, 0.3)
TURN_RATES = (0.15, 0.3, 0.5, 0.8)
#: Extra cells: straights (speed efficiency alone) and reversing (is it the same backwards?).
STRAIGHTS = (0.1, 0.2, 0.3)
REVERSE = (-0.2, (0.3, 0.8))
#: Wheel track and the fastest a wheel may go (m/s), as the follower's limit: cells beyond it are left out.
WHEEL_BASE, MAX_WHEEL_SPEED = 0.555, 0.5
#: Prediction step for the segment's drawing, seconds.
STEP = 0.05


def grid() -> list[tuple[float, float]]:
    """Every cell (v, ω) of the command grid, within the wheel speed limit."""
    cells = [(v, sign * w) for v in SPEEDS for w in TURN_RATES for sign in (1.0, -1.0)]
    cells += [(v, 0.0) for v in STRAIGHTS]
    reverse, rates = REVERSE
    cells += [(reverse, sign * w) for w in rates for sign in (1.0, -1.0)]
    return [(v, w) for v, w in cells if abs(v) + abs(w) * WHEEL_BASE / 2 <= MAX_WHEEL_SPEED + 1e-9]


def settings_of(v: float, w: float) -> dict:
    """A cell as run settings (saved as the run's "template"); speeds as caps, so the record shows them."""
    return {"name": TWIST, "parameters": {"v": v, "w": w, "duration": SEGMENT}, "mode": TWIST,
            "linear_speed": abs(v), "turn_rate": abs(w)}


def cell_of(settings: dict) -> tuple[float, float] | None:
    """The cell (v, ω) of a run's settings, or None if it is no constant-command run."""
    if settings.get("name") != TWIST:
        return None
    parameters = settings["parameters"]
    return round(float(parameters["v"]), 3), round(float(parameters["w"]), 3)


def twist_plan(serial: str, v: float, w: float, anchor) -> Plan:
    """A run from `anchor` as the ideal robot would drive it: a straight, an arc, or a turn on the spot."""
    model = BaseModel()
    steps = int(round(SEGMENT / STEP))
    poses = np.empty((steps + 1, 3))
    poses[0] = anchor
    for k in range(steps):
        poses[k + 1] = model.step(poses[k], v, w, STEP)
    label = f"Twist v={v:+.2f} m/s w={math.degrees(w):+.0f}°/s"
    return Plan(serial, poses, None, FollowerPath.from_poses(*poses.T), SEGMENT, label, "twist", settings_of(v, w))


class TwistGrid:
    """Hands out the command grid's cells, the least attempted first (ties at random), and counts the attempts."""

    def __init__(self, attempts: Iterable[dict], rng: random.Random):
        """Start from the settings of earlier runs in the same experiment and setup, so a series resumes."""
        self._cells = grid()
        self._rng = rng
        self._attempts: Counter = Counter()
        for settings in attempts:
            self.count(settings, True)

    def count(self, settings: dict, done: bool) -> None:
        """Count a run driven to its end."""
        cell = cell_of(settings)
        if cell is not None and done:
            self._attempts[cell] += 1

    def candidates(self, per_template: int = 0) -> Iterator[dict]:
        """The cells as settings, the least attempted first; `per_template` is unused (as `Sampler.candidates`)."""
        cells = list(self._cells)
        self._rng.shuffle(cells)
        for v, w in sorted(cells, key=lambda cell: self._attempts[_key(cell)]):
            yield settings_of(v, w)

    @property
    def summary(self) -> str:
        """Where the grid stands, e.g. "round 2: 12/37 commands"."""
        attempts = [self._attempts[_key(cell)] for cell in self._cells]
        low = min(attempts)
        return f"round {low + 1}: {sum(n > low for n in attempts)}/{len(attempts)} commands"


def _key(cell: tuple[float, float]) -> tuple[float, float]:
    """A cell rounded as `cell_of` rounds it."""
    return round(cell[0], 3), round(cell[1], 3)
