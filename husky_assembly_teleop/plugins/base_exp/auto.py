"""
Automated base experiments: which test paths to drive, and clearance checks against everything around the robot.

- The grid: per template a few equally spaced numbers (both directions), all at one SPEED, geometric.
- `StandardSet` drives STANDARD_SET, a few grid cells that are the same for every scenario, the least attempted first,
  so scenarios compare path by path.
- `Sampler` draws from the whole grid, balanced within one collection run: the template with the fewest successful
  runs first; within it the least covered cell, and among those the one whose numbers were used least; ties at random.
- `Clearance` checks a path, or the robot where it stands, in a private PyBullet mirror of the scene: the whole robot
  (arms included) against other robots and scene bodies, e.g. the obstacles plugin's furniture and border walls.

! `Clearance` runs every check on its own worker thread: a mirror belongs to one thread.
"""

from __future__ import annotations

import asyncio
import itertools
import math
import random
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
from typing import Iterable, Iterator

import numpy as np
import pybullet as p

from ...world.mirrors.pybullet import PyBulletMirror
from ...world.scene import SceneSnapshot

#: A path must keep this far from everything, metres: tracking error plus a safety margin.
PATH_CLEARANCE = 0.3
#: During a run the robot is stopped when it comes this close to anything, metres.
STOP_DISTANCE = 0.1
#: Checking resolution along a path: metres driven, radians turned.
CHECK_STEP, CHECK_TURN = 0.05, math.radians(5.0)

#: The one speed every automated run uses: linear (m/s) and turn rate (°/s) caps. Speed is held fixed so runs differ
#: only in the path; at these caps the tightest curve the robot can follow has a radius of about 0.38 m.
SPEED = (0.2, 30.0)
#: The mode every automated run uses: every controller can follow a geometric path. Manual runs can still be timed.
MODE = "Geometric"
#: Equally spaced numbers per template, as in the panel (metres, degrees); negative: backwards or clockwise. Every
#: curve keeps a radius of 0.5 m or more, so it can be followed at SPEED (sines: radius wavelength² / (4π² amplitude)).
GRID = {
    "Straight": {"length": (-2.0, -1.5, -1.0, -0.5, 0.5, 1.0, 1.5, 2.0)},
    "Turn on the spot": {"angle": (-360.0, -270.0, -180.0, -90.0, 90.0, 180.0, 270.0, 360.0)},
    "Arc": {"radius": (0.5, 1.0), "angle": (-180.0, -90.0, 90.0, 180.0)},
    "Sine": {"length": (1.0, 2.0), "amplitude": (0.1, 0.2), "wavelength": (2.0,)},
    "Drive, turn, drive": {"length": (0.5, 1.0), "angle": (-180.0, -90.0, 90.0, 180.0), "length_2": (0.5, 1.0)},
}
#: The standard paths: grid cells driven in every scenario, in this order. Every segment type (straight, curve, tight
#: curve, spot turn) in both directions. ! Change it only between experiments: scenarios compare on the same paths.
STANDARD_SET = (
    ("Straight", {"length": 1.0}),
    ("Straight", {"length": -1.0}),
    ("Turn on the spot", {"angle": 180.0}),
    ("Turn on the spot", {"angle": -180.0}),
    ("Arc", {"radius": 1.0, "angle": 90.0}),
    ("Arc", {"radius": 1.0, "angle": -90.0}),
    ("Arc", {"radius": 0.5, "angle": 90.0}),
    ("Arc", {"radius": 0.5, "angle": -90.0}),
    ("Sine", {"length": 2.0, "amplitude": 0.1, "wavelength": 2.0}),
    ("Drive, turn, drive", {"length": 0.5, "angle": 90.0, "length_2": 0.5}),
)


def cells(name: str) -> list[dict]:
    """Every grid cell of one template, as settings (see `plan.plan_from`)."""
    grid = GRID[name]
    return [_settings(name, dict(zip(grid, numbers))) for numbers in itertools.product(*grid.values())]


def standard_set() -> list[dict]:
    """The standard paths as settings, in order."""
    return [_settings(name, parameters) for name, parameters in STANDARD_SET]


def _settings(name: str, parameters: dict) -> dict:
    """Settings of an automated run: the template's numbers at SPEED, in MODE."""
    linear, turn = SPEED
    return {"name": name, "parameters": dict(parameters), "mode": MODE, "linear_speed": linear,
            "turn_rate": math.radians(turn)}


def levels(settings: dict) -> list[tuple]:
    """The levels a cell uses, one per number: (template, number, value)."""
    name = settings["name"]
    return [(name, key, round(float(value), 3)) for key, value in sorted(settings["parameters"].items())]


def cell_key(settings: dict) -> tuple:
    """What makes two settings the same grid cell: template and numbers."""
    return (settings["name"], *levels(settings))


class Sampler:
    """Hands out grid cells, the least covered first, and counts the successful ones."""

    def __init__(self, successes: Iterable[dict], rng: random.Random):
        """Start from the settings of earlier successful runs in the same setup.

        Args:
            successes: Settings of runs that ended done; off-grid ones still count for their template.
            rng: Breaks ties.
        """
        self._rng = rng
        self._templates: Counter = Counter()
        self._cells: Counter = Counter()
        self._levels: Counter = Counter()
        for settings in successes:
            self.add(settings)

    def add(self, settings: dict) -> None:
        """Count one more successful run."""
        self._templates[settings["name"]] += 1
        self._cells[cell_key(settings)] += 1
        self._levels.update(levels(settings))

    def count(self, settings: dict, done: bool) -> None:
        """Count a run the controller drove: only successful ones balance the grid."""
        if done:
            self.add(settings)

    def candidates(self, per_template: int) -> Iterator[dict]:
        """Settings to try in order: templates by fewest successes, within each its `per_template` least covered cells.

        Cells come by their own count, then by how often their levels were used; ties are shuffled.
        """
        def cell_order(cell: dict) -> tuple[int, int]:
            return self._cells[cell_key(cell)], sum(self._levels[level] for level in levels(cell))

        for name in self._least(GRID, lambda name: self._templates[name]):
            yield from itertools.islice(self._least(cells(name), cell_order), per_template)

    @property
    def counts(self) -> dict[str, int]:
        """Successful runs per template."""
        return {name: self._templates[name] for name in GRID}

    @property
    def summary(self) -> str:
        """Successful runs per template in short, e.g. "done: Straight 3, Turn 2, Arc 2, Sine 1, DTD 2"."""
        short = {"Turn on the spot": "Turn", "Drive, turn, drive": "DTD"}
        return "done: " + ", ".join(f"{short.get(name, name)} {count}" for name, count in self.counts.items())

    def _least(self, items: Iterable, count) -> list:
        """`items` by `count`, ties in random order."""
        items = list(items)
        self._rng.shuffle(items)
        return sorted(items, key=count)


class StandardSet:
    """Hands out the standard paths, the least attempted first (ties in set order), and counts the attempts."""

    def __init__(self, attempts: Iterable[dict] = ()):
        """Start from the settings of earlier attempts in the same experiment and setup, so a series resumes.

        Only attempts at SPEED and in MODE count, as in the analysis.
        """
        self._paths = standard_set()
        self._attempts: Counter = Counter()
        for settings in attempts:
            if settings.get("mode") == MODE and math.isclose(settings.get("linear_speed", 0.0), SPEED[0]):
                self.count(settings, True)

    def count(self, settings: dict, done: bool) -> None:
        """Count a run the controller drove, done or failed: every path gets the same number of attempts."""
        self._attempts[cell_key(settings)] += 1

    def candidates(self, per_template: int = 0) -> Iterator[dict]:
        """The standard paths, the least attempted first; `per_template` is unused (as `Sampler.candidates`)."""
        yield from sorted(self._paths, key=lambda settings: self._attempts[cell_key(settings)])

    @property
    def summary(self) -> str:
        """Where the standard paths stand, e.g. "round 2: 3/10 paths"."""
        attempts = [self._attempts[cell_key(settings)] for settings in self._paths]
        low = min(attempts)
        return f"round {low + 1}: {sum(n > low for n in attempts)}/{len(attempts)} paths"


def check_poses(polyline: np.ndarray) -> np.ndarray:
    """Poses to check along a path drawing (n x 3): a pose every CHECK_STEP driven or CHECK_TURN turned."""
    picked, moved = [polyline[0]], 0.0
    for previous, pose in zip(polyline[:-1], polyline[1:]):
        moved += max(math.hypot(*(pose[:2] - previous[:2])) / CHECK_STEP, abs(pose[2] - previous[2]) / CHECK_TURN)
        if moved >= 1.0 - 1e-9:
            picked.append(pose)
            moved = 0.0
    if not np.array_equal(picked[-1], polyline[-1]):
        picked.append(polyline[-1])
    return np.array(picked)


class Clearance:
    """Collision checks of one robot against the rest of the world, in its own mirror on its own worker thread."""

    def __init__(self):
        """Start the worker; the mirror is made on the worker at the first check."""
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="base-exp-clearance")
        self._mirror: PyBulletMirror | None = None

    async def path_hit(self, snapshot: SceneSnapshot, serial: str, poses: np.ndarray,
                       margin: float = PATH_CLEARANCE) -> str | None:
        """What the robot would come within `margin` of along `poses` (n x 3), or None if the path is clear."""
        return await self._run(self._path_hit, snapshot, serial, check_poses(poses), margin)

    async def pose_hit(self, snapshot: SceneSnapshot, serial: str, pose, margin: float = STOP_DISTANCE) -> str | None:
        """What the robot at floor `pose` is within `margin` of, or None."""
        return await self._run(self._path_hit, snapshot, serial, np.array([pose]), margin)

    def close(self) -> None:
        """Close the mirror and stop the worker."""
        self._executor.submit(self._close_mirror)
        self._executor.shutdown(wait=True)

    async def _run(self, function, *args):
        """Run `function` on the worker and wait for it."""
        return await asyncio.get_running_loop().run_in_executor(self._executor, function, *args)

    def _path_hit(self, snapshot: SceneSnapshot, serial: str, poses: np.ndarray, margin: float) -> str | None:
        """Worker: sync the mirror to the snapshot, put the robot at each pose, and name the first thing it hits."""
        if self._mirror is None:
            self._mirror = PyBulletMirror()
        self._mirror.sync(snapshot)
        body, client = self._mirror.robot(serial), self._mirror.client_id
        z = p.getBasePositionAndOrientation(body, physicsClientId=client)[0][2]
        for x, y, yaw in poses:
            # ? Moving a robot is fine: the mirror re-poses robots on every sync.
            p.resetBasePositionAndOrientation(body, (x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
                                              physicsClientId=client)
            hits = self._mirror.collisions(serial, margin)
            if hits:
                return snapshot.label(hits[0])
        return None

    def _close_mirror(self) -> None:
        """Worker: close the mirror, if one was made."""
        if self._mirror is not None:
            self._mirror.close()
            self._mirror = None
