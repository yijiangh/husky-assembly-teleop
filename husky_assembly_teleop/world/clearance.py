"""
Clearances: while a task drives a robot, stop it once it comes too close to anything in the scene.

Optional, like freezes. Whoever puts bodies into the scene (obstacles, process, tracked objects) need not know.

- Checked against the whole tick snapshot, other robots included; allowed contacts (`Body.touches`, what a robot
  holds) are skipped.
- ! What cannot be checked counts as too close: a lost base pose, or a robot the mirror cannot place.
- One worker thread checks one snapshot at a time, a tick or two behind the robot. Not an emergency stop.
- On a trip, `PluginContext.clearance` soft stops the robot (Resume needed) and cancels the task. The monitor keeps
  the last TRIPS_KEPT trips with what came too close (`ClearanceTrip`), for the collisions plugin.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from typing import Any, Hashable

from scaffolding_assembly_core.scene import Scene

from .base_collision import BaseCollisionWorld
from .collision_hit import CollisionHit

#: Default distance, metres, below which a clearance trips.
STOP_DISTANCE = 0.1
#: How many trips the monitor keeps.
TRIPS_KEPT = 10


@dataclass(eq=False)
class Clearance:
    """Robots a task keeps clear of everything, and why it tripped.

    Attributes:
        serials: The robots, by serial.
        margin: Distance below which it trips, metres.
        problem: Why it tripped, or None while it holds.
        hit: What came too close, once tripped; None while it holds, or when it could not be checked.
    """

    serials: tuple[str, ...]
    margin: float
    problem: str | None = None
    hit: CollisionHit | None = None


@dataclass(frozen=True)
class ClearanceTrip:
    """One trip, as the monitor keeps it.

    Attributes:
        time: When it tripped, ROS time in seconds.
        serial: The robot stopped.
        task: The task cancelled, e.g. "base_exp: run path".
        problem: Why, as in `Clearance.problem`.
        hit: What came too close, with the scene as checked; None when the robot could not be checked.
    """

    time: float
    serial: str
    task: str
    problem: str
    hit: CollisionHit | None


class ClearanceChecker:
    """Checks robots against a scene snapshot on its own worker thread, one snapshot at a time."""

    def __init__(self):
        """Start idle; the worker's collision world loads on the first check."""
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="clearance")
        self._world = BaseCollisionWorld()
        self._pending: Future | None = None

    def check(self, snapshot: Scene, asked: list[tuple[Hashable, str, float]]) -> bool:
        """Start checking each (key, robot id, margin) in `snapshot`, unless the last check still runs.

        Returns:
            bool: Whether the check started; False while the last one runs (this snapshot is skipped).
        """
        if self._pending is not None:
            return False
        self._pending = self._executor.submit(self._check, snapshot, list(asked))
        return True

    def finished(self) -> list[tuple[Any, str, CollisionHit | None]] | None:
        """The (key, why, hit) of each robot the last check found too close, once; None while it runs or none started.

        Raises:
            Exception: Whatever the check raised.
        """
        if self._pending is None or not self._pending.done():
            return None
        done, self._pending = self._pending, None
        return done.result()

    def close(self) -> None:
        """Close the collision world and stop the worker, after the running check."""
        self._executor.submit(self._world.close)
        self._executor.shutdown(wait=True)

    def _check(self, snapshot: Scene,
               asked: list[tuple[Hashable, str, float]]) -> list[tuple[Any, str, CollisionHit | None]]:
        """Sync the world to `snapshot` and say why each robot that is too close, or cannot be checked, is. Worker."""
        self._world.sync(snapshot)
        found = []
        for key, own, margin in asked:
            try:
                hit = self._world.collision_hit(own, margin, "clearance")
            except (ValueError, KeyError) as error:
                found.append((key, f"cannot be checked: {error}", None))
                continue
            if hit is not None:
                found.append((key, f"came within {margin * 100:.0f} cm of {snapshot.label(hit.pairs[0][1])}", hit))
        return found
