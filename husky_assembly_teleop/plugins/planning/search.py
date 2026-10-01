"""
RRT-Connect (`birrt`) between two states: the planner supplies sampling, stepping, distance and collision checks.

! Runs on a planner's worker thread; the main thread ends it early through `abort`.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Generic, Sequence, TypeVar

from pybullet_planning.motion_planners.rrt_connect import birrt
from pybullet_planning.motion_planners.utils import waypoints_from_path

#: Give up searching after this long, seconds.
TIME_LIMIT = 10.0
#: RRT-Connect iterations per attempt, extra attempts, and smoothing iterations.
MAX_ITERATIONS = 2000
RESTARTS = 2
SMOOTHING = 100

State = tuple[float, ...]
P = TypeVar("P")


class PlanningAborted(Exception):
    """Raised inside a search to end it early."""


@dataclass
class PlanResult(Generic[P]):
    """What a search came back with.

    Attributes:
        path: The path, or None if none was found.
        reason: Why not, when `path` is None; empty otherwise.
        seconds: How long it took, syncing the world included.
        direct: Whether the straight move was already free.
    """

    path: P | None
    reason: str = ""
    seconds: float = 0.0
    direct: bool = False


@dataclass
class Corners:
    """The corners of a collision-free path, or why there is none.

    Attributes:
        states: Start, the corners in between, and exactly the goal; None if no path.
        reason: Why not, when `states` is None.
        direct: Whether the straight move was already free.
    """

    states: list[State] | None
    reason: str = ""
    direct: bool = False


def connect(start: State, goal: State, distance: Callable[[State, State], float], sample: Callable[[], State],
            extend: Callable[[State, State], Sequence[State]], collides: Callable[[State], bool],
            abort: threading.Event) -> Corners:
    """Find a collision-free path from `start` to `goal`, trying the straight move first.

    Args:
        start: Where it is; must itself be free.
        goal: Where it should end; must itself be free.
        distance: Cost of the move between two states.
        sample: A random state to grow towards.
        extend: The states from one to another (first excluded, last included),
            close enough together that checking each checks the move.
        collides: Whether a state is in collision.
        abort: Set from the main thread to end the search early.

    Returns:
        Corners: The path's corners, or why there is none ("cancelled" when aborted).
    """
    def collision_fn(q, **_kwargs) -> bool:
        # birrt may pass a `diagnosis` flag; ignored.
        if abort.is_set():
            raise PlanningAborted()
        return collides(q)

    try:
        direct = not any(collision_fn(q) for q in extend(start, goal))
        path = [start, goal] if direct else birrt(
            start, goal, distance, sample, extend, collision_fn, max_time=TIME_LIMIT,
            max_iterations=MAX_ITERATIONS, restarts=RESTARTS, smooth=SMOOTHING)
    except PlanningAborted:
        return Corners(None, "cancelled")
    if path is None:
        return Corners(None, f"no path found in {TIME_LIMIT:.0f} s")

    # birrt returns every small step, maybe without the start: keep the corners, ending exactly at the goal.
    if tuple(path[0]) != tuple(start):
        path = [start] + list(path)
    return Corners(waypoints_from_path(path)[:-1] + [goal], direct=direct)
