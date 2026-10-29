"""
RRT-Connect (`birrt`) between two states: the planner supplies sampling, stepping, distance and collision checks.

Each path found is shortened by jumping straight over the corners it can; of several searches the shortest is kept.

! Runs on a planner's worker thread; the main thread ends it early through `abort`.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Sequence

from pybullet_planning.motion_planners.rrt_connect import birrt
from pybullet_planning.motion_planners.utils import waypoints_from_path

from .planner_search import TIME_LIMIT, PlanningAborted

#: RRT-Connect iterations per attempt, extra attempts, and smoothing iterations.
MAX_ITERATIONS = 2000
RESTARTS = 2
SMOOTHING = 100

State = tuple[float, ...]


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
            abort: threading.Event, time_limit: float = TIME_LIMIT, attempts: int = 1,
            improve_time: float = 0.0, draw_fn: Callable | None = None) -> Corners:
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
        time_limit: Give up searching after this long, seconds.
        attempts: Searches to keep the shortest path of, by `distance` between corners; more avoid the odd detour.
        improve_time: Seconds after which no further search starts once a path is found.
        draw_fn: birrt's hook for each sample and node (`SearchTree.draw_fn`), or None.

    Returns:
        Corners: The path's corners, or why there is none ("cancelled" when aborted).
    """
    def collision_fn(q, **_kwargs) -> bool:
        # birrt may pass a `diagnosis` flag; ignored.
        if abort.is_set():
            raise PlanningAborted()
        return collides(q)

    started = time.time()
    best, best_cost = None, float("inf")
    try:
        if not any(collision_fn(q) for q in extend(start, goal)):
            return Corners([start, goal], direct=True)
        for _ in range(attempts):
            elapsed = time.time() - started
            if elapsed >= time_limit or (best is not None and elapsed >= improve_time):
                break
            path = birrt(start, goal, distance, sample, extend, collision_fn, max_time=time_limit - elapsed,
                         max_iterations=MAX_ITERATIONS, restarts=RESTARTS, smooth=SMOOTHING, draw_fn=draw_fn)
            if path is None:
                break
            # birrt returns every small step, maybe without the start: keep the corners, ending exactly at the goal.
            if tuple(path[0]) != tuple(start):
                path = [start] + list(path)
            corners = _skip_corners(waypoints_from_path(path)[:-1] + [goal], extend, collision_fn)
            cost = sum(distance(a, b) for a, b in zip(corners[:-1], corners[1:]))
            if cost < best_cost:
                best, best_cost = corners, cost
        # * birrt gives up at once when start or goal itself collides (a planner's checks may grow near them).
        blocked = [] if best is not None else [name for name, q in (("start", start), ("target", goal))
                                               if collision_fn(q)]
    except PlanningAborted:
        return Corners(None, "cancelled")
    if blocked:
        return Corners(None, f"the {' and the '.join(blocked)} is too close to something for the search's checks")
    if best is None:
        return Corners(None, f"no path found in {time_limit:.0f} s")
    return Corners(best)


def _skip_corners(corners: list[State], extend: Callable[[State, State], Sequence[State]],
                  collides: Callable[[State], bool]) -> list[State]:
    """From each corner, move straight on to the furthest later corner that a clear move reaches."""
    kept, here = [corners[0]], 0
    while here < len(corners) - 1:
        there = next(there for there in range(len(corners) - 1, here, -1)
                     if there == here + 1 or not any(collides(q) for q in extend(corners[here], corners[there])))
        kept.append(corners[there])
        here = there
    return kept
