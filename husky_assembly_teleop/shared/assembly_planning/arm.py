"""
RRT-Connect over one arm's six joints on a synced compas_fab mirror, the path timed at a constant joint speed.

It avoids the robot itself, the other robots and every scene body.

! Worker thread only, on the thread that owns the mirror, after `mirror.sync`.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Sequence

import numpy as np

from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror

from ...robot_interface.arm import JOINT_MOVE_MAX_SPEED, UR_JOINT_LIMITS
from .joints import arm_joint_names
from ..planner.planner_search import TIME_LIMIT, PlanResult
from ..planner.rrt_connect import connect
from ..planner.timed_path import TimedPath

#: Collision-checking resolution along a move, radians per joint.
JOINT_STEP = 0.05
#: Joint speed of the preview, rad/s: the same as `joint_move`'s.
PREVIEW_SPEED = JOINT_MOVE_MAX_SPEED


@dataclass(frozen=True)
class ArmPath(TimedPath):
    """A joint path for one arm, timed at a constant joint speed for the preview.

    `points` are joint values in radians, shape (n, 6), in `joint_names` order.

    Attributes:
        joint_names: The six joints, in the order of each waypoint.
    """

    joint_names: tuple[str, ...]

    @classmethod
    def at_preview_speed(cls, joint_names: Sequence[str], waypoints: np.ndarray) -> ArmPath:
        """Time each segment by its largest joint change at PREVIEW_SPEED.

        Args:
            joint_names: The six joints.
            waypoints: (n, 6) joint values, radians; the first is the start.

        Returns:
            ArmPath: The timed path.
        """
        steps = np.max(np.abs(np.diff(waypoints, axis=0)), axis=1) / PREVIEW_SPEED
        return cls(times=np.concatenate([[0.0], np.cumsum(steps)]), points=waypoints, joint_names=tuple(joint_names))


def _extend(q1: Sequence[float], q2: Sequence[float]) -> list[tuple[float, ...]]:
    """Points from q1 (excluded) to q2 (included), at most JOINT_STEP apart on every joint."""
    q1, q2 = np.asarray(q1, dtype=float), np.asarray(q2, dtype=float)
    count = max(1, math.ceil(float(np.max(np.abs(q2 - q1))) / JOINT_STEP))
    return [tuple(q1 + (q2 - q1) * i / count) for i in range(1, count + 1)]


def _distance(q1: Sequence[float], q2: Sequence[float]) -> float:
    """Straight-line distance in joint space, radians."""
    return float(np.linalg.norm(np.subtract(q1, q2)))


def plan_arm(mirror: CompasFabMirror, arm_name: str, goal: Sequence[float], abort: threading.Event, *,
             limits: tuple[Sequence[float], Sequence[float]], label: Callable[[str], str] = str,
             time_limit: float = TIME_LIMIT, draw_fn: Callable | None = None) -> PlanResult[ArmPath]:
    """Plan a collision-free joint path for one arm, from where it is in the synced world to `goal`.

    Args:
        mirror: The robot's mirror, already synced.
        arm_name: Which arm, e.g. "ur_arm".
        goal: Six joint values, radians, UR driver order.
        abort: Set from the main thread to end the search early.
        limits: The URDF's lower and upper limits of the six joints (`joints.joint_limits`); the search samples
            within them and within UR_JOINT_LIMITS.
        label: Display text for a scene id, for the reasons.
        time_limit: Give up searching after this long, seconds.
        draw_fn: birrt's hook for each sample and node (`SearchTree.draw_fn`), or None.

    Returns:
        PlanResult[ArmPath]: The path, or why there is none.
    """
    started = time.time()
    names = arm_joint_names(arm_name)
    start = tuple(float(mirror.state.robot_configuration[name]) for name in names)
    goal = tuple(float(value) for value in goal)

    def refused(where: str, pair: tuple[str, str]) -> PlanResult[ArmPath]:
        return PlanResult(None, f"{where} is in collision: {label(pair[0])} with {label(pair[1])}",
                          time.time() - started)

    # * Check the start in full once; the fast `search_check` skips pairs the arm does not move.
    hits = mirror.collisions()
    if hits:
        return refused("start", hits[0])
    check = mirror.search_check(names)
    pair = check(goal)
    if pair is not None:
        return refused("target", pair)

    lower, upper = np.maximum(limits[0], -UR_JOINT_LIMITS), np.minimum(limits[1], UR_JOINT_LIMITS)
    corners = connect(start, goal, _distance, lambda: tuple(np.random.uniform(lower, upper)), _extend,
                      lambda q: check(q) is not None, abort, time_limit, draw_fn=draw_fn)
    if corners.states is None:
        return PlanResult(None, corners.reason, time.time() - started)
    path = ArmPath.at_preview_speed(names, np.array(corners.states, dtype=float))
    return PlanResult(path, seconds=time.time() - started, direct=corners.direct)
