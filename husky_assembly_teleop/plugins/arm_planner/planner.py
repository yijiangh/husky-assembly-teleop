"""
RRT-Connect over one arm's six joints, avoiding the robot itself, the other robots and every scene body.

Plans on one robot's mirror from a `MirrorPool`; a collision comes back as a `CollisionHit` (`collision_hit`).

! Worker thread only: sync, search and close on the worker.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Sequence

import numpy as np

from ...world.collision_hit import CollisionHit
from ...shared.assembly_planning.collision import collision_hit, collisions
from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.planner.timed_path import TimedPath
from ...shared.planner.planner_search import PlanResult
from ...shared.planner.rrt_connect import connect
from ...robot_interface.arm import UR_JOINT_LIMITS, UR_JOINT_NAMES, joint_path_move
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.scene import Scene

#: Collision-checking resolution along a move, radians per joint.
JOINT_STEP = 0.05
#: Searches the shortest path is kept from, and seconds after which no new one starts once a path is found.
ATTEMPTS = 3
IMPROVE_TIME = 3.0
#: The search samples each joint at most this far, radians, beyond the range between start and goal, and never
#: past the joint's limits (`joint_limits`).
SAMPLE_SLACK = np.pi
#: Longest stretch of each side of a corner, radians in joint space, that rounding it replaces.
BLEND_LENGTH = 0.3
#: Times a rounding that collides is halved before the arm rests at that corner instead.
BLEND_TRIES = 4


@dataclass(frozen=True)
class ArmPath(TimedPath):
    """A joint path for one arm, timed as it is sent (`joint_path_move`), so the preview shows the arm's motion.

    `points` are joint values in radians, shape (n, 6), in `joint_names` order.

    Attributes:
        joint_names: The six joints, in the order of each sample.
        velocities: Joint speeds at each sample, rad/s, shape (n, 6).
        rests: How often the arm comes to rest on the way, at corners left sharp.
        slowdown: How many times longer than its fastest timing it takes (`slowed`).
    """

    joint_names: tuple[str, ...]
    velocities: np.ndarray
    rests: int
    slowdown: float = 1.0

    @property
    def fastest(self) -> float:
        """float: Seconds it takes at its fastest timing, the one it was planned with."""
        return self.duration / self.slowdown

    def slowed(self, slowdown: float) -> ArmPath:
        """The same path taking `slowdown` times its fastest timing: times stretched, speeds divided by it.

        Accelerations drop by its square, so every joint keeps within the limits the fastest timing kept.
        """
        ratio = slowdown / self.slowdown
        return replace(self, times=self.times * ratio, velocities=self.velocities / ratio, slowdown=slowdown)

    @classmethod
    def along(cls, joint_names: Sequence[str], points: np.ndarray, stops: Sequence[int] | None = None) -> ArmPath:
        """Time a path on `joint_path_move`'s profile.

        Args:
            joint_names: The six joints.
            points: (n, 6) joint values, radians; the first is the start.
            stops: Indices of the points to rest at, as `joint_path_move` takes them; None: every point.

        Returns:
            ArmPath: The timed path.

        Raises:
            ValueError: If the points never move.
        """
        positions, velocities, duration = joint_path_move(points, stops)
        rests = len(points) - 2 if stops is None else len(set(stops) - {0, len(points) - 1})
        return cls(times=np.linspace(0.0, duration, len(positions)), points=positions,
                   joint_names=tuple(joint_names), velocities=velocities, rests=rests)


def path_hit(mirror: CompasFabMirror, scene: Scene, arm_name: str, waypoints: np.ndarray) -> CollisionHit | None:
    """Why a given joint path for one arm collides: its first waypoint, or straight joint move between two, that does.

    Args:
        mirror: The robot's mirror, synced to `scene`; only this arm's joints change.
        scene: The scene the mirror was synced to.
        arm_name: Which arm, e.g. "ur_arm".
        waypoints: (n, 6) joint values, radians, UR driver order; checked as given, moves at JOINT_STEP.

    Returns:
        CollisionHit | None: The first configuration that collides, or None if the whole path is clear.
    """
    names = arm_joint_names(arm_name)
    check = mirror.search_check(names)
    previous = None
    for index, waypoint in enumerate(np.asarray(waypoints, dtype=float)):
        for point in [tuple(waypoint)] if previous is None else _extend(previous, waypoint):
            hit = check(point)
            if hit is not None:
                where = f"waypoint {index + 1}" if np.allclose(point, waypoint) else f"the move to waypoint {index + 1}"
                return collision_hit(mirror, scene, where, dict(zip(names, point)), names, hit)
        previous = waypoint
    return None


def _extend(q1: Sequence[float], q2: Sequence[float]) -> list[tuple[float, ...]]:
    """Points from q1 (excluded) to q2 (included), at most JOINT_STEP apart on every joint."""
    q1, q2 = np.asarray(q1, dtype=float), np.asarray(q2, dtype=float)
    count = max(1, math.ceil(float(np.max(np.abs(q2 - q1))) / JOINT_STEP))
    # * One array operation: the searches call this for every segment, many thousand times per plan.
    return list(map(tuple, (q1 + np.outer(np.arange(1, count + 1) / count, q2 - q1)).tolist()))


def joint_limits(mirror: CompasFabMirror, joint_names: Sequence[str]) -> tuple[np.ndarray, np.ndarray]:
    """Each joint's limits from the robot's URDF, within the UR's own (`UR_JOINT_LIMITS`), radians.

    Args:
        mirror: The robot's mirror.
        joint_names: The joints, in UR driver order.

    Returns:
        tuple[np.ndarray, np.ndarray]: Lower and upper limits.
    """
    model = mirror.cell.robot_model
    lower, upper = -np.array(UR_JOINT_LIMITS, dtype=float), np.array(UR_JOINT_LIMITS, dtype=float)
    for index, name in enumerate(joint_names):
        limit = model.get_joint_by_name(name).limit
        if limit is not None and limit.lower < limit.upper:
            lower[index], upper[index] = max(lower[index], limit.lower), min(upper[index], limit.upper)
    return lower, upper


def _outside(q: Sequence[float], lower: np.ndarray, upper: np.ndarray) -> str:
    """The joints of `q` past their limits, for a message; empty if none is."""
    return "; ".join(f"{UR_JOINT_NAMES[i]} at {math.degrees(value):.1f} deg, outside "
                     f"{math.degrees(lower[i]):.0f} to {math.degrees(upper[i]):.0f} deg"
                     for i, value in enumerate(q) if not lower[i] - 1e-6 <= value <= upper[i] + 1e-6)


def _distance(q1: Sequence[float], q2: Sequence[float]) -> float:
    """Straight-line distance in joint space, radians."""
    return math.dist(q1, q2)


def round_corners(corners: Sequence[Sequence[float]],
                  collides: Callable[[tuple[float, ...]], bool]) -> tuple[np.ndarray, list[int]]:
    """Round each corner of a clear path with a curve, where the curve is clear too, so the arm need not stop there.

    The curve is a parabola from BLEND_LENGTH before the corner to as far after it (at most half of either side),
    halved while it collides; a corner still blocked after BLEND_TRIES stays sharp.

    Args:
        corners: Start, corners, goal: (n, 6) joint values whose straight moves are clear.
        collides: Whether a configuration is in collision.

    Returns:
        tuple[np.ndarray, list[int]]: The points, with each rounded corner sampled at most JOINT_STEP / 2 apart, and
            the indices of the corners left sharp, for `joint_path_move` to rest at.
    """
    corners = np.asarray(corners, dtype=float)
    corners = corners[np.concatenate([[True], np.any(np.diff(corners, axis=0) != 0.0, axis=1)])]
    points, sharp = [corners[0]], []
    for before, corner, after in zip(corners[:-2], corners[1:-1], corners[2:]):
        incoming, outgoing = corner - before, after - corner
        length = min(BLEND_LENGTH, np.linalg.norm(incoming) / 2, np.linalg.norm(outgoing) / 2)
        for _ in range(BLEND_TRIES):
            enter = corner - length * incoming / np.linalg.norm(incoming)
            leave = corner + length * outgoing / np.linalg.norm(outgoing)
            u = np.linspace(0.0, 1.0, max(3, int(np.ceil(4 * length / JOINT_STEP)) + 1))[:, None]
            curve = (1 - u) ** 2 * enter + 2 * u * (1 - u) * corner + u ** 2 * leave
            if not any(collides(tuple(q)) for q in curve):
                points.extend(curve)
                break
            length /= 2
        else:
            sharp.append(len(points))
            points.append(corner)
    points.append(corners[-1])
    return np.array(points), sharp


def plan_arm(mirror: CompasFabMirror, scene: Scene, arm_name: str, goal: Sequence[float],
             abort: threading.Event) -> PlanResult[ArmPath]:
    """Plan a collision-free joint path for one arm within its URDF joint limits, from where it is to `goal`.

    Args:
        mirror: The robot's mirror, synced to `scene`.
        scene: The scene the mirror was synced to, for a hit's scene.
        arm_name: Which arm, e.g. "ur_arm".
        goal: Six joint values, radians, UR driver order.
        abort: Set from the main thread to end the search early.

    Returns:
        PlanResult[ArmPath]: The path, or why there is none.
    """
    started = time.time()
    names = arm_joint_names(arm_name)
    start = tuple(float(mirror.state.robot_configuration[name]) for name in names)
    goal = tuple(float(value) for value in goal)

    def refused(hit: CollisionHit) -> PlanResult[ArmPath]:
        return PlanResult(None, hit.message, time.time() - started, hit=hit)

    lower, upper = joint_limits(mirror, names)
    if outside := _outside(start, lower, upper):
        return PlanResult(None, f"the arm is past its joint limits: {outside}; move it back inside first",
                          time.time() - started)
    if outside := _outside(goal, lower, upper):
        return PlanResult(None, f"the target is past the joint limits: {outside}", time.time() - started)
    # * Check the start in full once; the fast `search_check` skips pairs the arm does not move.
    hits = collisions(mirror)
    if hits:
        return refused(collision_hit(mirror, scene, "start", first=hits[0]))
    check = mirror.search_check(names)
    pair = check(goal)
    if pair is not None:
        return refused(collision_hit(mirror, scene, "target", dict(zip(names, goal)), names, pair))

    if np.allclose(start, goal, rtol=0.0, atol=1e-6):
        return PlanResult(None, "the arm is already at the target", time.time() - started)

    # * Sample near start and goal: far samples wind the wrists through needless turns. Within the limits, every
    #   straight move and rounded corner between samples stays within them too.
    low = np.maximum(np.minimum(start, goal) - SAMPLE_SLACK, lower)
    high = np.minimum(np.maximum(start, goal) + SAMPLE_SLACK, upper)

    def collides(q: tuple[float, ...]) -> bool:
        return check(q) is not None

    corners = connect(start, goal, _distance, lambda: tuple(np.random.uniform(low, high)), _extend, collides, abort,
                      attempts=ATTEMPTS, improve_time=IMPROVE_TIME)
    if corners.states is None:
        return PlanResult(None, corners.reason, time.time() - started)
    path = ArmPath.along(names, *round_corners(corners.states, collides))
    return PlanResult(path, seconds=time.time() - started, direct=corners.direct)
