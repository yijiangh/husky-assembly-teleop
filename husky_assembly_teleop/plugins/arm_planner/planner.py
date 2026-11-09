"""
RRT-Connect over one arm's joints, or several arms' together, avoiding the robot, the other robots and the scene.

Plans on one robot's mirror from a `MirrorPool`; a collision comes back as a `CollisionHit` (`collision_hit`). A path
keeps PLAN_DISTANCE from everything, more than a commit's clearance; only a pair its start or target is already closer
for keeps as much as it has there (`endpoint_margins`).

! Worker thread only: sync, search and close on the worker.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, replace
from typing import Callable, Sequence

import numpy as np

from ...world.clearance import PLAN_DISTANCE, SELF_DISTANCE
from ...world.collision_hit import CollisionHit
from ...shared.assembly_planning.collision import collision_hit, collisions
from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.planner.timed_path import TimedPath
from ...shared.planner.planner_search import PlanResult
from ...shared.planner.rrt_connect import connect
from ...robot_interface.arm import UR_JOINT_LIMITS, joint_path_move
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.scene import Scene

#: Collision-checking resolution along a move, radians per joint.
JOINT_STEP = 0.05
#: How finely a path found is checked again before it is kept, radians per joint: a pass closer than its margin can
#: fall between two of the search's checks (the tool moves centimetres in one JOINT_STEP).
VERIFY_STEP = 0.005
#: Give up searching after this long, seconds: longer than the shared 20 s (`planner_search.TIME_LIMIT`), as a free
#: move of both arms between built bars can take most of it.
TIME_LIMIT = 60.0
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
#: The most a rounding curve turns between two of its points, radians: its points follow the curve closely enough
#: for a second timing (Exec's `retime`) to glide along it.
CURVE_TURN = math.radians(1.0)
#: How finely `endpoint_margins` rounds down the margin of a pair closer than usual at the start or target, metres.
MARGIN_RESOLUTION = 0.0005


@dataclass(frozen=True)
class ArmPath(TimedPath):
    """A joint path for one arm, timed as it is sent (`joint_path_move`), so the preview shows the arm's motion.

    `points` are joint values in radians, shape (n, 6), in `joint_names` order: the samples of that timing, 0.2 s
    apart. ! To time the path again (`retime`, as Exec does), pass `waypoints`, never `points`: on a curve two samples
    meet at an angle the curve itself never has, and a second timing stops there.

    Attributes:
        joint_names: The six joints, in the order of each sample.
        velocities: Joint speeds at each sample, rad/s, shape (n, 6).
        rests: How often the arm comes to rest on the way, at corners left sharp.
        slowdown: How many times longer than its fastest timing it takes (`slowed`).
        margin: How far it keeps from everything, metres, as checked when planned: PLAN_DISTANCE, or the smallest
            margin in `close`; 0 is contact only.
        close: The pairs it keeps less from, as (part, part, metres), closest first: those its start or target is
            already that close to (`endpoint_margins`). Every other pair keeps PLAN_DISTANCE.
        shape: Where it goes, untimed: the corners with their rounding curves, densely sampled (`round_corners`), in
            `joint_names` order; None for a path made from timed points only. `waypoints` gives it.
    """

    joint_names: tuple[str, ...]
    velocities: np.ndarray
    rests: int
    slowdown: float = 1.0
    margin: float = 0.0
    close: tuple[tuple[str, str, float], ...] = ()
    shape: np.ndarray | None = None

    @property
    def waypoints(self) -> np.ndarray:
        """np.ndarray: Where it goes, untimed (`shape`): the rows to time again; the points when it has no shape."""
        return self.points if self.shape is None else self.shape

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
            joint_names: The joints of each point: one arm's six, or several arms' after each other.
            points: (n, len(joint_names)) joint values, radians; the first is the start.
            stops: Indices of the points to rest at, as `joint_path_move` takes them; None: every point.

        Returns:
            ArmPath: The timed path.

        Raises:
            ValueError: If the points never move.
        """
        positions, velocities, duration = joint_path_move(points, stops)
        rests = len(points) - 2 if stops is None else len(set(stops) - {0, len(points) - 1})
        return cls(times=np.linspace(0.0, duration, len(positions)), points=positions,
                   joint_names=tuple(joint_names), velocities=velocities, rests=rests,
                   shape=np.asarray(points, dtype=float))


def path_hit(mirror: CompasFabMirror, scene: Scene, arm_name: str, waypoints: np.ndarray,
             margin: float = 0.0) -> CollisionHit | None:
    """Why a given joint path for one arm collides: its first waypoint, or straight joint move between two, that does.

    Args:
        mirror: The robot's mirror, synced to `scene`; only this arm's joints change.
        scene: The scene the mirror was synced to.
        arm_name: Which arm, e.g. "ur_arm".
        waypoints: (n, 6) joint values, radians, UR driver order; checked as given, moves at JOINT_STEP.
        margin: Closer than this to anything counts as a hit, metres; against the robot itself at most SELF_DISTANCE,
            as a clearance checks it.

    Returns:
        CollisionHit | None: The first configuration that collides, or None if the whole path is clear.
    """
    names = arm_joint_names(arm_name)
    check = mirror.search_check(names, margin, min(margin, SELF_DISTANCE))
    previous = None
    for index, waypoint in enumerate(np.asarray(waypoints, dtype=float)):
        for point in [tuple(waypoint)] if previous is None else _extend(previous, waypoint):
            hit = check(point)
            if hit is not None:
                where = f"waypoint {index + 1}" if np.allclose(point, waypoint) else f"the move to waypoint {index + 1}"
                return collision_hit(mirror, scene, where, dict(zip(names, point)), names, hit)
        previous = waypoint
    return None


def _extend(q1: Sequence[float], q2: Sequence[float], step: float = JOINT_STEP) -> list[tuple[float, ...]]:
    """Points from q1 (excluded) to q2 (included), at most `step` apart on every joint."""
    q1, q2 = np.asarray(q1, dtype=float), np.asarray(q2, dtype=float)
    count = max(1, math.ceil(float(np.max(np.abs(q2 - q1))) / step))
    # * One array operation: the searches call this for every segment, many thousand times per plan.
    return list(map(tuple, (q1 + np.outer(np.arange(1, count + 1) / count, q2 - q1)).tolist()))


def joint_limits(mirror: CompasFabMirror, joint_names: Sequence[str]) -> tuple[np.ndarray, np.ndarray]:
    """Each joint's limits from the robot's URDF, within the UR's own (`UR_JOINT_LIMITS`), radians.

    Args:
        mirror: The robot's mirror.
        joint_names: The joints: one arm's six in UR driver order, or several arms' after each other.

    Returns:
        tuple[np.ndarray, np.ndarray]: Lower and upper limits.
    """
    model = mirror.cell.robot_model
    arms = max(1, len(joint_names) // len(UR_JOINT_LIMITS))
    upper = np.tile(np.array(UR_JOINT_LIMITS, dtype=float), arms)[:len(joint_names)]
    lower = -upper
    for index, name in enumerate(joint_names):
        limit = model.get_joint_by_name(name).limit
        if limit is not None and limit.lower < limit.upper:
            lower[index], upper[index] = max(lower[index], limit.lower), min(upper[index], limit.upper)
    return lower, upper


def _outside(q: Sequence[float], lower: np.ndarray, upper: np.ndarray, names: Sequence[str]) -> str:
    """The joints of `q` past their limits, for a message; empty if none is."""
    return "; ".join(f"{names[i]} at {math.degrees(value):.1f} deg, outside "
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
        turn = math.acos(float(np.clip(incoming @ outgoing / np.linalg.norm(incoming) / np.linalg.norm(outgoing),
                                       -1.0, 1.0)))
        for _ in range(BLEND_TRIES):
            enter = corner - length * incoming / np.linalg.norm(incoming)
            leave = corner + length * outgoing / np.linalg.norm(outgoing)
            # * Densely enough on length (the collision checks) and on turn: a short curve sampled coarsely meets at
            #   sharp angles again, where a second timing (Exec's) would stop.
            u = np.linspace(0.0, 1.0, max(3, int(np.ceil(4 * length / JOINT_STEP)) + 1,
                                          int(np.ceil(turn / CURVE_TURN)) + 1))[:, None]
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
    """Plan a collision-free joint path for one arm within its URDF joint limits, from where it is to `goal`."""
    return plan_arms(mirror, scene, (arm_name,), goal, abort)


def plan_arms(mirror: CompasFabMirror, scene: Scene, arm_names: Sequence[str], goal: Sequence[float],
              abort: threading.Event) -> PlanResult[ArmPath]:
    """Plan a collision-free joint path for one or several arms together, from where they are to `goal`.

    One search over all their joints (twelve for Cindy's two arms), so each arm keeps clear of the other on the way.

    The path keeps PLAN_DISTANCE from everything, the robot itself included; only a pair its start or target is
    already closer for keeps as much as it has there (`endpoint_margins`, `ArmPath.close`).
    A path found is checked again every VERIFY_STEP (`fine_hit`) and searched for again until one holds those
    margins, within TIME_LIMIT.

    Args:
        mirror: The robot's mirror, synced to `scene`.
        scene: The scene the mirror was synced to, for a hit's scene.
        arm_names: Which arms, e.g. ("left_ur_arm", "right_ur_arm").
        goal: Six joint values per arm, radians, UR driver order, the arms after each other.
        abort: Set from the main thread to end the search early.

    Returns:
        PlanResult[ArmPath]: The path, or why there is none.
    """
    started = time.time()
    names = [name for arm in arm_names for name in arm_joint_names(arm)]
    start = tuple(float(mirror.state.robot_configuration[name]) for name in names)
    goal = tuple(float(value) for value in goal)

    def refused(hit: CollisionHit) -> PlanResult[ArmPath]:
        return PlanResult(None, hit.message, time.time() - started, hit=hit)

    lower, upper = joint_limits(mirror, names)
    if outside := _outside(start, lower, upper, names):
        return PlanResult(None, f"the arm is past its joint limits: {outside}; move it back inside first",
                          time.time() - started)
    if outside := _outside(goal, lower, upper, names):
        return PlanResult(None, f"the target is past the joint limits: {outside}", time.time() - started)
    # * Check the start in full once; the fast `search_check` skips pairs the arm does not move.
    hits = collisions(mirror)
    if hits:
        return refused(collision_hit(mirror, scene, "start", first=hits[0]))
    close = endpoint_margins(mirror, names, start, goal)
    check = mirror.search_check(names, PLAN_DISTANCE,
                                pair_margins={frozenset(pair): kept for pair, kept in close.items()})
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

    nearest = tuple(sorted(((*pair, kept) for pair, kept in close.items()), key=lambda item: item[2]))
    # * A path is kept only once checked finely (VERIFY_STEP); else the search runs again in the time left.
    deadline, straight, broken = started + TIME_LIMIT, True, None
    while True:
        corners = connect(start, goal, _distance, lambda: tuple(np.random.uniform(low, high)), _extend, collides,
                          abort, time_limit=max(0.0, deadline - time.time()), attempts=ATTEMPTS,
                          improve_time=IMPROVE_TIME, straight=straight)
        if corners.states is None:
            reason = corners.reason if broken is None or corners.reason == "cancelled" else (
                f"every path found came closer than its margin between {part_name(broken[0])} and "
                f"{part_name(broken[1])} between the search's checks")
            return PlanResult(None, reason, time.time() - started)
        path = replace(ArmPath.along(names, *round_corners(corners.states, collides)),
                       margin=min([PLAN_DISTANCE, *close.values()]), close=nearest)
        # * Both ways it is driven: through its waypoints (timed again by Exec) and through its 0.2 s samples (sent
        #   as they are from the panel), which cut across its curves.
        broken = fine_hit(path.waypoints, check) or fine_hit(path.points, check)
        if broken is None:
            return PlanResult(path, seconds=time.time() - started, direct=corners.direct)
        straight = straight and not corners.direct


def fine_hit(points: Sequence[Sequence[float]], check: Callable[[tuple[float, ...]], tuple[str, str] | None],
             step: float = VERIFY_STEP) -> tuple[str, str] | None:
    """The first pair `check` reports along a path, its moves checked every `step` on every joint; None if clear."""
    previous = None
    for point in points:
        for q in [tuple(point)] if previous is None else _extend(previous, point, step):
            pair = check(q)
            if pair is not None:
                return pair
        previous = point
    return None


def endpoint_margins(mirror: CompasFabMirror, names: Sequence[str], start: Sequence[float],
                     goal: Sequence[float]) -> dict[tuple[str, str], float]:
    """The pairs closer than PLAN_DISTANCE at a path's start or target, each with the margin it can keep.

    A closer pair keeps a little less than its distance at the nearer end, rounded down to MARGIN_RESOLUTION; 0 when
    it touches (contact only). The robot against itself counts as any other pair: the clearance stops at half the
    smallest margin kept, below every margin planned.
    """
    check = mirror.search_check(names, PLAN_DISTANCE)
    kept: dict[tuple[str, str], float] = {}
    for values in (start, goal):
        for pair, distance in check.distances(values).items():
            # * Strictly below the distance, so that end itself stays clear (the check counts a point at its margin).
            margin = max(0.0, math.floor((distance - 1e-6) / MARGIN_RESOLUTION) * MARGIN_RESOLUTION)
            kept[pair] = min(kept.get(pair, margin), margin)
    return kept


def margin_note(margin: float, close: Sequence[tuple[str, str, float]] = ()) -> str:
    """What a path's margin means for its commit: "keeps 1 cm", or which pair its start or target is closer for.

    Args:
        margin: `ArmPath.margin`.
        close: `ArmPath.close`, closest first.
    """
    if margin >= PLAN_DISTANCE:
        return f"keeps {PLAN_DISTANCE * 100:g} cm"
    if not close:
        return (f"keeps only {round(margin * 100, 2):g} cm: the start or target is that close" if margin > 0
                else "only free of contact: the start or target touches")
    first, second, kept = close[0]
    more = f" (and {len(close) - 1} more pair{'s' if len(close) > 2 else ''})" if len(close) > 1 else ""
    if kept > 0:
        return (f"keeps only {round(kept * 100, 2):g} cm between {part_name(first)} and {part_name(second)}{more}: "
                f"the start or target is that close; {PLAN_DISTANCE * 100:g} cm from everything else")
    return (f"only free of contact between {part_name(first)} and {part_name(second)}{more}: the start or target "
            f"touches; {PLAN_DISTANCE * 100:g} cm from everything else")


def part_name(part_id: str) -> str:
    """A part's id in plain words, e.g. "right_ur_arm_wrist_1_link", "the right_ur_arm tool", "joints/J25-26_mocap"."""
    kind, _, rest = part_id.partition("/")
    if kind == "tools":
        return f"the {rest.split('/', 1)[-1]} tool"
    if kind == "robots":
        return rest.split("/", 1)[-1]
    return rest if kind == "process" else part_id
