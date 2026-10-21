"""
RRT-Connect over one arm's six joints, avoiding the robot itself, the other robots and every scene body.

! Worker thread only: sync, search and close on the worker.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Mapping, Sequence

import numpy as np
import pybullet as p

from ...world.collision_hit import CollisionHit, Contact, scene_at
from ...shared.planner.timed_path import TimedPath
from ...shared.planner.planner_search import PlanResult
from ...shared.planner.rrt_connect import connect
from ...robot_interface.arm import UR_JOINT_LIMITS, UR_JOINT_NAMES, joint_path_move
from scaffolding_assembly_core.mirrors.compas import subtree
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.scene import Scene
from scaffolding_assembly_core.ids import link_id, robot_id

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


def arm_joint_names(arm_name: str) -> list[str]:
    """The URDF names of one arm's six joints (e.g. arm "left_ur_arm"), in the UR driver's order."""
    return [f"{arm_name}_{name}" for name in UR_JOINT_NAMES]


@dataclass(frozen=True)
class ArmPath(TimedPath):
    """A joint path for one arm, timed as it is sent (`joint_path_move`), so the preview shows the arm's motion.

    `points` are joint values in radians, shape (n, 6), in `joint_names` order.

    Attributes:
        joint_names: The six joints, in the order of each sample.
        velocities: Joint speeds at each sample, rad/s, shape (n, 6).
        rests: How often the arm comes to rest on the way, at corners left sharp.
    """

    joint_names: tuple[str, ...]
    velocities: np.ndarray
    rests: int

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


class ArmPlanningWorld:
    """One compas_fab mirror per robot planned for, and the robot shown in the window. Worker thread only."""

    def __init__(self):
        """Start empty; a robot's mirror is made on its first sync (slow: loads the models)."""
        self._mirrors: dict[str, CompasFabMirror] = {}
        #: The robot synced last; the PyBullet window shows its world.
        self.serial: str | None = None
        #: Whether the PyBullet window is open.
        self.gui = False
        #: The snapshot synced last: display text for our ids, and the scene of a `CollisionHit`.
        self.snapshot = Scene()
        #: Given to each mirror: logs each full rebuild of its cell, and why. Set by the plugin.
        self.log: Callable[[str], None] | None = None

    def sync(self, snapshot: Scene, serial: str) -> CompasFabMirror:
        """Make one robot's planning world match a snapshot.

        Args:
            snapshot: The tick's copy of the world, taken on the main thread.
            serial: The robot to plan for.

        Returns:
            CompasFabMirror: Its mirror, synced.
        """
        mirror = self._mirror(serial)
        if self.gui and self.serial is not None and serial != self.serial:
            # ? One window per process: it follows the robot being planned for.
            self._mirrors[self.serial].set_gui(False)
            mirror.set_gui(True)
        self.serial = serial
        mirror.sync(snapshot)
        self.snapshot = snapshot
        return mirror

    def set_gui(self, gui: bool, snapshot: Scene, serial: str) -> None:
        """Open or close PyBullet's own window on a robot's planning world, for debugging. Worker thread.

        Args:
            gui: Whether the window should be open.
            snapshot: The world now, shown even before the first plan.
            serial: The robot whose world to show.

        Raises:
            RuntimeError: If there is no X display.
            pybullet.error: If another PyBullet window is open in this process.
        """
        if self.serial is not None:
            self._mirrors[self.serial].set_gui(False)
        self.gui = False
        if gui:
            self._mirror(serial).set_gui(True)  # ? before sync, so the cell is built once
            self.gui = True
            self.sync(snapshot, serial)

    def _mirror(self, serial: str) -> CompasFabMirror:
        """A robot's mirror, made on first use."""
        if serial not in self._mirrors:
            self._mirrors[serial] = CompasFabMirror(robot_id(serial), log=self.log)
        return self._mirrors[serial]

    def close(self) -> None:
        """Disconnect every world. On the worker, once no search is running."""
        for mirror in self._mirrors.values():
            mirror.close()
        self._mirrors.clear()


def path_hit(world: ArmPlanningWorld, mirror: CompasFabMirror, arm_name: str,
             waypoints: np.ndarray) -> CollisionHit | None:
    """Why a given joint path for one arm collides: its first waypoint, or straight joint move between two, that does.

    Args:
        world: The planning world, for the snapshot.
        mirror: The robot's mirror, already synced; only this arm's joints change.
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
                return collision_hit(world, mirror, where, dict(zip(names, point)), hit, names)
        previous = waypoint
    return None


def collision_hit(world: ArmPlanningWorld, mirror: CompasFabMirror, where: str, joints: Mapping[str, float],
                  first: tuple[str, str], moved: Sequence[str] | None) -> CollisionHit:
    """Every pair that blocks a configuration, with its contacts and the scene as checked. On the worker.

    Leaves the world at that configuration; every worker job syncs first anyway.

    Args:
        world: The planning world, synced.
        mirror: The robot's mirror, synced.
        where: Which configuration, for the message.
        joints: The joints changed from the synced state, by name.
        first: The pair the check stopped on.
        moved: The joints a search moved: only pairs with a moving side block it, as in `search_check`, and that side
            comes first. None: every pair blocks (a state checked in full, such as the start).

    Returns:
        CollisionHit: The hit, `first` first.
    """
    pairs = mirror.collisions(joints, full_report=True)  # ! poses the whole world at `joints`, for the contacts
    if moved is not None:
        # * The moving side first: what moved into what.
        moving = _moving_ids(mirror, moved)
        pairs = [(a, b) if a in moving else (b, a) for a, b in pairs if a in moving or b in moving]
        first = first if first[0] in moving else (first[1], first[0])
    unique = {frozenset(first): first}
    for pair in pairs:
        unique.setdefault(frozenset(pair), pair)
    pairs = list(unique.values())
    contacts = tuple(_contact(mirror, world.snapshot, pair) for pair in pairs)
    return CollisionHit(scene_at(world.snapshot, mirror.robot_id, joints), mirror.robot_id, where, tuple(pairs),
                        contacts)


# * Core candidate: `_moving_ids` and `_contact` belong next to `CompasFabMirror.collisions`, as `contacts(joints)`.
#   They repeat the cell's naming here (links as "<robot id>/<link>", tools and bodies by our id, other robots as one
#   tool with "<flange>_tool" links) and read compas_fab's client maps, as `SearchCheck` does.

def _moving_ids(mirror: CompasFabMirror, joint_names: Sequence[str]) -> set[str]:
    """Our ids of what moves with these joints: links below them, the tools on those links and the bodies held."""
    model, state, cell = mirror.cell.robot_model, mirror.state, mirror.cell
    links = set().union(*(subtree(model, model.get_joint_by_name(name).child.link) for name in joint_names))
    tools = {name for name, tool in state.tool_states.items()
             if tool.attached_to_group is not None and cell.get_end_effector_link_name(tool.attached_to_group) in links}
    bodies = {name for name, body in state.rigid_body_states.items()
              if body.attached_to_link in links or body.attached_to_tool in tools}
    return {link_id(mirror.robot_id, link) for link in links} | tools | bodies


def _contact(mirror: CompasFabMirror, snapshot: Scene, pair: tuple[str, str]) -> Contact | None:
    """The deepest contact of a pair in the world as it stands, or None if PyBullet finds no point."""
    client_id = mirror.client.client_id
    deepest = None
    for body_a, link_a in _bodies(mirror, pair[0]):
        for body_b, link_b in _bodies(mirror, pair[1]):
            links = {**({"linkIndexA": link_a} if link_a is not None else {}),
                     **({"linkIndexB": link_b} if link_b is not None else {})}
            for point in p.getClosestPoints(body_a, body_b, 0.0, physicsClientId=client_id, **links):
                # ? Point fields: 1, 2 bodies; 3, 4 links; 5, 6 positions on A and B; 8 signed distance.
                if deepest is None or point[8] < deepest[8]:
                    deepest = point
    if deepest is None:
        return None
    narrowed = (_touched(mirror, snapshot, pair[0], deepest[1], deepest[3]),
                _touched(mirror, snapshot, pair[1], deepest[2], deepest[4]))
    return Contact(narrowed, tuple(deepest[5]), tuple(deepest[6]), float(deepest[8]))


def _bodies(mirror: CompasFabMirror, our_id: str) -> list[tuple[int, int | None]]:
    """The PyBullet (body, link index or None for all) behind one of our ids in a collision pair."""
    client, prefix = mirror.client, f"{mirror.robot_id}/"
    if our_id.startswith(prefix):
        return [(client.robot_puid, client.robot_link_puids[our_id[len(prefix):]])]
    if our_id in client.tools_puids:
        return [(client.tools_puids[our_id], None)]
    return [(body, None) for body in client.rigid_bodies_puids.get(our_id, ())]


def _touched(mirror: CompasFabMirror, snapshot: Scene, our_id: str, body: int, link: int) -> str:
    """Narrow a whole other robot to the link (or mounted tool) a contact touched; any other id stays."""
    robot = snapshot.robots.get(our_id)
    if robot is None:
        return our_id
    client_id = mirror.client.client_id
    name = (p.getBodyInfo(body, physicsClientId=client_id)[0] if link < 0
            else p.getJointInfo(body, link, physicsClientId=client_id)[12]).decode()
    flange = name.removesuffix("_tool")
    if flange != name and flange in robot.model.tools:
        return robot.model.tools[flange].id
    return link_id(our_id, name)


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


def plan_arm(world: ArmPlanningWorld, mirror: CompasFabMirror, arm_name: str, goal: Sequence[float],
             abort: threading.Event) -> PlanResult[ArmPath]:
    """Plan a collision-free joint path for one arm within its URDF joint limits, from where it is to `goal`.

    Args:
        world: The planning world, for the snapshot.
        mirror: The robot's mirror, already synced.
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
    hits = mirror.collisions()
    if hits:
        return refused(collision_hit(world, mirror, "start", {}, hits[0], None))
    check = mirror.search_check(names)
    pair = check(goal)
    if pair is not None:
        return refused(collision_hit(world, mirror, "target", dict(zip(names, goal)), pair, names))

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
