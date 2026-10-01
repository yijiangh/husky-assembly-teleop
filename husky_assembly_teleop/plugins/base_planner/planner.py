"""
The base planner: an RRT-Connect search (`planning.search.connect`) over the base's
floor pose (x, y, yaw), avoiding the other robots and every scene body.

The search runs on the plugin's one worker thread, in a private PyBullet world
(`PlanningWorld`) synced on that thread from the tick's scene snapshot.

! Worker thread only: create nothing here on the main thread but the object,
  and sync, search and close on the worker. Use raw `p` calls with
  `physicsClientId`; `pp.CLIENT` is one global shared by the whole process.
"""

from __future__ import annotations

import math
import threading
import time

import numpy as np
import pybullet as p

from ...planning.search import PlanResult, connect
from ...world.mirrors.pybullet import PyBulletMirror
from ...world.scene import SceneSnapshot, robot_id
from .path import BasePath, steer, steer_cost, steer_points, timed_path

#: Keep at least this far from every other robot and body, metres.
COLLISION_MARGIN = 0.05
#: Samples are drawn from the box around start and goal, grown by this much per side, metres.
SAMPLE_PADDING = 2.0
#: Collision-checking resolution along a move: metres driven, radians turned.
POSITION_STEP = 0.05
YAW_STEP = 0.05


class PlanningWorld:
    """A scene mirror for collision checks, plus a cheap floor-footprint pre-check. Worker thread only."""

    def __init__(self):
        """Start empty; the first `sync` loads the robots and bodies (slow, on the worker)."""
        self.mirror: PyBulletMirror | None = None
        # Display text for our ids, from the last synced snapshot.
        self._label = SceneSnapshot().label
        # ? Cheap first pass for `hit_by`: floor footprints (min x, min y, max x, max y)
        #   by our id, and each robot's reach. Anything out of reach skips the PyBullet query.
        self._footprints: dict[str, tuple[float, float, float, float]] = {}
        self._reach: dict[str, float] = {}

    def sync(self, snapshot: SceneSnapshot) -> None:
        """Make the world match a snapshot, and refresh footprints and reach.

        Args:
            snapshot: The tick's copy of the world, taken on the main thread.
        """
        if self.mirror is None:
            # * Created here, on the worker: a mirror lives on one thread.
            self.mirror = PyBulletMirror()
        self.mirror.sync(snapshot)
        self._label = snapshot.label
        # Footprint and reach depend on arm pose, so refresh them every sync.
        ids = [robot_id(serial) for serial in self.mirror.robots] + self.mirror.obstacle_ids()
        self._footprints = {object_id: self._footprint(self.mirror.body_ids(object_id)) for object_id in ids}
        self._reach = {}
        for serial, body in self.mirror.robots.items():
            footprint = self._footprints[robot_id(serial)]
            position = p.getBasePositionAndOrientation(body, physicsClientId=self.mirror.client_id)[0]
            corners = np.array([(footprint[i], footprint[j]) for i in (0, 2) for j in (1, 3)])
            self._reach[serial] = float(np.max(np.linalg.norm(corners - np.array(position[:2]), axis=1)))

    def _footprint(self, bodies: list[int]) -> tuple[float, float, float, float]:
        """The floor footprint of some PyBullet bodies: their bounding box over every link, in x and y.

        Args:
            bodies: Body ids in the mirror's world.

        Returns:
            tuple[float, float, float, float]: (min x, min y, max x, max y), metres.
        """
        client = self.mirror.client_id
        boxes = [p.getAABB(body, link, physicsClientId=client)
                 for body in bodies for link in range(-1, p.getNumJoints(body, physicsClientId=client))]
        low = np.min([lo for lo, _ in boxes], axis=0)
        high = np.max([hi for _, hi in boxes], axis=0)
        return float(low[0]), float(low[1]), float(high[0]), float(high[1])

    def hit_by(self, serial: str, pose) -> str | None:
        """Place a robot at a floor pose and say what it hits, if anything.

        Args:
            serial: The robot being planned for.
            pose: (x, y, yaw).

        Returns:
            str | None: The label of the robot or body it comes within
                COLLISION_MARGIN of (its id if it has no label); None if it is clear.
        """
        body = self.mirror.robot(serial)
        client = self.mirror.client_id
        z = p.getBasePositionAndOrientation(body, physicsClientId=client)[0][2]
        x, y, yaw = pose
        # ? Moving the robot here is fine: the mirror re-poses robots on every sync.
        p.resetBasePositionAndOrientation(body, (x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
                                          physicsClientId=client)
        own = robot_id(serial)
        reach = self._reach.get(serial, math.inf) + COLLISION_MARGIN
        candidates = []
        for object_id, footprint in self._footprints.items():
            # Distance from the robot's base origin to the footprint rectangle.
            dx = max(footprint[0] - x, 0.0, x - footprint[2])
            dy = max(footprint[1] - y, 0.0, y - footprint[3])
            if object_id != own and math.hypot(dx, dy) <= reach:
                candidates.append(object_id)
        hits = self.mirror.collisions(serial, COLLISION_MARGIN, candidates)
        return self._label(hits[0]) if hits else None

    def set_gui(self, gui: bool, snapshot: SceneSnapshot) -> tuple[bool, str]:
        """Open or close PyBullet's own window on the planning world, for debugging. Worker thread.

        Args:
            gui: Whether the window should be open.
            snapshot: The world now. An opened window shows it, even before the first plan.

        Returns:
            tuple[bool, str]: Whether it is open now, and why not if it was asked for.
        """
        if self.mirror is None:
            self.mirror = PyBulletMirror()
        is_open = self.mirror.set_gui(gui)
        if gui:
            self.sync(snapshot)
        return is_open, self.mirror.window_problem

    def window(self) -> tuple[bool, str]:
        """Whether the window is open, and why not. Worker thread, e.g. after a search.

        Returns:
            tuple[bool, str]: As `set_gui` returns.
        """
        return (False, "") if self.mirror is None else (self.mirror.gui, self.mirror.window_problem)

    def close(self) -> None:
        """Disconnect the world. On the worker, once no search is running."""
        if self.mirror is not None:
            self.mirror.close()
            self.mirror = None


def plan_birrt(world: PlanningWorld, serial: str, start, goal, abort: threading.Event) -> PlanResult[BasePath]:
    """Plan a collision-free base path from start to goal. Runs on the worker thread.

    Args:
        world: The planning world, already synced. Owned by this call until it returns.
        serial: The robot to plan for.
        start: (x, y, yaw) where it is.
        goal: (x, y, yaw) where it should end up.
        abort: Set from the main thread to end the search early.

    Returns:
        PlanResult[BasePath]: The path, or why there is none.
    """
    started = time.time()
    start, goal = tuple(map(float, start)), tuple(map(float, goal))

    # * Report a blocked start or target explicitly; the search would only fail.
    for name, pose in (("start", start), ("target", goal)):
        other = world.hit_by(serial, pose)
        if other is not None:
            return PlanResult(None, f"{name} is in collision with {other}", time.time() - started)

    low = np.minimum(start[:2], goal[:2]) - SAMPLE_PADDING
    high = np.maximum(start[:2], goal[:2]) + SAMPLE_PADDING

    def sample():
        x, y = np.random.uniform(low, high)
        return (float(x), float(y), float(np.random.uniform(-math.pi, math.pi)))

    def extend(q1, q2):
        return steer_points(q1, q2, POSITION_STEP, YAW_STEP)

    def collides(q) -> bool:
        return world.hit_by(serial, q) is not None

    corners = connect(start, goal, steer_cost, sample, extend, collides, abort)
    if corners.states is None:
        return PlanResult(None, corners.reason, time.time() - started)

    # * Rejoin the corners with `steer` and re-check: the joined path is not guaranteed collision-free.
    poses = [start]
    for q1, q2 in zip(corners.states[:-1], corners.states[1:]):
        poses += steer(q1, q2)
        if abort.is_set():
            return PlanResult(None, "cancelled", time.time() - started)
        if any(collides(q) for q in extend(q1, q2)):
            return PlanResult(None, "the smoothed path collides; plan again", time.time() - started)
    return PlanResult(timed_path(poses), seconds=time.time() - started, direct=corners.direct)
