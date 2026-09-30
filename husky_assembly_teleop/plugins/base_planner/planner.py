"""
The base planner: an RRT-Connect search (`birrt`) over the base's floor pose
(x, y, yaw), avoiding the other robots and the static boxes.

The search runs on a worker thread, in a private PyBullet world (`PlanningWorld`)
that the main thread poses from the shared scene just before each search.

! In the worker, use only raw `p` calls with an explicit `physicsClientId`:
  `pp` functions go through `pp.CLIENT`, which belongs to the main thread.
! One search at a time per world; do not snapshot while one runs.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

import numpy as np
import pybullet as p
from pybullet_planning.motion_planners.rrt_connect import birrt
from pybullet_planning.motion_planners.utils import waypoints_from_path

from ...config import RobotConfig
from ..obstacles import Box
from .path import BasePath, steer, steer_cost, steer_points, timed_path

#: Keep at least this far from every other robot, metres.
COLLISION_MARGIN = 0.05
#: Samples are drawn from the box around start and goal, grown by this much per side, metres.
SAMPLE_PADDING = 2.0
#: Give up after this long, seconds.
TIME_LIMIT = 10.0
#: RRT-Connect iterations per attempt, extra attempts, and smoothing iterations.
MAX_ITERATIONS = 2000
RESTARTS = 2
SMOOTHING = 100
#: Collision-checking resolution along a move: metres driven, radians turned.
POSITION_STEP = 0.05
YAW_STEP = 0.05


class PlanningAborted(Exception):
    """Raised inside a search to end it early."""


@dataclass
class PlanResult:
    """What a search came back with.

    Attributes:
        path: The timed path, or None if none was found.
        reason: Why not, when `path` is None; empty otherwise.
        seconds: How long the search took.
        direct: Whether the straight move was already free.
    """

    path: BasePath | None
    reason: str = ""
    seconds: float = 0.0
    direct: bool = False


class PlanningWorld:
    """A private DIRECT PyBullet client holding a copy of every robot, for collision checks."""

    def __init__(self, robots: tuple[RobotConfig, ...]):
        """Connect, and load every robot's URDF. Slow; call from `setup`.

        Args:
            robots: The configured robots.
        """
        # * Raw p calls, not pp's: pp.connect would repoint pp.CLIENT, which the shared scene uses.
        self.client_id = p.connect(p.DIRECT)
        #: Robot serial -> body id in this client.
        self.robots: dict[str, int] = {
            config.serial: p.loadURDF(str(config.urdf_file), useFixedBase=False, physicsClientId=self.client_id)
            for config in robots}
        #: Box body id in this client -> the box's name, for collision messages.
        self.boxes: dict[int, str] = {}
        # ? Cheap first pass for `hit_by`: obstacle footprints (min x, min y, max x, max y)
        #   and each robot's reach. Obstacles out of reach skip the PyBullet query.
        self._footprints: dict[int, tuple[float, float, float, float]] = {}
        self._reach: dict[str, float] = {}

    def set_boxes(self, boxes: tuple[Box, ...]) -> None:
        """Replace the static boxes. Call from `setup`.

        Args:
            boxes: The boxes, from the obstacles plugin.
        """
        for body in self.boxes:
            p.removeBody(body, physicsClientId=self.client_id)
            self._footprints.pop(body, None)
        self.boxes = {}
        for box in boxes:
            shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[side / 2 for side in box.size],
                                           physicsClientId=self.client_id)
            body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=shape, basePosition=box.center,
                                     baseOrientation=box.orientation, physicsClientId=self.client_id)
            self.boxes[body] = box.name
            self._footprints[body] = self._footprint(body)

    def snapshot(self, scene_client: int, scene_robots: dict[str, int]) -> None:
        """Pose every robot here as it stands in the shared scene. Main thread only.

        Args:
            scene_client: The shared scene's client id.
            scene_robots: Serial -> body id in the shared scene.
        """
        for serial, body in self.robots.items():
            source = scene_robots[serial]
            position, orientation = p.getBasePositionAndOrientation(source, physicsClientId=scene_client)
            p.resetBasePositionAndOrientation(body, position, orientation, physicsClientId=self.client_id)
            for joint in range(p.getNumJoints(source, physicsClientId=scene_client)):
                value = p.getJointState(source, joint, physicsClientId=scene_client)[0]
                p.resetJointState(body, joint, value, physicsClientId=self.client_id)
            # Footprint and reach depend on arm pose, so refresh them here.
            self._footprints[body] = footprint = self._footprint(body)
            corners = np.array([(footprint[i], footprint[j]) for i in (0, 2) for j in (1, 3)])
            self._reach[serial] = float(np.max(np.linalg.norm(corners - np.array(position[:2]), axis=1)))

    def _footprint(self, body: int) -> tuple[float, float, float, float]:
        """A body's floor footprint: its bounding box over every link, in x and y.

        Args:
            body: Body id in this client.

        Returns:
            tuple[float, float, float, float]: (min x, min y, max x, max y), metres.
        """
        boxes = [p.getAABB(body, link, physicsClientId=self.client_id)
                 for link in range(-1, p.getNumJoints(body, physicsClientId=self.client_id))]
        low = np.min([lo for lo, _ in boxes], axis=0)
        high = np.max([hi for _, hi in boxes], axis=0)
        return float(low[0]), float(low[1]), float(high[0]), float(high[1])

    def hit_by(self, serial: str, pose) -> str | None:
        """Place a robot at a floor pose and say which robot or box it hits, if any.

        Args:
            serial: The robot being planned for.
            pose: (x, y, yaw).

        Returns:
            str | None: The serial of the robot, or the name of the box, it comes
                within COLLISION_MARGIN of; None if it is clear.
        """
        body = self.robots[serial]
        z = p.getBasePositionAndOrientation(body, physicsClientId=self.client_id)[0][2]
        x, y, yaw = pose
        p.resetBasePositionAndOrientation(body, (x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
                                          physicsClientId=self.client_id)
        # No reach before the first snapshot: check everything.
        reach = self._reach.get(serial, math.inf) + COLLISION_MARGIN
        obstacles = [(other, obstacle) for other, obstacle in self.robots.items() if other != serial]
        obstacles += [(name, obstacle) for obstacle, name in self.boxes.items()]
        for name, obstacle in obstacles:
            footprint = self._footprints.get(obstacle)
            if footprint is not None:
                # Distance from the robot's base origin to the footprint rectangle.
                dx = max(footprint[0] - x, 0.0, x - footprint[2])
                dy = max(footprint[1] - y, 0.0, y - footprint[3])
                if math.hypot(dx, dy) > reach:
                    continue
            if p.getClosestPoints(body, obstacle, COLLISION_MARGIN, physicsClientId=self.client_id):
                return name
        return None

    def close(self) -> None:
        """Disconnect the client. Only once no search is running."""
        p.disconnect(physicsClientId=self.client_id)


def plan_birrt(world: PlanningWorld, serial: str, start, goal, abort: threading.Event) -> PlanResult:
    """Plan a collision-free base path from start to goal. Runs on the worker thread.

    Args:
        world: The planning world, already snapshotted. Owned by this call until it returns.
        serial: The robot to plan for.
        start: (x, y, yaw) where it is.
        goal: (x, y, yaw) where it should end up.
        abort: Set from the main thread to end the search early.

    Returns:
        PlanResult: The path, or why there is none.
    """
    started = time.time()
    start, goal = tuple(map(float, start)), tuple(map(float, goal))

    # * Report a blocked start or target explicitly; birrt would only return None.
    for name, pose in (("start", start), ("target", goal)):
        other = world.hit_by(serial, pose)
        if other is not None:
            return PlanResult(None, f"{name} is in collision with {other}", time.time() - started)

    low = np.minimum(start[:2], goal[:2]) - SAMPLE_PADDING
    high = np.maximum(start[:2], goal[:2]) + SAMPLE_PADDING

    def sample_fn():
        x, y = np.random.uniform(low, high)
        return (float(x), float(y), float(np.random.uniform(-math.pi, math.pi)))

    def extend_fn(q1, q2):
        return steer_points(q1, q2, POSITION_STEP, YAW_STEP)

    def collision_fn(q, **_kwargs):
        # birrt may pass a `diagnosis` flag; ignored.
        if abort.is_set():
            raise PlanningAborted()
        return world.hit_by(serial, q) is not None

    direct = all(not collision_fn(q) for q in extend_fn(start, goal))
    waypoints = birrt(start, goal, steer_cost, sample_fn, extend_fn, collision_fn,
                      max_time=TIME_LIMIT, max_iterations=MAX_ITERATIONS, restarts=RESTARTS, smooth=SMOOTHING)
    if waypoints is None:
        return PlanResult(None, f"no path found in {TIME_LIMIT:.0f} s", time.time() - started)

    # ! A free straight move comes back from birrt as every small step, without the
    #   start. Restore the start and keep only the corners so both results match.
    if tuple(waypoints[0]) != start:
        waypoints = waypoints_from_path([start] + list(waypoints))

    # * Rejoin the corners with `steer` and re-check: the joined path is not guaranteed collision-free.
    poses = [start]
    for q1, q2 in zip(waypoints[:-1], waypoints[1:]):
        poses += steer(q1, q2)
        if any(collision_fn(q) for q in extend_fn(q1, q2)):
            return PlanResult(None, "the smoothed path collides; plan again", time.time() - started)
    return PlanResult(timed_path(poses), seconds=time.time() - started, direct=direct)
