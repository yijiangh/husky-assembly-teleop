"""
BaseCollisionWorld: what a robot base at a floor pose comes near, in a private PyBullet mirror of the scene.

* Used by every plugin that drives a base: planners check their moves, experiments their paths and the robot as it
  drives. Each keeps its own world on its own worker thread; a planner may subclass it for more checks.
! Worker thread only: construct it anywhere, but sync, check and close on one worker. A mirror belongs to one thread.
! Use raw `p` calls with `physicsClientId`: `pp.CLIENT` is one global shared by the whole process.
"""

from __future__ import annotations

import math

import numpy as np
import pybullet as p

from scaffolding_assembly_core.mirrors.pybullet import PyBulletMirror
from scaffolding_assembly_core.scene import Scene


class BaseCollisionWorld:
    """A private PyBullet mirror of the scene that says what a robot base at a floor pose comes near. Worker only."""

    def __init__(self):
        """Start empty; the first `sync` loads the robots and bodies (slow, on the worker)."""
        self.mirror: PyBulletMirror | None = None
        # Display text for our ids, from the last synced snapshot.
        self._label = Scene().label
        # ? Cheap pre-check for `hit_by`: floor footprints (min x, min y, max x, max y) by id, and robot reach.
        self._footprints: dict[str, tuple[float, float, float, float]] = {}
        self._reach: dict[str, float] = {}

    def sync(self, snapshot: Scene) -> None:
        """Make the world match a snapshot, and refresh footprints and reach.

        Args:
            snapshot: The tick's copy of the world, taken on the main thread.
        """
        if self.mirror is None:
            self.mirror = PyBulletMirror()  # here, on the worker: a mirror lives on one thread
        self.mirror.sync(snapshot)
        self._label = snapshot.label
        # Footprint and reach depend on arm pose, so refresh them every sync.
        ids = list(self.mirror.robots) + self.mirror.obstacle_ids()
        self._footprints = {object_id: self._footprint(self.mirror.body_ids(object_id)) for object_id in ids}
        self._reach = {}
        for object_id, body in self.mirror.robots.items():
            footprint = self._footprints[object_id]
            position = p.getBasePositionAndOrientation(body, physicsClientId=self.mirror.client_id)[0]
            corners = np.array([(footprint[i], footprint[j]) for i in (0, 2) for j in (1, 3)])
            self._reach[object_id] = float(np.max(np.linalg.norm(corners - np.array(position[:2]), axis=1)))

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

    def hit_by(self, own: str, pose, margin: float) -> str | None:
        """Place a robot at a floor pose and say what it hits, if anything.

        Args:
            own: The robot being planned for, by id.
            pose: (x, y, yaw).
            margin: Keep at least this far from everything else, metres.

        Returns:
            str | None: Label of the first robot or body within `margin`; None if clear.
        """
        body = self.mirror.robot(own)
        client = self.mirror.client_id
        z = p.getBasePositionAndOrientation(body, physicsClientId=client)[0][2]
        x, y, yaw = pose
        # ? Moving the robot here is fine: the mirror re-poses robots on every sync.
        p.resetBasePositionAndOrientation(body, (x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
                                          physicsClientId=client)
        reach = self._reach.get(own, math.inf) + margin
        candidates = []
        for object_id, footprint in self._footprints.items():
            # Distance from the robot's base origin to the footprint rectangle.
            dx = max(footprint[0] - x, 0.0, x - footprint[2])
            dy = max(footprint[1] - y, 0.0, y - footprint[3])
            if object_id != own and math.hypot(dx, dy) <= reach:
                candidates.append(object_id)
        hits = self.mirror.collisions(own, margin, candidates)
        return self._label(hits[0]) if hits else None

    def path_hit(self, own: str, poses, margin: float) -> str | None:
        """What a robot comes within `margin` of at any of some floor poses, first in order; None if all are clear.

        Args:
            own: The robot, by id.
            poses: (x, y, yaw) poses, close enough together that checking each checks the way between.
            margin: Keep at least this far from everything else, metres.
        """
        for pose in poses:
            hit = self.hit_by(own, pose, margin)
            if hit is not None:
                return hit
        return None

    def close(self) -> None:
        """Disconnect the world. On the worker, once no search is running."""
        if self.mirror is not None:
            self.mirror.close()
            self.mirror = None
