"""
BaseCollisionWorld: what a robot base at a floor pose comes near, in a private PyBullet mirror of the scene.

* Used by every plugin that drives a base: planners check their moves, experiments their paths. Each keeps its own
  world on its own worker thread; a planner may subclass it for more checks. Clearances (`clearance.py`) use one too.
* What a robot holds (`held_bodies`) moves with it and is checked with it; contact between the two is left out, as
  driving never changes it.
! Worker thread only: construct it anywhere, but sync, check and close on one worker. A mirror belongs to one thread.
! Use raw `p` calls with `physicsClientId`: `pp.CLIENT` is one global shared by the whole process.
"""

from __future__ import annotations

import math

import numpy as np
import pybullet as p

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.ids import TRACKED, link_id
from scaffolding_assembly_core.mirrors.pybullet import PyBulletMirror
from scaffolding_assembly_core.scene import Attachment, Scene

from .collision_hit import CollisionHit, Contact


def held_bodies(snapshot: Scene, robot: str) -> list[str]:
    """The bodies a robot holds: attached to one of its links or its base, or to a body it holds.

    Args:
        snapshot: The world.
        robot: The robot, by id.
    """
    def holder(body_id: str, seen: frozenset) -> str | None:
        placement = snapshot.bodies[body_id].placement
        if not isinstance(placement, Attachment):
            return None
        if placement.parent in snapshot.robots:
            return placement.parent
        if placement.parent in snapshot.bodies and placement.parent not in seen:
            return holder(placement.parent, seen | {body_id})
        return None

    return [body_id for body_id in snapshot.bodies if holder(body_id, frozenset()) == robot]


class BaseCollisionWorld:
    """A private PyBullet mirror of the scene that says what a robot base at a floor pose comes near. Worker only."""

    def __init__(self):
        """Start empty; the first `sync` loads the robots and bodies (slow, on the worker)."""
        self.mirror: PyBulletMirror | None = None
        #: The snapshot synced last: display text for our ids, and the scene of a `CollisionHit`.
        self.snapshot = Scene()
        # ? Cheap pre-check for `hit_by`: floor footprints (min x, min y, max x, max y) by id, and robot reach.
        self._footprints: dict[str, tuple[float, float, float, float]] = {}
        self._reach: dict[str, float] = {}
        # What each robot holds, by robot id: body id -> (pose in the robot's base frame, pose as synced).
        self._held: dict[str, dict[str, tuple[Pose, Pose]]] = {}
        # Robots whose held bodies a check moved: the mirror never moves a body back, so `sync` does.
        self._moved: set[str] = set()

    def sync(self, snapshot: Scene) -> None:
        """Make the world match a snapshot, and refresh footprints, reach and what each robot holds.

        Args:
            snapshot: The tick's copy of the world, taken on the main thread.
        """
        if self.mirror is None:
            self.mirror = PyBulletMirror()  # here, on the worker: a mirror lives on one thread
        for robot in self._moved:
            for body_id, (_, synced) in self._held[robot].items():
                self._put(body_id, synced)
        self._moved = set()
        self.mirror.sync(snapshot)
        self.snapshot = snapshot
        # Footprint, reach and what is held depend on arm pose, so refresh them every sync.
        ids = list(self.mirror.robots) + self.mirror.obstacle_ids()
        self._footprints = {object_id: self._footprint(self.mirror.body_ids(object_id)) for object_id in ids}
        self._held, self._reach = {}, {}
        for object_id, body in self.mirror.robots.items():
            base = invert(snapshot.robots[object_id].base)
            self._held[object_id] = {body_id: (compose(base, snapshot.world_poses[body_id]),
                                               snapshot.world_poses[body_id])
                                     for body_id in held_bodies(snapshot, object_id) if body_id in self._footprints}
            footprint = self._footprint([body for part in self._parts(object_id)
                                         for body in self.mirror.body_ids(part)])
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
        hit = self._hit(own, pose, margin)
        return self.snapshot.label(hit[1]) if hit is not None else None

    def collision_hit(self, own: str, margin: float, where: str, pose=None) -> CollisionHit | None:
        """Everything robot `own`, or what it holds, comes within `margin` of, where the last `sync` or `pose` put it.

        Args:
            own: The robot, by id.
            margin: Distance below which a pair counts, metres.
            where: What the robot was doing, for the hit's message.
            pose: (x, y, yaw) to place the robot at first, with what it holds; None: where the last `sync` put it.

        Returns:
            CollisionHit | None: Every such pair, robot or held body first, with its closest contact; None if clear.
                Its scene has the robot at `pose`.

        Raises:
            ValueError: If the robot cannot be checked, e.g. a joint is not measured (`RobotObject.acting_problems`).
        """
        if pose is not None:
            self._place(own, pose)
        parts = self._parts(own)
        candidates = [object_id for object_id in self._footprints if object_id not in parts]
        pairs = tuple((part, other) for part in parts for other in self.mirror.collisions(part, margin, candidates))
        if not pairs:
            return None
        snapshot = self.snapshot if pose is None else self._scene_at(own, pose)
        # ? The mirror's rule: a free, unmeasured body with a non-convex mesh is built concave, the rest as hulls.
        concave = frozenset(object_id for pair in pairs for object_id in pair if object_id in snapshot.bodies
                            and isinstance(snapshot.bodies[object_id].placement, Pose)
                            and not object_id.startswith(f"{TRACKED}/")
                            and any(not getattr(shape, "convex", True)
                                    for shape in snapshot.bodies[object_id].geometry.collision))
        return CollisionHit(snapshot, own, where, pairs, tuple(self._contact(pair, margin) for pair in pairs),
                            concave=concave, margin=margin)

    def _scene_at(self, own: str, pose) -> Scene:
        """A copy of the synced snapshot with robot `own` at a floor pose (x, y, yaw), as `_place` puts it."""
        scene = self.snapshot.copy()
        robot = scene.robots[own]
        x, y, yaw = pose
        robot.base = Pose((x, y, robot.base.position[2]), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        for body_id, (offset, _) in self._held.get(own, {}).items():
            scene.world_poses[body_id] = compose(robot.base, offset)
        return scene

    # * Core candidate: `_contact` and `_names` belong next to `PyBulletMirror.collisions`, which finds the same
    #   points but keeps only the other side's id; they repeat its private naming (`_names`: robot links, and each
    #   mounted tool as a "<flange>_tool" link).

    def _contact(self, pair: tuple[str, str], margin: float) -> Contact | None:
        """The closest contact of a pair within `margin`, allowed contacts left out; None if PyBullet finds none."""
        client = self.mirror.client_id
        closest, names = None, None
        for body_a in self.mirror.body_ids(pair[0]):
            for body_b in self.mirror.body_ids(pair[1]):
                for point in p.getClosestPoints(body_a, body_b, margin, physicsClientId=client):
                    # ? Point fields: 3, 4 links; 5, 6 positions on A and B; 8 signed distance.
                    sides = self._names(pair[0], body_a, point[3]), self._names(pair[1], body_b, point[4])
                    if any(self.mirror.allowed(a, b) for a in sides[0] for b in sides[1]):
                        continue
                    if closest is None or point[8] < closest[8]:
                        closest, names = point, sides
        if closest is None:
            return None
        return Contact((names[0][-1], names[1][-1]), tuple(closest[5]), tuple(closest[6]), float(closest[8]))

    def _names(self, object_id: str, body: int, link: int) -> list[str]:
        """The ids one side of a contact goes by, narrowest last: a body; or a robot link, then its mounted tool."""
        robot = self.snapshot.robots.get(object_id)
        if robot is None:
            return [object_id]
        client = self.mirror.client_id
        name = (p.getBodyInfo(body, physicsClientId=client)[0] if link < 0
                else p.getJointInfo(body, link, physicsClientId=client)[12]).decode()
        flange = name.removesuffix("_tool")
        tool = robot.model.tools.get(flange) if flange != name else None
        return [link_id(object_id, name)] + ([tool.id] if tool is not None else [])

    def reach(self, own: str) -> float:
        """How far robot `own` and what it holds reach across the floor from base_footprint, as last synced, metres."""
        return self._reach[own]

    def _hit(self, own: str, pose, margin: float) -> tuple[str, str] | None:
        """Place a robot at a floor pose with what it holds, and say which of them comes within `margin` of what.

        Returns:
            tuple[str, str] | None: The robot's or held body's id, and the id of the first thing it hits; None if clear.
        """
        x, y, _ = pose
        self._place(own, pose)
        reach = self._reach.get(own, math.inf) + margin
        parts = self._parts(own)
        candidates = []
        for object_id, footprint in self._footprints.items():
            # Distance from the robot's base origin to the footprint rectangle.
            dx = max(footprint[0] - x, 0.0, x - footprint[2])
            dy = max(footprint[1] - y, 0.0, y - footprint[3])
            if object_id not in parts and math.hypot(dx, dy) <= reach:
                candidates.append(object_id)
        for part in parts:
            hits = self.mirror.collisions(part, margin, candidates)
            if hits:
                return part, hits[0]
        return None

    def _parts(self, own: str) -> list[str]:
        """A robot and what it holds, robot first."""
        return [own, *self._held.get(own, {})]

    def _place(self, own: str, pose) -> None:
        """Move a robot to a floor pose (x, y, yaw), at its height, with what it holds."""
        body = self.mirror.robot(own)
        client = self.mirror.client_id
        z = p.getBasePositionAndOrientation(body, physicsClientId=client)[0][2]
        x, y, yaw = pose
        base = Pose((x, y, z), (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        # ? Moving the robot here is fine: the mirror re-poses robots on every sync.
        p.resetBasePositionAndOrientation(body, base.position, base.orientation, physicsClientId=client)
        held = self._held.get(own)
        if held:
            self._moved.add(own)
            for body_id, (offset, _) in held.items():
                self._put(body_id, Pose(*p.multiplyTransforms(base.position, base.orientation, offset.position,
                                                              offset.orientation)))

    def _put(self, body_id: str, pose: Pose) -> None:
        """Move every PyBullet body of a body to a world pose."""
        for body in self.mirror.body_ids(body_id):
            p.resetBasePositionAndOrientation(body, pose.position, pose.orientation,
                                              physicsClientId=self.mirror.client_id)

    def path_hit(self, own: str, poses, margin: float) -> CollisionHit | None:
        """What a robot comes within `margin` of at the first of some floor poses where it comes that close.

        Args:
            own: The robot, by id.
            poses: (x, y, yaw) poses, close enough together that checking each checks the way between.
            margin: Keep at least this far from everything else, metres.

        Returns:
            CollisionHit | None: The hit at that pose ("pose i of n"), or None if all are clear.
        """
        for index, pose in enumerate(poses):
            if self._hit(own, pose, margin) is not None:
                return self.collision_hit(own, margin, f"pose {index + 1} of {len(poses)}", pose)
        return None

    def close(self) -> None:
        """Disconnect the world. On the worker, once no search is running."""
        if self.mirror is not None:
            self.mirror.close()
            self.mirror = None
