"""
The planning world: a private PyBullet copy of the scene, for the base planner's collision checks.

! Worker thread only: construct `PlanningWorld` anywhere, but sync, check and close on the worker.
! Use raw `p` calls with `physicsClientId`: `pp.CLIENT` is one global shared by the whole process.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pybullet as p

from scaffolding_assembly_core.mirrors.pybullet import PyBulletMirror
from scaffolding_assembly_core.scene import Scene

from ...config import RobotConfig
from ...world.kinematics import stow_joints

#: Keep at least this far from every other robot and body, metres.
COLLISION_MARGIN = 0.05
#: `clearance` measures up to CLEARANCE_LIMIT, to CLEARANCE_STEP, metres.
CLEARANCE_LIMIT = 0.30
CLEARANCE_STEP = 0.005
#: Things lower than this, metres, are floor: the floor maps leave them out.
FLOOR_BOTTOM = 0.05
#: Rays per PyBullet call, below its batch limit.
RAY_BATCH = 8192
#: Where the planned robot is moved out of the way, metres along x and y.
PARK = -1000.0


def stowed(snapshot: Scene, object_id: str, config: RobotConfig) -> tuple[Scene, float]:
    """The snapshot with the robot's arms at their stow pose, as the base drives; and how far the arms are from it.

    Returns:
        tuple[Scene, float]: The snapshot, a copy; the largest joint's distance from its stow angle, radians, or 0 for
        joints not measured.
    """
    robot = snapshot.robots[object_id]
    stow = stow_joints(config, robot.model.movable_joints)
    away = max((abs(math.remainder(robot.joints.get(name, 0.0) - value, math.tau)) for name, value in stow.items()
                if name not in robot.unmeasured), default=0.0)
    robot = replace(robot, joints={**robot.joints, **stow}, unmeasured=robot.unmeasured.difference(stow))
    return replace(snapshot, robots={**snapshot.robots, object_id: robot}), away


class PlanningWorld:
    """A scene mirror for collision checks, plus a cheap floor-footprint pre-check. Worker thread only."""

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

    def hit_by(self, own: str, pose, margin: float = COLLISION_MARGIN) -> str | None:
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

    def touch(self, own: str, pose, margin: float = COLLISION_MARGIN) -> tuple[str, tuple[float, float, float]] | None:
        """What a robot at a floor pose hits first, and the point on it nearest the robot, for drawing.

        Args:
            own: The robot being planned for, by id.
            pose: (x, y, yaw).
            margin: Keep at least this far from everything else, metres.

        Returns:
            tuple[str, tuple[float, float, float]] | None: The label and the point, metres; None if clear.
        """
        if self.hit_by(own, pose, margin) is None:
            return None
        other = self.mirror.collisions(own, margin)[0]
        client = self.mirror.client_id
        # ? Nearest over all links: a link pair allowed to touch may be the nearest, which only moves the dot.
        nearest = min((contact for body in self.mirror.body_ids(other)
                       for contact in p.getClosestPoints(self.mirror.robot(own), body, margin, physicsClientId=client)),
                      key=lambda contact: contact[8])
        return self._label(other), tuple(nearest[6])

    def clearance(self, own: str, poses, limit: float = CLEARANCE_LIMIT) -> tuple[float, tuple | None]:
        """The least distance from a robot to everything else over some floor poses, to CLEARANCE_STEP.

        Args:
            own: The robot, by id.
            poses: (x, y, yaw) poses, close enough together that checking each checks the way between.
            limit: Measure up to this, metres.

        Returns:
            tuple[float, tuple | None]: The clearance, metres (0 where it touches, `limit` if never nearer), and the
            pose where it is least (None if never nearer than `limit`).
        """
        lowest, where = limit, None
        for pose in poses:
            # * Only a pose nearer than the lowest so far needs measuring.
            if self.hit_by(own, pose, lowest) is None:
                continue
            clear, blocked = 0.0, lowest
            if self.hit_by(own, pose, 0.0) is not None:
                blocked = 0.0
            while blocked - clear > CLEARANCE_STEP:
                middle = (clear + blocked) / 2.0
                if self.hit_by(own, pose, middle) is None:
                    clear = middle
                else:
                    blocked = middle
            lowest, where = clear, tuple(float(value) for value in pose)
        return lowest, where

    def floor_heights(self, own: str, low, cell: float, shape: tuple[int, int], bottom: float) -> np.ndarray:
        """How high the other robots and bodies reach above each floor cell, from `bottom` up.

        One ray per cell centre, from above everything down, so only the topmost surface counts: something above the
        robot hides what is under it. Bodies `own` may touch are left out, and `own` is moved out of the way.

        Args:
            own: The robot being planned for, by id.
            low: (x, y) of the centre of cell (0, 0), metres.
            cell: Cell size, metres.
            shape: Cells along x and along y.
            bottom: Lowest height that counts, metres.

        Returns:
            np.ndarray: Heights, metres, of `shape`; -inf where nothing is.
        """
        self._park(own)
        others = {other for object_id in self._footprints if object_id != own
                  and not self.mirror.allowed(own, object_id) for other in self.mirror.body_ids(object_id)}
        # * Start above every body: a ray that starts inside one does not see it.
        client = self.mirror.client_id
        top = max((p.getAABB(body, link, physicsClientId=client)[1][2] for body in others
                   for link in range(-1, p.getNumJoints(body, physicsClientId=client))), default=bottom) + 0.1
        x, y = np.meshgrid(low[0] + cell * np.arange(shape[0]), low[1] + cell * np.arange(shape[1]), indexing="ij")
        heights = self._rays(np.column_stack([x.ravel(), y.ravel()]), top, bottom, others)
        return np.where(np.isnan(heights), -np.inf, heights).reshape(shape)

    def robot_heights(self, own: str, cell: float, bottom: float) -> tuple[np.ndarray, np.ndarray]:
        """How high and how low robot `own`, as it stands now, reaches above each cell around its base, at yaw 0.

        Args:
            own: The robot, by id; moved out of the way of everything else.
            cell: Cell size, metres.
            bottom: Lowest height that counts, metres.

        Returns:
            tuple[np.ndarray, np.ndarray]: Its highest and lowest surface per cell, metres (-inf and inf where it is
            not), on a square grid of odd size whose centre cell is base_footprint, x along the first axis.
        """
        top = self._park(own)
        body = self.mirror.robot(own)
        x, y = p.getBasePositionAndOrientation(body, physicsClientId=self.mirror.client_id)[0][:2]
        n = int(math.ceil(self._reach.get(own, 0.0) / cell)) + 1
        dx, dy = np.meshgrid(cell * np.arange(-n, n + 1), cell * np.arange(-n, n + 1), indexing="ij")
        xy = np.column_stack([x + dx.ravel(), y + dy.ravel()])
        bodies = set(self.mirror.body_ids(own))
        highest, lowest = self._rays(xy, top, bottom, bodies), self._rays(xy, bottom, top, bodies)
        return (np.where(np.isnan(highest), -np.inf, highest).reshape(dx.shape),
                np.where(np.isnan(lowest), np.inf, lowest).reshape(dx.shape))

    def _park(self, own: str) -> float:
        """Move robot `own` far from everything, at yaw 0, and return the height of its top, metres."""
        client = self.mirror.client_id
        body = self.mirror.robot(own)
        z = p.getBasePositionAndOrientation(body, physicsClientId=client)[0][2]
        p.resetBasePositionAndOrientation(body, (PARK, PARK, z), (0.0, 0.0, 0.0, 1.0), physicsClientId=client)
        return max(p.getAABB(body, link, physicsClientId=client)[1][2]
                   for link in range(-1, p.getNumJoints(body, physicsClientId=client)))

    def _rays(self, xy: np.ndarray, start: float, end: float, bodies: set[int]) -> np.ndarray:
        """Cast a vertical ray at each (x, y) from height `start` to `end`, and return the heights where they hit.

        A ray's height is nan where it hits nothing, or something not in `bodies` first.
        """
        heights = []
        for i in range(0, len(xy), RAY_BATCH):
            chunk = xy[i:i + RAY_BATCH]
            hits = p.rayTestBatch(np.column_stack([chunk, np.full(len(chunk), start)]).tolist(),
                                  np.column_stack([chunk, np.full(len(chunk), end)]).tolist(),
                                  physicsClientId=self.mirror.client_id)
            heights += [hit[3][2] if hit[0] in bodies else math.nan for hit in hits]
        return np.array(heights, dtype=float)

    def set_gui(self, gui: bool, snapshot: Scene) -> None:
        """Open or close PyBullet's own window on the planning world, for debugging. Worker thread.

        Args:
            gui: Whether the window should be open.
            snapshot: The world now, shown even before the first plan.

        Raises:
            RuntimeError: If there is no X display.
            pybullet.error: If another PyBullet window is open in this process.
        """
        if self.mirror is None:
            self.mirror = PyBulletMirror()
        self.mirror.set_gui(gui)
        self.sync(snapshot)

    def close(self) -> None:
        """Disconnect the world. On the worker, once no search is running."""
        if self.mirror is not None:
            self.mirror.close()
            self.mirror = None
