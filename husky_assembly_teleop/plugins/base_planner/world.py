"""
The planning world: the shared base collision world (`BaseCollisionWorld`) plus the base planner's own checks.

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
from ...shared.base_collision.base_collision_world import BaseCollisionWorld
from ...world.kinematics import stow_joints

#: Keep at least this far from every other robot and body, metres.
COLLISION_MARGIN = 0.05
#: `clearance` measures up to CLEARANCE_LIMIT, to CLEARANCE_STEP, metres.
CLEARANCE_LIMIT = 0.30
CLEARANCE_STEP = 0.005
#: Things lower than this, metres, are floor: the floor maps leave them out.
FLOOR_BOTTOM = 0.05
#: Rays per floor cell along x and y in `floor_heights`, spread evenly over it: FLOOR_SAMPLES rays across a 5 cm cell
#: leave gaps narrower than the 2 cm bars, so no bar falls between them.
FLOOR_SAMPLES = 3
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


class PlanningWorld(BaseCollisionWorld):
    """The base collision world, plus what the base planner measures: clearance, contact points and floor maps."""

    def hit_by(self, own: str, pose, margin: float = COLLISION_MARGIN) -> str | None:
        """Like `BaseCollisionWorld.hit_by`, keeping the planner's own COLLISION_MARGIN by default."""
        return super().hit_by(own, pose, margin)

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

        FLOOR_SAMPLES by FLOOR_SAMPLES rays per cell, from above everything down, and the cell takes the highest hit:
        anything over part of a cell counts for all of it. Only the topmost surface counts: something above the robot
        hides what is under it. Bodies `own` may touch are left out, and `own` is moved out of the way.

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
        offsets = cell * ((np.arange(FLOOR_SAMPLES) + 0.5) / FLOOR_SAMPLES - 0.5)
        heights = np.full(shape, -np.inf)
        for dx in offsets:
            for dy in offsets:
                hit = self._rays(np.column_stack([x.ravel() + dx, y.ravel() + dy]), top, bottom, others)
                heights = np.fmax(heights, np.where(np.isnan(hit), -np.inf, hit).reshape(shape))
        return heights

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
