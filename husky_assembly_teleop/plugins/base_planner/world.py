"""
The planning world: the shared base collision world (`BaseCollisionWorld`) plus the base planner's own checks.

The planner plans the robot as it is, arms and what it holds included; `stowed` is for the experiments, which hold the
arms stowed.

! Worker thread only: construct `PlanningWorld` anywhere, but sync, check and close on the worker.
! Use raw `p` calls with `physicsClientId`: `pp.CLIENT` is one global shared by the whole process.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
import pybullet as p

from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.pybullet import PyBulletMirror
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene, world_poses

from ...config import RobotConfig
from ...shared.base_collision.base_collision_world import BaseCollisionWorld, held_bodies
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


def from_stow(robot: RobotObject, config: RobotConfig) -> float:
    """How far a robot's arms are from their stow pose: the largest measured joint's distance from it, radians."""
    stow = stow_joints(config, robot.model.movable_joints)
    return max((abs(math.remainder(robot.joints.get(name, 0.0) - value, math.tau)) for name, value in stow.items()
                if name not in robot.unmeasured), default=0.0)


def stowed(snapshot: Scene, object_id: str, config: RobotConfig) -> tuple[Scene, float]:
    """The snapshot with the robot's arms at their stow pose, and what it holds moved with them; and how far they are.

    Returns:
        tuple[Scene, float]: The snapshot, a copy; and `from_stow` before stowing.
    """
    robot = snapshot.robots[object_id]
    away = from_stow(robot, config)
    stow = stow_joints(config, robot.model.movable_joints)
    robot = replace(robot, joints={**robot.joints, **stow}, unmeasured=robot.unmeasured.difference(stow))
    poses = world_poses(snapshot.bodies, {object_id: robot},
                        lambda held_by, link: link_pose(held_by.model.urdf, held_by.base, held_by.joints, link))
    held = {body_id: poses[body_id] for body_id in held_bodies(snapshot, object_id) if body_id in poses}
    return replace(snapshot, robots={**snapshot.robots, object_id: robot},
                   world_poses={**snapshot.world_poses, **held}), away


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
        hit = self._hit(own, pose, margin)
        if hit is None:
            return None
        part, other = hit
        client = self.mirror.client_id
        # ? Nearest over all links: a link pair allowed to touch may be the nearest, which only moves the dot.
        nearest = min((contact for own_body in self.mirror.body_ids(part) for body in self.mirror.body_ids(other)
                       for contact in p.getClosestPoints(own_body, body, margin, physicsClientId=client)),
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
        hides what is under it. Bodies `own` may touch or holds are left out, and `own` is moved out of the way.

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
        parts = self._parts(own)
        others = {other for object_id in self._footprints if object_id not in parts
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
        """How high and how low robot `own` and what it holds reach above each cell around its base, at yaw 0.

        Args:
            own: The robot, by id, its arms as they stand; moved out of the way of everything else.
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
        bodies = {body for part in self._parts(own) for body in self.mirror.body_ids(part)}
        highest, lowest = self._rays(xy, top, bottom, bodies), self._rays(xy, bottom, top, bodies)
        return (np.where(np.isnan(highest), -np.inf, highest).reshape(dx.shape),
                np.where(np.isnan(lowest), np.inf, lowest).reshape(dx.shape))

    def _park(self, own: str) -> float:
        """Move robot `own` and what it holds far from everything, at yaw 0; the height of their top, metres."""
        client = self.mirror.client_id
        self._place(own, (PARK, PARK, 0.0))
        return max(p.getAABB(body, link, physicsClientId=client)[1][2]
                   for part in self._parts(own) for body in self.mirror.body_ids(part)
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
