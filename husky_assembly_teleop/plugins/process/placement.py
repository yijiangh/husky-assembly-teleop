"""
Where the process is built: one rigid transform of the build (bars, connector halves, obstacles, arm targets).

Only the floor ("ground/…") keeps its authored pose. Everything else the process puts in the world moves by
`placement`, so every relation inside the build (grasps, joints) is kept. Robot bases stand on the floor: each is
moved, then projected onto the floor plane (its height kept, turned only about z). A placement off the floor plane (a
height, a roll or a pitch) then leaves the arm targets where no authored joints reach them: the live IK at the live
base (Step IK, the executor's) solves them, the joints in the process are only its guess.

Pure: any thread.
"""

from __future__ import annotations

import math
from dataclasses import replace

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.process import Process
from scaffolding_assembly_core.process.types import (GROUND_PREFIX, Action, LineSpec, Movement, RobotState,
                                                     RobotTarget, State, Target)

#: The authored place: no move.
IDENTITY = Pose()
#: Bodies that belong to the lab, not the build: they never move. Obstacles are part of the structure and move.
LAB_PREFIXES = (GROUND_PREFIX,)
#: Metres and radians under which a placement counts as on the floor plane (`off_floor`).
FLOOR_TOLERANCE = (1e-4, math.radians(0.01))


def placed(process: Process, placement: Pose) -> Process:
    """The process with its build moved by `placement` (world <- authored world); the authored process for none.

    Args:
        process: The process as authored.
        placement: The build's authored world frame in the world.

    Returns:
        Process: A new process (the same folder), or `process` itself when `placement` is the identity.
    """
    if is_identity(placement):
        return process
    move = _Mover(placement)
    bodies = {body_id: body if _stays(body_id) else replace(body, assembly_pose=move.pose(body.assembly_pose))
              for body_id, body in process.bodies.items()}
    actions = {action_id: _action(action, move) for action_id, action in process.actions.items()}
    return replace(process, bodies=bodies, actions=actions)


def is_identity(placement: Pose) -> bool:
    """Whether a placement moves nothing (within FLOOR_TOLERANCE)."""
    turn = Rotation.from_quat(placement.orientation).magnitude()
    return float(np.linalg.norm(placement.position)) < FLOOR_TOLERANCE[0] and turn < FLOOR_TOLERANCE[1]


def off_floor(placement: Pose) -> bool:
    """Whether a placement lifts or tilts the build: a height, a roll or a pitch (robots then stand off the floor)."""
    z_axis = Rotation.from_quat(placement.orientation).apply([0.0, 0.0, 1.0])
    tilt = math.acos(max(-1.0, min(1.0, float(z_axis[2]))))
    return abs(float(placement.position[2])) >= FLOOR_TOLERANCE[0] or tilt >= FLOOR_TOLERANCE[1]


def placement_text(placement: Pose) -> str:
    """The placement for people: "x, y, z m, roll, pitch, yaw °"."""
    x, y, z = placement.position
    roll, pitch, yaw = Rotation.from_quat(placement.orientation).as_euler("xyz", degrees=True)
    return f"{x:.3f}, {y:.3f}, {z:.3f} m, {roll:.1f}, {pitch:.1f}, {yaw:.1f}°"


class _Mover:
    """Moves world poses and directions by one placement."""

    def __init__(self, placement: Pose):
        self._placement = placement
        self._rotation = Rotation.from_quat(placement.orientation)

    def pose(self, pose: Pose) -> Pose:
        return compose(self._placement, pose)

    def base(self, base: Pose) -> Pose:
        """A robot base moved, then projected onto the floor plane: its own height kept, turned only about z."""
        moved = compose(self._placement, base)
        heading = self._rotation.apply([1.0, 0.0, 0.0])
        turn = Rotation.from_euler("z", math.atan2(heading[1], heading[0]))
        orientation = (turn * Rotation.from_quat(base.orientation)).as_quat()
        return Pose.from_arrays([moved.position[0], moved.position[1], base.position[2]], orientation)

    def direction(self, direction) -> tuple[float, float, float]:
        return tuple(float(value) for value in self._rotation.apply(direction))


def _stays(body_id: str) -> bool:
    return body_id.startswith(LAB_PREFIXES)


def _action(action: Action, move: _Mover) -> Action:
    return replace(action, movements=tuple(_movement(movement, move) for movement in action.movements))


def _movement(movement: Movement, move: _Mover) -> Movement:
    line = {flange: LineSpec(move.direction(spec.direction), spec.distance) for flange, spec in movement.line.items()}
    target = None if movement.target is None else _target(movement.target, move)
    return replace(movement, start=_state(movement.start, move), line=line, target=target)


def _state(state: State, move: _Mover) -> State:
    robots = {robot_id: None if robot is None else RobotState(None if robot.base is None else move.base(robot.base),
                                                              robot.joints)
              for robot_id, robot in state.robots.items()}
    poses = {body_id: pose if _stays(body_id) else move.pose(pose) for body_id, pose in state.poses.items()}
    return replace(state, robots=robots, poses=poses)


def _target(target: Target, move: _Mover) -> Target:
    robots = {robot_id: RobotTarget(None if part.base is None else move.base(part.base), part.joints,
                                    {link: move.pose(pose) for link, pose in part.link_poses.items()})
              for robot_id, part in target.robots.items()}
    return replace(target, robots=robots)
