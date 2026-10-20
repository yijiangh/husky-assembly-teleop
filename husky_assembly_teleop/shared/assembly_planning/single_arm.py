"""
A straight tool0 move of one arm: exact ssik IK at every step of the line, nearest branch first, collision-checked.

compas_fab's cartesian planner (descent IK within 1 mm) runs where no ssik solver fits the arm.

! Worker thread only, on the thread that owns the mirror, after `mirror.sync`. The search check leaves the robot at
  the last pose checked, and the compas_fab fallback lends the mirror: sync it again after `plan_linear`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np
from compas_fab.backends import BackendError
from compas_fab.robots import FrameWaypoints, TargetMode
from scipy.spatial.transform import Rotation, Slerp

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.compas import frame_from_pose, planning_group
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.robot import RobotObject

from .collision import collisions, world_frame_fk
from .ik import branches, fitting_solver, ssik_unavailable
from .joints import arm_joint_names

#: Largest tool0 move between two steps of the line: metres, and radians of turn.
STEP_DISTANCE = 0.005
STEP_ANGLE = 0.05
#: Largest change of any joint between two steps, radians: a bigger one is a jump to another branch.
MAX_JOINT_STEP = math.radians(10.0)
#: compas_fab's cartesian planner, as the old monitor ran it for the support robots.
CARTESIAN_OPTIONS = {"max_step_distance": STEP_DISTANCE, "max_step_angle": STEP_ANGLE, "max_jump_revolute": 0.35,
                     "check_collision": True, "verbose": False,
                     # ? Its default of 20 descent steps gave up a quarter of the way along a 1 mm line.
                     "max_descend_iterations": 200}
#: Position and orientation tolerance of compas_fab's targets, metres and radians.
# ! Equal on purpose: compas_fab gives every step the orientation tolerance as its position tolerance too.
CARTESIAN_TOLERANCE = 1e-3


@dataclass(frozen=True)
class LinearPath:
    """A straight tool0 move of one arm, or why there is none.

    Attributes:
        joint_names: The arm's six joints, the order of each row of `points`.
        points: Joint values of every step, the start first; None if no path.
        backend: "ssik", or "compas_fab" where no ssik solver fits.
        reason: Why there is no path; empty otherwise.
        waypoint: The step that failed, from 1; 0 if none did.
        colliding: The pair that blocked the step, as scene ids, if a collision did.
    """

    joint_names: tuple[str, ...]
    points: np.ndarray | None
    backend: str
    reason: str = ""
    waypoint: int = 0
    colliding: tuple[str, str] | None = None


def interpolate_poses(start: Pose, goal: Pose, max_step_distance: float = STEP_DISTANCE,
                      max_step_angle: float = STEP_ANGLE) -> list[Pose]:
    """Poses evenly along the straight line from start to goal, turning evenly (slerp), within both step limits.

    Args:
        start: The first pose.
        goal: The last pose.
        max_step_distance: Largest move between two poses, metres.
        max_step_angle: Largest turn between two poses, radians.

    Returns:
        list[Pose]: n + 1 poses, exactly `start` first and `goal` last; two when nothing moves.
    """
    a, b = np.asarray(start.position, dtype=float), np.asarray(goal.position, dtype=float)
    rotations = Rotation.from_quat([start.orientation, goal.orientation])
    angle = (rotations[0].inv() * rotations[1]).magnitude()
    count = max(1, math.ceil(float(np.linalg.norm(b - a)) / max_step_distance), math.ceil(angle / max_step_angle))
    fractions = np.linspace(0.0, 1.0, count + 1)
    turns = Slerp([0.0, 1.0], rotations)(fractions).as_quat()
    inner = [Pose.from_arrays(a + (b - a) * fraction, turn) for fraction, turn in zip(fractions[1:-1], turns[1:-1])]
    return [start, *inner, goal]


def plan_linear(mirror: CompasFabMirror, robot: RobotObject, arm: str, goal: Pose, *,
                log: Callable[[str], None] = print) -> LinearPath:
    """Move one arm's tool0 in a straight line from where the robot has it to a world goal.

    Every step after the first takes the nearest ssik branch to the step before that moves no joint more than
    MAX_JOINT_STEP and collides with nothing.

    Args:
        mirror: The robot's mirror, synced to the scene `robot` is from.
        robot: The robot, at its live base and start joints.
        arm: The arm, e.g. "ur_arm".
        goal: World pose of the arm's tool0 at the end.
        log: Told when compas_fab's planner runs instead of ssik.

    Returns:
        LinearPath: The steps, or the one that failed and why.
    """
    names = tuple(arm_joint_names(arm))
    urdf = robot.model.urdf
    why = ssik_unavailable()
    solver = None if why else fitting_solver(urdf, arm)
    if solver is None:
        log(f"! {robot.id} {arm}: linear move by compas_fab's cartesian planner (within 1 mm): "
            f"{why or 'no ssik solver fits its URDF'}")
        return _cartesian(mirror, arm, goal, names)

    poses = interpolate_poses(link_pose(urdf, robot.base, robot.joints, f"{arm}_tool0"), goal)
    check = mirror.search_check(names)
    points = [np.array([robot.joints[name] for name in names], dtype=float)]
    for index, pose in enumerate(poses[1:], start=1):
        where = f"waypoint {index}/{len(poses) - 1}"
        previous = points[-1]
        found = branches(urdf, robot.base, arm, solver, pose, previous)
        if not found:
            return LinearPath(names, None, "ssik", f"{where}: unreachable", index)
        near = [q for q in found if float(np.max(np.abs(q - previous))) <= MAX_JOINT_STEP]
        if not near:
            step = float(np.max(np.abs(found[0] - previous)))
            return LinearPath(names, None, "ssik", f"{where}: joint jump of {step:.3f} rad (most "
                                                   f"{MAX_JOINT_STEP:.3f})", index)
        free = next((q for q in near if check(q) is None), None)
        if free is None:
            pairs = collisions(mirror, dict(zip(names, near[0].tolist())), full_report=True)
            pair = pairs[0] if pairs else None
            hit = f": {pair[0]} with {pair[1]}" if pair else ""
            return LinearPath(names, None, "ssik", f"{where}: collides{hit}", index, pair)
        points.append(free)
    return LinearPath(names, np.array(points), "ssik")


def _cartesian(mirror: CompasFabMirror, arm: str, goal: Pose, names: tuple[str, ...]) -> LinearPath:
    """The straight move by compas_fab's cartesian planner, on the lent mirror from its synced state."""
    waypoints = FrameWaypoints([frame_from_pose(goal)], TargetMode.ROBOT, tolerance_position=CARTESIAN_TOLERANCE,
                               tolerance_orientation=CARTESIAN_TOLERANCE)
    try:
        group = planning_group(mirror.cell, f"{arm}_tool0")
        with mirror.lend() as planner, world_frame_fk(planner):
            trajectory = planner.plan_cartesian_motion(waypoints, mirror.state, group, options=dict(CARTESIAN_OPTIONS))
    except (BackendError, ValueError) as error:
        # ? `message`, not str(): compas_fab rewrites a joint jump's message after making the error.
        return LinearPath(names, None, "compas_fab", f"compas_fab: {getattr(error, 'message', None) or error}")
    order = [list(trajectory.joint_names).index(name) for name in names]
    points = np.array([[point.joint_values[i] for i in order] for point in trajectory.points], dtype=float)
    return LinearPath(names, points, "compas_fab")
