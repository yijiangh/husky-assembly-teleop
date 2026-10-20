"""
Exact inverse kinematics at a robot's live base with the ssik solvers, from the model URDF alone.

ssik solves tool0 in the arm's base link: T = (B·A)⁻¹·F*, with B the robot base, A the arm base link in the robot's
root frame and F* the world tool0 target. Every branch is exact; the nearest collision-free combination wins.
PyBullet's IK runs where no solver fits.

- ! Worker thread only, after `mirror.sync(scene)` of the scene `robot` comes from. Sync again after `solve_ik`
  before building a search check: the collision checks move the robot in the mirror.
- ! Solve on the live robot's model: the process URDFs of Alice and Belle fit no solver.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from functools import lru_cache
from itertools import product
from pathlib import Path
from typing import Callable, Mapping, Sequence

import numpy as np
from compas_fab.backends import InverseKinematicsError
from compas_fab.backends.pybullet.exceptions import PlanningGroupNotSupported
from compas_fab.robots import FrameTarget, RobotCellState, TargetMode
from husky_assembly_tamp.keyframe import ssik_inprocess
from husky_assembly_tamp.motion_planner.ssik_ik import ik_backend, rebranch_toward_seed
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.compas import frame_from_pose, planning_group
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.robot import RobotObject

from .collision import collisions, world_frame_fk
from .joints import arm_joint_names, joint_limits

#: The ssik solvers that may solve each arm, tried in order: each was built from one robot's calibrated URDF.
SSIK_SOLVERS = {"left_ur_arm": ("left",), "right_ur_arm": ("right",), "ur_arm": ("alice", "belle")}
#: Most branches ssik lists per arm (a UR arm has at most 8).
MAX_BRANCHES = 8
#: How close a solver's forward kinematics must be to the URDF's to fit it, metres and radians.
FIT_TOL = 1e-6
#: The joint sets the fit is checked at.
FIT_JOINTS = (np.zeros(6), np.full(6, 0.5), np.array([1.0, -1.2, 1.4, -0.8, 0.6, -0.4]))
#: PyBullet's IK stops this close to the target, metres and radians.
PYBULLET_TOLERANCE = (0.001, 0.01)
#: PyBullet's IK: attempts (random restarts after the first) and descent steps per attempt.
PYBULLET_OPTIONS = {"max_results": 20, "max_descend_iterations": 200}

# (URDF, arm) already told that IK falls back to PyBullet: told once, not on every solve.
_told: set[tuple[Path, str]] = set()


@dataclass(frozen=True)
class IkResult:
    """What `solve_ik` found.

    Attributes:
        joints: The solution's arm joints by name, or None.
        max_change: Largest joint move from the reference, radians.
        rank: Position of the solution among the combinations, nearest first, from 1; 0 if none.
        combinations: How many combinations of branches there were.
        backend: "ssik" or "pybullet".
        solvers: The ssik solver of each arm, in target order; empty for PyBullet.
        colliding: The nearest combination's first colliding pair, if it collided.
        colliding_joints: Its joints, if it collided.
        reason: Why there is no solution, when `joints` is None.
    """

    joints: dict[str, float] | None
    max_change: float = 0.0
    rank: int = 0
    combinations: int = 0
    backend: str = "ssik"
    solvers: tuple[str, ...] = ()
    colliding: tuple[str, str] | None = None
    colliding_joints: dict[str, float] | None = field(default=None, repr=False)
    reason: str = ""


def ssik_unavailable() -> str | None:
    """Why the ssik solvers can't run in this Python, or None if they can."""
    if ik_backend() != "ssik":
        return "HUSKY_IK_BACKEND=gradient"
    if not ssik_inprocess.available():
        return "ssik is not installed in this Python ('pip install \"ssik>=4.1,<5\"')"
    return None


@lru_cache(maxsize=None)
def fitting_solver(urdf: Path, arm: str) -> str | None:
    """The first ssik solver whose forward kinematics match the URDF's for an arm, at all FIT_JOINTS; None if none.

    ! Only when `ssik_unavailable()` is None.

    Args:
        urdf: The robot's URDF.
        arm: The arm, e.g. "left_ur_arm" or "ur_arm".
    """
    names = arm_joint_names(arm)
    try:
        arm_base = link_pose(urdf, Pose(), {}, f"{arm}_base_link")
        in_base = [compose(invert(arm_base), link_pose(urdf, Pose(), dict(zip(names, q)), f"{arm}_tool0")).matrix()
                   for q in FIT_JOINTS]
    except KeyError:
        return None  # the URDF has no such arm
    for solver in SSIK_SOLVERS.get(arm, ()):
        if ssik_inprocess.joint_names(solver) == names and all(
                _close(model, ssik_inprocess.fk(solver, q)) for model, q in zip(in_base, FIT_JOINTS)):
            return solver
    return None


def branches(urdf: Path, base: Pose, arm: str, solver: str, target: Pose,
             reference: Sequence[float]) -> list[np.ndarray]:
    """Every ssik branch of one arm reaching a world tool0 target, turned toward the reference and nearest first.

    Args:
        urdf: The robot's URDF.
        base: World pose of the robot's root link.
        arm: The arm, e.g. "left_ur_arm".
        solver: Its fitting solver (`fitting_solver`).
        target: World pose of the arm's tool0.
        reference: The arm's six joints to stay near, UR driver order.

    Returns:
        list[np.ndarray]: At most MAX_BRANCHES joint sets, sorted by largest joint move from the reference; empty if
            the target is out of reach.
    """
    reference = np.asarray(reference, dtype=float)
    in_base = compose(invert(link_pose(urdf, base, {}, f"{arm}_base_link")), target).matrix()
    lower, upper = joint_limits(urdf, arm_joint_names(arm))
    found = [rebranch_toward_seed(q, reference, list(zip(lower, upper)))
             for q, _residual in ssik_inprocess.solve(solver, in_base, max_solutions=MAX_BRANCHES, q_seed=reference)]
    return sorted(found, key=lambda q: float(np.max(np.abs(q - reference))))


def solve_ik(mirror: CompasFabMirror, robot: RobotObject, targets: Mapping[str, Pose],
             reference: Mapping[str, float] | None = None, check_collision: bool = True,
             log: Callable[[str], None] = print) -> IkResult:
    """Arm joints that put each arm's tool0 at its world target, nearest the reference and collision-free.

    With ssik, the combinations of every arm's branches are tried nearest first (by the largest joint move); the
    first without collision wins. Without a fitting solver, PyBullet's IK runs instead, logged once per URDF and arm.

    Args:
        mirror: The robot's mirror, synced to the scene `robot` is from.
        robot: The robot, at its live base.
        targets: Arm name ("left_ur_arm") -> world pose of its tool0.
        reference: Joints to stay near; the robot's own where not given.
        check_collision: Refuse combinations that collide (`mirror.collisions`).
        log: Gets the fallback line.

    Returns:
        IkResult: The solution, or why there is none; when every combination collides, the nearest one's pair.
    """
    reference = {**robot.joints, **(reference or {})}
    urdf, why = robot.model.urdf, ssik_unavailable()
    solvers = {arm: None if why else fitting_solver(urdf, arm) for arm in targets}
    if None in solvers.values():
        for arm in [arm for arm, solver in solvers.items() if solver is None and (urdf, arm) not in _told]:
            _told.add((urdf, arm))
            log(f"! IK falls back to PyBullet for {robot.id} {arm}: {why or 'no ssik solver fits its URDF'} "
                f"(within 1 mm / 0.01 rad, random restarts)")
        return _pybullet_ik(mirror, targets, reference, check_collision)

    names = tuple(solvers.values())
    arms = []
    for arm, target in targets.items():
        seed = np.array([reference.get(name, 0.0) for name in arm_joint_names(arm)])
        found = branches(urdf, robot.base, arm, solvers[arm], target, seed)
        if not found:
            return IkResult(None, solvers=names, reason=f"{arm}: ssik finds no solution, the target is out of reach")
        arms.append((arm_joint_names(arm), seed, found))

    def largest_move(combination: tuple) -> float:
        """The largest joint move from the reference over the arms of one combination, radians.

        Args:
            combination: One branch per arm, in target order.

        Returns:
            float: The largest change of any joint.
        """
        return max(float(np.max(np.abs(q - seed))) for q, (_names, seed, _found) in zip(combination, arms))

    combinations = sorted(product(*(found for _names, _seed, found in arms)), key=largest_move)
    nearest = None
    for rank, combination in enumerate(combinations, start=1):
        joints = {name: float(value) for q, (joint_names, _seed, _found) in zip(combination, arms)
                  for name, value in zip(joint_names, q)}
        hit = collisions(mirror, joints) if check_collision else []
        if not hit:
            return IkResult(joints, largest_move(combination), rank, len(combinations), "ssik", names,
                            *(nearest or (None, None)))
        nearest = nearest or (hit[0], joints)
    return IkResult(None, largest_move(combinations[0]), 0, len(combinations), "ssik", names, *nearest,
                    reason=f"all {len(combinations)} solutions collide; the nearest: {nearest[0][0]} with "
                           f"{nearest[0][1]}")


def _pybullet_ik(mirror: CompasFabMirror, targets: Mapping[str, Pose], reference: Mapping[str, float],
                 check_collision: bool) -> IkResult:
    """IK by PyBullet through compas_fab, one arm after the other, each from the last; checked once, merged."""
    state = mirror.state_at(reference)
    options = {**PYBULLET_OPTIONS, "return_full_configuration": True, "check_collision": False}
    try:
        with mirror.lend() as planner, world_frame_fk(planner):
            for arm, target in targets.items():
                goal = FrameTarget(frame_from_pose(target), TargetMode.ROBOT, tolerance_position=PYBULLET_TOLERANCE[0],
                                   tolerance_orientation=PYBULLET_TOLERANCE[1])
                # ? iter_inverse_kinematics, not inverse_kinematics: that one hands out the next solution on a repeat.
                group = planning_group(mirror.cell, f"{arm}_tool0")
                configuration = next(planner.iter_inverse_kinematics(goal, state, group, options))
                state = RobotCellState(robot_base_frame=state.robot_base_frame, robot_configuration=configuration,
                                       tool_states=state.tool_states, rigid_body_states=state.rigid_body_states)
    except (InverseKinematicsError, PlanningGroupNotSupported, KeyError) as error:
        return IkResult(None, backend="pybullet", reason=f"PyBullet IK: {error}")
    names = [name for arm in targets for name in arm_joint_names(arm)]
    joints = {name: float(state.robot_configuration[name]) for name in names}
    change = max(abs(joints[name] - reference.get(name, 0.0)) for name in names)
    hit = collisions(mirror, joints) if check_collision else []
    if hit:
        return IkResult(None, change, 0, 1, "pybullet", (), hit[0], joints, reason=f"the solution collides: {hit[0]}")
    return IkResult(joints, change, 1, 1, "pybullet")


def _close(model: np.ndarray, solver: np.ndarray) -> bool:
    """Whether two 4x4 transforms agree within FIT_TOL in position and angle."""
    turn = Rotation.from_matrix(model[:3, :3].T @ np.asarray(solver)[:3, :3]).magnitude()
    return float(np.linalg.norm(model[:3, 3] - np.asarray(solver)[:3, 3])) <= FIT_TOL and turn <= FIT_TOL
