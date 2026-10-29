"""
Small moves of an arm's tool0 given in tool0's own frame, solved into joint angles on the robot's URDF.

The UR driver takes no Cartesian moves, so `tool_step_target` finds the joints with a few Newton steps on the URDF's
forward kinematics; `ArmInterface.send_tool_step` sends them as a short joint move. A jog while a button is held
(`tool_jog_target`) runs on the same solver, one point per tick ahead of the arm along a line from the press pose.
"""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

#: Largest tool step: metres, and radians.
TOOL_STEP_MAX = 0.01
TOOL_STEP_MAX_ANGLE = np.radians(5.0)
#: Largest change of any joint one tool step may need, radians: more means the arm is near a singularity.
TOOL_STEP_MAX_JOINT = np.radians(10.0)
#: A step is solved once tool0 is this close to its target: metres, and radians.
TOOL_STEP_TOLERANCE = 1e-7
TOOL_STEP_ANGLE_TOLERANCE = 1e-6
#: Newton steps tried before giving up.
TOOL_STEP_ITERATIONS = 10
#: Joint offset for the Jacobian's finite differences, radians.
_DELTA = 1e-7
#: A tool0 jog: how far ahead of the arm the point sent each tick lies, seconds; how fast a turn is per mm/s of the
#: speed slider, degrees per second; the fastest any joint may move; the slider's range and start, mm/s.
TOOL_JOG_LEAD = 0.2
TOOL_JOG_DEG_PER_S_PER_MM_S = 0.2
TOOL_JOG_MAX_JOINT_SPEED = np.radians(30.0)
TOOL_JOG_SPEED_MIN, TOOL_JOG_SPEED_MAX, TOOL_JOG_SPEED_DEFAULT = 1.0, 50.0, 5.0


def tool_step_target(urdf: Path, arm: str, joints: Sequence[float], step: Pose,
                     pivot: Sequence[float] = (0.0, 0.0, 0.0), max_shift: float = TOOL_STEP_MAX,
                     max_turn: float = TOOL_STEP_MAX_ANGLE) -> np.ndarray:
    """The joints that move the arm's tool0 by `step`, given in tool0 and turning about `pivot`.

    Args:
        urdf: The robot's URDF.
        arm: The arm's name, the prefix of its URDF joints and links, e.g. "ur_arm".
        joints: Joint angles now, radians, UR_JOINT_NAMES order.
        step: The move in tool0's frame: a shift, metres, and a turn.
        pivot: The point in tool0 the turn is about, metres, e.g. a tool's tip; it stays put if `step` only turns.
        max_shift: The largest shift taken, metres; a jog's per-tick step passes its own.
        max_turn: The largest turn taken, radians.

    Returns:
        np.ndarray: The joint angles, UR_JOINT_NAMES order.

    Raises:
        ValueError: If the step is larger than `max_shift` or `max_turn`, needs a joint to move more than
            TOOL_STEP_MAX_JOINT (near a singularity), or is not solved.
    """
    shift, turn = float(np.linalg.norm(step.position)), Rotation.from_quat(step.orientation).magnitude()
    if shift > max_shift or turn > max_turn:
        raise ValueError(f"a tool step of {shift * 1e3:.1f} mm and {np.degrees(turn):.1f}° is over the limit of "
                         f"{max_shift * 1e3:g} mm and {np.degrees(max_turn):g}°")

    def tool0(values: np.ndarray) -> np.ndarray:
        return tool0_pose(urdf, arm, values)

    start = np.asarray(joints, dtype=float)
    about = np.eye(4)
    about[:3, 3] = pivot
    target = tool0(start) @ about @ step.matrix() @ np.linalg.inv(about)
    here = start.copy()
    for _ in range(TOOL_STEP_ITERATIONS):
        now = tool0(here)
        error = _difference(target, now)
        if np.linalg.norm(error[:3]) <= TOOL_STEP_TOLERANCE and np.linalg.norm(error[3:]) <= TOOL_STEP_ANGLE_TOLERANCE:
            break
        jacobian = np.column_stack([_difference(tool0(here + _DELTA * unit), now) / _DELTA for unit in np.eye(6)])
        here = here + np.linalg.lstsq(jacobian, error, rcond=None)[0]
        moved = np.abs(here - start)
        if moved.max() > TOOL_STEP_MAX_JOINT:
            raise ValueError(f"the step needs {np.degrees(moved.max()):.0f}° on {UR_JOINT_NAMES[int(moved.argmax())]}: "
                             f"the arm is too near a singularity; move it away by free drive or joint jog")
    else:
        raise ValueError(f"the tool step was not solved in {TOOL_STEP_ITERATIONS} steps")
    return here


def tool0_pose(urdf: Path, arm: str, joints: Sequence[float]) -> np.ndarray:
    """Tool0 at `joints` (radians, UR_JOINT_NAMES order) as a 4x4 matrix in the URDF's root: a jog's anchor."""
    names = [f"{arm}_{name}" for name in UR_JOINT_NAMES]
    return link_pose(urdf, Pose(), dict(zip(names, np.asarray(joints, dtype=float))), f"{arm}_tool0").matrix()


def jog_offset(axis: int, amount: float) -> np.ndarray:
    """A move along one tool0 axis (`axis` 0-2, `amount` metres) or a turn about one (3-5, radians), as a 4x4."""
    offset = np.eye(4)
    if axis < 3:
        offset[axis, 3] = amount
    else:
        offset[:3, :3] = Rotation.from_rotvec(amount * np.eye(3)[axis - 3]).as_matrix()
    return offset


def tool_jog_target(urdf: Path, arm: str, joints: Sequence[float], anchor: np.ndarray, axis: int, speed: float,
                    lead: float = TOOL_JOG_LEAD, travelled_before: float = 0.0) -> tuple[np.ndarray, float]:
    """The joints to send next while a jog button is held: `speed * lead` beyond where tool0 is along the jog.

    The jog runs along a line (or a turn) from `anchor`, the press pose, in tool0's own axes: tool0's progress along
    it is measured from the live joints, so a lagging arm is never asked to jump and never drifts off the line.

    Args:
        urdf: The robot's URDF.
        arm: The arm's name, e.g. "ur_arm".
        joints: Joint angles now, radians, UR_JOINT_NAMES order.
        anchor: Tool0 at the press (`tool0_pose`), 4x4 in the URDF's root.
        axis: 0-2 moves along tool0's x, y, z; 3-5 turns about them.
        speed: Metres (or radians) per second, signed.
        lead: How far ahead the point lies, seconds.
        travelled_before: The progress returned last tick, so a turn past 180° keeps counting.

    Returns:
        tuple[np.ndarray, float]: The joint angles, and the progress along the jog now, metres or radians.

    Raises:
        ValueError: If the step is not solved, or needs a joint to move more than TOOL_STEP_MAX_JOINT.
    """
    now = tool0_pose(urdf, arm, joints)
    travelled = float(_difference(now, anchor, local=True)[axis])
    if axis >= 3:  # the rotation vector wraps at 180°; keep counting from last tick's progress
        travelled += 2.0 * np.pi * np.round((travelled_before - travelled) / (2.0 * np.pi))
    target = anchor @ jog_offset(axis, travelled + speed * lead)
    step = Pose.from_matrix(np.linalg.inv(now) @ target)
    # * Twice the per-tick travel, so a lagging arm's catch-up is not refused as a large step.
    bound = 2.0 * abs(speed) * lead + 1e-6
    return tool_step_target(urdf, arm, joints, step, max_shift=bound, max_turn=bound), travelled


def _difference(a: np.ndarray, b: np.ndarray, local: bool = False) -> np.ndarray:
    """How far pose `a` is from pose `b` (4x4 matrices): shift, then turn as a rotation vector.

    In the world's axes by default; with `local`, in `b`'s own axes (what `jog_offset` moves along).
    """
    if local:
        return np.concatenate([b[:3, :3].T @ (a[:3, 3] - b[:3, 3]),
                               Rotation.from_matrix(b[:3, :3].T @ a[:3, :3]).as_rotvec()])
    return np.concatenate([a[:3, 3] - b[:3, 3], Rotation.from_matrix(a[:3, :3] @ b[:3, :3].T).as_rotvec()])
