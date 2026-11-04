"""
The tip offset from tool0 poses that all touch one point (a pivot calibration, as the pendant's TCP wizard does).

Each pose (R, p) puts the tip t at the same point c: R t + p = c, solved for t and c by least squares. The tilts must
be about at least two axes: along an axis all poses turn about, t cannot be told.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose

#: Poses needed to solve: two leave the tip unknown along the axis between their turns.
PIVOT_MIN_POSES = 3


@dataclass(frozen=True)
class Pivot:
    """A solved tip offset.

    Attributes:
        offset: The tip in tool0, metres.
        point: The point every pose touched, in the frame the poses are in.
        residuals: Each pose's tip distance from `point`, metres.
        tilt: The poses' RMS turn from their mean rotation about the axis they turn least about, radians: about a
            third of how far they are turned apart about it. Near zero leaves the offset unknown along it.
    """

    offset: np.ndarray
    point: np.ndarray
    residuals: np.ndarray
    tilt: float


def pivot_fit(tool0s: Sequence[Pose]) -> Pivot:
    """Solve the tip offset from tool0 poses that all touch one point.

    Args:
        tool0s: Tool0 in one fixed frame, e.g. the robot's, at each touch.

    Raises:
        ValueError: With fewer than PIVOT_MIN_POSES poses.
    """
    if len(tool0s) < PIVOT_MIN_POSES:
        raise ValueError(f"{len(tool0s)} poses: a tip offset needs {PIVOT_MIN_POSES}")
    rotations = np.array([Rotation.from_quat(pose.orientation).as_matrix() for pose in tool0s])
    positions = np.array([pose.position for pose in tool0s])
    system = np.concatenate([np.concatenate([rotation, -np.eye(3)], axis=1) for rotation in rotations])
    solution = np.linalg.lstsq(system, -positions.reshape(-1), rcond=None)[0]
    offset, point = solution[:3], solution[3:]
    residuals = np.linalg.norm(np.einsum("nij,j->ni", rotations, offset) + positions - point, axis=1)
    # * (R_i - mean R) v is about pose i's turn from the mean about v, times |v|: its smallest RMS gain is the tilt.
    spread = rotations - rotations.mean(axis=0)
    gain = np.linalg.eigvalsh(np.einsum("nij,nik->jk", spread, spread) / len(tool0s))[0]
    return Pivot(offset, point, residuals, float(np.sqrt(max(gain, 0.0))))
