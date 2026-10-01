"""
Poses and ids: the vocabulary shared by design files and the monitor's scene.

! No imports from outside `design_io`: this package moves to its own repository.
"""

from __future__ import annotations

import re
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

#: What an id may contain: path segments of letters, digits, `_`, `.` and `-`, joined by `/`.
ID_PATTERN = re.compile(r"[A-Za-z0-9_.\-]+(/[A-Za-z0-9_.\-]+)*")


# --- --- --- --- --- POSES --- --- --- --- ---

@dataclass(frozen=True)
class Pose:
    """A pose in the world, or in a parent frame.

    Attributes:
        position: (x, y, z), metres.
        orientation: Quaternion (x, y, z, w), the ROS and PyBullet order.
    """

    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    @classmethod
    def from_arrays(cls, position, orientation) -> Pose:
        """Build a pose from any sequences (numpy arrays included).

        Args:
            position: (x, y, z), metres.
            orientation: Quaternion (x, y, z, w).

        Returns:
            Pose: The same pose, as plain floats.
        """
        return cls(tuple(float(v) for v in position), tuple(float(v) for v in orientation))

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> Pose:
        """Build a pose from a 4x4 homogeneous transform.

        Args:
            matrix: 4x4 transform.

        Returns:
            Pose: The same pose.
        """
        # ! Copy: yourdfpy hands out read-only matrices, which this scipy version refuses.
        return cls.from_arrays(matrix[:3, 3], Rotation.from_matrix(np.array(matrix[:3, :3])).as_quat())

    def matrix(self) -> np.ndarray:
        """Return this pose as a 4x4 homogeneous transform."""
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat(self.orientation).as_matrix()
        matrix[:3, 3] = self.position
        return matrix


def compose(a: Pose, b: Pose) -> Pose:
    """Chain two poses: `b` is given in the frame of `a`; the result is in the frame `a` is in.

    Args:
        a: The parent frame's pose.
        b: The pose inside that frame.

    Returns:
        Pose: `b` in the frame `a` is given in.
    """
    rotation = Rotation.from_quat(a.orientation)
    return Pose.from_arrays(np.asarray(a.position) + rotation.apply(b.position),
                            (rotation * Rotation.from_quat(b.orientation)).as_quat())


def invert(a: Pose) -> Pose:
    """The inverse pose: where the parent frame is, seen from inside `a`."""
    inverse = Rotation.from_quat(a.orientation).inv()
    return Pose.from_arrays(-inverse.apply(a.position), inverse.as_quat())


# --- --- --- --- --- IDS --- --- --- --- ---

def check_id(body_id: str) -> None:
    """Refuse an id with characters outside ID_PATTERN.

    Raises:
        ValueError: If the id is not a valid path.
    """
    if not ID_PATTERN.fullmatch(body_id):
        raise ValueError(f"invalid id {body_id!r}: use letters, digits, '_', '.', '-' and '/' only")
