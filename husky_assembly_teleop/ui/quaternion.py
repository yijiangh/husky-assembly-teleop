"""
Quaternion order at the viser boundary: everything else (ROS, PyBullet, RobotState,
scene poses) is (x, y, z, w); viser alone wants (w, x, y, z).

! A leaf module, importing nothing of ours, so every ui module can use it.
"""

from __future__ import annotations

from typing import Iterable


def quaternion_to_wxyz(quaternion: Iterable[float]) -> tuple[float, float, float, float]:
    """Reorder a quaternion from xyzw (ROS, PyBullet, RobotState) to viser's wxyz.

    ! A wrong order still renders, just rotated, so it is easy to miss.

    Args:
        quaternion: Orientation as (x, y, z, w).

    Returns:
        tuple[float, float, float, float]: The same rotation as (w, x, y, z).
    """
    x, y, z, w = (float(value) for value in quaternion)
    return (w, x, y, z)
