"""
Poses between the world, the husky and each arm.

    world ─(mocap)─► base_footprint ─(URDF, fixed)─► base_link [HUSKY_FRAME]
          ─(URDF, fixed: mount calibration)─► <arm>_base_link ─(reported TCP)─► tool0

Each URDF step is a constant transform (fixed joints only), read once at startup.
`<arm>_base_link` is the arm controller's own base_link (doc/ur_frames.md).

A pose is (position in metres as a numpy array, scipy Rotation). Quaternions
elsewhere are (x, y, z, w).
"""

from __future__ import annotations

from pathlib import Path
from xml.etree.ElementTree import parse

import numpy as np
from scipy.spatial.transform import Rotation

#: Frame of husky Cartesian targets (the URDF's husky base_link).
HUSKY_FRAME = "base_link"
#: Frame mocap tracks.
MOCAP_FRAME = "base_footprint"

Pose = tuple[np.ndarray, Rotation]


def compose(a: Pose, b: Pose) -> Pose:
    """Chain two poses: `b` given in the frame of `a`, returned in the frame `a` is in."""
    return a[0] + a[1].apply(b[0]), a[1] * b[1]


def invert(a: Pose) -> Pose:
    """Return the inverse pose."""
    inverse = a[1].inv()
    return -inverse.apply(a[0]), inverse


def fixed_transform(urdf_file: Path, parent: str, child: str) -> Pose:
    """Return the pose of link `child` in link `parent`, through fixed joints only.

    Args:
        urdf_file: The URDF.
        parent: Upper link.
        child: Lower link.

    Returns:
        Pose: `child` in `parent`.

    Raises:
        ValueError: If `child` is not below `parent`, or a movable joint lies
            between them.
    """
    by_child = {j.find("child").get("link"): j for j in parse(urdf_file).getroot().findall("joint")}
    pose: Pose = (np.zeros(3), Rotation.identity())
    link = child
    while link != parent:
        joint = by_child.get(link)
        if joint is None:
            raise ValueError(f"{urdf_file.name}: {child} is not below {parent}")
        if joint.get("type") != "fixed":
            raise ValueError(f"{urdf_file.name}: joint {joint.get('name')} between {parent} and {child} moves")
        origin = joint.find("origin")
        xyz = [float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split()]
        rpy = [float(v) for v in (origin.get("rpy", "0 0 0") if origin is not None else "0 0 0").split()]
        pose = compose((np.array(xyz), Rotation.from_euler("xyz", rpy)), pose)
        link = joint.find("parent").get("link")
    return pose
