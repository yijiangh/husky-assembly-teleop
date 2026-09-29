"""
Poses between the world, the husky and each arm.

    world ─(mocap)─► base_footprint ─(URDF, fixed)─► base_link [HUSKY_FRAME]
          ─(URDF, fixed: mount calibration)─► <arm>_base_link ─(reported TCP)─► tool0

Every URDF step is a chain of fixed joints, so each is one constant transform,
read once at startup. `<arm>_base_link` is the arm controller's own base_link
(doc/ur_frames.md), so a pose in it is exactly what the controller takes.

A pose here is (position, rotation): a numpy array in metres and a scipy
Rotation. Quaternions elsewhere are (x, y, z, w).
"""

from __future__ import annotations

from pathlib import Path
from xml.etree.ElementTree import parse

import numpy as np
from scipy.spatial.transform import Rotation

#: The husky frame Cartesian targets are given in: the URDF's husky base_link.
HUSKY_FRAME = "base_link"
#: The frame mocap tracks (the relay applies base_mocap_from_base_footprint).
MOCAP_FRAME = "base_footprint"

Pose = tuple[np.ndarray, Rotation]


def compose(a: Pose, b: Pose) -> Pose:
    """`b` expressed in the frame `a` is expressed in (a then b, as in a chain)."""
    return a[0] + a[1].apply(b[0]), a[1] * b[1]


def invert(a: Pose) -> Pose:
    """The inverse transform."""
    inverse = a[1].inv()
    return -inverse.apply(a[0]), inverse


def fixed_transform(urdf_file: Path, parent: str, child: str) -> Pose:
    """The pose of link `child` in link `parent`, through fixed joints only.

    Args:
        urdf_file: The URDF.
        parent: Upper link.
        child: Lower link.

    Returns:
        Pose: `child` in `parent`.

    Raises:
        ValueError: If `child` is not below `parent`, or a movable joint lies
            between them (then there is no constant transform).
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
