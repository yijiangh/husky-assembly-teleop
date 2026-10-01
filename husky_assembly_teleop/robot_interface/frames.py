"""
Poses between the world, the husky and each arm.

    world ─(mocap)─► base_footprint ─(URDF, fixed)─► base_link [HUSKY_FRAME]
          ─(URDF, fixed: mount calibration)─► <arm>_base_link ─(reported TCP)─► tool0

Each URDF step is a constant transform (fixed joints only), read once at startup.
`<arm>_base_link` is the arm controller's own base_link (doc/ur_frames.md).

Poses are `design_io.pose.Pose`, the one pose type of the package; quaternions are
(x, y, z, w).
"""

from __future__ import annotations

from pathlib import Path
from xml.etree.ElementTree import parse

from scipy.spatial.transform import Rotation

from ..design_io.pose import Pose, compose, invert  # noqa: F401  (re-exported for robot and arm)

#: Frame of husky Cartesian targets (the URDF's husky base_link).
HUSKY_FRAME = "base_link"
#: Frame mocap tracks.
MOCAP_FRAME = "base_footprint"

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
    pose = Pose()
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
        pose = compose(Pose.from_arrays(xyz, Rotation.from_euler("xyz", rpy).as_quat()), pose)
        link = joint.find("parent").get("link")
    return pose
