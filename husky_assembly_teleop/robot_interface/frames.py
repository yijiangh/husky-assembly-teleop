"""
Poses between the world, the husky and each arm.

    world ─(mocap)─► base_footprint ─(URDF, fixed)─► base_link [HUSKY_FRAME]
          ─(URDF, fixed: mount calibration)─► <arm>_base_link ─(reported TCP)─► tool0

URDF steps are fixed joints, read once at startup. `<arm>_base_link` is the arm controller's own
base_link (doc/ur_frames.md). Quaternions are (x, y, z, w).
"""

from __future__ import annotations

from pathlib import Path
from xml.etree.ElementTree import Element, parse

from scipy.spatial.transform import Rotation

from bar_assembly_core.design_io.pose import Pose, compose

#: Frame of husky Cartesian targets (the URDF's husky base_link).
HUSKY_FRAME = "base_link"
#: Frame mocap tracks.
MOCAP_FRAME = "base_footprint"


def joint_origin(joint: Element) -> tuple[list[float], Rotation]:
    """Return a URDF joint's `<origin>` as (xyz, rotation); a missing origin or attribute is zero."""
    origin = joint.find("origin")
    xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
    return [float(v) for v in xyz.split()], Rotation.from_euler("xyz", [float(v) for v in rpy.split()])


def fixed_transform(urdf_file: Path, parent: str, child: str) -> Pose:
    """Return the pose of link `child` in link `parent`, through fixed joints only.

    Raises:
        ValueError: If `child` is not below `parent`, or a movable joint lies between them.
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
        xyz, rotation = joint_origin(joint)
        pose = compose(Pose.from_arrays(xyz, rotation.as_quat()), pose)
        link = joint.find("parent").get("link")
    return pose
