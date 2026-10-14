"""
Poses between the world, the husky and each arm.

    world ─(mocap)─► base_footprint ─(URDF, fixed)─► base_link [HUSKY_FRAME]
          ─(URDF, fixed: mount calibration)─► <arm>_base_link ─(reported TCP)─► tool0

URDF steps are fixed joints, read once at startup. `<arm>_base_link` is the arm controller's own
base_link (doc/ur_frames.md). Quaternions are (x, y, z, w).
"""

from __future__ import annotations

from pathlib import Path

from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.urdf import load_urdf

#: Frame of husky Cartesian targets (the URDF's husky base_link).
HUSKY_FRAME = "base_link"
#: Frame mocap tracks.
MOCAP_FRAME = "base_footprint"


def fixed_transform(urdf_file: Path, parent: str, child: str) -> Pose:
    """Return the pose of link `child` in link `parent`, through fixed joints only.

    Raises:
        ValueError: If `child` is not below `parent`, or a movable joint lies between them.
    """
    by_child = {joint.child: joint for joint in load_urdf(urdf_file).robot.joints}
    pose = Pose()
    link = child
    while link != parent:
        joint = by_child.get(link)
        if joint is None:
            raise ValueError(f"{urdf_file.name}: {child} is not below {parent}")
        if joint.type != "fixed":
            raise ValueError(f"{urdf_file.name}: joint {joint.name} between {parent} and {child} moves")
        # ? A joint without <origin> has none in yourdfpy: its child sits on its parent.
        if joint.origin is not None:
            pose = compose(Pose.from_matrix(joint.origin), pose)
        link = joint.parent
    return pose
