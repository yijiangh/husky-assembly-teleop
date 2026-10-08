"""
Conventions of the UR arms every user of the core shares.

Joint names, the arm links a mounted tool touches, and the check that a URDF keeps the stock UR base frames
(doc/ur_frames.md).
"""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Tuple
from xml.etree.ElementTree import Element, parse

import numpy as np
from scipy.spatial.transform import Rotation

#: Joint names in the UR driver's order. The URDF has the same names with the arm's prefix ("left_ur_arm_").
UR_JOINT_NAMES = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")

#: Links of its arm ("<arm>_<suffix>") a mounted tool may touch. The SRDFs don't list these pairs,
#: so collision checkers add them themselves.
TOOL_TOUCHES_ARM_LINKS = ("wrist_2_link", "wrist_3_link", "flange", "tool0")

#: Stock ur_description turns both joints below `<arm>_base_link` 180 deg about z.
STOCK_YAW = np.pi


def joint_origin(joint: Element) -> Tuple[List[float], Rotation]:
    """Return a URDF joint's `<origin>` as (xyz, rotation); a missing origin or attribute is zero."""
    origin = joint.find("origin")
    xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
    return [float(v) for v in xyz.split()], Rotation.from_euler("xyz", [float(v) for v in rpy.split()])


def stock_frame_problem(urdf_file: Path, arm_name: str) -> Optional[str]:
    """Check the two joints below an arm's base_link are the stock ones.

    Args:
        urdf_file: The robot's URDF.
        arm_name: The arm's prefix, e.g. "ur_arm".

    Returns:
        str | None: None if stock, otherwise what is wrong and how to fix it.
    """
    wrong = []
    for joint in parse(urdf_file).getroot().findall("joint"):
        if joint.find("parent").get("link") != f"{arm_name}_base_link":
            continue
        if joint.find("child").get("link") not in (f"{arm_name}_base_link_inertia", f"{arm_name}_base"):
            continue
        xyz, rotation = joint_origin(joint)
        turn = (Rotation.from_euler("z", STOCK_YAW).inv() * rotation).magnitude()
        if np.linalg.norm(xyz) > 1e-6 or turn > 1e-6:
            wrong.append(f"{joint.get('name')} (rpy {joint.find('origin').get('rpy')})")
    if not wrong:
        return None
    return (f"URDF {Path(urdf_file).name}, arm {arm_name}: joints below {arm_name}_base_link are not the stock UR "
            f"ones (xyz 0 0 0, rpy 0 0 pi): {', '.join(wrong)}. So {arm_name}_base_link is not the robot "
            f"controller's base_link. The arm's mounting belongs in the joint above it. "
            f"Fix with scripts/fix_ur_base_frames.py; see doc/ur_frames.md.")
