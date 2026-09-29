"""
The UR frame convention, and a startup check that our URDFs keep it.

Why: doc/ur_frames.md.
"""

from __future__ import annotations

from pathlib import Path
from xml.etree.ElementTree import parse

import numpy as np
from scipy.spatial.transform import Rotation

#: Stock ur_description: both `<arm>_base_link -> <arm>_base_link_inertia` and
#: `<arm>_base_link -> <arm>_base` turn 180 deg about z.
STOCK_YAW = np.pi

#: The compliance controller's `base_link`, seen from the UR controller's Base
#: frame (the frame of the driver's reported TCP). Fixed by the stock
#: description the robot runs -- never taken from our URDFs.
BASE_LINK_FROM_UR_BASE = Rotation.from_euler("z", STOCK_YAW)


def stock_frame_problem(urdf_file: Path, arm_name: str) -> str | None:
    """Check that the two joints below an arm's base_link are the stock ones.

    Args:
        urdf_file: The robot's URDF.
        arm_name: The arm's prefix, e.g. "ur_arm".

    Returns:
        str | None: None if they are stock, otherwise what is wrong and where to fix it.
    """
    wrong = []
    for joint in parse(urdf_file).getroot().findall("joint"):
        if joint.find("parent").get("link") != f"{arm_name}_base_link":
            continue
        if joint.find("child").get("link") not in (f"{arm_name}_base_link_inertia", f"{arm_name}_base"):
            continue
        origin = joint.find("origin")
        xyz = np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
        turn = (Rotation.from_euler("z", STOCK_YAW).inv() * Rotation.from_euler("xyz", rpy)).magnitude()
        if np.linalg.norm(xyz) > 1e-6 or turn > 1e-6:
            wrong.append(f"{joint.get('name')} (rpy {origin.get('rpy')})")
    if not wrong:
        return None
    return (f"URDF {urdf_file.name}, arm {arm_name}: joints below {arm_name}_base_link are not the stock UR "
            f"ones (xyz 0 0 0, rpy 0 0 pi): {', '.join(wrong)}. So {arm_name}_base_link is not the robot "
            f"controller's base_link. The arm's mounting belongs in the joint above it. "
            f"Fix with scripts/fix_ur_base_frames.py; see doc/ur_frames.md.")
