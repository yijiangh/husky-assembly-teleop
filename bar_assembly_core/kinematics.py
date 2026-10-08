"""
Forward kinematics from URDF files, in numpy: link poses at given joint values, for scenes without a live robot.

Each URDF is parsed once per path. Revolute, continuous and prismatic joints move, mimic joints follow their source;
any other type counts as fixed. Safe to call from several threads: nothing parsed is ever changed.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple
from xml.etree.ElementTree import parse

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import Pose, compose

#: Joint types that move: rotate about the axis, or slide along it.
_ROTATING, _SLIDING = ("revolute", "continuous"), ("prismatic",)


@dataclass(frozen=True)
class _Joint:
    """One URDF joint: its parent link, the child's origin in the parent frame, and how it moves."""

    name: str
    parent: str
    origin: np.ndarray
    kind: str
    axis: np.ndarray
    # (source joint, multiplier, offset), or None.
    mimic: Optional[Tuple[str, float, float]]


def _origin(element) -> np.ndarray:
    """The 4x4 transform of an `<origin xyz rpy>` element; identity if absent."""
    matrix = np.eye(4)
    if element is not None:
        xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
        # ? URDF rpy: roll, pitch, yaw about the fixed X, Y, Z axes, which is scipy's extrinsic "xyz".
        matrix[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
        matrix[:3, 3] = xyz
    return matrix


@lru_cache(maxsize=32)
def _tree(urdf_file: Path) -> Dict[str, _Joint]:
    """Child link -> the joint above it, for every joint of a URDF."""
    joints = {}
    for element in parse(str(urdf_file)).getroot().findall("joint"):
        axis = element.find("axis")
        mimic = element.find("mimic")
        joints[element.find("child").get("link")] = _Joint(
            name=element.get("name"), parent=element.find("parent").get("link"),
            origin=_origin(element.find("origin")), kind=element.get("type"),
            axis=np.array([float(v) for v in (axis.get("xyz") if axis is not None else "1 0 0").split()]),
            mimic=None if mimic is None else (mimic.get("joint"), float(mimic.get("multiplier", 1.0)),
                                              float(mimic.get("offset", 0.0))))
    return joints


@lru_cache(maxsize=32)
def _links(urdf_file: Path) -> frozenset:
    """Every link name of a URDF."""
    return frozenset(link.get("name") for link in parse(str(urdf_file)).getroot().findall("link"))


def _motion(joint: _Joint, joints: Mapping[str, float]) -> np.ndarray:
    """The 4x4 transform a joint adds at the given joint values (identity for a fixed joint)."""
    matrix = np.eye(4)
    if joint.kind not in _ROTATING + _SLIDING:
        return matrix
    if joint.mimic is not None:
        source, multiplier, offset = joint.mimic
        value = multiplier * float(joints.get(source, 0.0)) + offset
    else:
        value = float(joints.get(joint.name, 0.0))
    axis = joint.axis / np.linalg.norm(joint.axis)
    if joint.kind in _ROTATING:
        matrix[:3, :3] = Rotation.from_rotvec(axis * value).as_matrix()
    else:
        matrix[:3, 3] = axis * value
    return matrix


def link_pose(urdf_file: Path, base: Pose, joints: Mapping[str, float], link: str) -> Pose:
    """World pose of one link's frame.

    Args:
        urdf_file: The robot's URDF.
        base: World pose of the URDF's root link.
        joints: Values by joint name; joints not given are 0, unknown names are ignored.
        link: A URDF link name, e.g. "left_ur_arm_tool0".

    Returns:
        Pose: The link frame in the world.

    Raises:
        KeyError: If the URDF has no such link.
    """
    path = Path(urdf_file).resolve()
    if link not in _links(path):
        raise KeyError(f"{path.name} has no link {link!r}")
    tree = _tree(path)
    matrix = np.eye(4)
    while link in tree:
        joint = tree[link]
        matrix = joint.origin @ _motion(joint, joints) @ matrix
        link = joint.parent
    return compose(base, Pose.from_matrix(matrix))
