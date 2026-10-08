"""
Forward kinematics from URDF files: link poses at given joint values, for scenes without a live robot.

Each URDF is parsed once, without meshes (~2 ms). Needs yourdfpy, which the rest of the core does not.

! One thread per `ForwardKinematics`: a parsed model keeps the joint values it was last set to.
"""

from __future__ import annotations

from pathlib import Path
from typing import Dict, Mapping

from yourdfpy import URDF

from .geometry import Pose, compose


def load_urdf(urdf_file: Path) -> URDF:
    """Parse a URDF for kinematics only: no meshes are loaded.

    Args:
        urdf_file: The URDF.

    Returns:
        URDF: The parsed model, with a scene graph for link transforms.
    """
    return URDF.load(str(urdf_file), load_meshes=False, build_collision_scene_graph=False,
                     load_collision_meshes=False)


class ForwardKinematics:
    """Link poses of robots given by URDF, at any base pose and joint values. One thread only."""

    def __init__(self) -> None:
        """Start with nothing parsed; each URDF is parsed on first use."""
        # Resolved URDF path -> its parsed model.
        self._urdfs: Dict[Path, URDF] = {}

    def urdf(self, urdf_file: Path) -> URDF:
        """The parsed model of a URDF, parsed on first use. ! Its joint values are those last set."""
        path = Path(urdf_file).resolve()
        urdf = self._urdfs.get(path)
        if urdf is None:
            urdf = self._urdfs[path] = load_urdf(path)
        return urdf

    def link_pose(self, urdf_file: Path, base: Pose, joints: Mapping[str, float], link: str) -> Pose:
        """World pose of one link's frame.

        Args:
            urdf_file: The robot's URDF.
            base: World pose of the URDF's root link.
            joints: Values by actuated joint name; joints not given are 0, unknown names are ignored.
            link: A URDF link name, e.g. "left_ur_arm_tool0".

        Returns:
            Pose: The link frame in the world.

        Raises:
            KeyError: If the URDF has no such link.
        """
        urdf = self.urdf(urdf_file)
        if link not in urdf.link_map:
            raise KeyError(f"{Path(urdf_file).name} has no link {link!r}")
        urdf.update_cfg([float(joints.get(name, 0.0)) for name in urdf.actuated_joint_names])
        return compose(base, Pose.from_matrix(urdf.get_transform(frame_to=link, frame_from=urdf.base_link)))
