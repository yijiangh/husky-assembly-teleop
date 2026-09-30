"""
Each robot's base pose, joint values and link poses, fixed once per tick.

    tick:  pump ROS → kinematics.update → take_snapshot → plugins → draw

* One per monitor, updated once per tick right after the ROS pump and before the
  world copy, so every reader in the tick sees the same robot poses.
* The only place outside the 3D view that parses URDFs with yourdfpy. It has its
  own parsed models (no meshes), separate from the 3D view's, so neither changes
  the other's joint state.

! Main thread only.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Mapping

from yourdfpy import URDF

from .scene import Pose, compose
from ..tool_urdfs import resolve_mesh_path

if TYPE_CHECKING:
    from ..config import RobotConfig
    from .measured import WorldState


@dataclass(eq=False)
class _Robot:
    """One robot's parsed model and its latest measured values.

    Attributes:
        urdf: Parsed model without meshes; its link transforms follow `joints`.
        base: World pose of the URDF's root link.
        joints: Value per actuated joint, in the URDF's order.
        unmeasured: Joints never measured; their value in `joints` is 0.
        link_poses: World poses already asked for this tick, by link. Emptied by `update`.
    """

    urdf: URDF
    base: Pose
    joints: dict[str, float]
    unmeasured: frozenset[str]
    link_poses: dict[str, Pose] = field(default_factory=dict)


class Kinematics:
    """Each robot's base pose, joint values and link poses, fixed once per tick. Main thread only."""

    def __init__(self, robots: tuple[RobotConfig, ...], log_warn: Callable[[str], None]):
        """Parse every robot's URDF once and start each at its default pose, joints at 0.

        Args:
            robots: The robots' configurations; `urdf_file` is the stitched URDF.
            log_warn: Reports a robot whose measurements do not match its URDF.
        """
        self._log_warn = log_warn
        self._robots: dict[str, _Robot] = {}
        # (serial, joint name) pairs already warned about, so each is reported once.
        self._unknown_joints: set[tuple[str, str]] = set()
        for config in robots:
            urdf = _load_urdf(config)
            names = urdf.actuated_joint_names
            self._robots[config.serial] = _Robot(
                urdf=urdf, base=Pose.from_arrays(config.default_position, config.default_orientation),
                joints=dict.fromkeys(names, 0.0), unmeasured=frozenset(names))
            urdf.update_cfg([0.0] * len(names))

    def update(self, world: WorldState) -> None:
        """Take every robot's latest measurements and recompute its link transforms.

        ! A missing measurement keeps the last value, not zero: an untracked base
          keeps its last pose, and an absent joint keeps its last value.

        Args:
            world: Measured state; robots it doesn't have keep their last values.
        """
        for serial, robot in self._robots.items():
            interface = world.robots.get(serial)
            if interface is None:
                continue
            state = interface.state
            if state.base.tracked:
                robot.base = Pose.from_arrays(state.base.position, state.base.orientation)
            measured = []
            for name, value in state.joint_positions.items():
                if name in robot.joints:
                    robot.joints[name] = float(value)
                    measured.append(name)
                elif (serial, name) not in self._unknown_joints:
                    self._unknown_joints.add((serial, name))
                    self._log_warn(f"robot {serial} reports joint {name!r}, which its URDF "
                                   f"does not have; it will not move in the scene")
            if measured:
                robot.unmeasured = robot.unmeasured.difference(measured)
            # ? `joints` keeps the URDF's order, so its values line up with actuated_joint_names.
            robot.urdf.update_cfg(list(robot.joints.values()))
            robot.link_poses.clear()

    def base_pose(self, serial: str) -> Pose:
        """World pose of a robot's URDF root link (at base_footprint): last mocap fix, or its default pose.

        Raises:
            KeyError: If no robot has that serial.
        """
        return self._robot(serial).base

    def joints(self, serial: str) -> Mapping[str, float]:
        """Value per actuated joint. ! The live dict: copy it to keep it past this tick.

        Raises:
            KeyError: If no robot has that serial.
        """
        return self._robot(serial).joints

    def unmeasured(self, serial: str) -> frozenset[str]:
        """Actuated joints never measured; their value is 0.

        Raises:
            KeyError: If no robot has that serial.
        """
        return self._robot(serial).unmeasured

    def joint_names(self, serial: str) -> tuple[str, ...]:
        """Actuated joint names, in the URDF's order.

        Raises:
            KeyError: If no robot has that serial.
        """
        return tuple(self._robot(serial).joints)

    def link_names(self, serial: str) -> tuple[str, ...]:
        """Every link name of the robot's URDF.

        Raises:
            KeyError: If no robot has that serial.
        """
        return tuple(self._robot(serial).urdf.link_map)

    def link_pose(self, serial: str, link: str) -> Pose:
        """World pose of one link's frame, from this tick's base pose and joint values.

        ? Computed on the first call after `update` (about 70 us), then looked up.

        Args:
            serial: The robot.
            link: A URDF link name, e.g. "arm_0_tool0".

        Returns:
            Pose: The link frame in the world.

        Raises:
            KeyError: If no robot has that serial, or its URDF has no such link.
        """
        robot = self._robot(serial)
        pose = robot.link_poses.get(link)
        if pose is None:
            if link not in robot.urdf.link_map:
                raise KeyError(f"robot {serial} has no link {link!r} in its URDF")
            in_base = robot.urdf.get_transform(frame_to=link, frame_from=robot.urdf.base_link)
            pose = robot.link_poses[link] = compose(robot.base, Pose.from_matrix(in_base))
        return pose

    def _robot(self, serial: str) -> _Robot:
        """Look up one robot, with a clear error for an unknown serial."""
        robot = self._robots.get(serial)
        if robot is None:
            raise KeyError(f"no robot {serial!r} in kinematics; known: {sorted(self._robots)}")
        return robot


def _load_urdf(config: RobotConfig) -> URDF:
    """Parse a robot's URDF for kinematics only: no meshes are loaded.

    Args:
        config: The robot; its `urdf_file` is parsed.

    Returns:
        URDF: The parsed model, with a scene graph for link transforms.
    """

    def resolve(fname: str) -> str:
        """Turn one mesh reference into an absolute path.

        ! Keep the name `fname`: yourdfpy passes it as a keyword.
        """
        return resolve_mesh_path(fname, config.urdf_file)

    return URDF.load(str(config.urdf_file), filename_handler=resolve, load_meshes=False,
                     build_collision_scene_graph=False, load_collision_meshes=False)
