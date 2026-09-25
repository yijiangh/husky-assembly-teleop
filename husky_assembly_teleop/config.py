"""
Everything about configuring one run: the shape of the configuration, which
URDF each robot runs, and how a run is read off the command line. Resolved once
at startup, then read-only.

Nothing imports a global; everything receives a config. Why that replaced the
old module-level constants: doc/refactor_rationale.md.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

from rclpy.node import Node


@dataclass(frozen=True)
class RobotConfig:
    """Everything needed to talk to, and draw, one physical robot.

    ! The kinematic configuration comes from the URDF and only from the URDF.
      No `dual_arm` / `ee_types` / `connect_gripper` flags: mount a different
      tool, ship a different URDF.

    Attributes:
        serial: Clearpath serial such as "a200-0806". Identifies the robot in
            crl_husky's config_resolver and in mocap calibration files.
        ros_namespace: ROS2 namespace the robot publishes under, e.g. "a200_0806".
        urdf_file: Absolute path to the URDF describing this robot as built,
            including whatever end effectors are currently mounted.
        default_position: Where the robot stands before mocap has said
            otherwise, as (x, y, z) in metres. Set by `row_layout_position` so
            several robots do not all sit on top of each other at the origin.
        default_yaw: Which way it faces at that pose, radians about Z. Ground
            robots only ever rotate about Z, so one angle is the whole story.
        calibration_file: Joint-origin overlay applied on top of the URDF, or
            None to use it as shipped. Nothing reads it yet.

            ! Unresolved, and worth settling before this is wired up.
              The URDFs the monitor currently loads already have calibration
              baked into their filenames (Alice_Calibrated, All_Calibrated --
              see `_CALIBRATED_URDF_BY_SERIAL` below), which is the arrangement this
              overlay was meant to replace. Both cannot survive: either the
              overlay happens and those collapse to one URDF per robot type, or
              this field comes out. An overlay is a delta on one joint's origin
              xyz/rpy, applied after parsing -- reviewable in git and
              independent of which tool is mounted. Do not diff two URDFs
              against each other; doc/refactor_rationale.md says why that fails.
    """

    serial: str
    ros_namespace: str
    urdf_file: Path
    default_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    default_yaw: float = 0.0
    calibration_file: Path | None = None

    @property
    def default_orientation(self) -> tuple[float, float, float, float]:
        """tuple[float, float, float, float]: `default_yaw` as an (x, y, z, w) quaternion.

        xyzw is what PyBullet, ROS and RobotState all use. Viser wants wxyz, and
        `visualization.quaternion_to_wxyz` does that conversion at the boundary.
        """
        half = self.default_yaw / 2.0
        return (0.0, 0.0, math.sin(half), math.cos(half))


@dataclass(frozen=True)
class MonitorConfig:
    """Everything the monitor and its plugins need to know about this run.

    Attributes:
        robots: The robots to connect to, in display order.
        data_directory: Root for meshes, URDFs and design data.
        tick_period: Seconds between ticks. 0.05 (20 Hz) matches the old monitor.
        viser_port: Port the viser web UI listens on.
        enabled_plugins: Plugins to load, listed explicitly so a new file under
            plugins/ cannot enable itself everywhere it is installed. Empty runs
            the core with no plugins, which is a useful thing to ask for.
        max_plugin_errors: Consecutive ticks a plugin may raise in before it is
            torn down instead of filling the log forever.
        slow_step_warn_ratio: Warn when one plugin's step eats more than this
            fraction of the tick budget.
        slow_step_warn_period: Seconds between repeats of one plugin's slow-step
            warning, so a plugin that is slow every tick cannot bury the log.
        max_cleanup_steps: Extra steps a cancelled job gets for its cleanup
            before its plugin's UI is taken away. Bounded, because cleanup that
            never finishes must not block a shutdown.
    """

    robots: tuple[RobotConfig, ...]
    data_directory: Path
    tick_period: float = 0.05
    viser_port: int = 8080
    enabled_plugins: tuple[str, ...] = ()
    max_plugin_errors: int = 3
    slow_step_warn_ratio: float = 0.5
    slow_step_warn_period: float = 5.0
    max_cleanup_steps: int = 10


# --- --- --- --- --- WHERE ROBOTS STAND BEFORE MOCAP --- --- --- --- ---
#: Metres between neighbouring robots in the startup layout. A husky is about a
#: metre long and its arms reach a further 0.85 m, so two metres keeps a fleet
#: clear of itself while still fitting on screen.
ROBOT_LAYOUT_SPACING = 2.0


def row_layout_position(index: int, count: int,
                        spacing: float = ROBOT_LAYOUT_SPACING) -> tuple[float, float, float]:
    """Lay `count` robots out in a row along Y, centred on the origin.

    ? Why a default pose is needed at all.
      Until mocap produces a fix a robot's pose is not tracked, and both the
      PyBullet scene and the viser scene deliberately leave an untracked robot
      where it was rather than snapping it to the origin. Without a starting
      pose "where it was" is the origin for every robot, so a fleet loads as one
      pile of overlapping meshes.

    Centred rather than starting at the origin, so a single robot still lands at
    (0, 0, 0) and the common case looks exactly as it did before.

    Args:
        index: Which robot this is, from 0.
        count: How many robots are being placed.
        spacing: Metres between neighbours.

    Returns:
        tuple[float, float, float]: Position as (x, y, z), on the floor.

    Example:
        >>> [row_layout_position(i, 3)[1] for i in range(3)]
        [-2.0, 0.0, 2.0]
    """
    return (0.0, (index - (count - 1) / 2.0) * spacing, 0.0)


# --- --- --- --- --- WHICH URDF EACH ROBOT RUNS --- --- --- --- ---
# TODO temporary. Which URDF a robot uses should come from the robot, not from a
#      table here -- crl_husky is the natural home, but it ships no robot URDF
#      today (only a gripper xacro) and its config_resolver exposes only mocap
#      calibration, so there is nothing to look one up in yet. Until that exists,
#      this reproduces the mapping the old code had, whose sources were
#      old/cfab_session.py:44-56 (the calibrated per-robot files) and
#      old/husky_world.py:189-223 (the domain-id table that decided dual-arm).
#
# ! These are the *calibrated* files, and which one a robot gets is not
#   cosmetic: the arm kinematics differ per machine. Alice and Belle are the
#   single-arm rigs, Cindy is dual-arm.
_URDF_ROOT = "husky_urdf"
_CALIBRATED_URDF_BY_SERIAL = {
    "0804": f"{_URDF_ROOT}/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Alice_Calibrated.urdf",
    "0805": f"{_URDF_ROOT}/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Belle_Calibrated.urdf",
    "0806": f"{_URDF_ROOT}/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf",
}


def robot_config_from_serial(token: str, data_directory: Path,
                             default_position: tuple[float, float, float] = (0.0, 0.0, 0.0),
                             default_yaw: float = 0.0) -> RobotConfig:
    """Build one RobotConfig from a serial written any of the usual ways.

    Accepts "0806", "a200-0806", "a200_0806" or "/a200_0806" and derives the
    canonical serial and ROS namespace from the four digits, matching the layout
    of crl_husky's config/robots/<digits>/robot.yaml. The URDF is the calibrated
    file for that robot -- see the note on `_CALIBRATED_URDF_BY_SERIAL`.

    Args:
        token: The serial as typed on the command line.
        data_directory: Root the URDF path is resolved against.
        default_position: Where to stand it until mocap says otherwise, usually
            from `row_layout_position`.
        default_yaw: Which way it faces there, radians about Z.

    Returns:
        RobotConfig: Identity and URDF for that robot.

    Raises:
        ValueError: If no four-digit serial can be read out of `token`, or that
            serial has no known URDF.
        FileNotFoundError: If the URDF it maps to is not on disk, which usually
            means `data_directory` is wrong.
    """
    digits = re.search(r"(\d{4})\s*$", token.strip().strip("/"))
    if digits is None:
        raise ValueError(f"cannot read a four-digit robot serial out of {token!r}; "
                         f"expected something like '0806' or 'a200-0806'")
    number = digits.group(1)

    relative = _CALIBRATED_URDF_BY_SERIAL.get(number)
    if relative is None:
        known = ", ".join(sorted(_CALIBRATED_URDF_BY_SERIAL))
        raise ValueError(f"no URDF known for robot {number!r}; known robots are {known}")

    urdf_file = data_directory / relative
    if not urdf_file.is_file():
        raise FileNotFoundError(f"URDF for robot {number!r} not found at {urdf_file}; "
                                f"check the data_directory parameter")

    return RobotConfig(
        serial=f"a200-{number}",
        ros_namespace=f"a200_{number}",
        urdf_file=urdf_file,
        default_position=default_position,
        default_yaw=default_yaw,
    )


# --- --- --- --- --- READING ONE RUN OFF THE COMMAND LINE --- --- --- --- ---
def config_from_ros_parameters(node: Node) -> MonitorConfig:
    """Build the frozen run configuration from `node`'s ROS2 parameters.

    Keeping configuration in ROS parameters means a run can be described by a
    launch file or a yaml.

    Typical invocation, picking robots by serial:

        ros2 run husky_assembly_teleop husky_monitor --ros-args \\
            -p robots:="['0804','0806']" \\
            -p plugins:="['cell']"
    """
    node.declare_parameter("robots", [""])
    node.declare_parameter("plugins", [""])
    node.declare_parameter("data_directory", "")

    def string_list(name: str) -> tuple[str, ...]:
        """Read a string-array parameter, dropping the empty-string default."""
        raw = node.get_parameter(name).get_parameter_value().string_array_value
        return tuple(item for item in raw if item)

    data_text = node.get_parameter("data_directory").get_parameter_value().string_value.strip()
    data_directory = (Path(data_text).expanduser() if data_text
                      else Path(__file__).resolve().parent.parent / "data")

    return MonitorConfig(
        robots=_robots_in_a_row(string_list("robots"), data_directory),
        data_directory=data_directory,
        enabled_plugins=string_list("plugins"),
    )


def _robots_in_a_row(serials: tuple[str, ...], data_directory: Path) -> tuple[RobotConfig, ...]:
    """Build a RobotConfig per serial, spaced out along Y in the order given.

    Args:
        serials: Robot serials as typed on the command line.
        data_directory: Root the URDF paths are resolved against.

    Returns:
        tuple[RobotConfig, ...]: One per serial, in the same order.
    """
    return tuple(
        robot_config_from_serial(serial, data_directory,
                                 default_position=row_layout_position(index, len(serials)))
        for index, serial in enumerate(serials)
    )
