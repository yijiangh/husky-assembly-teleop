"""
Run configuration: its dataclasses, the URDF for each robot, and reading it from
ROS parameters. Resolved once at startup, then read-only; pass a config around
instead of importing globals.
"""

from __future__ import annotations

import math
import re
import shutil
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Literal, get_args

from crl_husky.config_resolver import get_primary_mocap_id_for_robot_serial
from rclpy.node import Node

from .tool_urdfs import stitch_tools


#: End effectors the monitor can drive; each has a class in
#: robot_interface/end_effectors.py and a model in tool_urdfs.TOOL_URDFS.
#:   robotiq         Robotiq 2F-85 gripper.
#:   scaffolding_v1  Scaffolding tool switched by UR tool digital outputs.
#:   scaffolding_v3  Scaffolding tool with an RS485 driver.
EndEffectorKind = Literal["robotiq", "scaffolding_v1", "scaffolding_v3"]
END_EFFECTOR_KINDS: tuple[str, ...] = get_args(EndEffectorKind)
#: Extra names the `tools` parameter accepts: crl_husky's "robotiq_2F_85", and
#: "none" for a bare arm.
_TOOL_ALIASES: dict[str, str | None] = {"robotiq_2F_85": "robotiq", "none": None}

#: Robot URDFs with tools stitched on; wiped and rewritten on every start.
STITCHED_URDF_DIRECTORY = Path(tempfile.gettempdir()) / "husky_stitched_urdf"

#: Plugins loaded before the requested ones; `-p no_default:=true` skips them.
DEFAULT_PLUGINS: tuple[str, ...] = ("health",)


@dataclass(frozen=True)
class ArmConfig:
    """One UR arm on a robot, and what is mounted on it.

    Attributes:
        name: The arm's URDF joint-name prefix without the trailing underscore,
            e.g. "ur_arm". Also the key used to look up the arm everywhere.
        ros_namespace: The arm's namespace under the robot's, e.g. "ur5e".
        end_effector: What is mounted on the flange, or None for a bare arm.
        end_effector_namespace: The end effector driver's namespace under the
            robot's, e.g. "gripper". Unused when `end_effector` is None.
        cartesian_test_mode: When on, Cartesian commands run every check and
            show in the 3D view but are not sent to the compliance controller.
            ! Test each arm in this mode first, and again after any change to
              its URDF, calibration or pendant mounting: Cartesian targets use
              a frame only those define, and a wrong one sends the arm off-target.
            ! Turn it off only when the startup log shows no URDF frame error
              for this arm (doc/ur_frames.md), then make a small, watched first
              move with a hand on the e-stop.
        stow_joints: Parking joint angles in radians, in UR driver order, or
            None. The Stow button loads them as a target; the move still needs Start.
    """

    name: str
    ros_namespace: str
    end_effector: EndEffectorKind | None = None
    end_effector_namespace: str = ""
    stow_joints: tuple[float, ...] | None = None
    cartesian_test_mode: bool = False


@dataclass(frozen=True)
class RobotConfig:
    """Everything needed to talk to, and draw, one physical robot.

    ! Two sources, split by purpose. The URDF says what the robot *is*:
      kinematics, meshes, joint names. The config says which *drivers* to talk
      to: which arms have a ROS stack and which end effector is mounted on each.
      The URDF cannot answer the second -- the robot URDFs end at each arm's
      tool0 -- so it is not asked to. It goes the other way round: the config
      picks the tools, and their models are stitched onto the URDF.

      What *can* change while running, such as which controller an arm is
      using, is not configuration. It is tracked in RobotState.

    Attributes:
        serial: Clearpath serial such as "a200-0806". Identifies the robot in
            crl_husky's config_resolver and in mocap calibration files.
        ros_namespace: ROS2 namespace the robot publishes under, e.g. "a200_0806".
        urdf_file: Absolute path to the URDF describing this robot as built,
            including whatever end effectors are currently mounted: the robot's
            own URDF with the tools stitched on (tool_urdfs.stitch_tools).
        default_position: Where the robot stands before mocap has said
            otherwise, as (x, y, z) in metres. Set by `row_layout_position` so
            several robots do not all sit on top of each other at the origin.
        default_yaw: Which way it faces at that pose, radians about Z. Ground
            robots only ever rotate about Z, so one angle is the whole story.
        arms: The arms on this robot, in the order the multi-arm trajectory
            message expects them (the first is its `trajectory1`).
        mocap_id: Rigid-body id of the base in the mocap system, or None if the
            robot is not tracked. `primary_mocap_id` from crl_husky's
            config/robots/<digits>/mocap_config.json. The base pose comes only from mocap, so an
            untracked robot never has a measured pose.
        calibration_file: Joint-origin overlay applied on top of the URDF, or
            None to use it as shipped. Nothing reads it yet.

            ! Unresolved, and worth settling before this is wired up.
              The URDFs the monitor currently loads already have calibration
              baked into their filenames (Alice_Calibrated, All_Calibrated --
              see `_ROBOTS_BY_SERIAL` below), which is the arrangement this
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
    arms: tuple[ArmConfig, ...] = ()
    mocap_id: int | None = None
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
        design_directory: Design folder (ActionSchedule.json, BarActions/,
            RobotCell*.json) the `cell` plugin loads at startup, or None to
            pick one in its panel.
        tick_period: Seconds between ticks. Default 0.05 (20 Hz)
        viser_port: Port the viser web UI listens on.
        enabled_plugins: Plugins to load.
            DEFAULT_PLUGINS followed by the requested ones.
        max_plugin_errors: Consecutive ticks a plugin may raise in before it is
            stopped instead of filling the log forever.
        slow_step_warn_ratio: Warn when one plugin's step eats more than this
            fraction of the tick budget.
        slow_step_warn_period: Seconds between repeats of one plugin's slow-step
            warning, so a plugin that is slow every tick cannot bury the log.
    """

    robots: tuple[RobotConfig, ...]
    data_directory: Path
    design_directory: Path | None = None
    tick_period: float = 0.05
    viser_port: int = 8080
    enabled_plugins: tuple[str, ...] = ()
    max_plugin_errors: int = 3
    slow_step_warn_ratio: float = 0.5
    slow_step_warn_period: float = 5.0


# --- --- --- --- --- WHERE ROBOTS STAND BEFORE MOCAP --- --- --- --- ---
#: Metres between neighbouring robots in the startup layout.
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


# --- --- --- --- --- WHAT EACH ROBOT IS --- --- --- --- ---
# TODO temporary. Which URDF a robot uses should come from the robot, not from a
#      table here -- crl_husky is the natural home, but it ships no robot URDF
#      today (only a gripper xacro) and its config_resolver exposes only mocap
#      calibration, so there is nothing to look one up in yet. Until that exists,
#      this reproduces the mapping the old code had, whose sources were
#      old/cfab_session.py:44-56 (the calibrated per-robot files) and
#      old/husky_world.py:189-223 (the domain-id table: dual-arm, tools).
#
# ! These are the *calibrated* URDFs, and which one a robot gets is not
#   cosmetic: the arm kinematics differ per machine. Alice and Belle are the
#   single-arm rigs, Cindy is dual-arm.
#
# ! "_StockUrFrames": the calibrated URDFs with the stock UR base joints
#   restored (scripts/fix_ur_base_frames.py). See doc/ur_frames.md.
_URDF_ROOT = "husky_urdf"
_SINGLE_ARM_URDF = (_URDF_ROOT + "/mt_husky_moveit_config/urdf/"
                    "husky_ur5_e_no_base_joint_{}_Calibrated_StockUrFrames.urdf")
_DUAL_ARM_URDF = (_URDF_ROOT + "/mt_husky_dual_ur5_e_moveit_config/urdf/"
                  "husky_dual_ur5_e_no_base_joint_All_Calibrated_StockUrFrames.urdf")

# TODO real stow poses. These are placeholders taken from the old code, and
#      neither is a stowed arm: the single-arm one is old/husky_robot.py's
#      UR5e_HOME_STATE (arm pointing straight up), the dual-arm ones are the two
#      halves of old/utils.py's HUSKY_DUAL_ARM_HOME_CONF_12, which the old code
#      itself calls "extended arms". Replace with measured stow poses.
_SINGLE_ARM_STOW = (1.569, -2.973, 2.705, -2.958, 1.572, 0.0)
_LEFT_ARM_STOW = (1.569, -2.973, 2.705, -2.958, 1.572, 0.0)
_RIGHT_ARM_STOW = (1.569, -2.973, 2.705, -2.958, 1.572, 0.0)

_SINGLE_ARM_WITH_ROBOTIQ = (
    ArmConfig(name="ur_arm", ros_namespace="ur5e",
              end_effector="robotiq", end_effector_namespace="gripper", stow_joints=_SINGLE_ARM_STOW),
)
_DUAL_ARM_WITH_SCAFFOLDING_V3 = (
    ArmConfig(name="left_ur_arm", ros_namespace="left_ur5e",
              end_effector="scaffolding_v3", end_effector_namespace="left_gripper", stow_joints=_LEFT_ARM_STOW),
    ArmConfig(name="right_ur_arm", ros_namespace="right_ur5e",
              end_effector="scaffolding_v3", end_effector_namespace="right_gripper", stow_joints=_RIGHT_ARM_STOW),
)

_ROBOTS_BY_SERIAL = {
    "0804": dict(urdf=_SINGLE_ARM_URDF.format("Alice"), arms=_SINGLE_ARM_WITH_ROBOTIQ),
    "0805": dict(urdf=_SINGLE_ARM_URDF.format("Belle"), arms=_SINGLE_ARM_WITH_ROBOTIQ),
    "0806": dict(urdf=_DUAL_ARM_URDF, arms=_DUAL_ARM_WITH_SCAFFOLDING_V3),
}

#: The robots' names, accepted anywhere a serial is, in any letter case.
_ROBOT_NAMES = {"alice": "0804", "belle": "0805", "cindy": "0806"}


def robot_config_from_serial(token: str, data_directory: Path,
                             default_position: tuple[float, float, float] = (0.0, 0.0, 0.0),
                             default_yaw: float = 0.0,
                             tools: tuple[EndEffectorKind | None, ...] | None = None) -> RobotConfig:
    """Build one RobotConfig from a serial written any of the usual ways.

    Accepts "0806", "a200-0806", "a200_0806", "/a200_0806" or the robot's name
    ("cindy"), and derives the
    canonical serial and ROS namespace from the four digits, matching the layout
    of crl_husky's config/robots/<digits>/robot.yaml. URDF, arms and end
    effectors come from `_ROBOTS_BY_SERIAL`, unless `tools` says what is
    mounted today; the mocap id from crl_husky's config_resolver. The mounted
    tools' models are stitched onto the URDF (tool_urdfs.stitch_tools).

    Args:
        token: The serial as typed on the command line.
        data_directory: Root the URDF path is resolved against.
        default_position: Where to stand it until mocap says otherwise, usually
            from `row_layout_position`.
        default_yaw: Which way it faces there, radians about Z.
        tools: The end effector on each arm, in the robot's arm order, None for
            a bare arm; or None to keep the defaults in `_ROBOTS_BY_SERIAL`.

    Returns:
        RobotConfig: Identity, URDF, arms and mocap id for that robot.

    Raises:
        ValueError: If no four-digit serial can be read out of `token`, that
            serial is not in `_ROBOTS_BY_SERIAL`, or `tools` does not name one
            tool per arm.
        FileNotFoundError: If the URDF it maps to is not on disk, which usually
            means `data_directory` is wrong, or crl_husky has no
            mocap_config.json for the robot.
    """
    number = _serial_digits(token)
    if number is None:
        raise ValueError(f"cannot read a robot serial or name out of {token!r}; "
                         f"expected something like '0806', 'a200-0806' or 'cindy'")

    spec = _ROBOTS_BY_SERIAL.get(number)
    if spec is None:
        known = ", ".join(sorted(_ROBOTS_BY_SERIAL))
        raise ValueError(f"no configuration known for robot {number!r}; known robots are {known}")

    urdf_file = data_directory / spec["urdf"]
    if not urdf_file.is_file():
        raise FileNotFoundError(f"URDF for robot {number!r} not found at {urdf_file}; "
                                f"check the data_directory parameter")

    arms = spec["arms"]
    if tools is not None:
        if len(tools) != len(arms):
            raise ValueError(f"robot {number!r} has {len(arms)} arm(s) ({', '.join(a.name for a in arms)}), "
                             f"but {len(tools)} tool(s) were given")
        arms = tuple(replace(arm, end_effector=tool) for arm, tool in zip(arms, tools))
    stitched = stitch_tools(urdf_file, {arm.name: arm.end_effector for arm in arms}, data_directory,
                            STITCHED_URDF_DIRECTORY / f"a200_{number}.urdf")

    return RobotConfig(
        serial=f"a200-{number}",
        ros_namespace=f"a200_{number}",
        urdf_file=stitched,
        arms=arms,
        mocap_id=get_primary_mocap_id_for_robot_serial(number),
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
            -p plugins:="['cell']" \\
            -p design_directory:=/path/to/260814_RobArch_support_ik

    `-p tools:=[...]` says which tool is mounted on each arm, when that differs
    from the defaults in `_ROBOTS_BY_SERIAL`. One entry per robot, the tools in
    the robot's arm order (left, right), "none" for a bare arm:

            -p tools:="['0804:scaffolding_v3', '0806:scaffolding_v3,scaffolding_v1']"

    Tool names are END_EFFECTOR_KINDS, or crl_husky's `gripper` launch argument
    values (robotiq_2F_85), so the robot's launch line can be copied.

    `-p no_default:=true` loads only the requested plugins, without
    DEFAULT_PLUGINS.

    Robots may be named instead of numbered ("alice", "belle", "cindy"), in
    `robots` and in `tools` entries alike.

    Args:
        node: The monitor node, whose parameters are read.
    """
    node.declare_parameter("robots", [""])
    node.declare_parameter("tools", [""])
    node.declare_parameter("plugins", [""])
    node.declare_parameter("data_directory", "")
    node.declare_parameter("design_directory", "")
    node.declare_parameter("no_default", False)

    shutil.rmtree(STITCHED_URDF_DIRECTORY, ignore_errors=True)

    def string_list(name: str) -> tuple[str, ...]:
        """Read a string-array parameter, dropping the empty-string default."""
        raw = node.get_parameter(name).get_parameter_value().string_array_value
        return tuple(item for item in raw if item)

    data_text = node.get_parameter("data_directory").get_parameter_value().string_value.strip()
    data_directory = (Path(data_text).expanduser() if data_text
                      else Path(__file__).resolve().parent.parent / "data")

    design_text = node.get_parameter("design_directory").get_parameter_value().string_value.strip()

    no_default = node.get_parameter("no_default").get_parameter_value().bool_value
    return MonitorConfig(
        robots=_robots_in_a_row(string_list("robots"), data_directory, _parse_tools(string_list("tools"))),
        data_directory=data_directory,
        design_directory=Path(design_text).expanduser() if design_text else None,
        enabled_plugins=_enabled_plugins(string_list("plugins"), not no_default),
    )


def _enabled_plugins(requested: tuple[str, ...], use_default_plugins: bool) -> tuple[str, ...]:
    """The defaults followed by the requested plugins, each name once.

    Args:
        requested: Plugin names from the `plugins` parameter.
        use_default_plugins: Whether to put DEFAULT_PLUGINS in front.

    Returns:
        tuple[str, ...]: Names in load order, duplicates dropped.
    """
    defaults = DEFAULT_PLUGINS if use_default_plugins else ()
    # dict keeps the first occurrence of each name, in order.
    return tuple(dict.fromkeys(defaults + requested))


def _parse_tools(entries: tuple[str, ...]) -> dict[str, tuple[EndEffectorKind | None, ...]]:
    """Read the `tools` parameter: "<serial>:<tool>[,<tool>...]" per robot.

    Args:
        entries: The parameter's strings, e.g. ("0806:scaffolding_v3,none",).

    Returns:
        dict[str, tuple[EndEffectorKind | None, ...]]: The tools per arm, keyed
            by the robot's four-digit serial; None for a bare arm.

    Raises:
        ValueError: For an entry without a serial, or an unknown tool name.
    """
    tools = {}
    for entry in entries:
        serial, _, names = entry.partition(":")
        number = _serial_digits(serial)
        if number is None or not names:
            raise ValueError(f"tools entry {entry!r} should look like '0806:scaffolding_v3,scaffolding_v1'")
        kinds = []
        for name in (name.strip() for name in names.split(",")):
            if name in _TOOL_ALIASES:
                kinds.append(_TOOL_ALIASES[name])
            elif name in END_EFFECTOR_KINDS:
                kinds.append(name)
            else:
                valid = ", ".join((*END_EFFECTOR_KINDS, *_TOOL_ALIASES))
                raise ValueError(f"unknown tool {name!r} in tools entry {entry!r}; valid tools are {valid}")
        tools[number] = tuple(kinds)
    return tools


def _robots_in_a_row(serials: tuple[str, ...], data_directory: Path,
                     tools: dict[str, tuple[EndEffectorKind | None, ...]]) -> tuple[RobotConfig, ...]:
    """Build a RobotConfig per serial, spaced out along Y in the order given.

    Args:
        serials: Robot serials as typed on the command line.
        data_directory: Root the URDF paths are resolved against.
        tools: Mounted tools per four-digit serial (`_parse_tools`). Robots not
            in it keep their defaults.

    Returns:
        tuple[RobotConfig, ...]: One per serial, in the same order.
    """
    # ! A tools entry for a robot that is not loaded is almost certainly a
    #   typo in the serial, which would leave the real robot with the wrong tool.
    unused = set(tools) - {_serial_digits(serial) for serial in serials}
    if unused:
        raise ValueError(f"tools given for robot(s) {', '.join(sorted(unused))}, which are not in `robots`")
    return tuple(
        robot_config_from_serial(serial, data_directory,
                                 default_position=row_layout_position(index, len(serials)),
                                 tools=tools.get(_serial_digits(serial)))
        for index, serial in enumerate(serials)
    )


def _serial_digits(token: str) -> str | None:
    """The four-digit serial in "0806", "a200-0806", "/a200_0806", "Cindy" and the like, or None."""
    name = token.strip().strip("/")
    if name.lower() in _ROBOT_NAMES:
        return _ROBOT_NAMES[name.lower()]
    digits = re.search(r"(\d{4})\s*$", name)
    return None if digits is None else digits.group(1)


def find_robot_serial(robots: tuple[RobotConfig, ...], token: str) -> str | None:
    """The serial of the configured robot that `token` names, or None if none does.

    For plugins that meet a robot by name, e.g. a design that says "Alice", and
    need the serial the monitor knows it by ("a200-0804").

    Args:
        robots: The configured robots, usually `ctx.config.robots`.
        token: A serial or robot name, written any of the ways `robots` accepts.

    Returns:
        str | None: The matching robot's serial, or None if it is not configured.
    """
    digits = _serial_digits(token)
    for robot in robots:
        if digits is not None and _serial_digits(robot.serial) == digits:
            return robot.serial
    return None
