"""
Run configuration: its dataclasses, the URDF for each robot, and reading it from ROS parameters.

Resolved once at startup, then read-only; pass a config around instead of importing globals.
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
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node

from scaffolding_assembly_core.robot import FLANGE_SUFFIX, RobotModel, robot_model
from .drive import drive_root
from .tool_urdfs import absolute_mesh_urdf, tool_spec


#: End effectors the monitor can drive (classes in robot_interface/end_effectors.py).
#:   robotiq         Robotiq 2F-85 gripper.
#:   scaffolding_v1  Scaffolding tool switched by UR tool digital outputs.
#:   scaffolding_v3  Scaffolding tool with an RS485 driver.
EndEffectorKind = Literal["robotiq", "scaffolding_v1", "scaffolding_v3"]
END_EFFECTOR_KINDS: tuple[str, ...] = get_args(EndEffectorKind)
#: Extra names the `tools` parameter accepts: crl_husky's "robotiq_2F_85", and "none" for a bare arm.
_TOOL_ALIASES: dict[str, str | None] = {"robotiq_2F_85": "robotiq", "none": None}

#: Robot URDFs with absolute mesh paths (`RobotModel.urdf`); wiped and rewritten on every start.
ROBOT_URDF_DIRECTORY = Path(tempfile.gettempdir()) / "husky_robot_urdf"

#: Plugins loaded before the requested ones; `-p no_default_plugins:=true` skips them.
DEFAULT_PLUGINS: tuple[str, ...] = ("health",)


@dataclass(frozen=True)
class ArmConfig:
    """One UR arm on a robot, and what is mounted on it.

    Attributes:
        name: URDF joint-name prefix without the trailing underscore, e.g. "ur_arm"; the arm's key everywhere.
        ros_namespace: The arm's namespace under the robot's, e.g. "ur5e".
        end_effector: What is mounted on the flange, or None for a bare arm.
        end_effector_namespace: The tool driver's namespace under the robot's, e.g. "gripper".
        cartesian_test_mode: Cartesian commands run every check and show in the 3D view, but are not sent.
            ! Use it after any change to the arm's URDF, calibration or pendant mounting: a wrong frame
              sends the arm off-target. Turn it off only when the startup log shows no URDF frame error.
        stow_joints: Parking joint angles, radians, UR driver order, or None. The Stow button loads them.
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

    `model` says what the robot is: its URDF (kinematics, meshes), SRDF and mounted tools, read by kinematics,
    drawing and mirrors alike. The config says which drivers it has. Anything that changes while running lives in
    RobotState.

    Attributes:
        serial: Clearpath serial such as "a200-0806".
        ros_namespace: ROS2 namespace the robot publishes under, e.g. "a200_0806".
        model: The robot as the core describes it: its URDF without tools (mesh paths absolute), its SRDF, and each
            mounted tool as a `Tool` on its flange.
        srdf_file: The robot's SRDF, or None. Needed by compas_fab planners.
        default_position: Where the robot is drawn before the first mocap fix, (x, y, z) metres.
        default_yaw: Which way it faces there, radians about Z.
        park_pose: Where the robot parks on the floor, (x, y, yaw) metres and radians, or None. The base planner's
            Park button loads it, and the robot starts there until the first mocap fix.
        arms: The arms, in the multi-arm trajectory's order (the first is `trajectory1`).
        mocap_id: Rigid-body id of the base in the mocap system, or None if the robot is not tracked.
        calibration_file: Joint-origin overlay for the URDF, or None. Nothing reads it yet.
    """

    serial: str
    ros_namespace: str
    model: RobotModel
    srdf_file: Path | None = None
    arms: tuple[ArmConfig, ...] = ()
    mocap_id: int | None = None
    default_position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    default_yaw: float = 0.0
    park_pose: tuple[float, float, float] | None = None
    # TODO: wire up calibration_file and drop the calibrated URDFs in `_ROBOTS_BY_SERIAL`, or remove it.
    calibration_file: Path | None = None

    @property
    def default_orientation(self) -> tuple[float, float, float, float]:
        """tuple[float, float, float, float]: `default_yaw` as an (x, y, z, w) quaternion."""
        half = self.default_yaw / 2.0
        return (0.0, 0.0, math.sin(half), math.cos(half))


@dataclass(frozen=True)
class MonitorConfig:
    """Everything the monitor and its plugins need to know about this run.

    Attributes:
        robots: The robots to connect to, in display order.
        data_directory: Root for meshes, URDFs and process data.
        process_directory: Process folder the `process` plugin loads at startup, relative to `drive_root`, or None
            to pick one in its panel.
        drive_root: The project's Google Drive folder, for experiment data (see `drive.py`), or None if not given.
        tick_period: Seconds between ticks.
        viser_port: Port the desktop page listens on. Pinned: the monitor refuses to start if it is taken.
        phone_port: Port the phone page listens on. Pinned too.
        enabled_plugins: Plugins to load: DEFAULT_PLUGINS, then the requested ones.
        max_plugin_errors: Consecutive ticks a plugin may raise in before it is stopped.
        late_tick_warn_ratio: Warn, with where the time went, when a tick starts more than this fraction of
            `tick_period` late.
        late_tick_warn_period: Seconds between two late-tick warnings; the worst tick in between is reported.
        shutdown_grace: Seconds each plugin's cancelled tasks get to clean up at shutdown.
        lab: The lab file the `obstacles` plugin loads: a name under `data_directory/lab/`, or a .json path;
            empty for its default.
        ghost_timeout: Seconds a plugin's ghost robots stay shown after the last input in it;
            0 keeps them. Ghosts appear only after an input in their plugin.
    """

    robots: tuple[RobotConfig, ...]
    data_directory: Path
    process_directory: Path | None = None
    drive_root: Path | None = None
    tick_period: float = 0.05
    viser_port: int = 8080
    phone_port: int = 8081
    enabled_plugins: tuple[str, ...] = ()
    max_plugin_errors: int = 3
    late_tick_warn_ratio: float = 0.2
    late_tick_warn_period: float = 5.0
    shutdown_grace: float = 2.0
    lab: str = ""
    ghost_timeout: float = 20.0


# --- --- --- --- --- WHAT EACH ROBOT IS --- --- --- --- ---
# TODO move this table to crl_husky once it ships robot URDFs.
# ! Calibrated URDFs: arm kinematics differ per machine, so each robot must get its own. Their stock UR base frames
#   are checked by test (doc/ur_frames.md).
_URDF_ROOT = "husky_urdf"
_SINGLE_ARM_URDF = (_URDF_ROOT + "/mt_husky_moveit_config/urdf/"
                    "husky_ur5_e_no_base_joint_{}_Calibrated.urdf")
_DUAL_ARM_URDF = (_URDF_ROOT + "/mt_husky_dual_ur5_e_moveit_config/urdf/"
                  "husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf")
_SINGLE_ARM_SRDF = _URDF_ROOT + "/mt_husky_moveit_config/config/{}.srdf"
_DUAL_ARM_SRDF = _URDF_ROOT + "/mt_husky_dual_ur5_e_moveit_config/config/dual_arm_husky.srdf"

_SINGLE_ARM_STOW = (1.569, -2.973, 2.705, -2.958, 1.572, 0.0)
# Measured parking poses on Cindy (0806), entered in degrees as the status panel shows them.
_LEFT_ARM_STOW = tuple(math.radians(d) for d in (-88.4, -2.2, -161.3, -108.8, -0.8, 90.7))
_RIGHT_ARM_STOW = tuple(math.radians(d) for d in (87.0, -179.8, 159.3, -74.7, 1.9, -90.2))

# Parked base poses, (x, y, yaw) in the mocap frame, metres and radians: in a row along the front of the building
# cabinet (data/lab/main.json, front face at y 2.00), facing +y with the battery and screen at the back towards the
# open lab, 15 cm between the stowed robot's front and the cabinet.
_PARK_Y = 1.35
_ALICE_PARK = (0.3, _PARK_Y, math.pi / 2)
_BELLE_PARK = (1.7, _PARK_Y, math.pi / 2)
_CINDY_PARK = (3.1, _PARK_Y, math.pi / 2)

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
    "0804": dict(urdf=_SINGLE_ARM_URDF.format("Alice"), srdf=_SINGLE_ARM_SRDF.format("husky"),
                 arms=_SINGLE_ARM_WITH_ROBOTIQ, park=_ALICE_PARK),
    "0805": dict(urdf=_SINGLE_ARM_URDF.format("Belle"), srdf=_SINGLE_ARM_SRDF.format("belle"),
                 arms=_SINGLE_ARM_WITH_ROBOTIQ, park=_BELLE_PARK),
    "0806": dict(urdf=_DUAL_ARM_URDF, srdf=_DUAL_ARM_SRDF, arms=_DUAL_ARM_WITH_SCAFFOLDING_V3, park=_CINDY_PARK),
}

#: The robots' names, accepted anywhere a serial is, in any letter case.
_ROBOT_NAMES = {"alice": "0804", "belle": "0805", "cindy": "0806"}


def robot_config_from_serial(token: str, data_directory: Path,
                             default_position: tuple[float, float, float] | None = None,
                             default_yaw: float | None = None,
                             tools: tuple[EndEffectorKind | None, ...] | None = None) -> RobotConfig:
    """Build one RobotConfig from a serial ("0806", "a200-0806", "/a200_0806") or name ("cindy").

    URDF and arms come from `_ROBOTS_BY_SERIAL`, the mocap id from crl_husky's config_resolver.

    Args:
        token: The serial or name as typed on the command line.
        data_directory: Root the URDF path is resolved against.
        default_position: Where to draw it until the first mocap fix, or None for its parked pose.
        default_yaw: Which way it faces there, radians about Z, or None for its parked yaw.
        tools: The end effector per arm, in arm order (None for a bare arm), or None for the defaults.

    Returns:
        RobotConfig: The robot, its model holding its tools.

    Raises:
        ValueError: If `token` names no known robot, or `tools` does not give one tool per arm.
        FileNotFoundError: If the URDF is missing (usually a wrong `data_directory`), or crl_husky has no
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

    arms, park = spec["arms"], spec["park"]
    if tools is not None:
        if len(tools) != len(arms):
            raise ValueError(f"robot {number!r} has {len(arms)} arm(s) ({', '.join(a.name for a in arms)}), "
                             f"but {len(tools)} tool(s) were given")
        arms = tuple(replace(arm, end_effector=tool) for arm, tool in zip(arms, tools))
    urdf = absolute_mesh_urdf(urdf_file, ROBOT_URDF_DIRECTORY / f"a200_{number}.urdf")
    tools = {f"{arm.name}{FLANGE_SUFFIX}": tool_spec(arm.end_effector, arm.name, data_directory,
                                                     f"tools/a200-{number}/{arm.name}")
             for arm in arms if arm.end_effector is not None}

    return RobotConfig(
        serial=f"a200-{number}",
        ros_namespace=f"a200_{number}",
        model=robot_model(f"a200-{number}", urdf, data_directory / spec["srdf"],
                          {flange: tool for flange, tool in tools.items() if tool is not None}),
        srdf_file=data_directory / spec["srdf"],
        arms=arms,
        mocap_id=get_primary_mocap_id_for_robot_serial(number),
        default_position=(park[0], park[1], 0.0) if default_position is None else default_position,
        default_yaw=park[2] if default_yaw is None else default_yaw,
        park_pose=park,
    )


# --- --- --- --- --- READING ONE RUN OFF THE COMMAND LINE --- --- --- --- ---
def config_from_ros_parameters(node: Node) -> MonitorConfig:
    r"""Build the frozen run configuration from `node`'s ROS2 parameters.

    - `tools`: one entry per robot, tools in arm order, "none" for a bare arm; only where it differs
      from `_ROBOTS_BY_SERIAL`.
    - `no_default_plugins:=true` skips DEFAULT_PLUGINS.
    - `drive_root`: the project's Google Drive folder; empty falls back to HUSKY_DRIVE_ROOT (`drive.py`).
    - Robots may be named instead of numbered, in `robots` and `tools` alike. `robots:="[]"` runs without robots.
    - `viser_port` (8080) and `phone_port` (default the next port): a second monitor needs `viser_port:=8095`.

    Args:
        node: The monitor node, whose parameters are read.

    Example:
        ros2 run husky_assembly_teleop husky_monitor --ros-args \
            -p robots:="['0804','cindy']" -p plugins:="['process']" \
            -p tools:="['0804:scaffolding_v3', '0806:scaffolding_v3,none']" \
            -p process_directory:=data_design_study/260814_RobArch_support_ik
    """
    node.declare_parameter("robots", [""])
    node.declare_parameter("tools", [""])
    node.declare_parameter("plugins", [""])
    node.declare_parameter("data_directory", "")
    node.declare_parameter("process_directory", "")
    node.declare_parameter("drive_root", "")
    node.declare_parameter("no_default_plugins", False)
    node.declare_parameter("ghost_timeout", MonitorConfig.ghost_timeout)
    node.declare_parameter("lab", "")
    node.declare_parameter("viser_port", MonitorConfig.viser_port)
    # * 0: the port after viser_port, so a second monitor needs only -p viser_port:=8095.
    node.declare_parameter("phone_port", 0)

    shutil.rmtree(ROBOT_URDF_DIRECTORY, ignore_errors=True)

    def string_list(name: str) -> tuple[str, ...]:
        """Read a string-array parameter, dropping the empty-string default.

        ? `-p robots:="[]"` gives an empty list with no element type, which ROS keeps as an unset parameter.
        """
        try:
            raw = node.get_parameter(name).get_parameter_value().string_array_value
        except ParameterUninitializedException:
            return ()
        return tuple(item for item in raw if item)

    data_text = node.get_parameter("data_directory").get_parameter_value().string_value.strip()
    data_directory = (Path(data_text).expanduser() if data_text
                      else Path(__file__).resolve().parent.parent / "data")

    process_text = node.get_parameter("process_directory").get_parameter_value().string_value.strip()

    no_default_plugins = node.get_parameter("no_default_plugins").get_parameter_value().bool_value
    viser_port = node.get_parameter("viser_port").value
    return MonitorConfig(
        robots=_robots(string_list("robots"), data_directory, _parse_tools(string_list("tools"))),
        data_directory=data_directory,
        process_directory=Path(process_text) if process_text else None,
        drive_root=drive_root(node.get_parameter("drive_root").get_parameter_value().string_value),
        enabled_plugins=_enabled_plugins(string_list("plugins"), not no_default_plugins),
        ghost_timeout=node.get_parameter("ghost_timeout").value,
        lab=node.get_parameter("lab").get_parameter_value().string_value.strip(),
        viser_port=viser_port,
        phone_port=node.get_parameter("phone_port").value or viser_port + 1,
    )


def _enabled_plugins(requested: tuple[str, ...], use_default_plugins: bool) -> tuple[str, ...]:
    """The defaults (if `use_default_plugins`) followed by the requested plugins, each name once."""
    defaults = DEFAULT_PLUGINS if use_default_plugins else ()
    # dict keeps the first occurrence of each name, in order.
    return tuple(dict.fromkeys(defaults + requested))


def _parse_tools(entries: tuple[str, ...]) -> dict[str, tuple[EndEffectorKind | None, ...]]:
    """Read the `tools` parameter: "<serial>:<tool>[,<tool>...]" per robot.

    Args:
        entries: The parameter's strings, e.g. ("0806:scaffolding_v3,none",).

    Returns:
        dict[str, tuple[EndEffectorKind | None, ...]]: The tools per arm by four-digit serial; None for a bare arm.

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


def _robots(serials: tuple[str, ...], data_directory: Path,
            tools: dict[str, tuple[EndEffectorKind | None, ...]]) -> tuple[RobotConfig, ...]:
    """Build a RobotConfig per serial, each starting at its parked pose.

    Args:
        serials: Robot serials as typed on the command line.
        data_directory: Root the URDF paths are resolved against.
        tools: Mounted tools per four-digit serial (`_parse_tools`); robots not in it keep their defaults.

    Returns:
        tuple[RobotConfig, ...]: One per serial, in the same order.
    """
    # ! A tools entry for a robot that is not loaded is likely a typo that leaves the real robot misconfigured.
    unused = set(tools) - {_serial_digits(serial) for serial in serials}
    if unused:
        raise ValueError(f"tools given for robot(s) {', '.join(sorted(unused))}, which are not in `robots`")
    return tuple(robot_config_from_serial(serial, data_directory, tools=tools.get(_serial_digits(serial)))
                 for serial in serials)


def _serial_digits(token: str) -> str | None:
    """The four-digit serial in "0806", "a200-0806", "/a200_0806", "Cindy" and the like, or None."""
    name = token.strip().strip("/")
    if name.lower() in _ROBOT_NAMES:
        return _ROBOT_NAMES[name.lower()]
    digits = re.search(r"(\d{4})\s*$", name)
    return None if digits is None else digits.group(1)


def find_robot_serial(robots: tuple[RobotConfig, ...], token: str) -> str | None:
    """The serial ("a200-0804") of the configured robot that `token` ("Alice", "0804") names, or None.

    Args:
        robots: The configured robots, usually `ctx.config.robots`.
        token: A serial or robot name, written any of the ways `robots` accepts.
    """
    digits = _serial_digits(token)
    for robot in robots:
        if digits is not None and _serial_digits(robot.serial) == digits:
            return robot.serial
    return None
