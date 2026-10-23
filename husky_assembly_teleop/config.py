"""
Run configuration: its dataclasses, the URDF for each robot, and reading it from ROS parameters.

Resolved once at startup, then read-only; pass a config around instead of importing globals.
"""

from __future__ import annotations

import math
import re
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Literal, get_args

import yaml
from crl_husky.config_resolver import (get_park, get_primary_mocap_id_for_robot_serial, get_robot_model,
                                       get_robot_models, get_ur_calibration_file)
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.robot import FLANGE_SUFFIX, RobotModel, robot_model
from scaffolding_assembly_core.urdf import JointOriginChange, apply_ur_calibration
from .drive import drive_root
from .tool_urdfs import absolute_mesh_urdf, tool_spec


#: End effectors the monitor can drive (classes in robot_interface/end_effectors.py).
#:   robotiq         Robotiq 2F-85 gripper.
#:   scaffolding_v1  Scaffolding tool switched by UR tool digital outputs.
#:   scaffolding_v3  Scaffolding tool with an RS485 driver.
EndEffectorKind = Literal["robotiq", "scaffolding_v1", "scaffolding_v3", "calibration_marker"]
END_EFFECTOR_KINDS: tuple[str, ...] = get_args(EndEffectorKind)
#: Extra names the `tools` parameter accepts: crl_husky's "robotiq_2F_85", and "none" for a bare arm.
_TOOL_ALIASES: dict[str, str | None] = {"robotiq_2F_85": "robotiq", "none": None}

#: Robot URDFs with absolute mesh paths (`RobotModel.urdf`), in a folder of this process's own, removed when it exits.
#: ? Per process: two monitors, or a monitor and the tests, must not delete each other's copies.
_ROBOT_URDF_FOLDER = tempfile.TemporaryDirectory(prefix="husky_robot_urdf_")
ROBOT_URDF_DIRECTORY = Path(_ROBOT_URDF_FOLDER.name)

#: Plugins loaded before the requested ones; `-p no_default_plugins:=true` skips them.
DEFAULT_PLUGINS: tuple[str, ...] = ("health",)


@dataclass(frozen=True)
class ArmConfig:
    """One UR arm on a robot, and what is mounted on it.

    Attributes:
        name: URDF joint-name prefix without the trailing underscore, e.g. "ur_arm"; the arm's key everywhere.
        ros_namespace: The arm's namespace under the robot's, e.g. "ur5e".
        label: Short name for tabs and health rows, e.g. "Left".
        end_effector: What is mounted on the flange, or None for a bare arm.
        end_effector_namespace: The tool driver's namespace under the robot's, e.g. "gripper".
        cartesian_test_mode: Cartesian commands run every check and show in the 3D view, but are not sent.
            ! Use it after any change to the arm's URDF, calibration or pendant mounting: a wrong frame
              sends the arm off-target. Turn it off only when the startup log shows no URDF frame error.
        stow_joints: Parking joint angles, radians, UR driver order, or None. The Stow button loads them; from
            crl-husky's park.yaml, where the simulator starts the arm too.
        pendant_tcp: The TCP set on this arm's teach pendant, in tool0: the reported TCP includes it, so the monitor
            takes it off to get tool0. tool0 itself (zero) by decision, whatever tool is mounted; the tool's own
            TCP is `Tool.tcp`, for planners. ! Must equal the pendant's active TCP, or every Cartesian command
            starts from the wrong point (the health `tcp` chip turns amber).
    """

    name: str
    ros_namespace: str
    label: str = "Arm"
    end_effector: EndEffectorKind | None = None
    end_effector_namespace: str = ""
    stow_joints: tuple[float, ...] | None = None
    cartesian_test_mode: bool = False
    pendant_tcp: Pose = Pose()


@dataclass(frozen=True)
class UrCalibrationCheck:
    """Whether one arm's joints in our URDF are the UR factory calibration its driver runs with (crl-husky's copy).

    Attributes:
        arm: The arm's name, e.g. "left_ur_arm".
        file: The calibration file in crl-husky, or None if it is not there yet (then nothing was checked).
        changes: The joints the calibration moved, with how far; empty when the URDF already had it.
    """

    arm: str
    file: Path | None
    changes: tuple[JointOriginChange, ...] = ()

    @property
    def out_of_date(self) -> bool:
        """bool: The URDF's joints differ from the calibration (the robot is built with the calibration)."""
        return bool(self.changes)

    def describe(self) -> str:
        """One line for the log and the health panel."""
        if self.file is None:
            return f"{self.arm}: no UR calibration in crl-husky, so the URDF's arm joints are not checked"
        if not self.changes:
            return f"{self.arm}: URDF matches its UR calibration ({self.file.name})"
        worst = max(self.changes, key=lambda change: (change.distance, change.angle))
        return (f"{self.arm}: URDF OUT OF DATE with its UR calibration {self.file}: {len(self.changes)} joint(s) "
                f"differ, up to {worst.distance * 1e3:.2f} mm / {math.degrees(worst.angle):.3f}° "
                f"({worst.joint}). The monitor uses the calibration; update the URDF in husky_urdf.")


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
            Park button loads it, and the robot starts there until the first mocap fix; from crl-husky's park.yaml,
            where the simulator starts the base too.
        arms: The arms, in the multi-arm trajectory's order (the first is `trajectory1`).
        mocap_id: Rigid-body id of the base in the mocap system, or None if the robot is not tracked.
        ur_calibration: Per arm, whether the URDF's arm joints match the UR calibration in crl-husky. The model is
            built with the calibration applied, so it always matches the robot's driver.
        source_urdf: The URDF in the data directory the model's copy was made from (relative mesh paths, the same
            file on every PC), or None.
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
    ur_calibration: tuple[UrCalibrationCheck, ...] = ()
    source_urdf: Path | None = None

    @property
    def default_orientation(self) -> tuple[float, float, float, float]:
        """tuple[float, float, float, float]: `default_yaw` as an (x, y, z, w) quaternion."""
        half = self.default_yaw / 2.0
        return (0.0, 0.0, math.sin(half), math.cos(half))


#: How a run's recording may be saved: the formats `Trace.save` writes (`plugin_api/trace.py` FORMATS).
RECORDING_FORMATS = ("npz", "json")


@dataclass(frozen=True)
class MonitorConfig:
    """Everything the monitor and its plugins need to know about this run.

    Attributes:
        robots: The robots to connect to, in display order.
        data_directory: Root for meshes, URDFs and process data.
        process_directory: Process folder the `process` plugin loads at startup, relative to `drive_root`, or None
            to pick one in its panel.
        drive_root: The project's Google Drive folder, for experiment data (see `drive.py`), or None if not given.
        recording_format: How plugins save a run's recording: "npz" (compact) or "json" (readable text, several times
            larger); `shared/recordings/` reads both.
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
    recording_format: str = "npz"
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
# * Name, URDF and SRDF come from crl-husky (`get_robot_model`, its config/robots/<serial>/model.yaml), and so do the
#   parked base pose and arm joints (`get_park`, park.yaml), which the simulators start from; the arms and tools are
#   here. Each arm of the URDF is checked against its UR calibration in crl-husky
#   at startup (`_apply_ur_calibrations`), its stock UR base frames by test (doc/ur_frames.md).
# ? TODO consolidate a robot's config further: it is spread over crl-husky's robot.yaml, env/setup.bash,
#   mocap_config.json, model.yaml and this table.
#: Where crl-husky's model paths start, below the data directory.
_URDF_ROOT = "husky_urdf"

_SINGLE_ARM_WITH_ROBOTIQ = (
    ArmConfig(name="ur_arm", ros_namespace="ur5e",
              end_effector="robotiq", end_effector_namespace="gripper"),
)
_DUAL_ARM_WITH_SCAFFOLDING_V3 = (
    ArmConfig(name="left_ur_arm", ros_namespace="left_ur5e", label="Left",
              end_effector="scaffolding_v3", end_effector_namespace="left_gripper"),
    ArmConfig(name="right_ur_arm", ros_namespace="right_ur5e", label="Right",
              end_effector="scaffolding_v3", end_effector_namespace="right_gripper"),
)

_ROBOTS_BY_SERIAL = {
    "0804": dict(arms=_SINGLE_ARM_WITH_ROBOTIQ),
    "0805": dict(arms=_SINGLE_ARM_WITH_ROBOTIQ),
    "0806": dict(arms=_DUAL_ARM_WITH_SCAFFOLDING_V3),
}


def robot_config_from_serial(token: str, data_directory: Path,
                             default_position: tuple[float, float, float] | None = None,
                             default_yaw: float | None = None,
                             tools: tuple[EndEffectorKind | None, ...] | None = None) -> RobotConfig:
    """Build one RobotConfig from a serial ("0806", "a200-0806", "/a200_0806") or name ("cindy").

    Arms come from `_ROBOTS_BY_SERIAL`; URDF, SRDF, parked poses and mocap id from crl_husky's config_resolver.

    Args:
        token: The serial or name as typed on the command line.
        data_directory: Root the URDF path is resolved against.
        default_position: Where to draw it until the first mocap fix, or None for its parked pose.
        default_yaw: Which way it faces there, radians about Z, or None for its parked yaw.
        tools: The end effector per arm, in arm order (None for a bare arm), or None for the defaults.

    Returns:
        RobotConfig: The robot, its model holding its tools.

    Raises:
        ValueError: If `token` names no known robot, crl-husky has no model.yaml or park.yaml for it, park.yaml has
            no joints for one of its arms, or `tools` does not give one tool per arm.
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

    model = get_robot_model(number)
    if model is None:
        raise ValueError(f"crl-husky has no model.yaml for robot {number!r} (config/robots/{number}/)")
    urdf_file = data_directory / _URDF_ROOT / model["urdf"]
    srdf_file = data_directory / _URDF_ROOT / model["srdf"]
    if not urdf_file.is_file():
        raise FileNotFoundError(f"URDF for robot {number!r} not found at {urdf_file}; "
                                f"check the data_directory parameter")

    park = get_park(number)
    if park is None:
        raise ValueError(f"crl-husky has no park.yaml for robot {number!r} (config/robots/{number}/)")
    unparked = [arm.name for arm in spec["arms"] if _arm_side(arm.name) not in park["arms"]]
    if unparked:
        raise ValueError(f"crl-husky's park.yaml for robot {number!r} has no joints for {', '.join(unparked)}")
    arms = tuple(replace(arm, stow_joints=park["arms"][_arm_side(arm.name)]) for arm in spec["arms"])
    park = park["base"]
    if tools is not None:
        if len(tools) != len(arms):
            example = ",".join(tool or "none" for tool in (*tools, *(None,) * len(arms))[:len(arms)])
            raise ValueError(f"robot {number!r} has {len(arms)} arm(s) ({', '.join(a.name for a in arms)}), "
                             f"but {len(tools)} tool(s) were given; list one tool per arm in that order, "
                             f"\"none\" for a bare arm, e.g. -p tools:=\"['{number}:{example}']\"")
        arms = tuple(replace(arm, end_effector=tool) for arm, tool in zip(arms, tools))
    urdf = absolute_mesh_urdf(urdf_file, ROBOT_URDF_DIRECTORY / f"a200_{number}.urdf")
    checks = _apply_ur_calibrations(urdf, number, arms)
    tools = {f"{arm.name}{FLANGE_SUFFIX}": tool_spec(arm.end_effector, arm.name, data_directory,
                                                     f"tools/a200-{number}/{arm.name}")
             for arm in arms if arm.end_effector is not None}

    return RobotConfig(
        serial=f"a200-{number}",
        ros_namespace=f"a200_{number}",
        model=robot_model(f"a200-{number}", urdf, srdf_file,
                          {flange: tool for flange, tool in tools.items() if tool is not None}),
        srdf_file=srdf_file,
        arms=arms,
        mocap_id=get_primary_mocap_id_for_robot_serial(number),
        default_position=(park[0], park[1], 0.0) if default_position is None else default_position,
        default_yaw=park[2] if default_yaw is None else default_yaw,
        park_pose=park,
        ur_calibration=checks,
        source_urdf=urdf_file,
    )


def _arm_side(arm_name: str) -> str | None:
    """An arm's side as crl-husky names its calibration file: "left", "right", or None for a one-arm robot."""
    side = arm_name.split("_", 1)[0]
    return side if side in ("left", "right") else None


def _apply_ur_calibrations(urdf: Path, number: str, arms: tuple[ArmConfig, ...]) -> tuple[UrCalibrationCheck, ...]:
    """Apply each arm's UR calibration from crl-husky to the robot's URDF copy, in place, and say what changed.

    With up-to-date URDFs nothing changes. If something does, the URDF in husky_urdf is out of date: the copy gets
    the calibration anyway, so the monitor matches the robot's driver.

    Args:
        urdf: The robot's URDF copy (`absolute_mesh_urdf`); rewritten only if a calibration changes it.
        number: The robot's four serial digits.
        arms: The robot's arms.
    """
    text = original = urdf.read_text()
    checks = []
    for arm in arms:
        file = get_ur_calibration_file(number, _arm_side(arm.name))
        if file is None:
            checks.append(UrCalibrationCheck(arm.name, None))
            continue
        text, changes = apply_ur_calibration(text, arm.name, yaml.safe_load(file.read_text()))
        checks.append(UrCalibrationCheck(arm.name, file, tuple(changes)))
    if text != original:
        urdf.write_text(text)
    return tuple(checks)


# --- --- --- --- --- READING ONE RUN OFF THE COMMAND LINE --- --- --- --- ---
def config_from_ros_parameters(node: Node) -> MonitorConfig:
    r"""Build the frozen run configuration from `node`'s ROS2 parameters.

    - `tools`: one entry per robot, tools in arm order, "none" for a bare arm; only where it differs
      from `_ROBOTS_BY_SERIAL`.
    - `no_default_plugins:=true` skips DEFAULT_PLUGINS.
    - `drive_root`: the project's Google Drive folder; empty falls back to HUSKY_DRIVE_ROOT (`drive.py`).
    - `recording_format`: "npz" (the default) or "json".
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
    node.declare_parameter("recording_format", MonitorConfig.recording_format)
    node.declare_parameter("no_default_plugins", False)
    node.declare_parameter("ghost_timeout", MonitorConfig.ghost_timeout)
    node.declare_parameter("lab", "")
    node.declare_parameter("viser_port", MonitorConfig.viser_port)
    # * 0: the port after viser_port, so a second monitor needs only -p viser_port:=8095.
    node.declare_parameter("phone_port", 0)

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

    recording_format = node.get_parameter("recording_format").get_parameter_value().string_value.strip()
    if recording_format not in RECORDING_FORMATS:
        raise ValueError(f"recording_format must be one of {RECORDING_FORMATS}, not {recording_format!r}")

    no_default_plugins = node.get_parameter("no_default_plugins").get_parameter_value().bool_value
    viser_port = node.get_parameter("viser_port").value
    return MonitorConfig(
        robots=_robots(string_list("robots"), data_directory, _parse_tools(string_list("tools"))),
        data_directory=data_directory,
        process_directory=Path(process_text) if process_text else None,
        drive_root=drive_root(node.get_parameter("drive_root").get_parameter_value().string_value),
        recording_format=recording_format,
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
    # The robots' names (crl-husky's CRL_HUSKY_NAME) work anywhere a serial does, in any letter case.
    for serial, model in get_robot_models().items():
        if model["name"] and model["name"].lower() == name.lower():
            return serial[-4:]
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
