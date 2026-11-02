"""The monitor, with each of its ROS parameters as a launch argument (README, Parameters).

    ros2 launch husky_assembly_teleop monitor.launch.py robots:="alice" plugins:="robot_control"
    ros2 launch husky_assembly_teleop monitor.launch.py robots:="alice belle cindy" plugins:="robot_control" \\
        tools:="alice:scaffolding_v3 cindy:scaffolding_v3:none"
    ros2 launch husky_assembly_teleop monitor.launch.py plugins:="mocap_probe"    # no robots
    ros2 launch husky_assembly_teleop monitor.launch.py --show-args                # every argument

- Lists are always quoted, their items separated by spaces or commas; the `--ros-args` form
  (`robots:="['alice','belle']"`) works too.
- An empty argument leaves the parameter out, so the monitor's own default applies.
- Ctrl-C once stops it as with `ros2 run`: the plugins get time to send their final hold commands. A second Ctrl-C
  skips them.
"""

import json
import re

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ! Launch runs on the system Python: import nothing of husky_assembly_teleop here (it needs the venv).
#: The monitor's parameters (config.py): (name, type, description).
PARAMETERS = (
    ("robots", "list", "Robots by serial or name, quoted and space separated: \"alice belle\"; empty for none"),
    ("plugins", "list", "Plugins to load, quoted and space separated; health and console are always added"),
    ("tools", "list", "Only where the mounted tools differ, one tool per arm, quoted and space separated: "
                      "\"alice:scaffolding_v3 cindy:scaffolding_v3:none\""),
    ("process_directory", "str", "Process folder the process plugin loads at startup, relative to drive_root"),
    ("data_directory", "str", "Root for meshes, URDFs and processes; empty for this repo's data/"),
    ("drive_root", "str", "Local copy of the project's Drive folder; empty for $HUSKY_DRIVE_ROOT"),
    ("recording_format", "str", "How runs are saved: npz or json; empty for npz"),
    ("no_default_plugins", "bool", "true skips the default plugins (health, console)"),
    ("ghost_timeout", "float", "Seconds a plugin's ghost robots stay after the last input in it; empty for 20"),
    ("layout", "str", "A person's layout file; empty for ~/.config/husky_monitor/layout.json"),
    ("lab", "str", "The lab file obstacles loads: a name in data/lab/ or a .json path; empty for main"),
    ("viser_port", "int", "Port of the desktop page; empty for 8080"),
    ("phone_port", "int", "Port of the phone page; empty for viser_port + 1"),
)
#: Seconds the monitor gets to shut down after Ctrl-C before launch sends SIGTERM (launch's default is 5): every
#: plugin's tasks get 2 s to clean up, one plugin after another.
SHUTDOWN_SECONDS = "30"


def generate_launch_description():
    """Declare an argument per monitor parameter, then start the monitor with the ones given."""
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value="", description=description)
          for name, _kind, description in PARAMETERS),
        OpaqueFunction(function=_monitor),
    ])


def _monitor(context):
    """The monitor node, with a parameter for each argument that is not empty."""
    values = {name: LaunchConfiguration(name).perform(context) for name, _kind, _description in PARAMETERS}
    return [Node(package="husky_assembly_teleop", executable="husky_monitor", output="screen", emulate_tty=True,
                 ros_arguments=ros_arguments(parameters(values)), sigterm_timeout=SHUTDOWN_SECONDS)]


def ros_arguments(typed: dict) -> list[str]:
    """`-p name:=value` per parameter, the value as JSON.

    ! Not `Node(parameters=...)`: its params file leaves strings unquoted, so ROS reads a serial (0804) as a number.
    """
    return [argument for name, value in typed.items() for argument in ("-p", f"{name}:={json.dumps(value)}")]


def parameters(values: dict[str, str]) -> dict:
    """The monitor's parameters from the arguments' text, typed; empty arguments left out.

    Raises:
        ValueError: For a value that does not read as its parameter's type.
        yaml.YAMLError: For a list in the --ros-args form that does not parse.
    """
    result = {}
    for name, kind, _description in PARAMETERS:
        text = values.get(name, "").strip()
        if not text:
            continue
        if kind == "list":
            items = yaml.safe_load(text) if text.startswith("[") else re.split(r"[\s,]+", text)
            # * ROS cannot type an empty list: [""] is the monitor's own "none".
            result[name] = [str(item).strip() for item in items if str(item).strip()] or [""]
        elif kind == "bool":
            if text.lower() not in ("true", "false"):
                raise ValueError(f"{name} must be true or false, not {text!r}")
            result[name] = text.lower() == "true"
        else:
            result[name] = {"int": int, "float": float, "str": str}[kind](text)
    return result
