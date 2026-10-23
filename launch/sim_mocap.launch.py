"""Mocap for a simulation: Motive stood in for by sim_motive, and the real relay on its simulated rigid bodies.

Start the robot's simulator with its base pose going to the relay, then this, in the same domain of their own:

    export ROS_DOMAIN_ID=70
    ros2 launch crl_husky sim_robot.launch.py robot:=0805 mocap_relay:=true
    ros2 launch husky_assembly_teleop sim_mocap.launch.py robots:=0805 markers:=0805:ur_arm

- `markers`: "<serial>:<arm>[:<rigid body>]" per marker, comma separated; the rigid body defaults to the calibration
  marker's (DEFAULT_TOOL_BODY). Empty for none.
- `calibration_error`: how far the true base calibration is from crl-husky's default, "dx,dy,dz,droll,dpitch,dyaw"
  (metres, degrees), so a calibration in the simulation has something to find.
- Mocap imperfections (noise, dropouts, marker errors, joint offsets, drifts), all off by default: see the arguments
  and doc/calibration_manual.md, "Testing in a simulation".
- The relay reads the simulated sessions on the Drive (calibration_data_sim/domain_<id>), never the robots'.
"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from crl_husky.config_resolver import get_primary_mocap_id_for_robot_serial
from crl_husky.launch_checks import check_simulation_domain

# ! Launch runs on the system Python: import nothing of husky_assembly_teleop here (it needs the venv).
#: The calibration marker's rigid body, a marker's default. ! Keep equal to mocap_relay/sim_motive.py's.
DEFAULT_TOOL_BODY = 1866
#: sim_motive's mocap imperfections, comma separated numbers: (name, default, description).
IMPERFECTIONS = (
    ("base_noise", "0,0", "Base body noise per sample: position std m, angle std deg"),
    ("marker_noise", "0,0", "Marker body noise per sample: position std m, angle std deg"),
    ("base_marker_error", "0.0003", "Base body's reported mean marker error, m"),
    ("marker_error", "0.0003", "Marker body's reported mean marker error, m"),
    ("dropout", "0", "Chance per sample that a body is lost"),
    ("dropout_burst_s", "0", "How long a loss lasts, s"),
    ("joint_offsets", "0,0,0,0,0,0", "Marker arm's encoder zero offsets, deg"),
    ("marker_drift", "0,0,0,0,0,0", "Marker sliding on the flange: m/min and deg/min"),
    ("base_drift", "0,0,0,0,0,0", "Base tracker creeping on the robot: m/min and deg/min"),
)


def generate_launch_description():
    """Check the domain, then start sim_motive and the relay on its rigid bodies."""
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="0805", description="Simulated robots' serials, comma separated"),
        DeclareLaunchArgument("markers", default_value="0805:ur_arm",
                              description="Markers, '<serial>:<arm>[:<rigid body>]', comma separated"),
        DeclareLaunchArgument("calibration_error", default_value="0,0,0,0,0,0",
                              description="True base calibration minus crl-husky's default: m and deg"),
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in IMPERFECTIONS),
        DeclareLaunchArgument("noise_seed", default_value="0", description="Seed of the noise and dropouts"),
        check_simulation_domain(),
        OpaqueFunction(function=_nodes),
    ])


def _nodes(context):
    robots = [token.strip() for token in LaunchConfiguration("robots").perform(context).split(",") if token.strip()]
    markers = [token.strip() for token in LaunchConfiguration("markers").perform(context).split(",") if token.strip()]
    error = [float(v) for v in LaunchConfiguration("calibration_error").perform(context).split(",")]
    # ? Parsed into floats here: launch would hand a bare "0" to sim_motive as an integer, which a float parameter
    # ? refuses. One number is a scalar, several a list.
    imperfections = {"noise_seed": int(LaunchConfiguration("noise_seed").perform(context))}
    for name, _, _ in IMPERFECTIONS:
        values = [float(v) for v in LaunchConfiguration(name).perform(context).split(",")]
        imperfections[name] = values if len(values) > 1 else values[0]
    ids = [get_primary_mocap_id_for_robot_serial(serial[-4:]) for serial in robots]
    for marker in markers:
        _serial, _arm, *rest = marker.split(":")
        ids.append(int(rest[0]) if rest else DEFAULT_TOOL_BODY)
    return [
        Node(package="husky_assembly_teleop", executable="husky_sim_motive", output="screen",
             # ? Full serials: launch writes a bare "0805" to the parameter file as a number.
             parameters=[{"robots": ParameterValue(",".join(f"a200-{s[-4:]}" for s in robots), value_type=str),
                          "markers": ParameterValue(",".join(markers), value_type=str), "calibration_error": error,
                          **imperfections}]),
        Node(package="husky_assembly_teleop", executable="husky_mocap_relay", output="screen",
             parameters=[{"source": "sim", "sim_ids": ParameterValue(",".join(str(i) for i in ids), value_type=str)}]),
    ]
