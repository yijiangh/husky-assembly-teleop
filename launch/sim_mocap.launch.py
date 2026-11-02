"""A fake Motive for a simulation: sim_motive, between the simulators and the relay, with Motive's imperfections.

The relay (mocap.launch.py) and the simulated robots run in the same domain of their own; this takes every simulated
body's true pose (each base, and the marker of an arm carrying tool:=calibration_marker) and publishes it as Motive
would, which the relay then takes in their place:

    export ROS_DOMAIN_ID=70
    ros2 launch husky_assembly_teleop mocap.launch.py sim:=true
    ros2 launch crl_husky sim_robot.launch.py robot:=0805 tool:=calibration_marker
    ros2 launch husky_assembly_teleop sim_mocap.launch.py offsets:="1850:0.004,-0.003,0.002,0,0,0.5"

- Per-body arguments: values for every body, or "<id>:<values>" per body, space separated (one body's own entry wins).
- `offsets`: where Motive's body frame sits from the true pose, as a calibration gives it (x,y,z m, roll,pitch,yaw
  deg); for a robot's base, how far its true calibration is from crl-husky's default, so a calibration in the
  simulation has something to find.
- The other imperfections (delay, noise, dropouts, marker errors, drifts) are all off by default: see the arguments.
- It also announces what Motive's model descriptions give the relay: a body definition per faked body (`refines`
  nudges one, as a refine in Motive) and the lab's cameras (`cameras_file`).
"""


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from crl_husky.launch_checks import check_simulation_domain

# ! Launch runs on the system Python: import nothing of husky_assembly_teleop here (it needs the venv).
#: sim_motive's numbers: (name, default, description).
NUMBERS = (
    ("delay_sec", "0", "Seconds from true to the relay: Motive's latency"),
    ("dropout", "0", "Chance per sample that a body is lost"),
    ("dropout_burst_s", "0", "How long a loss lasts, s"),
)
#: Its per-body values, passed on as text: (name, default, description).
PER_BODY = (
    ("noise", "", "Noise per sample: position std m, angle std deg"),
    ("marker_error", "0.0003", "Reported mean marker error, m"),
    ("drift", "", "The body frame creeping: x,y,z m/min, rotation vector deg/min"),
    ("offsets", "", "The true pose in Motive's body frame: x,y,z m, roll,pitch,yaw deg; a base's over the default"),
    ("refines", "", "Refines per body (markers nudged); also: ros2 param set /sim_motive refines \"1850:1\""),
    ("cameras_file", "", "Cameras to announce, JSON as /mocap_relay/cameras; empty: the lab's, none: none"),
    ("recalibration", "", "Motive's frame moved: x,y,z m, roll,pitch,yaw deg; also ros2 param set while running"),
)


def generate_launch_description():
    """Check the domain, then start sim_motive."""
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=default, description=description)
          for name, default, description in NUMBERS + PER_BODY),
        DeclareLaunchArgument("noise_seed", default_value="0", description="Seed of the noise and dropouts"),
        check_simulation_domain(),
        OpaqueFunction(function=_nodes),
    ])


def _nodes(context):
    # ? Parsed into floats here: launch would hand a bare "0" to sim_motive as an integer, which a float parameter
    # ? refuses.
    parameters = {name: float(LaunchConfiguration(name).perform(context)) for name, _, _ in NUMBERS}
    parameters.update({name: ParameterValue(LaunchConfiguration(name).perform(context), value_type=str)
                       for name, _, _ in PER_BODY})
    parameters["noise_seed"] = int(LaunchConfiguration("noise_seed").perform(context))
    return [Node(package="husky_assembly_teleop", executable="husky_sim_motive", output="screen",
                 parameters=[parameters])]
