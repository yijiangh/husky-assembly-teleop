"""
Fake lab: every robot simulated as crl-husky's sim_all.launch.py does (base and arms, arms at their parked joints),
but each base starts shifted from its parked pose in -y (launch argument shift_y, metres).

! Why the shift: the parked row (park.yaml, y = 1.35 m, facing +y) overlaps the process's Rhino cabinet
  (obstacles/env39, y 1.63-2.23 m), so once the process is loaded the base planner refuses every parked start. The
  cabinet as measured with the probe (data/lab/main.json) starts at y = 2.00 m: the real park row is fine, and the
  Rhino file's cabinet is 0.38 m off.
? The Rhino cabinet only counts while the process panel's "Rhino obstacles collide" box is ticked (off at start), so
  shift_y:=0 (the park row itself) works too while it is off.
! Only in a ROS domain of its own, never the robots' (checked before anything starts). Run it through sim.sh.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from crl_husky.config_resolver import get_park, get_robot_models
from crl_husky.launch_checks import check_simulation_domain

#: Default shift of every base from its parked pose, in y, metres.
SHIFT_Y = "-0.5"


def generate_launch_description() -> LaunchDescription:
    """Check the domain, then simulate each robot that has a model, its base shifted from its parked pose.

    Returns:
        LaunchDescription: The shift argument, the domain check, then one sim_robot.launch.py per robot.
    """
    return LaunchDescription([
        DeclareLaunchArgument("shift_y", default_value=SHIFT_Y,
                              description="Shift of every base from its parked pose in y, metres"),
        check_simulation_domain(),
        OpaqueFunction(function=_robots),
    ])


def _robots(context) -> list[IncludeLaunchDescription]:
    """One sim_robot.launch.py per robot, started at its parked pose shifted by shift_y.

    Args:
        context: The launch context, for shift_y.

    Returns:
        list[IncludeLaunchDescription]: The includes.

    Raises:
        ValueError: If a robot has no park.yaml.
    """
    shift_y = float(LaunchConfiguration("shift_y").perform(context))
    robot = PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare("crl_husky"), "launch", "sim", "sim_robot.launch.py"]))
    includes = []
    for serial in (serial.removeprefix("a200-") for serial in get_robot_models()):
        park = get_park(serial)
        if park is None:
            raise ValueError(f"robot {serial}: no config/robots/{serial}/park.yaml, so nowhere to start")
        x, y, yaw = park["base"]
        # ! Every start argument given, as in sim_all: includes do not scope launch configurations.
        includes.append(IncludeLaunchDescription(robot, launch_arguments={
            "robot": serial, "start_x": str(x), "start_y": str(y + shift_y), "start_yaw": str(yaw)}.items()))
    return includes
