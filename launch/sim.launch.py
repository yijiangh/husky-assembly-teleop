"""A simulation in one launch: crl-husky's simulated robot (sim_robot, or sim_all), and with it a fake Motive.

The relay (mocap.launch.py) and the monitor start once, as with the real robots; this comes and goes beside them:

    export ROS_DOMAIN_ID=70
    ros2 launch husky_assembly_teleop sim.launch.py robot:=0805
    ros2 launch husky_assembly_teleop sim.launch.py robot:=0805 tool:=calibration_marker fake_motive:=true \\
        offsets:="1850:0.004,-0.003,0.002,0,0,0.5"
    ros2 launch husky_assembly_teleop sim.launch.py robot:=all

- Every argument of crl_husky's sim_robot.launch.py and of sim_mocap.launch.py is one of this launch's, read from those
  files with their defaults: `ros2 launch husky_assembly_teleop sim.launch.py --show-args` lists them.
- `fake_motive:=true` adds sim_motive (sim_mocap.launch.py); without it the relay uses the true poses as they are.
- `robot:=all` simulates every robot (sim_all.launch.py): parked, each arm with the robot's tool (crl-husky's
  model.yaml).
"""

import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

#: The launches this one starts: crl-husky's robot simulation, and this package's fake Motive.
SIM_ROBOT = Path(get_package_share_directory("crl_husky")) / "launch" / "sim" / "sim_robot.launch.py"
SIM_ALL = SIM_ROBOT.with_name("sim_all.launch.py")
FAKE_MOTIVE = Path(__file__).resolve().with_name("sim_mocap.launch.py")


def generate_launch_description():
    """Offer both launches' arguments, then start the robots and, if asked, the fake Motive."""
    robot, motive = _declarations(SIM_ROBOT), _declarations(FAKE_MOTIVE)
    return LaunchDescription([
        *robot, *motive,
        DeclareLaunchArgument("fake_motive", default_value="false",
                              description="Also start sim_motive, a fake Motive with Motive's imperfections"),
        OpaqueFunction(function=_includes, args=[[d.name for d in robot], [d.name for d in motive]]),
    ])


def _declarations(path: Path) -> list:
    """The arguments a launch file declares, as it declares them (defaults and descriptions)."""
    spec = importlib.util.spec_from_file_location(path.name.replace(".", "_"), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return [entity for entity in module.generate_launch_description().entities
            if isinstance(entity, DeclareLaunchArgument)]


def _includes(context, robot_arguments, motive_arguments):
    """sim_robot (or sim_all) with this launch's values for its arguments, and the fake Motive if asked."""
    def forward(names):
        return {name: LaunchConfiguration(name) for name in names}.items()

    if LaunchConfiguration("robot").perform(context).strip() == "all":
        if any(LaunchConfiguration(name).perform(context).strip() for name in ("tool", "tool_left", "tool_right")):
            raise ValueError("robot:=all puts each robot's own tools on (model.yaml): simulate a robot on its own to "
                             "choose its tools or the marker")
        actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(SIM_ALL)))]
    else:
        actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(SIM_ROBOT)),
                                            launch_arguments=forward(robot_arguments))]
    if LaunchConfiguration("fake_motive").perform(context).lower() == "true":
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(FAKE_MOTIVE)),
                                                launch_arguments=forward(motive_arguments)))
    return actions
