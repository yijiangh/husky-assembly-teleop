"""
The only package that talks to a robot.

Everything above reads each part's state and calls its command methods;
nothing above knows a topic name. One module per part:

  robot.py               HuskyRobotInterface, RobotState -- composes the parts
  base.py                the mobile base: mocap pose, cmd_vel
  arm.py                 one UR arm: joints, TCP, wrench, IO, commands
  end_effectors.py       one class per tool kind, picked by configuration
  controller_manager.py  the base and every arm each have one; handled alike

! Configuration decides which parts exist (arms, mounted tools). Anything that
  changes while running, such as the active controller, is tracked in state.
"""

from .arm import ArmInterface, ArmState
from .base import BaseInterface, BaseState
from .controller_manager import ControllerManagerInterface, ControllerManagerState
from .end_effectors import EndEffector, RobotiqGripper, ScaffoldingV1, ScaffoldingV3
from .robot import HuskyRobotInterface, RobotState

__all__ = [
    "ArmInterface", "ArmState",
    "BaseInterface", "BaseState",
    "ControllerManagerInterface", "ControllerManagerState",
    "EndEffector", "RobotiqGripper", "ScaffoldingV1", "ScaffoldingV3",
    "HuskyRobotInterface", "RobotState",
]
