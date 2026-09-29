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

! One rule for missing data, in every state dataclass here and in WorldState:

    A measured value is None until it has been measured, and after that keeps
    the last measured value.

  No stand-ins that look like measurements: no zero position, no identity
  quaternion, no empty string, no 0 count, no 0.0 timestamp. A forgotten check
  then fails loudly (TypeError) instead of quietly using a made-up value --
  which is how the old code teleported the planning robot to the origin.

  Judgements we make ourselves (`tracked`, `is_executing`, `moving`) are plain
  bools, False until shown true. `tracked` implies the pose is set, so code
  that checks `tracked` first needs no further None check. In per-item
  collections (`joint_positions`, `controllers`) a missing key means "not
  measured".
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
