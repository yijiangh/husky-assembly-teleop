"""
The only package that talks to a robot; callers read state and call commands, never topics.

  robot.py               HuskyRobotInterface, RobotState: composes the parts
  base.py                the mobile base: mocap pose, cmd_vel, onboard path follower
  arm.py                 one UR arm: joints, TCP, wrench, IO, commands
  end_effectors.py       one class per tool kind, picked by configuration
  controller_manager.py  one per base and per arm
  mocap.py               mocap samples for bases and tracked objects

! A measured value is None until measured, then keeps the last value. Never use stand-ins (zero
  position, identity quaternion), so a forgotten check fails with TypeError instead of using made-up data.

- Our own judgements (`tracked`, `is_executing`, `moving`) are bools, False until shown true.
- In per-item collections (`joint_positions`, `controllers`) a missing key means "not measured".
"""
