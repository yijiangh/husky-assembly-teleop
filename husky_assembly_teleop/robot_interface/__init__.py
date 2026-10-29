"""
The only package that talks to a robot; callers read state and call commands, never topics.

  robot.py               HuskyRobotInterface, RobotState: composes the parts
  base.py                the mobile base: mocap pose, cmd_vel, onboard path follower
  arm.py                 one UR arm: joints, TCP, wrench, IO, commands
  end_effectors.py       one class per tool kind, picked by configuration
  controller_manager.py  one per base and per arm
  mocap.py               mocap samples for bases and tracked objects
  mocap_relay.py         the mocap relay itself: the base calibrations it applies, Motive's definitions, reloading
  mocap_definitions.py   Motive's rigid body definitions and cameras as the relay announces them, and their changes
  mocap_state.py         what a mocap-tracked body stores, without ROS (world/mocap.py judges it)
  simulated_parts.py     which parts of a robot announce themselves as simulated
  soft_stop.py           the soft-stop latch: refuses commands that could move a stopped robot
  robot_lease.py         robot leases: motion commands and controller switches need the robot's lease
  refusal.py             Refusal: a command a part did not send and why, for the button's status chip
  frames.py              poses between the world, the husky and each arm
  ur_frames.py           the UR frame convention the arms' Cartesian commands rely on
  tool_step.py           small tool0 moves in tool0's own frame, solved into joint angles on the URDF
  stream_stats.py        how well a high-rate stream gets through: rate, gaps, delay
  connections.py         the ROS entities one interface created, so a reconnect can recreate them
  event.py               Event: an interface's messages and sent commands, handed to listeners such as recordings

! A measured value is None until measured, then keeps the last value. Never use stand-ins (zero
  position, identity quaternion), so a forgotten check fails with TypeError instead of using made-up data.

- Our own judgements (`tracked`, `is_executing`, `moving`) are bools, False until shown true.
- In per-item collections (`joint_positions`, `controllers`) a missing key means "not measured".
"""
