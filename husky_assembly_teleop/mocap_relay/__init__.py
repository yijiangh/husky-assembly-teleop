"""
The mocap relay: OptiTrack's NatNet stream into ROS topics, one relay per ROS domain.

  node.py         the ROS node (`husky_mocap_relay`), started by launch/mocap.launch.py
  log_text.py     wrapped lines and boxes for its log
  sim_motive.py   a fake Motive for simulations: every simulated body's true pose, with Motive's imperfections
  body_definitions.py  Motive's rigid body definitions from NatNet, and how a body's definition changed
  natnet/         OptiTrack's NatNet SDK client (NaturalPoint, Apache 2.0)

Which base calibration each robot's rigid body gets is `world/calibration_sessions.py`, shared with the monitor.

! Its own process, never inside the monitor: robots follow paths onboard on its poses, also while the monitor is down.
Moved from crl-husky, which keeps the messages (`crl_husky_msgs`) and the robots' config. Docs: doc/mocap_relay.md.
"""
