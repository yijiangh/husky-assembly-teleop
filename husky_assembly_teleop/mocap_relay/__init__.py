"""
The mocap relay: OptiTrack's NatNet stream into ROS topics, one relay per ROS domain.

  node.py         the ROS node (`husky_mocap_relay`), started by launch/mocap.launch.py
  log_text.py     wrapped lines and boxes for its log

Which base calibration each robot's rigid body gets is `world/calibration_sessions.py`, shared with the monitor.

! Its own process, never inside the monitor: robots follow paths onboard on its poses, also while the monitor is down.
Moved from crl-husky, which keeps the messages (`crl_husky_msgs`) and the robots' config. Docs: doc/mocap_relay.md.
"""
