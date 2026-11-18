"""
Stop recovery: after an e-stop or a protective stop, move an arm that held something back to where it stood before.

  pose_log.py  every rest pose and every stop of each robot, kept in a day file under ~/.ros/stop_recovery
  plugin.py    the plugin: logs the poses, shows the last few per robot, and moves the robot back to the one picked
"""

from . import plugin  # noqa: F401  registers StopRecoveryPlugin
