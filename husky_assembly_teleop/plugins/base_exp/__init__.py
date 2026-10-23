"""
The base experiment: test paths for the onboard pure pursuit follower, sent by hand or automated, shown and recorded.

  templates.py  the test paths: poses in the start frame, placed; plain math
  plan.py       a path ready to send (Plan) and building one from settings; plain math
  auto.py       which paths automated runs drive (standard or random paths), and clearance checks in a PyBullet mirror
  twist.py      constant commands: the command grid (v, ω), driven open loop, for how the response changes with it
  record.py     a run's setup (sim or real, controller) read from ROS, and its recording folder
  report.py     a run's experiment.json and overview.png; matplotlib, no ROS
  tracking.py   tracking errors measured in the monitor: the pose against the path sent, for every controller
  plugin.py     the plugin: panel, runs, automation and guard; its 3D view is ui/base_markers.py

The offline reports, and the motion model fitted to the runs, are in `experiments/base_exp/`.
"""

from . import plugin  # noqa: F401  registers BaseExperimentPlugin
