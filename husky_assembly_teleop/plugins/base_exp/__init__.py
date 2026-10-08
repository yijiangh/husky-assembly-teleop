"""
The base experiment: test paths for the onboard pure pursuit follower, sent by hand or automated, shown and recorded.

  templates.py  the test paths: poses in the start frame, placed and timed; plain math
  plan.py       a path ready to send (Plan) and building one from settings; plain math
  auto.py       which paths automated runs drive (standard or random paths), and clearance checks in a PyBullet mirror
  twist.py      constant commands: the command grid (v, ω), driven open loop, for how the response changes with it
  record.py     a run's setup (sim or real, controller) read from ROS, and its recording folder
  report.py     a run's experiment.json and overview.png; matplotlib, no ROS
  markers.py    the 3D view: path lines, trail, the follower's points
  plugin.py     the plugin: panel, runs, automation and guard
  identify.py, replay.py  how the robot responds to commands, and how well a model predicts it; plain numpy
  analysis/     offline reports over an experiment's runs (scripts/analyze_base_exp.py)
"""

from . import plugin  # noqa: F401  registers BaseExperimentPlugin
