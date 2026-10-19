"""
Offline analysis of base experiment runs (`scripts/analyze_base_exp.py`).

Pandas and matplotlib; the plugin never imports it.

  replay.py       replaying recorded commands through a motion model, against where mocap saw the robot
  identify.py     the motion model fitted to recorded runs
  common.py       the runs of an experiment (path and constant-command runs), robot and scenario labels, tables
  controller.py   summary.md: how well each scenario's controller follows the paths (path runs)
  robot_model.py  robot_model.md: how the robot responds to commands, the model fitted to it, and checks (path runs)
  commands.py     constant_commands.md: how that response changes with speed and turn rate (constant-command runs)
"""
