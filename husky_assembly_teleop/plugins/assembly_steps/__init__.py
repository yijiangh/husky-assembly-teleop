"""
The operator's step panel: the run's current step, its goal checked, its motion planned and previewed, then Exec.

  plugin.py   the plugin: follows the run's current step, checks goals, plans through motion_planner, hands Exec to
              movement_exec, moves on to the next step
  panel.py    its panel: the step, Prev and Next, Check goal, the path, Exec, the motion time, Mark done
"""

from . import plugin  # noqa: F401  registers AssemblyStepsPlugin
