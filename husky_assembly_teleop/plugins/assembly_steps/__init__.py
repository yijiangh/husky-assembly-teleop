"""
The operator's steps in the Assembly tab: the current step, its goal checked, its motion planned and previewed, Exec.

  plugin.py   the plugin: follows the run's current step, checks goals, plans through motion_planner, hands Exec to
              movement_exec, moves on to the next step
  panel.py    its part of the Assembly tab: the step, Live IK, Plan, the preview, Exec, the motion time, the settings
  goal.py     Live IK: the step's goal solved by IK through motion_planner, kept per step
  step_panel_base.py  PlannerPlugin, its base class: its own copy of a planner panel
"""

from . import plugin  # noqa: F401  registers AssemblyStepsPlugin
