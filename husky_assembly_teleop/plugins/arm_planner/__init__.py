"""
The arm planner: a collision-free path for one arm to a joint target, sent as one trajectory.

  planner.py  the joint-space search, its compas_fab planning worlds (worker thread) and the timed path
  plugin.py   `arm_planner`: the headless planner (`ArmPlannerPluginApi`)
  panel.py    `arm_planner_panel`: target sliders, arm choice and the shared `planning.panel` flow, on that API
"""

from . import panel, plugin  # noqa: F401  registers ArmPlannerPlugin and ArmPlannerPanel
