"""
The arm planner: a collision-free path for one arm to a joint target, sent as one trajectory.

  planner.py         the joint-space search, its compas_fab planning worlds (worker thread) and the timed path
  planner_plugin.py  `arm_planner`: the headless planner (`ArmPlannerContract`)
  panel_plugin.py    `arm_planner_panel`: target sliders and arm choice on `PlannerPanel`, through that contract
"""

from . import panel_plugin, planner_plugin  # noqa: F401  registers ArmPlannerPlugin and ArmPlannerPanel
