"""
The planner plugins' shared base; registers no plugin itself.

  path.py    TimedPath: states over time, sampled for the preview; `timing` of a pose path at given speeds
  search.py  RRT-Connect with abort and straight-move shortcut; PlanResult
  service.py PlannerService: a headless planner plugin; Plan and the PlannerPluginApi other plugins use
  arm.py     ArmPlanRequest and ArmPlannerPluginApi: what the arm planner plans for and offers
  base.py    BasePlanRequest and BasePlannerPluginApi: what the base planner plans for and offers
  panel.py   PlannerPanel: the plan/play/clear/commit panel on that API, ghosts

! No collision world here: each planner brings its own.
"""
