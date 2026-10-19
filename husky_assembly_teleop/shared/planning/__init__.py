"""
The planner plugins' shared base; registers no plugin itself.

  path.py    TimedPath: states over time, sampled for the preview; `timing` of a pose path at given speeds
  search.py  RRT-Connect with abort and straight-move shortcut; PlanResult
  service.py PlannerService: a headless planner plugin; Plan and the PlannerPluginApi other plugins use
  panel.py   PlannerPanel: the plan/play/clear/commit panel on that API, ghosts (and the older PlannerPlugin)

! No collision world here: each planner brings its own.
"""
