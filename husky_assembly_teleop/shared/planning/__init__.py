"""
The planner plugins' shared base; registers no plugin itself.

  path.py    TimedPath: states over time, sampled for the preview; `timing` of a pose path at given speeds
  search.py  RRT-Connect with abort and straight-move shortcut; PlanResult
  panel.py   PlannerPlugin: the plan/play/clear/commit panel, worker thread, ghosts

! No collision world here: each planner brings its own.
"""
