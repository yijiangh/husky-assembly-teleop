"""
Shared base for the planner plugins; registers no plugin itself.

  path.py    TimedPath: states over time, sampled for the preview
  search.py  RRT-Connect with abort and straight-move shortcut; PlanResult
  panel.py   PlannerPlugin: the plan/play/clear/commit panel, worker thread, ghosts

! No PyBullet or compas_fab here: each planner brings its own collision world.
"""
