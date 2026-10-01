"""
What every planner plugin shares, whatever it plans in and with.
  path.py    TimedPath: states over time, sampled for the preview
  search.py  running birrt: abort, straight-move shortcut, corners; PlanResult
  panel.py   PlannerPlugin: the plan/play/clear/commit panel, worker thread, ghosts

! No PyBullet, no compas_fab here. A planner brings its own collision world (a mirror
  from world/mirrors), its own target input and its own PyBullet window toggle.
"""
