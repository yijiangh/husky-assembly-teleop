"""
The planner plugins' shared base, and the foundations of planning a process on the real robots; registers no plugin.

  path.py       TimedPath: states over time, sampled for the preview; `timing` of a pose path at given speeds
  search.py     RRT-Connect with abort and straight-move shortcut; PlanResult
  panel.py      PlannerPlugin: the plan/play/clear/commit panel, worker thread, ghosts
  joints.py     arm joint names and sides, URDF limits, goals unwrapped by whole turns next to a start
  movement.py   a process movement classified by its parts: routine, arms, goal, the drives really sent
  steps.py      an action's movements grouped into the operator's steps
  scene_for.py  a process step's planning scene on the real robots; unmeasured robots from a belief or the design
  collision.py  checks on a synced compas_fab mirror: a state, a path swept between waypoints; world-frame FK
  ik.py         exact ssik IK at the live base, nearest collision-free branch first; PyBullet IK as fallback

- ! No collision world here: each planner brings its own mirror. collision.py and ik.py use it on the planner's
  worker thread, after `sync`; the other modules are pure.
- ! Only panel.py imports viser: the others never import it, nor `plugins/`.
"""
