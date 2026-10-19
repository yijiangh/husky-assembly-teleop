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
  arm.py        one arm's RRT-Connect on a synced mirror, the path timed at the preview speed
  single_arm.py one arm's straight tool0 move: exact ssik IK at every step; compas_fab's cartesian planner as fallback
  tamp_route.py one process movement planned by its routine: tamp's dual-arm planners, the one-arm search or line
  route_search.py  the searches, checks and results a route is built of
  route_result.py  RouteResult, what planning one movement gave; pose error and line goals

- ! No collision world here: each planner brings its own mirror. collision.py, ik.py, arm.py, single_arm.py and
  the route use it on the planner's worker thread, after `sync`; the other modules are pure.
- ! Only panel.py imports viser: the others never import it, nor `plugins/`.
"""
