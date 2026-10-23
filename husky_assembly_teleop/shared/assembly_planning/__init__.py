"""
Planning a process on the real robots, for the assembly's planners; registers no plugin itself.

The process as steps, planning scenes, routines, IK and collision checks on a planner's mirrors, and the tamp route.

  process_steps.py  the process as steps (`LoadedProcess`, `Step`): every movement in schedule order
  joints.py         arm joint names and sides, URDF limits, goals unwrapped by whole turns next to a start
  movement.py       a process movement classified by its parts: routine, arms, goal, the drives really sent
  labels.py         a movement as the operator knows it: its old monitor name, the action flowchart's kind and
                    colours, one sentence on what it does
  steps.py          an action's movements grouped into the operator's steps
  scene_for.py      a process step's planning scene on the real robots; unmeasured robots from a belief or the design
  collision.py      checks on a synced compas_fab mirror: a state, a path swept between waypoints; world-frame FK
  ik.py             exact ssik IK at the live base, nearest collision-free branch first; PyBullet IK as fallback
  arm.py            one arm's RRT-Connect on a synced mirror, the path timed at the preview speed
  single_arm.py     one arm's straight tool0 move: exact ssik IK at every step; compas_fab's cartesian planner as
                    fallback
  tamp_route.py     one process movement planned by its routine: tamp's dual-arm planners, the one-arm search or line
  route_search.py   the searches, checks and results a route is built of
  route_result.py   RouteResult, what planning one movement gave; GoalCheck, its goal by IK; pose error and line goals
  mirrors.py        MirrorPool: a planner's compas_fab mirrors, one per robot, and the PyBullet window

- ! No collision world here: each planner brings its own mirrors (`MirrorPool`). collision.py, ik.py, arm.py,
  single_arm.py and the route use them on the planner's worker thread, after `sync`; the other modules are pure.
- ! Never imports viser, `plugins/`, or a contract file except for type hints.
"""
