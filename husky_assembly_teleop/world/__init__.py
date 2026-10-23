"""
What is in the world, and where.

  measured.py   WorldState: what sensors report (robots, tracked objects)
  checks.py     Check and its GOOD/WARN/BAD levels: a status verdict as data
  mocap.py      how a mocap fix is judged, for bases and objects alike
  tcp.py        how the TCP an arm reports is judged against tool0 by the robot's kinematics
  freeze.py     freezes: whether what a task relies on still stands where it was measured
  clearance.py  clearances: stop a driven robot that comes too close to anything in the scene
  base_collision.py  what a robot base comes near, in a PyBullet mirror of the scene
  collision_hit.py   CollisionHit: what a collision check hit, with the scene as checked, for the collisions plugin
  follower.py   how the onboard path follower's reports are judged: running, and how a sent path goes
  kinematics.py each robot's link poses, fixed once per tick
  scene.py      collision objects we don't measure, and the per-tick snapshot
  lab.py        the lab's obstacles and floor border as data, and their files
  model_check.py  whether two models of one robot agree (planned and real, URDF and UR calibration)
  calibration_sessions.py  base calibrations: crl-husky's default or a Drive session, and which one the relay applies
  marker_fit.py   fitting known marker layouts to raw mocap markers
"""
