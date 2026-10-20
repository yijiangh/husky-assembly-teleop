"""
What is in the world, and where.

  measured.py   WorldState: what sensors report (robots, tracked objects)
  checks.py     Check and its GOOD/WARN/BAD levels: a status verdict as data
  mocap.py      how a mocap fix is judged, for bases and objects alike
  freeze.py     freezes: whether what a task relies on still stands where it was measured
  clearance.py  clearances: stop a driven robot that comes too close to anything in the scene
  base_collision.py  what a robot base comes near, in a PyBullet mirror of the scene
  follower.py   how the onboard path follower's reports are judged: running, and how a sent path goes
  kinematics.py each robot's link poses, fixed once per tick
  scene.py      collision objects we don't measure, and the per-tick snapshot
  lab.py        the lab's obstacles and floor border as data, and their files
  model_check.py  whether two models of one robot agree (planned and real, URDF and UR calibration)
  marker_fit.py   fitting known marker layouts to raw mocap markers
"""
