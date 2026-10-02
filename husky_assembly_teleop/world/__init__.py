"""
What is in the world, and where.

  measured.py   WorldState: what sensors report (robots, tracked objects)
  checks.py     Check and its GOOD/WARN/BAD levels: a status verdict as data
  mocap.py      how a mocap fix is judged, for bases and objects alike
  kinematics.py each robot's link poses, fixed once per tick
  scene.py      collision objects we don't measure, and the per-tick snapshot
  mirrors/      private copies of a snapshot in one collision backend each
"""
