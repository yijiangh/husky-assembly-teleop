"""
What is in the world, and where.
  measured.py   WorldState: what sensors report (robots, tracked objects)
  mocap.py      how mocap samples are read and judged, for bases and objects alike
  kinematics.py each robot's link poses, fixed once per tick
  geometry.py   shapes of scene bodies: meshes and primitives
  scene.py      collision objects we don't measure, and the per-tick snapshot
  mirrors/      private copies of a snapshot in one collision backend each
"""
