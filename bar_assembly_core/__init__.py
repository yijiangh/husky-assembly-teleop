"""
The shared core of bar assembly: design files, the scene of one moment, and its mirrors in collision backends.

`design_io` reads and writes designs; the monitor, the Rhino plugin and the planners use the same package.

- ! Imports nothing from `husky_assembly_teleop`, ROS or viser: the package moves to its own repository as a copy.
- ! Runs on Python 3.9 (Rhino 8). Core dependencies: numpy, scipy, trimesh (`requirements.txt` lists the extras).
"""

from __future__ import annotations
