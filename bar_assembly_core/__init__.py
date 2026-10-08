"""
The shared core of bar assembly, used alike by the monitor, the Rhino plugin and the planners.

Layers, each importing only the ones above it:
    geometry, ids     poses, shapes, ids                                          numpy, scipy, trimesh
    urdf, kinematics  URDF/SRDF files, UR conventions; forward kinematics         kinematics: + yourdfpy
    robot, scene      RobotModel, RobotObject; Body, Scene: the world at one moment
    design            the file format; design.scenes turns a design into scenes   design.scenes: + yourdfpy
    mirrors           a scene in PyBullet or compas_fab, for planners             + pybullet, compas_fab
    legacy            the old compas_fab export (optional)                       + compas, rs_data_structure

- ! Imports nothing from `husky_assembly_teleop`, ROS or viser: the package moves to its own repository as a copy.
- ! Runs on Python 3.9 (Rhino 8).
"""

from __future__ import annotations
