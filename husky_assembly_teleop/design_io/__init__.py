"""
Read, write and validate design folders (doc/design_format.md), and convert them to and from compas_fab.

`design_io.compas_fab` is imported on its own, so compas stays optional.

! Imports nothing from `husky_assembly_teleop` outside this package: it moves to its own
  repository later. Runs on Python 3.9 (Rhino 8).
"""

from __future__ import annotations

from .carry import assumed_joints, assumed_start_all
from .geometry import (BoxShape, CylinderShape, Geometry, Shape, TriMesh, box_geometry, cylinder_geometry,
                       shape_mesh)
from .pose import ID_PATTERN, Pose, check_id, compose
from .read import read
from .types import (ACTION_TYPES, BODY_PREFIXES, CONTROLLERS, MOVEMENT_TYPES, Action, ActionType, Attached,
                    BodySpec, Controller, Design, DesignError, LinkPose, Movement, MovementType, RobotSpec,
                    RobotState, SchemaMismatch, State, Target, ToolSpec, Writer, link_id, split_link_id,
                    world_pose)
from .validate import validate
from .version import LIBRARY, SCHEMA, writer_info
from .write import write

__all__ = [
    "ACTION_TYPES", "BODY_PREFIXES", "CONTROLLERS", "ID_PATTERN", "LIBRARY", "MOVEMENT_TYPES", "SCHEMA",
    "Action", "ActionType", "Attached", "BodySpec", "BoxShape", "Controller", "CylinderShape", "Design",
    "DesignError", "Geometry", "LinkPose", "Movement", "MovementType", "Pose", "RobotSpec", "RobotState",
    "SchemaMismatch", "Shape", "State", "Target", "ToolSpec", "TriMesh", "Writer", "assumed_joints",
    "assumed_start_all", "box_geometry", "check_id", "compose", "cylinder_geometry", "link_id", "read",
    "shape_mesh", "split_link_id", "validate", "world_pose", "write", "writer_info",
]
