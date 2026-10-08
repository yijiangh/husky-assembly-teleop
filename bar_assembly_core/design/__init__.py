"""
The design file format (doc/design_format.md): the `Design` in memory, and reading, writing and validating folders.

`design.scenes` turns a design into scenes (`Design.scene_at`); it needs yourdfpy, so it is not imported here.

! Runs on Python 3.9 (Rhino 8) with numpy, scipy and trimesh only.
"""

from __future__ import annotations

from .carry import assumed_joints, assumed_start_all
from .read import read
from .types import (ACTION_TYPES, BODY_PREFIXES, CONTROLLERS, MOVEMENT_TYPES, Action, ActionType, Attached, BodySpec,
                    Controller, Design, DesignError, Movement, MovementType, RobotSpec, RobotState, SchemaMismatch,
                    State, Target, Writer)
from .validate import validate
from .version import LIBRARY, SCHEMA, writer_info
from .write import write

__all__ = [
    "ACTION_TYPES", "BODY_PREFIXES", "CONTROLLERS", "LIBRARY", "MOVEMENT_TYPES", "SCHEMA", "Action", "ActionType",
    "Attached", "BodySpec", "Controller", "Design", "DesignError", "Movement", "MovementType", "RobotSpec",
    "RobotState", "SchemaMismatch", "State", "Target", "Writer", "assumed_joints", "assumed_start_all", "read",
    "validate", "write", "writer_info",
]
