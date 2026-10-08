"""
The design file format, schema 2 (doc/design_format.md): the `Design` in memory, and reading, writing and validating.

`design.scenes` turns a design into scenes (`Design.scene_at`); it needs yourdfpy, so it is not imported here.

! Runs on Python 3.9 (Rhino 8) with numpy, scipy and trimesh only.
"""

from __future__ import annotations

from .carry import assumed_joints, assumed_start_all
from .read import read
from .types import (ACTION_TYPES, BODY_PREFIXES, CONTROLLERS, ENDS_ON, PATHS, Action, ActionType, BodySpec, Carried,
                    Design, DesignError, LineSpec, Movement, Producer, RobotSpec, RobotState, SchemaMismatch, State,
                    Target, ToolState, Writer)
from .validate import validate
from .version import LIBRARY, SCHEMA, writer_info
from .vocabulary import ON, TOOL_CHANNELS
from .write import content_hash, write

__all__ = [
    "ACTION_TYPES", "BODY_PREFIXES", "CONTROLLERS", "ENDS_ON", "LIBRARY", "ON", "PATHS", "SCHEMA", "TOOL_CHANNELS",
    "Action", "ActionType", "BodySpec", "Carried", "Design", "DesignError", "LineSpec", "Movement", "Producer",
    "RobotSpec", "RobotState", "SchemaMismatch", "State", "Target", "ToolState", "Writer", "assumed_joints",
    "assumed_start_all", "content_hash", "read", "validate", "write", "writer_info",
]
