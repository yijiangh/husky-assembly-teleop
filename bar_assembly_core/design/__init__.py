"""
The design file format, schema 2 (doc/design_format.md): the `Design` in memory, and reading, writing and validating.

Not imported here: `design.scenes` (scenes from a design, `Design.scene_at`), `design.plan_check` (the plan checks
B1–B14) and `design.solutions` (planner results), so `import design` stays free of scenes and robots.

! Runs on Python 3.9 (Rhino 8) with numpy, scipy and trimesh only.
"""

from __future__ import annotations

from .carry import assumed_joints, assumed_start_all
from .read import read
from .types import (ACTION_TYPES, BODY_PREFIXES, CONTROLLERS, ENDS_ON, PATHS, Action, ActionType, BodySpec, Design,
                    DesignError, Holder, LineSpec, Movement, Producer, RobotSpec, RobotState, SchemaMismatch, State,
                    Target, ToolState, Writer)
from .validate import validate
from .version import LIBRARY, SCHEMA, writer_info
from .vocabulary import GRIP, TOOL_KINDS
from .write import content_hash, write

__all__ = [
    "ACTION_TYPES", "BODY_PREFIXES", "CONTROLLERS", "ENDS_ON", "GRIP", "LIBRARY", "PATHS", "SCHEMA", "TOOL_KINDS",
    "Action", "ActionType", "BodySpec", "Design", "DesignError", "Holder", "LineSpec", "Movement", "Producer",
    "RobotSpec", "RobotState", "SchemaMismatch", "State", "Target", "ToolState", "Writer", "assumed_joints",
    "assumed_start_all", "content_hash", "read", "validate", "write", "writer_info",
]
