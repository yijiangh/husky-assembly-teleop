"""
The base planner: a target for a husky base, a path to it, and committing it.

  path.py     a timed floor path, and the (stub) straight-line planner.
              Plain math, no viser or PyBullet.
  plugin.py   the plugin: target input, plan, time slider, ghost, commit.

The target input (fields plus drag gizmo) is shared in ui/pose_input.py.

Importing this package registers BasePlannerPlugin.
"""

from .path import BasePath, plan_straight_line
from .plugin import BasePlannerPlugin

__all__ = ["BasePath", "BasePlannerPlugin", "plan_straight_line"]
