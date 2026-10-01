"""
The base planner: a target for a husky base, a path to it, and committing it.

  path.py     a timed floor path and the turn-drive-turn steer. Plain math.
  planner.py  the search in a private PyBullet world (worker thread).
  plugin.py   the plugin: target input, floor line, and the shared planner panel.

The flow (plan, preview, stale, commit) is shared with the arm planner in
planning/panel.py; the target input (fields plus drag gizmo) in ui/pose_input.py.

Importing this package registers BasePlannerPlugin.
"""

from .path import BasePath
from .plugin import BasePlannerPlugin

__all__ = ["BasePath", "BasePlannerPlugin"]
