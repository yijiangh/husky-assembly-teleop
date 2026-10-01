"""
The base planner: a target for a husky base, a path to it, and committing it.

  path.py     the timed floor path and the turn-drive-turn steer; plain math
  planner.py  the search in a private PyBullet world (worker thread)
  plugin.py   the plugin: target input, floor line, and the shared `planning.panel` flow
"""

from . import plugin  # noqa: F401  registers BasePlannerPlugin
