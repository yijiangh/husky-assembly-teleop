"""
The arm planner: a joint target for one arm, a collision-free path to it, and committing it.

  planner.py  the joint-space search, its compas_fab planning worlds (worker thread) and the timed path
  plugin.py   the plugin: target sliders, arm choice, and the shared `planning.panel` flow
"""

from . import plugin  # noqa: F401  registers ArmPlannerPlugin
