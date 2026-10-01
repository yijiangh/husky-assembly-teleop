"""
The arm planner: a joint target for one arm, a collision-free path to it, and committing it.

  planner.py  the joint-space search and its compas_fab planning worlds (worker thread),
              and the timed path. No viser.
  plugin.py   the plugin: target sliders, arm choice, and the shared planner panel.

The flow (plan, preview, stale, commit) is shared with the base planner in
planning/panel.py.

Importing this package registers ArmPlannerPlugin.
"""

from .planner import ArmPath, plan_arm
from .plugin import ArmPlannerPlugin

__all__ = ["ArmPath", "ArmPlannerPlugin", "plan_arm"]
