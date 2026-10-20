"""
The motion planner: one worker thread that plans process movements, as a service for other plugins.

  plugin.py   the plugin: queues requests on its worker, answers with asyncio futures, shows what runs
  work.py     what the worker runs for a request: a plan, an IK goal check, a collision check

What it offers (`MotionPlannerContract`) and its requests are `shared/motion_planner_contract.py`.
"""

from . import plugin  # noqa: F401  registers MotionPlannerPlugin
