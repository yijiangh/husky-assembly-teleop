"""
The motion planner: one worker thread that plans process movements, as a service for other plugins.

  plugin.py   the plugin: queues requests on its worker, answers with asyncio futures, shows what runs

What it offers (`MotionPlannerPluginApi`), the requests and the work are `shared/planning/service.py`.
"""

from . import plugin  # noqa: F401  registers MotionPlannerPlugin
