"""
The process plugin: an authored process, the selected movement, and a view of its state.

  steps.py     a process (scaffolding_assembly_core.process) and every movement of its schedule as a step
  drawing.py   one process state in viser, from forward kinematics
  plugin.py    the plugin: owns the selected step and serves it to planner plugins
"""

from . import plugin  # noqa: F401  registers ProcessPlugin
