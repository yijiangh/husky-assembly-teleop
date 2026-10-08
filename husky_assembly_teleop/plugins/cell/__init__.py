"""
The cell: an authored design, the selected movement, and a view of its state.

  design.py    a design (bar_assembly_core.design) and every movement of its schedule as a step
  drawing.py   one cell state in viser, from forward kinematics
  plugin.py    the plugin: owns the selected step and serves it to planner plugins
"""

from . import plugin  # noqa: F401  registers CellPlugin
