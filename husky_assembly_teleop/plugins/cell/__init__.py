"""
The cell: an authored design, the selected movement, and a view of its state.

  design.py    a schema 1 design (via design_io), its compas_fab cells, and
               every movement of the schedule as a step.
  drawing.py   a robot cell state in viser, from forward kinematics.
  plugin.py    the plugin: owns the selected step, steps through it, and
               answers what other plugins ask about it.

Importing this package registers CellPlugin.
"""

from .design import CellDesign, Step, load_design
from .plugin import CellPlugin

__all__ = ["CellDesign", "CellPlugin", "Step", "load_design"]
