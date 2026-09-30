"""
The cell: an authored design, the selected movement, and a view of its state.

  design.py    the design as authored: robot cells, the schedule, every
               movement's cell state. Plain data, loaded from a folder.
  drawing.py   a robot cell state in viser, from forward kinematics.
  plugin.py    the plugin: owns the selected step, steps through it, and
               answers what other plugins ask about it.

Importing this package registers CellPlugin.
"""

from .design import Design, ScheduledAction, Step, load_design
from .plugin import CellPlugin

__all__ = ["CellPlugin", "Design", "ScheduledAction", "Step", "load_design"]
