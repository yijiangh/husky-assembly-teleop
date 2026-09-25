"""
The cell: the design, where we are in it, and the geometry that implies.

Split in two because they are different kinds of thing:

  assembly.py   the design: elements and the order they go in. Plain data.
  plugin.py     the plugin: owns the selected index and the compas_fab planning
                session, and answers the questions other plugins ask.

Importing this package registers CellPlugin, which is how plugin discovery finds
it.
"""

from .assembly import Assembly, AssemblyElement, AssemblyStep
from .plugin import CellPlugin

__all__ = ["Assembly", "AssemblyElement", "AssemblyStep", "CellPlugin"]
