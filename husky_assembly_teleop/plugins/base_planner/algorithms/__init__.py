"""
The base planner's algorithms, offered in its Algorithm dropdown.

  base.py      what every algorithm provides (`Algorithm`), and `Debug`, the record of a search's inner workings
  rrt_arcs.py  RRT-Connect, corners rounded into arcs, checked for a range of turning centres
  lattice.py   hybrid A* with turn-and-drive moves down to 2.5 degrees, for tight spots and narrow passages

* To add one: subclass `Algorithm` in a file of its own here, and add it to ALGORITHMS.
"""

from .base import Algorithm, Debug, Layer
from .lattice import Lattice
from .rrt_arcs import RrtArcs

#: Every algorithm, the first being the default.
ALGORITHMS: tuple[type[Algorithm], ...] = (RrtArcs, Lattice)

__all__ = ["ALGORITHMS", "Algorithm", "Debug", "Layer"]
