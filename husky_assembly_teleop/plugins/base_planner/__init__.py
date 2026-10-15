"""
The base planner: a target for a husky base, a path to it, and committing it.

  path.py        the timed floor path, the turn-drive-turn steer, arcs, and where a base really drives; plain math
  world.py       the private PyBullet world the algorithms check collisions in (worker thread)
  follow.py      where the base really goes when the onboard follower drives a path, simulated
  algorithms/    the selectable planning algorithms, each with its settings and debug drawing
  debug_view.py  draws an algorithm's debug record on the floor and in the panel
  plugin.py      the plugin: target input, algorithm choice, floor drawing, and the shared `planning.panel` flow
  tasks.py       the experiment's standard tasks: starts, targets and virtual obstacles
  recording.py   saving plan attempts and their runs to the Drive
  report.py      one run's report and one attempt's picture; matplotlib, no ROS
  analysis.py    an experiment's report over all its attempts (scripts/analyze_base_planner.py), with figures.py
"""

from . import plugin  # noqa: F401  registers BasePlannerPlugin
