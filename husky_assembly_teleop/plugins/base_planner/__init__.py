"""
The base planner: a target for a husky base, a path to it, and committing it.

  path.py            the timed floor path, the turn-drive-turn steer, arcs, and where a base really drives; plain math
  world.py           the private PyBullet world the algorithms check collisions in (worker thread)
  follow.py          where the base really goes when the onboard follower drives a path, simulated
  algorithms/        the selectable planning algorithms, each with its settings and debug drawing
  debug_view.py      draws an algorithm's debug record on the floor and in the panel
  planner_plugin.py  the headless planner `base_planner`: plans on request, commits to the follower, records
  panel_plugin.py    its panel `base_planner_panel`: target input, algorithm choice, floor drawing
  tasks.py           the experiment's standard tasks: starts, targets and virtual obstacles
  recording.py       saving plan attempts and their runs to the Drive

The experiment's reports are in `experiments/base_planner/`.
"""

from . import panel_plugin, planner_plugin  # noqa: F401  registers BasePlannerPanel and BasePlannerPlugin
