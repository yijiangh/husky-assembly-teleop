"""
The robot control panel: each robot's state, and basic manual control of its base, arms and tools.

  arm_panel.py  an arm's inputs: joint targets, a Cartesian target with a force, and its tool's buttons
  dpad.py       the base's D-pad of hold buttons, and its speed slider
  markers.py    the 3D target, TCP and force markers drawn while the Cartesian controller runs
  status.py     the CTRL section that switches controllers, and the status readouts
  plugin.py     the plugin: builds the tabs, streams base twists, runs arm moves, draws state
"""

from . import plugin  # noqa: F401  registers RobotControlPlugin
