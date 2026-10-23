"""
The robot control panel: each robot's state, and basic manual control of its base, arms and tools.

  arm_panel.py  an arm's inputs: joint jog buttons, a Cartesian target with a force, and its tool's buttons
  markers.py    the 3D target, TCP and force markers and the target gizmo, shown while the Cartesian controller runs
  status.py     the CTRL section that switches controllers, and the status readouts
  teleop_lease.py  the robots it leases while the operator commands them, given back once idle
  plugin.py     the plugin: builds the tabs, streams base twists, runs arm moves, draws state
"""

from . import plugin  # noqa: F401  registers RobotControlPlugin
