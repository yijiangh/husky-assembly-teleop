"""
The assembly guide: walks the operator through each action's parts, Prepare → Check → Go, on the planners.

  plugin.py  the plugin: follows the process plugin's selection, plans through base_planner and arm_planner, previews,
             commits on Go, and moves the selection on once a part is done
  parts.py   an action's parts, each handed to one layer, and when a part counts as done
"""

from . import plugin  # noqa: F401  registers AssemblyGuidePlugin
