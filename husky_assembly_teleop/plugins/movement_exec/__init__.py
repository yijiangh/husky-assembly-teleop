"""
The executor: runs one step of the assembly on its robot, each part after the operator's Confirm, recorded in the run.

  plugin.py     the plugin: takes requests, holds the robot while it works, records every part
  sequences.py  how each kind of part runs: a motion, a Robotiq move, a scaffolding grasp, a step by hand
  panel.py      its panel and the phone's Exec tab: what runs, the prompt, Confirm, Cancel, the dry run

What it offers (`MovementExecPluginApi`) and the requests are `shared/movement_exec.py`; timing, sending, tools, the
prompt, the lease and staleness are `shared/execution/`.
"""

from . import plugin  # noqa: F401  registers MovementExecPlugin
