"""
The process plugin: an authored process, the selected movement, and a view of its state.

  drawing.py   one process state in viser, from forward kinematics
  plugin.py    the plugin: owns the selected step and serves it to other plugins (`selection`)

A process and its steps as data are `shared/loaded_process.py`; what this plugin offers others is
`shared/process_contract.py`.
"""

from . import plugin  # noqa: F401  registers ProcessPlugin
