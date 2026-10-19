"""
The process plugin: an authored process, the selected movement, and a view of its state.

  plugin.py        the plugin: owns the selected movement and serves it to other plugins (`selection`)
  loading.py       reading a process folder; an old compas_fab export is converted first
  scene_bodies.py  the process scene at a movement as live scene bodies, and the process <-> live id maps
  drawing.py       one process state in viser, from forward kinematics
  colors.py        colours of process bodies

The process itself is the core's (`scaffolding_assembly_core.process`); what this plugin offers others is
`shared/process_contract.py`.
"""

from . import plugin  # noqa: F401  registers ProcessPlugin
