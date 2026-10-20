"""
The process plugin: an authored process, the selected movement, and a view of its state.

  plugin.py        the plugin: owns the selected movement and serves it to other plugins (`selection`)
  loading.py       reading a process folder; an old compas_fab export is converted first
  drawing.py       one process state in viser, from forward kinematics

The process itself is the core's (`scaffolding_assembly_core.process`); what this plugin offers others is
`shared/process_contract.py`. Its scene bodies and colours are `shared/process_scene/`, which the assembly's
planning scenes use too.
"""

from . import plugin  # noqa: F401  registers ProcessPlugin
