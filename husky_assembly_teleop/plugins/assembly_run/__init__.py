"""
The assembly run: which actions of the loaded process are done, where the robots were left, and the run's settings.

  plugin.py   the plugin: its requests and hooks, opening and continuing the process's run
  writes.py   the plugin's writing half (mixed in): marking, starting runs, the current action, refusals, the writer
  panel.py    its panel: the run, the action list, the buttons, the prompt, the settings

The run format is `shared/runs/runs.py`; what it offers other plugins, and the rules it marks by, are
`shared/assembly_run_contract.py`.
"""

from . import plugin  # noqa: F401  registers AssemblyRunPlugin
