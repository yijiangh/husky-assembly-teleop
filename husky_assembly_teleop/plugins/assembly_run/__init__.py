"""
The assembly run: which actions of the loaded process are done, where the robots were left, and the run's settings.

  plugin.py   the plugin: opens, continues and writes the process's run, marks and reopens actions
  panel.py    its panel: the run, the action list, the buttons, the prompt, the settings

The run format and its rules are `shared/runs.py`; what it offers other plugins is `shared/assembly_run.py`.
"""

from . import plugin  # noqa: F401  registers AssemblyRunPlugin
