"""
The observer: everything that happens while the monitor runs, recorded on this PC for a week, whoever does it.

  columns.py  what is recorded every tick: continuous values of each robot, tracked object and the relay's drift
  watch.py    the events: every switching state, commands sent, motions and their cause, streams lost, relay changes
  export.py   the local days kept, dropping old ones, and copying a time range out
  plugin.py   the plugin: the session's folder, chunks, the events file, and its panel with Copy to Drive
"""

from . import plugin  # noqa: F401  registers AssemblyObserverPlugin
