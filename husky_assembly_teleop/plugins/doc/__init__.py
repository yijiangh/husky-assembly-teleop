"""
The Doc tab: what the assembly of the loaded process leaves behind, its report pages and its runs.

  plugin.py   the plugin: the report buttons and their link, the list of runs, the warning window
  report.py   a report page's script run as a subprocess, its folder served over HTTP
"""

from . import plugin  # noqa: F401  registers DocPlugin
