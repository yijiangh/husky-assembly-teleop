"""
Frame check: the built frame's measured bars against their design, live on the phone, for straightening it by hand.

  picture.py  each bar's ends against the design, and the picture from the side, the front or above
  plugin.py   the plugin: the desktop and phone tabs, refreshed from `mocap_bars`' fits
"""

from . import plugin  # noqa: F401  registers FrameCheckPlugin
