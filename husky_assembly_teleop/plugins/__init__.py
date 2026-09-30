"""
One module (or package) per feature, found by plugin.discover. Add a file with
@register and list it in `enabled_plugins` to run it.

! Everything that is not a real robot lives here. The core knows nothing about
  bars, racks or assembly steps, and plugins draw only into their own PluginView.

! PyBullet is shared raw: whatever a plugin loads, every plugin sees, and that
  plugin must remove it in its own teardown (nothing tracks it).
"""
