"""
One module (or package) per feature, imported by plugin.discover. A new feature
is a new file here plus a @register decorator -- no edit to the monitor, no new
flag. Name it in `enabled_plugins` to run it.

! Everything that is not a real robot lives here. The core owns the node, the
  real robots, the PyBullet scene, the viser server and the plugin machinery. It
  has no idea what a bar, a rack, a calibration or an assembly step is. Anything
  that draws does so into its own PluginView.

! PyBullet is handed over raw, and plugins clean up after themselves.
  `ctx.scene.client_id` and `ctx.scene.robots` are real ids. Whatever a plugin
  loads, it removes in its own teardown -- nothing tracks it.

The plugins this is being ported to, and the old flag each replaces:
doc/plugin_roadmap.md.
"""
