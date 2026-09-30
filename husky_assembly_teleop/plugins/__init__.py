"""
One module (or package) per feature, found by plugin.discover. Add a file with
@register and list it in `enabled_plugins` to run it.

! Everything that is not a real robot lives here. The core knows nothing about
  bars, racks or assembly steps, and plugins draw only into their own PluginView.

* The scene (doc/scene_refactor_plan.md §8):
  - Put collision objects in `ctx.scene` under "<plugin name>/…". The core draws
    them and removes them when the plugin closes.
  - Change a body in place or `put` it again. ! Never edit a `Geometry`: assign a
    new one. Build large meshes on a loading thread (`ctx.run_in_thread`).
  - Robot and link poses come from `ctx.kinematics`; don't parse URDFs yourself.
  - To plan, take `ctx.scene.snapshot` on the main thread and sync your own
    mirror from it on your own worker thread. Report results with our ids.
  - Measured objects never go in the scene: use `ctx.track_object`.
"""
