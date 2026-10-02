"""
One module (or package) per feature, found by `plugin.discover`.

Add a file with @register and list it in `enabled_plugins` to run it.

! Everything that is not a real robot lives here; the core knows nothing about
  bars, racks or assembly steps. Plugins draw only into their own PluginView.

* Scene rules:
  - Put collision objects in `ctx.scene` under "<plugin name>/…"; the core draws
    and removes them. ! Never edit a `Geometry` in place: assign a new one. Build
    large meshes on a loading thread (`ctx.run_in_thread`).
  - Read robot and link poses from `ctx.kinematics`, not from URDFs.
  - To plan, take `ctx.scene.snapshot` on the main thread and sync your own mirror
    from it on your own worker thread. Report results with our ids.
  - Measured objects never go in the scene: use `ctx.track_object`.
"""
