"""
One module (or package) per feature, found by `plugin.discover`.

Add a file with @register; run it with `-p plugins:="['<name>']"`.

! Everything that is not a real robot lives here; the core knows nothing about
  bars, racks or assembly steps. Plugins draw only into their own PluginView.
! Plugins never import each other (test/structure/test_layers.py): shared code, data types and service interfaces go in
  `shared/`, and another plugin's live data comes only from `ctx.require`, for names in `requires`, typed by its
  `<Name>Contract` Protocol in `shared/`.

* Scene rules:
  - Put collision objects in `ctx.scene` under "<plugin name>/…"; the core draws
    and removes them. ! Never edit a `Geometry` in place: assign a new one. Build
    large meshes on a loading thread (`ctx.run_in_thread`).
  - Read robot and link poses from `ctx.kinematics`, not from URDFs.
  - To plan, take `ctx.scene.snapshot` on the main thread and sync your own mirror
    from it on your own worker thread. Report results with our ids.
  - Measured objects never go in the scene: use `ctx.track_object`.

* Soft stop (Stop all): each robot refuses motion commands until Resume, raising RobotStopped, and every task is
  cancelled unless spawned with `survives_soft_stop` (only for work that never commands a robot). ! In `update`,
  check `soft_stopped` before commanding: RobotStopped there skips the rest of the hook.
* Leases: every command that could move a robot, or switches its controllers, needs that robot's lease, else raises
  NotLeased; stopping and holding need none. `ctx.lease_robots` holds it for a `with` block in a task, `ctx.take_lease`
  across ticks. ! Hold it while the motion runs, not only while sending. Leases only keep this monitor's code apart.
* Freezes (optional, `ctx.freeze`): a task that assumes other robots or objects stand still is cancelled once they
  move, whoever moved them. Example of both: `examples/lease_and_freeze.py`.
* Clearances (optional, `ctx.clearance`): while a task drives a robot, the robot is soft stopped and the task
  cancelled once it comes within a margin of anything in the scene. Plugins only provide bodies; the runtime checks.
  The last trips, with what came too close, are in `ctx.clearance_trips`; the `collisions` plugin draws them.

* Header: `ctx.view.header()` puts a short status above every plugin's folder, e.g. health's banner.
* Tabs: `ctx.view.tab(...)` puts a panel into the side panel's tabs: Control (robot_control) and Plan, with a tab
  per planner panel.
* Phone: a plugin may add a tab to the phone page with `ctx.view.phone()` (and a short status above the tabs with
  `ctx.view.phone_header()`). Keep it to what an operator needs next to the robots; the rest stays on the laptop.
"""
