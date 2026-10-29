"""
One module (or package) per feature, found by `plugin.discover`.

Add a file with @register; run it with `-p plugins:="['<name>']"`. Its tests go in `test/plugins/<name>/`, where
`pytest -m <name>` runs them all, slow ones too (AGENTS.md).

! Everything that is not a real robot lives here; the runtime knows nothing about
  bars, racks or assembly steps. Plugins draw only into their own PluginView.
! Plugins never import each other (test/structure/test_layers.py): shared code, data types and service interfaces go in
  `shared/`, and another plugin's live data comes only from `ctx.require`, for names in `requires`, typed by its
  `<Name>Contract` Protocol in `shared/`.

* Scene rules:
  - Put collision objects in `ctx.scene` under "<plugin name>/…"; the runtime draws
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
  NotLeased; stopping and `hold` need none. `ctx.lease_robots` holds it for a `with` block in a task, `ctx.take_lease`
  across ticks. ! Keep it while the motion runs, not only while sending. Leases only keep this monitor's code apart.
* Freezes (optional, `ctx.freeze`): a task that assumes other robots or objects stand still is cancelled once they
  move, whoever moved them. Example of both: `examples/lease_and_freeze.py`.
* Clearances (optional, `ctx.clearance`): while a task drives a robot, the robot is soft stopped and the task
  cancelled once it comes within a margin of anything in the scene. Plugins only provide bodies; the runtime checks.
  Strict by default: unknown arm joints trip it too. Manual moves pass `unknown_arms=ALL_ARMS` (or arm names): those
  arms count where last seen, or stowed. The base pose must always be known. Other robots not measured are checked
  as their stand-ins.
* Stand-ins (`world/stand_ins.py`): every collision check treats another robot that is not measured the same way:
  where it was last seen, else at its default pose with its arms stowed. ! Show its warnings to the operator.
  The last trips, with what came too close, are in `ctx.clearance_trips`; the `collisions` plugin draws them.

* Panels never jump (AGENTS.md, "Panels: nothing jumps"): chips that can appear go on a line that is always there
  (`ui.style.chip_line`), text that comes and goes below the buttons.
* Prefer tabs to folders: users can drag tabs into windows of their own, folders stay put.
  - `ctx.view.ui()` is the plugin's tab in the side panel, at its `tab_path` (e.g. `("Control",)`, default its
    name); a path under "Plan" (`ui.visualization.PLAN_TAB`) goes in the Plan window at the top instead.
    `hidden = True` starts it in the minimized panel at the right edge, for tools not needed every session.
  - Plugins with the same `tab_path` share one tab through bands, `ctx.view.ui(band=...)`, sorted by number; the
    first to start sets its icon. The Assembly tab holds process, assembly_steps, movement_exec and assembly_run
    (`ui/assembly_tab.py`).
  - `ctx.view.tab(label, hidden=...)` adds a further tab, nested in a tab named after the plugin, so its owner shows.
  - `ctx.view.folder()` is a folder in the main panel, for what must always show, like health's rows.
  - ! Set `icon`: every tab a plugin owns at the top of a panel carries it (also its phone tab); shared parents like
    Plan carry none; tabs nested inside the plugin's tab pick their own.
* Header: `ctx.view.header()` puts a short status under Stop all, e.g. health's banner.
* Phone: a plugin may add a tab to the phone page with `ctx.view.phone()` (and a short status above the tabs with
  `ctx.view.phone_header()`). Keep it to what an operator needs next to the robots; the rest stays on the laptop.
"""
