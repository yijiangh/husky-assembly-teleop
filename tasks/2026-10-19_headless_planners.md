# Headless planners with a panel on their public API (2026-10-19)

Status: **contract done** (`shared/planning/service.py`, `PlannerPanel` in `shared/planning/panel.py`, tests in
`test/shared/test_planner_service.py`); arm and base planners being ported in parallel, one agent each.

## Why

The 19 Oct review panel found three gaps one change closes: planners are panels, not services; `ProcessPluginApi`
is a type hint only; the services rule in `2026-10-19_shared_plugin_layer.md` has no user. Splitting each planner into
a headless planner plugin and a panel plugin that drives it through its public API gives the rule its first user
and keeps the plan and commit paths ready for an executor.

## Decisions

- **Two plugins per planner, in the same package.** `arm_planner` and `base_planner` become headless
  (`PlannerService`); their panels are new plugins `arm_planner_panel` and `base_planner_panel`, which require the
  planner. Operators enable the panel; `requires` pulls in the planner.
- **Partial separation (user, 19 Oct):** the panel sets the target, plans, cancels, clears, commits and reads results
  only through the API (`PlannerPluginApi`). It may read the planner's internals for debug drawing and state that
  only the panel shows (e.g. the base planner's debug record, the PyBullet window toggle). The `!` note at the top
  of the planner files says so.
- **The request carries the target.** Each planner defines a frozen request type in `shared/planning/<arm|base>.py`
  (imports nothing from `plugins/`; settings as plain values), and an alias `<Name>PluginApi =
  PlannerPluginApi[<Request>]`.
- **Staleness is split.** The planner judges the robot (`stale_reason`: it moved since planning); the panel judges
  its own target against `plan.request` (`target_change`) before calling `commit`.
- **Commit and what follows it live in the planner**: `send_to_arm`, `send_to_onboard_follower`, the base planner's
  follow task and recording.
- **Base path type stays monitor-local** (`TimedPath`/`BasePath`), not the core's `MovementResult`: the core has
  no drive path. Revisit with the executor.

## Contract (done)

`shared/planning/service.py`:

- `Plan[R]`: frozen; `id`, `request`, `state` (planning, ready, stale, sent, failed, cancelled), `path`, `message`,
  `warning`. Replaced on every change.
- `PlannerPluginApi[R]` (runtime-checkable Protocol): `plan`, `request_plan(request) -> Future[Plan]`, `cancel()`,
  `clear()`, `commit(plan_id) -> Future[(sent, message)]`.
- `PlannerService[R]`: the worker thread, the latest plan and its futures. Subclasses supply `make_search(ctx,
  request, abort) -> Search`, `stale_reason(ctx, plan)`, `send(ctx, plan)`, optionally `forget(plan)`; call
  `super().setup` and `super().update`. Every future resolves: a cancelled or soft-stopped search gives a
  "cancelled" plan (fixes the "stuck at planning" review item for the new classes).
- `Search` (no `serial`; `accepted` gets the result), `with_guesses`, `WAIT_MARGIN`, `PlanningWorld` moved here.

`shared/planning/panel.py`: `PlannerPanel` (new) beside the old `PlannerPlugin` (removed once both are ported).
Subclasses set `planner`, put it in `requires`, call `_connect(ctx)` first in setup, and supply `make_request`,
`target_change`, `show_target`, `show_path(ctx, ghost, plan)`. UI helpers as before (`_add_path_controls`,
`_add_ghosts`, `_set_up_idle`, `_draw_slider`, `_draw_ghosts`, `_plan_chip`, `_message_html`, `_process_step`).

## Port (per planner, one agent each, own worktree)

1. Request type and `<Name>PluginApi` alias in `shared/planning/<arm|base>.py`.
2. `plugins/<planner>/plugin.py`: the headless planner on `PlannerService`, keeping name `<planner>`.
3. `plugins/<planner>/panel.py`: the panel on `PlannerPanel`, name `<planner>_panel`.
4. Tests in `test/plugins/<planner>/`: existing ones updated, plus that the planner is a `PlannerPluginApi` and that the
   panel plans and commits through it.
5. README, `doc/plugin_roadmap.md`, the planner's manual and launch examples: the panel's plugin name.
6. Behaviour for the operator unchanged apart from the plugin name.

**Base planner (done):** `BasePlanRequest` holds serial, goal (x, y, yaw), algorithm label ("" for the default) and
its settings as (name, value) pairs, debug on/off, and the recording inputs: task name, record on/off, experiment name
(the planner keeps the start pose in the plan's path). The planner requires `process` for the step it records. The
panel owns the tasks' virtual obstacles (`base_planner_panel/task/…`): choosing a task is an operator's scene setup.
Record now follows the plan's request: a run is saved only when its plan was requested with `record`.

Then (me): merge both, delete `PlannerPlugin` and the old `Search` from `panel.py`, and
`test/shared/test_planner_panel.py`'s tests of the old class (its two strict xfails are fixed by the service).

## Tests

- `test/shared/test_planner_service.py` (done): ready, failed, timeout, cancel and soft stop settle; a new request
  replaces a running one; a raising search fails the plan and the task; commit only the latest ready plan; clear.
- `python3 -m pytest -m ""` green after each port and after the merge.

## Not in this task

- Stopping what a commit started on cancel, timeout or Clear (review safety #8-#10), motion ownership, the core's
  solution format, an executor plugin.
