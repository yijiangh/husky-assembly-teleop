# Headless planners with a panel on their public API (2026-10-19)

Status: **done 19 Oct, not pushed**: contract (400bb66), arm planner (3f66fd6), base planner (07b7ff0), the old
`PlannerPlugin` removed. All tests and linters pass; not yet run on a robot.

## Why

The 19 Oct review panel found three gaps one change closes: planners are panels, not services; `ProcessPluginApi`
is a type hint only (`ProcessContract` since); the services rule in `2026-10-19_shared_plugin_layer.md` has no user. Splitting each planner into
a headless planner plugin and a panel plugin that drives it through its public API gives the rule its first user
and keeps the plan and commit paths ready for an executor.

## Decisions

- **Two plugins per planner, in the same package.** `arm_planner` and `base_planner` become headless
  (`PlannerService`); their panels are new plugins `arm_planner_panel` and `base_planner_panel`, which require the
  planner. Operators enable the panel; `requires` pulls in the planner.
- **Partial separation (user, 19 Oct):** the panel sets the target, plans, cancels, clears, commits and reads results
  only through the API (`PlannerContract`). It may read the planner's internals for debug drawing and state that
  only the panel shows (e.g. the base planner's debug record, the PyBullet window toggle). The `!` note at the top
  of the planner files says so.
- **The request carries the target.** Each planner defines a frozen request type in `shared/planning/planner_contract.py`
  (imports nothing from `plugins/`; settings as plain values), and an alias `<Name>Contract =
  PlannerContract[<Request>]`.
- **Staleness is split.** The planner judges the robot (`stale_reason`: it moved since planning); the panel judges
  its own target against `plan.request` (`target_change`) before calling `commit`.
- **Commit and what follows it live in the planner**: `send_to_arm`, `send_to_onboard_follower`, the base planner's
  follow task and recording.
- **Base path type stays monitor-local** (`TimedPath`/`BasePath`), not the core's `MovementResult`: the core has
  no drive path. Revisit with the executor.

## Contract (done)

`shared/planning/planner_service.py`:

- `Plan[R]`: frozen; `id`, `request`, `state` (planning, ready, stale, sent, failed, cancelled), `path`, `message`,
  `warning`. Replaced on every change.
- `PlannerContract[R]` (runtime-checkable Protocol): `plan`, `request_plan(request) -> Future[Plan]`, `cancel()`,
  `clear()`, `commit(plan_id) -> Future[(sent, message)]`.
- `PlannerService[R]`: the worker thread, the latest plan and its futures. Subclasses supply `make_search(ctx,
  request, abort) -> Search`, `stale_reason(ctx, plan)`, `send(ctx, plan)`, optionally `forget(plan)`; call
  `super().setup` and `super().update`. Every future resolves: a cancelled or soft-stopped search gives a
  "cancelled" plan (fixes the "stuck at planning" review item for the new classes).
- `Search` (no `serial`; `accepted` gets the result), `with_guesses`, `WAIT_MARGIN`, `PlanningWorld` moved here.

`shared/planning/planner_panel.py`: `PlannerPanel` (new) beside the old `PlannerPlugin` (removed once both are ported).
Subclasses set `planner`, put it in `requires`, call `_connect(ctx)` first in setup, and supply `make_request`,
`target_change`, `show_target`, `show_path(ctx, ghost, plan)`. UI helpers as before (`_add_path_controls`,
`_add_ghosts`, `_set_up_idle`, `_draw_slider`, `_draw_ghosts`, `_plan_chip`, `_message_html`, `_process_step`).

## Port (per planner, one agent each, own worktree)

1. Request type and `<Name>Contract` alias in `shared/planning/planner_contract.py`.
2. `plugins/<planner>/plugin.py`: the headless planner on `PlannerService`, keeping name `<planner>`.
3. `plugins/<planner>/panel.py`: the panel on `PlannerPanel`, name `<planner>_panel`.
4. Tests in `test/plugins/<planner>/`: existing ones updated, plus that the planner is a `PlannerContract` and that the
   panel plans and commits through it.
5. README, `doc/plugin_roadmap.md`, the planner's manual and launch examples: the panel's plugin name.
6. Behaviour for the operator unchanged apart from the plugin name.

**Base planner (done):** `BasePlanRequest` holds serial, goal (x, y, yaw), algorithm label ("" for the default) and
its settings as (name, value) pairs, debug on/off, and the recording inputs: task name, record on/off, experiment name
(the planner keeps the start pose in the plan's path). The planner requires `process` for the step it records. The
panel owns the tasks' virtual obstacles (`base_planner_panel/task/…`): choosing a task is an operator's scene setup.
Record now follows the plan's request: a run is saved only when its plan was requested with `record`.

**Arm planner (done):** `ArmPlanRequest` holds serial, arm and goal joints; the planner keeps the base position the
latest plan started from. The panel reaches inside only for the PyBullet window toggle.

**Merge (done):** both ported in parallel and cherry-picked; `PlannerPlugin`, the old `Search` and its tests removed
(the service's tests cover them, and its "cancelled" plans fix the "stuck at planning" bug). A ready plan whose target
the panel moved away from shows "target moved: replan" (`PlannerPanel.update`), as "stale" did before the split.

## Tests

- `test/shared/test_planner_service.py` (done): ready, failed, timeout, cancel and soft stop settle; a new request
  replaces a running one; a raising search fails the plan and the task; commit only the latest ready plan; clear.
- `python3 -m pytest -m ""` green after each port and after the merge.

## Not in this task

- Stopping what a commit started on cancel, timeout or Clear (review safety #8-#10), motion ownership, the core's
  solution format, an executor plugin.
