# Shared layer for plugins, and plugin dependency chains (2026-10-19)

Status: **done 19 Oct, not pushed**: phase 0 (3275f2b..38984b5), 1 (dda698f), 2 (2c17184), 4 (d729adc); phase 3
left as is (see there). Scope was refactoring only, with one exception: arm_planner's `send_to_arm` is implemented.
All tests and linters pass; the arm path send is not yet tried on a robot.

## Decisions

- **Plugins never import each other.** What two plugins need goes into a new layer, `shared/`, between `plugin_api`
  and `plugins`. Its one purpose, stated in its docstring: **for plugins to share code, share data, and offer
  services to each other.** Nothing else goes there.
- **Data follows `requires`.** `shared/` defines what data looks like; each plugin owns its live copy. A plugin
  reaches another's data only with `ctx.require(name)`, for names in its `requires`.
  - Read: the provider exposes a read-only property returning a frozen shared type, and replaces it on change, never
    edits it (consumers may compare by identity).
  - Services (when one is needed later): the provider offers methods from a shared interface that return an
    `asyncio.Future`. The work runs in the provider's own context: it keeps its ctx from `setup` and queues the
    request with `ctx.submit`, so its errors, soft stop and teardown apply. Results go back through the future or
    the provider's state; no callbacks registered by consumers.
- **Consumers type a required plugin against a Protocol in `shared/`**, not its concrete class, so the provider can
  change everything else. Concrete classes may still appear under `TYPE_CHECKING`.

## Layers

```
monitor · experiments · mocap_relay
plugins           each plugin: its state, UI and tasks; imports shared/ and below, never another plugin
shared            for plugins to share code, data and services: planning/, process.py   (new)
plugin_api        PluginContext, HuskyPlugin, trace, signals
ui · world · robot_interface · config/drive/tool_urdfs/plots
```

## Plan

### 0. Import restructure (done 18 Oct, committed 19 Oct)

Mocap sample types into `robot_interface/mocap_state.py`, `signals.py` into `plugin_api`, ui type-only imports,
`timing` into `planning/path.py`, `world/lab.py`, `plots.py`, offline code into `experiments/`, and
`test/test_layers.py`. All tests and linters pass; every commit fails no test the commit before it passed.

### 1. The `shared/` layer, with `planning/` (done, dda698f)

- `husky_assembly_teleop/shared/__init__.py`: the purpose above, and the rules (imports nothing from `plugins/`;
  holds types, interfaces and helpers, never a plugin's live state).
- Move `plugins/planning/` to `shared/planning/`; update both planners and their tests.
- `test/test_layers.py`: add `("shared",)` between `plugin_api` and `plugins`; drop the `planning` exception, so any
  plugin-to-plugin import fails.
- `plugins/__init__.py` and `AGENTS.md`: one line each pointing to the rule.

### 2. Process data: `shared/process.py` and `ProcessSource` (done, 2c17184)

Today the planners and `mocap_bars` read the process plugin through untyped attribute chains (`step`, `index`,
`loaded.process.robots`, `ids`; `_process_step` returns `tuple[Any, str, str]`), so a renamed field breaks them only
at runtime.

- Move `plugins/process/steps.py` (pure data and loading: `Step`, `LoadedProcess`, `load_process`, `live_ids`,
  `scene_bodies`, …) into `shared/process.py`.
- Add a frozen `Selection`: the loaded process, the selected step and its index, the acting robot's configured serial
  (or None and why), and the `ids`/`process_ids` maps.
- `ProcessSource` Protocol: `selection: Selection | None`.
- `ProcessPlugin` implements it; `planning/panel.py:_process_step`, both planners and `mocap_bars` read `selection`.
  Same behaviour, same messages, except for a loaded process without movements: it now counts as nothing
  selected (base_planner records no process step instead of 0).
- `revision` stays: only the process plugin itself uses it.

### 3. Duplicated base path code (only if it is really the same): left as is

`base_exp/plugin.py:376-394` and `base_planner/plugin.py:413-416` both send a path and wait on the follower, with
different checks and timeouts. Compare them; if one helper can serve both without changing either plugin's behaviour,
put it in `shared/`. Otherwise leave them and note why.

Not the same. What they share, judging the follower against a sent path, is already `world/follower.path_check`,
which base_planner uses. base_exp's run adds an acknowledgement timeout, the clearance guard, e-stop and mocap-loss
ends, a settling phase, and outcome texts the offline analysis reads back from recorded runs; merging would change
what base_exp records.

### 4. arm_planner sends its path (the one feature change; done, d729adc)

`send_to_arm` (`plugins/arm_planner/plugin.py`) is a stub that only logs and reports success. Implement its TODO:
resample the planned path with a smooth speed profile and send it with `ArmInterface.send_joint_trajectory`; report
failure when the arm refuses. If robot_control's move code is needed for it, it goes into `shared/`, not imported.

Done as `robot_interface.arm.joint_path_move`: straight joint segments, each on `joint_move`'s cosine profile and
stopping at every waypoint (so the arm stays on the checked segments), each lasting whole sampling steps, at
`JOINT_MOVE_MAX_SPEED` and half the trajectory acceleration limit. robot_control's code was not needed.
To try on a robot: a short plan in sim, then on an arm with a hand on the pendant stop.

## Tests

- `test_layers.py` after phase 1: no plugin-to-plugin import, `shared/` imports no plugin.
- The existing quick, slow and linter suites after each phase, unchanged in what they check.
- Phase 2: a test that `ProcessPlugin.selection` is replaced, not edited, when the step changes.

## Not in this task

- An executor plugin, and planner services (`request_plan`/`request_commit`): later, following the services rule.
- Arm planner: no other feature changes.
- Motion ownership in `robot_interface` (today base_exp and base_planner can both command one base): a separate task.
- Review fixes that are not about dependencies, e.g. `plan_state` stuck at "planning" after a cancel
  (`planning/panel.py:254`): separate commits.
