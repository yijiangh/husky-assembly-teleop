# M4: the `runs/` record, the planner service, and the run plugin (spec)

Part of the core-port plan (milestone M4). The new monitor must remember how far an assembly got (which actions are
done, where robots that are not measured were last seen, run settings) and needs one planning worker other plugins call
as a service. M4 delivers: the run format + rules (`shared/runs.py`), a service-style planner (requests carry a
main-thread snapshot; results come back as asyncio futures), the `assembly_run` plugin, and a temporary `goal_check`
plugin — the smallest UI that proves IK at the live base matches the old monitor (B20 "Check goal").

Worktree `ros2_ws_next/src/husky-assembly-teleop`; `file:line` refer to 91a3a9e (core 439bc18). Old code at 0e45758:
M = husky_monitor.py, PIO = progress_io.py, PSI = plan_skeleton_io.py. Real old line numbers: `_load_skeleton_state`
M:11107; auto-load M:11553, 11583–11626; Mark done M:11649–11775; Reopen M:11792–11901; Rescan M:11903; Reset
M:11919, 11932.

Note: the process plugin does NOT auto-load at start (`plugins/process/plugin.py:169`, Jakob's choice; `config.py:154`
says otherwise). The lab clicks Load; `assembly_run` reacts to the load. Not changed here.

## 1. Where `runs/` lives, and what a re-export does
`runs/<YYYYMMDD_HHMMSS>/` sits inside the process folder (`loaded.source`; for an old export the converted
`<export>_process`). Core `write(process, folder, overwrite=True)` (`process/write.py:32`) deletes only `actions/*.json`
(:55-56) and `meshes/` (:57), rmtree's `robots/<name>/` only when its source files are not inside it (:131), writes
`process.json` + scheduled actions (:62-65), and leaves `source/`, `solutions/`, `runs/` alone (docstring :33,
`doc/process_format.md:62`). Rhino's RSExportProcess calls exactly this. So `runs/` survives a re-export into the same
folder; the hashes change (§2: the old run no longer matches); an action dropped from the schedule loses its file
(carry-over drops it); an export into a NEW folder starts with no runs (continuing across folders is out of M4).

## 2. NEW `shared/runs.py` (~300 lines; the format doc is its module docstring)
| File | Contents |
|---|---|
| `run.json` | `schema`, `kind:"run"`, `name`, `created_at`, `process` (Drive-relative if possible), `process_hash`, `action_hashes{id→sha}`, `robots{live id→{serial, urdf_sha256, relay_calibration:""}}`, `commits{monitor, core, tamp}`, `settings`, `continued_from`, `changed_since[]` |
| `progress.json` | `schema`, `kind:"progress"`, `updated_at`, `current{action, movement}`, `actions{id→{status, seq, marked_at, assumed, ends{live id→RobotRecord}}}`, `robots{live id→RobotRecord}` (beliefs) |
| `events.jsonl` | `{time, ros_time, event, action, movement, …}` per line |
| `movements/<action>/<movement>.<n>.json` | `{schema, kind:"movement", …payload}` (written in M5) |
| `servo/<action>/<movement>.<n>.json` | same, `kind:"servo"` (M7) |

Constants: `SCHEMA = "husky_run/1"` (other values → `RunError(ValueError)`); `PENDING, DONE`; `LIVE, ASSUMED`;
`LIVE_JOINTS_AGE = 0.5`; `LIVE_BASE_AGE = 1.0` (M:280). Poses stored as `[x,y,z,qx,qy,qz,qw]`.

Frozen dataclasses:
```
RunSettings(transfer_as_free=True, insert_split_mm=5.0, spiral=False, servo_by_bar=False)   # M:454, M:242/783, M:794-798, M:564
RobotRecord(base: Pose, joints: Mapping[str,float], source: str, at: str, after_action: str|None=None)
ActionRecord(status=PENDING, seq=0, marked_at=None, assumed=False, ends: Mapping[str,RobotRecord]={})
Progress(actions, current: tuple[str,int]|None, robots: Mapping[str,RobotRecord], updated_at: str)
RunInfo(name, created_at, process, process_hash, action_hashes, robots, commits, settings, continued_from=None, changed_since=())
Run(folder: Path, info: RunInfo, progress: Progress)
```
Functions (pure ones return a new `Run`):
| Function | Rule |
|---|---|
| `file_hashes(process) -> (str, dict)` | `content_hash` (core write.py:318) of process.json and each actions/<id>.json; same values as `solved_against` (solutions.py:104) |
| `find_runs(process_folder) -> (list[Run], list[str])` | newest first; unreadable / other-schema folders → problem list |
| `continue_run(runs, hashes) -> Run \| None` | newest run whose process_hash AND all action_hashes are equal |
| `new_run(process, robots, settings, *, continued_from=None, now=None) -> Run` | I/O: folder (clash → `_2`, `_3`), run.json, progress.json, first event (`new_run`/`continue`); with `continued_from`: carry statuses/ends/beliefs/current by action id, record `changed_since`, drop ids that are gone (logged) |
| `mark_done(run, action_id, ends, assumed, *, now=None)` | status done, `seq = max+1`, ends stored; each robot in ends → `robots[id] = replace(end, after_action=action_id)` (PIO:207) |
| `reopen(run, action_id)` | refuses if pending; `ActionRecord()`; beliefs that came from this action → the end of the highest-seq done action with an end for that robot, else dropped (replaces PIO:660, from recorded ends) |
| `save_progress(run)`, `save_info(run)` | atomic: `.tmp`, flush, `os.fsync`, `os.replace` (PIO:470-486); stray `.tmp` ignored; missing progress.json → fresh Progress |
| `record_event(folder, event, ros_time, **fields)` / `read_events(folder)` | append + flush + fsync; reader skips an unparseable last line |
| `record_movement(folder, action, movement, payload, kind="movements") -> Path` | n = 1 + highest; atomic; kind ∈ {movements, servo} |
| `commits() -> dict` | `git rev-parse --short=12 HEAD` (+ dirty) for this package, core (`writer_info()`, version.py:25), tamp |
Settings changes rewrite run.json (`save_info`) + a `settings` event.

Beliefs when every robot is on Zenoh (`scene_for` picks per field, scene_for.py:161-192; `RunView.beliefs` offers every recorded robot):
| Robot in this run | Planning scene uses | Written at Mark done |
|---|---|---|
| configured, base tracked, joints measured at least once | live (Kinematics keeps the last joints, kinematics.py:94) | its end, if `is_live` |
| configured, joints never measured this session | belief joints → design → stowed → parked | acting robot: export's end, `assumed`, after Confirm |
| configured, base not tracked | belief base → design base → parked | — |
| not in `robots:=` | left out (scene_for.py:114) | — |
Holds tracking (PIO:225) is not ported: process states say who holds what.

## 3. NEW `shared/assembly_run.py` (~110 lines)
- `RunView` (frozen): `folder`, `process`, `statuses`, `assumed: frozenset`, `current: tuple[str,int]|None`,
  `beliefs: Mapping[str, RobotBelief]`, `settings: RunSettings`, `offer: str`, `revision: int`.
- `AssemblyRunPluginApi(Protocol)`: `view`; `request_set_current(action_id, movement_index=None)`,
  `request_mark_done(action_id=None)`, `request_reopen(action_id=None)`, `request_new_run()`, `request_continue()`,
  `request_settings(**changes)`, `request_record(event, *, action=None, movement=None, data=None, file=None)` — main
  thread; operator errors logged + shown, never raised.
- Helpers: `is_live(robot, now)` (enabled, `base_tracked`, no `unmeasured`, base ≤ 1.0 s, joints ≤ 0.5 s old;
  fields from world/scene.py:62-69); `live_record(robot, at)` (`model.movable_joints`); `exported_end(loaded,
  action_id)` (`end_state(last movement)`, plan_check.py:133 → `robots[action.robot]`, source `assumed`; None if base or
  joints null); `beliefs_of(progress)`; `first_pending(progress, schedule)`; `first_step(process, action,
  transfer_as_free)` = `operator_steps(...)[0].primary` (with the workaround B20_J → B20_J_transfer).

## 4. EDIT: process (plan edit 4)
- `shared/process.py:111` `ProcessPluginApi`: add `request_select(self, index: int) -> None` (main thread; clamped;
  ignored without a process).
- `shared/process.py:65` `LoadedProcess`: add `index_of(action_id, movement_index=0) -> int`
  (= `first_step_of(schedule.index(id)) + movement_index`, :83).
- Move `action_window` from `plugins/process/plugin.py:58` to `shared/process.py` (the plugin imports it back;
  `test_process_panel.py` unchanged).
- `plugins/process/plugin.py`: `request_select(self, index)` → `self.select(index)` (:225); module docstring.

## 5. Planning (shared)
- NEW `shared/planning/mirrors.py`: move `ArmPlanningWorld` (plugins/arm_planner/planner.py:23-88) here unchanged as
  `MirrorPool`, plus `mirror(serial) -> CompasFabMirror` (first half of `sync`, :48-53). planner.py re-exports it as
  `ArmPlanningWorld` (keeps `__all__`; arm_planner plugin and its tests unchanged).
- EDIT `route_result.py`: `RouteResult.routine: Routine | None` ("None: could not be classified"). Add frozen
  `GoalCheck`: `movement`, `routine`, `ok`, `reason=""`, `acting=""`, `base`, `start` (live arm joints), `ik`,
  `goal_joints` (unwrapped), `authored` (`spec.goal_joints`, the IK reference), `sources`, `warnings`, `seconds`;
  property `max_from_authored` = max |ik.joints − authored| RAW (before unwrap) — the old printout's quantity
  (M:4865, M:4780); `as_record() -> dict` (JSON-safe).
- EDIT `tamp_route.py`: factor `_route(mirror, loaded, step, snapshot, robots, spec, *, transfer_as_free, beliefs,
  ignore_built, goal_joints, abort, log) -> _Route`; split `_Route.run` into `prepare()` (sync, names, start) + the start
  check (plan_movement behaviour unchanged). New `check_goal(mirror, loaded, step, snapshot, robots, *,
  transfer_as_free=False, beliefs=None, ignore_built=False, log=print) -> GoalCheck`: only TRANSFER_FREE, TRANSFER_HELD,
  SUPPORT_APPROACH have an IK goal (others → ok=False "goal not solved by IK"); `_route` → `prepare()` →
  `ik_goal(unwrap=routine is not SUPPORT_APPROACH)`; `Refused`/any exception → ok=False (fail closed).
- NEW `shared/planning/service.py` (~180 lines): `PlanRequest` (frozen: `loaded`, `step`, `snapshot`, `robots`,
  `beliefs={}`, `transfer_as_free=False`, `ignore_built=False`, `goal_joints=None`, `joints=None`, `label=""`; built on
  the main thread from `ctx.scene.snapshot`, `ctx.config.robots`, run beliefs; `scene_for` runs on the worker);
  `CollisionReport` (frozen: `pairs` in process terms, `reason`, `sources`, `warnings`, `seconds`);
  `MotionPlannerPluginApi(Protocol)`: `busy`, `running`, `request_plan(r) -> asyncio.Future[RouteResult]`,
  `request_ik(r) -> asyncio.Future[GoalCheck]`, `request_collisions(r) -> asyncio.Future[CollisionReport]`,
  `request_cancel()`. Worker functions `run_plan`, `run_ik`, `run_collisions(pool, request, abort, log)`: abort set →
  "cancelled"; acting serial = `live_ids(...)[0](step.action.robot)` (none → "no configured robot acts");
  `snapshot.robots[acting].acting_problems()` non-empty → fail; `pool.mirror(serial)` → `plan_movement` / `check_goal` /
  (`scene_for(..., end_contacts=True)` → sync → `collisions(mirror, joints, full_report=True)`); any exception → a
  failed result (never raised).

## 6. NEW `plugins/motion_planner/{__init__,plugin}.py` (~220 lines), `requires=()`, `experimental=True`
- In `__init__`: `_executor = ThreadPoolExecutor(1, "motion-plan")` (the only thread touching the pool),
  `_pool = MirrorPool()` (lazy mirrors), `_jobs: list[(concurrent Future, abort, label)]`.
- `request_*(r)`: `abort = threading.Event()`; `job = executor.submit(run_x, pool, r, abort, ctx.log_info)`;
  `fut = asyncio.wrap_future(job)`; `fut.add_done_callback(lambda f: abort.set() if f.cancelled() else log_result(f))`;
  return `fut` (a caller's timeout/cancel aborts that search between tries, route_search.py:176; callers await inside
  `async_timeout.timeout(...)`).
- `busy` = any job not done; `running` = a label the worker sets in try/finally; `request_cancel()` sets every in-flight
  abort (queued → "cancelled" unrun; running → stops at its next try).
- `teardown`: abort all, `submit(pool.close)`, `shutdown(wait=True)` (as panel.py:210-215).
- UI: status chips (idle / planning <label> / N queued; robots with a loaded world), last result line, Cancel, robot
  dropdown + `add_pybullet_window_toggle(ctx, gui, self._executor, ...)` (ui/pybullet_window.py:21, panel.py:76; one
  PyBullet window per OS process; it follows the robot being planned).

## 7. NEW `plugins/assembly_run/{__init__,plugin,panel}.py` (~300 + ~150 lines), `requires=("process",)`
- State: `_loaded`, `_hashes`, `_run`, `_offer`, `_settings`, `_current`, `_prompt`/`_answer`, `_revision`, cached
  `_view`, `_writer = ThreadPoolExecutor(1, "run-writer")` (all writes queued in order; in-memory state first; a write
  error → chip + `log_error`; teardown `shutdown(wait=True)`).
- `update`: when `process.selection.loaded is not self._loaded` → cancel open/confirm tasks, reset,
  `ctx.spawn("open run", _open(ctx, loaded))`.
- `_open`: `run_in_thread(file_hashes + find_runs)`; `continue_run` → continue (`session` event); else if any run
  exists → `_offer` = newest ("made on another export: Continue / New run"); else no run yet (folder created lazily by
  `_ensure_run()` on the first mark/reopen/record/settings write; `request_set_current` never creates a run).
  Current = saved current if still pending, else `first_pending`; movement = `first_step(...)`; if the action has a
  configured robot: `await ctx.wait_until(lambda: is_live(...), timeout_s=15.0, ...)` (M:283, M:11583-11626),
  `WaitTimeout` → warning, select anyway; `_set_current` → `process.request_select(loaded.index_of(...))`.
- `request_mark_done`: refuse if no run can be opened (offer pending), unknown or already done, a prompt is pending, or
  any arm of the acting robot `state.is_executing` (robot_interface/arm.py:296; M:11660). Ends = `live_record` of every
  `is_live` robot. No robot or robot in ends → mark now; else a confirm task: prompt "<id> is <robot>'s; <robot> is not
  live: mark done with the export's end (assumed)?", `wait_until(answer, 120 s)`; Confirm → fresh snapshot for the
  ends, `ends[acting] = exported_end`, mark `assumed=True` (M:11669-11722). Then current = first pending →
  `request_select`; save; `mark_done` event.
- `request_reopen`: `reopen`, current = that action (M:11900), save, event.
- `request_new_run`: Confirm, `new_run(...)` with current settings; old run kept (replaces Reset, M:11932).
- `request_continue`: `new_run(continued_from=_offer)`.
- `request_settings`: validate (split 0–30 mm, M:243); no-op changes ignored; `save_info` + event.
- `request_record`: `file` → `record_movement` + event naming the file; else `record_event`.
- Panel (fixed rows, never hidden — process/plugin.py:193): chips (run name / "no run yet" / "export changed"; "k/N
  done"; save error); `ACTION_ROWS` rows via `action_window`, label `"3. B20_J · cindy · done"`, colours current blue,
  done green, assumed orange, pending gray; click → `request_set_current`; buttons "Run" [Continue, New run], "Action"
  [Mark done, Reopen], "Prompt" [Confirm, Cancel] + prompt text; settings (Transfer workaround, Insert split mm, Spiral,
  Servo by bar); details (current action/movement, beliefs line `"alice ← live (B21_H) · belle ← none"`, message).
  No phone tab yet (M5).

## 8. NEW `plugins/goal_check.py` (~150 lines), `requires=("process","assembly_run","motion_planner")`, experimental, M4 only
Removed (or folded) when M5's `assembly_steps` gets Check goal. Button "Check goal" → spawn: selected step →
`PlanRequest(..., beliefs=view.beliefs, transfer_as_free=view.settings.transfer_as_free)`; `async with timeout(120)`;
`await planner.request_ik(r)`. Log `"[goal_check] <movement> at the live base: max |goal - authored goal| = %.4f rad;
goal (rad) left […] right […]; base x y yaw"`. Goal ghost (`robot_ghosts(ctx, robots, "goal")`, ghost.py:113;
`show(base, raw ik joints)` :82, like the old blue ghost M:4818-4826), hidden when the selection changes.
`run.request_record("check_goal", action=…, movement=…, data=check.as_record())`.

## 9. `shared/planning/panel.py` async `Search.run` — deferred to M5
M4 has no PlannerPlugin subclass planning through the service; M5's row lists edits 1–3. With §6's futures it is ~10
lines: `Search.run: Callable[[], Awaitable[PlanResult]] | None`, `_run` (:255-275) awaits it instead of
`run_in_executor`.

## 10. Tasks
| # | Title | Files | Depends |
|---|---|---|---|
| A | run format + rules | shared/runs.py, shared/assembly_run.py, test/test_runs.py | — |
| B | process `request_select` | shared/process.py, plugins/process/plugin.py, test/test_process_selection.py | — |
| C | planning service | shared/planning/{mirrors,service}.py, route_result.py, tamp_route.py, plugins/arm_planner/planner.py, tests | — |
| D | `motion_planner` plugin | plugins/motion_planner/*, test/test_motion_planner.py | C |
| E | `assembly_run` plugin | plugins/assembly_run/*, test/plugin_fakes.py, test/test_assembly_run.py | A, B |
| F | `goal_check` plugin | plugins/goal_check.py, test/test_goal_check.py | A, C |
| G | docs, full test run, monitor smoke, status section | — | all |

## 11. Verification
Tests:
- `test_runs.py` (quick, `tmp_path` process via `write(build_process(tmp), tmp/"p")`, process_fixtures.py:105):
  (1) new_run → read_run round trip; schema; hashes = file_hashes. (2) continue_run picks the newest matching; others
  ignored; other schema → problems. (3) mark_done sets ends, beliefs, seq, assumed; later marks overwrite only their
  robots. (4) reopen restores the belief from the highest-seq earlier end, else drops it; reopening a pending action
  raises. (5) New run: old folder byte-identical; same-second name → `_2`. (6) Re-export: `write(..., overwrite=True)`
  keeps runs/; continue_run → None; carry-over keeps statuses, lists changed_since, drops removed ids. (7) Crash safety:
  `os.replace` raising leaves the old progress.json readable; stray .tmp ignored; truncated last events line skipped.
  (8) record_movement numbers .1, .2; kind="servo" → servo/. (9) is_live at the age limits (0.5 s joints, 1.0 s base,
  untracked, unmeasured). (10) exported_end → None on null joints.
- `test_assembly_run.py` (quick; new `test/plugin_fakes.py` FakeContext generalising FakeClock from
  test_base_planner_commit.py:167-205: spawn → create_task, run_in_thread, require, scene.snapshot, config, logs):
  auto-select after the live wait / after 15 s simulated with a warning; Mark done live → run created, current
  advances, request_select called; Mark done not live → Cancel changes nothing, Confirm records assumed; Reopen; New
  run; a new plugin instance on the same folder continues (restart round trip); settings persisted; `view` is the same
  object between changes.
- `test_motion_planner.py`: quick (monkeypatched run_*, fake pool): FIFO; busy; request_cancel → queued "cancelled"
  unrun; caller timeout sets abort; exceptions → failed results; teardown closes the pool once. Slow (purged process,
  `design_scene(purged, "B20_J_transfer")`, three configs, asyncio.run): request_ik → ok, backend ssik,
  `max_from_authored ≤ 1e-6`; request_plan(transfer_as_free=True) → ok, TRANSFER_FREE, "tamp.plan_free_dual_arm";
  request_collisions returns a report; teardown empties the pool.
- `test_planning_tamp_route.py`: quick `check_goal` refuses B20_J_insert ("not solved by IK"); slow B21_H_approach by
  ("alice",).
- `test_goal_check.py`: as_record survives json.dumps; log-line format; request_record("check_goal", …) with a fake
  planner.
- `test_process_selection.py`: request_select moves and clamps. `test_layers.py` covers the new plugins.

Commands:
```
cd /home/yijiangh/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
cd src/husky-assembly-teleop
python3 -m pytest test -q -p no:cacheprovider
HUSKY_DRIVE_ROOT="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly" python3 -m pytest test -q -p no:cacheprovider -m slow
python3 -m pytest test -q -p no:cacheprovider -m linter -k "flake8 or pep257"
timeout 25 ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice','belle','cindy']" -p plugins:="['goal_check']" -p viser_port:=8095   # all set up, no "failed"
```
Before the lab day (headless): confirm the B20 transfer target joints in the process the lab loads equal the old
export's stored goal (insert start, M:8028) within 1e-6 rad; if not, try converting the old export
`261006_3bar_holding_test` in a scratch copy (convert_export refused 260814 and 261016 per the M3 status).

Lab acceptance (robots on, no motion; monitor and visual steps only):
1. Old monitor (Cyclone, Cindy domain 86, lab workspace): B20 auto-loads, workaround ticked; transfer movement →
   "Transfer start: Confirm manual pose (IK check)"; write down X from "max |goal - stored goal| = X rad"; look at the
   blue ghost's elbows and wrists; quit. Cindy does not move.
2. Switch the robots to Zenoh (crl-husky README, "Zenoh").
3. New monitor: `cd ~/Code/ros2_ws_next && source venv/bin/activate && source install/setup.bash && ros2 daemon stop &&
   ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice','belle','cindy']"
   -p plugins:="['assembly_run','motion_planner','goal_check']" -p process_directory:=<B20 process>`.
4. Process panel: Load. `assembly_run`: "no run yet"; within 15 s current = B20_J; process panel on B20_J_transfer;
   beliefs line: all three "live".
5. `goal_check`: Check goal (the first loads models, up to 1 min). Read Y and look at the goal ghost. Pass:
   |X − Y| ≤ 0.001 rad and the same elbow/wrist branch as the old ghost. Optional: the PyBullet window.
6. Run controls (no motion): Mark done B20_J → green, current B20_R; click the B20_J row, then Reopen → pending;
   restart + Load → same run, statuses kept; New run → Confirm → new name, all pending.
7. Send back: both printouts, the goal_check lines, the run folder.

## 12. Risks
1. Lab comparison validity: relay vs Motive bases differ ≤ 1 mm (~1e-3 rad at the arm); the new scene checks other
   robots live (+ CC.6), the old one used beliefs — either can pick another branch; the check_goal record (base,
   joints, raw IK) allows an offline re-solve.
2. Shared Drive writes: runs land in the Drive process folder; tests only use `tmp_path` copies; never point
   assembly_run at the purged process in tests.
3. Thread ownership: mirrors only on the motion_planner worker; the window toggle uses the same executor; a second
   PyBullet window (arm_planner + motion_planner) unticks itself with a warning.
4. Teardown waits for a running tamp try (≤ 120 s), as PlannerPlugin.teardown.
5. Staleness: a disconnected robot keeps its last joints in planning; beliefs fill only robots never measured this
   session or untracked; Mark done records only robots fresh within 0.5 s / 1.0 s.

## Deviations (from the plan, by design)
1. `request_ik` returns `Future[GoalCheck]` (IkResult + base, unwrapped goal, max diff from the authored goal).
2. Check goal lives in a temporary `goal_check` plugin, keeping `assembly_run`'s `requires=("process",)`.
3. panel.py async `Search.run` deferred to M5.
4. "Offers to continue" → auto-continue the newest matching run; explicit Continue / New run only when the newest run
   was made on another export.
5. Dropped from old code: the `skipped` status, holds tracking, Rescan, the phone tab; `relay_calibration` recorded
   empty (the monitor cannot see it yet).
6. `ArmPlanningWorld` → `shared/planning/mirrors.py` as `MirrorPool` (re-exported under the old name).
7. Run folder names `YYYYMMDD_HHMMSS` (the core doc says `yyyy-mm-dd_hhmmss`, process_format.md:59, §10.2 "specified
   later" — a core doc pointer to update later, with the user's OK).
8. `RouteResult.routine` becomes Optional.

## Status (2026-10-19)

Tasks A–G done on `yh/core-port`: 54e72d6 runs (A), 383257c process `request_select` (B), 63ab61d planning service
(C), 02ad8c6 `motion_planner` (D), 35c3e71 `assembly_run` (E), 21d47d3 `goal_check` (F); 0cc190a and 11110cc small
fixes; this spec, README and roadmap with G.

Tests: default 402 → 441 passed, 2 skipped; `-m slow` 45 → 47 passed, 1 xpassed; flake8 and pep257 clean. (A
first slow run beside the monitor smoke failed `test_base_planner_rrt::test_goes_around_a_body`, a 10 s wall-clock
RRT, once; alone it passed 3 of 3, and the full re-run passed.)
New: test_runs.py (10), test_assembly_run.py (12, a panel test with a fake GUI among them), test_motion_planner.py
(5 + 1 slow), test_planning_service.py (6), test_goal_check.py (3), check_goal in test_planning_tamp_route.py (1 + 1
slow), request_select / index_of in test_process_selection.py (2).

Monitor smoke (`ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1`, CycloneDDS's file left out, new workspace only):
health, process, assembly_run, motion_planner and goal_check set up (goal_check 248 ms: its ghosts), first tick 19 ms,
no "failed"; the only warnings are the experimental plugins and the robots' missing controller managers.

### Before the lab day (headless)

- B20's transfer target joints in the purged process (`261008_2ModulesDemo_purged_process`) equal the stored goal of
  the old export the old monitor loads (`261006_3bar_holding_test`, B20_J_M4_LM_insert's start configuration,
  M:8028) within 4.9e-13 rad on all 12 joints; the insert's start joints too. The two tool0 targets equal the old
  `target_ee_frames` within 6.6e-13 m and 0 rad. No conversion needed.
- `check_goal` at B20's design base: rank 1 of 64 by ssik, max |IK − authored| 4.0e-7 rad (the transfer with and
  without the workaround); the planned goal is IK's, not turned (1.3 s, models loaded first).
- The chain on a scratch copy of the purged process (`FakeContext`, real plugins): no run yet; B20_J current on
  B20_J_transfer once Cindy is live; Check goal → ok, recorded (the run starts: new_run, check_goal); Mark done →
  current B20_R (B20_R_retreat selected); Reopen B20_J → pending; a restart continues the same run (session event).

### Where the code differs from this spec

1. `tamp_route._route(...)` takes no `transfer_as_free`: it gets the classified spec, which already holds it.
2. `runs.py` (499 lines with the format doc): also `read_run(folder)`, `with_current(run, current)` and a public
   `stamp()`; `new_run(..., root=None)` takes the Drive root to record the process folder relative to it (runs.py
   reads no ROS config); `mark_done` refuses an unknown or done action and stores the ends with `after_action` set,
   as the beliefs; `continued_from` is the run's name; `changed_since` lists "process.json" and each changed or new
   action id; the actions dropped from the schedule are named in the first event (`continue`: from, changed,
   dropped).
3. `request_record`: `data` goes under the event's `data` key (a GoalCheck record has its own `movement`, which
   would clash with the line's); `file` is the payload of a `movements/` record, the event names it (`file` key).
   Servo records (`kind="servo"`) are left to M7.
4. `assembly_run` (plugin 578 lines, panel 159): the prompt is set by the request itself, so a second click is
   refused at once ("answer the prompt first"); nothing that writes starts while the runs are being read; the 15 s
   live wait is its own task, cancelled when the operator selects, marks, reopens or starts a run;
   `request_set_current` saves `current` in an existing run (never starts one). `RunView.process` is the
   `LoadedProcess`. After the review: see "Review and its fixes" below.
5. `GoalCheck.start`, `goal_joints` and `authored` hold the planned arm joints only (12 or 6).
6. `goal_check`: `ui/ghost.py` gets a "goal" ghost kind (dark blue); the log line's "goal (rad)" is IK's own joints
   per arm (what the ghost shows and `max_from_authored` measures, as the old printout), base as "x y m yaw deg".
7. `MirrorPool.serials` (read only) for the planner panel's "worlds" chip; the panel's "Window" dropdown picks the
   robot the PyBullet window opens on, then the window follows the robot planned for.
8. `first_step` gives 0 for an action whose movements can't be classified; it lives in `shared/planning/steps.py`
   (runs.py uses it for a carried current too).
9. `test/plugin_fakes.py` also fakes the GUI (`FakeGui`, `FakeWidget`, `defer`, `defer_value`);
   `test_base_planner_commit.py`'s FakeClock is unchanged. Extra file test_planning_service.py.

### Open questions

1. `runs/` is a fixed folder name inside a Drive process folder: with selective sync, a `runs/` that exists in Drive
   but not on this PC would be uploaded as a second `runs` (README, "Folders under the root are never created").
   Keep the lab's process folders fully synced, or name runs differently?
2. Reopen takes the current action; right after Mark done B20_J the current is B20_R (lab step 6 now says to click
   the B20_J row first).
3. Changing the transfer workaround does not move the current step (B20_J stays on its transfer); M5's
   assembly_steps decides.
4. `relay_calibration` is recorded empty (§12 deviation 5); the monitor commit's "+dirty" covers the package folder
   only (as the core's `writer_info`).

### Review (request changes) and its fixes

- **M1, a movement the action lacks**: `request_set_current`, an open and a continue all go through one check: a
  movement index the action does not have becomes the action's first step, with a warning, and that is what is
  saved. `new_run` carries such a current as the action's first step (`current_moved` in its first event);
  `first_step` moved to `shared/planning/steps.py` for that. `LoadedProcess.index_of` raises on a movement the
  action lacks instead of selecting into the next action; the panel shows a bad current as text, never raises.
- **M2, an interrupted open**: the plugin knows whether the runs were read (opening, open, not opened). Nothing is
  written (mark, reopen, record, settings, new run, continue) until they were; an open stopped at any point (also
  before its first step) is read again at the next tick, one that failed 5 s (ROS time) later.
- **M3, a reverted export**: the newest run is continued only when it was made on these very files. Otherwise the
  newest run is offered, and with it an older run made on these files (`RunView.match`); the panel's Run buttons
  are Continue newest (carried into a new run), Continue match (written on directly) and New run;
  `request_continue(run_name=None)`.
- L1 `record_event` ends a line cut by a crash before appending; `read_events(folder, log=None)` skips every
  unreadable line and says once how many. L2 one line per robot: "live now", or "not live; last recorded <source>
  at <action>, <age> ago". L3 the runs are read on the writer thread, behind queued writes. L4 a robot whose arm
  runs a trajectory is never recorded (the acting one refuses the mark, others are left out and named), checked
  again after Confirm. L5 settings are checked by type and range, refused with a reason, never raised. L6 a step
  picked in the process panel during the open or the 15 s wait stops the automatic selection. L7 settings are
  refused while the runs are read or runs are offered, and the panel puts its widgets back. L9 Reopen during an
  offer names it.
- Nits: core imports before relative ones; `PROCESS_FILE` defined in runs.py (not from the core's legacy module);
  Args sections; `urdf_sha256` is of the URDF in the data folder (`RobotConfig.source_urdf`, new), the same on
  every PC; the planner panel's "worlds" chip refreshed after the PyBullet window opens; lab step 6 corrected.
- Tests: the reviewer's probes are tests that failed before the fixes (the movement out of range asked, saved and
  carried; the open stopped before and after its first step; the failed open; the reverted export; the cut events
  line), plus: a moving arm, an arm moving while the prompt waits, a second click during a prompt, settings and
  Reopen during an offer, settings that make no sense, a step picked during the wait, the URDF hash, the carried
  current in runs.py, goal_check's ghost shown and hidden, and the motion planner's running request stopped by its
  abort.

Re-run: default 457 passed, 2 skipped; `-m slow` 47 passed, 1 xpassed; flake8 and pep257 clean; monitor smoke
(domain 70, localhost only) all five plugins set up, no "failed", clean teardown; the headless chain on a scratch
copy of the purged process as before. Not done, pending the user: an owner or lock file for `runs/`.

### Re-review (approve with nits) and its fixes (2026-10-20, on yh/core-port after M5)

- "Newest" is now the run **used last**: `find_runs` orders by progress.json's `updated_at` (to the microsecond),
  then the folder name; adopting a run (a restart, or Continue match) saves its progress, so after Continue match the
  next restart continues that run without asking. While runs are offered nothing is made current or selected:
  Continue or New run selects.
- A second Continue while the first still runs (or while a run is being started) is refused ("a run is being
  started: wait a moment").
- A soft stop during the 15 s wait for the robot makes the first pending action current at once (a done callback on
  the waiting task), so Mark done still has an action.
- `new_run` builds everything that can fail before it makes the run folder: no empty `runs/<name>_2/` is left. A
  carried movement that is no index ("1", true) becomes the action's first step, in `new_run` and in the plugin.
- A run folder that can't be read and was made after the run on these files stops the silent continue: that run is
  offered (offer = match), the panel says "newer run unreadable". `find_runs` returns (folder name, why) per problem.
- Messages: Continue match with no match says "no run made on these files is offered"; an open that raises after
  the read says "could not open the run (…)"; a run a setting started says "run <name> started"; `_scan` typed.
- Tests: test_runs (+3: used-last order, a carried movement that is no index, a failed new run leaves no folder),
  test_assembly_run (+5: Continue match then restart, a second Continue, a soft stop during the wait, an unreadable
  newer run, the messages); each fails on the code before.
