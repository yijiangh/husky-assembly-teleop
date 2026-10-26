# M6b: one-mate bars: no spiral, the holding arm finishes its line (spec, 2026-10-26)

Follows M6 (`tasks/2026-10-23_m6_compliance.md`, 2b8aa26 on `yh/m6-compliance`). A one-mate bar is an insert held by
two arms where one arm's joint screw tightens (`InsertPlan.mating` one name, `arms` two): B21 and the vertical bars
right after the ground bars. A two-mate bar has both arms mating; a ground bar none. The user's decisions of
2026-10-26 are final.

## Decisions
- D1 No spiral for a one-mate bar: with one joint correcting, a spiral turns the bar about it and skews it. The Spiral
  box stays where it is (nothing jumps) but is `disabled` (greyed) and shows unticked unless the run's current step's
  insert has two mating joints; the stored `spiral` setting is kept; a one-mate bar's `plan_insert` runs the straight
  line and its prompt says "straight line: no spiral for a bar with one mating joint".
- D2 A one-mate bar's compliant part ends per arm: at the stall the MATING arm holds at its TCP and goes back to the
  joint controller at once (as today); the HOLDING arm streams the rest of its planned line under compliance until its
  end target has gone out (elapsed ≥ T_c; no timer, no new constant), then holds and goes back. In the lab the abrupt
  switch of the holding arm, still settling against the force, left the bar skewed. A stall after T_c (during the
  hold) or before the first compliant target (during the rigid part or the switch) ends both at once, as today.
  NO_STALL, Cancel, Stop all: as today. Two-mate and ground bars: unchanged. The record keeps each arm's switch back
  as phases.

## 0. Behaviour, with the code
### 0.1 The rule (`shared/execution/spiral.py`, next to `spiral_problem`)
`spiral_fits(mating: Collection[str]) -> bool` = `len(mating) >= 2`. ! It lives in a `shared/` subfolder, never in a
contract file (`test_layers.py::test_shared_subfolders_never_import_contracts`): `plan_insert` and the panel import it.

### 0.2 Plan (`shared/execution/insert.py:plan_insert`)
After `spiral = None if grounded or rigid_only else spiral`: `line = "straight line"` (moved up), then
`if spiral is not None and not spiral_fits(mating): spiral, line = None, "straight line: no spiral for a bar with one
mating joint"`, then the existing `spiral_problem` check (a one-mate bar's too-fast spiral is ignored, not refused).
The MIN_AXIS fallback stays. `plan.spiral` and `as_record()["spiral"]` are None; the run's `command.settings.spiral`
still records what was asked. `plugins/movement_exec/sequences.py:_insert` is unchanged.

### 0.3 Run (`insert.py:run_insert`, `_compliant`, new `_finish_holding`)
- `run_insert`: beside `gate` a `back_early: set[str]` (arms whose early switch back was confirmed). `_compliant` gets
  `record, phase, back_early` and appends the compliant phases itself (time order). The `finally` leaves compliance
  for `[arm for arm in arms if arm.config.name not in back_early]` only: `arm.hold()` under the joint controller
  publishes a single-point trajectory, so an arm already back must not be held again. STOP, `restored` → FAILED, the
  log line: unchanged.
- `_compliant`, straight line: `stalled_at = inf`; `going(elapsed)` sets `stalled_at = elapsed` and returns False once
  `gate.all_stalled()` (the `run_spiral` pattern: `stream` does not return its elapsed); after the stream,
  `phase("compliant", since, ended)`; `holding = [arms not in plan.mating]`; if `ended == "stalled" and holding and
  0.0 < stalled_at < segment.seconds`: `ended = await _finish_holding(...)`. Spiral branch: `phase("spiral", ...)`.
- `_finish_holding(ctx, robot, plan, segment, stalled_at, holding, gate, recorder, phase, back_early) -> str`:
  1. `ok = await leave_compliance(ctx, [the mating arms], label)`; `phase("<mating> back", since, "back"/"not back")`;
     not ok → return "stalled" (the finally asks every arm again; the loud error was logged; FAILED via `restored`).
  2. `back_early.update(plan.mating)`.
  3. `rest = Segment({n: line_target(start[n], end[n], seconds, stalled_at) for n in holding}, {n: end[n]},
     segment.seconds - stalled_at, 0.0)`: the holding arm's line goes on from the point reached at the stall tick.
  4. `to_the_end(elapsed)`: False once the end target went out (`elapsed >= rest.seconds` seen on the previous call);
     the stream ends one tick after τ = 1, nothing more.
  5. `recorder.phase = "compliant <holding>"`; one `log_info`: "<serial>: insert: <gate.text> STALLED at X of T_c s;
     <mating> back under joint control, <holding> finish the line under compliance (Y s)"; `ended = await stream(ctx,
     robot, rest, keep_going=to_the_end, recorder=recorder)`; `phase(recorder.phase, since, ended)`; return "stalled"
     if ended == "ended" else ended (a base drift or refused target here fails the insert as any stream failure).
- Cancel during the rest: CancelledError out of `stream` → the finally: STOP, `leave_compliance([holding])`; the mating
  arm is in `back_early`, no second hold. Cancel during the early leave: not yet in `back_early` → asked again in the
  finally (a duplicate restore task, harmless). `_dry_run` unchanged.
- Record: `phases` = rigid, switch, compliant ("stalled", t1 = the stall), "<mating> back", "compliant <holding>"
  ("ended"); `switch.back_s/back_ok` = the holding arm's; the samples' `phase` names the arm; the movement_exec live
  line shows `recorder.phase`.

### 0.4 Panel (`plugins/assembly_run/panel.py`, `plugins/assembly_run/plugin.py`)
- `PanelStatus` + `spiral_fits: bool`. `RunPanel.__init__`: the Spiral box `disabled=True` at start (no step yet), hint
  "The insert's compliant part searches in a spiral; only for a bar with two mating joints, greyed otherwise".
- The settings loop: `ctx.defer_value(setting, ..., operator_only=True)`. * viser fires `on_update` when the plugin
  writes `widget.value` (skipped only when unchanged): while blocked the box shows unticked, which must not reach
  `request_settings`. `PluginContext.defer_value(operator_only=True)` exists for this; the fake ignores it, so a test
  cannot show a regression: say so in the comment.
- `RunPanel.draw`: key `(view.settings, status.settings_refused, status.spiral_fits)`; `self._spiral.disabled = not
  status.spiral_fits`; `self._spiral.value = settings.spiral and status.spiral_fits`.
- `AssemblyRunPlugin.draw`: `PanelStatus(..., self._spiral_fits(ctx))`. New `_spiral_fits(ctx)`: `transfer =
  ctx.require("process").transfer_as_free`; `found = current_step(self.view, transfer)`; None → False; `part =
  part_plan(self.view.process, action, steps[index].primary, ctx.config.robots, transfer)`; `return part.kind ==
  INSERT_PART and spiral_fits(part.tools)`. Imports: `INSERT_PART, current_step, part_plan` from
  `shared.movement_exec_contract` (already pure and shared; `writes.py` imports from it), `spiral_fits` from
  `shared.execution.spiral`. Per tick, as `assembly_steps._follow` already pays. `writes.py` unchanged.

## 1. Files
| File | Change |
|---|---|
| shared/execution/spiral.py | `spiral_fits` (§0.1) |
| shared/execution/insert.py | §0.2, §0.3; module / InsertPlan docstrings one line each |
| plugins/assembly_run/panel.py, plugin.py | §0.4 |
| test/plugin_fakes.py `FakeGui.add` | the widget also gets `disabled=options.get("disabled", False)` |
| README.md movement_exec row | "a bar with one mating joint runs the straight line whatever Spiral says (the box is greyed then), and after its stall the other arm finishes its line under compliance before switching back" |
| tasks/2026-10-23_m6_compliance.md Decisions | one bullet pointing here; lab step 4 (the spiral) needs a two-mate bar (B22), not B21 |

## 2. Tests (all `safety`; the fake clock, never wall time; 0.15 s = three ticks of margin)
- `test_execution_spiral.py`: `spiral_fits` of one arm False, both True, none False.
- `test_execution_insert.py` (`Lab.plan()` is B21-like = one-mate; ROWS 14.6 mm, split 5 → T_c 1.71 s): the spiral
  tests move to `mating=ARMS` (`test_a_spiral_too_fast_is_refused_at_plan`, `test_the_spiral_settings_reach_the_spiral`
  with both screws stalling at 6 s, the spiral dry run); `test_a_mating_insert_runs_rigid…` (stall at 6 s > T_c)
  adds: both holds in the same tick. New: a one-mate bar ignores the spiral and the prompt says so (two-mate keeps it);
  a one-mate bar's holding arm finishes its line after the stall (`Lab(left={"screw_stall_after": 2.0})`,
  `plan(split_mm=30.0)` so the whole 5 s path is compliant: STALLED ok; phases `switch, compliant, left_ur_arm back,
  compliant right_ur_arm`; left: switches `[COMPLIANCE, JOINT]`, last target a hold at its TCP, no single-point
  trajectory; right: last streamed target = the line's end, its hold ≥ 2.5 s after the left's, many more targets than
  the left; commands left `[STOP, TIGHTEN, STOP]`, right `[STOP, STOP]`; `running() == [JOINT, JOINT]`;
  `record["switch"]["back_ok"]`; `json.dumps(record)`); a two-mate bar ends both arms at the stall (phases `switch,
  compliant`, both holds in the same tick); cancel while the holding arm finishes: right STOP, hold, JTC; left no
  second hold; `[JOINT, JOINT]`.
- `test_movement_exec.py::test_the_insert_settings_are_read_once_and_reach_its_plan`: the spiral labs move to B22_J;
  at B21 with `spiral=True` the prompt says "straight line: no spiral for a bar with one mating joint",
  `command.settings.spiral` True, `command.plan.spiral` None.
- `test_exec_compliant.py` (slow): `STALL_AFTER = 1.0` (inside B21's 1.7 s line; at 2 s the old test never left the
  hold case): phases `rigid, switch, compliant, left_ur_arm back, compliant right_ur_arm`; the last ≥ `compliant_s −
  1.0 − 0.3` long; `switch.back_ok`.
- `test_assembly_run.py`: the Spiral box `disabled` at construction; on a by-hand step: disabled, value False, the
  stored `spiral` still True; on the one-tool insert step: still blocked; with `part_plan` patched to a two-tool
  INSERT_PART: enabled and ticked, and the box's callback reaches the plugin.

## 3. Verification
`-n 0 -m ""` on the files above and `test/structure`; everything (`-m ""`) before the commit.

## 4. Risks
- `operator_only=True` is load-bearing (viser echoes `.value` writes); the fake cannot show a regression.
- Only arms whose early leave CONFIRMED go into `back_early`; the finally holds the rest. Keep `0.0 < stalled_at`:
  a stall before the first compliant target ends both at once, else the holding arm would push its whole line
  against a closed joint.
- The rest stream's base-drift reference restarts at its own start; the Recorder's `base_drift_max_mm` spans the
  insert. A base drift or refused target during the rest ends FAILED although the screw stalled (the record shows
  it); the operator uses Next ▶ / Mark done (A3's rule).
