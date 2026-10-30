# Cross-check old vs new monitor for the process assembly test, and close the gaps

Spec and instructions for the branch `yh/lab_crosscheck_gaps` (from the plan of 2026-10-30; kept up to date as the work goes).

## Context

Su ran the full two-module process assembly test on the OLD monitor (manual `~/ros2_ws/src/husky-assembly-teleop/doc/three_robot_assembly_test_manual.md`, lab days 2026-10-20…29) and the new monitor had its first real-robot run on 2026-10-29 (`~/husky_real_lab/logs/monitor_20261029_152642.log`, run `…/261008_2ModulesDemo_purged_process/runs/20261029_155758/`). The user wants (1) a cross-check of retreat, insert (ground / one-mate / two-mate), ground-bar (MoCap) servo, tool0 servo and the logging/report infrastructure, (2) the old console printout and live plot replicated, (3) a toggle that draws ghost bodies of every tracked bar at its MoCap fit, with the fit stats per bar and per marker (dropdown), and (4) the "Old vs new monitor: features" doc (https://claude.ai/artifact/X6Aya43umHxwpdATFibWW1, Claude Docs id f3ab06b3-45f7-4c53-83c0-9e6f165bfe46, tab body node `bca01660-5a5b`) updated after the work.

### What the 2026-10-29 run showed (facts from the log and the servo records)

- Motive streamed no bar assets that day (the relay created publishers for 1840/1850/1860/1863 only), so every servo reading said `the bar could not be measured (no marker sample)`. The loop warned once, then silently aimed tool0 for 8 passes on B20 (transfer and after-insert), "ran all 8 passes without converging" (right flange residual 1.4–2.6 mm the bar-held linear planner cannot remove: spec D6), the force change on the transfer servo climbing to 22 N. The end reason went only to the panel line (`job.show`), never to the log. B21/B22 servos converged on tool0 in 1 pass.
- The Exec plots window shows nothing while a pass plans / waits for Confirm / moves / settles (up to ~10 s per pass), and its uPlot legend shows hover values, not the last pass.

### Decisions (the user, 2026-10-30)

1. Servo-by-bar rule stays D4: only a GROUND bar held in both tools steers on the MoCap bar.
2. Report: keep the old page (`4_assembly_viewer.js` + `viewer_common.js` unchanged); a NEW Python collector in the new repo builds the same scene-data from the new records.
3. As-built records go to the OLD Drive location `<drive>/data_experiment/bar_action_live_replan_exe/<YYYYMMDD>-asbuilt/as_built_<problem>.jsonl`.
4. Retreat: port the old STIFF screw-out as the default and the live tool0 −z goal (the compliant break-out stays only behind a run setting, for tests).
5. Servo by bar with no usable fit at reading 0: a WARNING prompt (Confirm = go on with tool0, Cancel = stop and record), and every pass says what it aims at.

## Cross-check: old vs new (the findings that go into the doc)

| Area | Old monitor (tested) | New monitor (core-port 65053b6) | Gap |
|---|---|---|---|
| Retreat goal | live tool0 moved 50 mm along its OWN −z (encoders), warn > 20° from the Rhino axis, retry shorter 5 mm steps down to 25 mm when blocked | the export's LineSpec (Rhino axis + distance) from the live flange; no shorten retry | port (decision 4) |
| Retreat control | STIFF: no FT zero, joint controller throughout, guard 0.57° from the planned start and line > 12 mm, both jaws LOOSEN + 12 mm lead at 2.4 mm/s (72 rpm, 2 mm pitch, 8 mm thread + 4 mm margin) in one tick, then 38 mm rigid, STOP tools; Cancel = rest not sent | gripper `open()`, FT zero, compliant break-out of the split (5 mm, ~0.5 s) under the compliance controller, back to JTC, tail replanned, checks (report only), tail | port (decision 4) |
| Insert ground bar | rigid only, no tighten, then a second servo (tool0 or bar) gated 3 mm | same (`plan_insert` grounded, AFTER_INSERT servo, GROUNDED_END_TOL 3 mm) | none |
| Insert one-mate | rigid to the split, compliant both arms, only the mating screw tightens (from the start), no spiral, per-arm end: stall → mating arm stiff, holding arm finishes its line and settles (0.5 mm / 0.5 s, 3 s ceiling) | same (M6b/M6c) | none |
| Insert two-mate | both tighten, ends when both stall, spiral optional (15 mm, 20 s, 4 turns, 2 s return, 30 s hold), no force limit | same | none |
| Servo math | 8 passes, 0.2 mm tool0, bar 0.5 mm / 0.1°, gates 30 mm / 3°, rms 3 mm, 20 N after a grounded insert, confirm pass 1 and > 5°, 3°/s, 2 s settle | ported unchanged (`shared/execution/servo.py`) | none |
| Servo by bar | any held bar with markers (since 10-27) | ground bars only (D4) | kept by decision 1 |
| Servo printout | `### SERVOING iteration k/8`, tool0 pos, load + force change, arms vs last waypoint, "First move planned …", `### SERVOING converged/stopped/ran all/cancelled`, `### bar pose at termination`, "Servoing data saved to …" | tool0 pos + track, the bar line, the planner's line, the prompt; NO end line in the log; no load line; no status while a pass runs | A1 |
| Servo live plot | Dear PyGui window, 4 history plots, x = iteration, opens at the first move | viser uPlot window (5 servo plots + force/torque), opens by itself; legend = hover values; no phase status | A2 |
| Marker source | Motive asset 10xx, else the unlabeled cloud; NatNet in-process | the relay's asset markers only (cloud deferred to M10) | documented; a loud check before the servo (A1) |
| MoCap bar fit | one bar followed live at 10 Hz; `[mocap bar]` 3-line report (summary, per-marker residuals + missing + strays, bar ends above the floor); cyan ghost of the measured bar; plate geometry (pair length, spacing, bow) | `mocap_bars`: one bar, 3-line readout, markers drawn, the bar mesh at the fit (no toggle); no per-marker readout, no plate geometry | C |
| As-built record | per executed step start/end, every streaming bar fitted, reference pose, tool0 FK vs target, wrench, base, joints → `<day>-asbuilt/as_built_<problem>.jsonl`; "record now" button | nothing (servo readings and movement records only) | B1 |
| Report page | `4_assembly_viewer.py` → `assembly_<day>.html` (+ Claude artifact) | nothing | B2 |
| Tick boxes in the report | parsed from `~/husky_real/monitor.log` | will be written into the record itself | B1 |

## Implementation plan

Four work packages, each its own side branch off `core-port` (A, B, C, D in that order; A and C are independent of B).
Every package: `pytest` before its commit, `pytest -m ""` before merging. Comments follow AGENTS.md "Writing comments".

### A. Servo printout, status and the Exec plots window (`plugins/movement_exec/`, `ui/trace_plot.py`)

Facts: reading 0 is always taken first, so x = pass is right today; the window looked empty because uPlot draws
lines only (one point is invisible) and nothing arrives between reading 0 and the end of pass 1 (plan + Confirm +
move + settle). The end reason is only `job.show`n. uPlot's legend with `live=False` shows labels, no numbers.

A1 `servo_measure.py`, new pure helpers:
- `aim_text(plan, engaged, bar)`: "the MoCap bar B20 onto the insert start (fit 8/8, rms 0.6 mm)" / "tool0 at the
  authored targets of the insert start (the MoCap bar B20 is not usable: <reason>)" / "tool0 at the authored targets
  of <target>".
- `load_line(k, reading, wrench0)`: the old W:2327 line `[servo k] load L=23.2 N / 2.05 Nm (+0.0 N vs iter 0) | R=…
  | force change since iter 0 L=… R=… N`.
- `planned_line(k, planner, move, flanges_move)`: `[servo] First move planned: N waypoints, travels x deg (ends y deg
  from where it starts), over T s; flanges move L a mm, R b mm (<planner>)`; later passes `[servo k] move planned: …`.
- `end_line(result, status, reason, k)`: `### SERVOING cancelled by user.` / `### SERVOING ran all 8 iterations
  without converging.` / `### SERVOING converged at iter k on tool0 (< 0.2 mm)` (or `… on the MoCap bar: dpos …`) /
  `### SERVOING stopped at iter k: <reason>` (`stopped before the first pass: …` when k = 0).

A2 `servo_part.py` (`ServoLoop`): fields `wrench0`, `line`, `phase`; `_phase(text, color)` → `job.status(text,
color)` (the Exec plots chip) and `job.show(f"{text} · {self.line}")`. Phases: "reading 0: waiting for the bar's
markers" / "reading 0: measuring" / "pass k/8: planning" / "waiting for Confirm" / "moving (T s)" / "settling 2 s" /
"measuring" / "<result>: <reason>". `_pass` opens with `### SERVOING iteration k/8: aiming <aim_text>`, logs
`planned_line` after the plan. `_measure` logs three lines in the old order: `[servo k] tool0 pos L=… R=… mm, rot
L=x/y/z R=x/y/z deg`, `[servo k] arms vs last commanded waypoint: L=… R=… mm` (after a move), `load_line`.
`close()` logs `Servoing data saved to <run folder>/<servo_file> (its .png beside it)`. `run_servo`'s `finally`
logs `end_line` (warn for FAILED/CANCELLED/HOLD_NOT_SENT/REFUSED) before `loop.close()`, so the order is end line →
bar pose at termination → carry → saved path, as the old.

A3 Decision 5, in `ServoLoop.run` after the on-target check: `if plan.by_bar and not self.engaged:` → warning
prompt through `sequences.confirmed(job, run, text, warn=True)`: `"<target> — SERVO by bar: no usable MoCap fit of
B20 (asset 1020): <reason>. Confirm to servo tool0 onto the authored targets of the insert start instead (the bar
is only recorded; the loop steers on it if a fit comes later); Cancel stops and records."`; Cancel → CANCELLED
"stopped before the first pass: cancelled at the bar prompt; nothing sent" (reading 0 has a bar entry, so the record
is still written).

A4 `sequences.Job` gets `status: Callable[[str, str], None]` (default no-op, added last so positional constructions
keep working); `plugin.request_execute` passes `self._plots.status`.

A5 `ui/trace_plot.TracePlot.__init__(..., points: bool = False, live_legend: bool = True)`: `points` adds
`viser.uplot.Series_Points(show=True, size=6)` to every data series; `live_legend` passes
`viser.uplot.Legend(live=...)` to `add_uplot`.

A6 `plots.ExecPlots`: servo plots built with `points=True, live_legend=False`; under each a fixed-row `values()`
readout (`ui.style.numbers`) with the LAST pass's numbers (tool0 pos: 2 rows L/R × x y z |d|; rot: 2 rows; base pos,
base rot, bar, force change: 1 row; first cell `pass k`; "—" when missing; reset by `start_servo`); a sixth servo
signal `force_change` (L, R, N) and its plot; `start_servo` sets `self._servo.start = 0.0` so x stays the pass even
if reading 0 were missing; `status(text, color)` writes the chip beside `servo: <label>` on the one chip line.

A7 Tests: `test/plugins/movement_exec/test_servo_part.py` (the old lines in the log, the status chip, the by-bar
prompt: Cancel records one reading and sends nothing [safety], Confirm goes on with tool0 [safety], the
ran-all-8 end line, the load line after an insert), `test_exec_plots.py` (points + legend options, reading 0 at
once, x without reading 0, force change, status chip).

### C. mocap_bars: every bar, ghosts, per-marker and plate stats, the `[mocap bar]` report, a contract

C0 `world/marker_fit.py` (pure): `MarkerRow` (name, state MATCHED/MISSING/STRAY, sensed_id, motive_mm, fit_mm) and
`marker_rows(fit, sensed, layout)` (port of old `marker_match`, BMF:1027); `PlateRow`, `SpacingRow`,
`PlateGeometry` and `plate_geometry(fit, sensed, layout)` (port of old `pair_geometry`, BMF:1072-1168, unchanged
math: plates grouped by `name.rsplit("/", 1)[0]`, design pair length from the layout); `bar_report(bar, fit, sensed,
layout, planned, deviation) -> (line1, line2, line3)`: the old three `[mocap bar]` lines (summary | residuals per
marker, missing, strays | bar ends world mm, Rhino and MoCap; no floor line: the new monitor has none).
`world/marker_layouts.fit_bar(..., full_search: bool = True)`: with a hint that fails and `full_search=False`,
return a failed fit "no match around the last pose (the full search waits its turn)".

C1 `shared/mocap_bars_contract.py` (new): `BarMeasurement` (bar, mocap_id, fit, pose, sensed, source, measured_at,
tracked, planned, deviation; `to_json`) and `MocapBarsContract` Protocol: `fits -> Mapping[str, BarMeasurement]`
and `measure_now(hints=None) -> dict[str, BarMeasurement]` (a full fit of every tracked bar from its newest sample,
for the as-built record).

C2 `plugins/mocap_bars.py`: on a process load track EVERY bar with a layout as `fitted_bar/<B20>` (asset
`bar_asset_id`, warn when a bar has none); the `Bar` dropdown + `Mocap ID` + `Track` re-track only the chosen bar
with an override id. Per-bar `_Followed` state; `update` fits each bar at most every FIT_PERIOD with the hint (last
good pose within 1 s, else the planned pose), and at most ONE full search per tick across bars (round robin,
`SEARCH_RETRY` 1 s after a failed search). Checkboxes `Show markers` (as now), `Ghost bars` (new, on by default:
every fitted bar's body + mounted halves drawn see-through in the old MoCap cyan (0, 0.8, 0.9, 0.5) at its fit;
hidden when unticked or without a fit; the chosen bar's markers/lines/planned label as now), `Log live` (the
three `[mocap bar]` lines of the chosen bar every 5 s), button `Measure + log` (a full fit now, then the three
lines). A `stats` section BELOW every button: a fixed-row `values()` block (rows = 4 + max markers + 2 × max
plates − 1 over the process's layouts): summary (source, age, n/N, rms, max, ambiguous, dropped, occluded), one
row per design marker (Motive residual, fit residual, matched/missing), strays, one row per plate (pair length and
error, roll, bow) and per spacing, the deviation vs the planned pose (dpos, dang, bar-frame dx/dy/dz). The stats
follow the `Bar` dropdown (one chooser). `fits` / `measure_now` as C1. Health: each tracked bar is a chip on the
"objects" row; a bar Motive does not stream shows red (true problem for a by-bar servo: keep).
The servo keeps its own `track_object` of the asset (two subscriptions on one topic are allowed; `monitor.
track_object` refuses only a duplicate name); switching `ServoLoop._bar` to the contract is a later 10-line change.

C3 Tests: `test/world/test_marker_fit.py` (marker rows, plate geometry on `fake_sensed_markers`: lengths 48.3,
a 2 mm shifted inner plate → bend ≈ 2, the three report lines), `test/world/test_bar_fit.py` (waits-its-turn),
`test/plugins/mocap_bars/test_mocap_bars.py` (every bar tracked, the id override, one full search per tick,
ghosts visible/hidden, fixed stats rows, Measure + log and Log live cadence, the contract).

Risk: main-thread cost ≤ 4 × 1 ms + 1 × 20 ms per tick only when new samples arrive; watch the `[ticks]` report.

### B. The as-built record and the assembly page (decisions 2 and 3)

B1 `shared/as_built.py` (new, pure; port of the old `as_built_io.py` where pure: `json_safe`, `append_record`
(makes only the day folder, one JSON line, flushed), `read_records` (drops a cut-off last line)). Schema v2 keeping
the old top-level keys the page expects: `schema_version, stamp, stamp_epoch, problem` (the process folder's name),
`run_id` (the run folder's name), `robot, serial, action {index, id, type, kind J/R/H/HR/P, bar, bar_id, robot},
step {index, movement_id, routine, part {index, kind, …, last}, phase start/end/manual, outcome, status, reason,
dry_run}, instrumented_bars, bars {bar_id: {bar_fit (BarFit.to_json), T_world_from_bar, sensed, raw, layout,
marker_source, marker_age_s, held_by, T_link_from_bar, reference_kind, T_reference, T_assembled,
deviation_vs_reference, deviation_vs_assembled (at the bar START, as the servo), milestone}}, robot_state
{base_tracked, base_pose, arms {side: {joint_names, joints, tool0_fk, tool0_target_end}}, fk_note}, tool0
(`servo.tool0_errors`), load {arms {side: force_N, torque_Nm, wrench}, zero {at, reason}}, servo` (the servo part's
file + last reading summary: `servo_block(payload, file)`), `options` (transfer_as_free, servo_by_bar + why +
engaged, spiral + spiral_fits, insert_mode rigid / compliant-one-mate / compliant-two-mate, dry_run, ignore_built,
obstacles_collide, split, spiral s/turns: NO monitor-log parsing any more), `robots` (every live robot via
`live_record`; the others filled from the run's `progress.robots` by `with_saved_robots`, source "assumed"),
`settings` (RunSettings). Dropped: beliefs, world_from_design, mocap_markers_path, commanded_end.
Functions: `as_built_path(drive_root, problem, day)` = `<drive>/data_experiment/bar_action_live_replan_exe/<day>-asbuilt/as_built_<problem>.jsonl`
(FileNotFoundError as `drive_folder` does), `day_of(stamp)`, `milestone_of(action_type, routine, part_kind,
phase, is_active, last_part, after_insert)` (the old 11 milestones re-expressed in routines: J transfer's last
part end → before_insert; J insert's last part end → after_insert; J operator step after the insert end →
after_fix; R retreat start/end → before/after_retreat; H approach/linear/handoff ends → hold_approach /
hold_at_grasp / hold_grasped; HR open/linear ends → hold_released / hold_retreated; other bars → bystander;
manual → None), `reference_of(process, action_id, movement_index, bar, phase, servo_bar)` (the old
`_as_built_bar_reference` on the core: servo_target / assembled / resting via `relations.placement` /
`held_authored:<movement>` = the flange's authored `link_poses` ∘ grasp / held_no_authored_pose), `build_record(...)`,
`bar_lines(record)` (the `[as-built]` per-bar texts).

B2 Where it fires: `movement_exec` records itself (`requires` += "mocap_bars"; the smallest: everything needed is
in Job/PartRun/ExecRequest; a separate recorder would need new contract events). At the START of a request's
first part and the END of every part (skipping WITH_MOTION and REFUSED parts), in `plugin._run` before
`await run_servo/run_part` and in `_record_part` (which `_run`, `_cancelled` and `_raised` all call). New
`_as_built(job, run, phase)` wrapped in try/except → `[as-built] <phase> of <mv> NOT recorded: <error!r>`; bars from
`ctx.require("mocap_bars").measure_now(hints)` (hint = the reference pose, else FK ∘ grasp, else the seat: ~1 ms
per bar, a full search only for a bar that moved). New fields: `PartRun.servo_plan` (set in `run_servo`),
`PartRun.servo` (= `servo_block`, set in `ServoLoop.close`), `Job.ft_zero: dict[serial, {at, reason}]` filled by
the travel-to-load zero and the handoff's. Manual: `MovementExecContract.request_as_built()` + button **As-built:
record now** in the Exec panel below Dry run. Log lines (old wording): `[as-built] <mv> <phase> (<outcome>): B20
[before_insert] fit ok 8/8, rms 0.6 mm | vs held_authored:<mv> dpos 0.4 mm, dang 0.030 deg` / `… B20 fit FAILED
(<reason>) | no reference (<kind>)` / `… no bar measured` / `… recorded -> <path>` / `[as-built] no bar with MoCap
markers in this process: nothing recorded` (once per request).
`assembly_run/writes.py`: a `kind == "as_built"` branch in `_queue_record` → writer thread `_append_as_built(folder,
ros_time, action, movement, record, data, drive_root)`: `with_saved_robots`, `append_record(as_built_path(...))`,
then an `as_built` event in `events.jsonl` naming the Drive path; a dedicated done-callback logs `recorded -> path`
or the NOT recorded line. `runs.KINDS` unchanged.

B3 The page: `data/bar_holding_acc_data/4_assembly_viewer.js` and `viewer_common.js` copied UNCHANGED from the old
repo; `scripts/fetch_three_js.sh` (three-r185 into `data/bar_holding_acc_data/vendor/`, gitignored).
`husky_assembly_teleop/experiments/as_built/{collect,page,env}.py` + `scripts/as_built_page.py` (argparse + main:
`day` default newest, `--problem`, `--process` default `<drive>/data_design_study/<problem>`, `--drive`, `--out`
default `<runs>/<day>-asbuilt-result/assembly_<day>.html`, `--open`, `--no-robots`, `--env-3dm`, `--no-cameras`,
`--vendor`). `collect.py`: ported verbatim `shape_bar`, `add_history`, `ball_check`, `ball_findings`, thresholds,
`METRICS`, `EXTRA_FORMULAS`, `MILESTONE_LABEL`, `PAGE_CSS`, `PAGE_CONTROLS`, `collect_assembly`'s loop (bar ends via
`world.bar_pose.bar_ends`, origin at the middle). Replaced: `step_options` → `options_of(record)` (same labels and
why-texts from `record.options` + `record.servo`), `servo_numbers` → `record.servo`, `load_numbers` →
`record.load`, `tool0_numbers` → `record.tool0`, `record_robots` → `record.robots` with `RobotModels` on the
process's URDFs (`kinematics.link_poses`, `urdf.link_geometry`), `timeline(process)` from the schedule (J/R/H/HR
letters, `exists`, `held`), `assembled_bars(process)` from the assembly poses, marker rows and plate geometry from
`world/marker_fit.py` (C0). `page.py` = `viewer_common.py`'s `write_page`, `pack_link_meshes`, vendor handling
verbatim; `env.py` = the lab json boxes (`world/lab`) + the optional 3dm room/cameras (rhino3dm, lazy). Page words
identical to the old page; the finished HTML is then published as a Claude artifact as before.

B3b (the user, 2026-10-30, while A was built): a button in the Assembly tab, **Build as-built page**, that compiles the
page from the records so far (any time, mid-experiment) on a worker thread and shows it: the monitor serves the result
folder over HTTP (a small `http.server` thread, port 8082 by default), the panel shows the link, the log the path.
No artifact needed for that; publishing stays a separate step.

B4 Tests: `test/shared/test_as_built.py` (the old io tests ported, milestones table, `reference_of` on
`process_fixtures.build_process`, saved robots, servo_block, missing Drive), `test/plugins/movement_exec/
test_as_built.py` (two records per request start/end with a fake mocap_bars contract on `fake_sensed_markers`,
retreat → before/after_retreat, cancelled → outcome cancelled, measure_now raising → error line and the step still
DONE, the manual record and button), `test/plugins/assembly_run/test_assembly_run.py` (the Drive line + event,
a missing Drive folder said not raised), `test/experiments/test_as_built_page.py` (slow: a fixture jsonl + the
fixture process → scene keys, `options_of` words, the html; skipped without the vendor files).

B5 Docs: `doc/run_record.md` "The as-built file"; `doc/real_lab_test_manual.md` a short as-built part (the old
manual's §9 in the new words: when, path, expected lines, the if-not table, the page command); README rows
(movement_exec, assembly_run, scripts); `doc/plugin_roadmap.md` mocap_bars row.

Order: B1 → writes branch → movement_exec hook (needs C1's `measure_now`) → page → docs.

### D. The retreat: stiff screw-out and the live −z goal (decision 4)

D1 `shared/execution/retreat.py` rewritten around the old stiff screw-out (W:5645-5732). Constants:
`SCREW_PITCH_MM 2.0`, `SCREW_ENGAGEMENT_MM 8.0`, `SCREW_OUT_MARGIN_MM 4.0`, `SCREW_RPM 72.0` → `SCREW_OUT_MM 12`,
`SCREW_RATE_MM_S 2.4` (5.0 s); `START_TOL 0.01 rad`; `STILL_WAIT = MOVING_WITHIN + 1 s`. `RetreatPlan` = the
first 12 mm of the planned joint path as a constant-speed `MotionCommand` (row 0 live), the rest rows with their
share of the slider time (`split_rows(..., from_start=True)`, `split_seconds`), the goals and tool0 paths for the
checks, the prompt text: `"RETREAT, stiff: the M1 screws of left_ur_arm + right_ur_arm LOOSEN while both tools back
straight out 12.0 mm at 2.40 mm/s (5.0 s, joint controller), then 38 mm rigid (3.8 s). Watch both tools let go and
back off along their lines."`; a plan not longer than 12 mm is refused with the old text. `run_retreat`: prechecks →
the path checks (length / end / backtrack / turn / joint travel / speed only, report once before anything is sent)
→ `stop_and_clear(GRIPPER_MOTOR)` → in ONE tick `tool.open()` on every tool (log `LOOSEN the gripper screws of …`)
and `send_motion(screw_out)` (phase "screw-out"; the motor states logged on every change as `screw-out t=0.3s:
gripper motors L/R LOOSENING/LOOSENING`) → `screw-out done after 5.x s (gripper motors … still loosening); now the
rest of the way out.` → wait ≤ STILL_WAIT for the arms to stand still (the arm refuses a trajectory within
MOVING_WITHIN of its last) → the rest rebuilt from the live joints (over START_TOL from the cut → FAILED, nothing
sent), phase "rigid" → `finally: stop_all(tools)` → `RETREATED: retreated 50 mm; the gripper motors stopped (…)`.
No FT zero, no controller switch, no tail replan. Cancel during the screw-out: the rest is not sent, every arm
held (new: braked, not left to finish), tools STOP, the part CANCELLED, logged as a warning. Record: `phases`
screw-out / rigid, `motor_states`, `screw_out`, `rigid`, `grippers_at_end`, peak force, samples.
The compliant break-out (today's code: FT zero, compliant stream of the split, leave compliance, tail replan,
`_replan`) stays ONLY behind `RunSettings.retreat_compliant: bool = False` (panel box "Retreat compliant", test
mode, with the old 4 s hold `RETREAT_COMPLIANT_HOLD_S` before its 12 mm line) as chosen. ? Dropping it instead
saves ~250 lines and 8 tests (it stays in git at 65053b6); say so at review.
`retime.constant_speed(path, duration)` (even samples along the arc, zero end velocities) + `motion_command(...,
constant_speed=False)`: the screw-out runs at the screw rate throughout, not on the cosine profile (which would peak
at 3.8 mm/s and pull on the screws).
`sequences.py`: `live_path_of/live_path/_along_plan` take a `tolerance` (the retreat passes START_TOL 0.57°, the
others keep STALE_ARM 2°), message `the arms are 0.60° from the plan's start (over 0.57°): replan`; `_retreat`
calls the new plan/run; `_replan` and the tail imports go unless the compliant variant keeps them. Texts in
`movement_exec_contract.py:285` and `assembly_steps/panel.py:224` follow.

D2 The goal from the live tool0: `route_result.live_line_goal(start, distance)` (the start moved `distance` back
along its own −z, orientation kept), `axis_angle(start, line)`, `RETREAT_AXIS_WARN 20°`, and
`live_retreat_goals(movement_id, spec, ends, live) -> (goals, info line, warnings)`; `assembly_steps.make_search`
passes them as `goal_poses` (where the insert's carried goals already go) when `RunSettings.retreat_live_axis`
(default True, panel box "Retreat live axis") and logs `[retreat live] B20_R_retreat: goals = the live tool0 poses
moved 50 mm back along their own -z (encoders): left along (…), 0.1 deg from the Rhino axis, 1.2 mm from the
exported goal; right …` plus the > 20° warning `the left tool axis is 25.0 deg from the Rhino retreat axis; check
the tool before Exec.`; the plan event records `goal_live`. `tamp_route._retreat_linear`: when the full line is
blocked, retry shorter in 5 mm steps down to 25 mm (`RETREAT_SHORTEN_STEP`, `RETREAT_MIN_DISTANCE`), warning
`the full 50 mm retreat has no collision-free straight line (…); planned a SHORTER one: 35 mm back; the free move
home starts from there`, else `no straight retreat of 25-50 mm: …`.

D3 Tests (`safety` where a command changes): `test/shared/test_execution_retreat.py` (the plan's cut at 12 mm and
5.0 s with even 2.40 mm/s steps, the too-short refusal, LOOSEN + screw-out in the same tick then the rest then
STOP, tools that never stall don't hold it up, cancel during the screw-out sends no rest, dry run, refused),
`test_execution_retime.py` (`constant_speed`), `test_execution_send.py`, `test/plugins/movement_exec/
test_movement_exec.py` (the stiff prompt text, ungrasp done, the 0.57° refusal), `test_exec_compliant.py`
(the retreat scenario rewritten: no switches, every arm under the JTC), `test/shared/test_planning_tamp_route.py`
(slow: blocked at full length → shorter with the warning; blocked within 25 mm → refused; the existing
`test_a_retreat_into_a_box…` expectation changes), `test/plugins/assembly_steps/test_assembly_steps.py` (the live
goals, the setting off, the 25° warning), `test_assembly_run.py` (the two new boxes).

D4 Docs: README movement_exec row, `doc/plugin_roadmap.md`, `doc/run_record.md` (retreat phases), the fake-lab
and real-lab manuals' retreat lines (the old manual 5.3 a) in the new wording).

Risks: the rest's send needs the arms still for 0.5 s (a ~0.5–1 s pause at the cut the old did not have; one joined
command is the alternative); `constant_speed` samples by joint arc (equal to tool travel on a straight line).

Landed (2026-10-30, commit "retreat: the old stiff screw-out …"): as D1–D4, with these differences. The stiff plan is
its own `StiffRetreatPlan` (`plan_stiff_retreat` / `run_stiff_retreat`); `RetreatPlan` / `plan_retreat` /
`run_retreat` stay as they were for the compliant variant, which kept its tail replan and did NOT get the old 4 s
hold. The box tests in `test_planning_tamp_route.py`: a 2 cm box 8 cm behind the left tool blocks every line down to
25 mm (refused, the full line's waypoint named), 10 cm behind lets a 35 mm one through (planned, the warning). The
fake arms arrive within 0.5° of a send's end (ARRIVED_TOLERANCE), so the executor test checks the end to that.

### E. Update the "Old vs new monitor: features" doc (Claude Docs id f3ab06b3-45f7-4c53-83c0-9e6f165bfe46)

After the code lands, with the connector (`update` on node `bca01660-5a5b`, `find` targets on the rows' words;
re-read with `sinceRev` first): rows 45 (retreat: stiff screw-out + live −z goal ported, compliant behind a
setting), 47 (servo: D4 kept, the by-bar warning prompt, aim lines), 48 (Exec plots: last-pass readout, status
chip, force-change plot, the old end lines in the log), 53 (mocap_bars: every bar, ghost bars toggle, per-marker and
plate stats, the `[mocap bar]` report, the contract), 54 (as-built record on the Drive + `scripts/as_built_page.py`
→ Ported), and a new row for the cross-check findings that stay as differences (marker cloud fallback not ported;
servo by bar ground-only). The "Still missing" paragraph and the "Last change" line get the new branch and task
notes. Also update the dropdown statuses (enum `d5aa6403-153e`: 0 Ported, 1 Changed, 2 Partly, 3 Not ported,
4 Dropped).

## Verification

- Unit: per package the folder/marker sets above with `-n 0`; `pytest -m safety` after D; `pytest` before each
  commit; `pytest -m ""` before merging into `core-port` (and `test/structure/test_layers.py` for the new
  `shared/mocap_bars_contract.py` and `shared/as_built.py`).
- Fake lab end to end (`doc/fake_lab_test_manual.md`, `scripts/fake_lab/`): B20_J with "Servo by bar" on and no
  bar markers → the warning prompt before pass 1, then the `### SERVOING …` lines, the Exec plots chip moving
  through the phases, the last-pass readout under each plot; the `[as-built] …` lines at each part's start/end
  and the jsonl under `<drive>/…/<day>-asbuilt/`; B20_R → the stiff prompt, LOOSEN + screw-out in the log,
  `RETREATED`; the mocap_bars tab with four bars tracked, Ghost bars on, the stats block; then
  `python scripts/as_built_page.py <day> --open` builds the page and it opens with the slider, pills, cards.
- With the fake Motive streaming B20's markers (`fake_sensed_markers` / the sim relay): the by-bar servo engages,
  the ghost bar sits on the planned bar, the stats show 8/8 and plate lengths 48.3.
- Then publish the page as an artifact and update the features doc (E).

## Task note

`tasks/2026-10-30_old_vs_new_crosscheck.md` = this file (context, cross-check table, decisions, A–E), updated as
the work goes; each package may add its own note as the agents proposed.
