# M6: compliant execution — insert, retreat, spiral, Robotiq handoff (spec, IMPLEMENTED 2026-10-26 on yh/m6-compliance; lab test pending)

Ported from 0e45758 onto Jakob's runtime: four behaviours → seven small modules in `shared/execution/` + four new part
kinds in movement_exec. The math below is copied from the old code with line numbers. Every place the new runtime forces a
difference, or where this spec adds a guard, is marked ⚑. The user settled A1–A6 and Q1–Q5 on 2026-10-23 (Decisions,
below); the implementer asks nothing more about them. One answer differs from the proposal: A5, the rigid-only mode and
the spiral's duration and turns are run settings again.

Old code at 0e45758 (verified): W = husky_world.py, M = husky_monitor.py, R = husky_robot.py, BIO = bar_action_io.py,
MAN = doc/three_robot_assembly_test_manual.md. Real locations: compliant insert `execute_planned_trajectory_compliant`
W:5268-5775; switch W:5619-5633, `switch_arm_controllers` W:4037-4114; streamed targets W:3973-4026, W:4121-4156; spiral
W:4703-4822, W:4962-5051, W:5437-5457; `retreat_guardrails` W:5054-5199; F/T zero W:5203-5265; retreat replan M:7790;
handoff W:6637-6869; M:242, BIO:854, BIO:885, R:837 as cited. New paths under
`ros2_ws_next/src/husky-assembly-teleop/husky_assembly_teleop/` at 145f72d.

> Paths moved since 145f72d (at 59760fe): `shared/planning/*` → `shared/assembly_planning/*` (route_result,
> route_search, tamp_route, movement, ik, joints, collision); `PlanRequest` → `shared/planner/plan.py` and
> `shared/motion_planner_contract.py` (`MotionPlanRequest`); `run_plan` → `plugins/motion_planner/work.py`;
> `shared/movement_exec.py` → `shared/movement_exec_contract.py`; `shared/runs.py` → `shared/runs/runs.py`; tests sit
> in `test/<area>/` (`test/shared/`, `test/plugins/<plugin>/`). The implementer maps every path here onto the current
> layout; the math and the behaviour do not change.

## Decisions (the user, 2026-10-23; final)
- A1 / Q1 Targets stay fixed in the WORLD: each tick the target is converted with the live mocap base, as the old code
  did (W:3999 with M:13476-13481). ⚑ Added: the stream stops if the base moves > 2 mm / 0.5° from where it was at the
  stream start. The base is not frozen at the stream start.
- A2 "Where the tool is" = the driver's measured TCP (`ArmState.tcp_position`). Before any switch to compliance, check
  that URDF FK of the live joints agrees with it within 10 mm / 2° — the handoff's own check (W:6643-6757), now on every
  compliant part (insert, retreat, handoff).
- A3 / Q2 A straight-line insert with no stall within 30 s ends FAILED (not done); arms back under joint control; the
  operator uses Next ▶ or Mark done; no Confirm prompt like the spiral's (the old code simply ended). The rigid-only
  insert's hold ends the same way (§0.5).
- A4 A failed tail replan in the retreat ends FAILED, no fallback tail (the old fallback's first step jumped, W:5691-5706).
- A5 / Q4 (changed from the proposal) The old rigid-only mode and the spiral's duration and turns sliders come back as
  run settings (M:786-798, sliders M:12931-12960): `RunSettings.rigid_only` (off), `spiral_seconds` 20 s (1–60 s,
  M:250), `spiral_turns` 4 (1–10 whole turns, M:251). The spiral radius stays 15 mm. Rigid-only runs the whole insert
  under joint control with the mating screws tightening, then holds there until they stall (§0.5); it ignores the split
  and the spiral, a ground bar ignores rigid-only (it is rigid with no tighten anyway), and the retreat ignores it
  (W:5385-5406, W:5432-5437).
  ⚑ Added: a spiral faster than 50 mm/s at full radius is refused at Plan (§0.4).
- A6 Cancel during the handoff stops the fingers (M5's rule); the old code let them keep closing (W:6816-6819).
- Q3 A failed retreat check only reports (log + record); the tail still runs (retreat step 9, §1.1).
- Q5 No force stop during the compliant insert, as before: peak |F| is recorded and the UR's protective stop applies (§4).
- M6b (2026-10-26, `tasks/2026-10-26_m6b_one_mate_bars.md`): a one-mate bar (one joint screw tightening, two arms)
  never spirals, the Spiral box greyed then, and after its stall the holding arm finishes its line under compliance.

## 0. Math, ported unchanged
### 0.1 Frames and where each pose is measured
| Symbol | Meaning | Old | New |
|---|---|---|---|
| B_t | base_footprint in the world at tick t | `hi.position` from mocap (M:13362-13394), live each tick (M:13476) | `robot.base.state` (relay); `robot.husky_in_world()` robot.py:158 = B_t·H |
| H | base_footprint → husky base_link | URDF | `robot._husky_in_mocap_frame` robot.py:121 |
| A_s | base_link → `<s>_base_link` (the controller's frame) | ghost link pose W:3999, turned by R:811-835 | `arm.base_in_husky` robot.py:119; via `arm.from_husky(*robot.from_world(·))` (arm.py:681, robot.py:176) |
| X_s | measured tool0 in `<s>_base_link` | `hi.arm_tcp_pose` (R:720-728) | `arm.state.tcp_position/orientation` arm.py:843-855 |
| FK_s(q) | URDF tool0 in `<s>_base_link` | ghost FK W:4410-4443, 4535-4559, 4865-4888 | core `link_pose(robot.config.model.urdf, …)` kinematics.py:90, relative to `<s>_base_link` |
| q_0…q_N | planned rows | `planned_arm_trajectory` | `route.result.trajectory`, q_0 := live joints (sequences.py:155-186) |
| e | seconds since the stream started | `time.time()` | `ctx.now()` (ROS time) |
Tick: 50 ms in both (old W:110; new config.py:179).

### 0.2 Split (W:4446-4527; MIN_RIGID_CHUNK_MM = 0.5, W:4407)
- d_i = max_s ‖p FK_s(q_N) − p FK_s(q_i)‖ (mm); D = d_0. Insert: a = split; retreat: a = D − split (from the start).
- a ≥ D − 0.5 → no leading part (one mode); a ≤ 0 → everything leading. Else i* = first i with d_i ≤ a;
  f = (d_{i*−1} − a)/(d_{i*−1} − d_{i*}) clamped to [0,1]; q_split = q_{i*−1} + f(q_{i*} − q_{i*−1}); leading =
  q_0…q_{i*−1}, q_split; trailing = q_split, q_{i*}…q_N; achieved a′ = max_s ‖p_s(q_N) − p_s(q_split)‖.
- ⚑ < 2 rows: the old code returns a 3-tuple (W:4479) while callers unpack four → the port returns the 4-tuple.
- Split setting `insert_split_mm`, default 5 (M:242), range 0–30 (M:243); the retreat uses the same value (W:5472).
- Times: T = traj time, default 5 s (BIO:528-529, W:5378). Insert: T_r = T(D − a′)/D, T_c = max(T − T_r, 0.5)
  (W:5416-5419). Retreat: c = D − a′; T_c = max(T·c/D, 0.5); T_tail = max(T − T·c/D, 0.5) (W:5484-5486).

### 0.3 Streamed target, every tick (W:3992-4024, W:4141-4156)
- τ = min(e/T_c, 1); p(G_s) = p(S_s) + τ(p(E_s) − p(S_s)) + Δ(e); R(G_s) = slerp(R(S_s), R(E_s), τ).
- Sent: (B_t·H·A_s)⁻¹·G_s (W:4021-4024) — linear in time (not `cartesian_move`'s cosine profile). Both arms in the same
  tick; the same Δ for both (keeps the grasp). The stop test runs BEFORE publishing each tick (W:4153). The stream ends
  when e > T_c + T_wait + 1 (W:3994, W:128).
- Old S/E: insert S = FK of live joints after the rigid part (W:5609-5611), E = FK(q_N) at exec start (W:5370-5373);
  retreat S = FK(planned q_0) (W:5365-5368), E = FK(q_split) (W:5483).
- ⚑ New, both: S_s = B·H·A_s·X_s at the handover tick (measured TCP); E_s = B·H·A_s·FK_s(q_end) with the base at the
  part's start.

### 0.4 Spiral (W:4721-4822; set up W:5437-5457)
- Axis a = (pE_L − pS⁰_L) + (pE_R − pS⁰_R) from FK of the PLANNED q_0 and q_N (W:5441-5442); ‖a‖ < 1e-6 → straight line.
- u = â×ẑ normalised (‖â×ẑ‖ < 1e-6 → â×x̂); v = â×u.
- f = clamp(e/T_sp, 0, 1); r = R·f, R = 15 mm; φ = 2π·n·f; Δ = r(cos φ·u + sin φ·v); T_sp replaces T_c (W:5448).
  T_sp = `spiral_seconds` (default 20 s), n = `spiral_turns` (default 4), read when the part starts (W:5448-5449);
  pitch R/n (3.75 mm at 4 turns; the old note keeps it under the joint's ~5 mm catch, M:12947-12950).
- ⚑ Speed at full radius v = (R/T_sp)·√(1 + (2πn)²): 18.9 mm/s at 20 s, 4 turns. `plan_insert` refuses v > 50 mm/s
  (the retreat checks' speed limit) and names the shortest duration for those turns, T_sp ≥ 0.3 s·√(1 + (2πn)²):
  1.91 s at 1 turn, 7.55 s at 4, 11.31 s at 6, 18.85 s at 10; the message rounds up to 0.1 s (2.0, 7.6, 11.4,
  18.9). That bounds each 20 Hz step to 2.5 mm, under F1's 1 cm.
- Segment A: T_wait = ∞ (W:5003). Segment B (after Confirm): from E + Δ(T_sp) to E over 2 s (W:4718), then wait ≤ 30 s
  (W:5038-5040).

### 0.5 Stop tests
| Phase | Ends when | Old |
|---|---|---|
| rigid part (insert) | every mating M2 stalled, or T_r + 1 tick | W:4665-4676 |
| rigid-only insert (A5) | the whole path as the rigid part (T_r = T): every mating M2 stalled, or T + 1 tick; then the joint controller holds the last row while the stall gate waits: every mating M2 stalled → STALLED, or 30 s → NO_STALL (FAILED, A3). Compliance never engaged | W:5389-5406, W:5596-5603, W:4928-4958, `HOLD_FOR_STALL_TIMEOUT_S` W:4701 |
| straight compliant line | every mating M2 stalled, or e > T_c + 30 + 1 | W:5501-5503, W:4701 |
| spiral A | stall; Cancel; at f = 1 a prompt; Confirm → B | W:5006-5023 |
| spiral B | stall; Cancel; e > 2 + 30 + 1 | W:5041-5051 |
| retreat compliant | runs while e < T_c; then stops once both arms within 2 mm of E; hard end e > T_c + 1 | W:5509-5522, W:4693-4697 |
| rigid tail | T_tail; no stall gate | W:5735-5739 |
| handoff | gripper result; drift > 10 mm; 30 s; Cancel | W:6796-6820 |
- Stall gate (W:4181-4226): a tool counts once it reported something other than STALLED since the gate was made and
  reports STALLED now; done when every mating tool counts. Mating tools = the kept tighten drives (movement.py:224-264,
  replacing BIO:885).
- Error metric (W:4891-4915), per arm: travel = ‖pE − pS‖, achieved = ‖p_now − pS‖, remaining = ‖pE − p_now‖; warning
  if the worst remaining > 2 mm (W:5659).
- Retreat checks (report only, W:5056-5199): start ≤ 0.01 rad from live; compliant part worst remaining ≤ 3 mm and
  left/right spread ≤ 3 mm; |F| ≤ 20 N; grippers STALLED (waits ≤ 3 s while LOOSENING); per-arm tail: length vs
  remaining ≤ 5 mm, end ≤ 5 mm, backtrack ≤ 2 mm, turn ≤ 3°; joint travel ≤ 0.35 rad; speed ≤ 50 mm/s.
- Tail goals: the original plan's retreat goals (M:9742-9785, used M:7826-7834); 1 mm / 0.05 rad steps (M:257-258);
  full scene collision checks.
- Handoff (W:6637-6869): anchor = FK(live q) (W:6739) vs reported TCP within 10 mm and 2° (W:6645-6646), else a stiff
  close; F/T zero then 0.5 s (W:6648); drift > 10 mm ends the hold (W:6641); each tick the fixed anchor + zero wrench
  (W:6787-6804); 30 s ceiling.
- Wrench: both arms every tick; peak |F| per arm, raw (includes tool and bar weight) (W:5538-5558, W:5766-5775).

### 0.6 Forced (F) and added (+) differences
| # | Old | New |
|---|---|---|
| F1 target checks | 5 cm per axis, 0.05 per quaternion component (≈5.7°) (R:850); a failed target was dropped and the stream went on | 5 cm per axis (arm.py:633), 5° (:636), reach 0.85 m (:640), unit quaternion, TCP ≤ 0.5 s old, and 1 cm / 3° from the previous target if sent within 0.5 s (:706-718); a refusal stops the stream, holds, fails the part. Worst steps at 20 Hz: insert 0.15 mm, widest spiral turn 0.94 mm at 20 s / 4 turns (≤ 2.5 mm for any accepted setting, §0.4), spiral return 0.38 mm, retreat ≤ 1 mm — all far under 10 mm |
| F2 tool start | STOP and TIGHTEN back to back (W:5567-5571) | `_drive` refuses while cached STALLED (end_effectors.py:348): STOP, wait ≤ 2 s for a non-STALLED status newer than the STOP (as `scaffolding_grasp`, tools.py:146-153), then drive; never clears → refuse before anything moves |
| F3 rigid part | dt = T/(n−1) (R:1040); by time; the switch cut the trajectory (W:4614-4618) | `motion_command` + `send_motion(until=gate)` (new: hold the arms before the switch); ARRIVED / STOPPED / ENDED all continue to compliance |
| F4 F/T zero | waited for the service ≤ 2 s + 0.1 s (W:5243-5253) | `zero_ft_sensor()` keeps no future (arm.py:778-790): fixed 0.5 s wait; log |F| before/after |
| F5 Robotiq result | `hi.gripper_result` | `not state.moving` and `last_result_ok` (= reached or stalled, end_effectors.py:276-287) |
| F6 compliant start | FK of live joints (insert); FK of planned row 0 (retreat) | the measured TCP after the FK-vs-TCP check (A2): no jump from FK/TCP disagreement or the 2° row-0 gap |
| F7 Cancel | ignored during the straight compliant part (W:5578) | Cancel, Hold and Stop all always stop the part safely (§4) |
| F8 ending | stop publishing, switch | hold (target = TCP, E1 arm.py:642), then switch, on every ending |
| F9 tail replan | blocking cfab call (M:7855) + fallback tail | async `motion_planner.request_plan` with `goal_poses` and 1 mm steps; failure → FAILED (A4) |
| + added | – | base-drift stop (A1); the stream's clock advances ≤ 0.1 s per tick (a late tick slows, never jumps); a zero target wrench right after each switch to compliance (robot_control may have left one, arm.py:745-746); `cartesian_test_mode` or `urdf_problem` → compliant parts refused; a spiral over 50 mm/s refused at Plan (§0.4) |

## 1. Files
### 1.1 New in `shared/execution/`
- `controllers.py` (~130): `SWITCH_TIMEOUT=5.0` (W:4031), `QUERY_TIMEOUT=2.0` (W:4034), `RESTORE_TIMEOUT=10.0`;
  `SwitchOutcome(ok, reason, seconds, active)`; `async switch_arms(ctx, arms, controller, *, timeout_s)` (port
  W:4037-4114: refuse if any `state.switch_in_flight` (controller_manager.py:131); `refresh()` and wait ≤ 2 s for a newer
  answer, else cached; all already active → ok without a request; `switch()` each, False → `switch_error`; then each
  tick until every arm `is_active` and none in flight; any `switch_error` or 5 s → fail; never falls back);
  `request_joint_control(ctx, arms)` (sync: `arm.hold()`, then switch to JTC where needed);
  `async restore_joint_control(ctx, arms, *, label, timeout_s)` (each tick: wait while in flight; an arm not under JTC
  with nothing in flight → request again (late compliance ack, W:6841-6858); timeout → loud error "ARMS MAY BE LEFT
  UNDER cartesian_compliance_controller: … switch them in robot_control before anything else");
  `async leave_compliance(ctx, arms, *, label)` (the finally helper: `request_joint_control` synchronously (goes out within
  the 2 s shutdown grace, config.py:186), then `await ctx.join(ctx.spawn("restore joint control …", restore_…))`
  (`join` is shielded: a second Cancel can't cut the restore); spawn refused (closing) → log "requested, not confirmed").
- `stream.py` (~230): `MAX_CLOCK_STEP=0.1`, `BASE_DRIFT=STALE_BASE` (basis.py), `ARRIVAL=0.002` (W:4697), `END_WAIT=1.0`
  (W:128), `FK_TCP_TOL=(0.010, radians(2))`; `Segment(start, end, seconds, wait)` (world poses);
  `PartOutcome(status, reason, seconds, record)` with statuses STALLED, RIGID_DONE, RETREATED, CLOSED (ok); NO_STALL,
  STIFF, CANCELLED, REFUSED, FAILED, DRY_RUN; `split_rows(rows, points, split_mm, from_start)` (§0.2); `tool0_fk`,
  `arm_base_in_world`, `measured_tool0`, `fk_tcp_gap`; `line_target(start, end, seconds, elapsed, lateral=None)`
  (§0.3); `progress(robot, segment)` (W:4891-4915); `plan_stream(segment, dt, lateral=None)` (offline: dry runs, tests);
  `target_problem(previous, target, tcp)` (the arm's time-independent checks); `Recorder` (one sample per tick; peak
  |F|); `async stream(ctx, robot, segment, *, keep_going, lateral=None, recorder) -> str`: each tick (1) advance e by
  ≤ 0.1 s; (2) `not keep_going(e)` → "ended"; e > seconds + wait + 1 → "time"; base drifted → "base moved X mm";
  (3) per arm `arm.send_cartesian_target(*arm.from_husky(*robot.from_world(G)))` (False → "refused: <arm>");
  (4) `recorder.tick`, `await ctx.next_tick()` (pattern of robot_control `_stream_cartesian_move`, plugin.py:540-577).
- `spiral.py` (~120): `lateral_basis(axis)` (W:4721-4747); `Spiral(axis, seconds, turns, radius=SPIRAL_RADIUS)` with
  `frac`, `offset`, `done`, `text` (W:4750-4822) and `speed` (m/s at full radius, §0.4); `SPIRAL_RADIUS=0.015`,
  `SPIRAL_SECONDS=20.0`, `SPIRAL_TURNS=4` (the settings' defaults), `SPIRAL_RETURN=2.0` (W:4708-4718),
  `SPIRAL_MAX_SPEED=0.05`; `spiral_problem(seconds, turns) -> str` ("" or, for 5 s, "spiral too fast: 75 mm/s at 15 mm, limit
  50; 4 turns need ≥ 7.6 s", the shortest duration rounded up); `async run_spiral(ctx, robot, segment, spiral, gate, operator, recorder) -> str` ("stalled" |
  "cancelled" | "no_stall" | a stream failure): A then B; the warning prompt opens once via `operator.open(text,
  warn=True)` and is polled each tick BEFORE publishing.
- `tools.py` additions (~60): `StallGate(tools, motor)` (W:4181-4226: `all_stalled()`, `stall_times`, `text`; a status
  older than `STATUS_MAX_AGE` never counts, logged once); `async stop_and_clear(ctx, tools, motor) -> ToolOutcome |
  None` (refactor `scaffolding_grasp` onto it); `stop_all(tools)`.
- `insert.py` (~240): `GROUNDED_END_TOL = 0.003` (W:1659, for M7); `HOLD_FOR_STALL = 30.0` (W:4701);
  `InsertPlan(serial, arms, names, rows, mating, grounded, rigid_only, split_mm, achieved_mm, total_mm, rigid:
  MotionCommand | None, compliant_seconds, ends, spiral, text, as_record)`; `plan_insert(robot, names, rows, *, mating,
  grounded, split_mm, rigid_only, spiral: tuple[float, int] | None (seconds, turns), traj_time) -> InsertPlan | str`
  (str: `spiral_problem`, or why not); `async run_insert(ctx, robot, plan, tools, *, operator, dry_run) -> PartOutcome`.
  Mode, as W:5385-5437: ground bar → rigid, no tighten; else rigid-only → the whole path rigid, the split and the spiral
  ignored (logged once); else the split, then the straight line or the spiral. Sequence (port W:5268-5775):
  (1) prechecks: every arm under JTC, no `urdf_problem`, not test mode, FK-vs-TCP within tolerance, every tool status
  fresh; (2) dry run stops here after `plan_stream` +
  `target_problem` on every target and `build_trajectory` on the rigid part; (3) `stop_and_clear(all tools, JOINT)`,
  `drive_screw(1)` on the MATING tools only; (4) rigid part `send_motion(rigid, until=gate.all_stalled)`; (5) ground
  bar: no tighten, the whole path rigid; record `end_gap_mm`; log "<who>: servo after the insert: M7, not run";
  (5b) rigid-only: no switch, no target sent; each tick until `gate.all_stalled()` → STALLED, or `HOLD_FOR_STALL` →
  NO_STALL (FAILED, A3) (W:4928-4958); the joint controller holds the last row; (6) else `switch_arms(compliance)`, zero
  wrench, S = measured TCP, straight line with `keep_going = not gate.all_stalled()` or `run_spiral` with
  `Spiral(axis, seconds, turns)`; `progress`; (7) `finally`: `stop_all(all tools)`, `leave_compliance`.
- `retreat.py` (~200): `CHECK_*` = W:5056-5065; `TAIL_STEP=0.001`, `TAIL_ANGLE=0.05` (M:257-258); `FT_SETTLE=0.5`;
  `plan_retreat(robot, names, rows, goals, *, split_mm, traj_time)`; `retreat_checks(...)` (pure);
  `async run_retreat(ctx, robot, plan, tools, *, replan: Callable[[], Awaitable[RouteResult]], dry_run)`. Sequence:
  (1) prechecks; (2) `stop_and_clear(tools, GRIPPER)`, `open()` (M1 loosen) on every tool of the moving arms
  (W:5572-5576) — before the F/T zero, as the old order; (3) `zero_ft_sensor()` per arm, wait 0.5 s; (4) switch to
  compliance, zero wrench, stream from S = measured TCP to E = FK(q_split); (5) `progress`; (6) `leave_compliance` (fails →
  FAILED, nothing more commanded, W:5674-5679); (7) `await replan()` (failure → FAILED, A4); (8) wait ≤ 3 s while any
  gripper LOOSENING; (9) log `retreat_checks` "check PASS/FAIL …" (report only, no second Confirm, MAN §7.3:822-826);
  (10) rigid tail `motion_command(tail rows, T_tail)` + `send_motion`; (11) `finally`: `stop_all`, `leave_compliance`.
  The `replan` callable comes from movement_exec (the shared layer never imports a plugin).
- `handoff.py` (~140): `MAX_DRIFT = 0.01`, `ANCHOR_TOL = (0.01, 2°)`, `FT_SETTLE = 0.5`, `TIMEOUT = 30` (W:6641-6652);
  `async run_handoff(ctx, robot, arm, gripper, *, dry_run) -> PartOutcome`: (1) gripper ready and not moving
  (tools.py:85-90); (2) `switch_arms({arm}, JTC)`; (3) zero F/T, 0.5 s; (4) anchor = measured X; > 10 mm / 2° from FK →
  stiff `robotiq_move(CLOSE)`, return STIFF (not ok) (W:6749-6757); (5) switch to compliance (failure → FAILED, nothing
  closed); (6) ⚑ send the anchor target + zero wrench BEFORE the close goal (refusal → FAILED, nothing closed; the old code
  closed first); (7) `gripper.move(0.8, 0.1)`; (8) each tick: drift > 10 mm → FAILED, else anchor + zero wrench; not
  moving → `last_result_ok` → CLOSED else FAILED; 30 s → FAILED; (9) `finally`: fingers still moving on cancel/failure →
  `gripper.stop()` (A6); `leave_compliance({arm})`.

### 1.2 Small edits
| File:line | Change |
|---|---|
| execution/send.py:158 | `send_motion(…, until: Callable[[], bool] \| None = None)`: when `until()` is true, hold every arm, return the new status `ENDED` |
| execution/operator.py:99 | `poll(prompt) -> bool \| None` (non-blocking; once answered cleans up like `wait`'s finally) and `close(prompt)` |
| planning/route_result.py:29 | `RouteResult.goals: dict[str, Pose]` (flange → world goal) |
| planning/route_search.py:273 | `solved` stores `{part.flange: goal}` |
| planning/service.py:37 | `PlanRequest.goal_poses`, `linear_step`, passed through `run_plan` → `plan_movement` (tamp_route.py:60) → `_Route` |
| planning/tamp_route.py:310-323 | `_retreat_linear`: use `goal_poses` when given (every moving flange needs one, else Refused); `max_step_distance = linear_step` or tamp's default 0.005 |
| shared/runs/runs.py:71 | `RunSettings` + `rigid_only: bool = False`, `spiral_seconds: float = 20.0`, `spiral_turns: int = 4` (an older run.json without them loads these defaults: the reader keeps known keys only); `SPIRAL_SECONDS_RANGE = (1.0, 60.0)`, `SPIRAL_TURNS_RANGE = (1, 10)` beside it (M:250-251); docstring: the split also sets the retreat's compliant part, from the start; rigid-only ignores the split and the spiral |
| plugins/assembly_run/panel.py (the settings section and its refresh) | under "Spiral": a checkbox "Rigid insert" (hint: the whole insert under joint control, the screw tightens, no compliance; the split and the spiral are ignored) and two sliders "Spiral s" (1–60, step 1, 20) and "Spiral turns" (1–10, step 1, 4), wired like the others and refreshed from `view.settings` |
| plugins/assembly_run/writes.py `_settings_problem` | `_settings_problem`: each number setting in its range (the split as now; spiral seconds; spiral turns a whole number), every other setting a bool |
| plugins/assembly_run/plugin.py `request_settings` | `request_settings` converts by the field's type (float for the split and the seconds, int for the turns), not by name |

### 1.3 movement_exec integration
- `shared/movement_exec.py`: part kinds `INSERT_PART`, `RETREAT_PART`, `HANDOFF_PART`, `WITH_MOTION_PART`;
  `MOTION_KINDS = {MOTION_PART, INSERT_PART, RETREAT_PART}`; `DEFAULT_TRAJ_TIME` += INSERT 5.0, RETREAT_LINEAR 5.0; `MOTION`
  listed explicitly as the six plain routines; `LATER = {TRANSFER_HELD: "M7"}`. `part_plan`: INSERT → kept tighten tools →
  their arms via the flange map, each must carry scaffolding_v3 → INSERT_PART; `drives.grounded` → INSERT_PART with no
  tools ("rigid, ground bar, no tighten"); neither → REFUSED "no mating arm: export error" (W:5333-5338); RETREAT_LINEAR →
  RETREAT_PART; a Robotiq "closed" grip → HANDOFF_PART. `step_parts`: an ungrasp folded into a retreat → WITH_MOTION_PART
  (its arms must equal the retreat's, else REFUSED).
- `plugins/movement_exec/sequences.py`: `run_part` → `world_problem(job, motion=kind in MOTION_KINDS)`; `_insert`,
  `_retreat`, `_handoff`, `_with_motion`. `_insert`/`_retreat`: `live_command(job)` (row 0 = live, ≤ 2°); read run
  settings once (split clamped 0–30, rigid_only, spiral with its seconds and turns; recorded); `plan_insert`/`plan_retreat` → prompt → `world_problem` again →
  rebuild with `live_command` → run → set `run.end`, `run.base` (as `_motion`, :244-247). `_retreat` builds the replan
  callable (a `PlanRequest` from `ctx.scene.snapshot` with `goal_poses=route.goals`, `linear_step=TAIL_STEP`, awaited via
  `request_plan` in a 60 s timeout). `_with_motion` DONE once the retreat opened its arms (`job.opened`). `Job` gains
  `opened` and an `operator` adapter (`open`/`poll`) recording prompt/confirm/cancel. `_by_hand` drops "(or drive the base
  there)" for by-hand parts — J_fix_foundation runs here.
- Call sites: `plugins/movement_exec/plugin.py:172, :178, :278`, `plugins/assembly_steps/plugin.py:289, :383` use
  `MOTION_KINDS`; `assembly_steps/panel.py:129` wording; the movement_exec panel shows one live line: phase · e · stalls ·
  |F| left/right.

Prompts (each "<robot> (<serial>): <movement> — "):
| Part | Text | Warn |
|---|---|---|
| insert, mating | INSERT: rigid {D−a′} mm ({T_r} s), then COMPLIANT {a′} mm (straight line \| spiral 15 mm, {T_sp} s, {n} turns, pitch {R/n} mm); the joint screw of {tools} TIGHTENS from the start until it stalls (≤ 30 s), then STOP and back to joint control. Watch the bar enter its joint and the held bars stay in their jaws. Confirm to start. | no |
| insert, rigid-only | RIGID-ONLY insert: the whole {D} mm under joint control ({T} s); the joint screw of {tools} TIGHTENS from the start; the arms hold the end until it stalls (≤ 30 s), then STOP. Compliance is never engaged; the split and the spiral are ignored. Watch the bar enter its joint and the held bars stay in their jaws. Confirm to start. | no |
| insert, ground bar | RIGID insert of a ground bar, no tighten, {T} s, largest joint travel {d}°. | no |
| spiral end | Spiral complete without a stall: HOLDING its end under compliance. Confirm = back to the straight-line end over 2 s, then hold for the stall (≤ 30 s); Cancel = end the insert here. | yes |
| retreat | RETREAT: the M1 screws of {tools} OPEN; force sensors zeroed; COMPLIANT break-out {c} mm ({T_c} s); joint control; the rest replanned from where the tools are and run at once ({T_tail} s, no second Confirm). Watch both tools let go and back off along their lines. | no |
| handoff | CLOSE + compliant handoff on {arm}: the arm goes COMPLIANT before the close (gives way ≤ 10 mm); jaws 0.8, effort 0.1, ≤ 30 s; then joint control. Watch Cindy's tools are not pulled; hand on the e-stop. | no |
| fix foundation | {label} by hand, then Confirm. | no |

Record `outcome` = `PartOutcome.record`: `{status, reason, seconds, settings, plan (split achieved/total, T_r, T_c,
T_tail, mating, grounded, rigid_only, spiral {seconds, turns, pitch_mm, speed_mm_s}), phases [{name, t0, t1, end}], switch {to_s, back_s, ok}, stall_times {arm: s after TIGHTEN},
spiral_text, peak_force_N, end_error_mm {arm: travel/achieved/remaining}, fk_tcp_gap, base_drift_max_mm, checks [lines],
tail {planner, waypoints, seconds}, end_gap_mm, stream [{t, phase, arms {target [p, q], tcp [p], wrench [6]}, motors
{arm: [m1, m2]}}]}` (≤ ~1300 samples).

## 2. Tasks
| # | Title | Files | Depends |
|---|---|---|---|
| A | controllers | controllers.py; `FakeControllers` in plugin_fakes.py; test_execution_controllers.py | – |
| B | stream + spiral math | stream.py, spiral.py; test_execution_stream.py, test_execution_spiral.py | – |
| C | tools: stall gate, clear, both motors in the fake | tools.py; `ScaffoldingDriver` gains M2 + one-motor-at-a-time; test_execution_tools.py | – |
| D | small edits, and the three run settings (A5) | send.py, operator.py, route_result.py, route_search.py, service.py, tamp_route.py, runs/runs.py, assembly_run/{panel,writes,plugin}.py + tests (test/shared/test_runs.py, test/plugins/assembly_run/test_assembly_run.py) | – |
| E | insert | insert.py; `ArmDriver` compliance mode (TCP follows the last target with a lag; under JTC TCP = FK(joints)); test_execution_insert.py | A–D |
| F | retreat | retreat.py; test_execution_retreat.py | A–D |
| G | handoff | handoff.py; test_execution_handoff.py | A, B |
| H | movement_exec integration | shared/movement_exec.py, sequences.py, plugin.py, panel.py, assembly_steps; test_movement_exec.py | E–G |
| I | slow test, README/roadmap, spec status | test_exec_compliant.py (slow) | all |

## 3. Verification
Commands: as M5 §8 (default, slow, linters, monitor smoke on domain 70 localhost only).
| Test | Asserts |
|---|---|
| controllers | switch ok; already active → no request; service down / error / 5 s timeout / switch in flight → fail; `leave_compliance` still requests JTC when cancelled twice; a late compliance ack is switched back; 10 s → the loud error |
| stream | `line_target` = §0.3 to 1e-12 at τ = 0, 0.37, 1, >1 (shortest slerp); a moving base shifts the arm-frame target by exactly B_t⁻¹·B_0; drift 2.1 mm stops, 1.9 mm not; a 0.6 s tick advances e by 0.1 s; a refused target → nothing more sent; the stop test before publishing. `split_rows`: 15 mm path, split 5 → T_r = 3.33 s, T_c = 1.67 s; split 0 → all rigid; split ≥ D − 0.5 → one mode; from the start; q_split on the path; < 2 rows |
| spiral | basis orthonormal ⟂ axis; vertical fallback; offset(0) = 0; offset(T) = 15 mm; pitch 3.75 mm at 4 turns, 2.5 mm at 6; every 20 Hz step < 1 mm at 20 s / 4 turns; `speed` = §0.4 to 1e-12; `spiral_problem`: 4 turns 7.5 s refused, 7.6 s accepted, 10 turns 18.8 s refused, 18.9 s accepted; a stall in A → "stalled" + text; f = 1 → one prompt; B's first target = A's last; Cancel → "cancelled"; B ceiling 33 s |
| settings | an older run.json without the three new keys loads rigid_only off, 20 s, 4 turns; spiral seconds 0.5 and 61, turns 0, 11 and 4.5, a bool for a number refused with the range; accepted changes saved and shown back by the panel |
| tools | a leftover STALLED doesn't count; one-sided waits for one tool; a stale status never counts; `stop_and_clear` refuses after 2 s |
| insert (fakes) | B21-like: STOP both, TIGHTEN left only; early stall → hold → switch; targets tick by tick = the formula (read from `_target_frame`); stall → STOP, hold, JTC; ceiling → NO_STALL; cancel during compliance → STOP, hold target = TCP, JTC, CancelledError; switch refused → no target; right arm fails after the left switched → left restored; FK-vs-TCP 11 mm → REFUSED, nothing sent; test mode → REFUSED; ground bar → no TIGHTEN, no switch, rigid_only ignored; rigid-only: TIGHTEN the mating tool, the whole path one joint motion, no switch and no Cartesian target ever, stall during the motion or the hold → STALLED, no stall in 30 s → NO_STALL (FAILED), split and spiral ignored; spiral settings 30 s / 6 turns reach `Spiral`; a too-fast spiral → refused at Plan, nothing sent; dry run publishes nothing |
| retreat | order (STOP, clear, LOOSEN M1 both, F/T zero both, switch, stream to FK(q_split), JTC, replan with goal_poses, checks, tail); early exit on arrival; replan None → FAILED, no tail; every `retreat_checks` threshold |
| handoff | anchor gap 11 mm → STIFF; drift 10.5 mm → FAILED; stalled → CLOSED; 30 s → FAILED; cancel → fingers stopped, hold, JTC; switch refused → no close goal; anchor target sent before the close goal |
| movement_exec | purged process: B20 insert ground, no tools; B21 (left_ur_arm,); B22 both; B20 retreat + its ungrasp both done; B21_H_close a HANDOFF; LATER only TRANSFER_HELD; every prompt starts with the robot; record survives `json.dumps` |
| slow test_exec_compliant | B21 insert planned for real, run on a fake Cindy that follows Cartesian targets, tools stalling 2 s into the compliant part → DONE; B20 insert → rigid (ground bar), no TIGHTEN; B20 retreat → tamp tail replanned with goal_poses → DONE |

Lab acceptance (monitor + visual steps only; robots on Zenoh, e-stops in hand; a Dry run of each part first):
1. Handoff in the air, Belle then Alice (MAN §6:643-674), on B35_H / B21_H close: Exec, prompt names the robot, Confirm;
   log order: JTC, F/T zeroed, anchor gap, compliance, close, "gripper done", JTC; the arm doesn't move. Repeat with
   Cancel during the close → fingers stop, controller chip shows JTC.
2. Cindy, no bar, Ignore built bars on, B21_J: transfer (workaround), Confirm the mount pause empty, grasp. Insert: Plan,
   Exec, read the prompt (rigid 10 mm, compliant 5 mm, left screw only), Confirm. Once "compliant", stall the left joint
   screw by hand (gloved, as on the bench) → "STALLED … STOP … joint control". Second run to the ceiling → NO_STALL after
   ~31 s, JTC back. Third run with "Rigid insert" ticked: the prompt says RIGID-ONLY; the controller chip stays on JTC
   throughout; stall the screw by hand during the hold → STALLED, STOP.
3. B21_R in the air, split 30 mm, traj time 30 s: Exec → M1 opens on both tools, F/T lines, compliant ~18 s; Cancel
   mid-way → hold, STOP, JTC. Plan again, Exec → the check lines, the tail runs.
4. Spiral in the air, on a two-mate bar (B22_J, both screws; on B21 the Spiral box is greyed, M6b): tick Spiral, run
   the insert (tools circle ≤ 15 mm); Cancel mid-spiral → safe ending; again to the end without a stall → the warning
   prompt; Confirm → 2 s return, then the ceiling. Then "Spiral s" 30, "Spiral turns" 6: the prompt reads 30 s,
   6 turns, pitch 2.5 mm, and the circles are visibly tighter. "Spiral s" 3: Plan refuses it ("spiral too fast … need
   ≥ 11.4 s"); set it back to 20.
5. Real run, MAN §7.2–7.4: B20 insert rigid (no screw, JTC stays) → fix-foundation prompt → Mark; B20_R retreat; B21
   insert (left screw only); Alice's §8 handoff on B21 (watch Cindy's tools); §9.2 B21_R with Alice holding.
6. Send back: the terminal log, `runs/<name>/movements/*.json`, per step pass/fail, stall times, peak |F|, end errors,
   retreat check lines, anchor gaps.

## 4. Safety, specific to compliance
| Hazard | Guard |
|---|---|
| force | M6 sends no target wrench but zero. Stiffness 500 N/m, 20 N·m/rad (crl-husky ur_controllers.yaml:256) → force ≈ K × target offset; offset bounded by 5 cm (25 N, arm.py:633) and per step by 1 cm (5 N); a 5 mm split lead ≈ 2.5 N; the 15 mm spiral ≈ 7.5 N; peak |F| recorded. As before, no force stop; the UR's protective stop applies |
| Zenoh/wifi drop while streaming | TCP > 0.5 s old → target refused → stream stops → hold (refused on stale data, "HOLD NOT SENT … pendant") → STOP published → restore retried 10 s → loud error. On the robot, the compliance controller keeps its last target (crl-husky sets no timeout for it; only free drive has `inactive_timeout`, :137); the target is ≤ one tick ahead; the screw motor stops by itself on a stall (end_effectors.py:293); nothing resumes when the link returns; e-stop is the backstop |
| stall never comes / stale STALLED | the gate latch; status freshness; ceilings T_c + 31 s (line), 33 s (spiral B); the spiral's hold waits on the operator; STOP in every finally |
| controller switch fails partway | per-arm check; any failure → hold all arms, request JTC on every arm that switched, STOP tools, FAILED; a switch in flight blocks new ones (the restore waits and asks again); joint trajectories are refused under compliance and Cartesian targets under JTC (arm.py:512) |
| Cancel / Stop all during compliance | task cancelled → no further target → finally: STOP, hold (target = TCP), request JTC synchronously, then a shielded wait; Stop all also stops the arms via the safety sync (robot.py:198); at shutdown the request goes out before the 2 s grace ends |
| operator presses Hold | `arm.hold()` + `lease.stop()`; with a prompt open it answers Cancel and the loop ends before its next publish; else it cancels the task |
| mocap jump | the 2 mm / 0.5° drift stop; the 1 cm step check behind it |
| late tick | the stream clock advances ≤ 0.1 s per tick |
| wrong frames (the 2026-09-29 e-stop) | FK-vs-TCP precheck; start = measured TCP; `urdf_problem` and test mode refuse the part |
| a leftover target wrench from robot_control | a zero wrench after every switch to compliance |
| a fast spiral set on the sliders | refused at Plan over 50 mm/s at full radius (§0.4); the prompt shows seconds, turns and pitch; the radius stays 15 mm (≈ 7.5 N) |
| rigid-only: the screw pulls against arms under joint control | the old baseline (W:5385-5388); the stall gate and the 30 s ceiling; STOP in the finally; the peak force recorded; the UR's protective stop |
Risks: (1) mocap in the loop (A1): a marker swap < 2 mm still moves the target that far; (2) a restore that never lands
(controller_manager down, a second monitor): arms stay compliant at their last target — the manual must say how to switch
back; (3) bar-weight sag under compliance (F/T not zeroed during the insert, W:5538): ~10–20 mm at 500 N/m — why the rigid
split exists; (4) M5 under safety review: `send_motion`, `PromptBox`, `world_problem` signatures may change; (5) pinch
hazard in lab step 2.

## Open questions
None: Q1–Q5 were decided on 2026-10-23 (Decisions, at the top).

## Status (2026-10-26)
- Built: `shared/execution/controllers.py`, `stream.py`, `spiral.py`, `insert.py`, `retreat.py`, `handoff.py`, the
  `tools.py` additions, §1.2's edits and settings, §1.3's movement_exec parts; the tests of §3, the slow end-to-end
  one in `test/plugins/movement_exec/test_exec_compliant.py`. All on fakes so far: the lab acceptance (§3, steps 1–6)
  still has to show the real screw's stall ending an insert, the switches and the restore on the real controller
  manager, the spiral's prompt and force, the retreat's checks and tail on a bar, the handoff's anchor gap and Cancel.
- Deviations from the text: `HOLD_FOR_STALL` lives in `stream.py`; `StallGate(ctx, tools, motor)`; the run functions
  take `watch` (the motion watch) and `recorder` (the panel's live line); the retreat's `replan` may answer None
  (60 s) → FAILED; the stream also stops when mocap loses the base; the fake arm follows targets with its joints (IK).
