# M7: the MoCap servo and the bar fit (spec, AWAITING USER APPROVAL of §0 and the open questions §9)

> Paths are as of `yh/core-port` 2912470. If Jakob's newer `jg/viser-cleanup` commits are merged first
> (`shared/planning` → `shared/planner`, one `*_contract.py` per plugin, tests in folders that mirror the package),
> the implementer maps every path here onto that layout; the math and the behaviour do not change.

One line: port the old servo loops (tool0 dual-arm, tool0 single-arm, the grounded-bar MoCap servo of 0e45758) and the
user's bar fit (`bar_mocap_fit.py`) onto the new monitor, as one new step part (SERVO) run by `movement_exec` under the
M5 lease and guards, with records in `runs/<run>/servo/`. The math is ported unchanged; §0 states it with line numbers.

Sources at 0e45758 (read with `git show`): W = husky_world.py, M = husky_monitor.py, BMF = bar_mocap_fit.py,
TBF = test/test_bar_mocap_fit.py, SMK = scripts/headless_bar_servo_smoke.py, MB = doc/mocap_bar_pose_test_manual.md,
MAN = doc/three_robot_assembly_test_manual.md. New paths are under `ros2_ws_next/src/husky-assembly-teleop/
husky_assembly_teleop/` at 2912470. tamp = `external/husky_assembly_tamp` at 2306271.

## Assumptions (defaults; the user decides in §9)
- A1 The servo is the last part of its step (kind SERVO), run right after the step's other parts in the same Exec.
  A servo that did not finish stays pending: Exec runs it again (old button 3b). No extra button.
- A2 Done = converged, already on target, or nothing left to plan. Every other end is not done (8 passes without
  converging, fit lost, load guard, IK or plan failed, cancel, interrupt): the step stays; Exec retries, Next ▶ skips.
- A3 The bar is measured at the old origin, the bar START (§0.8 choice 1): the stop test is the code's, unchanged.
- A4 After a grounded insert the servo runs whether "servo by bar" is on or off (tool0 when off), as the code does.
- A5 Markers come from the bar's own Motive asset (Bxx streams as 10xx). Jakob's marker cloud is optional (§3, O1).
- A6 A one-arm correction move is swept-checked for collisions (a guard added; the old code checked only its end).
- A7 TRANSFER_HELD (M5: "LATER: M7") becomes a motion part with a servo after it; not lab-tested while the
  workaround is in use.
- A8 A servo move is timed by its travel (3°/s, at least 3 s), not by the step's traj time slider.

## 0. Math (ported unchanged; the CODE is the reference)

### 0.1 Frames, and where every measured pose comes from
| Symbol | Meaning | Old source | New source | Age allowed |
|---|---|---|---|---|
| B | base_footprint in the world | `(hi.position, hi.rotation)` from Motive (W:1543-1547, M:3474-3476) | `ctx.scene.snapshot.robots[robots/<serial>].base`: the relay's calibrated rigid body, capture-time stamped (`base_time` = `last_fix_time`, world/scene.py:69) | ≤ 1.0 s (`is_live`, runs.py:58), read 2 s after the arms arrived |
| q_s | arm s joints, s ∈ {left, right} (Cindy) or {arm} | `hi.arm_joint_pose` | snapshot `robot.joints` | ≤ 0.5 s (runs.py:57) |
| A_s | base_footprint → `<arm>_base_link` (URDF, fixed) | PyBullet body | `link_pose(urdf, B, {}, f"{arm}_base_link")` = B·A_s (shared/planning/ik.py:132) | – |
| F_s | flange (tool0) in the world: F_s = B·FK_s(q_s) | ghost at live base + joints, W:1522-1562 | `link_pose(urdf, B, q, f"{arm}_tool0")` (route_search.py:133) | as B, q |
| F*_s | authored target flange | previous movement's `target_ee_frames` (`_start_ee_source`, M:9878-9894) | `movement.target.robots[robot].link_poses[flange]` of the TARGET movement (§1) | – |
| L, S | bar length; S = Trans(0, 0, −L/2) | old bar frame at the bar START (BMF:404-414) | process bar frame at the MIDDLE; L = the bar's `CylinderShape.height` (B20: 1.8 m) | – |
| h, G | the bar's first holder flange and its grasp (bar in flange frame) | `attached_to_link`, `attachment_frame` (W:1816-1819, M:2155-2214) | `movement.start.attached[bar][0]`: `Holder.to`, `Holder.grasp` | – |
| B* | design bar: B* = F*_h·G | `get_movement_start_bar_pose` (W:2226-2230) | same formula; checked: B20 insert start = assembly pose + 15 mm z, insert end = assembly pose (0.0 mm) | – |
| B_fk | FK bar: B_fk = F_h·G | W:1818-1819 | same | as F |
| B̂ | MoCap bar: Kabsch of the layout points P_k (bar frame) onto the sensed markers m_k, min Σ‖R·P_k + t − m_k‖², R proper (never a mirror) | `match_and_fit` / `_fit_near_hint` (BMF:362-387, 679-833), around the hint B_fk (W:1820) | `world/bar_fit.py`, same code; hint B_fk | markers ≤ 0.5 s old (M:160) |
| m_k | the bar's markers | Motive asset 10xx labeled markers, else unlabeled (M:10339-10369) | relay `MocapRigidBodyPose.marker_*` of rigid body 1000+xx (mocap_relay/node.py:455-510), only `measured`, residual ≤ 3 mm (BMF:74, 889-899) | capture time ≤ 0.5 s |
| f_s | F/T force vector in its sensor frame, NOT zeroed (tool + bar weight) | `hi.arm_ft_sensor[i][:3]` (W:1714-1735) | `robot.arms[arm].state.wrench[:3]` (arm.py:898-901) | latest |

### 0.2 Errors (every reading, after a move: arrival + 2 s settle)
- tool0, per flange: e_s = ‖p(F_s) − p(F*_s)‖ in mm, always against the AUTHORED F*_s, also in bar mode (W:1554-1556;
  the target is captured once, W:2395-2396). Rotation, recorded only: θ_s,i = arccos(R(F_s)[:,i] · R(F*_s)[:,i])
  for each world axis i (W:1497-1519).
- base drift from the servo start B_0: Δb = p(B) − p(B_0), per-axis angles as above (W:1609-1631). Recorded only.
- bar: D(X, Y) = pose_deviation(X·S, Y·S): T_rel = (Y·S)⁻¹(X·S); dpos = ‖p(X·S) − p(Y·S)‖ mm; dang = |rotvec(R(T_rel))|
  in degrees (BMF:457-476). Three of them per reading: MoCap-vs-design D(B̂, B*), FK-vs-design D(B_fk, B*),
  MoCap-vs-FK D(B̂, B_fk) (W:1826-1856).
- force change: c_s = ‖f_s − f_s⁰‖ N, f⁰ = the first reading with a force (W:2297-2302).
- correction (before planning, at the live base B): k_s = ‖p(B·FK_s(q_goal)) − p(B·FK_s(q))‖ mm (W:1868-1898).
- tracking (log + record): ‖p(B·FK_s(q)) − p(B·FK_s(q_planned end))‖ mm (W:1565-1606, W:2592-2597).

### 0.3 Targets and IK
- tool0 mode: F*_s authored.
- bar mode, engaged at the first usable reading: T_corr = B*·B̂⁻¹ (BMF:479-492); next targets F*_s := T_corr·F_s with F_s
  of that reading, both flanges (W:2341-2348). Until the first usable reading the targets stay tool0 (W:2349-2357).
  T_corr does not depend on where the bar frame's origin is: (B*O)(B̂O)⁻¹ = B*B̂⁻¹.
- Usable reading (W:1833-1848): fit ok (≥ max(4, ⌈0.75·M⌉) of M markers matched, BMF:650-659, and no second assignment
  as good, BMF:759-775) ∧ source not "fake" ∧ rms < 3 mm (W:1651) ∧ MoCap-vs-design dpos ≤ 30 mm and dang ≤ 3°
  (W:1648-1649).
- IK: T_s = (B·A_s)⁻¹·F*_s (ik.py:132; old W:3581). ssik lists ≤ 8 branches per arm (W:3436; ik.py:41), each turned to
  the 2π copy nearest the reference within the joint limits (`rebranch_toward_seed`); combinations sorted by
  max_s ‖q_s − q_ref,s‖_∞ (W:3589-3594; ik.py:177-188); the first collision-free one in the planning scene wins
  (W:3595-3611; ik.py:189-196). ! The servo's reference is the LIVE joints (W:3553-3555, M:9925-9927), not the
  authored target joints that Check goal uses (tamp_route.py:401). Then unwrapped to ±π of the live joints within the
  URDF limits (M:5534-5544; joints.py:31). PyBullet IK (1 mm / 0.01 rad) only where no ssik solver fits.

### 0.4 Stop tests, in the code's order
Two arms (W:2373-2639), passes k = 1 … 8 (W:2373; max_iters 8 at W:5925, W:6018):
1. Pass 1 only, after reading 0: bar mode ∧ usable ∧ dpos < 0.5 mm ∧ dang < 0.1° → "already on its target" (W:2404-2411).
2. Every pass, after IK, before planning: max_s k_s < 0.2 mm → "nothing left to plan" (W:2428-2441).
3. IK fails or no plan → stop (W:2388-2390, W:2462-2470). Cancel at a prompt → stop (W:2496-2497, W:2515-2516).
4. After the move and the reading, in this order (W:2603-2636):
   a. load guard (after a grounded insert only, W:2211-2214, W:6024): max_s c_s > 20 N → stop;
   b. bar mode engaged: reading not usable → "fit lost", stop; dpos < 0.5 mm ∧ dang < 0.1° → converged;
   c. otherwise: max_s e_s < 0.2 mm → converged.
5. All 8 passes ran → "ran all 8 passes without converging" (W:2637-2639).
One arm (W:6128-6193): reading 0; each pass: e < 0.2 mm → converged; IK fails → stop; k < 0.2 mm → nothing left to
plan; move; reading. At most 8 passes. No bar, no load guard.

### 0.5 The move between passes, and whether the constraint holds
- Two arms, bar held (after the transfer's mount, or after the insert): tamp `plan_constrained_dual_arm_linear(
  goal_ee_frames=F*)` (M:5553-5560; tamp api.py:1610-1690). The bar pose is interpolated in steps of 5 mm / 0.05 rad
  from the live bar (left flange FK ∘ the grasp derived from the live state) to F*_left ∘ that grasp; each step's IK
  starts from the last. So the left-to-right flange transform stays the one actually held at every waypoint: the bar
  is not strained. ! F*_right is not used by this planner (only by the IK and the correction check): in tool0 mode a
  real grasp that differs from the authored one leaves a right-arm residual the loop cannot remove.
- If no straight line is found and not linear-only: tamp `plan_constrained_dual_arm(goal_conf=q_goal)` (sampling, bar
  held, stage 3, 0.01 m / 0.025 rad, 120 s, one try) (M:5580-5597 = tamp_route.HELD). It explores: the 5° confirm
  catches a long path.
- Linear-only (no sampling) after a grounded insert: the bar stands on the floor (W:2208-2214, M:5475-5478, M:5564-5571).
- One arm (support robot, no bar): no planner; the two rows [q, q_goal] as one joint move (W:6154-6156). No
  end-effector constraint applies. ⚑ A6: the two-row path is swept-checked (`collision.swept_check`).
- Duration: T = clamp(x / (3°/s), 3 s, 90 s), x = the excursion = max over arms, joints and rows of |q_k − q_0|
  (W:1958-2007, W:1661-1685); `retime` may make it longer. Confirm on pass 1 always (W:2484-2492, W:6162-6168), and on
  a later pass when x > 5° (warning) (W:2499-2516, W:6169-6175). Settle 2 s after the arrival (W:105, W:2535-2545).

### 0.6 Carried into the insert, and the gate after it
- Bar mode engaged at the transfer's servo → the insert's plan goes to G_s = F*end_s · (F*start_s)⁻¹ · F_s, per flange,
  F_s live at planning (BMF:495-513; M:9359-9362, M:9382-9429; set W:2658-2662). F*start = the transfer's target link
  poses, F*end = the insert's. Without it the insert goes to F*end (authored).
- The servo after a grounded insert runs only if the insert part ended with max_s ‖p(B·FK_s(q)) − p(B·FK_s(q_end))‖
  ≤ 3 mm, q_end = the insert plan's last row (W:5994-6007, W:1659).

### 0.7 Thresholds, and old line → new constant (all in `shared/execution/servo.py` unless said)
| Value | Old | New |
|---|---|---|
| 8 passes | W:2087, 5925, 6018, 6062, 6248 | `MAX_PASSES` |
| tool0 0.2 mm (converged and nothing-left) | W:2087, 5925, 6018, 6062, 6248 | `TOOL0_TOL = 0.0002` |
| bar 0.5 mm / 0.1° | W:1644-1645 | `BAR_TOL = (0.0005, radians(0.1))` |
| gate 30 mm / 3° | W:1648-1649 | `BAR_GATE` |
| fit rms 3 mm | W:1651 | `BAR_MAX_RMS` |
| force change 20 N | W:1656 | `MAX_FORCE_CHANGE` |
| grounded insert end 3 mm | W:1659 | `GROUNDED_END_TOL` (M6 spec has the same, insert.py; keep one) |
| confirm over 5° | W:1640 | `CONFIRM_TRAVEL = radians(5)` |
| 3°/s, at least 3 s | W:1663-1664 | `SPEED`, `MIN_TIME`; cap `movement_exec.MAX_TRAJ_TIME` 90 s (M:777) |
| settle 2 s | W:105 | `SETTLE` |
| markers ≤ 0.5 s old | M:160 | `MARKER_MAX_AGE` |
| Bxx → 1000 + xx | W:81-95 | `world/marker_layouts.BAR_ASSET_BASE`, `bar_asset_id` |
| match 4 mm, track gate 20 mm, marker residual 3 mm, start triangle 15 mm high, sides 2 mm, ≤ 24 sensed, inliers max(4, ⌈0.75M⌉) | BMF:62-74, 650-659, 680-681 | `world/bar_fit.py`, same names |
| ssik ≤ 8 branches | W:3436 | `ik.MAX_BRANCHES` (exists) |
| linear step 5 mm / 0.05 rad | tamp api.py:1617-1618 defaults | tamp defaults (unchanged) |

### 0.7b Formula → old line → new function
| Formula | Old | New |
|---|---|---|
| F_s = B·FK_s(q) | W:1543-1552 | `servo.flange_poses` (core `link_pose`) |
| e_s, θ_s,i (per-axis) | W:1497-1519, W:1553-1561 | `servo.tool0_errors`, `servo.per_axis_angles_deg` |
| Δb | W:1609-1631 | `servo.base_drift` |
| B*, B_fk | M:2155-2214, W:1818-1819 | `servo.servo_plan` (design_bar), `servo.bar_reading` |
| B̂ (match, refine, hint) | BMF:679-833, 850-915 | `bar_fit.match_and_fit`, `bar_fit.fit_near_hint`, `marker_layouts.fit_bar` |
| D(X, Y) at B·S | BMF:457-476 | `bar_pose.pose_deviation(bar_start(X, L), bar_start(Y, L))` inside `servo.bar_reading` |
| usable gates | W:1833-1848 | `servo.bar_reading` |
| T_corr, F* := T_corr·F | BMF:479-492, W:2341-2348 | `bar_pose.bar_servo_correction`, `servo.bar_targets` |
| c_s | W:1714-1735, 2297-2302 | `servo.force_vectors` + `servo_part.measure` |
| T_s = (B·A_s)⁻¹F*, branches, L∞ sort, first clear | W:3540-3613 | `ik.solve_ik` (exists) via `servo_route.ServoRoute` |
| unwrap to ±π of live | M:5534-5544 | `joints.unwrap_to_nearest` (exists) |
| k_s | W:1868-1898 | `servo_route.ServoRoute` (`RouteSearch.flange`) |
| stop tests 0.4 | W:2404-2411, 2428-2441, 2603-2639, 6128-6193 | `servo.stop_after_move`, `servo_part.run_servo` |
| linear bar-held plan, sampling fallback, linear-only | M:5545-5597, W:2208-2214 | `servo_route.ServoRoute` (tamp calls as `_insert`, `_transfer_held`) |
| one-arm two-row move | W:6154-6156 | `servo_route.ServoRoute` (+ `swept_failure`) |
| excursion x, T(x) | W:1958-2007, 1667-1685 | `servo.excursion`, `servo.recommended_time` |
| confirm pass 1, > 5° | W:2484-2516, 6162-6175 | `servo_part.run_servo` → `sequences.confirmed` |
| settle 2 s | W:2535-2545 | `servo_part.run_servo` (`ctx.sleep(SETTLE)`) |
| tracking mm | W:1565-1606 | `servo_part.measure` |
| carried goal G_s | BMF:495-513, M:9382-9429 | `bar_pose.apply_authored_move`, `servo.carried_insert_goals` |
| insert-end gate | W:5994-6007 | `servo.insert_end_gap` |
| record + plot | W:2685-2816 | `servo_record.servo_payload`, `servo_record.plot_servo` |

### 0.8 Where the code differs from the plan's math block (the code wins)
- D1 θ_s: the plan writes θ_s = ∠(R(F_s)ᵀR(F*_s)); the code records three per-axis angles (0.2) and never stops on
  rotation. tool0 stops on position only.
- D2 Two position stops, not one: "nothing left to plan" (k_s, before planning) and "converged" (e_s, after the move).
  In bar mode the tool0 test is not used at all.
- D3 Bar stop point: the code measures at the bar frame's ORIGIN, which was the bar START. The process frame is at the
  MIDDLE, so the port evaluates at B·S (`T_old = T_new · Trans(0, 0, −L/2)`, verified: B20's markers sit 0.9 m lower
  in the process frame than in the old table, x and y identical). Choice 1 below.
- D4 Bar mode is not "always": only a grounded bar's held servo, after a first usable reading, with the gates of 0.3;
  a reading lost after that stops the loop.
- D5 The IK reference is the live joints (the plan's block does not say which reference).
- D6 The bar-held linear planner aims the bar from the LEFT target only (0.5).
- D7 The 20 N force guard exists only after a grounded insert; "> 5°" is the excursion over the whole path.
- D8 One arm: no planner, a two-row joint move.
- D9 After a grounded insert the servo runs with the setting off too (tool0) — the brief says "behind the setting".

Choice 1, where the bar error is measured (pros/cons; recommended: a):
- a) the bar START (old origin, B·S). + the exact old stop test and records (20261001 comparison, the A/B); − an end
  chosen by the old export, not by the design.
- b) the MIDDLE (process origin). + natural in the new data; − a 0.1° tilt then moves the ends up to L/2·0.1° = 1.6 mm
  on B20 while "converged"; changes the approved test.
- c) both ends, max < 0.5 mm. + bounds every point of the bar; − a new rule, slower to converge.

## 1. The dispatch rows M7 owns
| Row | M5 (done) | M6 (specced) | M7 adds | The operator sees |
|---|---|---|---|---|
| J_transfer, workaround (TRANSFER_FREE) | Check goal, Plan (bar hidden), Exec: motion → place bar by hand → grasp; logs "servo correction: M7, not run" (sequences.py:268-269) | – | SERVO part last: tool0, or the MoCap bar if the insert is grounded and "servo by bar" is on; target = the transfer's target link poses = the insert start; planned in the insert's scene (bar held); records; the carry | Exec → Confirm motion → Confirm the mount (by hand) → Confirm grasp → servo prompt for the FIRST move (and any later > 5°) → the servo line per pass → "converged …" |
| J_transfer, held (TRANSFER_HELD) | refused (LATER: M7) | – | a motion part (default 10 s, BAI:527 DUAL_CONSTRAINED_FREE) + the same SERVO part | Plan, Exec, Confirm, first servo move |
| J_insert, ground bar | refused (M6) | rigid INSERT part, no tighten | the insert's Plan with the carried goal (0.6) when the transfer's bar servo engaged; SERVO part after the insert: gate 3 mm, linear only, 20 N guard, bar or tool0 | Plan logs "MoCap-bar correction carried: …"; after the insert: the servo prompt, the force line per pass |
| H_approach (SUPPORT_APPROACH) | Check goal, Plan (one-arm search), Exec motion; logs "servo: M7" | – | SERVO part, one arm, tool0 | Exec → Confirm motion → servo prompt for the first move (tool0 X mm) → passes → "converged" |
The servo part needs nothing from M6 to run. The after-insert servo is reachable on a robot only once M6's INSERT part
exists; until then it runs in tests only.

## 2. Module layout (each file ≤ ~400 lines; `test/test_layers.py` holds: world < shared < plugins)
### 2.1 world/ (pure numpy; the user's fit, split in three)
- `world/bar_fit.py` (~360): port of BMF:44-86 (constants), 172-247 (`BarFit`), 516-833 (the matcher) unchanged:
  `BarFit` (+ `to_json`), `match_and_fit(sensed, design, *, tol_m, side_tol_m, min_inliers, min_triangle_height_m,
  max_sensed) -> BarFit`, `fit_near_hint(sensed, design, T_hint, *, gate_m, tol_m, min_inliers) -> BarFit | None`
  (was `_fit_near_hint`), private helpers as in BMF. Loader (BMF:285-358) dropped: layouts come from the process.
- `world/bar_pose.py` (~220): `kabsch`, `transform_points`, `PoseDeviation` (+ `to_json`), `pose_deviation`,
  `_rotation_vector`, `bar_servo_correction`, `apply_authored_move`, `bar_slide_roll` (BMF:250-283, 360-513), plus
  `bar_start(T: np.ndarray, length: float) -> np.ndarray` (= T·S) and `bar_ends(T, length) -> np.ndarray` (2×3, ±L/2;
  replaces `design_bar_axis_endpoints`).
- `world/marker_layouts.py` (~220):
  ```
  BAR_ASSET_BASE = 1000
  @dataclass(frozen=True) class BarLayout: bar: str; names: tuple[str, ...]; points: np.ndarray; length: float; warnings: tuple[str, ...]
  def bar_layouts(process: Process) -> dict[str, BarLayout]            # marker_fit.bar_markers (reused, = Rhino's) + cylinder height; collinear → warning
  def bar_asset_id(bar: str) -> int | None                             # "bars/B20" -> 1020 (W:85-95)
  @dataclass(frozen=True) class Sensed: points (n,3); ids; residuals; age: float; source: str ("asset" | "cloud" | "fake"); dropped: int; occluded: int
  def sensed_from_raw(raw: RawMarkers, now: float) -> Sensed            # measured only, residual ≤ 3 mm
  def sensed_near(cloud, layout, prior, now, radius=0.08) -> Sensed     # O1 only: cloud markers within 80 mm of the layout at the prior (cf80552's gate)
  def fit_bar(sensed: Sensed, layout: BarLayout, *, max_age: float | None, hint: np.ndarray | None, gate_m=0.02, **kwargs) -> BarFit   # BMF:850-915
  def fake_sensed_markers(T, points, *, slide_m=0.03, roll_deg=2.0, noise_m=0.0005, seed=0) -> Sensed   # BMF:920-952, tests
  ```
  ! Motive's marker size is not relayed, so the 12–26 mm size filter (BMF:78, 896) is gone (Q6). `measured` False
  replaces the occluded flag (BMF:891-894).
- `world/marker_fit.py`: `bar_markers` and `bar_deviation` stay; `fit_markers`, `_fit`, `MarkerFit` are removed with
  their 2 tests once `mocap_bars` uses `fit_bar` (Q8). `world/__init__.py` file list updated.

### 2.2 shared/execution/
- `servo.py` (~380): the constants of 0.7, and
  ```
  TRANSFER, APPROACH, AFTER_INSERT = "transfer", "approach", "after_insert"
  @dataclass(frozen=True) class ServoPlan: kind; action; primary: int; target: str (movement id giving F*);
      step_index: int (loaded.steps index of the movement whose planning scene is used: the insert, or the approach);
      flanges: tuple[str, ...]; arms: tuple[str, ...]; targets: dict[str, Pose]; held: bool; bar: str | None;
      holder: tuple[str, Pose] | None; layout: BarLayout | None; design_bar: Pose | None; grounded: bool;
      by_bar: bool; why_tool0: str; linear_only: bool; max_force_change: float | None
  def servo_kind(loaded, action_id, primary, transfer_as_free) -> str | None        # §1 rows; None: no servo
  def servo_plan(loaded, action_id, primary, settings: RunSettings, layouts) -> ServoPlan | str   # str: why not
  def flange_poses(robot: RobotObject, arms) -> dict[str, Pose]                      # F_s
  def tool0_errors(flanges, targets) -> dict[str, dict]     # side -> pos_err_mm, pos_norm_mm, rot_err_deg, observed_pose (W:1522-1562)
  def per_axis_angles_deg(qa, qb) -> list[float]             # W:1497-1519
  def base_drift(base, start) -> dict                        # W:1609-1631
  def wrench_magnitudes(robot, arms) -> dict | None; def force_vectors(robot, arms) -> dict | None   # W:1688-1735
  def excursion(rows) -> tuple[int, float, float]            # waypoints, net, excursion degrees (W:1958-2007)
  def recommended_time(excursion_deg: float, max_time: float) -> float                           # W:1667-1685
  def bar_reading(plan, flanges, fit, source, now_design) -> dict   # T_design/T_fk/T_mocap/T_corr, 3 deviations at B·S, usable, reason (W:1779-1865)
  def bar_targets(T_corr, flanges) -> dict[str, Pose]        # W:2341-2348
  def stop_after_move(reading, plan, engaged) -> tuple[str, str] | None   # (status, line), order of 0.4 step 4
  def carried_insert_goals(loaded, action_id, insert: int, robot: RobotObject) -> dict[str, Pose]   # 0.6
  def insert_end_gap(robot, names, end_row, arms) -> float   # metres, 0.6
  ```
- `servo_record.py` (~200): `servo_payload(...) -> dict` (§2.9), `plot_servo(payload: Mapping, path: Path) -> None`:
  W:2725-2816 with matplotlib's `Figure` + `FigureCanvasAgg` (never pyplot), same four panels and colours (W:116-118).
  Runs on assembly_run's writer thread.

### 2.3 shared/planning/
- `service.py`: `PlanRequest` gains `goal_poses: Mapping[str, Pose] | None = None` (flange id → world tool0 goal; the
  insert's carried goal; ! the same field and meaning as M6 §1.2, whoever lands second merges) and
  `servo: ServoAsk | None = None`;
  `@dataclass(frozen=True) class ServoAsk: targets: Mapping[str, Pose]; stop_below: float; linear_only: bool;
  held: bool`. `run_servo_step(pool, request, abort, log) -> ServoStep` (as `run_plan`, service.py:112-131).
  `MotionPlannerPluginApi.request_servo_step(request) -> asyncio.Future[ServoStep]`.
- `servo_route.py` (~200, new): `@dataclass(frozen=True) class ServoStep: route: RouteResult; correction: dict[str,
  float]; nothing_left: bool` (`ok` = route ok or nothing_left). `plan_servo_step(mirror, loaded, step, snapshot,
  robots, ask, *, beliefs, ignore_built, abort, log) -> ServoStep` with `class ServoRoute(RouteSearch)`:
  1. planning = `scene_for(..., end_contacts=True)` (never `hide_held`: the bar is in the tools); `prepare` as
     tamp_route.py:196-203; start = live joints.
  2. `solve_ik(mirror, robot, {arm: ask.targets[flange]})` with the default reference (= live); none → Refused.
  3. unwrap to the start within the limits, `check_limits`; correction k_s from `self.flange(part, goal)` vs
     `self.flange(part)`; max < `stop_below` → `nothing_left`, no plan.
  4. held: `self.linear(plan_constrained_dual_arm_linear(... goal_ee_frames={side: frame(target)},
     skip_env_collisions=False ...))` as `_insert` (tamp_route.py:292-308); Refused and not `linear_only` → one try of
     `plan_constrained_dual_arm(goal_conf=goal, random_seed=0, **HELD)` (tamp_route.py:263-290 without its loop).
     One arm: rows [start, goal], `self.swept_failure(rows)` → Refused if it hits (A6).
  5. `self.solved(rows, goals)`.
- `tamp_route.py` `_insert` (:292-308): use `goal_poses` for the parts it names, else `part.target`, else the line
  (3 lines; the field is threaded through `plan_movement` → `_route` → `RouteSearch.goal_poses`).
- `movement.py`: nothing new; `classify(...).drives.grounded` is the grounded test (= old `held_ground_joints`,
  BAI:854).

### 2.4 shared/ APIs
- `shared/movement_exec.py`: `SERVO_PART = "servo"`; `SERVO_INDEX = -1` (! the servo is no movement of its own: its
  part's index, so `ExecResult.done` and the step panel's done set treat it apart from the primary); `step_parts`
  appends `PartPlan(SERVO_INDEX, primary id, routine, SERVO_PART, arms, "servo … onto …")` when `servo_kind` is not
  None; `DEFAULT_TRAJ_TIME[TRANSFER_HELD] = 10.0`; `LATER` loses TRANSFER_HELD. `@dataclass(frozen=True) class
  Carried: loaded; action: str; insert: int; serial: str; at: float`; `MovementExecPluginApi.carried -> Carried |
  None`.
- `shared/assembly_run.py` Protocol: `request_record(..., kind: str = "movements", figure: Callable[[Mapping[str,
  Any], Path], None] | None = None)`.

### 2.5 plugins/
- `movement_exec/servo_part.py` (~350, new): `async def run_servo(job: Job, run: PartRun) -> None` (§2.7) and
  `measure(job, plan, k, state) -> dict`. Tracks the bar asset as `servo_B20` (`ctx.track_object(f"servo_{bar number}",
  bar_asset_id(bar))`; no geometry, so never an obstacle; it shows on health), waits ≤ 1 s for a first sample,
  untracks in `finally`.
- `movement_exec/sequences.py`: `run_part` dispatches SERVO_PART; `live_command(job)` becomes a wrapper of
  `live_command_of(job, names, rows, traj_time)` (the same 2° rule, row 0 = live); the "servo: M7, not run" line goes;
  `world_problem(job, motion, basis=None)` takes the pass's basis (default: the request's) and also refuses a basis
  older than the robot's last motion (`Job.moved_at`, ⚑ new guard).
- `movement_exec/plugin.py`: `moved_at: dict[str, float]` set after every arrived motion and servo pass;
  `_plan_refusal` refuses "the arms moved (servo) since this plan was made: replan" when `basis.time < moved_at`;
  `carried` property; `_carried` cleared when a transfer motion part of that action starts, on another action, and
  on a process reload.
- `movement_exec/panel.py`: one servo line (desktop and phone): "servo k/8 · tool0 L 0.31 R 0.27 mm" or "servo k/8 ·
  bar B20 MoCap 0.62 mm / 0.04° · FK 0.10 mm" and, after an insert, "force change L 3.1 R 2.0 N".
- `motion_planner/plugin.py`: `request_servo_step` = `self._submit(run_servo_step, request, "servo step")`; `describe`
  gets a ServoStep branch.
- `assembly_steps/plugin.py` (+~12 lines): in `make_search` (:296), for an INSERT primary whose `executor.carried`
  is this (process, action, primary): `goal_poses = carried_insert_goals(...)`, logged as M:9427-9428 and recorded
  (`"goal_carried": true` in the plan event). `assembly_steps/panel.py` `exec_line` (:117-134): "servos {arms}".
- `assembly_run/plugin.py` (+~10 lines): `_record_file` takes `kind` and `figure`; after the JSON, `figure(payload,
  path.with_suffix(".png"))`, a plot failure logged and the JSON kept.
- `mocap_bars.py`: `fit_bar(sensed_from_raw(...), layout, max_age=0.5, hint=last ok fit (≤ 1 s) else the planned
  pose)` instead of `fit_markers`, at most every 0.1 s (old "follow live (10 Hz)"; the full search is ~20 ms).

### 2.6 robot_interface (runtime edit E6)
`robot_interface/mocap_state.py:20` `RawMarkers` + `residuals: np.ndarray | None = None`, `capture_time: float |
None = None`; `robot_interface/mocap.py:52` fills them from `message.marker_residuals` and the header stamp (the
relay already sends both; nothing in crl-husky changes).

### 2.7 One servo part, pass by pass (main thread, inside movement_exec's task, the robot leased)
```
plan = servo_plan(...)            str → REFUSED; log "[servo] feedback: …" (W:2246-2252); bar mode off with a reason → warn once (W:2243-2245)
gate (AFTER_INSERT with the insert route in the request): insert_end_gap > 3 mm → REFUSED (W:6004-6007)
no force reading after an insert → warn "load guard OFF" (W:6013-6015)
reading 0 = measure()             robot not live → REFUSED; bar engaged / already on target (0.4 step 1)
for k in 1..8:
    others_moving / process / current action → REFUSED;  basis = basis_of(snapshot, loaded, <the primary's Step>, …)
    step = await planner.request_servo_step(PlanRequest(view.process, loaded.steps[plan.step_index], snapshot, …,
                                            beliefs, ignore_built, servo=ServoAsk(targets, TOOL0_TOL, linear_only, held)))
           (timeout HELD max_time + 60 + WAIT_MARGIN)
    nothing_left → DONE "nothing left to plan";  not ok → FAILED "<IK | plan>: reason"
    x = excursion(rows); T = recommended_time(x); command = live_command_of(…, T)
    k == 1 or x > 5°: confirmed(job, run, text, warn=x > 5°)   Cancel → CANCELLED "stopped at pass k"
    re-check: world_problem(job, motion=True, basis=basis) (stale_reason, is_live) → REFUSED;  command rebuilt from live joints
    outcome = send_motion(…, dry_run, watch=others_moving, progress)   DRY_RUN → stop DRY_RUN; not ARRIVED → its status
    moved_at[serial] = now; await ctx.sleep(SETTLE); reading k = measure()
    stop_after_move(...) → DONE / FAILED;  bar mode: update targets (engage, or "fit lost")
else: FAILED "ran all 8 passes without converging"
finally: untrack; "### bar pose at termination" (W:2651-2657); carry if engaged and kind TRANSFER (W:2658-2662);
         record if len(data) > 1 or any bar reading (W:2664-2668)
```
Prompt texts, each "<robot> (<serial>): <primary> — ": pass 1: "SERVO 1/8 onto the <insert start | approach pose |
insert end> (<tool0 | the MoCap bar B20>): <planner>, <n> rows, travels <x>° (ends <net>° from where it starts),
<T> s; flanges move L <a> mm, R <b> mm. Confirm to move; Cancel stops and records." Later: "SAFEGUARD servo <k>/8:
this path travels <x>° … Confirm to move, or Cancel to stop and record." (warning).

### 2.8 Endings, and how each ends cleanly
| How | What happens |
|---|---|
| Cancel at a prompt | nothing sent this pass; status cancelled; record written; arms where the last pass left them |
| Cancel / Hold / Stop all during a move | `send_motion` holds every arm, says whether each hold went out, re-raises; the part records progress and the servo file in `finally`, then re-raises; plugin `_cancelled` (plugin.py:247-267) records the part and a soft_stop event for Stop all |
| another robot moves (`watch`) | INTERRUPTED, every arm held, record |
| stale / not live after Confirm | REFUSED, nothing sent, record |
| HOLD_NOT_SENT | loud error, the result and the record say "stop it with the pendant or the e-stop" |
| dry run | pass 1 planned and built (`build_trajectory`), DRY_RUN, record with `dry_run: true`; never done |

### 2.9 The record `runs/<run>/servo/<action>/<primary>.<n>.json` (+ `.png`), via `request_record("servo", kind="servo", figure=plot_servo)`
The old keys (W:2719-2722): `planner_mode` ("transfer", "transfer+mocap_bar", "single_arm_joint"), `movement_id`
(the target movement), `servo_feedback` ("tool0" | "mocap_bar"), `bar_name`, `servoing_data`: per reading `iter`,
`tool0` {side: pos_err_mm, pos_norm_mm, rot_err_deg, observed_pose}, `base`, `wrench`, `force_change_N`, `track_mm`,
`bar` {bar, grounded, source, fit (`BarFit.to_json`), T_design, T_fk, T_mocap, T_corr (4×4, process frame),
fk_vs_design, mocap_vs_design, mocap_vs_fk (at the bar start), usable, reason}. New: `robot`, `serial`, `action`,
`kind`, `bar_frame` {"origin": "middle", "length_m", "measured_at": "start (T·Trans(0,0,−L/2))"}, `settings` (the
constants used), `moves` per pass {planner, attempts, rows, ik {backend, rank, combinations, max_change},
correction_mm, excursion_deg, net_deg, traj_time, duration, prompted, outcome}, `outcome` {status, reason,
passes}, `dry_run`. The part's own exec record (movements/) names the servo file. Sides: "left", "right", "arm"
(`joints.arm_side`).

## 3. Relay and markers
- Have now (888854a/2912470 + crl-husky a24f198): the relay attaches every rigid body's LABELED markers (member ids,
  positions, residuals, measured flag) to its `MocapRigidBodyPose` (mocap_relay/node.py:455-510; the message fields
  are in a24f198's `MocapRigidBodyPose.msg`). A bar set up as Motive asset 10xx therefore works: `ctx.track_object(…,
  1020)` gives `TrackedObject.markers`. Motive needs "Labeled Markers" on. Missing: residuals and capture time are
  dropped on the monitor side → E6 (§2.6). This is the old "asset" source, the one the old servo used first.
- Not on our branch: Jakob's unlabeled cloud. faeaaa1 (relay publishes `/mocap/markers`, monitor keeps
  `world.markers`) imports `crl_husky_msgs/MocapMarkers` at startup, so merging it alone stops the monitor from
  starting. The message is crl-husky f3335c1 on `origin/jg/mocap-marker-cloud`, a sibling of a24f198 (both on 1addd5e):
  not in our crl-husky worktree. `git log --all -- '*MocapMarkers*'` finds only f3335c1.
- Optional step O1 (not needed for acceptance): cherry-pick f3335c1 onto the ros2_ws_next crl-husky worktree
  (1 msg + 1 CMake line; rebuild `crl_husky_msgs`); cherry-pick faeaaa1 into yh/core-port, resolving it against
  3275f2b9 (RawMarkers moved to robot_interface/mocap_state.py: `MarkerCloud` goes there too) and monitor.py; then
  `sensed_near` feeds the cloud as the fallback source "cloud" when the asset's markers are missing or older than
  0.5 s (old order, M:10339-10369). Only the control PC running the relay and monitor needs the message.
- cf80552 (his fit near the plan): not taken. Worth reusing only its gate idea: keep cloud markers within 80 mm of the
  layout placed at the prior (B_fk) before the user's matcher, which refuses more than 24 markers (BMF:717-719). That
  is `sensed_near`, used only with O1. His Kabsch equals the user's; his worker-thread fit is not needed (8 asset
  markers: ~1 ms tracked, ~20 ms full).

## 4. Porting the bar-fit tests (`test/test_bar_fit.py`)
TBF has 31 test functions at 0e45758 (28 at fe37618, plus the 3 servo ones; 37 cases with the parametrized ones).
Marker names change `J20-21_mocap_M1` → `joints/J20-21_mocap/M1`; points move z → z − L/2; order is by half id then
label (J20-169, J20-21, J20-22, J20-25). `B20_MARKERS` is read from the purged process where the test needs the real
bar (skipped without the Drive), else kept as a constant in the new frame.
| Old tests | Port |
|---|---|
| exact_recovery, noise_one_mm, one_marker_dropped, ghost_marker, three_markers_dropped, mirror_is_refused, symmetric_layout_is_ambiguous, m1_m2_swap_resolved, two_bars_in_view, too_few_or_too_many_markers, kabsch_never_mirrors (11) | unchanged on `match_and_fit` / `kabsch`; `t20_design` and B20 shifted to the middle frame |
| collinear_design | fit part unchanged; the loader warning becomes `bar_layouts` warning on a process half layout |
| loader_round_trip, loader_refuses_schema_1, loader_missing_file, loader_refuses_broken_files[3] (4 functions) | replaced (no file any more): `bar_layouts(purged)` gives B20, B21, B34, B35 with 8 named markers and L = 1.8 for B20; a bar with < 3 markers is left out; a malformed `markers` entry is refused by the core reader (core validate) |
| demo_sidecar_matches_bar_actions | becomes: the process B20 layout equals the old `261016_2ModulesDemo_YH/MoCapMarkers.json` B20 table shifted by L/2 (< 0.05 mm), read with `json`; skipped without the Drive |
| old_rig_take_does_not_fit_a_t20_bar | unchanged, via a small cache → `Sensed` adapter in the test |
| fake_markers_round_trip, cache_occluded_markers_dropped | on `Sensed` / `RawMarkers` (`measured` False instead of flag 0x01) |
| cache_size_forms_and_filters | the residual and age parts kept; the size part dropped (no size relayed) |
| tracked_fit_* (5) | unchanged on `fit_bar(…, hint=…)` |
| pose_deviation, axis_endpoints | unchanged; endpoints now ±L/2 (`bar_ends`) |
| bar_servo_correction…, apply_authored_move, one_servo_step_closes_on_the_bar | unchanged |
New: `test_the_old_origin_is_the_bar_start` (deviation at B·S from process-frame poses = the old-frame deviation,
1e-9) and `test_sensed_from_raw_drops_unmeasured_and_noisy_markers`.

## 5. Tasks
| # | Title | Files | Depends |
|---|---|---|---|
| A | bar fit in world/ + its tests | world/{bar_fit,bar_pose,marker_layouts}.py, world/__init__.py; test/test_bar_fit.py | – |
| B | E6 residuals and capture time | robot_interface/{mocap_state,mocap}.py; test/test_mocap_check.py (its `store_sample` tests) | – |
| C | servo math and records | shared/execution/{servo,servo_record}.py; test/test_servo_math.py | A |
| D | servo step on the planner, carried insert goal | shared/planning/{service,servo_route,tamp_route,route_search}.py, plugins/motion_planner/plugin.py; test/test_servo_route.py (slow) | C (constants) |
| E | servo records through assembly_run | shared/assembly_run.py, plugins/assembly_run/plugin.py; a test in test_assembly_run.py | C |
| F | the SERVO part | shared/movement_exec.py, plugins/movement_exec/{servo_part,sequences,plugin,panel}.py; test/test_servo_part.py; FakeContext.track_object in test/plugin_fakes.py | B, C, D, E |
| G | carry in assembly_steps, exec_line | plugins/assembly_steps/{plugin,panel}.py | F |
| H | mocap_bars on the user's fit; fit_markers removed | plugins/mocap_bars.py, world/marker_fit.py, test/test_marker_fit.py | A, B |
| I | slow B20 chain, README + roadmap rows, spec status | test/test_servo_chain.py, README.md, doc/plugin_roadmap.md | all |
| O1 | (optional) marker cloud | crl-husky f3335c1; faeaaa1; marker_layouts.sensed_near | A |

## 6. Verification (headless; the new venv; never source ~/Code/ros2_ws/install)
```
cd ~/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop && source install/setup.bash
cd src/husky-assembly-teleop
export HUSKY_DRIVE_ROOT="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly"
python3 -m pytest test/test_bar_fit.py test/test_servo_math.py test/test_servo_part.py test/test_layers.py -q -p no:cacheprovider
python3 -m pytest test -q -p no:cacheprovider                      # all quick (M5 baseline 578 passed)
python3 -m pytest test -q -p no:cacheprovider -m slow              # incl. test_servo_route.py, test_servo_chain.py
python3 -m pytest test -q -p no:cacheprovider -m linter -k "flake8 or pep257"
env -u CYCLONEDDS_URI ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1 timeout 25 ros2 run husky_assembly_teleop husky_monitor \
  --ros-args -p robots:="['alice','belle','cindy']" -p plugins:="['assembly_steps','robot_control','mocap_bars']" -p viser_port:=8095
```
Tests that never write into the Drive: quick ones use `FakeRun`; the chain copies the purged process to `tmp_path`.
| Test | Asserts |
|---|---|
| test_bar_fit (quick) | §4 |
| test_servo_math (quick) | every formula of §0 to 1e-12 on hand-made poses: e_s, per-axis angles, k_s, excursion (an out-and-back path reads 0 net, x > 0), T (3 s floor, 90 s cap), T_corr·B̂ = B*, `bar_targets` keep the flange-to-flange transform, the deviation at B·S, each gate reason at its threshold (rms 2.9/3.1 mm, 29/31 mm, 2.9/3.1°, fake, ambiguous, 5 of 8), `stop_after_move` order (force 20.1 N wins over converged), carried goal = authored move from the live flange, `insert_end_gap` 2.9/3.1 mm |
| test_servo_part (quick; RobotLab, a stub planner solving with `ik.branches` (ssik, no PyBullet) and returning [live, goal]) | (1) Cindy at J_transfer's end, base measured 10 mm off: one pass, first-pass prompt, converged at pass 1 (< 0.2 mm), record `servo/B20_J/B20_J_transfer.1.json` with 2 readings; (2) bar servo on synthetic markers streamed from `E·B_fk` each tick: a flange-side E (3 mm / 0.1°) closes in 1 pass, a world-frame E in ≤ 3; every target = T_corr·F of the reading before; carry set; record `servo_feedback` mocap_bar; (3) markers older than 0.5 s, rms 3.1 mm, 31 mm off, a 5-of-8 fit → tool0, "not usable" said once; a fit lost after engaging stops; (4) Cancel at pass 1's prompt → nothing sent, CANCELLED, record only with a bar reading; (5) Cancel during pass 2's move → every arm held, CancelledError out, record with `outcome.status` cancelled and 2 readings, ExecResult done = (3, 1, 2) without SERVO_INDEX; Exec again runs only the servo part; (6) Belle starts moving during pass 1 → INTERRUPTED, held, record; Belle moved within 2 s before a pass → REFUSED; (7) base moved 3 mm after Confirm → REFUSED; (8) after-insert: gate 3.1 mm refused, 2.9 mm runs; force changing 25 N after pass 1 → stopped "force … changed by 25.0 N"; linear_only passed; (9) a later pass travelling 6° prompts with a warning, one of 4° does not; (10) dry run: one pass, DRY_RUN, never done; (11) Alice approach: one arm, two-row path, converged; (12) a plan made before a servo pass is refused at Exec |
| test_servo_route (slow, real motion planner on the purged process) | at B20_J_insert's start (design scene, base +10 mm): IK at the live base, a straight bar-held plan whose last row puts both flanges within 0.2 mm of the authored targets; with a 0.1 mm target: `nothing_left`; at the insert END (ground halves on the floor) a 1 mm correction plans straight (the old smoke's KEY question, SMK check 3); B21_H_approach one arm: two rows, swept-checked |
| test_servo_chain (slow, the M5-style headless chain) | scratch copy; real assembly_run, motion_planner, movement_exec, assembly_steps on one FakeContext, fake robots that follow and tools that stall; workaround on; B20_J transfer: Plan (tamp), Exec, Confirm motion, Confirm mount, grasp stalls, base shifted 10 mm, servo pass 1 Confirm → converged → `servo/B20_J/B20_J_transfer.1.json` + `.png`, events new_run, plan, prompt, confirm, exec, servo; servo by bar on with synthetic markers (world-frame E 3 mm / 0.1°): converged ≤ 3 passes, `executor.carried` names B20_J_insert, the insert's PlanRequest carries `goal_poses` = authored move from the live flanges (1e-9); the Drive folder untouched |
| test_layers, flake8, pep257 | clean |
Monitor smoke: as M5 (§8 of its spec): every plugin sets up, no "failed", clean teardown.

## 7. Lab acceptance (monitor and visual steps only; robots on Zenoh; e-stops in hand)
L0 Set up. Relay running (`ros2 launch husky_assembly_teleop mocap.launch.py`), Motive streaming B20 as 1020 with
"Labeled Markers" on and its 8 balls visible. `ros2 daemon stop`, then `ros2 run husky_assembly_teleop husky_monitor
--ros-args -p robots:="['alice','belle','cindy']" -p plugins:="['assembly_steps','robot_control','mocap_bars']"
-p process_directory:=<the B20–B37 process>` 2>&1 | tee ~/m7_lab.log. New run. In mocap_bars pick B20, Mocap ID
1020, Track: the blue bar sits on the real B20. Pendant speed slider at 100 %. Transfer workaround on.
L1 Cindy: workaround transfer, mount, servo (tool0). "Servo by bar" off. B20_J: Check goal, Plan, Play, Exec.
  - Prompts in order: the transfer motion; "B20_J_place_bar … by hand" (mount B20, Confirm); the grasp (screws
    stall); then "SERVO 1/8 onto the insert start (tool0) …, flanges move L a mm, R b mm": a few mm. Confirm.
  - The servo line counts passes; later passes run alone unless > 5° (a warning prompt). Log: "[servo k] tool0 pos
    L=… R=… mm" and "[servo k] bar bars/B20: FK-vs-design …, MoCap-vs-design …, MoCap-vs-FK … (fit 8/8, rms … mm,
    asset)" each pass.
  - It ends "converged at pass k (< 0.2 mm)" or "nothing left to plan"; then "### bar pose at termination …".
    The step panel moves to B20_J_insert.
  - Cancel test (once, on the first servo move): Confirm, then press Cancel while the arms move (≥ 3 s): both arms
    stop at once ("holding every arm"), the panel says cancelled. Exec again: only the servo runs; Confirm; it ends
    as above. Two servo records: `.1` cancelled, `.2` converged.
L2 Alice: approach, servo. B21_H_approach: Check goal, Plan, Play, Exec, Confirm. After arriving: "SERVO 1/8 onto the
  approach pose (tool0): … tool0 X mm, …". Confirm. Passes end "converged". Watch the jaws stop ~100 mm short of B21.
L3 B20 grounded-bar A/B (MB §10).
  - Run "bar": "Servo by bar" on; B20_J as L1. Log "[servo] feedback: the MoCap bar bars/B20 (converges at 0.5 mm /
    0.1°)". mocap_bars: the blue bar closes on where B20 should be. Ends "converged on the MoCap bar …" and "the
    MoCap-bar correction is carried into the plan of B20_J_insert". B20_J_insert: Plan → "MoCap-bar correction
    carried: … X mm / Y° from the authored end" (X ≈ the MoCap-vs-FK gap). [needs M6] Exec the insert (rigid) →
    "SERVO 1/8 onto the insert end (the MoCap bar) …" → Confirm; each pass shows "force change L … R … N"; it stops
    by itself past 20 N. Then fix the foundation by hand.
  - Run "tool0": un-mount B20, back to its loading spot, Prev ◀ to the transfer step (Reopen B20_J if it was marked
    done), "Servo by bar" off, run it again. No carry line. [needs M6] after the insert the servo runs on tool0.
  - Compare the "### bar pose at termination" lines: MoCap-vs-FK about the same in both runs; MoCap-vs-design
    < 0.5 mm / 0.1° in run "bar".
L4 Send back: `~/m7_lab.log`, `runs/<run>/servo/` and `movements/`, `events.jsonl`, per step pass/fail, the
  termination lines, which prompts appeared.

## 8. Safety: M5's §9 guards stay, and the servo adds
| Hazard | Guard |
|---|---|
| a wrong marker match moves the arms | gates 30 mm / 3°, rms 3 mm, ≥ 6 of 8, no ambiguity, never "fake"; pass-1 Confirm; > 5° Confirm; the linear-only rule after an insert |
| the servo moves from a stale state | every pass: live (base 1 s, joints 0.5 s), others not moving, plan basis re-checked after Confirm, row 0 = live (≤ 2°) |
| a plan from before a servo pass replayed | `moved_at` guard (§2.5); the step panel drops plans on a step change |
| another robot moves during a pass | `send_motion(watch=others_moving)` → INTERRUPTED, held |
| bar pushed into the floor | 20 N force-change stop, linear only, gate 3 mm |
| Cancel not reaching the arms | as M5 #5; the servo `finally` writes the record, untracks, never sends |
| one-arm two-row path through something | swept check (A6) |

## 9. Risks and open questions (recommended answers)
Risks, by severity:
1. Mocap in the loop: a marker swap within the gates still steers (≤ 30 mm, then the next pass sees it). Pass-1
   Confirm and the excursion confirm are the backstop.
2. tool0 mode with a mounted grasp ≠ the authored one: the right flange keeps a residual (0.5, D6) → 8 passes without
   converging, by design; the record shows it. With A2 that step is "not done": the operator presses Next ▶.
3. The after-insert plan starts on the floor: if the planning scene refuses the ground contact, it ends "no plan"
   (test_servo_route checks it before the lab).
4. Mocap noise (~0.1–0.3 mm) vs 0.2 mm: "nothing left to plan" stops small chases, as before.
5. Overlap with M6 (PlanRequest.goal_poses, part kinds and LATER, GROUNDED_END_TOL, M6's log "servo after the insert:
   M7, not run"): whoever lands second merges; no M7 code needs M6.
6. Files near the limit: assembly_steps/plugin.py 459 and assembly_run/plugin.py 719 lines (+~12 and +~10); the split
   stays in the post-M5 cleanup.
7. runs/ sits in the Drive-synced process folder (decision 8 still open): one PNG (~100 kB) per servo.
Open questions:
- Q1 After a grounded insert with "servo by bar" off: run the tool0 servo (code, MB §10.3) or skip (brief)? → run it.
- Q2 Where the bar error is measured (§0.8 choice 1)? → the bar start (a).
- Q3 A servo that does not converge: done or not done (A2)? → not done; Exec retries, Next ▶ skips.
- Q4 Confirm only on pass 1 and > 5° (M5 asks on every motion)? → yes, as the code and the plan.
- Q5 Swept check of the one-arm two-row move (A6)? → yes.
- Q6 The marker size filter is gone (not relayed): add size to the relay? → no; labeled asset markers suffice.
- Q7 Marker cloud (O1) now? → later, with the relay work of M10.
- Q8 mocap_bars on the user's fit and Jakob's `fit_markers` removed? → yes: one fit in the code.
- Q9 TRANSFER_HELD executable in M7 (A7)? → yes, untested in the lab until CDFM is ready.
- Q10 No preview ghost for servo moves (the prompt gives travel and flange mm), no slider override (A8)? → yes.
Nothing blocks M7. The insert half of L3 (carried insert executed, the servo after it) needs M6 implemented.

## 10. Commit plan (small, one change each; run-mode flags never committed)
1. `[feat] bar fit: the MoCap bar fit of the old monitor in world/, its layouts from the process markers, the bar frame at its middle` (A)
2. `[test] bar fit: the 31 old checks on process markers, and the old origin at the bar start` (A)
3. `[feat] mocap: a rigid body's marker residuals and capture time kept with its markers` (B)
4. `[feat] servo: errors, targets, stop tests and the record ported unchanged from the old monitor` (C)
5. `[feat] motion_planner: a servo step, IK at the live base and the correction straight with the bar held, or one arm's two-row move` (D)
6. `[feat] assembly_run: servo records and their plot under runs/<run>/servo/` (E)
7. `[feat] movement_exec: the servo after a transfer, an approach and a grounded insert, every pass under the lease and its checks` (F)
8. `[feat] assembly_steps: the insert planned from where the MoCap-bar servo left the flanges` (G)
9. `[feat] mocap_bars: fits bars with the old monitor's matcher, ten times a second at most` (H)
10. `[test] servo: a 10 mm base shift, the bar servo, cancel, the interlock, the load guard, and the B20 chain` (F, I)
11. `[doc] M7 spec status, README and roadmap rows` (I)

## Deviations from the request and the plan
1. The servo is a step part (SERVO, index −1), not a separate routine or button; Exec re-runs an unfinished one.
2. Three world/ modules (bar_fit, bar_pose, marker_layouts) instead of bar_fit + marker_layouts: BMF is 952 lines.
3. Bar error measured at the old origin (D3), shown as a choice.
4. Added guards: swept check of the one-arm move; the `moved_at` plan guard; the insert-end gate without M6's value.
5. Dropped: the marker size filter (not relayed), the MoCapMarkers.json loader and its tests, the live tracker
   window (the panel line and the PNG replace it), the slider override during the servo prompt.
6. TRANSFER_HELD made executable (M5 listed it for M7).
7. The test count is 31 at 0e45758 (the plan says 28: that is fe37618).
