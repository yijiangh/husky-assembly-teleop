# 2026-10-01 — Dry-run findings: fixes for the teleop monitor (implementation plan)

Source: the user's worklog (2026-10-01 section of `Husky Assembly Worklog.md`, text + 3 screenshots)
written while running Part A of `doc/support_robot_test_manual.md` (fake hardware, loopback DDS),
plus the session log `~/husky_dryrun/monitor_pane.log` and GPU renders made with
`~/husky_dryrun/probe/gui_visibility_probe.py`.
Background: `tasks/2026-09-30_support_robot_schedule_monitor.md` (spec), `tasks/2026-10-01_session_note.md`.
Rhino-side problems from the same notes are described (not planned) in
`<Dropbox>/bar_joint_rhino_design_workflow/docs/support_export_issues_from_monitor.md` (D5–D7).

## 0. The notes, sorted by where the fix belongs

| # | Note (worklog) | Belongs to | Item |
|---|---|---|---|
| 1 | "M3 cdfm start plan and M0 plan can be combined into one button" | teleop | F1 |
| 2 | "excessive tool acknowledgement … combined together in the schedule" | teleop | F2 |
| 3 | "support robot's FM M0 should adopt the M3-start → M0-target structure" (`B3_H_M2_LM_to_grasp`) | teleop | F3 |
| 4 | "ACM needed": `joint_G1-T20Ground-1_ground` vs `obstacle_ground` at `B1_J_M5_LM_insert` | teleop now (the floor body is created by the monitor) + Rhino note D6 | F4 |
| 5 | "existing built bars (B1 + joints) are not displayed properly" (B1_R screenshot) | teleop (two monitor causes) | F5 |
| 6 | "no built bars are properly rendered" at `B4_J_M0_free_to_load` | teleop (same cause as 5a) + Rhino note D5 (why the flag was on) | F5 |
| 7 | "there should not be an untighten step, only an ungrasp step" | Rhino note D7; teleop prepares for the re-export | F7 |
| 8 | "M0 for vertical bar hard to find a plan" (`B3_J_M0_free_to_load`) | teleop / planner — debug together | F8 |
| 9 | support robot's joint state while on Cindy's ROS domain; propagate states between actions | teleop | F6 |

Notes 5, 6 and 8 were listed under "rhino side" in the worklog; the evidence below puts them in the
monitor / planner.

## 1. What was verified (evidence)

- **Built bars hidden = the accuracy-test flag.** With `BAR_ACTION_MOCAP_ACCURACY_TEST = 1`
  `_hide_built_assembly_for_mocap` (`husky_monitor.py`) sets `is_hidden` on every static
  `bar_*`/`joint_*` body and `_sync_pp_visibility_to_hidden` blanks them. Renders: flag 1 → at
  `B4_J_M0` no B1/B3; in `B1_R` only the bare bar B1 (its joints blanked). Flag 0 → B1 + joints at
  `B1_R_M1..M3`, B1 and vertical B3 at `B4_J_M0`. The flag was on because the test manual asks for it
  (Alice's exported hold scene collides otherwise — Rhino note D5).
- **Held bar blanked in tool / manual steps.** At `B1_R_M0` (bar still clamped by the tools,
  `attached_to_link`) nothing is drawn with EITHER flag value: `_authored_motion_type` returns
  `'free'` for every movement without a Cindy role, and `_refresh_preview_attached_bodies` blanks all
  attached bar/joint bodies for `'free'` ("[preview] free: bar/joints hidden (not mounted)").
  Same for `J_M1`, `J_M2`, `J_M4`.
- **Ground joint vs floor.** `B1__J` states: `joint_G1-T20Ground-{0,1}_ground` ride with the bar
  (`attached_to_link`) with `touch_bodies = ['AT3L'|'AT3R', 'bar_B1']`. The floor `obstacle_ground` is
  not in the export: `cfab_session` builds it from `WalkableGround.json` and
  `inject_ground_rigid_body_state` allows only the wheels (+ obstacle robots). At the assembled pose
  the ground joint stands on the floor → `CC.4 ... - COLLISION`. compas_fab's CC.4 check is symmetric
  (`pybullet_check_collision.py:234-237`): listing either body in the other's `touch_bodies` allows it.
- **J_M0 has no goal until J_M3's start is chosen** (`M0 has no target_configuration; plan M1 first`):
  today two separate actions (movement 3 → `M1: Confirm manual start pose`, movement 0 → `Plan Movement`).
- **Single-arm free move goes to the EXPORTED configuration.** `_plan_by_kind` (SINGLE_FREE) plans to
  `mv.target_configuration`, solved in Rhino for the authored base; with a live (mocap) base the
  flange ends elsewhere and `H_M2` starts from the wrong place.
- **State after a restart (fake hardware).** `_seed_viz_huskies_from_progress` poses only the OTHER
  huskies from `progress.json`. The connected robot's simulated arms restart at home, although its own
  belief (e.g. Cindy holding B3 after entry 2) is on disk.
- **Planning** (log): `B3_J_M0` `plan_free_dual_arm failed: birrt_failed` twice; `B4_J_M3` BiRRT 5/5
  "no path found" (2 min each); M1 manual start prints "the branch nearest the goal collided; another
  IK branch was taken", start↔goal joint delta 4.6–5.5 rad. `ssik` is not installed in the venv, so
  the gradient IK fallback is in use.

## 2. Fixes

Order: F5 → F4 → F6 → F3 → F1 → F2 → F7 (F8 in parallel, with the user). Each item lists files,
functions and its check. Rules as in the spec §0 (locate by name, google docstrings + type hints,
plain comments with Better-Comments markers, legacy problems unchanged, tests + the two smoke scripts
stay green).

### F5 — show the built structure; make "ignore its collisions" a separate, visible switch

a. **Split the flag.** New class flag `IGNORE_BUILT_ASSEMBLY_COLLISIONS` (comment: planner/IK ignore the
   built bars; they are drawn faint). Default `0`. Legacy keeps its behaviour:
   `_ignore_built_assembly()` returns the new flag in schedule mode, and
   `BAR_ACTION_MOCAP_ACCURACY_TEST` when no schedule is active (accuracy test unchanged).
   Replace the `BAR_ACTION_MOCAP_ACCURACY_TEST` test at the two call sites of
   `_hide_built_assembly_for_mocap` (in `_finish_action_load` / `load_selected_movement`) with it.
   The accuracy-test UI block stays keyed on `BAR_ACTION_MOCAP_ACCURACY_TEST`.
b. **Panel toggle** (schedule section, DPG `Toggle`): "Ignore built-bar collisions (bars drawn
   faint)". Changing it clears `_mocap_hide_applied`, restores the flags on the loaded movements
   (reload the entry through `load_schedule_entry(self._loaded_entry.index)`), and logs one line.
   Needed today for Alice's hold entries (Rhino note D5); the operator sees that it is on.
c. **Faint, not invisible.** `_hide_built_assembly_for_mocap` records the bodies IT hid in
   `self._collision_ignored_bodies` (set of names). `_sync_pp_visibility_to_hidden` draws those with
   `BUILT_IGNORED_RGBA = (0.6, 0.6, 0.6, 0.25)` and keeps `TRANSPARENT` for bodies the export itself
   hides (not built yet). compas_fab does not re-pose `is_hidden` bodies, so place each ignored body
   from its state frame: `pp.set_pose(body, pose_from_frame(rb.frame))` for static ones.
   Rename the log line: `[built bars] collisions with N built bodies ignored (drawn faint)`.
d. **Held bar in tool / manual steps.** `_authored_motion_type(mv)`: for a movement without a Cindy
   role return `'bar_held'` when any built-assembly body in `mv.start_state.rigid_body_states` is
   attached (`attached_to_link` or `attached_to_tool`), else `'free'`. Role movements unchanged.
e. **Stationary steps need a configuration to be drawn at.** `J_M1`/`J_M2` export
   `robot_configuration = None` (seeded with home today), so the held bar would be drawn at the home
   pose. In `_accept_trajectory` / the M1 adopt path (`adopt_m1_derived_start`,
   `confirm_m1_manual_start`), also write the adopted start into every stationary movement between
   M0 and M1 (walk `STATIONARY_MOVEMENT_TYPES` neighbours), the same way the single-arm branch already
   carries its end through gripper steps.
- Check: extend `gui_visibility_probe.py` → flag 0: B1 + joints at `B1_R_M0..M3`; B1, B3 at `B4_J_M0`;
  toggle on: same bodies faint; `J_M2` after M1 confirm shows the bar in the tools at the bar-loading pose.
  Headless smoke: add "R_M0 preview motion type is bar_held".

### F4 — ground joints may touch the floor

`cfab_session.inject_ground_rigid_body_state`: add to the ground's `touch_bodies` every rigid body of
the state whose name is a ground joint (`joint_*_ground` / `env_joint_*_ground`; helper
`bar_action_io.is_ground_joint_body(name)`), both when creating the entry and when merging into an
existing one. A ground joint stands on the floor by construction, attached or built.
- Check: unit test on `B1__J.json` movement 5 (state only, no cell) for the allowance; smoke: with
  the built-bar switch OFF, `B1_J_M5_LM_insert` start state has no `obstacle_ground` pair in
  `planner.check_collision(..., full_report)`.
- Rhino note D6 describes the proper fix on the export side (the floor as a cell rigid body).

### F6 — state propagation between actions (answer to the worklog question)

Today: only the ROS-connected robot has live joints; multi-robot ROS is deferred (Zenoh). The
mechanism asked for exists as the **belief** in `progress.json`: `Mark entry done` stores the acting
robot's end state (`live` for the connected robot, `assumed` = exported end state for another
robot's entry), and every load poses the other robots' `ObstacleRobot<Name>` and drawn huskies from
it; a fresh mocap base overrides the stored base. Gaps to close:

a. **The connected robot's own state at start-up.** In `husky_world.init`, after
   `create_registry_huskies`: seed `huskies[0].interface.arm_joint_pose` (and base when mocap is off)
   from its own belief — generalise `_seed_viz_huskies_from_progress(monitor, connected,
   include_connected=True)`. ROS overwrites it with the first `joint_states` message; in fake hardware
   it is the state the simulation continues from (e.g. Cindy still holding B3 after Alice's run).
   Log: `[Schedule] Cindy starts from her progress.json state (entry 2, live)`.
b. **Entry start = previous end.** At `load_schedule_entry` (executable entry): when the connected
   robot has NO live joint source (fake hardware) and its belief's `after_entry` is the entry's
   predecessor in that robot's own sequence, write the belief into movement 0's start configuration
   (already what `_movement_starts_live` + the seeded interface give — verify with the smoke test).
   With ROS the live joints win, as now.
c. **Readout.** Schedule header gets a second line: `others: Alice <- live (entry 3) | Belle <- parked`
   from `progress_io.obstacle_sources`, so the source of each robot's pose is visible without the log.
d. Double drawing: a robot now appears as the solid viz husky AND the red obstacle tool at the same
   pose. Keep both (the red one is what the planner checks); note it in the manual.
- Check: headless smoke: mark entry 2 done → new monitor object for Cindy → her simulated arms equal
  the stored belief; header line names the sources.

### F3 — support robot: resolve the approach IK at the live base and back-fill (M1-start → M0-goal structure)

New `HuskyMonitor._resolve_single_arm_goal_live(mv) -> bool` (used by SINGLE_FREE planning):
1. target frame = `mv.target_ee_frames[side]` (H_M0 authors the approach pose = H_M2's start);
2. state = copy of `mv.start_state` with the live base (`_apply_live_base_to_movement`) and obstacle
   beliefs; `world.solve_goal_ik_generic(planner, state, {group: frame}, seed_confs=[exported
   target_configuration, live arm conf])`, collision-checked;
3. on success write the solution to `mv.target_configuration` and to the next ARM movement's
   `start_state.robot_configuration` (through the stationary gripper step — same carry rule as
   `_accept_single_arm_trajectory`), log `max |live IK - exported|`;
4. on failure: warn with the colliding pair / "no IK" and do not plan. When the base is not tracked
   (same base as authored) the exported configuration is kept and no IK runs.
Call it at the top of the SINGLE_FREE branch of `_plan_by_kind`. `H_M2` / `HR_M1` already plan in
Cartesian space from their start, so they follow automatically.
- Check: smoke script with the stub base shifted 3 cm: H_M0's end flange pose equals the authored
  approach frame (< 1 mm), H_M2 start conf equals H_M0's end, H_M2 plans ≈ 22 points.

### F1 — one button: confirm the transfer start and plan the travel-out

`confirm_m1_manual_start` returns `bool` (True when a start was adopted). New
`confirm_m1_start_and_plan_m0()`:
1. if the loaded movement is not the M1-role movement, load it (slider values are kept);
2. `confirm_m1_manual_start()`; stop on False;
3. select the M0-role movement, `load_selected_movement()`, `plan_selected_movement()`;
4. leave M0 loaded with its trajectory preview; log what to do next (`Exec`).
Button `M1: Confirm start + Plan M0` next to the existing confirm button (kept, for adjusting the pose
without replanning). Works straight after `Load entry` (uses the current slider values).
- Check: smoke: after `Load entry 0` one call leaves movement 0 loaded with a goal configuration;
  with a failing IK nothing is planned and the M1 movement stays loaded.

### F2 — steps instead of movements: no stop for tool actions that run with the next motion

a. `bar_action_io.tool_runs_with_next_motion(mv) -> bool`: scaffolding step with `overlaps_next`, or
   `tool_action in ('ungrasp', 'untighten')` — the one rule already used by
   `world.run_scaffolding_tool_step` (mark-only) and `schedule_ui.step_button_label`; both call it.
b. `bar_action_io.operator_steps(movements) -> list[OperatorStep]`,
   `OperatorStep(primary: int, absorbed: tuple[int, ...])`: an absorbed tool step is merged into the
   next arm movement. `B1__J` → `[0] [1] [2] [3] [4+5]`; `B1__R` → `[0+1+2] [3]`; `B3__H`/`HR` unchanged.
c. Monitor, schedule mode only (legacy slider 0..9 of the accuracy manual unchanged):
   `self._loaded_steps`; the Movement slider is labelled `Step (idx)` and ranges over steps;
   `_selected_movement_index()` maps step → primary movement index (single place; used by
   `load_selected_movement`, the readouts, `_now_line`); the `-> movement` readout shows
   `B1_J_M5_LM_insert  (+ J_M4 tool_tighten runs with it)`; the `Now:` line counts steps.
d. Exec on a merged step logs which tool action the compliant flow issues; nothing else changes
   (the M2/M3 flows already send tighten / loosen).
- Check: unit tests for `operator_steps` on the four fixture actions; smoke: entry 0 has 5 steps,
  entry 1 has 2; no "Mark tool step done" button appears any more.

### F7 — prepare for the release action without "untighten" (Rhino note D7)

After the re-export a release file is `R_M0 ungrasp, R_M1 LM_retreat, R_M2 free_home`.
- `bar_action_io.movement_role`: the id table `{(R,2): M3, (R,3): M4}` then mislabels ids. Roles for
  split actions already come from class + action (`cycle_roles`); make `movement_role`'s split-id
  branch a legacy helper only: for `_J_`/`_R_` ids return None unless the id matches the known slug
  (`_LM_retreat` → M3, `_free_home` → M4, `_free_to_load` → M0, `_CDFM_` → M1, `_LM_insert` → M2), and
  drop the mismatch print for split ids.
- Tests that pin the old layout (`test_schedule_ui` step-kind sequence of `B1__R`, `operator_steps`
  expectations) read the layout from the fixture instead of hard-coding four movements.
- No other code depends on the untighten step (it is mark-only since 2026-09-30).

### F8 — planning failures (debug together; not to be patched blind)

Facts in §1. First experiments, in this order:
1. Install the analytical IK the planners were tuned with: `pip install "ssik>=4.1,<5"` in the venv
   (declared in `external/husky_assembly_tamp/setup.py` extras; watch the numpy pin), unset
   `HUSKY_IK_BACKEND`, repeat `B3_J` (M1 confirm → M0 plan) and `B4_J_M3`. Compare the M1 start branch
   ("branch nearest the goal collided") and the start↔goal joint delta.
2. `B3_J_M0`: run `scripts/derive_m1_headless.py --bar B3` equivalent for the split files, look at the
   M1 dashboard (`scripts/m1_dashboard_server.py`) for where candidates die; try the `vertical` anchor
   with the perpendicular shifts the user found for B4 (+0.235 m gave a 65-waypoint M0).
3. M0 budget: `plan_free_dual_arm(max_time=120, max_iterations=50)` returned `birrt_failed` after
   ≈ 60 s; check whether start (live arms at home) or goal sits in a narrow passage with
   `diagnosis=True`.
Outcome of each goes into `doc/m1_planner_changelog.md`.

### Small items

- `traj time` shows 90 s before any movement is loaded (slider maximum as initial value): initialise
  it to the first movement's default at `Load entry`.
- `Move Arms to Movement Start (offline target)` publishes even with `FAKE_HARDWARE = 1`: add the
  same simulated branch `execute_arm_trajectory_all` has (move the simulated arms).
- Update `doc/support_robot_test_manual.md` + `doc/support_robot_schedule_manual.md`: steps instead
  of movements, the combined M1 button, the built-bar toggle (replaces "set
  `BAR_ACTION_MOCAP_ACCURACY_TEST`"), state line in the header.

## 3. Verification (whole change)

`pytest` (registry, bar_action_io, schedule_io, progress_io, schedule_ui) · `scripts/smoke_single_arm_plan.py`
· `scripts/headless_schedule_smoke.py` (extended as listed per item) · legacy harness
(`scripts/headless_live_monitor_test.py`, B6 M2 and B3__J M3) identical to the baseline logs ·
GPU renders with `~/husky_dryrun/probe/gui_visibility_probe.py` in the shared tmux session · then the
user's Part A walk-through.
