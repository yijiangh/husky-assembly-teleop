# 2026-10-01 — Drop the legacy M0–M4 roles + dry-run fixes (spec and change log)

Approved plan (read its "Decisions", "Mapping" and "Sequence" sections first):
`/home/yijiangh/.claude/plans/i-realize-that-you-iterative-token.md`.
Detailed design of the dry-run items F1–F8: `tasks/2026-10-01_dryrun_fixes_plan.md`. Where the two
disagree, the approved plan wins.
Background: `tasks/2026-09-30_support_robot_schedule_monitor.md` (§0 rules), `tasks/2026-10-01_session_note.md`.

## 0. Rules for every implementer

- Repo root: `/home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop`, branch `yh/multi-robot-monitor`.
  Do NOT commit; the main session commits after review.
- ! `husky_monitor.py` carries an uncommitted dry-run flag flip (`USE_MOCAP = 0`, `FAKE_HARDWARE = 1`).
  Leave those two lines exactly as they are.
- Edit only the files your step lists. Locate code by symbol name (grep), never by line number alone.
  Do not reformat unrelated code. Do not touch `external/`.
- Style (CLAUDE.md): simplest change, reuse existing helpers; google-style docstrings with type hints on
  every new/changed function; imports at the top, `from x import fn`; plain-language comments (no jargon
  such as "no-op"); Better Comments markers `# *` (important), `# !` (warning), `# ?` (question).
- No movement index literal (`'J_M3'`, `'R_M2'`, ...) in package code. Tests and docs may use real ids of
  the fixture.
- Data: tests may open only `ActionSchedule.json` and `BarActions/*.json` of the fixture, never
  `RobotCell*.json` in pytest. Smoke scripts work on a scratch copy (existing `make_scratch_problem`).
- Environment for running:
  ```bash
  cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate && source install/setup.bash
  export DESIGN_DATA_DIRECTORY="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly/data_design_study"
  export DESIGN_PROBLEM_NAME=260920_RobArch_demo_revamp_backup
  export HUSKY_IK_BACKEND=gradient     # ssik is not installed in the venv
  python -m pytest src/husky-assembly-teleop/test -q -p no:cacheprovider \
      --ignore=src/husky-assembly-teleop/test/test_flake8.py \
      --ignore=src/husky-assembly-teleop/test/test_pep257.py \
      --ignore=src/husky-assembly-teleop/test/test_copyright.py
  python src/husky-assembly-teleop/scripts/headless_schedule_smoke.py     # ~1-2 min, ~1 GB RAM per cell
  python src/husky-assembly-teleop/scripts/smoke_single_arm_plan.py
  ```
  Run long scripts with output redirected to a log file under `~/husky_dryrun/roles_refactor/` and read
  the summary lines; do not paste whole logs.
- Baseline before any change (commit 3a1abd8): pytest 99 passed / 1 skipped; `smoke_single_arm_plan.py`
  7/7; `headless_schedule_smoke.py` 34/34 (log `~/husky_dryrun/roles_refactor/baseline_schedule_smoke.log`).
- Report at the end: files changed, what each change does, the commands you ran and their summary lines,
  anything you could not do or decided differently from the spec (with the reason).

## 1. Step 0b — dispatch check (gate for every later step)

File: `scripts/headless_schedule_smoke.py` only.

New run function `dispatch_check(results, problem, root, schedule)` (its own Cindy monitor via the
existing `make_monitor`, like `cindy_run`), called FIRST in `main` (before `cindy_run`, `alice_run`).
For entry 0 (`B1_J`) and entry 1 (`B1_R`) of the fixture (use `schedule.entry(i)`; assert their kinds are
J and R), load the entry (`monitor.load_schedule_entry`) and, for EVERY movement index of the loaded action,
`select_movement(monitor, idx)` and record:

| Field | How |
|---|---|
| `planner` | Before `plan_selected_movement()`, replace on the monitor INSTANCE `_plan_M0_dispatch`, `_plan_M1_dispatch`, `_plan_M2_dispatch`, `_plan_M3_dispatch`, `_plan_M4_dispatch` with recorders that note a label (`free_to_load`, `transfer`, `insert`, `retreat`, `free_home`) and return None (no real planning). A step where none is called records `none`. Keep the label↔method table in ONE dict at the top of the function (stage 4 renames the methods; only that dict changes). Reload the entry after the planner sweep (a failed plan may clear state). |
| `exec` | For arm movements only: give the monitor a short fake planned trajectory so `exec_selected_movement_traj` passes its preconditions (e.g. two waypoints at the movement's start configuration; reuse whatever helper the monitor uses to stamp `planned_arm_trajectory`), replace `world.execute_planned_trajectory_compliant`, `world.execute_trajectory_and_zero_ft`, `world.execute_arm_trajectory_both`, `world.execute_arm_trajectory_all` (monkeypatch on the `husky_assembly_teleop.husky_world` module, restored in a `finally`) with recorders (generator ones return an empty generator), call `exec_selected_movement_traj()`, record `compliant` / `zero_ft` / `arm_both` / `arm_all`. Stationary steps record `-`. |
| `tool_cmd` | Only for movements whose `exec` is `compliant`: run the REAL `world.execute_planned_trajectory_compliant(monitor)` generator with the stub interface until the first `send_scaffolding_cmd` calls are seen (give `StubInterface` a `send_scaffolding_cmd(direction, motor, index)` recorder if it has none) or 50 steps, then `close()` it. Record e.g. `tighten` (direction +1 on the joint motor, both arms) / `loosen_gripper` (direction -1 on the gripper motor, both arms). Use the motor constants the world module uses. If the generator cannot reach the commands headless, report why instead of faking it. |
| `traj_time` | `monitor.trajectory_time` right after `select_movement` (load sets it from the default). Record `None` when the movement has no default (stationary): compare with the value before the load. |
| `starts_live` | `idx in monitor._live_start_indices` after the entry load. |
| `preview` | `monitor._authored_motion_type(mv)`. |

Expected table (today's behaviour = the plan's mapping table), as ONE dict in the script keyed by
`(entry_kind, movement_index)`; each row checked with `results.check('dispatch: <movement_id> ...', ...)`
showing the recorded values:

| entry, idx | movement | planner | exec | tool_cmd | traj_time | starts_live | preview |
|---|---|---|---|---|---|---|---|
| J 0 | J_M0_free_to_load | free_to_load | zero_ft | - | 30 | True | free |
| J 1 | J_M1_manual_mount_bar | none | - | - | None | False | free |
| J 2 | J_M2_tool_grasp_bar | none | - | - | None | False | free |
| J 3 | J_M3_CDFM_transfer_to_approach | transfer | arm_both | - | 10 | False | bar_held |
| J 4 | J_M4_tool_tighten_joint | none | - | - | None | False | free |
| J 5 | J_M5_LM_insert | insert | compliant | tighten | 5 | False | bar_held |
| R 0 | R_M0_tool_untighten_joint | none | - | - | None | False | free |
| R 1 | R_M1_tool_ungrasp_bar | none | - | - | None | False | free |
| R 2 | R_M2_LM_retreat | retreat | compliant | loosen_gripper | 5 | False | free |
| R 3 | R_M3_free_home | free_home | arm_both | - | 10 | False | free |

If the CURRENT code records something different from this table, do not change the table to match and do
not change package code: report the difference (it is information for the main session).
Add a comment above the table: stage 1 (F5d) intentionally changes `preview` of the stationary steps whose
start state has the bar attached (J 1, J 2, J 4, R 0) to `bar_held`.
Update the module docstring's flow description (one paragraph for the dispatch check).

## 2. Phase A — dry-run blockers (design: `tasks/2026-10-01_dryrun_fixes_plan.md` §2)

### A-F4 ground joints may touch the floor
Files: `husky_assembly_teleop/cfab_session.py`, `husky_assembly_teleop/bar_action_io.py` (helper
`is_ground_joint_body(name) -> bool`: `joint_*_ground` or `env_joint_*_ground`), tests in
`test/test_bar_action_io.py` (helper) and a new state-only test for the allowance (no cell): build the
ground entry the way `inject_ground_rigid_body_state` does on the fixture's `B1__J.json` movement 5 start
state and assert both ground joints are in the ground's `touch_bodies`. If `inject_ground_rigid_body_state`
needs the cell, factor the touch-list computation into a small pure function and test that.

### A-F5 built structure visible; collision ignore is a separate switch (F5a–c)
Files: `husky_monitor.py`, `scripts/headless_schedule_smoke.py`.
As in the dry-run plan F5 a–c, plus the F4 smoke check (with the switch OFF, `B1_J_M5_LM_insert` start
state has no `obstacle_ground` pair in the full collision report). The switch defaults to 0; the smoke's
Alice / Cindy hold-scene runs need it ON — set it explicitly where the smoke relied on
`BAR_ACTION_MOCAP_ACCURACY_TEST` hiding the built assembly, and say so in a comment.
F5d and F5e are NOT part of this step (they land in stage 1 and stage 2).

### A-F6 state propagation between actions
Files: `husky_world.py` (`_seed_viz_huskies_from_progress` / `init`), `husky_monitor.py`
(`load_schedule_entry`, schedule header), `progress_io.py` (only if `obstacle_sources` needs a small
addition), `scripts/headless_schedule_smoke.py`, manual note (F6 d) goes with the docs step at the end.
As in the dry-run plan F6 a–c.

### A-small
- `traj time` initial value: set to the first movement's default at `Load entry`.
- `Move Arms to Movement Start (offline target)`: with `FAKE_HARDWARE = 1` move the simulated arms
  (same simulated branch `execute_arm_trajectory_all` has) instead of publishing.

## 3. Change log
(appended by the main session after each reviewed step)
