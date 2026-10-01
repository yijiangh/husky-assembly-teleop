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

## 4. Stages 1 and 2 — detailed spec (planner, verified against the data)

### Spec: take the M0–M4 roles out of the code (Stage 1) and add one carry rule for configurations (Stage 2)

Stage 1 replaces every role lookup with a lookup by **movement kind** plus **`is_free_home`**. It moves the role code unchanged into a new legacy module and lands F5d. Stage 2 adds one forward-carry helper that `_accept_trajectory`, `_accept_single_arm_trajectory` and the transfer adopt path all use (the chain fix plus F5e).


---

#### 0. Assumptions and facts I checked

**Facts I checked against the data (read-only scripts in the scratchpad):**
- **Every Cindy action in 11 problems passes the proposed `check_action_kinds` rules.**
  - Problems: 260920_backup, 260929, 260921 (including its sidecars), 260814, 260920_revamp, 260715, 260716, 260807, 260811, 2026-05-14 and 260703.
  - Jointing actions always have 1 transfer and 1 insert. Release actions always have 1 retreat.
  - Every legacy `BarAssemblyAction` has 1 transfer, 1 insert, 1 retreat and exactly 2 `DUAL_FREE`.
- **Which start states have the bar attached (fixture B1/B3):**
  - J: indices 1, 2, 3, 4, 5. R: index 0 only. H and HR steps: none.
  - Legacy 260715 B1/B4: only M1 and M2 have it attached.
  - So F5d gives the old answer for every arm movement. It changes only the stationary steps J1, J2, J4 and R0. The support robots are not affected.
- **Exported controllers:**
  - insert = `cartesian_compliant`.
  - retreat = `joint_tracking`, so the routing warning will fire for it.
  - Legacy `B*_M2_LM_mate` = `joint_tracking`, so it warns as well.
- **Exported start configurations (B1, B3, B12 and 260929 B1):** tighten start == insert start == transfer target, and every R start == the insert target, all with 0.0 difference. So Stage 2's comparison against "the next arm movement" gives the same result as today's `index+1` comparison on unchanged files.
- **Readers of the preview fields:** `_traj_ghost_bodies` and `planned_trajectory_motion_type` are written but never read anywhere in the package. So `bar_held` on a stationary step only re-colours bodies (green instead of transparent). It does not touch planning or collision settings.
- **`build/husky_assembly_teleop/husky_assembly_teleop` is a symlink to `src`.** A new module is importable without rebuilding.

**Assumptions:**
1. Phase A (the `IGNORE_BUILT_ASSEMBLY_COLLISIONS` switch, faint drawing, connected-robot seeding, the "traj time initial value at Load entry" item) has landed before Stage 1.
   - Phase A may add new calls to `default_trajectory_time(mv, role, MOVEMENT_TRAJECTORY_TIME_S)` or `_match_movement_role` (for example the initial value at Load entry). They must be converted too; the grep gate in §1.6 catches them.
2. **Legacy single-file rule (decided):** `check_action_kinds` *does* check the legacy `BarAssemblyAction`: exactly 1 transfer, 1 insert, 1 retreat and 2 `DUAL_FREE`. It is verified on about 100 files, and it makes `is_free_home` ("the last `DUAL_FREE`") unambiguous.
3. **Log text scope for Stage 1:**
   - Must change: every `role` variable or parameter, and every user-facing hint that tells the operator to navigate by role or index. These are wrong today for split exports (for example "Movement slider 1" should be 3).
   - Stays until Stage 4: fixed debug prefixes inside the planner functions and the M1 tools (`[M1 manual]`, `[M1 derive]`, `[M2 inter-EE invariance]`, `"M2: missing start conf."` and similar), plus comments and docstrings outside the edited gates.
4. Rules from `tasks/2026-10-01_drop_legacy_movement_roles.md` §0 apply:
   - Leave `USE_MOCAP = 0` / `FAKE_HARDWARE = 1` exactly as they are. Do not commit. Do not touch `external/` or docs.
   - Locate code by symbol name.
   - Google-style docstrings with type hints, `from x import y`, plain-language comments, Better Comments markers.

---

### STAGE 1: roles out (one declared behaviour change: F5d)

#### 1.1 Context
The monitor still turns every movement into an M0..M4 role through `_match_movement_role`, and the planner, exec, UI and chain rules all branch on that role. Stage 1 keys every one of those branches on `movement_kind(mv)` and `is_free_home`. Behaviour stays identical, with one exception: tool and manual steps whose start state holds the bar now preview as `bar_held` (F5d). The role code survives only in `legacy_bar_action_io.py`, for the offline accuracy analysis of old takes.

#### 1.2 Files and changes

##### A. `husky_assembly_teleop/bar_action_io.py`

1. **Move to the new legacy module, unchanged:** `_LEGACY_ROLE_RE`, `_SPLIT_ROLE_RE`, `_HOLD_ROLE_RE`, `_SPLIT_ROLES`, `movement_role`, `_ROLE_BY_CLASS`, `cycle_roles`, `roles_for_action`. Also drop `import re` if `_natural_key` no longer needs it (it does need it, so keep).
2. **Delete:** `movement_controller` and `_CINDY_CONTROLLER_BY_ROLE`. After that, `CONTROLLER_CARTESIAN_COMPLIANT` and `CONTROLLER_JOINT_TRACKING` are unused in this module, so remove them from the import (flake8).
3. **Keep:** `CINDY_ACTION_TYPES` stays here, with its comment changed to "Cindy's actions". The legacy module imports it from here.
4. **Add in the "movement kinds" section, after `kind_fits_robot`:**
   - `COMPLIANT_KINDS = frozenset({DUAL_CONSTRAINED_LINEAR, DUAL_INDEPENDENT_LINEAR})`
     - Comment: these are the insert (tightens the joint screws) and the retreat (loosens the grippers), both run under the Cartesian compliance controller.
   - `FREE_HOME_TRAJECTORY_TIME_S = 10.0`
     - Comment: the free move home is shorter than the travel out; the kind table says 30 s.
   - `is_free_home(action: BarSceneAction, mv: Movement) -> bool`
     - Docstring: "Whether a `DUAL_FREE` movement is the free move HOME (not the travel out to the loading pose)."
     - False when the class kind is not `DUAL_FREE`. Use `_KIND_BY_CLASS.get(type(mv))` so an unknown class does not raise.
     - True when `isinstance(action, BarAssemblyReleaseAction)`.
     - For `isinstance(action, _bar_action_module.BarAssemblyAction)` (the legacy single file): True when `mv` is the **last** `DUAL_FREE` movement of `action.movements`, compared by identity. This is what `cycle_roles` did.
     - False for everything else: Jointing, Hold and HoldRelease actions.
   - `check_action_kinds(action: BarSceneAction, source: Optional[str] = None) -> None`
     - Use a module table `_REQUIRED_KIND_COUNTS`:
       - Jointing: `{DUAL_CONSTRAINED_FREE: 1, DUAL_CONSTRAINED_LINEAR: 1}`
       - Release: `{DUAL_INDEPENDENT_LINEAR: 1}`
       - Legacy `_bar_action_module.BarAssemblyAction`: `{DCF: 1, DCL: 1, DIL: 1, DUAL_FREE: 2}`
     - Pick the rule with `isinstance`. Other action types are not checked; return.
     - Count with `movement_kind` (an unknown class raises its `TypeError`).
     - On any count mismatch, raise `ValueError` naming `source or action.action_id`, the action type, and each wrong count (kind value, meaning word, expected, found).
     - Docstring: the monitor finds transfer, insert and retreat by kind, so two of them, or none, would silently pick the wrong one.
5. **Change `default_trajectory_time`:**
   - Old: `default_trajectory_time(mv, role=None, role_table=None)`.
   - New: `default_trajectory_time(mv: Movement, free_home: bool = False) -> Optional[float]`.
   - Returns `FREE_HOME_TRAJECTORY_TIME_S` when `free_home` and the kind is `DUAL_FREE`, otherwise `TRAJECTORY_TIME_BY_KIND_S.get(movement_kind(mv))`. The flag is ignored for other kinds.
   - Update the comment above `TRAJECTORY_TIME_BY_KIND_S`: it no longer refers to the monitor's role table. Move in the "why these durations" explanation from the comment block that `husky_monitor.py` deletes (see B1).
6. **Module docstring:** replace the `movement_role` / `cycle_roles` / `roles_for_action` / `movement_controller` bullets with `COMPLIANT_KINDS`, `is_free_home` and `check_action_kinds`. The "To classify…" paragraph points to kinds plus `is_free_home`, and says the old roles live in `legacy_bar_action_io` for old takes only.
7. **Circular imports:** `bar_action_io` must never import `legacy_bar_action_io`.

##### B. NEW `husky_assembly_teleop/legacy_bar_action_io.py`

- **Module docstring:** "The old M0..M4 roles, kept only so offline tools can read takes stamped with a role (e.g. 'M3') and for their tests. The monitor must not import this module."
- **Imports:**
  - `re`, `Optional`
  - From `rs_data_structure.bar_action`: `BarSceneAction`, `BarAssemblyJointingAction`, `BarAssemblyReleaseAction`, `EndEffectorConstrainedDualArmFreeMovement`, `EndEffectorConstrainedDualArmLinearMovement`, `IndependentDualArmLinearMovement`, `IndependentDualArmFreeMovement`
  - `from husky_assembly_teleop.bar_action_io import CINDY_ACTION_TYPES` (importing it also registers the legacy dtype).
- **Body:** the moved code, byte-identical, with **one change (F7)**. In `cycle_roles`' cross-check loop, also `continue` when `_SPLIT_ROLE_RE.search(mv.movement_id)` matches. This means no mismatch print for split ids; legacy ids still print.

##### C. `husky_assembly_teleop/schedule_io.py`
- **Import:** replace `roles_for_action` with `check_action_kinds`.
- **`LoadedEntry`:** delete the `roles: list` field and its docstring line.
- **`load_entry`:**
  - Directly after `action = _load_checked(entry, path)`, call `check_action_kinds(action, os.path.basename(path))`. The predecessor goes through the same recursive call, so it is checked too.
  - Drop `roles=...` from the constructor.
  - Docstring: "derive kinds, start poses"; `Raises` gains "an action holds two (or no) transfer / insert / retreat movements".

##### D. `husky_assembly_teleop/husky_monitor.py`

**Imports** (top of the file):
- Remove `movement_role` and `cycle_roles` from the `bar_action_io` import.
- Add `COMPLIANT_KINDS`, `is_free_home` and `check_action_kinds`.
- Change the `rs_data_structure.bar_action` import to `CONTROLLER_CARTESIAN_COMPLIANT, CONTROLLER_JOINT_TRACKING`.

**Delete:**
- `MOVEMENT_TRAJECTORY_TIME_S` and the comment block above it that explains role durations.
  - The block starts "Default execution duration per movement role" and ends before "M2 (mate) is executed in two chunks". Keep the M2 split comment.
- `self._loaded_movement_roles` (in `__init__`).
- `_match_movement_role`.
- In the `__init__` comment near `m2_compliant_split_mm`: "See MOVEMENT_TRAJECTORY_TIME_S's neighbourhood" becomes "See M2_COMPLIANT_SPLIT_MM".

**New helpers.** Put them in the "PER-MOVEMENT BARACTION FLOW" section where `_match_movement_role` was. Google docstrings throughout. Read `_loaded_movements` and `_loaded_action_slots` through `getattr(..., [])`, because the headless harnesses skip `__init__`.

| Helper | Behaviour |
|---|---|
| `_kind_of(self, mv) -> Optional[MovementKind]` | `movement_kind(mv)`. Returns None for `mv is None` or an unknown class (catch `TypeError`). |
| `_is_free_home(self, mv) -> bool` | Find the slot action in `_loaded_action_slots` whose `movements` contains `mv` **by identity**, then return `is_free_home(action, mv)`. False when `mv` is not loaded. Works for the schedule (one slot), the legacy J+R list (two slots) and the legacy single file. |
| `_is_free_to_load(self, mv) -> bool` | `self._kind_of(mv) is MovementKind.DUAL_FREE and not self._is_free_home(mv)` |
| `_loaded_index_of(self, kind: MovementKind, free_home: Optional[bool] = None) -> Optional[int]` | First index in `_loaded_movements` with that kind. When `free_home` is given, the movement's `_is_free_home(m)` must also equal it. |
| `_loaded_movement_of(self, kind, free_home=None) -> Optional[Movement]` | One-line wrapper over `_loaded_index_of`. |
| `_warn_controller_mismatch(self, mv, runs: str) -> None` | If `mv.controller != runs` and `mv.movement_id` is not in `getattr(self, '_controller_mismatch_warned', set())`, warn once, in plain text: `"{id!r}: the export asks for the {mv.controller} controller; the monitor runs this {kind} movement under {runs}."` Then add the id to the set (create the set on the instance if it is missing). |
| `_chain_sequence(self) -> list` | Indices for Plan Chain (see below). Also used by `headless_live_monitor_test.py`. |

**Per-function changes** (search by name):

| Symbol | Now | Stage 1 |
|---|---|---|
| `_movement_starts_live` | `role == 'M0' or (idx == 0 and conf None)` | `self._is_free_to_load(mv) or (...)`. The schedule J/H idx-0 rule is unchanged. Docstring: "the travel to the loading pose". |
| `_finish_action_load` | sets `_loaded_movement_roles` from `loaded.roles` or `cycle_roles`; roster print `role=` | Drop both role assignments. Keep the `start_ee_sources` lines. Roster line: `f"  [{i}] {mv.movement_id!r} kind={(self._kind_of(mv) or type(mv).__name__)}"` (print `.value` when there is a kind). Docstring: drop "roles". |
| `load_bar_action_file` (legacy list) | `slots = load_action_cycle(...)` then `_finish_action_load` | After `load_action_cycle`, for each `(action, p)` in slots call `check_action_kinds(action, os.path.basename(p))`. Catch `(ValueError, TypeError)`: `self.get_logger().error(f"Not loading {fname}: {e}")` and return. |
| `_hide_unmounted_active_bar` | `!= 'M0'` | `if not self._is_free_to_load(mv): return`. Log prefix `[M0]` becomes `[{mv.movement_id}]`. |
| `_authored_motion_type` (**F5d**) | `'bar_held' if role in (M1, M2)` | Returns `'bar_held'` when `mv.start_state` is not None and any `name, rb` in `rigid_body_states` has `is_built_assembly_body(name)` and `(rb.attached_to_link or rb.attached_to_tool)`; otherwise `'free'`. Docstring: the rule plus the fact that it covers tool and manual steps (bar drawn in the tools). |
| `_refresh_bar_action_readouts` | `f"[{idx}] {role}  {mv.movement_id}"` | `f"{mv.movement_id}  ({kind_text})"`, where `kind_text = kind.value`, or `type(mv).__name__` for an unknown class. The slider shows the index itself (both backends display the value). |
| `load_selected_movement` | `default_trajectory_time(mv, role, MOVEMENT_TRAJECTORY_TIME_S)`; prints `role=` | `free_home = self._is_free_home(mv)`; `default_trajectory_time(mv, free_home=free_home)`, keeping the `TypeError` try/except. Print `(default for {kind}{', free move home' if free_home else ''})`. Final print: `kind=` instead of `role=`. Comments "this role's default" become "this movement's default". |
| `plan_selected_movement` | role None → `_plan_by_kind`, else role dict | Keep the three guards (`_refuse_while_tasks_run`, `_refuse_display_only_entry`, no movement), then `self._plan_by_kind(self.current_movement)`. Docstring updated. |
| `_plan_by_kind` (**only dispatcher**) | single-arm only | See the pseudo-code below. |
| `_CHAIN_ROLE_ORDER` / `plan_movement_chain_live` | role order M1, M2, M3, M0, M4; first movement per role | See below. Logs: `{kind.value} idx= id=`; `stopped_at_role` becomes `stopped_at` (the movement id). Docstrings: "transfer -> insert -> retreat -> travel to load -> free move home". |
| `replan_free_to_movement_start_live`, `replan_transfer_to_movement_start_live` | `role not in ('M2','M3')` | `self._kind_of(mv) not in COMPLIANT_KINDS`. Warning: `"... only works on the insert or the retreat; {id} is a {kind}."` |
| `_ensure_bar_attached_for_mocap` | prefer donor role M2 | `if self._kind_of(mv2) is MovementKind.DUAL_CONSTRAINED_LINEAR: break` (prefer the insert). |
| `exec_selected_movement_traj` | role routing | See below. |
| `_accept_trajectory(mv, jt, *, source='Plan', role=None)` | role-keyed chain | Signature becomes `_accept_trajectory(self, mv, jt, *, source: str = 'Plan') -> None`. Details below. |
| `load_selected_movement_trajectory` | passes `role=` | Drop the `role=` keyword. |
| `_backfill_m0_target_from_m1` | finds M0 and M1 by role | `free_to_load = self._loaded_movement_of(DUAL_FREE, free_home=False)`, `transfer = self._loaded_movement_of(DUAL_CONSTRAINED_FREE)`. Print the real ids. |
| `_drop_movement_trajectory` | `role == 'M1'` → clear start conf | `self._kind_of(mv) is DUAL_CONSTRAINED_FREE` |
| `_drop_m2_m3_after_m1_chain_break` | roles M2, M3 | `self._kind_of(m) in COMPLIANT_KINDS` |
| `_clear_m1_start_conf_without_trajectory` | `!= 'M1'` | `is not DUAL_CONSTRAINED_FREE`. Print the movement id. |
| `_save_m1_m0_confs_to_bar_action_file` | roles M0, M1 | `[m for m in (free_to_load, transfer) if m is not None]`. The log names them by id. |
| `export_m0_plan_to_bar_action_file` | role M0 | `self._loaded_movement_of(DUAL_FREE, free_home=False)`. Messages: "has no travel-to-load movement"; "select {id} and click 'Plan Movement' first"; `Export M0` becomes `Export travel-to-load`. |
| `_plan_free_and_validate(mv, role, …)`, `_diagnose_free_plan_endpoints(mv, goal_conf, role)`, `_patch_preplanned_to_live(mv, role)`, `_resync_start_state_to_live(mv, role)`, `replan_linear_to_target_from_live(mv, role)` | `role` used only as a log tag | Rename the parameter to `tag: str`; docstring "log tag (the movement id)". Bodies: `[{role}` becomes `[{tag}`. `_resync…`: `f'{tag} pre-plan'`. Positional callers keep working. |
| `_plan_M0_dispatch`, `_plan_M4_dispatch` | pass `'M0'` / `'M4'` | Pass `mv.movement_id`. Name and body otherwise unchanged. `_plan_M0_dispatch`'s warning becomes: `"{id} has no goal yet: confirm or plan the transfer start first ({transfer id}); that start becomes this movement's goal."` |
| `_validate_cdfm_planned_path` | `'CDFM' not in movement_id` | `if self._kind_of(mv) is not MovementKind.DUAL_CONSTRAINED_FREE: return` |
| `_m1_goal_conf` | role M2 | `self._loaded_movement_of(DUAL_CONSTRAINED_LINEAR)` |
| `_m1_live_context` | `!= 'M1'`; hint "Load M1 first (Movement slider 1 -> Load Movement)" | `is not DUAL_CONSTRAINED_FREE`. Hint: `"Load the transfer movement first ({id}, index {i}) -> Load Movement."` (or "this action has no transfer movement"). Wrong-movement warning: `"{what} only works on the transfer movement ({transfer id}); loaded is {id} ({kind})."` |
| `_assembled_bar_pose` | roles M3, M2 | Retreat = `_loaded_movement_of(DUAL_INDEPENDENT_LINEAR)`; insert = `_loaded_movement_of(DUAL_CONSTRAINED_LINEAR)`. |
| `_poll_m1_manual_preview`, `_preview_m1_manual_bar` | `!= 'M1'` | `self._kind_of(mv) is not MovementKind.DUAL_CONSTRAINED_FREE` |
| `confirm_m1_manual_start` | final hint mentions "Movement 0" | Final print names the travel-to-load id and index instead of "Movement 0". |
| `adopt_m1_derived_start` | role M1; "Movement slider 0 ->" | `transfer = self._loaded_movement_of(DUAL_CONSTRAINED_FREE)`. Last hint: `"next: select {free_to_load id} (index {i}) -> Load Movement -> Plan Movement -> Exec."` |
| `build_ui` labels | see below | Labels only. Callbacks and attribute names unchanged. |

**`_plan_by_kind(self, mv) -> None`, new control flow.** It replaces the role dict in `plan_selected_movement`.
```
kind = self._kind_of(mv);  None -> warn "Can not plan {id}: unknown movement class; skipping." return
kind in STATIONARY_KINDS -> info "Nothing to plan ..." return            (unchanged)
spec = self._connected_robot()
not kind_fits_robot(kind, spec.dual_arm) -> warn (unchanged text) return   # legacy guard, removed in stage 3
if kind in DUAL_ARM_KINDS:
    # * Cindy: the preamble her four planners share (moved verbatim from plan_selected_movement)
    re-read fm_swept_validation_slider (unchanged block)
    overwrite warning if mv.trajectory is not None
    if not self._apply_live_base_to_movement(mv): return
    self._fill_missing_start_conf(mv.start_state)          # ! dual-arm only: the SINGLE_LINEAR "no start conf" refusal must stay reachable
    planner = { DUAL_FREE: (self._plan_M4_dispatch if self._is_free_home(mv) else self._plan_M0_dispatch),
                DUAL_CONSTRAINED_FREE: self._plan_M1_dispatch,
                DUAL_CONSTRAINED_LINEAR: self._plan_M2_dispatch,
                DUAL_INDEPENDENT_LINEAR: self._plan_M3_dispatch }[kind]   # ! build per call from self.<name>: the dispatch check replaces these on the instance
    jt = planner(mv)
    if jt is None and kind is DUAL_CONSTRAINED_FREE: self._clear_m1_start_conf_without_trajectory()
else:
    overwrite warning; existing SINGLE_FREE / SINGLE_LINEAR bodies unchanged (they return early on their own precondition failures)
    (delete the now-unreachable "Unknown movement role ... skipping" branch)
if jt is None:
    warn f"Plan for {id!r} ({kind.value}) FAILED."; _reset_planned_arm_trajectory(); _preview_joint_data=None; _draw_preview_joint_values()
    warn "Trajectory preview CLEARED ..." (the dual-arm text; fine for both)
    return
self._accept_trajectory(mv, jt, source='Plan')
```
The docstring lists the 4 dual-arm and 2 single-arm kinds and their planners: DUAL_FREE gives travel to load or the free move home (`is_free_home`), DUAL_CONSTRAINED_FREE gives the transfer, and so on.

**Plan Chain.**
- Replace `_CHAIN_ROLE_ORDER` with the class attribute `_CHAIN_KIND_ORDER = (DUAL_CONSTRAINED_FREE, DUAL_CONSTRAINED_LINEAR, DUAL_INDEPENDENT_LINEAR, DUAL_FREE)`. Comment: the old order transfer → insert → retreat → travel to load → free home; `DUAL_FREE` covers the last two, travel-to-load first.
- `_chain_sequence(self) -> list`: for each kind in order, take every index with that kind. Within `DUAL_FREE`, sort with `key=lambda i: self._is_free_home(self._loaded_movements[i])` (False first; the sort is stable).
- With `check_action_kinds`, each of the first three kinds is unique, so the result equals the old "first per role".
- `plan_movement_chain_live` uses `self._chain_sequence()` instead of building `role_to_idx`. The "nothing to plan" warning names the kinds.

**`exec_selected_movement_traj` routing.**
- Keep the order of today's guards. The current fallback for `kind = None` ("run it the old way by role") becomes a refusal: `warn "{id}: unknown movement class; can not tell how to run it."`, then return.
- Then:
```
if kind in COMPLIANT_KINDS:            runs = CONTROLLER_CARTESIAN_COMPLIANT; task = world.execute_planned_trajectory_compliant
elif kind in SINGLE_ARM_KINDS:         runs = CONTROLLER_JOINT_TRACKING;      call world.execute_arm_trajectory_all(self)
elif kind is DUAL_FREE and not self._is_free_home(mv): runs = JOINT_TRACKING; task = world.execute_trajectory_and_zero_ft
else:                                  runs = JOINT_TRACKING;                 call world.execute_arm_trajectory_both(self)
self._warn_controller_mismatch(mv, runs)   # replaces the single-arm compliant warning
```
- Always call through the module (`world.execute_*`). The dispatch check monkeypatches the module attributes.
- Text updates in this function: "('Plan Movement' for M0)" becomes "(or 'Plan Movement')"; `[Exec] M2 split/mode from slider` becomes `[Exec] insert split/mode from slider`. Docstring: the kind routing.

**`_accept_trajectory`, Stage 1 behaviour (identical to today, keyed by kind).**
- Set `kind = self._kind_of(mv)`.
- Start-conf rule:
  - `kind in COMPLIANT_KINDS` → the existing reject rules.
  - Otherwise write `conf_from_12vec(path[0])`.
- Forward write:
  - `kind is DUAL_FREE` → no forward write. Comment: travel to load ends at the transfer start, which the transfer owns; the free move home ends the action.
  - Otherwise the existing `index+1` write, with the warning text "(transfer / insert / retreat chain rule)".
- Chain break:
  - `kind is DUAL_CONSTRAINED_FREE` → `_drop_m2_m3_after_m1_chain_break(f"{source} {mv.movement_id} endpoint changed by …")`.
  - `kind is DUAL_CONSTRAINED_LINEAR and self._kind_of(next_mv) is DUAL_INDEPENDENT_LINEAR` → `_drop_movement_trajectory(next_mv, …)`.
- Backward check: unchanged.
- `tag = source`.
- Preview: unchanged call (`self._refresh_preview_attached_bodies(self._authored_motion_type(mv), mv.start_state)`).
- **Validation gate:** the `show_transfer_validation` gate becomes `kind in (DUAL_CONSTRAINED_FREE, DUAL_CONSTRAINED_LINEAR)`, as a local tuple with the comment "the rigid two-hand hold is a property of these two classes". It stays off the motion type on purpose. Under `BAR_ACTION_MOCAP_ACCURACY_TEST`, `_ensure_bar_attached_for_mocap` attaches the bar in the retreat's start state, and with F5d the retreat would otherwise get a meaningless bar-hold drift check.
- `if kind is DUAL_CONSTRAINED_FREE: self._backfill_m0_target_from_m1()`.

**`build_ui` labels** (labels are display-only; I checked that no code or test looks widgets up by label):

| Old | New |
|---|---|
| `"Movement (idx; 0=M0_synth)"` | `"Movement (index in this action file)"` |
| `"M1 home anchor (0:all,1:horiz,2:vert,3:back)"` | `"Transfer start: home anchor (0:all,1:horiz,2:vert,3:back)"` |
| `"M1 manual start: slide along bar (m)"`, and roll / perp. 1 / perp. 2 | `"Transfer start: slide along bar (m)"`, `"…: roll about bar (deg)"`, `"…: shift perp. 1 (m)"`, `"…: shift perp. 2 (m)"` |
| `'M1: Confirm manual start pose (IK check)'` | `'Transfer start: Confirm manual pose (IK check)'` |
| `'M1: Derive Start/Goal only (no RRT)'` | `'Transfer start: Derive start/goal only (no RRT)'` |
| `'M1: Adopt derived start -> M0 goal'` | `'Adopt derived start -> travel-to-load goal'` |
| `"Adopt also saves M0/M1 confs to file"` | `"Adopt also saves travel-to-load / transfer confs to file"` |
| `'Save M0 Plan to BarAction file'` | `'Save travel-to-load plan to BarAction file'` |
| `"M2 rigid->compliant split (mm to goal)"` | `"Insert: rigid->compliant split (mm to goal)"` |
| `"M2 exec (0:rigid+compliant split, 1:rigid only)"` | `"Insert exec (0:rigid+compliant split, 1:rigid only)"` |
| `"M0/M4 swept collision check (0:off, 1:on)"` | `"Free moves: swept collision check (0:off, 1:on)"` |
| `'2) IK Replan & Transit → Mv Start (live, M2/M3)'` | `'… (live, insert/retreat)'` |

Also update the comments next to these (`# * Button 1: plan the M1->M2->M3->M0->M4 chain`, `Auto-dispatch by role: M2/M3 ->…`), plus the hint strings that quote old labels: `"Click 'M1: Adopt derived start' …"` in `_show_m1_endpoints`, `"Nothing to adopt: click 'M1: Derive Start/Goal only' first."`, and `"tick 'Adopt also saves M0/M1 confs to file'"`.

##### E. `husky_assembly_teleop/husky_world.py`
- **Import:** `from husky_assembly_teleop.bar_action_io import COMPLIANT_KINDS, MovementKind, movement_kind, tool_event`.
- **`save_markerset_data`:**
  - `movement_id = getattr(mv, 'movement_id', None)` (the real id).
  - From `slot = monitor._slot_of_movement(...)`: `movement_index = slot[2] if slot else None` (the index inside that file) and `action_file = os.path.basename(action_path) if action_path else None`.
  - Add `'movement_index'` and `'action_file'` to the payload. Keep `'bar_action_path'`.
  - Rewrite the stale comment that says "movement_index is intentionally dropped".
- **`servo_to_movement_start_live`:** use a safe kind lookup (try `movement_kind`, `TypeError` → None) and gate on `kind not in COMPLIANT_KINDS`. Messages: `'Load the insert or the retreat movement first.'` and `f'Servoing loop only supports the insert / retreat; {mv.movement_id} is a {kind}.'`
- **`execute_planned_trajectory_compliant`:**
  - Set `mv = monitor.current_movement`; kind via the safe lookup; `tag = mv.movement_id`.
  - Gate: `kind not in COMPLIANT_KINDS` → warn `"Compliant exec only runs the insert or the retreat; {tag} is a {kind}."`, then return.
  - `is_insert = kind is MovementKind.DUAL_CONSTRAINED_LINEAR`, `is_retreat = kind is MovementKind.DUAL_INDEPENDENT_LINEAR`. Every `role == 'M2'` becomes `is_insert`; every `role == 'M3'` becomes `is_retreat`.
  - Log tags `[M2]` / `[M3]` / `M2:` / `M3:` / `(role {role})` become `[{tag}]` / `({kind.value})`.
  - `monitor.replan_linear_to_target_from_live(mv, tag)`.
  - Both `_execute_rigid_chunk` calls get `label=tag`. Change its default `label='M2'` to `'rigid'`, use `[{label}]` in its "both joint motors STALLED" line, and docstring "label: log tag (the movement id)".
  - `_hold_until_joint_motors_stall` gains `label: str = 'insert'`, uses `[{label}]` in place of `[M2]`, and the caller passes `label=tag`.
  - The tool commands, splits, rigid-only, stall and settle logic, and confirm pause are **unchanged**.
  - The docstring summary names the insert (DUAL_CONSTRAINED_LINEAR) and the retreat (DUAL_INDEPENDENT_LINEAR).
- **User-facing hints by role (cheap; reword by meaning):**
  - `execute_trajectory_and_zero_ft`'s three log lines: "M0 done" becomes "Travel to load done"; "then load and execute M1" becomes "then load and execute the transfer".
  - `run_scaffolding_tool_step`'s `runs_in` strings drop "(M2)" and "(M3)".
  - Do not touch the other docstrings and comments.

##### F. Scripts and offline tools

- **`scripts/headless_schedule_smoke.py`:**
  - `DISPATCH_EXPECTED` rows `('J',1)`, `('J',2)`, `('J',4)`, `('R',0)`: `preview` becomes `'bar_held'`. Reword the comment above to "since stage 1 (F5d) …".
  - `plan_release`: in the two labels, `(role {monitor._match_movement_role(mv)})` becomes `({movement_kind(mv).value})`; import `movement_kind`.
  - Also F7: find the retreat and the free move home by kind instead of `_loaded_movements[2]` / `select_movement(monitor, 3)`, using `monitor._loaded_index_of(DUAL_INDEPENDENT_LINEAR)` and `monitor._loaded_index_of(DUAL_FREE, free_home=True)`.
  - Module docstring: "R_M2 (retreat, role M3) and R_M3 (home, role M4)" becomes "the retreat and the free move home".
- **`scripts/headless_live_monitor_test.py`:**
  - `_print_roster`: `role=` becomes `kind=`.
  - `_diagnose_m0_transit_failure`: replace `monitor._loaded_movements[1]` with `monitor._loaded_movement_of(MovementKind.DUAL_CONSTRAINED_FREE)`, and the skip message with "no transfer movement loaded". This was a real bug: index 1 is the manual mount in split exports.
  - `_TREE_DRAW_ROLES` becomes `_TREE_DRAW_KINDS = (DUAL_FREE, DUAL_CONSTRAINED_FREE)`; `_BIRRT_DRAW_ROLES` becomes "kind is DUAL_FREE" via `monitor._kind_of(cur_mv)`.
  - The `_plan_M1_dispatch` monkeypatch stays as it is; the dispatcher still calls `self._plan_M1_dispatch`.
  - `_replay_saved_trajectories`: print the kind.
  - `main`: delete `role_to_idx`.
    - `--only-movement KEY`: digits give an index into `monitor._loaded_movements`; otherwise call `find_movement(SimpleNamespace(movements=monitor._loaded_movements), key)` (an exact id, then a `_key_` fragment, then a substring). On `KeyError`/`IndexError`, print FAIL with the available ids.
    - The default sequence is `monitor._chain_sequence()`.
    - The failure diagnosis uses `monitor._kind_of(mv) is DUAL_FREE` / `monitor._is_free_to_load(mv)`.
  - `_run_button_mode(monitor, sequence, button, bar_action)`, with `role_to_idx` dropped:
    - Choices become `('chain', 'replan-insert', 'replan-retreat')`.
    - Prerequisites by kind: insert needs `[DCF]`; retreat needs `[DCF, DCL]`.
    - Indices come from `monitor._loaded_index_of`.
  - argparse:
    - `--only-movement`: no choices; help "index in the loaded list (a J file loads its J then its R movements) or an id fragment, e.g. J_M3 or LM_insert".
    - `--button`: the new choices.
  - Update the module docstring lines about `_plan_M{0..4}_dispatch` and the order.
- **`scripts/movement_collision_inspector.py`:**
  - `_find_movement_by_role` becomes `_pick_movement(monitor, key: str) -> tuple`, with the same rule as `--only-movement` (reuse `find_movement` via `SimpleNamespace`).
  - Parameter `movement_role` becomes `movement_key`.
  - `--movement`: no choices; `DEFAULT_MOVEMENT = "0"`; the commented alternative becomes `"1"`.
  - Docstring usage: `--movement 0` and `--movement J_M5`.
- **`scripts/derive_m1_headless.py`:**
  - Drop the `movement_role` import; import `MovementKind`, `movement_kind` and `is_free_home`.
  - Replace `roles_of` with `transfer_steps(action, release=None) -> dict` keyed by `MovementKind` (the first of each kind over action then release).
  - `free_home`: `next((m for m in owner.movements if is_free_home(owner, m)), None)` over `(release, action)`.
  - `stamp_keyframes`: `confs` keyed by `MovementKind`, matched with `movement_kind(mv)`.
  - `prepare_state(session, stub, transfer, insert, hide_built=True)`.
  - **The only role strings allowed:** one local constant next to the `solve_chain_with_base_search` call:
    `TAMP_KEYFRAME_KEYS = {'M1': DUAL_CONSTRAINED_FREE, 'M2': DUAL_CONSTRAINED_LINEAR, 'M3': DUAL_INDEPENDENT_LINEAR}`
    with `# ! the external tamp solver (walkable_ground.solve_chain_with_base_search) names its keyframes by the old roles; this is the only place they appear`. Build `{key: steps[kind] ...}` for the call and map `solved[key]` back to kinds.
  - `plan_bar`: `if DUAL_CONSTRAINED_FREE not in steps: raise ...`; `movement_id=steps[DCF].movement_id`.
- **`data/bar_holding_acc_data/1_compare_to_cell_state.py`:**
  - Import `cycle_roles` from `husky_assembly_teleop.legacy_bar_action_io`. `find_movement`, `load_action_cycle` and `slot_of_index` stay from `bar_action_io`.
  - The `_resolve_take_movement` body is unchanged. Old takes stamped `'M3'` hit `key in roles`; new takes stamped with a real id hit the exact-id search in each half.
  - Update the docstring line "Takes stamp the classic ROLE" to "Old takes stamp…; newer takes stamp the real id".
  - `0_bar_acc_data_processing.py` only prints `movement_id`; no change.

##### G. Tests
- **`test/test_bar_action_io.py`:**
  - Fix the import block: remove `movement_role`, `cycle_roles`, `roles_for_action` and `movement_controller`; add `COMPLIANT_KINDS`, `is_free_home`, `check_action_kinds` and `FREE_HOME_TRAJECTORY_TIME_S`.
  - **Move unchanged** to `test/test_legacy_bar_action_io.py` (imports adjusted only):
    - `test_roles_legacy_ids`, `test_roles_split_ids`
    - `test_unchanged_bar_gives_the_same_reference_pose_in_both_exports`, `test_legacy_only_bar_loads_as_one_cycle`, `test_new_only_bar_loads_both_halves`, `test_start_ee_source_is_cleared_by_an_arm_movement`, `test_sidecar_falls_back_to_the_clean_release_half` (all of these use `cycle_roles` through `_role_index` / `_start_ee_source`)
    - `test_hold_ids_have_no_role`, `test_roles_for_real_actions`
    - Copy the small helpers and constants they need into the new file (`_actions_dir`, `_mv`, `_role_index`, `_start_ee_source`, `_load_schedule_action`, `needs_exports`, `needs_schedule_export`, the problem/bar constants). Do not import between test modules.
  - **Delete** `test_movement_controller_warns_on_disagreement`.
  - **Rewrite** `test_default_trajectory_time`:
    - `DUAL_FREE` 30, and 10 with `free_home=True`
    - DCF 10; DCL and DIL 5
    - `SingleArmFree` 15; `SingleArmLinear` 5
    - `GripperTool` / `Manual` → None, including with `free_home=True`
    - `free_home=True` on DCF still 10
- **New tests** (pure / in-memory unless noted; never a `RobotCell` in pytest):
  - `test_compliant_kinds`: equals `{DCL, DIL}`.
  - `test_is_free_home_in_memory`: actions built from rs_data_structure classes (`BarAssemblyJointingAction(movements=[...])` and similar):
    - Jointing `DUAL_FREE` → False.
    - Release `DUAL_FREE` → True; a release retreat → False.
    - Legacy `BarAssemblyAction([free0, dcf, dcl, dil, free1])`: free0 False, free1 True.
    - A `BarHoldingAction`'s `SingleArmFree` → False.
  - `test_is_free_home_on_fixture` (`@needs_schedule_export`): B1__J has no free-home movement. In B1__R exactly the one with id `B1_R_M3_free_home` is free home (test data may use real ids).
  - `test_is_free_home_on_legacy_export` (`@needs_exports`): in 260715 `B1.json` only the last movement is free home.
  - `test_check_action_kinds_accepts_real_exports`: the fixture's `B1__J`, `B1__R`, `B3__H`, `B3__HR` (`@needs_schedule_export`) and 260715 `B1.json`, `B4.json` (`@needs_exports`) raise nothing.
  - `test_check_action_kinds_rejects_bad_counts` (in memory), each raising `ValueError` with the `source` string in the message:
    - a Jointing action with two DCF and no DCL
    - a Release action with no DIL
    - a legacy action with one `DUAL_FREE`
  - Also in that test: a Holding action with any movements passes.
- **`test/test_legacy_bar_action_io.py`:** add one F7 test, `test_no_mismatch_print_for_split_ids(capsys)`. Call `cycle_roles([(BarAssemblyReleaseAction(movements=[IndependentDualArmLinearMovement(movement_id='B1_R_M1_LM_retreat')]), None)])` (the D7-style id that `movement_role` reads as None). Assert the result is `['M3']` and nothing is printed.
- **`test/test_schedule_io.py`:**
  - Drop the three `.roles` asserts (lines with `loaded.roles`).
  - F7: in `test_release_starts_where_the_jointing_left_the_flanges`, derive the layout from the file:
    - `kinds[-2:] == [DIL, DUAL_FREE]` and every earlier kind is `SCAFFOLDING_TOOL`
    - loop the start-source check over `range(len(kinds) - 1)`
    - check the retreat index via `kinds.index(DIL)`
  - New `test_load_entry_refuses_two_transfers(tmp_path)`:
    - `_copy_problem(tmp_path, ['B1__J.json'])`.
    - In the copy's text, replace `EndEffectorConstrainedDualArmLinearMovement` with `EndEffectorConstrainedDualArmFreeMovement`. There is exactly one occurrence, and both classes have the same fields.
    - `pytest.raises(ValueError, match='B1__J.json')` on `load_entry(load_schedule(root), copy.entry(0))`.
- **`test/test_schedule_ui.py` (F7):** in `test_step_kind_sequences` the B1_R row becomes a structural check: the last two are `'arm'`, all earlier are `'scaffold'`. Keep the B1_J row.

#### 1.3 Tasks (Stage 1)
| # | Task | Files | Depends on |
|---|---|---|---|
| S1-A | Data layer + legacy module + tests | `bar_action_io.py`, NEW `legacy_bar_action_io.py`, `schedule_io.py`, `test/test_bar_action_io.py`, NEW `test/test_legacy_bar_action_io.py`, `test/test_schedule_io.py`, `test/test_schedule_ui.py` | none |
| S1-B | Monitor | `husky_monitor.py` | the S1-A API (names and signatures fixed above), so it can run in parallel; verify after S1-A |
| S1-C | World | `husky_world.py` | the S1-A names; parallel |
| S1-D | Scripts | `headless_schedule_smoke.py`, `headless_live_monitor_test.py`, `movement_collision_inspector.py`, `derive_m1_headless.py`, `data/bar_holding_acc_data/1_compare_to_cell_state.py` | the monitor helper names from S1-B (`_kind_of`, `_loaded_index_of`, `_loaded_movement_of`, `_is_free_to_load`, `_chain_sequence`); start after S1-B, or in parallel against this spec |

One person should own `husky_monitor.py` (10k lines). Do not split it.

#### 1.4 Dispatch-check expectations after Stage 1
Identical to today except `preview` = `bar_held` for `J_M1_manual_mount_bar`, `J_M2_tool_grasp_bar`, `J_M4_tool_tighten_joint` and `R_M0_tool_untighten_joint`. Expected: all 10 rows pass, plus the 2 count checks. `planner`, `exec`, `tool_cmd`, `traj_time` and `starts_live` must be unchanged. Any other difference means stop and report it.

#### 1.5 Risks (Stage 1), most severe first
1. **Hardware routing.** Insert gets compliant + TIGHTEN; retreat gets compliant + gripper LOOSEN. The dispatch check's `tool_cmd` column is the guard; it runs the real compliant generator.
   - Do not import `execute_*` by name into the monitor, and do not build the planner dict from class functions. Both would bypass the check's monkeypatches and record `none`.
2. **`_fill_missing_start_conf` must run for dual-arm kinds only.** Running it for `SINGLE_LINEAR` makes the "no start configuration" refusal dead, so a support linear move would sweep from home.
3. **New behaviour on a single-arm robot with a Cindy file loaded through the legacy list.** Previously the role path ran Cindy's planners; now the `kind_fits_robot` guard refuses it. This is intended, but it is a change.
4. **F5d plus the accuracy test.** After `_ensure_bar_attached_for_mocap`, the retreat previews as `bar_held`. That is correct for mount-once, so I kept `show_transfer_validation` keyed on the two constrained kinds to avoid a bogus drift check on the retreat.
5. **Loading errors.** `load_schedule_entry` does not catch `ValueError` from `load_entry`; it behaves exactly like today's type/robot mismatch. The legacy list loader catches and logs.
6. **Phase A call sites.** Code added by Phase A that still calls the old `default_trajectory_time` signature or `_match_movement_role` breaks at import or at runtime. Run the grep gate in §1.6.

#### 1.6 Verification (Stage 1)
```bash
cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate && source install/setup.bash
export DESIGN_DATA_DIRECTORY="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly/data_design_study"
export DESIGN_PROBLEM_NAME=260920_RobArch_demo_revamp_backup
export HUSKY_IK_BACKEND=gradient
mkdir -p ~/husky_dryrun/roles_refactor
R=src/husky-assembly-teleop
# 1. imports + compile
python -c "import husky_assembly_teleop.husky_monitor, husky_assembly_teleop.husky_world, husky_assembly_teleop.schedule_io, husky_assembly_teleop.legacy_bar_action_io"
python -m py_compile $R/scripts/*.py $R/data/bar_holding_acc_data/1_compare_to_cell_state.py
# 2. grep gates (expected output in comments)
grep -rnE "_match_movement_role|_loaded_movement_roles|MOVEMENT_TRAJECTORY_TIME_S|movement_controller|_CINDY_CONTROLLER_BY_ROLE|_CHAIN_ROLE_ORDER|role_table" $R/husky_assembly_teleop $R/scripts $R/test     # -> nothing
grep -rnE "movement_role|cycle_roles|roles_for_action" $R/husky_assembly_teleop $R/scripts $R/test $R/data/bar_holding_acc_data/*.py   # -> only legacy_bar_action_io.py, test_legacy_bar_action_io.py, 1_compare_to_cell_state.py
grep -nE "['\"]M[0-4]['\"]" $R/husky_assembly_teleop/*.py   # -> only legacy_bar_action_io.py, husky_robot.py / scaffolding_tool_client.py (tool MOTOR names)
grep -nE "['\"]M[0-4]['\"]" $R/scripts/*.py                 # -> only derive_m1_headless.py TAMP_KEYFRAME_KEYS
# 3. pytest  (expect: previous total - 1 deleted test + new tests, 0 failed)
python -m pytest $R/test -q -p no:cacheprovider --ignore=$R/test/test_flake8.py --ignore=$R/test/test_pep257.py --ignore=$R/test/test_copyright.py
# 4. smokes (read summary lines only)
python $R/scripts/headless_schedule_smoke.py > ~/husky_dryrun/roles_refactor/stage1_schedule_smoke.log 2>&1; grep -E "dispatch:|passed|failed|FAIL" ~/husky_dryrun/roles_refactor/stage1_schedule_smoke.log | tail -30
python $R/scripts/smoke_single_arm_plan.py > ~/husky_dryrun/roles_refactor/stage1_single_arm.log 2>&1; tail -5 ~/husky_dryrun/roles_refactor/stage1_single_arm.log   # 7/7
# 5. scripts parse their args
python $R/scripts/headless_live_monitor_test.py --help >/dev/null && python $R/scripts/movement_collision_inspector.py --help >/dev/null && python $R/scripts/derive_m1_headless.py --help >/dev/null
```
6. **Read-only probe of the offline tools.** Write it in the scratchpad.
   - Load `1_compare_to_cell_state.py` with `importlib.util.spec_from_file_location` (the name starts with a digit).
   - `_resolve_take_movement(<260929>/BarActions/B1__J.json, 'M3')` and `(…, 'B1_R_M2_LM_retreat')` both return the movement `B1_R_M2_LM_retreat`.
   - `(<260715>/BarActions/B1.json, 'M3')` returns `B1_M3_LM_retreat`.
   - Import `derive_m1_headless` from `scripts/` and check that `transfer_steps(parse(B1__J), parse(B1__R))` gives DCF/DCL/DIL = `B1_J_M3_…` / `B1_J_M5_…` / `B1_R_M2_…`.
   - **Do not** run `derive_m1_headless --plan` or `--manual-start` on real data: `place_base` writes `.solved_keyframe.json` into the shared drive when a base is unplaced.
7. **GUI (the user or main session):** `B1_R_M0` shows the bar in the tools.

---

### STAGE 2: one carry rule for configurations (chain fix + F5e)

#### 2.1 Context
`_accept_trajectory` writes a planned end only into `index+1`. In split exports that is the tighten step, so the insert never sees the transfer's real end. Meanwhile `_accept_single_arm_trajectory` already walks across stationary steps. Stage 2 makes one helper for that walk and uses it for both robots and for the transfer-start adopt path. The adopt path is F5e: the mount and grasp steps get the bar-loading configuration, so the held bar is drawn there.

#### 2.2 Changes (`husky_monitor.py` only, plus the smoke)

**New helpers** (next to `_merge_arm_values`):
- `_next_arm_index(self, idx: int) -> Optional[int]`: the first index > idx whose `_kind_of` is not in `STATIONARY_KINDS`. An unknown kind counts as an arm movement, so the walk stops there.
- `_previous_arm_index(self, idx: int) -> Optional[int]`: the last index < idx that is not stationary.
- `_carry_configuration_forward(self, idx: int, values, *, source: str = 'carry') -> None`
  - Docstring: "Write an arm configuration into the start state of every stationary step after `idx` and of the next arm movement, then stop. `idx = -1` starts at the first movement."
  - Writes by name with `self._merge_arm_values(next_mv.start_state, self._connected_robot().all_arm_joint_names, values)`. That is 12 joints for Cindy (left then right, the same order as `HUSKY_DUAL_UR5e_JOINT_NAMES`) and 6 for a support robot.
  - Skips movements with no `start_state`.
  - Prints one line per write: `[{source}] propagated ... -> {id}.start_state.robot_configuration`.
  - This comes from the loop in `_accept_single_arm_trajectory`.

**`_accept_single_arm_trajectory`:** replace its forward loop with `self._carry_configuration_forward(idx, path[-1], source=source)` when `idx is not None`. No behaviour change.

**`_accept_trajectory` (dual arm)**, inside `if path:` after the start-conf rule:
```
idx = self.current_movement_index
if kind is DUAL_CONSTRAINED_FREE:            # * F5e: mount/grasp steps stand where the transfer starts
    prev = self._previous_arm_index(idx)
    self._carry_configuration_forward(-1 if prev is None else prev, path[0], source=source)
if kind is not DUAL_FREE:                    # free moves still carry nothing (travel to load ends at the transfer's own start; home ends the action)
    nxt = self._next_arm_index(idx)
    if nxt is not None:
        next_mv = self._loaded_movements[nxt]
        existing_vec = vec12(next_mv.start_state.robot_configuration) if set, elif _trajectory_has_waypoints(next_mv): its first waypoint, else None
        if existing_vec is not None and max|path[-1]-existing_vec| > 1e-3:
            warn "... differs from existing {next id}.start ...; overwriting (carry rule)."
            if kind is DUAL_CONSTRAINED_FREE: self._drop_m2_m3_after_m1_chain_break(...)            # same rule as today
            elif kind is DUAL_CONSTRAINED_LINEAR and self._kind_of(next_mv) is DUAL_INDEPENDENT_LINEAR:
                self._drop_movement_trajectory(next_mv, ...)                                  # insert -> retreat (legacy files)
    self._carry_configuration_forward(idx, path[-1], source=source)
backward check: prev = self._previous_arm_index(idx); compare movements[prev].trajectory[-1] vs path[0] (skip when prev is None or no trajectory)
```
- **Decision on the old "M2 → M3 drop" rule:** keep it by kind, and apply it to the next **arm** movement the carry reaches.
  - Legacy single file: the retreat directly follows the insert.
  - Legacy J+R list: the retreat follows after `R_M0` and `R_M1`. The insert's end now reaches the retreat; that is new and correct (physically the retreat starts where the insert ended).
  - Schedule mode: the insert is the last movement of J, so nothing is carried and the rule never fires. The R entry is loaded separately.

**F5e in the adopt path (`adopt_m1_derived_start`; `confirm_m1_manual_start` reaches it).** After writing the transfer's start:
- `ti = self._loaded_index_of(DUAL_CONSTRAINED_FREE)`; `prev = self._previous_arm_index(ti)`.
- `self._carry_configuration_forward(-1 if prev is None else prev, derived['start_conf'], source='Adopt')`.
- This writes the mount and grasp steps, and the transfer again with the same value.

**What reads the tighten step's start configuration?** Nothing breaks when it is overwritten:
- `load_selected_movement` (preview pose): now the transfer's end, which is the intended F5e drawing.
- `world.move_arms_to_movement_start`: moves to that end, which is fine.
- The sidecar save.
- Not read by `progress_io.belief_after` (it uses only the last movement), not by `scan_entry_flags` (target confs), not by `run_scaffolding_tool_step` (mark-only), not by `_movement_starts_live` (cached at load, index 0 only).

**Smoke (`scripts/headless_schedule_smoke.py`):** new `carry_check(results, problem, root, schedule)`, called in `main` after `dispatch_check`:
- Cindy monitor via `make_monitor` / `add_viz_huskies`; `_load_schedule_state()`; `load_schedule_entry(0)`.
- Indices: `ti`, `ii` (insert) and `fi` (travel to load) from `_loaded_index_of`.
- (a) Adopt:
  - Set `monitor._m1_adopt_writes_file = False` (the harness skips `__init__`).
  - `monitor._m1_derived = {'start_conf': s, 'goal_conf': insert start vec, 'corridor': None, 'source': 'manual'}` with `s = insert start vec + 0.05`.
  - Call `adopt_m1_derived_start()`.
  - Check: every stationary index in `(fi, ti)` has start == s (max abs < 1e-9), and `movements[fi].target_configuration == s`.
- (b) Accepting a transfer plan:
  - `select_movement(monitor, ti)`.
  - Stub on the instance: `_validate_cdfm_planned_path`, `show_transfer_validation` and `show_planned_joint_values` become no-ops (display only); `delattr` afterwards.
  - `jt = joint_trajectory_from_path([s, e])` with `e = insert start + 0.02`; `monitor._accept_trajectory(movements[ti], jt, source='Plan')`.
  - Check: insert start == e; every stationary step in `(ti, ii)` has start == e. Label: "after accepting a transfer plan, insert start = transfer's last waypoint".
- Close the cfab session in `finally`.

#### 2.3 Tasks (Stage 2)
One task, `husky_monitor.py` plus `headless_schedule_smoke.py`. It runs after the Stage 1 commit (it uses `_kind_of` and `_loaded_index_of`).

#### 2.4 Dispatch check after Stage 2
No change from the Stage 1 table. The planner recorders return None and the exec sweep stamps its path with `set_arm_trajectory`, so nothing passes through `_accept_trajectory`.

#### 2.5 Risks (Stage 2)
1. **`_m1_goal_conf` follows the plan.** It reads the insert's start configuration, which after a transfer plan is the plan's end, possibly on a different IK branch than the authored one. Replanning the transfer then aims at its own last end. Legacy single files always behaved this way (M1+1 = M2). Watch the "[M1] goal_conf <- …" line during the GUI gate.
2. **Writes are by joint name now.** A start configuration that was None becomes `zero_full_configuration()` plus the arm values, instead of a 12-joint `Configuration`. Consumers read by name (`vec12_from_conf`), so this should be harmless; it is listed here in case a compas_fab state push complains.
3. **New continuity warnings.** The backward check now reaches across stationary steps, so travel to load vs transfer and transfer vs insert can warn where they were silent before. That is information, not a failure.
4. **Stale copies after a failed transfer plan.** `_clear_m1_start_conf_without_trajectory` clears only the transfer's own start; the mount and grasp steps keep the adopted copy. They are only drawn there.

#### 2.6 Verification (Stage 2)
The same commands as §1.6 points 1–5, with logs to `stage2_*.log`. In addition:
- The smoke must show the two `carry_check` lines passing and the dispatch rows unchanged from Stage 1.
- GUI: after the transfer confirm, `J_M2` shows the bar at the bar-loading pose.

---

#### Things I could not settle (for the main session)
1. **`load_schedule_entry` does not catch `ValueError`.** A bad file raises into the button callback, exactly like today's type/robot mismatch. Should it log one error line and return instead? That is in the spirit of Stage 3's "refused with one clear error". It is not done in this spec.
2. **Pre-existing bug, not fixed (your rule: discuss, don't patch).** `headless_live_monitor_test._install_tree_drawing._patched_m1` imports `M1_POSITION_RES` and `M1_ROTATION_RES`, which no longer exist in `husky_monitor` (they are now `CDFM_*`), and reads `self_.constrained_planner_stage`. So `--draw-tree` with a transfer in the sequence crashes today.
3. **Stage 4 grep gate scope.** It does not catch the log prefixes `[M1 manual]`, `[M2 inter-EE invariance]` and similar, or the CLI choice strings. Decide whether Stage 4 renames them.
4. **Index numbering in legacy J+R list mode.** `--only-movement N` and the new slider label ("index in this action file") count across both files (0..9). This is only true per file in schedule mode, until Stage 3 removes the list.
5. **Saved takes.** `movement_index` is the index inside `action_file`, while `_resolve_take_movement` reads an int key as a cycle index. That only matters for takes with no `movement_id`, and those also have no index.

#### Where this spec deviates from the request
- **`show_transfer_validation` gate:** stays on the two constrained kinds, not on F5d's motion type. Otherwise the retreat in the accuracy test would be checked after the bar is injected.
- **`--button replan-m2/replan-m3`:** renamed to `replan-insert/replan-retreat` in Stage 1. These are role literals in a script, and `--only-movement` changes anyway.
- **Exec of an unknown movement class:** refused with a warning instead of today's fallback to `arm_both`. Unreachable for loaded files after `check_action_kinds` and `LoadedEntry.kinds`.
- **Controller mismatch:** warned once per movement per run. The old single-arm "compliant requested" warning folds into it; it used to repeat on every click.
- **Stage 2 free moves:** `DUAL_FREE` acceptance still carries nothing forward. Travel to load's end is the transfer's own start; the mount and grasp steps get it through the adopt path and the transfer's own accept.
- **Reused helper:** `RobotSpec.all_arm_joint_names` gives the joint names for the carry. I found no existing "first movement of kind" or "warn once" helper, so `_loaded_index_of` and `_warn_controller_mismatch` are new.

Relevant files:
- /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/bar_action_io.py
- /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/legacy_bar_action_io.py (new)
- /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/schedule_io.py
- /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_monitor.py
- /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_world.py
- /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/scripts/headless_schedule_smoke.py
- /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/scripts/headless_live_monitor_test.py
- /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/scripts/movement_collision_inspector.py
- /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/scripts/derive_m1_headless.py
- /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/data/bar_holding_acc_data/1_compare_to_cell_state.py
- /home/yijiangh/Code/ros2_ws/src/husky-assembly-teleop/test/test_bar_action_io.py, test_legacy_bar_action_io.py (new), test_schedule_io.py, test_schedule_ui.py

### Main-session decisions on the spec's open points (2026-10-01)
1. `load_schedule_entry` catches `ValueError` from `load_entry` (bad file, kind counts, type/robot mismatch):
   one `get_logger().error` line naming the file and the reason, then return (nothing loaded). Stage 1.
2. `headless_live_monitor_test._install_tree_drawing._patched_m1` imports names that no longer exist
   (`M1_POSITION_RES`, `M1_ROTATION_RES`): pre-existing, reported to the user, not patched here.
3. Stage 4 renames the log prefixes (`[M1 manual]`, `[M2 inter-EE invariance]`, ...) and script CLI choices too.

## 5. Change log
(appended by the main session after each reviewed step)

- `11eac94` A-F4: the floor allows ground joints (`is_ground_joint_body`, `cfab_session.ground_touch_bodies`). pytest 103 / 1 skipped.
- `465b7bf` Step 0b: dispatch check in `headless_schedule_smoke.py`; today's code matches all 10 rows. Smoke 46/46.
  Found (not fixed, outside scope): `husky_world.split_path_by_distance_to_goal` returns 3 values on its
  short-path early exit; both callers unpack 4.
- `b19010d` A-F5a–c: `IGNORE_BUILT_ASSEMBLY_COLLISIONS` + `_ignore_built_assembly()`, panel toggle (reloads the
  entry), ignored built bodies drawn faint at their state frame; preview bodies set up before the hidden ones are
  drawn. Smoke 54/54. Hold scenes still need the switch ON (Rhino note D5).
- `7938c32` A-F6 + small items: connected robot seeded from its own belief at start-up; header line
  "others: …"; traj time of an entry that opens with tool steps; fake-hardware Move Arms to Movement Start.
  Smoke 60/60, pytest 104 passed. (b) answer: J/H entries already start from the seeded arms; R/HR entries open
  with a tool step whose exported start is not live (same as with ROS) — left unchanged.
- Stage 1 (roles out, + F5d, F7 tests): reviewer APPROVE WITH NITS (no defect; every insert/retreat branch of the
  compliant exec maps 1:1; probe over ~870 movements of 8 problems: F5d rule = old preview on every arm movement,
  `is_free_home` = old M4 on every DUAL_FREE). Nits fixed in the main session. pytest 116 / 1 skipped, smoke 60/60,
  single-arm 7/7. Known leftovers: `scripts/inspect_bar_action_collision_geometry.py` `DEFAULT_MOVEMENT = "M1"`
  (correct for its legacy default problem; stage 3 moves it to schedule entries); pre-existing
  `husky_world.py` `DATA_FOLDER` undefined in the kissing-probe branch; pre-existing `headless_live_monitor_test`
  `--draw-tree` imports `M1_POSITION_RES`.
- Stage 2 (carry rule + F5e): `_next_arm_index`, `_previous_arm_index`, `_carry_configuration_forward`; used by
  `_accept_trajectory`, `_accept_single_arm_trajectory`, `adopt_m1_derived_start`. Smoke 62/62 (new `carry_check`:
  adopt writes J_M1/J_M2 + travel-to-load goal; accepting a transfer plan writes J_M4 and the insert's start).
