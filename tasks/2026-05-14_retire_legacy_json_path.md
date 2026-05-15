# Retire legacy `--grasp-json` / `--start-state` / `--end-state` path

Date: 2026-05-14
Scope: `external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/stage1/`
Affected files: `minimal_rrt.py`, `real_state_study.py`, `debug_runner.py`, `trajectory_replay.py`, and the package `README.md`.

## 1. Goal + non-goals

**Goal.** Drop the legacy "JSON state + GraspTargets" code path. Every stage1 entry point now builds its `scene_spec` exclusively via `build_gdrive_scene_spec` or `build_gdrive_bar_action_scene_spec`. The CLI args `--grasp-json`, `--start-state`, `--end-state` and the JSON loader helpers (`load_grasp_targets`, `load_robot_cell_state`, `load_robot_cell_state_data`, `build_default_paths`, `build_real_design_goal_spec`, `load_design_study_bar_mesh`, `design_study_active_bar_body_name`, `DESIGN_STUDY_BAR_SEQUENCE`, `DESIGN_STUDY_BAR_NAME_TO_INDEX`) all go away. `setup_planning_scene` becomes `scene_spec`-only.

**Non-goals.** No changes to scaffolding/RS485/hardware code. Behavior of the gdrive code paths is preserved exactly; only the dead legacy branches and their CLI surface disappear. No change to `path_validation.py`, `trajectory_io.py`, `zh_archive/`, or `husky_assembly_tamp/model/target_parse.py` (its own copies of `load_grasp_targets` are not in scope).

## 2. File-by-file edits

Use grep / Read by symbol name; line numbers below are anchors only — verify before editing because surrounding code may have shifted.

### 2.1 `minimal_rrt.py`

Delete (verify each symbol has no remaining caller after the other-file edits in 2.2–2.4 land):

- `DESIGN_STUDY_BAR_SEQUENCE` and `DESIGN_STUDY_BAR_NAME_TO_INDEX` module-level constants (around L89–104). Only the deleted helpers reference them.
- `load_grasp_targets` (around L193).
- `load_robot_cell_state_data` (around L215) and `load_robot_cell_state` (around L225).
- `design_study_active_bar_body_name` (around L278) and `load_design_study_bar_mesh` (around L284). Note: `load_gdrive_active_bar_mesh` (around L1557) is the new replacement — keep it.
- `build_real_design_goal_spec` (around L456).
- `build_default_paths` (around L1522). Drop the docstring comment block (L1530–L1542) only if it is now stale; keep the rest of the gdrive block intact.

Edit:

- `setup_planning_scene` (def around L1954):
  - Drop positional params `grasp_json`, `start_state_json`, `end_state_json`. New signature: `setup_planning_scene(scene_spec: Dict[str, Any], use_gui: bool = False, swap_grasps: bool = False) -> Dict[str, Any]`.
  - Make `scene_spec` required (no `Optional`, no default). At the top, replace `scene_spec = dict(scene_spec or {})` with `scene_spec = dict(scene_spec)` and add an explicit check that the required keys are present (see §3 below).
  - In the `grasp_targets = scene_spec.get("grasp_targets")` block (around L2002–L2006), drop the JSON-fallback branch and the now-stale `f"... in {grasp_json}"` message — raise `ValueError("scene_spec must contain non-empty 'grasp_targets'")` instead.
  - In the start/end joint values block (around L2014–L2023), drop the `else` branches that call `load_robot_cell_state`. Both must come from `scene_spec`. Keep the lazy-loading comment removed.
  - `swap_grasps` parameter: keep for now (caller still threads it). It is now a no-op once `grasp_targets` is mandatory in `scene_spec`; either delete it entirely *or* leave with a comment that it is reserved. **Recommended: delete `swap_grasps` from `setup_planning_scene` and `run_stage_trial` signatures and remove the `--swap-grasps` arg from `minimal_rrt.main()` and `real_state_study.parse_args()`.** Grep first to confirm no external caller passes `swap_grasps=...` (none expected outside this dir).

- `run_stage_trial` (def around L2107):
  - Drop `grasp_json`, `start_state_json`, `end_state_json` positional params and the matching forwards at the `setup_planning_scene(...)` call (around L2143–L2150).
  - Make `scene_spec` required (drop the `Optional`/default).
  - Drop `swap_grasps` if §2.1 setup change drops it.

- `main()` (around L2686):
  - Delete the `default_grasp_json, default_start_state, default_end_state = build_default_paths()` line and the three `parser.add_argument(...)` calls for `--grasp-json`, `--start-state`, `--end-state` (around L2687–L2691).
  - Validate that exactly one of `--gdrive-state` / `--gdrive-bar-action` is supplied; raise on neither. The existing mutual-exclusion check (L2787–L2788) stays.
  - In the final `run_stage_trial(...)` call (around L2814–L2841), drop `grasp_json=args.grasp_json`, `start_state_json=args.start_state`, `end_state_json=args.end_state`. Pass `scene_spec=gdrive_scene_spec` as before (required now).
  - Drop `--swap-grasps` and `swap_grasps=args.swap_grasps` if §2.1 change removes the param.

Keep (verify): `DATA_DIR` import stays — it is used by `HUSKY_DUAL_URDF_PATH` / `HUSKY_DUAL_SRDF_PATH` (L42–48). `GDRIVE_DATA_DIRECTORY`, `load_gdrive_active_bar_mesh`, `_resolve_gdrive_state_path`, `_resolve_gdrive_bar_action_path`, `build_gdrive_scene_spec`, `build_gdrive_bar_action_scene_spec`, `_fk_dual_arm_grasps_in_mb_frame`, `_joint_values_from_robot_cell_state`, `_joint_values_from_configuration` all stay.

### 2.2 `real_state_study.py`

Delete imports (top-of-file block around L16–L40):

- `DESIGN_STUDY_BAR_SEQUENCE`
- `build_default_paths`
- `build_real_design_goal_spec`
- `load_robot_cell_state`

Delete functions:

- `compute_common_start_context` (around L327): only the deleted `else` branch calls it.
- `derive_start_pose_from_home_left_tool` (around L338): only called from `run_endpoint_ik_diagnosis` and `validate_auto_home_start_context_with_temporary_scene` (also legacy-only). Grep before deleting; confirm no other live caller.
- `validate_auto_home_start_context` (around L659) and `validate_auto_home_start_context_with_temporary_scene` (around L728): both feed the deleted legacy planning branch.
- `run_endpoint_ik_diagnosis` (around L770), `summarize_endpoint_ik_diagnosis` (around L547), `write_endpoint_ik_report` (around L561), `hold_gui_pose` (around L600), `evaluate_endpoint_ik` (around L612): all only used via the diagnose-endpoint-ik flow that depends on the legacy `spec["grasp_json"]` / `args.start_state` plumbing. Sweep them all unless the user explicitly wants to keep endpoint-IK diagnosis on gdrive inputs. **Recommendation: delete all of them and the related `--diagnose-endpoint-ik` / `--diagnose-start-collision` args.** If diagnosis is wanted later, re-add a gdrive-native version.
- `build_scene_spec_from_start_context` (around L409): used only by the deleted legacy branch + the deleted `run_endpoint_ik_diagnosis`.

Edit:

- `build_replay_command` (around L228): remove `grasp_json`, `start_state_json`, `end_state_json` parameters and the three corresponding `--grasp-json` / `--start-state` / `--end-state` shell tokens. Update the one caller at L1175.
- `parse_args` (around L905):
  - Drop the `default_grasp_json, default_start_state, default_end_state = build_default_paths()` line (L906) and the three legacy `add_argument` calls (L918–L920).
  - Drop `--diagnose-start-collision` and `--diagnose-endpoint-ik` if the diag functions are removed (recommended).
  - Drop `--swap-grasps` if §2.1 removes the param.
  - Validate that `args.gdrive` or `args.gdrive_bar_action` is set after parsing; raise a clear error otherwise (there is no legacy fallback anymore).
- `main` (around L1050):
  - Always go through the gdrive branch; remove the `if args.gdrive or args.gdrive_bar_action: ... else: ...` split. Either of the `args.gdrive` / `args.gdrive_bar_action` is required.
  - In each gdrive branch, drop the `args.start_state = spec["state_json"]` shim line (L1076 and L1085) — `args.start_state` no longer exists. Drop the `diagnose_mode` block if diag is removed; otherwise leave but only run on gdrive inputs.
  - Drop the legacy `else: build_real_design_goal_spec(...)` branch (L1087–L1096) entirely.
  - Delete the `start_context = derive_start_pose_from_home_left_tool(...)` + `validate_auto_home_start_context_with_temporary_scene(...)` + `scene_spec = build_scene_spec_from_start_context(...)` block (L1118–L1133). The spec built by `build_gdrive_*_target_spec` (which itself wraps `build_gdrive_*_scene_spec`) already carries everything `setup_planning_scene` needs; pass that `scene_spec` directly to the stage runner.
  - In the `stage_runner(...)` call (L1139–L1156), drop `grasp_json=spec["grasp_json"]`, `start_state_json=args.start_state`, `end_state_json=spec["state_json"]`. Pass `scene_spec=<the scene_spec built above>`. Drop `swap_grasps` if §2.1 removes the param.
  - Update the `build_replay_command(...)` call site (L1175–L1181) per the new signature.
  - In the payload write block (L1226+), the `common_start_pose` section references `common_start["mobile_base_from_tool0_left_home"]` and `args.home_left_tool_offset`. These still work because `compute_gdrive_common_start` returns that key. The `args.home_left_tool_offset` arg comes from `--home-left-tool-offset`, which stays.

- `build_gdrive_target_spec` / `build_gdrive_bar_action_target_spec` (around L83 and L126): both currently return a "spec" dict that mixes legacy shape (`target_name`, `grasp_json`, `state_json`, `robot_state`) with new keys. After the legacy callers go away, the only consumer is `summarize_result` (reads `target_name`, `active_bar_mesh`, `goal_pose`) and `save_replay_bundle` (reads `grasp_targets`, `active_bar_mesh`, `built_bars`). Simplify the return dicts to only those keys plus `scene_spec` (the underlying gdrive scene_spec). The fields `grasp_json`, `state_json`, `robot_state` exist only as legacy shims and can be dropped — verify each by grepping callers within the file.

### 2.3 `debug_runner.py`

Delete:

- Import `build_default_paths` (L19).
- In `parse_args` (around L1240): `default_grasp_json, default_start_state, default_end_state = build_default_paths()` (L1241) and the three `--grasp-json` / `--start-state` / `--end-state` `add_argument` calls (L1243–L1245).

Edit:

- Decision: `debug_runner.py` becomes gdrive-only too (see §4). Add a CLI surface mirroring `minimal_rrt.main()`: `--gdrive-state`, `--gdrive-bar-action`, `--movement`, `--gdrive-problem`, `--gdrive-no-env`, `--gdrive-no-active-extras`. Validate mutual exclusion and presence in `parse_args`.
- At the top of `main()` (around L1291), before any analysis branching, build `gdrive_scene_spec` exactly the way `minimal_rrt.main()` does (around L2787–L2812). Stash it on `args` (e.g. `args._scene_spec = gdrive_scene_spec`) so the four `run_stage_trial(...)` call sites can pick it up without restructuring.
- In all four `run_stage_trial(...)` call sites (around L588–L631 in `run_stage_analysis` and L777–L820 in `run_stage_summary_only`, plus the one-shot at L1303–L1322), drop `grasp_json=args.grasp_json`, `start_state_json=args.start_state`, `end_state_json=args.end_state` and add `scene_spec=args._scene_spec`. Note: the same `scene_spec` dict is reused across trials by `setup_planning_scene` (which copies it internally via `dict(scene_spec)`), so this is safe.

### 2.4 `trajectory_replay.py`

Delete:

- Import `build_default_paths` (L17).
- In `parse_args` (around L90): `default_grasp_json, default_start_state, default_end_state = build_default_paths()` (L91) and the three legacy `add_argument` calls (L95–L97).

Edit:

- Make `--metadata-json` required (decision in §4). Update `--metadata-json` `add_argument` accordingly (`required=True`).
- `metadata_to_scene_spec(metadata)` already builds a full `scene_spec` from the metadata JSON's `"scene_spec"` block. Raise a clear error if `metadata.get("scene_spec")` is missing rather than silently returning `None`.
- In `main()` (around L101), drop the three positional args to `setup_planning_scene`. New call:
  ```
  scene = setup_planning_scene(
      scene_spec=metadata_to_scene_spec(metadata),
      use_gui=True,
  )
  ```
- Verify that every metadata JSON emitted by `save_replay_bundle` (in `real_state_study.py` L209–L225) already populates the required `scene_spec` keys (`grasp_targets`, `start_joint_values`, `end_joint_values`, `active_bar_mesh`, `built_bars`, etc.). It does — `save_replay_bundle` already writes the gdrive shape.

### 2.5 `README.md` (`external/husky_assembly_tamp/README.md`)

- Drop the `--grasp-json`, `--start-state`, `--end-state` rows from the Key flags table (L43–L45).
- Replace the "Adapting to a different robot cell / design study" section (L86–L147), specifically:
  - Subsection **1. Default data paths in minimal_rrt.py** (L90–L105): remove (no longer applies).
  - Subsection **4. Per-target file naming convention** (L129–L141): replace with a pointer to gdrive convention (`GDRIVE_DATA_DIRECTORY` and `build_gdrive_*_scene_spec`).
  - Subsections 2, 3, 5: leave or minor refresh; they reference `DESIGN_STUDY_BAR_SEQUENCE` / `default_design_root` / `MOBILE_BASE_FROM_TOOL0_LEFT_HOME`. `DESIGN_STUDY_BAR_SEQUENCE` is going away — drop subsection 3's reference to it and update `DEFAULT_TARGET_NAMES` guidance.
- Update the `real_state_study.py` example commands to use `--gdrive --targets B3_approach.json` or `--gdrive-bar-action --targets B1.json` instead of bare target names.

## 3. New `scene_spec` contract for `setup_planning_scene`

After this refactor, `setup_planning_scene(scene_spec=..., use_gui=False)` requires `scene_spec` to be a dict that contains, at minimum:

**Required keys** (will raise `ValueError` if missing):

- `grasp_targets`: non-empty sequence of `(world_from_bar_pose, world_from_tool0_pose)` tuples in mobile-base frame. Length 1 yields single-arm planning; length ≥ 2 yields dual-arm.
- `start_joint_values`: 12-element array of joint values (left-arm 6 + right-arm 6) matching `HUSKY_DUAL_ARM_JOINT_NAMES`.
- `end_joint_values`: same shape.

**Optional keys** (have safe defaults):

- `active_bar_mesh`: `BarMeshSpec` dict (`vertices`, `faces`, `aabb_dims`, `name`/`body_name`). If absent, falls back to `BAR_BOX_DIMS` cube. Always populated by `build_gdrive_*_scene_spec`.
- `built_bars`: list of `{name, mesh, pose, collision, color}` dicts for static obstacle bars. Defaults to `[]`.
- `world_from_bar_start`, `world_from_bar_goal`: explicit start/goal bar poses. If absent, derived from `grasp_targets` and `mobile_base_from_tool0_left_home`.
- `mobile_base_from_tool0_left_home`: left-tool home pose. Defaults to `MOBILE_BASE_FROM_TOOL0_LEFT_HOME`.

All gdrive scene_spec builders (`build_gdrive_scene_spec`, `build_gdrive_bar_action_scene_spec`) populate the required keys plus `active_bar_mesh`, `built_bars`, `world_from_bar_start`, `world_from_bar_goal`. The implementer can lift the explicit required-key check to fail fast at the top of `setup_planning_scene` for clearer error messages.

## 4. Open-question resolutions

### (a) `debug_runner.py` standalone usage

**Decision**: gain a `--gdrive-state` / `--gdrive-bar-action` (+ `--movement`, `--gdrive-problem`, `--gdrive-no-env`, `--gdrive-no-active-extras`) CLI surface mirroring `minimal_rrt.main()`. Build the `scene_spec` once at the top of `main()` and pass it into every `run_stage_trial(...)` call.

Rationale: cheapest viable option. The four call sites already share a uniform shape; threading one extra kwarg through them is mechanical. `run_stage_analysis` and `run_stage_summary_only` already accept `args`, so the spec lives on `args` (or a local) without changing their signatures.

Alternative considered: delete `debug_runner.py` entirely. Rejected because the report/comparison flows still have value, and several `tasks/*.md` entries reference it.

### (b) `trajectory_replay.py` metadata source

**Decision**: make `--metadata-json` **required**.

Rationale: every emitter (`save_replay_bundle` in `real_state_study.py`, plus any future ones) writes a metadata JSON alongside the trajectory. No production caller currently invokes the replay without metadata. Failing fast is simpler than adding a second gdrive-from-CLI path inside the replay tool.

If a future caller needs a metadata-less replay, add an opt-in `--gdrive-state` / `--gdrive-bar-action` later — same pattern as in `debug_runner.py`.

## 5. Verification commands

Run inside the project venv. **`cd` to ros2_ws first** (the venv is at `ros2_ws/venv`).

```
cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate

# 1. Syntax check on each touched file.
python -m py_compile \
  src/husky-assembly-teleop/external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/stage1/minimal_rrt.py \
  src/husky-assembly-teleop/external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/stage1/real_state_study.py \
  src/husky-assembly-teleop/external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/stage1/debug_runner.py \
  src/husky-assembly-teleop/external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/stage1/trajectory_replay.py

# 2. Import smoke (catches missing-symbol fallout).
python -c "from husky_assembly_tamp.motion_planner.stage1 import minimal_rrt, real_state_study, debug_runner, trajectory_replay; print('ok')"

# 3. Headless gdrive bar-action run via minimal_rrt (the user-supplied happy path).
python -m husky_assembly_tamp.motion_planner.stage1.minimal_rrt --gdrive-bar-action B1.json --no-gui --stage 3 --max-attempts 1 --max-time 10

# 4. Headless real_state_study on the same input.
python -m husky_assembly_tamp.motion_planner.stage1.real_state_study --gdrive-bar-action --targets B1.json --stage 3 --max-attempts 1 --max-time 10

# 5. debug_runner one-shot.
python -m husky_assembly_tamp.motion_planner.stage1.debug_runner --gdrive-bar-action B1.json --stage 3 --max-attempts 1 --max-time 10

# 6. Confirm the legacy CLI args are gone (each grep should print nothing).
grep -nR "\-\-grasp-json\|\-\-start-state\|\-\-end-state\|build_default_paths\|build_real_design_goal_spec\|load_grasp_targets\|load_robot_cell_state" \
  src/husky-assembly-teleop/external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/stage1/ || true
```

Step 6 should produce zero hits inside `stage1/` (matches in `zh_archive/`, `model/target_parse.py` are out of scope and expected to remain). If step 3 or 4 fails with a missing-key error inside `setup_planning_scene`, inspect what the gdrive builder produced — that is the bug the new required-key check is designed to surface.

## 6. Risks & non-obvious details

- **`setup_planning_scene` signature change is breaking**: every caller across the four files must update positional-and-kwarg passing in lockstep. Grep for `setup_planning_scene(` after the edit lands to catch stragglers. The same applies to `run_stage_trial(`.
- **`swap_grasps` drop is optional**: leaving it as a no-op parameter is safe; removing it tightens the API. Pick one — do not leave a half-removed surface.
- **`build_gdrive_*_target_spec` simplification (§2.2)**: the dicts they return are consumed by both `summarize_result` and `save_replay_bundle`. If you simplify, double-check both callers. `save_replay_bundle` reads `spec["grasp_targets"]`, `spec["active_bar_mesh"]`, `spec["built_bars"]` — all populated by the gdrive scene_spec.
- **README is not load-bearing for tests but is the public face**: keep updates aligned with the actual CLI; stale README is a worse failure mode than no README.
- **`tasks/cc_lessons.md` rule (from project CLAUDE.md)**: if the user corrects anything during implementation, append the lesson to `tasks/cc_lessons.md`. Do *not* preemptively add entries.
- **`trajectory_replay --metadata-json` becoming required** breaks any caller that previously omitted it. Grep for `trajectory_replay` invocations in `tasks/`, `scripts/`, and `husky_assembly_teleop/` before flipping the flag — `real_state_study.build_replay_command` already includes it, so should be safe.
- **Endpoint-IK diagnosis path is being amputated** (recommendation in §2.2). If the user needs it on gdrive inputs, surface this in the implementer's PR/notes; do not silently delete a feature the user might still want.

## Deviations from the user's request

- The user listed `compute_common_start_context` / `derive_start_pose_from_home_left_tool` / `validate_auto_home_start_context*` / `run_endpoint_ik_diagnosis` / `build_scene_spec_from_start_context` as "audit each: delete if only the legacy branch uses it." Verification shows all of them are reachable only via the legacy branch and the `--diagnose-endpoint-ik` flow — both being retired. Spec recommends **deleting all of them plus the `--diagnose-endpoint-ik` / `--diagnose-start-collision` args**. If the user actually wants to keep endpoint-IK diagnosis, this needs a follow-up to port it onto gdrive inputs (out of scope here).
- The user asked whether `DESIGN_STUDY_BAR_*`, `load_design_study_bar_mesh`, `design_study_active_bar_body_name`, `DATA_DIR` should be removed. Verified: `DATA_DIR` stays (URDF/SRDF paths still use it); the rest are removable.
- The user asked about README updates "decide whether to update." Spec proposes a targeted README refresh in §2.5; the implementer can defer this if time-boxed.
