# Rename `stage1/` → `dual_arm_task_space_rrt/`, split planner core/smooth, fold `real_state_study` into `run.py`

**Status**: spec approved 2026-05-14
**Plan source**: `/home/yijiangh/.claude/plans/concurrent-imagining-moler.md`

## Context

`external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/stage1/minimal_rrt.py` is ~2.7k LOC and mixes (a) core RRT planning, (b) path smoothing, (c) scene/BarAction loading, (d) stage runners, and (e) CLI. The sibling `real_state_study.py` benchmarks the same stage runners across a list of targets and writes report/video/trajectory artifacts; `debug_runner.py` is a third batch driver. The directory name "stage1" is misleading — the same module handles stages 1/2/3.

Goal: rename the module to its actual purpose (`dual_arm_task_space_rrt`), split the planner internals into `core.py` (RRT) and `smooth.py` (shortcut smoothing), fold the post-plan output helpers from `real_state_study.py` into a single `run.py`. `run.py` keeps the simpler-than-headless-monitor debug-visualization role. `debug_runner.py` and `real_state_study.py` are deleted. Upstream callers (`husky_world.py`, `scripts/headless_live_monitor_test.py`, `scripts/smoke_constrained_api.py`) must continue to work.

## Difference between `minimal_rrt.py main()` and `real_state_study.py main()`

| Aspect | `minimal_rrt.py` | `real_state_study.py` |
| --- | --- | --- |
| Mode | single planning run | batch over `--targets`, one report row per target |
| Stage entry | `run_stage_trial(stage=...)` | `run_stage{1,2,3}_trial` (thin wrappers around `run_stage_trial(stage=N)`) |
| GUI default | **on** (use `--no-gui`) | **off** (use `--gui`) |
| Input flags | `--gdrive-state` XOR `--gdrive-bar-action` (single file) | `--gdrive` XOR `--gdrive-bar-action` + `--targets <files>` list |
| Planner knobs only here | `--goal-bias`, `--dist-metric`, `--floating-collision`, `--draw-rrt-tree`, `--no-lock-renderer-during-search`, `--smoothing/--no-smoothing`, `--smooth-iterations`, `--smooth-max-time`, `--smooth-min-improvement`, `--use-angle-normalization` | — |
| Scene/eval knobs only here | — | `--include-built-bars`, `--enable-built-bar-collision`, `--video-frame-step`, `--video-frame-sleep` (also vestigial: `--home-left-tool-offset`, `--home-left-tool-local-yaw`, `--auto-home-pose`, `--auto-home-ik-candidates`) |
| Output | terminal log; GUI viewer loop if GUI on | per-target validation plot, trajectory JSON + metadata JSON, MP4 video (when not GUI), batch markdown report + JSON support file |
| Post-plan view | `run_visualization_loop` if GUI | `run_visualization_loop` if GUI **and** `--visualize-path` |

Both call the same `run_stage_trial`, both use `build_gdrive_scene_spec` / `build_gdrive_bar_action_scene_spec`, both default to the same `--gdrive-problem`.

## Decisions

- **Type aliases**: drop `ArmConf` (zero references). Keep `GraspTarget` and `FullConf` (both heavily used in type hints).
- **Home-pose helpers vs `derive_constrained_start`**:
  - `derive_constrained_start` (line 750): IK-validated, stages 2/3, public API. Keep.
  - `bar_orientation_from_grasps` (line 263): private helper of `derive_constrained_start`. Keep.
  - `auto_compute_home_bar_pose` (line 295): private helper of `derive_constrained_start`. Keep.
  - `derive_home_start_poses_from_grasps` (line 241): no-IK helper used by `setup_planning_scene`:1874 for stage-1 visual debug. Keep.
- **Stage runner wrappers**: delete `run_stage1_trial`, `run_stage2_trial`, `run_stage3_trial`. Callers go through `run_stage_trial(stage=args.stage, ...)`.
- **CLI flags dropped (vestigial, never plumbed into `run_stage_trial`)**: `--home-left-tool-offset`, `--home-left-tool-local-yaw`, `--auto-home-pose`, `--auto-home-ik-candidates`.
- **Start/end pose diagnosis MUST be preserved**: `pp.draw_pose(world_from_bar_start/world_from_bar_goal)`, `pp.add_text("Start"/"Goal", ...)`, `add_grasp_pose_markers`, ghost_start/ghost_goal bodies, `log_validation_summary`, `validate_stage_trajectory` 6-panel plot, `run_visualization_loop` slider.

## Target layout

```
external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/
├── api.py                                              # unchanged location; imports updated
├── dual_arm_task_space_rrt/                            # renamed from stage1/
│   ├── __init__.py
│   ├── core.py                                         # NEW: RRT + IK + helpers
│   ├── smooth.py                                       # NEW: smoothing
│   ├── run.py                                          # was minimal_rrt.py; scene + BarAction + merged batch CLI
│   ├── path_validation.py                              # moved as-is
│   ├── trajectory_io.py                                # moved as-is
│   ├── trajectory_replay.py                            # moved; imports updated
│   ├── debug_goals.md                                  # moved as-is
│   ├── reports/                                        # moved as-is
│   └── temp/                                           # moved as-is
└── drake_dual_arm_planner/                             # untouched
```

Deleted: `stage1/real_state_study.py`, `stage1/debug_runner.py`, `stage1/` directory.

## Function split

### → `dual_arm_task_space_rrt/core.py`

- Type aliases / planner constants: `PoseLike`, `GraspTarget`, `FullConf`, `BAR_RADIUS`, `BAR_LENGTH`, `BAR_BOX_DIMS`, `DEFAULT_JOINT_CONTINUITY_THRESHOLD_RAD`, `DEFAULT_USE_ANGLE_NORMALIZATION`, `STAGE3_GRASP_MASK_LINKS`, `TOOL_LINK_LEFT`, `TOOL_LINK_RIGHT`.
- Distance / sampling / tree: `pose_to_feature_vec`, `pose_distance`, `_pose_path_cost`, `_pose_path_inflection_indices`, `get_bar_feature_points`, `sample_pose`, `nearest_node`, `export_tree`, `goal_pose_reached`.
- Collision: `get_pose_collision_fn`, `get_joint_collision_fn`, `get_disabled_collisions_from_link_names`, `validate_dual_arm_bar_pose`.
- IK + joint: `solve_single_arm_ik`, `solve_dual_arm_pose_ik`, `solve_endpoint_dual_arm_ik`, `maybe_normalize_angles`, `joint_step_exceeds_threshold`, `summarize_joint_continuity`, `reconstruct_joint_path_for_pose_path`.
- Home / start derivation: `bar_orientation_from_grasps`, `auto_compute_home_bar_pose`, `_grid_in_box`, `derive_constrained_start`.
- RRT loop: `extend_toward`, `update_debug_tree`, `plan_pose_rrt`.

### → `dual_arm_task_space_rrt/smooth.py`

- `smooth_dual_arm_pose_path`
- imports from `.core`: `_pose_path_cost`, `_pose_path_inflection_indices`, `reconstruct_joint_path_for_pose_path`, `solve_dual_arm_pose_ik`, `joint_step_exceeds_threshold`.

### → `dual_arm_task_space_rrt/run.py`

- URDF / joint-name / pose constants: `HUSKY_DUAL_URDF_PATH`, `HUSKY_DUAL_SRDF_PATH`, `HUSKY_DUAL_ARM_JOINT_NAMES`, `INIT_ARM_JOINT_ANGLES`, `STAGE1_DEBUG_START_OFFSET`, `MOBILE_BASE_FROM_TOOL0_LEFT_HOME`, `MOBILE_BASE_FROM_BAR_HOME_POSITION`, `DEFAULT_HOME_LEFT_TOOL_Z_OFFSET`, `GDRIVE_DATA_DIRECTORY`, `GDRIVE_DEFAULT_PROBLEM`.
- Mesh / scene helpers: `_normalize_vector`, `frame_data_to_pose`, `suppress_native_output`, `triangulate_faces`, `compas_mesh_data_to_pybullet_mesh`, `mesh_vertices_aabb_dims`, `create_bar_mesh_body`, `get_goal_pose_from_grasp_targets`, `compas_frame_to_pose`, `load_gdrive_active_bar_mesh`, `_fk_dual_arm_grasps_in_mb_frame`, `_resolve_gdrive_state_path`, `_resolve_gdrive_bar_action_path`, `_joint_values_from_robot_cell_state`, `_joint_values_from_configuration`, `build_gdrive_bar_action_scene_spec`, `build_gdrive_scene_spec`, `import_static_bar_bodies`, `create_visual_ee_marker`, `add_grasp_pose_markers`, `derive_home_start_poses_from_grasps`, `setup_planning_scene`, `teardown_planning_scene`.
- Stage runner: `run_stage_trial` (the three wrappers are deleted), `log_validation_summary`, `run_visualization_loop`.
- Folded from `real_state_study.py`: `pose_to_json`, `to_jsonable`, `reports_dir`, `support_dir`, `save_replay_bundle`, `build_replay_command`, `record_trajectory_video`, `summarize_result`, `write_report`.

## CLI design

```
python -m husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.run \
    --gdrive-bar-action --targets B1.json B2.json B3.json \
    --movement M1 --stage 3 [--no-gui]
```

Flag set:

- Source: `--gdrive-bar-action` XOR `--gdrive-state`; `--gdrive-problem`, `--movement`, `--gdrive-no-env`, `--gdrive-no-active-extras`.
- Targets: `--targets <file [file ...]>` (accepts bare basenames or `.json`). Default = `[B1.json]` (BarAction) or `[B3_approach.json]` (state).
- Stage / planner: `--stage`, `--goal-bias`, `--dist-metric`, `--position-res`, `--rotation-res`, `--max-time`, `--max-iterations`, `--max-attempts`, `--endpoint-ik-attempts`, `--random-seed`, `--smoothing/--no-smoothing`, `--smooth-iterations`, `--smooth-max-time`, `--smooth-min-improvement`, `--joint-continuity-threshold`, `--use-angle-normalization`, `--floating-collision`, `--draw-rrt-tree/--no-draw-rrt-tree`, `--no-lock-renderer-during-search`.
- Scene: `--include-built-bars`, `--enable-built-bar-collision`.
- GUI / output: `--gui` (default OFF), `--visualize-path/--no-visualize-path` (default True when GUI on), `--video-frame-step`, `--video-frame-sleep`. Report+video+trajectory bundle generation runs automatically for every target.

## Import-site updates

1. `external/.../motion_planner/api.py:155` lazy import: `from husky_assembly_tamp.motion_planner.stage1.minimal_rrt import derive_constrained_start as _derive_constrained_start` → `from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core import derive_constrained_start as _derive_constrained_start`.
2. `external/.../motion_planner/api.py:205` lazy import: split into
   ```python
   from .dual_arm_task_space_rrt.core import (
       DEFAULT_JOINT_CONTINUITY_THRESHOLD_RAD, plan_pose_rrt, get_joint_collision_fn,
   )
   from .dual_arm_task_space_rrt.smooth import smooth_dual_arm_pose_path
   ```
3. `dual_arm_task_space_rrt/run.py` top imports: `from .path_validation import validate_stage_trajectory`; `from .core import (...)`; `from .smooth import smooth_dual_arm_pose_path`. Late import at old line ~2019 (`from husky_assembly_tamp.motion_planner.api import derive_grasps_from_state`) unchanged.
4. `dual_arm_task_space_rrt/trajectory_replay.py`: `from .run import ...` and `from .trajectory_io import load_joint_trajectory_as_path`.
5. `husky_assembly_teleop/husky_world.py:1725`:
   ```python
   from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core import (
       derive_constrained_start,
       get_bar_feature_points,
   )
   ```
6. `scripts/smoke_constrained_api.py:31`: two imports — `HUSKY_DUAL_ARM_JOINT_NAMES` from `.run`; `TOOL_LINK_LEFT`, `TOOL_LINK_RIGHT`, `derive_constrained_start`, `get_bar_feature_points` from `.core`.
7. `scripts/headless_live_monitor_test.py` 5 sites:
   | Line | Old import | New |
   | --- | --- | --- |
   | 251 | `stage1.minimal_rrt` for `HUSKY_DUAL_ARM_JOINT_NAMES` | `.run` |
   | 455 | `stage1.path_validation` for `get_disabled_collisions_from_link_names` | `.path_validation` |
   | 458 | `stage1.minimal_rrt` for `HUSKY_DUAL_URDF_PATH`, `HUSKY_DUAL_SRDF_PATH`, `STAGE3_GRASP_MASK_LINKS` | URDF/SRDF from `.run`; `STAGE3_GRASP_MASK_LINKS` from `.core` |
   | 554 | `stage1.path_validation` for `validate_stage_trajectory` | `.path_validation` |
   | 557 | `stage1.minimal_rrt` for `HUSKY_DUAL_URDF_PATH`, `HUSKY_DUAL_SRDF_PATH`, `STAGE3_GRASP_MASK_LINKS`, `DEFAULT_USE_ANGLE_NORMALIZATION`, `log_validation_summary` | URDF/SRDF/`log_validation_summary` from `.run`; `STAGE3_GRASP_MASK_LINKS`/`DEFAULT_USE_ANGLE_NORMALIZATION` from `.core` |
8. Replay-command emitted by `build_replay_command`: `husky_assembly_tamp.motion_planner.stage1.trajectory_replay` → `husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.trajectory_replay`.

Docs touched only opportunistically: `external/.../README.md`, `external/.../docs/plan_code_simplification.md`, `external/.../memory.md`, `tasks/2026-05-14_retire_legacy_json_path.md`, `tasks/cc_lessons.md`.

## Mechanics

1. `git mv stage1 dual_arm_task_space_rrt` (preserve history).
2. `git mv dual_arm_task_space_rrt/minimal_rrt.py dual_arm_task_space_rrt/run.py`.
3. `git rm dual_arm_task_space_rrt/debug_runner.py` and `git rm dual_arm_task_space_rrt/real_state_study.py`.
4. Carve `core.py` and `smooth.py` out of `run.py` by **moving** (cut, not copy) the bodies listed above. Preserve function order, comments, indentation. Delete `ArmConf = np.ndarray`. Add `from .core import ...` / `from .smooth import smooth_dual_arm_pose_path` at the top of `run.py`.
5. In `run.py`: delete `run_stage1_trial`, `run_stage2_trial`, `run_stage3_trial`.
6. In `run.py`, append the kept helpers from `real_state_study.py`.
7. Rewrite `main()` to loop over `--targets`, call `run_stage_trial(stage=args.stage, scene_spec=...)`, write per-target artifacts, then write the batch markdown report + JSON support file.
8. Update all import sites.
9. Run verification.

## Verification

From `/home/yijiangh/Code/ros2_ws`:

```bash
source venv/bin/activate

# 1. Import surface + CLI help
python -c "
from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core import (
    plan_pose_rrt, derive_constrained_start, get_joint_collision_fn,
    bar_orientation_from_grasps, auto_compute_home_bar_pose,
    DEFAULT_JOINT_CONTINUITY_THRESHOLD_RAD, STAGE3_GRASP_MASK_LINKS,
    TOOL_LINK_LEFT, TOOL_LINK_RIGHT,
)
from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.smooth import smooth_dual_arm_pose_path
from husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.run import (
    run_stage_trial, build_gdrive_bar_action_scene_spec, build_gdrive_scene_spec,
    HUSKY_DUAL_ARM_JOINT_NAMES, HUSKY_DUAL_URDF_PATH, HUSKY_DUAL_SRDF_PATH,
    setup_planning_scene, teardown_planning_scene,
)
from husky_assembly_tamp.motion_planner.api import (
    plan_constrained_dual_arm, plan_free_dual_arm, derive_constrained_start,
)
print('imports ok')
"
python -m husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.run --help

# 2. Single-target headless smoke
python -m husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.run \
    --gdrive-bar-action --targets B1.json --movement M1 --stage 3 --max-attempts 1 --max-time 10

# 3. Multi-target batch
python -m husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.run \
    --gdrive-bar-action --targets B1.json B2.json --movement M1 --stage 3 --max-attempts 1 --max-time 10

# 4. Upstream callers
python scripts/smoke_constrained_api.py
python scripts/headless_live_monitor_test.py --help

# 5. Importability of teleop entry
python -c "import husky_assembly_teleop.husky_world; print('husky_world imports ok')"

# 6. Confirm no leftover references
grep -rn "stage1\." external/husky_assembly_tamp/ husky_assembly_teleop/ scripts/ --include="*.py" | grep -v __pycache__ | grep -v zh_archive
grep -rn "ArmConf\|run_stage1_trial\|run_stage2_trial\|run_stage3_trial\|home_left_tool_offset\|home_left_tool_local_yaw\|auto_home_pose\|auto_home_ik_candidates" external/husky_assembly_tamp/ husky_assembly_teleop/ scripts/ --include="*.py" | grep -v __pycache__ | grep -v zh_archive
```
