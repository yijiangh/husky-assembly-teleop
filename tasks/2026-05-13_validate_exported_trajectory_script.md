# 2026-05-13 — Standalone script to validate exported dual-arm trajectory

## Goal
Inspect a compas `JointTrajectory` JSON exported by the husky monitor
(`Export Constrained Dual-Arm Traj`) by replaying it through
`husky_assembly_tamp.motion_planner.stage1.path_validation.
validate_stage_trajectory` — the same validator
`run_stage_trial` calls after planning.

## File
`scripts/validate_exported_trajectory.py`

## Usage
```
cd /home/yijiangh/Code/ros2_ws
source venv/bin/activate
source install/setup.bash
python src/husky-assembly-teleop/scripts/validate_exported_trajectory.py \
    --bar-action B1.json \
    --movement 0 \
    --trajectory B1_A0_assemble_B1_M1_CDFM_home_to_approach_constrained_dual_arm_JointTrajectory.json
```

- `--bar-action`/`--trajectory` accept either an absolute path or a bare
  filename (resolved under `<problem>/BarActions/` /
  `<problem>/Trajectories/` of `DESIGN_DATA_DIRECTORY`).
- `--problem` defaults to `VALIDATION_PROBLEM_NAME`.
- `--movement` accepts an int index or a movement_id substring.
- `--no-gui` for headless.

## What it does
1. Parses the BarAction + selected `Movement.start_state`.
2. Loads the exported `JointTrajectory` via `JointTrajectory.from_json`
   and maps 12-DOF joint values into the planner's order
   (`HUSKY_DUAL_ARM_JOINT_NAMES`).
3. Builds a planner `scene_spec`:
   - grasp transform from `active_bar.attachment_frame`
     (= `left_tool0_from_bar`, inverted to feed the planner's
     `bar_from_tool0` convention),
   - active bar mesh + env bar meshes from
     `<problem>/RobotCell.json`, re-expressed in mobile-base frame
     from the start_state's `robot_base_frame`,
   - grasp_targets derived by FK at the trajectory's last waypoint.
4. Calls `setup_planning_scene` with that scene_spec (URDF/SRDF come
   from `minimal_rrt`). A serialized stub `start_state` file is also
   written to a tempdir because `setup_planning_scene` evaluates
   `load_robot_cell_state(start_state_json)` as the default arg of
   `dict.get`, so the file must exist even when scene_spec wins.
5. Runs `validate_stage_trajectory(stage=3, ..., bar_pose_source=
   "left_grasp")` — the bar pose is FK-derived per waypoint, so the
   `path` kwarg is a same-length list of identity placeholder poses.
6. Prints the standard validation summary (collision_free,
   joint_continuity_ok, relative_transform_ok), the trajectory PNG
   plot path (saved under
   `husky_assembly_tamp/motion_planner/stage1/reports/`), and (if GUI)
   enters `run_visualization_loop` for interactive playback.

## Smoke verification
Synthesized a fake `_smoke_constrained_dual_arm_JointTrajectory.json`
(5-waypoint linear interp between two 12-DOF configs) for `B1.json`,
ran with `--no-gui`. Validator executed end-to-end: collision check
(7 robot_static hits — expected for a hand-crafted bogus path),
joint continuity pass (0.05 rad max delta), relative transform drift
fail (expected, the interp doesn't preserve the bar-grasp constraint),
PNG report written. Confirms all three subsystems are exercised.

## Notes / gotchas
- `build_gdrive_scene_spec` (which the planner CLI uses) keys off
  `active_bar_*` / `env_*` rigid-body name prefixes. BarAction
  start_states use `bar_<id>` instead, so we mirror its logic locally
  instead of reusing it: active bar = `bar_{action.active_bar_id}`,
  env bars = all other `bar_*` entries with a non-None `frame`.
- `start_state.robot_configuration` in our BarAction snapshots is all
  zeros (placeholder — the planner runs IK to derive the goal_conf).
  We instead read goal_conf from the exported trajectory's last
  waypoint for the FK that builds `grasp_targets`.
- The validator's `path` argument is unused for actual bar pose when
  `bar_pose_source="left_grasp"`, but still must be non-empty and of
  the same length as `joint_path`.
- Validator's joint-continuity threshold defaults to 10° here (same
  as `minimal_rrt.DEFAULT_JOINT_CONTINUITY_THRESHOLD_RAD`).

## Caveats
- Assumes the active bar is grasped by the left tool0 (the only case
  observed in the current dataset). The script logs a warning if the
  attachment is to a different link; the FK math would need updating
  for a right-tool primary attachment.
- Tool bodies (`AssemblyLeftArmToolBody` / `AssemblyRightArmToolBody`)
  are not loaded as collision geometry because they are not part of
  the planner's URDF. Validation is wrist-link-based, matching the
  planner's own collision model.
