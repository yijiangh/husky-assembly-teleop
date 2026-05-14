# 2026-05-13 — Dual-arm constrained trajectory: export/parse buttons

## Goal
Add two husky_monitor.py buttons to (1) export the constrained dual-arm
trajectory produced by "Plan & Stage Constrained" to a compas
`JointTrajectory` JSON file, and (2) parse such a file back into the
monitor's `constrained_trajectory` slots.

## Files
- `husky_assembly_teleop/husky_monitor.py`

## UI changes
Buttons added inside the `BOARD_VALIDATION` block, right after
`Plan & Stage Constrained`:

- `Export Constrained Dual-Arm Traj (compas JSON)` →
  `self.export_constrained_dual_arm_trajectory()`
- `Parse Constrained Dual-Arm Traj (compas JSON)` →
  `self.parse_constrained_dual_arm_trajectory()`

The parse side reuses the existing `Joint Trajectory` slider /
`available_joint_trajectories` list. After exporting, the list is
refreshed so the parse slider can immediately pick up the new file.

## File format
- Single 12-DOF `compas_fab.robots.JointTrajectory` per file
  (left arm joint names then right arm joint names, from
  `HUSKY_DUAL_UR5e_JOINT_NAMES`).
- Each `JointTrajectoryPoint`: `joint_values` (concat L+R),
  `joint_types = [Joint.REVOLUTE]*12`, `joint_names`, and
  `time_from_start` ramped linearly from 0 → `monitor.trajectory_time`.
- `start_configuration` is a plain `compas_robots.Configuration`
  (not a `JointTrajectoryPoint`) — round-trip through
  `Data.to_json/from_json` requires this because
  `Configuration.__from_data__` rejects `velocities/accelerations/effort/
  time_from_start` keys.
- Serialized with `Data.to_json(path, pretty=True)`; loaded with
  `JointTrajectory.from_json(path)`.

## File path
`<DESIGN_DATA_DIRECTORY>/<VALIDATION_PROBLEM_NAME>/Trajectories/<name>.json`

Default filename when current BarAction + movement available:
`{action_id}_{movement_id}_constrained_dual_arm_JointTrajectory.json`,
otherwise timestamped fallback. Helper `_trajectories_dir()` ensures
directory exists.

## Parse side effects
- Splits 12 joint values into left/right by joint-name lookup (per-point
  if joint_names present, else trajectory-level).
- Sets `self.constrained_trajectory = [(left_arr, None, total_time, None),
  (right_arr, None, total_time, None)]` (numpy arrays, matches what
  `husky_world.plan_and_stage_constrained` writes).
- Updates `constrained_start_conf`, `constrained_goal_conf`,
  per-arm display via `set_arm_trajectory`, sets display mode = 1,
  refreshes displayed trajectory and (if cfab + movement_start_state
  available) rebuilds the scrub sliders.

## Verification
- `python3 -m colcon build --symlink-install --packages-select
  husky_assembly_teleop` → clean.
- Standalone roundtrip script (build a fake 7-waypoint dual-arm
  trajectory, write via `to_json(pretty=True)`, read via
  `from_json`, split back into L/R np arrays): left/right values match,
  total time preserved.

## Notes / gotchas
- `JointTrajectoryPoint` requires explicit `joint_types` (else
  `compas_robots.Configuration.__init__` raises). Use
  `Joint.REVOLUTE` from `compas_robots.model` (not `compas_robots`).
- Cannot pass a `JointTrajectoryPoint` as `start_configuration` because
  re-instantiation goes through `Configuration(**data)` which doesn't
  accept the extra trajectory-point fields.
- The existing `Load Joint Trajectory` button parses raw JSON
  (`data['data']['points']`) instead of using `JointTrajectory.from_json`.
  We don't change it — the new Parse button is the compas-Data path
  for the constrained dual-arm flow.
