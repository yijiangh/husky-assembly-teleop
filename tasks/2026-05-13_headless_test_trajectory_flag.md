# 2026-05-13 — `--trajectory` flag for headless_live_monitor_test

## Goal
Reuse the existing `headless_live_monitor_test.py` BarAction scene
reconstruction + GUI scrubber to inspect a previously-exported
constrained dual-arm trajectory, without re-running the planner.

## File
`scripts/headless_live_monitor_test.py`

## Usage
```bash
cd /home/yijiangh/Code/ros2_ws
source venv/bin/activate
source install/setup.bash
python src/husky-assembly-teleop/scripts/headless_live_monitor_test.py \
    --bar-action B7.json --movement M1 \
    --trajectory B7_A0_assemble_B7_M1_CDFM_home_to_approach_constrained_dual_arm_JointTrajectory.json \
    [--gui]
```
- `--trajectory` accepts an absolute path or a bare filename (resolved
  under `<DESIGN_DATA_DIRECTORY>/<problem>/Trajectories/`).
- `--validate` is auto-enabled; PNG + markdown report land under
  `scripts/../reports/bar_action_validation/`.
- `--gui` opens the cfab PyBullet window with the scrub sliders.

## Behavior
- When `--trajectory` is set:
  - planning is skipped entirely (parallel to `--replay`),
  - `_load_compas_trajectory` parses the compas `JointTrajectory` JSON,
    splits 12 joint values into left/right per waypoint, and writes
    `monitor.constrained_trajectory` in the same shape the planner
    would produce.
  - `_bar_action_plan_ctx` is reconstructed (stage from
    `monitor.constrained_planner_stage`; `grasp_bar_from_left =
    invert(active_bar.attachment_frame)`; `grasp_bar_from_right` from
    FK at the goal waypoint; `obstacles_for_constrained` = all rigid
    bodies except the active bar and the EE ghost spheres; `path_poses`
    pre-computed by per-waypoint FK so the validator has a non-empty
    pose path).
- `--save-plan` and `--show-collision-setup` are gated off in trajectory
  mode (they need a fresh planner context).
- GUI scrubber works the same as in planning / replay mode (uses
  `monitor.constrained_trajectory`).

## Verification
- Syntax check: `python -c "import ast; ast.parse(open(...))"` clean.
- End-to-end against a real exported trajectory:
  `--bar-action B7.json --movement M1 --trajectory <B7 export>` →
  127-waypoint compas JointTrajectory loaded, validation ran, PNG +
  markdown report written. Validator surfaced 81 `robot_static`
  collisions starting at waypoint 54 — confirms the validator is
  actually replaying the trajectory through the BarAction scene's
  static obstacles, not just rubber-stamping.

## Notes
- The scene reconstruction relies on the existing
  `monitor.load_bar_action(...)` + cfab→pp bridge in `main()`, so
  `monitor._bar_action_husky`, `monitor.active_bar_body`,
  `monitor.movement_start_state` are populated by the time we load
  the trajectory.
- `--trajectory` and `--replay` both bypass planning. Trajectory mode
  is for compas `JointTrajectory` exports (the husky monitor's
  Export button); replay mode is for the legacy
  `husky_bar_action_plan/v1` JSON dumped by `--save-plan`.
- Assumes the active bar is grasped by the left tool0 (matches
  `validate_exported_trajectory.py` and current dataset).
