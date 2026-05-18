# Session learnings — BAR_HOLDING_ACCURACY_TEST + per-movement UI revamp

Date: 2026-05-18. Scope: bar reaching accuracy test, per-movement plan/load
workflow, mocap-axis convention, frame comparisons, calibration tooling.

## Architecture decisions

- **UI**: BAR_HOLDING_ACCURACY_TEST block replaced with explicit slider+button
  grid — `BarAction file` slider, `Load BarAction`, `Movement (idx; 0=M0_synth)`
  slider, `Load Movement`, `Plan Movement`, `Load Movement Trajectory`,
  `IK Live Base (debug)`, exec/record/save. Synthetic M0 (live → M1.start)
  prepended to the loaded movement list.
- **Trajectory storage**: `mv.trajectory` is a single 12-joint `compas_fab`
  `JointTrajectory` (left || right). Helpers in `utils.py`:
  `vec12_from_conf`, `conf_from_12vec`, `joint_trajectory_from_path`,
  `path_12_from_joint_trajectory`, `HUSKY_DUAL_ARM_HOME_CONF_12`.
- **Persistence**: saved to
  `<DESIGN_DATA_DIRECTORY>/<problem>/Trajectories/<movement_id>_trajectory.json`
  via `compas.data.json_dump`. `Load Movement Trajectory` reads it back.
- **Per-role planners**: M1 = `plan_and_stage_constrained` (RRT); M2 =
  new `plan_constrained_dual_arm_linear` (bar-frame interpolation +
  per-waypoint IK, inter-EE preserved); M3 = new
  `plan_dual_arm_linear_independent` (each arm interpolates independently,
  retreat); M4 = `plan_free_dual_arm` to fixed home; M0 = same, live →
  M1.start. Shared `_run_dual_arm_cartesian_ik_loop` + `_fk_link_frame`
  helpers in `external/husky_assembly_tamp/.../api.py`.
- **Attached-body ghost**: reuse cfab-spawned rigid bodies (same pp.CLIENT),
  recolor `TRAJECTORY_GREEN`, re-pose each tick via
  `goal_model.get_link_pose_from_name(attached_link) ⊕ attachment_frame`.

## compas_fab / compas_robots gotchas

- `JointTrajectory(trajectory_points=..., joint_names=...)` — kwarg is
  `trajectory_points`, NOT `points`. `.points` is the read-side attribute.
- `JointTrajectoryPoint(joint_values=..., joint_types=..., joint_names=...)`
  requires `joint_types` (e.g. `[Joint.REVOLUTE]*12`). `Joint` import is
  `from compas_robots.model import Joint`, not `from compas_robots`.
- `FrameInterpolator(start, goal, options)` options keys are
  `max_step_distance` (m) and `max_step_angle` (rad). NOT `position_step` /
  `rotation_step`.
- `check_collision` accepts hidden `_skip_cc1`..`_skip_cc5` flags:
  CC.1 = robot self, CC.2 = robot↔tool, CC.3 = robot↔body, CC.4 =
  attached-body↔body, CC.5 = tool↔body. Setting `_skip_cc3/4/5 = True`
  skips env-related checks while keeping self + tool checks. Forward
  same flags into IK via the same options dict.
- `inverse_kinematics(target, state, group, options)` defensively re-pushes
  `set_robot_cell_state(state)` internally, but call it once externally
  before the loop to be safe.

## UI / monitor gotchas

- `common.Slider.update()` is **not** auto-polled by `HuskyMonitor.update()`
  — each slider needs an explicit `.update()` call in the tick loop
  (pattern: `if hasattr(self, 'bar_movement_slider') and self.bar_movement_slider: self.bar_movement_slider.update()`).
- After `reset_ui()` rebuilds widgets, every `Slider(...)` is recreated
  with its `current_val` argument. Pass the saved `int(self._selected_*)`
  so the slider visual + the legacy-pybullet poll don't snap back to 0.
- Use `integer=True` for index sliders to get deterministic drags.
- `goal_base_pose_frozen` guard is required so `update()`'s live-mocap
  branch doesn't keep overwriting `goal_base_pose` away from the value
  IK Live Base / load_movement just set.
- Trajectory preview ghost should ride on the **live** husky base, not
  the cell-state base, so the preview matches what executing at the
  real-robot location would look like.

## Frame chain bugs / fixes (real bites this session)

- `('ur_arm_wrist_2_link')` is a parenthesized string, **not** a tuple.
  Must be `('ur_arm_wrist_2_link',)`. Cost: hours of debugging
  `link_from_name(robot, 'left_u')` failures.
- IK self-test FK MUST read from `monitor.movement_goal_state`
  (live_base + IK_conf), not the local `live_state` (which still has the
  OLD seed conf). Otherwise residual = pure base offset, masking a
  successful IK as a failure.
- `inverse_kinematics` writes the new conf onto a **copy** of the state
  you pass in. The original `live_state` is unchanged.
- Ghost render uses `(self.goal_base_pose, self.goal_arm_pose)` together;
  setting one without the other gives a tool0 that drifts by the offset.

## Mocap axis convention + calibration

- Rhino convention: `rhino_x = mocap_x`, `rhino_y = -mocap_z`,
  `rhino_z = mocap_y`. Legacy rotated convention: `(mocap_z, mocap_x, mocap_y)`.
  Switch via `HuskyMonitor.MOCAP_AXIS_CONVENTION`. Saved JSONs stamp the
  convention top-level so offline scripts pick the right corrector.
- Quat axis components transform like a 3-vector under change-of-basis.
- `convert_to_rhino.py` empirical fix: right-multiply
  `base_mocap_from_base_footprint.quat` by `q_z(-90°)`. The pure-math
  derivation says invariant; the empirical robot disagrees.

## Bar geometry / analysis

- **Auto-pair markers** by cross-bar distance (~82 mm ± 3 mm) using
  greedy match on `|d − nominal|`. Hardcoded `MARKER_NAME_PAIRS` gave
  7 mm straightness_max on a clean dataset; auto-pair gave 0.92 mm.
- `straightness_max` renamed to `center_to_line_dist_max_m` — max
  perpendicular distance from any pair midpoint to the fitted line.
- **Rhino RobotCell export bug**: `bar_rb.frame.point` = bar's LOWER tip
  (smallest world z), NOT midpoint. Comparison metric for now:
  `start_dev_m = ‖fitted_lower_tip − goal_pos‖`. `pos_dev(ocf↔goal)`
  retained as bug-compat reference.

## Outstanding mysteries

- **140 mm constant offset** between fitted bar and goal bar: most
  likely mocap-world ≠ design-world (different origins), not a per-bar
  installation error. Diagnostic: compare `mv.start_state.robot_base_frame.point`
  vs `monitor.huskies[0].interface.position`; if they differ by ~140 mm
  in Y, that's the origin mismatch propagating.

## Collaboration patterns

- Always diff agent edits against session-start `git status` before
  reverting — files marked `M` at bootstrap are user WIP, not scope
  creep.
- For non-trivial work, delegate to planner/implementer/reviewer
  subagents; always re-verify by reading the final file state and
  running real tests via `ros2_ws/venv`.
- Skill writeups belong in `tasks/<date>_<topic>.md`; corrections to
  past mistakes append to `tasks/cc_lessons.md`.
- User prefers minimal scope: don't introduce back-compat shims unless
  asked; don't expand into adjacent WIP.
