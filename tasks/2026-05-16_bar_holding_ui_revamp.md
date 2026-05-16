# Revamp BAR_HOLDING UI: per-movement load + plan dispatch + debug IK

## Context

The current BAR_HOLDING_ACCURACY_TEST UI exposes only "Load BarAction (selected M)" + "Replan Free (live base)". Two problems:
1. The "selected M" is a single global movement index; the user can't see / iterate through all movements in the BarAction.
2. The replan dispatches based on hardcoded paths (free vs constrained) instead of dispatching by movement type. There is no way to plan M3 (retreat) or M4 (free-to-home), and no synthetic M0 (live → M1.start staging) exists.

We replace the UI block with an explicit BarAction-file slider + Movement-index slider + load/plan/IK buttons. Planning dispatches per movement_id: M0 (synthetic, free), M1 (constrained free / RRT), M2 (constrained linear, bar held), M3 (linear retreat, independent EE), M4 (free-to-home). Trajectories are stored on `mv.trajectory` as a single compas_fab `JointTrajectory` (12 joints, left||right). All planners ignore environment collisions initially (CC.3 / CC.4 / CC.5 skipped) — only robot self-collision + tool-robot collision remain. Goal ghost on `Load Movement` reflects the cell-state's stored config + base.

## Files to modify

### A. `external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/api.py`

Add two new planners. Both share the same per-waypoint IK loop and skip env collisions.

**A1. `plan_constrained_dual_arm_linear`**
- Signature: `(planner, robot_cell, start_state, start_conf, goal_world_from_bar, bar_from_left_tool0, bar_from_right_tool0, *, num_steps=None, position_step=0.005, rotation_step=0.05, max_results=20, max_descend_iterations=200) -> JointTrajectory | None`
- Algorithm (mirrors `GH_dual_arm_approach_plan.py:43-200`):
  1. FK at `start_conf` → `start_world_from_left_tool0`, `start_world_from_right_tool0`.
  2. `start_world_from_bar = start_world_from_left_tool0 * inv(bar_from_left_tool0)` (or use the stored attachment frame).
  3. `bar_interp = FrameInterpolator(Frame.from_transformation(start_world_from_bar), Frame.from_transformation(goal_world_from_bar), options)`. Use `compas_fab.backends.pybullet.backend_features.pybullet_plan_cartesian_motion.FrameInterpolator`.
  4. For each `t` in `linspace(0, 1, N)`:
     - `bar_t = bar_interp.get_interpolated_frame(t)`
     - `left_t_frame = bar_t * bar_from_left_tool0`
     - `right_t_frame = bar_t * bar_from_right_tool0`
     - Solve left IK with `start_state` as seed (using `_skip_cc3/4/5`).
     - Solve right IK with the merged state.
     - Use the prior waypoint's conf as seed for the next iteration.
  5. Concatenate 12 joint_values per waypoint into a single `JointTrajectory(joint_names=LEFT_NAMES + RIGHT_NAMES)`.

**A2. `plan_dual_arm_linear_independent`** (for M3 retreat)
- Signature: `(planner, robot_cell, start_state, start_conf, target_left_frame, target_right_frame, *, num_steps=None, position_step=0.005, rotation_step=0.05, max_results=20, max_descend_iterations=200) -> JointTrajectory | None`
- Algorithm (mirrors `GH_dual_arm_retreat_plan.py:54-200`):
  1. FK at `start_conf` → `start_left_frame`, `start_right_frame`.
  2. Build two independent `FrameInterpolator`s (one per arm).
  3. `N = max(left.regular_interpolation_steps, right.regular_interpolation_steps) + 1`.
  4. For each `t`, interpolate left and right frames independently and IK.

Both functions skip env collisions via `options = {"check_collision": True, "_skip_cc3": True, "_skip_cc4": True, "_skip_cc5": True, "max_results": 20, "max_descend_iterations": 200}`. They reuse `husky_world._augment_tool_touch_links_for_v3(state, husky)` (call it once on the start_state copy before IK loop) so wrist_1/2 ↔ tool is allowed when assembly_tool_v3 is mounted.

### B. `husky_assembly_teleop/husky_monitor.py`

**B1. State init** (around L143):
```python
self._loaded_action = None              # BarAssemblyAction | None
self._loaded_movements = []             # list[Movement]; index 0 = synthetic M0
self._selected_action_file_idx = 0
self._selected_movement_idx = 0
self._ee_target_pose_uids = []          # pybullet debug uids for pp.draw_pose
self._home_conf_12 = None               # cached on first M4 plan
```

**B2. Synthetic M0 helper**:
```python
def _make_synthetic_m0(self):
    """Create a RoboticFreeMovement representing 'staging from live → M1.start'.
    
    start_state captures the live robot conf + base (a fresh RobotCellState
    copy of M1.start_state with the live base + live conf overwritten).
    target_ee_frames = None; the planner uses M1.start.robot_configuration
    as the goal joint conf (set when M1 is planned).
    """
    ...
```

**B3. New methods**:
- `load_bar_action_file()` — reads `_selected_action_file_idx` → loads the .json from `available_robot_cell_states`, parses via `bar_action_io.parse_bar_action`, sets `self._loaded_action`. Prepends synthetic M0 to `self._loaded_movements`. Logs `[BarAction] loaded N movements: M0, B226_M1_constrained, ...`.
- `load_selected_movement()` — pulls `mv = self._loaded_movements[self._selected_movement_idx]`. If `mv` is synthetic M0, snapshot live conf/base into its start_state. Calls the cfab cell-state push (`set_robot_cell_state`), bridges to pp. Sets `goal_arm_pose`/`goal_base_pose` from `mv.start_state.robot_configuration`/`robot_base_frame` (NOT from IK). For each entry in `mv.target_ee_frames` (`left`/`right`), call `pp.draw_pose(pose_from_frame(frame), length=0.15)` and store uids in `self._ee_target_pose_uids` for clean-up on next load. Prints `[Movement] loaded {mv.movement_id} type={movement_type(mv)}`.
- `plan_selected_movement()` — dispatches by `_match_movement_role(mv)` (helper that returns `"M0" | "M1" | "M2" | "M3" | "M4"` based on movement_id substring; M0 only when synthetic). Calls the matching `_plan_{Mn}_dispatch` method. If `mv.trajectory is not None`, log `WARN: overwriting existing trajectory for {mv.movement_id}`.
- `ik_live_base_for_selected_movement()` — copy `mv.start_state`, overwrite `robot_base_frame` from live mocap, run `_solve_bar_action_goal_ik(self, live_state, skip_env_collisions=True, verbose=True)`. On success, set `self.goal_arm_pose` to the IK conf so the user can drive the real robot to that ghost via the existing "Plan Both Arms to Goal (composite)" button. Do NOT write to `mv.trajectory`.

**B4. Per-role planners** (private methods on HuskyMonitor that wrap the api):
- `_plan_M0_dispatch(mv)`: scene = current cfab+pp scene; `start_conf = live_12dof_conf`; `goal_conf = vec12_from_conf(self._loaded_movements[1].start_state.robot_configuration)` (where index 1 = M1). Call `plan_free_dual_arm(scene, start, goal, ...)`. Wrap into JointTrajectory; assign to `mv.trajectory`.
- `_plan_M1_dispatch(mv)`: call `world.plan_and_stage_constrained(self, ignore_env_obstacles=True)` and convert `self.constrained_trajectory` into a 12-joint JointTrajectory.
- `_plan_M2_dispatch(mv)`: derive `bar_from_left_tool0`, `bar_from_right_tool0` from `mv.start_state.rigid_body_states[bar_name].attachment_frame` (the gripper-link → bar relative pose). `goal_world_from_bar = mv.target_ee_frames["left"] * inv(bar_from_left_tool0)` (and validate with right). Call `plan_constrained_dual_arm_linear(...)`.
- `_plan_M3_dispatch(mv)`: call `plan_dual_arm_linear_independent(planner, robot_cell, start_state, start_conf, mv.target_ee_frames["left"], mv.target_ee_frames["right"])`.
- `_plan_M4_dispatch(mv)`: `goal_conf = vec12_from_conf(self._home_conf_12 or HOME_CONF)`. Call `plan_free_dual_arm(...)`.

After every successful plan: write the trajectory to `mv.trajectory`, propagate first conf to `mv.start_state.robot_configuration`, propagate last conf to the NEXT movement's `start_state.robot_configuration` (with a console warning if the next movement already had a different `robot_configuration` set). Also wire into `monitor.planned_arm_trajectory` so the existing trajectory visualizer (`set_to_show_traj_state`, `update_traj_goal_configuration`) displays it.

**B5. JointTrajectory helpers** (top of file or in utils):
- `conf_from_12vec(vec12) -> compas Configuration`
- `vec12_from_conf(conf) -> np.ndarray (12,)`
- `joint_trajectory_from_path(path_12, dt=None)` — wraps `JointTrajectory(joint_names=L+R, points=[JointTrajectoryPoint(joint_values=..., joint_names=...)])`.
- `path_12_from_joint_trajectory(jt) -> np.ndarray (T, 12)`

**B6. Fixed home config constant** (top of file or utils):
```python
HUSKY_DUAL_ARM_HOME_CONF_12 = np.array([
    -1.381079037103113, -0.08674286382411818, -2.8050931738052864,
    -1.7444565873683324, 0.23963370629882144, 1.4217452086745808,
     1.3946926052686688, -3.0267499888085663,  2.8043950421044888,
    -1.727003294848389, -0.40561451816348215, -1.2402309664671707,
])
```

**B7. New UI block** (replace L1863-1873):
```python
if self.BAR_HOLDING_ACCURACY_TEST:
    self.dump_sep_sliders.append(Slider("----------Bar Holding Acc Test", lambda: None))
    n_files = len(self.available_robot_cell_states)
    if n_files >= 1:
        self.bar_action_file_slider = Slider(
            "BarAction file (idx)",
            lambda v: setattr(self, '_selected_action_file_idx', int(round(float(v)))),
            0, max(0, n_files - 1), 0,
        )
    self.buttons.append(Button('Load BarAction', self.load_bar_action_file))
    self.bar_movement_slider = Slider(
        "Movement (idx 0=M0_synth)",
        lambda v: setattr(self, '_selected_movement_idx', int(round(float(v)))),
        0, 8, 0,  # cap at 8; real movements typically ≤5
    )
    self.buttons.append(Button('Load Movement', self.load_selected_movement))
    self.buttons.append(Button('Plan Movement', self.plan_selected_movement))
    self.buttons.append(Button('IK Live Base (debug)', self.ik_live_base_for_selected_movement))
    self.buttons.append(Button('Exec Both Arm Trajs', lambda: world.execute_arm_trajectory_both(self)))
    self.buttons.append(Button('Record markerset take', self.record_bar_holding_marker_take))
    self.buttons.append(Button('Save markerset data', self.save_bar_holding_marker_data))
```
Remove the old `Replan Free (live base)` / `Replan Constrained (live base)` / "BarAction Movement (M index)" / old "Load BarAction (selected M)" entries. Old methods (`replan_free_from_live_base`, `replan_constrained_from_live_base`, `_hide_goal_bar`, `_show_goal_bar`) stay on the class for back-compat but are no longer wired.

**B8. Cleanup on movement re-load**:
- Clear `_ee_target_pose_uids` via `pp.remove_debug(uid)` before drawing the new movement's target frames.
- Reset `_bar_holding_fit_line_uids` already handled.

### C. `husky_assembly_teleop/husky_world.py`

No structural change. Two minor edits:
- `_solve_bar_action_goal_ik` already supports `skip_env_collisions` — no change.
- The existing `_augment_tool_touch_links_for_v3(state, husky)` helper is reused from api.py via a sibling import (or moved to a shared utils module if circular-import becomes an issue; defer until implementation).

### D. `husky_assembly_teleop/bar_action_io.py`

No change. `parse_bar_action` already loads the BarAction; movements expose `mv.trajectory` (slot already exists per `rs_data_structure/bar_action.py:42-51`).

## Reused existing functions

| Function | Path | Use |
|---|---|---|
| `parse_bar_action`, `find_movement`, `movement_type` | `bar_action_io.py:31-94` | Load + classify movements |
| `_solve_bar_action_goal_ik` | `husky_world.py:1752` | Per-movement endpoint IK + debug IK live-base |
| `plan_free_dual_arm` | `external/.../api.py:64-126` | M0, M4 |
| `plan_and_stage_constrained` | `husky_world.py:1906-1966` | M1 (called with `ignore_env_obstacles=True`) |
| `FrameInterpolator` | `compas_fab.backends.pybullet.backend_features.pybullet_plan_cartesian_motion` | M2, M3 interpolation |
| `_augment_tool_touch_links_for_v3` | `husky_world.py` | Wrist-tool ACM for IK in M2/M3 |
| `set_to_show_traj_state` / `set_arm_trajectory` / `update_traj_goal_configuration` | `husky_monitor.py:381-383, 668-671` | Display planned trajectory |
| `pose_from_frame`, `frame_from_pose` | `utils.py:61-66` | Frame ↔ pp.Pose |

## Verification

### 1. Unit smoke (no ROS)
```bash
cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate && source install/setup.bash && python -c "
from husky_assembly_tamp.motion_planner.api import plan_constrained_dual_arm_linear, plan_dual_arm_linear_independent
print('new planners present')
from husky_assembly_teleop.husky_monitor import HuskyMonitor
attrs = ['_make_synthetic_m0','load_bar_action_file','load_selected_movement','plan_selected_movement','ik_live_base_for_selected_movement']
for a in attrs: assert hasattr(HuskyMonitor, a), a
print('monitor methods present')
"
```

### 2. Headless plan dispatch
Add a small script `scripts/headless_test_plan_movement_dispatch.py` (or extend the existing `scripts/headless_live_monitor_test.py`) that:
- Loads `B226.json` from `2026-05-14_foc_demo_reduced/BarActions/`.
- Calls `load_selected_movement` for each idx and asserts the goal ghost pose changes per movement.
- Calls `plan_selected_movement` for each of M1, M2, M3, M4 in order.
- Asserts each `mv.trajectory` is a `JointTrajectory` with `len(points) >= 2`, and adjacent movement-start configs are consistent.

### 3. Live monitor (manual)
Build + launch:
```bash
cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate \
  && python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop \
  && source install/setup.bash \
  && ros2 run husky_assembly_teleop monitor
```
With `BAR_HOLDING_ACCURACY_TEST=1`, `USE_MOCAP=1`, `USE_CELL_STATE_BASE_POSE=0`:
1. Slide `BarAction file` → click `Load BarAction`. Console: list of N movements (M0 + real).
2. Slide `Movement` to 1 (M1) → click `Load Movement`. Goal ghost moves to M1 start config; EE target frames drawn in pybullet.
3. Cycle through 0..N-1; verify each shows a distinct ghost / EE frames.
4. Plan order: M1 → M2 → M3 → M4 → M0. After each `Plan Movement`, the trajectory visualizer should sweep through the path.
5. Load M2; click `IK Live Base (debug)`. Goal ghost updates to live-base IK conf. Click `Plan Both Arms to Goal (composite)` to drive there.
6. Click `Exec Both Arm Trajs` on each movement in turn (M0 → M1 → M2 → M3 → M4) for end-to-end execution.

## Risks / Non-obvious

- **Movement dispatch by id substring**: relies on movement_id containing `M0`/`M1`/`M2`/`M3`/`M4` substrings. The actual BarAction JSON (`B226.json`) must be inspected during implementation; if the convention differs, fall back to dispatching by `(movement_type, position-in-list)`.
- **`plan_and_stage_constrained` returns via `monitor.constrained_trajectory`**: M1's dispatch must read from there after the call and convert to JointTrajectory.
- **Inter-EE drift on M2**: deriving `bar_from_left_tool0` and `bar_from_right_tool0` from the same start_state guarantees consistency at t=0; if the bar attachment_frame is only stored for ONE arm (typical: bar attached to `left_tool0`), we compute `bar_from_right_tool0` via `inv(start_world_from_bar) * start_world_from_right_tool0` (FK at start). Both derived from start_conf → inter-EE is preserved by construction.
- **`mv.trajectory` consistency check**: when propagating last conf to `next_mv.start_state.robot_configuration`, if `next_mv` already has a non-None config that differs, log a warning. The user's stated plan order (M1 → M2 → M3 → M4 → M0) chains correctly without conflicts.
- **Synthetic M0 capture timing**: snapshot live conf/base inside `load_selected_movement` (not at `load_bar_action_file`) so the snapshot reflects the user's actual robot state at the moment they decide to plan M0.
- **Slider range cap of 8**: pybullet/dpg sliders are float; clamping to `min(8, len(movements)-1)` inside the lambda is safer. Implementation should clamp in `load_selected_movement` against `len(self._loaded_movements)`.
- **Old methods staying as dead code**: `replan_free_from_live_base` / `replan_constrained_from_live_base` are no longer in the UI but stay defined to avoid breaking imports elsewhere. Mark with a deprecation comment.
- **Collisions all off initially**: per user instruction, every planner runs with `_skip_cc3/4/5 = True` and (for M1's pp side) `ignore_env_obstacles=True`. Plan-only correctness is the first goal; collisions go back on per-movement after manual validation.

## Tasks (parallelizable)

| # | File | Notes |
|---|---|---|
| T1 | `external/.../api.py` | Add `plan_constrained_dual_arm_linear` + `plan_dual_arm_linear_independent` + shared IK-loop helper |
| T2 | `husky_assembly_teleop/utils.py` | Add `conf_from_12vec` / `vec12_from_conf` / `joint_trajectory_from_path` helpers + `HUSKY_DUAL_ARM_HOME_CONF_12` constant |
| T3 | `husky_assembly_teleop/husky_monitor.py` | State init + synthetic M0 helper + 4 new methods + 5 per-role planners + new UI block (replacing the old) |

T1, T2 can run in parallel. T3 depends on both.
