# Revamp BAR_HOLDING_ACCURACY_TEST — Spec + Implementation Plan

Date: 2026-05-15
Scope: live-monitor bar-holding accuracy test workflow

## Context

Build a live-monitor workflow to validate "where the bar physically ends up held"
relative to the planned BarAction M2 goal pose. The user loads M2 (transparent
ghost shown), manually jogs the husky base via joystick (mocap-tracked), then
replans from the live base + same world-frame EE targets, executes, records
mocap marker takes, fits a line through paired bar markers, and reports
pos/orient deviation vs. the goal bar pose. Data persisted to a new gdrive
`data_experiment/bar_holding_acc_data/<date>/` root for offline re-analysis.

The pipeline reuses all existing infra:
- `load_bar_action` (already populates `target_ee_frames`, `grasp_link_from_bar`, `goal_arm_pose`, `goal_base_pose`, ghost robot)
- `_solve_bar_action_goal_ik` (re-IKs with the cell-state base the implementer just overwrites; `husky_world.py:1576`)
- `plan_both_arms_to_goal` (composite free dual-arm; `husky_world.py:1412`)
- `plan_and_stage_constrained_bar_action` (constrained mode; `husky_monitor.py:1107`)
- `request_marketset_button` + `save_markerset_data` (`husky_world.py:822, 861`)
- existing scikit-spatial line-fit logic from `data/bar_holding_acc_data/0_bar_acc_data_processing.py:105-138`

## User-confirmed design decisions

1. **IK targets for replan**: Keep `M2.target_ee_frames` (world) unchanged; re-IK at the live base.
2. **Analysis timing**: Inline on every "Record markerset data" click — fit + print + draw axis. Save when "Save markerset data" is clicked.
3. **Bar viz during free motion**: Hide bar during the motion, show at goal only.
4. **Marker pair config**: Constant in `mocap_experiment.py`; both monitor and analyze scripts import.

---

## Files to modify

### A. `husky_assembly_teleop/__init__.py`
Add after L53:
```python
EXPERIMENT_DATA_DIRECTORY = '/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly/data_experiment'
```

### B. `husky_assembly_teleop/mocap_experiment.py`

- Add module-level `MARKER_NAME_PAIRS = [['5','6'], ['7','8'], ['2','4'], ['1','3']]` (copied from `data/bar_holding_acc_data/0_bar_acc_data_processing.py:16-21`).
- Add `fit_bar_axis_from_markerset(labeled_marker_dict, marker_pairs=None) -> dict`:
  - Input shape matches `monitor._mocap_labeled_marker_cache['bar_rig']` entries `{id: {'pos':[x,y,z], 'error':...}}`.
  - Compute pair midpoints → `Line.best_fit(centers)` (lazy `from skspatial.objects import Line` inside fn).
  - Return `{'point': list, 'direction': list (unit), 'midpoints': [...]}`.
- Add `bar_deviation_from_goal(fit, goal_bar_pose) -> (pos_dev_m, angle_rad)`:
  - `goal_z = R(goal_quat) @ [0,0,1]`.
  - `angle = acos(clip(abs(dot(goal_z, fit_dir)), 0, 1))` — folds direction sign.
  - `pos_dev = ||(goal_pos - fit_point) - dot((goal_pos - fit_point), fit_dir) * fit_dir||` (perpendicular distance from goal_pos to fitted line).

### C. `husky_assembly_teleop/husky_world.py`

**C1.** Top of file: `from husky_assembly_teleop import EXPERIMENT_DATA_DIRECTORY` and
```python
BAR_HOLDING_ACC_EXPERIMENT_DIR = os.path.join(EXPERIMENT_DATA_DIRECTORY, "bar_holding_acc_data")
```

**C2. `request_marketset_button` (L822-859)**: after appending the take dict:
- Run `fit_bar_axis_from_markerset(labeled_marker_data)`.
- Run `bar_deviation_from_goal(fit, monitor.get_world_from_bar_goal_pose())`.
- Stash `fitted_line`, `pos_deviation`, `orient_deviation_rad` into the just-appended take.
- Draw the line via `pp.add_line(p - 0.5*d, p + 0.5*d, color=GOAL_BLUE)`, append uid to `monitor._bar_holding_fit_line_uids`.
- Log `pos_dev_mm` and `orient_dev_deg`.

**C3. `save_markerset_data` (L861-878)**: add kwarg `use_experiment_dir=False`. When True, swap save root to `BAR_HOLDING_ACC_EXPERIMENT_DIR`. Schema unchanged at top level (`{'raw_data': monitor.marker_set_data}`); per-take fields already enriched in C2.

**C4. New `plan_free_dual_arm_from_live_base(monitor)` near L1412**:
```python
live_state = monitor.movement_start_state.copy()
hi = monitor.huskies[monitor.selected_robot_id].interface
live_state.robot_base_frame = frame_from_pose((hi.position, hi.rotation))
monitor.cfab.planner.set_robot_cell_state(live_state)
conf12 = _solve_bar_action_goal_ik(monitor, live_state)   # L1576
if conf12 is None:
    return
monitor.goal_arm_pose[0] = conf12[:6]
monitor.goal_arm_pose[1] = conf12[6:]
plan_both_arms_to_goal(monitor, use_composite=True)
```
Imports: add `from husky_assembly_teleop.utils import frame_from_pose` if not already present.

**C5. New `plan_constrained_from_live_base(monitor)`** near `plan_and_stage_constrained` (L1651): same first 4 lines as C4 to push `live_state` into cfab planner. **Also set `monitor.movement_start_state = live_state`** (see Risks — verify by grep). Then call `monitor.plan_and_stage_constrained_bar_action()`.

### D. `husky_assembly_teleop/husky_monitor.py`

**D1. Imports (~L34)**:
```python
from husky_assembly_teleop.mocap_experiment import (
    fit_bar_axis_from_markerset, bar_deviation_from_goal, MARKER_NAME_PAIRS,
)
```

**D2. State init (~L134)**:
```python
self._bar_holding_fit_line_uids = []
self.goal_base_pose_frozen = False
```

**D3. Freeze goal ghost when test active**:
- In `load_bar_action` after L969 (`self.goal_base_pose = pose_from_frame(...)`):
  ```python
  if self.BAR_HOLDING_ACCURACY_TEST:
      self.goal_base_pose_frozen = True
  ```
- In `update()` at L1864-1872, guard the live-mocap branch that overwrites `self.goal_base_pose`:
  ```python
  if not self.goal_base_pose_frozen:
      self.goal_base_pose = (hi.position, hi.rotation)
  ```

**D4. New monitor methods**:
- `replan_free_from_live_base(self)` → `world.plan_free_dual_arm_from_live_base(self)`; then `self._hide_goal_bar()`.
- `replan_constrained_from_live_base(self)` → `world.plan_constrained_from_live_base(self)`; then `self._show_goal_bar()`.
- `_hide_goal_bar(self)` → `pp.set_color(self.goal_gripper_model, TRANSPARENT)`.
- `_show_goal_bar(self)` → `pp.set_color(self.goal_gripper_model, GOAL_BLUE)`.
- `record_bar_holding_marker_take(self)` → `world.request_marketset_button(self, 'bar_rig')`.
- `save_bar_holding_marker_data(self)` → `world.save_markerset_data(self, use_experiment_dir=True)`; clear `marker_set_data`; iterate `_bar_holding_fit_line_uids` and `pp.remove_debug(uid)`; clear list.

**D5. UI block** (replace L1618-1622):
```python
if self.BAR_HOLDING_ACCURACY_TEST:
    self.dump_sep_sliders.append(Slider("----------Bar Holding Acc Test", lambda: None))
    self.bar_holding_movement_slider = Slider(
        "BarAction Movement (M index)",
        lambda v: setattr(self, '_bar_holding_movement_idx', int(round(float(v)))),
        0, 5, 2,
    )
    self.buttons.append(Button('Load BarAction (selected M)',
        lambda: self.load_bar_action(movement=getattr(self, '_bar_holding_movement_idx', 2))))
    self.buttons.append(Button('Replan Free (live base)', self.replan_free_from_live_base))
    self.buttons.append(Button('Replan Constrained (live base)', self.replan_constrained_from_live_base))
    self.buttons.append(Button('Exec Both Arm Trajs', lambda: world.execute_arm_trajectory_both(self)))
    self.buttons.append(Button('Record markerset take', self.record_bar_holding_marker_take))
    self.buttons.append(Button('Save markerset data', self.save_bar_holding_marker_data))
```

### E. `data/bar_holding_acc_data/` — script revamp (no back-compat)

User has new bar rig: 4 unevenly-spaced pairs. Two **end** pairs (axis points), two **mid** pairs (also axis points). Each end pair-center extends by `BAR_END_OFFSET_M = 0.026` along the fitted axis outward → bar tip. OCF = midpoint of tips. Fit line through all 4 pair-centers → bar Z axis.

#### E1. Update shared config in `husky_assembly_teleop/mocap_experiment.py`
Replace flat list with tagged tuples; add geometry constants:
```python
# (m1, m2, is_end) — is_end=True means this pair is at a bar tip
MARKER_NAME_PAIRS = [
    ('5', '6', True),    # end pair A
    ('1', '3', True),    # end pair B
    ('7', '8', False),   # mid pair
    ('2', '4', False),   # mid pair
]
BAR_END_OFFSET_M = 0.026          # pair-center → bar tip along axis
PAIR_NOMINAL_DIST_M = 0.082       # short cross-bar between paired markers
PAIR_DIST_TOL_M = 0.002
MARKER_ERROR_TOL_M = 0.002
```

#### E2. New helper `fit_bar_from_markerset(labeled_marker_dict, pairs=None) -> dict`
Returns:
```python
{
    'pair_centers':            [c_end_A, c_end_B, c_mid1, c_mid2],
    'pair_is_end':             [True, True, False, False],
    'fitted_line':             {'point': [...], 'direction': [unit]},
    'bar_end_points':          [tip_A, tip_B],
    'ocf_position':            [...],          # midpoint of tips = bar center
    'bar_length_observed':     float,           # ||tip_A − tip_B||
    'straightness_residuals_m':[r0, r1, r2, r3],
    'straightness_max_m':      float,
    'straightness_rms_m':      float,
}
```
Algorithm:
1. Build 4 pair-centers.
2. `Line.best_fit(centers)` → direction `d` (unit).
3. End-pair tips: `pair_center ± BAR_END_OFFSET_M * d`. Sign chosen so the tip moves **away** from the centroid of the centers.
4. `ocf_position = 0.5 * (tip_A + tip_B)`.
5. Residuals: each `c_i` perpendicular distance to fitted line.

Update `bar_deviation_from_goal` to use `fit['ocf_position']` (NOT `line_fit.point`) as observed bar center.

#### E3. Inline (monitor) recording — stamp BarAction reference
At save time in `world.save_markerset_data` (husky_world.py:861), top-level JSON gains:
```json
{
  "bar_action_path": "<absolute path or basename under DESIGN_DATA_DIRECTORY>",
  "movement_id": "M2",
  "movement_index": 2,
  "raw_data": [...]
}
```
Source: `monitor.current_action.action_id` (or full BarAction file path tracked at load time), `monitor.current_movement.movement_id`, `monitor.current_movement_index`. Implementer: add `self._current_action_path` saved in `load_bar_action` (host of L854 resolved path).

#### E4. Rewrite `0_bar_acc_data_processing.py` — OCF + axis only (no back-compat, no cell state)
Simplest possible. Per take:
- Call `fit_bar_from_markerset(marker_pts)`.
- Print: `OCF=(x,y,z) m | axis_z=(dx,dy,dz) | bar_len=L m | straightness_max=S mm`.
- Optional: matplotlib 3D scatter of pair_centers + fitted line + tips for the whole batch.
- Export compiled JSON with: `pair_centers`, `bar_end_points`, `ocf_position`, `fitted_line`, `bar_length_observed`, `straightness_max_m`, `straightness_rms_m`, plus `joint_conf` and `footprint_base_link_pose` passthrough.

Drop everything else: closest-axis classification, robot CoM, support polygon, `tool0_from_bar_center`, URDF loading. Path lookup also simplified (no fallback):
```python
from husky_assembly_teleop import EXPERIMENT_DATA_DIRECTORY
data_folder = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'bar_holding_acc_data', DATA_BATCH)
```

#### E5. Delete `1_bar_acc_stat_analysis.py` and `2_grasp_data_analysis.py`
Obsolete.

#### E6. New `1_compare_to_cell_state.py` — goal-vs-observed using BarAction cell state
Reads marker JSON files in a batch folder; for each:
- Reads top-level `bar_action_path` + `movement_id` (stamped at record time per E3).
- Loads the BarAction via existing `husky_assembly_teleop.bar_action_io.parse_bar_action` + `find_movement`.
- Pulls **goal bar pose** from the cell state: `mv.start_state.rigid_body_states[f'bar_{action.active_bar_id}']`. Compose with the gripper link pose to get world bar frame (or use the `world_from_bar_pose` already in `mv` data if present — check `bar_action_io.py`).
- For each take in `raw_data`:
  - Call `fit_bar_from_markerset(...)`.
  - Compute deviations vs. cell-state goal: `pos_dev`, `angle_dev`, `lateral_dev`.
- Print per-take + aggregate stats (mean/std/max).
- Save `compared_<batch>.json` with deviations alongside the OCF data.

Layout near `0_`:
```
data/bar_holding_acc_data/
  0_bar_acc_data_processing.py     # OCF + axis only
  1_compare_to_cell_state.py       # OCF + cell-state goal comparison
```

#### E7. Update `data/bar_holding_acc_data/check_mocap_data.md`
Document the new MARKER_NAME_PAIRS semantics (end vs mid), `BAR_END_OFFSET_M`, and the two-script pipeline.

---

## Verification

### 1. Headless fit + dev unit check (no ROS needed)
```bash
cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate && python -c "
from husky_assembly_teleop import EXPERIMENT_DATA_DIRECTORY
from husky_assembly_teleop.mocap_experiment import fit_bar_axis_from_markerset, bar_deviation_from_goal
markers = {str(i): {'pos':[x,y,1.0],'error':0.0005} for i,(x,y) in enumerate(
    [(0.0,0.04),(0.1,0.04),(0.0,-0.04),(0.1,-0.04),(0.2,0.04),(0.2,-0.04),(0.3,0.04),(0.3,-0.04)], 1)}
fit = fit_bar_axis_from_markerset(markers)
print(fit, bar_deviation_from_goal(fit, ([0.15,0.0,1.0],[0.0,0.7071,0.0,0.7071])))
print('exp dir:', EXPERIMENT_DATA_DIRECTORY)
"
```
Expect direction along ±x; `pos_dev` ~ 0; `angle` ~ 0.

### 2. Live-base IK + free plan headless smoke
Extend `scripts/headless_live_monitor_test.py`: load BarAction with M2, perturb `movement_start_state.robot_base_frame` by 5cm in x, call `_solve_bar_action_goal_ik` → assert 12-vec returned. Optional: call `plan_both_arms_to_goal(..., use_composite=True)` and assert non-empty trajectory.

### 3. Build + colcon
```bash
cd /home/yijiangh/Code/ros2_ws && source venv/bin/activate \
  && python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop \
  && source install/setup.bash
```

### 4. Live workflow
Set in `husky_monitor.py`:
- `BAR_HOLDING_ACCURACY_TEST=1`
- `DUAL_ARM_ACCURACY_TEST=0`
- `BOARD_VALIDATION=0`
- `USE_MOCAP=1`
- `USE_CELL_STATE_BASE_POSE=0`

Steps:
1. Launch monitor → click `Load BarAction (selected M)` → see transparent goal ghost + blue bar.
2. Jog husky base via joystick → goal ghost stays frozen, live robot moves.
3. Click `Replan Free (live base)` → trajectory previewed; bar mesh hides.
4. Click `Exec Both Arm Trajs` → arms move to goal-EE world frames.
5. Manually place real bar in grippers; close.
6. Click `Record markerset take` ×3 → console prints `pos_dev_mm` / `orient_dev_deg`; fitted line drawn each click.
7. Click `Save markerset data` → JSON appears under
   `~/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly/data_experiment/bar_holding_acc_data/<YYYYMMDD>/bar_holding_acc_<ts>.json`; debug lines cleared.
8. Re-run with `Replan Constrained (live base)` to validate the constrained path.

---

## Risks / non-obvious gotchas

- **Ghost drift**: `update()` L1864-1872 will keep overwriting `goal_base_pose` from live mocap unless `goal_base_pose_frozen` guards it (D3). Most likely first-run bug.
- **Existing button name confusion**: current L1620-1621 has "Record" → request-fn and "Save" → save+clear-fn. New UI (D5) uses dedicated wrappers; replacement of the whole `if` block is fine.
- **Constrained plan base source of truth**: `plan_and_stage_constrained` (husky_world.py:1651+) may read base from `monitor.movement_start_state` rather than from cfab planner state. **Implementer: grep that function body — if it touches `movement_start_state`, C5 must overwrite `monitor.movement_start_state = live_state` before calling.**
- **IK requires planner state already pushed**: `_solve_bar_action_goal_ik` uses `monitor.cfab.planner`; the C4 `set_robot_cell_state(live_state)` call must precede the IK call.
- **skspatial dep**: confirm `pip show skspatial` in `ros2_ws/venv` returns OK. Keep import lazy inside `fit_bar_axis_from_markerset` so monitor import does not fail if missing.
- **Debug line cleanup**: `_bar_holding_fit_line_uids` accumulates across recording sessions; clear on `Save` and on next `Load BarAction`.
- **Bar viz mode**: free-mode hides `goal_gripper_model`; constrained-mode shows it. `Load BarAction` always shows.

---

## Tasks (parallelizable)

| # | File | Notes |
|---|---|---|
| T1 | `__init__.py` | Add `EXPERIMENT_DATA_DIRECTORY` const |
| T2 | `mocap_experiment.py` | Tagged `MARKER_NAME_PAIRS` + geometry constants + `fit_bar_from_markerset` + `bar_deviation_from_goal` |
| T3 | `husky_world.py` | Paths + extend `save_markerset_data` (stamp BarAction ref) + extend `request_marketset_button` (inline fit + draw tips) + 2 replan helpers (single Implementer call covers C1-C5) |
| T4 | `husky_monitor.py` | Imports + state + 6 methods + UI block + ghost-freeze guard + `_current_action_path` (single Implementer call covers D1-D5) |
| T5 | `data/bar_holding_acc_data/0_bar_acc_data_processing.py` | Rewrite: OCF+axis only, no back-compat |
| T6 | `data/bar_holding_acc_data/1_compare_to_cell_state.py` | New: load BarAction cell state, compare to OCF |
| T7 | Delete `data/bar_holding_acc_data/{1_bar_acc_stat_analysis,2_grasp_data_analysis}.py` | Obsolete |

T1, T2 can run in parallel. T3 depends on T1+T2. T4 depends on T2+T3. T5/T6/T7 depend on T2 only.
