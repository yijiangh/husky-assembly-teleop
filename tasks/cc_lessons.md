# cc_lessons.md

Patterns and lessons from working in this repo. Append entries here after any
correction or non-obvious finding so we can reuse them in future sessions.

## Constrained dual-arm planner integration (live monitor) — 2026-05

### Active bar identification

- The held bar is the rigid body in `RobotCellState.rigid_body_states` whose
  state has `attached_to_tool != None`. Spawn it as a separate body
  (`monitor.active_bar_body`) — do NOT add it to `monitor.static_obstacles`.
  Bodies with only `attached_to_link` still skip; bodies with neither field
  go into static obstacles as before.

### Constrained-planner contract (does NOT start from the live conf)

- The constrained planner derives its own `start_conf` via
  `derive_constrained_start` (which calls
  `derive_home_start_poses_from_grasps` + `solve_endpoint_dual_arm_ik`).
  The live robot reaches that `start_conf` via a **separate free-space
  staging plan**. Between executing staging and executing the constrained
  plan, the user manually places the bar in the end-effectors.
- Two trajectories are produced and stored separately on the monitor
  (`staging_free_trajectory`, `constrained_trajectory`); a slider toggles
  which one is displayed/executed via the existing `Exec Both Arm Trajs`.

### Grasp transforms

- Prefer FK at `goal_conf` + bar pose at goal (via
  `derive_grasps_from_state`). For the **start** of the planner, do NOT
  re-FK at `seed_conf` — instead reconstruct the goal-state tool0 pose
  directly: `world_from_tool0_goal = world_from_bar_goal *
  grasp_bar_from_tool0`. (Reviewer A flagged this; the offline
  `derive_home_start_poses_from_grasps` math requires goal-state pairs.)
- Optional fallback for grasp loading: `monitor.grasp_targets_override`
  set to `(grasp_bar_from_left, grasp_bar_from_right)` directly.

### Always wrap planner calls in `pp.WorldSaver`

- `get_joint_collision_fn` mutates joints + bar pose during planning;
  without `WorldSaver` the live GUI scene jumps to the goal pose after
  planning. The api wrapper handles this internally and additionally
  saves/restores `bar_body` pose explicitly (since `WorldSaver` doesn't
  always cover non-robot bodies' pose).

### Never call `setup_planning_scene` from the live monitor

- `setup_planning_scene` in `external/.../stage1/minimal_rrt.py` connects
  its own PyBullet client and reloads URDFs. The new
  `husky_assembly_tamp.motion_planner.api` functions take live body ids
  directly so they reuse the monitor's existing scene.

### Feature points from real meshes

- `get_bar_feature_points(aabb_dims)` derives RRT distance-metric features
  from a 3-tuple of AABB extents. For the live mesh-loaded bar, capture
  extents at spawn time via `pp.get_aabb_extent(pp.get_aabb(bar_body))` and
  pass them to the api wrapper. Falls back to default `BAR_BOX_DIMS` if the
  monitor doesn't have one.

### `plan_transit_motion` `dual_arm_index="both"` requires len==2 attachments

- `husky_assembly_teleop/utils.py:191` raises `ValueError` if
  `attachments` is not a list of length 2 in dual-arm composite mode.
  `api.plan_free_dual_arm` enforces this upfront with a clear error.
  Pass the gripper attachments from `husky.object.ee_list[i][1]` for both
  arms (those are always present, regardless of whether a bar is held).

### FK at goal_conf disagrees with GraspTargets JSON by ~50mm

In the antenna design-study datasets, FK at the cell state's goal_conf
produces a `world_from_tool0_left` that's ~50mm off from the value
authored in the corresponding `<target>_GraspTargets.json` (consistent
across D1/G1/V1/H1 — same magnitude, ~0° rotation difference). The
GraspTargets JSON values are authoritative.

**Why it matters:** if the wrapper FK-derives `grasp_bar_from_left` from
`(goal_conf, world_from_bar_goal)`, the resulting grasp transform is
50mm off, which propagates through `derive_home_start_poses_from_grasps`
to a wrong `world_from_bar_start`. The endpoint IK either fails or
returns a config that's in a hard-to-reach region — RRT can't find a
path. Symptom: `task_space_failure` even though the prototype solves
the same target in <1s.

**How to apply:** when a cell state has a sibling `_GraspTargets.json`,
use its authored grasps (`monitor.grasp_targets_override`). The live
monitor's `load_board_validation_state` does this automatically via
`_load_grasp_targets_if_available`. Falls back to FK-derivation when no
JSON exists.

### Live cell state contains the WHOLE assembly, prototype tests don't

The prototype's `setup_planning_scene` creates only the active bar +
robot in scene (`built_bars=[]` by default). The live `load_rigid_body_states_as_obstacles`
loads ALL `rigid_body_states` from the cell state — that's 60+ bodies for
the antenna case (every bar at its final install pose, plus joint
connectors plus structural elements).

Without filtering, the constrained planner has to navigate through a
densely cluttered scene containing future-built bars. The prototype's
parameters aren't tuned for this and the RRT times out.

**How to apply:** in `husky_world.plan_and_stage_constrained`, the
constrained planner's obstacle list filters out:
1. The active bar (it's the manipulated body).
2. Bodies named `b\d+(_0|_joint_\d+)` — design-study assembly elements.
3. Bodies within 5mm of the bar at goal pose (the install neighbors).

What remains: only structural/foundation elements. Trade-off: the bar
is allowed to pass through other assembly bars mid-trajectory. For
follow-up work, replace this with a sequence-based filter (only include
bars at indices < active_bar_index — true predecessors).

### Submodule `disabled_collisions` mismatch (known limitation)

- `get_joint_collision_fn` reads a hard-coded SRDF path inside the
  submodule and does not accept an override. If the live monitor uses a
  different SRDF, link-name matches still work but link-pair masks won't.
  `scene["disabled_collisions"]` is currently informational. Future work:
  extend `get_joint_collision_fn` to accept an explicit
  `disabled_collisions` argument.

### GUI freeze (known limitation)

- The monitor's `update()` loop runs at 0.05s. `plan_pose_rrt` blocks the
  loop during its `max_time` budget. Default `max_time=5.0` in the api
  wrapper; future work: run planning in a background thread with a future.

### Workflow

- For non-trivial multi-file integrations, use a planner / implementer /
  reviewer subagent chain with self-contained prompts (see memory/feedback
  for the user preference). Run real smoke tests inside
  `/home/yijiangh/Code/ros2_ws/venv` instead of static-only checks.

- After `pip install -e <subpkg>`, verify that the *editable* installs of
  `external/compas_fab` and `external/pybullet_planning` are still pointing
  at the local submodules (pip may reinstall transitive deps from PyPI).
  Restore with:
  ```
  pip install -e src/husky-assembly-teleop/external/pybullet_planning \
              -e src/husky-assembly-teleop/external/compas_fab --no-deps
  ```

## Dear PyGui control panel (`HuskyMonitor.USE_DPG_UI`) — 2026-05

### What it is

A feature-flagged replacement for PyBullet's `addUserDebugParameter`
debug GUI. When `HuskyMonitor.USE_DPG_UI=1` (default) the monitor
opens a Dear PyGui window beside PyBullet's 3D viewport with real
buttons, sliders, checkboxes, dropdowns, file dialogs, text inputs,
and live plots. When `0`, the legacy PyBullet sliders return.

### Where the dispatch lives

- `husky_assembly_teleop/ui_backend.py` — `UIBackend`, `PyBulletBackend`,
  `DearPyGuiBackend`, and `make_backend(use_dpg, ...)` factory.
- `husky_assembly_teleop/common.py` — extended `Button`/`Slider`/
  `SliderGroup` plus new shims: `Toggle`, `Dropdown`, `TextInput`,
  `FilePicker`, `LivePlot`, `Group`, `Separator`. All dispatch through
  `common._global_backend`, which the monitor sets in `__init__`.
- `husky_monitor.py` — `USE_DPG_UI` class flag near `FAKE_HARDWARE`;
  `make_backend` called between `start_pybullet` and `world.init`;
  `step()` called at the top of `update()`; cleanup in `destroy_node()`.

### How to add a new widget

```python
# Inside HuskyMonitor.build_ui (or wherever you assemble UI):
from husky_assembly_teleop.common import (
    Button, Slider, Toggle, Dropdown, TextInput, FilePicker, Group,
)

with Group("State Loading", collapsible=True):
    Button("Load Robot Cell State", self.load_board_validation_state)
    Slider("State Index", self.update_idx, 0, max_idx, 0, integer=True)
    Toggle("Auto-load on launch", self._set_autoload, current=False)
    Dropdown("Active Bar", self._on_bar_pick, options=names, current=0)
    FilePicker("Pick state file", self._on_state_file,
               base_dir=DATA_DIRECTORY, ext_filter=".json")
```

`Group` is a context manager that produces a collapsible section in
DPG and a separator slider in legacy mode. Existing decorative
`Slider("----------XYZ", lambda: None, 0,0,0)` calls keep working —
no migration required.

### Known limitations

- **Long-running planners freeze the DPG window** (same as today with
  PyBullet). When a button calls `plan_pose_rrt` synchronously for
  5–30s, the monitor's `update()` loop blocks and the DPG render
  stalls. Future work: run blocking ops in a worker thread.

- **Closing the DPG window triggers full monitor shutdown**: any
  pending ROS work is lost. Don't close the window casually.

- **PyBulletBackend degrades** new widget types: `Toggle` and
  `Dropdown` become 0..1 / 0..N-1 sliders with rounding (warned once
  at startup); `TextInput`, `FilePicker`, `LivePlot` raise
  `NotImplementedError`. If you genuinely need these in legacy mode
  you must keep `USE_DPG_UI=1`.

- **Slider integer dispatch** is opt-in via `Slider("name", action,
  ..., integer=True)`. Existing call sites are float by default; when
  you find a slider that's actually an index, prefer flipping it to
  `integer=True` for a cleaner DPG widget.

### Verify after changes

```
cd /home/yijiangh/Code/ros2_ws
source venv/bin/activate
pip install "dearpygui>=1.10"
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash
ros2 run husky_assembly_teleop husky_monitor
```

Two windows pop up: the PyBullet 3D viewport and the Dear PyGui
control panel. All existing buttons/sliders work; new shims unlock
the rest.

## Dual-arm tracking-validation recording — 2026-05

### Stale globals from old workflow

`execute_and_log_mocap` (`husky_world.py:1074`) used to start with
`global bar_pose, next_bar_pose; bar_pose = next_bar_pose` — leftover
from the old random-bar-arc workflow (`husky_world.py:295–336`). The
constrained planner stores trajectories on `monitor.planned_arm_trajectory`
directly, so those globals are not used. Removed.

**How to apply:** when migrating a workflow, grep for module-level
mutable state (`global X` declarations) referenced from the old entry
point — they're often dead by the time you swap entry points but linger
silently. Pattern: read the `global` declarations of the calling
function and verify each name is still meaningful in the new flow.

### JSON log format extension is additive

`save_dual_arm_E_mocap` now optionally takes `metadata=...` and writes a
`metadata` key alongside `raw_data`. The analysis script (`data/dual_arm_acc_data/0_dual_arm_acc_data_processing.py`)
reads `metadata.reference_right_from_left` if present, otherwise
gracefully skips the absolute-deviation pass — old JSONs from `20250509`–`20250612`
keep loading.

**How to apply:** when extending logged-data formats, prefer adding a
new top-level key over restructuring existing ones. Have the consumer
fall back gracefully when the new key is absent.

### Two complementary metrics for constraint validation

The recording loop captures `right_from_left = inv(right_EE) * left_EE`
each tick. Two metrics distinguish failure modes:

1. **Jitter (variance around per-recording mean)** — controller-side
   noise / per-axis tracking ripple. Independent of an "expected" value.
2. **Absolute deviation vs reference** — saved at start_conf before
   execution, when the constraint is known to hold. Drift over
   trajectory = tracker bias or planner-side constraint violation at
   intermediate knots.

Wraparound caveat: jitter rotation uses Euler-minus-mean, which blows up
near +/-180° (visible in `20250612_1730` and `_1746`). Abs-dev rotation
uses `ref^-1 * sample` so stays well-defined. Future work: switch jitter
to axis-angle / log-map.

## DPG font sizing (HiDPI / readability) — 2026-05

- DPG's default font is the bundled ProggyClean ~13px bitmap — looks
  tiny next to native apps. Fix: load a system TTF in a
  `dpg.font_registry()` and `dpg.bind_font(...)` BEFORE
  `dpg.setup_dearpygui()`. See `DearPyGuiBackend._bind_default_font`
  in `husky_assembly_teleop/ui_backend.py`.
- Tunable via `HuskyMonitor.UI_FONT_SIZE` (px). Default 18.
- Fallback when no TTF is present: `dpg.set_global_font_scale(...)`.
  Works but scales the bitmap font (blocky).
- **PyBullet's `ExampleBrowser` font is hardcoded in C++** — there is
  no Python API to scale it. Workarounds: hide the side panels with
  `p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)` (loses the
  `addUserDebugParameter` sliders too), shrink the viewport window so
  the font is relatively larger, or rely on OS-level display scaling.
  With `USE_DPG_UI=1` the only PyBullet-side widget left is the lone
  "Traj viz time" slider in `start_pybullet`; porting it to a DPG
  `Slider` would let us turn the panel off entirely.


## Dual-arm kissing/insertion port from c81e373 — 2026-05

Plan file: `tasks/2026-05_dual_arm_kissing_port_plan.md`. Summary lessons.

### Hand-port > cherry-pick when target branch has unrelated drift

- jg/dev (which holds c81e373) had ~28 commits beyond the merge-base, mixing
  KISSING/COMPLIANCE with unrelated data dirs and (later) a new ROS2 mocap
  pkg the user explicitly didn't want. Cherry-picking would have meant
  resolving conflicts in 5+ commits across `husky_monitor.py`/`husky_world.py`
  /`husky_robot.py` AND inheriting the data-dir noise.
- Hand-port: read `git show c81e373:<path>`, append target functions/blocks
  verbatim into HEAD versions. One clean additive commit. Strategy chosen
  here, worked smoothly.

### Mode-flag pattern for opt-in feature buttons

- Pattern: add a `<FEATURE> = 0` class attribute on `HuskyMonitor` next to
  `USE_MOCAP / FAKE_HARDWARE / CALIBRATION / DUAL_ARM_ACCURACY_TEST /
  BOARD_VALIDATION` (`husky_monitor.py:51–66`). Wrap the new button block in
  `if self.<FEATURE>:`. Default off. Existing users see no change.
- Used for `DUAL_ARM_KISSING` (kissing experiment + compliance controller
  buttons).

### Compliance-controller infra is foundational — port it whole

- `cartesian_compliance_controller` workflow needs:
  subscribers (`dynamic_joint_states` for tcp_pose, `io_states`, `ft_sensor_wrench`),
  publishers (`target_frame`, `target_wrench`),
  service clients (`switch_controller`, `zero_ftsensor`),
  state slots (`arm_tcp_pose`, `arm_ft_sensor`, `io_states`,
  `active_controller`),
  callbacks (`dynamic_arm_callback`, `io_state_callback`, `ft_sensor_callback`),
  methods (`switch_controller`, `zero_ft_sensor`, `set_screw`,
  `send_arm_cmd_cartesian`, `send_arm_cmd_cartesian_force`).
- All on `HuskyRobotInterface`. Topic naming mirrors the existing `sub_arms`
  left/right/single-arm split (`<name>/{left_,right_,}ur5e/...`).

### TCP-pose correction transform is mounting-orientation-coupled

- `dynamic_arm_callback` applies
  `pp.Pose(pp.Point(), pp.Euler(0, 0, np.deg2rad(-180 if self.dual_arm else -90)))`
  to the raw `tcp_pose` interface value. The `-180`/`-90` Z-rotation encodes
  the v1 tool mounting yaw. **Will need updating for v3 tools** — surface
  this constant when adapting.

### `set_screw(state, idx)` vs `toggle_screw(idx)`

- HEAD only had `toggle_screw`. c81e373 has both — the kissing experiment
  needs the explicit-state setter (start screw on then off twice as a
  forward/reverse signal). Refactored HEAD's `toggle_screw` to delegate:
  `return self.set_screw(not self.screw_states[index], index)`.

### Reuse-don't-rebuild: HEAD already had the buttons we needed

- "load state" / "load trajectory" / "plan to goal" / "move to goal" all
  existed on HEAD as `Load Robot Cell State` / `Load Joint Trajectory` /
  `Plan Both Arms to Goal (composite)` / `Exec Both Arm Trajs`. Only the
  compliance-specific `Switch to Joint (BOTH)` ("ensure joint controller")
  and `Conduct Kissing Experiment` ("start experiment") were new. Mapping
  user-language buttons to existing implementations saves 80% of the work
  — always check what's already wired before porting.

### `controller_manager_msgs` import is a load-time hard dep

- `husky_robot.py` imports `from controller_manager_msgs.srv._switch_controller
  import SwitchController` at module top. Without
  `apt install ros-humble-controller-manager-msgs`, the entire monitor fails
  to import. Easy fix on a fresh dev box; flag it in setup docs.

### Verification steps that worked

1. `python -m py_compile <files>` — catches syntax / undefined-name errors
   without needing ROS deps installed.
2. AST + regex scan for expected symbol presence — catches "function
   defined in wrong scope" bugs that import wouldn't reveal until call time.
3. Full `python -c "from <pkg> import ..."` inside venv with
   `source /opt/ros/humble/setup.bash` — catches ROS msg API mismatches.
   Skip on dev boxes missing rig-only ROS deps; do on the rig before the
   live experiment.
