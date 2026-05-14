# cc_lessons.md

Patterns and lessons from working in this repo. Append entries here after any
correction or non-obvious finding so we can reuse them in future sessions.

## Diff modified files against session-start git status before reverting — 2026-05-14

When a subagent returns, `git status` shows the cumulative working-tree
state — pre-existing user WIP + agent's edits — not the agent's diff alone.
During the `stage1/` → `dual_arm_task_space_rrt/` refactor I assumed
`VALIDATION_PROBLEM_NAME` → `DESIGN_PROBLEM_NAME` changes in
`husky_assembly_teleop/__init__.py` + `husky_monitor.py` were implementer
scope creep and `git checkout --` reverted them. They were actually
user WIP from before the session, and reverting destroyed in-progress work.

Discipline:

- Capture the session-start `gitStatus` block (Claude Code includes it in
  the bootstrap context). Before reverting any "unexpected" working-tree
  edit, check whether the file was already listed there — if yes, the
  agent's edit may just be propagating a rename or fix the user started.
- For implementer subagents, still declare the file list in the prompt
  ("do not touch files outside this list"), so the agent doesn't *add*
  out-of-scope changes on top of WIP.
- If genuinely uncertain, surface the question to the user instead of
  reverting — the cost of asking is lower than the cost of nuking WIP.

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
- Offline/minimal RRT must follow the same goal-first contract: solve
  `goal_conf`, rederive FK-consistent grasps at `world_from_bar_goal`, then
  call `derive_constrained_start` for both `world_from_bar_start` and
  `start_conf`. Do not compute start_conf by directly IK-solving the
  setup-time geometric start pose.
- For BarAction M1, the direct goal configuration is in the next movement's
  `start_state` cell state (e.g. M2 starts at M1's approach goal). Prefer that
  FK-consistent cell-state configuration over PyBullet IK branch search.
  Accept it with cfab-scale tolerance (about 1 mm / 0.01 rad), not the stricter
  internal IK residual tolerance, because authored cell-state FK can differ by
  a few tenths of a millimeter.

### Grasp transforms

- FK at `goal_conf` + bar pose at goal (via `derive_grasps_from_state`)
  is the single source of truth. For the **start** of the planner, do NOT
  re-FK at `seed_conf` — instead reconstruct the goal-state tool0 pose
  directly: `world_from_tool0_goal = world_from_bar_goal *
  grasp_bar_from_tool0`. (The offline `derive_home_start_poses_from_grasps`
  math requires goal-state pairs.)

### Always wrap planner calls in `pp.WorldSaver`

- `get_joint_collision_fn` mutates joints + bar pose during planning;
  without `WorldSaver` the live GUI scene jumps to the goal pose after
  planning. The api wrapper handles this internally and additionally
  saves/restores `bar_body` pose explicitly (since `WorldSaver` doesn't
  always cover non-robot bodies' pose).

### Never call `setup_planning_scene` from the live monitor

- `setup_planning_scene` in `external/.../dual_arm_task_space_rrt/run.py` connects
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

### Grasps come from FK at goal_conf (RobotCellState is the single source of truth)

Going forward, no `_GraspTargets.json` files. The cell state alone
defines the grasp: FK both tool0s at `goal_conf` and combine with the
active bar's pose. `husky_world.plan_and_stage_constrained` always
calls `derive_grasps_from_state` — no JSON override path.

A historical note: the antenna datasets had a ~50mm offset between FK
and authored grasps. We previously worked around this with a
`_load_grasp_targets_if_available` helper + `grasp_targets_override`.
Both are removed. New datasets are authored such that FK matches.

If you hit the symptom (`Endpoint IK failed`, `task_space_failure`),
verify the cell-state self-consistency check in
`scripts/headless_live_monitor_test.py --diagnose`:

```
[diagnose] cell-state self-consistency: |bar_via_L - wfb_goal| = 0.00 mm,
                                          |bar_via_R - wfb_goal| = 0.00 mm
```

If those errors are non-zero, the cell state's joint values disagree
with its bar pose — fix the data, not the code.

### Husky base ≠ world origin → must compose `world_from_mobile_base`

`derive_home_start_poses_from_grasps` operates in the **mobile-base
frame** (its anchor `MOBILE_BASE_FROM_TOOL0_LEFT_HOME` is in mb coords).
When the cell state's `robot_base_frame` is non-trivial (e.g., the
gdrive transfer-test dataset has the husky at `(0.778, 1.569, 0,
yaw=180°)`), the api wrapper must:

1. Convert `world_from_bar_goal` and the goal-state tool0 poses into
   mobile-base frame before calling `derive_home_start_poses_from_grasps`.
2. Lift the returned `mobile_base_from_bar_start` back to world frame
   via `world_from_mobile_base * mobile_base_from_bar_start`.

`api.derive_constrained_start` accepts an optional
`world_from_mobile_base` keyword (default identity for backward compat).
`husky_world.plan_and_stage_constrained` reads the husky's current
PyBullet pose (`pp.get_pose(robot)`) and passes it through. The
headless harness must do the same — apply
`pp.set_pose(robot_body, monitor.goal_base_pose)` after `Load Robot
Cell State` and before planning.

Implementation location: `derive_constrained_start` lives in
`dual_arm_task_space_rrt/core.py` with the other RRT/IK helpers.
`api.py` may re-export a compatibility wrapper, but new direct imports should
come from `husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.core`.

BarAction target EE frames are in the cell/world frame. When using them in the
minimal planner, convert them by `mobile_base_from_world` from the relevant
goal cell state's `robot_base_frame`; do not treat them as already in
mobile-base coordinates.

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

### Raw joint wraps are hardware-unsafe

- Do not call `+pi/-pi` crossings "physically fine" for this stack. Even if
  the circular angle distance is small, the exported/executed trajectory sends
  raw joint positions to the UR controller. A segment like `179 deg -> -179 deg`
  can be interpreted as a near-`2*pi` move and has caused hardware e-stop /
  speed-limit faults.
- Planner and validation continuity must reject raw wraps or unwrap each IK
  result relative to the previous commanded waypoint before export/execution.
  Circular-distance PASS is not enough for hardware safety.

### Headless monitor must stub live-only UI models

- `scripts/headless_live_monitor_test.py` bypasses `HuskyMonitor.__init__`,
  so any live-only fields used by planner success paths must be explicitly
  stubbed. Example: `husky_world._plan_and_stage_body` calls
  `monitor.update_traj_goal_configuration()`, which expects
  `monitor.goal_model.set_pose(...)`. Headless tests should provide a no-op
  `goal_model` instead of changing live monitor code.

### Headless constrained tests can intentionally ignore env collisions

- For quick constrained-planner debugging in
  `scripts/headless_live_monitor_test.py`, environment/static-body collisions
  may be disabled by clearing `monitor.static_obstacles` after the cfab->pp
  bridge. This keeps robot self-collision and attached tool/bar-vs-robot
  collision checks active through `get_joint_collision_fn(...,
  obstacle_bodies=[])`, while avoiding failures caused by assembly/env bodies.

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

## Gdrive dataset convention (2026-05+)

- Path: `/home/yijiangh/gdrive/0_projects/2025-03 Husky Assembly/data_design_study/`
  (`DESIGN_DATA_DIRECTORY` in `husky_assembly_teleop/__init__.py`). The
  `data/husky_assembly_design_study` git submodule is being deprecated.

- Filename pattern: `<bar_tag>_<phase>.json` (e.g., `B3_approach.json`,
  `B3_assembly.json`). Not `*_RobotCellState.json`.
  `_load_available_bar_actions` accepts any `*.json` excluding
  `_GraspTargets.json`, `_JointTrajectory.json`, and `RobotCell.json`.

- Rigid body roles encoded in name:
  - `active_bar_*` — the manipulated bar (the constrained planner uses it).
  - `active_*` (siblings, e.g. `active_joint_J2-3_male`) — bodies that
    travel rigidly with the active bar during install. Captured into
    `monitor.active_extra_bodies` with their bar-relative offsets at
    load time; excluded from the constrained obstacle list; visually
    repositioned to follow the bar at start pose.
  - `env_*` (e.g. `env_bar_B1`, `env_joint_J1-2_male`) — already-built
    environment obstacles. Stay in the obstacle list (no special handling).
  - `Assembly[Left|Right]ArmToolBody` — gripper bodies attached via
    `attached_to_link` to `[left|right]_ur_arm_tool0`. These get loaded
    as static obstacles by the existing path; collision checks should
    be tolerant since they always move with the robot (filtered by
    PyBullet's self-collision rules via SRDF).

- Active-bar identification (`HuskyMonitor._identify_active_bar`) tries
  in order: Convention 1 (`attached_to_tool`) -> Convention 3 (gdrive
  `active_bar_*` regex) -> Convention 2 (legacy filename ->
  `DESIGN_STUDY_BAR_NAME_TO_INDEX`). Resets `active_bar_body` and
  `active_extra_bodies` on every load.

- Grasps come from FK at goal_conf — no GraspTargets JSON. The new
  datasets are authored such that FK matches the bar pose
  (verifiable via `headless_live_monitor_test.py --diagnose`'s
  cell-state self-consistency check).

### `USE_CELL_STATE_BASE_POSE` flag (mocap on, base from cell state)

Default `1`. When `USE_MOCAP=1`, the live monitor normally tracks the
husky base pose from mocap each tick. For dual-arm accuracy tests you
often want mocap on (for end-effector marker tracking) while the husky
is physically far from the scaffolding, so the assembly-frame base
pose from the loaded `RobotCellState.robot_base_frame` should be used
instead of the mocap-derived one. Set `USE_CELL_STATE_BASE_POSE=1`
for that. Set `0` when actually teleoperating the base in mocap.

Flag-gated sites in `husky_monitor.py`:
- `update()` tick loop (line ~2131): skip mocap-base-pose overwrite of
  `goal_base_pose` and pose the husky from `goal_base_pose` instead.
- `update_selected_robot_id` callback (line ~342): skip the
  mocap-derived re-init of `goal_base_pose`.

Trajectory execution (`execute_arm_trajectory*`) keeps using
`hi.position/rotation` directly because those paths are real-rig
motion control, not assembly-frame planning visualization.

## Sampling DOFs: lock the kinematically-impossible ones first — 2026-05

Pattern from the home-bar-pose sweep
(`auto_compute_home_bar_pose` in `external/.../dual_arm_task_space_rrt/core.py`).

### Probe before sweep when one DOF value is geometrically infeasible

The original `flip_yaw ∈ {0, π}` x `bar_axis_theta ∈ 360 samples` =
720 candidates. But only ONE `flip_yaw` value is kinematically
reachable — the other points the bar to the wrong side of the left
arm, leaving zero IK solutions for the right arm. Half the candidates
were guaranteed to fail IK regardless of collisions.

**How to apply:** when sweeping multiple DOFs, identify any DOF where
some values are *kinematically* infeasible (no IK at all), independent
of collision and the other DOFs. Lock those via a one-shot probe
before iterating:

```python
def _probe_flip_yaw(scene, common_start, spec, rng):
    for fy in (0.0, np.pi):
        bar_pose = ...  # representative bar pose at theta=0
        conf = solve_endpoint_dual_arm_ik(..., max_attempts=1, collision_fn=None)
        if conf is not None:
            return fy
    raise NoValidatedHomeError(...)
```

The probe ignores collisions on purpose — it's only checking
geometric reachability. Lock the result, pass as `forced_flip_yaw`
to the inner sweep.

### Start coarse, refine if needed

The 360-sample bar-axis resolution (1° step) was vastly more than
needed. Locking down to 12 samples (30° step) is plenty for the
home-bar problem and makes the outer EE-position sweep tractable.

**How to apply:** when sampling resolution feels arbitrary, default
coarse (e.g., 30° for orientation, 0.1m for translation). Refining
later is cheap; over-sampling early bloats every outer loop.

## Auto-home pose: silent fallback to in-collision pose — 2026-05

`auto_compute_home_bar_pose` had a silent `logger.warning` +
fallback path: when none of the top-N geometric candidates passed
IK validation, it returned the top-1 unvalidated geometric candidate
anyway. Caller (`validate_auto_home_start_context`) accepted it
without re-checking, so the planner started from an in-collision
configuration.

**How to apply:** when a search function has both "validated" and
"unvalidated" return paths, surface the distinction in the return
value (e.g., `ik_validated=True/False` flag) and let the caller
decide whether to accept the fallback. For the diagnose path, treat
unvalidated as a hard failure (`failure_reason="start_no_valid_home_pose"`).
"Best effort with a warning" is a footgun when downstream code
assumes the result is valid.

## RNG drift between validator and final IK call — 2026-05

In `run_endpoint_ik_diagnosis`, a single `rng = np.random.default_rng(...)`
was passed to BOTH the IK validator (which iterated many candidates
during home-pose search) AND the final `evaluate_endpoint_ik` call.
`solve_endpoint_dual_arm_ik` mutates the rng on attempt-1+ random
restarts, so by the time the final call ran, the rng state differed
from when the validator approved the chosen candidate. If attempt 0
also failed in the final call, it would draw *different* random
seeds than the validator did, finding a *different* (possibly
in-collision) IK solution for the same bar pose.

**How to apply:** when a validator already produced a verified
result, **cache the conf and reuse it** instead of re-solving.
Don't share a stateful rng between a multi-call validator phase and
a single-call finalization phase if the IK is non-deterministic
beyond attempt 0. Also: make the IK collision-aware via an optional
`collision_fn` kwarg, so any retry path filters out colliding confs
internally.

## Bar-anchored start strategy: anchor the GRASP MIDPOINT, not the bar-frame origin — 2026-05

When switching from EE-anchored to bar-anchored start sweep
(`derive_start_pose_from_home_bar`), naively setting
`mobile_base_from_bar = (HOME_POSITION, quat)` puts the **bar-frame
origin** at the home position. For datasets where the bar's local
frame origin sits at one grasp end (e.g. the gdrive
`B3_approach`-style scenes where `bar_from_tool0_left.pos ≈ (0, 0, 0)`
and `bar_from_tool0_right.pos ≈ (0, 0, 1.0)`), this leaves one arm at
the home position and the other ~1 m away — outside reach.

**Fix:** before the outer-sweep loop, compute
`grasp_midpoint_in_bar = 0.5 * (bar_from_tool0_left.pos +
bar_from_tool0_right.pos)`, rotate it by the home-bar quaternion into
mobile-base frame, and **subtract** from the home position. The bar-
frame origin used to compose `candidate_bar` becomes
`HOME_POSITION - midpoint_in_mb`, so the geometric midpoint of the two
grasps (not the bar origin) lands at `HOME_POSITION`.

**How to apply:** any time you anchor a multi-grasp object by its
"home pose," check whether the object's local-frame origin coincides
with the geometric center of the grasp points. If not, shift by the
midpoint offset so the *grasps* (the things that actually need to be
reachable) end up balanced at the target.

## Prefer compas_fab's PyBulletPlanner.check_collision over translating to pp — 2026-05

When integrating with the husky-assembly-teleop planner stack, prefer
`compas_fab.backends.PyBulletPlanner.check_collision(state, options)`
over translating RobotCellState into `pp.get_collision_fn` +
hand-built `extra_disabled_collisions` tuples. The compas_fab path
owns rigid-body spawning, tool attachment, and ACM (`touch_bodies` /
`touch_links` on `RigidBodyState` / `ToolState`) natively.

**Why:** the hand-rolled translation duplicates state-tracking
(monitor's `static_obstacles` + `pp.Attachment` list) AND introduces
translation errors when the JSON encoder/decoder changes (e.g., the
new ACM fields, new attached_to_link semantics).

**How to apply:** stand up a long-lived `PyBulletClient` +
`PyBulletPlanner` per problem (see `husky_assembly_teleop/cfab_session.py`).
Load `RobotCell.json` once and call `planner.set_robot_cell(robot_cell)`.
Per movement: `planner.set_robot_cell_state(mv.start_state)` materializes
the whole scene (poses + attachments + ACM). Inside any RRT
collision_fn, wrap `planner.check_collision(state_with_q, options)`
and catch `CollisionCheckError` for a binary result.

**Trade-off accepted:** per-sample state-copy + check_collision is
slower than pp's `get_collision_fn`, but ACM correctness + zero
translation gap is worth it. Optimize later via
`options["_skip_set_robot_cell_state"]=True` + `client.set_joint_positions`
if profiling shows the inner loop is dominated.

## Dual-arm IK from `target_ee_frames` needs `max_results >= 20` — 2026-05

When using `planner.inverse_kinematics(FrameTarget, state, group, options)`
for the husky dual-arm BarAction goal, the right-arm IK regularly snaps
to a self-collision configuration (e.g., `right_ur_arm_forearm_link` vs
`AssemblyRightArmToolBody`) at the default `max_results=1`. Bumping to
`max_results=20` exhausts the offending IK seeds and finds a valid one
within ~ms.

**Why:** the BarAction's ACM whitelists `wrist_*_link` touch for the
tool body but NOT the forearm. A wraparound IK solution can fold the
forearm into the gripper. `max_results>1` causes the IK solver to
re-seed and find a non-folded branch.

**How to apply:** set `ik_options["max_results"] = 20` (or higher) when
calling `planner.inverse_kinematics` for dual-arm BarAction goals.
Cheaper than adding ACM whitelist entries by hand.

## Legacy dtype alias for `core.bar_action/*` — 2026-05

JSON files written before `bar_action.py` was extracted to
`rs_data_structure` carry `dtype: "core.bar_action/<Class>"`. The
compat shim in `rs_data_structure/__init__.py` registers
`sys.modules["core.bar_action"]` as an alias, so `compas.data.json_load`
resolves the old dtype to the new class. **But the shim only fires
when `rs_data_structure` is imported.**

**How to apply:** any module that calls `compas.data.json_load` on a
BarAction file MUST first `import rs_data_structure` (or import a
sub-symbol from it). `husky_assembly_teleop.bar_action_io` and
`husky_assembly_teleop.cfab_session` already do this. New consumers
should follow the same pattern.

## compas_fab FK returns WCF, IK accepts target_frame in RCF — 2026-05

`PyBulletPlanner.forward_kinematics(state, TargetMode.ROBOT, group)`
returns the tool0 frame in **world coords (WCF)** — it pre-multiplies
by `robot_base_frame` (see
`compas_fab/backends/pybullet/backend_features/pybullet_forward_kinematics.py:100-102`).

`PyBulletPlanner.inverse_kinematics(FrameTarget, state, group, options)`
with `TargetMode.ROBOT` consumes `target_frame` in **robot-base coords
(RCF)** — `target_frames_to_pcf` returns the input unchanged for ROBOT
mode and feeds it straight to pybullet's IK (whose world IS the robot
base). See `pybullet_inverse_kinematics.py:302-304`.

**Net:** a naive FK→IK round-trip fails when the robot base is not at
the world origin. The pybullet world doesn't know about
`robot_base_frame` for IK purposes.

**How to apply:** when feeding an FK output (or any WCF-derived frame)
into IK, convert it to RCF first:
```python
wcf_from_rcf = Transformation.from_frame(state.robot_base_frame)
rcf_from_wcf = wcf_from_rcf.inverse()
rcf_target = rcf_from_wcf * Transformation.from_frame(wcf_frame)
ik_target = FrameTarget(Frame.from_transformation(rcf_target), ...)
```
Conversely, `target_ee_frames` stored in `BarAction.json` are in RCF
already; feed them to IK directly without transformation.

## M1's "ConstrainedMovement" is actually free-space dual-arm reach — 2026-05

Despite the class name `RoboticDualArmConstrainedMovement`, the M1
start state has only the **LEFT** arm gripping the bar
(`notes.bar_arm_side='left'`); the RIGHT arm is at HOME, ~1.2 m from
the bar. The rigid dual-arm constraint applies to M2/M3 where both
arms grip rigidly. M1 is best planned as a **free-space** dual-arm
motion: left carries the bar attached, right arm reaches independently
to `target_ee_frames[right]`.

**Why:** the class name is forward-looking metadata about the
constraint the consumer should apply during M2/M3 execution. For M1
itself, applying the rigid constraint produces a Cartesian
interpolation of the bar between unrelated start/goal poses (right
arm not on bar at start) and leads to mid-path IK failures.

**How to apply:** plan M1 as a 12-DOF free joint-space motion. The
attached bar (`bar_<active_bar_id>` with `attached_to_link=left_tool0`)
follows the left arm automatically through compas_fab's
`set_robot_cell_state` — collision checks against the existing
structure (`bar_B1`, `bar_B2`, ...) catch when the swept-volume
intersects already-built bars.

## PyBullet IK randomness defeats numpy.random.seed — 2026-05

`pybullet.calculateInverseKinematics` uses its own C-level RNG for the
random-restart fallback (when the start-state seed lands in collision
or fails to converge). `np.random.seed(...)` does NOT make this
deterministic. So two consecutive `planner.inverse_kinematics(...)`
calls with the same inputs can return different IK branches; some
branches are kinematically much further from start than others, which
makes downstream BiRRT either fast or hopeless.

**How to apply:** wrap IK + RRT in an outer retry loop. On each
attempt, re-solve goal IK (gets a fresh branch) and re-run BiRRT
against the new goal. Cap with `max_outer_attempts` (5 is plenty for
M1 of B6.json — typical convergence is 1-2 attempts; 3-second per
attempt budget).

## BiRRT collision_fn pybullet_planning quirk — diagnosis kwarg — 2026-05

`pybullet_planning.motion_planners.birrt` may call the
`collision_fn(q, diagnosis=...)` keyword through its inner
`check_direct` step. A naive `def collision_fn(q): ...` crashes with
`unexpected keyword argument 'diagnosis'`.

**How to apply:** always accept and discard extra kwargs:
```python
def collision_fn(q, **_kw):
    ...
```
This is harmless and forwards-compatible with future pp updates.

## Use existing pp-based `plan_and_stage_constrained`, not a cfab-native re-implementation — 2026-05

When wiring the BarAction (cfab `RobotCellState`) path to a real planner,
the temptation is to write a cfab-native planner that consumes the cell
state directly. **Don't.** The dedicated SE(3) bar-pose RRT lives in
`external/husky_assembly_tamp/.../motion_planner/api.py::plan_constrained_dual_arm`
and is already wired to the "Plan & Stage Constrained" button via
`husky_world.plan_and_stage_constrained`. It enforces the rigid-grasp
closed-chain constraint by construction.

The right adapter is **inside `plan_and_stage_constrained`**, not a
parallel planner: when `monitor.current_movement is not None`, derive
`world_from_bar_start/goal` + `grasp_bar_from_left/right` from the
RobotCellState's rigid-body attachments + `target_ee_frames` (skip
`derive_constrained_start`); leave the obstacle filter + planner
calls + trajectory storage untouched.

**Why:** A free 12-DOF BiRRT cannot enforce a fixed-relative-EE constraint
even with ACM whitelisting — the whitelist hides the constraint violation
from collision checking; both arms drift independently. Compas_fab's
`RigidBodyState.attached_to_link` is a scalar (no multi-parent
attachment), so the kinematic loop MUST be enforced at planning time,
not via attachment.

**How to apply:** when the BarAction movement has `notes.bar_arm_side`
set (M1 `RoboticDualArmConstrainedMovement` and M2 `RoboticLinearMovement`
with rigid co-grasp), route to `plan_and_stage_constrained`'s bar-action
branch. For headless emulation, bridge cfab → pp by
`pp.CLIENT = monitor.cfab.client.client_id`, then resolve body ids from
`cfab.client.robot_puid` and `cfab.client.rigid_bodies_puids[name][0]`.

## Headless test: bridge cfab → pp via shared client (don't load a second world) — 2026-05

`monitor.cfab.client` IS a real PyBullet client. To call pp-based
helpers (e.g. `plan_and_stage_constrained`) without spinning up a second
PyBullet connection: set `pp.CLIENT = monitor.cfab.client.client_id`
and resolve husky/bar/obstacle body ids from
`cfab.client.robot_puid` and `cfab.client.rigid_bodies_puids` (which
stores `dict[str, list[int]]` — always take `[0]`).

For the `plan_free_dual_arm` staging plan that requires `len==2`
attachments, stub `husky.object.ee_list` with two identity
`pp.Attachment(robot, tool_link, identity_pose, robot)` — pp.Attachment
needs SOMETHING to bind to, but for staging (bar not held) the value is
not load-bearing.

**Why:** the live monitor instantiates a `Husky` (which loads a fresh
URDF in pp's `CLIENT`) AND a `CfabSession` (which loads its own URDF
in its own client). The headless test uses cfab only — bridging avoids
loading a duplicate husky.

## Register external PyBullet clients with `pp.CLIENTS` — 2026-05

`pp.LockRenderer` (and other internals) reads from a module-global
`pybullet_planning.CLIENTS` dict, populated by `pp.connect`. If the
client was created OUTSIDE pp (e.g., by `compas_fab.PyBulletClient`),
`pp.CLIENTS` doesn't know about it and any pp call going through
`LockRenderer` (including `plan_transit_motion`) crashes with
`KeyError: <client_id>` at the line `self.state = CLIENTS[self.client]`.

**How to apply:** after setting `pp.CLIENT = <external_client_id>`,
also register the entry:
```python
import pybullet_planning as pp
pp.CLIENTS[external_client_id] = True if has_gui else None
```
(`True` = GUI client; `None` = direct/headless.)

## `pp.Attachment(robot, link, _, robot)` ≠ no-op — use a distinct ghost body — 2026-05

For `plan_free_dual_arm` / `plan_transit_motion`, `scene["attachments"]`
must be a list of 2 `pp.Attachment`. The naïve stub
`pp.Attachment(robot, tool_link, identity_pose, robot)` (attaching the
husky to itself) does NOT degenerate to "no extra collision pairs":
`pp.get_collision_fn` checks `attached_child vs robot_self_links`, and
with `child == robot` this collapses to "robot vs robot" and rejects
every config — including HOME — as in self-collision. Symptom:
`plan_transit_motion` prints "initial configuration is in collision /
initial and end conf not valid / transit path not found" even though
the seed is clearly feasible.

**How to apply:** create two tiny invisible PyBullet sphere bodies as
ghost EE proxies in the cfab/pp client, and bind `pp.Attachment(robot,
tool_link, identity_pose, ghost)`. Also exclude those ghost body ids
from `monitor.static_obstacles` so they're not treated as obstacles.

## Headless: sample feasible staging seed near UR5e HOME (HOME itself self-collides ~2mm) — 2026-05

Dual-arm husky URDF's UR5e HOME = `[0, -π/2, 0, -π/2, 0, π/2]` sits ~2mm
inside the `shoulder_link` ↔ `base_link_inertia` self-collision margin.
`plan_free_dual_arm` rejects it for staging. Headless tests must
perturb the seed until `pp.get_collision_fn(robot, joints,
self_collisions=1, max_distance=0)` passes (use the SAME params
`plan_transit_motion` uses internally to ensure agreement). The live
monitor never hits this — it starts from the real physical robot pose.

## Save successful plans to JSON for offline replay — 2026-05

The constrained dual-arm planner is stochastic (BiRRT) and can fail or
take many attempts. When it succeeds, save the trajectories to JSON
(`schema: husky_bar_action_plan/v1`, fields: `bar_action`,
`movement_id`, `joint_names.{left,right}`, `staging_trajectory`,
`constrained_trajectory`). The replay flow loads the BarAction
normally (rebuilds the cfab scene) then loads the JSON to skip
planning and goes straight to GUI visualization. Headless test exposes
`--save-plan PATH` and `--replay PATH` for this.

## pybullet `addUserDebugParameter` segfaults when rangeMin == rangeMax — 2026-05

A zero-width slider range crashes pybullet's GUI render thread
(div-by-zero in the slider widget). Symptom: hard segfault at startup
in `build_ui` (or wherever the slider is created), no Python traceback
even with `PYTHONFAULTHANDLER=1` (crash is in the C GUI thread). Easy
to misdiagnose because Python stdout is block-buffered → the last
*visible* line is whatever happened a few prints before the actual
crash site.

Trigger in this repo: `build_ui` made a "Bar Action" / "Joint
Trajectory" selection slider as `Slider(name, cb, 0, len(files)-1, ...)`
— with exactly 1 file `len-1 == 0` ⇒ `0..0` ⇒ segfault.

**How to apply:** never create a debug-param slider with `min == max`.
`ui_backend._nonzero_range()` widens degenerate ranges as a backstop,
but the cleaner fix is at the call site: don't show a selection slider
for a 0- or 1-element list (0 ⇒ nothing, 1 ⇒ just the action button).
Counts are fixed at launch so static handling in `build_ui` is fine.

## Use the workspace venv for ROS-adjacent scripts — 2026-05

When running repo scripts that import `husky_assembly_teleop`, do not use
plain `/usr/bin/python3`; it may lack `pybullet` and the local ROS package
index. Follow `AGENTS.md`: run from `/home/yijiangh/Code/ros2_ws`, activate
`venv`, and source `install/setup.bash` when the script imports the ROS
package. If running a source-file script directly, also set `PYTHONPATH` to
include `src/husky-assembly-teleop` and the local `external/*` source paths
unless the package has been installed into the venv.

## Lock PyBullet rendering around expensive planners — 2026-05

When a planner expands many states in a GUI PyBullet client, wrap the planner
body in `with pp.LockRenderer():`. Put the lock after any temporary
`pp.CLIENT` switch so it locks the client doing the planning, not the monitor's
other GUI client. This is especially important for constrained planning where
rendering every intermediate scene update can dominate runtime.

## Manual staging ignores the active bar and needs real planner budgets — 2026-05

For the manual staging move before constrained execution, the active bar is not
mounted or held yet. Free staging planners must exclude `active_bar_body` and
active-bar extras from obstacles even if those bodies are loaded for
visualization/constrained planning. Also, if `plan_transit_motion()` prints
only `transit path not found` and not `initial and end conf not valid`, the
start and goal configs passed endpoint collision checks; the failure is search
budget/connectivity, not direct goal collision. Make sure wrapper parameters
like `max_time` and `max_iterations` actually pass through to the underlying
BiRRT instead of being silently ignored.

## Do not alias planner target state for trajectory preview — 2026-05

`goal_arm_pose` is planner input, not just a visualization buffer. In
`HuskyMonitor.update()`, never do `goal_arm_pose = self.goal_arm_pose` before
writing preview waypoints into it; that aliases the list and silently changes
the actual planning target. Use copied arrays for preview display. This matters
when constrained planning sets the manual staging target to constrained-start:
displaying the constrained trajectory can otherwise overwrite the target with a
trajectory waypoint or endpoint.

## Manual staging obstacle set must match constrained-start validation — 2026-05

If the constrained-start IK was validated while excluding design-study assembly
bar bodies (`b<N>_0`, `b<N>_joint_*`), the manual free staging planner must use
the same obstacle filtering. Otherwise the endpoint check can report
`Warning: end configuration is in collision` against a future/loaded assembly
body even though the constrained-start target is valid for the intended staging
phase. Excluding only `active_bar_body` is not enough in BarAction scenes.

## Quick-test composite staging can ignore environment obstacles — 2026-05

When the goal is just to validate the constrained-start handoff quickly, the
composite manual staging planner may use `obstacles=[]` while keeping the two
tool attachments. This checks robot self-collision and robot-tool collision but
ignores assembly/environment bodies. Make this explicit in logs so it is not
mistaken for a production collision policy.
