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
