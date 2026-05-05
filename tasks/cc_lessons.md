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
