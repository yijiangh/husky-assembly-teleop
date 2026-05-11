# husky_monitor.py: pp → cfab full migration plan

**Status:** plan only; foundation pieces (cfab session + headless test
cleanup) already landed. Live monitor migration is the remaining work.

**Goal:** eliminate the parallel `pybullet_planning` (pp) client from the
live `HuskyMonitor` ROS node, so the compas_fab `PyBulletClient` is the
sole PyBullet world for visualization, planning, and state management.
The cfab `RobotCell` + `RobotCellState` becomes the single source of
truth.

**Why:** today the live monitor maintains two parallel PyBullet
simulations (pp for visualization+legacy planners, cfab for BarAction
planning). Keeping them in sync is fragile and the legacy translation
layer (e.g., `monitor.static_obstacles` of pp body ids ↔ RigidBodyState
ACM lookup) introduces error surfaces.

---

## What's already done (foundation, 2026-05)

- `rs_data_structure` extracted to standalone repo; pinned as pip dep in
  all 3 consumer repos.
- `husky_assembly_teleop/cfab_session.py` — `CfabSession(problem_name,
  connection_type, enable_debug_gui)` owns a long-lived `PyBulletClient`
  + `PyBulletPlanner` per design-study problem; `set_robot_cell` is
  called once.
- `husky_assembly_teleop/bar_action_io.py` — thin wrapper around
  `compas.data.json_load` for BarAction files.
- `husky_monitor.load_bar_action(action_path, movement)` — replaces
  `load_board_validation_state`. Uses cfab natively.
- `husky_world.plan_bar_action_movement(monitor)` — cfab-driven goal IK
  + BiRRT (12-DOF, `pybullet_planning.motion_planners.birrt` with a
  `make_cfab_collision_fn` wrapper).
- `husky_assembly_teleop/design_interface/` deleted (outdated).
- Ad-hoc URDF/tool loading removed from `common.py`
  (`generate_and_cache_tool_urdfs`, `validation_tool_pair` ee_type,
  `_copy_urdf_with_meshes` all deleted; `set_robot_cell` handles tool
  URDFs).
- `scripts/headless_live_monitor_test.py` is **cfab-only**: no
  `pp.connect`, no pp robot stub, single PyBullet client. With `--gui`
  the cfab client opens the visible window with a `Path t` debug slider
  to scrub the planned trajectory.

The live `HuskyMonitor` Node still has both pp and cfab; this plan
removes pp from the live monitor too.

---

## Subsystem-by-subsystem migration

### 1. GUI substrate (foundation step)

**Today:** `husky_monitor.start_pybullet()` calls `pp.connect(use_gui=True,
shadows=True, color=...)` — this is the visible PyBullet window.

**After:** Open `CfabSession(VALIDATION_PROBLEM_NAME,
connection_type="gui", enable_debug_gui=True)` in `start_pybullet`.
Store as `self.cfab` (already an attribute on the monitor).

**Concerns:**
- PyBullet supports only one GUI per process — pp must NOT call
  `pp.connect(use_gui=True)` after we open the cfab GUI.
- The world frame markers `pp.draw_pose(pp.unit_pose(), 0.1)` need
  porting to cfab: use `pybullet.addUserDebugLine(...,
  physicsClientId=client.client_id)` for the X/Y/Z axes.

**Files touched:** `husky_monitor.py` (`start_pybullet`, `__init__`
ordering — `start_pybullet` must run BEFORE `world.init`).

---

### 2. Husky robot model (visualization + state)

**Today:** `Husky` / `HuskyObject` in `common.py`:
- `pp.load_pybullet(URDF, fixed_base=False, cylinder=False)` loads the
  husky into pp.
- `pp.create_attachment(robot, tool0_link, ee_body)` attaches gripper
  stubs.
- `HuskyObject.set_pose(base_pose, arm_joint_states)` calls
  `pp.set_pose(robot, base_pose)` + `pp.set_joint_positions(robot,
  arm_joints, ...)` and `attachment.assign()` for each EE.

**After:** The cfab client's `robot_puid` (set by
`planner.set_robot_cell(robot_cell)`) IS the husky. To set pose:

```python
def set_pose(self, base_pose, arm_joint_states):
    state = self._monitor.movement_start_state or self._monitor._cfab_state_cache
    state.robot_base_frame = frame_from_pose(base_pose)
    if len(arm_joint_states) >= 1:
        for n, v in zip(LEFT_JOINT_NAMES, arm_joint_states[0]):
            state.robot_configuration[n] = float(v)
    if len(arm_joint_states) >= 2:
        for n, v in zip(RIGHT_JOINT_NAMES, arm_joint_states[1]):
            state.robot_configuration[n] = float(v)
    self._monitor.cfab.planner.set_robot_cell_state(state)
```

**HuskyObject becomes a shim** that holds `monitor` ref + a cached
working `RobotCellState`. External API unchanged
(`husky.object.robot`, `husky.object.ee_list`, `set_pose`, `set_color`,
`get_arm_joint_names`, `get_ee_pose`, `get_link_pose_from_name`).

**ee_list:** today returns `[(ee_body, pp_attachment), ...]`. After:
ee_body = cfab tool/rigid-body puid; pp_attachment goes away (cfab
handles attachment via `set_robot_cell_state`). Callers reading
`ee_list[i][1]` (the attachment) need to switch to reading the
attached tool/body's puid directly (`client.tools_puids[name]`).

**`get_link_pose_from_name(link_name)`:** becomes
`pybullet.getLinkState(client.robot_puid,
client.robot_link_puids[link_name], physicsClientId=client.client_id)`
→ returns WCF pose tuple. Note: this is in pybullet world frame =
**RCF** (robot-base frame, since cfab places the robot at origin
inside pybullet); for WCF, multiply by `state.robot_base_frame`. See
`cc_lessons.md` for the FK/IK frame-convention lesson.

**Files touched:** `common.py` (rewrite HuskyObject), `husky_monitor.py`
(callers — many; grep `husky.object`).

---

### 3. Goal-state ghost model (transparent preview)

**Today:** `load_goal_model` (`husky_monitor.py:1641`) instantiates a
second `HuskyObject` (transparent) for showing where the robot would
end up if you executed the current goal_arm_pose.

**After two options:**

**(a) Second URDF body in the cfab client.** Manually load the husky
URDF as a separate pybullet body in cfab's client:
```python
ghost_id = pybullet.loadURDF(URDF_PATH, useFixedBase=True,
                             physicsClientId=cfab.client.client_id)
# Set color to transparent
for link_id in range(pybullet.getNumJoints(ghost_id, ...) + 1):
    pybullet.changeVisualShape(ghost_id, link_id, rgbaColor=[0, 0.2, 0.5, 0.3],
                               physicsClientId=cfab.client.client_id)
```
Re-pose via raw pybullet calls. Bypasses `set_robot_cell_state` but
that's fine — it's a visualization-only body.

**(b) Drop the ghost model.** Replace with debug-axis primitives at the
target tool0 frames via `pybullet.addUserDebugLine`. Less informative
but simpler.

**Recommendation:** (a) for parity. Wire a helper
`monitor._spawn_ghost_husky_in_cfab()` called by `load_goal_model`.

**Files touched:** `husky_monitor.py:load_goal_model`,
`update_goal_model_and_color`, etc.

---

### 4. Single-arm and base planners

**Today (in `husky_world.py`):**
- `plan_arm_to_goal`, `plan_arm_to_transfer_element`,
  `plan_arm_to_retract_to_home`: pp.get_collision_fn + pp robot/joints.
- `plan_base_to_goal`: pp planning of mobile base.
- `next_dual_arm_bar_trajectory`: pp dual-arm planning (legacy).
- `execute_arm_trajectory`, `execute_arm_trajectory_both`: apply pp
  joint positions in the live monitor.
- `sample_calib_motion`: pp-based calibration motion sampler.

**After:** Same pattern as `plan_bar_action_movement`. For each planner:

```python
def plan_arm_to_goal(monitor):
    planner = monitor.cfab.planner
    state = monitor.cfab_state  # current working RobotCellState
    start_conf = ...  # extract from state.robot_configuration
    goal_conf = ...   # solve IK on target frame
    collision_fn = make_cfab_collision_fn(planner, state, joint_names)
    path = pp_birrt(start_conf, goal_conf, ..., collision_fn, ...)
    return path
```

**Single-arm specifics:** use the `Left arm` or `Right arm` planning
group; IK is single-arm, BiRRT is 6-DOF.

**Base planner:** the cfab client doesn't have an `Husky Base` planning
group (base is fixed in cfab). Either:
- Add a base group to `RobotCell.semantics` (covers base_joint_x,
  base_joint_y, base_joint_yaw) and let cfab handle it natively.
- OR keep base planning as a separate 3-DOF Cartesian planner that
  doesn't touch pybullet at all (just geometric collision against the
  cfab scene's static bodies).

**Execute path:** `execute_arm_trajectory` today calls
`pp.set_joint_positions(robot, joints, conf)`. After: build a state
per waypoint, `planner.set_robot_cell_state(state)` to redraw.
(Trajectory execution itself goes through ROS to real hardware — the
PyBullet update is purely for the live preview.)

**Files touched:** `husky_world.py` (~10 functions, ~500 lines).

---

### 5. Static obstacles dict

**Today:** `monitor.static_obstacles: dict[str, pp_body_id]` populated
manually; consumed by free-arm trajectory planning, calibration,
collision filters.

**After:** **Delete the dict entirely.** Obstacles live in
`monitor.cfab.client.rigid_bodies_puids` (populated by
`set_robot_cell`). Collision is via `planner.check_collision`, which
iterates the client's known bodies internally — no separate caller-side
list needed.

**Grep for callers** and replace:
- `list(monitor.static_obstacles.values())` → no equivalent needed; the
  collision_fn already knows about them via the cfab client.
- `monitor.static_obstacles[name]` → `monitor.cfab.client.rigid_bodies_puids[name][0]`.

**Files touched:** `husky_world.py` (19 references confirmed),
`husky_monitor.py` (init, `add_static_obstacles` becomes no-op or
deleted).

---

### 6. Mocap-tracked objects

**Today:** `TrackedObject` (`common.py:409`):
- `pp.create_obj(model_file)` body.
- `mocap_callback(pos, rot, ts)` stores pose.
- `set_pose(base_pose)` → `pp.set_pose(self.body, base_pose)`.

**After:**
- Pre-condition: every tracked OBJ is registered as a RigidBody in
  `RobotCell.rigid_body_models` under a stable name (e.g.,
  `mocap_<name>`). Then `set_robot_cell` materializes it with the rest
  of the scene.
- `TrackedObject` holds a reference to the cfab client + the body's
  puid (looked up from `client.rigid_bodies_puids[name]`).
- `mocap_callback` writes to `monitor._mocap_pose_cache[name]` (already
  has a lock).
- In `update()` (live tick), flush the cache: for each tracked body,
  call `pybullet.resetBasePositionAndOrientation(body_puid, pos, quat,
  physicsClientId=cfab.client.client_id)` directly. Bypasses
  `set_robot_cell_state` for ~100× speedup per tick.

**Threading:** mocap callbacks run on a network thread; the per-tick
flush runs on the ROS main thread. Pybullet is single-threaded; the
flush must own all writes. The existing `_mocap_cache_lock` already
serializes the cache.

**Files touched:** `common.py:TrackedObject`,
`husky_monitor.py:update`, `husky_monitor.py:start_mocap`.

---

### 7. Goal-arm-pose sliders

**Today:** Slider callbacks (e.g., `update_goal_arm_pose_left_arm`)
mutate `monitor.goal_arm_pose[arm][joint]` and call
`pp.set_joint_positions(goal_model_robot, ...)` to update the
transparent ghost husky in the pp scene.

**After:** Slider callback:
1. Mutate `monitor.goal_arm_pose[arm][joint]` (unchanged).
2. Build (or mutate) a `RobotCellState` for the goal preview.
3. Call `planner.set_robot_cell_state(goal_state)` IF the ghost model
   uses the cfab `robot_puid` (rare — typically a second body).
4. Or set the ghost husky's joint positions via raw pybullet on its
   body puid: `pybullet.resetJointStatesMultiDof(ghost_id, joint_ids,
   targetValues=[[v] for v in values], physicsClientId=...)`.

**Files touched:** `husky_monitor.py` (slider callbacks),
`load_goal_model` integration.

---

### 8. Calibration mode

**Files:** `husky_world.py::_ensure_calibration_conf`,
`sample_calib_motion`, `calibrate_button`, `save_calibration`,
`record_punch_reference`, `save_punch_validation_data`,
`calibrate_joint`, `execute_and_log_mocap`,
`_capture_reference_relative_EE`. ~600 lines.

**Today:** Reads pp link poses to record end-effector tip positions in
world frame. Saves them as JSON tied to pp joint configurations.

**After:**

- **EE tip pose:** the canonical way in compas_fab is to register the
  attached calibration tool as a `Tool` model with a TCP (tool center
  point) in the `ToolModel.frame_in_tool0_frame` attribute. Then
  `planner.forward_kinematics(state, TargetMode.TOOL, group)` returns
  the TCP pose directly. Alternative: `forward_kinematics_to_link` to
  a specific link id, then compose with a tip offset Frame.

- **Punch-tool geometry:** add a `RigidBody` to `RobotCell` named
  `punch_tool_<side>` with `attached_to_link="<side>_ur_arm_tool0"` and
  a Mesh body (cone). The `monitor.punch_tool_offset` becomes the
  offset baked into the cone mesh (origin at tool0, apex at tip).

- **`monitor.tool0_from_punch_tip`:** becomes the `attachment_frame`
  field of the `punch_tool_<side>` RigidBodyState — RCF, meters.

- **Calibration JSON output:** unchanged. It's already a list of
  Frame/Configuration dicts that compas serializes natively. Just
  build the dicts from `state.robot_configuration` instead of from
  `pp.get_joint_positions`.

- **`sample_calib_motion`:** rewrite the joint-trajectory generator
  (random joint scan + visual record) on top of
  `make_cfab_collision_fn`. ~50 lines.

**Effort:** ~1 day. The serialization stays compatible — only the
in-memory body bookkeeping changes.

---

### 9. Bar-holding accuracy test

**Files:** `husky_world.py::randomize_bar_location_for_ik_and_transfer`,
`sample_bar_location_for_ik_and_transfer`, `compute_ik_for_bar`,
`update_goal_gripper_model_pose`. Gated by
`monitor.BAR_HOLDING_ACCURACY_TEST`.

**Today:** Generates random Frame for the bar; plans transfer via pp.

**After:**
- The bar is already a RigidBody in `RobotCell` (for BarAction
  problems). For accuracy-test mode, register a generic test bar
  similarly.
- "Randomize bar location" mutates `state.rigid_body_states["bar"].frame`
  to a random Frame in a box → `planner.set_robot_cell_state(state)`
  to redraw → `planner.inverse_kinematics(FrameTarget on the grasp,
  state, group)` for the IK.
- Transfer trajectory: cfab-driven BiRRT, same shape as
  `plan_bar_action_movement`.

**Effort:** ~2 hours. Same pattern as the BarAction work.

---

### 10. Dual-arm kissing experiment

**File:** `husky_world.py::kissing_experiment(monitor)` (~700 lines,
gated by `monitor.DUAL_ARM_KISSING`).

**Today:** Pure pp-based: simulates dual-arm compliance approach +
retreat with `pp.get_link_pose` checks.

**After:** The simulation logic is mostly Cartesian-frame math.
Replace `pp.get_link_pose(robot, link)` calls with
`planner.forward_kinematics(state, TargetMode.ROBOT, group=<side>_arm)`
(with WCF→RCF conversion at the boundary). Collision checks →
`planner.check_collision`.

**Effort:** ~3 hours — well-isolated function.

---

## Order of operations (minimize broken-state windows)

Each step is a separate commit; can stop and rollback if anything gets
too messy.

1. **§1 (GUI substrate)** — swap `start_pybullet` to open cfab GUI.
   After this, pp.connect is gone; pp-based planners crash if called.
2. **§2 (Husky model)** — rewrite `HuskyObject` to use
   `set_robot_cell_state`. Now the live husky is rendered by cfab.
3. **§4 (planners)** — migrate single-arm + base + free planners. Now
   all motion planning works.
4. **§5 (static_obstacles)** — delete the dict + all callers.
5. **§7 (sliders)** — point goal-arm-pose sliders at cfab.
6. **§3 (ghost model)** — optional; keep (a) or drop (b).
7. **§6 (mocap)** — if you use it.
8. **§8 (calibration)**, **§9 (accuracy)**, **§10 (kissing)** — in any
   order; each is self-contained.

After step 5 you have a clean cfab-only BarAction live monitor (~1.5
days). Steps 6-10 are mode-specific (~1.5 more days).

**Total estimate:** ~3 days of focused work.

---

## Risks & open questions

- **`base_joint_x/y/yaw`:** husky base is a 3-DOF prismatic+revolute
  chain in the URDF. Does the existing `RobotCell.semantics` include a
  group for it? If not, base planning needs either a new group or a
  separate non-pybullet 3-DOF planner.
- **GUI single-window limit:** if anything calls `pp.connect` after
  step 1, pybullet won't open a second GUI window — would crash or
  silently downgrade. Audit calls to `pp.connect` across the package.
- **Mocap latency:** per-tick `set_robot_cell_state` is too slow.
  Optimization (raw `resetBasePositionAndOrientation`) is documented
  but needs testing under load.
- **Ghost model URDF loaded twice:** if option (a), the ghost husky's
  URDF is loaded into cfab as a separate body. This doubles URDF parse
  time at startup — fine for the live monitor (~1-2 s), but worth
  noting.

---

## Reference

- BarAction migration plan (already executed):
  `~/.claude/plans/concurrent-squishing-gizmo.md`
- Lessons from this migration: `tasks/cc_lessons.md` (entries dated
  2026-05; see "compas_fab FK returns WCF, IK accepts target_frame in
  RCF", "Prefer compas_fab's PyBulletPlanner.check_collision over
  translating to pp", "PyBullet IK randomness defeats numpy.random.seed").
- compas_fab pybullet backend:
  `external/compas_fab/src/compas_fab/backends/pybullet/`
- Pattern reference for cfab-driven planning:
  `husky_assembly_teleop/husky_world.py::plan_bar_action_movement`
- Test pattern:
  `scripts/headless_live_monitor_test.py` (cfab-only, no pp)
