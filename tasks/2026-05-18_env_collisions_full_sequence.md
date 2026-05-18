# 2026-05-18 — env collisions ON + headless full-sequence test (M1→M2→M3→M4→M0)

Continuation of `tasks/2026-05-18_session_learnings_bar_holding_acc.md`.
Goal: re-enable environment-collision checking across all per-movement
planners, and adapt `scripts/headless_live_monitor_test.py` to plan the
full BarAction sequence by mirroring the UI button clicks.

## Changes

### `husky_assembly_teleop/husky_monitor.py`

- `_plan_M1_dispatch` (was passing `ignore_env_obstacles=True`) →
  `False`. The constrained planner's existing filter at
  `husky_world.plan_and_stage_constrained` already drops active bar +
  design-study `b\d+(_0|_joint_N)` siblings + active-bar extras +
  bodies within 5 mm at goal. The remaining 14 obstacles (on B6.json)
  are the structural set we want enforced.
- `_plan_M2_dispatch`: added explicit `skip_env_collisions=False` on
  the `plan_constrained_dual_arm_linear(...)` call. The api default is
  `True`, so without an override env collisions silently stayed off.
- `_plan_M3_dispatch`: same — explicit `skip_env_collisions=False` on
  `plan_dual_arm_linear_independent(...)`.
- `_build_pp_scene_for_free` (consumed by `_plan_M0_dispatch` and
  `_plan_M4_dispatch`): `scene["obstacles"]` was hardcoded `[]`. Now
  `list(self.static_obstacles.values())`. `static_obstacles` is built
  by `_bridge_cfab_to_pp_for_bar_action` to already exclude the active
  bar and the EE ghost spheres — which matches the "all rigid bodies −
  active_bar − ghosts" set we agreed on. M0 in particular ignores the
  bar's geometry by intent (the bridge uses ghost-sphere attachments,
  not the bar, as EE children).

### `scripts/headless_live_monitor_test.py`

Rewrote `main()` to mirror the UI button sequence:

1. Pre-create the `CfabSession` in `direct` or `gui` mode (instead of
   letting `load_bar_action_file` default to `gui` with no override).
2. Pin `pp.CLIENT = cfab.client.client_id` so `pp.set_color` /
   `pp.draw_pose` calls inside `load_selected_movement` route to the
   right client.
3. Probe-parse the BarAction once to materialize a stub
   `huskies[0].interface` from M1.start_state — needed because
   `_make_synthetic_m0` (called inside `load_bar_action_file`) reads
   `huskies[0].interface.{position, rotation, arm_joint_pose}`. With
   no ROS/mocap in headless, this stub anchors M0 to M1.start exactly,
   making M0 a no-op pair-up in headless (live → M1.start with the
   "live" snapshot == M1.start).
4. Call `monitor.load_bar_action_file()` — same as the UI button.
5. Loop in order `[M1, M2, M3, M4, M0]`:
   - `monitor._selected_movement_idx = idx`
   - `monitor.load_selected_movement()` — same as the UI button.
   - `monitor.plan_selected_movement()` — same as the UI button.
   `_accept_trajectory` saves `<DESIGN_DATA_DIRECTORY>/<problem>/Trajectories/<movement_id>_trajectory.json`
   and propagates `path[-1] → next_mv.start_state.robot_configuration`.
   Abort the sequence on the first planner failure.
6. CLI distilled to `--bar-action`, `--problem`, `--gui`,
   `--only-movement {M0..M4}`, `--no-save`. All the legacy planner
   kwargs (`--max-time`, `--random-seed`, `--draw-rrt`, etc.) were
   removed — the UI button doesn't pass them, so the headless can't
   either while staying faithful to button behavior.

### Stub additions in `_bypass_init_monitor`

Failures during the first headless run showed the bypass-init stub
needed a few more attributes that real `__init__` sets:

- `goal_model.dual_arm = True` — read by
  `update_traj_goal_configuration` after M1 plans.
- `goal_model.set_color = _noop` and `.get_link_pose_from_name = _stub`
  — defensive (used by ghost rendering paths the headless script
  doesn't actually exercise, but cheap insurance).
- `_selected_action_file_idx`, `_loaded_movements`, `_loaded_action`,
  `_traj_ghost_bodies`, `_traj_ghost_orig_colors`,
  `_ee_target_pose_uids`, `planned_arm_trajectory`,
  `BAR_HOLDING_ACCURACY_TEST`, `FAKE_HARDWARE`, `_is_live_monitor`,
  `goal_base_pose_frozen`.

## Results on B6.json (`2026-05-16_double_kissing_jig_demo`)

| step | role | result | notes |
|------|------|--------|-------|
| 1 | M1 | PASS  | constrained RRT, 22 waypoints; 14 static obstacles checked; CDFM sparse validation `joint_continuity=True, raw_wraps=0, ee_constraint=True` |
| 2 | M2 | PASS  | linear bar-held, 4 waypoints; inter-EE drift 0.70 mm / 0.025° |
| 3 | M3 | PASS  | linear retreat, 4 waypoints |
| 4 | M4 | FAIL  | `initial configuration is in collision`; `transit path not found` |
| 5 | M0 | skipped (sequence aborted at M4) |

`--only-movement M{1,2,3}` each succeed in isolation. M4/M0 require
predecessor plans to populate their `start_state.robot_configuration`.

### M4/M0 failure — RESOLVED (mounted-body ACM filter)

Diagnostic in the script revealed the offender on both M4 (B6) and M0
(B6 standalone):

```
robot<->body hits at start_conf (distance<=1mm; <0 == penetration) (2):
  [OBSTACLE] AssemblyLeftArmToolBody(#0)  depths=[-0.0551, -0.0137] robot_links=['left_ur_arm_wrist_2_link', 'left_ur_arm_wrist_3_link']
  [OBSTACLE] AssemblyRightArmToolBody(#1) depths=[-0.055,  -0.0137] robot_links=['right_ur_arm_wrist_2_link','right_ur_arm_wrist_3_link']
```

`Assembly{Left,Right}ArmToolBody` are the **wrist-mounted tools** —
rigidly attached to `*_ur_arm_tool0`. Their cfab collision meshes
intentionally overlap the wrist links by 1–5 cm. The constrained
planner's `expected_neighbor_contacts` (5 mm probe of bar↔body at
goal) absorbs this overlap because, at the M1 goal pose, the bar sits
right against the gripper, so the tool body is < 5 mm from the bar
and gets excluded. The **free planner has no such probe** — and it
fed all `static_obstacles` (including the mounted tool bodies) into
`pp.get_collision_fn`. Every initial-conf check therefore tripped on
the wrist↔mounted-tool overlap.

Fix (`husky_monitor.py:_build_pp_scene_for_free`): exclude any rigid
body whose `start_state.rigid_body_states[name].attached_to_link` is
non-None from `scene["obstacles"]`. This generalises the bridge's
existing single-body `active_bar` exclusion to **every robot-mounted
body** (tools + held bar + active extras). It's an ACM at the
obstacle-set level rather than at the disabled_collisions level — the
mounted body just isn't checked.

After the fix, the full B6 sequence plans clean: M0=105 wp,
M1=22 wp, M2=4 wp, M3=4 wp, M4=188 wp.

The diagnostic itself lives in `scripts/headless_live_monitor_test.py`
as `_diagnose_free_plan_collision`, auto-fired on M0/M4 plan failures.
Keep it: any future obstacle-set regression surfaces the offending
body name in one run.

### (historical, now resolved) M4 failure — original notes

`plan_free_dual_arm` → `plan_transit_motion`
(`husky_assembly_teleop/utils.py:340-352`) uses
`pp.get_collision_fn(robot, movable_joints, obstacles=scene_obstacles,
attachments=ee_ghosts, self_collisions=1, max_distance=0)`.

It rejects M4.start_conf (= M3.path[-1]) as initially-in-collision. M3
itself accepted that same conf under `_run_dual_arm_cartesian_ik_loop`
with `skip_env_collisions=False` — i.e., cfab's IK collision check
(via `_skip_cc3/4/5=False`) passed it. So the two collision stacks
disagree about whether M3.end is in env collision.

Hypotheses (not yet verified):

1. **Different obstacle sets.** `scene["obstacles"] = list(self.static_obstacles.values())` is built by the cfab→pp bridge and contains every rigid body in `cfab.client.rigid_bodies_puids` except the active bar and the EE ghosts. cfab's own IK collision check (CC.3/4/5) may be operating on a smaller set, or with name-pattern exclusions that pp.get_collision_fn doesn't have.
2. **Different margins.** pp.get_collision_fn uses `max_distance=0.000`; cfab's IK collision check may carry a small margin internally that lets a 1–2 mm penetration through.
3. **Ghost-vs-bar attachment mismatch.** `plan_transit_motion` passes `attachments=ee_ghosts` (the two tiny spheres at the tool0 links). cfab during M3's IK has the *bar* attached, not the ghost spheres, so the swept volume around the wrist is different — but the *robot link* collision should be the same. Worth confirming the offending pair is robot-vs-static, not attachment-vs-static.

Earlier in the same run, the M1 constrained planner logged a warning
during its goal-IK pass:
```
[WARNING] pairwise link collision: (Body #dual-arm_husky_Cindy, Link #left_ur_arm_wrist_2_link) - (Body #body0, Link #link0)
[WARNING] Penetration depth: 0.013681 (m) | point1 (0.285589,0.195866,0.972388), point2 (0.272285,0.195872,0.975581)
```
M1 then proceeded and succeeded. The `body0:link0` body name is
suggestive — it looks like a cfab-loaded body without a name pattern.
Could be the same body that trips M4.

### Suggested next steps (deferred to user)

- Add diagnostic printing in `plan_transit_motion` (or wrap
  `_plan_M4_dispatch` with a one-shot collision report) to identify
  which (link, body) pair fires the initial-conf rejection. Then
  decide:
  - Tighten M3's collision check to use the same `pp.get_collision_fn`
    semantics, so the conf M3 hands off is one M4 accepts.
  - Or: relax M4 with a small `max_distance` margin (≤ 2 mm) consistent
    with cfab IK.
  - Or: add a name-pattern filter to `_build_pp_scene_for_free`
    matching the design-study filter on the constrained planner
    (decided against in this round, but the M4 failure may reopen it).

## How to repro

```
cd /home/yijiangh/Code/ros2_ws
source venv/bin/activate
source install/setup.bash 2>/dev/null
python src/husky-assembly-teleop/scripts/headless_live_monitor_test.py \
    --bar-action B6.json --no-save
```

Single-role triage:
```
python src/husky-assembly-teleop/scripts/headless_live_monitor_test.py \
    --bar-action B6.json --only-movement M2 --no-save
```

With cfab GUI window:
```
python src/husky-assembly-teleop/scripts/headless_live_monitor_test.py \
    --bar-action B6.json --gui --no-save
```
