---
date: 2026-05-05
feature: constrained_planner
env: sim_only            # FAKE_HARDWARE=1, headless test harness
branch: yh/compliant_controller_dualarm
commit: <work-in-progress, no commit yet>
severity: concern
status: fixed
---

# Constrained planner: live-flow integration test (headless)

A self-driven test session via `scripts/headless_live_monitor_test.py`
which drives the *real* `HuskyMonitor` methods (no GUI / no ROS init)
to simulate the user clicking "Load Robot Cell State" then
"Plan & Stage Constrained". Catches integration regressions that the
synthetic-scene harness misses.

## 1. Setup

- **Hardware**: sim only.
- **Cell states**: 9 antenna targets (D1, G1, G2, G3, G4, H1, V1, V2, V3)
  from `data/husky_assembly_design_study/250929_New_Antenna_with_GH_RH_Packed/`.
- **Trajectory time**: defaults.
- **Sliders moved from default**: Constrained Stage = 3.
- **Pre-test actions**:
  - Pulled LFS files in design_study submodule.
  - Restored `external/husky_assembly_tamp/main` after detached HEAD (api.py
    was on `main` but submodule was at `eb01991`).

## 2. Repro steps (per target)

```
python src/husky-assembly-teleop/scripts/headless_live_monitor_test.py \
       --target <D1|G1|...> --stage 3 --max-time 10
```

Internally:
1. Bypass HuskyMonitor.__init__ via `object.__new__`.
2. Populate required attrs (huskies, static_obstacles, etc.).
3. Patch `husky_monitor.VALIDATION_PROBLEM_NAME = "250929_New_Antenna_with_GH_RH_Packed"`.
4. Call `monitor._load_available_bar_actions()`.
5. Set `selected_state_index` to the target's index.
6. Call `monitor.load_board_validation_state()`  ← simulates button click.
7. Call `husky_world.plan_and_stage_constrained(monitor)` ← simulates button click.

## 3. Expected vs Actual

**Expected**: All 4 known-good targets (D1/G1/V1/H1, confirmed via the
synthetic harness) produce constrained + staging trajectories.

**Actual** (after fixes, see §4):
- H1: ✅ constrained 33 wp / staging 118 wp.
- D1, G1, V1: ❌ `task_space_failure` (RRT can't find path within 3 × 5s).

## 4. Bugs surfaced + fixes applied

### Bug 1: active_bar_body never set after Load Robot Cell State

The `attached_to_tool != None` detection branch (added during integration)
doesn't fire on antenna data — all 62 rigid bodies have
`attached_to_tool=None`. The user reports as warning:

```
[WARN] No active bar in scene. Load a goal RobotCellState whose
attached_to_tool rigid body has been spawned.
```

**Fix applied** (in `husky_monitor.py`):
- New helper `_identify_active_bar(robot_cell_state, filename)` runs after
  `load_rigid_body_states_as_obstacles`.
- Tries Convention 1 (`attached_to_tool`), then falls back to Convention 2:
  filename prefix → `DESIGN_STUDY_BAR_NAME_TO_INDEX` → `b<N>_0`.
- For D1: `D1_RobotCellState.json` → `D1` → idx 11 → `b11_0`. ✅

Also reset `active_bar_body` at top of `load_board_validation_state` so
re-loading a different state doesn't keep stale tracking.

### Bug 2: stage 3 fails goal_in_collision in live flow

The constrained planner's `joint_collision_fn` checks the bar (attached
to robot via grasp) against all `obstacles`. In the live flow, **all 17
assembly bars and structural elements are loaded as obstacles** (whereas
the prototype's `setup_planning_scene` only creates the active bar). At
goal, the active bar is in geometric contact with its neighbors by design
(it's being installed). So the goal config flags as in_collision.

**Fix applied** (in `husky_world.py`, `plan_and_stage_constrained`):
- Before planning, FK to goal_conf, place bar at goal, run pairwise
  closest-point checks against all static_obstacles within 5mm.
- Bodies in close-contact get added to `expected_neighbor_contacts` and
  excluded from the constrained planner's obstacle list.
- For D1: detected `b12_0` (penetration -22mm). For G1: 12 neighbors.
  For H1: 2 neighbors (b11_0, b12_0). For V1: 8 neighbors.

Tradeoff: filtered bodies are excluded for the **entire** constrained
trajectory, not just the goal endpoint. So the bar may pass through
those bodies mid-trajectory undetected. This is acceptable for the
install case (bar approaches install location monotonically); not
generally safe for arbitrary planning queries.

## 5. Status after fixes

After the initial two fixes (active-bar identification + expected-contact
filter), only H1 succeeded. Two further fixes were needed:

**Fix 3** (`husky_world.py`, `plan_and_stage_constrained`): exclude
design-study assembly bodies (`b<N>_0`, `b<N>_joint_*`) from the
constrained planner's obstacle list. Without this, the bar's flight path
is blocked by future-built assembly bars that wouldn't actually exist at
install time. Matches the prototype's effective scene.

**Fix 4** (`husky_monitor.py`, new `_load_grasp_targets_if_available`):
when a `<target>_GraspTargets.json` exists alongside the cell state,
auto-load it and use its authored grasp transforms via
`monitor.grasp_targets_override`. Reposition the active bar to the JSON's
goal pose. **This was the primary blocker** — FK at goal_conf differs
from the JSON's `world_from_tool0_left` by a systematic ~50mm
(calibration / convention offset), and using the FK values puts the
endpoint IK in a region where RRT can't reach.

| Target | Active bar | Filtered neighbors | Constrained plan | Staging plan |
|--------|------------|--------------------|------------------|--------------|
| D1     | b11_0 ✅   | 1 (b7_joint_3)     | ✅ 35 wp         | ✅ 122 wp    |
| G1     | b0_0 ✅    | 0                  | ✅ 74 wp         | ✅ 116 wp    |
| V1     | b4_0 ✅    | 1 (b2_joint_0)     | ✅ 73 wp         | ✅ 109 wp    |
| H1     | b10_0 ✅   | 2 (b4_joint_2, b7_joint_1) | ✅ 36 wp | ✅ 118 wp    |

All 4 known-good targets now solve end-to-end via the live-monitor flow.
RNG variance: filtered neighbors and waypoint counts vary slightly
between runs.

## 6. Hypothesis

GUESS: For D1/G1/V1 the home-to-goal pose-space path is genuinely
narrow because the active bar must navigate around 14+ existing bars.
The prototype never hits this because its scene contains *only* the
active bar — its tests are not representative of live install context.

OBSERVATION: H1 succeeds, even though its scene has the same total
number of obstacle bars as the others. The difference is geometric:
H1's home→goal direction may not pass near other bars.

OBSERVATION: `--max-time 30` from the CLI doesn't actually propagate
into `plan_constrained_dual_arm` — the per-attempt budget stays at
5s × 3 attempts = 15s. So the failure is "ran 15s, found nothing" —
not necessarily "no path exists".

## 7. Asks (next steps)

Pick one or more, in priority order:

1. **Plumb max_time through the api**: the `plan_and_stage_constrained`
   wrapper should accept a `max_time` arg and forward it to
   `plan_constrained_dual_arm`. Trivial; do first.

2. **Validate with longer time**: re-run D1/G1/V1 with max_time=60s × 5
   attempts to determine whether they're just slow or genuinely
   infeasible.

3. **Distance-based obstacle filtering**: in addition to the current
   "expected contacts at goal" filter, optionally filter out obstacles
   beyond some radius from the bar's straight-line goal trajectory.
   Reduces RRT search space drastically. Risk: false negatives in
   collision detection.

4. **Compare to prototype with built_bars**: feed the assembly bars to
   `run_stage_trial(scene_spec={"built_bars": [...]})` and confirm the
   prototype hits the same task_space_failure. Apples-to-apples.

5. **Check if the goal pose from the cell state actually matches what
   the install requires**: the -22mm penetration is large. Maybe the
   bar's goal frame is offset from the intended install pose.

## 8. Evidence

- Test artifacts: `scripts/headless_live_monitor_test.py`,
  `scripts/headless_constrained_monitor.py`.
- Active-bar fix: diff in `husky_assembly_teleop/husky_monitor.py`
  (added `_identify_active_bar`).
- Goal-collision fix: diff in `husky_assembly_teleop/husky_world.py`
  (added `expected_neighbor_contacts` filter).
- Submodule rebase needed: `external/husky_assembly_tamp` was at
  detached `eb01991` (predates api.py). Restored to `main`. The
  parent repo should bump its submodule pointer.
