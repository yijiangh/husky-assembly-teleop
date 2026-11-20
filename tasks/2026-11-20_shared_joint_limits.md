# Joint space defined in the core; the monitor normalizes on import

## Context

On 19 Nov (report https://claude.ai/artifact/759FbPoCxNJPJgoCHEksmQ) Rhino exported shoulder lifts a whole turn off
(B21_H 328.3°, B50_H 267.7°, ...). The shoulder lift must stay within −270°..+90°: at +90° (= −270°) the upper arm
points down into the platform and no path crosses it, so 328° is the same pose as −31.7° but out of reach. Seeded with
328°, the monitor's IK produced an unreachable goal and B21_H_approach failed every time. The rule lives only in the
monitor (`shared/assembly_planning/joints.py` `LIFT_RANGE`), Rhino does not know it.

Decided with the user (2026-11-20):
- every joint-space limit is defined once in the shared core (`scaffolding_assembly_core`);
- new exports respect it and Rhino plans with it (the Rhino agent's side);
- the monitor still normalizes wrong values when it loads a process, so IK seeds and plan goals are both covered;
- the core's plan check warns (B16), it does not block;
- the monitor side writes the core commit; the Rhino agent reviews it and wires its side;
- the chain check at the approach goal and the wrist-1 clearance margin are later tasks.

## 1. Core (worktree `~/Code/ros2_ws_next/worktrees/core-joint-limits`, branch `yh/joint-limits` off core main caa8d28)

`scaffolding_assembly_core/urdf.py`, UR ARMS section, next to `UR_JOINT_NAMES`:
- `UR_JOINT_LIMITS`: the UR controller's own limits (elbow ±180°, the others ±360°), the values of the monitor's
  `robot_interface/arm.py:78`.
- `SHOULDER_LIFT_RANGE = (-1.5π, 0.5π)`.
- `joint_limits(urdf, names) -> (lower, upper)`: the joint space planners and exports keep to: the URDF's limits
  (±inf for a continuous joint), within `UR_JOINT_LIMITS` for UR joints and the lift window for shoulder lifts.
  (= the monitor's `turn_limits`, plus the UR controller limits.)
- `shoulder_lift_in_range(names, values) -> list[float]`: each shoulder lift moved by whole turns into the window;
  values inside it up to rounding stay (both edges); other joints untouched.

New `scaffolding_assembly_core/process/joint_range.py`:
- `lifts_in_range(process) -> (Process, lines)`: every movement's start and target joints with their lifts in the
  window; one line per value written outside it (each target, and each start that does not repeat the value before
  it: the Rhino-side review found starts after a `target: None` would otherwise go unseen), e.g.
  "B21_H_approach: alice left... shoulder_lift_joint 328.3° -> -31.7°". Equal values map to equal values, so B11 still
  holds; poses are unchanged, so B14/B15 too.

`process/plan_check.py`: **B16, warning**, one per line of `lifts_in_range`; it names the exporter's fix
(`urdf.shoulder_lift_in_range`), since a Rhino user cannot edit joints by hand.

`doc/process_format.md`: B16 row in §11; a short "Joint space" note (limits defined by `urdf.joint_limits`; values
outside the lift window are the same pose but out of reach; write them inside); what the monitor does with a target:
a free move's goal is solved again at the measured base (`link_poses` = the pose, `joints` pick the IK branch), linear
moves run from the live flange, joints-only goals (home, load, leave) are used as they are; the monitor wraps lifts on
load.

Tests (`tests/test_urdf.py`, `tests/test_plan_check.py`, new `tests/test_joint_range.py`): 328.3 → −31.7,
252.6 → −107.4, −300 → 60, +90 and −270 stay, other joints untouched; `joint_limits` on a test URDF; B16 warns on a
328° target and is silent on −31.7°; after `lifts_in_range` the plan check has no new errors (B11, B15).
`tests/test_core_py39.py` covers Python 3.9.

Core AGENTS.md: a reviewer subagent from Rhino's side (diff + Rhino's `process_source.py`, `process_builder.py`,
`rs_export_process.py`); core tests; push the branch (asks first); the Rhino agent reviews and runs its four consumer
tests; then merge to core main and push.

## 2. Monitor (worktree `~/Code/ros2_ws_next/worktrees/husky-joint-limits`, branch `yh/joint-limits` off `hs/experiment2`)

Now (touches no file of the uncommitted IK-choice work):
- Pin the core commit.
- `plugins/process/loading.py` `load_process`: after `read`, `lifts_in_range`; each line to a `warn` callback.
  `plugins/process/plugin.py` `_load_in_background` / `_load`: log them as warnings ("shoulder lift moved into
  −270°..+90°, the same pose: ..."). `experiments/assembly_plan_check.py` gets it through `load_process`.
- Test: loading a process whose target has a 328° lift gives −31.7° and one warning.
- Task note `tasks/2026-11-20_shared_joint_limits.md`.

After the IK-choice work is committed (it imports `joint_limits`, `turn_limits` from `joints.py`):
- `joints.py`: `joint_limits`, `turn_limits`, `within_lift_range`, `LIFT_*` replaced by the core's; callers in
  `ik.py`, `tamp_route.py`, `route_search.py`, `assembly_guide/plugin.py`, `arm_planner/planner.py` use the core's
  `joint_limits`; `robot_interface/arm.py` takes `UR_JOINT_LIMITS` from the core (run `-m safety`).

## 3. Rhino (the Rhino agent)

Pin the core commit (22aaf37 → it; brings caa8d28's changes too); write values in range (`shoulder_lift_in_range`
in `joints_from_groups`, or `lifts_in_range` on the process before writing); plan and solve IK with
`urdf.joint_limits`; its tests; re-export B21_H, B25_H, B39_H, B50_H with B16 silent.

## Later tasks

- Chain check at the approach goal (an IK solution counts only if the insert/to_grasp and retreat lines from it
  pass), after the IK-choice work lands.
- Wrist-1 clearance margin in the core, once the value is agreed.

## Verification

- Core worktree: `python3 -m pytest`.
- Monitor worktree: `python3 -m pytest -n 0 test/plugins/process test/shared` while iterating, then
  `python3 -m pytest` before the commit (`-m safety` for the later step).
- Load `261117_2ModulesDemo_order_bracing` headless through `load_process`: 10 lift warnings (the report's table),
  none after Rhino's re-export.
- Report doc: asks 1 and 6 point at the core commit.

## Status (2026-11-20)

- Core commit a9fb0f2 on local branch `yh/joint-limits` (core worktree); reviewed from Rhino's side: nothing breaks
  Rhino's tests; current exports gain B16 warnings until Rhino's fix lands with its pin bump; Rhino must fix in
  `joints_from_groups`, not with `lifts_in_range` before `write` (the solutions' hashes would then never match the
  document's build, `plan_steps.changed_since_planning`).
- Monitor: load normalization committed on `yh/joint-limits` (monitor worktree), pinned to a9fb0f2, rebased onto
  `hs/experiment2`, whose a8bd2b73 also wraps the lift IK takes as its reference.
- Not pushed. Next: push the core branch, Rhino agent review + tests, merge to core main, then the monitor.
