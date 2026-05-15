# Dual-arm task-space RRT: BiRRT + multi-start for hard cases (B226)

## Goal

Find a valid Stage-3 motion plan for the hard case `B226.json` M1 movement
without touching collisions or the problem definition. Baseline (B235) must
keep working.

## Findings

- B235 succeeds in ~0.5 s plan time with the legacy single-tree RRT.
- B226 had **no** solution: 5 attempts × 30 s wasted; ~0.2% successful
  extends; tree barely grew because the (fixed) home start at z≈0.87 m
  cannot reach the goal at z≈0.05 m (80 cm vertical drop while carrying a
  2 m bar). Restarting the same start doesn't help.

## Algorithm changes (limited to RRT + start computation; no collision/problem changes)

1. `core.derive_constrained_start` — added `shuffle_deltas: bool = False`. When
   true, the bar-position grid is randomly permuted by `random_seed` instead
   of distance-sorted, so different seeds produce different starts.

2. `run.run_stage_trial` — added `start_retries: int = 1`. When > 1, re-derive
   the home start with a different seed AND a widened sweep box
   (`((-0.4, 0.4), (-0.4, 0.4), (-0.5, 0.3))` instead of `±0.3` in all axes)
   after each planning failure, then re-run the planner.

3. `core.plan_pose_birrt` — new bidirectional RRT-Connect that grows two trees
   (one rooted at start, one at goal). Same SE(3) sampling + dual-arm IK
   propagation as the single-tree planner.

4. **Stitch IK with fallback** — after the two trees meet at a pose, the planner
   re-IKs along the goal-side path seeded from the start-side seam conf to
   keep the entire path on a single IK branch. If that stitch fails on
   collision/continuity, it falls back to re-IKing the start-side path from
   the goal-side branch. Without this, ~95% of BiRRT connections fail the
   seam continuity check from branch flips.

5. CLI flags wired through `run.py`:
   - `--start-retries N` (default 1)
   - `--bidirectional` (monkey-patches `plan_pose_rrt` -> `plan_pose_birrt`)

## Defaults preserved (regression check)

- Calling the CLI without `--start-retries` and without `--bidirectional`
  reproduces the legacy planner exactly. B235 still completes in 4.7 s with
  44 smoothed waypoints, identical waypoint count and stop-reason histogram
  as before.

## Working B226 invocation

```bash
python -m husky_assembly_tamp.motion_planner.dual_arm_task_space_rrt.run \
  --gdrive-bar-action --targets B226.json --movement M1 \
  --gdrive-problem 2026-05-14_foc_demo_reduced --stage 3 \
  --bidirectional --start-retries 6 \
  --max-time 60 --max-attempts 2 --random-seed 0 \
  --no-smoothing
```

Result: **success in ~64 s, 190 waypoints, collisions + continuity + EE
drift all pass.** Reproduces with `--random-seed 42` (101 waypoints,
~120 s).

## Smoothing caveat

With smoothing on, the smoothed path was flagged by validation with
~119 `bar_robot` collisions (first at waypoint 0). The planner's
`joint_collision_fn` (built via `pp.get_collision_fn(... attachments=[bar] ...)`)
and the validation's `bar_robot_collision_fn` (built via
`pp.get_floating_body_collision_fn(bar_body, obstacles=[robot])`) appear to
mask collisions slightly differently, so the smoother accepts intermediate
poses that validation rejects. **Workaround used: `--no-smoothing` for
B226.** Long-term fix would unify the planner-side and validation-side
collision checks; not in scope for this run.

## Artifacts

In `external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/dual_arm_task_space_rrt/reports/`:

- `dual_arm_task_space_rrt_report_20260515_011631.md` — markdown summary
- `_support/dual_arm_task_space_rrt_stage3_B226_B226_M1_CDFM_home_to_approach_20260515_011631_trajectory.mp4`
- `_support/.../_trajectory.json`, `_trajectory_metadata.json`
- `_support/trajectory_validation_stage3_20260515_011737.png`

## Exploration driver (left in place for future tuning)

`external/.../dual_arm_task_space_rrt/temp/explore.py` — calls
`run_stage_trial` directly, suppresses validation-plot/mp4/md output so
parameter sweeps don't pollute `reports/`. Accepts `--start-retries`,
`--bidirectional`, `--no-smoothing`, and the usual RRT knobs.
