# 2026-05-14 — Per-segment pacing + dual-arm sync for trajectory send

## Goal
Stop the UR external-control safety check from rejecting the planned
constrained dual-arm trajectory ("velocity 21.8855 required ... within
0.002 seconds is exceeding the joint velocity limits"), while preserving
synchronized arrival of the two arms at every waypoint (so the bar grasp
is not torn).

## Root cause
- `husky_robot.to_trajectory_msg` was setting a uniform
  `dt = trajectory_time / (n - 1)` and leaving `point.velocities` empty.
- For the planner output, `max_j(|Δq_j|)` per segment can reach ≈10 rad/s
  on `wrist_1` even at dt ≈ 0.95 s — the URCap then fires when the
  controller's 2 ms cycle command exceeds the joint velocity limit.
- And: per-arm dt computation would desync the two arms, breaking the
  bar grasp constraint.

## Files
- `husky_assembly_teleop/husky_robot.py`
  - New class constants on `HuskyRobotInterface`:
    - `_DEFAULT_V_TARGET = 0.5  # rad/s`  (single conservative cap)
    - `_MIN_SEG_DT = 0.01  # s`  (floor for repeated waypoints)
  - New helpers:
    - `_start_pose_ok(positions, index)` — extracted from existing
      atol=0.1 rad start-pose check.
    - `_compute_shared_time_schedule(combined_positions, min_total_time,
      v_target=None)` (classmethod) — returns cumulative time array
      where each `seg_dt = max(MIN_SEG_DT, max_j(|Δq_j|) / v_target)`,
      then scaled UP if natural total < `min_total_time`. Operates on
      6-DOF (single-arm) or 12-DOF (dual-arm concatenated) input.
    - `_build_trajectory_msg(positions, joint_names, t_schedule,
      velocities=None, v_target=None)` (classmethod) — wraps positions +
      time schedule into a `JointTrajectory`. When `velocities` is None,
      computes centered finite differences against `t_schedule`,
      endpoints = 0, clipped to ±v_target.
  - `send_dual_arm_cmd` — now computes ONE shared 12-DOF schedule from
    `np.hstack([pos_L, pos_R])` and feeds the same `t_schedule` into
    both `_build_trajectory_msg` calls. Both arms' `time_from_start`
    are byte-identical. Uses `max(L_time, R_time)` as the lower bound.
  - `to_trajectory_msg` and `send_arm_cmd` — refactored to use the
    same helpers. `send_arm_cmd` keeps its existing tolerances and the
    publish-vs-action-client branch. Dropped the leftover
    `print('monitor trajectory time:', traj_time); print('dt:', dt)`
    debug at the old line 612–613.

- `scripts/check_dual_arm_send_pacing.py` (new) — offline check that
  loads `B7_A0_assemble_B7_M1_CDFM_home_to_approach_constrained_dual_arm_JointTrajectory.json`
  via `compas_fab.robots.JointTrajectory.from_json`, splits per-arm
  using `HUSKY_DUAL_UR5e_JOINT_NAMES`, runs the new helpers, and
  asserts all four invariants. Prints a before/after diagnostic
  including the worst (joint, segment) combo.

## Verification
1. `colcon build --packages-select husky_assembly_teleop` — clean.
2. `python scripts/check_dual_arm_send_pacing.py` against the live
   B7 export: 211 waypoints loaded; all four invariants pass; worst
   case `left_ur_arm_wrist_1_joint` at segment 126 went from
   10.973 rad/s (old uniform) to 0.194 rad/s (new per-segment).
3. Hardware end-to-end test pending: re-run `Plan & Stage Constrained`
   on B7/M1, then `Exec Both Arm Trajs`. Should no longer trigger the
   URCap rejection on either pendant; INFO log shows
   `[dual-arm send] N waypoints, total=Xs (requested >= 120.00s),
   v_target=0.5 rad/s — shared schedule`.

## Notes / gotchas
- Sync invariant relies on the relay
  (`crl_husky/onboard/multi_arm_relay.py`) publishing both
  `JointTrajectory`s back-to-back. Microsecond-scale gap is well within
  bar grasp tolerance; if a future bug introduces real wall-clock skew,
  this fix does NOT add a re-sync mechanism.
- `_DEFAULT_V_TARGET = 0.5` is intentionally conservative; raise via
  the class constant if motions feel slow.
- Pendant speed-scaling is per-arm. If the two pendants are at
  different speed overrides, controllers will desync regardless — that's
  an operator/hardware concern.
- Lower-bound semantics: when natural pacing total > requested, no
  scaling. When natural total < requested, `t_schedule` scales UP to
  the requested. This means the "traj time" slider acts as a floor;
  motions that would naturally need more than the slider value get the
  extra time (instead of overspeeding).
