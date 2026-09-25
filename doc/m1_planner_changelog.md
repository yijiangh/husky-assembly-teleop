# M1 transfer planner — change log

Living note: one entry per change to the bar-held transfer planner — the
**start derivation** ("the sweep", `derive_constrained_start_tracked` in
`external/husky_assembly_tamp/.../dual_arm_task_space_rrt/core.py`, driven from
`_derive_constrained_start_for_plan` in `motion_planner/api.py`) and the
**BiRRT** (`plan_pose_birrt`, same `core.py`) — plus the changes around them
that alter what the planner sees (floor, goal, base). Newest first. Each entry
says what changed, where, why (with the evidence), what it did to the numbers,
and how it was verified. **Update it in the same turn as the change.**

How to read a run: `doc/m1_derive_dashboard.md`. Session workflow:
`doc/bar_holding_acc_manual.md`. Reproduce any number below at the desk with
`scripts/derive_m1_headless.py` (`--problem`, `--bar`, `--anchor`, `--plan`).

## Vocabulary

- **sweep / start derivation** — from the goal bar pose, walk the bar in a
  straight line (positions lerp, orientation by the shortest rotation) back to
  candidate *home* poses: 3 carry anchors (horizontal / vertical / back) x 54
  orientation variants (roll / yaw ±30..180) x 343 position offsets on a 10 cm
  grid; per-waypoint warm-seeded ssik IK; a walk *breaks* on an IK miss or a
  joint jump > 10° (branch flip). A fully walked, collision-free walk is a
  **corridor** and is used as the M1 path directly (no RRT). A walked-but-blocked
  home is a **start only** (BiRRT then searches from it). 120 s budget, 40 s
  share per anchor; the sweep exits early only on a corridor.
- **goal-branch probe** — before the sweep, an 8 s walk test from the authored
  goal configuration; if it reaches a home continuously the authored goal is
  kept, else ssik re-pairs the goal branch against the homes.
- **BiRRT** — bidirectional RRT-Connect in bar-pose space with per-waypoint IK,
  up to 5 attempts x `max_time` (120 s); extensions stop on collision / IK
  failure / continuity.

## 2026-09-25 — half-turn walks with a fixed direction: tried, regressed, reverted

- **Finding (kept):** on 260716 B3 the goal bar orientation and the canonical
  horizontal home differ by exactly 180° (a roll about the bar's own axis: the
  tools face the other way). The walk interpolates orientation with pybullet's
  slerp, which for a half turn picks the direction from the sign of a ~1e-8 dot
  product — floating-point noise that the base pose changes. Evidence: identical
  goal configuration and identical home poses in the robot frame, yet at the
  authored base (1.963, 0.584, yaw −90°) candidate #101 was a clear corridor
  while at the live mocap base (1.839, 0.539, yaw −93.1°) the same candidate
  broke at 84 % with a 38° right-elbow flip; 97 of the first 102 candidates
  changed outcome; waypoint orientations differed by 2.9° at wp 1, 28.6° at
  wp 10, 85.7° at wp 30 with identical positions. Reproduced headless at the
  live base; the 0.7° mocap tilt made no difference. Runs:
  `20260925-111718_260716_phase1_test_B3_all.json` (monitor, live base),
  `20260923-175813_260716_phase1_test_B3_all.json` (headless, authored base).
- **What was tried:** `core.py` `_walk_rotation` / `_walk_poses`: for a rotation
  within 10° of a half turn, fix the axis sign by a rule (along the bar's own
  axis for a roll, else up, else forward), turn the positive way, and if that
  walk breaks, walk the same home the other way round and keep the better one.
- **Why it was reverted (same day):** the fixed direction was the wrong one for
  B3 and the other-way retry doubles the cost of every broken canonical
  candidate, so each anchor's 40 s share covered half as many home poses.
  Authored-base B3: 30 s corridor → 397 s via BiRRT; live base, all anchors: no
  path in 721 s; live base, horizontal only: corridor at 76 s (the old
  direction, reached only after 102 double walks). 260921 spot checks with the
  change: B3 corridor 18 s, B9 BiRRT 545 s (was 397), B15 BiRRT 168 s (was
  182). Runs `20260925-*_260716_phase1_test_B3_{all,horizontal}.json`
  (movement ids `halfturn-live-all`, `halfturn-live-horizontal`,
  `halfturn-authored-all`) and `20260925-12*_260921_motion_sample_B{3,9,15}__J_all.json`.
  The code is back to the slerp walk; the finding is an open item below.
- **Consequence:** the automatic start derivation is base-pose sensitive on
  bars whose goal roll is 180° from the canonical home. For the mocap session
  the start is now chosen by hand (next entry).

## 2026-09-25 — human-in-the-loop M1 start (mocap bar-reaching session)

- **Where:** `husky_assembly_teleop/m1_manual_start.py` (`manual_m1_start`),
  monitor sliders *slide along bar* / *roll about bar* / *shift perp. 1, 2* +
  button **M1: Confirm manual start pose (IK check)** (`confirm_m1_manual_start`;
  the derive button shares `_m1_live_context` / `_show_m1_endpoints`),
  `scripts/derive_m1_headless.py --manual-start ANCHOR,SLIDE,ROLL[,P1,P2]`.
- **Why:** the automatic derivation is base-pose sensitive on this bar (entry
  above) and the session could not wait. The operator now picks the carry
  anchor (existing slider) and adjusts the bar pose; Confirm solves the
  collision-checked dual-arm IK for it with the goal's grasps on the branch
  nearest the goal (`solve_endpoint_dual_arm_ik` twice: reachability, then with
  the cfab collision function) and adopts the start (M1 start, M0 goal).
  'Plan Movement' on M1 then runs the BiRRT from that stored start
  (`derive_start=False`, unchanged); Step B's transfer loop never used the
  derivation and is unchanged.
- **Planner change:** none — the sweep and the BiRRT are untouched; the manual
  path only replaces *how the start configuration is chosen*.
- **Verification (headless, 260716 B3, authored and live mocap base alike):**
  horizontal roll 0 → collision (that is the canonical carry, 180° from the
  goal's roll; 2.8 s because every branch pair is collision-checked);
  horizontal roll 180 → start found in 0.2 s; vertical → no IK; back → start
  found in 0.2 s; anchor `all` → back in 2.9 s. Robot session: pending.

## 2026-09-24 — floor slab at the exported ground height

- **Where:** `husky_assembly_teleop/cfab_session.py` `_slab_mesh_from_polygon`
  / `_walkable_ground_slabs` (`top_z` from the patch's vertex height).
- **Why:** the slab top was hard-wired at z = 0 while `260921_motion_sample`
  exports its ground at −15.55 mm. The goal collision gate then reported the
  feet of ground bar B40 (authored to stand on the real ground) 3 mm inside
  the floor at the approach pose (`goal_in_collision`).
- **Effect:** B40 passes the goal gate (its transfer still has no path: start
  found, no clear corridor, BiRRT 5 x 120 s failed — open). The other 15 bars
  of the batch stay valid (the old floor was stricter).

## 2026-09-24 — batch runner + base heuristic for base-less exports (tooling)

- `scripts/derive_m1_headless.py --bar all --plan --no-hide-built`; bars whose
  export leaves the base at the origin are placed with the code's own
  heuristic (`derive_seed_base` + `solve_chain_with_base_search`), sidecars
  `B<n>__J/__R.solved_keyframe.json`. Batch on `260921_motion_sample`: 15/16
  bars planned, 13 straight from a corridor (1–39 s), B9 and B15 via BiRRT
  (397 s / 182 s), B40 open (see above). Table:
  `recorded_data/m1_derive_runs/batch_20260924-002242_260921_motion_sample.md`.

## 2026-09-23 — BiRRT instrumentation (no behaviour change)

- **Where:** `core.py` `plan_pose_birrt` / `update_debug_tree` — records the
  outcome (`connected` / `max_time` / `max_iterations` / start or goal in
  collision), attempts, iterations, both trees (subsampled), closest gap, and
  the stop reason of every extension (reached / collision / ik_failure /
  continuity, plus the connect/stitch variants); `api.py` forwards it as
  `info["profile"]`, `info["planner"]`, `info["path_poses"]`. The dashboard's
  RRT card reads it.
- **Why:** a start that derives fine but never plans was invisible; now the
  extension-stop mix says whether the search dies on collisions or on branch
  flips (260716 B3 `--anchor back`: 53 % collision / 40 % continuity / 6 % IK).

## 2026-09-23 — authored goal kept when it walks home (goal-branch probe)

- **Where:** `api.py` `_derive_constrained_start_for_plan` —
  `GOAL_BRANCH_PROBE_S = 8.0`; `_planning_context` stamps the goal (authored
  vs used, re-branch distance, probe result) on every returned `info`.
- **Why:** ssik's goal re-pairing moved 260716 B3's authored goal by up to 215°
  on one joint onto a branch from which 722 of 728 walks flipped; the sweep
  then spent its whole 120 s. With the authored goal kept (it walks home
  continuously), the same bar found a corridor in 26 s (derive) / 30 s (plan,
  64 waypoints) at the authored base. Bars without an authored goal
  configuration (goal given as EE frames) still go through the pairing.

## 2026-09-23 — start-derivation trace (tamp commit `02ea290`)

- **Where:** `core.py` `derive_constrained_start_tracked` — `_track_to` returns
  why a walk broke (`ik_miss` vs `branch_flip` with the worst joint and jump);
  one record per candidate (variant, anchor, offset, home pose, how far it
  got, outcome, collision pairs for blocked / arrival-collision homes,
  subsampled joint track); run-level `variants_mb`, `sweep_params`, `profile`
  (IK / collision-check time and counts, per-anchor spend, budget cuts).
- **Why:** the console counters could not say where 120 s went. First real
  reading (260716 B3): 747 candidates, 728 track breaks of which 722 branch
  flips — the goal branch, not reachability, was the problem (led to the probe).

## Before this log (from git, for orientation)

- 2026-08-12 tamp `80dfeda` — selectable home carry anchors (horizontal /
  vertical / back), coarse-screened start tracking; teleop `49357da` home-anchor
  GUI selector. 120 s budget split per anchor.
- 2026-08-11 tamp `5e0b4fd` — CLI-tunable planning resolutions
  (`CDFM_POSITION_RES` 0.01 m, `CDFM_ROTATION_RES` 0.025 rad in the monitor).
- 2026-08-03 tamp `93ad6eb` — ssik ≥ 4.1 native in-process IK (no sidecar).

## Open items

- Half-turn walks (goal roll 180° from the canonical home): the slerp direction
  comes from floating-point noise and changes with the base pose. A fix must
  pick the direction without doubling the walks — e.g. by the smoother first
  few IK steps, or by making the canonical home roll follow the goal's — and
  be verified on 260716 B3 at both the authored and the live base.
- A walk that arrives at home in collision (`arrival_collision`) is discarded
  entirely; it could feed the "partial" fallback.
- The sweep exits early only on a corridor, so a scene with no clear corridor
  always burns the full budget before the BiRRT starts.
- In the live monitor (GUI PyBullet client) one IK solve costs ~6.2 ms vs
  3.3–3.8 ms headless, so each anchor's 40 s share covers ~40 % fewer
  candidates.
- B40 of `260921_motion_sample` (ground bar): start found, no corridor, BiRRT
  fails; probably a base-placement question.
- The tamp headless planner (`scripts/headless_bar_action_planner.py`,
  `match_role`) still only knows the legacy single-file action ids.
