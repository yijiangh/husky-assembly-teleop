# CDFM refinement: loading poses the robot can drive in with, strict contacts on the carry, dense validation

Source: `Shared Work Notes/0_Projects/2025 Husky assembly/cdfm refinement 20-11-2026.md` and its eight screenshots of
`seq_1120`. All work is in the tamp submodule (`external/husky_assembly_tamp`, branch `yh/cdfm-planner`); runs on the
mock data root as before.

## Context

**Issue 1, loading pose cannot drive in** (B38, B41, B42, B53, B54, B55, B56 in the screenshots). The robot takes the
loading arm posture at the operator's station, gets the bar, and keeps that posture while it drives to the assembly
base pose. A loading pose that is only free once the robot already stands inside the structure is useless. New
filter for every loading pose the carry starts from (precomputed, rederived, or reached over waypoints): a
collision-free base path must exist from the station to the action's base pose, arms and bar frozen.

**Issue 2, path leads to collision** (B43, waypoint 38). Measured in the planner's own scene:

- It is not resolution. Tool `AT3R` is up to 18 mm inside `joints/J40-43_female` (the receiver on built bar B40) at
  waypoints 34 to 38, and compas_fab skips that pair. The core lets a pending mate's parts touch during the whole
  movement (process format 8.4: tool and receiver, the two halves, each half and the other's bar).
- 5 of the 34 saved paths cut through such a body (B41 worst: bar 40 mm into `joints/J26-41_female`; also B35, B37,
  B50 into `*_mocap` bodies). No path collides between waypoints at 6 joint-interpolated points per step.
- At the carry's end the halves do not overlap, but on B41, B43, B53, B55 their collision hulls are only 0.008 mm
  apart; the other 30 bars have more than 2 mm.

**Decisions taken with you**

| Question | Decision |
|---|---|
| Where the loaded drive starts | A station pose option, x y yaw in the process's world frame |
| How strict the drive check is | Geometry only: Jakob's hybrid A* and 3D collision check copied into tamp, monitor's 10 cm margin relaxed at the ends, no follower-tracking checks, no crl-husky code |
| Mate contacts during the carry | Strict, no allowance at all, in tamp only; the core stays as it is |
| Dense validation | Added as you asked, with the strict rule |

## Open point to settle in step 0: the station pose

The default we agreed (Cindy's park pose, 3.1, 1.35, yaw 90 deg) collides in the process scene: `obstacles/env157`
(the cabinet) spans y 1.63 to 2.23 there, about 0.4 m off the lab frame, and the pose is only 1 to 2.5 m from the
carry bases. Step 0 probes free floor in two or three bar scenes with the 18 carry postures, and I come back to you
with a top view and two or three candidate stations before step 4 uses one. Steps 1 to 3 do not depend on it.

## Part A: strict contacts on the carry (issue 2)

- New `motion_planner/contacts.py`: `strict_carry_state(state) -> (state copy, pairs removed)`. What rides on the arms
  (bodies passing `sampling.is_carried`, the arm tools) may not touch what stands. It prunes `touch_bodies` in both
  directions between those two sets and keeps everything else (`touch_links`, carried-to-carried, carried-to-tool,
  other robots on standing bodies).
- `api.plan_constrained_dual_arm(..., strict_contacts: bool = True)`, applied next to `floor`, so the search, the
  workers, the smoothing and the rounding all see it. Workers need no change: `scenes.build_scene_cell` copies
  `touch_bodies` and the scene cache key hashes them. Roadmaps and tracking files stay valid (the precompute scene has
  no standing bodies).
- `benchmark/follow_on.py` (insert and retreat) is untouched: those movements need the allowances and build their own
  states.
- When the goal comes out unholdable, the wrapper explains it: if every hit pair is one that was pruned, the failure
  reason is `goal_touches_pending_mate`, naming the pair and saying the insert offset in the export is too small. This
  is how B41, B43, B53, B55 will show if the 0.008 mm gap flips.
- ! Consequence: default on means the monitor's held transfers become strict at the next submodule bump, and those
  four bars may then fail there at the goal (the monitor passes the authored goal joints). Escape:
  `strict_contacts=False`. Say so if you would rather have it off by default.

## Part B: dense validation of the returned path

- New `motion_planner/dense_check.py`: `dense_check(confs, collides_many, step_rad)`, built on the existing
  `path_validation.refine_joint_path_for_validation` and its 0.1 deg step: joint interpolation between waypoints,
  every point collision-checked with the strict state.
- `plan_on_roadmap(..., keep_pool=None)` hands its worker pool to the caller instead of closing it, so api.py
  validates the final path in the pool (`online.check_confs`) after smoothing and rounding.
- Order tried: rounded path, then smoothed, then the raw route; the first that passes is returned and
  `info["dense_check"]` says which. If none passes: `failure_reason = "dense_check_collision"` with the first pair.
  `timings["dense_check_s"]` is its own phase.
- `scripts/prm_check_solved.py`: applies `strict_carry_state` itself (it rebuilds the scene from the process) and
  adds the dense check as a sixth item counted in `ok`. It would then have failed the five old paths.

## Part C: the base planner copied into tamp

New package `husky_assembly_tamp/base_planner/`, each file headed with its source file and monitor commit, kept as
close to the original as possible so it can be re-synced (about 1,750 copied lines):

| New file | From (monitor) | Cut |
|---|---|---|
| `collision.py` | `world/base_collision.py` | self-hit, path-hit and naming helpers |
| `world.py` | `plugins/base_planner/world.py` | `stowed`, GUI, `RobotConfig` |
| `path.py` | `plugins/base_planner/path.py` | all but `steer`, `densify` and their constants |
| `support.py` | `planner_search.py`, `algorithms/base.py`, `planner_plugin.py`, `clearance.py` | keeps `PlanResult`, the steps, `kept_margin`, `end_margins`, `BASE_PLAN_DISTANCE`, a do-nothing `Debug` |
| `lattice.py` | `algorithms/lattice.py` | the viser class; follower spread and model replaced by one fixed turning-centre offset; `follows` becomes a geometric recheck |

Entry point: `plan_drive(world, own, start, goal, margin=BASE_PLAN_DISTANCE, time_limit=8.0) -> PlanResult`.
`roomier_goal` is left out on purpose (it moves the target, and the carry is solved for the exact base).

## Part D: the drive-in filter (issue 1)

- New `benchmark/drive_in.py`, class `DriveIn` modelled on `FollowOn`: built once per bar in the main process with
  its own PyBullet world from `carry_scene(process, bar_id)`; `__call__(conf12) -> ""` or the reason; `summary()`.
  - Scene preparation: the process's ground sheet is disabled (only wheels may touch it, so the 10 cm margin would
    refuse every pose) and so are bodies above the robot's height (the planner's floor map keeps only the top
    surface); built bars, obstacles and the other robots stay obstacles.
  - Cheapest first: verdict cache per configuration; re-check of drive paths already found for this bar with the new
    arms; station or goal in collision; boxed in at the goal (no first move free); then the search.
  - "No path" is a time-out, not a proof: each refusal records `timed_out`, and the limit is tuned in step 4.
- `plan_on_roadmap(..., start_accept=None)`, the mirror of `goal_accept`: asked on a found route's first node, cached;
  a refused start leaves the start set, is listed in `info["starts_rejected"]`, and the search goes on. Starts
  reached over waypoints are covered, since the route's first node is always a loading pose. An empty start set gives
  `prm_starts_rejected`.
- `walk_in.walk_in_starts(..., start_accept=None)`: the check sits after the ranking and before the walks join the
  roadmap, up to 6 asks per class; a refused pose is dropped for every goal posture. `derived_starts.json` only ever
  gets poses that passed (marked `drive_in: true`); the existing files on the mock root are deleted before the run.
- CLI `plan_process_carry.py`: `--station X Y YAW_DEG`, `--no-drive-in`, `--drive-time-limit`; a loud line at start
  when the station itself collides. API: only the `start_accept` argument, so the monitor is unaffected until it
  opts in.
- Recorded: `timings["drive_in_s"]`, and `plan_info["drive_in"]` with the station, the drive path's points, length,
  time, asks, and the refused loading poses with their reasons.

## Part E: the page

`prm_plans_page.py` and `plans-viewer.js`: the drive-in path drawn as a line on the floor of a bar's 3D view (moved
into the robot's base frame, which is the page's frame), a "drive-in" row in the bar's stats with the refused loading
poses, `drive_in_s` and `dense_check_s` as phases in both time charts, the dense check's result in the check panel,
and a drive-in column on the front page.

## Not in this plan

- The monitor: its guide still drives first and loads at the assembly pose. Wiring the station, the loaded drive
  and the fallbacks into the monitor is a separate job.
- A bar for which no loading pose passes the filter fails with `prm_starts_rejected`; I report those bars and we
  discuss them rather than loosening the filter.

## Order of work, each step verified before the next

| Step | Work | Verified by |
|---|---|---|
| 0 | Task note `tasks/2026-11-20_cdfm_drive_in.md`; probe station candidates | Top view and candidates brought to you |
| 1 | Part A | B43 replanned: nothing through `J40-43_female`; B41, B43, B53, B55 plan or fail naming the pair |
| 2 | Part B | Checker fails the 5 old saved paths, passes fresh ones |
| 3 | Part C with tests | Tests: free line is direct; a post forces a detour only with the bar held; bar inside a post refused at once; boxed in refused without a search |
| 4 | Part D | One first-search bar, one rederived bar, one waypoint bar; seconds per ask read off, limit tuned |
| 5 | Part E | Headless Chrome on those bars |
| 6 | All 34 bars cold on 16 workers (new set `seq_1121`, mock root), checker, page | Every solved bar passes the checker; results table against `seq_1120` |

Tests added to `benchmark/tests/` (none depends on machine speed): `strict_carry_state` prunes and keeps the right
pairs; `dense_check` with a fake check that collides only between two waypoints; the accept bookkeeping shared by
goal and start; `walk_in_starts` with a start_accept that refuses everything yields no starts; the base planner cases
above with a box robot from a URDF written to `tmp_path`. The tamp suite and the monitor's planner tests are run
before any commit; nothing is committed or pushed without you asking.

## Progress log

### 2026-11-20, steps 0 to 4 done, clean pass `seq_1121` running

- **Station:** A = (3.0, 0.5) m, yaw 270 deg (facing -y), process world frame; chosen by the user from the top view
  (`station_top_view.png` in the session scratchpad). It is the pose nearest Cindy's park pose that is free for all 34
  loading postures of `seq_1120`. No default is baked in: `plan_process_carry.py --station 3.0 0.5 270`.
- **Strict contacts (step 1):** B41, B53, B55 plan and pass the checker; their 0.008 mm end gap does not flip to a
  collision. B43 and B37 no longer find a route (best one left: 64 and 62 mm from holding). At B43's goal the halves
  of both mates sit face to face, so 16 of 32 probe moves away from the goal hit the receiver at once; the old path
  left through the receiver.
- **Dense check (step 2):** 4 to 13 s per plan in the 16 workers (0.1 deg per joint, 500 to 1000 points). The
  checker's own dense check runs in one process, 60 to 130 s per bar. The new checker fails the old B43 path (tool in
  `joints/J40-43_female` from waypoint 34) and passes the old B38 path.
- **Base planner copy (step 3):** `husky_assembly_tamp/base_planner/` (collision, world, path, support, lattice), from
  monitor commit b75b9975; 10 tests. One addition to the copy: `Settings.one_way_no` (a search direction that tried
  every pose it can reach answers for both; off by default, on in `plan_drive`).
- **Drive-in filter (step 4):** B21 keeps its precomputed horizontal pose (3.6 m drive path, 1.2 s). B38's old pose is
  refused; it now starts from a vertical pose at [0.7, 0, 0.7] m with an 8.1 m drive path. A refusal that is definite
  takes 1.5 to 5 s, one that runs out of time 13 s (limit 8 s per direction); B38 spent 132 s of 321 s on 17
  refusals. A loading pose refused once is dropped with all its arm postures (`START_POSE_REFUSALS = 1`).
- All seven bars of issue 1 are refused with their `seq_1120` loading postures (implementer's sweep at limit 5 s:
  22 of 34 old postures can drive in; B172, B45, B49 need a longer limit; the rest time out or are definite).

### 2026-11-20, step 6: the clean pass `seq_1121` (mock root, 16 workers, station A)

All 34 bars cold, strict contacts, dense check, drive-in filter. 27 bars plan, and all 27 pass the checker (dense
check included). `seq_1120` planned 34 of 34, but without these three rules.

| | `seq_1120` | `seq_1121` |
|---|---|---|
| Bars with a path | 34 | 27 |
| Median plan time, the 27 bars that plan in both | 90 s | 115 s |
| Slowest bar with a path | 291 s | 626 s (B54) |
| All 34 bars, one after another | 70 min | 129 min |

Where the 27 start: 16 from a precomputed pose on the first search, 8 from a rederived pose, 3 over waypoints (B42,
B44, B54). Drive paths are 2.4 to 10.0 m long. Of the seven bars of issue 1, B38, B42 and B54 now start from a
vertical pose that can drive in (drive paths 8.1, 8.1 and 6.0 m).

The seven bars without a path:

| Bar | Result | What happened |
|---|---|---|
| B37 | `prm_no_route`, best 62 mm | Strict contacts. 3 loading postures refused (all definite); starts existed. |
| B43 | `prm_no_route`, best 64 mm | Strict contacts: the goal sits between the receivers of two mates. Starts existed. |
| B41 | `prm_no_route`, best 60 mm | Planned under strict contacts alone (134 s). All 10 loading postures asked were refused, 7 by time-out. |
| B53 | `prm_no_route`, best 62 mm | Planned under strict contacts alone (67 s). 8 refused, all by time-out. |
| B55 | `prm_no_route`, best 62 mm | Planned under strict contacts alone (247 s). 8 refused (definite), 35 starts that can drive in, no route from them. |
| B56 | `prm_no_route`, best 62 mm | 6 refused (2 by time-out), 1 start that can drive in, no route from it. |
| B50 | `dense_check_collision` | Same route as in `seq_1120`. The bar touches `joints/J46-50_mocap` between two waypoints on the rounded, smoothed and raw path. |

Time over the 34 bars by phase: search 1745 s, waypoint rounds 1668 s, walk-in 1456 s, smoothing and rounding
1199 s, drive-in 848 s (81 refusals, 39 of them by time-out), dense check 366 s.

Found while reading the page, fixed after the pass:

- `DriveIn.summary` looked the plan's first waypoint up by a rounded key, so B34's drive path was not recorded
  although its posture had passed. It now finds the accepted posture by distance; `summary` also gives `accepted`.
  B34 was planned again into `seq_1121` for its record (72 s, 152 waypoints, drive path 3.9 m; the first result,
  78 s and 156 waypoints, is kept in the session scratchpad).
- The page said "no loading pose was found that the robot can drive in with" on every failed plan. It now gives the
  counts of refused postures and time-outs.

### 2026-11-20, after the pass: three diagnoses of the failing bars (sets `dev_b50`, `dev_limit30`)

- **B50, the dense-check failure is a graze.** The refused path kept and measured: the bar is 0.13 mm from
  `joints/J46-50_mocap` at waypoint 145, 0.69 mm inside it between waypoints 145 and 146 (7 of 12 dense steps in
  contact), and 0.76 mm clear at waypoint 146. The saved waypoints themselves are free. The planner keeps no
  clearance from standing bodies, and a dense failure ends the plan instead of searching again.
- **B41 and B53, the refusals are not time-outs.** With `--drive-time-limit 30` both still fail with `prm_no_route`
  (916 s and 1035 s on 12 workers). B41: 1 posture accepted, 9 refused (6 definite, 3 by time-out), no route from the
  accepted one. B53: 0 accepted, 8 refused (7 definite). A definite refusal means the search back from the bar's base
  pose tried every pose it can reach: with the bar in that posture the robot is boxed in there.
- **What boxes the robot in.** At the base pose the loaded robot is 1 to 9 cm from built parts (B41: `bars/B40`;
  B53: `bars/B52`, `joints/M39-T20-1_mocap`; B55 and B56: `bars/B39`, `bars/B40`, `bars/B168`), and the moves away hit
  `bars/B45`, `bars/B44` and the support robots where the process places them (`belle` on B37, B41, B53, B55, B56;
  `alice` on B43).
- **Only six postures per class are asked.** `WalkSettings.drive_tries = 6`. B53 had 54 class-grid poses free all the
  way and 114 arriving free; six were asked, all refused. The ranking that picks them knows nothing of the drive.

Open, for the user: more asks per class or a ranking that favours compact postures; a clearance margin on the carry
and a new search after a dense failure; whether the support robots stand where the process puts them while Cindy
drives in; and whether B37 and B43, boxed in by the receivers under the strict rule, need a change in the design.
