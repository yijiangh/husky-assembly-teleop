# CDFM planner tests: a simulator test and a real-lab test

Status (2026-11-20): design proposed, decisions 1-7 still open; three offline sequence runs done (results below): the roadmap planner with its two fallbacks now plans all 34 bars.
Follows `doc/experiments/experiment_design_guide.md`, stage 1: the agent proposes, the user decides.

Branch: `yh/cdfm-planner` (was `yh/insertion-demo-cdfm`, with `origin/core-port` 7f37ec10 merged in, 309cbc63).
"CDFM" is the held transfer: Cindy carries the bar with both arms from the loading pose to the insert's start. Two
planners plan it: tamp's task-space BiRRT (`birrt`) and its precomputed roadmap (`prm`), chosen in the Assembly
tab's Advanced planning (`RunSettings.plan_cdfm_planner`). Today the lab runs the **Transfer workaround** instead:
the arms travel empty and the bar is placed at the insert's start.

## Decisions the user must make first

1. **Which design problem.** The newest export on the Drive is `261117_2ModulesDemo_order_bracing` (17 Nov, 80
   actions, 34 bars jointed by Cindy, no runs yet); the last one run in the lab is `261110_SRD_B25belle` (76
   actions, 32 bars). The repo's lab scripts still name `261103_SRD`, which is not on this PC's Drive. This note
   assumes `261117_2ModulesDemo_order_bracing`; everything below is the same for the other, only the roadmaps differ.
2. **Where the loading pose comes from.** The process gives the transfer no start joints (`B21_J_load` and
   `B21_J_transfer.start` hold only Cindy's base), so with the workaround off the travel to load is "not planned:
   needs the transfer start" and the transfer plans from wherever the arms are. Open since 2026-11-05. Options:
   - (a) *Recommended.* The planner derives it: the travel to load goes to the start the chosen planner derives
     (the roadmap's carry pose, the BiRRT's sweep; tamp's `derive_start`, `info["derived_start_conf"]`), as
     `plan_process_carry.py` does offline. A code change in `tamp_route.py` and `assembly_steps`, tested in sim.
   - (b) A carry planned offline per bar and saved in the process's `solutions/` (the insertion demo's way). Tests
     the planners offline only; the monitor just replays.
   - (c) The operator jogs the arms to a loading pose by hand. No code, but the start is arbitrary and the test
     then measures the operator as much as the planner.
3. **The Assembly guide.** core-port's new `assembly_guide` plans every arm move with the default search: it
   ignores the CDFM planner dropdown and the run's other Advanced planning settings. Fix it in this work (pass
   `assembly_steps`' search settings), or test in the Assembly tab only?
4. **Pass marks.** The thresholds in the hypotheses below are proposals; change any of them.
5. **Roadmap size.** The report's roadmaps have 6000 nodes with every edge within 15 mm tracked beforehand
   (`NODES=6000 TRACK_MM=15`, the script's defaults); the ones on the Drive for B20–B22 have 600 nodes and no
   tracking, so the planner tracks every edge it tries while the operator waits (B21: 350 s on one worker on
   5 Nov). Proposal: A builds both sizes for B20–B22 first and the better one for the rest.
6. **Corner rounding in the monitor.** tamp's offline script rounds a found path's corners by default
   (`round_corners`, radius 0.3 m); the monitor's held search does not pass it, so its paths keep their corners.
   Pass it in the monitor too, or test the monitor as it is?
7. **Where data goes.** The benchmark's plans and roadmaps on the local mock root first
   (`~/Code/ros2_ws_next/dev/bench_mock/cdfm_planner_benchmark`, not synced), and onto the Drive
   (`data_experiment/cdfm_planner_benchmark/`) only with the user, for the lab PC.

## Goal

Decide whether the lab can run the transfer with the bar held (Transfer workaround off), and with which planner by
default.

## Hypotheses

Each names its deciding number. "Carry" = one held transfer, loading pose to insert start.

| | Claim | Deciding number | Refuted when |
|---|---|---|---|
| H1 | The roadmap plans the carry of every bar Cindy joints, in the scene of that bar's turn (bars built so far, the other robots where the process has them). | bars solved / bars | any bar unsolved with a roadmap of the default size; compare the BiRRT at the monitor's budget (3 tries of 120 s) |
| H2 | An operator can wait for a plan. | planning seconds per carry, one worker as in the monitor | median over 120 s, or worst over 300 s |
| H3 | A planned path holds the bar: the two tools keep their relative pose. | largest change of one tool0 seen from the other along the path, checked on the URDF at every row and halfway (`insertion_demo/parts.py` `grasp_drift`), not by the planner | over 2 mm or 1° anywhere |
| H4 | A planned path is a motion the arms can run. | largest joint step between rows; the duration timed by the arms' limits; the smallest clearance in the monitor's own check | a step over 8°; over 60 s; a collision the monitor's check finds |
| H5 | It still plans from where the robot really stands. | bars solved with Cindy's base off the design base by what the base planner leaves (sim: offsets of 2 cm and 2°; lab: where she stops) | a bar solved at the design base and not at the live one |
| H6 | On Cindy, the carry does not strain the bar or the grasp. | change of each wrist's force and torque over the carry from its value at the grasp (the plots' `|F|`, `|T|`); tool0 error at the end | over 30 N or 5 N·m (the servo's limit is 20 N for a change after an insert); end over 5 mm / 3° (the insert's start gate) |
| H7 | An operator following the manual gets through a bar without help. | per bar: refusals whose text did not say what to do; times the operator had to leave the Assembly tab; minutes from "bar loaded" to "at the insert start" against the workaround's | any refusal without a next step; over twice the workaround's minutes |

Not hypotheses, but checked once each as usability cases (pass / fail, in both manuals where they apply): no roadmap
for the bar (the refusal names the build command); a roadmap built for an older export (says rebuild); Cancel during
a roadmap plan (it has no time limit); the choice kept in `run.json` over a monitor restart; a plan gone stale after
the base moved; the preview shows the bar carried.

## The experiments

| | Question | Hypotheses | What runs | Repeats |
|---|---|---|---|---|
| **A. Benchmark** (offline, part of the simulator test) | Which planner solves which bar, how fast, with what path? | H1–H4, H5 with a shifted base | tamp's `scripts/prm_runs/process_precompute.sh` (roadmaps) and `process_plans.sh` (both planners, one worker, shifted base; every path checked on its own by `prm_check_solved.py`), for every bar Cindy joints | roadmap: once per bar (a query has no randomness; to confirm); BiRRT: 5 seeds |
| **B. Simulator** (fake lab, three simulated robots) | Does the operator's flow work with the workaround off, and does the monitor plan as the benchmark did? | H2, H4, H5, H7, the usability cases | the monitor on B20, B21, B22 and the two hardest bars of A, each from base drive to retreat, with `prm` and with `birrt` | once per bar and planner (the simulator repeats itself), plus the base offsets |
| **C. Real lab** | Does Cindy carry a real bar along these paths? | H5, H6, H7 | Cindy on B20, B21, B22: first each carry with empty tools (the path in the air), then with the bar; `prm` first, `birrt` where it finds a path | 3 carries per bar and planner; a repeat = release, travel to load, place, grasp, carry |

Names keep the data apart: `benchmark_<date>` for A; B and C are assembly runs (`runs/<date>_<time>/`) of the
process, in the scratch Drive for B and the real Drive for C.

## Method

- **Varies:** the planner (and for H5 the base; in A also the roadmap's size). **Held fixed, recorded with every plan:** the process export and
  its hash, the roadmap (nodes, tracking, the tamp and core commits it was built with), the planners' default
  settings, one worker, the PC.
- **Measured independently of the planner:** H3, H4 by the monitor's own model and checks; H6 by the force
  sensors and encoders; planning time by the caller's clock.
- **Unit:** one carry planned from the loading pose in the scene of that bar's turn.
- **Reference:** the transfer workaround (the free move with empty tools) for the same bar: its planning time and
  motion, for H7 and to read H4 against.
- **Checks with known answers:** a path with a tool moved 5 mm must fail H3's check; a bar without a roadmap must
  be refused; the roadmap of another bar must be refused.
- **Effects the system has anyway:** two position-controlled arms on one rigid bar always load it a little; the
  arms' encoders see no bar slip; the fake lab's tools and bar have no weight or force unless `sim_wrench` runs.
- **Failure of the planner:** no route, a path failing H3 or H4, a plan over its time. **Problem of the setup,
  run left out:** e-stop, lost mocap, a stale roadmap, the operator's Cancel.

## What must be built before either test runs

1. The loading pose (decision 2).
2. The guide (decision 3).
3. A failed plan in the run's events (today only an accepted plan writes a `plan` record; a refusal is only a log
   line), so the analysis counts failures from the run, not from the console.
4. The fake lab on another process: `scripts/fake_lab/env.sh` and `drive.sh` name
   `261008_2ModulesDemo_purged_process` and cannot be told otherwise.
5. This PC's setup after the merge: crl-husky is 6 commits behind `jg/ros2` (the base planner's Commit needs
   `1956556` or later), Pillow 9.0.1 is too old for `frame_check`'s picture.
6. Roadmaps for the chosen process's bars (about 34; minutes each at the 600 nodes used on 5 Nov, far more at the
   script's default 6000), on the mock root.

## What the planner does today, as read from the code (2026-11-18)

- `backend="prm"` has no time limit and no retries (`max_time` is only recorded); nothing stops it but its end.
- With `derive_start=True` its start is one of the roadmap's carry poses, returned as
  `info["derived_start_conf"]`; with `False` (the monitor today) the live arms become a new start node.
- It returns `prm_goal_unholdable`, `prm_no_start_free`, `prm_no_route` (with the gap left) or `repair_<cause>`;
  it raises for a missing roadmap, a roadmap of another bar, grasp or tool contacts (`RoadmapMismatch`), a tracking
  file made with other settings.
- The path is checked at the caller's step: 1 cm / 0.025 rad in the monitor (`HELD`), not the report's 2 mm.
- The report audit (what to update in `docs/holding_prm_report.tex`) is in the session's answer of 2026-11-18.

## First sequence run, 2026-11-18 (mock root, before the decisions)

A roadmap per bar for all 34 bars of `261117_2ModulesDemo_order_bracing` (600 samples, 779–1378 nodes, no tracking,
~2 min each on 8 workers), each carry planned in its turn's scene (`plan_process_carry.py`, one worker, start derived
from the carry poses) and checked by `prm_check_solved.py`. Plans and checks:
`<mock root>/prm/plans/261117_2ModulesDemo_order_bracing/seq_1118/` (tag `prm`: goal = authored joints; `prmf`: goal =
flange poses, for the bars that failed with `prm`).

- 8 of 34 solved with the authored goal, 16 of 34 with either goal; 18 failed both (B21 B22 B35 B36 B37 B172 B169 B26
  B40 B38 B41 B42 B43 B44 B52 B54 B56 B55). Every solved path holds within 0.17 mm and 0 saved waypoints collide.
- Authored goal one whole turn off (16 bars): it is a roadmap goal node turned by 360° in a joint (mostly shoulder pan),
  so it joins no node within the 90° endpoint radius and the search ends at once. The monitor unwraps its goal next to
  the start (`unwrap_to_nearest`), which need not match the roadmap either.
- Too sparse: within 10 mm no roadmap but B53 and B55 joins a goal node to a carry pose before the scene; B21 B35 B40
  B47 B52 join none even within 30 mm.
- Time (H2): median 451 s over the 16 solved, 165–1729 s; failures with `prmf` took up to 1865 s. Far over 120 s.
- Floor: B34's path takes the ground joint half 85 mm below the floor (waypoint 280); B20 passes 1 mm above it.
  Neither planner checks the floor.

## Changes after the first run, 2026-11-19 (the user's takes; tamp submodule, uncommitted)

The user's answers to the first run: (1) the authored tool0 poses must be reached, the joints may be resolved by IK
there, as long as the insert and the retreat have collision-free, joint-continuous keyframes from that posture;
(2) 1000 samples per roadmap, parallel collision checks wherever possible; (3) a floor in the planning scene, level
with the robot's base link; then a dashboard of the plans with a slider and the time breakdown.

- `motion_planner/floor.py`: a slab under the robot, top at the base frame's height. `plan_constrained_dual_arm`
  plans with it by default (`floor=True`); the roadmap build's workers load it too. The arms, tools and everything
  carried are checked against it; the robot's other links and the other robots may stand on it.
- `benchmark/follow_on.py` (`FollowOn`) and `goal_accept=` on `plan_constrained_dual_arm` (roadmap backend): the
  roadmap asks about the end posture of every route found; `plan_process_carry.py --goal frames` answers by planning
  the insert (`plan_constrained_dual_arm_linear`) and the retreat (`plan_dual_arm_linear_independent`) from it in
  their own scenes, and saves the joints they end in (`plan_info.json` `follow_on`). `--no-follow-on` turns it off.
- Parallel: the pool sets one maths-library thread per worker (each had ~60, fighting over the cores); the lazy search
  fills idle workers with the edges of the next-best routes (`search.lazy_search(fill=)`).
- `scripts/prm_plans_page.py` + `benchmark/dashboard/plans.html`, `plans-viewer.js`: the page of a plan set.
- Not done: the monitor still plans with one worker (`tamp_route.PRM_OPTIONS`; more workers need a local
  `scene_cache_dir`, the roadmap folder is on the Drive), does not ask `goal_accept`, and passes the authored joints
  as the goal; smoothing and corner rounding run in one process.
- Roadmaps rebuilt on the mock root with 1000 samples (1179-1778 nodes, ~100 s each on 12 workers), the 600-sample
  ones kept in `prm/_n600_1118/`. Plans: `prm/plans/261117_2ModulesDemo_order_bracing/seq_1119/`.

## Second sequence run, 2026-11-19 (mock root, the changes above)

Plans one at a time on 16 workers, goal = flange poses with the follow-on check, floor on, 1000-sample roadmaps;
`seq_1119/` (first pass copied to `seq_1119_first/`). Page: serve the mock root, open
`prm/plans/261117_2ModulesDemo_order_bracing/seq_1119/dashboard/`.

- Roadmap: 21 of 34 bars with a path (19 with the default tolerances 10/20/30 mm and one tube round; B20 and B22
  with `--prm-tolerances-mm 10,20,30,45,60 --prm-tube-rounds 4`). Median 68 s, 50-144 s but B22 457 s; 18 of 21
  within 120 s. Of the solved time: search ~40 %, smoothing and rounding ~45 %, the follow-on check ~4 s a plan.
  No end posture was refused. Every roadmap path stays 43 mm or more above the floor.
- B59's path fails its own check: a collision when re-solved at 2 mm, none at its 1 cm waypoints.
- 13 bars without a roadmap route (B36 B37 B38 B40 B41 B42 B43 B44 B47 B52 B54 B55 B56): the goal postures free in
  the scene are 50-170 deg (largest joint) from every carry pose of `carry/carry_v1/starts.json`; the best bridge
  breaks the holding by ~60 mm. 4000 samples did not help (B36: 62 mm). Not a density problem: the start set.
- The BiRRT on those 13 (120 s a try, 5 tries): 9 paths, 4 none (B43 B44 B52 B54). It derives its start by tracking
  back from the goal: 6 of the 9 carry the bar 3-9 cm only (B37 B40 B42 B47 B55 B56), so they are no test of a
  carry from a loading pose; B41's end posture does not let the insert follow.
- Open: where the loading pose comes from decides this (decision 2). A roadmap whose carry anchors are derived per
  goal posture (tracked back from the goal to a loading pose, as the BiRRT does) is the likely fix.

## Fallbacks for the 13 bars, and the third sequence run, 2026-11-20 (mock root; tamp submodule, uncommitted)

The user's instruction: rederive loading poses for the difficult bars as a fallback (the precomputed poses first,
the working bars unchanged), keeping the three classes (back, front vertical, front horizontal); if that is not
enough, track back from the goal, put that into the roadmap as an intermediate waypoint and connect it to one of the
three classes, a few rounds if needed.

What the diagnosis showed (scratch scripts, 16 workers, each bar in its own scene):

- Joint distance to the carry poses does not separate failing from solved bars (solved B21 looks the same).
- B36: its goal postures reach two of the 18 carry poses by a straight walk, but arrive one whole turn of the
  right shoulder away from the roadmap's node (which is on the turn nearest home); 11 other horizontal poses have a
  straight walk that is collision-free all the way.
- B43, B44, B37: straight walks leave the goal collision-free for 4-16 % of their length (median slide 4-7 cm):
  the arms and the bar sit among built bars, joint halves and a support robot. These need waypoints.
- B52: the insert follows from the authored end joints but not from the same arm posture on the whole turns
  nearest home (three joints 360 deg apart; one then meets its limit in the insert).

What was built (`holding_prm/walk_in.py`, wired into `online.plan_on_roadmap`; off unless `walk_in` is passed,
`plan_process_carry.py` passes it):

1. First search as before: the roadmap's carry poses, tolerances 10/20/30 mm, one tube round. Unchanged.
2. Fallback 1: every goal posture is walked on a straight line (both arms following, 1 cm / 0.025 rad steps, the
   10 deg continuity gate) to the roadmap's carry poses, then to a grid of other poses in the three class boxes
   (0.3 m, 30 deg rolls: 972). Arrivals that are collision-free become starts; the walk's configurations join the
   graph as a chain (each node also joined to its 10 nearest roadmap nodes). Per goal posture and class the best 2:
   free all the way first, then the roadmap's own poses, then the longest free part, then arm comfort.
3. Fallback 2: rounds of waypoints. From each frontier node (the goal postures first) 26 slides of 0.3 m, 6 turns
   of 45 deg on the spot and walks toward up to 8 loading poses per class; the free part of the 4 longest is kept
   as a chain; the next round walks from the 8 new ends that are furthest apart. A search after each round (up to 8).
4. Last: tolerances 45 and 60 mm and one more tube round.
5. End postures are also offered on the whole turns nearest the authored end joints (`goal_turn_reference`), in the
   fallbacks only.
6. A rederived loading pose that gave a path is saved as `<roadmap>/derived_starts.json` and tried first next time
   (B36: 232 s cold, 55 s with the saved pose).

Third run, `seq_1120/` (every bar cold, one plan at a time on 16 workers, default options, checks in the background):

- 34 of 34 bars with a path, 34 of 34 pass their own check (2 mm re-solve, no waypoint in collision). Median 89 s;
  22 within 120 s, 32 within 300 s, slowest B47 385 s.
- 19 from the first search (median 79 s): node for node the same routes as in `seq_1119/`.
- 4 from a precomputed carry pose walked to from the goal (B20 B22 B41 B55; 76-173 s). B20 and B22 needed wider
  tolerances and 4 tube rounds before.
- 8 from a rederived loading pose (47-385 s): horizontal for B36 B38 B42 B54 B56, back for B40 B47 B52.
- 3 over waypoints, each to a vertical pose at [0.7, 0, 0.7] (B37 4 rounds 291 s, B43 2 rounds 225 s, B44 6 rounds 315 s).
- Time over the 34 plans: search 30 %, fallback 1 24 %, fallback 2 8 %, smoothing and rounding 25 %.
- No end posture refused at the end; B41 had 2 refused first. Lowest point of any path 43 mm above the floor.
- Page: `prm/plans/261117_2ModulesDemo_order_bracing/seq_1120/dashboard/` (loading pose per bar, fallback stages).
  Added 2026-11-20 on request: a bar's page charts the joint values and the flange-to-flange position and per-axis
  rotation against the first waypoint, with a cursor that follows the slider; the front page's time chart links
  each bar to its page and marks its loading pose (shape = class, colour = precomputed or rederived).
- Tests: tamp `benchmark/tests` 68 passed, 1 skipped; monitor `test_planning_tamp_route.py` + `motion_planner`
  24 passed, 8 skipped.

Open after this run:

- A rederived loading pose is only ranked by a free straight walk and arm comfort; it was not put through the
  carry search's nudge test (`prm_carry_search.py`), and 4 carries are short (B40 B52 0.13 m, B55 0.11 m, B41 0.21 m).
- The monitor does not use any of this yet (`tamp_route.PRM_OPTIONS`: one worker, authored joints as the goal). On
  one worker the fallbacks would take many minutes; parallel workers need a local `scene_cache_dir`.
- Fallback 1 walks every free arrival again with all collision checks (B47: 418 of them, 241 s); a cap would help.
- Waypoints are not saved: a bar that needed them searches for them again at the next plan.

## Deliverables, in order

1. This note agreed.
2. Items 1–5 above, then experiment A on the mock root: its numbers set the two hardest bars and tell whether the
   pass marks are sane.
3. `doc/experiments/cdfm_planner_sim_manual.md`: A's commands, then B's operator steps in the fake-lab manual's
   format, every Expect line taken from a run of mine in my own simulator session.
4. `doc/experiments/cdfm_planner_lab_manual.md`: C, in the real-lab manual's format, written from B's steps.
5. An analysis script over A's plans and B's and C's runs, and its report, after the first data.
