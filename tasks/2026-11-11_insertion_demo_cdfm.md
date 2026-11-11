# Insertion demo: the bar carried from a loading pose (CDFM, roadmap planner)

Status (2026-11-11): implemented on `yh/insertion-demo-cdfm` (off `yh/insertion-demo` a9903848, `yh/cdfm-planners`
merged in, tamp at ac08ae7, the core at ed9a3a5); nothing pushed. Run end to end on the simulated Cindy. The saved
carry is on the real Drive's process since 2026-11-11 15:00. Open: the carry on the real robot (below).

## What was asked

A variant of the demo: with the Assembly tab's **Transfer workaround** box unticked, the bar is carried with both
arms (the held transfer, "CDFM") from a saved loading pose (the bar over the robot's back) to the insert's start,
along a path planned once by tamp's roadmap planner (PRM), saved, and replayed forwards and reversed. After a Retreat
the existing **Return** brings the arms back to the insert's start, where the reversed path begins.

Decisions made with the user: the loading pose computed by tamp from its "back" carry pose; with the box unticked
**Cycle** runs the whole loop from the loading pose and back; the work on a new branch in `ros2_ws_next/src`.

## What was built

- **The carry is planned offline and only replayed by the monitor.** tamp's `scripts/plan_process_carry.py` plans a
  bar's carry from a start it derives to the authored goal joints and saves it as the process's solution,
  `solutions/B6_J.json` (the core's format: `B6_J_transfer` with its start joints, end joints and trajectory). One
  file holds the loading pose (the first row) and the path; a changed process or action makes it stale.
  Planning from the Approach button was dropped: the roadmap call takes minutes and cannot be cancelled, the planner
  refuses a live start that does not hold the bar consistently, and `ik_goal` may turn the goal by whole turns away
  from the insert's start. `shared/` is unchanged.
- **`plugins/insertion_demo/parts.py`**: `saved_carry` (the core's `read_solutions` and `is_stale`), `load_step` (the
  action's travel to load).
- **`plugins/insertion_demo/plugin.py`**: `Demo.held` (the process plugin's box unticked); the Move button **Load**;
  `_carry` (the saved carry checked once per saved path with `parts.return_problem` on the monitor's model, and its
  end at the insert's start for Insert mm), `_plan_carry` (shown forwards or reversed, no planner; only shown while
  the arms are over 2° from its first row), `_plan_load` (held: the carry reversed; free: the travel to load planned
  to the carry's first row, joints exact); the held cycle and its preview with the carry at both ends; the chip
  `bar held` / `bar free`; a planned move is dropped when the box flips.

| Button | Box ticked (free, bar hidden or tools empty) | Box unticked (bar held) |
|---|---|---|
| **Approach** | free move from anywhere to the insert's start, as before | the saved carry, forwards, from the loading pose |
| **Load** | free move from anywhere to the loading pose | the saved carry, reversed, from the insert's start |
| **Cycle** / **Preview cycle** | as before | from the loading pose: carry, insert, settle, retreat, Return, carry back |

- `scripts/insertion_demo/sim_drive.sh` keeps `solutions/` in the dry run's copy of the process.
- Tests: `test/plugins/insertion_demo/test_demo_carry.py`. Docs: `doc/lab/plugins.md`, the hall manual's 3.10 (the
  sequence and the planning commands), the dry run manual's 3.13.

## Changed the same day, after the first try in the hall

The user found the ticking and unticking confusing, and wanted to check a transfer before running it.

- **The box is unticked once and stays.** The first Move button is its own button and reads **Transit** (box ticked:
  the free move, the bar hidden) or **Transfer** (unticked: the saved path, the bar held); the steps are named
  `transit` and `transfer`. The Move row is back to five buttons on one line.
- **Load with the box unticked** picks its move by where the arms stand: at the insert's start the saved path
  reversed (the bar held); elsewhere the free move to the loading pose, planned only after a **Release** that ran to
  its end (stalled, or opened for its whole 15 s: the simulated screws never stall open), and no gripper screw seen
  tightening since. Otherwise it refuses, so a held bar is never taken on a free move by mistake.
- **The grasp plots**: Plan plots gets two more plots for a move that holds the bar (`PlanPlots(grasp=True)`,
  `parts.grasp_drift`): one tool0 seen from the other against where it is at the first waypoint, its position (mm)
  and the angle of its x, y and z axis (deg), at every waypoint and halfway between (the old monitor's bar-hold
  drift, tamp's per-axis angles). The largest of each is in the line below the buttons. Shown for Transfer, Load
  along the saved path, Insert, Return and a cycle preview that holds the bar all along.
- **The bar on the preview robot**: on a move that holds the bar the green robot carries the bar and its joint halves
  (`parts.held_bodies` from the insert's start state, `RobotGhost.hold` hangs them on the holding link's frame).
- The cycle is unchanged: box unticked, from the loading pose: transfer, insert, settle, retreat, return, transfer
  reversed; box ticked: insert, settle, retreat, return (tamp's constrained dual-arm linear move). A live CDFM plan
  for the way back was considered and dropped at the user's word.
- Checked in the dry run (box unticked all along): Load refused, Release, Load (free), Execute, Grasp, Transfer with
  the bar on the preview robot and the plots (0.01 mm, 0.002° on the saved path), one whole Cycle.

## What the planning showed (2026-11-11, on the dry run's copy of the process, a mock benchmark root)

- tamp's own back pose (grasp midpoint 0.10, 0, 1.04 m in the base frame, roll 17.27° so both tools face up) is
  holdable for B6 (10 arm postures), and 7 of them are free with the jig. The roadmap (6274 nodes, 5 min to build,
  untracked) found **no route** from it in 12 min (`prm_no_route`, the best one needs 30 mm).
- tamp's search (`--cdfm-backend birrt --cdfm-home-anchor back`) derives a start 0.2 m further back and 0.3 m higher
  (the bar at about -0.17, -0.02, 1.42 m) and finds a direct path from it in under 30 s.
- A second roadmap anchored on that pose (7 min to build) gives a route of 12 nodes within 10 mm in 44 s: the
  **PRM carry, 308 waypoints, saved as the solution**. Both carries start at the same arm joints and pass
  `prm_check_solved.py` (no collision, the grasp between waypoints within 0.05 mm):

| | waypoints | largest joint step | bar travel | joint reversals |
|---|---|---|---|---|
| roadmap (saved) | 308 | 2.1° | 1.04 m | 19 |
| direct search | 172 | 4.0° | 0.68 m | 6 |

- At tamp's default resolution the direct path steps up to 9.9°, over the demo's 5° gate: plan with
  `--cdfm-position-res 0.004 --cdfm-rotation-res 0.01`.
- The roadmap's edge tracking (`prm_pilot.py track`) would take over two hours for this roadmap and was skipped.
- Files: `~/husky_insertion_demo_sim/drive/data_design_study/261104_SRD_insertion/process/solutions/B6_J.json`, both
  plans in `…/261104_SRD_insertion/carry_plans/`; the roadmaps in
  `dev/bench_mock/cdfm_planner_benchmark/prm/261104_SRD_insertion__B6` (tamp's back pose) and `…__B6_v2`.

## Found on the way

- The demo's Save had rewritten the insert's start on the Drive (30 mm since 2026-11-11 11:49); two demo tests
  assumed 15 mm and now read the process's own length.
- First whole-loop Cycle in the sim: the carry back was refused, "joint state is 0.8 s old": checking the 308-row
  carry took the tick in which it was sent. The check is cached per saved path, and the cycle lets a tick pass
  between showing a carry and sending it.

## Verified

- `pytest` (the default set) passes; `test/plugins/insertion_demo`: 53 tests.
- Dry run on the simulated Cindy (own tmux server, domain 73): Load (free), Execute; untick; Approach shows the
  saved carry, Execute; Insert, Retreat, Return; Load (the carry reversed); a monitor restart keeps using the saved
  carry; Cycle runs the whole loop to `cycle done: back at the loading pose`; Preview cycle shows `carry, insert,
  back-out, return, carry back`; Insert mm 40 refuses the carry.

## Left for the hall (with the user)

- The solution is in the real Drive's process folder (`…/261104_SRD_insertion/process/solutions/B6_J.json`), copied
  from the dry run's on 2026-11-11 at the user's request (the two processes were identical; `saved_carry` reads it as
  fresh). ! It is stale after any Save of the demo: plan it again with the hall manual's 3.10 and copy it again.
- The bar rack on Cindy's base is not in the process: the carry over the back is planned blind to it.
- After an insert and retreat the grasp is twisted about 1°; the carry back pulls the arms onto the planned path in
  its first segment. The 2° start check per joint is the only gate.
- The script plans on the process's robot model; the monitor checks the path on its own and refuses over 2 mm / 1°.
- When this branch is pushed: tamp's `yh/holding-prm` first (its ac08ae7 is not on the remote).
