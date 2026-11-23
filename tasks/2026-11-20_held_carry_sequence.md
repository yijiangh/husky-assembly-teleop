# Held carry in the monitor: load at a station, drive in with the bar, carry planned live

## Context

With the Transfer workaround off, Cindy's J action cannot run today: the process holds no base movement, the
guide drives once (to the assembly pose) before the load, and nothing picks the loading arm configuration (the load's
Plan says "needs the transfer start"). The offline planner can now plan every bar's carry from a loading pose the
robot can drive in with (`seq_1121`, station A: 27 of 34 bars; the other seven wait for decisions in
`tasks/2026-11-20_cdfm_drive_in.md` and have no carry to load from until then). This plan wires that into the monitor as a new movement sequence,
then tests it in the fake lab with a manual, then in the real lab with a manual. Experiment A of
`tasks/2026-11-18_cdfm_planner_tests.md` is dropped; B and C stay. The workaround-on flow is not touched.

Decisions taken (2026-11-20): the two drives live in the monitor's guide, the process stays as exported; the loading
configuration comes from a carry planned offline first (planning it in the monitor comes later); the station pose is
lab data in this repo; the manual gives the guide as the main flow and Assembly tab + Plan › Base as the by-hand one.

## The sequence (workaround off; a J action of bar X, then its R action)

| # | Part | From | Base |
|---|---|---|---|
| 1 | Drive to the station | new guide DRIVE, target = the robot's station pose | moves |
| 2 | Travel to load | `X_J_load` (FREE_TO_LOAD), goal = first row of X's saved carry | still |
| 3 | Place the bar, clamp | `X_J_place_bar`, `X_J_grasp` (by hand, as today) | still |
| 4 | Drive in, bar held | the guide's existing DRIVE to the action's base, moved here; shown movement = `X_J_transfer`, so the bar is attached and the base planner plans with it | moves |
| 5 | Transfer with the bar, servo | `X_J_transfer` (TRANSFER_HELD), planned from the live arms at the live base | still |
| 6 | Insert | as today | still |
| 7, 8 | Retreat, home | R action, as today | still |

The authored order of arm and tool movements is already this one; only the drives and the load's goal are new.
No station configured for the robot, or the workaround on: the guide's parts are exactly today's.

## Step 5 is the risk: planning from a fixed start

The roadmap gives a fixed start none of the fallbacks (`online.py`: `walk_in = settings.walk_in and start_conf is
None`), and about 15 of 34 bars needed them offline. Replaying the saved path is not an option: the base stops
centimetres off, so the bar would end off by that and the collision checks would not hold.

Fix, in tamp: a saved carry seeds the live plan.

- Saved path `Q = [q_0 .. q_n]` (12 joints a row), planned at the authored base. Every row holds the bar at any base
  (the grasp constraint is in the robot's frame); only its collisions and its end pose depend on the base.
- `prm_options["seed_path"] = Q`: thinned with `walk_in.chain_of` and added with `walk_in.add_chains` (each node
  joined in a row and to its 10 nearest roadmap nodes), before the search. Start = the live arms (a new node, as
  today, which lands next to `q_0`).
- Goal = the tool0 frames in the world (`goal_ee_frames`), solved at the live base; `goal_turn_reference = q_n`, so
  the end posture is the one whose insert and retreat were checked offline. The goal nodes join the chain's tail
  through the roadmap as any node does; every edge is tracked and collision-checked in the live scene.
- If that finds no route: the waypoint rounds (`via_round`) from the goal postures toward the chain, which today
  only run without a fixed start. Whether this is needed is read off the offline test below.

## Work, in order (each verified before the next)

**0. Task note.** This file; the status line and deliverables of `tasks/2026-11-18_cdfm_planner_tests.md` updated
(A dropped, decisions 2 and 3 answered).

**1. tamp, offline (submodule `external/husky_assembly_tamp`).**
- `plan_process_carry.py --save-carry`: writes `<roadmap_dir>/carry.json` (the path, the base it was planned at, the
  station, the end joints of insert and retreat, tamp commit); `--start-from <carry.json>`: fixed start = its first
  row, the path as seed. `api.plan_constrained_dual_arm` / `online.plan_on_roadmap`: the `seed_path` option.
- Offline test of H5 before any monitor code: every bar of `seq_1121`, `--start-from` its own carry with
  `--base-shift` of 2 cm / 2° in four directions and 5 cm / 3° in two, 8 workers. Report bars solved and seconds;
  bars that fail are brought to you, not patched around.
- Tests in `benchmark/tests/`: the seed's chain joins the graph; a fixed start with a seed finds the route a derived
  start found.

**2. Monitor.**
- Station: `data/lab/main.json` gets `"stations": {"0806": [3.0, 0.5, 270.0]}`, read in `world/lab.py`;
  `plugins/base_planner/panel_plugin.py` gets a "Station" load button beside "Park" (a fixed button, nothing jumps).
  ? First check that (3.0, 0.5) is free in the monitor's lab obstacles too: the process's cabinet sits about 0.4 m
  off the measured one.
- Saved carry: new `shared/assembly_planning/carry_plan.py` reads `<roadmap root>/prm/<process>__<bar>/carry.json`
  and refuses with the build command when it is missing, or was planned for another station or base.
- `shared/assembly_planning/tamp_route.py`: FREE_TO_LOAD without a goal takes the saved carry's first row (the gate
  at `plan_movement` and `_free_to_load`); `held_search` with `prm` passes `goal_ee_frames`, `goal_turn_reference`,
  `seed_path`, several workers and a local `scene_cache_dir` (nothing is written into the roadmap folder). A warning
  when the live arms are not at the loading configuration.
- `plugins/assembly_guide/parts.py` `action_parts`: with the workaround off, a station for the robot and a
  TRANSFER_HELD step, the station DRIVE goes first and the action-base DRIVE goes before the transfer's step with
  `movement = X_J_transfer`. `plugin.py`: `_later_ran`, `at_end`, `_flag_drive_before` with two drives in an action.
- Arms watched during the loaded drive: `BasePlannerPlugin.execute` freezes the robot's own arm joints while it
  holds a body (`ctx.freeze(take_basis(...))`, as the executor does for the other robots).
- The guide passes the run's Advanced planning settings to its arm plans (decision 3).
- A refused plan is written to the run's events (item 3 of the test note).
- Fake lab on another process: `scripts/fake_lab/env.sh` and `drive.sh` take `FAKE_LAB_PROCESS`; `drive.sh` also
  copies the bars' roadmaps and carries from the mock bench root into the scratch Drive.
- Tests: guide part order on and off (`test/plugins/assembly_guide/`), the carry reader, the load's goal
  (`test/shared/test_planning_tamp_route.py`), the station in `test/world/`.

**3. Simulator run and its manual.** My own fake-lab session (private tmux server, domain 72, port 8097, scratch
home), `261117_2ModulesDemo_order_bracing`: B20, B21, B22 and two hard bars chosen from `seq_1121`, each from the
station drive to the retreat with `prm`, then `birrt` on the transfer; base offsets for H5; the usability cases of
the test note. Then `doc/experiments/cdfm_planner_sim_manual.md` in the fake-lab manual's format, guide flow first,
Assembly tab + Plan › Base flow once; every Expect line from that run.

**4. Real-lab manual.** `doc/experiments/cdfm_planner_lab_manual.md` from the sim steps, in the real-lab manual's
format: empty tools first, then the bar; force and torque watched during the loaded drive too.

**5. Later, not in this plan.** The monitor planning the carry ahead itself when no `carry.json` exists; drives as
authored base movements in the process.

## Open risks

- The monitor's base planner keeps a 10 cm margin and may move the target for room; tamp's drive-in check relaxes
  the margin at the ends. A drive-in tamp accepted may be refused in the monitor: seen in step 3, discussed then.
- A live plan that fails with the bar already clamped: the operator's ways out are the BiRRT in the dropdown, or
  driving closer to the design base. The manual says so.
- Nothing is committed or pushed without you asking; nothing is written under the real Drive.

## Verification

- Step 1: the shifted-base table for 34 bars; tamp `benchmark/tests`.
- Step 2: the tests named above while iterating; `pytest` (default set) and tamp's suite before any commit.
- Step 3: the run folder of my simulator session (plans, events, marks) and the page checked in headless Chrome.

## Progress log

### 2026-11-20

- Plan approved; step 0 done.
- **Step 1 (tamp, uncommitted).** `holding_prm/seed.py` and `seed_path` in `plan_on_roadmap` / `prm_options`: the
  saved carry's rows join the graph as a chain from the fixed start to the goal postures within 30 deg of its last
  row. With `seed_base_planned` and `seed_base_live` the rows are first solved again so that the bar keeps its planned
  path in the world (the base's offset is taken up over the first half of the bar's travel). `benchmark/saved_carry.py`
  and `<roadmap>/carry.json`; `plan_process_carry.py --save-carry` and `--start-from`; `scripts/prm_save_carries.py`
  kept the 27 `seq_1121` plans as carries on the mock root. The "goal touches the pending mate" label is kept for
  failures at the goal only. Tests: tamp `benchmark/tests` 75 passed, 10 skipped.
- **Shifted-base test, first pass (2 cm, 2 deg, seed in the robot's frame, set `seq_h5` tag `a`):** 20 of 27 bars plan
  from their carry's first row, 27 to 80 s on 8 workers. Not planned: B34 (the carry's end posture is not free at the
  shifted base: the arm would have to change branch 5 rows before the end), B40 (no collision-free end posture at
  all; at the planned base 2 of 32), B54 and B59 (a seed edge collides; both plan with the base unshifted, B54 with
  2 cm alone, not with 1 deg), and B35, B47, B48 where the carry is found but the follow-on check refuses the retreat
  from its end. Solving the seed again in the world frame did not change B54, B59 or B34. Full test (six shifts,
  follow-on refusals planned again without the check) running: `seq_h5` tags `p1`-`p4`, `q1`, `q2`.
- **Step 2 (monitor, uncommitted).** Stations in the lab file (`Lab.stations`, `station_of`; Cindy at 3.0, 0.5, 270
  deg) and Plan > Base's Station button; `shared/assembly_planning/carry_plan.py`; the travel to load ends on the
  carry's first row (motion planner and the guide's arm planner); the roadmap transfer is seeded, with the goal solved
  next to the carry's end; the guide's two drives (`GuidePart.carrying`, `BasePlanRequest.arms_still`: a freeze on the
  robot's own arms); `run_search` shared by the Assembly tab and the guide; `plan_failed` events; `FAKE_LAB_PROCESS`
  and `FAKE_LAB_BENCH` for the fake lab; `doc/lab/plugins.md`, `doc/design/run_record.md`.
- This PC: crl-husky fast-forwarded from 23c74c7 to `jg/ros2` 15d61ab and the workspace rebuilt (the base planner's
  Commit needs 1956556). Default tests: 2671 passed; the 9 `frame_check` tests fail on Pillow 9.0.1 (too old).
- **Shifted-base test, pass `p1` (2 cm, 0, 2 deg; the seed solved again in the world frame):** 20 of 27 plan with the
  follow-on check; B35, B47, B48 plan once that check is left out (it refuses the retreat from their end at the
  shifted base); B34, B40, B54, B59 do not plan.
- **Shifted-base test, all six shifts (27 bars each, 8 workers; `seq_h5` on the mock root).** 139 of 162 plans are
  found, 12 of them only without the follow-on check (`*`). 14 bars plan at every shift.

  | Shift (x, y in cm; yaw in deg) | Planned | of these `*` | Not planned |
  |---|---|---|---|
  | `p1` (+2, 0, +2) | 23 | 3 | B34 (goal far from the seed), B40 (goal not holdable), B54, B59 (no route) |
  | `p2` (-2, 0, -2) | 27 | 3 | |
  | `p3` (0, +2, +2) | 22 | 2 | B35 (goal not holdable), B40, B54 (no route), B42, B59 (the dense check finds a collision) |
  | `p4` (0, -2, -2) | 26 | 1 | B26 (dense check) |
  | `q1` (+5, 0, +3) | 19 | 0 | B34, B48 (goal far from the seed), B35, B40, B47, B52 (goal not holdable), B54, B59 (no route) |
  | `q2` (-3.5, -3.5, -3) | 22 | 3 | B25, B40, B52, B169 (goal not holdable), B44 (no route) |

  - Plan times: median 40 to 45 s, at most 102 s (`p3` to `q2`). `p1` and `p2` took about twice as long (median 88
    and 74 s), probably because my simulator ran next to them.
  - The sign of the offset matters: turned 2 deg to the left 9 plans fail, turned 2 deg to the right 1.
  - "Goal not holdable" (11 cases) is no planner limit: at that base no posture holds the bar at the insert start
    free of collision. Those need the base closer to its design pose, whatever plans the carry.
  - B40, B54 and B59 fail at three or four of the six shifts; B34, B35 and B52 at two.
- **Step 3, first simulator run (my fake lab: tmux `cdfm_sim` on my private server, domain 72, page 8097, scratch
  home `~/Code/ros2_ws_next/dev/cdfm_sim`, run `runs/20261120_191320`).** B20_J in the guide with the workaround off
  and `prm`, in order: drive to the station, travel to load (arm planner, 30 s motion, to the carry's first row), bar
  placed and clamped (Done), drive in with the bar (base planner 2.23 m, 34 s; ended 0.1 cm / 0.3 deg from the
  base; the arm watch did not trip), transfer with the bar (roadmap seeded by `carry.json`: 154 waypoints in 95 s,
  17 s motion, end within 0.02 deg), servo (tool0 0.13 / 0.07 mm from the authored insert start), insert (rigid,
  done). Found and fixed on the way:
  - tamp's worker processes looked the bar up under the roadmap's name, not the live scene's (`process/bars/B20`):
    `open_pool(active_bar_id=...)`. Several workers had never run from the monitor.
  - The goal check solved the transfer's goal next to the authored joints, another arm branch than the carry's end
    (2.97 rad apart): no route. `Route.ik_goal` now takes the saved carry's end by itself for a roadmap transfer, and
    `check_goal` gets the run's search settings (guide and Live IK).
  - Seeded plans print tamp's progress lines into the monitor's log; a failed roadmap plan names its edge verdicts.
- **Open, for the user:**
  - **The station.** At (3.0, 0.5, 270 deg) the bar on the arms reaches into the lab's border wall at x max (B20
    1.11 m to the robot's left; over the 27 loading postures the bars reach 1.15 m left, 1.57 m right, 0.93 m
    ahead): the drive in is refused at its start. The simulator run used (2.3, 0.2, 270 deg), now in
    `data/lab/main.json`, as a trial. The offline carries were checked for the drive in from (3.0, 0.5): the monitor
    warns when the robot loads more than 0.3 m from there. tamp's drive-in check does not know the lab's border or
    measured obstacles.
  - **Bars that do not replan at a base 2 cm / 2 deg off** (B34, B40, B54, B59) and the three whose retreat check
    fails there; the seven bars without a carry (`tasks/2026-11-20_cdfm_drive_in.md`).
  - After a monitor restart a finished drive counts as "to do" until a later part ran: the next part asks twice.

### 2026-11-20, a station that is free for now (the user: "any other collision free station", to be measured in the lab)

- **Station: (-1.3, -1.9), yaw 180 deg**, west of the structure, facing the bar tables; in `data/lab/main.json`.
- How it was found (scripts in the session scratchpad, `station/`):
  - From above, every 10 cm and 90 deg of the floor: the hull of the robot's footprint and the bar in its loading
    posture, for each of the 27 saved carries in the scene of its own turn (the lab's obstacles and border, the bars
    built by then, the robots where the process has them, a box on the park row for the others). East of the
    structure nothing keeps more than 0.18 m; west of it (x -1.9 to -0.9, y -2.2 to -1.7) up to 0.51 m, at this pose.
  - In tamp's copy of the base planner's 3D collision world, with the lab's obstacles, border walls and parked
    robots added: all 27 loading postures are collision-free at this station (and at the eight other candidates
    tried, the first trial (2.3, 0.2, 270) included).
- **The drive in from it, same world, 8 s per search:** 21 of 27 bars (paths 1.1 to 6.9 m). Not B165, B168, B169,
  B172, B26, B59: the search back from the bar's base pose ends at the border, a table, a parked robot or a built
  bar. The same bars fail from every west candidate, and 11 to 12 bars fail from the two east ones: their loading
  postures were chosen with a drive-in check that knows neither the lab's obstacles nor its border, from (3.0, 0.5).
- Not done: the carries planned again from this station with the lab in the drive-in check (their `carry.json`
  still names (3.0, 0.5), so the monitor warns at every load); the empty drive from the park row to the station was
  not planned offline (the monitor plans it live).

### 2026-11-23, the user's first simulator run (B21's drive in refused), and what was added for it

- `doc/experiments/cdfm_planner_sim_manual.md` written as a draft from the B20 run (guide flow only).
- `prm` is the run's default CDFM planner (`RunSettings.plan_cdfm_planner`); `FreeSearch` alone stays on `birrt` (it
  has no roadmap root).
- The 34 roadmap folders of `261117_2ModulesDemo_order_bracing` (roadmap, settings, the 27 carries; 12 MB) copied to
  the Drive's `data_experiment/cdfm_planner_benchmark/prm/`, on the user's word: an empty Roadmap root works there.
- The user's B21 drive in with the bar: `the path as followed hits joints/J20-21_mocap ... at (1.51, -1.53)`, 35 cm
  before the target, from both search directions. B21's saved carry loads the bar `horizontal` (across the front).
  tamp's drive-in check had accepted it.
- A base path that hits as followed now comes with a `CollisionHit` (the robot and what it holds at that pose): the
  Collisions tab lists and draws it, as it does a start or target in collision (`algorithms/lattice.py`).
- The loading pose's class can be chosen: tamp `prm_options["start_carry"]` / `OnlineSettings.start_carry`
  (`plan_process_carry.py --cdfm-home-anchor horizontal|vertical|back` now also restricts the roadmap backend: its
  start anchors and the fallbacks' loading poses); the carry file records `loading_class`.
- `scripts/plan_carry.sh <bar> [class]`: plans one bar's carry again on the local bench root from the station in
  `data/lab/main.json`, keeps it as `carry.json`, copies it to the fake lab's scratch Drive and prints the copy
  command for the real Drive. The guide reads the file again at the travel to load's next Prepare.
- ? Open: tamp's drive-in check and the monitor's base planner disagree (B21). The lab's bodies, the joints with
  their markers and the monitor's margin are not in tamp's check.
- B21 planned again by class, from the new station (scratch set `seq_cls`, nothing saved): `back` finds a route from a
  grid pose (drive path exists) but the dense check refuses it (`bars/B21` against `bars/B20`, segment 136 of the
  route), 1042 s; `vertical` has no start on B21's roadmap (69 horizontal, 54 back) and no grid pose is reached, 902
  s, now reported as `prm_no_start_of_class`. Over the 34 roadmaps: 5184 horizontal, 2469 back, 846 vertical starts.
- The guide's **Loading pose** folder (the user: "yes I want this"), shown on every part of an action that carries its
  bar: a chip with the saved carry's class, **Class** (any, horizontal, vertical, back), **Plan carry again** (second
  press) and **Stop planning**, the time and tamp's last line below them.
  - `shared/assembly_planning/carry_plan.py` `CarryReplan`: tamp's `plan_process_carry.py --save-carry` as a process
    of its own (half the cores, nice 10, a session of its own so Stop ends its workers too), on a copy of the bar's
    roadmap under `~/.cache/husky_assembly_teleop/carry_replan` (tamp writes scenes and plans beside the roadmap it
    reads); only `adopt` writes, the new `carry.json` beside the roadmap. The station is the robot's in the lab file.
  - The guide polls it each tick (`_follow_replan`): a carry found is read again by the parts (`_loading` forgotten),
    none found leaves the saved one; `guide` events `carry_replan_started`, `_saved`, `_failed`.
  - The guide does not reopen the travel to load itself: the operator shows it, Redo from here, Prepare.
  - `scripts/plan_carry.sh` stays as the same thing from a terminal.
  - Checked: tests on fakes and with a stand-in script; the panel built on a real viser gui (found `add_dropdown`'s
    start value must go by keyword); one real run of `CarryReplan` outside the monitor, on a scratch root: B20, any
    class, from the station (−1.3, −1.9, 180°): a carry in 368 s, class horizontal, only `carry.json` written beside
    the roadmap. ? Not yet pressed in a running monitor.
- B21's refused drive in is the base planner's choice of side, not the carry (measured offline with the monitor's
  planner, the user's exact message reproduced): it lines up 0.30 m north of the target, between it and B20, to back
  in, and the turn there brings the base's corner 48 mm from `joints/J20-21_mocap` (limit 50 mm); from a start 0.5 m
  further south it drives in forwards in 0.7 s. The other side is never tried once the first line-up spot is
  accepted. The user: leave it, drive the base by hand for now. ? Section 6.1 of the sim manual still reads as if
  another loading class could help B21.
- The guide's preview of a carry (the user: as the dashboard's): what the robot holds at the part's start rides on
  the preview robot (`RobotGhost.hold`), and a **Guide plots** window shows the joints, the joint step and the held
  grasp (`PlanPlots(grasp=True, timed=True)`), a yellow line at the Preview time (`PlanPlots.cursor`: a third axis on
  the x scale with one tick; viser's uPlot has no line to set).
  - `shared/assembly_tab/held_preview.py`: `held_bodies`, `relative_tool0`, `with_midpoints`, `grasp_drift` moved
    here from the insertion demo's `parts.py` (which imports them back), and `held_drift`: the grasp over the timed
    path's seconds, at its waypoints and at three points between each two, on the URDF alone, off the main thread.
  - Checked: guide and insertion demo tests; the plots with B21's saved carry in headless Chrome (cursor drawn and
    moving). ? Not yet seen in a running monitor; the Assembly tab's Plan plots and path ghost are as they were.
