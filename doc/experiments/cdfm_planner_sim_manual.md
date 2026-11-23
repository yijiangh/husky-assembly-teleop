# CDFM planner in the simulator: a bar loaded at the station and carried in (Transfer workaround off)

With the Transfer workaround unticked, Cindy loads the bar at a loading station, drives to the bar's base pose with the
bar on the arms, and the CDFM planner plans the carry to the insert start from where the arms really are. This manual
runs that for B20 on the simulated robots, in the Assembly guide.

> **Draft, 2026-11-23.** Every **Expect** line is from one simulator run of B20_J on 2026-11-20, from the trial
> station (2.3, 0.2, 270°). The station has since moved to (−1.3, −1.9, 180°), and nothing was run from it yet: the
> lines marked *(station)* will read differently. Not in the draft yet: B21, B22 and two harder bars, the `birrt`
> planner, base offsets, the same test by hand in Assembly + Plan › Base.

The fake lab itself (panes, page, health chips, safety) is as in
[fake_lab_test_manual.md](../lab/fake_lab_test_manual.md): its parts 0 to 3.b apply, with the changes below.

## 0. What the test does

| Guide part | What happens | Base |
|---|---|---|
| 3 | drive to the loading station (a pose saved in `data/lab/main.json`, `stations`) | moves |
| 4 | `B20_J_load`: the empty arms travel to the loading posture, the first row of the bar's saved carry | still |
| 5, 6 | `B20_J_place_bar`, `B20_J_grasp`: you place and clamp the bar by hand | still |
| 7 | drive to B20_J's start with the bar: the base planner plans with the bar on the arms, the arms are watched | moves |
| 8 | `B20_J_transfer`: the CDFM planner (`prm`, the roadmap) plans the carry from the live arms at the live base | still |
| 9 | servo onto the insert start | still |
| 10 to 13 | insert, its servo, the foundation by hand, mark done; then B20_R (retreat, home) | still |

Parts 1 and 2 park Alice and Belle; they stand on the park row already and show `done`.

**The saved carry.** Each bar's carry was planned ahead by tamp and kept beside its roadmap
(`<roadmap root>/prm/<process>__<bar>/carry.json`). The travel to load ends on its first row. The live carry is planned
with its path as a seed, so it stays near what was checked offline. 27 of the process's 34 bars have one; B20, B21 and
B22 do.

## 1. Prepare  [SETUP — every test]

- [ ] If a `fake_lab` session still runs, stop it first: `tmux kill-session -t fake_lab` (`tmux.sh` only re-attaches
      to an old one, which has the other process).
- [ ] Copy the process and the bars' roadmaps and carries into the scratch Drive. This reads the real Drive and the
      benchmark's local root, and writes only under `~/husky_fake_lab`:

**▶ any terminal** (keep it: 2 uses the same two settings)
```bash
export FAKE_LAB_PROCESS=data_design_study/261117_2ModulesDemo_order_bracing
export FAKE_LAB_BENCH=~/Code/ros2_ws_next/dev/bench_mock/cdfm_planner_benchmark
~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/fake_lab/drive.sh --fresh
mkdir -p ~/husky_fake_lab/drive/data_experiment/bar_action_live_replan_exe
```

**Expect:** `drive.sh: 34 roadmap folder(s) copied to .../data_experiment/cdfm_planner_benchmark/prm`, then
`drive.sh: .../data_design_study/261117_2ModulesDemo_order_bracing is ready (...)`.

The `mkdir` gives the as-built record its folder in the scratch Drive. Without it every movement logs an error
`[as-built] ... start NOT recorded: FileNotFoundError(...)` and runs all the same.

## 2. Bring up the simulated lab and the monitor  [SIM ON — nothing moves]

- [ ] In the same terminal: `~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/fake_lab/tmux.sh`, then follow
      fake_lab_test_manual.md 2.a, 3.a and 3.b (router, robots, relay; Enter in pane monitor; the page).
- [ ] button [Acknowledge] at the top of the page, if it shows (the bases' calibration is older than the fake Motive's
      bodies).

## 3. Load the process and pick the planner  [CHECK — nothing moves]

- [ ] Assembly › PROCESS: button [Load] once; wait about 20 s
  - **Expect:** pane monitor: `loaded .../261117_2ModulesDemo_order_bracing in 17.6 s: ...; 80 actions, 283
    movements`, and three warnings `a process robot differs from the configured one: ...`.
- [ ] box [Transfer workaround]: untick it
  - **Expect:** pane monitor: `Transfer workaround off: jointing actions as authored`. The list reads
    `1. B20_J · cindy · 6 mv · pending`, and the chip beside the process's name `80 actions · 317 movements`.
- [ ] Assembly › folder **Advanced planning** (scroll down, open it) › list [CDFM planner]: reads `prm`, the default;
      [Roadmap root]: empty (the Drive's `data_experiment/cdfm_planner_benchmark`, here the scratch copy of 1)
  - A run that saved `birrt` keeps it: choose `prm`.
  - The guide plans with these settings too. `birrt` is the other planner; it does not use the saved carry's path.
- [ ] tab **Assembly guide**: in the second row of tabs at the top of the side panel, under **Assembly**
      (`Assembly guide`, `Control`, `Doc`). Every button of part 4 is in this tab, none in Assembly.
  - **Expect:** `B20_J · part ... of 13`, and under THE ACTION'S PARTS the thirteen parts of the table in 0, the first
    two `done`, the others `to do`.

**The guide's buttons.** The two arrows at the top step back and forward through the parts. Each part is
[Prepare …] (plan), [Play] (watch the preview), [Go …] (the robot moves), [Done] (next part). Leave Mode on `Off`.

## 4. B20_J in the guide  [SIM RUN — recorded]

### 4.1 Part 3: drive to the loading station

In the **Assembly guide** tab (3, last step).

- [ ] Step to `part 3 of 13 · DRIVE · drive to the loading station for B20_J` (the back arrow, if the guide opened
      further on)
- [ ] button [Prepare: plan the drive] (about 10 s)
  - **Expect:** under the buttons: `Drive cindy (0806), drive to the loading station for B20_J: Lattice (hybrid A*):
    1.00 m, 14.2 s, found in 7.0 s. Check the path is clear of people ...; e-stop in hand, then Go.` and `waits for
    Go` *(station: the length and time will differ; the new station is west of the structure, several metres from the
    park row, and this drive was not planned from it yet)*.
  - **If not** (no path): Plan › Base: button [0806] → [Load: Station] → [Path: Plan] → [Commit], or drive in
    Control › `0806` › Base; then [Done] in the guide. Send me the refusal's text.
- [ ] button [Play], then button [Go: drive]
  - **Expect:** pane monitor: `base path N sent to a200-0806: ... m, ... waypoints`, then `base path N on a200-0806:
    done` and `guide: drive to the loading station for B20_J done`. The guide's `base` line reads about `0.1 cm
    +0.3°`, `at its end`.

### 4.2 Part 4: the travel to load

- [ ] forward arrow: `part 4 of 13 · MOVE · B20_J_load: travel to load`
- [ ] button [Prepare: plan the move] (about 20 s the first time: `rebuilt the planning cell for robots/a200-0806 in
      13.9 s; why: first build`)
  - **Expect:** `Run B20_J_load: travel to load: 30.0 s. Play the preview, check the arms clear everything; e-stop in
    hand, then Go.` and `waits for Go`.
  - A warning `the robot stands ... m from the station (3.00, 0.50) the saved carry's drive in was checked from` is
    right for now: the carries were planned before the station moved.
  - **If not:** `no loading posture: no saved carry for B20 at ...`: the roadmaps were not copied (1), or the
    Roadmap root in Advanced planning points elsewhere.
- [ ] button [Play]: the empty arms swing out to the loading posture; then button [Go: run]
  - **Expect:** pane monitor: `B20_J_load — move left_ur_arm, right_ur_arm along the planned path (Rhino obstacles
    not checked), 30 s, largest joint travel 242°` (a warning: over 60°), then `arrived`.

### 4.3 Parts 5 and 6: the bar, by hand

- [ ] forward arrow: `part 5 of 13 · BY HAND · B20_J_place_bar: by hand`
- [ ] boxes [Bar placed] and [Hands clear]: tick both → button [Done]
  - **Expect:** pane monitor: `guide: B20_J_place_bar: by hand done`. The guide steps to `part 6 of 13 · BY HAND ·
    B20_J_grasp: by hand`.
- [ ] boxes [Screws clamped] and [Hands clear]: tick both → button [Done]
  - **Expect:** pane monitor: `guide: B20_J_grasp: by hand done`. The guide steps to `part 7 of 13 · DRIVE · drive
    to B20_J's start with the bar`.

### 4.4 Part 7: the drive in, with the bar

- [ ] button [Prepare: plan the drive] (about 15 s)
  - **Expect:** `target x +1.116 m y -1.516 m yaw +90.0°`; pane monitor: `Lattice (hybrid A*): 2.23 m, 33.9 s, found
    in 12.8 s; keeps only 8 cm from robots/a200-0805 ...; 10 cm from everything else`; `waits for Go` *(station: the
    length, the time and which body is nearest will differ)*.
  - **If not:** `start is in collision with ...`: the bar on the arms reaches into something at the station. Send me
    the text; this is what the station pose is being checked for.
- [ ] button [Play]: the ghost drives with the bar on its arms → button [Go: drive]
  - **Expect:** pane monitor: `base path N sent to a200-0806: 2.23 m, 110 waypoints, slowed per pose`, then `done`
    and `guide: drive to B20_J's start with the bar done`. The `base` line reads about `0.1 cm +0.3°`.
  - The arms stay as they are, and are watched during the drive: an arm that moves stops it. In my run the watch
    did not trip.

### 4.5 Part 8: the transfer, with the bar (the CDFM planner)

- [ ] forward arrow: `part 8 of 13 · MOVE · B20_J_transfer: transfer with the bar`
- [ ] button [Prepare: plan the move] (1 to 2 minutes; tamp's progress lines print in pane monitor)
  - **Expect:** pane monitor: `plan B20_J_transfer: transfer_held solved by tamp.plan_constrained_dual_arm/prm: 154
    waypoints, 1 tries, 95.8 s`; the guide: `Run B20_J_transfer: transfer with the bar: 17.4 s. Play the preview,
    ...` and `waits for Go`.
  - **If not:** `plan B20_J_transfer: transfer_held failed: tamp.plan_constrained_dual_arm/prm: no path in ... s
    (tamp: prm_...)`; the reason is also under the buttons and in the run's events (`plan_failed`). See 6.
- [ ] drag **Preview time**, or button [Play], before Go *(added 2026-11-23, not yet seen in a running monitor)*
  - **Expect:** the green preview robot carries the bar and its joint halves from the loading posture to the insert
    start; watch it past the built bars and the floor.
  - **Expect:** the window **Guide plots** opens at the bottom: `joint values (deg)`, `joint step (deg)`, and the two
    plots of the tools' relative pose against the first waypoint, `position (mm)` (at the waypoints, and the largest
    between two) and `axis angles (deg)`. A yellow line marks the Preview time in the three plots over seconds and
    moves with the slider. B21's saved carry, as a guide of size: under 0.05 mm at the waypoints, up to 0.5 mm
    between two, under 0.03°.
- [ ] button [Go: run]
  - **Expect:** pane monitor: `B20_J_transfer — move left_ur_arm, right_ur_arm along the planned path (Rhino
    obstacles not checked), 17 s, largest joint travel 93°`, then `B20_J_transfer: arrived, end within 0.02°` and
    `guide: B20_J_transfer: transfer with the bar done`.

### 4.6 Parts 9 and 10: the servo and the insert

- [ ] forward arrow: `part 9 of 13 · SERVO · B20_J_transfer: servo` → button [Prepare: plan the first pass]
  - **Expect:** pane monitor: `[servo 0] tool0 pos L=0.13 R=0.07 mm, ...`, then the prompt `SERVO by bar: no usable
    mocap fit of B20: ... Tool0 servo (Confirm) ...` (the fake Motive streams no bar markers).
- [ ] button [Go: confirm]
  - **Expect:** `the tool0 correction is carried into the plan of B20_J_insert`, `Servoing data saved to
    .../servo/B20_J/B20_J_transfer.1.json`, `guide: B20_J_transfer: servo done`.
- [ ] forward arrow: `part 10 of 13 · INSERT · B20_J_insert: insert` → button [Prepare: plan the insert] (about 8 s)
  - **Expect:** `plan B20_J_insert: insert solved by tamp.plan_constrained_dual_arm_linear: 8 waypoints`.
- [ ] button [Go …]
  - **Expect:** `insert: RIGID_DONE: ground bar in, no tighten; the flanges stopped 0.0 mm from the planned end`,
    `guide: B20_J_insert: insert done`.

## 5. The rest of the action

Parts 11 to 13 (the servo after the insert, the foundation by hand, mark done) and B20_R (retreat, home) are not
changed by the Transfer workaround. I did not run them in this test; they are fake_lab_test_manual.md 4.3 to 4.5, done
with the guide's buttons.

## 6. If the transfer does not plan

The bar is already clamped, so these are the ways on, in order:

1. Read the reason. `prm_goal_unholdable` or `prm_goal_far_from_seed`: from this base no free posture holds the bar at
   the insert start next to the planned one. Bring the base closer to its design pose: step back to part 7, [Prepare:
   plan the drive] and [Go: drive] again (the bar stays on the arms), then [Prepare] the transfer again.
2. `prm_no_route`: the same, then Advanced planning › [CDFM planner]: `birrt` and [Prepare] again.
3. Give up on the carry: open the tools by hand, take the bar off, tick [Transfer workaround] and run the action the
   old way.

Offline, with the base 2 cm and 2° off its design pose, 22 to 27 of the 27 bars plan, depending on the direction; B20,
B21 and B22 plan at every offset tried, B22 once only without the check of its retreat
(`tasks/2026-11-20_held_carry_sequence.md`).

### 6.1 If the drive in with the bar does not plan (part 7)

`no plan: ... the path as followed hits <body> ... ; Collisions lists it`: the robot, or the bar on its arms, would
come closer than 5 cm to something on the way in ("hits" also means "within the margin").

- [ ] tab **Collisions**: the hit is listed under `base_planner_panel` and ticked
  - **Expect:** the 3D view shows the robot with the bar at the pose where it is too close: what moved in solid red,
    what it came near in see-through cyan, the distance between them. Untick it (or plan again) to take it away.

**If the red body is the base** (B21: `49.9 mm apart` from `joints/J20-21_mocap` on B20): the loading posture has
nothing to do with it. The base planner ends every drive with a straight piece onto the target and here lines up for
it on the side where B20 lies; it does not try the other side. Drive the base by hand to the other side first:

- [ ] tab **Plan** › **Base**: set a target about 0.5 m south of the station's line (for B21 from the station
      (−1.3, −1.9): (−1.3, −2.4, 180°) plans) → [Plan] → [Commit]; then in the guide [Prepare: plan the drive] again
  - **Expect:** the drive in plans, straight in forwards. *(Measured offline with the monitor's planner on
    2026-11-23, not yet driven.)*

**If the red body is the bar or an arm:** choose another loading posture for this bar and plan its carry again. The
roadmap is kept; only the loading posture (one the base can drive in with from the station) and the carry from it are
planned again. A bar whose roadmap connects plans in 1 to 3 minutes; one that needs the fallbacks takes 15 minutes or
more.

**▶ monitor** — tab **Assembly guide**, folder **Loading pose** (below the part's own folder; shown on every part of an
action that carries its bar)

- **Expect:** the `carry` line shows the class of the bar's saved carry, e.g. `class horizontal` (`class not
  recorded` for a carry saved before 2026-11-23).
- [ ] **Class**: your judgement of what drives in best: `horizontal` (the bar across the front), `vertical` (upright
      in front), `back` (along the robot, over its back), or `any` (the planner chooses)
- [ ] [Plan carry again] → [Plan carry again?]
  - **Expect:** an amber `planning B21` chip, and under the buttons the time and the planner's last line. The guide
    stays usable; nothing moves. [Stop planning] ends it and keeps nothing.
  - **Expect, when a carry is found:** a green `new carry` chip, `B21: saved after ..., class back. Show the travel to
    load, Redo from here, then Prepare`, and the `carry` line shows the new class. The bar's `carry.json` beside its
    roadmap is replaced (in the fake lab: under `~/husky_fake_lab/drive`; in the lab: on the Drive).
  - **If not:** an amber `no new carry` chip with the reason, and the saved carry is as it was. A class may have no
    loading pose for a bar (`no loading pose of this class is reached from the insert start`), or its route may fail
    the final check (`the route found touches something in the final check`). The whole output is in the log file
    the line names (`~/.cache/husky_assembly_teleop/carry_replan/logs/`).
- **B20, `any`, planned on 2026-11-23 from the station (−1.3, −1.9, 180°):** a new carry after 6 minutes, class
  `horizontal` (run outside the monitor, through the code the button uses).
- **B21, tried on 2026-11-23 from the station (−1.3, −1.9, 180°):** `back` fails the final check after 17 minutes (B21
  touches B20 near the end of the carry); `vertical` has no loading pose (15 minutes). Only its `horizontal` carry
  exists. Its refused drive in was the base, see above, so B21 needs no other class. No other bar was tried by class.
- [ ] In the guide: back arrow to part 4 (`travel to load`) → [Redo from here] → [Prepare: plan the move]: the travel
      to load now ends on the new loading posture. Then parts 5 to 7 again.
  - In the lab the bar comes off the arms first: the travel to load is planned for empty tools.

The same from a terminal, without the monitor (the new carry is copied to the fake lab's scratch Drive; for the real
Drive it prints the copy command):

```bash
export HUSKY_BENCH_ROOT=~/Code/ros2_ws_next/dev/bench_mock/cdfm_planner_benchmark
~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/plan_carry.sh B21 back
```

## 7. Known issues

- After a monitor restart, a drive that was done counts as `to do` until a later part ran: the next part's Go asks
  again (`Go asks again: out of order, not done yet: drive to ...; press Go again within 5 s to go anyway`). Press
  [Go?] again.
- The page lags a few seconds behind a press while a plan runs; wait for the button's text to change before the next.
- Six bars cannot be driven in from the current station in my offline check (B26, B59, B165, B168, B169, B172): their
  loading postures were chosen before the lab's border and obstacles were known to that check. Seven bars have no
  saved carry at all; the travel to load refuses with the command that plans one.

## 8. What to send back

The run folder (`~/husky_fake_lab/drive/data_design_study/261117_2ModulesDemo_order_bracing/runs/<date>_<time>`), the
monitor's log (`~/husky_fake_lab/logs/monitor_<date>_<time>.log`), and for anything that differs from an Expect line:
the part's number and the text under the guide's buttons.
