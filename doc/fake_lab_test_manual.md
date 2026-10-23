# Fake-lab test manual: the core port on three simulated robots

Tests the new monitor (`yh/core-port`: Jakob's newer code, merged on 2026-10-21, plus the Rhino obstacles box of
2026-10-22, and since 2026-10-23 every panel in a tab of the side panel) against three simulated robots on this laptop.
The robots come from Jakob's simulation (`launch/sim.launch.py robot:=all`, [simulation_manual.md](simulation_manual.md)),
on crl-husky `jg/ros2` (78041cf or later): a simulated base and the UR driver's fake hardware for the arms, parked, with a fake Motive
that adds Motive's measured latency, jitter and marker errors, and the mocap relay that publishes each base as on the
lab PC. Everything talks over Zenoh, as the lab does with the new monitor, through a Zenoh router of the fake lab's own.
There is no robot, no real Motive and no tool. Plan about an hour.

Every step was run on 2026-10-22, on Zenoh, with this code, and gave the results it lists. On 2026-10-23 the panels
moved into tabs of the side panel (**Assembly** › Run, Exec, Steps; **Plan** › Base, Motion, Arm); their buttons and
results are the same. Also on 2026-10-23 the fake lab moved to Jakob's simulation: the robots start on their park
row instead of 0.5 m behind it, with mocap. Sections 0–2 were run again then; the base plans of 4.1, 4.2 and 9.2 were
not, and say so.

## What you are testing, and whose it is

The new monitor is Jakob's rewrite. Our port adds the assembly on top of it: everything the old monitor did to run a
bar's steps.

| Feature | Whose | Where in this test |
|---|---|---|
| **The monitor itself.** The web page, phone page, plugins and panels; health chips; Stop all, which now latches each robot until its Resume; per-robot leases; robot_control (jog, tools) in the side panel's **Control** tab; arms move only when the onboard safety sync vouches for them. | Jakob | 1–3, 6, 7 |
| **process.** Loads a process folder (Rhino's new export) and shows a movement in 3D. | Jakob | 3, 4 |
| **The Rhino obstacles box (new).** "Rhino obstacles collide" in the process panel, off at start. While it is off, the process's own obstacles from Rhino (the cabinet `env39`) are drawn see-through and left out of every collision check: base and arm planners, clearances, assembly planning. Tick it to count them. A plan made before a change goes stale. | **Ours** | 3, 4, 5 |
| **obstacles.** The lab's measured obstacles, from `data/lab/main.json`: the cabinet, tables, tripods, trusses and the border walls. They always count. | Jakob | 2, 4 |
| **The planners.** **base_planner_panel**, in **Plan** › **Base**, plans a base path and sends it to the robot's follower, with a clearance watch while it drives. **Plan** › **Arm** plans one arm to a joint target, and the **Collisions** tab lists what blocked a plan. | Jakob | 4, 6, 9 |
| **The simulated robots.** Jakob's simulation (`launch/sim.launch.py`): crl-husky's simulator, a fake Motive and the mocap relay, with the `SIM` chips. The fake-lab scripts around it are ours. | Jakob | 1, 2 |
| **assembly_run (ours, M4).** The run record `runs/<date>_<time>/`: which actions are done, where each robot was left, an event log. Mark done, Reopen, New run, and continue after a restart. The run's settings, such as the transfer workaround. | **Ours** | 4–6, 11 |
| **motion_planner (ours, M3–M4).** Plans a step's motion at the live base through tamp, solves goals by exact IK (ssik), checks collisions. One worker, which the other plugins ask. | **Ours** | 5–7, 9 |
| **assembly_steps (ours, M5).** The operator's step panel: Check goal, Plan, Play, Exec, Prev / Next, Mark done. A plan goes stale when anything moves, or when a setting changes. | **Ours** | 5–9 |
| **movement_exec (ours, M5).** Runs a step's parts after Confirm (desktop or the phone's Exec tab). One motion at a time across all robots: it leases every robot while it runs, so Jakob's panels get no lease. Checks before every send: another robot moving, a silent link, the safety sync. Retimes paths to the arms' limits. Tool steps, by-hand steps, Cancel, and a record of every part. | **Ours** | 5–8, 10 |
| **Under the hood (ours, M2–M3, not visible as a panel).** Each movement mapped to a routine, steps grouped (with the transfer workaround), the planning scene per movement, swept collision checks, and the core's tool-vs-tool check (CC.6). | **Ours** | everywhere a step is planned |
| **Not in this test:** insert, retreat and handoff (M6, compliance); the servo and the MoCap bar fit (M7); calibration (M10). | Specs, waiting for your OK | – |

### The new monitor, in old-monitor terms

| Old monitor | New monitor (one web page, http://localhost:8080) |
|---|---|
| One monitor per robot, each robot on its own Cyclone domain (84/85/86). | One monitor for all three robots, all on Zenoh in domain 80. Here they are simulated, on Zenoh in domain 70. |
| The PyBullet window. | The 3D view, on the left of the page. |
| The obstacles in the PyBullet scene, from the export. | The process's Rhino obstacles (drawn; they collide only when **Rhino obstacles collide** is ticked), plus the measured lab obstacles (the **obstacles** panel). |
| The compas_fab export: `BarActions/`, `ActionSchedule.json`. | A *process folder*: `process.json` plus `actions/`, from Rhino's RSExportProcess. You open it in the **process** panel. |
| `progress.json` and its Reset. | **assembly_run**: the run folders, with Mark done, Reopen and New run. An old run is never deleted. |
| Check goal, Plan and Exec for the current movement. | **assembly_steps**: the same buttons for the run's current step, plus Play, and Prev / Next. |
| The confirmation before each motion or tool action. | **movement_exec**: one prompt at a time, with Confirm and Cancel. The phone page has the same prompt. |
| Jog, stow and gripper buttons. | The side panel on the right: **Control** has a tab per robot (Base, and Arm or Left / Right); **Plan** has the base planner (Base), motion_planner (Motion) and the arm planner (Arm); **Collisions** has a tab of its own. |

Each tab of the side panel on the right belongs to one plugin and is named after it; the assembly's three share the
**Assembly** tab (assembly_run is **Assembly** › **Run**, movement_exec **Exec**, assembly_steps **Steps**), the
planners the **Plan** tab. Health floats at the top left.

**Words used below:**
- **Milestones M2–M10.** These are steps of the port plan, not the tool's M1/M2 motors (the grasp prompt also mentions
  those).
- **Transfer workaround.** The same rule as in the old monitor since 2026-10-06: the transfer runs as a free move
  without the bar, and you mount the bar at its end. Here that is the place-bar and grasp prompts.
- **F5.** Finding 5 of the safety review of M5: what one robot's silent link does to the other robots' motions. You
  decide its rule after this test (section 8).

> ⚠️ **Safety.** Everything runs on ROS domain 70, from a clean environment. Every node connects only to the fake
> lab's own Zenoh router, which listens on the loopback only (port 7470; the lab's routers use 7447) and connects to
> no robot. The relay looks for Motive at 127.0.0.1 only. Your `.bashrc` puts every new terminal on Cyclone and
> domain 86 (Cindy's), but these scripts never use it.
> The robots now in use can't see this test, and it can't see them. The monitor's Drive root is the scratch folder
> `~/husky_fake_lab/drive`, so its `runs/` never reach the real Drive.

Steps are numbered so you can report them as, for example, "5.4 not OK: …".

---

## 0. Once, before the first test

**0.1** Install the UR driver, Zenoh's ROS middleware and tmux. The simulated arms run on the UR driver's fake
hardware, which added 39 packages. Zenoh adds 2 more; the lab needs them anyway (README, One-time setup). tmux holds the
panes (section 1). Nothing is upgraded.

```shell
sudo apt install ros-humble-ur-robot-driver ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor tmux
```

Before the first real-lab run, check that the robots have the same Zenoh version (crl-husky's README requires it):
`dpkg -l ros-humble-rmw-zenoh-cpp` on each robot and here. This laptop gets 0.1.11.

**0.2** Copy the test process into the scratch Drive root. `--fresh` also deletes the runs of an earlier test.

```shell
~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/fake_lab/drive.sh --fresh
```

**Expect:** `drive.sh: …/husky_fake_lab/drive/data_design_study/261008_2ModulesDemo_purged_process is ready (84M)`.

This is Jakob's export of the two-module demo. Its first 18 actions are the three-robot sequence B20–B37.

**0.3** Build the workspace from this branch, with crl-husky on `jg/ros2` (78041cf or later): the fake lab runs the installed relay
and fake Motive, and `env.sh` refuses an install without them.

```shell
cd ~/Code/ros2_ws_next && source venv/bin/activate && python3 -m colcon build --symlink-install
```

---

## 1. The shared tmux session

**1.1** In any terminal, run:

```shell
~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/fake_lab/tmux.sh
```

This opens the tmux session **`fake_lab`** with one window and four panes:
- **top:** the three simulated robots and the fake Motive (`sim.sh`). They start at once.
- **middle, the monitor pane:** the monitor command (`monitor.sh`), typed in but not started yet.
- **bottom left, small:** the Zenoh router (`router.sh`) that every node meets through, as the PC's and the robots'
  routers do in the lab. It starts first.
- **bottom right, small:** the mocap relay (`relay.sh`), as on the lab PC: it publishes each base's pose from the fake
  Motive.

I read the panes from my side (`tmux capture-pane`). I never type into them unless you ask.

- **Leave the session:** Ctrl-b d. It keeps running.
- **Come back:** run `tmux.sh` again, or `tmux attach -t fake_lab`.
- **More panes:** Ctrl-b " splits one, for example for section 8.

**Expect, in the bottom left pane:** `fake lab: Zenoh router for ROS domain 70, on 127.0.0.1:7470 only`.

**Expect, in the bottom right pane:** `fake lab: mocap relay on ROS domain 70, Zenoh router 127.0.0.1:7470, no Motive`,
its calibrations (every robot on `crl-husky default`), and once the robots are up
`rigid body 1840: now from the fake Motive, was the simulator's truth`, the same for 1850 and 1860. A
`No NatNet frames … reconnecting` line every 30 s is right: no Motive runs here.

**Expect, in the top pane:**
- `fake lab: alice, belle and cindy on ROS domain 70, Zenoh router 127.0.0.1:7470`;
- then many `Configured and activated …` lines, and `sim_motive: rigid body 1840: faking it, noise [1.5e-05, 0.003],
  marker_error 0.0005`, the same for 1850 and 1860;
- four `process has died` lines, one per arm. That is the cartesian controllers' spawner: those controllers aren't
  installed here, and nothing in this test uses them.

**1.2** When the top pane goes quiet (about 20 s), click into the monitor pane and press **Enter**.

**Expect:**
- `fake lab: open http://localhost:8080 …` and the path of the log it saves;
- about 10 s later, `started in … ms`.


> **Why `monitor.sh` starts the monitor in an odd way.** It imports pytest first, as a workaround for a bug I found
> while preparing this test. compas_fab closes stdout for a moment around every model it loads. Inside the monitor,
> other threads open files at the same moment, so Check goal and Plan failed with
> `OSError(16, 'Device or resource busy')`. compas_fab skips that step when pytest is imported. The real fix is your
> decision, and comes after this test. Side effect: PyBullet's own messages print in the monitor pane.

---

## 2. First look

**2.1** Open http://localhost:8080 in Chrome.

**Expect:**
- **3D view:**
  - three robots side by side on their park row, y = 1.35 m (Alice at x = 0.3, Belle at 1.7, Cindy at 3.1), facing
    +y, arms at their stow pose.
  - the measured lab (Jakob's **obstacles** plugin, its panel minimized at the right edge): the cabinet behind the robots,
    tables, tall grey tripod cylinders, and an orange border around the floor.
- **health:** every arm shows a green `arm` chip and a `SIM` chip. Every base shows a `SIM` chip, a green `ctrl` chip
  and a `mocap` chip: green for Alice and Cindy, amber for Belle, whose fake Motive body reports the 1.2 mm marker
  error measured in the lab (hover it).
- **Top left:** health, with Stop all.
- **Side panel on the right:** the tabs **Process**, **Assembly** (Run, Exec, Steps: the order this manual uses them
  in), **Plan** (Base, Motion, Arm), **Control** (one tab per robot) and **Collisions**; **Obstacles** sits minimized
  at the right edge.

**Expected in the simulation:**
- `robotiq` and `scaffolding_v3` stay red: no tool drivers.
- The top shows about `12 PROBLEMS`: the four tools, the seven `SIM` chips and Belle's `mocap`.
- On a busy PC a base's `signal` chip may turn red (hover: a gap); see the known issues.

---

## 3. Load the process, the run, and the Rhino obstacles box

**3.1** In the **process** panel, the folder is filled in. Click **Load** once, and wait 5–10 s.

It doesn't load by itself: the README says it does, but the code waits for this button.

**Expect:**
- **While it loads:** the chip shows the progress, such as `reading the robots' URDFs and meshes`.
- **Then:**
  - **process:** a green chip `261008_2ModulesDemo_purged_process`, `78 actions · 306 movements`, and an amber `robots
    differ`. The amber chip is expected: this export used the old UR frames, and the monitor plans with the configured
    robots.
  - **ACTIONS:** `1/78`, with `1. B20_J · cindy · 5 mv` on top.
  - **assembly_run:** `no run yet` and `0/78 done`; `current B20_J · B20_J_transfer (4/5)`; `alice`, `belle` and
    `cindy` each `live now`.

**Open the page before you click Load, and keep that one page open.**
- A page that opens or reloads after Load first receives the whole drawn process: about 200 bodies with their meshes.
  Until it has them, it shows the panel's starting state (`no process`, empty rows). That took about 20 s in my test.
- Every Load draws everything again. So clicking Load again while a page catches up only sets it further back.
- If the panel shows `no process` while the monitor pane says `[process] loaded … 78 actions, 306 movements`, wait
  about 30 s. Don't click Load again, and tell me if it doesn't recover.

**The process panel's Step slider** only changes which movement the 3D overlay shows. The run's current step is chosen
in assembly_run (click an action) and assembly_steps (Prev / Next).

**3.2** **The Rhino obstacles box (ours).** Right under **Load**: **Rhino obstacles collide**.

**Expect:**
- unticked, with the grey chip `Rhino obstacles: drawn only` under it;
- **3D view:** Rhino's cabinet `env39` see-through, a long box just in front of the measured cabinet (on the robots'
  side). It sits about 0.38 m nearer the robots than the measured one; the known issues say more.

Tick it.

**Expect:**
- the chip turns green: `Rhino obstacles collide`;
- the Rhino cabinet turns solid;
- **monitor pane:** `[process] Rhino obstacles now count in every collision check`.

Untick it again.

**Expect:** `[process] Rhino obstacles are drawn only: left out of every collision check; replan`.

If a red `room unchecked` chip ever shows next to it, the measured obstacles are not loaded, and nothing in the room is
checked. `monitor.sh` loads them; tell me if you see it.

---

## 4. The base planner: the box on Belle, then Cindy to her B20 base (Jakob's planner)

**4.1** **The box on a base plan.** Belle's park pose is inside Rhino's cabinet.

> Not run again since the robots start on the park row: Belle now starts at her park pose, so step 4 may name the
> start rather than the target, and the plan of step 5 is about 0 m long.

1. Open **Plan** › **Base** (base_planner_panel) in the side panel.
2. Pick Robot **a200-0805**.
3. Click Load **Park**.
   - **Expect:** `target +1.700 +1.350 +90.0°`.
4. Tick **Rhino obstacles collide**, then click **Plan**.
   - **Expect:** `no plan: target is in collision with obstacles/env39`.
5. Untick it and click **Plan** again.
   - **Expect:** `Lattice (hybrid A*) (straight): 0.50 m, 2.5 s`.
6. Click **Clear**. Belle stays where she is.

**4.2** **Cindy to her B20 base.** The base planner plans around the robots, the obstacles and whatever each robot
holds at the movement the **process** panel shows. The run selects `B20_J_transfer` there, and at that movement the
design has Cindy *holding* bar B20. A held bar counts as part of Cindy, and from her start it sticks into the measured
cabinet behind her, so her plan would be refused (`start is in collision with building cabinet`).

So first show the action's start, before the bar is held: in the **process** panel, click **◀ action** once.

**Expect:** `movement 1/5  B20_J_load`. This changes only the 3D overlay and the planners' loaded targets, not the
run's current step.

Then, in **base_planner_panel**:
1. Pick Robot **a200-0806**.
2. Click Load **Step**.
   - **Expect:** `target +0.527 -3.067 +89.6°`.
3. Click **Plan**.
   - **Expect:** after about 2 s, `Lattice (hybrid A*) (straight): 4.83 m, 27.3 s`. That was from the old start,
     0.5 m behind the park row; from the row the length and time differ a little.
4. Click **Commit**.

**Expect:**
- `sent to a200-0806's follower`, and Cindy drives in the 3D view for about a minute: the simulated follower is
  slower than the plan's 27 s.
- **Control** → `0806` → `Base` ends at about `xyz +0.52 -3.06`: the simulated follower stops within about 1 cm and
  2°.

While a base drives, Jakob's clearance watch soft-stops it if it comes within the plan's margin of anything.

Alice and Belle stay where they are until section 9.

The simulated base has a stand-in controller manager (crl-husky), so its `ctrl` chip is green and driving it by hand
(**Control** → `Base` → **Vel**) can switch its velocity controller on; not tried in this test.

---

## 5. B20_J on Cindy: the transfer, as the workaround runs it (ours)

The **assembly_steps** panel shows:

```
B20_J · cindy (a200-0806) · step 1/2 · no plan
B20_J_transfer (transfer_free) · then: B20_J_place_bar, B20_J_grasp · skipped: B20_J_load
```

**5.1** Click **Check goal**. It takes about 10 s.

**Expect:**
- **Monitor pane:** `check goal B20_J_transfer: goal found (ssik, rank 1 of 64), max |goal - authored| 0.07 rad`.
  - ssik is the exact IK solver; "rank 1 of 64" means it took its solution closest to the authored joints.
  - The 0.07 rad is because the base stopped about 2° off the design base.
- **assembly_run:** `run <date>_<time> started`. A run folder is created at the first thing it records.

**5.2** Click **Plan**. It takes 20–40 s.

**Expect:**
- **Chip:** `plan ready`.
- **Panel:** `min … s by the arms' limits`, about 17 s. That is the 90°/s² acceleration cap at work.
- **Monitor pane:** `transfer_free solved by tamp.plan_free_dual_arm: … waypoints, 1 tries`.

**5.3** Click **Play**.

**Expect:** the preview runs as it will move. Drag `t s` to scrub through it.

**5.4** Click **Exec**.

**Expect:** the **movement_exec** panel shows `waiting for Confirm` and the whole sentence:

> cindy (a200-0806): B20_J_transfer — move left_ur_arm, right_ur_arm along the planned path (Rhino obstacles not
> checked), 17 s, largest joint travel …° (Play it first). Confirm to move.

"(Rhino obstacles not checked)" is there because the box is unticked.

Click **Confirm** in movement_exec.

**Expect:**
- Both arms move.
- **Monitor pane:** `B20_J_transfer: arrived, end within 0.00°`, then `B20_J_transfer: no servo correction (not supported yet)`.

**5.5** **Expect:** the next prompt is the bar by hand:

> B20_J_place_bar — do Operator places the bar by hand, then Confirm (the robot panels and their free drive refuse while
> this step has the robots' leases; Cancel to use them).

Click **Confirm**.

**5.6** **Expect:** the next prompt is the grasp:

> B20_J_grasp — GRASP: STOP, then the M1 screw of left_ur_arm, right_ur_arm TIGHTENS until STALLED (≤30 s), then STOP.

Click **Confirm**.

**Expect:** the grasp is refused with `left_ur_arm: no tool status in the last 1 s; is its driver running?`. That is
right for the simulation, which has no tool drivers.

**5.7** Step 2/2 is the insert. Don't run it: its Exec line says the insert is not supported yet.

**5.8** In **assembly_run**, click Action **Mark done**.

**Expect:**
- `B20_J marked done in run …; recorded robots/a200-0804, robots/a200-0805, robots/a200-0806`;
- the current action moves to `B20_R`.

---

## 6. B20_R: home, and one motion at a time (ours)

**6.1** Step 1/2 is the retreat together with the ungrasp. Its Exec line says neither is supported yet. Click
**Next ▶**.

**6.2** Step 2/2 is `B20_R_home (free_home)`. Click **Plan**.

**Expect:** it fails with `the start collides: tools/cindy/AT3L with bars/B20; …`.

That is right for the simulation: no insert or retreat ran, so the grippers still sit around B20. Ignore built bars
doesn't help either, because B20 is this action's own bar. Jakob's arm planner (Plan → Arm, Load Stow) refuses for
the same reason: `start is in collision: left_ur_arm tool with bars/B20`.

To get past it, jog Cindy's grippers off the bar by hand:
1. Open **Control** → `0806` → `Left`.
2. Press and hold **+ lift** for about 3 s. At Speed 0.5 the lift joint turns about 25°.
3. Do the same in `Right`.

These are raw joint moves with no collision check (the panel says so), so use them in the simulation only.

**6.3** Click **Plan** again.

**Expect:** `free_home solved by tamp.plan_free_dual_arm: … waypoints`.

Then set **Motion s** to **25**. The slow motion gives you time for 6.4.

**6.4** Click **Exec**, then **Confirm**. While Cindy moves:

- **Alice's arm:** open **Control** → `0804` → `Arm`, then press **+ lift** for a moment.
  - **Expect:** a `lease refused` chip on Alice's arm, and in the monitor pane a line saying robot a200-0804 is leased
    by `movement_exec: exec B20_R_home`.

After about 25 s, **expect:** `B20_R_home: arrived, end within 0.00°` and `Exec B20_R_home: done — every part done`.

**6.5** The step panel now has a button **Mark B20_R done**. Click it.

**Expect:**
- the current action becomes `B21_J`;
- **assembly_run** shows `2/78 done`.

---

## 7. Staleness, Cancel and Stop all (B20_J again)

**7.1** In **assembly_run**, click the row `1. B20_J · cindy · done`, then Action **Reopen**.

**Expect:** `B20_J reopened in run …`, and B20_J's transfer is the current step again.

**7.2** Click **Plan**. It takes about 20 s, from Cindy's home pose.

Then move Alice's wrist: open **Control** → `0804` → `Arm` and press **+ w3** for half a second (about 7°).

**Expect:**
- the chip turns `plan stale: replan` at once;
- clicking **Exec** answers `nothing to Exec: the plan is stale; Plan first`.

This is ours.

**7.3** **Changing the Rhino obstacles box makes a plan stale (ours).**
1. Click **Plan**.
2. Click **Exec**, but don't Confirm yet. The prompt says `(Rhino obstacles not checked)`.
3. Tick **Rhino obstacles collide** in the process panel.
4. Click **Confirm**.

**Expect:**
- **Monitor pane:** `Exec B20_J_transfer: refused — B20_J_transfer: the Rhino obstacles were switched on since
  planning`;
- the step panel: `plan stale: replan`.

Untick the box again.

**7.4** **Cancel during a motion (ours).** Plan, Exec, Confirm. While the arms move, click **Cancel** in movement_exec.

**Expect:**
- **Monitor pane:** `cancelled: holding every arm`, and `Exec B20_J_transfer: cancelled — … (operator)`.
- **3D view:** the arms stop mid-path.

**7.5** **Stop all during a motion (Jakob's button; our executor's reaction).**

1. Plan again. The plan starts from where 7.4 stopped and is short.
2. Set **Motion s** to 20.
3. Click Exec, then Confirm.
4. While the arms move, click **Stop all (Esc)** at the top of the page (or press Esc).

**Expect:**
- **Monitor pane:**
  - `robot a200-0806: SOFT STOP: Stop all`;
  - an error `/a200_0806: switch_controller is not available, cannot stop`: the simulated bases have no controller
    manager;
  - `cancelled: holding every arm`, and `Exec B20_J_transfer: cancelled — … (Stop all (the soft stop))`.
- **health:** every arm's `ctrl` chip turns amber, with `last stop: stopped by operator`. Each robot stays stopped until
  its own Resume.

Then click **Resume** under each of the three robots.

**Expect:** `robot a200-080x: resume arms: resuming`, and that robot's arm chips turn green again.

---

## 8. A robot's link drops (decision "F5", ours)

You'll decide later how strict this should be. This shows you today's rule: strict, so one silent robot stops every
motion.

`link.sh` freezes one robot's simulator processes, as if its WiFi dropped, and wakes them again; nothing is killed. Run
it in a new pane: press Ctrl-b " to split, then `cd ~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/fake_lab`.

> **Bring every link back up within 60 s.** On Zenoh, a frozen robot's sessions expire after 60 s, and its arm then
> doesn't come back after `up`; only restarting the fake lab helps (`tmux kill-session -t fake_lab`, then `tmux.sh`).
> A real WiFi drop doesn't do this: there, the robot's own processes stay attached to the robot's router.

**8.1** Plan B20_J's transfer again, but don't Exec yet. Then run `./link.sh belle down`.

**Expect:**
- `link.sh: belle down (6 processes)`;
- within about 2 s, health turns red for A200-0805;
- **assembly_run:** `belle not live; last recorded live at …`.

**8.2** Click **Exec**.

**Expect:** it is refused before any prompt:

> busy: can't tell whether a200-0805 ur_arm is moving: its joint states are … s old; no automatic motion until they
> come back (check its link in Health) or the monitor restarts

**8.3** Run `./link.sh belle up`.

**Expect:**
- within about 2 s, health turns green and the run panel shows `belle live now`;
- no Reconnect click is needed;
- a burst of red `[rmw_zenoh_cpp] Query queue depth of 10 reached …` lines in the monitor pane: harmless (see the
  known issues).

**8.4** **The link drops during a motion.**

1. Plan if needed, then Exec and Confirm on Cindy.
2. While her arms move, run `./link.sh belle down`.

**Expect, within about 0.5 s:**

> a200-0806: can't tell whether a200-0805 ur_arm is moving: its joint states are 0.5 s old … holding every arm

The Exec ends failed, with Cindy held mid-path: a *parked* robot's link stopped her.

Note what you think. Is that right, or too strict for a robot that was standing still? Then run `./link.sh belle up`.

---

## 9. The support robot: Alice at B21_H

**9.1** In **assembly_run**, click the row `4. B21_H · alice · pending`.

**Expect:** the step panel shows `B21_H · alice (a200-0804) · step 1/4 · B21_H_approach (support_approach)`, and the
process panel `movement 1/4  B21_H_approach`.

**9.2** Drive Alice to her design base with **base_planner_panel** (Jakob's):
1. Pick Robot **a200-0804**.
2. Click Load **Step**.
   - **Expect:** `target +0.638 -1.561 -29.6°`. If it shows another target, the process panel hadn't followed yet:
     click Step again.
3. Click **Plan**.
   - **Expect:** `Lattice (hybrid A*): 3.12 m, 25.2 s`. That was from the old start, 0.5 m behind the park row; from
     the row the length and time differ a little.
4. Click **Commit**.

**Expect:**
- Alice drives in the 3D view, for about 50 s in the simulation.
- **Control** → `0804` → `Base` shows `xyz ≈ +0.63 -1.56`, `yaw ≈ -32°`.

**9.3** In **assembly_steps** (ours), run the approach.

1. Click **Check goal**.
   - **Expect:** `goal found (ssik, rank 1 of 8)`, about 0.07 rad from the authored goal, because the base stopped
     about 2° off.
2. Click **Plan**.
   - **Expect:** `support_approach solved by arm.plan_arm`.
3. Click **Exec**, then **Confirm**.
   - **Expect:** `B21_H_approach: arrived, end within 0.00°`.

**9.4** Step 2/4 is `B21_H_open`. Click Exec; the prompt is `B21_H_open — OPEN the gripper (0.426). Confirm to send.`
Click Confirm.

**Expect:** it is refused with `/a200_0804/gripper: the gripper's action server is not available`. That is right: the
simulation has no Robotiq driver. Stop the B21_H steps here.

---

## 10. The phone page

**10.1** Open http://localhost:8081, either in a narrow browser window or on your phone on the same network as
`http://<laptop IP>:8081`.

**Expect:** the tabs **Health**, **Exec** (ours) and **Control**, and **Stop all** on top.

**10.2** Start any Exec on the desktop page (B21_H_open again is enough), then open the phone page's **Exec** tab.

**Expect:** it shows the same prompt with **Confirm** and **Cancel**. Confirm from the phone: the desktop's monitor pane
shows the same result as a desktop Confirm.

---

## 11. The run record, a restart, and a new run (ours)

**11.1** Open the run folder:
`~/husky_fake_lab/drive/data_design_study/261008_2ModulesDemo_purged_process/runs/<date>_<time>/`.

**Expect:**
- `run.json`, `progress.json` and `events.jsonl`;
- `movements/<action>/<movement>.<n>.json`, one file per part, including the refused and cancelled ones. A planned
  motion's file holds its plan's basis, with `obstacles_collide` (the box as it was at Plan).

**11.2** In the monitor pane:
1. Press **Ctrl-C once** and wait for `(viser) Server stopped` twice.
2. Press **Up**, then **Enter**, to start the monitor again.
3. Reload the page and click **Load**.

**Expect:**
- `continuing run <date>_<time>`;
- the done actions are still done;
- **Rhino obstacles collide** is unticked again: the box always starts off.

The robots keep where they are across the restart, so there's no need to drive again.

**11.3** In **assembly_run**, click Run **New run**. It asks in the panel's Prompt row (`Start a new run? Every action
starts pending; the old run is kept.`); click **Confirm** there.

**Expect:**
- a new run folder, with `0/78 done` and current B20_J;
- the old folder stays.

---

## 12. Stop

1. Press Ctrl-C once in the monitor pane and wait for `Server stopped`.
2. Press Ctrl-C in the top pane.
3. Run `tmux kill-session -t fake_lab`. This also stops the router and the relay.

---

## Known issues you will meet

These are not mistakes on your side.

| What you see | Why |
|---|---|
| The odd monitor start (pytest imported). | A workaround for compas_fab's stdout race (see section 1). It needs a real fix before any lab use: decision 1 afterwards. |
| Rhino's cabinet `env39` is in a different place from the measured one. | Rhino's cabinet (y 1.63–2.23 m) is about 0.38 m nearer the park row (y = 1.35 m) than the probe-measured cabinet (`data/lab/main.json`, from y = 2.00 m). With **Rhino obstacles collide** ticked, the base planner refuses any start or target on the park row (4.1); unticked, the default, the row is fine, and the robots start on it. The Rhino file needs the fix for Rhino's own planning. |
| Cindy's base path is refused while the process shows a movement where she holds a bar (4.2). | The base planner counts a held bar as part of the robot, and at `B20_J_transfer` the design has B20 in her grippers; from her start it reaches the measured cabinet. With the transfer workaround, the bar isn't actually there while she drives, but the base planner doesn't know the workaround. Show the action's start (**◀ action**) first. |
| Grasp and Robotiq steps are refused. | The simulation has no tool drivers. |
| Insert, retreat and handoff are refused; the servo logs "no servo correction (not supported yet)". | M6 and M7 are specified, not built. |
| B20_R_home won't plan until Cindy's grippers are jogged off the bar (6.2). | In the simulation the insert and retreat never ran. |
| Don't press **Cancel** in base_planner_panel right after a **Commit** while that robot is still driving an earlier path. | A race in Jakob's code (review finding M1, your decision 3a): within about one tick, the earlier path can keep driving with nothing watching it. Stop a driving base with **Stop all**. |
| After a link comes back (8.3), a burst of red `[rmw_zenoh_cpp] Query queue depth of 10 reached …` lines. | Jakob's controller-manager check asks once a second without waiting for the answer; the questions pile up while the link is down. Harmless; a fix is Jakob's. |
| A base's `signal` chip turns red now and then (hover: a gap). | The monitor's own callbacks run late on a busy PC; the simulator publishes at a steady 50 Hz. Close other simulations and test runs, and run the monitor under the performance power profile. |
| While a robot's link is down (8.1), its mocap pose keeps arriving, but ever older. | The fake Motive keeps publishing the frozen base, as Motive keeps tracking a robot whose WiFi dropped, but with the stamp of its last true pose. A fix in the fake Motive is Jakob's. |
| A robot's arm stays red after `link.sh … up`. | The link was down for more than 60 s, and Zenoh's sessions expired (section 8). Restart the fake lab. |
| process needs **Load**. | The README says it loads at start; the code waits for the button. A doc fix. |
| A plan made in base_planner_panel or **Plan** → **Arm** before you change the Rhino obstacles box is kept. | Jakob's planners don't replan on a scene change (the same as when an obstacle is edited). Replan after changing the box. The assembly steps' plans do go stale (7.3). |
| B20's transfer planned from Cindy's home failed 3 × 120 s once. | Alice and Belle stood next to Cindy with their arms straight out. With them away it solves in about 20 s. If Plan says "no path in 120 s", look at what stands in the way. |
| Not visible here: the force sensors are never zeroed while the transfer workaround is on. | The port zeroes them only after the load step, which the workaround skips. Today's old-monitor commit 979ffac fixed the same gap there; the port needs it too. |

---

## What to send back

Tell me the step numbers that did not go as written, and what you saw. A screenshot helps for anything on the page. I
read the monitor and robot panes myself, and the monitor log is at `~/husky_fake_lab/logs/`.

Then we go through the open decisions together, starting with the compas_fab fix.
