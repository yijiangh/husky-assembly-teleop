# Fake-lab test manual: the first three bars, end to end, on three simulated robots

This is the operator manual for the first three bars of the design problem **`261008_2ModulesDemo_purged_process`**
(the two-module demo, 78 actions) with the new monitor on three simulated robots on this PC: schedule entries 1–7,
**B20_J, B20_R, B21_J, B21_H, B21_R, B22_J, B22_R**. Every movement of every action runs as the process has it: the
base drives, the transfer, the bar placed by hand, the grasp, the servo, the insert, the retreat, the support robot's
approach, open, close and retreat. The simulated tools answer as the real ones do, the compliant parts switch the arms
to the compliance controller as in the lab, and nothing is skipped or faked. The real lab runs the same steps on the
robots (`doc/lab/real_lab_test_manual.md`). Plan about two hours.

> **?** Parts 4–6 run as written; with the Cartesian controllers built and the fake wrench on, so do the compliant
> insert (5.2) and, with the run's **Retreat compliant**, the retreat's compliant break-out; the retreat itself is the
> old stiff screw-out by default (4.4). The handoff (5.3 e) and Alice's open (5.3 c) still wait for
> `ros-humble-gripper-controllers` (1.a): those two say "not run yet" and give what the code does. Tell me where the
> page differs (Part 11).

- **First time:** read Parts 0–3 in order.
- **Done it before:** use the roadmap (0.2) as a checklist; each row links to its part.

How to read the steps:
- `- [ ]` is one thing to **do**, named by the control and what to do with it, with the page's own labels:
  `button [Plan]`, `slider [Preview time]`, `box [Dry run]: ticked`, `row [4. B21_H]`, `pane monitor: Enter`,
  `by hand: ...`, `by eye: ...`. A tab path is written `Plan › Base`; a button of a group `button [Load: Step]`. `→`
  joins controls you use one right after the other (`button [Load: Step] → button [Path: Plan]`).
- The indented lines under it:
  - **Expect:** the lines you should see in a terminal pane, quoted, with the pane named (`pane monitor:`); `...`
    stands for what varies. Only the few lines that tell the step went right.
  - **If not:** what to do when they don't come (or a pointer to Part 10).
  - A line without a label is a note: what the page or the 3D view shows, why, what to write down.
- `→` always means "then do the next thing"; it never stands for a result.
- **`▶ pane NAME`** above a code block says which terminal pane to paste it into (0.7). Paste the block as is: its
  last line, `###`, does nothing but marks in the pane where the command ends and its output begins. Blocks without a
  tag are output or diagrams, not commands.
- **!** starts a warning, **?** a side note.
- A badge in a heading says what kind of part it is:

| badge | means |
|---|---|
| `[SETUP — once]` | installs and builds, once before the first test (the build again after every pull); nothing moves |
| `[SETUP — every test]` | the fresh copy of the process, before every test |
| `[SIM ON — nothing moves]` | starting the simulated robots, the router and the relay |
| `[CHECK — nothing moves]` | the monitor's start, first look, Load |
| `[SIM RUN — recorded]` | the assembly on the simulated robots; every part is recorded in the run, `runs/<date>_<time>/` in the scratch Drive |

Steps are numbered so you can report them as, for example, "4.2 not OK: …".

---

## 0. Start here

### 0.1 What the test does

Three simulated huskies build the first three bars of the two-module demo:
- **Cindy** (dual-arm, two UR5e arms with scaffolding tools) joints B20, B21 and B22, and backs off after each.
- **Alice** (single-arm, Robotiq 2F-85 gripper) holds **B21** while Cindy joints it, and still holds it at the end.
- **Belle** (single-arm, Robotiq 2F-85) stays parked, so the planners see where she stands.

The robots are Jakob's simulation (`launch/sim.launch.py robot:=all`, [simulation_manual.md](simulation_manual.md)), on
crl-husky `jg/ros2` (the commit README names, or later): a simulated base with the real follower, the UR driver's fake
hardware for the arms, each arm with the robot's tool simulated (a Robotiq that reaches every goal at once, a
scaffolding tool whose screws stall a fixed time after a tighten), a fake Motive with Motive's measured latency, jitter
and marker errors, a fake force reading per arm, and the mocap relay, as on the lab PC. Everything talks over Zenoh
through a router of the fake lab's own.

**What you are testing, and whose it is:**

| Feature | Whose | Where |
|---|---|---|
| **The monitor.** The web page and phone page, health (a blue `sim` chip per simulated base, arm and tool), Stop all (latches each robot until its Resume), leases, the console at the bottom right, robot_control in the **Control** tab. | Jakob | 2–3, 8 |
| **The simulated robots and tools.** Bases, arms, a Robotiq and a scaffolding tool per `model.yaml`, the fake Motive and the relay. The fake-lab scripts around them are ours. | Jakob | 2 |
| **process.** Loads the process folder and draws one movement at a time; its action list carries the run's marks. | Jakob, the marks ours | 3 |
| **obstacles.** The measured lab from `data/lab/main.json`, with a floor the arms and what they hold may not cross. | Jakob | everywhere a plan is made |
| **base_planner_panel** (**Plan** › **Base**). A base path around robots and obstacles, committed to the follower. | Jakob | 4.1, 5.1, 5.3, 6.1 |
| **assembly_run.** The run folder: marks, where each robot was left, the settings (insert split, spiral, the spiral's seconds and turns, servo by bar, the Advanced planning search settings), the event log; Mark done, Reopen, New run. | **Ours** | 3–7 |
| **motion_planner.** Plans a step's motion at the live base through tamp, goals by exact IK (ssik), collision checks. | **Ours** | every Plan |
| **assembly_steps.** The run's current step in the **Assembly** tab: Live IK, Plan, Play, Exec; the Plan plots window. | **Ours** | 4–6 |
| **movement_exec.** Runs a step's parts (a motion on Exec, placing the bar by hand, a Robotiq step and the servo's first pass after Confirm), one motion at a time across all robots, with the compliant parts (insert, handoff), the stiff retreat and the servo after a transfer, an approach or a ground bar's insert. | **Ours** | 4–6 |

**The seven actions** (the process panel lists them with the transfer workaround on):

| # | action | who | what happens |
|---|---|---|---|
| 1 | `B20_J` | Cindy | B20 is the ground bar: transfer, the bar by hand, servo, **rigid** insert with no tighten, then a second servo |
| 2 | `B20_R` | Cindy | her jaws loosen while her tools back out at the screw rate, then home |
| 3 | `B21_J` | Cindy | B21 mates on its **left** joint only: rigid, then compliant while the left joint screw tightens |
| 4 | `B21_H` | **Alice** | approach and servo, open, 100 mm straight in, close with the compliant handoff |
| 5 | `B21_R` | Cindy | as 2, with Alice holding B21 |
| 6 | `B22_J` | Cindy | B22 mates on **both** joints: both screws tighten |
| 7 | `B22_R` | Cindy | as 2; then the current step is `B34_J` |

### 0.2 Roadmap (the whole test)

| # | what | part | pane | does anything move? | recorded in the run? |
|---|---|---|---|---|---|
| 1 | once: packages, the Cartesian controllers, the build | 1.a–1.c, 1.e | any terminal | no | no |
| 2 | a fresh copy of the process | 1.d | any terminal | no | no |
| 3 | the four panes: router, simulated robots, relay | 2 | sim, router, relay | no | no |
| 4 | the monitor: start, first look, Load, settings | 3 | monitor + page | no | no |
| 5 | B20: Cindy joints the ground bar, retreats | 4 | page | Cindy (simulated) | yes |
| 6 | B21: Cindy joints, Alice holds, Cindy retreats | 5 | page | Cindy, Alice | yes |
| 7 | B22: Cindy joints with both screws, retreats | 6 | page | Cindy | yes |
| 8 | the run record and a restart | 7 | monitor + page | no | read |
| 9 | optional: Cancel and Stop all during a motion | 8 | page | stops | yes |
| 10 | stop | 9 | all | no | — |
| 11 | send the results back | 11 | — | no | — |

Something wrong at any point: Part 10.

### 0.3 What you need

| what | where | note |
|---|---|---|
| this PC | any PC with the workspace: the RobotX PC (`CRLRobotxPC`) or Yijiang's laptop | everything runs on it |
| workspace | `~/Code/ros2_ws_next` (venv `venv`, ssik installed), on this manual's branch | crl-husky in `src/crl-husky` on `jg/ros2` (the commit README names, or later); fzi `cartesian_controllers` in `src/cartesian_controllers` |
| the Drive | read once by `drive.sh`, which copies the process to the scratch root `~/husky_fake_lab/drive` | the runs go to the scratch copy, never to the real Drive |
| ports | the router on `127.0.0.1:7470`, the monitor on 8080 and 8081 | the real lab's monitor uses 8080 and 8081 too: not both at once |
| logs | `~/husky_fake_lab/logs/` | the monitor's |

### 0.4 How it connects

```text
  this PC, ROS domain 70, everything on 127.0.0.1:
    router (Zenoh, 7470) ◄─┬─ sim: alice, belle, cindy: bases (the real follower), arms (UR fake hardware),
                           │       tools (Robotiq, scaffolding tool), the fake Motive, the fake wrench
                           ├─ relay: the fake Motive's bodies as the robots' base poses
                           └─ monitor: the web page (8080), the phone page (8081)
  Nothing connects to a robot: the lab's routers use port 7447 and domain 80.
```

### 0.5 Safety first

> ⚠️ Everything runs on ROS domain 70, from a clean environment. Every node connects only to the fake lab's own Zenoh
> router, which listens on the loopback only (port 7470; the lab's routers use 7447) and connects to no robot. The
> relay looks for Motive at 127.0.0.1 only. The robots in use can't see this test, and it can't see them. The
> monitor's Drive root is the scratch folder `~/husky_fake_lab/drive`, so its `runs/` never reach the real Drive.

- **!** Stop the fake lab (Part 9) before a real-lab session: its monitor holds ports 8080 and 8081.

### 0.6 Words used in this manual

| word | meaning |
|---|---|
| **monitor** | the operator program: a web page served by this PC (`monitor.sh`, 3.a). It drives all three robots |
| **action**, **movement** | an action is one unit of the schedule (`B20_J` Cindy joints B20); a movement (step) is one part of it. The step line under the buttons names the current one |
| **run** | the record of one assembly: `runs/<date>_<time>/` in the process folder: marks, where each robot was left, every part executed, the servo readings |
| **transfer workaround** | Cindy's transfer runs as a free move with empty tools to the insert start, where you place the bar. It stays on in this test: this export carries no joints for the loading pose (the travel to load has nothing to plan to), so the process cannot run its load, place, grasp and held transfer yet. When an export has the loading pose, untick the box and the five movements run in order |
| **servo** | after a transfer, a support approach or a ground bar's insert, the executor measures the flanges and moves them onto the authored targets, pass by pass, until they are within 0.2 mm. **Servo by bar** (on by default) steers a ground bar (B20 here) on its mocap fit; every other bar servos on tool0. The fake Motive streams no bar markers, so B20's servo asks to go on with tool0 (4.2) |
| **split** | the last millimetres of an insert run under the compliance controller (**Insert split mm**, 5 by default) |
| **ground bar / one-mate / two-mate** | B20 stands on ground joints: rigid insert, no tighten. B21 mates on its left joint only, B22 on both |
| **pane** | one of the four terminal areas of 0.7 |

### 0.7 The four terminal panes

`tmux.sh` (2.a) opens the tmux session **`fake_lab`**: one window, four panes, each with its number and name on its top
border.

```text
┌──────────────────────────────────────────────┐
│ 0 sim                                        │
├──────────────────────────────────────────────┤
│ 1 monitor                                    │
├──────────────────────┬───────────────────────┤
│ 2 router             │ 3 relay               │
└──────────────────────┴───────────────────────┘
```

| pane | for | tmux.sh left it |
|---|---|---|
| sim | the three simulated robots, their tools, the fake Motive and the fake wrench (`sim.sh`) | running |
| monitor | the monitor (`monitor.sh`) | typed in: press Enter in 3.a |
| router | the Zenoh router every node meets through (`router.sh`) | running, started first |
| relay | the mocap relay (`relay.sh`), as on the lab PC | running |

- I read the panes from my side (`tmux capture-pane`). I never type into them unless you ask.
- **Leave the session:** `Ctrl-b d`. It keeps running. **Come back:** run `tmux.sh` again, or `tmux attach -t fake_lab`.
- A terminal for a command (3.e): `Ctrl-b "` splits a pane; the new one gets the session's settings.

### 0.8 The monitor page

Open in Chrome on this PC: http://localhost:8080 (3.b). One page at a time.

- **Main panel, top left:** Stop all, the health rows.
- **Side panel, right:** the tabs **Assembly**, **Collisions**, **Control** (a tab per robot) and **Plan** (**Base**,
  **Motion** and **Arm**, one per planner panel); **Obstacles** minimized at the right edge. A layout file
  (`~/.config/husky_monitor/layout.json`) may put Plan in a window of its own (`doc/lab/plugins.md`, Your own layout).
- **Assembly** tab, top to bottom: the **PROCESS** section (Folder, Load, the two boxes), the run, the action list
  and its `▲` / `▼`, the Step slider, `◀ mv` / `mv ▶`, the boxes for what the 3D view draws, the step line and the Exec
  line, the buttons **Live IK**, **Check tool0**, **Plan**, **Exec**, **Confirm** | **Cancel**, **Mark done** |
  **Reopen**, the settings, the closed **Advanced planning** folder.
- Three minimized bars at the bottom: **Plan plots** and **Exec plots** at the left, the **Console** at the right. Each
  opens by itself when it has something to show; the control at a bar's right end opens or closes it.

---

## 1. Prepare  [SETUP — once]

### 1.a Packages  [SETUP — once]

The UR driver, the Robotiq description and controllers, the gripper controllers, Zenoh's ROS middleware and tmux.

**▶ any terminal**
```bash
sudo apt install ros-humble-ur-robot-driver ros-humble-robotiq-description ros-humble-robotiq-controllers \
    ros-humble-gripper-controllers ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor tmux
###
```

**Expect:** apt's last line, `0 upgraded, N newly installed, ...`, or `... is already the newest version` per package.

Without them: the simulated arms do not start (no UR driver); `sim.launch.py robot:=all` stops at Alice with
`package 'robotiq_description' not found`; without the gripper controllers the Robotiq's spawner dies (`spawner
robotiq_gripper_controller … process has died`) and every Robotiq step is refused with `the gripper's action server is
not available`.

### 1.b The Cartesian controllers  [SETUP — once]

The insert's compliant part, a compliant retreat's break-out (the run's Retreat compliant) and the handoff switch the
arms to `cartesian_compliance_controller`, which the robots carry onboard and this workspace does not. The source is
cloned in `src/cartesian_controllers` (fzi `cartesian_controllers`, branch `ros2`; its simulation, tests, handles and
utilities carry a `COLCON_IGNORE`).

**▶ any terminal**
```bash
cd ~/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash
python3 -m colcon build --symlink-install --packages-select cartesian_controller_base cartesian_motion_controller \
    cartesian_force_controller cartesian_compliance_controller
###
```

**Expect:** the last line, `Summary: 4 packages finished [...]`.

Without them the simulated arms' spawner dies at start (four `process has died` lines in pane sim) and every compliant
part fails at the switch (`controller manager rejected the switch`).

### 1.c The fake force reading  [SETUP — once]

Nothing to do. The simulated arms report no force (the fake hardware reads NaN), so `sim.launch.py` also starts
`sim_wrench`, a near-zero force/torque reading per simulated arm, by default: the F/T plots, a compliant retreat's
contact check and the servo's load guard then read something. `wrench_bias:=0,0,-5,0,0,0` adds a standing reading,
`contact_n:=8` a push along the tool's z while a simulated screw is stalled; keep both small, the compliance controller
reads the same topic and moves the arm by it.

### 1.d The test process  [SETUP — every test]

Copies the test process into the scratch Drive root. `--fresh` also deletes the runs of an earlier test.

**▶ any terminal**
```bash
~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/fake_lab/drive.sh --fresh
###
```

**Expect:** `drive.sh: deleted the copy's runs/` (when there were any), then
`drive.sh: .../husky_fake_lab/drive/data_design_study/261008_2ModulesDemo_purged_process is ready (84M)`.

**If not:** `drive.sh: no .../process.json under '...'; set HUSKY_DRIVE_ROOT to your Drive folder`: Insync has not
synced the process, or the Drive is somewhere else.

### 1.e Build the workspace (after every pull)  [SETUP — once]

From this branch, with crl-husky on `jg/ros2` (the commit README names, or later): the fake lab runs the installed
relay, fake Motive, fake wrench and simulated tools, and `env.sh` refuses an install without them.

**▶ any terminal**
```bash
cd ~/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash
python3 -m colcon build --symlink-install
###
```

**Expect:** the last line, `Summary: N packages finished [...]`, with no `failed`.

**If not:** `Duplicate package names not supported: - husky_assembly_teleop: - src/husky-assembly-teleop -
worktrees/...`: a git worktree inside the workspace. colcon skips a folder with a `COLCON_IGNORE` file in it:
`touch ~/Code/ros2_ws_next/worktrees/COLCON_IGNORE`, then build again.

An install older than a script the launch needs dies later with `executable ... not found`: build again.

---

## 2. Bring up the simulated lab  [SIM ON — nothing moves]

### 2.a The four panes  [SIM ON — nothing moves]

- [ ] In any terminal, run:

**▶ any terminal**
```bash
~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/fake_lab/tmux.sh
###
```

The session `fake_lab` opens (0.7); the router starts first, then the relay and the simulated robots.

- [ ] pane router: its first line
  - **Expect:** `fake lab: Zenoh router for ROS domain 70, on 127.0.0.1:7470 only`.
- [ ] pane sim: wait until it goes quiet (about 20 s)
  - **Expect:**
    - `fake lab: alice, belle and cindy on ROS domain 70, Zenoh router 127.0.0.1:7470`;
    - many `Configured and activated ...` lines, one per controller of each arm;
    - `announcing left_gripper as simulated (scaffolding tool sim)`, the same for `right_gripper`;
    - `sim_motive: rigid body 1840: faking it, noise [1.5e-05, 0.003], marker_error 0.0005`, the same for 1850 and
      1860.
  - **If not:**
    - `Failed loading controller joint_state_broadcaster` (three robots starting at once can starve one; the arm's
      health chips then stay red): `Ctrl-C` in pane sim, then Up, Enter: `sim.sh` again.
    - `fake lab: N simulator nodes of an earlier start still run on domain 70 ...`: Part 10.
- [ ] pane relay
  - **Expect:** `fake lab: mocap relay on ROS domain 70, Zenoh router 127.0.0.1:7470, no Motive`, then once the robots
    are up `rigid body 1840: now from the fake Motive, was the simulator's truth`, the same for 1850 and 1860.
  - A `No NatNet frames … reconnecting` line every 30 s is right: no Motive runs here.

---

## 3. The monitor  [CHECK — nothing moves]

### 3.a Start it  [CHECK — nothing moves]

- [ ] pane monitor: Enter (`monitor.sh` is typed in), once pane sim is quiet
  - **Expect:** pane monitor: `fake lab: open http://localhost:8080 (phone page: port 8081); log: ...`, then about
    10 s later `started in ... ms`.
  - **If not:** `fake lab: no process copy yet; run .../drive.sh first`: 1.d.
  - PyBullet's own lines print in this pane too (Part 10 says why).

### 3.b First look  [CHECK — nothing moves]

- [ ] Chrome: http://localhost:8080; keep that one page open
- [ ] by eye: the 3D view: three robots side by side on their park row, y = 1.35 m (Alice at x = 0.3, Belle at 1.7,
      Cindy at 3.1), facing +y, arms stowed; the measured lab (cabinet, tables, tripods, trusses, the orange border).
- [ ] by eye: health, top left: every base shows `sim`, `signal`, `mocap`, `estop`, `battery 90%` and `ctrl`; every
      arm `sim`, `signal`, `arm`, then its tool after a `sim` chip of its own: `sim` `robotiq` on Alice and Belle,
      `sim` `scaffolding_v3` on each of Cindy's arms
  - The `sim` chips are blue: a simulated part is no problem, the chip only says which it is (hover it for the
    simulator). Belle's `mocap` is amber: her fake Motive body reports the 1.2 mm marker error measured in the lab
    (hover it). So `1 PROBLEM`: Belle's `mocap`.
- [ ] by eye: the side panel as in 0.8; Assembly open, its action list empty.

### 3.c Load the process  [CHECK — nothing moves]

- [ ] Assembly › PROCESS: the folder is filled in → button [Load] once; wait 5–10 s
  - **Expect:** pane monitor: `loaded .../261008_2ModulesDemo_purged_process in ...; 78 actions, 273 movements`, a
    warning `a process robot differs from the configured one: ...`, then `no run yet: the first mark starts one`.
  - The warnings are right: this export used the old UR frames; the monitor plans with the configured robots.
  - Every Load draws the whole process again, and a page that opens later first receives about 200 bodies (20 s):
    don't click Load twice.
- [ ] by eye: the page shows
  - a green chip `261008_2ModulesDemo_purged_process`, `78 actions · 273 movements`, and an amber `robots differ`;
  - **ACTIONS** `1/78`: `1. B20_J · cindy · 4 mv · pending` on top, in blue, then `2. B20_R · cindy · 3 mv`,
    `3. B21_J`, `4. B21_H · alice · 4 mv`, `5. B21_R`, `6. B22_J`, `7. B22_R`, every row ending in `· pending`;
  - under the buttons, `current B20_J · B20_J_transfer (4/5)`, `no run yet`, `0/78 done`, and `alice`, `belle`,
    `cindy` each `live now`.
- [ ] The two boxes under Load: box [Transfer workaround]: ticked (0.6; B20_J is listed as `J_transit`, `J_M1`,
      `J_M2`, `J_M4`, 4 mv); list [Rhino obstacles]: `ghost`, with the grey chip `Rhino obstacles: drawn
      only`. Leave both.
  - Rhino's cabinet `env39` is drawn see-through and left out of every collision check: it sits 0.38 m nearer the
    park row than the measured cabinet, and ticked it refuses any base pose on the row.

**The process panel picks the run's current step.** Click an action row or a movement button, drag the Step slider or
click `◀ mv` / `mv ▶`: the step under the buttons follows, except while an Exec has the robots' leases.

### 3.d The settings  [CHECK — nothing moves]

- [ ] The settings, under Mark done: Ignore built bars off, Insert split mm 5, Spiral off, Spiral s 20, Spiral turns 4,
      Servo by bar **on** (live only on a ground bar's action, B20_J here; greyed and unticked on any other), Dry run
      off. Leave them so.
  - A run that saved Servo by bar off keeps it off: tick it on B20_J. There is no Rigid insert box: a ground bar always
    inserts rigid, every other bar rigid then compliant.

### 3.e Optional: slower simulated screws  [CHECK — nothing moves]

The simulated screws stall 2 s after a tighten whatever the arms do, so an insert's 5 s path is held 40 % in. To see an
insert reach its end and stall in the hold, give the screws more time:

- [ ] `Ctrl-b "` in any pane of the session: a new terminal below it, with the session's settings.

**▶ the new pane**
```bash
source ~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/fake_lab/env.sh
ros2 param set --no-daemon /a200_0806/left_gripper/scaffolding_tool_sim tighten_s 8
ros2 param set --no-daemon /a200_0806/right_gripper/scaffolding_tool_sim tighten_s 8
###
```

**Expect:** `Set parameter successful` twice.

`--no-daemon` skips ros2's background helper, which can stop working and then fails every command with `!rclpy.ok()`.

---

## 4. B20: Cindy joints the ground bar, then retreats  [SIM RUN — recorded]

As in the old monitor, Exec sends a planned motion at once; Confirm is asked only for the bar (placed and clamped, one
prompt), a Robotiq step and the servo's first pass.

### 4.1 Cindy's base  [SIM RUN — recorded]

- [ ] Plan › Base: button [0806] → button [Load: Step] → button [Path: Plan]
  - The panel shows `target +0.527 -3.067 +89.6°`, then after about 2 s `Lattice (hybrid A*) (straight): 5.24 m,
    29.0 s`.
- [ ] button [Commit]
  - **Expect:** pane monitor: `base path N sent to a200-0806: 5.24 m, ... waypoints`, then after about 50 s
    `base path N on a200-0806: done`.
  - The panel says `sent to 0806's follower`; Cindy drives in the 3D view. Control › `0806` › Base then reads
    about `xyz +0.53 -3.06`, `yaw +88°`: the simulated follower stops within about 1 cm and 2°.

### 4.2 B20_J, step 1/2: the transfer, the bar, the grasp, the servo  [SIM RUN — recorded]

- [ ] Assembly › by eye: the step line reads `B20_J_transfer (transfer_free) · then: B20_J_place_bar, B20_J_grasp ·
      skipped: B20_J_load`, and the Exec line `Exec: cindy (0806) moves left_ur_arm, right_ur_arm; then waits
      for B20_J_place_bar + B20_J_grasp by hand; then servos left_ur_arm, right_ur_arm onto the insert start`
- [ ] button [Live IK: insert start] (about 5 s)
  - **Expect:** pane monitor: `check goal B20_J_transfer: goal found (ssik, rank 1 of 64), max |goal - authored| 0.06
    rad`, and `started run .../runs/<date>_<time>`.
  - A run folder is made at the first thing it records. The 0.06 rad is the base 2° off the design base.
- [ ] button [Plan] (15–40 s)
  - **Expect:** pane monitor: `transfer_free solved by tamp.plan_free_dual_arm: 66 waypoints, 1 tries`.
  - The page shows the chip `plan ready` and `min 16.8 s by the arms' limits`. The **Plan plots** window opens at the
    bottom, its two plots filled: every joint along the preview, and the largest joint step per row. Close it
    with the control at its bar's right end, or leave it: it only opens again at the next plan.
- [ ] button [Path: Play]: the preview runs as it will move; slider [Preview time] scrubs it.
- [ ] button [Exec]: both arms move at once, for 17 s
  - **Expect:** pane monitor: `cindy (0806): B20_J_transfer — move left_ur_arm, right_ur_arm along the planned
    path (Rhino obstacles not checked), 17 s, largest joint travel 214°` (a warning: over 60°), then
    `B20_J_transfer: arrived, end within 0.00°`, then the one prompt for the bar: `B20_J_place_bar + B20_J_grasp —
    Operator places the bar by hand; clamp it: tighten the gripper screws of left_ur_arm, right_ur_arm with the tools'
    buttons until they stall, then Confirm …`.
  - The monitor sends the tools nothing: in the lab you place the bar, press each tool's button and check the bar is
    held before you Confirm.
- [ ] button [Confirm] (here or on the phone's Exec tab: the bar is placed and clamped, in the simulation's mind)
  - **Expect:** pane monitor: `[servo] feedback: the mocap bar bars/B20 (converges at 0.5 mm / 0.1 deg).`, a warning
    `no marker sample of bars/B20 (asset 1020) within 1 s: is Motive streaming it?`, then the warning prompt
    `B20_J_transfer — SERVO by bar: no usable mocap fit of B20 (asset 1020): the bar could not be measured (no marker
    sample). Tool0 servo (Confirm): servo tool0 onto the authored targets of the insert start instead (...). Give up
    (Cancel): stops, recording.`, in a warning window beside the main panel with buttons [Give up] and [Tool0 servo]
  - The servo starts by itself. The fake Motive streams no bar markers, so the warnings are right. The **Exec plots**
    window opens at the bottom with the servo's plots (one point per pass, the phase as a chip, the last pass's
    numbers under each plot).
- [ ] button [Confirm] (tool0 it is, here)
  - **Expect:** pane monitor: `### SERVOING iteration 1/8: aiming tool0 at the authored targets of the insert start
    (the mocap bar B20 is not usable: ...)`, `### SERVOING stopped at iter 1: nothing left to plan (correction L …
    mm, R … mm, under 0.2 mm)`, `Servoing data saved to .../servo/B20_J/B20_J_transfer.1.json (its .png beside it)`
    and `Exec B20_J_transfer: done — every part done`.
  - Also `### bar pose at termination (bars/B20): FK-vs-design 0.11 mm/0.007°, mocap-vs-design n/a, mocap-vs-FK n/a`.
  - The flanges are already within 0.2 mm, so no servo pass moves. A pass that would move asks for Confirm first
    (`SERVO 1/8 onto the insert start (tool0): …`) and logs `[servo] First move planned: ...`. The exec section's
    last line keeps `servo done: nothing left to plan (correction L … mm, R … mm, under 0.2 mm)` until the next Exec.

### 4.3 B20_J, step 2/2: the insert, and the servo after it  [SIM RUN — recorded]

The step line reads `B20_J_insert (insert)`.

- [ ] button [Plan] (about 3 s)
  - **Expect:** pane monitor: `insert solved by tamp.plan_constrained_dual_arm_linear: 4 waypoints`.
- [ ] button [Exec]: the insert runs at once
  - **Expect:** pane monitor, in order:
    - `B20_J_insert — RIGID insert of a ground bar, no tighten, 5.0 s, largest joint travel 2.5°.`;
    - `insert: RIGID_DONE: ground bar in, no tighten; the flanges stopped 0.0 mm from the planned end; peak |F| not
      measured`;
    - the servo after the ground bar's insert: `nothing left to plan`;
    - `Exec B20_J_insert: done — every part done`.
  - A ground bar has no mating joint: nothing tightens and the compliance controller is not engaged.
  - Without a force reading the servo warns `no force reading: the servo's load guard is OFF. Watch the arms and use
    Cancel if the bar pushes on the floor.` (seen on 2026-10-26, before the fake wrench of 1.c).
- [ ] by eye: the line under the buttons says `every step of B20_J done: Mark done below` → button [B20_J done]
  - **Expect:** pane monitor: `B20_J marked done in run <date>_<time>; ...`
  - The row `1. B20_J` turns green `· done`, `1/78 done`, and the current step becomes `B20_R · B20_R_ungrasp (1/3)`.

### 4.4 B20_R, step 1/2: the retreat, with the ungrasp during it  [SIM RUN — recorded]

The step line reads `B20_R_retreat (retreat_linear) · with: B20_R_ungrasp`.

- [ ] button [Plan]
  - **Expect:** pane monitor: `[retreat live] B20_R_retreat: goals = the live tool0 poses moved 50 mm back along
    their own -z (encoders): left along (…), 0.0 deg from the Rhino axis, 0.x mm from the exported goal; right …`,
    then `retreat_linear solved by tamp.plan_dual_arm_linear_independent: 12 waypoints`.
- [ ] button [Exec]: the retreat runs at once (not run yet with this text: it says what the code does)
  - **Expect:** pane monitor, in order:
    - `B20_R_retreat — RETREAT, stiff: the M1 screws of left_ur_arm + right_ur_arm LOOSEN while both tools back
      straight out 12.0 mm at 2.40 mm/s (5.0 s, joint controller), then 38 mm rigid (3.8 s). ...`;
    - `retreat: LOOSEN the gripper screws of left_ur_arm + right_ur_arm`;
    - `screw-out done after 5.0 s (gripper motors L/R LOOSENING/LOOSENING, still loosening); now the rest of the way
      out.`;
    - `retreat: RETREATED: retreated 50 mm; every check passed; the gripper motors stopped (L/R LOOSENING/LOOSENING at
      the end)`;
    - `Exec B20_R_retreat: done — every part done`.
  - Before anything is sent, the checks print `PASS start`, `PASS` for both tool0 paths (50 mm, ends 0.1 mm from the
    goal), the joints and the speed. Also `screw-out t=0.1s: gripper motors L/R LOOSENING/LOOSENING` and
    `rigid chunk: N rows over 3.8 s`. No force zero and no controller switch.
  - The ungrasp counts as done with it. The **Exec plots** window opens with the force and torque of the part (its
    servo plots hidden meanwhile).

### 4.5 B20_R, step 2/2: home  [SIM RUN — recorded]

- [ ] button [Go: mv ▶] (the process section): the step line reads `B20_R_home (free_home)` → button [Plan] (20–40 s)
  - **Expect:** pane monitor: `free_home solved by tamp.plan_free_dual_arm: ...`
  - The page shows `min 26 s by the arms' limits`.
- [ ] button [Exec]: both arms move at once
  - **Expect:** pane monitor: `B20_R_home — move left_ur_arm, right_ur_arm along the planned path …, 26 s, largest
    joint travel 222°`, then after 26 s `B20_R_home: arrived, end within 0.00°`.
- [ ] button [B20_R done]
  - **Expect:** pane monitor: `B20_R marked done in run <date>_<time>; ...`
  - The page: `2/78 done`, the current step `B21_J · B21_J_transfer (4/5)`.

**!** Mark done accepts an action with steps left: wait for `every step of … done` before clicking it, or the next base
plan refuses because an arm still stands at the bar.

---

## 5. B21: Cindy joints the bar, Alice holds it, Cindy retreats  [SIM RUN — recorded]

### 5.1 Cindy's base  [SIM RUN — recorded]

- [ ] Plan › Base: button [0806] → button [Load: Step] → button [Path: Plan] → button [Commit]
  - **Expect:** pane monitor: `base path N sent to a200-0806: 0.77 m, ... waypoints`, then after about 30 s
    `base path N on a200-0806: done`.
  - **If not:** `base path N on a200-0806: timed out: still following`: the simulated follower sometimes never starts
    a short path. button [Path: Plan] and button [Commit] again.
  - The panel shows `target +0.947 -3.171 +90.0°` and a path of about `0.77 m, 11 s`. Control › `0806` › Base ends at
    about `xyz +0.95 -3.18`, `yaw +92°`.

### 5.2 B21_J  [SIM RUN — recorded]

As 4.2 and 4.3, with these differences:

- [ ] button [Live IK: insert start]
  - **Expect:** pane monitor: `check goal B21_J_transfer: goal found (ssik, rank 1 of 64), max |goal - authored| 0.09
    rad`.
  - The transfer's plan takes 19 s.
- [ ] the insert, button [Exec]: it runs at once
  - **Expect:** pane monitor, in order:
    - `B21_J_insert — INSERT: rigid 10.0 mm (3.3 s), then COMPLIANT 5.0 mm (straight line); the joint screw of
      left_ur_arm TIGHTENS from the start until it stalls (≤ 30 s), then STOP and back to joint control. ...`;
    - `insert: left_ur_arm joint screw TIGHTENING`;
    - `cartesian_compliance_controller running (0.3 s)`;
    - `left_ur_arm joint screw STALLED at 0.0 of 1.7 s; left_ur_arm back under joint control, right_ur_arm finish the
      line under compliance`;
    - a warning `the arms did NOT reach the line's end: left_ur_arm 0.1 of 8.0 mm, … right_ur_arm 7.8 of 8.1 mm`;
    - `insert: STALLED: left_ur_arm joint screw STALLED; peak |F| left_ur_arm 0.2 N, right_ur_arm 0.2 N`;
    - `Exec B21_J_insert: done`.
  - The simulated screw stalls 2 s after the tighten (3.e), so at the switch itself; with `tighten_s 8` the left arm
    runs its compliant line before the stall. Once in the pane the first switch back was `rejected` and taken 0.2 s
    later: a retry, not a fault.
  - No servo after this insert: it is not a ground bar.
- [ ] button [B21_J done]
  - **Expect:** pane monitor: `B21_J marked done in run <date>_<time>; ...`
  - The page: `3/78 done`, the current step `B21_H · B21_H_approach (1/4)`.

### 5.3 B21_H: Alice holds B21  [SIM RUN — recorded]

**a) Alice's base**
- [ ] Plan › Base: button [0804] → button [Load: Step] → button [Path: Plan] → button [Commit]
  - **Expect:** pane monitor: `base path N sent to a200-0804: ...`, then after about 50 s `base path N on a200-0804:
    done`.
  - The panel shows `target +0.638 -1.561 -29.6°` and a path of about 3 m. Control › `0804` › Base at about
    `xyz +0.64 -1.56`, `yaw -31°`.

**b) Step 1/4, the approach and its servo**
- [ ] button [Live IK: linear start] (the first plan for Alice builds her planning cell, about 7 s)
  - **Expect:** pane monitor: `check goal B21_H_approach: goal found (ssik, rank 1 of 8), max |goal - authored| 0.08
    rad`.
- [ ] button [Plan]
  - **Expect:** pane monitor: `support_approach solved by arm.plan_arm: 2 waypoints`.
- [ ] button [Exec]: the arm moves at once
  - **Expect:** pane monitor: `alice (0804): B21_H_approach — move ur_arm along the planned path …, 16 s, largest
    joint travel 206°`, `B21_H_approach: arrived, end within 0.00°`, then the servo: `[servo] feedback: tool0 (0.2 mm);
    no mocap bar recorded (no bar held).`, `nothing left to plan`, `Exec B21_H_approach: done — every part done`.
  - The jaws stop short of B21.

**c) Step 2/4, open**
- [ ] button [Exec] → the prompt `B21_H_open — OPEN the gripper (0.426). Confirm to send.` → button [Confirm]
  - **Expect:** pane monitor: `B21_H_open: at 0.426 rad`, at once: the simulated Robotiq reaches every goal at once.
  - **If not** (not run yet here): without `ros-humble-gripper-controllers` (1.a) it is refused with
    `/a200_0804/gripper: the gripper's action server is not available`; the step stays, `mv ▶` goes on.

**d) Step 3/4, to grasp**
- [ ] button [Plan] (`support_linear`) → button [Exec]: the arm moves at once
  - **Expect:** pane monitor: `B21_H_to_grasp — move ur_arm …, 5 s, largest joint travel 22°`, then `arrived, end
    within 0.00°`.

**e) Step 4/4, the close: the handoff**
- [ ] button [Exec]
  - **Expect:** pane monitor: `B21_H_close — CLOSE + compliant handoff on ur_arm: the arm goes COMPLIANT before the
    close (gives way ≤ 10 mm); jaws 0.8, effort 0.1, ≤ 30 s; then joint control. ...`
- [ ] button [Confirm] (not run yet here)
  - **Expect,** pane monitor, in order:
    - `a200-0804 ur_arm: handoff: force sensors zeroed: |F| ur_arm ... -> ...`;
    - `a200-0804 ur_arm: handoff: anchor X mm, Y° from the URDF's tool0 (limit 10 mm, 2°)`;
    - `ur_arm: cartesian_compliance_controller running (...)`;
    - `a200-0804 ur_arm: handoff: closing to 0.800 rad, effort 0.10, the arm compliant around its anchor ...`;
    - `ur_arm back under scaled_joint_trajectory_controller (... s)`;
    - `Exec B21_H_close: done`.
  - **If not:** refused as the open: `REFUSED: /a200_0804/gripper: the gripper's action server is not available;
    nothing sent` (1.a).
  - The arm does not move.

**f) Mark**
- [ ] button [B21_H done]
  - **Expect:** pane monitor: `B21_H marked done in run <date>_<time>; ...`
  - The page: `4/78 done`, the current step `B21_R · B21_R_ungrasp (1/3)`.

### 5.4 B21_R  [SIM RUN — recorded]

- [ ] As 4.4 and 4.5, with Alice holding B21: the retreat's lines and checks are the same. Then button [Go: mv ▶], the
      home move, button [B21_R done]
  - **Expect:** pane monitor: `B21_R marked done in run <date>_<time>; ...`
  - The page: `5/78 done`.

---

## 6. B22: Cindy joints the bar with both screws, then retreats  [SIM RUN — recorded]

### 6.1 Cindy's base  [SIM RUN — recorded]

- [ ] As 5.1
  - **Expect:** pane monitor: `base path N on a200-0806: done`.
  - The panel shows `target +1.166 -3.212 +95.8°` and a short path.

### 6.2 B22_J  [SIM RUN — recorded]

- [ ] As 5.2; the insert tightens both screws
  - **Expect:** pane monitor: `B22_J_insert — INSERT: ... the joint screw of left_ur_arm + right_ur_arm TIGHTENS ...`,
    and at its end `insert: STALLED: ...` once both have stalled.
  - **If not:** Plan says `no path in 120 s`: the transfer's plan from the retreat pose took up to 110 s here; Plan
    again.
- [ ] button [B22_J done]
  - **Expect:** pane monitor: `B22_J marked done in run <date>_<time>; ...`
  - The page: `6/78 done`.

### 6.3 B22_R  [SIM RUN — recorded]

- [ ] As 4.4 and 4.5 → button [B22_R done]
  - **Expect:** pane monitor: `B22_R marked done in run <date>_<time>; ...`
  - The page: `7/78 done`, and the current step is `B34_J`: the first three bars are built and the next bar is
    Belle's.

---

## 7. The run record and a restart  [CHECK — nothing moves]

### 7.1 The run folder

**▶ any terminal**
```bash
ls ~/husky_fake_lab/drive/data_design_study/261008_2ModulesDemo_purged_process/runs/*/
ls ~/husky_fake_lab/drive/data_design_study/261008_2ModulesDemo_purged_process/runs/*/movements/
###
```

**Expect:** `events.jsonl  movements  progress.json  run.json  servo`, then the action folders `B20_J  B20_R  B21_H
B21_J  B21_R  B22_J  B22_R`.

- `movements/<action>/<movement>.<n>.json`: one file per part, the refused and cancelled ones included (an insert's or
  retreat's file keeps its settings, phases, switches, stall times and checks).
- `servo/<action>/<movement>.<n>.json`, with a `.png` of each servo's readings.

### 7.2 Restart the monitor

- [ ] pane monitor: `Ctrl-C` once
  - **Expect:** pane monitor: `(viser) Server stopped` twice.
- [ ] pane monitor: Up, Enter → reload the page → button [Load]
  - **Expect:** pane monitor: `continuing run <date>_<time>`.
  - The done actions are still green; the settings are the run's (the workaround box on, Rhino obstacles off). The
    robots keep where they are across the restart.

### 7.3 A new run

- [ ] Under the run: button [Run: New run], then once more when it reads `New run?` (no time limit)
  - **Expect:** pane monitor: `started run .../runs/<date>_<time>`.
  - The page: `0/78 done`, every row pending; the old folder stays.

---

## 8. Optional: interruptions  [SIM RUN — recorded]

### 8.1 Cancel during a motion

- [ ] On any planned motion: button [Exec], then button [Cancel] (red) while the arms move
  - **Expect:** pane monitor: `cancelled: holding every arm`, then `Exec …: cancelled — … (operator)`.
  - The arms stop mid-path. Plan again from where they stopped.

### 8.2 Stop all during a motion

- [ ] button [Stop all (Esc)] at the top of the page while the arms move
  - **Expect:** pane monitor: `robot a200-0806: SOFT STOP: Stop all`, `cancelled: holding every arm`.
  - Every arm's `arm ctrl` chip under Health turns amber, its hover ending `sync: last stop: stopped by operator`; each robot stays stopped until its own
    button [Resume] under Health.

---

## 9. Stop

1. pane monitor: `Ctrl-C` once. **Expect:** `(viser) Server stopped`.
2. pane sim: `Ctrl-C`.
3. Any terminal: `tmux kill-session -t fake_lab`. This also stops the router and the relay.

---

## 10. If something goes wrong: the known issues

These are not mistakes on your side.

| What you see | Why, and what to do |
|---|---|
| PyBullet's lines in pane monitor. | `monitor.sh` imports pytest before the monitor: compas_fab closes stdout for a moment around every model it loads, other threads then fail with `OSError(16, 'Device or resource busy')`, and compas_fab skips that only under pytest. A real fix is your decision. |
| `robots differ` after Load. | The export used the old UR frames; the monitor plans with the configured robots. |
| An insert's screws stall after 2 s whatever the arms do; its path is held 40 % in. | The simulated tool's `tighten_s`, 2 s by default (3.e sets it). |
| A retreat's gripper motors read `LOOSENING` at the end, never `STALLED`. | The simulated screws loosen until a STOP instead of stalling fully open; the stiff retreat only reports them. A compliant retreat's contact check reads the fake wrench (1.c) and passes near zero. |
| A base's `xyz` flickers between two poses, pane relay says `rigid body 18xx: now from …` again and again. | A simulator of an earlier launch is still alive in the same domain (`pgrep -af husky_sim`: two with the same `__ns`); stop the old one. |
| `no marker sample of bars/B20 (asset 1020)`. | The fake Motive streams no bar markers; the servo runs on tool0 and records that the bar could not be measured. |
| A short base path `timed out: still following`. | The simulated follower sometimes never starts a path of a few decimetres. Plan and Commit again. |
| An arm's chips stay red after start. | Its spawners timed out while three robots started at once (`Failed loading controller joint_state_broadcaster` in pane sim). Restart `sim.sh`. |
| `sim.sh` refuses: `simulator nodes of an earlier start still run`. | A start that died part way (a missing package or script) left its nodes running. Run the `kill -INT …` it prints, then `sim.sh` again. Started anyway, a second controller manager under the same name answers for an arm at random: its `ctrl` chip flickers, Exec is refused with `scaled_joint_trajectory_controller is not active (active: None)`, and the new start's spawners report `Controller already loaded` then `no controller with this name exists`. |
| colcon: `Duplicate package names not supported`. | A git worktree inside the workspace: 1.e. |
| `executable ... not found` when the sim starts. | The install is older than a script the launch needs: build again (1.e). |
| Rhino's cabinet `env39` is 0.38 m nearer the park row than the measured one. | A Rhino-side fix; with **Rhino obstacles** not on `collide` (the default) it is drawn only. |
| The ground feet of B20 sit 15.6 mm into the floor at their design pose. | A Rhino-side fact. The planner lets a half mated to the ground touch the lab floor, as the core lets it touch the process's ground bodies; otherwise every ground bar's goal collides with the floor. |
| Mark done accepts an action with steps left. | It marks the action, not its steps. Wait for `every step of … done`. |
| Don't press **Cancel** in base_planner_panel right after a **Commit** while that robot still drives an earlier path. | A race in Jakob's code (review finding M1): within a tick, the earlier path can keep driving unwatched. Stop a driving base with **Stop all**. |

---

## 11. What to send back

Tell me the step numbers that did not go as written, and what you saw. Paste the pane's lines from the command to the
next prompt (the `###` line marks where its output starts). A screenshot helps for anything on the page. I read the
monitor and robot panes myself, and the monitor log is at `~/husky_fake_lab/logs/`. The compliant parts (5.2's insert,
5.3 e) and the stiff retreat (4.4) are the ones nobody has run in the simulation with this code yet: their log lines
matter most.
