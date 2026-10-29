# Cindy + Alice + Belle two-module test (B20–B59) — with the real robots, run from the robot-control PC

This is the operator manual for the design problem **`261023_2ModulesDemo`** (exported 2026-10-23
from `261019_2ModulesDemo.3dm`: **32 bars**, B20–B59 plus B165, B168, B169, in **76 actions**). Its
first 18 actions are the B20–B37 range run on 2026-10-20…22 from `261006_3bar_holding_test`, in the
same order. It is written for someone who has **never run this test**.

- **First time:** read Parts 0–5 in order.
- **Done it before:** use the roadmap (0.2) as a checklist; each row links to its part.

How to read the steps:
- `- [ ]` is one thing to **do**, named by the control and what to do with it, with the panel's own
  labels: `button [Plan Movement]`, `slider [Step]: 3`, `box [Workaround: ...]: ticked`,
  `joystick: ...`, `pane 1: ...`, `by hand: ...`, `by eye: ...`. `→` joins controls you use one
  right after the other (`slider [Step]: 3 → button [Load Movement]`).
- The indented lines under it: **Expect:** is what you should **see** (a log line in pane 1, or
  the 3D view); **If not:** says what to do when it doesn't (or points to Part 7). An indented
  line without a label is a side note.
- `→` always means "then do the next thing"; it never stands for a result.
- In Part 5 each procedure is split into lettered phases (a), b), ...); a table after a phase
  lists what differs per bar or per action.
- **`▶ pane N · NAME`** above a code block says which terminal pane to paste it into (0.7). Paste
  the block as is. Blocks without a tag are output or diagrams, not commands.
- **!** starts a warning, **?** a side note.
- A badge in a heading says what kind of part it is:

| badge | means |
|---|---|
| `[SETUP — once]` | settings and files, done once before the session; nothing moves |
| `[ROBOTS ON — nothing moves]` | powering and launching the robots |
| `[CHECK — nothing moves]` | read-only checks |
| `[TEST — arm / gripper move, no bar, nothing recorded]` | a test before the assembly; nothing is saved as done |
| `[REAL RUN — recorded]` | the assembly itself; every step is saved to `progress.json` |

Anything that does not match a "see" is worth a note (the log lines + what you did; Part 8). What
each monitor button does in detail: `doc/support_robot_plan_skeleton_manual.md`.

> **If you closed a terminal** (the five panes: 0.7)
>
> **The whole Ghostty window:** nothing is lost. tmux keeps all five panes running in the
> background (robot launches and monitor included). Open Ghostty again and run:
>
> **▶ any terminal**
> ```bash
> ~/ros2_ws/src/husky-assembly-teleop/scripts/husky/open_terminals.sh
> ```
>
> **Expect:** the `robots` window comes back exactly as it was (`tmux new -A -s husky_real` does the same).
>
> **One pane** (`exit`, `Ctrl-D` or `Ctrl-b x` in it): only that pane is gone; the other four keep
> running. What it stopped:
>
> | pane | what closing it stopped |
> |---|---|
> | 1 · MONITOR | the monitor, if one ran there. The robot's driver keeps running and its arms keep holding |
> | 2–4 · a robot's ssh | the ssh connection **and the launch in it**: that robot's arm, gripper / tool drivers are off. **!** Never close a robot's pane while it holds a bar |
> | 5 · CHECKS | nothing |
>
> To get the pane back without touching the others:
> 1. Click the pane next to where it was.
> 2. `Ctrl-b` then `"` (split below) or `Ctrl-b` then `%` (split right): a new, empty pane appears.
> 3. Give it its title back, e.g. for Alice's pane:
>
> **▶ the new pane**
> ```bash
> tmux select-pane -T "2 ALICE (ssh)"      # or "1 MONITOR", "3 BELLE (ssh)", "4 CINDY (ssh)", "5 CHECKS"
> ```
>
> 4. Redo what was in it: a robot pane: ssh + its launch block (Part 2); pane 1: the robot's
>    monitor start block (5.1); pane 5: the line below.
>
> **▶ pane 5 · CHECKS** (only for a new pane 5)
> ```bash
> source ~/ros2_ws/install/setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> ```

---

## 0. Start here

### 0.1 What the test does

Three mobile robots ("huskies") build 32 bars into a two-module structure:
- **Cindy** (dual-arm husky, two UR5e arms with scaffolding tools) joints and releases the bars.
- **Alice** (single-arm husky, Robotiq 2F-85 gripper) grabs **B21**, later **B25**, **B47** and **B50**,
  each after Cindy has inserted it.
- **Belle** (single-arm husky, Robotiq 2F-85 gripper) grabs **B35**, later **B39**.

A support robot holds its bar until the bars that close the loop are in, then lets go (the `HR`
actions: 16, 17, 40, 41, 55, 71).

**The story of one bar (B21):**
1. Cindy moves her two empty tools to where B21 will go (the *transfer*).
2. You mount B21 in her tools by hand and clamp it with the buttons on the tools.
3. Cindy corrects her pose by a few mm using mocap (the *servo*), then inserts B21: its left joint
   screws into B20.
4. Alice drives up, opens her gripper, slides it onto B21 and closes it (the *hold*).
5. Cindy lets go and backs off (the *release*). Alice holds B21 until B37 is in, then opens and
   backs off (the *let go*).

**The first 18 actions** (the rest, 18–75, in the run table below and in 5.13):

| # | action | who | what happens |
|---|---|---|---|
| 0 | `B20_J_joint` | Cindy | B20 jointing (ground bar): transfer → mount → grasp → insert (rigid, no tighten) → **fix foundation** (manual) |
| 1 | `B20_R_release` | Cindy | B20 release: retreat (the ungrasp runs with it) → free move home |
| 2 | `B21_J_joint` | Cindy | B21 jointing (one-sided, the **left** joint mates): transfer → mount → grasp → insert (the left tighten runs with it) |
| 3 | `B21_H_hold` | **Alice** | Alice grabs B21: free approach → gripper open → straight approach (100 mm) → gripper close + compliant handoff |
| 4 | `B21_R_release` | Cindy | B21 release, Alice holding B21 |
| 5 | `B22_J_joint` | Cindy | B22 jointing (both joints mate, to B21 and B20) |
| 6 | `B22_R_release` | Cindy | B22 release |
| 7 | `B34_J_joint` | Cindy | B34 jointing (ground bar, like B20): rigid insert, no tighten → **fix foundation** |
| 8 | `B34_R_release` | Cindy | B34 release |
| 9 | `B35_J_joint` | Cindy | B35 jointing (one-sided, the **left** joint mates, into B34) |
| 10 | `B35_H_hold` | **Belle** | Belle grabs B35: the same four steps as action 3 |
| 11 | `B35_R_release` | Cindy | B35 release, Belle holding B35 |
| 12 | `B36_J_joint` | Cindy | B36 jointing (both joints mate, to B35 and B34) |
| 13 | `B36_R_release` | Cindy | B36 release |
| 14 | `B37_J_joint` | Cindy | B37 jointing (both joints mate, to B21 held by Alice and B35 held by Belle) |
| 15 | `B37_R_release` | Cindy | B37 release |
| 16 | `B21_HR_hold_release` | **Alice** | Alice lets go of B21: gripper open → straight retreat (100 mm) |
| 17 | `B35_HR_hold_release` | **Belle** | Belle lets go of B35: gripper open → straight retreat (100 mm) |
| 18–75 | B25 … B168 | all three | the second module, the same four kinds of action; per run in 5.13 |

**One robot per monitor run.** The monitor (the operator program, 0.6) drives one robot at a time
and draws the other two from their live mocap base, or from what `progress.json` remembers. So
the session is 23 monitor runs in turn (a lab day covers a few of them; `progress.json` carries
over from one day to the next):

| run | robot | actions | part |
|---|---|---|---|
| 1 | Cindy | 0–2 | 5.6 |
| 2 | Alice | 3 | 5.7 |
| 3 | Cindy | 4–9 | 5.8 |
| 4 | Belle | 10 | 5.9 |
| 5 | Cindy | 11–15 | 5.10 |
| 6 | Alice | 16 | 5.11 |
| 7 | Belle | 17 | 5.12 |
| 8–23 | all three | 18–75 | 5.13 |

Quitting a monitor leaves the robot's driver running: the arms stay where they are and keep
holding.

### 0.2 Roadmap (the whole session)

| # | what | part | pane | robot | does anything move? | saved in `progress.json`**?** |
|---|---|---|---|---|---|---|
| 1 | open the five terminal panes | 0.7 | any terminal | — | no | no |
| 2 | settings, problem folder, network | 1 | 1 | — | no | no |
| 3 | pendants, robot launches, mocap | 2 | 2–4 | all | no | no |
| 4 | robot checks | 3 | 5 | all | no | no |
| 5 | tests without a bar: gripper (Belle, then Alice), spiral (Cindy) | 4.1, 4.2 | 1 + monitor window | Belle, Alice, Cindy | gripper; controller switch (the arm should stay still); Cindy's arms: to a bar-holding pose, 15 mm forward with the spiral, back | no |
| 6 | run 1: actions 0–2 | 5.6 | 1 + monitor window | Cindy | arms (+ base when driven) | yes |
| 7 | run 2: action 3 | 5.7 | 1 + monitor window | Alice | base, arm, gripper | yes |
| 8 | run 3: actions 4–9 | 5.8 | 1 + monitor window | Cindy | arms, base | yes |
| 9 | run 4: action 10 | 5.9 | 1 + monitor window | Belle | base, arm, gripper | yes |
| 10 | run 5: actions 11–15 | 5.10 | 1 + monitor window | Cindy | arms, base | yes |
| 11 | run 6: action 16 | 5.11 | 1 + monitor window | Alice | gripper, arm, then base away | yes |
| 12 | run 7: action 17 | 5.12 | 1 + monitor window | Belle | gripper, arm, then base away | yes |
| 13 | runs 8–23: actions 18–75 (the second module) | 5.13 | 1 + monitor window | all three | as runs 1–7 | yes |
| 14 | send the results back | 8 | — | — | no | — |
| 15 | the as-built page of the run (MoCap bars at every step) | 9 | any terminal | — | no | no |

Something wrong at any point: Part 7. What the first as-built run (2026-10-20) showed: Part 10
(temporary notes).

### 0.3 What you need

| what | where | note |
|---|---|---|
| this PC | `CRLRobotxPC`, user `su` | runs the monitor and the checks |
| workspace | `~/ros2_ws` (venv `~/ros2_ws/venv`, ssik 4.1 installed) | |
| shared drive | `~/Insync/2025-03 Husky Assembly` | the package's built-in data paths already point here; `progress.json` lives here (1.d) |
| robot network | this PC's built-in ethernet port **`eno1`**, DHCP from the router `192.168.0.1` (this PC is `192.168.0.201` since 2026-10-22, `192.168.0.25` before) | the Wi-Fi `wlp9s0` (+ Tailscale) is for the internet; ROS does not use them |
| DDS (how ROS talks) | `~/.cyclonedds.xml` pins CycloneDDS to `eno1`; `~/.bashrc` exports `CYCLONEDDS_URI` but **not** `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` | the monitor start blocks (5.1) set those two |
| Cindy's PC | `192.168.0.115`, user `administrator` | this PC's ssh key is accepted; on the lab network over Wi-Fi `wlp2s0` (her cable ports are her internal `br0`), so her launch pins DDS to it (2.b) |
| Alice's PC | `192.168.0.113`, user `administrator` | password `clearpath` if asked (`doc/husky_ros2.md`) |
| Belle's PC | `192.168.0.114`, user `administrator` | password `clearpath` if asked |
| Motive PC (mocap) | `192.168.0.28` | streams Cindy's rigid body **1860**, Alice's **1840**, Belle's **1850** |
| remote e-stops | one per robot | pair one husky at a time (`doc/husky_ros2.md`, troubleshooting) |

### 0.4 How the PCs connect

```text
  Motive PC 192.168.0.28 ---- mocap stream (robot base poses) ----+
                                                                   |
  Cindy's PC .115 --+                                              v
  Alice's PC .113 --+--- robot router 192.168.0.1 --- this PC (eno1, 192.168.0.201)
  Belle's PC .114 --+                                  runs the monitor + checks;
                                                       ssh into each robot PC
  Each robot PC runs its own "launch": the arm, gripper / tool and base drivers.
  Each robot has its own ROS domain (86 Cindy, 84 Alice, 85 Belle).
```

### 0.5 Safety first

| what | what it does |
|---|---|
| the **remote e-stop of the robot that moves** | in someone's hand for **every** arm motion; Cindy, Alice and Belle each have their own |
| the Husky e-stop | locks out the base; it does **not** stop the UR5e arms |
| `Cancel Exec` (monitor) | ends a step that waits |
| `Ctrl-C` in a robot's ssh pane (2–4) | stops that robot's arm and gripper / tool drivers. **!** Never while that robot holds a bar |

- **!** Preview every new motion first (drag `Traj viz time`), and raise the `traj time` slider for
  its first run. Loading a movement sets `traj time` to the movement's default (Part 6.b).
- **!** Never drive Alice while she holds B21 (actions 3–16), nor Belle while she holds B35
  (actions 10–17).
- **!** An arm left under the compliance controller: `Switch to Joint (BOTH)` (Part 6.b).

### 0.6 Words used in this manual

| word | meaning |
|---|---|
| **monitor** | the operator program (`ros2 run husky_assembly_teleop husky_monitor`): a button panel plus a 3D view (0.8). It drives one robot at a time |
| **run** (monitor run) | one start of the monitor for one robot, from `ros2 run` until you quit it. The assembly is 23 runs in turn (table in 0.1) |
| **action** | one unit of the plan: 76 here, numbered **0–75** in build order (table in 0.1; "action 0" = the first, Cindy jointing B20). `B20_J_joint` = Cindy joints B20. `J` jointing (insert), `R` release, `H` hold, `HR` hold release (let go) |
| **step** (movement) | one part of an action: a free move, a straight move, a gripper / tool step, or a manual step. The `Step` slider picks one; the readout shows `step i/n` |
| **plan skeleton** | the ordered list of all 76 actions, who does each and which are done (the `Actions` list in the panel) |
| **`progress.json`** | the file in the problem folder that holds which actions are done and each robot's last real pose. It is how one monitor run hands over to the next |
| **`Mark action done`** | records the action as done in `progress.json` and loads the next one |
| **Plan / preview / Exec** | `Plan Movement` computes a path; dragging `Traj viz time` previews it in the 3D view; `Exec Selected Mv Traj (auto)` runs it |
| **`Confirm Exec` / `Cancel Exec`** | answer a step that waits (its log line says `Click 'Confirm Exec' ...`) |
| **ROS domain** (`ROS_DOMAIN_ID`) | a number that keeps each robot's ROS traffic apart: **86 Cindy, 84 Alice, 85 Belle**. It decides which robot the monitor drives (5.1) |
| **namespace** | the prefix of a robot's ROS names: `/a200_0806` Cindy, `/a200_0804` Alice, `/a200_0805` Belle |
| **teach pendant** | the UR arm's tablet. Its **Local / Remote** switch (top right) must be on **Remote**, so the PC may run the arm |
| **`ros_control.urp`** | the arm program that hands the arm to ROS. The launch loads and plays it itself. `external_control.urp` is a different program: never load or start a program by hand |
| **launch** | the command started on each robot PC (in its ssh pane) that runs its drivers |
| **mocap**, **Motive**, **rigid body** | the camera tracking system. The Motive PC streams each robot base's marker body ("rigid body", by number) |
| **base calibration** | a file that turns the robot's mocap marker body into its real base pose (1.c) |
| **tool0** | the end of the arm (its flange), where the tool / gripper sits |
| **servo** | after a move, small corrections from the mocap base pose until tool0 is within 0.2 mm |
| **controllers** | how the arm follows commands: `scaled_joint_trajectory_controller` (stiff, follows joints; the normal one) or `cartesian_compliance_controller` (gives way to forces; only for the insert, the retreat and the grip handoff) |
| **ghost** | a blue figure in the 3D view showing a target pose |
| **transfer workaround** | a tick box (5.2): the transfer runs with empty tools, and the bar is mounted by hand at the insertion start |
| **ground bar / one-sided bar** | B20, B34 stand on ground joints: rigid insert, no tighten, then a manual foundation fix. B21, B35 mate on their **left** joint only |
| **tmux pane** | one of the five terminal areas of 0.7 |

### 0.7 The five terminal panes

The terminal app on this PC is **Ghostty**. Inside it, **tmux** splits the window into five
panes that keep running even when the Ghostty window closes (Part 6.i: why tmux).

- [ ] Open Ghostty and run:

**▶ any terminal**
```bash
~/ros2_ws/src/husky-assembly-teleop/scripts/husky/open_terminals.sh
```

**Expect:** five panes, each with its title on its top border and a one-line hint of what it is for:

```text
┌──────────────────────────┬────────────────┐
│ 1 MONITOR                │ 2 ALICE (ssh)  │
│  settings checks,        ├────────────────┤
│  then the monitor        │ 3 BELLE (ssh)  │
├──────────────────────────┼────────────────┤
│ 5 CHECKS                 │ 4 CINDY (ssh)  │
│  robot / joint checks,   │                │
│  base driving, recovery  │                │
└──────────────────────────┴────────────────┘
```

| pane | for | note |
|---|---|---|
| 1 MONITOR | settings checks (Part 1), then the monitor | everything shown here is also written to `~/husky_real/monitor.log` |
| 2 ALICE (ssh) | the ssh login to Alice's PC and her launch | stays running all session |
| 3 BELLE (ssh) | the same for Belle | |
| 4 CINDY (ssh) | the same for Cindy | |
| 5 CHECKS | `check_robots.sh`, finger / joint echoes, base driving, recovery commands | **!** never `export ROS_DOMAIN_ID` here: a command that needs a robot carries its domain on its own line |

- The script makes a tmux session `husky_real` (or, if it exists, adds a window `robots` to it,
  closing nothing). Run it again any time to come back to that window.
- The numbers 1–5 are the titles. tmux's own pane numbers (`Ctrl-b q`) differ: go by the titles.

The four tmux things you need (more in Part 6.i):

| do | what it does |
|---|---|
| click a pane | type into that pane |
| `Ctrl-b` then `z` | zoom the pane to the whole window, and back |
| mouse wheel; `q` | scroll back through a pane's output; back to the live end |
| `Ctrl-b` then `d` | leave tmux; everything keeps running. Back with the script above |

(`Ctrl-b` means: press Ctrl and b together, let go, then the next key.)

### 0.8 The monitor window

The monitor opens **two windows**: the **panel** (buttons and sliders, scroll down for more) and
the **3D view** (PyBullet). The panel, top to bottom (some sections depend on the flags of 1.a):

```text
┌ panel ───────────────────────────────────────────────────────────────┐
│ PLAN SKELETON                                                         │
│   plan skeleton: robot Cindy (domain 86) | ... | progress k/76 done   │
│   [Switch to Alice] [Switch to Belle] [Switch to Cindy (current)]     │
│   Action (idx) ──●──     -> action ...                                │
│   [Prev action] [Next action] [Load action] [Mark action done]        │
│   [Reopen action (reload clean)] [Rescan plan skeleton status]        │
│   [Reset plan skeleton to the Rhino export]                           │
│   [ ] Ignore built-bar collisions (bars drawn faint)                  │
│   now: Now: action k -- ... step i/n ...                              │
│   Actions ▸  (the 76 rows; click one to select it; grey = other robot)│
│   traj time ──●──    Traj viz time ──●──                              │
│   [Toggle Joint Live Stream] ...                                      │
│ BARACTION LIVE REPLAN & EXE                                           │
│   Step ──●──   -> movement ...   [Load Movement]                      │
│   [Transfer start: Confirm manual pose (IK check)]                    │
│   [Draw tool0: live (mocap + FK) vs target (Rhino)]                   │
│   [ ] Visual diagnosis (see-through robot, ...)                       │
│   [Plan Movement]                                                     │
│   [ ] Workaround: plan the transfer as a free move (no bar, ...)      │
│ MOVEMENT EXE                                                          │
│   [Exec Selected Mv Traj (auto)]  [<step button, e.g. Exec gripper    │
│    step: OPEN, Operator done (manual step) -> then Confirm Exec>]     │
│   [Confirm Exec] [Cancel Exec] [Zero Force Sensor (BOTH)]             │
│   [Toggle Servoing Tracker]  Insert: ... sliders                      │
│   [ ] Insert compliant part: SPIRAL search (...)    orange box (6.f)  │
│   [ ] Servo feedback: MoCap bar markers (...)       orange box (6.e)  │
│        both set by Load Movement; greyed when the step does not fit  │
│   [ ] Retreat screw-out: COMPLIANT (unticked = stiff ...)  (5.3 a)    │
│        unticked = the default; ticked only to test (5.3 a')          │
│   [Spiral test (no bar)]  (Cindy, Part 4.2)                           │
│   [Move Arms to Movement Start (offline target)]                      │
│ CONTROLLERS                                                           │
│   [Switch to Compliance (BOTH)] [Switch to Joint (BOTH)]              │
│ GRIPPER (Alice, Belle)   [Open Gripper Full] [Close Gripper for Bar]  │
│ SCAFFOLDING V3 (Cindy)   [Loosen Grippers (BOTH)] [Stop All (BOTH)]   │
│                          [Release test (BOTH)]  (doc/screw_rate_push_ │
│                          test.md)                                     │
│                          L / R: Stop All, Tighten / Loosen Gripper,   │
│                          Tighten / Loosen Joint                       │
│   ! a bar held by BOTH tools: only the (BOTH) buttons; one tool       │
│     letting go or stopping alone bends the bar                        │
│ DEBUG utils              [Remove all drawing]                         │
└───────────────────────────────────────────────────────────────────────┘
```

In the 3D view: the connected robot at its real base with its real arms; the other two robots
where mocap sees them (or where `progress.json` left them); red **obstacle robots** (what the
planner checks); the blue **goal ghost**; built bars drawn solid (faint when `Ignore built-bar
collisions` is ticked).

---

## 1. Prepare  [SETUP — once]

All in pane 1, before any monitor runs. Nothing moves.

### 1.a Monitor flags  [SETUP — once]

The flags are class attributes at the top of `class HuskyMonitor`
(`husky_assembly_teleop/husky_monitor.py`). The same values serve all three robots' runs.

**▶ pane 1 · MONITOR**
```bash
F=~/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_monitor.py
grep -nE "^    (FAKE_HARDWARE|USE_MOCAP|USE_CELL_STATE_BASE_POSE|CONNECT_COMPLIANT_CONTROLLER|LIST_CONTROLLER_SERVICES|IGNORE_BUILT_ASSEMBLY_COLLISIONS|TRANSFER_AS_FREE_MOVE|AUTO_LOAD_ACTION) *=" "$F"
grep -nE "^    (CALIBRATION|PUNCH_CALIB_VALIDATION|BAR_ACTION_LIVE_REPLAN_EXE|BAR_ACTION_MOCAP_ACCURACY_TEST|AS_BUILT_RECORD) *=" "$F"
```

**Expect:**

| flag | must be | why / note |
|---|---|---|
| `USE_MOCAP` | **1** | |
| `FAKE_HARDWARE` | **0** | |
| `LIST_CONTROLLER_SERVICES` | **1** | |
| `CONNECT_COMPLIANT_CONTROLLER` | **1** | Cindy's insert / retreat and the support robots' grip handoff switch controllers |
| `USE_CELL_STATE_BASE_POSE` | 0 | |
| `AUTO_LOAD_ACTION` | **1** | the first pending action loads by itself at start-up and after `Mark action done`; 0 = click `Load action` every time |
| `IGNORE_BUILT_ASSEMBLY_COLLISIONS` | 0 | built bars are obstacles; leave the panel toggle unticked, also for Alice and Belle: both holds plan with them, checked headless on this export |
| `TRANSFER_AS_FREE_MOVE` | 0 | tick the workaround box in the panel instead (5.2) |
| `CALIBRATION` | **0** | 1 only for a calibration session; Belle's on 2026-10-13/14 had both at 1 |
| `BAR_ACTION_LIVE_REPLAN_EXE` | **1** | 0 hides the whole bar-assembly panel, the plan skeleton included |
| `BAR_ACTION_MOCAP_ACCURACY_TEST` | 1 | the value before Belle's calibration; its extra buttons are not used in this test |
| `AS_BUILT_RECORD` | **1** (the default) | every MoCap bar is measured and saved, raw markers included, at the start and end of every executed step (Part 9). It records only when the design problem has a `MoCapMarkers.json` (9.1) |
| `PUNCH_CALIB_VALIDATION` | **0** | 1 only for a calibration session; Belle's on 2026-10-13/14 had both at 1 |

**!** To change a flag, edit the file and restart the monitor (no rebuild needed). A running monitor
keeps the values it started with.

### 1.b Mocap address  [SETUP — once]

`CLIENT_IP` is where the monitor listens for the mocap stream, so it differs per PC. `MOCAP_IP`
is the Motive PC.

**▶ pane 1 · MONITOR**
```bash
F=~/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_monitor.py
grep -nE "^(CLIENT_IP|MOCAP_IP) *=" "$F"
ip -4 -brief addr show eno1        # this PC's address on the robot network
```

**Expect:** the uncommented `CLIENT_IP` line is this PC's `eno1` address (`# husky PC`,
`192.168.0.201` since 2026-10-22, `192.168.0.25` before; the Razer laptop's `192.168.0.29` stays a comment), and
`MOCAP_IP = '192.168.0.28'` (the Motive PC).

**If not:** (both addresses are DHCP leases and can move: `doc/calibration_manual.md` section 1.4),
set `CLIENT_IP` to `eno1`'s address:

**▶ pane 1 · MONITOR**
```bash
F=~/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_monitor.py
IP=$(ip -4 -o addr show eno1 | awk '{print $4}' | cut -d/ -f1)
sed -i "s/^CLIENT_IP = '[0-9.]*'/CLIENT_IP = '$IP'/" "$F"
grep -n "^CLIENT_IP" "$F"          # expect: CLIENT_IP = '192.168.0.x' # husky PC ...
```

**!** With the wrong `CLIENT_IP` the monitor prints `mocap client connected: False` and sees no robot
base.

### 1.c Base calibrations  [SETUP — once]

A base calibration turns a robot's mocap marker body into its real base. Each robot reads its own
date folder from `BASE_CALIBRATION_DATE_BY_ROBOT` in `husky_assembly_teleop/__init__.py` (else
`CALIBRATION_DATE`).

**▶ pane 1 · MONITOR**
```bash
I=~/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/__init__.py
grep -nE "^CALIBRATION_DATE|^BASE_CALIBRATION_DATE_BY_ROBOT|^    '(Alice|Belle|Cindy)'" "$I"
C="$HOME/Insync/2025-03 Husky Assembly/data_experiment/calibration_data"
ls "$C"/20261016/calibrated_transformation_0806_rhino.json "$C"/20260623/calibrated_transformation_0804_rhino.json \
   "$C"/20261013/calibrated_transformation_0805_rhino.json
```

**Expect:** the three files listed, and these entries:

| robot | entry (folder) | quality | note |
|---|---|---|---|
| Cindy (0806) | `'Cindy': '20261016'` (left arm re-calibrated 2026-10-16) | validation over 15 poses: the tool tracker's offset found again within 1.6 mm RMS (at most 3.2 mm); punch test, 4 takes, 0.7 mm RMS | agrees with the old `20260916` (a copy of the June `20260622` result) within 0.8 mm / 0.02°, so `'Cindy': '20260916'` is a safe fallback |
| Alice (0804) | `'Alice': '20260623'` (her newest calibration) | June | **!** holds only if her base markers have not moved since. Its first check is where her open jaws end after the free approach (5.7) |
| Belle (0805) | `'Belle': '20261013'` (recorded 2026-10-14) | validation over 17 poses: tool tracker offset within 1.6 mm (std 0.7 mm) and 0.5°; punch test, 5 takes from different base poses, the punch tip within 1.5 mm of its mean (pairwise at most 3.0 mm) | her mocap body sits 180° turned compared with Alice's (tracker mounting), so her `base_mocap_from_base_footprint` turns by +90° where Alice's turns by −90°: expected. First check: 5.9 |

All three robots have their own entry, so `CALIBRATION_DATE` (`20261016`, the newest calibration
session) decides no robot's base; it only matters for a calibration session's recordings.

**!** Without a file the robot runs **uncalibrated**: its base is the marker body, cm off. As the
connected robot it then plans from the wrong place; as another robot it is not used as a live
obstacle (it falls back to `progress.json`).

### 1.d The problem folder  [SETUP — once]

The monitor reads and writes the design problem directly in the shared Insync folder.
`Mark action done` writes `progress.json` and `*.live-solved.json` (saved plans) there, and they
sync to the shared drive. All three robots' runs read and write the same `progress.json`: that is
how each run knows where the other robots stand.

**▶ pane 1 · MONITOR**
```bash
GD="$HOME/Insync/2025-03 Husky Assembly"
P="$GD/data_design_study/261023_2ModulesDemo"
ls "$P"                                        # expect: ActionSchedule.json BarActions MoCapMarkers.json RobotCell.json RobotCell_Alice.json RobotCell_Belle.json Trajectories WalkableGround.json
ls "$P"/BarActions/*__*.json | grep -vc live-solved     # expect: 76 action files (32 bars __J/__R; B21 B25 B35 B39 B47 B50 also __H/__HR)
ls -a "$P" | grep -c insyncdl                  # expect: 0 = Insync has finished syncing
ls "$P"/progress.json "$P"/BarActions/*.live-solved.json 2>/dev/null   # left over from an earlier run?
```

`261023_2ModulesDemo` has its **own** `MoCapMarkers.json` (the MoCap ball table): nine instrumented
bars, nothing borrowed any more (until 2026-10-23, 261006 borrowed 261019's through
`MOCAP_MARKERS_PROBLEM_NAME` in `husky_assembly_teleop/__init__.py`). At each action load, pane 1 prints
`[mocap bars] 9 instrumented bar(s): bar_B20 (8), bar_B21 (8), bar_B25 (8), bar_B34 (8), bar_B35 (8), bar_B39 (6), bar_B46 (8), bar_B47 (6), bar_B50 (6) | .../261023_2ModulesDemo/MoCapMarkers.json`.
Without the file no bar is measured and the as-built record (9) writes nothing.

**!** **B35's J35-40 ball plate moved in the design:** 710 mm along B35 now (770 mm in
261019 / 261006; its two balls also swapped sides). A B35 prepared for the 2026-10-22 run carries the
plate at the old place: move it to the new place before B35 is used, or its MoCap fit is wrong.

**!** **The whole structure sits about 1.5 m from where 261006 / 261019 put it** (moved +586 mm in x,
+1387 mm in y and 15.55 mm up, same orientation). Bars built at the old place are not this
problem's bars: start this problem on an empty site (`progress 0/76`).

**!** **The ground: WalkableGround raised by 23 mm (`DESIGN_GROUND_RAISE_MM`, default 23).**
261023's footers sit at z = 0 while `WalkableGround.json` still says −15.55 mm. The monitor first
laid Rhino's ground (−15.55 mm) on the measured floor, so the footers were planned ~15.5 mm above it
("higher than design = safer"); on site the feeler gauge read **~23 mm** under B20's footers
(2026-10-23), so the ground is now raised by 23 mm before it is laid on the floor: the whole design
sits 23 mm lower and the footers should **just touch** the real floor (by the model they sit 7.5 mm
into the fitted floor plane, which is lower there than the plane). The floor check before each
foundation fix (5.2 e) checks it on every ground bar. Back to the safe 15.5 mm gap:
`export DESIGN_GROUND_RAISE_MM=0` before starting the monitor.

**!** **B39's stored arm pose is 100 mm off (export error).** In `B39__J.json` the start states of
`B39_J_M4_tool_tighten_joint` and `B39_J_M5_LM_insert` hold a configuration whose **left** tool0 is
100 mm below the transfer's target (the right one is exact): B39 drawn in those steps hangs 100 mm
low, its lower end ~20 mm inside the floor. The transfer re-solves its goal from the targets at the
live base, so the real motion should be right, but the goal ghost, an IK seed or a reset to the
clean start uses the bad pose. Check on site before B39's transfer (5.13); the Rhino side is asked to
re-export B39.

If `progress.json` or saved plans are left over and you want to start fresh: `Reset plan skeleton
to the Rhino export` once Cindy's monitor is up (7.a). It moves them to `$P/archive/<time>/`
(nothing is deleted).

### 1.e This PC's network  [SETUP — once]

This PC is on the robot network through its built-in port `eno1` (cable to the robot router;
NetworkManager profile `Wired connection 1`, DHCP). No USB dongle is needed.

**▶ pane 1 · MONITOR**
```bash
ip -4 -brief addr show eno1                    # expect: eno1  UP  192.168.0.x/24
for h in 192.168.0.115 192.168.0.113 192.168.0.114 192.168.0.28; do   # Cindy, Alice, Belle, the Motive PC
  printf "%s: " $h; ping -c1 -W1 $h >/dev/null && echo up || echo NO REPLY
done
grep -o 'NetworkInterface name="[^"]*"' ~/.cyclonedds.xml   # expect: name="eno1"
```

**If not:**
- `eno1` DOWN or without a `192.168.0.x` address: check the cable, then
  `nmcli connection up "Wired connection 1"`.
- A robot `NO REPLY`: it is off (Part 2), or off the network.
- `eno1`'s address changed since the last run: update `CLIENT_IP` (1.b).

**?** Leave the Wi-Fi on: the default route for the internet goes through it, and the `192.168.0.x`
traffic still goes through `eno1`; Tailscale does not get in the way either. The NetworkManager
profile `cindy` (fixed `192.168.131.19` on a USB dongle) is the old direct cable to Cindy's PC; it
is not used for this test.

### 1.f Rebuild the package (only after pulling or changing code)  [SETUP — once]

Builds only `husky_assembly_teleop`, not the whole workspace:

**▶ pane 1 · MONITOR**
```bash
cd ~/ros2_ws && source venv/bin/activate
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash
```

Editing an existing `.py` file alone does not need it (the install links to the source; restart
the monitor). New files, new entry points or `setup.py` changes do.

---

## 2. Bring up the robots  [ROBOTS ON — nothing moves]

### 2.a Pendant first, launch second  [ROBOTS ON — nothing moves]

On **every** arm's teach pendant (Cindy has two):
- [ ] Power on and release the brakes
  - **Expect:** the pendant shows `Normal`.
- [ ] Set **Remote Control** mode with the Local / Remote switch in the pendant's top-right
      corner. (No switch there: ☰ → Settings → System → Remote Control → Enable, then use it.)
      **!** "External control mode" in older notes means this Remote mode, **not** the
      `external_control.urp` program.
- [ ] Do not load or start any program by hand. Once the launch runs, its safety-sync node loads
      and plays `ros_control.urp` itself.

**!** Launch only after that. If an arm is off or in Local mode when the launch starts, that arm's
controllers or tool driver may not come up; then relaunch (Part 3 shows how to tell).

**!** **Middleware: Cyclone, not Zenoh.** Each robot's base system (Clearpath's services: base
driver, joystick, `cmd_vel`) runs the middleware set in its `/etc/clearpath/robot.yaml`. On
2026-10-20 Cindy's was found switched to Zenoh on a shared domain 80; this test needs Cyclone on
the robot's own domain (86 Cindy, 84 Alice, 85 Belle). So 2.b–2.d first run `set_middleware.sh
cyclone` on each robot, before its launch:
- it rewrites `robot.yaml` and `/etc/clearpath/setup.bash`, then restarts only the base system
  (not an arm launch). Run it before the assembly, **never while that robot holds a bar**;
- on a robot already on Cyclone it changes nothing but restarts the base system once;
- it does not last: the stored config in crl-husky (`crl_husky/config/robots/<serial>/robot.yaml`)
  says Zenoh, so `install_robot_config.sh` (e.g. after an apt upgrade) switches the robot back;
- the Zenoh router service (`crl-zenoh-router`) keeps running; Cyclone ignores it.

### 2.b Cindy  [ROBOTS ON — nothing moves]

- [ ] Both arms on, pendants in Remote (2.a).
- [ ] Log in to Cindy's PC:

**▶ pane 4 · CINDY (ssh)**
```bash
ssh administrator@192.168.0.115
```

- [ ] Then, **in that ssh shell on Cindy**, put her base system back on Cyclone (2.a):

**▶ pane 4 · CINDY (ssh) — in the ssh shell on Cindy**
```bash
~/workspace/src/crl-husky/crl_husky/scripts/set_middleware.sh cyclone
source /etc/clearpath/setup.bash     # so this shell picks up the change too
```

  - **Expect:** it asks for the sudo password, then ends with `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
    `export ROS_DOMAIN_ID=86` and `Done. ...`.

- [ ] Then, still in that shell, start her arms + both scaffolding tool drivers:

**▶ pane 4 · CINDY (ssh) — in the ssh shell on Cindy**
```bash
# * Cindy's ROS must run CycloneDDS on domain 86, or this PC sees none of her topics.
# * After the step above her login shell sets both; the export only makes sure.
export ROS_DOMAIN_ID=86 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# * Send ROS traffic out of the port that holds her 192.168.0.x address (her Wi-Fi wlp2s0; both
# * cable ports belong to her robot-internal br0). Without it CycloneDDS may pick br0, and this PC
# * sees none of her topics (2026-10-27: launch up, check_robots "no topics at all").
IF=$(ip -o -4 addr show | awk '/ 192\.168\.0\./ {print $2; exit}')
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$IF\"/></Interfaces></General></Domain></CycloneDDS>"
echo "[cindy] domain $ROS_DOMAIN_ID | $RMW_IMPLEMENTATION | DDS on '$IF'"   # expect: domain 86 | rmw_cyclonedds_cpp | DDS on 'wlp2s0' (not '')
# Both arms + both scaffolding tool drivers (the default gripper:=none starts no tool driver).
ros2 launch crl_husky crl_dual_ur5e.launch.py namespace:='/a200_0806' gripper:=scaffolding_v3
```

### 2.c Alice  [ROBOTS ON — nothing moves]

- [ ] Arm on, pendant in Remote (2.a); her own remote e-stop paired and in reach (pair one husky
      at a time, `doc/husky_ros2.md` troubleshooting).
- [ ] Log in to Alice's PC (password `clearpath` if asked):

**▶ pane 2 · ALICE (ssh)**
```bash
ssh administrator@192.168.0.113
```

- [ ] Then, **in that ssh shell on Alice**, put her base system back on Cyclone (2.a):

**▶ pane 2 · ALICE (ssh) — in the ssh shell on Alice**
```bash
~/workspace/src/crl-husky/crl_husky/scripts/set_middleware.sh cyclone
source /etc/clearpath/setup.bash     # so this shell picks up the change too
```

  - **Expect:** it asks for the sudo password, then ends with `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
    `export ROS_DOMAIN_ID=84` and `Done. ...`.

- [ ] Then, still in that shell, start the arm + the Robotiq 2F-85 gripper driver:

**▶ pane 2 · ALICE (ssh) — in the ssh shell on Alice**
```bash
export ROS_DOMAIN_ID=84 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# * Send ROS traffic out of the port that holds her 192.168.0.x address (Wi-Fi or cable). Without
# * it CycloneDDS may pick her robot-internal br0, and this PC sees none of her topics (2026-10-19).
IF=$(ip -o -4 addr show | awk '/ 192\.168\.0\./ {print $2; exit}')
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$IF\"/></Interfaces></General></Domain></CycloneDDS>"
echo "[alice] domain $ROS_DOMAIN_ID | $RMW_IMPLEMENTATION | DDS on '$IF'"   # expect: domain 84 | rmw_cyclonedds_cpp | DDS on 'wlp...' or 'en...' (not '')
# The arm + the Robotiq 2F-85 gripper driver.
ros2 launch crl_husky crl_single_ur5e.launch.py namespace:='/a200_0804' gripper:=robotiq_2F_85
```

### 2.d Belle  [ROBOTS ON — nothing moves]

- [ ] Arm on, pendant in Remote (2.a); her own remote e-stop paired and in reach (pair one husky
      at a time).
- [ ] Pendant tool setting the same as Alice's. (The monitor's handoff compares its FK tool0 with
      the TCP the arm reports; a wrong pendant TCP shows as a constant offset along tool Z.)
- [ ] Log in to Belle's PC (password `clearpath` if asked):

**▶ pane 3 · BELLE (ssh)**
```bash
ssh administrator@192.168.0.114
```

- [ ] Then, **in that ssh shell on Belle**, put her base system back on Cyclone (2.a):

**▶ pane 3 · BELLE (ssh) — in the ssh shell on Belle**
```bash
~/workspace/src/crl-husky/crl_husky/scripts/set_middleware.sh cyclone
source /etc/clearpath/setup.bash     # so this shell picks up the change too
```

  - **Expect:** it asks for the sudo password, then ends with `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
    `export ROS_DOMAIN_ID=85` and `Done. ...`.

- [ ] Then, still in that shell, start the arm + the Robotiq 2F-85 gripper driver:

**▶ pane 3 · BELLE (ssh) — in the ssh shell on Belle**
```bash
# * Belle's ROS must run CycloneDDS on domain 85, or this PC sees none of her topics.
export ROS_DOMAIN_ID=85 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# * Send ROS traffic out of the port that holds her 192.168.0.x address (her Wi-Fi, wlp2s0, since
# * 2026-10-13). Without it CycloneDDS picks her robot-internal br0, and this PC sees nothing.
IF=$(ip -o -4 addr show | awk '/ 192\.168\.0\./ {print $2; exit}')
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$IF\"/></Interfaces></General></Domain></CycloneDDS>"
echo "[belle] domain $ROS_DOMAIN_ID | $RMW_IMPLEMENTATION | DDS on '$IF'"   # expect: domain 85 | rmw_cyclonedds_cpp | DDS on 'wlp2s0' (not '')
# The arm + the Robotiq 2F-85 gripper driver.
ros2 launch crl_husky crl_single_ur5e.launch.py namespace:='/a200_0805' gripper:=robotiq_2F_85
```

### 2.e Mocap  [ROBOTS ON — nothing moves]

- [ ] On the Motive PC: Motive streams Cindy's rigid body **1860**, Alice's **1840** and Belle's
      **1850**.

Leave all three launches running for the whole session (panes 2–4). **!** `Ctrl-C` in a robot's pane
stops that robot's arm and gripper / tool drivers: never while that robot holds a bar.

---

## 3. Check the robots  [CHECK — nothing moves]

One read-only script checks the robots from this PC (about 20 s per robot). It sets each robot's
domain inside itself only, so pane 5 keeps its own.

- [ ] Check all three robots. To check only some, put their names after the script (`Cindy`,
      `Alice`, `Belle`, separated by spaces; capitals do not matter), as in the `#` lines:

**▶ pane 5 · CHECKS**
```bash
~/ros2_ws/src/husky-assembly-teleop/scripts/husky/check_robots.sh                  # all three robots
# ~/ros2_ws/src/husky-assembly-teleop/scripts/husky/check_robots.sh Alice          # one robot
# ~/ros2_ws/src/husky-assembly-teleop/scripts/husky/check_robots.sh Alice Belle    # several robots
```

**?** To run it again: press the up-arrow key in pane 5 to bring the last command back, change or add
the name at its end, then `Enter`.

- [ ] **Expect:** its first line names the robots it checks; then every line `OK` and a last line
      `[check_robots] all OK`. Each `FAIL` line says what to do
      under it (`->`). Example, Cindy's left tool driver stopped (as on 2026-10-16):

```text
[check_robots] checking: Cindy Alice Belle (to check only some, put their names after the script, e.g. check_robots.sh Alice)
[check_robots] read-only; this terminal's own ROS_DOMAIN_ID stays 'unset'

=== Cindy (domain 86, /a200_0806; ssh in pane 4 · CINDY) ===
  OK    robot PC 192.168.0.115 answers ping
  OK    topics from /a200_0806 arrive
  OK    base: listens on /a200_0806/cmd_vel
  OK    left arm joint states: 49.9 Hz
  OK    left arm pendant: ros_control.urp PLAYING, robot RUNNING (so it is in Remote mode)
  OK    left arm controllers: joint active, compliance inactive (the monitor turns it on only when needed)
  ...
  FAIL  left tool status: no messages on /a200_0806/left_gripper/tool_status
        -> that tool's driver has stopped: restart Cindy's launch in pane 4 · CINDY (nothing clamped in the tools)
```

What it checks and what a `FAIL` means:

| line | OK means | FAIL: cause → do |
|---|---|---|
| robot PC answers ping | the robot PC is on the network | robot off, or this PC off the robot network → power it on; 1.e. The robot's other checks are skipped |
| topics arrive | this PC receives the robot's ROS traffic | `no topics at all ..., although its PC answers`: its launch is not running, **or** it runs but its ROS traffic leaves through the wrong network port (Alice and Belle on 2026-10-19, Cindy on 2026-10-27: the launch was up, this PC saw only `/parameter_events` and `/rosout`, or only the topics of its own monitor) → `Ctrl-C` the launch and start it again with the `IF=...` / `CYCLONEDDS_URI` lines of 2.b / 2.c / 2.d. The robot's other checks are skipped |
| base | the base driver listens on `<ns>/cmd_vel` (twist_mux) | Cindy: base driver down or network → 1.e; or her base system is on Zenoh → the middleware step of 2.b. Alice / Belle: only a `NOTE` (on 2026-10-16 their bases were not visible from this PC, only their arms); the monitor does not need it (it takes their base from mocap), only command-line driving does (6.a): use the joystick |
| joint states | ≈ **50 Hz** from the arm driver (`<ns>/<arm>/rate_limiter/joint_states`) | just after the launch: run the check again after ~10 s. Still nothing: read the pendant and controllers lines; else restart the launch |
| pendant | `ros_control.urp` PLAYING, robot RUNNING (so Remote mode) | `dashboard does not answer`: the arm is off or the launch is not running → start it (Part 2). `robot mode ...`: power on, release the brakes. **`in Local mode`**: switch the pendant to Remote (2.a); the launch then plays `ros_control.urp` itself and `scaled_joint_trajectory_controller` should turn active by itself; run the check again (Belle on 2026-10-16). `runs 'external_control.urp' ...`: the launch starts `ros_control.urp` within ~10 s: check again; still FAIL → restart the launch. Never start a program by hand |
| controllers | `scaled_joint_trajectory_controller` **active**, `cartesian_compliance_controller` **inactive** (inactive is right: the monitor turns it on only for the insert, the retreat and the grip handoff) | `none loaded`: the launch started before the arm was on and in Remote (Alice on 2026-10-16: arm on and in Remote, no controllers) → `Ctrl-C` the launch, start it again. `joint controller 'inactive'`: usually the pendant line (Local mode) → fix that first, else `Switch to Joint (BOTH)`. `compliance still active` → `Switch to Joint (BOTH)` (6.b). `compliance not loaded` → restart the launch |
| tool status (Cindy) | ≈ **20 Hz** from each scaffolding tool | `no messages` while that arm is fine: that tool's driver has stopped (2026-10-16: Cindy's left one; `ros2 node list \| grep tool` showed only `right_gripper/scaffolding_tool_driver`) → restart Cindy's launch with empty tools; or her launch lacks `gripper:=scaffolding_v3` (2.b). `message type unknown`: this PC's workspace is not built / sourced (1.f) |
| gripper (Alice, Belle) | the action `<ns>/gripper/robotiq_gripper_controller/gripper_cmd` is up | the launch lacks `gripper:=robotiq_2F_85` (2.c / 2.d), or the gripper driver failed: read its pane |

**?** The same checks by hand, one command at a time: Part 6.h.

---

## 4. Test  [no bar, nothing recorded]

Two tests **once, before the assembly starts** (before Part 5 and its first action, action 0:
Cindy jointing B20; table in 0.1). Both run with **no bar** and a clear workspace, so a problem
shows up before a bar depends on it. Nothing is saved: `progress.json` and the as-built record
stay as they are.

| | test | robot | what moves |
|---|---|---|---|
| 4.1 | gripper movement of the support robots (the grip handoff) | Belle, then Alice | gripper; controller switch (the arm should stay still) |
| 4.2 | spiral movement of the assembly robot (the insert's spiral search) | Cindy | both arms: to a bar-holding pose, 15 mm forward with the spiral, back |

**!** **Do not `Mark action done` here**: these are tests.

### 4.1 Support robots: gripper movement  [TEST — arm / gripper move, no bar, nothing recorded]

Alice and Belle take over a bar from Cindy with a **grip handoff**: the arm goes soft (compliance
controller) and the gripper closes, so the arm follows the bar instead of pulling it. **Belle's**
handoff has never run on hardware. **Alice's** ran on 2026-10-12; its tuning is still open
(`tasks/2026-10-12_alice_grip_handoff_tuning.md`).

Do steps a)–g) for **Belle first**, then again for Alice. Where they differ:

| | Belle | Alice |
|---|---|---|
| domain, namespace | 85, `/a200_0805` | 84, `/a200_0804` |
| her hold action, its close step | 10, `B35_H_M3_gripper_close` | 3, `B21_H_M3_gripper_close` |

**a) Start the robot's monitor**
- [ ] pane 1: paste her block (it sets her domain, then starts the monitor)

Belle:

**▶ pane 1 · MONITOR**
```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=85                   # 85 = Belle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
export DESIGN_PROBLEM_NAME=261023_2ModulesDemo
ros2 daemon stop      # ! every time, or the monitor may miss the robot's topics
ros2 run husky_assembly_teleop husky_monitor
```

Alice (after Belle is done):

**▶ pane 1 · MONITOR**
```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=84                   # 84 = Alice
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
export DESIGN_PROBLEM_NAME=261023_2ModulesDemo
ros2 daemon stop      # ! every time, or the monitor may miss the robot's topics
ros2 run husky_assembly_teleop husky_monitor
```

- [ ] pane 1: read the start-up lines (scroll back with the mouse wheel)
  - **Expect:** `Base calibration of Belle (0805): 20261013/calibrated_transformation_0805_rhino.json`
    (one such line per robot);
  - **Expect:** `[Plan skeleton] robot Belle (domain 85) | problem 261023_2ModulesDemo | progress 0/76 done`
    (Alice: `robot Alice (domain 84) ...`);
  - **Expect:** `[Plan skeleton] others: Cindy <- parked | Alice <- parked` (Alice: `Cindy <- parked | Belle <- parked`):
    nothing is done yet;
  - **Expect:** `mocap client connected: True`;
  - **Expect:** right under it, the two modes that are easy to miss: `insert spiral search: n/a (box greyed: no step loaded; ...)`
    and `servo feedback: tool0 (box greyed: no bar loaded; ...)` in yellow. Both boxes follow the
    loaded step (6.f, 6.e): green `ON` for an insert mating on both arms, green `MoCap bar` for a
    bar with markers; a click on either box prints its line again;
  - **Expect:** two windows open: the panel and the 3D view.
  - **If not:** the wrong robot name → `Ctrl-C` and paste the right block. `mocap client connected: False`:
    the monitor listens for mocap on the wrong address: fix `CLIENT_IP` as in 1.b, then restart.

**b) Find the buttons** in the panel (scroll down; map in 0.8)
- [ ] GRIPPER section: button [Open Gripper Full], button [Close Gripper for Bar]
- [ ] CONTROLLERS section: button [Switch to Compliance (BOTH)], button [Switch to Joint (BOTH)]
- [ ] MOVEMENT EXE section: button [Zero Force Sensor (BOTH)], button [Confirm Exec], button [Cancel Exec]
- [ ] PLAN SKELETON section (top): slider [Action (idx)], button [Load action]; further down
      slider [Step], button [Load Movement]

**c) Gripper by hand**
- [ ] pane 5: start watching the fingers (it prints the finger position over and over)

**▶ pane 5 · CHECKS**
```bash
ROS_DOMAIN_ID=85 ros2 topic echo /a200_0805/gripper/joint_states --field position       # Belle
# ROS_DOMAIN_ID=84 ros2 topic echo /a200_0804/gripper/joint_states --field position     # Alice: this line without the '#'
```

- [ ] button [Open Gripper Full]
  - **Expect:** the jaws open; the position goes to ≈ **0.43**.
- [ ] button [Close Gripper for Bar]
  - **Expect:** the jaws close; the position goes to ~**0.79** (an empty close).
  - The gripper action sends no feedback, only its final result (checked on Alice 2026-10-12);
    the handoff below does not need any.
  - The Robotiq driver may still close at 15 % speed: a close of ~4 s is expected until the robot
    gets Cindy's 2026-09-09 driver fix (`scripts/husky/patch_robotiq_driver.py`; its source is not
    on this PC, only a compiled leftover in `scripts/husky/__pycache__/`).
- [ ] pane 5: `Ctrl-C` to stop watching

**d) Controller switch by hand** (hand on the robot's e-stop, gripper empty, arm in a safe pose)
- [ ] button [Switch to Compliance (BOTH)]
  - **Expect:** the arm stays where it is.
- [ ] pane 5: ask which controller runs

**▶ pane 5 · CHECKS**
```bash
ROS_DOMAIN_ID=85 ros2 service call /a200_0805/ur5e/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers | tr ',' '\n' | grep -oE "name='[a-z_]+'|state='[a-z]+'" | paste - - | grep -E 'scaled_joint|cartesian_compliance'      # Belle
# ROS_DOMAIN_ID=84 ros2 service call /a200_0804/ur5e/controller_manager/list_controllers controller_manager_msgs/srv/ListControllers | tr ',' '\n' | grep -oE "name='[a-z_]+'|state='[a-z]+'" | paste - - | grep -E 'scaled_joint|cartesian_compliance'    # Alice: this line without the '#'
```

  - **Expect:** `name='cartesian_compliance_controller'  state='active'` and
    `name='scaled_joint_trajectory_controller'  state='inactive'`.
- [ ] button [Switch to Joint (BOTH)] → pane 5: the same command again (up-arrow key, `Enter`)
  - **Expect:** now `scaled_joint_trajectory_controller` `active`, `cartesian_compliance_controller` `inactive`.

**e) The handoff in the air** (the jaws close on nothing)
- [ ] button [Open Gripper Full]
- [ ] slider [Action (idx)]: **10** Belle / **3** Alice → button [Load action]
  - **Expect:** readout `step 1/4: ...`.
- [ ] slider [Step]: 3 → button [Load Movement]
  - **Expect:** readout `step 4/4: B35_H_M3_gripper_close ...` (Alice: `B21_H_M3_gripper_close`), and a
    button `Exec gripper step: CLOSE + compliant handoff` appears next to
    `Exec Selected Mv Traj (auto)` (MOVEMENT EXE).
- [ ] button [Exec gripper step: CLOSE + compliant handoff]
  - **Expect:** `[gripper step <close step>] CLOSE the gripper on the bar (position 0.8), the arm going compliant BEFORE the close [SupportGripper]. Click 'Confirm Exec' ...`
- [ ] hand on the robot's e-stop → button [Confirm Exec]
  - **Expect:** in order:
    - `[grip arm 0] phase 1/3: ensuring scaled_joint_trajectory_controller on arm 0.`
    - `F/T sensor zeroed (fingers open, touching nothing); going compliant in 0.5s.`
    - `anchor check: FK tool0 vs the arm-reported TCP 0.0 mm, 0.00 deg.` (above 10 mm / 2 deg:
      no compliance, a stiff close instead)
    - `phase 2/3: switching arm 0 scaled_joint_trajectory_controller -> cartesian_compliance_controller BEFORE the close (fingers still open).`
      The arm must stay where it is.
    - `phase 3/3: close goal sent (position 0.426 -> 0.800, effort 0.1); arm under cartesian_compliance_controller (target = tool0 pose captured before the switch, zero wrench) until the gripper stalls or reaches its goal; ...`
      The arm still stays put while the empty jaws close.
    - `gripper done: position ..., stalled ..., reached_goal ...`, then
      `restoring scaled_joint_trajectory_controller on arm 0.`: the arm is back on the joint
      controller.
  - **If not:** it logs `FK tool0 and the arm-reported TCP disagree by ...` (no handoff): the
    robot's TCP frame does not match the monitor's. Note the numbers. A constant offset along tool
    Z = the tool setting on her teach pendant differs from Alice's.
  - **If not:** it logs `switch to cartesian_compliance_controller FAILED: the gripper was NOT closed`:
    the controller switch was refused. Check the controllers with the d) command, then do e) again.
  - **!** **Belle, first time:** the monitor turns its targets by the same 90° as Alice's (her
    calibrated URDF has the same arm-base turn). This has not been confirmed on her hardware
    yet. If her arm moves at `phase 2/3` or `3/3`: e-stop, then note how far and which way. The
    drift limit stops the hold at 10 mm.

**f) Cancel test**
- [ ] e) once more → button [Cancel Exec] during phase 3
  - **Expect:** the hold ends **and the controller switches back to joint** (the gripper keeps closing with
    the arm stiff — logged).

**g) Finish**
- [ ] button [Open Gripper Full]
- [ ] pane 1: `Ctrl-C` (quit the monitor) **without** marking anything
  - Or button [Switch to Alice] (top of the PLAN SKELETON section) instead of `Ctrl-C` + her
    block: the monitor restarts for Alice by itself.
- [ ] After Belle: a)–g) again for Alice, starting with Alice's block in a).
- [ ] After Alice: 4.2 with Cindy.

### 4.2 Assembly robot: spiral movement  [TEST — both arms move, no bar, nothing recorded]

The insert's **spiral search** (6.f) has never run on Cindy: on 2026-10-20 and 10-21 every insert
used the straight line. This test runs it once with empty tools, the way a screwed bar's insert
runs it:
1. a free move to a **bar-holding pose**: both tools as if holding B21 at its insertion start,
   about 0.9 m in front of Cindy, 641 mm apart, both tool axes pointing forward;
2. the **insert**: both tools push **15 mm forward** along their axis (away from Cindy). The first
   10 mm are stiff (joint controller). The last 5 mm are compliant with the spiral: one sideways
   circle on both tools, growing from 0 to 15 mm (default 4 turns over 20 s);
3. with no bar nothing stalls, so the spiral **holds** its end until you click. The tools then
   go back to the straight line's end and **out** along the same line to the bar-holding pose.

Nothing is tightened (no scaffolding motor runs) and nothing is recorded. Both force sensors are
zeroed at the bar-holding pose (tools empty, arms still), as at the mount pause, so the compliant
part does not drift on an old zero.

**Needs:** Cindy's launch (2.b) and mocap (her base); **about 1 m clear in front of her** at tool
height; a hand on Cindy's e-stop the whole time.

**a) Start Cindy's monitor**
- [ ] pane 1: paste Cindy's block (the same as in 5.1; it only sets the terminal up, nothing starts)

**▶ pane 1 · MONITOR**
```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=86
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml   # already in ~/.bashrc; pins DDS to eno1
export DESIGN_PROBLEM_NAME=261023_2ModulesDemo
ros2 daemon stop >/dev/null 2>&1
echo "[cindy env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}') | problem $DESIGN_PROBLEM_NAME"
```

  - **Expect:** one line, `[cindy env] domain 86 via eno1 UP 192.168.0.201/24 | problem 261023_2ModulesDemo`,
    and a `(venv)` prefix on the prompt.
  - **If not** (another domain, or no `eno1` address): paste the block again; ! never paste another
    robot's block or a check block into this pane (5.1, "Which robot will the monitor drive?").
- [ ] pane 1: start the monitor

**▶ pane 1 · MONITOR**
```bash
ros2 daemon stop      # ! every time, or the monitor may miss Cindy's topics
ros2 run husky_assembly_teleop husky_monitor
```

  - **Expect:** `[Plan skeleton] robot Cindy (domain 86) | problem 261023_2ModulesDemo | progress k/76 done`,
    `mocap client connected: True`, and under it `insert spiral search: n/a (box greyed: no step loaded; ...)` and
    `servo feedback: tool0 (box greyed: no bar loaded; ...)` in yellow (all start lines: 6.c; the
    boxes follow the loaded step, 6.f / 6.e).
  - **If not** `robot Cindy`: `Ctrl-C`, paste Cindy's block again, start again.
- [ ] pane 5: check that both arms are up and on the joint controller (Part 3)

**▶ pane 5 · CHECKS**
```bash
~/ros2_ws/src/husky-assembly-teleop/scripts/husky/check_robots.sh Cindy
```

  - **Expect:** under `=== Cindy (domain 86, ...) ===` only `OK` lines (both arms' joints arriving,
    `scaled_joint_trajectory_controller` active on both), ending with `[check_robots] all OK`.
  - **If not** (`FAIL ...` lines, each with its `-> ...` fix, then `[check_robots] N FAIL:`): fix it
    as that line and Part 3 say before pressing anything in the panel.
- [ ] panel: read the `Now:` line
  - **Expect:** action 0 (`B20_J_joint`) loads by itself (`[Plan skeleton] auto-load ...`). The
    test plans its moves in the loaded step's scene (the room, the other robots, the built bars),
    at Cindy's real base.
  - **If not:** slider [Action (idx)]: 0 → button [Load action].

**b) Find the knobs** (MOVEMENT EXE section, under the `Insert:` sliders; map in 0.8)
- [ ] box [Insert compliant part: SPIRAL search ...] and box [Servo feedback: MoCap bar markers ...]
  - **Expect:** both boxes **orange**, like [Exec Selected Mv Traj (auto)]; both **greyed** until a
    step is loaded (they follow the loaded step: 6.f, 6.e).
  - The test always runs the spiral, ticked, greyed or not, and changes neither box.
- [ ] slider [Insert: rigid->compliant split (mm to goal)]: 5; slider [Insert spiral: duration (s)]:
      20; slider [Insert spiral: turns ...]: 4
  - The test reads them, like the real insert.
- [ ] button [Spiral test (no bar)] (orange), right under the two boxes.

**c) To the bar-holding pose** (tools empty)
- [ ] button [Spiral test (no bar)]
  - **Expect:** `[spiral test] planned N waypoints at 0.1 rad; verifying swept path...`,
    `[movement preview] 'spiral test 1/2': N waypoints, joint range [...] deg.`, then
    `[spiral test] 1/2: the arms move to the test pose, as if holding a bar at its insertion start (both tools about 0.9 m in front of Cindy; N waypoints, T s). Preview it with 'Traj viz time'; set its speed with 'traj time' (higher = slower). No bar in the tools; keep 1 m in front of her clear. 'Confirm Exec' to move, 'Cancel Exec' to stop.`
  - **Expect:** slider [traj time] jumps to T (8–60 s, about 10 deg/s per joint; the DPG panel
    only, a PyBullet slider stays where it was) and the joint plot shows the planned move.
  - **If not:** `[spiral test] no free path to the test pose from here; ...`: something is in the
    way in the 3D view (the room, another robot). Drive Cindy to open space (6.a), press again.
  - **If not:** `Load any action first ...` → a) again. `Spiral test refused: ...` → another step
    still runs or waits for `Confirm Exec`: finish it or `Cancel Exec`.
- [ ] slider [Traj viz time]: scrub it from 0 to 1
  - **Expect:** the ghost robot plays the move to the bar-holding pose: nothing in its way, both
    tools ending about 0.9 m in front of Cindy. **If not:** [Cancel Exec].
- [ ] slider [traj time]: higher = slower (first run: keep T or slower) → hand on Cindy's e-stop →
      button [Confirm Exec]
  - **Expect:** `[spiral test] 1/2: moving to the test pose over T s.` with the slider's value read
    at the click, then `[spiral test] rigid chunk: N waypoints over T s`: both arms move and stop
    with both tools in front of Cindy, pointing forward.
  - This move is stiff and cannot be cancelled from the panel: e-stop if needed.

**d) The insert with the spiral**
  - **Expect:** 2 s later, both force sensors zeroed (the arms stay still):
    `[spiral test] force sensors zeroed at the test pose (tools empty) (|F| a/b -> c/d N).`
    (left / right before and after; after ≈ 0).
  - **If not:** `[spiral test] could not zero the force sensors at the test pose (tools empty); ...`
    or `[spiral test] the force sensor zero was not confirmed ...`: the test goes on, and the next
    prompt says `2/2 (! force sensors NOT zeroed: the arms may drift when compliance switches on)`.
    Click [Cancel Exec] there to stop before anything moves, or go on and watch for drift.
  - **Expect:** then `[movement preview] 'spiral test 2/2': N waypoints, ...` and
    `[spiral test] 2/2 (force sensors just zeroed): both tools push 15 mm forward along their axis (x, y, z) like a screwed bar's insert: the first part stiff, the last 5 mm compliant with the SPIRAL search (4 turns over 20s, up to 15 mm sideways), then back out. The straight 15 mm line takes 5s ('traj time'; preview with 'Traj viz time'). 'Confirm Exec' to run, 'Cancel Exec' to stop.`
    Slider [traj time] is back at 5 (the insert's default).
  - **If not:** `[spiral test] no straight 15 mm line from the test pose; stopping here.` after a
    `[spiral test diag]` block (7.c): note it; the arms stay at the test pose.
- [ ] slider [Traj viz time]: scrub it → the ghost's tools move 15 mm forward, straight
- [ ] slider [traj time]: higher = slower for the straight line (5 = the real insert's default; the
      spiral's own duration is its slider in b)) → button [Confirm Exec]
  - **Expect:** `[spiral test] 10.0 mm RIGID (3.3s), then 5.0 mm COMPLIANT with the SPIRAL search: radius 0 -> 15 mm over 20.0s and 4 turns about the insertion axis.`
    (3.3 s = 10/15 of a traj time of 5 s; it scales with the slider.)
  - **Expect:** the tools move 10 mm forward, stiff. The controllers switch to compliance (the
    arms stay put). Both tools then circle **together**, the circle growing to 15 mm while they
    move the last 5 mm forward.
  - **Expect:** at the spiral's end, the warning `[spiral test] spiral complete (spiral 100% (20.0/20.0 s, radius 15.0 mm, 4.0 of 4 turns)): HOLDING its end under compliance. 'Confirm Exec' = back to the straight line's end, then out to the start; 'Cancel Exec' = stop here.`
    The arms hold still, soft.
- [ ] button [Confirm Exec]
  - **Expect:** the tools glide back onto the line's end (2 s); the joint controller comes back;
    the tools go 15 mm back out along the line, stiff:
    `[spiral test] done: back at the start of the line (nothing tightened, nothing recorded).`
- **Cancel:** button [Cancel Exec] at a prompt: nothing more moves. During the spiral (or its
  hold): it stops there, `'Cancel Exec' at spiral ...; stopping here.`, and the joint controller
  takes over and holds the arms where they are. Either way the arms stay in front of Cindy (press
  the test button again: it plans from there).

**e) What to note**
- Both tools trace the same circle (as one bar)? Either arm sagging or drifting while compliant?
- A controller switch failure: `the arms are NOT under the compliance controller ...` (nothing
  more is sent), or `ARMS LEFT UNDER cartesian_compliance_controller ...` → button
  [Switch to Joint (BOTH)] (6.b).

**f) Finish**
- [ ] The arms stay at the bar-holding pose. Keep this monitor for run 1 (5.6): its first step,
      the free move to load, plans from where the arms are. Or quit it:

**▶ pane 1 · MONITOR**
```text
Ctrl-C
```

  - The arms keep holding the bar-holding pose (joint controller); do not touch Cindy's launch pane.

---

## 5. The assembly  [REAL RUN — recorded]

### 5.1 Every monitor run: which robot it drives, start, switch, quit  [REAL RUN — recorded]

Part 4's tests start their monitors the same way.

#### Which robot will the monitor drive?

**The rule.** The monitor reads `ROS_DOMAIN_ID` from **the terminal it is started in**, once at
start-up: **86 Cindy, 84 Alice, 85 Belle**.
- Missing: Cindy, without a warning.
- A number that names no robot: Cindy, with `ROS_DOMAIN_ID='...' names no robot in the robot registry; defaulting to Cindy (0806).`

In that terminal, the **last** `export ROS_DOMAIN_ID=...` pasted decides, whichever block it came
from. On 2026-10-16 a Belle check block pasted into the monitor's pane made the next monitor run
Belle. That is why checks live in pane 5 and never `export` the domain.

**The start blocks.** One per robot, for pane 1. Paste the block of the robot the monitor should
drive, every time before you start the monitor. A block only tells the terminal which robot to
talk to: it prints **one line** and starts nothing.

| block | paste it whenever the monitor should drive | used in |
|---|---|---|
| Cindy's | Cindy | 4.2 (spiral test), 5.6, 5.8, 5.10 |
| Alice's | Alice | 4.1 (gripper test), 5.7, 5.11 |
| Belle's | Belle | 4.1 (gripper test), 5.9, 5.12 |

**Cindy's block:**

**▶ pane 1 · MONITOR**
```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=86
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml   # already in ~/.bashrc; pins DDS to eno1
export DESIGN_PROBLEM_NAME=261023_2ModulesDemo
ros2 daemon stop >/dev/null 2>&1
echo "[cindy env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}') | problem $DESIGN_PROBLEM_NAME"
```

**Alice's block:**

**▶ pane 1 · MONITOR**
```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=84
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
export DESIGN_PROBLEM_NAME=261023_2ModulesDemo
ros2 daemon stop >/dev/null 2>&1
echo "[alice env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}') | problem $DESIGN_PROBLEM_NAME"
```

**Belle's block:**

**▶ pane 1 · MONITOR**
```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=85
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
export DESIGN_PROBLEM_NAME=261023_2ModulesDemo
ros2 daemon stop >/dev/null 2>&1
echo "[belle env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}') | problem $DESIGN_PROBLEM_NAME"
```

**Starting the monitor, in this order:**
- [ ] a) pane 1: paste the robot's block
  - **Expect:** one line, e.g. `[cindy env] domain 86 via eno1 UP 192.168.0.201/24 | problem 261023_2ModulesDemo`
    (and a `(venv)` prefix on the prompt). The domain in that line is the robot's;
    `echo $ROS_DOMAIN_ID` shows it again any time.
- [ ] b) pane 1: start the monitor

**▶ pane 1 · MONITOR**
```bash
ros2 daemon stop      # ! every time, or the monitor may miss the robot's topics
ros2 run husky_assembly_teleop husky_monitor
```

  - **Expect:** its start line names the robot:
    `[Plan skeleton] robot Cindy (domain 86) | problem 261023_2ModulesDemo | progress k/76 done`
    (all start lines: Part 6.c).

**!** Run `ros2 daemon stop` right before **every** `ros2 run ... husky_monitor`, even when the block
was pasted already (the blocks stop it too). Without it the monitor pane often does not pick up
the robot's topics.

**Changing to another robot — two ways:**

| way | do | note |
|---|---|---|
| **1 (standard)** | pane 1: `Ctrl-C` the monitor, then a)–b) with the other robot's block | always works |
| **2 (button, new 2026-10-19)** | button [Switch to Alice] / [Switch to Belle] / [Switch to Cindy] at the top of the Plan skeleton section | the monitor restarts itself for that robot: its windows close and reopen; no pasting. The current robot's button is **red** and reads `(current)`; clicking it only logs `[Switch] already on ...`. If a button misbehaves, use way 1 |

- Way 2 prints `[Switch] restarting the monitor on Alice (domain 84): the windows close and reopen. Cindy's driver keeps running.`,
  then `[Switch] starting the monitor for Alice (domain 84) ...`.
- Way 2 is refused while a step runs or waits for `Confirm Exec`, or while an arm moves
  (`'Switch to Alice' refused: ...`): finish it or `Cancel Exec` first.
- **!** After a way-2 switch, pane 1's shell still holds the **old** domain (a program cannot change
  the shell that started it). After quitting that monitor with `Ctrl-C`, always paste the robot's
  block (a) before typing `ros2 run` again, or the old robot starts.

#### In every run, after the start

- [ ] pane 1: read the start lines (6.c)
  - **Expect:** the right robot and `progress k/76 done`; each run below names the lines to expect.
- [ ] panel: read the `Now:` line
  - **Expect:** it names the action the run expects. The action loads by itself once the robot's joints and
    mocap base have arrived (`[Plan skeleton] auto-load ...`), and `Mark action done` loads the
    next one.
  - **If not:** slider [Action (idx)]: k → button [Load action].
- [ ] At the end of a run: pane 1: `Ctrl-C` (or way 2, button [Switch to ...])
  - The robot keeps holding; do not touch its launch pane.

**?** Cindy's runs: the transfer workaround box starts unticked after every monitor start; 5.2 a)
ticks it in every jointing action.

**?** After every step that runs, the robot's real state is saved to `progress.json`, and the next
run plans around it (Part 6.k). `Mark action done` still marks the action done (the `k/76`
counts below assume you click it).

### 5.2 Procedure J — jointing a bar (Cindy)  [REAL RUN — recorded]

> **! Transfer workaround in use** (until the bar-held transfer planner is ready). The transfer
> runs as a plain free move **without the bar**:
> - the workaround box must be ticked (a) below). With it ticked,
>   `Transfer start: Confirm manual pose (IK check)` checks whether the arms reach the insertion
>   start (the Rhino pose) from where the base stands;
> - the `Transfer start:` sliders (the bar-loading pose) are not used.
>
> Order: **transfer** (empty tools, from the live arms to the insertion start) → **manual mount**
> (mount the bar by hand there) → **tool grasp** → **insert** (→ **fix foundation** for the
> ground bars B20 and B34). The travel to load is skipped. The mount pause does the manual mount
> and the tool grasp: their own steps are not clicked (Part 6.g).

An action has n = 6 steps for a ground bar, 5 for the others. `<bar>` = the bar, `k` = the
action's number.

**a) Transfer** (`<bar>_J_M3_CDFM_transfer_to_approach`)
- [ ] slider [Action (idx)]: k (or click its row in [Actions]) → button [Load action]
  - **Expect:** readout `step 4/n: <bar>_J_M3_CDFM_transfer_to_approach ...`. Usually the action has loaded
    by itself (5.1): then skip this line.
- [ ] slider [Step]: 3 → button [Load Movement]
  - **Expect:** the same readout, `traj time` 10 s. The action opens on this step; the line is how you come
    back to the transfer later.
- [ ] joystick: drive the base to the blue goal ghost (or from the command line, 6.a)
- [ ] box [Workaround: plan the transfer as a free move (no bar, from the live arms)] (under
      `Plan Movement`): ticked
  - **Expect:** the five `Transfer start:` sliders above it disappear (they come back when unticked). It
    starts unticked after every monitor start.
- [ ] button [Transfer start: Confirm manual pose (IK check)] → button [Plan Movement]
  - **Expect:** IK check: `[transfer workaround] insertion start reachable from the live base (blue ghost); max |goal - stored goal| = ... rad`.
    The blue ghost shows the arms at the insertion start. Nothing moves and nothing is staged for
    `Exec` yet.
  - **Expect:** Plan, three lines:
    - `[transfer workaround] <bar>_J_M3_CDFM_transfer_to_approach: goal from the last IK check (the base moved ... mm since).`
    - `[transfer workaround] free move (no bar) from the live arms to the insertion start; max |goal - stored goal| = ... rad`
      (same value as the IK check)
    - `[Plan] '<bar>_J_M3_CDFM_transfer_to_approach': N waypoints stored.`
  - If the base moved more than 2 mm since the check, the goal is solved again
    (`... solving the goal again.`).
  - **If not:** `... no collision-free IK for the insertion start at the live base ...`: the base
    stands too far from (or too close to) the bar's Rhino pose, or the drawn colliding pair is in
    the way → drive the base again, then the IK check again.
- [ ] slider [Traj viz time]: drag it through the preview
  - **Expect:** starts at the live arms, ends with the **empty** tools at the insertion start, just short of
    the bar's assembled pose, clear of the floor, the built bars and any support robot.
- [ ] slider [traj time]: higher = slower (first run) → button [Exec Selected Mv Traj (auto)]
  - **Expect:** the arms go there. The exec then stays running (the next buttons are refused until it
    ends): it corrects the tool0 placement with the visual servo of the bar-holding accuracy test
    (c) below). The base's centre of mass shifts under the arms, so the tools land a few mm off
    the Rhino pose.

**b) Mount pause**
- [ ] pane 1: read the log
  - **Expect:** first the force sensors are zeroed (tools still empty, so from here on the wrist
    forces are the bar's own load):
    `[transfer <bar>_J_M3_CDFM_transfer_to_approach] force sensors zeroed at the mount pause (tools empty) (|F| a/b -> c/d N).`
  - **Expect:** `[transfer <bar>_J_M3_CDFM_transfer_to_approach] the arms stand at the insertion start with EMPTY tools (force sensors just zeroed): mount the bar and clamp it with the tool buttons, then click 'Confirm Exec'. ...`
  - **If not:** `... could not zero the force sensors at the mount pause (tools empty); going on with the reading as is ...`
    or `... the force sensor zero was not confirmed ...`: the step goes on; the wrist forces in the
    as-built records are then relative to an older zero (the card's `load` block says which).
- [ ] by hand: mount the bar in both tools → buttons **on the tools**: clamp it
- [ ] button [Confirm Exec] (or button [Cancel Exec])
  - `Cancel Exec` ends it without a correction (`mount pause cancelled; no servo correction.`).

**c) Servo** (runs by itself; details: 6.e)
- [ ] pane 1: read the log
  - **Expect:** the insert is loaded by itself (`[Movement] loaded [..] '<insert>' ...`) and the servo runs:
    `### SERVOING iteration 1/8`, `[IK Live Base] ...`, `[transfer plan] OK (N waypoints, bar-held, LINEAR).`
  - **Expect:** the pause `[servo] First move planned: N waypoints, travels X deg ...`.
- [ ] slider [Traj viz time]: scrub it (a few mm with the bar held) → button [Confirm Exec]
  - Later iterations run alone unless a path travels more than 5 deg
    (`[servo] SAFEGUARD iter N: ...` → the same: preview, `Confirm Exec`).
- [ ] pane 1: read the end
  - **Expect:** `### SERVOING converged at iter N (< 0.2 mm)` or
    `### SERVOING stopped at iter N: nothing left to plan ...` (or after 8 iterations).

**d) Insert** (ground bar `<bar>_J_M4_LM_insert`; others `<bar>_J_M5_LM_insert`, with the tighten
`<bar>_J_M4_tool_tighten_joint` running inside it)
- [ ] slider [Step]: 4 (already loaded after the servo) → button [Load Movement]
  - **Expect:** readout `step 5/n: ...`.
- [ ] button [Plan Movement]
  - **Expect:** it starts where the servo left the arms.
- [ ] slider [Traj viz time]: preview
  - **Expect:** for a ground bar no ground-joint ↔ floor collision is reported.
- [ ] optional, a bar that tightens: box [Insert compliant part: SPIRAL search (...)]: ticked (6.f)
  - **Expect** at the insert: `compliant part: STRAIGHT line (box 'Insert compliant part: SPIRAL search' unticked).`,
    or the `SPIRAL search for the compliant part: ...` line when ticked.
- [ ] slider [traj time]: higher = slower (first run; the rigid insert has no force limit) →
      button [Exec Selected Mv Traj (auto)]
  - **Expect:** depends on the bar (table below).
- [ ] ground bar only: nothing to click; the same exec goes on into a **second servo**, which puts
      the bar on its assembled pose (the fix step's start) and loads the fix step by itself
  - **Expect:** `[insert <bar>_J_M4_LM_insert] servo to the start of <bar>_J_M5_manual_fix_foundation (the insert's end), feedback: tool0; straight-line corrections only.`
    (`feedback: MoCap bar` when box [Servo feedback: MoCap bar markers (unticked = tool0)] is ticked), then
    the lines of c): `[apply live base] '<bar>_J_M5_manual_fix_foundation': ...`,
    `[transfer plan] OK (2 waypoints, bar-held, LINEAR).` (each correction is planned like a short
    transfer; it is **not** a second transfer) and the pause
    `[servo] First move planned: 2 waypoints, travels 0.1 deg ...`.
- [ ] slider [Traj viz time]: scrub it (sub-mm, B20 on the floor) → button [Confirm Exec]
  - **Expect:** the end as in c). Seen 2026-10-20 on B20 (tool0): `### SERVOING ran all 8 iterations without converging.`:
    the right flange stays 1–3 mm from its Rhino target (the bar as mounted by hand) and the bar
    moved under 0.5 mm; harmless.
  - **Expect:** it stops by itself if the force at a flange changes by more than 20 N:
    `### SERVOING stopped at iter N: the force on an arm changed by ... N since iter 0 (limit 20 N); the bar may be pushed against something.`
  - `Cancel Exec` stops it at any time. The insert has already finished, so where the arms stand is
    carried forward: `[insert <bar>_J_M4_LM_insert] 'Cancel Exec' during the servo after the insert: ...`.
  - **If not** (no `servo to the start of` line): it was skipped, and one line says why: `'Cancel
    Exec': no servo after the insert.`, `servo after the insert skipped (FAKE_HARDWARE)` / `(the base
    is not tracked by mocap)`, or `the flanges stopped X mm from the end of the insert (limit 3.0 mm): no servo after it.`
- [ ] pane 5: `check_robots.sh Cindy` (Part 3)
  - **Expect:** Cindy holds the bar at its assembled pose, **joint controller active**.

| bar | what the insert does | log |
|---|---|---|
| **ground bar** (B20, B34) | held on its ground joints: **rigid only** (joint tracking, compliance never engaged); the joint motors **never tighten**; the `Insert: ...` knobs do not apply; the controllers stay on `scaled_joint_trajectory_controller` | `[<insert>] grounded bar (holds <ground joints>): rigid insert under scaled_joint_trajectory_controller (...), no tighten; ...`, then `[<insert>] grounded bar: rigid insert done (no tighten).` Also once `'<insert>': the export asks for the cartesian_compliant controller; the monitor runs this dual_constrained_linear movement under joint_tracking.` (expected; Rhino note D9) |
| **one-sided** (B21, B35) | rigid approach, then compliance for the last mm on **both** arms; only the **left** joint motor tightens until it stalls (the right one is never driven); then the joint controller is restored | `<bar>_J_M4_tool_tighten_joint ('tighten') runs with this movement: the compliant insert sends it (LEFT joint motor TIGHTENING only; the right joint has no female to screw into, its joint motor is not driven).`, `[<insert>] one-sided bar: the left joint mates, so only the L joint motor TIGHTENs ...`, `[scaffolding] <insert>: TIGHTEN joint motor on L arm (arm 0)` (no R line) |
| **both joints mate** (B22, B36, B37) | rigid approach, then compliance for the last mm; **both** joint motors tighten; the insert ends when both have stalled; then the joint controller is restored | `<bar>_J_M4_tool_tighten_joint ('tighten') runs with this movement: the compliant insert sends it (joint motors TIGHTENING).` and a `[scaffolding] <insert>: TIGHTEN joint motor on ...` line for **both** arms |

**e) Fix foundation** (ground bars only, `<bar>_J_M5_manual_fix_foundation`)
- [ ] slider [Step]: 5 (already loaded by the second servo in d)) → button [Load Movement]
  - **Expect:** readout `step 6/6: <bar>_J_M5_manual_fix_foundation ...`.
- [ ] button [Operator done (manual step) -> then Confirm Exec]
- [ ] **Floor check, before fixing anything** (the bar still held by Cindy): feeler gauge at the four
      corners of each ground joint's footer (`<bar>`'s two ground joints, e.g. B20: G20-T20-0 and
      G20-T20-1), between the footer's bottom and the floor; write the 8 gaps (mm) in the run notes
  - **Expect:** every gap **0–2 mm** (footers touching or just above the floor), at B20, B34 and B46:
    the design is laid on the measured floor with its ground raised by 23 mm (start line
    `design on the measured floor: ON (... WalkableGround raised by DESIGN_GROUND_RAISE_MM = 23.0 mm ...)`,
    6.c), set from B20's measured ~23 mm gap on 2026-10-23. **B34 and B46 are the first test of that
    number at other spots**: the floor is uneven by up to ~3 mm and was fitted to 2 mm rms, so their
    gaps can differ by a few mm. With `DESIGN_GROUND_RAISE_MM=0` the gap is ~23 mm at B20 (the
    earlier safe setting); without the placement it is uneven (~9–10 / ~17–18 / ~26–27 mm at
    B20 / B34 / B46 by the model).
  - **Expect** (optional cross-check): button [MoCap bar: measure + draw] → pane 1 line
    `[mocap bar] bar ends (world, mm): Rhino start [...] end [...], above the floor a / b | MoCap ...,
    above the floor c / d`: the bar's lower end has about the same `above the floor` number in Rhino
    and MoCap (within ~3 mm).
  - **If not**, a footer **pressed** into the floor (the bar or the footer bends, the wrist force
    jumps, no gauge fits under any corner while the insert still had travel left): **stop**, do not
    fix; Cindy may be pushing the bar into the floor. Note the corners, take a photo, and next time
    start the monitor with a smaller raise (e.g. `export DESIGN_GROUND_RAISE_MM=15`).
  - **If not**, gaps above ~5 mm: note them per corner; the raise may need to grow at that bar's spot
    (tell the lead, do not change it mid-action).
  - **If not**, gaps of ~9–10 mm at B20 / ~17–18 mm at B34 / ~26–27 mm at B46: the start lines (6.c)
    say `design on the measured floor: OFF (...)`: note the reason; quit and restart the monitor
    without `DESIGN_ON_MEASURED_FLOOR=0` (and with the floor file on the drive, 1.c), or carry on
    with the uneven gap noted.
  - **If not**, one footer high on one side and pressed on the other (tilted): the bar sits turned in
    the tools; note which corners, take a photo, do not force it down with the arms.
  - **If not**, every footer off by the same amount but the start line says ON: the floor may have
    moved since its probe (2026-10-22), or the robot's base calibration is off: look at Cindy's
    `[floor] Cindy: base_footprint ... above the measured floor` line (6.c, ~0 mm expected), note
    both, and tell whoever re-probes the floor.
  **Final footer height** (footer bottom above the real floor, each corner; `DESIGN_GROUND_RAISE_MM`
  = 23, the start line in 6.c says which value ran):

  | ground bar (joints) | expected gap | tolerance | how it is set | with `DESIGN_GROUND_RAISE_MM=0` |
  |---|---|---|---|---|
  | B20 (G20-T20-0 / -1) | **0 mm** (touching) | **+2 / −0 mm**: 0–2 mm OK; pressed = stop | measured: ~23 mm gap on 2026-10-23 with raise 0 | ~23 mm |
  | B34 (G34-T20-0 / -1) | **0 mm** | **± 3 mm**: up to +3 mm OK; pressed = stop | B20's offset; the floor there is untested | ~23 mm if the floor is as at B20 |
  | B46 (G46-T20-0 / -1) | **0 mm** | **± 3 mm**: up to +3 mm OK; pressed = stop | B20's offset; the floor there is untested | ~23 mm if the floor is as at B20 |

  - The model places every footer 7.45 mm below the **fitted** floor plane (15.55 − 23); B20's
    gauge showed the real floor there is that much lower than the plane. The ± is the floor fit
    (2 mm rms, up to ~3 mm uneven between probe points); B20's 0 mm is measured, B34 / B46 are the
    first test of it.
  - Write the 4 corner gaps per joint in the run notes; if B34 / B46 read more than +3 mm or are
    pressed, the raise needs a per-spot value (tell the lead; do not change it mid-action).
- [ ] by hand: fix the bar to the foundation **at the height it is held**: fill whatever gap is
      left under a footer (a shim, or as the lead decides on site) instead of pushing the bar down →
      button [Confirm Exec]

**f) Mark**
- [ ] button [Mark action done]
  - **Expect:** `[Plan skeleton] action k <action> marked done by Cindy; Cindy's belief <- live. Saved ...`,
    header `k+1/76`; the next action loads.

### 5.3 Procedure R — release (Cindy)  [REAL RUN — recorded]

Two steps: the retreat (the ungrasp runs with it), then the free move home.

**a) Retreat** (`<bar>_R_M1_LM_retreat`)
- [ ] slider [Step]: 0 (the action opens on it) → button [Load Movement]
  - **Expect:** readout `step 1/2: <bar>_R_M1_LM_retreat  (+ <bar>_R_M0_tool_ungrasp_bar runs with it)`;
    the bar and its joints drawn solid (built, collision-checked).
- [ ] button [Plan Movement]
  - **Expect:** it plans from where the arms **are** (the insert leaves them a little off its planned end):
    - `[<bar>_R_M1_LM_retreat] start_state resynced to the live arms (max joint drift ... deg)` and
    - `[retreat live] <bar>_R_M1_LM_retreat: goals = the live tool0 poses moved 50 mm back along their own -z (encoders): left along (...), 0.1 deg from the Rhino axis, N mm from the exported goal; right ...`.
  - Each tool0 backs off 50 mm along its **own** −z as the encoders read it now (since 2026-10-22;
    before, the Rhino retreat axis), orientation kept; scene collisions are checked. The two agreed
    within 0.2° in all six retreats of 2026-10-21.
  - **If not:** `[retreat live] ...: the left tool axis is N deg from the Rhino retreat axis; check the tool before Exec.`
    (past 20°): the tool sits turned on the bar, or the export's axis is wrong; look before Exec.
  - **If not:** no path: read the `[<bar>_R_M1_LM_retreat diag]` block (7.c).
- [ ] slider [Traj viz time]: preview
  - **Expect:** starts at the live arms.
- [ ] box [Retreat screw-out: COMPLIANT (unticked = stiff ...)]: unticked (the default since 2026-10-28)
  - **Expect:** at start-up and on every click, in green:
    `retreat screw-out: STIFF (joint controller, both tools straight out 12 mm at the screw rate, 72 rpm)`.
  - **?** Ticked runs the 10-27 compliant screw-out, only to test it: a') below.
- [ ] button [Exec Selected Mv Traj (auto)]
  - **Expect:** `[Exec] retreat screw-out: STIFF`, and no force sensor zero (since 2026-10-22).
  - **Expect:** in this order (the ungrasp runs inside it), the arms stiff the whole time (joint
    controller, no switch):
    1. `[<bar>_R_M1_LM_retreat] stiff screw-out: both jaws LOOSEN while both tools back straight out 12.0 mm at 2.40 mm/s (5.0 s, joint controller), then 38 mm rigid (3.8 s).`
    2. `[scaffolding] <bar>_R_M1_LM_retreat: LOOSEN gripper motor on L arm (arm 0)` and `... on R arm (arm 1)`,
       sent in one call; then a line each time the motors' states change:
       `[<bar>_R_M1_LM_retreat] screw-out t=0.3s: gripper motors L/R LOOSENING/LOOSENING`.
       Both tools back off along the planned line at the screws' own rate (8 mm of thread + 4 mm
       margin), so the screws neither push nor pull the bar, and the tools stay on their axes:
       nothing catches in the joint on the way out;
    3. `[<bar>_R_M1_LM_retreat] screw-out done after 5.x s (gripper motors L/R <state>/<state>, still loosening); now the rest of the way out.`
       The motors keep loosening until the end (user decision 2026-10-27); on Cindy they never
       report `STALLED` after a loosen. **Note the states in the run notes**;
    4. `[<bar>_R_M1_LM_retreat] rigid chunk: N waypoints over 3.8s`: the rest of the 50 mm, at its
       share of slider [traj time];
    5. `[scaffolding] L Stop All: gripper + joint motor (arm 0)` and `R ...`, then
       `Compliant exec done for <bar>_R_M1_LM_retreat (dual_independent_linear) [stiff screw-out]; N wrench samples, peak |F| ...`.
  - **Expect:** the force plot: the axial force stays about flat during the first 5 s.
  - **If not:** it grows during the screw-out: the tools push (too slow) or pull (too fast) on the
    screws, the speed does not match them. Note it in the run notes: `RETREAT_SCREW_RPM` (72, from
    the 2026-10-28 release-test videos) may need a change.
  - **If not:** `stiff screw-out: the arms are N deg from the planned retreat start (limit 0.57 deg); nothing was sent. Re-plan the retreat (Plan Movement).`:
    the arms moved since the plan (a servo, a button). Button [Plan Movement] again, then Exec;
  - **If not:** `stiff screw-out: the planned retreat (N mm) is not longer than the 12 mm screw-out; nothing was sent.`
    → button [Plan Movement] again;
  - **If not:** `stiff screw-out: the screw rate is unknown (RETREAT_SCREW_RPM is None); nothing was sent.`
    → set it in `husky_world.py` (or tick the box for the compliant screw-out);
  - **If not:** after any of these, `[Exec] <bar>_R_M1_LM_retreat: the step stopped before moving -- nothing carried forward ...`:
    the as-built record says `stopped`, `progress.json` is unchanged; nothing was loosened. Fix the
    cause, then run the retreat again;
  - **!** [Cancel Exec] cannot stop a joint trajectory already sent: pressed during the screw-out, it
    keeps the rest from being sent (`'Cancel Exec' during the screw-out: the rest of the way out is not sent; ...`),
    the motors stop, the step counts as stopped. In danger use the e-stop.

**a') The same step with the box ticked** (compliant screw-out, test only)

Why it is not the default: on 2026-10-27 (B20_R, video `doc/IMG_5753.MOV`) the compliant tools went
3.4 of the 10 mm out and ~3.6 mm sideways; the screws pushed B20 ~6 mm away, and the rigid run then
dragged it 7-18 mm (B20 3.2 mm / 0.76° off at the end, its left end 14 mm).
Since 2026-10-28 (evening) the box first HOLDS the tools still under compliance for 4 s after the
LOOSEN, so the screws push the tools back instead of the bar (the 10-23 way: B20 0.6 mm off), then
runs the same 12 mm line. `RETREAT_COMPLIANT_HOLD_S` in `husky_world.py`: 4 = the hold, 0 = the
10-27 line right away (run 2 of 2026-10-28); restart the monitor after changing it.
- [ ] box [Retreat screw-out: COMPLIANT (unticked = stiff ...)]: ticked
  - **Expect:** `retreat screw-out: COMPLIANT (test mode: 4 s hold, then 12 mm out at the screw rate under compliance; ...)`
    in yellow (without the hold: `(test mode: 12 mm out at the screw rate ...)`).
- [ ] button [Exec Selected Mv Traj (auto)]
  - **Expect:** once
    `'<bar>_R_M1_LM_retreat': the export asks for the joint_tracking controller; the monitor runs this dual_independent_linear movement under ...`
    (expected);
  - **Expect:** no force sensor zero (since 2026-10-22): the compliance reads the forces against
    the mount-pause zero (5.2 b, tools empty), so it lets the arms give up the bar's own load
    (its weight, the mount's preload) instead of springing the bar;
  - **Expect:** in this order (the ungrasp runs inside it):
    1. both arms switch to compliance while still gripping: they stay put;
    2. the screw-out: `[scaffolding] <bar>_R_M1_LM_retreat: LOOSEN gripper motor on L arm (arm 0)` (and R), and
       `[<bar>_R_M1_LM_retreat] screw-out: both jaws LOOSEN; the tools hold still under compliance for 4.0 s (the screws push them back), then back out 12 mm along the retreat axis at 2.40 mm/s (5 s = 6.0 turns of a 2 mm pitch), under compliance.`
       4 s later: `[<bar>_R_M1_LM_retreat] hold done after 4.0 s: the tools went L +x.x / R +x.x mm out along the retreat axis (a / b mm sideways); now 12 mm out from there.`
       **Note these numbers in the run notes**: how far the screws pushed each tool back.
       (Without the hold: `screw-out: both jaws LOOSEN while the tools back out 12 mm along the retreat axis ...`.)
       The tools are asked to back out at the screws' rate (8 mm + 4 mm margin). (On 2026-10-23 the
       tools held still for 2 s, then sat 11 s in front of the bar while the screws turned: 44-47 N
       on the bar at every release, video `doc/IMG_5692.MOV`.) A line each time the tool's motor
       states change: `[<bar>_R_M1_LM_retreat] screw-out t=0.3s: gripper motors L/R LOOSENING/LOOSENING`;
    3. `[<bar>_R_M1_LM_retreat] screw-out done after 9.x s; the gripper motors report L/R <state>/<state> and keep loosening until the retreat ends.`
       (5.x s without the hold.)
    4. the rigid part (next lines), replanned from where the screw-out left the tools:
       `[<bar>_R_M1_LM_retreat] split at 5.0 mm from the assembled pose: a 4 s COMPLIANT hold, then 12 mm COMPLIANT screw-out (5 s at 2.40 mm/s) along the first 5.0 mm, then 45.0 mm RIGID (N waypoints, Ts, replanned from where the screw-out ends).`
       The 5 mm is the insert's slider [Insert: rigid->compliant split (mm to goal)], mirrored; it
       only says where the rigid tail starts, the screw-out's length is fixed by the thread;
  - **If not:** `screw-out: RETREAT_SCREW_TURN_S is not measured, so the tools hold still ... 2.0 s ...`
    (a WARN): the screw rate is unset in `husky_world.py` (`RETREAT_SCREW_RPM`): the old 2 s
    hold-still runs and the bar gets pushed;
  - **If not:** the jaws are still visibly closed on the bar when the rigid part starts: button
    [Cancel Exec] at once (during the screw-out it stops there: `'Cancel Exec' during the screw-out;
    not moving out.`); the step is recorded as cancelled and nothing is carried forward. Open the
    jaws with button [Loosen Grippers (BOTH)], then button [Exec Selected Mv Traj (auto)] again;
  - **If not:** `[Exec] <bar>_R_M1_LM_retreat: the step stopped before moving -- nothing carried forward ...`
    (a controller switch failed): the as-built record says `stopped`, `progress.json` is unchanged;
    fix the controller (6.b), then run the retreat again;
  - **Expect:** after the screw-out the monitor prints its checks of the rigid part,
    `[<bar>_R_M1_LM_retreat] check PASS / FAIL ...` (replan, start, compliant part = the screw-out's
    travel, contact, grippers (report only), left / right tool0 path, joints, speed), then runs the
    rigid part at once, **no `Confirm Exec`**: `running the rigid part now (N waypoints over T s); every check passed.`
    Or `N check(s) FAILED, see the "check FAIL" lines above`: note them (7.c);
  - **Expect:** joint controller restored afterwards.

**b) Free move home** (`<bar>_R_M2_free_home`, may be skipped)
- [ ] slider [Step]: 1 → button [Load Movement] → button [Plan Movement]
- [ ] slider [Traj viz time]: preview → button [Exec Selected Mv Traj (auto)]
  - It may be skipped: the retreat's end is already saved, and the next jointing action's
    transfer plans from where the arms are (collision checked).

**c) Mark**
- [ ] button [Mark action done]
  - **Expect:** header `k+1/76`.

### 5.4 Procedure H — hold (Alice, Belle)  [REAL RUN — recorded]

Four steps: free approach, gripper open, straight approach (100 mm), gripper close + compliant
handoff. `<b>` = the bar (B21 Alice, B35 Belle).

**a) Load**
- [ ] slider [Action (idx)]: k → button [Load action] (usually loaded by itself: then skip)
  - **Expect:** `[obstacle robots] posed from: ...` (from `progress.json`; a fresh mocap base still
    overrides it). Readout `step 1/4: <b>_H_M0_free_to_approach ...`, `traj time` 15 s. The
    `Transfer start:` and `Insert: ...` controls are hidden.
  - Bars not built yet may be drawn too: the hold is exported with the bars that stand when it
    ends (Rhino note D5); the planner just avoids them.
  - **Expect:** the blue goal ghost stands where Rhino puts the robot, with its arm at the approach pose
    (`[goal ghost] '<b>_H_M0_free_to_approach': Rhino's base and target pose (drive the base there); ...`).
- [ ] joystick: drive the base towards the blue goal ghost (or from the command line, 6.a)
  - The plan itself starts from the live base and arm.

**b) Free approach** (`<b>_H_M0_free_to_approach`)
- [ ] button [Plan Movement]
  - **Expect:** the goal is re-solved by IK at the live base, with the robot's own exact ssik solver (built
    from its calibrated URDF):
    - once per run `[goal IK] ssik solver '<alice|belle>' fits the planner's robot.`, then
      `[goal IK] ssik: solution pair 1/8 OK (...)`;
    - `[live goal] <b>_H_M0_free_to_approach: goal re-solved at the live base; max |live IK - stored target| = ... rad.`;
    - `[Plan] '<b>_H_M0_free_to_approach': N waypoints stored.` (random planner, N varies).
  - **If not:** `[goal IK] no ssik solver (alice, belle) fits the planner's robot: PyBullet IK instead ...`
    = the planning cell is not the robot's calibrated model: PyBullet IK is used instead, accurate
    to 1 mm only; note it.
  - **If not:** `... no collision-free IK for its approach pose at the live base (...); not planning.`
    (the collision drawn if there is one): the robot stands where the approach pose cannot be
    reached → drive its base again, plan again.
  - **If not:** `[live goal] ...: base not tracked; ...` = mocap does not see the robot (Motive
    1840 / 1850, 2.e).
- [ ] slider [Traj viz time]: preview
  - **Expect:** ends with the open jaws 100 mm in front of the bar, clear of Cindy's arms and tools.
- [ ] slider [traj time]: higher = slower → button [Exec Selected Mv Traj (auto)]
  - **Expect:** the arm goes there. The exec then stays running and **servos tool0 onto the approach
    pose**, as Cindy's transfer does (the arm's motion shifts the base under it):
    `[servo 0] tool0 pos X mm, rot Y deg` (before any correction), `### SERVOING iteration 1/8`,
    then the pause `[servo] First move planned: tool0 X mm, joints Y deg, over T s. ...`.
- [ ] slider [Traj viz time]: scrub it (a mm-size move) → button [Confirm Exec]
  - Later moves run alone unless they travel more than 5 deg (`[servo] SAFEGUARD ...` →
    `Confirm Exec`).
- [ ] pane 1: read the end
  - **Expect:** `### SERVOING converged at iter N (< 0.2 mm)` or
    `### SERVOING stopped at iter N: nothing left to plan ...` (or after 8 moves), then the
    `[Servo] propagated arm configuration -> ... + robot_base_frame (live mocap).` lines for the
    gripper step and the straight approach (details: 6.e). An approach planned before is
    dropped: plan it again.

**c) Calibration check**
- [ ] by eye: the jaws stand ≈ 100 mm in front of the bar, **centred** on it
  - **If not:** off by more than ~1–2 cm sideways → stop: the base calibration is wrong
    (recalibrate the robot, `doc/calibration_manual.md`, and point its
    `BASE_CALIBRATION_DATE_BY_ROBOT` entry at the new folder; 1.c).

**d) Gripper open** (`<b>_H_M1_gripper_open`)
- [ ] slider [Step]: 1 → button [Load Movement] → button [Exec gripper step: OPEN]
  - **Expect:** `[gripper step <b>_H_M1_gripper_open] OPEN the gripper fully (position 0.426) [SupportGripper]. Click 'Confirm Exec' ...`
- [ ] button [Confirm Exec]
  - **Expect:** `gripper 0 done: position 0.426, ...`.

**e) Straight approach** (`<b>_H_M2_LM_to_grasp`)
- [ ] slider [Step]: 2 → button [Load Movement] → button [Plan Movement]
  - **Expect:** `traj time` 5 s;
  - **Expect:** `[<b>_H_M2_LM_to_grasp] start_state resynced to the live arms (max joint drift from the propagated conf: ... deg)`:
    it plans from where the arm really is, so the path stays on the arm's IK branch. A drift near
    360 deg = the chained start was a full turn off, now ignored;
  - **Expect:** `[Plan] '<b>_H_M2_LM_to_grasp': ≈ 22 waypoints stored.` (100 mm straight).
- [ ] slider [Traj viz time]: preview → button [Exec Selected Mv Traj (auto)]
  - **Expect:** the open jaws slide onto the bar.

**f) Gripper close + handoff** (`<b>_H_M3_gripper_close`)
- [ ] slider [Step]: 3 → button [Load Movement] → button [Exec gripper step: CLOSE + compliant handoff]
- [ ] hand on the robot's e-stop → button [Confirm Exec]
  - **Expect:** the phase lines of 4.1 e). The arm is compliant for the whole close: it gives way while
    the jaws centre on the bar, then `gripper done: position ..., stalled True, ...` and the joint
    controller is back.
  - **!** Watch Cindy's tools: they must not be pulled.

**g) Mark and quit**
- [ ] button [Mark action done]
  - **Expect:** `[Plan skeleton] action k <action> marked done by <robot>; <robot>'s belief <- live. Saved ...`.
  - `Mocap has not seen Alice in the last 1 s; ...` = her base was not tracked: the authored base
    is stored instead; note it.
- [ ] pane 1: `Ctrl-C` (quit the monitor)
  - The robot stays clamped on the bar (its driver keeps running).

### 5.5 Procedure HR — let go (Alice, Belle)  [REAL RUN — recorded]

Two steps: gripper open, straight retreat (100 mm) back to the approach pose.

**a) Load**
- [ ] slider [Action (idx)]: k → button [Load action] (usually loaded by itself: then skip)
  - **Expect:** readout `step 1/2: <b>_HR_M0_gripper_open ...`.

**b) Gripper open** (`<b>_HR_M0_gripper_open`)
- [ ] button [Exec gripper step: OPEN] → button [Confirm Exec]
  - **Expect:** `gripper 0 done: position 0.426, ...`. The bar must stay where it is.

**c) Retreat** (`<b>_HR_M1_LM_retreat`)
- [ ] slider [Step]: 1 → button [Load Movement] → button [Plan Movement]
  - **Expect:** `traj time` 5 s;
  - **Expect:** `[<b>_HR_M1_LM_retreat] start_state resynced to the live arms (max joint drift ... deg)`: it
    starts where the handoff left the arm;
  - **Expect:** `[Plan] '<b>_HR_M1_LM_retreat': ≈ 21 waypoints stored.`, back to the approach pose.
- [ ] slider [Traj viz time]: preview → button [Exec Selected Mv Traj (auto)]
  - **Expect:** the open jaws slide off the bar (plain joint tracking, no servo). Watch the jaws clear the
    bar.

**d) Mark**
- [ ] button [Mark action done]
  - **Expect:** `[Plan skeleton] action k <action> marked done by <robot>; <robot>'s belief <- live. Saved ...`.

**e) Quit and drive away**
- [ ] pane 1: `Ctrl-C` (quit the monitor) → joystick: drive the robot away from the structure (or 6.a)
  - The export assumes it leaves the scene; the next run draws it parked.

### 5.6 Run 1 — Cindy, actions 0–2  [REAL RUN — recorded]

- [ ] pane 1: Cindy's block → start the monitor (5.1)
  - **Expect:** `[Plan skeleton] robot Cindy (domain 86) | problem 261023_2ModulesDemo | progress 0/76 done`
    and `[Plan skeleton] others: Alice <- parked | Belle <- parked`.
- [ ] Actions 0–2 in turn, each with the procedure in the table (J: 5.2, R: 5.3):

| action | procedure | what differs |
|---|---|---|
| 0 `B20_J_joint` | J, ground bar (six steps: travel to load (skipped), manual mount, tool grasp, transfer, insert, fix foundation) | opens on `step 4/6: B20_J_M3_CDFM_transfer_to_approach ...`. B20 stands on its two ground joints `joint_G20-T20-0_ground`, `joint_G20-T20-1_ground` and has no tighten step. Insert readout `step 5/6: B20_J_M4_LM_insert  (dual_constrained_linear)`; the joint motors do not turn. Fix `B20_J_M5_manual_fix_foundation`. Mark: `[Plan skeleton] action 0 B20_J_joint marked done by Cindy; Cindy's belief <- live. Saved ...`, header `1/76` |
| 1 `B20_R_release` | R | `B20_R_M1_LM_retreat` (`B20_R_M0_tool_ungrasp_bar` runs with it), then `B20_R_M2_free_home`. Mark: `2/76` |
| 2 `B21_J_joint` | J, one-sided (five steps; the tighten runs with the insert) | built B20 and its joints drawn solid; the transfer's preview clears them. Transfer `B21_J_M3_CDFM_transfer_to_approach` (readout `step 4/5: ...`). Insert readout `step 5/5: B21_J_M5_LM_insert  (+ B21_J_M4_tool_tighten_joint runs with it)`. B21's left male (`joint_J20-21_male`) screws into B20. The right tool holds `joint_J157-21_male`, whose female sits on B157, outside this export. The export already names only `AT3L`, so there is no `tightens only ['AT3L']` warning. Optional spiral search (6.f), to compare with the straight line. Mark: `3/76`: Cindy's live base and arms were already saved to `progress.json` when the insert ended (Alice's run plans around them) |

- [ ] pane 1: `Ctrl-C` (quit Cindy's monitor), or button [Switch to Alice]
  - Cindy keeps holding B21 (her driver keeps running). Do not touch her launch pane.

### 5.7 Run 2 — Alice, action 3: Alice grabs B21  [REAL RUN — recorded]

- [ ] pane 1: Alice's block → start the monitor (5.1); or button [Switch to Alice]
  - **Expect:** `robot Alice (domain 84) ... progress 3/76 done` and
    `others: Cindy <- live (action 2) | Belle <- parked`; the slider on action 3, its row not grey.
  - **Expect:** 3D view: Cindy holding B21 at her real base (mocap; her arms from `progress.json`). She may
    show twice at the same pose: the solid husky and the red obstacle robot — the red one is
    what the planner checks.
- [ ] Procedure H (5.4) with `<b>` = B21. What differs:
  - load: `[obstacle robots] posed from: ObstacleRobotBelle <- exported, ObstacleRobotCindy <- live`.
    The export parks Belle far away. B21 is drawn in Cindy's tools; B22 and B34–B37 may be drawn
    too (not built yet, Rhino note D5);
  - free approach: `[goal IK] ssik solver 'alice' fits the planner's robot.`; ≈ 65 waypoints
    (random planner, varies); drive with domain 84, `/a200_0804` (6.a); the servo opens the
    `Servoing Live Tracker` (one arm, plus the base drift) and saves
    `B21_H_M0_free_to_approach_servoing_data_<ts>.json` / `_performance_<ts>.png`;
  - calibration check: Alice's June calibration (1.c);
  - Mark: `[Plan skeleton] action 3 B21_H_hold marked done by Alice; Alice's belief <- live. Saved ...`,
    header `4/76`.
- [ ] pane 1: `Ctrl-C` (quit Alice's monitor), or button [Switch to Cindy]
  - Alice stays clamped on B21.

### 5.8 Run 3 — Cindy, actions 4–9  [REAL RUN — recorded]

- [ ] pane 1: Cindy's block → start the monitor (5.1); or button [Switch to Cindy]
  - **Expect:** `[Plan skeleton] Cindy starts from the progress.json state (after action 2, live).` (her
    real joints take over at once), header `4/76`,
    `others: Alice <- live (action 3) | Belle <- parked`.
  - **Expect:** 3D view: Alice drawn on B21 at her hold pose (mocap base).
- [ ] Actions 4–9 in turn, each with the procedure in the table (J: 5.2, its a) ticks the
      workaround box again; R: 5.3):

| action | procedure | what differs |
|---|---|---|
| 4 `B21_R_release` | R, Alice holding B21 | load: `[obstacle robots] posed from: ObstacleRobotAlice <- live, ObstacleRobotBelle <- exported`; opens on `step 1/2: B21_R_M1_LM_retreat  (+ B21_R_M0_tool_ungrasp_bar runs with it)`. **B21 is clamped by Alice** (the ungrasp releases Cindy's tools from it). Retreat and `B21_R_M2_free_home` are planned with Alice in the scene: check in the retreat's preview that both tools back off clear of Alice's gripper. Mark: `5/76` |
| 5 `B22_J_joint` | J, both joints mate (five steps, as action 2) | B22's left male (`joint_J21-22_male`) screws into B21, which Alice still holds; its right male (`joint_J20-22_male`) into B20. Built B20, B21 and Alice (red obstacle robot) drawn; the transfer's preview clears them all. Transfer `B22_J_M3_CDFM_transfer_to_approach`: a warning `Plan end of 'B22_J_M3_CDFM_transfer_to_approach' differs from existing 'B22_J_M5_LM_insert'.start by max 6.2832 rad/m; overwriting (carry rule).` means one joint of the insertion start is a full turn (2π) from the exported one: same tool poses, harmless (seen headless). Insert readout `step 5/5: B22_J_M5_LM_insert  (+ B22_J_M4_tool_tighten_joint runs with it)`. **!** Watch B21 while B22 screws in: Alice holds it; it must not be pushed out of her jaws. Mark: `6/76` |
| 6 `B22_R_release` | R | opens on `step 1/2: B22_R_M1_LM_retreat  (+ B22_R_M0_tool_ungrasp_bar runs with it)`; then `B22_R_M2_free_home`; Alice in the scene. Mark: `7/76` |
| 7 `B34_J_joint` | J, ground bar (six steps, as action 0) | B34 is the second ground bar: it stands on its ground joints `joint_G34-T20-0_ground` and `joint_G34-T20-1_ground`, has no tighten step, and ends with a manual foundation fix. The base drives about 1.1 m from where it stood for B22. Built B20–B22 and Alice drawn; the transfer's preview clears them. Transfer `B34_J_M3_CDFM_transfer_to_approach`. Insert `B34_J_M4_LM_insert`: `[B34_J_M4_LM_insert] grounded bar (holds joint_G34-T20-0_ground, joint_G34-T20-1_ground): rigid insert ...` and `... rigid insert done (no tighten).`; the joint motors do not turn. Fix `B34_J_M5_manual_fix_foundation`. Mark: `8/76` |
| 8 `B34_R_release` | R | `step 1/2: B34_R_M1_LM_retreat  (+ B34_R_M0_tool_ungrasp_bar runs with it)`, then `B34_R_M2_free_home`. Mark: `9/76` |
| 9 `B35_J_joint` | J, one-sided (five steps, as action 2) | B35 is **one-sided**, like B21: its left male (`joint_J34-35_male`) screws into B34; the right tool holds `joint_J162-35_male`, whose female sits on B162, outside this export; the export names only `AT3L` for the tighten. Transfer `B35_J_M3_CDFM_transfer_to_approach` (mount B35 in the pause). Insert `B35_J_M5_LM_insert`: `[B35_J_M5_LM_insert] one-sided bar: the left joint mates, so only the L joint motor TIGHTENs ...` and `[scaffolding] B35_J_M5_LM_insert: TIGHTEN joint motor on L arm (arm 0)` (no R line). After the insert Cindy holds B35, joint controller active. Mark: `10/76` (Belle's run plans around Cindy's stored pose) |

- [ ] pane 1: `Ctrl-C` (quit Cindy's monitor), or button [Switch to Belle]
  - Cindy keeps holding B35.

### 5.9 Run 4 — Belle, action 10: Belle grabs B35  [REAL RUN — recorded]

The same four steps as Alice's hold. Her start-up prints
`Base calibration of Belle (0805): 20261013/...` (1.c); the calibration check after the free
approach is the first test of it on an assembly.

- [ ] pane 1: Belle's block → start the monitor (5.1); or button [Switch to Belle]
  - **Expect:** `robot Belle (domain 85) ... progress 10/76 done` and
    `others: Cindy <- live (action 9) | Alice <- live (action 3)`; the slider on action 10, its
    row not grey.
  - **Expect:** 3D view: Cindy holding B35, Alice clamped on B21.
- [ ] Procedure H (5.4) with `<b>` = B35. What differs:
  - load: `[obstacle robots] posed from: ObstacleRobotAlice <- live, ObstacleRobotCindy <- live`.
    B36 and B37 may be drawn although not built yet (Rhino note D5);
  - free approach: her own exact ssik solver (built 2026-10-13 from her calibrated URDF, like
    Alice's). Once per run `[goal IK] ssik solver 'alice' does not fit the planner's robot (built from another robot or calibration).`
    (expected: Alice's solver is tried first), then `[goal IK] ssik solver 'belle' fits the planner's robot.`
    and `[goal IK] ssik: solution pair 1/8 OK (...)`; ≈ 63 waypoints (varies); drive with domain
    85, `/a200_0805` (6.a); `base not tracked`: check Motive 1850;
  - servo: headless, a 10 mm base shift took one move, from 10 to 0.00 mm (or it stops after 8); the
    record `B35_H_M0_free_to_approach_servoing_*`;
  - calibration check: Belle's 2026-10-14 calibration (1.c);
  - handoff: hand on Belle's e-stop; watch Cindy's tools (must not be pulled);
  - Mark: `[Plan skeleton] action 10 B35_H_hold marked done by Belle; Belle's belief <- live. Saved ...`,
    header `11/76`.
- [ ] pane 1: `Ctrl-C` (quit Belle's monitor), or button [Switch to Cindy]
  - Belle stays clamped on B35.

### 5.10 Run 5 — Cindy, actions 11–15  [REAL RUN — recorded]

- [ ] pane 1: Cindy's block → start the monitor (5.1); or button [Switch to Cindy]
  - **Expect:** `[Plan skeleton] Cindy starts from the progress.json state (after action 9, live).`,
    header `11/76`, `others: Alice <- live (action 3) | Belle <- live (action 10)`.
  - **Expect:** 3D view: Alice on B21 and Belle on B35, at their hold poses (mocap bases).
- [ ] Actions 11–15 in turn, each with the procedure in the table (J: 5.2, its a) ticks the
      workaround box again; R: 5.3):

| action | procedure | what differs |
|---|---|---|
| 11 `B35_R_release` | R, Belle holding B35 | load: `[obstacle robots] posed from: ObstacleRobotAlice <- live, ObstacleRobotBelle <- live`; opens on `step 1/2: B35_R_M1_LM_retreat  (+ B35_R_M0_tool_ungrasp_bar runs with it)`. **B35 is clamped by Belle** (the ungrasp releases Cindy's tools from it). Then `B35_R_M2_free_home`. Check in the retreat's preview that both tools back off clear of Belle's gripper. Mark: `12/76` |
| 12 `B36_J_joint` | J, both joints mate (as action 5) | B36's left male (`joint_J35-36_male`) screws into B35, which Belle holds; its right male (`joint_J34-36_male`) into B34. Transfer `B36_J_M3_CDFM_transfer_to_approach`, two messages seen headless: the same 2π warning as B22's (`... differs from existing 'B36_J_M5_LM_insert'.start by max 6.2832 ...`): harmless; and once `[free validation] ... SWEPT COLLISION ... hits ['ground_WG0']` with `plan REJECTED: it sweeps through the scene between waypoints`: the insertion start is close to the floor and one random path grazed it → `Plan Movement` again. Insert `B36_J_M5_LM_insert`: **both** joint motors tighten. **!** Watch B35 while B36 screws in: Belle holds it; it must not be pushed out of her jaws. Mark: `13/76` |
| 13 `B36_R_release` | R | `step 1/2: B36_R_M1_LM_retreat ...`, then free move home; Alice and Belle in the scene. Mark: `14/76` |
| 14 `B37_J_joint` | J, both joints mate (as action 5) | the base drives about 1.5 m from where it stood for B36. B37's left male (`joint_J21-37_male`) screws into **B21 (Alice)**; its right male (`joint_J35-37_male`) into **B35 (Belle)**. Transfer `B37_J_M3_CDFM_transfer_to_approach`: the 2π warning may show here too (harmless). Insert `B37_J_M5_LM_insert`: **both** joint motors tighten. **!** Watch B21 and B35 while B37 screws in: neither may be pushed out of its support robot's jaws. Mark: `15/76` |
| 15 `B37_R_release` | R | `step 1/2: B37_R_M1_LM_retreat ...` → retreat. **!** **Free move home** (`B37_R_M2_free_home`): headless, the first `Plan Movement` failed in both of two runs (`plan_free_dual_arm failed: birrt_failed` and its diag block); the second try planned. Count on that and **plan again**. The home goal turns the right shoulder (`right_ur_arm_shoulder_pan_joint`) by about **5.05 rad (≈ 290°)**, past B36: preview it through, raise `traj time`, hand on Cindy's e-stop. Mark: `16/76` |

- [ ] pane 1: `Ctrl-C` (quit Cindy's monitor), or button [Switch to Alice]

### 5.11 Run 6 — Alice, action 16: Alice lets go of B21  [REAL RUN — recorded]

B21 now hangs in the structure (jointed to B20, B22 and B37); Alice opens her jaws and backs out
100 mm along her approach line.

- [ ] pane 1: Alice's block → start the monitor (5.1); or button [Switch to Alice]
  - **Expect:** `robot Alice (domain 84) ... progress 16/76 done` and
    `others: Cindy <- live (action 15) | Belle <- live (action 10)`.
- [ ] Procedure HR (5.5) with `<b>` = B21. What differs:
  - gripper open `B21_HR_M0_gripper_open`: B21 must stay where it is;
  - retreat `B21_HR_M1_LM_retreat`;
  - Mark: `[Plan skeleton] action 16 B21_HR_hold_release marked done by Alice; Alice's belief <- live. Saved ...`,
    header `17/76`;
  - drive away with domain 84 (6.a). The export assumes she leaves the scene, and Belle's run
    draws her parked.

### 5.12 Run 7 — Belle, action 17: Belle lets go of B35  [REAL RUN — recorded]

The same as 5.11, with Belle's values.

- [ ] pane 1: Belle's block → start the monitor (5.1); or button [Switch to Belle]
  - **Expect:** `robot Belle (domain 85) ... progress 17/76 done`.
- [ ] Procedure HR (5.5) with `<b>` = B35. What differs:
  - load: `[obstacle robots] posed from: ObstacleRobotAlice <- parked, ObstacleRobotCindy <- live`
    (Alice released), readout `step 1/2: B35_HR_M0_gripper_open ...`;
  - gripper open: B35 stays where it is;
  - retreat `B35_HR_M1_LM_retreat`: `[Plan] 'B35_HR_M1_LM_retreat': ≈ 21 waypoints stored.`;
    watch the jaws clear B35;
  - Mark: header `18/76`.
- [ ] The first module is done; the second goes on with run 8 (5.13).

### 5.13 Runs 8–23 — the second module, actions 18–75  [REAL RUN — recorded]

The same procedures (J 5.2, R 5.3, H 5.4, HR 5.5) and the same run start / switch (5.1). Each run
starts with `progress k/76 done`, k = its first action; Mark after its last action shows the next
run's k.

| run | robot | actions | what differs |
|---|---|---|---|
| 8 | Cindy | 18 `B25_J` | B25 **one-sided, the right joint mates** (the tighten runs on the right arm only) |
| 9 | Alice | 19 `B25_H` | Alice grabs B25 (her second hold; she let go of B21 in run 6) |
| 10 | Cindy | 20 `B25_R` – 27 `B39_J` | B25 release (Alice holding), B169, B26, B165 (both joints), then B39 **one-sided, right**. **!** B39's export holds a left-arm pose 100 mm low (1.d): after its transfer is planned, slider [Traj viz time] to the end: both tools must sit on B39's joints. If the left tool shows 100 mm low, stop and note it; do not use `Move Arms to Movement Start (offline target)` on B39's tighten / insert steps |
| 11 | Belle | 28 `B39_H` | Belle grabs B39 (her second hold) |
| 12 | Cindy | 29 `B39_R` – 39 `B43_R` | B39 release (Belle holding), then B40, B38, B41, B42, B43 (both joints) |
| 13 | Alice | 40 `B25_HR` | Alice lets go of B25 |
| 14 | Belle | 41 `B39_HR` | Belle lets go of B39 |
| 15 | Cindy | 42 `B44_J` – 48 `B47_J` | B44, B45, then **B46: ground bar** (as B20 / B34: rigid insert, no tighten, **fix foundation**, footer gaps), then B47 **one-sided, left** |
| 16 | Alice | 49 `B47_H` | Alice grabs B47 |
| 17 | Cindy | 50 `B47_R` – 54 `B49_R` | B47 release (Alice holding), B48, B49 |
| 18 | Alice | 55 `B47_HR` | Alice lets go of B47 |
| 19 | Cindy | 56 `B50_J` | B50 **one-sided, right** |
| 20 | Alice | 57 `B50_H` | Alice grabs B50 |
| 21 | Cindy | 58 `B50_R` – 70 `B57_R` | B50 release (Alice holding), B52, B53, B54, B56, B55, B57 |
| 22 | Alice | 71 `B50_HR` | Alice lets go of B50 |
| 23 | Cindy | 72 `B59_J` – 75 `B168_R` | B59, B168; Mark: header `76/76` |

- **Instrumented bars** (MoCap balls, measured at every step; 1.d): B20, B21, B25, B34, B35, B39,
  B46, B47, B50. In Motive their assets stream as 1000 + the bar number (B25 = 1025, B39 = 1039,
  B46 = 1046, B47 = 1047, B50 = 1050), with **Labeled Markers** on (9.1).
- Ground bars: B20, B34, B46 (actions 0, 7, 46). One-sided: B21, B35, B47 (left mate); B25, B39,
  B50 (right mate).
- **?** Only actions 0–17 have run on the robots (from 261006). Actions 18–75 have been checked
  headless only (they load and pass the action checks); expect first-time surprises, note them (Part 8).
- [ ] After run 23: the assembly is done: Part 8.

---

## 6. Reference

### 6.a Driving a base from the command line (when the joystick does not connect)

In pane 5. The commands below drive Cindy. For Alice use `ROS_DOMAIN_ID=84` and **`/a200_0804`**;
for Belle `ROS_DOMAIN_ID=85` and **`/a200_0805`**. (On 2026-10-16 Alice's and Belle's bases were
not visible from this PC, so this may only work for Cindy; use their joysticks.)

The base listens on `<ns>/cmd_vel` through `twist_mux`:
- **The base stops 0.5 s after the last message**, so commands are published continuously;
  stopping the publisher stops the base.
- The joystick, the RC remote and the interactive marker **override** `cmd_vel` while they
  publish; the Husky e-stop locks out everything.
- **!** Hand on the e-stop; drive slowly near the structure. After driving, plan again (the monitor
  plans at the live base).
- **!** Never drive Alice while she holds B21 (actions 3–16), nor Belle while she holds B35
  (actions 10–17).

Where a base should stand: on `Load Movement` the monitor logs
`[LoadMovement] cfab planning robot is correctly placed at [x y z], but: live husky [...] (err E m)`
when the live base is more than 1 mm from where the export authored the movement. `E` is how far
the real robot stands from that pose. Drive until the IK check (Cindy) or the free approach's goal
IK (Alice, Belle) succeeds; `E` does not have to reach 0.

**Keyboard (recommended):**

**▶ pane 5 · CHECKS**
```bash
ROS_DOMAIN_ID=86 ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:=/a200_0806/cmd_vel -p speed:=0.1 -p turn:=0.3
```

| key | does |
|---|---|
| `i` / `,` (hold) | forward / backward |
| `j` / `l` | turn left / right on the spot |
| `u` / `o` | forward while turning |
| `k` (or any other key) | stop; releasing the key also stops the base within 0.5 s |
| `q` / `z` | raise / lower both speeds by 10 % |
| `w` / `x` | only the linear speed |
| `e` / `c` | only the turning speed |
| `Ctrl-C` | quit |

**!** Every key typed in that pane is a drive command — quit it (`Ctrl-C`) before using the pane for
anything else.

**Fixed moves** (`-r 10` = 10 messages/s, `-t N` = N messages = N/10 s; `Ctrl-C` stops early):

**▶ pane 5 · CHECKS**
```bash
# forward 0.1 m/s for 2 s (about 20 cm); x: -0.1 to back up
ROS_DOMAIN_ID=86 ros2 topic pub -r 10 -t 20 /a200_0806/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
# turn on the spot 0.2 rad/s for 3 s (about 34 deg, counter-clockwise seen from above); z: -0.2 = clockwise
ROS_DOMAIN_ID=86 ros2 topic pub -r 10 -t 30 /a200_0806/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.2}}"
```

**What reaches the base:** `ROS_DOMAIN_ID=86 ros2 topic echo /a200_0806/platform/cmd_vel_unstamped`;
`ROS_DOMAIN_ID=86 ros2 topic echo /a200_0806/platform/emergency_stop` shows `data: true` while the
e-stop is engaged.

### 6.b Safety details and switching an arm back to the joint controller

- Loading a movement sets `traj time` to the movement's default:
  - Cindy: transfer 10 s, insert / retreat 5 s, free move home 10 s;
  - Alice and Belle: free approach 15 s, straight approach and hold-release retreat 5 s.
- If a log line says an arm may still be under the compliance controller: press
  **`Switch to Joint (BOTH)`** (under `CONTROLLERS`; on a support robot it switches its one arm),
  or:

**▶ pane 5 · CHECKS**
```bash
( export ROS_DOMAIN_ID=86   # Cindy; the ( ) keep the domain inside this block
  for a in left right; do
    ros2 service call /a200_0806/${a}_ur5e/controller_manager/switch_controller \
        controller_manager_msgs/srv/SwitchController \
        "{deactivate_controllers: [cartesian_compliance_controller], activate_controllers: [scaled_joint_trajectory_controller], strictness: 1}"
  done )
```

**▶ pane 5 · CHECKS**
```bash
# Alice (domain 84); Belle: ROS_DOMAIN_ID=85 and /a200_0805 instead of /a200_0804
ROS_DOMAIN_ID=84 ros2 service call /a200_0804/ur5e/controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{deactivate_controllers: [cartesian_compliance_controller], activate_controllers: [scaled_joint_trajectory_controller], strictness: 1}"
```

### 6.c What every monitor start prints

- [ ] One base calibration line per robot (all three huskies are created):
  - `Base calibration of Cindy (0806): 20261016/calibrated_transformation_0806_rhino.json`
  - `Base calibration of Alice (0804): 20260623/calibrated_transformation_0804_rhino.json`
  - `Base calibration of Belle (0805): 20261013/calibrated_transformation_0805_rhino.json`

  `Base calibration file not found for robot ...` = that robot's file is missing or not synced
  yet (1.c): stop before that robot's actions.
- [ ] `[Plan skeleton] robot <Cindy (domain 86) | Alice (domain 84) | Belle (domain 85)> | problem 261023_2ModulesDemo | progress k/76 done`,
      `[Plan skeleton] others: ...`, the 18-action roster, and `mocap client connected: True`
      (`False`: `CLIENT_IP`, 1.b), followed by `insert spiral search: OFF (...)` and
      `servo feedback: tool0 (...)` (yellow; green when the box is ticked).
- [ ] The floor lines (the design placed on the measured floor, since 2026-10-23):
  - green `design on the measured floor: ON (the Rhino ground at 7.45 mm (WalkableGround raised by DESIGN_GROUND_RAISE_MM = 23.0 mm) laid on the floor: 0.49 deg tilt,
    -10.0 mm at the MoCap origin, -45 to -10 mm over the probed floor; MoCap poses unchanged; ...)`:
    every Rhino pose (bars, joints, targets, the room) is tilted and moved so Rhino's ground, raised
    by 23 mm, lies on the real floor; in 261023 the ground bars' footers then just touch it (5.2 e);
    robot bases, `progress.json` and the markers are not moved. `7.45 mm` / `23.0 mm` differ when
    `DESIGN_GROUND_RAISE_MM` was exported (0 = the earlier safe 15.5 mm gap).
    Yellow `... OFF (<why>)` = the design as exported (footers ~9 / 17 / 26 mm above the floor at B20 / B34 / B46):
    `DESIGN_ON_MEASURED_FLOOR=0` was exported, or the floor file is missing (1.c).
  - once per robot, as soon as its mocap base arrives:
    `[floor] Cindy: base_footprint +x.x mm above the measured floor, leaning y.yy deg from its up`:
    ~0 mm and ~0 deg; a warning past 10 mm or 1 deg (the floor file or that robot's base calibration is off).
- [ ] Within a few seconds: `[Plan skeleton] auto-load at start-up: action k (<id>, <robot>), the first pending one.`,
      then the action's movement roster; the `Now:` line names action k. After 15 s without the
      robot's joints or mocap base it loads anyway with a warning
      (`no arm joint states ... after 15 s; loading the action anyway ...`): fix the driver /
      mocap (Part 3), then `Load action`.
- [ ] Panel: a **Plan skeleton** section, with:
  - the three `Switch to ...` buttons, the current robot's marked `(current)`;
  - `Ignore built-bar collisions (bars drawn faint)` unticked;
  - the slider above `Load Movement` called `Step`;
  - an `Actions` list. Clicking a row selects that action: the `Action (idx)` slider jumps there;
    it loads only with `Load action`. The other robots' actions are grey, `(other robot)`.
- [ ] 3D view:
  - the connected robot drawn at its real base (mocap) with its real arm poses
    (`Toggle Joint Live Stream` follows the joints);
  - the other two drawn where mocap sees them, or where `progress.json` left them.

### 6.d Optional diagnostic buttons (any loaded movement)

- **`Draw tool0: live (mocap + FK) vs target (Rhino)`** draws each arm's tool0 twice in the 3D
  view, joined by a black line: `<side> tool0 target (Rhino)` (long axes, the loaded movement's
  exported target) and `<side> tool0 live (mocap + FK)` (short axes, the live arm joints on the
  mocap base). It logs the gap:
  `[tool0 check] B20_J_M3_CDFM_transfer_to_approach: tool0 live (mocap base + FK) minus target (Rhino): left ... mm (dx/dy/dz ...), axes x/y/z ... deg | right ...`.
  Before the transfer runs the gap is large (the arms are not there yet); click it again after the
  transfer to see what is left. A movement without target poses only warns.
- **`Visual diagnosis (see-through robot, insert/retreat tool0 frames + path)`** (tick box under
  the button above): the grey live robot and its tools turn see-through. Every insert / retreat
  plan (`Plan Movement`, and the retreat's replan during exec) then draws, per arm:
  - `<side> Rhino start` / `<side> Rhino target` (long axes, the exported tool0 frames; the start
    is the previous movement's target);
  - `<side> plan start` / `<side> plan end` (short axes, the plan's first and last waypoint; for
    the retreat the start is the live tool0);
  - a black line from each Rhino frame to its plan frame;
  - the waypoint path IK follows (green line with small axes; red from the failing waypoint on
    when the plan failed).

  It logs `[B20_R_M1_LM_retreat viz] left: N waypoints over 50.0 mm; plan start is d mm / a deg from Rhino start; plan end is ... from Rhino target.`
  Ticking it after a plan draws that last plan; unticking removes the drawing and brings the robot
  back. For the insert only the left Rhino target is used (the right tool follows the bar), so a
  right `plan end` gap means the two exported targets disagree.

### 6.e The servo in detail

- Each pass logs `[servo N] tool0 pos L=... R=... mm` (Cindy) or `[servo N] tool0 pos X mm, rot Y deg`
  (Alice, Belle).
- The `Servoing Live Tracker` window opens by itself when the servo starts. It plots per iteration
  the tool0 position and rotation error per arm and the base drift, as in the bar-holding accuracy
  test. `Toggle Servoing Tracker` hides / shows it.
- Alice / Belle: each move is a joint move from the live arm to the exact IK solution at the live
  base, nearest the live arm.
- At the end: the record `... <movement>_servoing_data_<ts>.json` / `_performance_<ts>.png` under
  `bar_action_live_replan_exe/<date>-servoing/` on the drive, and
  `[Servo] propagated arm configuration -> '<step>'.start_state.robot_configuration + robot_base_frame (live mocap).`
  for the steps around it. For Cindy the corrected arms, on the base where mocap sees it now, are
  the transfer's end and the insert's start.
- `servo skipped (FAKE_HARDWARE ...)` / `(the base is not tracked by mocap)` (Alice, Belle:
  `[approach ...] servo skipped (FAKE_HARDWARE / the base is not tracked by mocap)`) = no
  correction (7.b).
- The box `Servo feedback: MoCap bar markers (unticked = tool0)` follows the loaded step (since
  2026-10-27, `Load Movement` sets it every time): **ticked and live** when the action's bar carries
  MoCap markers (every ground bar first of all: its MoCap feedback is what places the structure),
  **greyed and unticked** when it carries none (the servo then aims tool0, the flanges, and records
  nothing of the bar). Unticking it for a bar with markers prints a WARN
  (`'Servo feedback: MoCap bar markers' UNTICKED for bar_B21, which has markers: the servo will aim tool0 ...`;
  for a ground bar: `... a GROUND bar, the foundation of every bar after it ... Untick only on purpose.`).
  Unticked = tool0 by itself, nothing else to set; the servo's own `[servo] feedback: tool0 (...)`
  line says so at every run.

### 6.f The spiral search (insert of a bar that tightens on BOTH arms)

The box `Insert compliant part: SPIRAL search (unticked = straight line; radius 0->15 mm, bar-centric)`
follows the loaded step (since 2026-10-27, `Load Movement` sets it every time):
- an insert whose bar mates on **both** arms (B22, B36, B37...): **ticked and live** (the default;
  untick it for a straight-line run, the next `Load Movement` ticks it again);
- a **ground** bar (B20, B34) or a **one-mate** bar (B21, B35: only one male has a female to
  screw into, the other rests on a staging bar): **greyed and unticked**. The insert runs the
  straight line whatever the box shows; with one joint correcting the spiral turned the bar about
  that joint and skewed it (B21, B35 on 2026-10-23). The start line reads
  `insert spiral search: n/a (box greyed: ...)`.
With the box ticked the compliant part circles the bar around the planned insertion line instead
of driving straight along it. To see the motion first without a bar: the spiral test, 4.2.

A one-mate bar's insert also **ends per arm** (since 2026-10-27): when the mating screw stalls,
the mating arm's motor is stopped and that arm goes stiff at once
(`L joint motor STALLED at 2.4 of 1.7 s of the compliant part; L back under joint control, R has streamed its whole line, it settles at its end.`),
while the holding arm keeps its end target under compliance until its tool has stopped moving
(`R settled 0.6 s after its last target (moved 0.21 mm in the last 0.5 s).`; a WARN
`R NOT settled 3 s after its last target, still moving X mm per 0.5 s; held anyway` means the
spring never came to rest), then goes stiff too. The force plot of the holding arm should flatten
before its switch; the bar should not be skewed afterwards.
- One sideways offset goes to both arms (the grasp is kept, the bar only translates), its radius
  growing from 0 at the handover to 15 mm at the assembled depth over `Insert spiral: duration (s)`
  (default 20) with `Insert spiral: turns` revolutions (default 4, i.e. 3.75 mm per turn, under the
  joint's ~5 mm catch range). The rigid approach and the split slider are unchanged.
- Log: `SPIRAL search for the compliant part: radius 0 -> 15 mm over 20.0s and 4 turns ...`, then
  on success `L joint motor STALLED at spiral 63% (12.6/20.0 s, radius 9.4 mm, 2.5 of 4 turns).`
  Note those numbers for the comparison; the `Compliant exec done ... [spiral mode]` /
  `[linear mode]` line names the mode of each run.
- Spiral complete with **no** stall: WARN `spiral complete ... HOLDING the last spiral pose under compliance`.
  The arms hold there (still compliant, screw still tightening) until you click `Confirm Exec`
  (**Expect:** `returning from the spiral end to the nominal end pose over 2.0s`, then the usual hold for the
  stall with the 30 s ceiling) or `Cancel Exec` (ends the insert there: motors stopped, joint
  controller back).
- The `arms did NOT reach the commanded end pose [spiral mode]` warning is expected after a
  mid-spiral stall: it is measured against the straight-line end, so it reads the spiral's radius.

### 6.g Manual mount and tool grasp with the workaround box **unticked** (not used in this test)

- **Manual mount** (`<bar>_J_M1_manual_mount_bar`): `Step` → 1 → `Load Movement` (the bar is drawn
  in the tools at the insertion start) → `Operator done (manual step) -> then Confirm Exec` → mount
  the bar by hand in both tools → `Confirm Exec`. **Expect:** `confirmed done by the operator`.
- **Tool grasp** (`<bar>_J_M2_tool_grasp_bar`): `Step` → 2 → `Load Movement` → `Tool step: grasp` →
  `Confirm Exec`. **Expect:** `STOP`, gripper motors tighten,
  `every gripper motor STALLED after N s: bar clamped`, `STOP sent`.

### 6.h The robot checks by hand

What `check_robots.sh` (Part 3) does, one robot at a time. The `( )` keep the domain inside the
block, so pane 5 keeps its own. Source the **workspace**, not only `/opt/ros/humble`: Cindy's tool
status topics use `crl_husky_msgs/msg/ScaffoldingToolStatus`, which only the workspace has (pane 5
sources it when `open_terminals.sh` opens it). Without it `ros2 topic hz` cannot read them
(`The message type 'crl_husky_msgs/msg/ScaffoldingToolStatus' is invalid`) even when the tools
publish fine.

Cindy (expect joint states ≈ 50 Hz, tool status ≈ 20 Hz, both arms
`scaled_joint_trajectory_controller` active / `cartesian_compliance_controller` inactive):

**▶ pane 5 · CHECKS**
```bash
( export ROS_DOMAIN_ID=86
  ros2 daemon stop >/dev/null 2>&1
  for t in /a200_0806/left_ur5e/rate_limiter/joint_states /a200_0806/right_ur5e/rate_limiter/joint_states \
           /a200_0806/left_gripper/tool_status /a200_0806/right_gripper/tool_status; do
    # the last rate reading (the first one can be low while the subscriber starts), or the error
    echo "--- $t"; r=$(timeout 6 ros2 topic hz $t --window 20 2>&1 | grep -E 'average rate|invalid' | tail -1)
    echo "${r:-NO MESSAGES}"
  done
  for a in left right; do
    echo "--- controllers $a"
    timeout 15 ros2 service call /a200_0806/${a}_ur5e/controller_manager/list_controllers \
        controller_manager_msgs/srv/ListControllers 2>&1 | tr ',' '\n' \
        | grep -oE "name='[a-z_]+'|state='[a-z]+'" | paste - - | grep -E 'scaled_joint|cartesian_compliance'
  done )
```

Alice or Belle (one arm; the gripper action is checked through its hidden status topic, as
`ros2 action list` has no `--no-daemon`):

**▶ pane 5 · CHECKS**
```bash
( export ROS_DOMAIN_ID=84; NS=/a200_0804     # Alice; Belle: 85, /a200_0805
  ros2 daemon stop >/dev/null 2>&1
  echo "--- joint states"; r=$(timeout 6 ros2 topic hz $NS/ur5e/rate_limiter/joint_states --window 20 2>&1 | grep 'average rate' | tail -1); echo "${r:-NO MESSAGES}"
  echo "--- gripper action"; ros2 topic list --no-daemon --spin-time 4 --include-hidden-topics | grep "gripper_cmd/_action/status" || echo "NO GRIPPER ACTION"
  echo "--- controllers"
  timeout 15 ros2 service call $NS/ur5e/controller_manager/list_controllers \
      controller_manager_msgs/srv/ListControllers 2>&1 | tr ',' '\n' \
      | grep -oE "name='[a-z_]+'|state='[a-z]+'" | paste - - | grep -E 'scaled_joint|cartesian_compliance' )
```

Any arm's pendant, through its dashboard (changes nothing). **Expect:** `state='PLAYING'`
`program_name='ros_control.urp'`, `answer='Robotmode: RUNNING'`, `remote_control=True`:

**▶ pane 5 · CHECKS**
```bash
( export ROS_DOMAIN_ID=85; A=/a200_0805/ur5e   # Belle; Alice: 84, /a200_0804/ur5e; Cindy: 86, /a200_0806/left_ur5e or /a200_0806/right_ur5e
  timeout 8 ros2 service call $A/dashboard_client/program_state ur_dashboard_msgs/srv/GetProgramState | grep -oE "state='[A-Z_]+'|program_name='[^']*'"
  timeout 8 ros2 service call $A/dashboard_client/get_robot_mode ur_dashboard_msgs/srv/GetRobotMode | grep -oE "answer='[^']*'"
  timeout 8 ros2 service call $A/dashboard_client/is_in_remote_control ur_dashboard_msgs/srv/IsInRemoteControl | grep -oE "remote_control=[A-Za-z]+" )
```

- `remote_control=False`: the pendant is in Local mode; switch it to Remote.
- `program_name='external_control.urp'` with `state='STOPPED'` and `remote_control=True`: the
  safety sync should load `ros_control.urp` within a few seconds. If it does not, restart that
  robot's launch.

A base (expect `Publisher count: 0` — nobody drives it now — and `Subscription count: 1` — its
`twist_mux` listens, so this pane sees the robot):

**▶ pane 5 · CHECKS**
```bash
ROS_DOMAIN_ID=86 ros2 topic info /a200_0806/cmd_vel --no-daemon --spin-time 5   # Cindy; Alice: 84, /a200_0804; Belle: 85, /a200_0805
```

### 6.i tmux and the terminals

**Why tmux** (instead of only Ghostty's own splits, `Ctrl-Shift-O` right, `Ctrl-Shift-E` down,
`Ctrl-Alt-arrow` to move, which also work for a person alone):
- a session lives on its own: the shells in it (the robot launches!) keep running when the
  Ghostty window closes or an ssh connection drops, and any other terminal can attach to it;
- the monitor log of pane 1 (`tmux pipe-pane`) needs tmux;
- one script builds the titled panes;
- a coding agent (Claude Code) and you can share the session: it reads a pane's output
  (`tmux capture-pane`) and types into it (`tmux send-keys`) while you watch the same pane.

Back into the session from any terminal: `tmux new -A -s husky_real` (`-A`: attach if it already
exists; `tmux ls` lists the sessions). **Expect:** a **green status bar** along the bottom
(`[husky_real] 0:bash*`), the panes as you left them.

tmux keys: press `Ctrl-b` (the "prefix"), let go, then the key.

| keys | does |
|---|---|
| `Ctrl-b %` | split **left / right** (the new pane opens on the right) |
| `Ctrl-b "` | split **up / down** (the new pane opens below) |
| `Ctrl-b` arrow, or `Ctrl-b o` | go to the pane in that direction / the next pane |
| `Ctrl-b z` | zoom the current pane to the whole window, and back |
| `Ctrl-b x` | close the current pane (answer `y`) |
| `Ctrl-b c` / `Ctrl-b n` | new window / next window |
| `Ctrl-b d` | detach: the session keeps running in the background |

The mouse works in tmux on this PC (`~/.tmux.conf`):
- The wheel scrolls back through a pane's output (`q` returns to the live end); a click picks a
  pane.
- A drag selects text (yellow); **releasing the mouse copies it** to the system clipboard and keeps
  it highlighted. `Ctrl-C`, `Enter` or `y` on the selection copy it too and leave the selection;
  `q` / `Esc` leave without copying.
- Paste anywhere with `Ctrl-Shift-V`. Hold **Shift** while dragging for the terminal's own
  selection (copy it with `Ctrl-Shift-C`).

Without the script, log pane 1 by hand (once per session; it keeps logging across monitor runs):
`mkdir -p ~/husky_real && tmux pipe-pane -t <pane> -o 'cat >> ~/husky_real/monitor.log'`.

### 6.j What the monitor start blocks do not set

- No data-path exports: the package's built-in `DESIGN_DATA_DIRECTORY` / `EXPERIMENT_DATA_DIRECTORY`
  are this PC's `~/Insync/2025-03 Husky Assembly/...` folders. Only the problem name is set (the
  built-in default may be another problem).
- No `HUSKY_IK_BACKEND`: ssik is installed in this venv, so the default (ssik) is used.

### 6.k How the runs hand over

- After every step that runs, the robot's real state (measured arm joints, mocap base) is saved to
  `progress.json` (log `[Plan skeleton] <robot>'s live state after <step> saved for the next actions ...`),
  and the next monitor run draws and plans around it, also without `Mark action done`.
- A hold is recorded once the gripper close (the hold's last step) has run; a release once the
  release's retreat has run.
- `Mark action done` still marks the action done and saves its planned trajectories.
- Within an action, each step's real end becomes the next step's start
  (`[Exec] propagated arm configuration -> ...`).

---

## 7. If something goes wrong

### 7.a Starting over / redoing an action

- **Redo one action:** load it → `Reopen action (reload clean)` (pending again, clean export loaded,
  the robot's previous end state restored). Reopen a support robot's action (3, 10, 16, 17) from
  that robot's monitor.
- **Start the whole test over:** `Reset plan skeleton to the Rhino export` → `Confirm Exec`. The log
  shows `[Plan skeleton] reset to the Rhino export: moved N file(s) (...) to .../archive/<time>.`;
  every action is pending, action 0 loaded clean.
  - Nothing is deleted: the old `progress.json` and saved plans are in
    `~/Insync/2025-03 Husky Assembly/data_design_study/261023_2ModulesDemo/archive/<time>/`.
  - A monitor of another robot that is running keeps its old progress: restart it.
- **A support robot must let go early:** run its release action (16 / 17) from its monitor (gripper
  open, then the retreat), and note why. Do not mark the later actions.

### 7.b Problems and what to do

| situation | do |
|---|---|
| the monitor drives an unexpected robot | pane 1's last `export ROS_DOMAIN_ID` decided it (5.1). `Ctrl-C`, paste the right robot's block, `ros2 daemon stop`, `ros2 run`; or `Switch to <robot>` |
| `'Switch to <robot>' refused: a step / execution is still running ...` / `... an arm is still moving` | finish the step or `Cancel Exec` / wait until the arm stops, then click again |
| a step waits and you want out | `Cancel Exec` |
| "refused: a step / execution is still running" | finish it or `Cancel Exec`, then retry |
| a robot's launch pane keeps printing `wrong program 'external_control.urp', but 'ros_control.urp' should be running` and `... Command is not allowed due to safety reasons, please switch robot to Remote Control mode and reconnect to port 29999` | that pendant is in **Local** mode (Belle on 2026-10-16): switch it to Remote (2.a). Its controllers were `inactive`; the joint controller should turn active by itself once `ros_control.urp` plays: check again (Part 3) |
| a check `FAIL` | the line under it says what to do; more in the Part 3 table |
| arm may be left compliant (loud error in the log) | `Switch to Joint (BOTH)` or the service call in 6.b |
| `Exec` refused: arms not at the trajectory start | `Move Arms to Movement Start (offline target)` (refuses joint changes above pi/3), then `Exec` |
| the transfer plans with the bar held / tries the bar-held planner | the workaround box is unticked (it starts unticked after every restart): tick it, plan again |
| `[transfer workaround] ... no collision-free IK for the insertion start` (IK check or plan) | the base is too far from / too close to the Rhino pose, or the drawn colliding pair is in the way (a support robot?): move the base (6.a), IK check again; note it in the log |
| the IK check passes although the base is far from the Rhino pose; it prints `[M1 manual] ...` lines | the workaround box is unticked (the `Transfer start:` sliders are showing): the button then checks the bar-loading start next to the robot, not the insertion start. Tick it and check again |
| `[free validation] ... SWEPT COLLISION ...` / `plan REJECTED: it sweeps through the scene between waypoints` | the random path grazed something between two waypoints (B36: the floor, `ground_WG0`): `Plan Movement` again |
| `CC.6 between tool 'AT3L'` (or `'AT3R'`) `and tool 'ObstacleRobotBelle'` (or `Alice`) in a collision report | Cindy's scaffolding tool would hit that robot. Since 2026-10-15 every plan, IK and swept check rejects this. Before, the tools were never checked against the other robots (only Cindy's arm links were), so a plan could pass a tool through Belle. Plan again; at the insertion start itself, move the base (6.a) |
| a plan fails with "start ... in collision" | read the pair in the log. A support robot's run with `ObstacleRobotCindy`, or the support robot against a built bar: tick `Ignore built-bar collisions (bars drawn faint)` (it reloads the action; the built bars are then ignored, Rhino note D5), plan again. Cindy's run with `ObstacleRobotAlice` / `ObstacleRobotBelle` in the pair: note it (the stored hold pose overlaps the scene); check the robot is drawn where it really stands |
| `not planning: joint value(s) outside the URDF limits` (right away, no search) | the start or goal has a joint the arm cannot reach (the UR5e elbow turns only +/- pi); the message names it. Should not happen for the transfer any more (its goal keeps every joint in range): note the line in the log |
| `Plan for '..._LM_retreat'` / `'..._LM_insert' ... FAILED` (a straight-line plan) | read the `[<movement> diag]` block after it (7.c). A START collision the ACM does not allow = an export gap or the arms are not where the export expects: note the pair in the log. The retreat's replan during exec prints the same block, then runs the planned tail instead (its first step jumps; no pause) |
| the retreat's rigid part ran with `N check(s) FAILED` | the checks only report (no pause); note the `check FAIL` lines (7.c). Remote e-stop if the arms pull on the bar |
| `[<bar>_R_M1_LM_retreat] the full 50 mm retreat has no collision-free straight line (see the diag above); planned a SHORTER one: 40 mm back.` (Plan Movement) | expected when the room is tight (B35 on 2026-10-23: Cindy's left forearm reached her chassis at 45 mm): the retreat tries 45, 40 ... 25 mm (`RETREAT_SHORTEN_STEP_MM`, `RETREAT_MIN_DISTANCE_MM`; 25 mm still leaves the tools 10 mm clear of the 15 mm joint) and the free move home starts from there. `no straight retreat of 25-50 mm either: nothing planned.`: drive the base (6.a) or free the room, plan again |
| the spiral box is greyed on the insert you loaded | the bar is a ground bar or mates on one arm only: the insert runs the straight line on purpose (6.f); nothing to do |
| the servo box is greyed | the action's bar carries no MoCap markers (`[mocap bars] N instrumented bar(s): ...` at the load): the servo aims tool0 (6.e); nothing to do |
| `'Servo feedback: MoCap bar markers' UNTICKED for <bar> ... GROUND bar ...` (WARN) | you unticked the servo box on a ground bar: tick it again unless the untick was on purpose |
| `plan_free_dual_arm failed: birrt_failed` (both ends collision-free, no path found) | read the `[... diag]` lines after it (7.c). A tight pocket at the GOAL: drive the base to give the arms room, IK check, plan again. Otherwise plan again (action 15's free move home: expected on the first try) |
| `Plan end of '..._transfer_to_approach' differs from existing '..._LM_insert'.start by max 6.2832 rad/m` | one joint a full turn from the export's: same tool poses, harmless |
| `Cancel Exec` at the transfer's mount pause | no correction; the insert starts where the transfer ended (re-running the transfer is refused: the arms stand at its end, not its start). Mount and clamp the bar, then plan and run the insert as usual |
| `servo skipped (FAKE_HARDWARE)` / `(the base is not tracked by mocap)` after the transfer or a free approach | `FAKE_HARDWARE = 1`, or `USE_MOCAP = 0` / `USE_CELL_STATE_BASE_POSE = 1` (1.a): the servo needs the real arms and the mocap base |
| `Servoing: live-base IK failed; stopping.` / `[transfer plan] the straight-line bar move could not be solved ...` | the corrected pose is not reachable (or collides) from where the base stands; with the sampling fallback check the `SAFEGUARD` figures before confirming. The insert then starts where the servo left the arms |
| Alice / Belle: `no collision-free IK for its approach pose at the live base` | drive the robot (6.a, its domain) towards its hold spot, plan again |
| Belle: `ssik solver 'alice' does not fit the planner's robot ...` | expected: Alice's solver is tried first, then `'belle' fits` |
| Alice / Belle: `no ssik solver (alice, belle) fits the planner's robot: PyBullet IK instead` | not expected: the planning cell is not the robot's calibrated model (PyBullet IK, 1 mm only); note it |
| Alice / Belle: `FK tool0 and the arm-reported TCP disagree ...` | no compliant handoff: the gripper closed with the arm stiff. The bar is still clamped; note the numbers. Belle, constant offset along tool Z: her pendant tool setting (2.d) |
| Alice / Belle: `switch to cartesian_compliance_controller FAILED: the gripper was NOT closed` | nothing closed: check the robot's controllers (Part 3; `Switch to Joint (BOTH)` if needed), then run the close step again |
| Belle's arm moves when the handoff goes compliant | e-stop; her controller frame may differ from Alice's (4.1). Note how far and which way, close stiff with `Close Gripper for Bar` instead |
| the jaws not centred on the bar after the free approach | that robot's base calibration is off (1.c): stop before the straight approach; recalibrate it |
| `Base calibration file not found for robot 0805` (Belle) / `0804` (Alice) | the robot's file is not in the folder its `BASE_CALIBRATION_DATE_BY_ROBOT` entry names (1.c), or Insync has not synced it |
| no topics / `Unknown topic` | the robot is not visible: `eno1` on `192.168.0.x` and the robot pings (1.e); the right domain + CycloneDDS in this pane (5.1); the robot's launch still running with its domain + CycloneDDS (Part 2); or the base driver is down |
| Any robot: the launch runs, but this PC sees nothing from her (`ros2 topic list` shows only `/parameter_events` and `/rosout`) | her ROS traffic leaves through her robot-internal port `br0`: relaunch with the `IF=...` / `CYCLONEDDS_URI` lines of 2.b / 2.c / 2.d; the `echo` line must name her lab-network port (`DDS on 'wlp2s0'` for Belle and Cindy), not `''`. Still nothing: the Wi-Fi drops the discovery messages; add this PC as a peer, i.e. end the `CYCLONEDDS_URI` value with `...</Interfaces></General><Discovery><Peers><Peer address="192.168.0.201"/></Peers></Discovery></Domain></CycloneDDS>` (`doc/calibration_manual.md` 2.3). The export lasts only for that ssh shell; nothing changes on the robot |
| the monitor pane does not pick up the robot's topics | `ros2 daemon stop` right before `ros2 run` (5.1) |
| no tool status / grasp steps get no answer | `... ScaffoldingToolStatus' is invalid`: the pane did not source the workspace (6.h), the tools are fine. `NO MESSAGES`: that tool's driver stopped, or Cindy's launch is missing `gripper:=scaffolding_v3` (Part 3 table) |
| `mocap client connected: False` | `CLIENT_IP` is not this PC's `eno1` address (1.b), or the Motive PC is no longer `192.168.0.28` (`doc/calibration_manual.md` 1.4) |
| marked an action done by mistake | load it, `Reopen action (reload clean)` (a support robot's action from its own monitor) |
| progress is wrong beyond repair | `Reset plan skeleton to the Rhino export` (7.a) |

### 7.c Reading the diagnosis blocks

- **A failed straight-line plan** (`[<movement> diag]`) shows:
  - the START's colliding pairs, with the movement's ACM entry of each body (a contact is allowed
    only when listed there);
  - the start's joint limits;
  - the failing waypoint and why its IK candidates were turned down (collide / would flip an arm);
  - a `VERDICT`.

  The red planning robot and the contacts are drawn, plus that waypoint's two tool0 targets
  (`Remove all drawing` clears them).
- **The retreat's `check FAIL` lines:**
  - `grippers` not `STALLED` = a gripper did not open;
  - `contact` = a tool still pushed on the structure;
  - `compliant part` may fail from gravity sag alone (the limits are first guesses);
  - `replan` = the fallback tail ran, whose first step jumps.
- **A failed free plan** (`birrt_failed`) shows: the joint limits of start and goal, how many nearby
  configurations collide at each end (and with what), where the straight joint path first collides
  (drawn; the red robot stands there), and a `VERDICT` line.

---

## 8. What to send back

- The monitor log (`~/husky_real/monitor.log`, written from pane 1; or copy the terminal); it holds
  every monitor run of the day.
- `~/Insync/2025-03 Husky Assembly/data_design_study/261023_2ModulesDemo/progress.json` after
  the run.
- **Per jointing action:**
  - the `[transfer workaround] ... max |goal - stored goal|` line;
  - the `[tool0 check] ... minus target (Rhino)` line from `Draw tool0: ...` after the transfer;
  - whether the mounted bar sat in the tools at the insertion start;
  - how the insert finished (stall time, any rigid / compliant warnings).
- **The spiral test (4.2):** its `[spiral test]` lines and what you saw (4.2 e).
- **Per release action:** the `screw-out t=...: gripper motors L/R ...` lines and the `screw-out
  done after ...` line (what the tool reports when its jaws open), whether the axial force stayed
  flat during the screw-out (stiff default), the `[retreat live] ... deg from the Rhino axis`
  numbers, with the compliant box ticked the `[..._LM_retreat] check ...` lines (which passed, the
  numbers of any FAIL), and whether the arms backed off cleanly without moving the bar (actions
  4, 11: clear of the support robot's gripper).
- **Actions 0 and 7:** how the rigid insert ended (the `grounded bar` lines; the bar's final fit
  before fixing the foundation).
- **Action 15:** how many tries the free move home took.
- **Alice and Belle** (4.1, 5.7, 5.9, 5.11, 5.12):
  - the phase lines and any "no handoff" warning;
  - how the arm behaved while compliant (Belle: did it stay still when going compliant?);
  - the `[live goal]` line and the servo's iterations;
  - how far the open jaws were from the bar's centre after the free approach;
  - whether the held bar moved while it was clamped or while the next bars were screwed in;
  - whether the jaws cleared the bar on the release.
- **The as-built record** (9), when the run used a problem with MoCap markers: the day's
  `as_built_<problem>.jsonl` and the page built from it (9.3).

---

## 9. As-built record and the assembly page  [RECORDED BY ITSELF — read afterwards]

**What it is.** While the plan skeleton runs, the monitor measures every bar that carries MoCap
balls at the **start and the end of every step** it executes, on every robot's monitor, and appends
one line per measurement to a file of the day. Afterwards one script turns that file into a 3D web
page with a slider over the actions. Nothing here moves a robot.

Words used below:

| word | means |
|---|---|
| record | one measurement: every streaming MoCap bar, plus the robot's base, joints and tool0 |
| target | where the bar should be at that moment: its seat if it rests, the authored flange target with the grasp if an arm holds it (never FK) |
| seat | the bar's assembled pose in the design |
| milestone | a moment the page lists first (table in 9.1) |
| bystander | any other bar measured while another bar's action runs: how a placed bar is followed through the rest of the assembly |

### 9.1 What is recorded, and when

| moment | step, phase | milestone | the bar is compared with |
|---|---|---|---|
| before the insert | J transfer, end (after its servo) | `before_insert` | the insert's start (15 mm above the seat for B20) |
| after the insert and the tighten | J insert, end (the tighten runs inside it; B20's second servo too) | `after_insert` | the seat |
| after B20's foundation fix | J manual fix, end | `after_fix` | the seat |
| before / after the robot retreats | R retreat, start / end (the ungrasp runs inside it) | `before_retreat` / `after_retreat` | the seat |
| support robot holds | H approach / move in / gripper close, end | `hold_approach` / `hold_at_grasp` / `hold_grasped` | the seat |
| support robot lets go | HR gripper open / retreat, end | `hold_released` / `hold_retreated` | the seat |
| every other step, start and end | — | none (`bystander` for the other bars) | the seat |

Where the records go (one file per lab day and design problem, every robot's monitor appends to it):

```text
~/Insync/2025-03 Husky Assembly/data_experiment/bar_action_live_replan_exe/<YYYYMMDD>-asbuilt/as_built_<problem>.jsonl
```

What it needs:

- a **`MoCapMarkers.json`**: the design problem's own, else the one of `MOCAP_MARKERS_PROBLEM_NAME`
  (`husky_assembly_teleop/__init__.py`). `261023_2ModulesDemo` (this manual's) has its own: B20, B21,
  B25, B34, B35, B39, B46, B47, B50 (1.d);
- the flag `AS_BUILT_RECORD = 1` in `husky_monitor.py` (the default);
- Motive streaming the bar assets (1000 + the bar number: B20 = 1020, B21 = 1021, B25 = 1025, B34 = 1034,
  B35 = 1035, B39 = 1039, B46 = 1046, B47 = 1047, B50 = 1050) with **Labeled Markers** on, as in
  `doc/mocap_bar_pose_test_manual.md` 0.3.

**?** A record is about 8 KB per bar; a lab day of seven actions is well under 1 MB.

### 9.2 During the run

- [ ] nothing to click: run the steps as in Part 5.
  - **Expect:** after each `Exec ...` and each tool / gripper / manual step, pane 1 prints one line per bar
    and the file:
    `[as-built] B20_J_M3_CDFM_transfer_to_approach end (ok): bar_B20 [before_insert] fit ok 8/8, rms 0.6 mm | vs held_authored:B20_J_M3_CDFM_transfer_to_approach dpos 0.4 mm, dang 0.030 deg`
    `[as-built] B20_J_M3_CDFM_transfer_to_approach end (ok): recorded -> .../<YYYYMMDD>-asbuilt/as_built_261023_2ModulesDemo.jsonl`
    (named after `DESIGN_PROBLEM_NAME`, this manual's problem, even though the markers are borrowed)
  - **Expect:** the cyan MoCap copy of each measured bar is redrawn in PyBullet at that moment.
- [ ] optional: button [As-built: record now] (panel section `MoCap bar (joint markers)`) at a
      moment no step covers, e.g. after the bars stood for a while, or at the end of the session.
  - **Expect:** the same lines, with `manual` instead of `start` / `end`.

**If not:**

| you see | do |
|---|---|
| no `[as-built]` line at all | no `MoCapMarkers.json` was found (look for `[mocap bars] neither this design problem nor ...` at the action load; check `DESIGN_PROBLEM_NAME` and `MOCAP_MARKERS_PROBLEM_NAME`, 9.1), or `AS_BUILT_RECORD = 0` |
| `... no bar measured` | no instrumented bar streams: check the assets in Motive (1020, 1021) |
| `fit FAILED (only 5 of 8 design markers matched (need 6))` | balls hidden by the tools or a gripper; the record is kept with the reason. Free the balls if you can, then `As-built: record now` |
| `[as-built] end of ... NOT recorded: ...` | the drive folder could not be written (Insync not mounted?); the step itself went on |

### 9.3 Building the page

▶ any terminal

```bash
cd ~/ros2_ws && source venv/bin/activate
python src/husky-assembly-teleop/data/bar_holding_acc_data/4_assembly_viewer.py 20261020 --open
```

- **Expect:** `[assembly] N records from .../20261020-asbuilt/as_built_261006_3bar_holding_test.jsonl; problem 261006_3bar_holding_test, 18 actions, 7 bars, K servo files.`
  (seen 2026-10-20: 12 records, 6 servo files)
- **Expect:** `[assembly] 261008_2ModulesDemo_YH.3dm has no 'env' solids; the lab room and the camera rig come from 260929_phase1_retest.3dm (same Motive frame).`
- **Expect:** `[assembly] wrote .../bar_action_live_replan_exe/20261020-asbuilt-result/assembly_20261020.html (1.x MB)`

- **Expect:** `[assembly] tick boxes: N lines of 20261020 in /home/su/husky_real/monitor.log` (the
  page reads which tick boxes each step used from the monitor's log; without the log: `! no monitor
  log at ...` and the tick boxes show as not known)

One page per lab day, rebuilt in place each time the script runs (records written since appear).
Leave out the date for the newest day. Other options: `--problem NAME` (when one day recorded two
problems), `--no-robots`, `--no-cameras`, `--env-3dm PATH`, `--out PATH`, `--runs-dir PATH` (a
headless smoke's scratch folder, for a desk rehearsal), `--monitor-log PATH` (another copy of the
monitor's log).

**Shared copies online** (a Claude artifact: colleagues open the link in their own browser, nothing
to install; private until shared from the page's **Share** menu):

| Lab day | Problem | Link |
|---|---|---|
| 2026-10-20 | 261006_3bar_holding_test (B20) | https://claude.ai/artifact/1UzzufFCprw3AksFuRXEi8 |
| 2026-10-21 | 261006_3bar_holding_test (B20, B21, B22) | https://claude.ai/artifact/UXrEGhpfvqRhAyRpuR5RVD |
| 2026-10-23 | 261023_2ModulesDemo (B20–B37) | https://claude.ai/artifact/FxWwsbhnkZiK4br4BtV9qx |

Step-accuracy reports made from these records (which step changed each bar's error, what the servo
did, which tick boxes each step used): 20–21 Oct https://claude.ai/artifact/8uv8YGUqsaUMoseEnANqL5,
23 Oct https://claude.ai/artifact/FFPBiLaM5Jw6AVAedK7FXz (a copy beside the day's page:
`20261023-asbuilt-result/as_built_report_20261023.html`).

The link does not follow the drive: after a rebuild, ask Claude to republish the page to the same
link (it keeps the URL).

### 9.4 Reading the page

**Moving through the run**

| control | does |
|---|---|
| slider `action` (or ← / →) | the action, in the plan's order. Its label reads the bar first, e.g. `B20 · R (release) · Cindy`, then the plan's action number, how many bars exist and who holds which |
| the pills under it | that action's records, milestones first (`before insert`, `after insert`, ...); `[` / `]` step through them; struck through = a step that was cancelled or stopped short; ↻ = redone. **The pill you pick stays picked**: go to another action and back (e.g. `before retreat` at B20 R, then B20 J, then B20 R again) and it is still the one shown |
| `tick boxes at this step` (under the pills) | the panel's tick boxes that could change the picked record's step, one line each: `✓ used` or `✗ not used`, how the box was set (`ticked` / `unticked`) and why. Transfers: `Transfer as a free move (workaround)` and `Servo feedback: MoCap bar markers` (used only when the servo really aimed the bar: e.g. `unticked ... bar_B37 carries no MoCap markers`); inserts: `Insert compliant part: SPIRAL search` (a grounded bar's insert is rigid, so the box never acts there) and, for a grounded bar, the servo after the insert; a support robot's servo aims tool0 only. A box ticked at a step it cannot act on is listed as not used (the monitor still prints `[Exec] ... SPIRAL search (box ticked)` before a retreat). Read from `~/husky_real/monitor.log` |
| `all records` | also lists the step starts and the free moves |
| the strip at the bottom | one cell per action in the same order, coloured by the placement error of the action's own bar at its last step that ended well (grey: recorded, but that bar has no MoCap markers or no fit; pale: not executed); click a cell to jump there. `strip by` → `worst bar of each action` colours by the worst bar in the scene instead, and `<bar>` shows one bar's error against its seat through the whole run |
| left click a bar or a ball | its card; Esc or × closes it |

**What the drawing means**

| drawn | is |
|---|---|
| thin grey line | a placed bar at its seat (authored), one continuous line from its start to its end |
| thin blue line | the action's own bar; at its step target when that is not the seat (the seat then faint) |
| thin orange / violet / light blue line | a bar held by Alice / Belle / Cindy |
| see-through, a little thicker, shaded along its length | a bar as MoCap measured it, coloured by its distance from where it should be at each point. Both kinds are drawn thin (measured 5.5 mm across, authored 4 mm) so the few millimetres between them show |
| filled balls | the balls Motive saw: red = stray (matched no design ball), grey = dropped before the fit (hidden, wrong size) |
| hollow balls | where the design balls should be (the same size as the filled ones, the real ball's); red = no ball matched it |
| robots | the one whose monitor recorded, as measured; the others as last saved in `progress.json` (`(saved)`, paler) |

**What the `data by` picker draws on the bars.** Both pickers list the measurements in the card's order
(placement, rotation, tool0, servo, plate length, spacing, bending, fit residual, load). With no bar clicked, on every measured bar; once a
bar is clicked, on that bar only (nothing of another measurement or another bar on top of it). Every
arrow is drawn **20× longer** than the error it shows (a millimetre on a 2 m bar cannot be seen); the
numbers beside them are the true values. The bar's name label then shows only its name.

| `data by` | drawn |
|---|---|
| placement error | a small ball at the start, middle and end of the authored bar (dark) and of the measured one (coloured by its error), an arrow from each authored ball towards its measured one, and the distance there (`2.54 mm`). None when the step has no target for the bar (e.g. before the mount) |
| rotation error | at the bar's start, thin axes: the authored bar's x (red) / y (green) / z (blue, along the bar) faint, the measured bar's solid; each measured axis with its letter `X` / `Y` / `Z` at its tip and, just above the letter, the turn **about** it: `about Z: 1.79°` (the bar's roll; the Z labels sit just above the bar), `about Y: -0.02°`, `about X: -0.02°`; around each axis near its tip, a small **purple** arc sweeping that turn (20×; none when too small to see); and `centre lines 0.027° apart` under the z axis. Against the step target, else the seat. The start is not an anchor: a rigid bar's turn is the same wherever it is measured; the axes sit there because the bar's own frame does |
| bending | every plate named (`J20-22`, `J20-21`, ...); at each inner plate a red arrow the way the bar bows there, and `J20-21 · bow 1.51 mm` |
| plate length error | at each plate, its name and how far its two balls' distance is off the design (`J20-25 · balls off -0.68 mm`) |
| spacing | a small ball at each plate on the bar, and between neighbouring plates their distance, measured / design (off by): `J20-22 → J20-21 · 333.7 / 335.1 (off -1.37) mm` |
| fit residual | at each plate, its name and its two balls' distances from their design spots after the fit, M1 / M2 (`J20-22 · 2.46 / 2.04 mm`) |
| tool0 vs target | at each flange (where FK puts it), that arm's tool0 against the step's target (`left · tool0 vs target 0.11 mm`, `right · ... 2.59 mm`) |
| servo residual | at each flange, that arm's servo residual, the servo's last iteration (`left · servo residual 0.13 mm`); `no servo in this step` on records without one (only the end of a transfer and of a grounded bar's insert have one) |
| load imbalance | at each flange, that wrist's force (`left · 26.51 N`, `right · 24.67 N`), and between the two the gap (`gap 1.83 N`) |
| any other | its number on the bar's name label |

Plate labels alternate below and above the bar, so neighbouring plates' labels never sit side by side.

**The card**

| block | answers |
|---|---|
| header | line 1: the step and the robot, `B20_R_M1_LM_retreat / Cindy (17:29:48)`; line 2: when the record was taken (`at the start of the step` / `at the end of the step` / `recorded by a click`) and the moment of the bar's life (`after the retreat`, ...). A step that did not end well gets a red line under it |
| placement error | the MoCap bar vs where it should be (the step's target), at the start, the middle and the end, the largest in bold |
| rotation error | the angle between the centre lines, then the turn split into a tilt about the bar's x and y and a roll about its own axis z (the colours of the axes in the view with `data by: rotation error`), and the offset at the bar's start. One value for the whole bar: it is rigid, so it turns as one piece, the angle is the same wherever it is measured, and there is no anchor and no start / middle / end |
| robot: tool0 vs target, servo residual | each flange by FK against the step's target (`tool0 vs target, left / right`); the servo's last iteration (`servo residual, left / right`, on the end records of a transfer and of a grounded bar's insert only); then **where the error comes from**: `FK bar vs target` is what the robot can see (its servo drives it to ~0.2 mm), `MoCap vs target` the total, `MoCap vs FK` what the robot cannot see (base calibration, arm, TCP, the bar in the jaws). `data by: tool0 vs target` or `servo residual` marks this block |
| bar shape & fit | from its balls: is a ball off, is the bar bent, does the fit hold? In mm, as `measured / design (off by)`. A **plate** is one T20 MoCap joint's plate with its two balls, 48.3 mm apart by design: one row per plate (`J20-25  47.6 / 48.3 (off -0.68)`); one row per pair of neighbouring plates, their distance (`J20-22 → J20-21  333.7 / 335.1 (off -1.37)`); the **bow** at every inner plate, the largest in bold (`J20-21: 1.51, J20-169: 0.69`); **Motive noise this day** (how much a plate's ball distance wanders between records); then the fit: balls found, the typical mismatch (rms: the root-mean-square distance between each seen ball and its design spot after the best rigid fit), the worst ball, hidden balls, extra balls near the bar. Ends with the **verdict** (below) |
| load | Cindy's two wrist forces, left and right, then their gap (\| left − right \|, the load imbalance), and what they are relative to: `force sensors zeroed at <time> at the mount pause (tools empty)` (records since 2026-10-21) |
| history of this bar | every record since the bar was placed (the ones before, on its loading spot or standing by, are counted, not listed): its distance from the seat at start / middle / end, its bow, and after the robot let go, **how far its ends moved since**: the answer to "did the bar move or deform during the rest of the assembly" |

**The verdict** (`bar shape and fit`) tells the four causes apart, using the whole day's records of
the bar. Thresholds at the top of `4_assembly_viewer.py`, set from the 2026-10-20 run (section 10):

| the verdict says | because | what it means |
|---|---|---|
| `bad ball on plate X: its two balls are 47.6 mm apart, not 48.3` | a plate's ball distance is off by more than 0.6 mm (bending cannot change it) | a ball loose, half hidden or mislabelled; `in every record this day` = mounted off its spot or a wrong table entry |
| `bending at plate X: its bow is ... away from its usual` | an inner plate's bow moved more than 0.5 mm from its day average | the bar bends under a load there (or a ball moved) |
| `plates off the design table by the same amount in every record` | a plate sits more than 0.5 mm off where the fit puts it, and moves less than 0.5 mm over the day | the plates are mounted off their table spots: a constant offset, not bending |
| `typical ball mismatch ... mostly the constant plate offsets` / `the fit itself is doubtful` | rms over 1.0 mm, explained or not by the line above; or another assignment of the balls fits as well | trust the placement numbers less than the rms |
| `balls good, bar straight, fit tight` | none of the above | |

The day lines need at least 3 records of the bar.

**!** The page can say how big the robot-blind part is, not which part of the robot causes it: that
needs many bars compared in the room, base and flange frames (the `3_session_report.py` method),
which is left for when a run has enough of them. Every record keeps what that needs (FK, targets,
base, joints, wrench).

### 9.5 Rehearsal with B20 and B21

The as-built record needs no extra step; this is the order that exercises every milestone with the
two instrumented bars (this manual's problem then, `261006_3bar_holding_test`, markers borrowed from
`261016_2ModulesDemo_YH`, 9.1; run 1 done 2026-10-20):

| run | robot | actions | the page then shows |
|---|---|---|---|
| 1 | Cindy | 0 `B20_J`, 1 `B20_R` | B20 `before_insert`, `after_insert`, `after_fix`, `before_retreat`, `after_retreat` |
| 2 | Cindy | 2 `B21_J` | B21 before / after its insert, B20 as a bystander in the same records |
| 3 | Alice | 3 `B21_H` | Alice's records: approach, at grasp, grasped (no wrist force on a support robot) |
| 4 | Cindy | 4 `B21_R`, 5 `B22_J`, 6 `B22_R` | B21 released by Cindy while Alice holds it; at actions 5–6 B20 placed, B21 held by Alice, B22 being built (no markers: dashed only); B20's history says whether it moved |

### 9.6 Things to know

- The servo after a transfer is saved under the **insert's** name (`B20_J_M4_LM_insert_servoing_data_...`);
  the page matches servo files by time and says so in the card.
- Only B20 and B21 carry MoCap joints in this export; B34 and B35 stream from Motive but cannot be
  measured until their T20_MoCap joints are added in Rhino.
- The other robots are drawn where `progress.json` last saved them, not measured.
- The room and the camera rig come from `260929_phase1_retest.3dm`, placed only roughly (the
  placement TODO in `doc/bar_holding_acc_manual.md`).
- The start record of a J action's first step measures the bar wherever it lies before mounting;
  the page leaves it out of the bar's history (counted under the chart).
- The file is JSON **lines** (`.jsonl`), one record per line, not one JSON document: every monitor
  only appends a line, never rewrites the file, so three monitors can share it, and a crash or a
  drive sync mid-write costs at most the last line (`read_as_built` drops a cut-off last line).
  `progress.json` is different: a small **state** file (about 8 KB) that a monitor rewrites whole
  from its own memory each time (write a temporary file, then replace), because only the latest
  state matters; the as-built file is a **log**, where every record must be kept.
- The wrist forces in a record are read against the sensors' last zero, which the record names
  (`load` block). Where the zeros happen:
  - **the mount pause** (5.2 b), tools empty: from then on the J records read the bar's own load
    (since 2026-10-21; before that nothing zeroed in a J action with the workaround);
  - **the retreat's start**, with the bar still gripped, **until 2026-10-21** only: those
    retreats' end records read **minus** the load the bar put on the arms. Since 2026-10-22 the
    retreat does not zero, so its records read against the mount-pause zero too;
  - the panel button `Zero Force Sensor (BOTH)`.
  On 2026-10-20 the bar's own load at the end, just before the release, was about 25 N along the
  bar at each arm, equal and opposite (the mount's preload), and about 10 N down in total (its
  weight); section 10.2.
- How the recorder works, for whoever changes it: `_exec_then_propagate` in `husky_monitor.py`
  (the one function every executed step runs through) calls `_as_built_record` before and after
  the step; the file format is `husky_assembly_teleop/as_built_io.py`.

## 10. Notes from the 2026-10-20 run (temporary)

> **?** Temporary: what the first real as-built run showed (B20 on Cindy, actions 0 `B20_J` and
> 1 `B20_R`, problem `261006_3bar_holding_test`). Fold into section 9 or drop once the next run
> confirms or contradicts it.

Files: `20261020-asbuilt/as_built_261006_3bar_holding_test.jsonl` (12 records) and the servo files
`20261020-servoing/B20_J_M4_LM_insert[_end]_servoing_data_20261020_17*.json`. The first try (monitor
run 16:37) measured no bar (B20's markers were not measured; its records carry the robot only); the
numbers below are from the second (17:07).

### 10.1 Step by step

| step | the bar vs where it should be | what the robot did |
|---|---|---|
| transfer + mount + servo (17:09 → 17:15) | 5.2 mm, then **2.5 mm** after the first correction; no better after it | left flange 0.1 mm from its target, right flange 1.2 mm; all 8 iterations ran (6.e: the right flange stays 1–3 mm off, the bar as mounted by hand) |
| insert (17:21 → 17:25) | 2.5 mm, 1.7° | the bar went 15 mm down as planned; the vertical force on the arms changed by less than 1 N: the floor takes **none** of the bar's weight, nothing pushes into it |
| second servo (inside the insert, 5.2 d) | 2.65 → 2.50 mm | aims tool0, which cannot see the bar's own error (by FK the bar is 0.3 mm off); 8 iterations |
| fix foundation (17:27:43 → 17:27:57, **14 s**) | 1.7–2.5 mm (left end / right end) | the balls moved less than 0.1 mm |
| release (retreat, 17:29) | start 1.0 / middle 8.3 / **end 16.4 mm** | the bar **turned about its right end**: the left-end balls moved 13.5 mm sideways, the right-end ones 1.5 mm |

- While held, the error stays at about 2.5 mm from the transfer to the retreat, almost all of it a
  1.8° roll about the bar's own axis. It does not change with the motion, so it sits in the grasp,
  the marker table or the calibration; only a reference measurement can tell which (10.3 shows the
  marker table is about 1 mm off on its own).
- **!** The release swing: either the 14 s fix step was a click-through (a rehearsal) or the fix
  does not stop the bar turning in the horizontal plane. If B20 really was fixed, look at that first.

### 10.2 The wrist forces

- In that run nothing zeroed the sensors in the J action (the workaround skips the travel to load,
  whose end zeroes them); the last zero was an empty-tool one (0.3 N at the transfer start). Since
  2026-10-21 the mount pause zeroes them (5.2 b).
- Turned from each wrist's frame into the room's, the readings split cleanly: the left + right sum
  points straight down (the bar's weight), the left − right difference lies along the bar, equal
  and opposite at the two arms (the mount's preload: the arms squeeze or pull the bar; which one
  depends on the UR wrench's sign convention, not checked).
- The retreat then zeroed the sensors at its start with the bar still gripped (it no longer does
  since 2026-10-22), so its end record reads minus the bar's load: about **25 N along the bar** at each arm and about **10 N down** in total.
- The preload was larger right after the mount (about 55–60 N at the first servo reading, about
  38 N after that servo) and eased by about 2 N with every servo correction, while the right flange
  drifted 1–3 mm off its target: the closed chain settling.
- **!** The second servo's 20 N force stop therefore watches the preload easing, not floor contact:
  its change reached 12 N with no load on the floor at all (the first servo, which has no stop,
  reached 20.3 N). A stop on the left + right **sum** would react to the floor only. Not changed yet.

### 10.3 What the balls say (worked back from the plates)

A plate's two balls are a fixed 48.3 mm apart, so the plates measure Motive and the marker table
themselves. From the page's `bar shape and fit` block over the day's 7 records:

| the balls show | numbers | what it is |
|---|---|---|
| ball distance on J20-22, J20-21, J20-169 | within ±0.3 mm of 48.3, wandering record to record | **Motive's noise**: about ±0.3 mm per distance |
| ball distance on J20-25 | 47.3–47.6 mm in **every** record | a **ball off its spot** (mounting) or a wrong table entry; not bending (a plate cannot shrink), not the fit (this number never uses the design pose) |
| each plate's place against where the fit puts it | J20-22 1.2, J20-21 1.1, J20-169 0.5, J20-25 0.7 mm, the same in every record | the **plates are mounted off the design table** by about 1 mm |
| distance J20-22 → J20-21 | 333.8 mm against 335.1 in the table (−1.3 mm) | the same offsets, seen as a distance |
| bow at J20-21 | 1.5–1.6 mm (upwards), varies by 0.3 over the day, the same held, placed and released | that mounting offset again: **the bar does not bend** under its load |
| fit rms / worst ball | 1.4–1.6 / 2.1–2.7 mm | mostly these constant offsets, not noise |

So, before reading sub-millimetre placement errors on B20: re-measure its plates (or build an
as-measured marker table from a day's balls), and check the J20-25 balls.

How the bow is computed (`bar_mocap_fit.pair_geometry`): the fit places the design balls on the seen
ones as one rigid bar. For each plate, the **shift** is the mean of (seen ball − fitted design ball)
of its two balls, in the bar's own frame. A straight line is drawn through the shifts of the two end
plates; an inner plate's **bow** is how far its shift sits off that line, across the bar (the part
along the bar is left out). A straight, well-mounted bar reads ~0 (Motive noise). A bow can be a bent
bar **or** a plate mounted off the table: one record cannot tell them apart, the day can (a bow that
never changes is the mounting; the verdict says so).

### 10.4 Open points

- [ ] B20's release swing (10.1): was the bar fixed?
- [ ] The second servo's force stop on the left + right sum instead of each arm (10.2).
- [ ] The servo never converges while the right flange plateaus (re-mount per 6.e, or accept the plateau).
- [ ] The 1.8° roll while held: grasp, marker table or calibration (10.1, 10.3).

## 11. Notes from the 2026-10-23 run (temporary)

> **?** Temporary: what the first three-robot run of `261023_2ModulesDemo` showed (Cindy B20-B37,
> Alice holding B21, Belle holding B35; 78 as-built records, 16:28-19:37). The fixes made on
> 2026-10-27 (11.2) are to be confirmed in the lab; fold the rest into the sections above or drop
> it once the next run confirms or contradicts it.

Files: `20261023-asbuilt/` (records, the day page `assembly_20261023.html`, the report
`as_built_report_20261023.html`, and the frame-drift pages `drift_correction_20261023.html` /
`assembly_20261023_shift_corrected.html`), `20261023-servoing/`, `~/husky_real/monitor.log`.

### 11.1 What went wrong

| what | numbers | status |
|---|---|---|
| **MoCap frame drift**: everything MoCap sees sank together, bars and parked robots alike | about 8 mm down and 2 mm sideways between 16:28 and 19:37, in steps, already going at 14:16 (Alice's parked height: `[floor] /a200_0804: base_footprint ... mm above the measured floor` at every restart). Not the floor file (a fixed file). Partly a tilt about the anchor markers' line (they sit almost on one line along x at y ~ 2.3 m, so a rotation about it is unconstrained: the far module sank 1.2x the near one, the vertical B21 leaned 0.35 mrad) plus a ~1 mm slide along x. Bars placed at different times end up misaimed by the drift in between (B21-B20 1.7 mm, B22-B20 3.4, B36-B34 4.1, the B37 bridge 6.6 / 5.3 mm). | **open.** A weekend test with continuous calibration on and nothing moving drifted far less. Cause unknown (cameras moving? the anchors?). To do: more anchor markers on the floor, spread out (three not in a line, one far from the truss near the floor); a still reference body in every as-built record; the 27 Oct floor file (11.3) was probed after re-anchoring |
| **One-mate bars skewed** (B21, B35) | the spiral turned the bar about its one correcting joint; at the stall both arms went stiff in the same tick, the holding arm frozen mid-settle: free flange 1.3 / 0.6 of 5 mm, top of the bar 8 mm short | **fixed 2026-10-27** (11.2 a) |
| **The retreat pushed every bar** | after the LOOSEN the tools stood within 1 mm of the bar for ~11 s while the grasp screws turned (2 s head start that lasts 3.1 s, a 0.5 s compliant move that went 0.4-0.7 mm, the replan, a 3 s wait for a STALLED that never comes); 44-47 N on the bar; `doc/IMG_5692.MOV` 0:12-0:19 | **fixed 2026-10-27** (11.2 b) |
| **B37's insert never planned** (`B37_J_M5_LM_insert` FAILED 5x, waypoint 1 of 5, right wrist_1 step 0.196 rad) | the right wrist_2 is -3.3 deg at the insertion start (singular; Rhino's plan had +6.4 deg), Cindy parked 21 mm / 3.4 deg off. Not the drift: +-8 mm shifts never pass either | **open**: change B37's right grasp so wrist_2 stays beyond ~15 deg |
| **Belle**: the e-stop activated by itself, repeatedly; the gripper did not open when the terminal asked it to (4 closes with the fingers unmoved logged `holding`) | | **open** (hardware; the stalled-is-not-grasped handoff is a known gap) |
| **Right arm further from its female joint than the left** (seen by hand at every insert) | flange FK vs its Rhino target at the end of the tool0 servo (20/21 Oct): left 0.1-0.2 mm, right 1.2-2.8 mm, always (+x, -y, +z); with the MoCap-bar servo (23 Oct) the right still carries the larger share (B34: 4.0 mm). The right arm has no MoCap base calibration of its own: it is placed through the URDF `right_arm_mount_joint` offset from the Aug 2025 two-arm point-touch calibration (2.2-2.5 mm residual) on top of the LEFT arm's 16 Oct base calibration; the June right-arm calibration (20260625) differs from the left's by ~(1.0, 2.9, 0.6) mm. The record has no male/female joint frames: the male-to-female gap itself needs the BarAction file (grasp + joint frames) reloaded offline | **open**: a right-arm MoCap base calibration after the next left one |

### 11.2 Fixes made

- **a) One-mate insert ends per arm** (6.f; `husky_world._finish_holding_arm`, `_SettleWatch`): no
  spiral for a ground or one-mate bar (box greyed; the executor ignores it anyway); at the mating
  screw's stall that arm's motor is stopped and it goes stiff at once, the holding arm streams
  whatever is left of its line and holds its end target until its tool has stopped moving
  (< 0.5 mm over 0.5 s, 3 s ceiling), then goes stiff. Two-mate bars unchanged, the spiral box
  ticked by default for them. The servo box follows the bar's markers (6.e).
- **b) Retreat screw-out at the screws' rate** (5.3 a'; `RETREAT_SCREW_*`): the jaws LOOSEN while
  the tools back out 10 mm at 2 mm x 73.7 rpm = 2.46 mm/s (4 s); the gripper motors keep loosening
  until the retreat ends; the rigid tail is replanned from where the screw-out ends; nothing waits
  for a STALLED any more. The rpm is the tool's display reading (11.4). **?** Run on 10-27 it
  moved B20 more than the 10-23 version; since 2026-10-28 it is the ticked-box test mode and the
  default is the stiff screw-out, 12 mm (5.3 a).
- **c) Shorter retreat when the full line collides** (made on the day, 18:55, during B35: 7.b):
  Plan Movement tries 45, 40 ... 25 mm when the 50 mm straight line has no collision-free path;
  the exec-time tail replan aims at the same shortened goal.
- **d) Floor calibration 2026-10-27** (11.3).

### 11.3 The floor file

`20261027_160043_mocap_floor_calibration.json` (22 points: 10 floor probes + the fixed floor points
origin / mark1-3, 0.26 deg tilt, 1.6 mm rms; its x, y and origin fitted to `main_floor_points.json`)
replaces the 22 Oct file (10 points, 0.49 deg) in `MOCAP_FLOOR_CALIBRATION_FILE`. The 23 Oct
as-built pages were built on the 22 Oct floor (each record carries its own `world_from_design`).
**!** `DESIGN_GROUND_RAISE_MM` = 23 was set on 2026-10-23 with the feeler gauge against the 22 Oct
floor: in the model it is floor-independent (the design ground always lands on the plane), but
the plane itself sits 15-23 mm higher at the bars in the new file than in the old one, so **re-check
the gauge under B20's footers (5.2 e) before trusting the raise** on the new floor.

Since 2026-10-29: `20261028_152335_mocap_floor_calibration.json` (13 points: origin / mark1-3, 3-4
probes each, 0.37 deg tilt, 0.8 mm rms; x, y and origin fitted to the marks and anchors of
`main_frame.json`). The relay's new format calls the map `floor_from_raw` (from Motive's own
coordinates, each sample's `raw_position`: what the monitor reads); the loader takes either name.
This floor is 2-5 mm lower than the 27 Oct one where the bars stand (B20 -2.1, B34 -3.4, B46
-4.8 mm), so the placed design sits that much lower too. **!** The gauge check above still applies.
The monitor reads the file at start: restart it to use the new floor.

### 11.4 Open points

- [x] The screw rate: `RETREAT_SCREW_RPM` = 72 (2.4 mm/s) since 2026-10-28, from the three release
      tests' videos (B20 ticks once per screw turn: 71-74 rpm; the tool's display reads 73.7).
- [ ] The release test prints wrong rates: it looks for the push in the wrist force, and B20 (footers
      only placed) slid away with less than 3 N. Fix it before using its numbers again.
- [ ] Confirm a) in the lab on B21 (greyed box, `STALLED at` line, `settled` line, no skew, no
      `ARMS LEFT UNDER`) and that B22 is unchanged (box ticked).
- [ ] Spread the MoCap anchor markers; a still reference body in every record (11.1).
- [ ] B37's right grasp (11.1).
- [ ] Belle's e-stop and gripper (11.1).
- [ ] A right-arm base calibration (11.1).
- [ ] The feeler gauge under B20 on the 27 Oct floor (11.3).
