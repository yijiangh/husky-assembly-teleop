# Cindy + Alice + Belle B20–B37 test — with the real robots, run from the robot-control PC

This is the operator manual for the design problem **`261006_3bar_holding_test`** (re-exported
2026-10-13 as the range **B20–B37**, 18 actions). It is written for someone who has **never run
this test**.

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

Three mobile robots ("huskies") build seven bars into a structure:
- **Cindy** (dual-arm husky, two UR5e arms with scaffolding tools) joints and releases the bars.
- **Alice** (single-arm husky, Robotiq 2F-85 gripper) grabs **B21** after Cindy has inserted it.
- **Belle** (single-arm husky, Robotiq 2F-85 gripper) grabs **B35** after Cindy has inserted it.

Both support robots hold their bar until B37 is in, then let go (actions 16 and 17).

**The story of one bar (B21):**
1. Cindy moves her two empty tools to where B21 will go (the *transfer*).
2. You mount B21 in her tools by hand and clamp it with the buttons on the tools.
3. Cindy corrects her pose by a few mm using mocap (the *servo*), then inserts B21: its left joint
   screws into B20.
4. Alice drives up, opens her gripper, slides it onto B21 and closes it (the *hold*).
5. Cindy lets go and backs off (the *release*). Alice holds B21 until B37 is in, then opens and
   backs off (the *let go*).

**The 18 actions:**

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

**One robot per monitor run.** The monitor (the operator program, 0.6) drives one robot at a time
and draws the other two from their live mocap base, or from what `progress.json` remembers. So
the session is seven monitor runs in turn:

| run | robot | actions | part |
|---|---|---|---|
| 1 | Cindy | 0–2 | 5.6 |
| 2 | Alice | 3 | 5.7 |
| 3 | Cindy | 4–9 | 5.8 |
| 4 | Belle | 10 | 5.9 |
| 5 | Cindy | 11–15 | 5.10 |
| 6 | Alice | 16 | 5.11 |
| 7 | Belle | 17 | 5.12 |

Quitting a monitor leaves the robot's driver running: the arms stay where they are and keep
holding.

### 0.2 Roadmap (the whole session)

| # | what | part | pane | robot | does anything move? | saved in `progress.json`**?** |
|---|---|---|---|---|---|---|
| 1 | open the five terminal panes | 0.7 | any terminal | — | no | no |
| 2 | settings, problem folder, network | 1 | 1 | — | no | no |
| 3 | pendants, robot launches, mocap | 2 | 2–4 | all | no | no |
| 4 | robot checks | 3 | 5 | all | no | no |
| 5 | in-air test: Belle, then Alice | 4 | 1 + monitor window | Belle, Alice | gripper; controller switch (the arm should stay still) | no |
| 6 | run 1: actions 0–2 | 5.6 | 1 + monitor window | Cindy | arms (+ base when driven) | yes |
| 7 | run 2: action 3 | 5.7 | 1 + monitor window | Alice | base, arm, gripper | yes |
| 8 | run 3: actions 4–9 | 5.8 | 1 + monitor window | Cindy | arms, base | yes |
| 9 | run 4: action 10 | 5.9 | 1 + monitor window | Belle | base, arm, gripper | yes |
| 10 | run 5: actions 11–15 | 5.10 | 1 + monitor window | Cindy | arms, base | yes |
| 11 | run 6: action 16 | 5.11 | 1 + monitor window | Alice | gripper, arm, then base away | yes |
| 12 | run 7: action 17 | 5.12 | 1 + monitor window | Belle | gripper, arm, then base away | yes |
| 13 | send the results back | 8 | — | — | no | — |

Something wrong at any point: Part 7.

### 0.3 What you need

| what | where | note |
|---|---|---|
| this PC | `CRLRobotxPC`, user `su` | runs the monitor and the checks |
| workspace | `~/ros2_ws` (venv `~/ros2_ws/venv`, ssik 4.1 installed) | |
| shared drive | `~/Insync/2025-03 Husky Assembly` | the package's built-in data paths already point here; `progress.json` lives here (1.d) |
| robot network | this PC's built-in ethernet port **`eno1`**, DHCP from the router `192.168.0.1` (this PC was `192.168.0.25` on 2026-10-07) | the Wi-Fi `wlp9s0` (+ Tailscale) is for the internet; ROS does not use them |
| DDS (how ROS talks) | `~/.cyclonedds.xml` pins CycloneDDS to `eno1`; `~/.bashrc` exports `CYCLONEDDS_URI` but **not** `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` | the monitor start blocks (5.1) set those two |
| Cindy's PC | `192.168.0.115`, user `administrator` | this PC's ssh key is accepted |
| Alice's PC | `192.168.0.113`, user `administrator` | password `clearpath` if asked (`doc/husky_ros2.md`) |
| Belle's PC | `192.168.0.114`, user `administrator` | password `clearpath` if asked |
| Motive PC (mocap) | `192.168.0.28` | streams Cindy's rigid body **1860**, Alice's **1840**, Belle's **1850** |
| remote e-stops | one per robot | pair one husky at a time (`doc/husky_ros2.md`, troubleshooting) |

### 0.4 How the PCs connect

```text
  Motive PC 192.168.0.28 ---- mocap stream (robot base poses) ----+
                                                                   |
  Cindy's PC .115 --+                                              v
  Alice's PC .113 --+--- robot router 192.168.0.1 --- this PC (eno1, 192.168.0.25)
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
| **run** (monitor run) | one start of the monitor for one robot, from `ros2 run` until you quit it. The assembly is seven runs in turn: Cindy 1, 3, 5; Alice 2, 6; Belle 4, 7 (table in 0.1) |
| **action** | one unit of the plan: 18 here, numbered **0–17** in build order (table in 0.1; "action 0" = the first, Cindy jointing B20). `B20_J_joint` = Cindy joints B20. `J` jointing (insert), `R` release, `H` hold, `HR` hold release (let go) |
| **step** (movement) | one part of an action: a free move, a straight move, a gripper / tool step, or a manual step. The `Step` slider picks one; the readout shows `step i/n` |
| **plan skeleton** | the ordered list of all 18 actions, who does each and which are done (the `Actions` list in the panel) |
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
│   plan skeleton: robot Cindy (domain 86) | ... | progress k/18 done   │
│   [Switch to Alice] [Switch to Belle] [Switch to Cindy (current)]     │
│   Action (idx) ──●──     -> action ...                                │
│   [Prev action] [Next action] [Load action] [Mark action done]        │
│   [Reopen action (reload clean)] [Rescan plan skeleton status]        │
│   [Reset plan skeleton to the Rhino export]                           │
│   [ ] Ignore built-bar collisions (bars drawn faint)                  │
│   now: Now: action k -- ... step i/n ...                              │
│   Actions ▸  (the 18 rows; click one to select it; grey = other robot)│
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
│   [ ] Insert compliant part: SPIRAL search (...)                      │
│   [Move Arms to Movement Start (offline target)]                      │
│ CONTROLLERS                                                           │
│   [Switch to Compliance (BOTH)] [Switch to Joint (BOTH)]              │
│ GRIPPER (Alice, Belle)   [Open Gripper Full] [Close Gripper for Bar]  │
│ SCAFFOLDING V3 (Cindy)   L / R: Stop All, Tighten / Loosen Gripper,   │
│                          Tighten / Loosen Joint                       │
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
grep -nE "^    (CALIBRATION|PUNCH_CALIB_VALIDATION|BAR_ACTION_LIVE_REPLAN_EXE|BAR_ACTION_MOCAP_ACCURACY_TEST) *=" "$F"
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
`192.168.0.25` on 2026-10-07; the Razer laptop's `192.168.0.29` stays a comment), and
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
P="$GD/data_design_study/261006_3bar_holding_test"
ls "$P"                                        # expect: ActionSchedule.json BarActions RobotCell.json RobotCell_Alice.json RobotCell_Belle.json WalkableGround.json ...
ls "$P"/BarActions/*__*.json | grep -vc live-solved     # expect: 18 action files (B20..B37 __J/__R, B21 and B35 __H/__HR)
ls -a "$P" | grep -c insyncdl                  # expect: 0 = Insync has finished syncing
ls "$P"/progress.json "$P"/BarActions/*.live-solved.json 2>/dev/null   # left over from an earlier run?
```

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
echo "[cindy] domain $ROS_DOMAIN_ID | $RMW_IMPLEMENTATION"   # expect: domain 86 | rmw_cyclonedds_cpp
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
| topics arrive | this PC receives the robot's ROS traffic | `no topics at all ..., although its PC answers`: its launch is not running, **or** it runs but its ROS traffic leaves through the wrong network port (Alice and Belle on 2026-10-19: the launch was up, this PC saw only `/parameter_events` and `/rosout`) → `Ctrl-C` the launch and start it again with the `IF=...` / `CYCLONEDDS_URI` lines of 2.c / 2.d. The robot's other checks are skipped |
| base | the base driver listens on `<ns>/cmd_vel` (twist_mux) | Cindy: base driver down or network → 1.e; or her base system is on Zenoh → the middleware step of 2.b. Alice / Belle: only a `NOTE` (on 2026-10-16 their bases were not visible from this PC, only their arms); the monitor does not need it (it takes their base from mocap), only command-line driving does (6.a): use the joystick |
| joint states | ≈ **50 Hz** from the arm driver (`<ns>/<arm>/rate_limiter/joint_states`) | just after the launch: run the check again after ~10 s. Still nothing: read the pendant and controllers lines; else restart the launch |
| pendant | `ros_control.urp` PLAYING, robot RUNNING (so Remote mode) | `dashboard does not answer`: the arm is off or the launch is not running → start it (Part 2). `robot mode ...`: power on, release the brakes. **`in Local mode`**: switch the pendant to Remote (2.a); the launch then plays `ros_control.urp` itself and `scaled_joint_trajectory_controller` should turn active by itself; run the check again (Belle on 2026-10-16). `runs 'external_control.urp' ...`: the launch starts `ros_control.urp` within ~10 s: check again; still FAIL → restart the launch. Never start a program by hand |
| controllers | `scaled_joint_trajectory_controller` **active**, `cartesian_compliance_controller` **inactive** (inactive is right: the monitor turns it on only for the insert, the retreat and the grip handoff) | `none loaded`: the launch started before the arm was on and in Remote (Alice on 2026-10-16: arm on and in Remote, no controllers) → `Ctrl-C` the launch, start it again. `joint controller 'inactive'`: usually the pendant line (Local mode) → fix that first, else `Switch to Joint (BOTH)`. `compliance still active` → `Switch to Joint (BOTH)` (6.b). `compliance not loaded` → restart the launch |
| tool status (Cindy) | ≈ **20 Hz** from each scaffolding tool | `no messages` while that arm is fine: that tool's driver has stopped (2026-10-16: Cindy's left one; `ros2 node list \| grep tool` showed only `right_gripper/scaffolding_tool_driver`) → restart Cindy's launch with empty tools; or her launch lacks `gripper:=scaffolding_v3` (2.b). `message type unknown`: this PC's workspace is not built / sourced (1.f) |
| gripper (Alice, Belle) | the action `<ns>/gripper/robotiq_gripper_controller/gripper_cmd` is up | the launch lacks `gripper:=robotiq_2F_85` (2.c / 2.d), or the gripper driver failed: read its pane |

**?** The same checks by hand, one command at a time: Part 6.h.

---

## 4. Test the support robots in the air  [TEST — arm / gripper move, no bar, nothing recorded]

Alice and Belle take over a bar from Cindy with a **grip handoff**: the arm goes soft (compliance
controller) and the gripper closes, so the arm follows the bar instead of pulling it. **Belle's**
handoff has never run on hardware. **Alice's** ran on 2026-10-12; its tuning is still open
(`tasks/2026-10-12_alice_grip_handoff_tuning.md`).

So test both **once, before the assembly starts** (before Part 5 and its first action, action 0:
Cindy jointing B20; table in 0.1). A problem then shows up before Cindy holds a bar for them. No
bar, clear workspace.

**!** **Do not `Mark action done` here**: these are tests.

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
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
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
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
ros2 daemon stop      # ! every time, or the monitor may miss the robot's topics
ros2 run husky_assembly_teleop husky_monitor
```

- [ ] pane 1: read the start-up lines (scroll back with the mouse wheel)
  - **Expect:** `Base calibration of Belle (0805): 20261013/calibrated_transformation_0805_rhino.json`
    (one such line per robot);
  - **Expect:** `[Plan skeleton] robot Belle (domain 85) | problem 261006_3bar_holding_test | progress 0/18 done`
    (Alice: `robot Alice (domain 84) ...`);
  - **Expect:** `[Plan skeleton] others: Cindy <- parked | Alice <- parked` (Alice: `Cindy <- parked | Belle <- parked`):
    nothing is done yet;
  - **Expect:** `mocap client connected: True`;
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

---

## 5. The assembly  [REAL RUN — recorded]

### 5.1 Every monitor run: which robot it drives, start, switch, quit  [REAL RUN — recorded]

Part 4's in-air test starts its monitors the same way.

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
| Cindy's | Cindy | 5.6, 5.8, 5.10 |
| Alice's | Alice | Part 4 (in-air test), 5.7, 5.11 |
| Belle's | Belle | Part 4 (in-air test), 5.9, 5.12 |

**Cindy's block:**

**▶ pane 1 · MONITOR**
```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=86
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml   # already in ~/.bashrc; pins DDS to eno1
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
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
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
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
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
ros2 daemon stop >/dev/null 2>&1
echo "[belle env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}') | problem $DESIGN_PROBLEM_NAME"
```

**Starting the monitor, in this order:**
- [ ] a) pane 1: paste the robot's block
  - **Expect:** one line, e.g. `[cindy env] domain 86 via eno1 UP 192.168.0.25/24 | problem 261006_3bar_holding_test`
    (and a `(venv)` prefix on the prompt). The domain in that line is the robot's;
    `echo $ROS_DOMAIN_ID` shows it again any time.
- [ ] b) pane 1: start the monitor

**▶ pane 1 · MONITOR**
```bash
ros2 daemon stop      # ! every time, or the monitor may miss the robot's topics
ros2 run husky_assembly_teleop husky_monitor
```

  - **Expect:** its start line names the robot:
    `[Plan skeleton] robot Cindy (domain 86) | problem 261006_3bar_holding_test | progress k/18 done`
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
  - **Expect:** the right robot and `progress k/18 done`; each run below names the lines to expect.
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
run plans around it (Part 6.k). `Mark action done` still marks the action done (the `k/18`
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
  - **Expect:** `[transfer <bar>_J_M3_CDFM_transfer_to_approach] the arms stand at the insertion start with EMPTY tools: mount the bar and clamp it with the tool buttons, then click 'Confirm Exec'. ...`
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
- [ ] slider [traj time]: higher = slower (first run; the rigid insert has no force limit) →
      button [Exec Selected Mv Traj (auto)]
  - **Expect:** depends on the bar (table below).
- [ ] ground bar only: nothing to click; the same exec goes on into a **second servo**, which puts
      the bar on its assembled pose (the fix step's start) and loads the fix step by itself
  - **Expect:** `[insert <bar>_J_M4_LM_insert] servo to the start of <bar>_J_M5_manual_fix_foundation (the insert's end), feedback: tool0; straight-line corrections only.`
    (`feedback: MoCap bar` when box [Servo: aim grounded bar by MoCap (else tool0)] is ticked), then
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
- [ ] by hand: fix the bar to the foundation → button [Confirm Exec]

**f) Mark**
- [ ] button [Mark action done]
  - **Expect:** `[Plan skeleton] action k <action> marked done by Cindy; Cindy's belief <- live. Saved ...`,
    header `k+1/18`; the next action loads.

### 5.3 Procedure R — release (Cindy)  [REAL RUN — recorded]

Two steps: the retreat (the ungrasp runs with it), then the free move home.

**a) Retreat** (`<bar>_R_M1_LM_retreat`)
- [ ] slider [Step]: 0 (the action opens on it) → button [Load Movement]
  - **Expect:** readout `step 1/2: <bar>_R_M1_LM_retreat  (+ <bar>_R_M0_tool_ungrasp_bar runs with it)`;
    the bar and its joints drawn solid (built, collision-checked).
- [ ] button [Plan Movement]
  - **Expect:** it plans from where the arms **are** (the insert leaves them a little off its planned end):
    - `[<bar>_R_M1_LM_retreat] start_state resynced to the live arms (max joint drift ... deg)` and
    - `[retreat live] <bar>_R_M1_LM_retreat: goals = the live tool0 poses moved 50 mm back: left along (...), N mm from the exported goal; right ...`.
  - Each tool0 backs off 50 mm along its exported retreat axis, orientation kept; scene collisions
    are checked.
  - **If not:** no path: read the `[<bar>_R_M1_LM_retreat diag]` block (7.c).
- [ ] slider [Traj viz time]: preview
  - **Expect:** starts at the live arms.
- [ ] button [Exec Selected Mv Traj (auto)]
  - **Expect:** the ungrasp runs with it (gripper motors LOOSENING), and once
    `'<bar>_R_M1_LM_retreat': the export asks for the joint_tracking controller; the monitor runs this dual_independent_linear movement under ...`
    (expected);
  - **Expect:** before the compliant part both force sensors are zeroed:
    `force sensors zeroed before the compliant part (|F| a/b -> c/d N)` (the left / right reading
    before and after; a leftover force would otherwise push the tools into the joint);
  - **Expect:** after the compliant part the monitor prints its checks of the rigid part,
    `[<bar>_R_M1_LM_retreat] check PASS / FAIL ...` (replan, start, compliant part, contact,
    grippers, left / right tool0 path, joints, speed), then runs the rigid part at once, **no
    `Confirm Exec`**: `running the rigid part now (N waypoints over T s); every check passed.` Or
    `N check(s) FAILED, see the "check FAIL" lines above`: note them (7.c);
  - **Expect:** joint controller restored afterwards.

**b) Free move home** (`<bar>_R_M2_free_home`, may be skipped)
- [ ] slider [Step]: 1 → button [Load Movement] → button [Plan Movement]
- [ ] slider [Traj viz time]: preview → button [Exec Selected Mv Traj (auto)]
  - It may be skipped: the retreat's end is already saved, and the next jointing action's
    transfer plans from where the arms are (collision checked).

**c) Mark**
- [ ] button [Mark action done]
  - **Expect:** header `k+1/18`.

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
  - **Expect:** the phase lines of Part 4 e). The arm is compliant for the whole close: it gives way while
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
  - **Expect:** `[Plan skeleton] robot Cindy (domain 86) | problem 261006_3bar_holding_test | progress 0/18 done`
    and `[Plan skeleton] others: Alice <- parked | Belle <- parked`.
- [ ] Actions 0–2 in turn, each with the procedure in the table (J: 5.2, R: 5.3):

| action | procedure | what differs |
|---|---|---|
| 0 `B20_J_joint` | J, ground bar (six steps: travel to load (skipped), manual mount, tool grasp, transfer, insert, fix foundation) | opens on `step 4/6: B20_J_M3_CDFM_transfer_to_approach ...`. B20 stands on its two ground joints `joint_G20-T20-0_ground`, `joint_G20-T20-1_ground` and has no tighten step. Insert readout `step 5/6: B20_J_M4_LM_insert  (dual_constrained_linear)`; the joint motors do not turn. Fix `B20_J_M5_manual_fix_foundation`. Mark: `[Plan skeleton] action 0 B20_J_joint marked done by Cindy; Cindy's belief <- live. Saved ...`, header `1/18` |
| 1 `B20_R_release` | R | `B20_R_M1_LM_retreat` (`B20_R_M0_tool_ungrasp_bar` runs with it), then `B20_R_M2_free_home`. Mark: `2/18` |
| 2 `B21_J_joint` | J, one-sided (five steps; the tighten runs with the insert) | built B20 and its joints drawn solid; the transfer's preview clears them. Transfer `B21_J_M3_CDFM_transfer_to_approach` (readout `step 4/5: ...`). Insert readout `step 5/5: B21_J_M5_LM_insert  (+ B21_J_M4_tool_tighten_joint runs with it)`. B21's left male (`joint_J20-21_male`) screws into B20. The right tool holds `joint_J157-21_male`, whose female sits on B157, outside this export. The export already names only `AT3L`, so there is no `tightens only ['AT3L']` warning. Optional spiral search (6.f), to compare with the straight line. Mark: `3/18`: Cindy's live base and arms were already saved to `progress.json` when the insert ended (Alice's run plans around them) |

- [ ] pane 1: `Ctrl-C` (quit Cindy's monitor), or button [Switch to Alice]
  - Cindy keeps holding B21 (her driver keeps running). Do not touch her launch pane.

### 5.7 Run 2 — Alice, action 3: Alice grabs B21  [REAL RUN — recorded]

- [ ] pane 1: Alice's block → start the monitor (5.1); or button [Switch to Alice]
  - **Expect:** `robot Alice (domain 84) ... progress 3/18 done` and
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
    header `4/18`.
- [ ] pane 1: `Ctrl-C` (quit Alice's monitor), or button [Switch to Cindy]
  - Alice stays clamped on B21.

### 5.8 Run 3 — Cindy, actions 4–9  [REAL RUN — recorded]

- [ ] pane 1: Cindy's block → start the monitor (5.1); or button [Switch to Cindy]
  - **Expect:** `[Plan skeleton] Cindy starts from the progress.json state (after action 2, live).` (her
    real joints take over at once), header `4/18`,
    `others: Alice <- live (action 3) | Belle <- parked`.
  - **Expect:** 3D view: Alice drawn on B21 at her hold pose (mocap base).
- [ ] Actions 4–9 in turn, each with the procedure in the table (J: 5.2, its a) ticks the
      workaround box again; R: 5.3):

| action | procedure | what differs |
|---|---|---|
| 4 `B21_R_release` | R, Alice holding B21 | load: `[obstacle robots] posed from: ObstacleRobotAlice <- live, ObstacleRobotBelle <- exported`; opens on `step 1/2: B21_R_M1_LM_retreat  (+ B21_R_M0_tool_ungrasp_bar runs with it)`. **B21 is clamped by Alice** (the ungrasp releases Cindy's tools from it). Retreat and `B21_R_M2_free_home` are planned with Alice in the scene: check in the retreat's preview that both tools back off clear of Alice's gripper. Mark: `5/18` |
| 5 `B22_J_joint` | J, both joints mate (five steps, as action 2) | B22's left male (`joint_J21-22_male`) screws into B21, which Alice still holds; its right male (`joint_J20-22_male`) into B20. Built B20, B21 and Alice (red obstacle robot) drawn; the transfer's preview clears them all. Transfer `B22_J_M3_CDFM_transfer_to_approach`: a warning `Plan end of 'B22_J_M3_CDFM_transfer_to_approach' differs from existing 'B22_J_M5_LM_insert'.start by max 6.2832 rad/m; overwriting (carry rule).` means one joint of the insertion start is a full turn (2π) from the exported one: same tool poses, harmless (seen headless). Insert readout `step 5/5: B22_J_M5_LM_insert  (+ B22_J_M4_tool_tighten_joint runs with it)`. **!** Watch B21 while B22 screws in: Alice holds it; it must not be pushed out of her jaws. Mark: `6/18` |
| 6 `B22_R_release` | R | opens on `step 1/2: B22_R_M1_LM_retreat  (+ B22_R_M0_tool_ungrasp_bar runs with it)`; then `B22_R_M2_free_home`; Alice in the scene. Mark: `7/18` |
| 7 `B34_J_joint` | J, ground bar (six steps, as action 0) | B34 is the second ground bar: it stands on its ground joints `joint_G34-T20-0_ground` and `joint_G34-T20-1_ground`, has no tighten step, and ends with a manual foundation fix. The base drives about 1.1 m from where it stood for B22. Built B20–B22 and Alice drawn; the transfer's preview clears them. Transfer `B34_J_M3_CDFM_transfer_to_approach`. Insert `B34_J_M4_LM_insert`: `[B34_J_M4_LM_insert] grounded bar (holds joint_G34-T20-0_ground, joint_G34-T20-1_ground): rigid insert ...` and `... rigid insert done (no tighten).`; the joint motors do not turn. Fix `B34_J_M5_manual_fix_foundation`. Mark: `8/18` |
| 8 `B34_R_release` | R | `step 1/2: B34_R_M1_LM_retreat  (+ B34_R_M0_tool_ungrasp_bar runs with it)`, then `B34_R_M2_free_home`. Mark: `9/18` |
| 9 `B35_J_joint` | J, one-sided (five steps, as action 2) | B35 is **one-sided**, like B21: its left male (`joint_J34-35_male`) screws into B34; the right tool holds `joint_J162-35_male`, whose female sits on B162, outside this export; the export names only `AT3L` for the tighten. Transfer `B35_J_M3_CDFM_transfer_to_approach` (mount B35 in the pause). Insert `B35_J_M5_LM_insert`: `[B35_J_M5_LM_insert] one-sided bar: the left joint mates, so only the L joint motor TIGHTENs ...` and `[scaffolding] B35_J_M5_LM_insert: TIGHTEN joint motor on L arm (arm 0)` (no R line). After the insert Cindy holds B35, joint controller active. Mark: `10/18` (Belle's run plans around Cindy's stored pose) |

- [ ] pane 1: `Ctrl-C` (quit Cindy's monitor), or button [Switch to Belle]
  - Cindy keeps holding B35.

### 5.9 Run 4 — Belle, action 10: Belle grabs B35  [REAL RUN — recorded]

The same four steps as Alice's hold. Her start-up prints
`Base calibration of Belle (0805): 20261013/...` (1.c); the calibration check after the free
approach is the first test of it on an assembly.

- [ ] pane 1: Belle's block → start the monitor (5.1); or button [Switch to Belle]
  - **Expect:** `robot Belle (domain 85) ... progress 10/18 done` and
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
    header `11/18`.
- [ ] pane 1: `Ctrl-C` (quit Belle's monitor), or button [Switch to Cindy]
  - Belle stays clamped on B35.

### 5.10 Run 5 — Cindy, actions 11–15  [REAL RUN — recorded]

- [ ] pane 1: Cindy's block → start the monitor (5.1); or button [Switch to Cindy]
  - **Expect:** `[Plan skeleton] Cindy starts from the progress.json state (after action 9, live).`,
    header `11/18`, `others: Alice <- live (action 3) | Belle <- live (action 10)`.
  - **Expect:** 3D view: Alice on B21 and Belle on B35, at their hold poses (mocap bases).
- [ ] Actions 11–15 in turn, each with the procedure in the table (J: 5.2, its a) ticks the
      workaround box again; R: 5.3):

| action | procedure | what differs |
|---|---|---|
| 11 `B35_R_release` | R, Belle holding B35 | load: `[obstacle robots] posed from: ObstacleRobotAlice <- live, ObstacleRobotBelle <- live`; opens on `step 1/2: B35_R_M1_LM_retreat  (+ B35_R_M0_tool_ungrasp_bar runs with it)`. **B35 is clamped by Belle** (the ungrasp releases Cindy's tools from it). Then `B35_R_M2_free_home`. Check in the retreat's preview that both tools back off clear of Belle's gripper. Mark: `12/18` |
| 12 `B36_J_joint` | J, both joints mate (as action 5) | B36's left male (`joint_J35-36_male`) screws into B35, which Belle holds; its right male (`joint_J34-36_male`) into B34. Transfer `B36_J_M3_CDFM_transfer_to_approach`, two messages seen headless: the same 2π warning as B22's (`... differs from existing 'B36_J_M5_LM_insert'.start by max 6.2832 ...`): harmless; and once `[free validation] ... SWEPT COLLISION ... hits ['ground_WG0']` with `plan REJECTED: it sweeps through the scene between waypoints`: the insertion start is close to the floor and one random path grazed it → `Plan Movement` again. Insert `B36_J_M5_LM_insert`: **both** joint motors tighten. **!** Watch B35 while B36 screws in: Belle holds it; it must not be pushed out of her jaws. Mark: `13/18` |
| 13 `B36_R_release` | R | `step 1/2: B36_R_M1_LM_retreat ...`, then free move home; Alice and Belle in the scene. Mark: `14/18` |
| 14 `B37_J_joint` | J, both joints mate (as action 5) | the base drives about 1.5 m from where it stood for B36. B37's left male (`joint_J21-37_male`) screws into **B21 (Alice)**; its right male (`joint_J35-37_male`) into **B35 (Belle)**. Transfer `B37_J_M3_CDFM_transfer_to_approach`: the 2π warning may show here too (harmless). Insert `B37_J_M5_LM_insert`: **both** joint motors tighten. **!** Watch B21 and B35 while B37 screws in: neither may be pushed out of its support robot's jaws. Mark: `15/18` |
| 15 `B37_R_release` | R | `step 1/2: B37_R_M1_LM_retreat ...` → retreat. **!** **Free move home** (`B37_R_M2_free_home`): headless, the first `Plan Movement` failed in both of two runs (`plan_free_dual_arm failed: birrt_failed` and its diag block); the second try planned. Count on that and **plan again**. The home goal turns the right shoulder (`right_ur_arm_shoulder_pan_joint`) by about **5.05 rad (≈ 290°)**, past B36: preview it through, raise `traj time`, hand on Cindy's e-stop. Mark: `16/18` |

- [ ] pane 1: `Ctrl-C` (quit Cindy's monitor), or button [Switch to Alice]

### 5.11 Run 6 — Alice, action 16: Alice lets go of B21  [REAL RUN — recorded]

B21 now hangs in the structure (jointed to B20, B22 and B37); Alice opens her jaws and backs out
100 mm along her approach line.

- [ ] pane 1: Alice's block → start the monitor (5.1); or button [Switch to Alice]
  - **Expect:** `robot Alice (domain 84) ... progress 16/18 done` and
    `others: Cindy <- live (action 15) | Belle <- live (action 10)`.
- [ ] Procedure HR (5.5) with `<b>` = B21. What differs:
  - gripper open `B21_HR_M0_gripper_open`: B21 must stay where it is;
  - retreat `B21_HR_M1_LM_retreat`;
  - Mark: `[Plan skeleton] action 16 B21_HR_hold_release marked done by Alice; Alice's belief <- live. Saved ...`,
    header `17/18`;
  - drive away with domain 84 (6.a). The export assumes she leaves the scene, and Belle's run
    draws her parked.

### 5.12 Run 7 — Belle, action 17: Belle lets go of B35  [REAL RUN — recorded]

The same as 5.11, with Belle's values.

- [ ] pane 1: Belle's block → start the monitor (5.1); or button [Switch to Belle]
  - **Expect:** `robot Belle (domain 85) ... progress 17/18 done`.
- [ ] Procedure HR (5.5) with `<b>` = B35. What differs:
  - load: `[obstacle robots] posed from: ObstacleRobotAlice <- parked, ObstacleRobotCindy <- live`
    (Alice released), readout `step 1/2: B35_HR_M0_gripper_open ...`;
  - gripper open: B35 stays where it is;
  - retreat `B35_HR_M1_LM_retreat`: `[Plan] 'B35_HR_M1_LM_retreat': ≈ 21 waypoints stored.`;
    watch the jaws clear B35;
  - Mark: header `18/18`.
- [ ] The assembly is done: Part 8.

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
- [ ] `[Plan skeleton] robot <Cindy (domain 86) | Alice (domain 84) | Belle (domain 85)> | problem 261006_3bar_holding_test | progress k/18 done`,
      `[Plan skeleton] others: ...`, the 18-action roster, and `mocap client connected: True`
      (`False`: `CLIENT_IP`, 1.b).
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

### 6.f The spiral search (optional, insert of a bar that tightens)

Tick `Insert compliant part: SPIRAL search (unticked = straight line; radius 0->15 mm, bar-centric)`
before `Exec Selected Mv Traj`: the compliant part then circles the bar around the planned
insertion line instead of driving straight along it.
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
    `~/Insync/2025-03 Husky Assembly/data_design_study/261006_3bar_holding_test/archive/<time>/`.
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
| Belle's arm moves when the handoff goes compliant | e-stop; her controller frame may differ from Alice's (Part 4). Note how far and which way, close stiff with `Close Gripper for Bar` instead |
| the jaws not centred on the bar after the free approach | that robot's base calibration is off (1.c): stop before the straight approach; recalibrate it |
| `Base calibration file not found for robot 0805` (Belle) / `0804` (Alice) | the robot's file is not in the folder its `BASE_CALIBRATION_DATE_BY_ROBOT` entry names (1.c), or Insync has not synced it |
| no topics / `Unknown topic` | the robot is not visible: `eno1` on `192.168.0.x` and the robot pings (1.e); the right domain + CycloneDDS in this pane (5.1); the robot's launch still running with its domain + CycloneDDS (Part 2); or the base driver is down |
| Alice / Belle: the launch runs, but this PC sees nothing from her (`ros2 topic list` shows only `/parameter_events` and `/rosout`) | her ROS traffic leaves through her robot-internal port `br0`: relaunch with the `IF=...` / `CYCLONEDDS_URI` lines of 2.c / 2.d; the `echo` line must name her lab-network port (`DDS on 'wlp2s0'` for Belle), not `''`. Still nothing: the Wi-Fi drops the discovery messages; add this PC as a peer, i.e. end the `CYCLONEDDS_URI` value with `...</Interfaces></General><Discovery><Peers><Peer address="192.168.0.25"/></Peers></Discovery></Domain></CycloneDDS>` (`doc/calibration_manual.md` 2.3). The export lasts only for that ssh shell; nothing changes on the robot |
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
  all seven monitor runs.
- `~/Insync/2025-03 Husky Assembly/data_design_study/261006_3bar_holding_test/progress.json` after
  the run.
- **Per jointing action:**
  - the `[transfer workaround] ... max |goal - stored goal|` line;
  - the `[tool0 check] ... minus target (Rhino)` line from `Draw tool0: ...` after the transfer;
  - whether the mounted bar sat in the tools at the insertion start;
  - how the insert finished (stall time, any rigid / compliant warnings).
- **Per release action:** the `[..._LM_retreat] check ...` lines (which passed, the numbers of any
  FAIL), and whether the arms backed off cleanly (actions 4, 11: clear of the support robot's
  gripper).
- **Actions 0 and 7:** how the rigid insert ended (the `grounded bar` lines; the bar's final fit
  before fixing the foundation).
- **Action 15:** how many tries the free move home took.
- **Alice and Belle** (Part 4, 5.7, 5.9, 5.11, 5.12):
  - the phase lines and any "no handoff" warning;
  - how the arm behaved while compliant (Belle: did it stay still when going compliant?);
  - the `[live goal]` line and the servo's iterations;
  - how far the open jaws were from the bar's centre after the free approach;
  - whether the held bar moved while it was clamped or while the next bars were screwed in;
  - whether the jaws cleared the bar on the release.
