# Cindy, Alice and Belle: the first three bars with the new monitor — real robots, from a control PC

This is the operator manual for the first three bars of the design problem **`261103_SRD`**
(76 actions) on the real robots, with the new monitor: schedule entries 1–7, **B20_J, B20_R,
B21_J, B21_H, B21_R, B22_J, B22_R**. It is the fake-lab test (`doc/lab/fake_lab_test_manual.md`) on the real robots: the
same plugins and steps, run over Zenoh from a control PC: the RobotX PC (`CRLRobotxPC`) or Yijiang's laptop. The fake
lab still runs the older `261008_2ModulesDemo_purged_process`: its base targets differ, and its B20_J has no
foundation step.

> **?** Written from the fake-lab runs and the code; Part 1 run on the RobotX PC on
> 2026-11-02, nothing on the robots yet. Log lines are quoted from the code; a number marked *(fake lab)* is what the
> simulation gave on that older process, and the robots will give their own. Tell me where the page or the robots
> differ (Part 9).

- **First time:** read Parts 0–5 in order.
- **Done it before:** use the roadmap (0.2) as a checklist; each row links to its part.

How to read the steps:
- `- [ ]` is one thing to **do**, named by the control and what to do with it, with the page's own labels.
  - What you click, drag or press is in `code`: button `Plan`, slider `Preview time`, box `Dry run`: ticked, row
    `4. B21_H`, key `Enter`. A tab path is written `Plan` › `Base`; a button in a row of buttons `Load: Step` (the
    button Step in the row Load). `→` joins controls you use one right after the other.
  - What you read (terminal lines, page text, chips, values, paths) is in *italic*; VS Code's preview shows it upright
    in teal (`doc/manual_preview.css`).
  - `pane cindy: Enter`, `by hand: ...`, `by eye: ...` name where you act.
- The indented lines under it:
  - **Expect:** the lines you should see in a terminal pane, quoted, with the pane named (`pane monitor:`); `...`
    stands for what varies. Only the few lines that tell the step went right.
  - **If not:** what to do when they don't come (or a pointer to Part 8).
  - A line without a label is a note: what the page or the robot shows, why, what to write down.
- `→` always means "then do the next thing"; it never stands for a result.
- In Part 6 each procedure is split into lettered phases (a), b), ...); the table in 6.5 lists what differs per action.
- **`▶ pane NAME`** above a code block says which terminal pane to paste it into (0.7). Paste the block as is: its
  last line, `###`, does nothing but marks in the pane where the command ends and its output begins. Blocks without a
  tag are output or diagrams, not commands.
- **!** starts a warning, **?** a side note.
- A badge in a heading says what kind of part it is:

| badge | means |
|---|---|
| `[SETUP — once]` | installs, settings and files, done once before the first session; nothing moves |
| `[SETUP — every session]` | the same, at the start of every session |
| `[ROBOTS ON — nothing moves]` | powering and launching the robots |
| `[CHECK — nothing moves]` | read-only checks; the monitor's start |
| `[TEST — base, arm or gripper move, no bar]` | tests before the assembly, in a run of their own; nothing is marked done |
| `[REAL RUN — recorded]` | the assembly itself; every part is recorded in the run, `runs/<date>_<time>/` on the Drive |

Anything that does not match an **Expect:** is worth a note (the log lines and what you did; Part 9). What each panel
and button does in detail: `doc/lab/plugins.md`.

---

## 0. Start here

### 0.1 What the test does

Three mobile robots ("huskies") build the first three bars of `261103_SRD`

**One monitor for all three robots.** The new monitor talks to every robot at once over Zenoh (ROS domain 80) and runs
one motion at a time across all of them: no restarts between robots, as the old monitor's seven runs needed. The
**run** (`runs/<date>_<time>/` in the process folder on the Drive) records every part, which actions are done and where
each robot was left.

### 0.2 Roadmap (the whole session)

| # | what | part | pane | robot | does anything move? | recorded in the run? |
|---|---|---|---|---|---|---|
| 1 | stop the fake lab | 1.a | the fake lab's | — | no | no |
| 2 | once: network, Zenoh router, time sync, ssh keys, build, the Drive (1.g only when Part 3 asks) | 1.b–1.h | any terminal | — | no | no |
| 3 | open the six terminal panes | 0.7 | any terminal | — | no | no |
| 4 | pendants, robot launches, Motive, relay | 2 | alice, belle, cindy, relay | all | no | no |
| 5 | robot checks | 3 | checks | all | no | no |
| 6 | the monitor: start, first look, Load | 4 | monitor + page | all | no | no |
| 7 | tests: each base under its follower, Alice's gripper and handoff in the air | 5 | page | Cindy, Alice | bases; Alice's gripper (her arm stays still) | a test run |
| 8 | the assembly, actions 1–7 | 6.0–6.5 | page | Cindy, Alice | bases, arms, tools, gripper | yes |
| 9 | Alice lets go of B21 (optional) | 6.6 | page | Alice | gripper, arm, base | yes |
| 10 | stop | 6.7 | all | all | no | — |
| 11 | send the results back | 9 | — | — | no | — |

Something wrong at any point: Part 8.

### 0.3 What you need

| what | where | note |
|---|---|---|
| this PC | the RobotX PC `CRLRobotxPC` (user `su`), or Yijiang's laptop (user `yijiangh`) | runs the monitor, the relay and the checks. Only its lab address and the Drive's path differ between the two |
| workspace | `~/Code/ros2_ws_next` (venv `venv`, ssik installed), branch `core-port` of `yijiangh/husky-assembly-teleop` | crl-husky in `src/crl-husky`, on `jg/ros2` at the commit README names, or later |
| the Drive | RobotX PC: `~/Insync/2025-03 Husky Assembly`; laptop: `~/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly` | the scripts find either. The process folder; the run is recorded there (`runs/`), the relay reads the base calibrations there |
| lab network | RobotX PC: its port `eno1`, `192.168.0.201`; laptop: the USB-ethernet dongle (e.g. `enxa0cec86472dd`), `192.168.0.203`. Each is that PC's reserved slot | the Wi-Fi stays for the internet; ROS does not use it |
| middleware | Zenoh (`rmw_zenoh_cpp`) on ROS domain 80; this PC's router `crl-zenoh-router` linked to every robot's | the scripts set it; `~/.bashrc` (for the old monitor) stays as it is |
| Cindy's PC | `192.168.0.115`, user `administrator`, password `12345678` | rigid body 1860 |
| Alice's PC | `192.168.0.113`, same user and password | rigid body 1840 |
| Belle's PC | `192.168.0.114`, same user and password | rigid body 1850 |
| Motive PC | `192.168.0.28` | the robots' bodies, and B20 / B21 as assets 1020 / 1021 (their mocap balls) |
| remote e-stops | one per robot | pair one husky at a time (crl-husky README, Troubleshooting) |
| the bars | B20, B21, B22, ready to hand next to Cindy's insert starts | B20 and B21 carry mocap balls, B22 none |
| a phone (optional) | on the robot Wi-Fi | `http://<this PC>:8081` (`192.168.0.201` or `.203`): Stop all, and Confirm / Cancel next to the robots (`doc/lab/phone_manual.md`) |

### 0.4 How the PCs connect

```text
  Motive PC .28 ── NatNet, wired ──► this PC: RobotX PC .201 (eno1), or the laptop .203 (USB-ethernet dongle)
                                       ├ mocap relay: Motive's bodies as ROS topics, the robots' calibrated
                                       ├ monitor: the web page (port 8080), the phone page (8081)
                                       └ Zenoh router, 7447 ─┬─ Cindy .115: router ◄─ her launch (arms, tools, follower)
                                                             ├─ Alice .113: router ◄─ her launch
                                                             └─ Belle .114: router ◄─ her launch
  Every machine runs ROS 2 on Zenoh in domain 80, with a router of its own; this PC's connects to the robots'.
  The robots reach this PC over the Wi-Fi; their followers take their base poses from this PC's relay.
```

### 0.5 Safety first

| what | what it does |
|---|---|
| the **remote e-stop of the robot that moves** | in someone's hand for **every** motion, base or arm; each robot has her own |
| the Husky's own e-stop | locks out the base; it does **not** stop the UR5e arms |
| `Stop all (Esc)` (top left of the page; the phone too) | a soft stop of every robot: bases, arms (through their safety sync), tools; each stays stopped until its own `Resume` under Health. **Not** an emergency stop: it runs over the Wi-Fi |
| `Cancel` (Assembly tab, red) | ends the running step and holds every arm; in a compliant part it also STOPs the tools and switches the arms back to joint control |
| `Ctrl-C` in a robot's pane | stops her launch: arm drivers, tools, follower. **!** Never while she holds a bar |

- **!** Preview every motion with `Play` before `Exec`, and raise `Motion time` for its first run if you like.
- **!** Every pendant's speed slider at **100 %**: slower, a motion times out and is held mid-path.
- **!** Never drive Alice while she holds B21 (from action 4 on).
- **!** Tell everyone in the lab (Jakob above all) that this test drives the robots.
- **!** An arm left under the compliance controller (a loud `ARMS MAY BE LEFT UNDER cartesian_compliance_controller`):
  switch it back before anything else (7.b).
- **!** On Cindy's tools, press one button at a time: B1 and B2 together put that arm in free drive (crl-husky README,
  Free drive from the tool buttons).

### 0.6 Words used in this manual

| word | meaning |
|---|---|
| **monitor** | the operator program: a web page served by this PC (`monitor.sh`, 4.a). It drives all three robots |
| **process** | the design as the monitor runs it: `261103_SRD`, 76 actions in build order (the *schedule*) |
| **action** | one unit of the schedule: `B20_J` Cindy joints B20, `B20_R` she releases it, `B21_H` Alice holds B21, `B21_HR` she lets go |
| **movement**, **step** | one part of an action: a free move, a straight move, a gripper or tool step, a step by hand. The process panel picks one; the step line under the buttons names it |
| **run** | the record of one assembly: `runs/<date>_<time>/` in the process folder on the Drive: which actions are done, where each robot was left, every part executed, the servo readings |
| **Mark done** | the yellow button (it reads `B20_J done`): records the action done and where every live robot stands |
| **Live IK / Plan / Play / Exec** | where the next movement starts, by IK at the live base (teal) / plan the step from the live robot (teal) / preview it (`Preview time`) / run it (blue) |
| **Confirm / Cancel** | answer a step that waits (green / red); its prompt names the robot |
| **Zenoh, router, domain 80** | how ROS talks here: every robot and control PC runs Zenoh on ROS domain 80, meeting through a router on each machine |
| **relay** | `relay.sh`: Motive's rigid bodies on ROS, each robot's as its calibrated base pose; the monitor and the robots' followers read it. One per domain |
| **follower** | the robot's own path follower (`pure_pursuit`, in her launch): drives a base path the base planner commits |
| **namespace** | the prefix of a robot's ROS names: `/a200_0806` Cindy, `/a200_0804` Alice, `/a200_0805` Belle |
| **teach pendant** | the UR arm's tablet. Its **Local / Remote** switch (top right) must be on **Remote** |
| **`ros_control.urp`** | the arm program that hands the arm to ROS; the launch's safety sync loads and plays it. Never load or start a program by hand |
| **safety sync** | `MultiArmSafetySync`, in each robot's launch: tells the monitor whether each arm is operational with its program playing; arm moves are refused without it |
| **launch** | what `robot.sh <robot>` starts on her PC: `crl_arms_and_base.launch.py` (arms, tools, safety sync, follower) |
| **health chips** | the coloured chips per robot, top left of the page; hover one for detail |
| **tool0** | the end of the arm (its flange), where the tool or gripper sits |
| **servo** | after a transfer, an approach or a ground bar's insert: small corrections from mocap until the flanges are within 0.2 mm of their targets; for B20 (**Servo by bar**) until the bar's mocap fit is within 0.5 mm / 0.1° of its design pose |
| **controllers** | `scaled_joint_trajectory_controller` (**Joint**: stiff, follows joints; the normal one) or `cartesian_compliance_controller` (**Cart**: gives way to forces; only for an insert's last millimetres, a retreat's first ones and the handoff) |
| **split** | those millimetres under the compliance controller (`Insert split mm`, 5) |
| **transfer workaround** | the process panel's box: Cindy's transfer runs as a free move with empty tools to the insert start, where you place the bar by hand (this export has no loading pose) |
| **ground bar / one-mate / two-mate** | B20 stands on ground joints: rigid insert, no tighten, then the operator fixes the foundation. B21 mates on its left joint only, B22 on both |
| **ghost** | a see-through robot in the 3D view: blue where the selected movement has the robot (after Live IK or a plan, on the live base) and at the base planner's target, green along a plan |
| **pane** | one of the six terminal areas of 0.7 |

### 0.7 The six terminal panes

The panes live in **tmux**, so they keep running when the terminal window closes, and I can read them from my side.

- [ ] Open a terminal and run:

**▶ any terminal on this PC**
```bash
~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/real_lab/tmux.sh
###
```

**Expect:** in each pane its title in bold and a one-line hint, e.g. pane monitor: `MONITOR  Press Enter once the
checks pass (manual 4.a). Ctrl-C once stops it.`

**If not:** pane monitor: `! The fake lab still runs: stop it first (manual 1.a). ...`: 1.a.

The session `real_lab`, six panes, each with its title on its top border (this manual's `pane cindy` is the pane
titled CINDY). The left column is 70 % wide, its panes 30, 30 and 40 % high; the right one 30 % wide, in even thirds:

```text
┌─ MONITOR ────────────────────────────[z] [x]┬─ ALICE ─────[z] [x]┐
│                                             │                    │
├─ CHECKS ─────────────────────────────[z] [x]┼─ BELLE ─────[z] [x]┤
│                                             │                    │
├─ RELAY ──────────────────────────────[z] [x]┼─ CINDY ─────[z] [x]┤
│                                             │                    │
│                                             │                    │
└─────────────────────────────────────────────┴────────────────────┘
```

| pane | for | tmux.sh left it | logged to |
|---|---|---|---|
| MONITOR | the monitor (4.a) | `monitor.sh` typed in: press Enter in 4.a | `~/husky_real_lab/logs/monitor_<date>.log` |
| CHECKS | `check_robots.sh` (Part 3) and any `ros2` command: domain 80 on Zenoh is loaded (`source env.sh`) | `check_robots.sh` typed in | — |
| RELAY | the mocap relay | running (`relay.sh`) | `~/husky_real_lab/logs/relay_<date>.log` |
| ALICE, BELLE, CINDY | that robot's launch, over ssh (`robot.sh <robot>`, Part 2) | `robot.sh <robot>` typed in: press Enter in Part 2 | `~/husky_real_lab/logs/<robot>_<date>.log` |

- **!** Never close a robot's pane while she holds a bar (the box at the top).
- The scripts are on every pane's PATH: when one stops, **Up, Enter** in its pane starts it again.
- Running `tmux.sh` again only re-attaches.
- I read the panes (`tmux capture-pane`) and the logs from my side; I never type into this session.

> | pane | what closing it stopped |
> |---|---|
> | monitor | the monitor, without its final holds. The robots' drivers keep running and the arms hold where they are |
> | relay | mocap for everyone in domain 80: the monitor's base poses and the robots' followers. **!** Never while a base drives |
> | alice, belle, cindy | the ssh session **and the launch in it**: that robot's arm drivers, tools and follower are off. **!** Never close a robot's pane while she holds a bar |
> | checks | nothing |
> To get the pane back without touching the others:
> 1. Click the pane next to where it was.
> 2. `Ctrl-b` then `"` (split below) or `Ctrl-b` then `%` (split right): a new pane appears, the scripts on its PATH.
> 3. Give it its title back, e.g. for Cindy's pane:
> **▶ the new pane**
> ```bash
> tmux set-option -p @name CINDY      # or MONITOR, CHECKS, RELAY, ALICE, BELLE
> ###
> ```
> 4. Redo what was in it: `robot.sh cindy` (a robot's pane, Part 2), `relay.sh`, `monitor.sh` (4.a), or
>    `source env.sh` (the checks pane).

### 0.8 The monitor page

`monitor.sh` opens http://localhost:8080 in Chrome on this PC once the monitor has started (4.a). One page at a time:
after a restart, close the older tab.

```text
┌ main panel (top left) ────────┐                                         ┌ side panel (right) ────────────┐
│ Stop all (Esc)    N PROBLEMS  │                                         │Assembly|Collisions|Control|Plan│
│ health: a row per robot       │                                         │ PROCESS: Folder, Load, boxes   │
│  base: signal mocap estop     │                                         │ RUN: its chips, New run, robots│
│        battery ctrl           │      3D view: the measured lab,         │ ACTIONS, Step, ◀ mv  mv ▶      │
│  arm:  signal arm <tool>      │      the robots where Motive sees       │ step line, Exec line           │
│  Unlock · Resume · Reconnect  │      them, the process, the ghosts      │ Live IK · Plan · Play          │
└───────────────────────────────┘                                         │ Preview time · Motion time     │
                                                                          │ Exec · Confirm | Cancel        │
                                                                          │ Mark done | Reopen             │
                                                                          │ settings · Advanced            │
 [Plan plots] [Exec plots]  (bottom, minimized)               [Console]   └────────────────────────────────┘
```

Plan holds a tab per planner panel (Base, Motion, Arm). A layout file in `~/.config/husky_monitor/layout.json` may
place these differently, e.g. Plan in a floating window and big Assembly buttons (`doc/lab/plugins.md`, Your own
layout).

- **Assembly** tab, top to bottom: the PROCESS section (`Folder`, `Load`, the boxes `Transfer workaround` and
  `Rhino obstacles`), the RUN section (its chips, `Continue newest` / `Continue match` / `New run`, where each
  robot is), the ACTIONS list, the `Step` slider, `◀ mv` / `mv ▶`, the movement buttons, the step line and the Exec
  line, the buttons Live IK, Check tool0 and Plan, the prompt line, the settings, the closed `Advanced planning`
  folder.
- Buttons are coloured by what they do: teal computes without moving (Live IK, Plan), blue starts a motion (Exec; amber
  while it asks for a second press with other robots unknown), green lets a waiting step go on (Confirm), red stops
  (Cancel), yellow records (Mark done).
- **Control** tab: a tab per robot (`0804`, `0805`, `0806`), each with its Base and its arms: their controllers (the
  `Run` buttons: `Vel` for a base; `Joint`, `Cart`, `Free` for an arm), the drive and jog buttons, the tool buttons
  (Robotiq: `Gripper: Open / Close / Stop`).
- **Plan plots** and **Exec plots** open by themselves when a plan or a servo / compliant part has something to show;
  the control at a bar's right end opens or closes it. The **Console** (bottom right) shows the newest log lines, the
  same as pane monitor.

---

## 1. Prepare  [SETUP — once]

All once, before the first session, except 1.a (every session). Nothing moves. On a PC that has done it before, each
step only confirms.

### 1.a Stop the fake lab  [SETUP — every session]

The fake lab's monitor holds ports 8080 and 8081, so the lab's monitor would refuse to start. (It cannot reach the
robots: its own router on 127.0.0.1:7470, domain 70.) Skip this when the fake lab does not run.

- [ ] In the fake lab's monitor pane: `Ctrl-C` once; then `Ctrl-C` in its sim pane.
  - **Expect:** the fake lab's monitor pane: `(viser) Server stopped`.

**▶ any terminal on this PC**
```bash
tmux kill-session -t fake_lab      # also stops its router and relay
tmux ls
###
```

**Expect:** `tmux ls` lists no `fake_lab:` line (`can't find session: fake_lab` before it: it was not running).

### 1.b This PC on the lab network  [SETUP — once]

- [ ] by hand: this PC cabled to the lab switch: the RobotX PC's port `eno1`; the laptop's USB-ethernet dongle.

**▶ any terminal on this PC**
```bash
ip -4 -brief addr | grep 192.168.0
for h in 192.168.0.115 192.168.0.113 192.168.0.114 192.168.0.28; do   # Cindy, Alice, Belle, the Motive PC
  printf "%s: " $h; ping -c1 -W1 $h >/dev/null && echo up || echo NO REPLY
done
###
```

**Expect:**
- `eno1  UP  192.168.0.201/24` (the RobotX PC) or `enx...  UP  192.168.0.203/24` (the laptop);
- `192.168.0.115: up`, `192.168.0.113: up`, `192.168.0.114: up`, `192.168.0.28: up`.

**If not:**
- no `192.168.0.x` line: check the cable, then `nmcli connection up "Wired connection 1"`.
- an address other than `.201` / `.203`: a port or dongle the router has no slot for. It works, but the robots take no
  time from this PC (1.d); have a slot reserved for its MAC (crl-husky `TIME_SYNC.md`).
- a robot `NO REPLY`: she is off (Part 2), or off the network. The Motive PC: off, or it moved to `.117` (7.h).

**?** Leave the Wi-Fi on: the internet goes through it, `192.168.0.x` through the cable.

### 1.c The Zenoh router on this PC  [SETUP — once]

Every ROS node on this PC meets the robots through this router. Without it a node waits for one, silently; `relay.sh`
and `monitor.sh` refuse to start and say so.

**▶ any terminal on this PC**
```bash
~/Code/ros2_ws_next/src/crl-husky/crl_husky/scripts/zenoh_router.sh --install
systemctl --user is-active crl-zenoh-router
###
```

**Expect:** `Installed /home/<user>/.config/systemd/user/crl-zenoh-router.service`, then `active`.

From now on it starts at login, listens on `127.0.0.1:7447` and this PC's lab address, port 7447, and connects to
every robot's router; off the lab network it retries every 10 s. Installed before (the RobotX PC), it says the same and
keeps running. The fake lab is not affected: its nodes connect only to its own router.

### 1.d Time sync  [SETUP — once]

The robots take their time from the control PCs (`.201`–`.210`), and the health chips flag a robot whose stamps run
more than 5 ms ahead of this PC's clock or 30 ms behind it. As in crl-husky `TIME_SYNC.md` (Control PC): this PC
serves its time and follows the other control PCs, so its own slot comes out of the list.

**▶ any terminal on this PC**
```bash
cd ~/Code/ros2_ws_next/src/crl-husky/crl_husky/config/time_sync
ME=$(ip -4 -o addr | awk '/ 192\.168\.0\./ {split($4, a, "/"); print a[1]; exit}')     # this PC's slot
echo "this PC: $ME"
sudo apt install chrony                     # replaces systemd-timesyncd
sudo cp control_pc_chrony.conf /etc/chrony/conf.d/crl_lab.conf
sudo sed -i "/^server $ME /d" /etc/chrony/conf.d/crl_lab.conf                  # this PC's own slot
sudo sed -i -E '/^pool /{/ prefer/!s/$/ prefer/}' /etc/chrony/chrony.conf     # its own internet first
sudo systemctl restart chrony
grep -c "^server" /etc/chrony/conf.d/crl_lab.conf
chronyc -n sources
###
```

**Expect:**
- `this PC: 192.168.0.201` (the RobotX PC) or `this PC: 192.168.0.203` (the laptop);
- `9`: the other nine control PCs;
- in the sources, one line starting `^*`: an internet server while the Wi-Fi has internet, else another control PC.

**If not:**
- `this PC: ` with nothing after it: this PC is not on the lab network (1.b); the list keeps its own slot. Run the
  block again once it is.
- a sources line with this PC's own address (`^? 192.168.0.201` on the RobotX PC): the list was made for another PC;
  run the block again.

### 1.e ssh keys for the robots  [SETUP — once]

So `robot.sh` and `check_robots.sh` log in without a password (without them `robot.sh` asks for it every time, and
`check_robots.sh` skips the robots' own settings).

**▶ any terminal on this PC**
```bash
for ip in 192.168.0.115 192.168.0.113 192.168.0.114; do ssh-copy-id administrator@$ip; done   # password 12345678
###
```

**Expect,** per robot, one of:
- `Number of key(s) added: 1`: the key is on her now;
- `WARNING: All keys were skipped because they already exist on the remote system.`: it was there already.

**If not:** `ERROR: ssh: connect to host 192.168.0.11x port 22: No route to host`: that robot is off or not on the lab
network. Do it for her once she is up (Part 2): the same line with only her address.

A robot's host key new to this PC (Belle's on the laptop): answer `yes`.

### 1.f Build the workspace (after every pull)  [SETUP — once]

**▶ any terminal on this PC**
```bash
cd ~/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash
python3 -m colcon build --symlink-install         # crl-husky and this package
###
```

**Expect:** the last line, `Summary: N packages finished [...]`, with no `failed`.

**If not:** `Duplicate package names not supported: - husky_assembly_teleop: - src/husky-assembly-teleop -
worktrees/...`: a git worktree inside the workspace. colcon skips a folder with a `COLCON_IGNORE` file in it:
`touch ~/Code/ros2_ws_next/worktrees/COLCON_IGNORE` (there on the RobotX PC since 2026-11-02), then build again.

Editing an existing `.py` file alone needs no build (the install links to the source): restart the monitor. New files,
messages or entry points do.

### 1.g A robot's crl-husky, only when Part 3 says it differs  [SETUP — once]

Skip this on a first pass. Each robot runs the crl-husky in her `~/workspace/src/crl-husky`, copied there by
crl-husky's `upload.sh` (files, no git). Part 3 compares it file by file with this PC's `src/crl-husky`, and only its
answer sends you here:
- **its messages differ** (`crl_husky_msgs`): FAIL. Topics between them may not decode: the monitor would read her
  wrong, or not at all. Bring both to one commit before the test.
- **only its code differs**: NOTE. It runs, but it is not the code the fake lab tested.

**!** Copying overwrites her crl-husky: ask Jakob which side is newer first. Her launch stopped, nothing in her tools.

- [ ] Copy this PC's crl-husky to her:

**▶ any terminal on this PC**
```bash
cd ~/Code/ros2_ws_next/src/crl-husky && ./upload.sh 192.168.0.115      # Cindy; .113 Alice, .114 Belle
###
```

**Expect:** `Uploading tracked files from crl-husky to 192.168.0.115...`, then `Uploaded to 192.168.0.115`.

- [ ] Log in to her, in the same terminal:

**▶ the same terminal**
```bash
ssh administrator@192.168.0.115
###
```

- [ ] Build what she runs, in the shell on her (the prompt reads `administrator@cpr-a200-0806`):

**▶ the same terminal, now on the robot**
```bash
cd ~/workspace && colcon build --symlink-install --packages-select crl_husky_msgs crl_husky
###
```

**Expect:** `Summary: 2 packages finished [...]`.

- Her install links to her source (Cindy's on 2026-11-02): build it the same way, with `--symlink-install`, or
  colcon can trip over its own links. Add `crl_husky_cpp` to the list if Part 3 named a file of it.
- [ ] `exit`: back on this PC; the prompt reads this PC's again (`su@CRLRobotxPC` on the RobotX PC).
  - **!** Type `exit` only in the shell on the robot: in a terminal on this PC it closes that terminal, or that pane.

Then start her launch again (Part 2) and run Part 3 again: pane checks reads `her crl-husky: the same code and messages
as this PC's`.

### 1.h The process on the Drive  [SETUP — once]

The monitor reads the process and records the run in the shared Drive folder directly (the fake lab used a scratch
copy). Everyone with the Drive sees the run.

- [ ] by eye: the terminal is on this PC, not in an ssh shell on a robot: its prompt reads `su@CRLRobotxPC` (the
      laptop: `yijiangh@...`), not `administrator@cpr-a200-...` (then `exit` first). Paste the block as it is.

**▶ any terminal on this PC**
```bash
P=$(bash -c 'source ~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/real_lab/env.sh && echo "$REAL_LAB_DRIVE/$REAL_LAB_PROCESS"')
echo "$P"                       # the process folder, as the scripts find it
ls "$P"
ls -a "$P" | grep -c insyncdl   # files Insync is still downloading
ls "$P/runs" 2>/dev/null        # runs of earlier tests
###
```

**Expect:**
- the folder: `/home/su/Insync/2025-03 Husky Assembly/data_design_study/261103_SRD` on the
  RobotX PC;
- `actions  meshes  process.json  robots  runs  source` (`runs` once a run exists);
- `0`: Insync has finished syncing;
- the names of earlier runs, if any. They are kept; this test starts its own (4.d, 6.0).

**If not:**
- `bash: line 1: /home/administrator/.../env.sh: No such file or directory`: this terminal is logged in to a robot (its
  prompt reads `administrator@cpr-a200-...`): `exit`, then the block again.
- a folder starting `/data_design_study`: no Drive found: Insync, or `REAL_LAB_DRIVE` (7.h).
- a number above `0`: wait for Insync, then again.

---

## 2. Bring up the robots  [ROBOTS ON — nothing moves]

The pendants first (2.a), then each robot's launch in her own pane (2.b–2.d), then Motive and the relay (2.e). Each
step says what a good start prints. Anything else: 2.f, which holds every fix.

### 2.a Pendant first, launch second  [ROBOTS ON — nothing moves]

On **every** arm's teach pendant (Cindy has two, Alice and Belle one each):
- [ ] Power on and release the brakes
  - The pendant shows `Normal`.
- [ ] Set **Remote Control** mode with the Local / Remote switch in the pendant's top-right corner. (No switch there:
      ☰ → Settings → System → Remote Control → Enable, then use it.)
- [ ] The speed slider at the bottom of the pendant at **100 %**.
- [ ] Do not load or start any program by hand. Once the launch runs, its safety sync loads and plays
      `ros_control.urp` itself.
- [ ] Each robot's remote e-stop paired (one husky at a time) and in reach.

**!** Launch only after that. An arm that is off or in Local mode when its launch starts comes up without its
controllers; then restart the launch (2.f b).

### 2.b Cindy  [ROBOTS ON — nothing moves]

- [ ] Both arms on, pendants in Remote (2.a); her remote e-stop in reach.
- [ ] pane CINDY: Enter (`robot.sh cindy` is typed in)
  - **Expect:** pane CINDY:
    - `[cpr-a200-0806] rmw_zenoh_cpp, ROS domain 80, robot 0806`;
    - `real lab: ros2 launch crl_husky crl_arms_and_base.launch.py gripper:=scaffolding_v3`;
    - her launch's `Configured and activated ...` lines; after about 20 s the pane goes quiet.
  - **If not:** 2.f.
  - Her launch starts each arm's driver and controllers, both scaffolding tool drivers, the safety sync (playing
    `ros_control.urp` on each arm) and `pure_pursuit`, and keeps running in this pane.

### 2.c Alice  [ROBOTS ON — nothing moves]

- [ ] Arm on, pendant in Remote (2.a); her remote e-stop in reach.
- [ ] pane ALICE: Enter (`robot.sh alice` is typed in)
  - **Expect:** pane ALICE:
    - `[cpr-a200-0804] rmw_zenoh_cpp, ROS domain 80, robot 0804`;
    - `real lab: ros2 launch crl_husky crl_arms_and_base.launch.py gripper:=robotiq_2F_85`;
    - her launch's `Configured and activated ...` lines; then the pane goes quiet.
  - **If not:** 2.f.
  - Her launch starts the arm driver, the Robotiq's driver and controller, the safety sync and `pure_pursuit`.

### 2.d Belle  [ROBOTS ON — nothing moves]

- [ ] Arm on, pendant in Remote (2.a). She stays parked, her arm stowed.
- [ ] pane BELLE: Enter (`robot.sh belle` is typed in)
  - **Expect:** pane BELLE: `[cpr-a200-0805] rmw_zenoh_cpp, ROS domain 80, robot 0805`, then
    `real lab: ros2 launch crl_husky crl_arms_and_base.launch.py gripper:=robotiq_2F_85`, then her launch's
    `Configured and activated ...` lines.
  - **If not:** 2.f.

Leave all three launches running for the whole session. **!** `Ctrl-C` in a robot's pane stops that robot's arm
drivers, tools and follower: never while she holds a bar.

### 2.e Motive and the relay  [ROBOTS ON — nothing moves]

- [ ] On the Motive PC: the rigid bodies **1840** (Alice), **1850** (Belle) and **1860** (Cindy) streaming; the bar
      assets **B20 = 1020** and **B21 = 1021** on; and **Labeled Markers** on in the streaming settings (the servo fits
      B20's balls; the old repo's `~/Code/ros2_ws/src/husky-assembly-teleop/doc/mocap_bar_pose_test_manual.md`, 0.3).
- [ ] pane RELAY: it has run since `tmux.sh` (Up, Enter if it stopped)
  - **Expect:** pane RELAY: `NatNet client connected.`, then `calibrations at start, Drive root ...` with a block per
    robot's body, then a `Created mocap publisher for rigid body id=...` line per body.
  - **If not:** 2.f.
  - The calibration block names the newest Drive session per robot (e.g. Alice `20260623`, Belle `20261013`,
    Cindy `20261016`), applied (with a warning when over 5 cm / 5° from crl-husky's default).

### 2.f When a launch or the relay does not come up  [ROBOTS ON — nothing moves]

Find the line the pane shows, then do what its row says. Every row was met in the lab or is printed by the scripts.

| the pane shows | why | do |
|---|---|---|
| `real lab: <robot> runs rmw_cyclonedds_cpp on domain 86, not Zenoh, 80.` | her base services run Cyclone, for the old monitor | a) |
| `[ur_ros2_control_node-N] [WARN] ... Failed to bind socket for port 50002 to address. Reason: Address already in useRetrying in 1 seconds`, every second (any port 50001–50004 or 60001–60004) | another program of her launch took that UR port as the random local port of its own connection (Cindy's left arm on 2026-11-02). That arm cannot get its control script, so it cannot move. It does not clear by itself | b). For good: the robots reserve those ports (`net.ipv4.ip_local_reserved_ports=50001-50004,60001-60004`): ask Jakob |
| `[pure_pursuit-1] [WARN] ... mocap stamp N ms in the future: is this robot time synced?` | her clock is behind this PC's: no control PC answered at her boot, so her boot service did not step it forward (Cindy, 2.7 days, on the morning of 2026-11-02). She drives, but her `signal` chips stay red (`every message over 100 ms late`) | c), or live with the red chips |
| `real lab: a crl_husky launch already runs on <robot>, so not starting another:` and the launch it found | one left from a session whose link dropped, or someone else's | hers and working (Part 3 OK): keep it, `exit`. Else stop it as the lines say, `exit`, Up, Enter. Two launches would mean two drivers for each arm |
| a password prompt | this PC's ssh key is not on her | `12345678`; then 1.e for her |
| `ssh: connect to host 192.168.0.11x port 22: No route to host` | she is off, or off the lab network | power her on (2.a); her PC must answer `ping` (1.b) |
| `real lab: this PC is robot ..., not <robot> (...): not launching.` | a wrong address | `exit`; tell me |
| a spawner's `process has died`, or Part 3 says an arm's controllers do not answer | the launch started before the arm was on and in Remote | b) |
| pane RELAY: `real lab: no Zenoh router on this PC (127.0.0.1:7447)` | the router is not installed or stopped | 1.c |
| pane RELAY: `Another mocap relay is already running in this ROS domain` | another PC runs one (the RobotX PC's or the laptop's) | use it, and note whose it is (its calibrations come from that PC's Drive copy); or stop it there, then Up, Enter in pane RELAY |
| pane RELAY: a box `... warning(s), see the lines marked '!'` | a robot's newest calibration is over 5 cm / 5° from crl-husky's default, or refused | note it; under `! refused` that robot keeps crl-husky's default, under `! warning` its session is applied anyway |

**a) Switch a robot to Zenoh** (only when `robot.sh` says she is not on it)

`robot.sh` has left you in a shell **on her**: the prompt reads `administrator@cpr-a200-0806` (Cindy), not
`su@CRLRobotxPC`. `crl-husky-mw-zenoh` is one of crl-husky's robot commands: it exists only on the robots.

**▶ her pane — the shell on her that robot.sh left open**
```bash
crl-husky-mw-zenoh      # asks for the sudo password (12345678), then restarts her base services
###
```

**Expect:** `Switching robot 0806 to ... on domain 80.`, `Restarted: crl-zenoh-router, clearpath-robot (active)` and
`Done. ...`.

**If not:**
- `crl-husky-mw-zenoh: command not found`: this shell is on this PC (prompt `su@CRLRobotxPC`), not on her. Type it
  only where `robot.sh` has left you on her.
- `Zenoh is not set up on this robot yet. Run once: sudo .../install_robot_config.sh`: she never ran Zenoh; ask Jakob.

It rewrites her `/etc/clearpath/robot.yaml` and `setup.bash` and restarts only her base system, not an arm launch.
`crl-husky-mw-cyclone` puts her back for the old monitor (crl-husky README, Still using Cyclone DDS).

- [ ] `exit`: back on this PC (the prompt reads `su@CRLRobotxPC`) → Up, Enter: `robot.sh <robot>` again.

**b) Restart her launch** (only when she holds nothing)
- [ ] her pane: `Ctrl-C` once
  - **Expect:** `real lab: the session on <robot> ended. ...`
- [ ] her pane: Up, Enter: `robot.sh <robot>` again
  - **Expect:** the lines of 2.b–2.d, without the one that sent you here.

**c) Her clock behind** (crl-husky `TIME_SYNC.md`)

A robot's chrony never jumps her clock, it only slews it, and days take weeks. Her boot service `crl-clock-forward`
jumps it forward once at boot, before her base services start, when a control PC answers within 60 s. **!** Never
jump a robot's clock while her base services run: a backward jump drives her base.

- [ ] See what her boot service did:

**▶ any terminal on this PC**
```bash
ssh administrator@192.168.0.115 'journalctl -u crl-clock-forward -b --no-pager | tail -3; chronyc -n tracking | grep "System time"'   # Cindy
###
```

**Expect:** `Clock is N s behind: stepping it forward.` (or no such line: nothing to step), and
`System time     : 0.000... seconds fast of NTP time` (or `slow`): well under a second.

**If not:** `No control PC within 60 s: the clock only slews until the next boot.`: no control PC answered at her boot.
With this PC on the lab network (its chrony serving, 1.d) and nothing in her tools, either:
- reboot her (`sudo reboot` in a shell on her): at boot her service jumps the clock forward; then 2.b again; or
- without a reboot, her launch stopped (b), `Ctrl-C` only): stop her base services, jump, start them again:

**▶ her pane, on this PC**
```bash
ssh -t administrator@192.168.0.115 'sudo systemctl stop clearpath-robot && sudo chronyc makestep && sudo systemctl start clearpath-robot; chronyc -n tracking | grep "System time"'   # Cindy
###
```

**Expect:** `200 OK`, then `System time     : 0.000... seconds fast of NTP time`. Then type `robot.sh cindy`, Enter
(Up now brings back the ssh line).

---

## 3. Check the robots  [CHECK — nothing moves]

One read-only script checks this PC, ROS domain 80 and the robots (about 20 s). It sends nothing to a robot: it pings
them, reads their settings over ssh and asks ROS what runs. What the monitor's health chips show once it runs (rates,
e-stops, batteries, the arms' safety state) is left to them.

- [ ] pane checks: Enter (`check_robots.sh` is typed in). To check only some robots, put their names after it
      (`check_robots.sh cindy`); to run it again, Up, Enter.
  - **Expect:** pane checks: first `[check_robots] checking this PC, ROS domain 80 and alice belle cindy; read-only,
    nothing is sent to a robot`, then every line `OK`, and last `[check_robots] all OK`.
  - **If not:** each `FAIL` line says what to do under it (`->`); the table below says more.

A good start looks like this:

```text
=== this PC ===
  OK    on the lab network: 192.168.0.201 on eno1 (a control-PC slot)
  OK    Zenoh router listening on 7447 (service crl-zenoh-router: active)
  OK    time: chrony synced, reference ...
  OK    port 8080 free for the monitor
  OK    port 8081 free for the monitor
  OK    the process on the Drive: data_design_study/261103_SRD
  OK    the Motive PC 192.168.0.28 answers ping

=== ROS domain 80 (Zenoh) ===
  OK    one mocap relay: this PC's (pane relay)
  OK    no monitor runs yet in domain 80
  OK    Motive streams 1020 1021 1840 1850 1860 (B20, B21 among them)

=== cindy (/a200_0806, 192.168.0.115; pane cindy) ===
  OK    her PC answers ping
  OK    this PC's Zenoh router is linked to hers
  OK    her base services: rmw_zenoh_cpp, domain 80
  OK    rmw_zenoh 0.1.11, as on this PC
  OK    her Zenoh router runs
  OK    her crl-husky: the same code and messages as this PC's
  OK    her launch runs: crl_arms_and_base.launch.py gripper:=scaffolding_v3
  OK    her clock (chrony): Reference ID : ... (192.168.0.201);System time : 0.000... seconds fast of NTP time
  OK    publishing: the arms' joint states, the safety sync, the follower, the base and both tools
  OK    left arm controllers: joint active, compliance loaded and inactive
  OK    right arm controllers: joint active, compliance loaded and inactive
  OK    Motive streams her rigid body 1860
```

What it checks and what a `FAIL` means:

| line | OK means | FAIL or NOTE: cause → do |
|---|---|---|
| on the lab network | this PC holds a `192.168.0.x` address, in a control-PC slot | no address: 1.b. NOTE outside `.201`–`.210`: 1.b |
| Zenoh router | this PC's router listens on 7447 | not installed or stopped: 1.c; `systemctl --user restart crl-zenoh-router` |
| time | chrony runs and is synced | not installed: 1.d. NOTE not synced yet: wait a minute, `chronyc -n sources` |
| port 8080 / 8081 | free for the monitor (or this lab's monitor holds them) | held by another domain's process: the fake lab's monitor (1.a) |
| the process on the Drive | `process.json` found, Insync done | not found: Insync, or `REAL_LAB_DRIVE` (7.h). NOTE still syncing: wait |
| the Motive PC | answers ping | off; or it moved to `.117`: in pane relay `Ctrl-C`, then `REAL_LAB_MOTIVE=192.168.0.117 relay.sh` (7.h) |
| one mocap relay | exactly one in domain 80, this PC's | none: pane relay, Up, Enter. Two or more: stop all but one. NOTE another PC's: fine if it is the lab's |
| monitor | how many run in domain 80 (several may) | nothing to do |
| Motive streams | the robots' bodies and B20, B21 | nothing: Motive not streaming to this PC (the relay's pane says). NOTE a bar missing: its servo aims tool0 only (2.e) |
| her PC answers ping | she is on the network | off (2.a) or off the network: her other checks are skipped |
| linked to hers | this PC's router holds a connection to her router (port 7447) | her router: `crl-husky-zenoh-restart` on her; this PC's: 8.b |
| her base services | Zenoh, domain 80 | Cyclone, or another domain: 2.f a) |
| rmw_zenoh | the same version on her as on this PC | every machine needs the same (crl-husky README, Zenoh): upgrade the older one |
| her Zenoh router | its service runs on her | `crl-husky-zenoh-restart` on her |
| her crl-husky | the same files as this PC's `src/crl-husky` | messages differ: FAIL, 1.g. NOTE code differs: 1.g |
| her launch runs | one `crl_arms_and_base.launch.py` runs on her | none: Enter in her pane (2.b–2.d). Two or more: two drivers for each arm: stop all but one. NOTE another launch: without the follower, Commit cannot drive her |
| her clock | chrony on her follows a control PC | NOTE nothing: chrony not set up on her (ask Jakob) |
| publishing | her arms' joint states, safety sync, follower and base publish; Cindy's two tools too | missing `platform/...`: her base services (`crl-husky-clearpath-restart` on her); the rest: her launch's pane says why |
| the Robotiq's action server (Alice, Belle) | the gripper's action server is up | her gripper driver failed: read her pane; a gripper dead after an e-stop: crl-husky README, Restart |
| arm controllers | Joint active, compliance loaded and inactive (the monitor switches to it only for a compliant part) | no answer: the launch started before the arm was on and in Remote: `Ctrl-C` it, Enter again. Joint not active: the pendant in Remote, its program playing? (the monitor's `arm` chip says why). Compliance active: 7.b. Not loaded: every compliant part would fail at the switch: ask Jakob |
| her rigid body | Motive streams it | her asset off in Motive (2.e) |

**?** Without ssh keys (1.e) the robots' own settings are skipped with a NOTE; the rest is checked.

---

## 4. Check the monitor  [CHECK — nothing moves]

### 4.a Start it  [CHECK — nothing moves]

- [ ] pane monitor: `Enter` (*monitor.sh* is typed in)
  - **Expect:** pane monitor:
    - *real lab: open http://localhost:8080 (phone: http://192.168.0.201:8081); log: .../husky_real_lab/logs/monitor\_\<date>.log*
      (*.203* on the laptop);
    - *robot a200-0806 left_ur_arm: URDF matches its UR calibration (ur_calibration_left.yaml)*, one per arm (four);
    - after about 10 s, *started in ... ms*.
  - **If not:**
    - *port 8080 ... taken* from the monitor itself: the fake lab (1.a).
    - *no Zenoh router on this PC*: 1.c.
  - Then the health chips turning green, one line each (*[health/chips] a200-0806 arm: arm red -> green: sync:
    reporting*), and PyBullet's own lines (8.b). All the start lines: 7.c.
  - At *started in ... ms*, Chrome opens the page (the default browser on a PC without Chrome).

### 4.b First look  [CHECK — nothing moves]

- [ ] Chrome: the page *monitor.sh* opened (else http://localhost:8080); keep that one page open
- [ ] by eye: the 3D view shows the measured lab (cabinet, tables, tripods, trusses, the orange border) and the three
      robots where Motive sees them, their arms as measured.
- [ ] by eye: health, top left: every base shows *signal*, *mocap*, *estop*, *battery NN%*, *ctrl*; every arm
      *signal*, *arm* and its tool (*robotiq* on Alice and Belle, *scaffolding_v3* on each of Cindy's arms)
  - No *sim* chip: nothing is simulated. Belle's *mocap* may be amber: her body's marker error was 1.2 mm in the lab
    (hover it).
  - **If not** (hover the chip):
    - *estop* red: that robot's remote e-stop is pressed (release it before anything moves).
    - a base's *ctrl* red: its *platform_velocity_controller* does not run; the follower needs it (4.d).
    - an arm's *arm* red: its safety sync says why: the pendant in Local, the program not playing, a protective
      stop (`Unlock` under Health, once you know why), or `Stop all` (`Resume`).
    - *signal* amber or red, *stamps ... in the future (robot clock ahead; time sync?)* or
      *every message over ... ms late*: the clocks (1.d), or a slow Wi-Fi link.
- [ ] Optional, the phone: *http://\<this PC>:8081* (*192.168.0.201* or *.203*) on the robot Wi-Fi
      (*doc/lab/phone_manual.md*): `Stop all`, `Health`, and the `Exec` tab's `Confirm` / `Cancel` next to the robots.

### 4.c Load the process  [CHECK — nothing moves]

- [ ] Tab `Assembly`, top section PROCESS: dropdown `Folder` reads
      *data_design_study/261103_SRD* → button `Load` once; wait 5–10 s
  - **Expect:** pane monitor: *loaded .../261103_SRD in ...; 76 actions, 269 movements*, a
    warning *a process robot differs from the configured one: ...*, then *no run yet: the first mark starts one* (or
    *continuing run \<date>\_\<time>*: 4.d).
  - `Folder` lists every process and old export on the Drive, newest first; the monitor starts with this one picked.
  - The warnings are right: this export used the old UR frames; the monitor plans with the configured robots.
  - Every `Load` draws the whole process again, and a page opened later first receives about 200 bodies (20 s): don't
    click `Load` twice.
- [ ] by eye: the page shows
  - in PROCESS, a green chip *261103_SRD*, *76 actions · 269 movements*, and an amber *robots
    differ*;
  - ACTIONS *1/76*: *1. B20_J · cindy · 5 mv · pending* on top, in blue, then *2. B20_R · cindy · 3 mv*, *3. B21_J*,
    *4. B21_H · alice · 4 mv*, *5. B21_R*, *6. B22_J*, *7. B22_R*, every row ending in *· pending* (*· done*, in
    green, after an earlier run: 4.d);
  - in MARK DONE, *current B20_J · B20_J_transfer (4/6)*; in RUN, *no run yet*, *0/76 done*, and *alice*, *belle*,
    *cindy* each *live now*.
- [ ] The two boxes under `Load`: box `Transfer workaround`: ticked (B20_J is listed as *J_transit*, *J_M1*, *J_M2*,
      *J_M4*, *J_M5*, 5 mv); list `Rhino obstacles`: `ghost`, with the grey chip *Rhino obstacles: drawn only*. Leave
      both.

The process panel picks the run's current step. Click an action row or a movement button, drag the `Step` slider or
click `◀ mv` / `mv ▶`: the step under the buttons follows, except while an Exec has the robots' leases. `▲` / `▼` (row
*Scroll*) scroll the action list a page and leave the step as it is; the list follows the step again once it changes.
The boxes under STEP only change what the 3D view draws:
- `Process overlay`: the process at this movement over the live scene: the planned robots in blue with their tools and
  held bars, and the Rhino obstacles while they don't collide. Unticked: the live scene only.
- `Ghost robots`: those blue robots see-through (ticked, at start) or solid.
- `Unbuilt bars`: also the bars not built yet at this movement, with their joints, see-through.
- `Rhino bar lines`: every bar's centre line and name, pink, where the design puts it.

### 4.d The settings, the run, the base controllers  [CHECK — nothing moves]

The Assembly tab: RUN sits between PROCESS and ACTIONS, near the top; below the big `Exec` button come EXEC, MARK
DONE (the current action's `… done` and `Reopen`) and SETTINGS.

- [ ] SETTINGS: as the monitor starts them; leave them so.

| setting | at start |
|---|---|
| `Ignore built bars`, `Visual diagnosis` | unticked |
| `Insert split mm` | *5* |
| `Spiral` | unticked and greyed; live only on an insert with two mating joints |
| `Spiral s`, `Spiral turns` | *20*, *4* |
| `Servo by bar` | ticked on a ground bar's action (*B20_J*, *B34_J*, *B46_J*); greyed and unticked on any other, which always servos on tool0 |
| `Retreat live axis` | ticked |
| `Retreat compliant`, `Dry run (every check, nothing sent)` | unticked |

- [ ] RUN, between PROCESS and ACTIONS: its first chip names the run the marks go to.
  - *no run yet*: nothing to do; the first mark starts one.
  - a run's name, e.g. *20261029_155758*, with *3/76 done* (MARK DONE reads *continuing run 20261029_155758*): an
    earlier test on these files. Start a new one: button `New run` (row *Run*); it then reads `New run?`:
    press it again.
    - **Expect:** pane monitor: *started run .../runs/\<date>\_\<time>*.
    - RUN shows the new run and *0/76 done*; every action row ends in *· pending*. The old run's folder stays.
    - No time limit between the two presses; a second press within half a second counts as a double click and is
      ignored.
  - Part 5's tests are recorded in this run; 6.0 starts another one for the assembly.
- [ ] Tab `Control` › `0806` › Base, and `0804` › Base: the controllers line shows *platform_velocity_controller*
      running (the base's *ctrl* chip green)
  - **If not:** button `Vel` (row *Run*). The follower drives only with it.

---

## 5. Tests before the assembly  [TEST — base, arm or gripper move, no bar]

Before Cindy holds a bar: each base that drives in this test under its follower once, and Alice's compliant handoff in
the air (it has never run on a real arm with the new monitor; in the fake lab it ran on a simulated Robotiq only). No
bar; clear floor and workspace.

**!** Do not Mark done here: these are tests.

### 5.a Each base under its follower: a short drive (Cindy, then Alice)  [TEST — base, arm or gripper move, no bar]

- [ ] `Plan` › `Base`: dropdown `Robot`: *a200-0806* → button `Load: Robot`
  - The fields `Target x y yaw°` show where she stands (world x, y in metres, yaw in degrees).
- [ ] fields `Target x y yaw°`: a point about 0.5 m away on open floor (change x or y, whichever way is clear) →
      button `Path: Plan`
  - After about 2 s the panel shows *Lattice (hybrid A\*) ...: ... m, ... s* and the path on the floor.
- [ ] by eye: the path clear of people and of things the scene does not know (cables, boxes, a cart)
- [ ] hand on Cindy's e-stop → button `Commit`
  - **Expect:** pane monitor: *base path N sent to a200-0806: L m, ... waypoints*, then, when she stops,
    *base path N on a200-0806: done*.
  - **If not:** she does not start: the base's *ctrl* chip (4.d), her follower in her pane (*pure_pursuit*). She
    drives off the path: `Stop all`; note what she did.
  - She drives it smoothly and stops. `Control` › `0806` › Base then reads about the target (the fake lab's follower
    stopped within about 1 cm and 2°): note the real numbers.
- [ ] The same back: the old x, y, yaw → button `Path: Plan` → button `Commit`
- [ ] The same for Alice: dropdown `Robot`: *a200-0804*.

### 5.b Alice's gripper by hand  [TEST — base, arm or gripper move, no bar]

- [ ] `Control` › `0804` › the arm's TOOL section: button `Gripper: Open`
  - The jaws open; the tool line reads about *0.43*.
- [ ] button `Gripper: Close`
  - The jaws close; about *0.79* (an empty close).
- [ ] button `Gripper: Open` again: the handoff below starts with open jaws.

### 5.c Alice's handoff in the air (the jaws close on nothing)  [TEST — base, arm or gripper move, no bar]

- [ ] `Assembly` › ACTIONS: row `4. B21_H` → movement button `H_M3` (*B21_H_close*, the old monitor's code)
  - The step line names *B21_H_close*; the Exec line says Alice closes her gripper with the compliant handoff.
- [ ] settings: box `Dry run`: ticked → button `Exec` (and button `Confirm` if it asks)
  - **Expect:** pane monitor: *DRY RUN a200-0804 ur_arm: handoff: passed every check (anchor X mm, Y° from the URDF's
    tool0), NOT SENT*.
  - The anchor is how far her arm's own TCP is from tool0 by the URDF: under 10 mm and 2° the handoff goes compliant;
    above, it closes stiff and says so. Note X and Y.
- [ ] box `Dry run`: unticked → button `Exec`
  - **Expect:** pane monitor: *alice (a200-0804): B21_H_close — CLOSE + compliant handoff on ur_arm: the arm goes
    COMPLIANT before the close (gives way ≤ 10 mm); jaws 0.8, effort 0.1, ≤ 30 s; then joint control. ...*
- [ ] hand on Alice's e-stop → button `Confirm`
  - **Expect,** pane monitor, in order:
    - *a200-0804 ur_arm: handoff: force sensors zeroed: |F| ur_arm ... -> ...*;
    - *a200-0804 ur_arm: handoff: anchor X mm, Y° from the URDF's tool0 (limit 10 mm, 2°)*;
    - *ur_arm: cartesian_compliance_controller running (...)*;
    - *a200-0804 ur_arm: handoff: closing to 0.800 rad, effort 0.10, the arm compliant around its anchor ...*;
    - *ur_arm back under scaled_joint_trajectory_controller (... s)*;
    - *Exec B21_H_close: done*.
  - **If not:**
    - *... no handoff, the gripper closes with the arm STIFF*: the anchor check failed: note the numbers (a constant
      offset along tool z: the pendant's tool setting).
    - *the arm moved ... mm from its anchor (limit 10 mm)*, or the arm moves when it goes compliant: e-stop; note how
      far and which way.
    - a switch refused: nothing closed; check the controllers (Part 3), then again.
  - The arm stays where it is while the empty jaws close. The line under the buttons then offers Mark done for
    B21_H: leave it, this is a test.
- [ ] `Control` › `0804`: button `Gripper: Open`
- [ ] Optional, a `Cancel` test: movement button `H_M2`, then `H_M3` again (a step's parts count as done until another
      step is picked) → button `Exec` → button `Confirm` → button `Cancel` while the jaws close
  - The fingers stop and the arm holds where it is, still compliant (`Control` › `0804` shows *Cart*; a cancel sends no
    controller switch). `Control` › `0804`: button `Joint` (row *Run*), then button `Gripper: Open`.

---

## 6. The assembly  [REAL RUN — recorded]

### 6.0 A run of its own  [REAL RUN — recorded]

- [ ] RUN: button `New run` (row *Run*), then again when it reads `New run?` (no time limit; a second press
      within half a second counts as a double click and is ignored)
  - **Expect:** pane monitor: *started run .../runs/\<date>\_\<time>*.
  - RUN reads *0/76 done*, every action row *· pending*; the test run's folder stays. Once its robot is live the run
    selects the first pending action's first step: *B20_J · B20_J_transfer*.

Then the seven actions in order, each with its procedures (6.1–6.4) and what differs for it (6.5).

### 6.1 Procedure B — drive a base (base planner)  [REAL RUN — recorded]

- [ ] `Plan` › `Base`: dropdown `Robot`: the action's robot (*a200-0806* Cindy, *a200-0804* Alice) → button `Load: Step`
      → button `Path: Plan`
  - The panel shows *target +x +y +yaw°* (the design's base for the current step; 6.5 lists them), then after about
    2 s *Lattice (hybrid A\*) (straight): L m, T s*, and the path on the floor.
  - **If not:** the panel says *start is in collision ...* / *target is in collision ...*: read the pair. A held bar
    counts: `Load Step` on the action's first movement (the one with empty tools).
- [ ] by eye: the path clear of people and of things the scene does not know
- [ ] hand on the robot's e-stop → button `Commit`
  - **Expect:** pane monitor: *base path N sent to a200-0806: L m, ... waypoints*, then *base path N on a200-0806:
    done*.
  - **If not:** *base path N on a200-0806: timed out: still following* (the fake lab's follower sometimes never
    started a short path): button `Path: Plan` and button `Commit` again. To stop a driving base, use `Stop all`, not
    the panel's `Cancel` right after a `Commit` (8.b).
  - She drives (in the fake lab about twice the planned time). `Control` › `0806` › Base then reads about the target.
- **!** Never drive Alice while she holds B21.

### 6.2 Procedure J — jointing a bar (Cindy)  [REAL RUN — recorded]

*\<bar>* = the bar. Four steps: the transfer, the bar placed and clamped by hand (one prompt), the servo, the insert.
A ground bar (B20) has a fifth: the foundation fixed by hand.

**a) Base:** Procedure B (6.1) with Cindy.

**b) Transfer** (*\<bar>_J_transfer*)
- [ ] by eye: the step line reads *\<bar>_J_transfer (transfer_free) · then: \<bar>_J_place_bar, \<bar>_J_grasp ·
      skipped: \<bar>_J_load*, and the Exec line *Exec: cindy (a200-0806) moves left_ur_arm, right_ur_arm; then
      waits for \<bar>_J_place_bar + \<bar>_J_grasp by hand; then servos left_ur_arm, right_ur_arm onto the insert
      start*
- [ ] button `Live IK: insert start` (about 5 s)
  - **Expect:** pane monitor: *check goal \<bar>_J_transfer: goal found (ssik, rank 1 of 64), max |goal - authored| X
    rad*; the run's first record also logs *started run .../runs/\<date>\_\<time>*.
  - **If not:** *no collision-free IK ...*: the base stands too far from (or too close to) the design's base, or a
    drawn pair collides: drive again (6.1), then `Live IK` again.
  - The blue process robot turns to Target and shows the arms at the insert start on the live base. X grows with how
    far the base stands from the design's.
- [ ] button `Plan` (15–40 s)
  - **Expect:** pane monitor: *transfer_free solved by tamp.plan_free_dual_arm: N waypoints, 1 tries*.
  - The page shows the chip *plan ready* and *min T s by the arms' limits*; the Plan plots window opens bottom
    left: every joint along the preview, and the largest joint step per row.
- [ ] button `Path: Play`, or drag slider `Preview time`
  - It starts at the live arms and ends with the empty tools at the insert start, clear of the floor, the built
    bars and Alice.
- [ ] slider `Motion time`: higher = slower (first run) → hand on Cindy's e-stop → button `Exec`
  - **Expect:** pane monitor: *cindy (a200-0806): \<bar>_J_transfer — move left_ur_arm, right_ur_arm along the planned
    path (Rhino obstacles not checked), T s, largest joint travel D°* (a warning over 60°), then
    *\<bar>_J_transfer: arrived, end within ...°*.
  - Both arms move at once; then the prompt for the bar.

**c) The bar, by hand**
- [ ] by eye: the prompt for the bar
  - **Expect:** pane monitor: *\<bar>_J_place_bar + \<bar>_J_grasp — Operator places the bar by hand; clamp it: tighten
    the gripper screws of left_ur_arm, right_ur_arm with the tools' buttons until they stall, then Confirm …*
  - The monitor sends the tools nothing here.
- [ ] by hand: place the bar in both tools → buttons on the tools: clamp it, until each gripper screw stalls
  - **!** One button at a time: B1 and B2 together put that arm in free drive.
- [ ] by eye: the bar sits in both jaws and is held
- [ ] button `Confirm` (here or on the phone's `Exec` tab)

**d) The servo** (runs by itself after `Confirm`)
- [ ] by eye: the servo's first pass, waiting for `Confirm`
  - **Expect,** pane monitor:
    - the feedback line: B20 *[servo] feedback: the mocap bar bars/B20 (converges at 0.5 mm / 0.1 deg).*; B21
      *[servo] feedback: tool0 (0.2 mm); the mocap bar bars/B21 is recorded alongside.*; B22 *... no mocap bar
      recorded (bars/B22 carries no mocap markers).*;
    - *[servo 0] tool0 pos L=a R=b mm, rot ... deg* (a few mm: the base gives under the arms);
    - the prompt *\<bar>_J_transfer — SERVO 1/8 onto the insert start (tool0, or the mocap bar B20): ... Confirm to
      move; Cancel stops and records.*
  - **If not** (B20): a WARNING prompt first, *B20_J_transfer — SERVO by bar: no usable mocap fit of B20 (asset 1020):
    \<why>. Tool0 servo (Confirm): servo tool0 onto the authored targets of the insert start instead (...). Give up
    (Cancel): stops, recording.*, in a warning window beside the main panel. B20's balls are not streamed (*no marker sample*: 2.e, check the Motive asset 1020 and Labeled Markers)
    or the fit failed (*\<why>* says what). `Give up`, fix Motive, `Exec` again; or `Tool0 servo` to place B20 by tool0 (the
    ground bar then stands where the flanges put it, not where its balls say: only on purpose).
  - The Exec plots window opens with the servo's plots (one point per pass). The exec section's last line and the
    Exec plots chip say which phase the pass is in (*pass 1/8: waiting for Confirm*, *moving (3 s)*, *settling 2 s*,
    *measuring*); under each plot stand the last pass's numbers.
- [ ] slider `Preview time`: scrub the pass (a few mm with the bar held) → button `Confirm`
  - **Expect,** pane monitor: each pass *### SERVOING iteration k/8: aiming ...*; at the end one of
    - *### SERVOING converged at iter k on tool0 (< 0.2 mm)* (B20: *... on the mocap bar: dpos ... mm, dang ... deg
      (< 0.5 mm / 0.1 deg)*),
    - *### SERVOING stopped at iter k: nothing left to plan (correction L ... mm, R ... mm, under 0.2 mm)*,
    - *### SERVOING ran all 8 iterations without converging.* (a warning);

    then *Servoing data saved to \<run>/servo/\<action>/\<movement>.\<n>.json (its .png beside it)* and
    *Exec \<bar>_J_transfer: done — every part done*.
  - **If not:** *### SERVOING stopped at iter k: a200-0804 ur_arm moved within the last 2 s* (or *... is running a
    trajectory*): another robot moved, which the monitor did not send (her pendant, free drive). Keep every other
    robot still, wait 2 s, then button `Exec` again: the step's done parts are skipped, so it starts again at the
    servo's first pass, waiting for `Confirm`. `Confirm` alone does nothing then: the prompt has ended.
  - Later passes run alone unless one travels over 5° (*SAFEGUARD servo k/8: ...*: scrub, `Confirm`).
  - Each pass also logs the readings (*[servo k] tool0 pos ...*, *arms vs last commanded waypoint*, *load ...*; B20
    and B21 *bar bars/B2x: mocap-vs-design ...*), and the end *### bar pose at termination (bars/\<bar>): ...*.
  - B20 converged on the mocap bar also logs *[servo] the mocap-bar correction is carried into the plan of
    B20_J_insert.*
  - All 8 passes without converging: the old monitor on B20 left the right flange 1–3 mm off, the bar as mounted by
    hand; harmless. The exec section's last line keeps *servo done: ...*.

**e) Insert** (*\<bar>_J_insert (insert)*)
- [ ] button `Plan` (about 3 s)
  - **Expect:** pane monitor: *insert solved by tamp.plan_constrained_dual_arm_linear: 4 waypoints* (B20 after its
    mocap-bar servo: *[insert plan] mocap-bar correction carried: ...* first).
  - It starts where the servo left the arms.
- [ ] optional, the first insert of its kind on the robot (B21, B22): box `Dry run`: ticked → button `Exec`
  - **Expect:** pane monitor: *DRY RUN a200-0806: insert: every compliant target passed the arm's checks, NOT SENT*.
  - Then box `Dry run`: unticked.
- [ ] hand on Cindy's e-stop → button `Exec`: the insert runs at once
  - **Expect:** pane monitor: the lines of the table below, for the bar.
  - **If not:** *NO_STALL* after about 31 s: the screw never stalled (the bar not in its joint?): note the line; the
    arms are back under joint control. *ARMS MAY BE LEFT UNDER cartesian_compliance_controller*: 7.b at once.
  - **!** Watch the bar enter its joint, and every held bar stay in its jaws.

| bar | **Expect:** pane monitor | note |
|---|---|---|
| ground bar (B20) | *B20_J_insert — RIGID insert of a ground bar, no tighten, 5.0 s, largest joint travel ...*; *insert: RIGID_DONE: ground bar in, no tighten; the flanges stopped ... mm from the planned end; peak ...*; then the servo's prompt *B20_J_insert — SERVO 1/8 onto the insert end (the mocap bar B20): ...*; after it *Exec B20_J_insert: done* | joint control throughout, no screw turns. The servo after the insert runs in the same Exec: scrub, Confirm; straight lines only, each pass with *force change L ... R ... N* on the panel line. It stops by itself when a flange's force changes by over 20 N (*the force on an arm changed by ... N since the first reading (limit 20 N); the bar may be pushed against something*) |
| one-mate (B21) | *B21_J_insert — INSERT: rigid 10.0 mm (3.3 s), then COMPLIANT 5.0 mm (straight line); the joint screw of left_ur_arm TIGHTENS ...*; *left_ur_arm joint screw STALLED at ...; left_ur_arm back under joint control, right_ur_arm finish the line under compliance*; *insert: STALLED: left_ur_arm joint screw STALLED; peak ...* | the rigid part, the switch (*cartesian_compliance_controller running*), the compliant line; after the stall the right arm settles against the force (at most 3 s) and goes back too. No servo after it: it is not a ground bar |
| two-mate (B22) | as B21, with *the joint screw of left_ur_arm + right_ur_arm TIGHTENS ...* | it ends when both screws have stalled. No servo after it |

**f) Fix the foundation** (ground bar only: *B20_J_fix_foundation*)
- [ ] by eye: the step line reads *Operator fixes the foundation (operator)*, and the Exec line *Exec: cindy
      (a200-0806) waits for B20_J_fix_foundation by hand*
- [ ] button `Exec`
  - **Expect:** pane monitor: *B20_J_fix_foundation — Operator fixes the foundation by hand, then Confirm (the robot
    panels and their free drive refuse while this step has the robots' leases; Cancel to use them).*
  - The monitor sends the robots nothing here; Cindy keeps holding B20.
- [ ] by hand: fix the foundation to the ground
- [ ] button `Confirm` (here or on the phone's `Exec` tab)
  - **Expect:** pane monitor: *Exec B20_J_fix_foundation: done — every part done*.

**g) Mark**
- [ ] by eye: the line under the buttons says *every step of \<bar>_J done: Mark done below* → button `<bar>_J done`
  - **Expect:** pane monitor: *\<bar>_J marked done in run \<date>\_\<time>; ...*
  - The row turns green *· done*, the count goes up by one, and the current step moves to the next action.

### 6.3 Procedure R — release (Cindy)  [REAL RUN — recorded]

Two steps: the retreat (the ungrasp runs with it), then home.

**a) Retreat** (*\<bar>_R_retreat (retreat_linear) · with: \<bar>_R_ungrasp*)
- [ ] button `Plan`
  - **Expect:** pane monitor: *[retreat live] \<bar>_R_retreat: goals = the live tool0 poses moved 50 mm back along
    their own -z (encoders): ...*, then *retreat_linear solved by tamp.plan_dual_arm_linear_independent: N waypoints*.
  - **If not:** a warning that a tool is over 20° from the Rhino axis: check the tool before `Exec`.
  - It plans from where the arms are. If the full 50 mm line is blocked it plans a shorter one (*planned a SHORTER
    one: 35 mm back*), down to 25 mm.
- [ ] optional, the first retreat on the robot (B20_R): box `Dry run`: ticked → button `Exec`
  - **Expect:** pane monitor: *DRY RUN ...: retreat: the screw-out passed every check, NOT SENT; ...*
  - Then box `Dry run`: unticked.
- [ ] hand on Cindy's e-stop → button `Exec`: it runs at once
  - **Expect,** pane monitor, in order:
    - *\<bar>_R_retreat — RETREAT, stiff: the M1 screws of left_ur_arm + right_ur_arm LOOSEN while both tools back
      straight out 12.0 mm at 2.40 mm/s (5.0 s, joint controller), then 38 mm rigid (T s). ...*;
    - *retreat: LOOSEN the gripper screws of left_ur_arm + right_ur_arm*;
    - *screw-out done after 5.x s (gripper motors ..., still loosening); now the rest of the way out.*;
    - *retreat: RETREATED: retreated 50 mm; every check passed; the gripper motors stopped (...)*;
    - *Exec \<bar>_R_retreat: done — every part done*.
  - **If not:**
    - *check FAIL* lines: the checks only report, the tail runs anyway; note every one.
    - *grippers: ... LOOSENING (want STALLED = fully open)*: a gripper screw did not open fully (in the fake lab the
      simulated screws never stall, so this check always failed there; on the real tools it should pass): look at
      the tool before the next move.
    - *contact: |F| ... N (limit 20 N)*: a tool still pushed on the bar. *break-out: ... short of its end*: the arm
      sagged or caught.
    - refused, the arms over 0.57° from the plan's start: `Plan` again.
  - **!** Watch both tools let go before they back off.
  - The checks (start, both tool0 paths, joints, speed) print *PASS* / *FAIL* before anything is sent; a line each time
    the gripper motors change (*screw-out t=0.3s: gripper motors L/R LOOSENING/LOOSENING*). No force zero, no
    controller switch: the whole step runs under the joint controller. The Exec plots show the part's force and
    torque.

**b) Home** (*\<bar>_R_home (free_home)*)
- [ ] button `Go: mv ▶` (the process section): the step line reads *\<bar>_R_home (free_home)* → button `Plan` (20–40 s)
  - **Expect:** pane monitor: *free_home solved by tamp.plan_free_dual_arm: ...*
- [ ] button `Path: Play` → slider `Motion time` → hand on Cindy's e-stop → button `Exec`
  - **Expect:** pane monitor: *\<bar>_R_home — move left_ur_arm, right_ur_arm along the planned path ..., T s, largest
    joint travel D°*, then *\<bar>_R_home: arrived, end within ...°*.

**c) Mark**
- [ ] by eye: *every step of \<bar>_R done* → button `<bar>_R done`
  - **Expect:** pane monitor: *\<bar>_R marked done in run \<date>\_\<time>; ...*

**!** Mark done accepts an action with steps left: wait for *every step of ... done* before clicking it, or the next
base plan refuses because an arm still stands at the bar.

### 6.4 Procedure H — hold (Alice, B21)  [REAL RUN — recorded]

Four steps: approach (and its servo), open, straight in, close with the compliant handoff.

**a) Base:** Procedure B (6.1) with Alice (*a200-0804*).

**b) Approach** (*B21_H_approach*)
- [ ] button `Live IK: linear start` (the first plan for Alice builds her planning cell, about 7 s)
  - **Expect:** pane monitor: *check goal B21_H_approach: goal found (ssik, rank 1 of 8), max |goal - authored| X rad*.
- [ ] button `Plan` → button `Path: Play`
  - **Expect:** pane monitor: *support_approach solved by arm.plan_arm: N waypoints*.
  - The preview ends with the jaws in front of B21, clear of Cindy's arms and tools.
- [ ] hand on Alice's e-stop → button `Exec`
  - **Expect:** pane monitor: *alice (a200-0804): B21_H_approach — move ur_arm along the planned path ..., T s,
    largest joint travel D°*, *B21_H_approach: arrived, end within ...°*, then the servo's prompt
    *B21_H_approach — SERVO 1/8 onto the approach pose (tool0): ...*.
  - Before the prompt: *[servo] feedback: tool0 (0.2 mm); no mocap bar recorded (no bar held).* and *[servo 0] tool0
    pos arm=X mm*.
- [ ] slider `Preview time`: scrub → button `Confirm`
  - **Expect:** pane monitor: *### SERVOING converged at iter k on tool0 (< 0.2 mm)* or *... nothing left to plan
    ...*, then *Exec B21_H_approach: done — every part done*.

**c) Calibration check**
- [ ] by eye: the gripper stands about 100 mm in front of B21, centred on it
  - **If not:** off by more than 1–2 cm sideways: stop. Alice's base calibration is off (the relay's table, 2.e, names
    the session it applied); recalibrate her before going on.

**d) Open** (*B21_H_open*)
- [ ] button `Exec` → the prompt *B21_H_open — OPEN the gripper (0.426). Confirm to send.* → button `Confirm`
  - **Expect:** pane monitor: *B21_H_open: at 0.426 rad*.

**e) To grasp** (*B21_H_to_grasp*)
- [ ] button `Plan` → button `Path: Play` → hand on Alice's e-stop → button `Exec`
  - **Expect:** pane monitor: *B21_H_to_grasp — move ur_arm ..., 5 s, largest joint travel ...°*, then *arrived, end
    within ...°*.
  - The open jaws slide onto B21 (22° of joint travel in the fake lab).

**f) Close and handoff** (*B21_H_close*)
- [ ] button `Exec` → the handoff prompt (5.c) → hand on Alice's e-stop → button `Confirm`
  - **Expect:** pane monitor: the lines of 5.c, ending with *Exec B21_H_close: done*.
  - The arm is compliant for the whole close, giving way while the jaws centre on B21; the gripper stalls on the bar;
    back under joint control.
  - **!** Watch Cindy's tools: they must not be pulled.

**g) Mark**
- [ ] button `B21_H done`
  - **Expect:** pane monitor: *B21_H marked done in run \<date>\_\<time>; ...*
  - The page: *4/76 done*, the current step *B21_R · B21_R_ungrasp (1/3)*.

### 6.5 The seven actions  [REAL RUN — recorded]

| # | action | procedures | what differs |
|---|---|---|---|
| 1 | *B20_J* | B (Cindy), J, ground bar | base target *+1.116 -1.516 +90.0°*, about 3.5 m from her park pose in a straight line. The servo steers on the mocap bar (`Servo by bar`; B20's balls must stream, 2.e). The insert is rigid, then the servo after the insert, then the foundation by hand (6.2 f). Mark: *1/76 done*, current *B20_R · B20_R_ungrasp (1/3)* |
| 2 | *B20_R* | R | the first retreat on a real robot: the `Dry run` first. Mark: *2/76*, current *B21_J · B21_J_transfer (4/5)* |
| 3 | *B21_J* | B (Cindy), J, one-mate | base target *+1.919 -1.578 +135.7°*, about 0.8 m. The servo on tool0, B21's mocap fit recorded alongside. The first compliant insert: the `Dry run` first. The left joint screw tightens into B20. Mark: *3/76*, current *B21_H · B21_H_approach (1/4)* |
| 4 | *B21_H* | B (Alice), H | base target *+0.875 -0.236 -8.8°*, about 1.7 m from her park pose. Mark: *4/76*, current *B21_R · B21_R_ungrasp (1/3)* |
| 5 | *B21_R* | R, Alice holding B21 | B21 is clamped by Alice: the ungrasp releases Cindy's tools from it. In the retreat's preview, check that both tools back off clear of Alice's gripper. Mark: *5/76* |
| 6 | *B22_J* | B (Cindy), J, two-mate | base target *+1.242 -1.591 +80.0°*, a short path. The transfer's plan from the retreat pose took up to 110 s in the fake lab; *no path in 120 s*: `Plan` again. The servo on tool0 (B22 has no mocap balls). Both joint screws tighten, into B21 and B20. **!** Watch B21 while B22 screws in: Alice holds it, and it must not be pushed out of her jaws. Mark: *6/76* |
| 7 | *B22_R* | R | Mark: *7/76*, and the current step is *B34_J*: the first three bars are built, and the next bar's robot is Cindy again |

### 6.6 Optional: Alice lets go of B21  [REAL RUN — recorded]

After action 7, Alice still clamps B21; in the process she lets go at entry 17 (*B21_HR*), after B34–B37. To end the
test here, run her let-go out of order:

- [ ] `Assembly` › process: button `Go: action ▶` until the step line names *B21_HR* (nine clicks from *B34_J*; or
      scroll the ACTIONS rows with `▼` and click its row) → movement button `HR_M0` (*B21_HR_open*)
      → button `Exec` → button `Confirm`
  - **Expect:** pane monitor: *B21_HR_open: at 0.426 rad*.
  - B21 stays where it is: it hangs on its joints to B20 and B22.
- [ ] movement button `HR_M1` (*B21_HR_retreat*) → button `Plan` → button `Path: Play` → hand on Alice's e-stop
      → button `Exec`
  - **Expect:** pane monitor: *B21_HR_retreat: arrived, end within ...°*.
  - **If not:** the plan refuses because of a bar that is not built yet: the planning scene takes the design's state
    at entry 17, B34–B37 included. Box `Ignore built bars`: ticked (B21, the action's own bar, still counts), `Plan`
    again, and untick it afterwards.
  - The open jaws slide off B21, back to the approach pose (100 mm).
- [ ] movement button `HR_leave` (*B21_HR_leave*) → button `Plan` → button `Path: Play` → button `Exec`: her arm
      away from the structure
- [ ] Optional: button `B21_HR done` (the run then says B21 is let go), and drive her back: Procedure B with
      button `Load: Park` instead of `Load Step`.

### 6.7 Stop

1. pane monitor: `Ctrl-C` once. **Expect:** *(viser) Server stopped* (the plugins send their final holds first).
2. Each robot's pane: `Ctrl-C` her launch, only when she holds nothing (Alice clamped on B21: let her go first,
   6.6, or leave her launch running). **Expect:** *real lab: the session on \<robot> ended ...*.
3. pane relay: `Ctrl-C`.
4. `tmux kill-session -t real_lab`.
5. Pendants: stop *ros_control.urp* before switching a pendant back to Local (crl-husky README, Startup). To power a
   robot off: `sudo shutdown now` in an ssh shell on her first, then the power button.

---

## 7. Reference

### 7.a Driving a base by hand

- **The page:** Control › robot › Base: hold a drive button, at the Speed set. `Clearance` (ticked) soft-stops the base
  when it comes within 5 cm of anything in the scene ([plugins.md, Clearance](plugins.md#clearance)); untick it only to drive out of a tight spot. The phone's Control tab has
  the same D-pad.
- **The joystick:** L1 or L2 held, the left stick.
- **The keyboard,** in pane checks (domain 80 is loaded there):

**▶ pane checks**
```bash
# Cindy; Alice: /a200_0804
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/a200_0806/joy_teleop/cmd_vel
###
```

| key | does |
|---|---|
| `i` / `,` (hold) | forward / backward |
| `j` / `l` | turn left / right on the spot |
| `k` (or any other key) | stop; releasing the key also stops the base within 0.5 s |
| `q` / `z` | raise / lower both speeds by 10 % |
| `Ctrl-C` | quit |

**!** Every key typed in that pane is a drive command: quit it (`Ctrl-C`) before using the pane for anything else.
**!** Hand on the e-stop; after driving, plan again (the monitor plans at the live base). Never drive Alice while she
holds B21.

### 7.b An arm back to the joint controller

- **The page:** Control › robot › the arm: button [Run: Joint].
- **The command line,** in pane checks:

**▶ pane checks**
```bash
for arm in left_ur5e right_ur5e; do     # Cindy; Alice: /a200_0804 and ur5e only
  ros2 service call /a200_0806/$arm/controller_manager/switch_controller controller_manager_msgs/srv/SwitchController \
      "{deactivate_controllers: [cartesian_compliance_controller], activate_controllers: [scaled_joint_trajectory_controller], strictness: 1}"
done
###
```

**Expect:** per arm, under `response:`, `controller_manager_msgs.srv.SwitchController_Response(ok=True)`.

### 7.c What the monitor prints at start

- the two viser boxes, `UI at http://localhost:8080, phone page at http://<this PC's IP>:8081`;
- one `URDF matches its UR calibration` line per arm (four);
- `experimental plugins, their behaviour and API may change: ...`;
- a line per scaffolding tool's gripper and joint screw (`idle`);
- `started in ... ms; slowest: ...`, and a few late-tick reports while everything warms up;
- the health chips' colour changes, one line each once held for 1.5 s (`[health/chips] ...`);
- PyBullet's own lines (8.b).

### 7.d The servo in detail

- It runs after a transfer (onto the insert start), a support approach (onto the approach pose) and a ground bar's
  insert (onto the insert end, straight lines only, stopped when a flange's force changes by over 20 N).
- The first pass waits for Confirm; a later one only when it travels over 5° (`SAFEGUARD`). At most 8 passes.
- Done: every flange within 0.2 mm of its target (tool0); or, steering on the mocap bar (B20 with Servo by bar), the
  bar's fit within 0.5 mm / 0.1° of its design pose; or `nothing left to plan` (the correction is under what the planner
  moves).
- The panel's last exec line shows each reading, and after the Exec how the servo ended, until the next Exec.
- Each servo is recorded as `runs/<run>/servo/<action>/<movement>.<n>.json`, with a `.png` of its readings.

### 7.e Dry run

Settings › box [Dry run]: ticked, every check runs and nothing is sent: a motion logs `DRY RUN <serial> <arm>: passed
every check, NOT SENT`, an insert `DRY RUN ...: insert: every compliant target passed the arm's checks, NOT SENT`, a
retreat `... retreat: the screw-out passed every check, NOT SENT; the rest is built from where it ends when it
runs`, the handoff its anchor gap.
The panel shows a `DRY RUN` chip while it is ticked. Untick it before the real Exec.

### 7.f The run record

The run's folder: `.../261103_SRD/runs/<date>_<time>/` on the Drive (`doc/design/run_record.md`):
`run.json`, `progress.json` (the marks, where each robot was left), `events.jsonl`;
`movements/<action>/<movement>.<n>.json`, one file per part, the refused and cancelled ones included (an insert's or a
retreat's keeps its settings, phases, switches, stall times, peak force, end errors and checks);
`servo/<action>/<movement>.<n>.json` and its `.png`.

**Restarting the monitor** keeps the run: pane monitor `Ctrl-C` once, wait for `(viser) Server stopped` twice, Up,
Enter; reload the page, button [Load]. **Expect:** pane monitor: `continuing run <date>_<time>`. The done actions are
still green; the robots keep where they are.

### 7.g tmux and the terminals

tmux keys: press `Ctrl-b` (the "prefix"), let go, then the key.

| keys | does |
|---|---|
| `Ctrl-b %` | split **left / right** (the new pane opens on the right) |
| `Ctrl-b "` | split **up / down** (the new pane opens below) |
| `Ctrl-b` arrow, or `Ctrl-b o` | go to the pane in that direction / the next pane |
| `Ctrl-b z` | zoom the current pane to the whole window, and back |
| `Ctrl-b x` | close the current pane (answer `y`); see the box at the top first |
| click `[z]` / `[x]` at a pane's top right | the same as `Ctrl-b z` / `Ctrl-b x` for that pane (`[u]` unzooms) |
| `Ctrl-b d` | detach: the session keeps running in the background |

The mouse works (`~/.tmux.conf`): the wheel scrolls back through a pane's output (`q` returns to the live end), a click
picks a pane, a drag selects and copies to the clipboard. A robot's pane frozen after she rebooted: press Enter, then
`~` and `.` (ssh's escape) to close the session.

### 7.h The scripts

All in `scripts/real_lab/`, beside the fake lab's `scripts/fake_lab/`. Each says at its top what it does.

| script | does |
|---|---|
| `tmux.sh` | the session `real_lab`, its six panes, the logs; attaches to it |
| `env.sh` | the lab's settings, sourced by the others and in pane checks: domain 80 on Zenoh, the workspace, the Drive, the robots' addresses |
| `robot.sh <robot>` | ssh to her and run `crl_arms_and_base.launch.py` there with her tools; a shell on her instead when she is not on Zenoh in domain 80, or a crl_husky launch already runs on her |
| `relay.sh` | the mocap relay against the Motive PC, with the Drive's calibrations |
| `monitor.sh` | the monitor: the three robots, the labs' plugins (`lab_common.sh`), the process from the Drive; opens its page once started |
| `check_robots.sh [robot ...]` | Part 3's read-only checks |
| `../lab_common.sh` | what both labs share, sourced by their scripts: the monitor's plugins, the clean environment each process starts in, the monitor's start line, the tmux panes |

Settings: given to `tmux.sh` when it makes the session, they reach every pane (`REAL_LAB_MOTIVE=192.168.0.117
tmux.sh`); given to one script in its pane, only that one (`REAL_LAB_MOTIVE=192.168.0.117 relay.sh`). Running
`tmux.sh` again with a setting changes nothing: the session exists, so it only re-attaches.

| setting | default | for |
|---|---|---|
| `REAL_LAB_MOTIVE` | `192.168.0.28` | the Motive PC (it is to move to `.117`) |
| `REAL_LAB_DRIVE` | the Insync copy of `2025-03 Husky Assembly` (0.3) | another Drive folder |
| `REAL_LAB_HOME` | `~/husky_real_lab` | where the logs go |
| `REAL_LAB_SESSION` | `real_lab` | another tmux session name |

### 7.i The as-built record and the assembly page  [RECORDED BY ITSELF — read afterwards]

While a step runs, the monitor measures every bar with mocap markers at the start of the step's first part and the
end of every part (`doc/design/run_record.md`, "The as-built file"), and appends one line per record to the old monitor's
file of the day: `<drive>/data_experiment/bar_action_live_replan_exe/<YYYYMMDD>-asbuilt/as_built_<process>.jsonl`.
Nothing here moves a robot.

| moment | part, phase | milestone | the bar is compared with |
|---|---|---|---|
| before the insert | J transfer's last part (its servo), end | `before_insert` | the authored flange target of the transfer's end, with the grasp |
| after the insert | J insert's last part (the insert, or a ground bar's servo), end | `after_insert` | the seat |
| after a ground bar's fix | J step by hand after the insert, end | `after_fix` | the seat |
| before / after the retreat | R retreat, start / end | `before_retreat` / `after_retreat` | the seat |
| support robot holds | H approach's servo / move in / handoff, end | `hold_approach` / `hold_at_grasp` / `hold_grasped` | the seat |
| support robot lets go | HR open / move out, end | `hold_released` / `hold_retreated` | the seat |
| every other part | start of the first, end of each | none (`bystander` for the other bars) | the seat |

- [ ] nothing to click: run the steps as in Part 6.
  - **Expect:** pane monitor, one line per bar at each record and one for the file:
    `[as-built] B20_J_transfer end (done): B20 [before_insert] fit ok 8/8, rms 0.6 mm | vs held_authored:B20_J_transfer dpos 0.4 mm, dang 0.030 deg`,
    `[as-built] B20_J_transfer end: recorded -> .../20261030-asbuilt/as_built_261103_SRD.jsonl`.
- [ ] optional: button [As-built: record now] (Assembly tab, under Dry run) at a moment no part covers.
  - **Expect:** pane monitor: the same lines with `manual`.
- [ ] any time, the progress so far: button [Build as-built page] (Doc tab)
  - **Expect:** pane monitor: `[as-built] building the as-built page: scripts/as_built_page.py`.
  - Then a link under the buttons (`http://localhost:8082/assembly_<day>.html`): click it, the page opens in a new tab
    (the slider over the actions, the pills, a bar's card on a click; the old manual's section 9.4 reads it). Built
    again on every click.
  - From a terminal instead: `python src/husky-assembly-teleop/scripts/as_built_page.py <day> --open` (the venv
    active, `HUSKY_DRIVE_ROOT` set); `scripts/fetch_three_js.sh` once per checkout first.
- [ ] any time: buttons [Build accuracy report] and [Build incident report] (Doc tab)
  - **Expect:** a link under the buttons as above, `accuracy_<day>.html` and `incidents_<day>.html`, in the same folder
    on the Drive. Generated pages: numbers and log lines, no conclusions. From a terminal:
    `scripts/as_built_report.py <day>`, `scripts/incident_report.py <day> --to <day>`.

**If not:**

| you see | do |
|---|---|
| no `[as-built]` line at all | `[as-built] no bar with mocap markers in this process: nothing recorded`: the process has no marker layouts; or `the mocap_bars plugin is not loaded` |
| `... no bar measured` | no bar's asset streams: Motive's assets 10xx with Labeled Markers (2.e); **Mocap › Bars** says which bars have a sample |
| `B20 fit FAILED (only 5 of 8 design markers matched (need 6))` | balls hidden by the tools; the record is kept with the reason; free the balls, then [As-built: record now] |
| `[as-built] end of ... NOT recorded: FileNotFoundError(...)` | the Drive's `data_experiment/bar_action_live_replan_exe` is not there (Insync not synced); the step went on |
| `not built: ...` under the button | read the log: usually the three.js files (`scripts/fetch_three_js.sh`) or no records today |

---

## 8. If something goes wrong

### 8.a Starting over / redoing an action

- **A step waits and you want out:** button [Cancel].
- **A motion must stop now:** the robot's remote e-stop; or `Stop all (Esc)` (each robot then waits for its `Resume`
  under Health).
- **Redo an action:** select it → button [Reopen] (pending again) → its steps again from where the robots stand.
- **Start the whole test over:** button [Run: New run], then once more when it reads `New run?`.
  Nothing is deleted: the old run's folder stays.
- **The monitor hangs or the page shows nothing new:** reload the page first; then restart the monitor (7.f).

### 8.b Problems and what to do

| situation | do |
|---|---|
| a robot's chips red, nothing from her | `check_robots.sh <robot>` (Part 3): her network, the router link, her launch |
| topics or arm control stall; a router's log says `Unable to push non droppable ... Closing transport!` | restart that machine's router: on a robot `crl-husky-zenoh-restart`; on this PC `systemctl --user kill -s KILL crl-zenoh-router` (it restarts itself) |
| `Timeout for service ...` or `Didn't receive final reply for query`, often minutes after the call | stale routes after a crash: restart the routers on every robot and on this PC |
| nodes die at start with `undefined symbol` in `librmw_zenoh_cpp.so` | `ros-humble-zenoh-cpp-vendor` older than `ros-humble-rmw-zenoh-cpp`: `sudo apt install ros-humble-zenoh-cpp-vendor ros-humble-rmw-zenoh-cpp` |
| colcon: `Duplicate package names not supported` | a git worktree inside the workspace: 1.f |
| a robot's pane: `Failed to bind socket for port 5000x to address. Reason: Address already in use`, every second | restart her launch (2.f b) |
| a robot's pane: `mocap stamp N ms in the future: is this robot time synced?` | her clock is behind: 2.f c) |
| `relay.sh`: `Another mocap relay is already running in this ROS domain` | 2.f |
| PyBullet's lines in pane monitor | `monitor.sh` imports pytest before the monitor: compas_fab closes stdout for a moment around every model it loads, other threads then fail with `OSError(16, 'Device or resource busy')`, and compas_fab skips that only under pytest. A real fix is your decision (as in the fake lab) |
| `robots differ` after Load | the export used the old UR frames; the monitor plans with the configured robots |
| an arm's `arm` chip red | hover it: the safety sync says why. Pendant in Local: Remote (2.a). Protective stop: find why, then `Unlock` under Health. After Stop all: `Resume` |
| Exec refused: `... is not under scaled_joint_trajectory_controller (it runs ...); switch it in robot_control` | 7.b |
| `HOLD NOT SENT on ...: stop ... with the teach pendant or the e-stop` | an arm refused the hold (stale joint states): stop it with the pendant or the e-stop |
| `ARMS MAY BE LEFT UNDER cartesian_compliance_controller` | 7.b before anything else |
| a compliant part refused: `the measured TCP is X mm, Y° from tool0 by the URDF ...: a frame is wrong` | the arm's TCP setting differs from the URDF: note the numbers; nothing was sent |
| a motion times out and is held mid-path | that pendant's speed slider is below 100 % (2.a) |
| the Robotiq dead after an e-stop: its driver logs `Requested 8 bytes, but got 0` | crl-husky README, Restart: power off the arm and reboot the robot PC |
| `base path ... timed out: still following` | Plan and Commit again (6.1) |
| don't press Cancel in the base planner right after a Commit while that robot still drives an earlier path | a race in the base planner (review finding M1): within a tick, the earlier path can keep driving unwatched. Stop a driving base with Stop all |
| Live IK: `no collision-free IK ...` | the base stands too far from (or too close to) the design's base: drive again (6.1) |
| Plan: `no path in 120 s` (B22's transfer) | Plan again |
| Mark done accepted an action with steps left | it marks the action, not its steps: wait for `every step of ... done`; button [Reopen] to undo |
| the servo ran all 8 passes without converging | harmless when the flanges stay 1–3 mm off (the bar as mounted by hand); note it |
| the servo after B20's insert stopped: `the force on an arm changed by ... N` | the bar may push on something (the floor?): look before going on; note it |
| `no marker sample of bars/B20 (asset 1020)` | Motive does not stream B20's balls (2.e): the servo aims tool0 |
| the jaws not centred on B21 after the approach | Alice's base calibration is off (6.4 c): stop before the straight approach |
| a robot's pane frozen after she rebooted | Enter, then `~` and `.`; then Enter again for `robot.sh` |

---

## 9. What to send back

Tell me the step numbers that did not go as written, and what you saw (e.g. "6.2 e) not OK on B21: ..."). Paste the
pane's lines from the command to the next prompt (the `###` line marks where its output starts). A screenshot helps for
anything on the page. I read the panes and the logs myself:
- `~/husky_real_lab/logs/`: the monitor's log and the relay's and each robot's pane;
- the run: `.../261103_SRD/runs/<date>_<time>/` (movements, servo files, events).

Worth noting as you go:
- **Part 3:** any FAIL or NOTE, and what fixed it.
- **5.a:** how each base drove its short path, and how far from the target it stopped.
- **5.c and 6.4 f):** the anchor gap (`anchor X mm, Y°`), the handoff's lines, and how Alice's arm behaved while
  compliant (did it stay still? Did Cindy's tools get pulled?).
- **Each J action:** the Live IK `max |goal - authored|`; how the servo ended (and its passes); how the insert ended
  (stall times, peak force, any warning); whether the bar sat in both tools after you clamped it.
- **B20:** the mocap-bar servo's ending and the `### bar pose at termination` line; the servo after the insert (its
  force changes).
- **Each R action:** the retreat's `check` lines (the FAIL ones with their numbers), and whether the tools let go and
  backed off cleanly (B21_R: clear of Alice's gripper).
- **B22:** whether B21 stayed in Alice's jaws while B22 screwed in.
