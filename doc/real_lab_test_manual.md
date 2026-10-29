# Cindy, Alice and Belle: the first three bars with the new monitor — real robots, from Yijiang's laptop

This is the operator manual for the first three bars of the design problem **`261008_2ModulesDemo_purged_process`**
(the two-module demo, 78 actions) on the real robots, with the new monitor: schedule entries 1–7, **B20_J, B20_R,
B21_J, B21_H, B21_R, B22_J, B22_R**. It is the fake-lab test (`doc/fake_lab_test_manual.md`) on the real robots: the
same process, plugins and steps, run from this laptop over Zenoh.

> **?** Written on 2026-10-28 from the fake-lab runs of 2026-10-26/27 and the code; not run on the robots yet. Log
> lines are quoted from the code; a number marked *(fake lab)* is what the simulation gave, and the robots will give
> their own. Tell me where the page or the robots differ (Part 9).

- **First time:** read Parts 0–5 in order.
- **Done it before:** use the roadmap (0.2) as a checklist; each row links to its part.

How to read the steps:
- `- [ ]` is one thing to **do**, named by the control and what to do with it, with the page's own labels:
  `button [Plan]`, `slider [Preview time]`, `box [Dry run]: ticked`, `row [4. B21_H]`, `pane cindy: Enter`,
  `by hand: ...`, `by eye: ...`. A tab path is written `Plan › Base`; a button of a group `button [Load: Step]`. `→`
  joins controls you use one right after the other (`button [Load: Step] → button [Path: Plan]`).
- The indented lines under it: **Expect:** is what you should **see** (a log line in pane monitor, the page, the
  robot); **If not:** says what to do when it doesn't (or points to Part 8). An indented line without a label is a
  side note.
- `→` always means "then do the next thing"; it never stands for a result.
- In Part 6 each procedure is split into lettered phases (a), b), ...); the table in 6.5 lists what differs per action.
- **`▶ pane NAME`** above a code block says which terminal pane to paste it into (0.7). Paste the block as is. Blocks
  without a tag are output or diagrams, not commands.
- **!** starts a warning, **?** a side note.
- A badge in a heading says what kind of part it is:

| badge | means |
|---|---|
| `[SETUP — once]` | installs, settings and files, done once before the first session; nothing moves |
| `[ROBOTS ON — nothing moves]` | powering and launching the robots |
| `[CHECK — nothing moves]` | read-only checks; the monitor's start |
| `[TEST — base, arm or gripper move, no bar]` | tests before the assembly, in a run of their own; nothing is marked done |
| `[REAL RUN — recorded]` | the assembly itself; every part is recorded in the run, `runs/<date>_<time>/` on the Drive |

Anything that does not match an **Expect:** is worth a note (the log lines and what you did; Part 9). What each panel
and button does in detail: the README (Panels; the Plugins table).

> **If you closed a terminal** (the six panes: 0.7)
>
> **The whole terminal window:** nothing is lost. tmux keeps the session `real_lab` running in the background, robot
> launches and monitor included. Open a terminal and run:
>
> **▶ any terminal**
> ```bash
> ~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/real_lab/tmux.sh
> ```
>
> **Expect:** the six panes come back exactly as they were (`tmux attach -t real_lab` does the same).
>
> **One pane** (`exit`, `Ctrl-D` or `Ctrl-b x` in it): only that pane is gone; the other five keep running. What it
> stopped:
>
> | pane | what closing it stopped |
> |---|---|
> | monitor | the monitor, without its final holds. The robots' drivers keep running and the arms hold where they are |
> | relay | mocap for everyone in domain 80: the monitor's base poses and the robots' followers. **!** Never while a base drives |
> | alice, belle, cindy | the ssh session **and the launch in it**: that robot's arm drivers, tools and follower are off. **!** Never close a robot's pane while she holds a bar |
> | checks | nothing |
>
> To get the pane back without touching the others:
> 1. Click the pane next to where it was.
> 2. `Ctrl-b` then `"` (split below) or `Ctrl-b` then `%` (split right): a new pane appears, the scripts on its PATH.
> 3. Give it its name back, e.g. for Cindy's pane:
>
> **▶ the new pane**
> ```bash
> tmux set-option -p @name cindy      # or monitor, checks, relay, alice, belle
> ```
>
> 4. Redo what was in it: `robot.sh cindy` (a robot's pane, Part 2), `relay.sh`, `monitor.sh` (4.a), or
>    `source env.sh` (the checks pane).

---

## 0. Start here

### 0.1 What the test does

Three mobile robots ("huskies") build the first three bars of the two-module demo:
- **Cindy** (dual-arm husky, two UR5e arms with scaffolding tools) joints B20, B21 and B22, and backs off after each.
- **Alice** (single-arm husky, Robotiq 2F-85 gripper) grabs **B21** once Cindy has inserted it, and still holds it at
  the end of the test (in the process she lets go at entry 17, after B34–B37; 6.6 lets her go early).
- **Belle** (single-arm husky, Robotiq 2F-85) stays parked. She is in the monitor so the planners see where she stands.

**The story of one bar (B21):**
1. Cindy's base drives to where the design puts it: the base planner plans a path and her own follower drives it.
2. Cindy moves her two empty tools to where B21 starts its insert (the *transfer*, a free move: the transfer
   workaround).
3. You place B21 in her tools by hand and clamp it with the buttons on the tools.
4. Cindy corrects her pose by a few mm using mocap (the *servo*), then inserts B21: its left joint screws into B20.
5. Alice drives up, servos, opens her gripper, slides it onto B21 and closes it with her arm compliant (the *hold*).
6. Cindy's tools let go, she backs off (the *release*) and goes home.

**The seven actions** (the process panel lists them with the transfer workaround on):

| # | action | who | movements | what happens |
|---|---|---|---|---|
| 1 | `B20_J` | Cindy | transit, place bar, grasp, insert | B20 is the ground bar: transfer, the bar by hand, servo (on the MoCap bar), **rigid** insert with no tighten, then a second servo |
| 2 | `B20_R` | Cindy | ungrasp + retreat, home | her tools open and back off (the first millimetres compliant), then home |
| 3 | `B21_J` | Cindy | transit, place bar, grasp, insert | B21 mates on its **left** joint only: rigid, then compliant while the left joint screw tightens |
| 4 | `B21_H` | **Alice** | approach, open, to grasp, close | Alice grabs B21: approach and servo, open, 100 mm straight in, close with the compliant handoff |
| 5 | `B21_R` | Cindy | ungrasp + retreat, home | as 2, with Alice holding B21 |
| 6 | `B22_J` | Cindy | transit, place bar, grasp, insert | B22 mates on **both** joints, to B21 and B20: both screws tighten |
| 7 | `B22_R` | Cindy | ungrasp + retreat, home | as 2; then the current step is `B34_J` |

**One monitor for all three robots.** The new monitor talks to every robot at once over Zenoh (ROS domain 80) and runs
one motion at a time across all of them: no restarts between robots, as the old monitor's seven runs needed. The
**run** (`runs/<date>_<time>/` in the process folder on the Drive) records every part, which actions are done and where
each robot was left.

### 0.2 Roadmap (the whole session)

| # | what | part | pane | robot | does anything move? | recorded in the run? |
|---|---|---|---|---|---|---|
| 1 | stop the fake lab | 1.a | the fake lab's | — | no | no |
| 2 | once: network, Zenoh router, time sync, ssh keys, build, crl-husky on the robots, the Drive | 1.b–1.h | any terminal | — | no | no |
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
| this laptop | Yijiang's laptop, user `yijiangh` | runs the monitor, the relay and the checks |
| workspace | `~/Code/ros2_ws_next` (venv `venv`, ssik installed), branch `yh/process_replan_execution` | crl-husky in `src/crl-husky` (03056b7 on 2026-10-28) |
| the Drive | `~/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly` | the process folder; the run is recorded there (`runs/`), the relay reads the base calibrations there |
| lab network | the USB-ethernet dongle (`enxa0cec86472dd` on 2026-10-28) cabled to the lab switch: `192.168.0.203`, this laptop's reserved slot | the Wi-Fi `wlo1` stays for the internet; ROS does not use it |
| middleware | Zenoh (`rmw_zenoh_cpp`) on ROS domain 80; this laptop's router `crl-zenoh-router` linked to every robot's | the scripts set it; `~/.bashrc` (Cyclone, domain 86, for the old monitor) stays as it is |
| Cindy's PC | `192.168.0.115`, user `administrator`, password `12345678` | rigid body 1860 |
| Alice's PC | `192.168.0.113`, same user and password | rigid body 1840 |
| Belle's PC | `192.168.0.114`, same user and password | rigid body 1850 |
| Motive PC | `192.168.0.28` | the robots' bodies, and B20 / B21 as assets 1020 / 1021 (their MoCap balls) |
| remote e-stops | one per robot | pair one husky at a time (crl-husky README, Troubleshooting) |
| the bars | B20, B21, B22, ready to hand next to Cindy's insert starts | B20 and B21 carry MoCap balls, B22 none |
| a phone (optional) | on the robot Wi-Fi | `http://192.168.0.203:8081`: Stop all, and Confirm / Cancel next to the robots (`doc/phone_manual.md`) |

### 0.4 How the PCs connect

```text
  Motive PC .28 ── NatNet, wired ──► this laptop .203 (USB-ethernet dongle)
                                       ├ mocap relay: Motive's bodies as ROS topics, the robots' calibrated
                                       ├ monitor: the web page (port 8080), the phone page (8081)
                                       └ Zenoh router, 7447 ─┬─ Cindy .115: router ◄─ her launch (arms, tools, follower)
                                                             ├─ Alice .113: router ◄─ her launch
                                                             └─ Belle .114: router ◄─ her launch
  Every machine runs ROS 2 on Zenoh in domain 80, with a router of its own; this laptop's connects to the robots'.
  The robots reach the laptop over the Wi-Fi; their followers take their base poses from this laptop's relay.
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
- **!** One monitor only: `monitor.sh` refuses to start while another one runs in domain 80, but it checks only once,
  at its start: a monitor started later on another PC goes unnoticed. Tell everyone in the lab (Jakob above all) that
  this test drives the robots.
- **!** An arm left under the compliance controller (a loud `ARMS MAY BE LEFT UNDER cartesian_compliance_controller`):
  switch it back before anything else (7.b).
- **!** On Cindy's tools, press one button at a time: B1 and B2 together put that arm in free drive (crl-husky README,
  Free drive from the tool buttons).

### 0.6 Words used in this manual

| word | meaning |
|---|---|
| **monitor** | the operator program: a web page served by this laptop (`monitor.sh`, 4.a). It drives all three robots |
| **process** | the design as the monitor runs it: `261008_2ModulesDemo_purged_process`, 78 actions in build order (the *schedule*) |
| **action** | one unit of the schedule: `B20_J` Cindy joints B20, `B20_R` she releases it, `B21_H` Alice holds B21, `B21_HR` she lets go |
| **movement**, **step** | one part of an action: a free move, a straight move, a gripper or tool step, a step by hand. The process panel picks one; the step line under the buttons names it |
| **run** | the record of one assembly: `runs/<date>_<time>/` in the process folder on the Drive: which actions are done, where each robot was left, every part executed, the servo readings |
| **Mark done** | the yellow button (it reads `B20_J done`): records the action done and where every live robot stands |
| **Live IK / Plan / Play / Exec** | where the next movement starts, by IK at the live base (teal) / plan the step from the live robot (teal) / preview it (`Preview time`) / run it (orange) |
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
| **servo** | after a transfer, an approach or a ground bar's insert: small corrections from mocap until the flanges are within 0.2 mm of their targets; for B20 (**Servo by bar**) until the bar's MoCap fit is within 0.5 mm / 0.1° of its design pose |
| **controllers** | `scaled_joint_trajectory_controller` (**Joint**: stiff, follows joints; the normal one) or `cartesian_compliance_controller` (**Cart**: gives way to forces; only for an insert's last millimetres, a retreat's first ones and the handoff) |
| **split** | those millimetres under the compliance controller (`Insert split mm`, 5) |
| **transfer workaround** | the process panel's box: Cindy's transfer runs as a free move with empty tools to the insert start, where you place the bar by hand (this export has no loading pose) |
| **ground bar / one-mate / two-mate** | B20 stands on ground joints: rigid insert, no tighten. B21 mates on its left joint only, B22 on both |
| **ghost** | a see-through robot in the 3D view: blue where the selected movement has the robot (after Live IK or a plan, on the live base), orange at the base planner's target, green along a plan |
| **pane** | one of the six terminal areas of 0.7 |

### 0.7 The six terminal panes

The panes live in **tmux**, so they keep running when the terminal window closes, and I can read them from my side.

- [ ] Open a terminal and run:

**▶ any terminal**
```bash
~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/real_lab/tmux.sh
```

**Expect:** the session `real_lab` with six panes, each with its name on its top border and a one-line hint of what it
is for:

```text
┌──────────────────────────────┬──────────────┐
│ monitor                      │ alice        │
│                              ├──────────────┤
├──────────────────────────────┤ belle        │
│ checks                       ├──────────────┤
├──────────────────────────────┤ cindy        │
│ relay                        │              │
└──────────────────────────────┴──────────────┘
```

| pane | for | tmux.sh left it | logged to |
|---|---|---|---|
| monitor | the monitor (4.a) | `monitor.sh` typed in: press Enter in 4.a | `~/husky_real_lab/logs/monitor_<date>.log` |
| checks | `check_robots.sh` (Part 3) and any `ros2` command: domain 80 on Zenoh is loaded (`source env.sh`) | `check_robots.sh` typed in | — |
| relay | the mocap relay | running (`relay.sh`) | `~/husky_real_lab/logs/relay_<date>.log` |
| alice, belle, cindy | that robot's launch, over ssh (`robot.sh <robot>`, Part 2) | `robot.sh <robot>` typed in: press Enter in Part 2 | `~/husky_real_lab/logs/<robot>_<date>.log` |

- The scripts are on every pane's PATH: when one stops, **Up, Enter** in its pane starts it again.
- Running `tmux.sh` again only re-attaches. If the fake lab still runs when it makes the session, the monitor pane's
  hint says so (1.a).
- I read the panes (`tmux capture-pane`) and the logs from my side; I never type into this session.

The four tmux things you need (more in 7.g):

| do | what it does |
|---|---|
| click a pane | type into that pane |
| `Ctrl-b` then `z` | zoom the pane to the whole window, and back |
| mouse wheel; `q` | scroll back through a pane's output; back to the live end |
| `Ctrl-b` then `d` | leave tmux; everything keeps running. Back with the script above |

(`Ctrl-b` means: press Ctrl and b together, let go, then the next key.)

### 0.8 The monitor page

Open in Chrome on this laptop: http://localhost:8080 (4.b). One page at a time.

```text
┌ main panel (top left) ────────┐ ┌ Plan: Base | Motion | Arm ─┐          ┌ side panel (right) ────────────┐
│ Stop all (Esc)    N PROBLEMS  │ │ a planner's panel per tab  │          │ Assembly | Collisions | Control│
│ health: a row per robot       │ └────────────────────────────┘          │ PROCESS: Folder, Load, boxes,  │
│  base: signal mocap estop     │                                         │   ACTIONS, Step, ◀ mv  mv ▶    │
│        battery ctrl           │      3D view: the measured lab,         │ step line, Exec line           │
│  arm:  signal arm <tool>      │      the robots where Motive sees       │ Live IK · Plan · Play          │
│  Unlock · Resume · Reconnect  │      them, the process, the ghosts      │ Preview time · Motion time     │
└───────────────────────────────┘                                         │ Exec · Confirm | Cancel        │
                                                                          │ Mark done | Reopen             │
                                                                          │ settings · the run · Advanced  │
 [Plan plots] [Exec plots]  (bottom left, minimized)          [Console]   └────────────────────────────────┘
```

- **Assembly** tab, top to bottom: the PROCESS section (folder, `Load`, the boxes `Transfer workaround` and
  `Rhino obstacles collide`, the ACTIONS list, the `Step` slider, `◀ mv` / `mv ▶`, the movement buttons), the step line
  and the Exec line, the big buttons, the prompt line, the settings, the run (its chips, `Continue newest` /
  `Continue match` / `New run`, where each robot is), the closed `Advanced planning` folder.
- Buttons are coloured by what they do: teal computes without moving (Live IK, Plan), orange starts a motion (Exec),
  green lets a waiting step go on (Confirm), red stops (Cancel), yellow records (Mark done).
- **Control** tab: a tab per robot (`0804`, `0805`, `0806`), each with its Base and its arms: their controllers (the
  `Run` buttons: `Vel` for a base; `Joint`, `Cart`, `Free` for an arm), the drive and jog buttons, the tool buttons
  (Robotiq: `Gripper: Open / Close / Stop`).
- **Plan plots** and **Exec plots** open by themselves when a plan or a servo / compliant part has something to show;
  the control at a bar's right end opens or closes it. The **Console** (bottom right) shows the newest log lines.

---

## 1. Prepare  [SETUP — once]

All once, before the first session, except 1.a (every session). Nothing moves.

### 1.a Stop the fake lab (every session)

The fake lab's monitor holds ports 8080 and 8081, so the lab's monitor would refuse to start. (It cannot reach the
robots: its own router on 127.0.0.1:7470, domain 70.)

- [ ] In the fake lab's monitor pane: `Ctrl-C` once, wait for `Server stopped`; `Ctrl-C` in its sim pane.

**▶ any terminal**
```bash
tmux kill-session -t fake_lab      # also stops its router and relay
tmux ls                            # expect: no fake_lab line
```

### 1.b This laptop on the lab network

- [ ] by hand: the USB-ethernet dongle plugged in and cabled to the lab switch.

**▶ any terminal**
```bash
ip -4 -brief addr | grep 192.168.0          # expect: enx...  UP  192.168.0.203/24
for h in 192.168.0.115 192.168.0.113 192.168.0.114 192.168.0.28; do   # Cindy, Alice, Belle, the Motive PC
  printf "%s: " $h; ping -c1 -W1 $h >/dev/null && echo up || echo NO REPLY
done
```

**If not:**
- no `192.168.0.x` address: check the cable, then `nmcli connection up "Wired connection 1"`.
- an address other than `.203`: another dongle (the router reserves `.203` for this one's MAC). It works, but the
  robots take no time from this laptop (1.d); have `.203` reserved for the new dongle (crl-husky `TIME_SYNC.md`).
- a robot `NO REPLY`: she is off (Part 2), or off the network.

**?** Leave the Wi-Fi on: the internet goes through it, `192.168.0.x` through the dongle.

### 1.c The Zenoh router on this laptop

Every ROS node on this laptop meets the robots through this router. Without it a node waits for one, silently;
`relay.sh` and `monitor.sh` refuse to start and say so.

**▶ any terminal**
```bash
~/Code/ros2_ws_next/src/crl-husky/crl_husky/scripts/zenoh_router.sh --install
systemctl --user is-active crl-zenoh-router      # expect: active
```

**Expect:** `Installed /home/yijiangh/.config/systemd/user/crl-zenoh-router.service`. From now on it starts at login,
listens on `127.0.0.1:7447` and `192.168.0.203:7447`, and connects to every robot's router; off the lab network it
retries every 10 s. The fake lab is not affected: its nodes connect only to its own router.

### 1.d Time sync

The robots take their time from the control PCs (`.201`–`.210`), and the health chips flag a robot whose stamps run
more than 5 ms ahead of this laptop's clock or 30 ms behind it. As in crl-husky `TIME_SYNC.md` (Control PC), with this
laptop's slot `.203`:

**▶ any terminal**
```bash
cd ~/Code/ros2_ws_next/src/crl-husky/crl_husky/config/time_sync
sudo apt install chrony                     # replaces systemd-timesyncd
sudo cp control_pc_chrony.conf /etc/chrony/conf.d/crl_lab.conf
sudo sed -i '/192\.168\.0\.203 /d' /etc/chrony/conf.d/crl_lab.conf           # this laptop's own slot
sudo sed -i -E '/^pool /{/ prefer/!s/$/ prefer/}' /etc/chrony/chrony.conf     # its own internet first
sudo systemctl restart chrony
chronyc -n sources
```

**Expect:** a source marked `^*`: an internet server while the Wi-Fi has internet, else another control PC (`.201`, the
RobotX PC).

### 1.e ssh keys for the robots

So `robot.sh` and `check_robots.sh` log in without a password (without them `robot.sh` asks for it every time, and
`check_robots.sh` skips the robots' own settings).

**▶ any terminal**
```bash
for ip in 192.168.0.115 192.168.0.113 192.168.0.114; do ssh-copy-id administrator@$ip; done   # password 12345678
```

**Expect:** per robot `Number of key(s) added: 1`. Belle's host key is new to this laptop: answer `yes`. A robot that
is off: do it for her later.

### 1.f Build the workspace (after every pull)

**▶ any terminal**
```bash
cd ~/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash
python3 -m colcon build --symlink-install         # crl-husky and this package
```

Editing an existing `.py` file alone needs no build (the install links to the source): restart the monitor. New files,
messages or entry points do.

### 1.g The robots' crl-husky

Each robot runs the crl-husky in her `~/workspace/src/crl-husky`, copied there by crl-husky's `upload.sh` (files, no
git). Part 3 compares it file by file with this laptop's `src/crl-husky`:
- **its messages differ** (`crl_husky_msgs`): FAIL. Topics between them may not decode: the monitor would read her
  wrong, or not at all. Bring both to one commit before the test.
- **only its code differs**: NOTE. It runs, but it is not the code the fake lab tested.

Which side is newer? Ask Jakob before copying anything. To put this laptop's crl-husky on a robot, with her launch
stopped and nothing in her tools:

**▶ any terminal**
```bash
cd ~/Code/ros2_ws_next/src/crl-husky && ./upload.sh 192.168.0.115      # Cindy; .113 Alice, .114 Belle
ssh administrator@192.168.0.115
```

**▶ the ssh shell on the robot**
```bash
# add crl_husky_cpp to the list if Part 3 named a file of it
cd ~/workspace && colcon build --packages-select crl_husky_msgs crl_husky
exit
```

Then start her launch again (Part 2) and run Part 3 again: **Expect:** `her crl-husky: the same code and messages as
this laptop's`.

### 1.h The process on the Drive

The monitor reads the process and records the run in the shared Drive folder directly (the fake lab used a scratch
copy). Everyone with the Drive sees the run.

**▶ any terminal**
```bash
P="$HOME/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly/data_design_study/261008_2ModulesDemo_purged_process"
ls "$P"                         # expect: actions  meshes  process.json  robots  source  (and runs/ once a run exists)
ls -a "$P" | grep -c insyncdl   # expect: 0 = Insync has finished syncing
ls "$P/runs" 2>/dev/null        # runs of earlier tests: kept; this test starts its own (4.d, 6.0)
```

---

## 2. Bring up the robots  [ROBOTS ON — nothing moves]

### 2.a Pendant first, launch second

On **every** arm's teach pendant (Cindy has two, Alice and Belle one each):
- [ ] Power on and release the brakes
  - **Expect:** the pendant shows `Normal`.
- [ ] Set **Remote Control** mode with the Local / Remote switch in the pendant's top-right corner. (No switch there:
      ☰ → Settings → System → Remote Control → Enable, then use it.)
- [ ] The speed slider at the bottom of the pendant at **100 %**.
- [ ] Do not load or start any program by hand. Once the launch runs, its safety sync loads and plays
      `ros_control.urp` itself.
- [ ] Each robot's remote e-stop paired (one husky at a time) and in reach.

**!** Launch only after that. An arm that is off or in Local mode when its launch starts comes up without its
controllers; then restart the launch (Part 3 shows how to tell).

### 2.b Zenoh on the robots

The robots' base services (Clearpath's: base driver, joystick) must run Zenoh in domain 80, as this laptop. They may
have been switched to Cyclone for the old monitor. `robot.sh` says on login what she runs, and opens a shell on her
instead of launching when it is not Zenoh in domain 80:

```text
real lab: cindy runs rmw_cyclonedds_cpp on domain 86, not Zenoh, 80.
  Switch her here with   crl-husky-mw-zenoh   (it restarts her base services: never while she holds a bar),
  then   exit   and start robot.sh cindy again (Up, Enter).
```

**▶ pane cindy — in the shell robot.sh opened on her**
```bash
crl-husky-mw-zenoh      # asks for the sudo password (12345678), then restarts her base services
exit                    # back on the laptop; then Up, Enter: robot.sh cindy again
```

- It rewrites her `/etc/clearpath/robot.yaml` and `setup.bash` and restarts only her base system, not an arm launch.
- `crl-husky-mw-cyclone` puts her back for the old monitor (crl-husky README, Still using Cyclone DDS).
- **If not:** `Zenoh is not set up on this robot yet. Run once: sudo .../install_robot_config.sh`: she never ran Zenoh;
  ask Jakob.

### 2.c Cindy

- [ ] Both arms on, pendants in Remote (2.a).
- [ ] pane cindy: Enter (`robot.sh cindy` is typed in)
  - **Expect:** `real lab: cindy, administrator@192.168.0.115 ...`, then
    `[cpr-a200-0806] rmw_zenoh_cpp, ROS domain 80, robot 0806` and
    `real lab: ros2 launch crl_husky crl_arms_and_base.launch.py gripper:=scaffolding_v3`; then her launch's lines:
    each arm's driver and controllers (`Configured and activated ...`), both scaffolding tool drivers, the safety sync
    (playing `ros_control.urp` on each arm), and `pure_pursuit`. The launch keeps running in this pane.
  - **If not:** `... not Zenoh, 80`: 2.b. `this PC is robot ..., not cindy`: a wrong address; `exit`. A password
    prompt: `12345678`, or 1.e. `a crl_husky launch already runs on cindy, so not starting another` and the launch it
    found: one left from a session whose link dropped, or someone else's. Hers and working (Part 3 OK): keep it, `exit`.
    Else stop it as the lines say, `exit`, and Up, Enter. Two launches would mean two drivers for each arm.

### 2.d Alice

- [ ] Arm on, pendant in Remote (2.a); her remote e-stop in reach.
- [ ] pane alice: Enter
  - **Expect:** `[cpr-a200-0804] rmw_zenoh_cpp, ROS domain 80, robot 0804`,
    `real lab: ros2 launch crl_husky crl_arms_and_base.launch.py gripper:=robotiq_2F_85`, then the arm driver, the
    Robotiq's driver and controller, the safety sync and `pure_pursuit`.

### 2.e Belle

- [ ] Arm on, pendant in Remote (2.a). She stays parked, her arm stowed.
- [ ] pane belle: Enter
  - **Expect:** as Alice, with `[cpr-a200-0805] ... robot 0805`.

Leave all three launches running for the whole session. **!** `Ctrl-C` in a robot's pane stops that robot's arm
drivers, tools and follower: never while she holds a bar.

### 2.f Motive and the relay

- [ ] On the Motive PC: the rigid bodies **1840** (Alice), **1850** (Belle) and **1860** (Cindy) streaming; the bar
      assets **B20 = 1020** and **B21 = 1021** on; and **Labeled Markers** on in the streaming settings (the servo fits
      B20's balls; the old repo's `~/Code/ros2_ws/src/husky-assembly-teleop/doc/mocap_bar_pose_test_manual.md`, 0.3).
- [ ] pane relay: it has run since `tmux.sh` (Up, Enter if it stopped)
  - **Expect:** `real lab: mocap relay on ROS domain 80, Motive 192.168.0.28, calibrations from .../2025-03 Husky
    Assembly`, `Starting mocap relay` with `NatNet: client 192.168.0.203, server 192.168.0.28, unicast`,
    `NatNet client connected.`, then `calibrations at start, Drive root ...` with a block per robot's body: the
    newest Drive session (Alice `20260623`, Belle `20261013`, Cindy `20261016` on 2026-10-28), applied when within
    5 cm / 5° of crl-husky's default; `Motive latency (mid-exposure to transmit): ... ms`; a
    `Created mocap publisher for rigid body id=...` line per body.
  - **If not:** `no Zenoh router on this laptop`: 1.c. `Another mocap relay is already running in this ROS domain`:
    another PC runs one (the RobotX PC?). Use it, and note whose it is (its calibrations come from that PC's Drive
    copy); or stop it there and start this one (Up, Enter). A box `... newer calibration(s) refused`: note it; that
    robot keeps crl-husky's default.

---

## 3. Check the robots  [CHECK — nothing moves]

One read-only script checks this laptop, ROS domain 80 and the robots (about 20 s). It sends nothing to a robot: it
pings them, reads their settings over ssh and asks ROS what runs. What the monitor's health chips show once it runs
(rates, e-stops, batteries, the arms' safety state) is left to them.

- [ ] pane checks: Enter (`check_robots.sh` is typed in). To check only some robots, put their names after it
      (`check_robots.sh cindy`); to run it again, Up, Enter.
  - **Expect:** its first line, `[check_robots] checking this laptop, ROS domain 80 and alice belle cindy; read-only,
    nothing is sent to a robot`; then every line `OK` and a last line `[check_robots] all OK`. Each `FAIL` line says
    what to do under it (`->`). A good start looks like this:

```text
=== this laptop ===
  OK    on the lab network: 192.168.0.203 on enxa0cec86472dd (a control-PC slot)
  OK    Zenoh router listening on 7447 (service crl-zenoh-router: active)
  OK    time: chrony synced, reference ...
  OK    port 8080 free for the monitor
  OK    port 8081 free for the monitor
  OK    the process on the Drive: data_design_study/261008_2ModulesDemo_purged_process
  OK    the Motive PC 192.168.0.28 answers ping

=== ROS domain 80 (Zenoh) ===
  OK    one mocap relay: this laptop's (pane relay)
  OK    no monitor runs yet in domain 80
  OK    Motive streams 1020 1021 1840 1850 1860 (B20, B21 among them)

=== cindy (/a200_0806, 192.168.0.115; pane cindy) ===
  OK    her PC answers ping
  OK    this laptop's Zenoh router is linked to hers
  OK    her base services: rmw_zenoh_cpp, domain 80
  OK    rmw_zenoh 0.1.11, as on this laptop
  OK    her Zenoh router runs
  OK    her crl-husky: the same code and messages as this laptop's
  OK    her launch runs: crl_arms_and_base.launch.py gripper:=scaffolding_v3
  OK    her clock (chrony): Reference ID : ... (192.168.0.203);System time : 0.000... seconds fast of NTP time
  OK    publishing: the arms' joint states, the safety sync, the follower, the base and both tools
  OK    left arm controllers: joint active, compliance loaded and inactive
  OK    right arm controllers: joint active, compliance loaded and inactive
  OK    Motive streams her rigid body 1860
```

What it checks and what a `FAIL` means:

| line | OK means | FAIL or NOTE: cause → do |
|---|---|---|
| on the lab network | the dongle holds a `192.168.0.x` address, in a control-PC slot | no address: 1.b. NOTE outside `.201`–`.210`: 1.b |
| Zenoh router | this laptop's router listens on 7447 | not installed or stopped: 1.c; `systemctl --user restart crl-zenoh-router` |
| time | chrony runs and is synced | not installed: 1.d. NOTE not synced yet: wait a minute, `chronyc -n sources` |
| port 8080 / 8081 | free for the monitor (or this lab's monitor holds them) | held by another domain's process: the fake lab's monitor (1.a) |
| the process on the Drive | `process.json` found, Insync done | not found: Insync, or `REAL_LAB_DRIVE` (7.h). NOTE still syncing: wait |
| the Motive PC | answers ping | off; or it moved to `.117`: in pane relay `Ctrl-C`, then `REAL_LAB_MOTIVE=192.168.0.117 relay.sh` (7.h) |
| one mocap relay | exactly one in domain 80, this laptop's | none: pane relay, Up, Enter. Two or more: stop all but one. NOTE another PC's: fine if it is the lab's |
| monitor | none yet (or this laptop's) | another PC's monitor: find it and stop it. Two monitors would both command the robots |
| Motive streams | the robots' bodies and B20, B21 | nothing: Motive not streaming to this laptop (the relay's pane says). NOTE a bar missing: its servo aims tool0 only (2.f) |
| her PC answers ping | she is on the network | off (2.a) or off the network: her other checks are skipped |
| linked to hers | this laptop's router holds a connection to her router (port 7447) | her router: `crl-husky-zenoh-restart` on her; this laptop's: 8.b |
| her base services | Zenoh, domain 80 | Cyclone, or another domain: 2.b |
| rmw_zenoh | the same version on her as on this laptop | every machine needs the same (crl-husky README, Zenoh): upgrade the older one |
| her Zenoh router | its service runs on her | `crl-husky-zenoh-restart` on her |
| her crl-husky | the same files as this laptop's `src/crl-husky` | messages differ: FAIL, 1.g. NOTE code differs: 1.g |
| her launch runs | one `crl_arms_and_base.launch.py` runs on her | none: Enter in her pane (2.c–2.e). Two or more: two drivers for each arm: stop all but one. NOTE another launch: without the follower, Commit cannot drive her |
| her clock | chrony on her follows a control PC | NOTE nothing: chrony not set up on her (ask Jakob) |
| publishing | her arms' joint states, safety sync, follower and base publish; Cindy's two tools too | missing `platform/...`: her base services (`crl-husky-clearpath-restart` on her); the rest: her launch's pane says why |
| the Robotiq's action server (Alice, Belle) | the gripper's action server is up | her gripper driver failed: read her pane; a gripper dead after an e-stop: crl-husky README, Restart |
| arm controllers | Joint active, compliance loaded and inactive (the monitor switches to it only for a compliant part) | no answer: the launch started before the arm was on and in Remote: `Ctrl-C` it, Enter again. Joint not active: the pendant in Remote, its program playing? (the monitor's `arm` chip says why). Compliance active: 7.b. Not loaded: every compliant part would fail at the switch: ask Jakob |
| her rigid body | Motive streams it | her asset off in Motive (2.f) |

**?** Without ssh keys (1.e) the robots' own settings are skipped with a NOTE; the rest is checked.

---

## 4. The monitor  [CHECK — nothing moves]

### 4.a Start it

- [ ] pane monitor: Enter (`monitor.sh` is typed in)
  - **Expect:** `real lab: monitor on ROS domain 80 (Zenoh), Drive root .../2025-03 Husky Assembly`, `real lab: open
    http://localhost:8080 (phone: http://192.168.0.203:8081); log: .../husky_real_lab/logs/monitor_<date>.log`; two
    viser boxes (ports 8080 and 8081); `UI at http://localhost:8080, phone page at http://<this PC's IP>:8081`;
    one line per arm, `robot a200-0806 left_ur_arm: URDF matches its UR calibration (ur_calibration_left.yaml)`
    (four in all); for each of Cindy's tools `gripper idle` and `joint screw idle`; after about 10 s
    `started in ... ms`; then the health chips turning green, e.g.
    `[health/chips] a200-0806 arm: arm red -> green: sync: reporting`. PyBullet's own lines print in this pane too
    (8.b).
  - **If not:** `a monitor already runs in ROS domain 80`: another PC's, or this laptop's in another terminal: stop it
    first. `ROS did not say which nodes run in domain 80`: the router is not linked (Part 3). `port 8080 ... taken`
    from the monitor itself: the fake lab (1.a). `no Zenoh router on this laptop`: 1.c.

### 4.b First look

- [ ] Chrome: http://localhost:8080; keep that one page open
  - **Expect:** 3D view: the measured lab (cabinet, tables, tripods, trusses, the orange border) and the three robots
    where Motive sees them, their arms as measured.
  - **Expect:** health, top left: every base shows `signal`, `mocap`, `estop`, `battery NN%`, `ctrl`; every arm
    `signal`, `arm` and its tool (`robotiq` on Alice and Belle, `scaffolding_v3` on each of Cindy's arms). No `SIM`
    chip: nothing is simulated. Belle's `mocap` may be amber: her body's marker error was 1.2 mm in the lab (hover it).
  - **If not** (hover the chip):
    - `estop` red: that robot's remote e-stop is pressed (release it before anything moves).
    - a base's `ctrl` red: its `platform_velocity_controller` does not run; the follower needs it (4.d).
    - an arm's `arm` red: its safety sync says why: the pendant in Local, the program not playing, a protective
      stop (`Unlock` under Health, once you know why), or Stop all (`Resume`).
    - `signal` amber or red, `stamps ... in the future (robot clock ahead; time sync?)` or
      `every message over ... ms late`: the clocks (1.d), or a slow Wi-Fi link.
- [ ] Optional, the phone: http://192.168.0.203:8081 on the robot Wi-Fi (`doc/phone_manual.md`): Stop all, Health, and
      the Exec tab's Confirm / Cancel next to the robots.

### 4.c Load the process

- [ ] Assembly › PROCESS: the folder reads `data_design_study/261008_2ModulesDemo_purged_process` → button [Load] once;
      wait 5–10 s
  - **Expect:** a green chip `261008_2ModulesDemo_purged_process`, `78 actions · 273 movements`, and an amber
    `robots differ` (this export used the old UR frames; the monitor plans with the configured robots).
  - **Expect:** ACTIONS `1/78`: `1. B20_J · cindy · 4 mv · pending` on top, in blue, then `2. B20_R · cindy · 3 mv`,
    `3. B21_J`, `4. B21_H · alice · 4 mv`, `5. B21_R`, `6. B22_J`, `7. B22_R`, every row ending in `· pending`.
  - **Expect:** under the buttons, `current B20_J · B20_J_transfer (4/5)`, `no run yet` (the first record makes it),
    `0/78 done`, and `alice`, `belle`, `cindy` each `live now`.
  - Every Load draws the whole process again, and a page opened later first receives about 200 bodies (20 s): don't
    click Load twice.
- [ ] The two boxes under Load: box [Transfer workaround]: ticked (B20_J is listed as `J_transit`, `J_M1`, `J_M2`,
      `J_M4`, 4 mv); box [Rhino obstacles collide]: unticked, with the grey chip `Rhino obstacles: drawn only`. Leave
      both.
  - Rhino's cabinet `env39` sits 0.38 m nearer the park row than the measured one; ticked, it refuses any base pose on
    the row.

**The process panel picks the run's current step.** Click an action row or a movement button, drag the Step slider or
click `◀ mv` / `mv ▶`: the step under the buttons follows, except while an Exec holds the robots.

### 4.d The settings, the base controllers

- [ ] The settings, under Mark done: Ignore built bars off, Insert split mm 5, Spiral off, Spiral s 20, Spiral turns 4,
      Servo by bar **on**, Dry run off. Leave them so.
- [ ] Control › `0806` › Base, and `0804` › Base: the controllers line shows `platform_velocity_controller` running
      (the base's `ctrl` chip green)
  - **If not:** button [Run: Vel]. The follower drives only with it.
- [ ] The run line: `no run yet` on the first Load ever. If it says `continuing run <date>_<time>` (an earlier lab test
      on these files): button [Run: New run], then once more when it reads `New run: click again` (no time limit; a
      second click within half a second counts as a double click and is ignored) → `0/78 done`, every row
      `· pending`; the old folder stays.
  - Part 5's tests are recorded in this run; 6.0 starts another one for the assembly.

---

## 5. Tests before the assembly  [TEST — base, arm or gripper move, no bar]

Before Cindy holds a bar: each base that drives in this test under its follower once, and Alice's compliant handoff in
the air (it has never run on a real arm with the new monitor; in the fake lab it ran on a simulated Robotiq only). No
bar; clear floor and workspace.

**!** Do not Mark done here: these are tests.

### 5.a Each base under its follower: a short drive (Cindy, then Alice)

- [ ] Plan › Base: dropdown [Robot]: `a200-0806` → button [Load: Robot]
  - **Expect:** the fields `Target x y yaw°` show where she stands (world x, y in metres, yaw in degrees).
- [ ] fields [Target x y yaw°]: a point about 0.5 m away on open floor (change x or y, whichever way is clear) →
      button [Path: Plan]
  - **Expect:** after about 2 s a path drawn on the floor and `Lattice (hybrid A*) ...: ... m, ... s`.
- [ ] by eye: the path clear of people and of things the scene does not know (cables, boxes, a cart)
- [ ] hand on Cindy's e-stop → button [Commit]
  - **Expect:** `sent to a200-0806's follower`; she drives it smoothly and stops; pane monitor:
    `base path ... on a200-0806: done`. Control › `0806` › Base reads about the target (the fake lab's follower stopped
    within about 1 cm and 2°; note the real numbers).
  - **If not:** she does not start: the base's `ctrl` chip (4.d), her follower in her pane (`pure_pursuit`). She
    drives off the path: Stop all; note what she did.
- [ ] The same back: the old x, y, yaw → button [Path: Plan] → button [Commit]
- [ ] The same for Alice: dropdown [Robot]: `a200-0804`.

### 5.b Alice's gripper by hand

- [ ] Control › `0804` › the arm's TOOL section: button [Gripper: Open]
  - **Expect:** the jaws open; the tool line reads about `0.43`.
- [ ] button [Gripper: Close]
  - **Expect:** the jaws close; about `0.79` (an empty close).
- [ ] button [Gripper: Open] again: the handoff below starts with open jaws.

### 5.c Alice's handoff in the air (the jaws close on nothing)

- [ ] Assembly › ACTIONS: row [4. B21_H] → movement button [H_M3] (`B21_H_close`, the old monitor's code)
  - **Expect:** the step line names `B21_H_close`; the Exec line says Alice closes her gripper with the compliant
    handoff.
- [ ] settings: box [Dry run]: ticked → button [Exec] (and button [Confirm] if it asks)
  - **Expect:** `DRY RUN a200-0804 ur_arm: handoff: passed every check (anchor X mm, Y° from the URDF's tool0), NOT
    SENT`. The anchor is how far her arm's own TCP is from tool0 by the URDF: under 10 mm and 2° the handoff goes
    compliant; above, it closes stiff and says so. Note X and Y.
- [ ] box [Dry run]: unticked → button [Exec]
  - **Expect:** the prompt `alice (a200-0804): B21_H_close — CLOSE + compliant handoff on ur_arm: the arm goes
    COMPLIANT before the close (gives way ≤ 10 mm); jaws 0.8, effort 0.1, ≤ 30 s; then joint control. Watch Cindy's
    tools are not pulled; hand on the e-stop.`
- [ ] hand on Alice's e-stop → button [Confirm]
  - **Expect,** in order: `a200-0804 ur_arm: handoff: F/T sensor zeroed: |F| ... before, ... 0.5 s later`;
    `a200-0804 ur_arm: handoff: anchor X mm, Y° from the URDF's tool0 (limit 10 mm, 2°)`; `ur_arm:
    cartesian_compliance_controller running (...)`; `a200-0804 ur_arm: handoff: closing to 0.800 rad, effort 0.10, the
    arm compliant around its anchor ...`; the close; `ur_arm back under scaled_joint_trajectory_controller (... s)`; and
    `Exec B21_H_close: done`. The arm stays where it is while the empty jaws close.
  - **If not:** `... no handoff, the gripper closes with the arm STIFF`: the anchor check failed: note the numbers (a
    constant offset along tool z: the pendant's tool setting). `the arm moved ... mm from its anchor (limit 10 mm)`, or
    the arm moves when it goes compliant: e-stop; note how far and which way. A switch refused: nothing closed; check
    the controllers (Part 3), then again.
  - The line under the buttons then offers Mark done for B21_H: leave it, this is a test.
- [ ] Control › `0804`: button [Gripper: Open]
- [ ] Optional, a Cancel test: movement button [H_M2], then [H_M3] again (a step's parts count as done until another
      step is picked) → button [Exec] → button [Confirm] → button [Cancel] while the jaws close
  - **Expect:** the fingers stop, the arm back under joint control (Control › `0804` shows `Joint`). Then
    button [Gripper: Open].

---

## 6. The assembly  [REAL RUN — recorded]

### 6.0 A run of its own

- [ ] Under the run: button [Run: New run], then once more when it reads `New run: click again` (no time limit; a
      second click within half a second counts as a double click and is ignored)
  - **Expect:** a new run (`0/78 done`, every row `· pending`); the test run's folder stays. Once its robot is live the
    run selects the first pending action's first step: `B20_J · B20_J_transfer`.

Then the seven actions in order, each with its procedures (6.1–6.4) and what differs for it (6.5).

### 6.1 Procedure B — drive a base (base planner)

- [ ] Plan › Base: dropdown [Robot]: the action's robot (`a200-0806` Cindy, `a200-0804` Alice) → button [Load: Step]
      → button [Path: Plan]
  - **Expect:** `target +x +y +yaw°` (the design's base for the current step; 6.5 lists them), then after about 2 s
    `Lattice (hybrid A*) (straight): L m, T s` and the path on the floor.
  - **If not:** `start is in collision ...` / `target is in collision ...`: read the pair. A held bar counts: Load Step
    on the action's first movement (the one with empty tools).
- [ ] by eye: the path clear of people and of things the scene does not know
- [ ] hand on the robot's e-stop → button [Commit]
  - **Expect:** `sent to a200-0806's follower`; she drives (in the fake lab about twice the planned time); pane monitor:
    `base path ... on a200-0806: done`. Control › `0806` › Base then reads about the target.
  - **If not:** `base path ... timed out: still following` (the fake lab's follower sometimes never started a short
    path): button [Path: Plan] and button [Commit] again. To stop a driving base, use Stop all, not the panel's Cancel
    right after a Commit (8.b).
- **!** Never drive Alice while she holds B21.

### 6.2 Procedure J — jointing a bar (Cindy)

`<bar>` = the bar. Four steps: the transfer, the bar placed and clamped by hand (one prompt), the servo, the insert.

**a) Base:** Procedure B (6.1) with Cindy.

**b) Transfer** (`<bar>_J_transfer`)
- [ ] by eye: the step line reads `<bar>_J_transfer (transfer_free) · then: <bar>_J_place_bar, <bar>_J_grasp · skipped:
      <bar>_J_load`, and the Exec line `Exec: cindy (a200-0806) moves left_ur_arm, right_ur_arm; then waits for
      <bar>_J_place_bar + <bar>_J_grasp by hand; then servos left_ur_arm, right_ur_arm onto the insert start`
- [ ] button [Live IK: insert start] (about 5 s)
  - **Expect:** pane monitor: `check goal <bar>_J_transfer: goal found (ssik, rank 1 of 64), max |goal - authored| X
    rad`; the blue process robot turns to Target and shows the arms at the insert start on the live base. X grows with
    how far the base stands from the design's.
    The run's first record also logs `run <date>_<time> started`.
  - **If not:** `no collision-free IK ...`: the base stands too far from (or too close to) the design's base, or a
    drawn pair collides: drive again (6.1), then Live IK again.
- [ ] button [Plan] (15–40 s)
  - **Expect:** the chip `plan ready`, `min T s by the arms' limits`, and in the pane `transfer_free solved by
    tamp.plan_free_dual_arm: N waypoints, 1 tries`. The **Plan plots** window opens bottom left: every joint along the
    preview, and the largest joint step per row.
- [ ] button [Path: Play], or drag slider [Preview time]
  - **Expect:** it starts at the live arms and ends with the **empty** tools at the insert start, clear of the floor,
    the built bars and Alice.
- [ ] slider [Motion time]: higher = slower (first run) → hand on Cindy's e-stop → button [Exec]
  - **Expect:** pane monitor (a warning over 60°): `cindy (a200-0806): <bar>_J_transfer — move left_ur_arm,
    right_ur_arm along the planned path (Rhino obstacles not checked), T s, largest joint travel D°`; both arms move at
    once; `<bar>_J_transfer: arrived, end within ...°`; then the prompt for the bar.

**c) The bar, by hand**
  - **Expect:** the prompt `<bar>_J_place_bar + <bar>_J_grasp — Operator places the bar by hand; clamp it: tighten the
    gripper screws of left_ur_arm, right_ur_arm with the tools' buttons until they stall, then Confirm …`. The monitor
    sends the tools nothing here.
- [ ] by hand: place the bar in both tools → buttons **on the tools**: clamp it, until each gripper screw stalls
  - **!** One button at a time: B1 and B2 together put that arm in free drive.
- [ ] by eye: the bar sits in both jaws and is held
- [ ] button [Confirm] (here or on the phone's Exec tab)

**d) The servo** (runs by itself after Confirm)
  - **Expect:** the **Exec plots** window opens with the servo's plots (one point per pass); pane monitor: the feedback
    line (B20: `[servo] feedback: the MoCap bar bars/B20 (converges at 0.5 mm / 0.1 deg).`; B21:
    `[servo] feedback: tool0 (0.2 mm); the MoCap bar bars/B21 is recorded alongside.`; B22: `... no MoCap bar recorded
    (bars/B22 carries no MoCap markers).`), then `[servo 0] tool0 pos L=a R=b mm` (a few mm: the base gives under the
    arms) and the prompt `<bar>_J_transfer — SERVO 1/8 onto the insert start (tool0, or the MoCap bar B20): ..., N rows,
    travels X° (...), T s; flanges move L a mm, R b mm. Confirm to move; Cancel stops and records.`
- [ ] slider [Preview time]: scrub the pass (a few mm with the bar held) → button [Confirm]
  - **Expect:** later passes run alone unless one travels over 5° (`SAFEGUARD servo k/8: ...`: scrub, Confirm). Each
    pass logs `[servo k] tool0 pos L=... R=... mm` (B20 and B21 also `[servo k] bar bars/B2x: MoCap-vs-design ...,
    FK-vs-design ..., MoCap-vs-FK ...`). It ends with one of: `converged on tool0 (< 0.2 mm)`; for B20
    `converged on the MoCap bar: dpos ... mm, dang ... deg (< 0.5 mm / 0.1 deg)` and `[servo] the MoCap-bar
    correction is carried into the plan of B20_J_insert.`; `nothing left to plan (correction L ... mm, R ... mm, under
    0.2 mm)`; or `ran all 8 passes without converging` (the old monitor on B20: the right flange stays 1–3 mm off, the
    bar as mounted by hand; harmless).
    Then `### bar pose at termination (bars/<bar>): FK-vs-design ..., MoCap-vs-design ..., MoCap-vs-FK ...` and
    `Exec <bar>_J_transfer: done — every part done`. The exec section's last line keeps `servo done: ...`.
  - **If not:** `[servo] no marker sample of bars/B20 (asset 1020) within 1 s: is Motive streaming it?`: B20's balls are
    not streamed (2.f); the servo aims tool0 and says so.

**e) Insert** (`<bar>_J_insert (insert)`)
- [ ] button [Plan] (about 3 s)
  - **Expect:** `insert solved by tamp.plan_constrained_dual_arm_linear: 4 waypoints` (B20 after its MoCap-bar servo:
    `[insert plan] MoCap-bar correction carried: ...`); it starts where the servo left the arms.
- [ ] optional, the first insert of its kind on the robot (B21, B22): box [Dry run]: ticked → button [Exec]
  - **Expect:** `DRY RUN a200-0806: insert: every compliant target passed the arm's checks, NOT SENT`. Then box
    [Dry run]: unticked.
- [ ] hand on Cindy's e-stop → button [Exec]: the insert runs at once
  - **Expect:** depends on the bar (table below).
  - **!** Watch the bar enter its joint, and every held bar stay in its jaws.
  - **If not:** `NO_STALL` after about 31 s: the screw never stalled (the bar not in its joint?): note the line; the
    arms are back under joint control. `ARMS MAY BE LEFT UNDER cartesian_compliance_controller`: 7.b at once.

| bar | the insert |
|---|---|
| **ground bar** (B20) | `B20_J_insert — RIGID insert of a ground bar, no tighten, 5.0 s, largest joint travel ...`: joint control throughout, no screw turns; `insert: RIGID_DONE: ground bar in, no tighten; the flanges stopped ... mm from the planned end; peak ...`. Then, in the same Exec, the **servo after the insert**: the prompt `B20_J_insert — SERVO 1/8 onto the insert end (the MoCap bar B20): ...` → scrub, Confirm; straight lines only, each pass with `force change L ... R ... N` on the panel line; it stops by itself when a flange's force changes by over 20 N (`the force on an arm changed by ... N since the first reading (limit 20 N); the bar may be pushed against something`). Then `Exec B20_J_insert: done` |
| **one-mate** (B21) | `B21_J_insert — INSERT: rigid 10.0 mm (3.3 s), then COMPLIANT 5.0 mm (straight line); the joint screw of left_ur_arm TIGHTENS from the start until it stalls (≤ 30 s), then STOP and back to joint control. Watch the bar enter its joint and the held bars stay in their jaws.`; `insert: left_ur_arm joint screw TIGHTENING`, the rigid part, `left_ur_arm, right_ur_arm: cartesian_compliance_controller running (...)`, the compliant line; at the stall `left_ur_arm joint screw STALLED at ...; left_ur_arm back under joint control, right_ur_arm finish the line under compliance`; the right arm settles against the force (at most 3 s) and goes back too; `insert: STALLED: left_ur_arm joint screw STALLED; peak ...`. No servo after it: it is not a ground bar |
| **two-mate** (B22) | as B21, with `the joint screw of left_ur_arm + right_ur_arm TIGHTENS ...`; it ends when both have stalled. No servo after it |

**f) Mark**
- [ ] by eye: the line under the buttons says `every step of <bar>_J done: Mark done below` → button [<bar>_J done]
  - **Expect:** `<bar>_J marked done in run ...`, the row turns green `· done`, the count goes up by one, and the
    current step moves to the next action.

### 6.3 Procedure R — release (Cindy)

Two steps: the retreat (the ungrasp runs with it), then home.

**a) Retreat** (`<bar>_R_retreat (retreat_linear) · with: <bar>_R_ungrasp`)
- [ ] button [Plan]
  - **Expect:** `retreat_linear solved by tamp.plan_dual_arm_linear_independent: N waypoints`; it plans from where the
    arms are.
- [ ] optional, the first retreat on the robot (B20_R): box [Dry run]: ticked → button [Exec]
  - **Expect:** `DRY RUN ...: retreat: passed every check, NOT SENT; its tail is replanned when it runs`. Then box
    [Dry run]: unticked.
- [ ] hand on Cindy's e-stop → button [Exec]: it runs at once
  - **Expect:** `<bar>_R_retreat — RETREAT: the M1 screws of left_ur_arm + right_ur_arm OPEN; force sensors zeroed;
    COMPLIANT break-out 5.0 mm (0.5 s); joint control; the rest replanned from where the tools are and run at once (4.5
    s). Watch both tools let go and back off along their lines.`; then `retreat: the gripper screws of left_ur_arm +
    right_ur_arm OPEN`; `force sensors zeroed`; the switch to `cartesian_compliance_controller`; the 0.5 s break-out;
    `back under scaled_joint_trajectory_controller`; the tail replanned (`retreat tail <bar>_R_retreat: ... waypoints`);
    the checks `PASS` / `FAIL` (start, break-out, contact, grippers, both tool0 paths, joints, speed); `running the tail
    now (N rows, T s)`; `retreat: RETREATED: retreated` and `Exec <bar>_R_retreat: done — every part done`. The **Exec
    plots** show the part's force and torque.
  - **!** Watch both tools let go before they back off.
  - **If not:** the checks only report, the tail runs anyway; note every `check FAIL` line. `grippers: ... LOOSENING
    (want STALLED = fully open)`: a gripper screw did not open fully (in the fake lab the simulated screws never stall,
    so this check always failed there; on the real tools it should pass): look at the tool before the next move.
    `contact: |F| ... N (limit 20 N)`: a tool still pushed on the bar. `break-out: ... short of its end`: the arm sagged
    or caught.

**b) Home** (`<bar>_R_home (free_home)`)
- [ ] button [Go: mv ▶] (the process section): the step line reads `<bar>_R_home (free_home)` → button [Plan] (20–40 s)
  - **Expect:** `free_home solved by tamp.plan_free_dual_arm`, `min T s by the arms' limits`.
- [ ] button [Path: Play] → slider [Motion time] → hand on Cindy's e-stop → button [Exec]
  - **Expect:** `<bar>_R_home — move left_ur_arm, right_ur_arm along the planned path ..., T s, largest joint travel
    D°`, then `<bar>_R_home: arrived, end within ...°`.

**c) Mark**
- [ ] `every step of <bar>_R done` → button [<bar>_R done]

**!** Mark done accepts an action with steps left: wait for `every step of ... done` before clicking it, or the next
base plan refuses because an arm still stands at the bar.

### 6.4 Procedure H — hold (Alice, B21)

Four steps: approach (and its servo), open, straight in, close with the compliant handoff.

**a) Base:** Procedure B (6.1) with Alice (`a200-0804`).

**b) Approach** (`B21_H_approach`)
- [ ] button [Live IK: linear start] (the first plan for Alice builds her planning cell, about 7 s)
  - **Expect:** `check goal B21_H_approach: goal found (ssik, rank 1 of 8), max |goal - authored| X rad`.
- [ ] button [Plan] → button [Path: Play]
  - **Expect:** `support_approach solved by arm.plan_arm: N waypoints`; the preview ends with the jaws in front of B21,
    clear of Cindy's arms and tools.
- [ ] hand on Alice's e-stop → button [Exec]
  - **Expect:** `alice (a200-0804): B21_H_approach — move ur_arm along the planned path ..., T s, largest joint travel
    D°`, `B21_H_approach: arrived, end within ...°`; then the servo: `[servo] feedback: tool0 (0.2 mm); no MoCap bar
    recorded (no bar held).`, `[servo 0] tool0 pos arm=X mm` and the prompt `B21_H_approach — SERVO 1/8 onto the
    approach pose (tool0): ...`.
- [ ] slider [Preview time]: scrub → button [Confirm]
  - **Expect:** passes until `converged on tool0 (< 0.2 mm)` or `nothing left to plan`; `Exec B21_H_approach: done —
    every part done`.

**c) Calibration check**
- [ ] by eye: the gripper stands about 100 mm in front of B21, **centred** on it
  - **If not:** off by more than 1–2 cm sideways: stop. Alice's base calibration is off (the relay's table, 2.f, names
    the session it applied); recalibrate her before going on.

**d) Open** (`B21_H_open`)
- [ ] button [Exec] → the prompt `B21_H_open — OPEN the gripper (0.426). Confirm to send.` → button [Confirm]
  - **Expect:** `B21_H_open: at 0.426 rad`.

**e) To grasp** (`B21_H_to_grasp`)
- [ ] button [Plan] → button [Path: Play] → hand on Alice's e-stop → button [Exec]
  - **Expect:** `B21_H_to_grasp — move ur_arm ..., 5 s, largest joint travel ...°` (22° in the fake lab), then
    `arrived, end within ...°`: the open jaws slide onto B21.

**f) Close and handoff** (`B21_H_close`)
- [ ] button [Exec] → the handoff prompt (5.c) → hand on Alice's e-stop → button [Confirm]
  - **Expect:** the lines of 5.c, now on the bar: the arm compliant for the whole close, giving way while the jaws
    centre on B21; the gripper stalls on the bar; back under joint control.
  - **!** Watch Cindy's tools: they must not be pulled.

**g) Mark**
- [ ] button [B21_H done]
  - **Expect:** `B21_H marked done in run ...`, `4/78 done`, the current step `B21_R · B21_R_ungrasp (1/3)`.

### 6.5 The seven actions

| # | action | procedures | what differs |
|---|---|---|---|
| 1 | `B20_J` | B (Cindy), J, ground bar | base target `+0.527 -3.067 +89.6°`, about 5 m from the park row (*fake lab:* 5.24 m planned, about 50 s driven). The servo steers on the **MoCap bar** (Servo by bar; B20's balls must stream, 2.f). The insert is rigid, then the servo after the insert. Mark: `1/78 done`, current `B20_R · B20_R_ungrasp (1/3)` |
| 2 | `B20_R` | R | the first compliant part on a real robot: the Dry run first. Mark: `2/78`, current `B21_J · B21_J_transfer (4/5)` |
| 3 | `B21_J` | B (Cindy), J, one-mate | base target `+0.947 -3.171 +90.0°`, about 0.8 m. The servo on tool0, B21's MoCap fit recorded alongside. The first compliant insert: the Dry run first. The left joint screw tightens into B20. Mark: `3/78`, current `B21_H · B21_H_approach (1/4)` |
| 4 | `B21_H` | B (Alice), H | base target `+0.638 -1.561 -29.6°`, about 3 m. Mark: `4/78`, current `B21_R · B21_R_ungrasp (1/3)` |
| 5 | `B21_R` | R, Alice holding B21 | B21 is clamped by Alice: the ungrasp releases Cindy's tools from it. In the retreat's preview, check that both tools back off clear of Alice's gripper. Mark: `5/78` |
| 6 | `B22_J` | B (Cindy), J, two-mate | base target `+1.166 -3.212 +95.8°`, a short path. The transfer's plan from the retreat pose took up to 110 s in the fake lab; `no path in 120 s`: Plan again. The servo on tool0 (B22 has no MoCap balls). Both joint screws tighten, into B21 and B20. **!** Watch B21 while B22 screws in: Alice holds it, and it must not be pushed out of her jaws. Mark: `6/78` |
| 7 | `B22_R` | R | Mark: `7/78`, and the current step is `B34_J`: the first three bars are built, and the next bar's robot is Cindy again |

### 6.6 Optional: Alice lets go of B21

After action 7, Alice still clamps B21; in the process she lets go at entry 17 (`B21_HR`), after B34–B37. To end the
test here, run her let-go out of order:

- [ ] Assembly › process: button [Go: action ▶] until the step line names `B21_HR` (nine clicks from `B34_J`; the
      ACTIONS list shows only the seven rows around the selected action) → movement button [HR_M0] (`B21_HR_open`)
      → button [Exec] → button [Confirm]
  - **Expect:** `B21_HR_open: at 0.426 rad`. B21 stays where it is: it hangs on its joints to B20 and B22.
- [ ] movement button [HR_M1] (`B21_HR_retreat`) → button [Plan] → button [Path: Play] → hand on Alice's e-stop
      → button [Exec]
  - **Expect:** the open jaws slide off B21, back to the approach pose (100 mm).
  - **If not:** the plan refuses because of a bar that is not built yet: the planning scene takes the design's state
    at entry 17, B34–B37 included. Box [Ignore built bars]: ticked (B21, the action's own bar, still counts), Plan
    again, and untick it afterwards.
- [ ] movement button [HR_leave] (`B21_HR_leave`) → button [Plan] → button [Path: Play] → button [Exec]: her arm
      away from the structure
- [ ] Optional: button [B21_HR done] (the run then says B21 is let go), and drive her back: Procedure B with
      button [Load: Park] instead of Load Step.

### 6.7 Stop

1. pane monitor: `Ctrl-C` once and wait for `Server stopped` (the plugins send their final holds).
2. Each robot's pane: `Ctrl-C` her launch, **only when she holds nothing** (Alice clamped on B21: let her go first,
   6.6, or leave her launch running). **Expect:** `real lab: the session on <robot> ended ...`.
3. pane relay: `Ctrl-C`.
4. `tmux kill-session -t real_lab`.
5. Pendants: stop `ros_control.urp` before switching a pendant back to Local (crl-husky README, Startup). To power a
   robot off: `sudo shutdown now` in an ssh shell on her first, then the power button.

---

## 7. Reference

### 7.a Driving a base by hand

- **The page:** Control › robot › Base: hold a drive button, at the Speed set. `Clearance` (ticked) soft-stops the base
  when it comes near anything in the scene; untick it only to drive out of a tight spot. The phone's Control tab has
  the same D-pad.
- **The joystick:** L1 or L2 held, the left stick.
- **The keyboard,** in pane checks (domain 80 is loaded there):

**▶ pane checks**
```bash
# Cindy; Alice: /a200_0804
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/a200_0806/joy_teleop/cmd_vel
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
```

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
- Done: every flange within 0.2 mm of its target (tool0); or, steering on the MoCap bar (B20 with Servo by bar), the
  bar's fit within 0.5 mm / 0.1° of its design pose; or `nothing left to plan` (the correction is under what the planner
  moves).
- The panel's last exec line shows each reading, and after the Exec how the servo ended, until the next Exec.
- Each servo is recorded as `runs/<run>/servo/<action>/<movement>.<n>.json`, with a `.png` of its readings.

### 7.e Dry run

Settings › box [Dry run]: ticked, every check runs and nothing is sent: a motion logs `DRY RUN <serial> <arm>: passed
every check, NOT SENT`, an insert `DRY RUN ...: insert: every compliant target passed the arm's checks, NOT SENT`, a
retreat `... retreat: passed every check, NOT SENT; its tail is replanned when it runs`, the handoff its anchor gap.
The panel shows a `DRY RUN` chip while it is ticked. Untick it before the real Exec.

### 7.f The run record

The run's folder: `.../261008_2ModulesDemo_purged_process/runs/<date>_<time>/` on the Drive (`doc/run_record.md`):
`run.json`, `progress.json` (the marks, where each robot was left), `events.jsonl`;
`movements/<action>/<movement>.<n>.json`, one file per part, the refused and cancelled ones included (an insert's or a
retreat's keeps its settings, phases, switches, stall times, peak force, end errors and checks);
`servo/<action>/<movement>.<n>.json` and its `.png`.

**Restarting the monitor** keeps the run: pane monitor `Ctrl-C` once, wait for `Server stopped` twice, Up, Enter;
reload the page, button [Load]. **Expect:** `continuing run <date>_<time>`, the done actions still green. The robots
keep where they are.

### 7.g tmux and the terminals

tmux keys: press `Ctrl-b` (the "prefix"), let go, then the key.

| keys | does |
|---|---|
| `Ctrl-b %` | split **left / right** (the new pane opens on the right) |
| `Ctrl-b "` | split **up / down** (the new pane opens below) |
| `Ctrl-b` arrow, or `Ctrl-b o` | go to the pane in that direction / the next pane |
| `Ctrl-b z` | zoom the current pane to the whole window, and back |
| `Ctrl-b x` | close the current pane (answer `y`); see the box at the top first |
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
| `monitor.sh` | the monitor: the three robots, the fake lab's plugins, the process from the Drive; refuses while another monitor runs in domain 80 (it checks once, at its start), or when ROS does not answer |
| `check_robots.sh [robot ...]` | Part 3's read-only checks |

Settings: given to `tmux.sh` when it makes the session, they reach every pane (`REAL_LAB_MOTIVE=192.168.0.117
tmux.sh`); given to one script in its pane, only that one (`REAL_LAB_MOTIVE=192.168.0.117 relay.sh`). Running
`tmux.sh` again with a setting changes nothing: the session exists, so it only re-attaches.

| setting | default | for |
|---|---|---|
| `REAL_LAB_MOTIVE` | `192.168.0.28` | the Motive PC (it is to move to `.117`) |
| `REAL_LAB_DRIVE` | the Insync copy of `2025-03 Husky Assembly` | another Drive folder |
| `REAL_LAB_HOME` | `~/husky_real_lab` | where the logs go |
| `REAL_LAB_SESSION` | `real_lab` | another tmux session name |

---

## 8. If something goes wrong

### 8.a Starting over / redoing an action

- **A step waits and you want out:** button [Cancel].
- **A motion must stop now:** the robot's remote e-stop; or `Stop all (Esc)` (each robot then waits for its `Resume`
  under Health).
- **Redo an action:** select it → button [Reopen] (pending again) → its steps again from where the robots stand.
- **Start the whole test over:** button [Run: New run], then once more when it reads `New run: click again`.
  Nothing is deleted: the old run's folder stays.
- **The monitor hangs or the page shows nothing new:** reload the page first; then restart the monitor (7.f).

### 8.b Problems and what to do

| situation | do |
|---|---|
| a robot's chips red, nothing from her | `check_robots.sh <robot>` (Part 3): her network, the router link, her launch |
| topics or arm control stall; a router's log says `Unable to push non droppable ... Closing transport!` | restart that machine's router: on a robot `crl-husky-zenoh-restart`; on this laptop `systemctl --user kill -s KILL crl-zenoh-router` (it restarts itself) |
| `Timeout for service ...` or `Didn't receive final reply for query`, often minutes after the call | stale routes after a crash: restart the routers on every robot and on this laptop |
| nodes die at start with `undefined symbol` in `librmw_zenoh_cpp.so` | `ros-humble-zenoh-cpp-vendor` older than `ros-humble-rmw-zenoh-cpp`: `sudo apt install ros-humble-zenoh-cpp-vendor ros-humble-rmw-zenoh-cpp` |
| `monitor.sh`: `a monitor already runs in ROS domain 80` | another PC's monitor (or this laptop's in another terminal): stop it first |
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
| `no marker sample of bars/B20 (asset 1020)` | Motive does not stream B20's balls (2.f): the servo aims tool0 |
| the jaws not centred on B21 after the approach | Alice's base calibration is off (6.4 c): stop before the straight approach |
| B20's ground feet sit 15.6 mm into the floor at their design pose | a Rhino-side fact; the planner lets a half mated to the ground touch the lab floor |
| Rhino's cabinet `env39` is 0.38 m nearer the park row than the measured one | a Rhino-side fix; with `Rhino obstacles collide` off (the default) it is drawn only |
| a robot's pane frozen after she rebooted | Enter, then `~` and `.`; then Enter again for `robot.sh` |

---

## 9. What to send back

Tell me the step numbers that did not go as written, and what you saw (e.g. "6.2 e) not OK on B21: ..."). A screenshot
helps for anything on the page. I read the panes and the logs myself:
- `~/husky_real_lab/logs/`: the monitor's log and the relay's and each robot's pane;
- the run: `.../261008_2ModulesDemo_purged_process/runs/<date>_<time>/` (movements, servo files, events).

Worth noting as you go:
- **Part 3:** any FAIL or NOTE, and what fixed it.
- **5.a:** how each base drove its short path, and how far from the target it stopped.
- **5.c and 6.4 f):** the anchor gap (`anchor X mm, Y°`), the handoff's lines, and how Alice's arm behaved while
  compliant (did it stay still? Did Cindy's tools get pulled?).
- **Each J action:** the Live IK `max |goal - authored|`; how the servo ended (and its passes); how the insert ended
  (stall times, peak force, any warning); whether the bar sat in both tools after you clamped it.
- **B20:** the MoCap-bar servo's ending and the `### bar pose at termination` line; the servo after the insert (its
  force changes).
- **Each R action:** the retreat's `check` lines (the FAIL ones with their numbers), and whether the tools let go and
  backed off cleanly (B21_R: clear of Alice's gripper).
- **B22:** whether B21 stayed in Alice's jaws while B22 screwed in.
