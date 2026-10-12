# Cindy + Alice three-bar test (B20, B21, B22) — with the real robots, run from the robot-control PC

> **Run from the robot-control PC** (`CRLRobotxPC`, user `su`). The laptop manual
> `doc/cindy_two_bar_test_manual.md` still describes the earlier two-bar export (B1, B3, Alice
> replaced by a stand); this file is the current one.
>
> | | this PC |
> |---|---|
> | workspace | `~/ros2_ws` (venv `~/ros2_ws/venv`, ssik 4.1 installed) |
> | shared drive | `~/Insync/2025-03 Husky Assembly` (the package's built-in data paths already point here) |
> | robot network | built-in ethernet port **`eno1`**, DHCP from the router `192.168.0.1` (was `192.168.0.25` on 2026-10-07) |
> | DDS | `~/.cyclonedds.xml` pins CycloneDDS to `eno1`; `~/.bashrc` exports `CYCLONEDDS_URI` but **not** `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` |
> | internet | Wi-Fi `wlp9s0` (+ Tailscale); ROS does not use them |

A standalone checklist for the design problem **`261006_3bar_holding_test`** (re-exported
2026-10-08 as the range **B20–B22**). Two robots: **Cindy** (dual-arm husky `/a200_0806`, ROS
domain 86) joints and releases the three bars; **Alice** (single-arm husky `/a200_0804`, domain 84,
Robotiq 2F-85 gripper) grabs B21 after Cindy has inserted it and keeps holding it until the end.

| entry | action | who | operator steps |
|---|---|---|---|
| 0 | `B20_J_joint` | Cindy | B20 jointing (ground bar): transfer → mount → grasp → insert (rigid, no tighten) → **fix foundation** (manual) |
| 1 | `B20_R_release` | Cindy | B20 release: retreat (the ungrasp runs with it) → free move home |
| 2 | `B21_J_joint` | Cindy | B21 jointing (one-sided, the **left** joint mates): transfer → mount → grasp → insert (the left tighten runs with it) |
| 3 | `B21_H_hold` | **Alice** | Alice grabs B21: free approach → gripper open → straight approach (100 mm) → gripper close + compliant handoff |
| 4 | `B21_R_release` | Cindy | B21 release, Alice holding B21 |
| 5 | `B22_J_joint` | Cindy | B22 jointing (both joints mate, to B20 and B21): transfer → mount → grasp → insert (both tightens run with it) |
| 6 | `B22_R_release` | Cindy | B22 release |

**One robot per monitor run.** The monitor drives the robot whose `ROS_DOMAIN_ID` it starts with
and draws the other two (from their live mocap base, or from what `progress.json` remembers). So the
session runs three monitor runs in turn: **Cindy** (entries 0–2) → **Alice** (entry 3) → **Cindy**
(entries 4–6). Quitting a monitor leaves the robot's driver running: the arms stay where they are
and keep holding. Before the assembly, Alice's gripper and handoff are checked once in the air
(section 6).

> **! Transfer workaround in use** (until the bar-held transfer planner is ready). The transfer
> runs as a plain free move **without the bar**: tick
> `Workaround: plan the transfer as a free move (no bar, from the live arms)` (under
> `Plan Movement`) once per Cindy monitor run — it starts unticked, so tick it again after the
> restart for entries 4–6. With the box ticked,
> `Transfer start: Confirm manual pose (IK check)` checks whether the arms reach the insertion
> start (the Rhino pose) from where the base stands; the `Transfer start:` sliders (the bar-loading
> pose) are not used and disappear from the panel while the box is ticked.
> Order of a jointing entry:
> **transfer** (empty tools, from the live arms to the insertion start) → **manual mount** (mount
> the bar by hand there) → **tool grasp** → **insert** (→ **fix foundation** for B20). The travel
> to load is skipped.

Every code block below is meant to be pasted as is. Tick the boxes as you go; anything that does
not match "expect" is worth a note (log lines + what you did). What the buttons do in detail:
`doc/support_robot_schedule_manual.md`.

---

## 0. Once, before the session

### 0.1 Monitor flags, the mocap address and the base calibrations

The switches are class attributes at the top of `class HuskyMonitor`
(`husky_assembly_teleop/husky_monitor.py`; no rebuild needed after editing, restart the monitor).
The same values serve Cindy's and Alice's runs.

```bash
F=~/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_monitor.py
grep -nE "^    (FAKE_HARDWARE|USE_MOCAP|USE_CELL_STATE_BASE_POSE|CONNECT_COMPLIANT_CONTROLLER|LIST_CONTROLLER_SERVICES|IGNORE_BUILT_ASSEMBLY_COLLISIONS|TRANSFER_AS_FREE_MOVE) *=" "$F"
grep -nE "^(CLIENT_IP|MOCAP_IP) *=" "$F"
ip -4 -brief addr show eno1        # this PC's address on the robot network
```

| flag | value for this test |
|---|---|
| `FAKE_HARDWARE` | **0** |
| `USE_MOCAP` | **1** |
| `USE_CELL_STATE_BASE_POSE` | 0 |
| `CONNECT_COMPLIANT_CONTROLLER` | **1** (Cindy's insert / retreat and Alice's grip handoff switch controllers) |
| `LIST_CONTROLLER_SERVICES` | **1** |
| `IGNORE_BUILT_ASSEMBLY_COLLISIONS` | 0 (built bars are obstacles; leave the panel toggle unticked, also for Alice: her B21 hold plans with them, checked headless) |
| `TRANSFER_AS_FREE_MOVE` | 0 (tick the workaround box in the panel instead) |
| `CLIENT_IP` (module level) | **this PC's `eno1` address** (`192.168.0.25` on 2026-10-07) |
| `MOCAP_IP` (module level) | `192.168.0.28` (the Motive PC) |

! `CLIENT_IP` is where the monitor listens for the mocap stream, so it differs per PC: the
uncommented line must be this PC's `eno1` address (`# husky PC`, `192.168.0.25`), not the Razer
laptop's (`192.168.0.29`, kept as a comment). With the wrong one the monitor prints
`mocap client connected: False`. If `eno1`'s address moved, set the line to it:

```bash
F=~/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_monitor.py
IP=$(ip -4 -o addr show eno1 | awk '{print $4}' | cut -d/ -f1)
sed -i "s/^CLIENT_IP = '[0-9.]*'/CLIENT_IP = '$IP'/" "$F"
grep -n "^CLIENT_IP" "$F"          # expect: CLIENT_IP = '192.168.0.x' # husky PC ...
```

Both addresses are DHCP leases and can move: see `doc/calibration_manual.md` section 1.4.

**Base calibrations** (mocap marker body → robot base), set in `husky_assembly_teleop/__init__.py`:
each robot reads its own date folder in `BASE_CALIBRATION_DATE_BY_ROBOT`, else `CALIBRATION_DATE`.

```bash
I=~/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/__init__.py
grep -nE "^CALIBRATION_DATE|^BASE_CALIBRATION_DATE_BY_ROBOT|^    '(Alice|Belle|Cindy)'" "$I"
C="$HOME/Insync/2025-03 Husky Assembly/data_experiment/calibration_data"
ls "$C"/20260916/calibrated_transformation_0806_rhino.json "$C"/20260623/calibrated_transformation_0804_rhino.json
```

- Cindy (0806): `20260916` (`CALIBRATION_DATE`). Alice (0804): `20260623` (her entry
  `'Alice': '20260623'`, her newest calibration).
- ! Alice's June calibration holds only if her base markers have not moved since. The first check
  of it is where her open jaws end after the free approach (section 9).
- Without a file the robot runs **uncalibrated**: its base is the marker body, cm off. As the
  connected robot it then plans from the wrong place; as another robot it is not used as a live
  obstacle (falls back to `progress.json`).

### 0.2 The problem folder (shared Insync folder)

The monitor reads and writes the design problem directly in the shared Insync folder:
`Mark entry done` writes `progress.json` and `*.live-solved.json` (saved plans) there, and they sync
to the shared drive. Both robots' runs read and write the same `progress.json`: that is how Cindy's
second run knows where Alice stands. Check that the folder is complete and see what an earlier run
left behind:

```bash
GD="$HOME/Insync/2025-03 Husky Assembly"
P="$GD/data_design_study/261006_3bar_holding_test"
ls "$P"                                        # ActionSchedule.json BarActions RobotCell.json RobotCell_Alice.json WalkableGround.json ...
ls "$P"/BarActions/*__*.json | grep -vc live-solved     # 7 action files (B20__J/R, B21__J/H/R, B22__J/R)
ls -a "$P" | grep -c insyncdl                  # 0 = Insync has finished syncing
ls "$P"/progress.json "$P"/BarActions/*.live-solved.json 2>/dev/null   # left over from an earlier run?
```

If `progress.json` or saved plans are left over and you want to start fresh, use
`Reset schedule to the Rhino export` once Cindy's monitor is up (section 15): it moves them to
`$P/archive/<time>/` (nothing is deleted).

### 0.3 Robot bring-up (robot side)

**Cindy**

- [ ] Both arms powered on, teach pendants in **external control** mode (`doc/husky_ros2.md`; do
      not start `ros_control.urp` by hand).
- [ ] Arms + scaffolding tools launched on Cindy's PC. Open a separate tmux window for it
      (`Ctrl-b c`; `Ctrl-b n` cycles windows) and log in. This PC's ssh key is accepted, so no
      password is asked (if it is: `doc/husky_ros2.md`):

```bash
ssh administrator@192.168.0.115
```

Then, **in that ssh shell on Cindy**:

```bash
# * Cindy's ROS must run CycloneDDS on domain 86, or this PC sees none of her topics.
# * Her login shell already sets both; the export only makes sure.
export ROS_DOMAIN_ID=86 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "[cindy] domain $ROS_DOMAIN_ID | $RMW_IMPLEMENTATION"   # expect: domain 86 | rmw_cyclonedds_cpp
# Both arms + both scaffolding tool drivers (the default gripper:=none starts no tool driver).
ros2 launch crl_husky crl_dual_ur5e.launch.py namespace:='/a200_0806' gripper:=scaffolding_v3
```

**Alice**

- [ ] Arm powered on, teach pendant in **external control** mode; her own remote e-stop paired
      and in reach (start one husky at a time, `doc/husky_ros2.md` troubleshooting).
- [ ] Arm + Robotiq gripper launched on Alice's PC, in another tmux window. Password `clearpath`
      if asked:

```bash
ssh administrator@192.168.0.113
```

Then, **in that ssh shell on Alice**:

```bash
export ROS_DOMAIN_ID=84 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "[alice] domain $ROS_DOMAIN_ID | $RMW_IMPLEMENTATION"   # expect: domain 84 | rmw_cyclonedds_cpp
# The arm + the Robotiq 2F-85 gripper driver.
ros2 launch crl_husky crl_single_ur5e.launch.py namespace:='/a200_0804' gripper:=robotiq_2F_85
```

Leave both launches running for the whole session; `Ctrl-C` in a window stops that robot's arm
and gripper / tool drivers (! never while that robot holds a bar). Section 2 checks that this PC
sees them.

- [ ] Motive streams Cindy's rigid body **1860** and Alice's **1840**.

### 0.4 This PC's network

- [ ] This PC is on the robot network through its built-in port `eno1` (cable to the robot
      router; NetworkManager profile `Wired connection 1`, DHCP). No USB dongle is needed:

```bash
ip -4 -brief addr show eno1                    # expect: eno1  UP  192.168.0.x/24
for h in 192.168.0.115 192.168.0.113 192.168.0.28; do   # Cindy's PC, Alice's PC, the Motive PC
  printf "%s: " $h; ping -c1 -W1 $h >/dev/null && echo up || echo NO REPLY
done
grep -o 'NetworkInterface name="[^"]*"' ~/.cyclonedds.xml   # expect: name="eno1"
```

- `eno1` DOWN or without a `192.168.0.x` address: check the cable, then
  `nmcli connection up "Wired connection 1"`.
- Leave the Wi-Fi on: the default route for the internet goes through it, and the `192.168.0.x`
  traffic still goes through `eno1`. Tailscale does not get in the way either.
- The NetworkManager profile `cindy` (fixed `192.168.131.19` on a USB dongle) is the old direct
  cable to Cindy's PC; it is not used for this test.
- If `eno1`'s address changed since the last run, update `CLIENT_IP` (0.1).

### 0.5 Rebuild the package (after pulling or changing code)

Builds only `husky_assembly_teleop`, not the whole workspace:

```bash
cd ~/ros2_ws && source venv/bin/activate
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash
```

Editing an existing `.py` file alone does not need it (the install links to the source; restart
the monitor). New files, new entry points or `setup.py` changes do.

---

## 1. Terminals

Use one tmux session with two panes: the **monitor** pane and a **checks / base** pane.

```bash
tmux new -A -s husky_real     # -A: attach if the session already exists. Ctrl-b "  splits it; Ctrl-b o switches pane
```

The mouse works in tmux on this PC (`~/.tmux.conf`): the wheel scrolls back through a pane's output
(`q` returns to the live end) and a click picks a pane. A drag selects text (yellow); **releasing the
mouse copies it** to the system clipboard and keeps it highlighted. `Ctrl-C`, `Enter` or `y` on the
selection copy it too and leave the selection; `q` / `Esc` leave without copying. Paste anywhere with
`Ctrl-Shift-V`. Hold **Shift** while dragging for the terminal's own selection (copy it with
`Ctrl-Shift-C`).

### 1.1 Monitor pane — paste before each monitor run

The domain decides which robot the monitor drives. Paste the block of the robot you start next
(the monitors run one after the other in the same pane; quit one with `Ctrl-C` before the next).

**Cindy** (entries 0–2, then 4–6):

```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=86
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml   # already in ~/.bashrc; pins DDS to eno1
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
ros2 daemon stop >/dev/null 2>&1
echo "[cindy env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}') | problem $DESIGN_PROBLEM_NAME"
```

**Alice** (the air checks in section 6, entry 3):

```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=84
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
ros2 daemon stop >/dev/null 2>&1
echo "[alice env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}') | problem $DESIGN_PROBLEM_NAME"
```

- No data-path exports: the package's built-in `DESIGN_DATA_DIRECTORY` / `EXPERIMENT_DATA_DIRECTORY`
  are this PC's `~/Insync/2025-03 Husky Assembly/...` folders. Only the problem name is set (the
  built-in default may be another problem).
- No `HUSKY_IK_BACKEND`: ssik is installed in this venv, so the default (ssik) is used.
- Keep a log of the monitor (from any other pane, once per session; it keeps logging across the
  monitor runs): `mkdir -p ~/husky_real && tmux pipe-pane -t husky_real:0.0 -o 'cat >> ~/husky_real/monitor.log'`.

### 1.2 Checks / base pane — paste once per new terminal, set the domain per robot

```bash
source ~/ros2_ws/install/setup.bash   # ROS + the workspace's message types (no venv needed)
export ROS_DOMAIN_ID=86               # 86 = Cindy, 84 = Alice
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml   # already in ~/.bashrc; pins DDS to eno1
ros2 daemon stop >/dev/null 2>&1
echo "[robot env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}')"
ros2 topic info /a200_0806/cmd_vel --no-daemon --spin-time 5   # Cindy; Alice: /a200_0804/cmd_vel. Expect: Subscription count: 1 (twist_mux)
```

To switch robot in this pane: `export ROS_DOMAIN_ID=84` (or `86`), then `ros2 daemon stop`.

`Unknown topic` = the robot is not visible: wrong domain, its ROS is not on CycloneDDS, `eno1` is
off the robot network, or the base driver is down.

! Source the **workspace**, not only `/opt/ros/humble`: Cindy's tool status topics use
`crl_husky_msgs/msg/ScaffoldingToolStatus`, which only the workspace has. Without it `ros2 topic hz`
cannot read them (`The message type ... is invalid`) even when the tools publish fine.

---

## 2. Connectivity checks (checks pane, after 1.2)

The controller checks call each controller manager's service directly.

**Cindy** (`export ROS_DOMAIN_ID=86; ros2 daemon stop`):

```bash
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
done
```

- [ ] Joint states ≈ **50 Hz** (both arms), tool status ≈ **20 Hz** (both tools).
      `The message type 'crl_husky_msgs/msg/ScaffoldingToolStatus' is invalid` = this pane sourced
      only `/opt/ros/humble`: paste 1.2 again.
- [ ] Both arms: `scaled_joint_trajectory_controller` **active**, `cartesian_compliance_controller` **inactive**.

**Alice** (`export ROS_DOMAIN_ID=84; ros2 daemon stop`):

```bash
echo "--- joint states"; r=$(timeout 6 ros2 topic hz /a200_0804/ur5e/rate_limiter/joint_states --window 20 2>&1 | grep 'average rate' | tail -1); echo "${r:-NO MESSAGES}"
echo "--- gripper action"; ros2 action list | grep gripper_cmd || echo "NO GRIPPER ACTION"
echo "--- controllers"
timeout 15 ros2 service call /a200_0804/ur5e/controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers 2>&1 | tr ',' '\n' \
    | grep -oE "name='[a-z_]+'|state='[a-z]+'" | paste - - | grep -E 'scaled_joint|cartesian_compliance'
```

- [ ] Joint states publish; the gripper action `/a200_0804/gripper/robotiq_gripper_controller/gripper_cmd`
      is listed (`NO GRIPPER ACTION`: the launch is missing `gripper:=robotiq_2F_85`, 0.3).
- [ ] `scaled_joint_trajectory_controller` **active**, `cartesian_compliance_controller` **inactive**.

---

## 3. Driving a base from the command line (when the joystick does not connect)

In the checks pane (1.2), with the domain of the robot you drive. The commands below drive Cindy;
for Alice use `ROS_DOMAIN_ID=84` and replace `/a200_0806` with **`/a200_0804`**. The base listens
on `<ns>/cmd_vel` through `twist_mux`:
- **The base stops 0.5 s after the last message**, so commands are published continuously;
  stopping the publisher stops the base.
- The joystick, the RC remote and the interactive marker **override** `cmd_vel` while they
  publish; the Husky e-stop locks out everything.
- ! Hand on the e-stop; drive slowly near the structure. After driving, plan again (the monitor
  plans at the live base). ! Never drive Alice while she holds B21.

Where a base should stand: on `Load Movement` the monitor logs
`[LoadMovement] cfab planning robot is correctly placed at [x y z], but: live husky [...] (err E m)`
when the live base is more than 1 mm from where the export authored the movement: `E` is how far
the real robot stands from that pose. Drive until the IK check (Cindy) or the free approach's
goal IK (Alice) succeeds; `E` does not have to reach 0.

**Keyboard (recommended):**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -r cmd_vel:=/a200_0806/cmd_vel -p speed:=0.1 -p turn:=0.3
```

**Hold** `i` forward, `,` backward, `j` / `l` turn left / right on the spot, `u` / `o` forward while
turning; `k` (or any other key) stops; releasing the key stops the base within 0.5 s. `q` / `z`
raise / lower both speeds by 10 %, `w` / `x` only the linear, `e` / `c` only the turning speed.
`Ctrl-C` quits. ! Every key typed in that pane is a drive command — quit it (`Ctrl-C`) before using
the pane for anything else.

**Fixed moves** (`-r 10` = 10 messages/s, `-t N` = N messages = N/10 s; `Ctrl-C` stops early):

```bash
# forward 0.1 m/s for 2 s (about 20 cm); x: -0.1 to back up
ros2 topic pub -r 10 -t 20 /a200_0806/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
# turn on the spot 0.2 rad/s for 3 s (about 34 deg, counter-clockwise seen from above); z: -0.2 = clockwise
ros2 topic pub -r 10 -t 30 /a200_0806/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.2}}"
```

**What reaches the base:** `ros2 topic echo /a200_0806/platform/cmd_vel_unstamped`;
`ros2 topic echo /a200_0806/platform/emergency_stop` shows `data: true` while the e-stop is engaged.

---

## 4. Safety

- The remote e-stop **of the robot that moves** in hand for every arm motion (Cindy and Alice each
  have their own); the Husky e-stop does **not** stop the UR5e arms.
- Every new motion: preview with `Traj viz time` first, and raise the `traj time` slider for the
  first run (loading a movement sets it to the movement's default: Cindy's transfer 10 s, insert /
  retreat 5 s, free move home 10 s; Alice's free approach 15 s, straight approach 5 s).
- `Cancel Exec` ends a waiting step.
- If a log line says an arm may still be under the compliance controller: press
  **`Switch to Joint (BOTH)`** (further down the panel, under `CONTROLLERS`; with Alice it switches
  her one arm), or from the checks pane:

```bash
# Cindy (domain 86)
for a in left right; do
  ros2 service call /a200_0806/${a}_ur5e/controller_manager/switch_controller \
      controller_manager_msgs/srv/SwitchController \
      "{deactivate_controllers: [cartesian_compliance_controller], activate_controllers: [scaled_joint_trajectory_controller], strictness: 1}"
done
# Alice (domain 84)
ros2 service call /a200_0804/ur5e/controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{deactivate_controllers: [cartesian_compliance_controller], activate_controllers: [scaled_joint_trajectory_controller], strictness: 1}"
```

---

## 5. What every monitor start prints

```bash
ros2 run husky_assembly_teleop husky_monitor
```

- [ ] `Base calibration of Cindy (0806): 20260916/calibrated_transformation_0806_rhino.json` and
      `Base calibration of Alice (0804): 20260623/calibrated_transformation_0804_rhino.json`
      (all three huskies are created; `Base calibration file not found for robot 0805` = Belle,
      expected: she is not calibrated and stays parked).
- [ ] `[Schedule] robot <Cindy (domain 86) | Alice (domain 84)> | problem 261006_3bar_holding_test | progress k/7 done`,
      `[Schedule] others: ...`, the 7-entry roster, and `mocap client connected: True`
      (`False`: `CLIENT_IP`, see 0.1).
- [ ] Panel: a **Schedule** section; `Ignore built-bar collisions (bars drawn faint)` unticked; the
      slider above `Load Movement` is called `Step`. Clicking a row of the `Schedule entries` list
      selects that entry (the `Schedule entry (idx)` slider jumps there; nothing is loaded until
      `Load entry`). The other robot's entries are grey, `(other robot)`.
- [ ] PyBullet: the connected robot drawn at its real base (mocap) with its real arm poses
      (`Toggle Joint Live Stream` follows the joints); the other robot drawn where mocap sees it
      (or where `progress.json` left it).

---

## 6. Alice — gripper and handoff checks in the air (before entry 0; no bar, clear workspace)

Alice's grip handoff has not run on hardware yet. Check it once before the assembly, so a problem
shows up before Cindy holds B21. **Do not `Mark entry done` here** — these are tests.

Start Alice's monitor (1.1 Alice block, then `ros2 run husky_assembly_teleop husky_monitor`):

- [ ] Section 5's lines with `robot Alice (domain 84) ... progress 0/7 done` and
      `others: Cindy <- parked | Belle <- parked` (nothing done yet).
- [ ] Panel: a **Gripper** section (`Open Gripper Full`, `Close Gripper for Bar`) and
      `Switch to Compliance (BOTH)` / `Switch to Joint (BOTH)` / `Zero Force Sensor (BOTH)`.
- [ ] **Gripper by hand:** `Open Gripper Full`, then `Close Gripper for Bar` → the jaws open and
      close. To watch the fingers, checks pane on domain 84:
      `ros2 topic echo /a200_0804/gripper/joint_states --field position` (≈ 0.43 open, ~0.79
      after an empty close; `Ctrl-C` afterwards). The gripper action itself sends no feedback,
      only its final result (checked 2026-10-12); the handoff below does not need any.
      (Alice's Robotiq driver may still close at 15 % speed: a close of ~4 s is expected until it
      gets Cindy's 2026-09-09 driver fix.)
- [ ] **Controller switch by hand** (hand on Alice's e-stop, gripper empty, arm in a safe pose):
      `Switch to Compliance (BOTH)` → the arm stays where it is; the section 2 Alice check shows
      `cartesian_compliance_controller` active. `Switch to Joint (BOTH)` → back to
      `scaled_joint_trajectory_controller`.
- [ ] **The handoff in the air** (the jaws close on nothing): `Open Gripper Full`. Select entry 3 →
      `Load entry` → `Step` → 3 (readout `step 4/4: B21_H_M3_gripper_close ...`) → `Load Movement` →
      `Exec gripper step: CLOSE + compliant handoff` → log
      `[gripper step B21_H_M3_gripper_close] CLOSE the gripper on the bar (position 0.8), the arm going compliant BEFORE the close [SupportGripper]. Click 'Confirm Exec' ...`
      → `Confirm Exec`. Expect, in order:
  - `[grip arm 0] phase 1/3: ensuring scaled_joint_trajectory_controller on arm 0.`
  - `F/T sensor zeroed (fingers open, touching nothing); going compliant in 0.5s.`
  - `anchor check: FK tool0 vs the arm-reported TCP 0.0 mm, 0.00 deg.` (above 10 mm / 2 deg:
    no compliance, a stiff close instead)
  - `phase 2/3: switching arm 0 scaled_joint_trajectory_controller -> cartesian_compliance_controller BEFORE the close (fingers still open).`
    The arm must stay where it is (hand on the e-stop).
  - `phase 3/3: close goal sent (position 0.426 -> 0.800, effort 0.1); arm under cartesian_compliance_controller (target = tool0 pose captured before the switch, zero wrench) until the gripper stalls or reaches its goal; ...`
    The arm still stays put while the empty jaws close.
  - `gripper done: position ..., stalled ..., reached_goal ...`, then
    `restoring scaled_joint_trajectory_controller on arm 0.`
- [ ] Afterwards the section 2 Alice check shows the **joint controller active**.
- [ ] If instead: `FK tool0 and the arm-reported TCP disagree by ...` (no handoff) =
      Alice's TCP frame does not match the monitor's FK: note the numbers;
      `switch to cartesian_compliance_controller FAILED: the gripper was NOT closed` = the
      controller switch was refused: check section 2's Alice controllers, then run the step again.
- [ ] Repeat once and press `Cancel Exec` during phase 3 → the hold ends **and the controller
      switches back to joint** (the gripper keeps closing with the arm stiff — logged).
- [ ] `Open Gripper Full` → quit Alice's monitor (`Ctrl-C`) **without** marking anything.

---

## 7. Start Cindy's monitor (monitor pane, after 1.1 Cindy block)

```bash
ros2 run husky_assembly_teleop husky_monitor
```

- [ ] Section 5's lines with `[Schedule] robot Cindy (domain 86) | problem 261006_3bar_holding_test | progress 0/7 done`
      and `[Schedule] others: Alice <- parked | Belle <- parked`.
- [ ] Tick `Workaround: plan the transfer as a free move (no bar, from the live arms)` (under
      `Plan Movement`) → the five `Transfer start:` sliders above it disappear (they come back
      when it is unticked).

## 8. Entry 0 — B20 jointing (ground bar)

Select entry 0 → `Load entry`. Six steps: travel to load (skipped), manual mount, tool grasp,
transfer, insert, fix foundation (manual). B20 stands on its two ground joints
(`joint_G20-T20-0_ground`, `joint_G20-T20-1_ground`) and has no tighten step.

- [ ] **Transfer** (`B20_J_M3_CDFM_transfer_to_approach`): `Load entry` opens on it, already loaded
      (readout `step 4/6: B20_J_M3_CDFM_transfer_to_approach ...`, `traj time` 10 s). To come back
      to it later: `Step` → 3 → `Load Movement`.
  - [ ] **IK check** (workaround box ticked): `Transfer start: Confirm manual pose (IK check)` →
        `[transfer workaround] insertion start reachable from the live base (blue ghost); max |goal - stored goal| = ... rad`.
        The blue ghost shows the arms at the insertion start. Nothing moves and nothing is staged
        for `Exec` yet.
  - Optional, any time a movement is loaded: `Draw tool0: live (mocap + FK) vs target (Rhino)`
    draws each arm's tool0 twice in PyBullet: `<side> tool0 target (Rhino)` (long axes, the loaded
    movement's exported target) and `<side> tool0 live (mocap + FK)` (short axes, the live arm
    joints on the mocap base), joined by a black line, and logs the gap:
    `[tool0 check] B20_J_M3_CDFM_transfer_to_approach: tool0 live (mocap base + FK) minus target (Rhino): left ... mm (dx/dy/dz ...), axes x/y/z ... deg | right ...`.
    Before the transfer runs the gap is large (the arms are not there yet); click it again after
    the transfer to see what is left. A movement without target poses only warns.
  - Optional, for the insert / retreat: tick `Visual diagnosis (see-through robot, insert/retreat
    tool0 frames + path)` (under the button above). The grey live robot and its tools turn
    see-through, and every insert / retreat plan (`Plan Movement`, and the retreat's replan during
    exec) draws, per arm: `<side> Rhino start` / `<side> Rhino target` (long axes, the exported
    tool0 frames; the start is the previous movement's target), `<side> plan start` /
    `<side> plan end` (short axes, the plan's first and last waypoint; for the retreat the start is
    the live tool0), a black line from each Rhino frame to its plan frame, and the waypoint path
    IK follows (green line with small axes; red from the failing waypoint on when the plan failed).
    It logs `[B20_R_M1_LM_retreat viz] left: N waypoints over 50.0 mm; plan start is d mm / a deg from Rhino start; plan end is ... from Rhino target.`
    Ticking it after a plan draws that last plan; unticking removes the drawing and brings the
    robot back. For the insert only the left Rhino target is used (the right tool follows the
    bar), so a right `plan end` gap means the two exported targets disagree.
  - `... no collision-free IK for the insertion start at the live base ...`: the base stands too
    far from (or too close to) B20's Rhino pose, or the drawn colliding pair is in the way. Drive
    the base (section 3) and click the IK check again.
  - [ ] `Plan Movement` →
        `[transfer workaround] B20_J_M3_CDFM_transfer_to_approach: goal from the last IK check (the base moved ... mm since).`,
        `[transfer workaround] free move (no bar) from the live arms to the insertion start; max |goal - stored goal| = ... rad`
        (same value as the IK check) and `[Plan] 'B20_J_M3_CDFM_transfer_to_approach': N waypoints stored.`
        If the base moved more than 2 mm since the check, the goal is solved again
        (`... solving the goal again.`).
  - [ ] Preview (`Traj viz time`): starts at the live arms, ends with the **empty** tools at the
        insertion start, just short of B20's assembled pose, clear of the floor.
  - [ ] Raise `traj time` (first run), `Exec Selected Mv Traj (auto)` → the arms go there. The
        exec then stays running (the next buttons are refused until it ends) and corrects the
        tool0 placement with the visual servo of the bar-holding accuracy test (the base's centre
        of mass shifts under the arms, so the tools land a few mm off the Rhino pose):
  - [ ] **Mount pause** (workaround box ticked): `[transfer B20_J_M3_CDFM_transfer_to_approach] the arms stand at the insertion start with EMPTY tools: mount the bar and clamp it with the tool buttons, then click 'Confirm Exec'. ...`
        → mount B20 by hand in both tools and clamp it with the **buttons on the tools** →
        `Confirm Exec`. This does the **Manual mount** and **Tool grasp** steps below: skip them,
        no click. `Cancel Exec` here ends it without a correction (`mount pause cancelled; no servo correction.`).
  - [ ] The insert is loaded by itself (`[Movement] loaded [..] 'B20_J_M4_LM_insert' ...`) and the
        servo runs: `### SERVOING iteration 1/8`, `[IK Live Base] ...`,
        `[transfer plan] OK (N waypoints, bar-held, LINEAR).` and the pause
        `[servo] First move planned: N waypoints, travels X deg ...` → scrub `Traj viz time`
        (a few mm with the bar held) → `Confirm Exec`. Later iterations run alone unless a path
        travels more than 5 deg (`[servo] SAFEGUARD iter N: ...` → preview, `Confirm Exec`).
        Each pass logs `[servo N] tool0 pos L=... R=... mm`; the `Servoing Live Tracker` window
        opens by itself when the servo starts and plots them per iteration (tool0 position and
        rotation error per arm, base drift), as in the bar-holding accuracy test;
        `Toggle Servoing Tracker` hides / shows it.
  - [ ] It ends with `### SERVOING converged at iter N (< 0.2 mm)` or
        `### SERVOING stopped at iter N: nothing left to plan ...` (or after 8 iterations), the
        record `... B20_J_M4_LM_insert_servoing_data_<ts>.json` / `_performance_<ts>.png` under
        `bar_action_live_replan_exe/<date>-servoing/` on the drive, and
        `[Servo] propagated arm configuration -> ...` for the steps around the transfer and the
        insert (the corrected arms are now the transfer's end and the insert's start).
        `servo skipped (FAKE_HARDWARE ...)` / `(the base is not tracked by mocap)` = no correction.
- [ ] **Manual mount** (`B20_J_M1_manual_mount_bar`), only with the workaround box **unticked**
      (otherwise done in the mount pause): `Step` → 1 → `Load Movement` (the bar is drawn
      in the tools at the insertion start) → `Operator done (manual step) -> then Confirm Exec` →
      mount B20 by hand in both tools → `Confirm Exec` → `confirmed done by the operator`.
- [ ] **Tool grasp** (`B20_J_M2_tool_grasp_bar`), likewise: `Step` → 2 → `Load Movement` → `Tool step: grasp` →
      `Confirm Exec` → `STOP`, gripper motors tighten, `every gripper motor STALLED after N s: bar clamped`,
      `STOP sent`.
- [ ] **Insert, grounded bar** (`B20_J_M4_LM_insert`): already loaded after the servo (else
      `Step` → 4 → `Load Movement`); readout `step 5/6: B20_J_M4_LM_insert  (dual_constrained_linear)` →
      `Plan Movement` (it starts where the servo left the arms) → preview; no ground-joint ↔ floor
      collision is reported.
      B20 is held on its ground joints, so its insert runs **rigid only** (joint tracking, compliance
      never engaged) and the joint motors **never tighten**; the `Insert: ...` knobs do not apply.
      Raise `traj time` (first run; the rigid insert has no force limit) → `Exec Selected Mv Traj (auto)` →
      `[B20_J_M4_LM_insert] grounded bar (holds joint_G20-T20-0_ground, joint_G20-T20-1_ground): rigid insert under scaled_joint_trajectory_controller (...), no tighten; ...`
      and, when the motion is done, `[B20_J_M4_LM_insert] grounded bar: rigid insert done (no tighten).`
      Also once: `'B20_J_M4_LM_insert': the export asks for the cartesian_compliant controller; the monitor runs this dual_constrained_linear movement under joint_tracking.`
      (expected; Rhino note D9).
  - [ ] The joint motors do not turn; the controllers stay on `scaled_joint_trajectory_controller`.
- [ ] **Fix foundation** (`B20_J_M5_manual_fix_foundation`): `Step` → 5 → `Load Movement` →
      `Operator done (manual step) -> then Confirm Exec` → fix B20 to the foundation by hand →
      `Confirm Exec`.
- [ ] `Mark entry done` → `[Schedule] entry 0 B20_J_joint marked done by Cindy; Cindy's belief <- live. Saved ...`,
      header `1/7`.

## 9. Entry 1 — B20 release

`Load entry` (slider on 1). Two steps; it opens on the retreat.

- [ ] Readout `step 1/2: B20_R_M1_LM_retreat  (+ B20_R_M0_tool_ungrasp_bar runs with it)`.
- [ ] B20 and its joints drawn solid (built, collision-checked).
- [ ] **Retreat** (`B20_R_M1_LM_retreat`): `Plan Movement` plans it from where the arms **are** (the
      insert leaves them a little off its planned end):
      `[B20_R_M1_LM_retreat] start_state resynced to the live arms (max joint drift ... deg)` and
      `[retreat live] B20_R_M1_LM_retreat: goals = the live tool0 poses moved 50 mm back: left along (...), N mm from the exported goal; right ...`
      (each tool0 backs off 50 mm along its exported retreat axis, orientation kept; scene
      collisions are checked). No path → the `[B20_R_M1_LM_retreat diag]` block (section 16).
      Then preview (starts at the live arms) → `Exec Selected Mv Traj (auto)` →
      log: the ungrasp runs with it (gripper motors LOOSENING), and once
      `'B20_R_M1_LM_retreat': the export asks for the joint_tracking controller; the monitor runs this dual_independent_linear movement under ...` (expected).
      Before the compliant part both force sensors are zeroed:
      `force sensors zeroed before the compliant part (|F| a/b -> c/d N)` (the left / right reading
      before and after; a leftover force would otherwise push the tools into the joint).
      After the compliant part the monitor prints its checks of the rigid part,
      `[B20_R_M1_LM_retreat] check PASS / FAIL ...` (replan, start, compliant part, contact, grippers,
      left / right tool0 path, joints, speed), then runs it at once, **no `Confirm Exec`**:
      `running the rigid part now (N waypoints over T s); every check passed.` (or
      `N check(s) FAILED, see the "check FAIL" lines above`: note them in the log, section 16).
      Joint controller restored afterwards.
- [ ] **Free move home** (`B20_R_M2_free_home`): `Step` → 1 → `Load Movement` → `Plan Movement` →
      preview → `Exec Selected Mv Traj (auto)`.
- [ ] `Mark entry done` → header `2/7`.

## 10. Entry 2 — B21 jointing (one-sided: the left joint mates)

`Load entry` (slider on 2). Five steps: travel to load (skipped), manual mount, tool grasp,
transfer, insert (the tighten runs with the insert). Same workflow as entry 0, with these step
numbers:

- [ ] Built B20 and its joints drawn solid; the transfer's preview clears them.
- [ ] **Transfer** (`B21_J_M3_CDFM_transfer_to_approach`): already loaded by `Load entry` (readout
      `step 4/5: ...`) → `Transfer start: Confirm manual pose (IK check)` → `Plan Movement` →
      preview → `Exec Selected Mv Traj (auto)` → mount pause: mount B21, clamp it with the tool
      buttons → `Confirm Exec` → the servo correction as in entry 0 (first move: preview,
      `Confirm Exec`) → `### SERVOING converged ...` / `stopped ...`. Manual mount and tool grasp
      are done in that pause (box unticked: `Step` → 1 → `Confirm Exec`; `Step` → 2 → `Confirm Exec`).
- [ ] **Insert** (`B21_J_M5_LM_insert`): already loaded after the servo (else `Step` → 4); readout
      `step 5/5: B21_J_M5_LM_insert  (+ B21_J_M4_tool_tighten_joint runs with it)` → `Plan Movement` →
      `Exec Selected Mv Traj (auto)` → log
      `B21_J_M4_tool_tighten_joint ('tighten') runs with this movement: the compliant insert sends it (LEFT joint motor TIGHTENING only; the right joint has no female to screw into, its joint motor is not driven).`
      B21 is **one-sided**: its left male (`joint_J20-21_male`) screws into B20; the right tool holds
      `joint_J157-21_male`, whose female sits on B157, outside this export. The export already names only `AT3L`, so no
      `tightens only ['AT3L']` warning. Log
      `[B21_J_M5_LM_insert] one-sided bar: the left joint mates, so only the L joint motor TIGHTENs ...`
      and `[scaffolding] B21_J_M5_LM_insert: TIGHTEN joint motor on L arm (arm 0)` (no R line).
      Rigid approach, compliance for the last mm on **both** arms, the **left** joint motor
      tightens until it stalls (the right one is never driven), then the joint controller is
      restored.
- [ ] After the insert: Cindy holds B21 at its assembled pose, **joint controller active**
      (section 2's controller check).
- [ ] `Mark entry done` → header `3/7`; this stores Cindy's live base and arms in `progress.json`
      (Alice's run plans around them).
- [ ] **Quit Cindy's monitor** (`Ctrl-C`). Cindy keeps holding B21 (her driver keeps running). Do
      not touch her launch window.

## 11. Entry 3 — Alice grabs B21 (Alice's monitor)

Monitor pane: 1.1 **Alice** block, then `ros2 run husky_assembly_teleop husky_monitor`.

- [ ] Section 5's lines with `robot Alice (domain 84) ... progress 3/7 done` and
      `others: Cindy <- live (entry 2) | Belle <- parked`; the slider on entry 3, its row not grey.
- [ ] PyBullet: Cindy drawn holding B21 at her real base (mocap; her arms from `progress.json`).
      She may show twice at the same pose: the solid husky and the red obstacle robot — the red
      one is what the planner checks.
- [ ] Select 3 → `Load entry` → `[obstacle robots] posed from: ObstacleRobotBelle <- exported, ObstacleRobotCindy <- live`
      (from `progress.json`; a fresh mocap base still overrides it); B21 drawn in Cindy's tools.
      B22 may be drawn too although it is not built yet: the hold is exported with the bars that
      stand when it ends (Rhino note D5); the planner just avoids it. Readout
      `step 1/4: B21_H_M0_free_to_approach ...`, `traj time` 15 s. The `Transfer start:` and
      `Insert: ...` controls are hidden.
- [ ] **Free approach** (`B21_H_M0_free_to_approach`): `Plan Movement` → the goal is re-solved by IK
      at Alice's live base, with her own exact ssik solver (built from her calibrated URDF, like
      Cindy's): once per run `[goal IK] ssik solver 'alice' fits the planner's robot.`, then
      `[goal IK] ssik: solution pair 1/8 OK (...)`,
      `[live goal] B21_H_M0_free_to_approach: goal re-solved at the live base; max |live IK - stored target| = ... rad.`,
      then `[Plan] 'B21_H_M0_free_to_approach': N waypoints stored.` (≈ 65, random planner, varies).
      (`... does NOT fit the planner's robot ...` = the planning cell is not Alice's calibrated
      model: PyBullet IK is used instead, accurate to 1 mm only; note it.)
  - `... no collision-free IK for its approach pose at the live base (...); not planning.` (the
    collision drawn if there is one): Alice stands where the approach pose cannot be reached —
    drive her base (section 3, domain 84, `/a200_0804`) and plan again.
  - `[live goal] ...: base not tracked; ...` = mocap does not see Alice (Motive 1840, 0.3).
  - [ ] Preview (`Traj viz time`): ends with the open jaws 100 mm in front of B21, clear of
        Cindy's arms and tools. Raise `traj time` → `Exec Selected Mv Traj (auto)` → the arm goes
        there, then the exec stays running and **servos tool0 onto the approach pose**, the same
        way as Cindy's transfer (the arm's motion shifts the base under it):
  - [ ] `[servo 0] tool0 pos X mm, rot Y deg` (before any correction), `### SERVOING iteration 1/8`,
        then the pause `[servo] First move planned: tool0 X mm, joints Y deg, over T s. ...` →
        scrub `Traj viz time` (a mm-size move) → `Confirm Exec`. Later moves run alone unless
        they travel more than 5 deg (`[servo] SAFEGUARD ...` → `Confirm Exec`). Each move is a
        joint move from the live arm to the exact IK solution at the live base, nearest the
        live arm. The `Servoing Live Tracker` window opens by itself (one arm, plus the base drift).
  - [ ] It ends with `### SERVOING converged at iter N (< 0.2 mm)` or `### SERVOING stopped at
        iter N: nothing left to plan ...` (or after 8 moves), the record
        `B21_H_M0_free_to_approach_servoing_data_<ts>.json` / `_performance_<ts>.png` under
        `bar_action_live_replan_exe/<date>-servoing/`, and `[Servo] propagated arm configuration
        -> ...` for the gripper step and the straight approach (an approach planned before is
        dropped: plan it again). `[approach ...] servo skipped (FAKE_HARDWARE / the base is not
        tracked by mocap)` = no correction.
- [ ] **Calibration check** (Alice's June calibration, 0.1): the jaws now stand ≈ 100 mm in front of
      B21, centred on the bar. Off by more than ~1–2 cm sideways: stop, the base calibration is
      wrong (recalibrate Alice, `doc/calibration_manual.md`, and point her
      `BASE_CALIBRATION_DATE_BY_ROBOT` entry at the new folder).
- [ ] **Gripper open** (`B21_H_M1_gripper_open`): `Step` → 1 → `Load Movement` →
      `Exec gripper step: OPEN` → `[gripper step B21_H_M1_gripper_open] OPEN the gripper fully (position 0.426) [SupportGripper]. Click 'Confirm Exec' ...`
      → `Confirm Exec` → `gripper 0 done: position 0.426, ...`.
- [ ] **Straight approach** (`B21_H_M2_LM_to_grasp`): `Step` → 2 → `Load Movement` (`traj time` 5 s)
      → `Plan Movement` → `[B21_H_M2_LM_to_grasp] start_state resynced to the live arms (max joint
      drift from the propagated conf: ... deg)` (it plans from where the arm really is, so the
      path stays on the arm's IK branch; a drift near 360 deg = the chained start was a full turn
      off, now ignored) and `[Plan] 'B21_H_M2_LM_to_grasp': ≈ 21 waypoints stored.` (100 mm
      straight) → preview: the open jaws slide onto B21 → `Exec Selected Mv Traj (auto)`.
- [ ] **Gripper close + handoff** (`B21_H_M3_gripper_close`): `Step` → 3 → `Load Movement` →
      `Exec gripper step: CLOSE + compliant handoff` → `Confirm Exec` → the phase lines of
      section 6. The arm is compliant for the whole close: it gives way while the jaws centre
      on B21, then
      `gripper done: position ..., stalled True, ...` and the joint controller is back.
      Watch Cindy's tools: they must not be pulled.
- [ ] `Mark entry done` → `[Schedule] entry 3 B21_H_hold marked done by Alice; Alice's belief <- live. Saved ...`,
      header `4/7`. (`Mocap has not seen Alice in the last 1 s; ...` = her base was not tracked: the
      authored base is stored instead; note it.)
- [ ] **Quit Alice's monitor** (`Ctrl-C`). Alice stays clamped on B21 (her driver keeps running).

## 12. Entry 4 — B21 release (Cindy's monitor again)

Monitor pane: 1.1 **Cindy** block, then `ros2 run husky_assembly_teleop husky_monitor`.

- [ ] Start-up: `[Schedule] Cindy starts from the progress.json state (after entry 2, live).`
      (her real joints take over at once), header `4/7`,
      `others: Alice <- live (entry 3) | Belle <- parked`.
- [ ] PyBullet: Alice drawn on B21 at her hold pose (mocap base).
- [ ] **Tick the transfer workaround box again** (needed for entry 5; it starts unticked).
- [ ] `Load entry` (slider on 4) → `[obstacle robots] posed from: ObstacleRobotAlice <- live, ObstacleRobotBelle <- exported`;
      it opens on the retreat (`step 1/2: B21_R_M1_LM_retreat  (+ B21_R_M0_tool_ungrasp_bar runs with it)`).
- [ ] **B21 is clamped by Alice** (the ungrasp releases Cindy's tools from it).
- [ ] **Retreat** and **free move home** as in entry 1 (`B21_R_M1_LM_retreat`, then
      `B21_R_M2_free_home`), both planned with Alice in the scene. Check in the retreat's preview
      that both tools back off clear of Alice's gripper.
- [ ] `Mark entry done` → header `5/7`.

## 13. Entry 5 — B22 jointing (both joints mate)

`Load entry` (slider on 5). Five steps, as entry 2. B22's left male (`joint_J21-22_male`) screws
into B21, which Alice still holds; its right male (`joint_J20-22_male`) into B20.

- [ ] Built B20, B21 and Alice (red obstacle robot) drawn; the transfer's preview clears them all.
- [ ] **Transfer** (`B22_J_M3_CDFM_transfer_to_approach`): as entry 2 (IK check → `Plan Movement` →
      preview → Exec → mount pause: mount B22, clamp → `Confirm Exec` → servo).
      A warning
      `Plan end of 'B22_J_M3_CDFM_transfer_to_approach' differs from existing 'B22_J_M5_LM_insert'.start by max 6.2832 rad/m; overwriting (carry rule).`
      means one joint of the insertion start is a full turn (2π) from the exported one: same tool
      poses, harmless (seen headless).
- [ ] **Insert** (`B22_J_M5_LM_insert`): readout
      `step 5/5: B22_J_M5_LM_insert  (+ B22_J_M4_tool_tighten_joint runs with it)` → `Plan Movement` →
      `Exec Selected Mv Traj (auto)` → log
      `B22_J_M4_tool_tighten_joint ('tighten') runs with this movement: the compliant insert sends it (joint motors TIGHTENING).`
      and a `[scaffolding] B22_J_M5_LM_insert: TIGHTEN joint motor on ...` line for **both** arms.
      Rigid approach, compliance for the last mm, both joint motors tighten; the insert ends when
      **both** have stalled, then the joint controller is restored.
  - [ ] Watch B21 while B22 screws in: Alice holds it; it must not be pushed out of her jaws.
- [ ] `Mark entry done` → header `6/7`.

## 14. Entry 6 — B22 release

- [ ] `Load entry` (slider on 6) → opens on `step 1/2: B22_R_M1_LM_retreat  (+ B22_R_M0_tool_ungrasp_bar runs with it)`.
- [ ] **Retreat** and **free move home** as in entry 1 (`B22_R_M1_LM_retreat`, `B22_R_M2_free_home`),
      Alice in the scene.
- [ ] `Mark entry done` → header `7/7`.

**After the run:** Alice still clamps B21: her release (`B21_HR`) comes after B37 and is not in
this export. To free her by hand: quit Cindy's monitor, start Alice's (1.1 Alice block) →
`Open Gripper Full` → select 3 → `Load entry` → `Step` → 2 → `Load Movement` (the straight
approach) → `Move Arms to Movement Start (offline target)`: the arm moves back to the pre-grasp
pose, 100 mm out (joint move; it refuses joint changes above pi/3). Watch the jaws clear B21.
Do not mark anything.

---

## 15. Starting over / redoing an entry

- **Redo one entry:** load it → `Reopen entry (reload clean)` (pending again, clean export loaded,
  the robot's previous end state restored). Reopen Alice's entry 3 from Alice's monitor.
- **Start the whole test over:** `Reset schedule to the Rhino export` → `Confirm Exec` →
  `[Schedule] reset to the Rhino export: moved N file(s) (...) to .../archive/<time>.`; every entry
  pending, entry 0 loaded clean. Nothing is deleted: the old `progress.json` and saved plans are in
  `~/Insync/2025-03 Husky Assembly/data_design_study/261006_3bar_holding_test/archive/<time>/`.
  A monitor of the other robot that is running keeps its old progress: restart it.

## 16. If something goes wrong

| situation | do |
|---|---|
| a step waits and you want out | `Cancel Exec` |
| "refused: a step / execution is still running" | finish it or `Cancel Exec`, then retry |
| arm may be left compliant (loud error in the log) | `Switch to Joint (BOTH)` or the service call in section 4 |
| `Exec` refused: arms not at the trajectory start | `Move Arms to Movement Start (offline target)` (refuses joint changes above pi/3), then `Exec` |
| the transfer plans with the bar held / tries the bar-held planner | the workaround box is unticked (it starts unticked after every restart): tick it, plan again |
| `[transfer workaround] ... no collision-free IK for the insertion start` (IK check or plan) | the base is too far from / too close to the Rhino pose, or the drawn colliding pair is in the way (entry 5: Alice?): move the base (section 3), IK check again; note it in the log |
| the IK check passes although the base is far from the Rhino pose; it prints `[M1 manual] ...` lines | the workaround box is unticked (the `Transfer start:` sliders are showing): the button then checks the bar-loading start next to the robot, not the insertion start. Tick it and check again |
| a plan fails with "start ... in collision" | read the pair in the log. Alice's run, `ObstacleRobotCindy` or Alice against a built bar: tick `Ignore built-bar collisions (bars drawn faint)` (it reloads the entry; the built bars are then ignored, Rhino note D5), plan again. Cindy's run with `ObstacleRobotAlice` in the pair: note it (Alice's stored hold pose overlaps the scene); check she is drawn where she really stands |
| `not planning: joint value(s) outside the URDF limits` (right away, no search) | the start or goal has a joint the arm cannot reach (the UR5e elbow turns only +/- pi); the message names it. Should not happen for the transfer any more (its goal keeps every joint in range): note the line in the log |
| `Plan for '..._LM_retreat'` / `'..._LM_insert' ... FAILED` (a straight-line plan) | read the `[<movement> diag]` block after it: the START's colliding pairs with the movement's ACM entry of each body (a contact is allowed only when listed there), its joint limits, the failing waypoint and why its IK candidates were turned down (collide / would flip an arm), and a `VERDICT`. The red planning robot and the contacts are drawn, plus that waypoint's two tool0 targets (`Remove all drawing` clears them). A START collision the ACM does not allow = an export gap or the arms are not where the export expects: note the pair in the log. The retreat's replan during exec prints the same block, then runs the planned tail instead (its first step jumps; no pause) |
| the retreat's rigid part ran with `N check(s) FAILED` | the checks only report (no pause); note the `check FAIL` lines: `grippers` not `STALLED` = a gripper did not open; `contact` = a tool still pushed on the structure; `compliant part` may fail from gravity sag alone (the limits are first guesses); `replan` = the fallback tail ran, whose first step jumps. Remote e-stop if the arms pull on the bar |
| `plan_free_dual_arm failed: birrt_failed` (both ends collision-free, no path found) | read the `[... diag]` lines after it: joint limits of start and goal, how many nearby configurations collide at each end (and with what), where the straight joint path first collides (drawn; the red robot stands there), and a `VERDICT` line. A tight pocket at the GOAL: drive the base to give the arms room, IK check, plan again. Otherwise plan again |
| `Plan end of '..._transfer_to_approach' differs from existing '..._LM_insert'.start by max 6.2832 rad/m` | one joint a full turn from the export's: same tool poses, harmless |
| `Cancel Exec` at the transfer's mount pause | no correction; the insert starts where the transfer ended (re-running the transfer is refused: the arms stand at its end, not its start). Mount and clamp the bar, then plan and run the insert as usual |
| `servo skipped (FAKE_HARDWARE)` / `(the base is not tracked by mocap)` after the transfer | `FAKE_HARDWARE = 1`, or `USE_MOCAP = 0` / `USE_CELL_STATE_BASE_POSE = 1` (section 0.1): the servo needs the real arms and the mocap base |
| `Servoing: live-base IK failed; stopping.` / `[transfer plan] the straight-line bar move could not be solved ...` | the corrected pose is not reachable (or collides) from where the base stands; with the sampling fallback check the `SAFEGUARD` figures before confirming. The insert then starts where the servo left the arms |
| Alice: `no collision-free IK for its approach pose at the live base` | drive Alice (section 3, domain 84) towards her hold spot, plan again |
| Alice: `FK tool0 and the arm-reported TCP disagree ...` | no compliant handoff: the gripper closed with the arm stiff. B21 is still clamped; note the numbers |
| Alice: `switch to cartesian_compliance_controller FAILED: the gripper was NOT closed` | nothing closed: check Alice's controllers (section 2; `Switch to Joint (BOTH)` if needed), then run the close step again |
| Alice's jaws not centred on B21 after the free approach | her base calibration is off (0.1): stop before the straight approach; recalibrate her |
| `Base calibration file not found for robot 0804` | Alice's file is not in the folder her `BASE_CALIBRATION_DATE_BY_ROBOT` entry names (0.1), or Insync has not synced it |
| no topics / `Unknown topic` | `eno1` on `192.168.0.x` and the robot pings (0.4), the right domain + CycloneDDS in this pane (1.1 / 1.2), the robot's launch still running with its domain + CycloneDDS (0.3) |
| no tool status (section 2) / grasp steps get no answer | `... ScaffoldingToolStatus' is invalid`: the checks pane did not source the workspace (1.2), the tools are fine. `NO MESSAGES`: Cindy's launch is missing `gripper:=scaffolding_v3` (0.3) |
| `mocap client connected: False` | `CLIENT_IP` is not this PC's `eno1` address (0.1), or the Motive PC is no longer `192.168.0.28` (`doc/calibration_manual.md` 1.4) |
| marked an entry done by mistake | load it, `Reopen entry (reload clean)` (Alice's entry from Alice's monitor) |
| progress is wrong beyond repair | `Reset schedule to the Rhino export` (section 15) |

## 17. What to send back

- The monitor log (`~/husky_real/monitor.log`, or copy the terminal); it holds all three monitor runs.
- `~/Insync/2025-03 Husky Assembly/data_design_study/261006_3bar_holding_test/progress.json` after the run.
- Per jointing entry: the `[transfer workaround] ... max |goal - stored goal|` line, the
  `[tool0 check] ... minus target (Rhino)` line from `Draw tool0: ...` after the transfer, whether the
  mounted bar sat in the tools at the insertion start, and how the insert finished (stall time,
  any rigid / compliant warnings).
- Per release entry: the `[..._LM_retreat] check ...` lines (which passed, the numbers of any FAIL),
  and whether the arms backed off cleanly (entries 4, 6: clear of Alice).
- Entry 0: how B20's rigid insert ended (the `grounded bar` lines; the bar's final fit before fixing
  the foundation).
- Alice (sections 6 and 11): the phase lines and any "no handoff"
  warning; how the arm behaved while compliant; the `[live goal]` line; how far the open jaws were
  from B21's centre after the free approach; whether B21 moved while she clamped it or while B22
  was screwed in.
