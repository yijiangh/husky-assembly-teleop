# Cindy + Alice + Belle B20–B37 test — with the real robots, run from the robot-control PC

> **Run from the robot-control PC** (`CRLRobotxPC`, user `su`). This file is the current manual
> for this export.
>
> | | this PC |
> |---|---|
> | workspace | `~/ros2_ws` (venv `~/ros2_ws/venv`, ssik 4.1 installed) |
> | shared drive | `~/Insync/2025-03 Husky Assembly` (the package's built-in data paths already point here) |
> | robot network | built-in ethernet port **`eno1`**, DHCP from the router `192.168.0.1` (was `192.168.0.25` on 2026-10-07) |
> | DDS | `~/.cyclonedds.xml` pins CycloneDDS to `eno1`; `~/.bashrc` exports `CYCLONEDDS_URI` but **not** `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` |
> | internet | Wi-Fi `wlp9s0` (+ Tailscale); ROS does not use them |

A standalone checklist for the design problem **`261006_3bar_holding_test`** (re-exported
2026-10-13 as the range **B20–B37**, 18 actions). Three robots:
- **Cindy** (dual-arm husky `/a200_0806`, ROS domain 86) joints and releases the seven bars.
- **Alice** (single-arm husky `/a200_0804`, domain 84, Robotiq 2F-85 gripper) grabs **B21** after
  Cindy has inserted it.
- **Belle** (single-arm husky `/a200_0805`, domain 85, Robotiq 2F-85 gripper) grabs **B35** after
  Cindy has inserted it.

Both support robots hold their bar until B37 is in, then let go (actions 16 and 17).

| # | action | who | operator steps |
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

**One robot per monitor run.** The monitor drives the robot whose `ROS_DOMAIN_ID` it starts with.
It draws the other two from their live mocap base, or from what `progress.json` remembers. So the
session runs seven monitor runs in turn:

| run | robot | actions | section |
|---|---|---|---|
| 1 | Cindy | 0–2 | 7 |
| 2 | Alice | 3 | 8 |
| 3 | Cindy | 4–9 | 9 |
| 4 | Belle | 10 | 10 |
| 5 | Cindy | 11–15 | 11 |
| 6 | Alice | 16 | 12 |
| 7 | Belle | 17 | 13 |

Quitting a monitor leaves the robot's driver running: the arms stay where they are and keep
holding. Before the assembly, Belle's and Alice's gripper and handoff are checked once in the air
(section 6).

**How the runs hand over.** After every step that runs, the robot's real state (measured arm
joints, mocap base) is saved to `progress.json` (log
`[Plan skeleton] <robot>'s live state after <step> saved for the next actions ...`), and the next
monitor run draws and plans around it, also without `Mark action done`. A hold is recorded once
the gripper close (the hold's last step) has run, a release once the release's retreat has run.
`Mark action done` still marks the action done (the `progress k/18 done` counts below assume you
click it) and saves its planned trajectories. Within an action, each step's real end becomes the
next step's start (`[Exec] propagated arm configuration -> ...`).

> **! Transfer workaround in use** (until the bar-held transfer planner is ready). The transfer
> runs as a plain free move **without the bar**:
> - Tick `Workaround: plan the transfer as a free move (no bar, from the live arms)` (under
>   `Plan Movement`) once per Cindy monitor run. It starts unticked, so tick it again after each
>   restart (runs 3 and 5).
> - With the box ticked, `Transfer start: Confirm manual pose (IK check)` checks whether the arms
>   reach the insertion start (the Rhino pose) from where the base stands.
> - The `Transfer start:` sliders (the bar-loading pose) are not used; they disappear from the
>   panel while the box is ticked.
>
> Order of a jointing action: **transfer** (empty tools, from the live arms to the insertion start)
> → **manual mount** (mount the bar by hand there) → **tool grasp** → **insert** (→ **fix
> foundation** for the ground bars B20 and B34). The travel to load is skipped.

Every code block below is meant to be pasted as is. Tick the boxes as you go; anything that does
not match "expect" is worth a note (log lines + what you did). What the buttons do in detail:
`doc/support_robot_plan_skeleton_manual.md`.

---

## 0. Once, before the session

### 0.1 Monitor flags, the mocap address and the base calibrations

The switches are class attributes at the top of `class HuskyMonitor`
(`husky_assembly_teleop/husky_monitor.py`; no rebuild needed after editing, restart the monitor).
The same values serve all three robots' runs.

```bash
F=~/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_monitor.py
grep -nE "^    (FAKE_HARDWARE|USE_MOCAP|USE_CELL_STATE_BASE_POSE|CONNECT_COMPLIANT_CONTROLLER|LIST_CONTROLLER_SERVICES|IGNORE_BUILT_ASSEMBLY_COLLISIONS|TRANSFER_AS_FREE_MOVE) *=" "$F"
grep -nE "^    (CALIBRATION|PUNCH_CALIB_VALIDATION|BAR_ACTION_LIVE_REPLAN_EXE|BAR_ACTION_MOCAP_ACCURACY_TEST) *=" "$F"
grep -nE "^(CLIENT_IP|MOCAP_IP) *=" "$F"
ip -4 -brief addr show eno1        # this PC's address on the robot network
```

| flag | value for this test |
|---|---|
| `FAKE_HARDWARE` | **0** |
| `USE_MOCAP` | **1** |
| `USE_CELL_STATE_BASE_POSE` | 0 |
| `CONNECT_COMPLIANT_CONTROLLER` | **1** (Cindy's insert / retreat and the support robots' grip handoff switch controllers) |
| `LIST_CONTROLLER_SERVICES` | **1** |
| `IGNORE_BUILT_ASSEMBLY_COLLISIONS` | 0 (built bars are obstacles; leave the panel toggle unticked, also for Alice and Belle: both holds plan with them, checked headless on this export) |
| `TRANSFER_AS_FREE_MOVE` | 0 (tick the workaround box in the panel instead) |
| `CALIBRATION`, `PUNCH_CALIB_VALIDATION` | **0** (1 only for a calibration session; Belle's on 2026-10-13/14 had both at 1) |
| `BAR_ACTION_LIVE_REPLAN_EXE` | **1** (0 hides the whole bar-assembly panel, the plan skeleton included) |
| `BAR_ACTION_MOCAP_ACCURACY_TEST` | 1 (the value before Belle's calibration; its extra buttons are not used in this test) |
| `CLIENT_IP` (module level) | **this PC's `eno1` address** (`192.168.0.25` on 2026-10-07) |
| `MOCAP_IP` (module level) | `192.168.0.28` (the Motive PC) |

! `CLIENT_IP` is where the monitor listens for the mocap stream, so it differs per PC. The
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

**Base calibrations** (mocap marker body → robot base) are set in
`husky_assembly_teleop/__init__.py`. Each robot reads its own date folder in
`BASE_CALIBRATION_DATE_BY_ROBOT`, else `CALIBRATION_DATE`.

```bash
I=~/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/__init__.py
grep -nE "^CALIBRATION_DATE|^BASE_CALIBRATION_DATE_BY_ROBOT|^    '(Alice|Belle|Cindy)'" "$I"
C="$HOME/Insync/2025-03 Husky Assembly/data_experiment/calibration_data"
ls "$C"/20260916/calibrated_transformation_0806_rhino.json "$C"/20260623/calibrated_transformation_0804_rhino.json \
   "$C"/20261013/calibrated_transformation_0805_rhino.json
```

| robot | calibration | where it is set |
|---|---|---|
| Cindy (0806) | `20260916` | her entry `'Cindy': '20260916'` |
| Alice (0804) | `20260623` | her entry `'Alice': '20260623'` (her newest calibration) |
| Belle (0805) | `20261013` (recorded 2026-10-14) | her entry `'Belle': '20261013'` |

All three robots have their own entry, so `CALIBRATION_DATE` (`20261013`, the newest calibration
session) decides no robot's base; it only matters for a calibration session's recordings.

- **Belle's calibration (2026-10-14)**, folder `20261013`: validation over 17 poses, the tool
  tracker's offset found again within 1.6 mm (std 0.7 mm) and 0.5°; punch test, 5 takes from
  different base poses, the punch tip within 1.5 mm of its mean (pairwise at most 3.0 mm).
  Her mocap body sits 180° turned compared with Alice's (tracker mounting), so her
  `base_mocap_from_base_footprint` turns by +90° where Alice's turns by −90°: expected.
- ! Alice's June calibration holds only if her base markers have not moved since. The first check
  of each support robot's calibration is where its open jaws end after the free approach
  (sections 8 and 10).
- Without a file the robot runs **uncalibrated**: its base is the marker body, cm off. As the
  connected robot it then plans from the wrong place. As another robot it is not used as a live
  obstacle (it falls back to `progress.json`).

### 0.2 The problem folder (shared Insync folder)

The monitor reads and writes the design problem directly in the shared Insync folder.
`Mark action done` writes `progress.json` and `*.live-solved.json` (saved plans) there, and they
sync to the shared drive. All three robots' runs read and write the same `progress.json`: that is
how each run knows where the other robots stand. Check that the folder is complete and see what an
earlier run left behind:

```bash
GD="$HOME/Insync/2025-03 Husky Assembly"
P="$GD/data_design_study/261006_3bar_holding_test"
ls "$P"                                        # ActionSchedule.json BarActions RobotCell.json RobotCell_Alice.json RobotCell_Belle.json WalkableGround.json ...
ls "$P"/BarActions/*__*.json | grep -vc live-solved     # 18 action files (B20..B37 __J/__R, B21 and B35 __H/__HR)
ls -a "$P" | grep -c insyncdl                  # 0 = Insync has finished syncing
ls "$P"/progress.json "$P"/BarActions/*.live-solved.json 2>/dev/null   # left over from an earlier run?
```

If `progress.json` or saved plans are left over and you want to start fresh, use
`Reset plan skeleton to the Rhino export` once Cindy's monitor is up (section 14): it moves them to
`$P/archive/<time>/` (nothing is deleted).

### 0.3 Robot bring-up (robot side)

Open a separate tmux window per robot (`Ctrl-b c`; `Ctrl-b n` cycles windows) and log in there.
This PC's ssh key is accepted on Cindy; on the single-arm huskies the password is `clearpath` if
asked (`doc/husky_ros2.md`).

**Cindy**

- [ ] Both arms powered on, teach pendants in **external control** mode (`doc/husky_ros2.md`; do
      not start `ros_control.urp` by hand).
- [ ] Arms + scaffolding tools launched on Cindy's PC:

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
      and in reach (pair one husky at a time, `doc/husky_ros2.md` troubleshooting).
- [ ] Arm + Robotiq gripper launched on Alice's PC:

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

**Belle**

- [ ] Arm powered on, teach pendant in **external control** mode; her own remote e-stop paired
      and in reach (pair one husky at a time).
- [ ] Pendant tool setting the same as Alice's (the monitor's handoff compares its FK tool0 with
      the TCP the arm reports; a wrong pendant TCP shows as a constant offset along tool Z).
- [ ] Arm + Robotiq gripper launched on Belle's PC:

```bash
ssh administrator@192.168.0.114
```

Then, **in that ssh shell on Belle**:

```bash
# * Belle's ROS must run CycloneDDS on domain 85, or this PC sees none of her topics.
export ROS_DOMAIN_ID=85 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
echo "[belle] domain $ROS_DOMAIN_ID | $RMW_IMPLEMENTATION"   # expect: domain 85 | rmw_cyclonedds_cpp
# The arm + the Robotiq 2F-85 gripper driver.
ros2 launch crl_husky crl_single_ur5e.launch.py namespace:='/a200_0805' gripper:=robotiq_2F_85
```

Leave all three launches running for the whole session. `Ctrl-C` in a window stops that robot's
arm and gripper / tool drivers (! never while that robot holds a bar). Section 2 checks that this
PC sees them.

- [ ] Motive streams Cindy's rigid body **1860**, Alice's **1840** and Belle's **1850**.

### 0.4 This PC's network

- [ ] This PC is on the robot network through its built-in port `eno1` (cable to the robot
      router; NetworkManager profile `Wired connection 1`, DHCP). No USB dongle is needed:

```bash
ip -4 -brief addr show eno1                    # expect: eno1  UP  192.168.0.x/24
for h in 192.168.0.115 192.168.0.113 192.168.0.114 192.168.0.28; do   # Cindy, Alice, Belle, the Motive PC
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

The mouse works in tmux on this PC (`~/.tmux.conf`):
- The wheel scrolls back through a pane's output (`q` returns to the live end); a click picks a
  pane.
- A drag selects text (yellow); **releasing the mouse copies it** to the system clipboard and
  keeps it highlighted. `Ctrl-C`, `Enter` or `y` on the selection copy it too and leave the
  selection; `q` / `Esc` leave without copying.
- Paste anywhere with `Ctrl-Shift-V`. Hold **Shift** while dragging for the terminal's own
  selection (copy it with `Ctrl-Shift-C`).

### 1.1 Monitor pane — paste before each monitor run

The domain decides which robot the monitor drives. Paste the block of the robot you start next.
The monitors run one after the other in the same pane; quit one with `Ctrl-C` before the next.

**Cindy** (runs 1, 3, 5):

```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=86
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml   # already in ~/.bashrc; pins DDS to eno1
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
ros2 daemon stop >/dev/null 2>&1
echo "[cindy env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}') | problem $DESIGN_PROBLEM_NAME"
```

**Alice** (her air checks in section 6, runs 2 and 6):

```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=84
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
ros2 daemon stop >/dev/null 2>&1
echo "[alice env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}') | problem $DESIGN_PROBLEM_NAME"
```

**Belle** (her air checks in section 6, runs 4 and 7):

```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=85
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
ros2 daemon stop >/dev/null 2>&1
echo "[belle env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}') | problem $DESIGN_PROBLEM_NAME"
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
export ROS_DOMAIN_ID=86               # 86 = Cindy, 84 = Alice, 85 = Belle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml   # already in ~/.bashrc; pins DDS to eno1
ros2 daemon stop >/dev/null 2>&1
echo "[robot env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}')"
ros2 topic info /a200_0806/cmd_vel --no-daemon --spin-time 5   # Cindy; Alice: /a200_0804, Belle: /a200_0805. Expect: Subscription count: 1 (twist_mux)
```

To switch robot in this pane: `export ROS_DOMAIN_ID=84` (or `85`, `86`), then `ros2 daemon stop`.

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

**Alice and Belle** (one block; set `NS` and the domain to the robot you check):

```bash
export ROS_DOMAIN_ID=84; NS=/a200_0804     # Alice. Belle: export ROS_DOMAIN_ID=85; NS=/a200_0805
ros2 daemon stop >/dev/null 2>&1
echo "--- joint states"; r=$(timeout 6 ros2 topic hz $NS/ur5e/rate_limiter/joint_states --window 20 2>&1 | grep 'average rate' | tail -1); echo "${r:-NO MESSAGES}"
echo "--- gripper action"; ros2 action list | grep gripper_cmd || echo "NO GRIPPER ACTION"
echo "--- controllers"
timeout 15 ros2 service call $NS/ur5e/controller_manager/list_controllers \
    controller_manager_msgs/srv/ListControllers 2>&1 | tr ',' '\n' \
    | grep -oE "name='[a-z_]+'|state='[a-z]+'" | paste - - | grep -E 'scaled_joint|cartesian_compliance'
```

- [ ] Joint states publish; the gripper action `<NS>/gripper/robotiq_gripper_controller/gripper_cmd`
      is listed (`NO GRIPPER ACTION`: the launch is missing `gripper:=robotiq_2F_85`, 0.3).
- [ ] `scaled_joint_trajectory_controller` **active**, `cartesian_compliance_controller` **inactive**.
- [ ] Done for **both** Alice and Belle.

---

## 3. Driving a base from the command line (when the joystick does not connect)

In the checks pane (1.2), with the domain of the robot you drive. The commands below drive Cindy.
For Alice use `ROS_DOMAIN_ID=84` and replace `/a200_0806` with **`/a200_0804`**; for Belle
`ROS_DOMAIN_ID=85` and **`/a200_0805`**. The base listens on `<ns>/cmd_vel` through `twist_mux`:
- **The base stops 0.5 s after the last message**, so commands are published continuously;
  stopping the publisher stops the base.
- The joystick, the RC remote and the interactive marker **override** `cmd_vel` while they
  publish; the Husky e-stop locks out everything.
- ! Hand on the e-stop; drive slowly near the structure. After driving, plan again (the monitor
  plans at the live base).
- ! Never drive Alice while she holds B21 (actions 3–16), nor Belle while she holds B35
  (actions 10–17).

Where a base should stand: on `Load Movement` the monitor logs
`[LoadMovement] cfab planning robot is correctly placed at [x y z], but: live husky [...] (err E m)`
when the live base is more than 1 mm from where the export authored the movement. `E` is how far
the real robot stands from that pose. Drive until the IK check (Cindy) or the free approach's goal
IK (Alice, Belle) succeeds; `E` does not have to reach 0.

**Keyboard (recommended):**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
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

! Every key typed in that pane is a drive command — quit it (`Ctrl-C`) before using the pane for
anything else.

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

- The remote e-stop **of the robot that moves** in hand for every arm motion (Cindy, Alice and
  Belle each have their own); the Husky e-stop does **not** stop the UR5e arms.
- Every new motion: preview with `Traj viz time` first, and raise the `traj time` slider for the
  first run. Loading a movement sets it to the movement's default:
  - Cindy: transfer 10 s, insert / retreat 5 s, free move home 10 s;
  - Alice and Belle: free approach 15 s, straight approach and hold-release retreat 5 s.
- `Cancel Exec` ends a waiting step.
- If a log line says an arm may still be under the compliance controller: press
  **`Switch to Joint (BOTH)`** (further down the panel, under `CONTROLLERS`; on a support robot it
  switches its one arm), or from the checks pane:

```bash
# Cindy (domain 86)
for a in left right; do
  ros2 service call /a200_0806/${a}_ur5e/controller_manager/switch_controller \
      controller_manager_msgs/srv/SwitchController \
      "{deactivate_controllers: [cartesian_compliance_controller], activate_controllers: [scaled_joint_trajectory_controller], strictness: 1}"
done
# Alice (domain 84); Belle (domain 85): /a200_0805 instead of /a200_0804
ros2 service call /a200_0804/ur5e/controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{deactivate_controllers: [cartesian_compliance_controller], activate_controllers: [scaled_joint_trajectory_controller], strictness: 1}"
```

---

## 5. What every monitor start prints

```bash
ros2 run husky_assembly_teleop husky_monitor
```

- [ ] One base calibration line per robot (all three huskies are created):
  - `Base calibration of Cindy (0806): 20260916/calibrated_transformation_0806_rhino.json`
  - `Base calibration of Alice (0804): 20260623/calibrated_transformation_0804_rhino.json`
  - `Base calibration of Belle (0805): 20261013/calibrated_transformation_0805_rhino.json`

  `Base calibration file not found for robot ...` = that robot's file is missing or not synced
  yet (0.1): stop before that robot's actions.
- [ ] `[Plan skeleton] robot <Cindy (domain 86) | Alice (domain 84) | Belle (domain 85)> | problem 261006_3bar_holding_test | progress k/18 done`,
      `[Plan skeleton] others: ...`, the 18-action roster, and `mocap client connected: True`
      (`False`: `CLIENT_IP`, see 0.1).
- [ ] Panel: a **Plan skeleton** section, with:
  - `Ignore built-bar collisions (bars drawn faint)` unticked;
  - the slider above `Load Movement` called `Step`;
  - an `Actions` list. Clicking a row selects that action: the
    `Action (idx)` slider jumps there, and nothing is loaded until `Load action`. The other
    robots' actions are grey, `(other robot)`.
- [ ] PyBullet:
  - the connected robot drawn at its real base (mocap) with its real arm poses
    (`Toggle Joint Live Stream` follows the joints);
  - the other two drawn where mocap sees them, or where `progress.json` left them.

---

## 6. Support robots — gripper and handoff checks in the air (before action 0; no bar, clear workspace)

**Belle's** grip handoff has never run on hardware. **Alice's** ran on 2026-10-12; its tuning is
still open (`tasks/2026-10-12_alice_grip_handoff_tuning.md`). Check both once before the assembly,
so a problem shows up before Cindy holds a bar for them. **Do not `Mark action done` here** — these
are tests. Run the checks below once per robot, **Belle first**, with that robot's values:

| | Belle | Alice |
|---|---|---|
| monitor env (1.1) | Belle block | Alice block |
| namespace, checks-pane domain | `/a200_0805`, 85 | `/a200_0804`, 84 |
| hold action, its close step | 10, `B35_H_M3_gripper_close` | 3, `B21_H_M3_gripper_close` |

Start the robot's monitor (its 1.1 block, then `ros2 run husky_assembly_teleop husky_monitor`):

- [ ] Section 5's lines with `robot <Belle (domain 85) | Alice (domain 84)> ... progress 0/18 done`
      and both others `<- parked` (nothing done yet).
- [ ] Panel: a **Gripper** section (`Open Gripper Full`, `Close Gripper for Bar`) and
      `Switch to Compliance (BOTH)` / `Switch to Joint (BOTH)` / `Zero Force Sensor (BOTH)`.
- [ ] **Gripper by hand:** `Open Gripper Full`, then `Close Gripper for Bar` → the jaws open and
      close.
  - To watch the fingers, checks pane on the robot's domain:
    `ros2 topic echo <NS>/gripper/joint_states --field position` (≈ 0.43 open, ~0.79 after an
    empty close; `Ctrl-C` afterwards).
  - The gripper action itself sends no feedback, only its final result (checked on Alice
    2026-10-12); the handoff below does not need any.
  - The Robotiq driver may still close at 15 % speed: a close of ~4 s is expected until the
    robot gets Cindy's 2026-09-09 driver fix (`scripts/husky/patch_robotiq_driver.py`).
- [ ] **Controller switch by hand** (hand on the robot's e-stop, gripper empty, arm in a safe pose):
      `Switch to Compliance (BOTH)` → the arm stays where it is; the section 2 check shows
      `cartesian_compliance_controller` active. `Switch to Joint (BOTH)` → back to
      `scaled_joint_trajectory_controller`.
- [ ] **The handoff in the air** (the jaws close on nothing):
  1. `Open Gripper Full`.
  2. Select the hold action (10 for Belle, 3 for Alice) → `Load action` → `Step` → 3 (readout
     `step 4/4: <close step> ...`) → `Load Movement`.
  3. `Exec gripper step: CLOSE + compliant handoff` → log
     `[gripper step <close step>] CLOSE the gripper on the bar (position 0.8), the arm going compliant BEFORE the close [SupportGripper]. Click 'Confirm Exec' ...`
  4. `Confirm Exec`. Expect, in order:
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
- [ ] ! **Belle, first time:** the monitor turns its targets by the same 90° as Alice's (her
      calibrated URDF has the same arm-base turn). This has not been confirmed on her hardware
      yet. If her arm moves at `phase 2/3` or `3/3`: e-stop, then note how far and which way.
      The drift limit stops the hold at 10 mm.
- [ ] Afterwards the section 2 check shows the **joint controller active**.
- [ ] If instead:
  - `FK tool0 and the arm-reported TCP disagree by ...` (no handoff): the robot's TCP frame does
    not match the monitor's FK. Note the numbers. A constant offset along tool Z = the pendant's
    tool setting (0.3).
  - `switch to cartesian_compliance_controller FAILED: the gripper was NOT closed`: the controller
    switch was refused. Check section 2's controllers, then run the step again.
- [ ] Repeat once and press `Cancel Exec` during phase 3 → the hold ends **and the controller
      switches back to joint** (the gripper keeps closing with the arm stiff — logged).
- [ ] `Open Gripper Full` → quit the monitor (`Ctrl-C`) **without** marking anything. Then the
      same for Alice.

---

## 7. Cindy's run 1 — actions 0–2

### 7.1 Start Cindy's monitor (monitor pane, after the 1.1 Cindy block)

```bash
ros2 run husky_assembly_teleop husky_monitor
```

- [ ] Section 5's lines with `[Plan skeleton] robot Cindy (domain 86) | problem 261006_3bar_holding_test | progress 0/18 done`
      and `[Plan skeleton] others: Alice <- parked | Belle <- parked`.
- [ ] Tick `Workaround: plan the transfer as a free move (no bar, from the live arms)` (under
      `Plan Movement`) → the five `Transfer start:` sliders above it disappear (they come back
      when it is unticked).

### 7.2 Action 0 — B20 jointing (ground bar)

Select action 0 → `Load action`. Six steps: travel to load (skipped), manual mount, tool grasp,
transfer, insert, fix foundation (manual). B20 stands on its two ground joints
(`joint_G20-T20-0_ground`, `joint_G20-T20-1_ground`) and has no tighten step.

- [ ] **Transfer** (`B20_J_M3_CDFM_transfer_to_approach`): `Load action` opens on it, already loaded
      (readout `step 4/6: B20_J_M3_CDFM_transfer_to_approach ...`, `traj time` 10 s). To come back
      to it later: `Step` → 3 → `Load Movement`.
  - [ ] **IK check** (workaround box ticked): `Transfer start: Confirm manual pose (IK check)` →
        `[transfer workaround] insertion start reachable from the live base (blue ghost); max |goal - stored goal| = ... rad`.
        The blue ghost shows the arms at the insertion start. Nothing moves and nothing is staged
        for `Exec` yet.
  - Optional, any time a movement is loaded: `Draw tool0: live (mocap + FK) vs target (Rhino)`.
    It draws each arm's tool0 twice in PyBullet, joined by a black line:
    `<side> tool0 target (Rhino)` (long axes, the loaded movement's exported target) and
    `<side> tool0 live (mocap + FK)` (short axes, the live arm joints on the mocap base).
    It logs the gap:
    `[tool0 check] B20_J_M3_CDFM_transfer_to_approach: tool0 live (mocap base + FK) minus target (Rhino): left ... mm (dx/dy/dz ...), axes x/y/z ... deg | right ...`.
    Before the transfer runs the gap is large (the arms are not there yet); click it again after
    the transfer to see what is left. A movement without target poses only warns.
  - Optional, for the insert / retreat: tick `Visual diagnosis (see-through robot, insert/retreat
    tool0 frames + path)` (under the button above). The grey live robot and its tools turn
    see-through. Every insert / retreat plan (`Plan Movement`, and the retreat's replan during
    exec) then draws, per arm:
    - `<side> Rhino start` / `<side> Rhino target` (long axes, the exported tool0 frames; the
      start is the previous movement's target);
    - `<side> plan start` / `<side> plan end` (short axes, the plan's first and last waypoint;
      for the retreat the start is the live tool0);
    - a black line from each Rhino frame to its plan frame;
    - the waypoint path IK follows (green line with small axes; red from the failing waypoint on
      when the plan failed).

    It logs `[B20_R_M1_LM_retreat viz] left: N waypoints over 50.0 mm; plan start is d mm / a deg from Rhino start; plan end is ... from Rhino target.`
    Ticking it after a plan draws that last plan; unticking removes the drawing and brings the
    robot back. For the insert only the left Rhino target is used (the right tool follows the
    bar), so a right `plan end` gap means the two exported targets disagree.
  - `... no collision-free IK for the insertion start at the live base ...`: the base stands too
    far from (or too close to) B20's Rhino pose, or the drawn colliding pair is in the way. Drive
    the base (section 3) and click the IK check again.
  - [ ] `Plan Movement` → three lines:
        - `[transfer workaround] B20_J_M3_CDFM_transfer_to_approach: goal from the last IK check (the base moved ... mm since).`
        - `[transfer workaround] free move (no bar) from the live arms to the insertion start; max |goal - stored goal| = ... rad`
          (same value as the IK check)
        - `[Plan] 'B20_J_M3_CDFM_transfer_to_approach': N waypoints stored.`

        If the base moved more than 2 mm since the check, the goal is solved again
        (`... solving the goal again.`).
  - [ ] Preview (`Traj viz time`): starts at the live arms, ends with the **empty** tools at the
        insertion start, just short of B20's assembled pose, clear of the floor.
  - [ ] Raise `traj time` (first run), `Exec Selected Mv Traj (auto)` → the arms go there. The
        exec then stays running (the next buttons are refused until it ends). It corrects the
        tool0 placement with the visual servo of the bar-holding accuracy test: the base's centre
        of mass shifts under the arms, so the tools land a few mm off the Rhino pose.
  - [ ] **Mount pause** (workaround box ticked):
        `[transfer B20_J_M3_CDFM_transfer_to_approach] the arms stand at the insertion start with EMPTY tools: mount the bar and clamp it with the tool buttons, then click 'Confirm Exec'. ...`
        → mount B20 by hand in both tools and clamp it with the **buttons on the tools** →
        `Confirm Exec`. This does the **Manual mount** and **Tool grasp** steps below: skip them,
        no click. `Cancel Exec` here ends it without a correction
        (`mount pause cancelled; no servo correction.`).
  - [ ] The insert is loaded by itself (`[Movement] loaded [..] 'B20_J_M4_LM_insert' ...`) and the
        servo runs: `### SERVOING iteration 1/8`, `[IK Live Base] ...`,
        `[transfer plan] OK (N waypoints, bar-held, LINEAR).` and the pause
        `[servo] First move planned: N waypoints, travels X deg ...` → scrub `Traj viz time`
        (a few mm with the bar held) → `Confirm Exec`.
        - Later iterations run alone unless a path travels more than 5 deg
          (`[servo] SAFEGUARD iter N: ...` → preview, `Confirm Exec`).
        - Each pass logs `[servo N] tool0 pos L=... R=... mm`.
        - The `Servoing Live Tracker` window opens by itself when the servo starts. It plots per
          iteration the tool0 position and rotation error per arm and the base drift, as in the
          bar-holding accuracy test. `Toggle Servoing Tracker` hides / shows it.
  - [ ] It ends with `### SERVOING converged at iter N (< 0.2 mm)` or
        `### SERVOING stopped at iter N: nothing left to plan ...` (or after 8 iterations). Then:
        - the record `... B20_J_M4_LM_insert_servoing_data_<ts>.json` / `_performance_<ts>.png`
          under `bar_action_live_replan_exe/<date>-servoing/` on the drive;
        - `[Servo] propagated arm configuration -> '<step>'.start_state.robot_configuration + robot_base_frame (live mocap).`
          for the steps around the transfer and the insert. The corrected arms, on the base
          where mocap sees it now, are the transfer's end and the insert's start.

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
  - B20 is held on its ground joints, so its insert runs **rigid only** (joint tracking,
    compliance never engaged) and the joint motors **never tighten**; the `Insert: ...` knobs do
    not apply.
  - Raise `traj time` (first run; the rigid insert has no force limit) →
    `Exec Selected Mv Traj (auto)` →
    `[B20_J_M4_LM_insert] grounded bar (holds joint_G20-T20-0_ground, joint_G20-T20-1_ground): rigid insert under scaled_joint_trajectory_controller (...), no tighten; ...`
    and, when the motion is done, `[B20_J_M4_LM_insert] grounded bar: rigid insert done (no tighten).`
  - Also once:
    `'B20_J_M4_LM_insert': the export asks for the cartesian_compliant controller; the monitor runs this dual_constrained_linear movement under joint_tracking.`
    (expected; Rhino note D9).
  - [ ] The joint motors do not turn; the controllers stay on `scaled_joint_trajectory_controller`.
- [ ] **Fix foundation** (`B20_J_M5_manual_fix_foundation`): `Step` → 5 → `Load Movement` →
      `Operator done (manual step) -> then Confirm Exec` → fix B20 to the foundation by hand →
      `Confirm Exec`.
- [ ] `Mark action done` → `[Plan skeleton] action 0 B20_J_joint marked done by Cindy; Cindy's belief <- live. Saved ...`,
      header `1/18`.

### 7.3 Action 1 — B20 release

`Load action` (slider on 1). Two steps; it opens on the retreat.

- [ ] Readout `step 1/2: B20_R_M1_LM_retreat  (+ B20_R_M0_tool_ungrasp_bar runs with it)`.
- [ ] B20 and its joints drawn solid (built, collision-checked).
- [ ] **Retreat** (`B20_R_M1_LM_retreat`). `Plan Movement` plans it from where the arms **are**
      (the insert leaves them a little off its planned end):
  - `[B20_R_M1_LM_retreat] start_state resynced to the live arms (max joint drift ... deg)` and
    `[retreat live] B20_R_M1_LM_retreat: goals = the live tool0 poses moved 50 mm back: left along (...), N mm from the exported goal; right ...`.
    Each tool0 backs off 50 mm along its exported retreat axis, orientation kept; scene
    collisions are checked. No path → the `[B20_R_M1_LM_retreat diag]` block (section 15).
  - Then preview (starts at the live arms) → `Exec Selected Mv Traj (auto)`. The log shows the
    ungrasp running with it (gripper motors LOOSENING), and once
    `'B20_R_M1_LM_retreat': the export asks for the joint_tracking controller; the monitor runs this dual_independent_linear movement under ...`
    (expected).
  - Before the compliant part both force sensors are zeroed:
    `force sensors zeroed before the compliant part (|F| a/b -> c/d N)` (the left / right reading
    before and after; a leftover force would otherwise push the tools into the joint).
  - After the compliant part the monitor prints its checks of the rigid part,
    `[B20_R_M1_LM_retreat] check PASS / FAIL ...` (replan, start, compliant part, contact,
    grippers, left / right tool0 path, joints, speed). It then runs the rigid part at once,
    **no `Confirm Exec`**: `running the rigid part now (N waypoints over T s); every check passed.`
    Or `N check(s) FAILED, see the "check FAIL" lines above`: note them in the log (section 15).
  - Joint controller restored afterwards.
- [ ] **Free move home** (`B20_R_M2_free_home`): `Step` → 1 → `Load Movement` → `Plan Movement` →
      preview → `Exec Selected Mv Traj (auto)`. It may be skipped: the retreat's end is already
      saved, and the next jointing action's transfer plans from where the arms are (collision
      checked).
- [ ] `Mark action done` → header `2/18`.

### 7.4 Action 2 — B21 jointing (one-sided: the left joint mates)

`Load action` (slider on 2). Five steps: travel to load (skipped), manual mount, tool grasp,
transfer, insert (the tighten runs with the insert). Same workflow as action 0, with these step
numbers:

- [ ] Built B20 and its joints drawn solid; the transfer's preview clears them.
- [ ] **Transfer** (`B21_J_M3_CDFM_transfer_to_approach`): already loaded by `Load action` (readout
      `step 4/5: ...`) → `Transfer start: Confirm manual pose (IK check)` → `Plan Movement` →
      preview → `Exec Selected Mv Traj (auto)` → mount pause: mount B21, clamp it with the tool
      buttons → `Confirm Exec` → the servo correction as in action 0 (first move: preview,
      `Confirm Exec`) → `### SERVOING converged ...` / `stopped ...`. Manual mount and tool grasp
      are done in that pause (box unticked: `Step` → 1 → `Confirm Exec`; `Step` → 2 → `Confirm Exec`).
- [ ] **Insert** (`B21_J_M5_LM_insert`): already loaded after the servo (else `Step` → 4).
  - Readout `step 5/5: B21_J_M5_LM_insert  (+ B21_J_M4_tool_tighten_joint runs with it)` →
    `Plan Movement` → `Exec Selected Mv Traj (auto)` → log
    `B21_J_M4_tool_tighten_joint ('tighten') runs with this movement: the compliant insert sends it (LEFT joint motor TIGHTENING only; the right joint has no female to screw into, its joint motor is not driven).`
  - B21 is **one-sided**: its left male (`joint_J20-21_male`) screws into B20. The right tool
    holds `joint_J157-21_male`, whose female sits on B157, outside this export. The export
    already names only `AT3L`, so there is no `tightens only ['AT3L']` warning.
  - Log `[B21_J_M5_LM_insert] one-sided bar: the left joint mates, so only the L joint motor TIGHTENs ...`
    and `[scaffolding] B21_J_M5_LM_insert: TIGHTEN joint motor on L arm (arm 0)` (no R line).
  - Rigid approach, then compliance for the last mm on **both** arms. The **left** joint motor
    tightens until it stalls (the right one is never driven), then the joint controller is
    restored.
- [ ] After the insert: Cindy holds B21 at its assembled pose, **joint controller active**
      (section 2's controller check).
- [ ] `Mark action done` → header `3/18`. Cindy's live base and arms were already saved to
      `progress.json` when the insert ended (Alice's run plans around them).
- [ ] **Quit Cindy's monitor** (`Ctrl-C`). Cindy keeps holding B21 (her driver keeps running). Do
      not touch her launch window.

---

## 8. Alice's run 2 — action 3: Alice grabs B21

Monitor pane: 1.1 **Alice** block, then `ros2 run husky_assembly_teleop husky_monitor`.

- [ ] Section 5's lines with `robot Alice (domain 84) ... progress 3/18 done` and
      `others: Cindy <- live (action 2) | Belle <- parked`; the slider on action 3, its row not grey.
- [ ] PyBullet: Cindy drawn holding B21 at her real base (mocap; her arms from `progress.json`).
      She may show twice at the same pose: the solid husky and the red obstacle robot — the red
      one is what the planner checks.
- [ ] Select 3 → `Load action` →
      `[obstacle robots] posed from: ObstacleRobotBelle <- exported, ObstacleRobotCindy <- live`
      (from `progress.json`; a fresh mocap base still overrides it). The export parks Belle far
      away. B21 is drawn in Cindy's tools.
  - B22 and B34–B37 may be drawn too although they are not built yet: the hold is exported with
    the bars that stand when it ends (Rhino note D5); the planner just avoids them.
  - Readout `step 1/4: B21_H_M0_free_to_approach ...`, `traj time` 15 s. The `Transfer start:`
    and `Insert: ...` controls are hidden.
- [ ] **Free approach** (`B21_H_M0_free_to_approach`): `Plan Movement` → the goal is re-solved by IK
      at Alice's live base, with her own exact ssik solver (built from her calibrated URDF, like
      Cindy's). Expect:
  - once per run `[goal IK] ssik solver 'alice' fits the planner's robot.`, then
    `[goal IK] ssik: solution pair 1/8 OK (...)`;
  - `[live goal] B21_H_M0_free_to_approach: goal re-solved at the live base; max |live IK - stored target| = ... rad.`;
  - `[Plan] 'B21_H_M0_free_to_approach': N waypoints stored.` (≈ 65, random planner, varies).

  `[goal IK] no ssik solver (alice, belle) fits the planner's robot: PyBullet IK instead ...` =
  the planning cell is not Alice's calibrated model: PyBullet IK is used instead, accurate to 1 mm
  only; note it.
  - `... no collision-free IK for its approach pose at the live base (...); not planning.` (the
    collision drawn if there is one): Alice stands where the approach pose cannot be reached —
    drive her base (section 3, domain 84, `/a200_0804`) and plan again.
  - `[live goal] ...: base not tracked; ...` = mocap does not see Alice (Motive 1840, 0.3).
  - [ ] Preview (`Traj viz time`): ends with the open jaws 100 mm in front of B21, clear of
        Cindy's arms and tools. Raise `traj time` → `Exec Selected Mv Traj (auto)` → the arm goes
        there. The exec then stays running and **servos tool0 onto the approach pose**, the same
        way as Cindy's transfer (the arm's motion shifts the base under it):
  - [ ] `[servo 0] tool0 pos X mm, rot Y deg` (before any correction), `### SERVOING iteration 1/8`,
        then the pause `[servo] First move planned: tool0 X mm, joints Y deg, over T s. ...` →
        scrub `Traj viz time` (a mm-size move) → `Confirm Exec`.
        - Later moves run alone unless they travel more than 5 deg (`[servo] SAFEGUARD ...` →
          `Confirm Exec`).
        - Each move is a joint move from the live arm to the exact IK solution at the live base,
          nearest the live arm.
        - The `Servoing Live Tracker` window opens by itself (one arm, plus the base drift).
  - [ ] It ends with `### SERVOING converged at iter N (< 0.2 mm)` or `### SERVOING stopped at
        iter N: nothing left to plan ...` (or after 8 moves). Then:
        - the record `B21_H_M0_free_to_approach_servoing_data_<ts>.json` /
          `_performance_<ts>.png` under `bar_action_live_replan_exe/<date>-servoing/`;
        - `[Servo] propagated arm configuration -> ... + robot_base_frame (live mocap).` for the
          gripper step and the straight approach. An approach planned before is dropped: plan it
          again.

        `[approach ...] servo skipped (FAKE_HARDWARE / the base is not tracked by mocap)` = no
        correction.
- [ ] **Calibration check** (Alice's June calibration, 0.1): the jaws now stand ≈ 100 mm in front of
      B21, centred on the bar. Off by more than ~1–2 cm sideways: stop, the base calibration is
      wrong (recalibrate Alice, `doc/calibration_manual.md`, and point her
      `BASE_CALIBRATION_DATE_BY_ROBOT` entry at the new folder).
- [ ] **Gripper open** (`B21_H_M1_gripper_open`): `Step` → 1 → `Load Movement` →
      `Exec gripper step: OPEN` → `[gripper step B21_H_M1_gripper_open] OPEN the gripper fully (position 0.426) [SupportGripper]. Click 'Confirm Exec' ...`
      → `Confirm Exec` → `gripper 0 done: position 0.426, ...`.
- [ ] **Straight approach** (`B21_H_M2_LM_to_grasp`): `Step` → 2 → `Load Movement` (`traj time` 5 s)
      → `Plan Movement`. Expect:
  - `[B21_H_M2_LM_to_grasp] start_state resynced to the live arms (max joint drift from the propagated conf: ... deg)`:
    it plans from where the arm really is, so the path stays on the arm's IK branch. A drift near
    360 deg = the chained start was a full turn off, now ignored.
  - `[Plan] 'B21_H_M2_LM_to_grasp': ≈ 22 waypoints stored.` (100 mm straight).

  Preview: the open jaws slide onto B21 → `Exec Selected Mv Traj (auto)`.
- [ ] **Gripper close + handoff** (`B21_H_M3_gripper_close`): `Step` → 3 → `Load Movement` →
      `Exec gripper step: CLOSE + compliant handoff` → `Confirm Exec` → the phase lines of
      section 6. The arm is compliant for the whole close: it gives way while the jaws centre
      on B21, then
      `gripper done: position ..., stalled True, ...` and the joint controller is back.
      Watch Cindy's tools: they must not be pulled.
- [ ] `Mark action done` → `[Plan skeleton] action 3 B21_H_hold marked done by Alice; Alice's belief <- live. Saved ...`,
      header `4/18`. (`Mocap has not seen Alice in the last 1 s; ...` = her base was not tracked: the
      authored base is stored instead; note it.)
- [ ] **Quit Alice's monitor** (`Ctrl-C`). Alice stays clamped on B21 (her driver keeps running).

---

## 9. Cindy's run 3 — actions 4–9

### 9.1 Restart Cindy's monitor

Monitor pane: 1.1 **Cindy** block, then `ros2 run husky_assembly_teleop husky_monitor`.

- [ ] Start-up: `[Plan skeleton] Cindy starts from the progress.json state (after action 2, live).`
      (her real joints take over at once), header `4/18`,
      `others: Alice <- live (action 3) | Belle <- parked`.
- [ ] PyBullet: Alice drawn on B21 at her hold pose (mocap base).
- [ ] **Tick the transfer workaround box again** (it starts unticked).

### 9.2 Action 4 — B21 release (Alice holding B21)

- [ ] `Load action` (slider on 4) → `[obstacle robots] posed from: ObstacleRobotAlice <- live, ObstacleRobotBelle <- exported`;
      it opens on the retreat (`step 1/2: B21_R_M1_LM_retreat  (+ B21_R_M0_tool_ungrasp_bar runs with it)`).
- [ ] **B21 is clamped by Alice** (the ungrasp releases Cindy's tools from it).
- [ ] **Retreat** and **free move home** as in action 1 (`B21_R_M1_LM_retreat`, then
      `B21_R_M2_free_home`), both planned with Alice in the scene. Check in the retreat's preview
      that both tools back off clear of Alice's gripper.
- [ ] `Mark action done` → header `5/18`.

### 9.3 Action 5 — B22 jointing (both joints mate)

`Load action` (slider on 5). Five steps, as action 2. B22's left male (`joint_J21-22_male`) screws
into B21, which Alice still holds; its right male (`joint_J20-22_male`) into B20.

- [ ] Built B20, B21 and Alice (red obstacle robot) drawn; the transfer's preview clears them all.
- [ ] **Transfer** (`B22_J_M3_CDFM_transfer_to_approach`): as action 2 (IK check → `Plan Movement` →
      preview → Exec → mount pause: mount B22, clamp → `Confirm Exec` → servo).
      A warning
      `Plan end of 'B22_J_M3_CDFM_transfer_to_approach' differs from existing 'B22_J_M5_LM_insert'.start by max 6.2832 rad/m; overwriting (carry rule).`
      means one joint of the insertion start is a full turn (2π) from the exported one: same tool
      poses, harmless (seen headless).
- [ ] **Insert** (`B22_J_M5_LM_insert`):
  - Readout `step 5/5: B22_J_M5_LM_insert  (+ B22_J_M4_tool_tighten_joint runs with it)` →
    `Plan Movement` → `Exec Selected Mv Traj (auto)` → log
    `B22_J_M4_tool_tighten_joint ('tighten') runs with this movement: the compliant insert sends it (joint motors TIGHTENING).`
    and a `[scaffolding] B22_J_M5_LM_insert: TIGHTEN joint motor on ...` line for **both** arms.
  - Rigid approach, then compliance for the last mm. Both joint motors tighten; the insert ends
    when **both** have stalled, then the joint controller is restored.
  - [ ] Watch B21 while B22 screws in: Alice holds it; it must not be pushed out of her jaws.
- [ ] `Mark action done` → header `6/18`.

### 9.4 Action 6 — B22 release

- [ ] `Load action` (slider on 6) → opens on `step 1/2: B22_R_M1_LM_retreat  (+ B22_R_M0_tool_ungrasp_bar runs with it)`.
- [ ] **Retreat** and **free move home** as in action 1 (`B22_R_M1_LM_retreat`, `B22_R_M2_free_home`),
      Alice in the scene.
- [ ] `Mark action done` → header `7/18`.

### 9.5 Action 7 — B34 jointing (ground bar, like action 0)

`Load action` (slider on 7). Six steps, as action 0. B34 is the second ground bar: it stands on its
ground joints `joint_G34-T20-0_ground` and `joint_G34-T20-1_ground`, has no tighten step, and ends
with a manual foundation fix. The base drives about 1.1 m from where it stood for B22.

- [ ] Built B20–B22 and Alice drawn; the transfer's preview clears them.
- [ ] **Transfer** (`B34_J_M3_CDFM_transfer_to_approach`): IK check → `Plan Movement` → preview →
      Exec → mount pause: mount B34, clamp → `Confirm Exec` → servo, as in action 0.
- [ ] **Insert, grounded bar** (`B34_J_M4_LM_insert`): as action 0 → `[B34_J_M4_LM_insert] grounded bar (holds joint_G34-T20-0_ground, joint_G34-T20-1_ground): rigid insert ...`
      and `... rigid insert done (no tighten).` The joint motors do not turn.
- [ ] **Fix foundation** (`B34_J_M5_manual_fix_foundation`): `Step` → 5 → `Load Movement` →
      `Operator done (manual step) -> then Confirm Exec` → fix B34 by hand → `Confirm Exec`.
- [ ] `Mark action done` → header `8/18`.

### 9.6 Action 8 — B34 release

- [ ] `Load action` (slider on 8) → `step 1/2: B34_R_M1_LM_retreat  (+ B34_R_M0_tool_ungrasp_bar runs with it)`.
- [ ] **Retreat** and **free move home** as in action 1 (`B34_R_M1_LM_retreat`, `B34_R_M2_free_home`).
- [ ] `Mark action done` → header `9/18`.

### 9.7 Action 9 — B35 jointing (one-sided: the left joint mates)

`Load action` (slider on 9). Five steps, as action 2. B35 is **one-sided**, like B21:
- its left male (`joint_J34-35_male`) screws into B34;
- the right tool holds `joint_J162-35_male`, whose female sits on B162, outside this export;
- the export names only `AT3L` for the tighten.

- [ ] **Transfer** (`B35_J_M3_CDFM_transfer_to_approach`): as action 2 (mount B35 in the pause).
- [ ] **Insert** (`B35_J_M5_LM_insert`): as action 2 → `[B35_J_M5_LM_insert] one-sided bar: the left joint mates, so only the L joint motor TIGHTENs ...`
      and `[scaffolding] B35_J_M5_LM_insert: TIGHTEN joint motor on L arm (arm 0)` (no R line).
- [ ] After the insert: Cindy holds B35 at its assembled pose, **joint controller active**.
- [ ] `Mark action done` → header `10/18` (Belle's run plans around Cindy's stored pose).
- [ ] **Quit Cindy's monitor** (`Ctrl-C`). Cindy keeps holding B35.

---

## 10. Belle's run 4 — action 10: Belle grabs B35

The same four steps as Alice's hold (section 8); only the differences are spelled out here.
Her start-up prints `Base calibration of Belle (0805): 20261013/...` (0.1); the calibration check
after the free approach below is the first test of it on an assembly.

Monitor pane: 1.1 **Belle** block, then `ros2 run husky_assembly_teleop husky_monitor`.

- [ ] Section 5's lines with `robot Belle (domain 85) ... progress 10/18 done` and
      `others: Cindy <- live (action 9) | Alice <- live (action 3)`; the slider on action 10, its row
      not grey.
- [ ] PyBullet: Cindy drawn holding B35, Alice clamped on B21.
- [ ] Select 10 → `Load action` →
      `[obstacle robots] posed from: ObstacleRobotAlice <- live, ObstacleRobotCindy <- live`.
      B36 and B37 may be drawn although not built yet (as in section 8). Readout
      `step 1/4: B35_H_M0_free_to_approach ...`, `traj time` 15 s.
- [ ] **Free approach** (`B35_H_M0_free_to_approach`): `Plan Movement`. The goal is re-solved
      with Belle's own exact ssik solver (built 2026-10-13 from her calibrated URDF, like
      Alice's). Once per run:
  - `[goal IK] ssik solver 'alice' does not fit the planner's robot (built from another robot or calibration).`
    (expected: Alice's solver is tried first);
  - `[goal IK] ssik solver 'belle' fits the planner's robot.`, then
    `[goal IK] ssik: solution pair 1/8 OK (...)`.

  Then on every plan:
  - `[live goal] B35_H_M0_free_to_approach: goal re-solved at the live base; ...`;
  - `[Plan] 'B35_H_M0_free_to_approach': N waypoints stored.` (≈ 63, varies).

  `[goal IK] no ssik solver (alice, belle) fits the planner's robot: PyBullet IK instead ...` =
  the planning cell is not Belle's calibrated model; note it.

  Errors as for Alice: no IK at the live base → drive Belle (section 3, domain 85, `/a200_0805`);
  `base not tracked` → Motive 1850.
  - [ ] Preview: ends with the open jaws 100 mm in front of B35, clear of Cindy's arms and tools.
        Raise `traj time` → `Exec Selected Mv Traj (auto)` → the servo onto the approach pose as
        in section 8 (first move: preview, `Confirm Exec`).
  - [ ] It ends with `### SERVOING converged at iter N (< 0.2 mm)` (headless, a 10 mm base
        shift: one move, 10 → 0.00 mm; or it stops after 8), the
        `B35_H_M0_free_to_approach_servoing_*` record and the `[Servo] propagated ...` lines.
- [ ] **Calibration check** (Belle's 2026-10-14 calibration, 0.1): the jaws stand ≈ 100 mm in front of B35,
      centred on the bar. Off by more than ~1–2 cm sideways: stop, her base calibration is wrong.
- [ ] **Gripper open** (`B35_H_M1_gripper_open`): `Step` → 1 → `Load Movement` →
      `Exec gripper step: OPEN` → `Confirm Exec` → `gripper 0 done: position 0.426, ...`.
- [ ] **Straight approach** (`B35_H_M2_LM_to_grasp`): `Step` → 2 → `Load Movement` → `Plan Movement`
      → `start_state resynced to the live arms ...`, `[Plan] 'B35_H_M2_LM_to_grasp': ≈ 22 waypoints stored.`
      → preview: the open jaws slide onto B35 → `Exec Selected Mv Traj (auto)`.
- [ ] **Gripper close + handoff** (`B35_H_M3_gripper_close`): `Step` → 3 → `Load Movement` →
      `Exec gripper step: CLOSE + compliant handoff` → `Confirm Exec` → the phase lines of
      section 6, hand on Belle's e-stop. Watch Cindy's tools: they must not be pulled.
- [ ] `Mark action done` → `[Plan skeleton] action 10 B35_H_hold marked done by Belle; Belle's belief <- live. Saved ...`,
      header `11/18`.
- [ ] **Quit Belle's monitor** (`Ctrl-C`). Belle stays clamped on B35.

---

## 11. Cindy's run 5 — actions 11–15

### 11.1 Restart Cindy's monitor

Monitor pane: 1.1 **Cindy** block, then `ros2 run husky_assembly_teleop husky_monitor`.

- [ ] Start-up: `[Plan skeleton] Cindy starts from the progress.json state (after action 9, live).`,
      header `11/18`, `others: Alice <- live (action 3) | Belle <- live (action 10)`.
- [ ] PyBullet: Alice on B21 and Belle on B35, at their hold poses (mocap bases).
- [ ] **Tick the transfer workaround box again.**

### 11.2 Action 11 — B35 release (Belle holding B35)

- [ ] `Load action` (slider on 11) → `[obstacle robots] posed from: ObstacleRobotAlice <- live, ObstacleRobotBelle <- live`;
      it opens on `step 1/2: B35_R_M1_LM_retreat  (+ B35_R_M0_tool_ungrasp_bar runs with it)`.
- [ ] **B35 is clamped by Belle** (the ungrasp releases Cindy's tools from it).
- [ ] **Retreat** and **free move home** as in action 1 (`B35_R_M1_LM_retreat`, `B35_R_M2_free_home`).
      Check in the retreat's preview that both tools back off clear of Belle's gripper.
- [ ] `Mark action done` → header `12/18`.

### 11.3 Action 12 — B36 jointing (both joints mate)

`Load action` (slider on 12). Five steps, as action 5. B36's left male (`joint_J35-36_male`) screws
into B35, which Belle holds; its right male (`joint_J34-36_male`) into B34.

- [ ] **Transfer** (`B36_J_M3_CDFM_transfer_to_approach`): as action 5. Two messages seen headless:
  - the same 2π warning as B22's (`... differs from existing 'B36_J_M5_LM_insert'.start by max 6.2832 ...`):
    harmless;
  - once, `[free validation] ... SWEPT COLLISION ... hits ['ground_WG0']` and
    `plan REJECTED: it sweeps through the scene between waypoints`. The insertion start is close
    to the floor and one random path grazed it: `Plan Movement` again.
- [ ] **Insert** (`B36_J_M5_LM_insert`): as action 5, **both** joint motors tighten.
  - [ ] Watch B35 while B36 screws in: Belle holds it; it must not be pushed out of her jaws.
- [ ] `Mark action done` → header `13/18`.

### 11.4 Action 13 — B36 release

- [ ] `Load action` (slider on 13) → `step 1/2: B36_R_M1_LM_retreat ...` → **retreat** and **free
      move home** as in action 1, Alice and Belle in the scene.
- [ ] `Mark action done` → header `14/18`.

### 11.5 Action 14 — B37 jointing (both joints mate, into the two held bars)

`Load action` (slider on 14). Five steps, as action 5. The base drives about 1.5 m from where it
stood for B36. B37's left male (`joint_J21-37_male`) screws into **B21 (Alice)**; its right male
(`joint_J35-37_male`) into **B35 (Belle)**.

- [ ] **Transfer** (`B37_J_M3_CDFM_transfer_to_approach`): as action 5; the 2π warning may show here
      too (harmless).
- [ ] **Insert** (`B37_J_M5_LM_insert`): as action 5, **both** joint motors tighten.
  - [ ] Watch B21 and B35 while B37 screws in: neither may be pushed out of its support robot's
        jaws.
- [ ] `Mark action done` → header `15/18`.

### 11.6 Action 15 — B37 release

- [ ] `Load action` (slider on 15) → `step 1/2: B37_R_M1_LM_retreat ...` → **retreat** as in action 1.
- [ ] **Free move home** (`B37_R_M2_free_home`). Headless, the first `Plan Movement` failed twice
      out of two (`plan_free_dual_arm failed: birrt_failed` and its diag block). The second try
      planned. ! Expect that and **plan again**.
  - The home goal turns the right shoulder (`right_ur_arm_shoulder_pan_joint`) by about
    **5.05 rad (≈ 290°)**, past B36.
  - Preview it through, raise `traj time`, hand on Cindy's e-stop.
- [ ] `Mark action done` → header `16/18`.
- [ ] **Quit Cindy's monitor** (`Ctrl-C`).

---

## 12. Alice's run 6 — action 16: Alice lets go of B21

B21 now hangs in the structure (jointed to B20, B22 and B37); Alice opens her jaws and backs out
100 mm along her approach line.

Monitor pane: 1.1 **Alice** block, then `ros2 run husky_assembly_teleop husky_monitor`.

- [ ] Section 5's lines with `robot Alice (domain 84) ... progress 16/18 done` and
      `others: Cindy <- live (action 15) | Belle <- live (action 10)`.
- [ ] Select 16 → `Load action` → readout `step 1/2: B21_HR_M0_gripper_open ...`.
- [ ] **Gripper open** (`B21_HR_M0_gripper_open`): `Exec gripper step: OPEN` → `Confirm Exec` →
      `gripper 0 done: position 0.426, ...`. B21 must stay where it is.
- [ ] **Retreat** (`B21_HR_M1_LM_retreat`): `Step` → 1 → `Load Movement` (`traj time` 5 s) →
      `Plan Movement`. Expect:
  - `[B21_HR_M1_LM_retreat] start_state resynced to the live arms (max joint drift ... deg)`:
    it starts where the handoff left the arm;
  - `[Plan] 'B21_HR_M1_LM_retreat': ≈ 21 waypoints stored.`, back to the approach pose.

  Preview: the open jaws slide off B21 → `Exec Selected Mv Traj` (plain joint tracking, no
  servo). Watch the jaws clear the bar.
- [ ] `Mark action done` → `[Plan skeleton] action 16 B21_HR_hold_release marked done by Alice; Alice's belief <- live. Saved ...`,
      header `17/18`.
- [ ] **Quit Alice's monitor**, then drive Alice away from the structure (section 3, domain 84).
      The export assumes she leaves the scene, and Belle's run draws her parked.

## 13. Belle's run 7 — action 17: Belle lets go of B35

The same as section 12, with Belle's values.

Monitor pane: 1.1 **Belle** block, then `ros2 run husky_assembly_teleop husky_monitor`.

- [ ] `robot Belle (domain 85) ... progress 17/18 done`.
- [ ] Select 17 → `Load action` →
      `[obstacle robots] posed from: ObstacleRobotAlice <- parked, ObstacleRobotCindy <- live`
      (Alice released). Readout `step 1/2: B35_HR_M0_gripper_open ...`.
- [ ] **Gripper open** (`B35_HR_M0_gripper_open`) → `Confirm Exec` → B35 stays where it is.
- [ ] **Retreat** (`B35_HR_M1_LM_retreat`): `Step` → 1 → `Load Movement` → `Plan Movement` →
      `[Plan] 'B35_HR_M1_LM_retreat': ≈ 21 waypoints stored.` → preview → `Exec Selected Mv Traj`.
      Watch the jaws clear B35.
- [ ] `Mark action done` → header `18/18`.
- [ ] Quit Belle's monitor; drive Belle away.

---

## 14. Starting over / redoing an action

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

## 15. If something goes wrong

| situation | do |
|---|---|
| a step waits and you want out | `Cancel Exec` |
| "refused: a step / execution is still running" | finish it or `Cancel Exec`, then retry |
| arm may be left compliant (loud error in the log) | `Switch to Joint (BOTH)` or the service call in section 4 |
| `Exec` refused: arms not at the trajectory start | `Move Arms to Movement Start (offline target)` (refuses joint changes above pi/3), then `Exec` |
| the transfer plans with the bar held / tries the bar-held planner | the workaround box is unticked (it starts unticked after every restart): tick it, plan again |
| `[transfer workaround] ... no collision-free IK for the insertion start` (IK check or plan) | the base is too far from / too close to the Rhino pose, or the drawn colliding pair is in the way (a support robot?): move the base (section 3), IK check again; note it in the log |
| the IK check passes although the base is far from the Rhino pose; it prints `[M1 manual] ...` lines | the workaround box is unticked (the `Transfer start:` sliders are showing): the button then checks the bar-loading start next to the robot, not the insertion start. Tick it and check again |
| `[free validation] ... SWEPT COLLISION ...` / `plan REJECTED: it sweeps through the scene between waypoints` | the random path grazed something between two waypoints (B36: the floor, `ground_WG0`): `Plan Movement` again |
| a plan fails with "start ... in collision" | read the pair in the log. A support robot's run with `ObstacleRobotCindy`, or the support robot against a built bar: tick `Ignore built-bar collisions (bars drawn faint)` (it reloads the action; the built bars are then ignored, Rhino note D5), plan again. Cindy's run with `ObstacleRobotAlice` / `ObstacleRobotBelle` in the pair: note it (the stored hold pose overlaps the scene); check the robot is drawn where it really stands |
| `not planning: joint value(s) outside the URDF limits` (right away, no search) | the start or goal has a joint the arm cannot reach (the UR5e elbow turns only +/- pi); the message names it. Should not happen for the transfer any more (its goal keeps every joint in range): note the line in the log |
| `Plan for '..._LM_retreat'` / `'..._LM_insert' ... FAILED` (a straight-line plan) | read the `[<movement> diag]` block after it (details below this table). A START collision the ACM does not allow = an export gap or the arms are not where the export expects: note the pair in the log. The retreat's replan during exec prints the same block, then runs the planned tail instead (its first step jumps; no pause) |
| the retreat's rigid part ran with `N check(s) FAILED` | the checks only report (no pause); note the `check FAIL` lines (details below this table). Remote e-stop if the arms pull on the bar |
| `plan_free_dual_arm failed: birrt_failed` (both ends collision-free, no path found) | read the `[... diag]` lines after it (details below this table). A tight pocket at the GOAL: drive the base to give the arms room, IK check, plan again. Otherwise plan again (action 15's free move home: expected on the first try) |
| `Plan end of '..._transfer_to_approach' differs from existing '..._LM_insert'.start by max 6.2832 rad/m` | one joint a full turn from the export's: same tool poses, harmless |
| `Cancel Exec` at the transfer's mount pause | no correction; the insert starts where the transfer ended (re-running the transfer is refused: the arms stand at its end, not its start). Mount and clamp the bar, then plan and run the insert as usual |
| `servo skipped (FAKE_HARDWARE)` / `(the base is not tracked by mocap)` after the transfer or a free approach | `FAKE_HARDWARE = 1`, or `USE_MOCAP = 0` / `USE_CELL_STATE_BASE_POSE = 1` (section 0.1): the servo needs the real arms and the mocap base |
| `Servoing: live-base IK failed; stopping.` / `[transfer plan] the straight-line bar move could not be solved ...` | the corrected pose is not reachable (or collides) from where the base stands; with the sampling fallback check the `SAFEGUARD` figures before confirming. The insert then starts where the servo left the arms |
| Alice / Belle: `no collision-free IK for its approach pose at the live base` | drive the robot (section 3, its domain) towards its hold spot, plan again |
| Belle: `ssik solver 'alice' does not fit the planner's robot ...` | expected: Alice's solver is tried first, then `'belle' fits` |
| Alice / Belle: `no ssik solver (alice, belle) fits the planner's robot: PyBullet IK instead` | not expected: the planning cell is not the robot's calibrated model (PyBullet IK, 1 mm only); note it |
| Alice / Belle: `FK tool0 and the arm-reported TCP disagree ...` | no compliant handoff: the gripper closed with the arm stiff. The bar is still clamped; note the numbers. Belle, constant offset along tool Z: her pendant tool setting (0.3) |
| Alice / Belle: `switch to cartesian_compliance_controller FAILED: the gripper was NOT closed` | nothing closed: check the robot's controllers (section 2; `Switch to Joint (BOTH)` if needed), then run the close step again |
| Belle's arm moves when the handoff goes compliant | e-stop; her controller frame may differ from Alice's (section 6). Note how far and which way, close stiff with `Close Gripper for Bar` instead |
| the jaws not centred on the bar after the free approach | that robot's base calibration is off (0.1): stop before the straight approach; recalibrate it |
| `Base calibration file not found for robot 0805` (Belle) / `0804` (Alice) | the robot's file is not in the folder its `BASE_CALIBRATION_DATE_BY_ROBOT` entry names (0.1), or Insync has not synced it |
| no topics / `Unknown topic` | `eno1` on `192.168.0.x` and the robot pings (0.4), the right domain + CycloneDDS in this pane (1.1 / 1.2), the robot's launch still running with its domain + CycloneDDS (0.3) |
| no tool status (section 2) / grasp steps get no answer | `... ScaffoldingToolStatus' is invalid`: the checks pane did not source the workspace (1.2), the tools are fine. `NO MESSAGES`: Cindy's launch is missing `gripper:=scaffolding_v3` (0.3) |
| `mocap client connected: False` | `CLIENT_IP` is not this PC's `eno1` address (0.1), or the Motive PC is no longer `192.168.0.28` (`doc/calibration_manual.md` 1.4) |
| marked an action done by mistake | load it, `Reopen action (reload clean)` (a support robot's action from its own monitor) |
| progress is wrong beyond repair | `Reset plan skeleton to the Rhino export` (section 14) |

**Reading the diagnosis blocks:**
- **A failed straight-line plan** (`[<movement> diag]`) shows:
  - the START's colliding pairs, with the movement's ACM entry of each body (a contact is
    allowed only when listed there);
  - the start's joint limits;
  - the failing waypoint and why its IK candidates were turned down (collide / would flip an
    arm);
  - a `VERDICT`.

  The red planning robot and the contacts are drawn, plus that waypoint's two tool0 targets
  (`Remove all drawing` clears them).
- **The retreat's `check FAIL` lines:**
  - `grippers` not `STALLED` = a gripper did not open;
  - `contact` = a tool still pushed on the structure;
  - `compliant part` may fail from gravity sag alone (the limits are first guesses);
  - `replan` = the fallback tail ran, whose first step jumps.
- **A failed free plan** (`birrt_failed`) shows: the joint limits of start and goal, how many
  nearby configurations collide at each end (and with what), where the straight joint path first
  collides (drawn; the red robot stands there), and a `VERDICT` line.

## 16. What to send back

- The monitor log (`~/husky_real/monitor.log`, or copy the terminal); it holds all seven monitor
  runs.
- `~/Insync/2025-03 Husky Assembly/data_design_study/261006_3bar_holding_test/progress.json` after the run.
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
- **Alice and Belle** (sections 6, 8, 10, 12, 13):
  - the phase lines and any "no handoff" warning;
  - how the arm behaved while compliant (Belle: did it stay still when going compliant?);
  - the `[live goal]` line and the servo's iterations;
  - how far the open jaws were from the bar's centre after the free approach;
  - whether the held bar moved while it was clamped or while the next bars were screwed in;
  - whether the jaws cleared the bar on the release.
