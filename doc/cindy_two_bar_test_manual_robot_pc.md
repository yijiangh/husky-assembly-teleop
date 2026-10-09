# Cindy two-bar test (B1, B3) — with the real robot, run from the robot-control PC

> **This is a copy of `doc/cindy_two_bar_test_manual.md` for the robot-control PC**
> (`CRLRobotxPC`, user `su`). The test steps are the same; only the paths, the network and the
> terminal setup differ. If the original manual changes, carry the change over here.
>
> | | this PC |
> |---|---|
> | workspace | `~/ros2_ws` (venv `~/ros2_ws/venv`, ssik 4.1 installed) |
> | shared drive | `~/Insync/2025-03 Husky Assembly` (the package's built-in data paths already point here) |
> | robot network | built-in ethernet port **`eno1`**, DHCP from the router `192.168.0.1` (was `192.168.0.25` on 2026-10-07) |
> | DDS | `~/.cyclonedds.xml` pins CycloneDDS to `eno1`; `~/.bashrc` exports `CYCLONEDDS_URI` but **not** `ROS_DOMAIN_ID` / `RMW_IMPLEMENTATION` |
> | internet | Wi-Fi `wlp9s0` (+ Tailscale); ROS does not use them |

A standalone checklist for the first two bars of the design problem
**`261006_3bar_holding_test`** (export of 2026-10-06), run by Cindy only (dual-arm husky
`/a200_0806`, ROS domain 86). Alice is not used: a person or a stand holds B3 where Alice would.

| entry | action | who | operator steps |
|---|---|---|---|
| 0 | `B1_J_joint` | Cindy | B1 jointing: transfer → mount → grasp → insert → **fix foundation** (manual) |
| 1 | `B1_R_release` | Cindy | B1 release: retreat (the ungrasp runs with it) → free move home |
| 2 | `B3_J_joint` | Cindy | B3 jointing: transfer → mount → grasp → insert (the tighten runs with it) |
| 3 | `B3_H_hold` | (Alice) | **stand-in**: a person / stand holds B3; marked done from Cindy's monitor |
| 4 | `B3_R_release` | Cindy | B3 release, as entry 1 |

What this export changed (vs. the 2026-09-20 one): B1 (a ground bar) has **no tighten step**; its
insert is followed by a manual `B1_J_M5_manual_fix_foundation` step. The monitor runs a ground bar's
insert rigid and never tightens it (it recognises the ground joints held in the tools). The releases have **no
untighten step** any more (ungrasp → retreat → free move home).

> **! Transfer workaround in use** (until the bar-held transfer planner is ready). The transfer
> runs as a plain free move **without the bar**: tick
> `Workaround: plan the transfer as a free move (no bar, from the live arms)` (under
> `Plan Movement`) once per monitor run — it starts unticked. With the box ticked,
> `Transfer start: Confirm manual pose (IK check)` checks whether the arms reach the insertion
> start (the Rhino pose) from where the base stands; the `Transfer start:` sliders (the bar-loading
> pose) are not used and disappear from the panel while the box is ticked.
> Order of a jointing entry:
> **transfer** (empty tools, from the live arms to the insertion start) → **manual mount** (mount
> the bar by hand there) → **tool grasp** → **insert** (→ **fix foundation** for B1). The travel
> to load is skipped.

Every code block below is meant to be pasted as is. Tick the boxes as you go; anything that does
not match "expect" is worth a note (log lines + what you did). What the buttons do in detail:
`doc/support_robot_schedule_manual.md`.

---

## 0. Once, before the session

### 0.1 Monitor flags and the mocap address

The switches are class attributes at the top of `class HuskyMonitor`
(`husky_assembly_teleop/husky_monitor.py`; no rebuild needed after editing, restart the monitor).

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
| `CONNECT_COMPLIANT_CONTROLLER` | **1** |
| `LIST_CONTROLLER_SERVICES` | **1** |
| `IGNORE_BUILT_ASSEMBLY_COLLISIONS` | 0 (built bars are obstacles; leave the panel toggle unticked) |
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

### 0.2 The problem folder (shared Insync folder)

The monitor reads and writes the design problem directly in the shared Insync folder:
`Mark entry done` writes `progress.json` and `*.live-solved.json` (saved plans) there, and they sync
to the shared drive. Check that the folder is complete and see what an earlier run left behind:

```bash
GD="$HOME/Insync/2025-03 Husky Assembly"
P="$GD/data_design_study/261006_3bar_holding_test"
ls "$P"                                        # ActionSchedule.json BarActions RobotCell*.json WalkableGround.json ...
ls "$P"/BarActions/*__*.json | grep -vc live-solved     # 48 action files
ls -a "$P" | grep -c insyncdl                  # 0 = Insync has finished syncing
ls "$P"/progress.json "$P"/BarActions/*.live-solved.json 2>/dev/null   # left over from an earlier run?
```

If `progress.json` or saved plans are left over and you want to start fresh, use
`Reset schedule to the Rhino export` once the monitor is up (section 11): it moves them to
`$P/archive/<time>/` (nothing is deleted).

### 0.3 Cindy bring-up (robot side)

- [ ] Both arms powered on, teach pendants in **external control** mode (`doc/husky_ros2.md`; do
      not start `ros_control.urp` by hand).
- [ ] Motive streams Cindy's rigid body **1860**.
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

Leave it running for the whole session; `Ctrl-C` in that window stops the arm and tool drivers.
Section 2 checks that this PC sees them.

### 0.4 This PC's network

- [ ] This PC is on the robot network through its built-in port `eno1` (cable to the robot
      router; NetworkManager profile `Wired connection 1`, DHCP). No USB dongle is needed:

```bash
ip -4 -brief addr show eno1                    # expect: eno1  UP  192.168.0.x/24
for h in 192.168.0.115 192.168.0.28; do        # Cindy's PC, the Motive PC
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

### 1.1 Monitor pane — paste once per new terminal

```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=86
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml   # already in ~/.bashrc; pins DDS to eno1
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
ros2 daemon stop >/dev/null 2>&1
echo "[cindy env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}') | problem $DESIGN_PROBLEM_NAME"
```

- No data-path exports: the package's built-in `DESIGN_DATA_DIRECTORY` / `EXPERIMENT_DATA_DIRECTORY`
  are this PC's `~/Insync/2025-03 Husky Assembly/...` folders. Only the problem name is set (the
  built-in default is another problem).
- No `HUSKY_IK_BACKEND`: ssik is installed in this venv, so the default (ssik) is used.
- Keep a log of the monitor (from any other pane, once per session):
  `mkdir -p ~/husky_real && tmux pipe-pane -t husky_real:0.0 -o 'cat >> ~/husky_real/monitor.log'`.

### 1.2 Checks / base pane — paste once per new terminal

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=86
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml   # already in ~/.bashrc; pins DDS to eno1
ros2 daemon stop >/dev/null 2>&1
echo "[robot env] domain $ROS_DOMAIN_ID via $(ip -4 -brief addr show eno1 | awk '{print $1, $2, $3}')"
ros2 topic info /a200_0806/cmd_vel --no-daemon --spin-time 5   # expect: Subscription count: 1 (twist_mux)
```

`Unknown topic` = Cindy is not visible: wrong domain, her ROS is not on CycloneDDS, `eno1` is off
the robot network, or the base driver is down.

---

## 2. Connectivity checks (checks pane, after 1.2)

The controller check calls the controller manager's service directly (`ros2 control
list_controllers -c /a200_0806/left_ur5e/controller_manager` also works on this PC).

```bash
for t in /a200_0806/left_ur5e/rate_limiter/joint_states /a200_0806/right_ur5e/rate_limiter/joint_states \
         /a200_0806/left_gripper/tool_status /a200_0806/right_gripper/tool_status; do
  echo "--- $t"; timeout 6 ros2 topic hz $t --window 20 2>&1 | grep -m1 'average rate' || echo 'NO MESSAGES'
done
for a in left right; do
  echo "--- controllers $a"
  timeout 15 ros2 service call /a200_0806/${a}_ur5e/controller_manager/list_controllers \
      controller_manager_msgs/srv/ListControllers 2>&1 | tr ',' '\n' \
      | grep -oE "name='[a-z_]+'|state='[a-z]+'" | paste - - | grep -E 'scaled_joint|cartesian_compliance'
done
```

- [ ] Joint states ≈ **50 Hz** (both arms), tool status ≈ **20 Hz** (both tools).
- [ ] Both arms: `scaled_joint_trajectory_controller` **active**, `cartesian_compliance_controller` **inactive**.

---

## 3. Driving the base from the command line (when the joystick does not connect)

In the checks pane (1.2). Cindy's base listens on `/a200_0806/cmd_vel` through `twist_mux`:
- **The base stops 0.5 s after the last message**, so commands are published continuously;
  stopping the publisher stops the base.
- The joystick, the RC remote and the interactive marker **override** `cmd_vel` while they
  publish; the Husky e-stop locks out everything.
- ! Hand on the e-stop; drive slowly near the structure. After driving, plan again (the monitor
  plans at the live base).

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

- Remote e-stop in hand for every arm motion; the Husky e-stop does **not** stop the UR5e arms.
- Every new motion: preview with `Traj viz time` first, and raise the `traj time` slider for the
  first run (loading a movement sets it to the movement's default: transfer 10 s, insert / retreat
  5 s, free move home 10 s).
- `Cancel Exec` ends a waiting step.
- If a log line says an arm may still be under the compliance controller: press
  **`Switch to Joint (BOTH)`** (further down the panel, under `CONTROLLERS`), or from the checks pane:

```bash
for a in left right; do
  ros2 service call /a200_0806/${a}_ur5e/controller_manager/switch_controller \
      controller_manager_msgs/srv/SwitchController \
      "{deactivate_controllers: [cartesian_compliance_controller], activate_controllers: [scaled_joint_trajectory_controller], strictness: 1}"
done
```

---

## 5. Start the monitor (monitor pane, after 1.1)

```bash
ros2 run husky_assembly_teleop husky_monitor
```

- [ ] Terminal: `[Schedule] robot Cindy (domain 86) | problem 261006_3bar_holding_test | progress 0/48 done`,
      `[Schedule] others: Alice <- parked | Belle <- parked`, the 48-entry roster, and
      `mocap client connected: True` (`False`: `CLIENT_IP`, see 0.1).
- [ ] Panel: a **Schedule** section; `Ignore built-bar collisions (bars drawn faint)` unticked; the
      slider above `Load Movement` is called `Step`. Clicking a row of the `Schedule entries` list
      selects that entry (the `Schedule entry (idx)` slider jumps there; nothing is loaded until
      `Load entry`).
- [ ] PyBullet: Cindy drawn at her real base (mocap) with her real arm poses
      (`Toggle Joint Live Stream` shows 12 joints following the arms).
- [ ] Tick `Workaround: plan the transfer as a free move (no bar, from the live arms)` (under
      `Plan Movement`) → the five `Transfer start:` sliders above it disappear (they come back
      when it is unticked).

---

## 6. Entry 0 — B1 jointing (ground bar)

Select entry 0 → `Load entry`. Six steps: travel to load (skipped), manual mount, tool grasp,
transfer, insert, fix foundation (manual). B1 has no tighten step.

- [ ] **Transfer** (`B1_J_M3_CDFM_transfer_to_approach`): `Load entry` opens on it, already loaded
      (readout `step 4/6: B1_J_M3_CDFM_transfer_to_approach ...`, `traj time` 10 s). To come back
      to it later: `Step` → 3 → `Load Movement`.
  - [ ] **IK check** (workaround box ticked): `Transfer start: Confirm manual pose (IK check)` →
        `[transfer workaround] insertion start reachable from the live base (blue ghost); max |goal - stored goal| = ... rad`.
        The blue ghost shows the arms at the insertion start. Nothing moves and nothing is staged
        for `Exec` yet.
  - Optional, any time a movement is loaded: `Draw tool0: live (mocap + FK) vs target (Rhino)`
    draws each arm's tool0 twice in PyBullet: `<side> tool0 target (Rhino)` (long axes, the loaded
    movement's exported target) and `<side> tool0 live (mocap + FK)` (short axes, the live arm
    joints on the mocap base), joined by a black line, and logs the gap:
    `[tool0 check] B1_J_M3_CDFM_transfer_to_approach: tool0 live (mocap base + FK) minus target (Rhino): left ... mm (dx/dy/dz ...), axes x/y/z ... deg | right ...`.
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
    It logs `[B1_R_M1_LM_retreat viz] left: N waypoints over 50.0 mm; plan start is d mm / a deg from Rhino start; plan end is ... from Rhino target.`
    Ticking it after a plan draws that last plan; unticking removes the drawing and brings the
    robot back. For the insert only the left Rhino target is used (the right tool follows the
    bar), so a right `plan end` gap means the two exported targets disagree.
  - `... no collision-free IK for the insertion start at the live base ...`: the base stands too
    far from (or too close to) B1's Rhino pose, or the drawn colliding pair is in the way. Drive
    the base (section 3) and click the IK check again.
  - [ ] `Plan Movement` →
        `[transfer workaround] B1_J_M3_CDFM_transfer_to_approach: goal from the last IK check (the base moved ... mm since).`,
        `[transfer workaround] free move (no bar) from the live arms to the insertion start; max |goal - stored goal| = ... rad`
        (same value as the IK check) and `[Plan] 'B1_J_M3_CDFM_transfer_to_approach': N waypoints stored.`
        If the base moved more than 2 mm since the check, the goal is solved again
        (`... solving the goal again.`).
  - [ ] Preview (`Traj viz time`): starts at the live arms, ends with the **empty** tools at the
        insertion start, just short of B1's assembled pose, clear of the floor and anything built.
  - [ ] Raise `traj time` (first run), `Exec Selected Mv Traj (auto)` → the arms go there.
- [ ] **Manual mount** (`B1_J_M1_manual_mount_bar`): `Step` → 1 → `Load Movement` (the bar is drawn
      in the tools at the insertion start) → `Operator done (manual step) -> then Confirm Exec` →
      mount B1 by hand in both tools → `Confirm Exec` → `confirmed done by the operator`.
- [ ] **Tool grasp** (`B1_J_M2_tool_grasp_bar`): `Step` → 2 → `Load Movement` → `Tool step: grasp` →
      `Confirm Exec` → `STOP`, gripper motors tighten, `every gripper motor STALLED after N s: bar clamped`,
      `STOP sent`.
- [ ] **Insert, grounded bar** (`B1_J_M4_LM_insert`): `Step` → 4 → `Load Movement`; readout
      `step 5/6: B1_J_M4_LM_insert  (dual_constrained_linear)` → `Plan Movement` (it starts where the
      transfer ended) → preview; no ground-joint ↔ floor collision is reported.
      B1 is held on its ground joints, so its insert runs **rigid only** (joint tracking, compliance
      never engaged) and the joint motors **never tighten**; the `Insert: ...` knobs do not apply.
      Raise `traj time` (first run; the rigid insert has no force limit) → `Exec Selected Mv Traj (auto)` →
      `[B1_J_M4_LM_insert] grounded bar (holds joint_G1-T20Ground-0_ground, joint_G1-T20Ground-1_ground): rigid insert under scaled_joint_trajectory_controller (...), no tighten; ...`
      and, when the motion is done, `[B1_J_M4_LM_insert] grounded bar: rigid insert done (no tighten).`
      Also once: `'B1_J_M4_LM_insert': the export asks for the cartesian_compliant controller; the monitor runs this dual_constrained_linear movement under joint_tracking.`
      (expected; Rhino note D9).
  - [ ] The joint motors do not turn; the controllers stay on `scaled_joint_trajectory_controller`.
- [ ] **Fix foundation** (`B1_J_M5_manual_fix_foundation`): `Step` → 5 → `Load Movement` →
      `Operator done (manual step) -> then Confirm Exec` → fix B1 to the foundation by hand →
      `Confirm Exec`.
- [ ] `Mark entry done` → `[Schedule] entry 0 B1_J_joint marked done by Cindy; Cindy's belief <- live. Saved ...`,
      header `1/48`.

## 7. Entry 1 — B1 release

`Load entry` (slider on 1). Two steps; it opens on the retreat.

- [ ] Readout `step 1/2: B1_R_M1_LM_retreat  (+ B1_R_M0_tool_ungrasp_bar runs with it)`.
- [ ] B1 and its joints drawn solid (built, collision-checked).
- [ ] **Retreat** (`B1_R_M1_LM_retreat`): `Plan Movement` plans it from where the arms **are** (the
      compliant insert leaves them a little off its planned end):
      `[B1_R_M1_LM_retreat] start_state resynced to the live arms (max joint drift ... deg)` and
      `[retreat live] B1_R_M1_LM_retreat: goals = the live tool0 poses moved 50 mm back: left along (...), N mm from the exported goal; right ...`
      (each tool0 backs off 50 mm along its exported retreat axis, orientation kept; scene
      collisions are checked). No path → the `[B1_R_M1_LM_retreat diag]` block (section 12).
      Then preview (starts at the live arms) → `Exec Selected Mv Traj (auto)` →
      log: the ungrasp runs with it (gripper motors LOOSENING), and once
      `'B1_R_M1_LM_retreat': the export asks for the joint_tracking controller; the monitor runs this dual_independent_linear movement under ...` (expected).
      After the compliant part the monitor checks the rigid part: `[B1_R_M1_LM_retreat] check PASS / FAIL ...`
      lines (replan, start, compliant part, contact, grippers, left / right tool0 path, joints,
      speed). All PASS and `Retreat: skip Confirm when the checks pass` ticked (the default) →
      `every retreat check passed: running the rigid part now ..., no Confirm.` and the arms back
      off without a pause. Before the compliant part both force sensors are zeroed:
      `force sensors zeroed before the compliant part (|F| a/b -> c/d N)` (the left / right reading
      before and after; a leftover force would otherwise push the tools into the joint).
      A FAIL → `retreat ready, but N check(s) FAILED ...` → read the FAIL lines,
      check the preview → `Confirm Exec` (or `Cancel Exec`). Joint controller restored afterwards.
- [ ] **Free move home** (`B1_R_M2_free_home`): `Step` → 1 → `Load Movement` → `Plan Movement` →
      preview → `Exec Selected Mv Traj (auto)`.
- [ ] `Mark entry done` → header `2/48`.

## 8. Entry 2 — B3 jointing

`Load entry` (slider on 2). Five steps: travel to load (skipped), manual mount, tool grasp,
transfer, insert (the tighten runs with the insert). Same workflow as entry 0, with these step
numbers:

- [ ] Built B1 and its joints drawn solid; the transfer's preview clears them.
- [ ] **Transfer** (`B3_J_M3_CDFM_transfer_to_approach`): already loaded by `Load entry` (readout
      `step 4/5: ...`) → `Transfer start: Confirm manual pose (IK check)` → `Plan Movement` →
      preview → `Exec Selected Mv Traj (auto)`.
- [ ] **Manual mount** (`Step` → 1) → mount B3 → `Confirm Exec`; **tool grasp** (`Step` → 2) → `Confirm Exec`.
- [ ] **Insert** (`B3_J_M5_LM_insert`): `Step` → 4; readout
      `step 5/5: B3_J_M5_LM_insert  (+ B3_J_M4_tool_tighten_joint runs with it)` → `Plan Movement` →
      `Exec Selected Mv Traj (auto)` → log
      `B3_J_M4_tool_tighten_joint ('tighten') runs with this movement: the compliant insert sends it (LEFT joint motor TIGHTENING only; the right joint has no female to screw into, its joint motor is not driven).`
      and, until the Rhino re-export (note D10), the warning
      `'B3_J_M4_tool_tighten_joint': the export lists tool_names ['AT3L', 'AT3R']; the monitor tightens only ['AT3L'] ...` (expected).
      B3 is **one-sided**: its right male's female sits on the staging bar B2, so only the left
      joint is screwed in. Log `[B3_J_M5_LM_insert] one-sided bar: the left joint mates, so only the L joint motor TIGHTENs ...`
      and `[scaffolding] B3_J_M5_LM_insert: TIGHTEN joint motor on L arm (arm 0)` (no R line).
      Rigid approach, compliance for the last mm on **both** arms, the **left** joint motor
      tightens until it stalls (the right one is never driven), then the joint controller is
      restored.
- [ ] After the insert: Cindy holds B3 at its assembled pose, **joint controller active**
      (section 2's controller check).
- [ ] `Mark entry done` → header `3/48`.

## 9. Entry 3 — stand-in for Alice's hold of B3

In the full run Alice clamps B3 here, so it stays in place when Cindy lets go in entry 4.

- [ ] **A person or a stand now supports B3** in Alice's place (it must not depend on Cindy's tools
      any more).
- [ ] Select 3 → `Load entry` → warning `Entry 3 belongs to Alice; loaded for display only`.
- [ ] `Mark entry done` → the log asks to confirm marking it `done with the EXPORTED end state (belief 'assumed')` →
      `Confirm Exec` → `[Schedule] entry 3 B3_H_hold marked done by Cindy; Alice's belief <- assumed. Saved ...`,
      header `4/48` and `others: Alice <- assumed (entry 3) | ...`.
- [ ] PyBullet: Alice drawn at her hold pose next to B3. She is **not really there**: Cindy's
      planner treats her as an obstacle, which is conservative.

## 10. Entry 4 — B3 release

- [ ] `Load entry` (slider on 4) → `[obstacle robots] posed from: ObstacleRobotAlice <- assumed, ...`;
      it opens on the retreat (`step 1/2: B3_R_M1_LM_retreat  (+ B3_R_M0_tool_ungrasp_bar runs with it)`).
- [ ] **B3 is supported** by the person / stand (the ungrasp releases it).
- [ ] **Retreat** and **free move home** as in entry 1 (`B3_R_M1_LM_retreat`, then `B3_R_M2_free_home`).
  - If a plan fails because of Alice (`ObstacleRobotAlice` in the colliding pair): select 3 →
    `Load entry` → `Reopen entry (reload clean)` (Alice goes back to the export's pose for this
    release), then load entry 4 and plan again.
- [ ] `Mark entry done` → header `5/48` (or `4/48` if entry 3 was reopened).

---

## 11. Starting over / redoing an entry

- **Redo one entry:** load it → `Reopen entry (reload clean)` (pending again, clean export loaded,
  Cindy's previous end state restored).
- **Start the whole test over:** `Reset schedule to the Rhino export` → `Confirm Exec` →
  `[Schedule] reset to the Rhino export: moved N file(s) (...) to .../archive/<time>.`; every entry
  pending, entry 0 loaded clean. Nothing is deleted: the old `progress.json` and saved plans are in
  `~/Insync/2025-03 Husky Assembly/data_design_study/261006_3bar_holding_test/archive/<time>/`.

## 12. If something goes wrong

| situation | do |
|---|---|
| a step waits and you want out | `Cancel Exec` |
| "refused: a step / execution is still running" | finish it or `Cancel Exec`, then retry |
| arm may be left compliant (loud error in the log) | `Switch to Joint (BOTH)` or the service call in section 4 |
| `Exec` refused: arms not at the trajectory start | `Move Arms to Movement Start (offline target)` (refuses joint changes above pi/3), then `Exec` |
| the transfer plans with the bar held / tries the bar-held planner | the workaround box is unticked (it starts unticked after every restart): tick it, plan again |
| `[transfer workaround] ... no collision-free IK for the insertion start` (IK check or plan) | the base is too far from / too close to the Rhino pose, or the drawn colliding pair is in the way: move the base (section 3), IK check again; note it in the log |
| the IK check passes although the base is far from the Rhino pose; it prints `[M1 manual] ...` lines | the workaround box is unticked (the `Transfer start:` sliders are showing): the button then checks the bar-loading start next to the robot, not the insertion start. Tick it and check again |
| a plan fails with "start ... in collision" | read the pair in the log; in entry 4 see the Alice fallback in section 10 |
| `not planning: joint value(s) outside the URDF limits` (right away, no search) | the start or goal has a joint the arm cannot reach (the UR5e elbow turns only +/- pi); the message names it. Should not happen for the transfer any more (its goal keeps every joint in range): note the line in the log |
| `Plan for '..._LM_retreat'` / `'..._LM_insert' ... FAILED` (a straight-line plan) | read the `[<movement> diag]` block after it: the START's colliding pairs with the movement's ACM entry of each body (a contact is allowed only when listed there), its joint limits, the failing waypoint and why its IK candidates were turned down (collide / would flip an arm), and a `VERDICT`. The red planning robot and the contacts are drawn, plus that waypoint's two tool0 targets (`Remove all drawing` clears them). A START collision the ACM does not allow = an export gap or the arms are not where the export expects: note the pair in the log. The retreat's replan during exec prints the same block before it falls back to the planned tail (Confirm pause) |
| the retreat stops at `retreat ready, but N check(s) FAILED` | read the `check FAIL` lines: `grippers` not `STALLED` = a gripper did not open (do not confirm before it is open); `contact` = a tool still pushes on the structure; `compliant part` may fail from gravity sag alone (note its numbers; the limits are first guesses); `replan` = the fallback tail, whose first step jumps. Confirm only when the preview looks right. Untick `Retreat: skip Confirm when the checks pass` to always get the pause |
| `plan_free_dual_arm failed: birrt_failed` (both ends collision-free, no path found) | read the `[... diag]` lines after it: joint limits of start and goal, how many nearby configurations collide at each end (and with what), where the straight joint path first collides (drawn; the red robot stands there), and a `VERDICT` line. A tight pocket at the GOAL: drive the base to give the arms room, IK check, plan again. Otherwise plan again |
| no Cindy topics / `Unknown topic` | `eno1` on `192.168.0.x` and Cindy pings (0.4), domain 86 + CycloneDDS in this pane (1.1 / 1.2), Cindy's launch still running with domain 86 + CycloneDDS (0.3) |
| no tool status (section 2) / grasp steps get no answer | Cindy's launch is missing `gripper:=scaffolding_v3` (0.3) |
| `mocap client connected: False` | `CLIENT_IP` is not this PC's `eno1` address (0.1), or the Motive PC is no longer `192.168.0.28` (`doc/calibration_manual.md` 1.4) |
| marked an entry done by mistake | load it, `Reopen entry (reload clean)` |
| progress is wrong beyond repair | `Reset schedule to the Rhino export` (section 11) |

## 13. What to send back

- The monitor log (`~/husky_real/monitor.log`, or copy the terminal).
- `~/Insync/2025-03 Husky Assembly/data_design_study/261006_3bar_holding_test/progress.json` after the run.
- Per jointing entry: the `[transfer workaround] ... max |goal - stored goal|` line, the
  `[tool0 check] ... minus target (Rhino)` line from `Draw tool0: ...` after the transfer, whether the
  mounted bar sat in the tools at the insertion start, and how the insert finished (stall time,
  any rigid / compliant warnings).
- Per release entry: the `[..._LM_retreat] check ...` lines (which passed, the numbers of any FAIL),
  and whether the arms backed off cleanly.
- Entry 0: how B1's rigid insert ended (the `grounded bar` lines; the bar's final fit before fixing
  the foundation).
