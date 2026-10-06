# Cindy two-bar test (B1, B3) — with the real robot

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
> start (the Rhino pose) from where the base stands; the `Transfer start:` sliders are not used.
> Order of a jointing entry:
> **transfer** (empty tools, from the live arms to the insertion start) → **manual mount** (mount
> the bar by hand there) → **tool grasp** → **insert** (→ **fix foundation** for B1). The travel
> to load is skipped.

Every code block below is meant to be pasted as is. Tick the boxes as you go; anything that does
not match "expect" is worth a note (log lines + what you did). What the buttons do in detail:
`doc/support_robot_schedule_manual.md`.

---

## 0. Once, before the session

### 0.1 Monitor flags

The switches are class attributes at the top of `class HuskyMonitor`
(`husky_assembly_teleop/husky_monitor.py`; no rebuild needed after editing, restart the monitor).

```bash
grep -nE "^    (FAKE_HARDWARE|USE_MOCAP|USE_CELL_STATE_BASE_POSE|CONNECT_COMPLIANT_CONTROLLER|LIST_CONTROLLER_SERVICES|IGNORE_BUILT_ASSEMBLY_COLLISIONS|TRANSFER_AS_FREE_MOVE) *=" \
    ~/Code/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_monitor.py
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

### 0.2 The problem folder (shared Insync folder)

The monitor reads and writes the design problem directly in the shared Insync folder:
`Mark entry done` writes `progress.json` and `*.live-solved.json` (saved plans) there, and they sync
to the shared drive. Check that the folder is complete and see what an earlier run left behind:

```bash
GD="$HOME/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly"
P="$GD/data_design_study/261006_3bar_holding_test"
ls "$P"                                        # ActionSchedule.json BarActions RobotCell*.json WalkableGround.json ...
ls "$P"/BarActions/*__*.json | grep -vc live-solved     # 48 action files
ls -a "$P" | grep -c insyncdl                  # 0 = Insync has finished syncing
ls "$P"/progress.json "$P"/BarActions/*.live-solved.json 2>/dev/null   # left over from an earlier run?
```

If `progress.json` or saved plans are left over and you want to start fresh, use
`Reset schedule to the Rhino export` once the monitor is up (section 11): it moves them to
`$P/archive/<time>/` (nothing is deleted).
Only after code changes: `cd ~/Code/ros2_ws && source venv/bin/activate && python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop`.

### 0.3 Cindy bring-up (robot side)

- [ ] Teach pendants of both arms in **external control** mode (`doc/husky_ros2.md`; do not start
      `ros_control.urp` by hand).
- [ ] On Cindy's PC (`ssh administrator@192.168.0.115`, login in `doc/husky_ros2.md`): her ROS runs
      **CycloneDDS on domain 86** (`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=86`
      before launching) — without it the workstation sees none of her topics.
- [ ] Arms + scaffolding tools launched the way you normally do (`doc/husky_ros2.md`:
      `ros2 launch crl-husky crl_dual_ur5e.launch.py namespace:='/a200_0806'`). Section 2 checks it.
- [ ] Motive streams Cindy's rigid body **1860**; the workstation's USB-ethernet dongle is on the
      robot network (`ip -brief addr` shows a `192.168.0.x` address on an `enx...` adapter).

---

## 1. Terminals

Use one tmux session with two panes: the **monitor** pane and a **checks / base** pane.

```bash
tmux new -s husky_real        # then Ctrl-b "  splits it; Ctrl-b o switches pane
```

### 1.1 Monitor pane — paste once per new terminal

```bash
cd ~/Code/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=86
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
IFACE=$(ip -o -4 addr show | awk '$4 ~ /^192\.168\.0\./ {print $2; exit}')
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$IFACE\"/></Interfaces></General></Domain></CycloneDDS>"
GD="$HOME/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly"
export DESIGN_DATA_DIRECTORY="$GD/data_design_study"
export DESIGN_PROBLEM_NAME=261006_3bar_holding_test
export EXPERIMENT_DATA_DIRECTORY="$GD/data_experiment"
export HUSKY_IK_BACKEND=gradient     # ssik is not installed in this venv
ros2 daemon stop >/dev/null 2>&1
echo "[cindy env] domain $ROS_DOMAIN_ID via ${IFACE:-NO ADAPTER ON 192.168.0.x} | problem $DESIGN_DATA_DIRECTORY/$DESIGN_PROBLEM_NAME"
```

- The adapter is found by its `192.168.0.x` address, so any USB-ethernet dongle works (no
  `~/.cyclonedds.xml` edit). `NO ADAPTER ON 192.168.0.x` = dongle not plugged in / no address.
- The data paths matter: the package's built-in defaults point to another PC (`/home/su/...`).
- Keep a log of the monitor (from any other pane, once per session):
  `mkdir -p ~/husky_real && tmux pipe-pane -t husky_real:0.0 -o 'cat >> ~/husky_real/monitor.log'`.

### 1.2 Checks / base pane — paste once per new terminal

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=86
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
IFACE=$(ip -o -4 addr show | awk '$4 ~ /^192\.168\.0\./ {print $2; exit}')
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$IFACE\"/></Interfaces></General></Domain></CycloneDDS>"
ros2 daemon stop >/dev/null 2>&1
echo "[robot env] domain $ROS_DOMAIN_ID via ${IFACE:-NO ADAPTER ON 192.168.0.x}"
ros2 topic info /a200_0806/cmd_vel --no-daemon --spin-time 5   # expect: Subscription count: 1 (twist_mux)
```

`Unknown topic` = Cindy is not visible: wrong domain, her ROS is not on CycloneDDS, or the base
driver is down.

---

## 2. Connectivity checks (checks pane, after 1.2)

The controller check uses the controller manager's service directly (`ros2 control` is not
installed on this workstation).

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
  **`Switch to Joint (BOTH)`**, or from the checks pane:

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
      `mocap client connected: True`.
- [ ] Panel: a **Schedule** section; `Ignore built-bar collisions (bars drawn faint)` unticked; the
      slider above `Load Movement` is called `Step`.
- [ ] PyBullet: Cindy drawn at her real base (mocap) with her real arm poses
      (`Toggle Joint Live Stream` shows 12 joints following the arms).
- [ ] Tick `Workaround: plan the transfer as a free move (no bar, from the live arms)` (under
      `Plan Movement`).

---

## 6. Entry 0 — B1 jointing (ground bar)

Select entry 0 → `Load entry`. Six steps: travel to load (skipped), manual mount, tool grasp,
transfer, insert, fix foundation (manual). B1 has no tighten step.

- [ ] **Transfer** (`B1_J_M3_CDFM_transfer_to_approach`): `Step` → 3 (readout
      `step 4/6: B1_J_M3_CDFM_transfer_to_approach ...`) → `Load Movement`.
  - [ ] **IK check** (workaround box ticked): `Transfer start: Confirm manual pose (IK check)` →
        `[transfer workaround] insertion start reachable from the live base (blue ghost); max |goal - stored goal| = ... rad`.
        The blue ghost shows the arms at the insertion start. Nothing moves and nothing is staged
        for `Exec` yet.
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
- [ ] **Retreat** (`B1_R_M1_LM_retreat`): `Plan Movement` → preview → `Exec Selected Mv Traj (auto)` →
      log: the ungrasp runs with it (gripper motors LOOSENING), and once
      `'B1_R_M1_LM_retreat': the export asks for the joint_tracking controller; the monitor runs this dual_independent_linear movement under ...` (expected).
      After the compliant part it stops: `[B1_R_M1_LM_retreat] retreat ready: N waypoints over ...s. ...`
      → check the preview → `Confirm Exec` → the arms back off. Joint controller restored afterwards.
- [ ] **Free move home** (`B1_R_M2_free_home`): `Step` → 1 → `Load Movement` → `Plan Movement` →
      preview → `Exec Selected Mv Traj (auto)`.
- [ ] `Mark entry done` → header `2/48`.

## 8. Entry 2 — B3 jointing

`Load entry` (slider on 2). Five steps: travel to load (skipped), manual mount, tool grasp,
transfer, insert (the tighten runs with the insert). Same workflow as entry 0, with these step
numbers:

- [ ] Built B1 and its joints drawn solid; the transfer's preview clears them.
- [ ] **Transfer** (`B3_J_M3_CDFM_transfer_to_approach`): `Step` → 3 (readout `step 4/5: ...`) →
      `Load Movement` → `Transfer start: Confirm manual pose (IK check)` → `Plan Movement` →
      preview → `Exec Selected Mv Traj (auto)`.
- [ ] **Manual mount** (`Step` → 1) → mount B3 → `Confirm Exec`; **tool grasp** (`Step` → 2) → `Confirm Exec`.
- [ ] **Insert** (`B3_J_M5_LM_insert`): `Step` → 4; readout
      `step 5/5: B3_J_M5_LM_insert  (+ B3_J_M4_tool_tighten_joint runs with it)` → `Plan Movement` →
      `Exec Selected Mv Traj (auto)` → log
      `B3_J_M4_tool_tighten_joint ('tighten') runs with this movement: the compliant insert sends it (joint motors TIGHTENING).`;
      rigid approach, compliance for the last mm, the joint motors tighten until they stall, then
      the joint controller is restored.
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
  `$GD/data_design_study/261006_3bar_holding_test/archive/<time>/`.

## 12. If something goes wrong

| situation | do |
|---|---|
| a step waits and you want out | `Cancel Exec` |
| "refused: a step / execution is still running" | finish it or `Cancel Exec`, then retry |
| arm may be left compliant (loud error in the log) | `Switch to Joint (BOTH)` or the service call in section 4 |
| `Exec` refused: arms not at the trajectory start | `Move Arms to Movement Start (offline target)` (refuses joint changes above pi/3), then `Exec` |
| the transfer plans with the bar held / tries the bar-held planner | the workaround box is unticked (it starts unticked after every restart): tick it, plan again |
| `[transfer workaround] ... no collision-free IK for the insertion start` (IK check or plan) | the base is too far from / too close to the Rhino pose, or the drawn colliding pair is in the way: move the base (section 3), IK check again; note it in the log |
| the IK check passes although the base is far from the Rhino pose; it prints `[M1 manual] ...` lines | the workaround box is unticked: the button then checks the bar-loading start next to the robot, not the insertion start. Tick it and check again |
| a plan fails with "start ... in collision" | read the pair in the log; in entry 4 see the Alice fallback in section 10 |
| `not planning: joint value(s) outside the URDF limits` (right away, no search) | the start or goal has a joint the arm cannot reach (the UR5e elbow turns only +/- pi); the message names it. Should not happen for the transfer any more (its goal keeps every joint in range): note the line in the log |
| `plan_free_dual_arm failed: birrt_failed` (both ends collision-free, no path found) | read the `[... diag]` lines after it: joint limits of start and goal, how many nearby configurations collide at each end (and with what), where the straight joint path first collides (drawn; the red robot stands there), and a `VERDICT` line. A tight pocket at the GOAL: drive the base to give the arms room, IK check, plan again. Otherwise plan again |
| no Cindy topics / `Unknown topic` | dongle (`ip -brief addr`), domain 86, Cindy's ROS on CycloneDDS (0.3) |
| marked an entry done by mistake | load it, `Reopen entry (reload clean)` |
| progress is wrong beyond repair | `Reset schedule to the Rhino export` (section 11) |

## 13. What to send back

- The monitor log (`~/husky_real/monitor.log`, or copy the terminal).
- `$GD/data_design_study/261006_3bar_holding_test/progress.json` after the run.
- Per jointing entry: the `[transfer workaround] ... max |goal - stored goal|` line, whether the
  mounted bar sat in the tools at the insertion start, and how the insert finished (stall time,
  any rigid / compliant warnings).
- Per release entry: the `retreat ready` line and whether the arms backed off cleanly.
- Entry 0: how B1's rigid insert ended (the `grounded bar` lines; the bar's final fit before fixing
  the foundation).
