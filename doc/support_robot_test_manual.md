# Testing the support-robot schedule monitor

A step-by-step test plan for the 2026-09-30 changes (ActionSchedule panel, Alice's
hold flow, the gripper → compliant handoff, obstacle robots from `progress.json`) and
the 2026-10-01 dry-run fixes (operator steps instead of movements, one click for the
transfer start, the built-bar switch, state carried over between runs).

- **Part A — without robots** (`FAKE_HARDWARE=1`): ~1 h at the workstation.
- **Part B — with the robots**: staged checks on Alice first, then the four-bar run.

What the buttons do is explained in `doc/support_robot_schedule_manual.md`; this
document only says what to click and **what you should see**. Tick the boxes as you
go; anything that does not match "Expect" is worth a note (log lines + what you did).

> **! Transfer workaround in use (since 2026-10-06, until the bar-held transfer planner is ready).**
> The bar-held transfer planner rarely finds a path, so every J entry runs the transfer as a
> plain free move **without the bar**: tick
> `Workaround: plan the transfer as a free move (no bar, from the live arms)` (under
> `Plan Movement`) once per monitor run. Order of a J entry: **transfer** (empty tools, from the
> live arms to the insertion start) → **manual mount** (mount the bar by hand there) →
> **tool grasp** → **insert**. The travel to load is skipped. The flow without the box is kept
> below under "Later, without the workaround".

---

## 0. Before both parts

```bash
cd ~/Code/ros2_ws && source venv/bin/activate
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash
```

The monitor's switches are class attributes at the top of `class HuskyMonitor` in
`husky_assembly_teleop/husky_monitor.py` (≈ lines 270–390). Editing them needs no
rebuild (symlink install) — just restart the monitor. **Write down the values you
change and put them back afterwards.**

Paths below use this workstation's Insync folder. On another PC, replace them.

```bash
GD="$HOME/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly"
```

---

# Part A — without robots (FAKE_HARDWARE)

## A1. CycloneDDS setup: loopback for the dry run

**Why.** `~/.bashrc` sets, for the robots:

```bash
export ROS_DOMAIN_ID=86
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml     # pins the USB-ethernet adapter enx34298f73396f
```

With that adapter unplugged the monitor dies at start-up with
`enx34298f73396f: does not match an available interface` /
`rcl node's rmw handle is invalid`. The dry run therefore uses the repo's loopback-only
config `config/cyclonedds_localhost.xml`: it needs no network at all, and nothing leaves
this machine, so a real robot can never receive a command from the dry run.

**Steps** (per terminal; your `~/.bashrc` is not changed):

1. Open a terminal (or the shared tmux session, A1b).
2. `source ~/Code/ros2_ws/src/husky-assembly-teleop/config/dryrun_env.sh 86`
   (`86` = Cindy, `84` = Alice). It activates the venv, sources the workspace, and sets
   `CYCLONEDDS_URI` to the loopback config, the dry-run problem folder (A2), the
   calibration folder and `HUSKY_IK_BACKEND=gradient` (ssik is not installed in this venv),
   then stops the old `ros2` CLI daemon. It prints
   `[dryrun] ROS_DOMAIN_ID=86 (loopback only) | problem .../260920_dryrun`.
3. Check: `echo $CYCLONEDDS_URI` ends in `config/cyclonedds_localhost.xml`.
4. Start the monitor (A3). In a second terminal with step 2 done:
   `ros2 topic info /a200_0806/left_ur5e/rate_limiter/joint_states --no-daemon` →
   **Publisher count: 0, Subscription count: 1** (the monitor listens, no robot talks).

To switch robot later: quit the monitor, `export ROS_DOMAIN_ID=84` (or source the script
again with `84`), start it again.

- [ ] In Part A, `Move Arms to Movement Start (offline target)` moves the **simulated**
      arms to the loaded movement's start (log `Moving arms to '<movement id>' start ...`;
      nothing is published; it refuses a joint change above pi/3 rad).
      `Exec Selected Mv Traj (auto)` moves the simulated arms from the trajectory's
      first waypoint.

### A1b. Shared terminal with Claude (optional)

Claude's own shell cannot open the PyBullet / panel windows, but it can type into and
read a tmux session that **you** start:

```bash
tmux new -s husky          # in your own terminal; detach with Ctrl-b d, re-attach: tmux attach -t husky
```

Start the monitor there **without a pipe** (no `| tee`): with its output piped, the
monitor's printed lines stop reaching the terminal partway through start-up. To keep a
log of the session use `tmux pipe-pane -o 'cat >> ~/husky_dryrun/monitor_pane.log'`.

Everything in that session runs as you, on your display. Claude sends commands with
`tmux send-keys -t husky ...` and reads the screen with `tmux capture-pane -t husky -p`;
you see every keystroke and can take over at any time.

## A2. A scratch copy of the problem (so the real folder stays clean)

`Mark entry done` writes `progress.json` and `*.live-solved.json` next to the
schedule. Use a copy outside Insync so nothing syncs to the shared drive:

```bash
SRC="$GD/data_design_study/260920_RobArch_demo_revamp_backup"
DRY="$HOME/husky_dryrun/design/260920_dryrun"
mkdir -p "$DRY"
cp "$SRC/ActionSchedule.json" "$SRC/WalkableGround.json" "$DRY/"
cp -r "$SRC/BarActions" "$DRY/"
rm -f "$DRY"/progress.json "$DRY"/BarActions/*.live-solved.json
ln -sf "$SRC"/RobotCell*.json "$DRY/"        # 3 x 340 MB, read only -> symlinks
```

Environment for every terminal in Part A — one line (see A1):

```bash
source ~/Code/ros2_ws/src/husky-assembly-teleop/config/dryrun_env.sh 86
```

Monitor flags for Part A:

| flag | Part A value |
|---|---|
| `FAKE_HARDWARE` | **1** |
| `USE_MOCAP` | **0** (no mocap PC; bases come from the exported poses) |
| `USE_ACTION_SCHEDULE` | 1 |
| `BAR_ACTION_LIVE_REPLAN_EXE` | 1 |
| `IGNORE_BUILT_ASSEMBLY_COLLISIONS` | 0 (the panel toggle below changes it while the monitor runs) |
| `CONNECT_COMPLIANT_CONTROLLER`, `LIST_CONTROLLER_SERVICES` | 0 |

**Built bars.** The schedule panel's toggle `Ignore built-bar collisions (bars drawn faint)`
replaces "set `BAR_ACTION_MOCAP_ACCURACY_TEST = 1`" for these runs:

- OFF (start value) — the built bars and joints are obstacles for the planner and IK and
  are drawn as usual. Use it for Cindy's entries.
- ON — the planner and IK ignore them and they are drawn faint grey. Alice's exported
  hold scene still needs it ON (Rhino note D5), see A4.
- Changing it reloads the loaded entry (planned paths not yet saved are dropped).

`BAR_ACTION_MOCAP_ACCURACY_TEST` no longer decides collisions in schedule mode; leave it
as it is (1 only adds the accuracy-test buttons and a pink line at the bar's assembled pose).

! Known issue (not fixed yet): a `*.live-solved.json` written while the built bars were
ignored keeps them hidden when it is reloaded, even with the toggle OFF. The sidecars left
in `$DRY/BarActions` by the 2026-10-01 dry run are such files, so **start over (next line)
before this walk-through**; later, use `Reopen entry (reload clean)` or delete the sidecar
before checking the built-bar display.

To start over at any time: quit the monitor, `rm "$DRY/progress.json" "$DRY"/BarActions/*.live-solved.json`.

## A3. Cindy's run (entries 0–2)

```bash
source ~/Code/ros2_ws/src/husky-assembly-teleop/config/dryrun_env.sh 86
ros2 run husky_assembly_teleop husky_monitor
```

**Start-up**
- [ ] Terminal prints `[Schedule] robot Cindy (domain 86) | problem 260920_dryrun | progress 0/48 done`,
      then `[Schedule] others: Alice <- parked | Belle <- parked`, and the 48-entry roster.
      Warnings `Base calibration file not found for robot 0804 / 0805`
      are expected (only Cindy has a file in the current calibration folder).
- [ ] DPG panel: a **Schedule** section (no "BarAction file" slider) with the same two header
      lines, rows `[00]..`; entry `[03] H B3 Alice` is grey with `(other robot)`; the toggle
      `Ignore built-bar collisions (bars drawn faint)` is unticked. Further down, the slider
      above `Load Movement` is called `Step`.
- [ ] PyBullet: Cindy only near the origin — Alice and Belle are parked far away
      (50 m, 50 m), not stacked on Cindy.

**Entry 0 — B1 jointing.** Select 0 → `Load entry`. The `Step` slider walks the 5 operator
steps: travel to load, manual mount, tool grasp, transfer, insert. The tighten has no step of
its own: it runs with the insert.
- [ ] Only bar B1 shows; the `now` line reads `Now: entry 0 -- J B1 by Cindy -- step 1/5 B1_J_M0_free_to_load [dual_free] ctrl=joint_tracking`;
      the readout under the `Step` slider reads `step 1/5: B1_J_M0_free_to_load  (dual_free)`.
- [ ] `traj time` shows the travel to load's default (log `[Movement] traj time -> ...s (default for dual_free)`),
      not 90 s.
- [ ] Tick `Workaround: plan the transfer as a free move (no bar, from the live arms)` (under
      `Plan Movement`; it keeps its tick when the panel is rebuilt). Skip step 1 (travel to load).
- [ ] **Transfer, as a free move without the bar** (`B1_J_M3_CDFM_transfer_to_approach`):
      `Step` → 3 (readout `step 4/5: B1_J_M3_CDFM_transfer_to_approach ...`) → `Load Movement` →
      `Plan Movement` →
      `[transfer workaround] free move (no bar) from the live arms to the insertion start; max |goal - stored goal| = ... rad`
      and `[Plan] 'B1_J_M3_CDFM_transfer_to_approach': N waypoints stored.`
      - [ ] The preview (`Traj viz time`) starts at the simulated arms and ends with the empty
            tools at the insertion start (just short of the bar's assembled pose); **no bar** is
            drawn moving with the tools.
      - [ ] `Exec Selected Mv Traj (auto)` → the simulated arms go there.
  - If it reports `no collision-free IK for the insertion start at the live base ...`, the drawn
    colliding pair says what is in the way; if the free move finds no path, note the log lines —
    planner matters we look at together, not pass/fail items of this checklist.
- [ ] Manual mount (`B1_J_M1_manual_mount_bar`): `Step` → 1 → `Load Movement`. The bar is drawn in
      the tools **at the insertion start** (where the transfer ended — this is where you mount
      it). A new button **`Operator done (manual step) -> then Confirm Exec`** appears. Click it →
      log asks for Confirm.
  - [ ] *One-task rule:* while it waits, click `Mark entry done` and `Load entry` → both
        **refused** ("still running or waiting for 'Confirm Exec'").
  - [ ] `Confirm Exec` → `confirmed done by the operator`.
- [ ] Tool grasp (`B1_J_M2_tool_grasp_bar`): `Step` → 2 → `Load Movement`; the bar is again drawn in
      the tools at the insertion start. Button `Tool step: grasp` → `Confirm Exec`. Without hardware
      no tool status arrives: either wait for `hit the 30s ceiling ... check the grip`, or click
      `Cancel Exec` → `cancelled by the operator; stopping the motors`. Both end with `STOP sent`.
- [ ] Insert: `Step` → 4 → `Load Movement`. The readout reads
      `step 5/5: B1_J_M5_LM_insert  (+ B1_J_M4_tool_tighten_joint runs with it)` and there is
      **no** `Mark tool step done` button. `Plan Movement` → the insert plans from where the
      transfer ended; preview. No collision is reported between a ground joint
      (`joint_G1-T20Ground-..._ground`) and the floor (`obstacle_ground`).
      **Do not Exec in Part A** — the compliant insert needs the real controllers (in fake mode
      it would only publish and wait 30 s for a screw stall).
- [ ] `Mark entry done` → `[Schedule] entry 0 B1_J_joint marked done by Cindy; Cindy's belief <- live.`
      Row 0 turns green, header `1/48`, the slider jumps to 1, and
      `cat "$DRY/progress.json"` shows entry `"0"` = done. A `B1__J.live-solved.json` appears in
      `$DRY/BarActions/` (it holds the trajectories you planned).

*Later, without the workaround (once the bar-held transfer planner is ready; box unticked).* The
order is then travel to load → manual mount → tool grasp → transfer (bar held) → insert, and the
travel to load ends at the bar-loading pose, which you choose with the `Transfer start: ...`
sliders (home anchor, slide along bar, roll about bar, shift perp. 1 / 2; all at 0 is a fine first
try). **`Confirm transfer start + plan travel to load`** adopts that pose
(`[M1 manual] start adopted -> transfer start / travel-to-load goal. ...`) and plans the travel to
load (`... B1_J_M0_free_to_load planned; check the preview, then 'Exec Selected Mv Traj (auto)'.`).
If no arm configuration holds the bar there, it says so and plans nothing;
`Transfer start: Confirm manual pose (IK check)` checks a pose without planning. The see-through
orange bar follows the sliders only while the transfer is loaded.

**Entry 1 — B1 release.** `Load entry`. Two steps: retreat, free move home.
- [ ] It opens on the retreat: readout
      `step 1/2: B1_R_M2_LM_retreat  (+ B1_R_M1_tool_ungrasp_bar runs with it; B1_R_M0_tool_untighten_joint not sent: 'Loosen Joint' by hand if needed)`;
      no `Mark tool step done` button.
- [ ] Built structure visible: bar B1 and its joints are drawn solid (not faint, not missing).
- [ ] Step 1, retreat: `Plan Movement` → ≈ 12 waypoints; **no Exec in Part A** (compliant).
- [ ] Step 2, free move home (`B1_R_M3_free_home`): `Step` → 1 → `Load Movement` → `Plan Movement` → `Exec`.
- [ ] `Mark entry done` → `2/48`.

**Entry 2 — B3 jointing.** `Load entry`.
- [ ] The built B1 and its joints are drawn solid during the travel to load (the planner checks them).
- [ ] Built-bar switch, before planning anything: tick `Ignore built-bar collisions (bars drawn faint)`
      → `[built bars] ignore built-bar collisions: ON -- reloading entry 2`, then
      `[built bars] collisions with N built bodies ignored (drawn faint)`; B1 and its joints turn
      faint grey. Untick it → the entry reloads and they are solid again. Leave it unticked for
      Cindy's entries.
- [ ] Then the same steps as entry 0 (transfer as a free move, manual mount, tool grasp, insert;
      mount and grasp may be skipped in Part A), then `Mark entry done` → `3/48`, slider on 3.

**Entry 3 — Alice's hold, seen from Cindy.** `Load entry`.
- [ ] Warning `Entry 3 belongs to Alice; loaded for display only`; its four movements are listed.
- [ ] `Plan Movement` and `Exec ...` → **refused** ("display only"); no step button is shown.
- [ ] `Mark entry done` → log asks to confirm marking with the EXPORTED end state → click
      `Cancel Exec` → entry 3 stays pending. (In the real run Alice marks it herself.)
- [ ] Quit the monitor (Ctrl-C).

## A4. Alice's run (entry 3)

```bash
source ~/Code/ros2_ws/src/husky-assembly-teleop/config/dryrun_env.sh 84
ros2 run husky_assembly_teleop husky_monitor
```

- [ ] Header `robot Alice (domain 84) ... 3/48 done` and `others: Cindy <- live (entry 2) | Belle <- parked`;
      slider on entry 3, its row not grey.
- [ ] PyBullet: **Cindy is drawn where her simulated arms were when you marked entry 2 done**
      (from `progress.json`; in Part A that is the end of the last movement you executed, since
      the insert is not executed here), Belle parked. Cindy may show twice at the same pose: the
      solid husky and the red obstacle robot — the red one is what the planner checks.
- [ ] Tick `Ignore built-bar collisions (bars drawn faint)` (Alice's exported hold scene needs it,
      Rhino note D5) → `[built bars] ignore built-bar collisions: ON`.
- [ ] `Load entry` → loads `RobotCell_Alice.json` (a few seconds), then
      `[built bars] collisions with N built bodies ignored (drawn faint)`; the built bars are faint.
      The `Transfer start: ...` sliders and buttons and the `Insert: ...` knobs are **hidden**;
      the joint plot has 6 traces. One step per movement: `step 1/4: B3_H_M0_free_to_approach ...`.
- [ ] Step 1, free approach (`B3_H_M0_free_to_approach`): `Plan Movement` →
      `[live goal] B3_H_M0_free_to_approach: base not tracked; planning to the stored target configuration (authored base).`
      (mocap is off), then ≈ 60–70 waypoints (random planner, varies); `Exec`.
- [ ] Step 2 (`B3_H_M1_gripper_open`): `Step` → 1 → `Load Movement` → button `Exec gripper step: OPEN`
      → `Confirm Exec` → in fake mode the warning `no gripper action client (viz-only husky?)`,
      then the step ends.
- [ ] Step 3, linear approach (`B3_H_M2_LM_to_grasp`): `Step` → 2 → `Load Movement` →
      `Plan Movement` → **≈ 22 points** (straight line, ~10 cm); `Exec`.
      If it fails with `start ... in collision: ObstacleRobotCindy <-> env_bar_B5`, check that
      `Ignore built-bar collisions (bars drawn faint)` is ticked.
- [ ] Step 4 (`B3_H_M3_gripper_close`): `Step` → 3 → `Load Movement` → button
      `Exec gripper step: CLOSE + compliant handoff` → `Confirm Exec` → log says
      **plain close** (`FAKE_HARDWARE is 1`), no controller switch. Correct for fake mode.
- [ ] `Mark entry done` → `Alice's belief <- live`, `3` done in `progress.json`, hold `B3` =
      `holding`, and `B3__H.live-solved.json` written. A warning `Mocap has not seen Alice ...;
      its belief stores the base B3_H_hold was authored at` is expected with `USE_MOCAP=0`.
- [ ] Quit.

## A5. Cindy again (entries 4–8)

`source .../config/dryrun_env.sh 86`, start the monitor.

- [ ] Start-up: `[Schedule] Cindy starts from the progress.json state (after entry 2, live).` —
      her simulated arms start where they were when you marked entry 2 done (not at home).
      Header `others: Alice <- live (entry 3) | Belle <- parked`. The built-bar toggle is unticked again.
- [ ] PyBullet: **Alice now stands at her hold pose next to B3** (not parked); like Cindy in A4
      she may show twice (solid husky + red obstacle robot).
- [ ] `Load entry` 4 (B3_R) → terminal line
      `[obstacle robots] posed from: ObstacleRobotAlice <- live, ObstacleRobotBelle <- exported`
      (Alice from what her run recorded — **not** the parked pose the B3__R file contains).
      It opens on the retreat (`step 1/2: B3_R_M2_LM_retreat ...`).
- [ ] Retreat `Plan Movement` → ≈ 12 waypoints with Alice in the scene; step 2 (free move home)
      plan → ≈ 75; Exec the free move home.
- [ ] `Mark entry done`; then entries 5–8 (B4, B5): each J entry as entry 0 (tick the transfer
      workaround again after a restart: transfer as a free move → mount → grasp → insert); plan each
      arm step; Exec all but the insert and the retreat (compliant); mark done → header `9/48`.

## A6. Recovery buttons

- [ ] **Reopen**: move the slider to 8, `Load entry`, then `Reopen entry (reload clean)` →
      `entry 8 ... reopened (pending)`, row 8 not green, trajectories cleared.
- [ ] **Reopen confirm**: slider to 7, `Load entry`, then move the slider to 2 (don't load) and click
      `Reopen` → it asks to confirm reopening the **LOADED entry 7, not the slider's entry 2** →
      `Cancel Exec` → nothing changes. (After `Mark entry done` the slider jumps to the next pending
      entry, so a Reopen right after marking also asks this question — that is intended.)
- [ ] **Reset All Mvs to Clean** → reloads the loaded entry from its clean export (log names the
      `.json` without `.live-solved`).
- [ ] **Rescan schedule status** → roster printed again, matching `progress.json`.
- [ ] **Legacy problem still works**: quit, `export DESIGN_PROBLEM_NAME=260929_phase1_retest` and
      `DESIGN_DATA_DIRECTORY="$GD/data_design_study"`, start → one line
      `ActionSchedule.json lists 180 entries but ... files are missing -- using the legacy BarAction list`
      and the old `BarAction file (idx)` slider. (Don't Mark anything here — it is the real folder.)

**Before Part B:** restore the Part A flags (`FAKE_HARDWARE`, `USE_MOCAP`) and use **fresh
terminals** (so `CYCLONEDDS_URI` is the bashrc robot config again, with the USB-ethernet adapter
plugged in), then `ros2 daemon stop` once. Delete `~/husky_dryrun` when done.

---

# Part B — with the robots

## B0. Safety

- Remote e-stop in hand for every motion; the Husky e-stop does **not** stop the UR5e.
- First run of each new motion at a long `traj time` (slider), and preview with `Traj viz time` first.
- `Cancel Exec` ends a waiting step; if a log line says the arm may still be under the
  compliance controller, press **`Switch to Joint (BOTH)`**, or from a terminal:
  `ros2 control switch_controllers -c /a200_0804/ur5e/controller_manager --deactivate cartesian_compliance_controller --activate scaled_joint_trajectory_controller`.

## B1. Pre-flight

**Data**
- [ ] The real problem folder `260920_RobArch_demo_revamp` is complete. Starting the monitor
      shows the schedule panel; if it prints `... files are missing -- using the legacy BarAction
      list`, the sync is incomplete (use `..._backup` instead).
- [ ] No leftover `progress.json` from earlier tests in that folder (unless you mean to continue).

**Calibration (base mocap ↔ robot)** — read from `<EXPERIMENT_DATA_DIRECTORY>/calibration_data/<CALIBRATION_DATE>/`
(`CALIBRATION_DATE = '20260916'` in `husky_assembly_teleop/__init__.py`).
- [ ] Cindy: `calibrated_transformation_0806_rhino.json` — present in `20260916`.
- [ ] **Alice: missing in `20260916`.** Her newest file is `20260623/calibrated_transformation_0804_rhino.json`.
      Either recalibrate Alice (`doc/calibration_manual.md`), or — only if her base markers have not
      moved since June — copy that file into `20260916/` and validate it in B4.2.
      Without a file, Alice's mocap base is off by her marker offset, and she never appears as a
      live obstacle in Cindy's runs (she falls back to `progress.json`, which is fine).
- [ ] Belle has never been calibrated: keep her out of the mocap volume or ignore her (she is parked
      in all four-bar planning).

**Monitor flags for Part B**

| flag | Cindy's runs | Alice's run |
|---|---|---|
| `FAKE_HARDWARE` | 0 | 0 |
| `USE_MOCAP` | 1 | 1 |
| `USE_CELL_STATE_BASE_POSE` | 0 | 0 |
| `CONNECT_COMPLIANT_CONTROLLER` | **1** | **1** |
| `LIST_CONTROLLER_SERVICES` | **1** | **1** |
| `IGNORE_BUILT_ASSEMBLY_COLLISIONS` (= panel toggle `Ignore built-bar collisions (bars drawn faint)`) | 0 / unticked (built bars collision-checked; R entries were verified to plan with them) | **1** / ticked (required with this export) |

The toggle replaces `BAR_ACTION_MOCAP_ACCURACY_TEST` for collisions in schedule mode (see A2,
including the ! known issue about sidecars).

## B2. Bring-up and connectivity (per robot)

On the robot PC (ssh, see `doc/husky_ros2.md`):

```bash
# Cindy (192.168.0.115, domain 86)
ros2 launch crl_husky crl_dual_ur5e.launch.py namespace:='/a200_0806' gripper:=scaffolding_v3
# Alice (192.168.0.113, domain 84)
ros2 launch crl_husky crl_single_ur5e.launch.py namespace:='/a200_0804' gripper:=robotiq_2F_85
```

On the workstation (`export ROS_DOMAIN_ID=<84|86>`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`):

- [ ] USB-ethernet adapter plugged into the robot switch: `ip -brief link | grep enx34298f73396f`
      shows it `UP`, and `echo $CYCLONEDDS_URI` is `file://$HOME/.cyclonedds.xml` (a fresh
      terminal, not one from Part A).
- [ ] Alice: `ros2 topic hz /a200_0804/ur5e/rate_limiter/joint_states` publishes;
      `ros2 action list | grep gripper_cmd` shows `/a200_0804/gripper/robotiq_gripper_controller/gripper_cmd`;
      `ros2 control list_controllers -c /a200_0804/ur5e/controller_manager` lists
      `scaled_joint_trajectory_controller` **active** and `cartesian_compliance_controller` **inactive**.
- [ ] Cindy: `ros2 topic hz /a200_0806/left_ur5e/rate_limiter/joint_states` and
      `/a200_0806/left_gripper/tool_status` publish.
- [ ] Mocap: Motive streams 1860 (Cindy) and 1840 (Alice). Monitor start-up prints
      `mocap client connected: True`.

## B3. Alice — hardware checks before the assembly (no bar, clear workspace)

Start the monitor as Alice (`ROS_DOMAIN_ID=84`) with the Part B flags (built-bar toggle
ticked). **Do not `Mark entry done` in B3** — these are tests.

**B3.1 Startup**
- [ ] Header `robot Alice (domain 84)`; Joint Live Stream (`Toggle Joint Live Stream`) shows 6 joints
      that follow the real arm; the drawn Alice moves with the real base (mocap).
- [ ] A **Gripper** section is present (`Open Gripper Full`, `Close Gripper for Bar`) and the
      `Switch to Compliance (BOTH)` / `Switch to Joint (BOTH)` / `Zero Force Sensor (BOTH)` buttons.

**B3.2 Does the gripper send feedback?** (decides whether the handoff can happen at all)
- [ ] Second terminal: `ros2 topic echo /a200_0804/gripper/robotiq_gripper_controller/gripper_cmd/_action/feedback`
- [ ] `Open Gripper Full`, then `Close Gripper for Bar`.
- [ ] **Note:** do feedback messages stream during the close, with `position` rising towards ~0.8?
      If **no feedback** arrives, the handoff never triggers and every close is stiff (safe, logged
      as "finished before 80 %"). Tell me — the trigger then needs another signal.

**B3.3 Controller switch, by hand** (hand on the e-stop, gripper empty, arm in a safe pose)
- [ ] `Switch to Compliance (BOTH)` → `list_controllers` shows `cartesian_compliance_controller`
      active; the arm should stay where it is (it holds its pose at activation).
- [ ] `Switch to Joint (BOTH)` → back to `scaled_joint_trajectory_controller`.

**B3.4 The handoff in the air** (no bar — the jaws close on nothing)
- [ ] `Open Gripper Full`. `Load entry` 3, `Step` slider → 3 (readout
      `step 4/4: B3_H_M3_gripper_close ...`) → `Load Movement`.
- [ ] Click `Exec gripper step: CLOSE + compliant handoff` → `Confirm Exec`. Expect, in order:
  - `phase 1/4: ensuring scaled_joint_trajectory_controller`
  - F/T sensor zeroed, then `phase 2/4: close goal sent (position 0.43 -> 0.800 ...)`
  - `phase 3/4: gripper at 0.6.. (80% of the stroke); ...` then the switch to `cartesian_compliance_controller`
  - `phase 4/4: arm under cartesian_compliance_controller (target = tool0 pose captured at the switch)`
  - gripper result (`reached_goal`), then the switch back to `scaled_joint_trajectory_controller`.
- [ ] `list_controllers` afterwards: **joint controller active**.
- [ ] **If instead** you see `FK tool0 and the arm-reported TCP disagree by more than 5 cm; no handoff`:
      the single-arm TCP frame does not match the monitor's FK — note the numbers and tell me.
- [ ] Repeat once and press `Cancel Exec` during phase 4 → the hold ends **and the controller
      switches back to joint** (the gripper keeps closing with the arm stiff — logged).

**B3.5 The approach motion in the air** (Cindy away from B3's location, or the area clear)
- [ ] Step 1, free approach (`B3_H_M0_free_to_approach`): `Plan Movement`. With mocap tracking
      Alice's base, its goal is re-solved by IK at the live base first:
      `[live goal] B3_H_M0_free_to_approach: goal re-solved at the live base; max |live IK - stored target| = ... rad`.
      If it says `... no collision-free IK for its approach pose at the live base (...); not planning.`,
      nothing is planned (the collision is drawn if there is one): the base stands where the
      approach pose cannot be reached. If it says `base not tracked`, mocap is not tracking her
      (check B2). Preview, `Exec` at a long `traj time`.
- [ ] **Calibration check:** the free approach ends at the pre-grasp pose ≈ 10 cm in front of where bar B3 will be.
      With the real structure (later), the open jaws must be centred on the bar, 10 cm out. If they
      are off by more than ~1–2 cm, the base calibration (B1) is wrong — stop and fix that first.
- [ ] Step 2 (open), step 3 (linear approach, `B3_H_M2_LM_to_grasp`, it starts from the re-solved
      goal): `Plan Movement` ≈ 22 points, `Exec` → straight 10 cm.
- [ ] Back off: with step 3 loaded, `Move Arms to Movement Start (offline target)` returns the
      arm to the pre-grasp pose (10 cm back). Move her further clear the way you normally do.
- [ ] Quit **without** marking anything (or `Reopen` if you did).

## B4. Cindy — new steps check (optional)

With a bar mounted on Cindy's tools: load entry 0, `Step` → 2 (`B1_J_M2_tool_grasp_bar`) →
`Load Movement` → `Tool step: grasp` →
`Confirm Exec` → `STOP`, gripper motors `TIGHTEN`, `every gripper motor STALLED after N s: bar clamped`,
`STOP sent`. (The same as the manual `L/R Tighten Gripper` buttons, plus the stall wait.)

## B5. The four-bar run (entries 0–8)

Follow the table in `doc/support_robot_schedule_manual.md` ("Four-bar test"). Checkpoints:

**Cindy, entries 0–2** (`ROS_DOMAIN_ID=86`)
- [ ] J entries with the transfer workaround ticked (see the note at the top): the transfer moves
      the **empty tools** from the live arms to the insertion start, right next to the built
      structure. Before `Exec`, check in the preview that the tools stop short of the bar's
      assembled pose and clear the built bars. After it: mount the bar by hand in the tools at that
      pose (`Operator done (manual step)` → `Confirm Exec`), run the tool grasp, then plan the
      insert (it starts where the transfer ended) and run it.
- [ ] Each entry ends with `Mark entry done` → `Cindy's belief <- live`; header count goes up.
- [ ] The insert (`B*_J_M5_LM_insert`) and the retreat (`B*_R_M2_LM_retreat`) run the compliant
      flow; afterwards the log says the joint controller is restored. At `Exec` the log names the
      tool steps that run with them, e.g.
      `B1_J_M4_tool_tighten_joint ('tighten') runs with this movement: the compliant insert sends it (joint motors TIGHTENING).`
      (retreat: the ungrasp → gripper motors LOOSENING; the untighten → nothing sent). The
      retreat also warns once that the export asks for `joint_tracking` — expected.
- [ ] After entry 2: Cindy is holding B3 at the assembled pose, **under the joint controller**
      (check `ros2 control list_controllers -c /a200_0806/left_ur5e/controller_manager` and `right_ur5e`).
      Then quit the monitor — Cindy keeps holding (her driver keeps running).

**Alice, entry 3** (`ROS_DOMAIN_ID=84`)
- [ ] PyBullet: Cindy drawn at her real pose holding B3 (live mocap base + her recorded joints).
- [ ] Built-bar toggle ticked (hold scene).
- [ ] Free approach (`[live goal]` line as in B3.5) → **calibration check from B3.5** (jaws centred
      on B3, 10 cm out) before going on.
- [ ] Gripper open → linear approach → gripper close + handoff (jaws around B3, the arm yields
      briefly while the jaws finish, then stiff again).
- [ ] `Mark entry done` → `Alice's belief <- live`. Quit. Alice stays clamped on B3.

**Cindy, entries 4–8**
- [ ] `Load entry` 4 → `[obstacle robots] posed from: ObstacleRobotAlice <- live ...` and Alice drawn
      on B3. The entry opens on the retreat (compliant; the ungrasp runs with it, the untighten is
      not sent — `Loosen Joint` by hand if needed), then the free move home; both plan and run with
      Alice in the scene.
- [ ] Entries 5–8 (B4, B5) as entries 0/1. B3 stays held by Alice throughout (her release, entry 16,
      is after B9 — out of scope).

## B6. If something goes wrong

| situation | do |
|---|---|
| a step waits and you want out | `Cancel Exec` |
| "refused: a step / execution is still running" | finish it or `Cancel Exec`, then retry |
| arm may be left compliant (loud error in the log) | `Switch to Joint (BOTH)` or the `ros2 control switch_controllers` line in B0 |
| marked an entry done by mistake | load it, `Reopen entry (reload clean)` (the robot's previous end state is restored) |
| progress is wrong beyond repair | quit, edit or delete `<problem>/progress.json` (all entries pending again) |
| a plan fails with "start ... in collision" | read the pair in the log; for `ObstacleRobotCindy <-> env_bar_*` in Alice's run check that `Ignore built-bar collisions (bars drawn faint)` is ticked |
| built bars missing although the toggle is OFF | a sidecar written while they were ignored (! known issue, A2): `Reopen entry (reload clean)` |
| the transfer plans with the bar held / tries the bar-held planner | the workaround box is unticked (it starts unticked after every restart): tick `Workaround: plan the transfer as a free move ...` and plan again |
| `[transfer workaround] ... no collision-free IK for the insertion start` | read the drawn colliding pair; note it and send the log |

## B7. What to send back

- The terminal log of each run (copy the whole thing).
- `progress.json` after the run.
- From B3.2: did gripper feedback arrive, and the positions you saw.
- From B3.4: the phase lines, any "disagree by more than 5 cm" warning, and how the arm behaved while compliant.
- From B3.5 / B5: the free-approach calibration check (how far off the jaws were) and its `[live goal]` line.
