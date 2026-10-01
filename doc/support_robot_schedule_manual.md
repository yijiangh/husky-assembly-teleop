# Support robots in the live monitor — operator manual

How to run an assembly that uses the support robots (Alice, Belle) from the live
monitor, one `ActionSchedule.json` entry at a time. Written for the first test:
**bars B1, B3 (held by Alice), B4, B5 of `260920_RobArch_demo_revamp`**
(schedule entries 0..8).

Design notes and the full change log: `tasks/2026-09-30_support_robot_schedule_monitor.md`;
the 2026-10-01 dry-run fixes: `tasks/2026-10-01_drop_legacy_movement_roles.md` (§5).

---

## What changed, in one paragraph

The monitor now reads the problem's `ActionSchedule.json` and steps through its
entries: `J` (Cindy joints a bar), `H` (a support robot grabs a bar), `R` (Cindy
releases), `HR` (the support robot releases). **One robot is connected per
monitor run** — the one whose `ROS_DOMAIN_ID` you start with (84 Alice,
85 Belle, 86 Cindy). The other two are drawn and collision-checked from their
**live mocap base** (when tracked and calibrated) or from what the monitor
remembers they did last (`progress.json`). To run another robot's entry you quit
and restart the monitor with that robot's domain; progress survives the restart.

---

## Pre-flight

1. **Data.** The problem folder must hold `ActionSchedule.json`, `RobotCell.json`,
   `RobotCell_Alice.json`, `RobotCell_Belle.json`, `WalkableGround.json` and
   **every** file the schedule lists under `BarActions/`. If any is missing the
   monitor falls back to the old BarAction list and says why in one line.
   Point the monitor at it (no code edit needed):
   ```bash
   export DESIGN_DATA_DIRECTORY="<...>/2025-03 Husky Assembly/data_design_study"
   export DESIGN_PROBLEM_NAME=260920_RobArch_demo_revamp
   ```
2. **Monitor flags** (class attributes at the top of `HuskyMonitor` in
   `husky_monitor.py`):

   | flag | set to | why |
   |---|---|---|
   | `USE_ACTION_SCHEDULE` | 1 | schedule panel instead of the BarAction file list |
   | `CONNECT_COMPLIANT_CONTROLLER` | 1 | Cindy's compliant insert/retreat AND Alice's grip handoff need the switch service |
   | `LIST_CONTROLLER_SERVICES` | 1 | reads which controller is active at start-up; with 0 the first switch can fail |
   | `IGNORE_BUILT_ASSEMBLY_COLLISIONS` | 0 for Cindy, 1 for Alice's hold | start value of the panel toggle below |
   | `FAKE_HARDWARE` | 0 on robots, 1 for a dry run | |

   **Built bars: the panel toggle `Ignore built-bar collisions (bars drawn faint)`**
   (Schedule section). OFF: the built bars and joints are obstacles for the
   planner and IK and are drawn as usual. ON: the planner and IK ignore them and
   they are drawn faint grey, so you can see that the switch is on. Changing it
   reloads the loaded entry (planned paths not yet saved are dropped). It
   replaces `BAR_ACTION_MOCAP_ACCURACY_TEST = 1`, which schedule mode no longer
   reads for collisions (the legacy BarAction list still does).

   ! Alice's hold entries need it ON: the exported hold scenes (`B*__H.json`)
   show the bars that exist when the hold ENDS (B4..B9 for B3's hold) while Cindy
   stands at her B3 assembled pose, which overlaps future bar B5 — so Alice's
   approach plans report "start in collision" otherwise (Rhino note D5). Cindy's
   J/R entries plan fine with it OFF.

   ! Known issue (not fixed yet): a `.live-solved.json` sidecar written while the
   built bars were ignored keeps them hidden when it is reloaded, even with the
   toggle OFF. Use `Reopen entry (reload clean)`, or delete that sidecar, before
   relying on the built-bar display.
3. **Base calibration files.** Each robot's
   `calibrated_transformation_<0804|0805|0806>.json` must exist in the
   `CALIBRATION_DATE` folder. A robot without one never gets a live obstacle
   pose (it falls back to `progress.json`).
4. **Mocap.** All three huskies stream (ids 1840 Alice, 1850 Belle, 1860 Cindy).

---

## Starting a run

On the robot (see `doc/husky_ros2.md` for the full bring-up):

```bash
# Cindy (domain 86) -- scaffolding_v3 starts the tool driver the grasp / tighten steps talk to
# (the bar-holding accuracy test used gripper:=none, see doc/bar_holding_acc_manual.md)
ros2 launch crl_husky crl_dual_ur5e.launch.py namespace:='/a200_0806' gripper:=scaffolding_v3
# Alice (domain 84) -- UR5e + Robotiq 2F-85
ros2 launch crl_husky crl_single_ur5e.launch.py namespace:='/a200_0804' gripper:=robotiq_2F_85
```

On the workstation:

```bash
cd ~/Code/ros2_ws && source venv/bin/activate && source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=86          # 86 = Cindy, 84 = Alice, 85 = Belle
ros2 run husky_assembly_teleop husky_monitor
```

---

## The Schedule panel

| widget | what it does |
|---|---|
| header line | connected robot, problem, `k/48 done`; a second line `others: Alice <- live (entry 3) \| Belle <- parked` says where each other robot's pose comes from (`live`, `assumed`, `exported` or `parked`; a fresh mocap base still overrides it) |
| `Schedule entry (idx)` + `Prev entry` / `Next entry` | pick an entry (the readout under it shows type, bar, robot, IK/TRJ counts, status) |
| `Load entry` | loads the entry. Another robot's entry loads **display only**: you see its movements, but Plan / Exec refuse and no step button is shown |
| `Mark entry done` | records the entry as done and remembers where the robot ended (see below) |
| `Reopen entry (reload clean)` | sets the LOADED entry back to pending, restores that robot's end state from its previous done entry, and reloads the clean export. If the slider shows a different entry it first asks for `Confirm Exec`, naming both |
| `Rescan schedule status` | re-reads `progress.json` and the IK/trajectory flags |
| `Ignore built-bar collisions (bars drawn faint)` | OFF = built bars are obstacles; ON = ignored by planner and IK, drawn faint (see Pre-flight 2). Changing it reloads the loaded entry |
| `now` line | `Now: entry k -- <type> <bar> by <robot> -- step i/n <movement id> [<kind>] ctrl=<controller>` |
| row list | grey = other robot, green = done, yellow = selected |

### Steps, not movements

The `Step` slider (above `Load Movement`) walks the entry's **operator steps**, not
its movements. A scaffolding tool step that the next compliant movement sends by
itself has no step of its own: it runs with that movement.

| entry | steps |
|---|---|
| J (e.g. `B1_J`) | 5: travel to load · manual mount · tool grasp · transfer · insert (+ the tighten runs with it) |
| R (e.g. `B1_R`) | 2: retreat (+ the ungrasp runs with it; the untighten is **not sent** — `Loosen Joint` by hand if needed) · free move home. The entry opens on its retreat |
| H, HR (support robot) | one step per movement |

The readout under the slider names the step and what runs with it, e.g.
`step 5/5: B1_J_M5_LM_insert  (+ B1_J_M4_tool_tighten_joint runs with it)`. At
`Exec` the log has one line per tool step that runs with the movement.

Under `Exec Selected Mv Traj (auto)` there is ONE step button whose label depends
on the loaded step:

| step | button | what happens |
|---|---|---|
| manual mount (`B1_J_M1_manual_mount_bar`) | `Operator done (manual step) -> then Confirm Exec` | waits for `Confirm Exec` |
| tool grasp (`B1_J_M2_tool_grasp_bar`) | `Tool step: grasp` | after `Confirm Exec`: STOP, gripper motors TIGHTEN until STALLED (max 30 s), STOP |
| gripper open (`B3_H_M1_gripper_open`, `B3_HR_M0_gripper_open`) | `Exec gripper step: OPEN` | after `Confirm Exec`: gripper opens |
| gripper close (`B3_H_M3_gripper_close`) | `Exec gripper step: CLOSE + compliant handoff` | see below |

The tighten, ungrasp and untighten have no step and no button (see above).

Arm movements keep `Plan Movement` → `Move Arms to Movement Start` →
`Exec Selected Mv Traj (auto)` (with `FAKE_HARDWARE=1`, `Move Arms to Movement
Start` moves the simulated arms). `traj time` starts at the loaded movement's
default. The traj button refuses tool / manual steps.

**Only one thing waits at a time.** While a step or an execution is running or
waiting for `Confirm Exec`, `Load entry`, `Mark entry done`, `Reopen`, `Rescan`,
another step, and (in schedule mode) Plan / Exec / Load Movement / Reset are
refused. Finish it or click `Cancel Exec`.

### The transfer start (J entries)

The travel to load ends where the transfer starts (the bar-loading pose), and the
export leaves that pose open, so it is chosen here:

1. `Load entry` (it opens on step 1, the travel to load);
2. set the `Transfer start: home anchor (0:all,1:horiz,2:vert,3:back)`,
   `Transfer start: slide along bar (m)`, `Transfer start: roll about bar (deg)`,
   `Transfer start: shift perp. 1 (m)` and `Transfer start: shift perp. 2 (m)`
   sliders (a see-through orange bar follows them while the transfer is loaded);
3. **`Confirm transfer start + plan travel to load`**: IK-checks the pose, adopts it
   as the transfer's start and the travel to load's goal, then loads and plans the
   travel to load. If no arm configuration holds the bar there, nothing is planned
   and the transfer stays loaded: adjust and click again;
4. preview (`Traj viz time`), then `Exec Selected Mv Traj (auto)`.

`Transfer start: Confirm manual pose (IK check)` is the same check without
planning, for adjusting the pose. Once the start is adopted, the manual mount and
tool grasp steps show the bar in the tools at the bar-loading pose.
`Transfer start: Derive start/goal only (no RRT)` + `Adopt derived start ->
travel-to-load goal` are the automatic alternative.

### The support robot's free approach

With mocap tracking the support robot's base, `Plan Movement` on its free approach
(e.g. `B3_H_M0_free_to_approach`) first re-solves the goal by IK at the live base
(the exported goal was solved for the authored base) and carries it into the
gripper step and the linear approach after it:
`[live goal] ...: goal re-solved at the live base; max |live IK - stored target| = ... rad`.
When the base is not tracked, the exported goal is used
(`[live goal] ...: base not tracked; ...`). When no collision-free IK exists at the
live base, nothing is planned (`... not planning.`, the collision drawn if there is one).

### The grip handoff (gripper close, Alice)

1. zero the F/T sensor (fingers open, touching nothing);
2. send the close goal, arm stiff (`scaled_joint_trajectory_controller`);
3. when the fingers have travelled 80 % of the way (gripper feedback), capture
   the current tool0 pose, check it agrees with the arm-reported TCP (≤ 5 cm),
   switch to `cartesian_compliance_controller`;
4. publish that CAPTURED pose + zero wrench every tick; end the hold if the arm
   drifts more than 20 mm (`GRIP_HANDOFF_MAX_DRIFT_M`);
5. when the gripper stalls / reaches its goal, switch back to joint tracking —
   always, also on Cancel, timeout or error.

If there is no gripper feedback, no compliance flag or `FAKE_HARDWARE=1`, the
gripper simply closes with the arm stiff (logged).

---

## Four-bar test (entries 0..8)

| run (`ROS_DOMAIN_ID`) | entry | what to do |
|---|---|---|
| 86 Cindy | 0 `B1_J` | Load entry · set the `Transfer start: ...` sliders → `Confirm transfer start + plan travel to load` → preview → Exec (travel to load) · manual mount step · tool grasp step · transfer Plan + Exec · insert Plan/Load traj + Exec (compliant insert; the tighten runs with it) · **Mark entry done** |
| 86 | 1 `B1_R` | opens on the retreat: Plan + Exec (compliant retreat; the ungrasp runs with it, the untighten is not sent) · free move home Plan + Exec · Mark done |
| 86 | 2 `B3_J` | as entry 0 |
| 86 | 3 `B3_H` | grey (Alice's). **Quit.** |
| 84 Alice | 3 `B3_H` | tick `Ignore built-bar collisions (bars drawn faint)` · free approach Plan (goal re-solved at the live base) + Exec · gripper open step · linear approach Plan + Exec · gripper close step (close + handoff) · Mark done · **Quit.** |
| 86 Cindy | 4 `B3_R` | Alice is now an obstacle at her hold pose. As entry 1 |
| 86 | 5..8 | B4 J/R, B5 J/R as entries 0/1 |

Entry 16 (`B3_HR`, Alice lets go) comes after B9 and is out of scope for this test.

---

## What "Mark entry done" remembers

`<problem>/progress.json` (next to `ActionSchedule.json`) keeps each entry's status
and, per robot, the base frame + arm joints it ended in:

- your own robot's entry: the **live** state (mocap base if tracked within 1 s,
  else the authored base; live joints);
- another robot's entry: allowed only after `Confirm Exec`; its **exported** end
  state is recorded as `assumed`.

The other robots' obstacle poses come, in order, from: live mocap (fresh < 0.5 s
and calibrated) → `progress.json` → the exported file. The header's `others:` line
shows which of the last two (or `parked`) each robot uses; a fresh mocap base
still overrides it. A support robot whose hold was released (its HR entry done) is
parked out of the scene. A robot can appear twice at the same pose: the solid
husky and the red obstacle robot — the red one is what the planner checks.

At start-up the connected robot starts from its own `progress.json` state (one
log line, e.g. `[Schedule] Cindy starts from the progress.json state (after entry 2, live).`).
With `FAKE_HARDWARE=1` this is where the simulated arms continue from; on the
robot the first joint state message overwrites the arms.

Planned trajectories are saved to `<bar>__<kind>.live-solved.json` next to the
clean export (never overwritten). `Load entry` prefers that file; after `Reopen`
the clean export is used for the rest of the run. The first movement of J and H
entries (the travel to load, the free approach) always starts from the live arm.

---

## Testing

Step-by-step test plan (dry run with `FAKE_HARDWARE`, then the robots):
`doc/support_robot_test_manual.md`.

---

## To confirm on hardware (not testable here)

- Robotiq `gripper_cmd` publishes **feedback** during the stroke (the monitor opens
  to 0.426 and closes to 0.8); without feedback every close is a stiff close.
- `switch_fraction = 0.8` switches before the fingers touch the bar.
- Alice's `arm_tcp_pose` matches the FK tool0 in `base_link` (otherwise the 5 cm
  check skips the handoff every time).
- Compliance holds the bar without sagging; tune `GRIP_HANDOFF_MAX_DRIFT_M`.
- Scaffolding `state_m1` reports STALLED on the grasp.
- All three mocap bodies tracked; Alice's belief vs live base agree when Cindy's
  run starts (both are logged when the obstacles are posed).

## Known limits

- One robot per monitor run (multi-domain networking via Zenoh is a follow-up).
- compas_fab's `forward_kinematics` applies the robot base twice; the monitor works
  around it inside `cfab_session.plan_linear_motion` only (remove once fixed upstream).
- Exporter issues are listed in
  `bar_joint_rhino_design_workflow/docs/support_export_issues_from_monitor.md` (the Dropbox copy of
  the Rhino repo: `~/Insync/yijiang94817@gmail.com/Dropbox/0_Projects/2025_husky_assembly/Code/`)
  (e.g. `B*__R.json` parks the robot that holds the bar — the monitor overrides it
  from `progress.json`).
