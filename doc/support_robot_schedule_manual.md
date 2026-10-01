# Support robots in the live monitor — operator manual

How to run an assembly that uses the support robots (Alice, Belle) from the live
monitor, one `ActionSchedule.json` entry at a time. Written for the first test:
**bars B1, B3 (held by Alice), B4, B5 of `260920_RobArch_demo_revamp`**
(schedule entries 0..8).

Design notes and the full change log: `tasks/2026-09-30_support_robot_schedule_monitor.md`.

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
   | `BAR_ACTION_MOCAP_ACCURACY_TEST` | see below | 1 hides the built bars from collision checks |
   | `FAKE_HARDWARE` | 0 on robots, 1 for a dry run | |

   ! `BAR_ACTION_MOCAP_ACCURACY_TEST`: the exported hold scenes (`B*__H.json`)
   show the bars that exist when the hold ENDS (B4..B9 for B3's hold) while Cindy
   stands at her B3 assembled pose, which overlaps future bar B5 — so Alice's
   approach plans report "start in collision" unless the built bars are hidden
   (flag = 1). This was a deliberate choice to keep the exported scene; Cindy's
   J/R entries plan fine with the flag at 0 as well.
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
| header line | connected robot, problem, `k/48 done` |
| `Schedule entry (idx)` + `Prev entry` / `Next entry` | pick an entry (the readout under it shows type, bar, robot, IK/TRJ counts, status) |
| `Load entry` | loads the entry. Another robot's entry loads **display only**: you see its movements, but Plan / Exec / step buttons refuse |
| `Mark entry done` | records the entry as done and remembers where the robot ended (see below) |
| `Reopen entry (reload clean)` | sets the LOADED entry back to pending, restores that robot's end state from its previous done entry, and reloads the clean export. If the slider shows a different entry it first asks for `Confirm Exec`, naming both |
| `Rescan schedule status` | re-reads `progress.json` and the IK/trajectory flags |
| `now` line | `entry k -- <type> <bar> by <robot> -- movement i/N <id> [<kind>] ctrl=<controller>` |
| row list | grey = other robot, green = done, yellow = selected |

Under `Exec Selected Mv Traj (auto)` there is ONE step button whose label depends
on the loaded movement:

| movement | button | what happens |
|---|---|---|
| manual (J_M1 mount bar) | `Operator done (manual step) -> then Confirm Exec` | waits for `Confirm Exec` |
| scaffolding grasp (J_M2) | `Tool step: grasp` | after `Confirm Exec`: STOP, gripper motors TIGHTEN until STALLED (max 30 s), STOP |
| scaffolding tighten (J_M4), ungrasp (R_M1) | `Mark tool step done (...)` | nothing sent — the compliant insert / retreat sends them |
| scaffolding untighten (R_M0) | `Mark tool step done (untighten: ...)` | nothing sent (the joint motor is never reversed); use `Loosen Joint` by hand if needed |
| gripper open (H_M1, HR_M0) | `Exec gripper step: OPEN` | after `Confirm Exec`: gripper opens |
| gripper close (H_M3) | `Exec gripper step: CLOSE + compliant handoff` | see below |

Arm movements keep `Plan Movement` → `Move Arms to Movement Start` →
`Exec Selected Mv Traj (auto)`. The traj button refuses tool / manual steps.

**Only one thing waits at a time.** While a step or an execution is running or
waiting for `Confirm Exec`, `Load entry`, `Mark entry done`, `Reopen`, `Rescan`,
another step, and (in schedule mode) Plan / Exec / Load Movement / Reset are
refused. Finish it or click `Cancel Exec`.

### The grip handoff (H_M3, Alice)

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
| 86 Cindy | 0 `B1_J` | Load entry. Movement 3 (J_M3) → `M1: Confirm manual start pose (IK check)` (gives J_M0 its goal) · J_M0 Plan + Exec · J_M1 step (mount bar) · J_M2 step (grasp) · J_M3 Plan + Exec · J_M4 step (mark) · J_M5 Plan/Load traj + Exec (compliant insert) · **Mark entry done** |
| 86 | 1 `B1_R` | R_M0 step (mark) · R_M1 step (mark) · R_M2 Plan + Exec (compliant retreat) · R_M3 Plan + Exec (home) · Mark done |
| 86 | 2 `B3_J` | as entry 0 |
| 86 | 3 `B3_H` | grey (Alice's). **Quit.** |
| 84 Alice | 3 `B3_H` | H_M0 Plan + Exec · H_M1 step (open) · H_M2 Plan + Exec (linear approach) · H_M3 step (close + handoff) · Mark done · **Quit.** |
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
and calibrated) → `progress.json` → the exported file. A support robot whose hold
was released (its HR entry done) is parked out of the scene.

Planned trajectories are saved to `<bar>__<kind>.live-solved.json` next to the
clean export (never overwritten). `Load entry` prefers that file; after `Reopen`
the clean export is used for the rest of the run. Movement 0 of J and H entries
always starts from the live arm.

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
