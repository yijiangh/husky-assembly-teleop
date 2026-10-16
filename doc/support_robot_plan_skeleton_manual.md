# Support robots in the live monitor — operator manual

How to run an assembly that uses the support robots (Alice, Belle) from the live
monitor, one `ActionSchedule.json` action at a time. Written for the first test:
**bars B1, B3 (held by Alice), B4, B5 of `260920_RobArch_demo_revamp`**
(actions 0..8).

Design notes and the full change log: `tasks/2026-09-30_support_robot_schedule_monitor.md`;
the 2026-10-01 dry-run fixes: `tasks/2026-10-01_drop_legacy_movement_roles.md` (§5).

---

## What changed, in one paragraph

The monitor now reads the problem's `ActionSchedule.json` and steps through its
actions: `J` (Cindy joints a bar), `H` (a support robot grabs a bar), `R` (Cindy
releases), `HR` (the support robot releases). **One robot is connected per
monitor run** — the one whose `ROS_DOMAIN_ID` you start with (84 Alice,
85 Belle, 86 Cindy). The other two are drawn and collision-checked from their
**live mocap base** (when tracked and calibrated) or from what the monitor
remembers they did last (`progress.json`). To run another robot's action you quit
and restart the monitor with that robot's domain; progress survives the restart.

---

## Pre-flight

1. **Data.** The problem folder must hold `ActionSchedule.json`, `RobotCell.json`,
   `RobotCell_Alice.json`, `RobotCell_Belle.json`, `WalkableGround.json` and
   **every** file the plan skeleton lists under `BarActions/`. If any is missing the
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
   | `USE_PLAN_SKELETON` | 1 | plan skeleton panel instead of the BarAction file list |
   | `CONNECT_COMPLIANT_CONTROLLER` | 1 | Cindy's compliant insert/retreat AND Alice's grip handoff need the switch service |
   | `LIST_CONTROLLER_SERVICES` | 1 | reads which controller is active at start-up; with 0 the first switch can fail |
   | `IGNORE_BUILT_ASSEMBLY_COLLISIONS` | 0 for Cindy, 1 for Alice's hold | start value of the panel toggle below |
   | `FAKE_HARDWARE` | 0 on robots, 1 for a dry run | |

   **Built bars: the panel toggle `Ignore built-bar collisions (bars drawn faint)`**
   (Plan skeleton section). OFF: the built bars and joints are obstacles for the
   planner and IK and are drawn as usual. ON: the planner and IK ignore them and
   they are drawn faint grey, so you can see that the switch is on. Changing it
   reloads the loaded action (planned paths not yet saved are dropped). It
   replaces `BAR_ACTION_MOCAP_ACCURACY_TEST = 1`, which plan skeleton mode no longer
   reads for collisions (the legacy BarAction list still does).

   ! Alice's hold actions need it ON: the exported hold scenes (`B*__H.json`)
   show the bars that exist when the hold ENDS (B4..B9 for B3's hold) while Cindy
   stands at her B3 assembled pose, which overlaps future bar B5 — so Alice's
   approach plans report "start in collision" otherwise (Rhino note D5). Cindy's
   J/R actions plan fine with it OFF.

   ! Known issue (not fixed yet): a `.live-solved.json` sidecar written while the
   built bars were ignored keeps them hidden when it is reloaded, even with the
   toggle OFF. Use `Reopen action (reload clean)`, or delete that sidecar, before
   relying on the built-bar display.
3. **Base calibration files.** Each robot's
   `calibrated_transformation_<0804|0805|0806>_rhino.json` must exist in its date
   folder: the robot's entry in `BASE_CALIBRATION_DATE_BY_ROBOT` (Alice:
   `20260623`, Belle: `20261013`, Cindy: `20260916`), else `CALIBRATION_DATE` (both in
   `husky_assembly_teleop/__init__.py`).
   At start-up the monitor prints `Base calibration of <robot> (<serial>): <date>/<file>`
   per robot, or a `Base calibration file not found` warning. A robot without one
   never gets a live obstacle pose (it falls back to `progress.json`), and as the
   connected robot it plans at the marker body instead of its base.
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

**The current action loads by itself** (`AUTO_LOAD_ACTION = 1`, top of `class HuskyMonitor`):

- at start-up, the first pending action of `progress.json` loads once the robot's
  arm joints (and, with mocap, its base) have arrived:
  `[Plan skeleton] auto-load at start-up: action k (<id>, <robot>), the first pending one.`
  Its live start is the real robot. If they have not arrived after 15 s it loads
  anyway with a warning (`no arm joint states ... from <robot> after 15 s ...`):
  check the driver / mocap, then click `Load action`;
- after `Mark action done`, the next pending action loads the same way
  (`auto-load after 'Mark action done': ...`).

It is whatever action the `Action (idx)` slider jumps to, so it can be another
robot's (display only, as with the button). `AUTO_LOAD_ACTION = 0` brings back
the manual `Load action`.

---

## The Plan skeleton panel

| widget | what it does |
|---|---|
| header line | connected robot, problem, `k/48 done`; a second line `others: Alice <- live (action 3) \| Belle <- parked` says where each other robot's pose comes from (`live`, `assumed`, `exported` or `parked`; a fresh mocap base still overrides it) |
| `Action (idx)` + `Prev action` / `Next action` | pick an action (the readout under it shows type, bar, robot, IK/TRJ counts, status) |
| `Load action` | loads the action. Another robot's action loads **display only**: you see its movements, but Plan / Exec refuse and no step button is shown. Usually not needed: the first pending action loads by itself at start-up and after `Mark action done` (see below) |
| `Mark action done` | records the action as done, remembers where the robot ended (see below), then loads the next pending action |
| `Reopen action (reload clean)` | sets the LOADED action back to pending, restores that robot's end state from its previous done action, and reloads the clean export. If the slider shows a different action it first asks for `Confirm Exec`, naming both |
| `Rescan plan skeleton status` | re-reads `progress.json` and the IK/trajectory flags |
| `Reset plan skeleton to the Rhino export` | after `Confirm Exec`: moves `progress.json` and every saved plan (`*.live-solved.json`) to `<problem>/archive/<time>/` (nothing deleted; move them back to undo), then every action is pending, no robot beliefs (others drawn parked), action 0 loads its clean export. A monitor of another robot that is running keeps its old progress in memory: restart it too |
| `Ignore built-bar collisions (bars drawn faint)` | OFF = built bars are obstacles; ON = ignored by planner and IK, drawn faint (see Pre-flight 2). Changing it reloads the loaded action |
| `now` line | `Now: action k -- <type> <bar> by <robot> -- step i/n <movement id> [<kind>] ctrl=<controller>` |
| row list | grey = other robot, green = done, yellow = selected |

### Steps, not movements

The `Step` slider (above `Load Movement`) walks the action's **operator steps**, not
its movements. A scaffolding tool step that the next compliant movement sends by
itself has no step of its own: it runs with that movement.

| action | steps |
|---|---|
| J (e.g. `B1_J`) | 5: travel to load · manual mount · tool grasp · transfer · insert (+ the tighten runs with it). The action opens on its transfer (loaded right away). After the transfer's motion its exec corrects the tool0 placement with the accuracy test's visual servo (see "Visual-servo correction after the transfer" below) and leaves the insert loaded |
| J of a **grounded bar** (held on its ground joints, e.g. `B1`, `B5` in the 2026-10-06 export) | 6: travel to load · manual mount · tool grasp · transfer · insert · manual fix foundation. The insert runs **rigid only** (joint tracking) and **never tightens**, whatever the export says (an exported tighten step is ignored; Rhino note D9) |
| J of a **one-sided bar** (one held male joint has no female in the insert's start state: its female sits on a staging bar; e.g. `B3`, `B7` mate left, `B12`, `B15` mate right in the 2026-10-06 export) | 5, as a normal J. The insert TIGHTENs only the **mating** arm's joint motor and ends when that motor stalls; both arms still run the rigid approach and the compliant last mm. The export's tighten step still names both tools: a warning says so until the Rhino re-export (note D10) |
| R (e.g. `B1_R`) | 2: retreat (+ the ungrasp runs with it; the untighten is **not sent** — `Loosen Joint` by hand if needed) · free move home. The action opens on its retreat. The retreat is planned from the live arms: each tool0 backs off the export's `lm_distance_mm` (50 mm) along its `retreat_axes_world`, scene collisions checked; a failed straight-line plan prints a `[<movement> diag]` block and draws the collision. After its compliant part, the rigid part runs at once, no `Confirm Exec`; its checks are printed first as a report (`husky_world.retreat_guardrails`: replan, start, compliant part, contact, grippers open, tool0 paths, joints, speed) |
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
waiting for `Confirm Exec`, `Load action`, `Mark action done`, `Reopen`, `Rescan`,
another step, and (in plan skeleton mode) Plan / Exec / Load Movement / Reset are
refused. Finish it or click `Cancel Exec`.

### The transfer start (J actions)

The travel to load ends where the transfer starts (the bar-loading pose), and the
export leaves that pose open, so it is chosen here:

1. `Load action` (it opens on the transfer, already loaded);
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

### Transfer workaround (temporary): the transfer as a free move, no bar

! **In use for every J action since 2026-10-06, until the bar-held transfer planner is
ready** (then the section above applies again).

The transfer's bar-held planner rarely finds a path yet. To test the live replan
and the insertion accuracy now, tick **`Workaround: plan the transfer as a free
move (no bar, from the live arms)`** (under `Plan Movement`; off by default, class
flag `TRANSFER_AS_FREE_MOVE`). While it is ticked, `Plan Movement` on the transfer
plans a plain dual-arm free move: from wherever the arms are now to the transfer's
usual goal (the insertion start), solved by IK at the live base. The bar is **not**
in the tools during that move: you mount it by hand at the insertion start
afterwards. Log: `[transfer workaround] free move (no bar) from the live arms to the
insertion start; max |goal - stored goal| = ... rad`.

1. `Load action` (a J action). The travel to load is not needed: skip it.
2. `Step` → the transfer step → `Load Movement`.
3. `Transfer start: Confirm manual pose (IK check)` → can the arms reach the insertion
   start (the Rhino pose) from where the base stands? `[transfer workaround] insertion
   start reachable from the live base (blue ghost); ...` and the blue ghost shows the
   arms there; or `... no collision-free IK for the insertion start at the live base ...`:
   drive the base and check again. The `Transfer start:` sliders are not used here: they
   disappear from the panel while the box is ticked (and the orange bar preview with them).
   `Draw tool0: live (mocap + FK) vs target (Rhino)` (any time a movement is loaded) draws
   each arm's live tool0 next to the loaded movement's target tool0 and logs the gap.
   `Visual diagnosis (...)` (the box under it) makes the grey live robot see-through and
   draws each insert / retreat plan: the Rhino start / target tool0 frames, the plan's start /
   end frames and its waypoint path (red from a failing waypoint), with the gaps logged as
   `[<movement> viz] ...`.
4. `Plan Movement` → the free move to the configuration the IK check showed (solved
   again when the base moved more than 2 mm since); preview with `Traj viz time`.
   `Confirm transfer start + plan travel to load` does 3 and 4 in one click.
5. `Exec Selected Mv Traj (auto)` → the arms go there (joint tracking, empty tools),
   then the exec pauses: `[transfer ...] the arms stand at the insertion start with EMPTY
   tools: mount the bar and clamp it with the tool buttons, then click 'Confirm Exec'.`
6. Mount the bar by hand and clamp it with the buttons on the tools → `Confirm Exec`.
   This replaces the manual mount and tool grasp steps (no click on them). The visual
   servo then corrects the tool0 placement (below) and loads the insert.
7. The insert → `Plan Movement` (it starts where the servo left the arms) → `Exec`.

! Nothing checks the bar during the move: do not tick the box with a bar already
mounted. Untick it to go back to the bar-held planner.

### Visual-servo correction after the transfer

Carrying the bar shifts the base's centre of mass, so after the transfer the tool0s
stand a few mm off the Rhino pose the insert starts from. `Exec Selected Mv Traj (auto)`
on the transfer therefore runs, after the motion (`world.execute_transfer_then_servo`):

1. Workaround box ticked: the mount pause (step 5-6 above). `Cancel Exec` there ends
   it with no correction.
2. The insert is loaded and the bar-holding accuracy test's servo loop runs on it
   (`servo_to_movement_start_live`, `doc/bar_holding_acc_manual.md`): live-base IK to the
   transfer's Rhino targets, a straight-line bar-held correction (the sampling planner
   only as fallback), first move previewed + `Confirm Exec`, later ones alone unless a
   path travels more than 5 deg (`SAFEGUARD`), up to 8 iterations, stop below 0.2 mm or
   when less than 0.2 mm is left to plan. `Cancel Exec` stops it between steps. The
   `Servoing Live Tracker` window opens when the servo starts and plots the residuals
   per iteration, as in the accuracy test.
3. Where the arms ended up becomes the transfer's end and the insert's start
   (`[Servo] propagated arm configuration -> ...`), also after reloading the transfer;
   a stale insert plan is dropped. The record goes to
   `bar_action_live_replan_exe/<date>-servoing/<insert id>_servoing_*` on the drive.

Needs the real arms and the mocap base (`FAKE_HARDWARE = 0`, `USE_MOCAP = 1`,
`USE_CELL_STATE_BASE_POSE = 0`); otherwise `servo skipped (...)` and the insert starts
where the transfer ended.

### The support robot's free approach

With mocap tracking the support robot's base, `Plan Movement` on its free approach
(e.g. `B3_H_M0_free_to_approach`) first re-solves the goal by IK at the live base
(the exported goal was solved for the authored base) and carries it into the
gripper step and the linear approach after it:
`[live goal] ...: goal re-solved at the live base; max |live IK - stored target| = ... rad`.
When the base is not tracked, the exported goal is used
(`[live goal] ...: base not tracked; ...`). When no collision-free IK exists at the
live base, nothing is planned (`... not planning.`, the collision drawn if there is one).
The IK is the robot's own exact ssik solver (`asset/ssik/alice_ur_arm_ik.py` /
`belle_ur_arm_ik.py` in husky_assembly_tamp, built 2026-10-12 / 2026-10-13 from each
robot's calibrated URDF). Both serve the same planning group, so the one whose forward
kinematics match the planner's robot is used; when none matches, PyBullet IK runs.

In a hold action, `Exec Selected Mv Traj (auto)` on the free approach then servos
tool0 onto the approach pose (`world.execute_free_approach_then_servo` ->
`servo_single_arm_to_target_live`), the single-arm twin of the correction after
Cindy's transfer: live-base IK nearest the live arm, a joint move per pass, first
move previewed + `Confirm Exec`, SAFEGUARD above 5 deg, stop below 0.2 mm, the
`Servoing Live Tracker`, a record under `bar_action_live_replan_exe/<date>-servoing/`.
The live arm then becomes the start of the gripper step and the straight approach.
The straight approach plans from the live arm (`start_state resynced to the live
arms ...`), so its IK stays on the arm's branch.

### The grip handoff (gripper close, Alice)

1. zero the F/T sensor (fingers open, touching nothing);
2. capture the current tool0 pose (arm still stiff), check it agrees with the
   arm-reported TCP (≤ 10 mm, 2 deg), then switch to `cartesian_compliance_controller`
   **before** anything closes. The target is sent in the controller's frame (UR's
   `base_link`): on Alice that is the monitor's arm base turned 90 deg about z
   (`HuskyRobotInterface._to_controller_frame`; before 2026-10-12 it was sent
   unturned and the arm jumped 21 mm towards a target 559 mm away);
3. send the close goal and publish that CAPTURED pose + zero wrench every tick,
   so the arm gives way while the jaws centre on the bar; end the hold if the arm
   drifts more than 10 mm (`GRIP_HANDOFF_MAX_DRIFT_M`);
4. when the gripper stalls / reaches its goal, switch back to joint tracking —
   always, also on Cancel, timeout or error.

Compliant first (2026-10-12) because a switch partway through the close would need
the finger position (Alice's humble `gripper_action_controller` sends no action
feedback) and would have to fire before the fingers touch the bar, which depends on
the bar diameter and the gripper speed. Without the compliance flag, with
`FAKE_HARDWARE=1`, or when FK and the arm's TCP disagree, the gripper simply closes
with the arm stiff (logged). When the switch to compliance fails, nothing is closed.

---

## Four-bar test (actions 0..8)

| run (`ROS_DOMAIN_ID`) | action | what to do |
|---|---|---|
| 86 Cindy | 0 `B1_J` | Load action · tick the transfer workaround (once per run) · transfer Plan (free move, no bar) + Exec · manual mount step (mount the bar at the insertion start) · tool grasp step · insert Plan + Exec (compliant insert; the tighten runs with it) · **Mark action done**. (Without the workaround, later: travel to load via `Confirm transfer start + plan travel to load` first, transfer with the bar held after the grasp.) |
| 86 | 1 `B1_R` | opens on the retreat: Plan + Exec (compliant retreat; the ungrasp runs with it, the untighten is not sent) · free move home Plan + Exec · Mark done |
| 86 | 2 `B3_J` | as action 0 |
| 86 | 3 `B3_H` | grey (Alice's). **Quit.** |
| 84 Alice | 3 `B3_H` | tick `Ignore built-bar collisions (bars drawn faint)` · free approach Plan (goal re-solved at the live base) + Exec · gripper open step · linear approach Plan + Exec · gripper close step (close + handoff) · Mark done · **Quit.** |
| 86 Cindy | 4 `B3_R` | Alice is now an obstacle at her hold pose. As action 1 |
| 86 | 5..8 | B4 J/R, B5 J/R as actions 0/1 |

Action 16 (`B3_HR`, Alice lets go) comes after B9 and is out of scope for this test.

---

## How the state is carried forward, and what "Mark action done" remembers

**After every step that runs** (`Exec Selected Mv Traj`, the step button; not after
`Cancel Exec`, nor when a joint move stopped short of its end), where the robot
really is (measured arm joints, mocap base when tracked) becomes the start of what
follows:

- in the action: the steps where no arm moves and the next arm movement start
  there (`[Exec] propagated arm configuration -> ...`); a plan of that movement
  made before is dropped (plan it again). After a servo run that is the
  corrected pose (`[Servo] propagated ...`);
- for the next actions: the robot's **live** state is saved to `progress.json`
  at once, without marking the action done
  (`[Plan skeleton] <robot>'s live state after <step> saved for the next actions ...`).
  So a skipped free move home or a skipped `Mark action done` never leaves an
  older state there. After an action's LAST step a hold action also records the
  hold (e.g. Alice holds B3 once her gripper closed) and a hold release the
  release;
- `Load action` puts the live state into every step up to the action's first arm
  movement (any action kind), so it starts where the robot is.

`<problem>/progress.json` (next to `ActionSchedule.json`) keeps each action's status
and, per robot, the base frame + arm joints it ended in. `Mark action done` marks
the action done and stores:

- your own robot's action: the **live** state (mocap base if tracked within 1 s,
  else the authored base; live joints);
- another robot's action: allowed only after `Confirm Exec`; its **exported** end
  state is recorded as `assumed`.

The other robots' obstacle poses come, in order, from: live mocap (fresh < 0.5 s
and calibrated) → `progress.json` → the exported file. The header's `others:` line
shows which of the last two (or `parked`) each robot uses; a fresh mocap base
still overrides it. A support robot whose hold was released (its HR action done) is
parked out of the scene. A robot can appear twice at the same pose: the solid
husky and the red obstacle robot — the red one is what the planner checks.

At start-up the connected robot starts from its own `progress.json` state (one
log line, e.g. `[Plan skeleton] Cindy starts from the progress.json state (after action 2, live).`).
With `FAKE_HARDWARE=1` this is where the simulated arms continue from; on the
robot the first joint state message overwrites the arms.

Planned trajectories are saved to `<bar>__<kind>.live-solved.json` next to the
clean export (never overwritten) by `Mark action done`. `Load action` prefers that
file; after `Reopen` the clean export is used for the rest of the run. Every step
up to an action's first arm movement always starts from the live arm.

---

## Testing

Step-by-step test plan (dry run with `FAKE_HARDWARE`, then the robots):
`doc/support_robot_test_manual.md`.

---

## To confirm on hardware (not testable here)

- Robotiq `gripper_cmd` sends **no feedback** on Alice (checked 2026-10-12); the
  handoff does not need it (compliant before the close). The handoff itself is still to
  be seen with a bar on hardware.
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
