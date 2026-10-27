# Fake-lab test manual: the first three bars, end to end, on three simulated robots

Builds B20, B21 and B22 of the two-module demo with the new monitor on three simulated robots on this laptop: Cindy
joints each bar and retreats, Alice holds B21 while Cindy joints it. That is schedule entries 1–7 of the process:
B20_J, B20_R, B21_J, B21_H, B21_R, B22_J, B22_R. Every movement of every action runs as the process has it: the base
drives, the transfer, the bar placed by hand, the grasp, the servo, the insert, the retreat, the support robot's
approach, open, close and retreat. The simulated tools answer as the real ones do, the compliant parts switch the arms
to the compliance controller as in the lab, and nothing is skipped or faked.

The robots are Jakob's simulation (`launch/sim.launch.py robot:=all`, [simulation_manual.md](simulation_manual.md)),
on crl-husky `jg/ros2` (78041cf or later): a simulated base with the real follower, the UR driver's fake hardware for
the arms, each arm with the robot's tool simulated (a Robotiq that reaches every goal at once, a scaffolding tool
whose screws stall a fixed time after a tighten), a fake Motive with Motive's measured latency, jitter and marker
errors, and the mocap relay, as on the lab PC. Everything talks over Zenoh through a router of the fake lab's own.
Plan about two hours.

Run on 2026-10-26 with this code (M6, M7 and Jakob's core-port of the same day merged): sections 1–6 as written;
run again on 2026-10-27 (branch `yh/lab-tools`) with the Cartesian controllers built and the fake wrench on: the
retreat's compliant break-out (4.4) and the compliant insert (5.2) as written below. The handoff (5.3 step 5) and
Alice's open (5.3 step 3) still wait for `ros-humble-gripper-controllers` (section 0.1): those two say "not run yet"
and give what the code does; tell me where the page differs.

## What you are testing, and whose it is

| Feature | Whose | Where |
|---|---|---|
| **The monitor.** The web page and phone page, health (a `SIM` chip per simulated base, arm and tool), Stop all (latches each robot until its Resume), leases, the console at the bottom right, robot_control in the **Control** tab. | Jakob | 1–2, 8 |
| **The simulated robots and tools.** Bases, arms, a Robotiq and a scaffolding tool per `model.yaml`, the fake Motive and the relay. The fake-lab scripts around them are ours. | Jakob | 1–2 |
| **process.** Loads the process folder and draws one movement at a time; its action list carries the run's marks. | Jakob, the marks ours | 3 |
| **obstacles.** The measured lab from `data/lab/main.json`, with a floor the arms and what they hold may not cross. | Jakob | everywhere a plan is made |
| **base_planner_panel** (**Plan** › **Base**). A base path around robots and obstacles, committed to the follower. | Jakob | 4.1, 5.1, 5.3, 6.1 |
| **assembly_run.** The run folder: marks, where each robot was left, the settings (transfer workaround, insert split, spiral, rigid insert, servo by bar), the event log; Mark done, Reopen, New run. | **Ours** | 3–7 |
| **motion_planner.** Plans a step's motion at the live base through tamp, goals by exact IK (ssik), collision checks. | **Ours** | every Plan |
| **assembly_steps.** The run's current step in the **Assembly** tab: Live IK, Plan, Play, Exec, the plots. | **Ours** | 4–6 |
| **movement_exec.** Runs a step's parts after Confirm, one motion at a time across all robots, with the M6 compliant parts (insert, retreat, handoff) and the M7 servo after a transfer, an approach or a grounded insert. | **Ours** | 4–6 |

**Words used below:**
- **Transfer workaround.** Cindy's transfer runs as a free move with empty tools to the insert start, where you place
  the bar. It stays on in this test: this export carries no joints for the loading pose (the travel to load has
  nothing to plan to), so the process cannot run its load, place, grasp and held transfer yet. When an export has the
  loading pose, untick the box and the five movements run in order.
- **Servo.** After a transfer, a support approach or a grounded insert, the executor measures the flanges and moves
  them onto the authored targets, pass by pass, until they are within 0.2 mm. On tool0 here: the fake Motive streams
  no bar markers, so **Servo by bar** stays off.
- **Split.** The last millimetres of an insert, and the first of a retreat, run under the compliance controller
  (**Insert split mm**, 5 by default).

> ⚠️ **Safety.** Everything runs on ROS domain 70, from a clean environment. Every node connects only to the fake
> lab's own Zenoh router, which listens on the loopback only (port 7470; the lab's routers use 7447) and connects to
> no robot. The relay looks for Motive at 127.0.0.1 only. The robots in use can't see this test, and it can't see
> them. The monitor's Drive root is the scratch folder `~/husky_fake_lab/drive`, so its `runs/` never reach the real
> Drive.

Steps are numbered so you can report them as, for example, "4.2 not OK: …".

---

## 0. Once, before the first test

**0.1** Install the UR driver, the Robotiq description and controllers, the gripper controllers, Zenoh's ROS
middleware and tmux. The simulated arms run on the UR driver's fake hardware; the simulated Robotiq on Alice and Belle
needs its description and controllers, or `sim.launch.py robot:=all` stops at Alice with `package
'robotiq_description' not found`, and its `GripperActionController` comes from `ros-humble-gripper-controllers`, or
its spawner dies (`spawner robotiq_gripper_controller … process has died`) and every Robotiq step is refused with
`the gripper's action server is not available`.

```shell
sudo apt install ros-humble-ur-robot-driver ros-humble-robotiq-description ros-humble-robotiq-controllers \
    ros-humble-gripper-controllers ros-humble-rmw-zenoh-cpp ros-humble-zenoh-cpp-vendor tmux
```

**0.2** Build the Cartesian controllers into the workspace. The insert's compliant part, the retreat's break-out and
the handoff switch the arms to `cartesian_compliance_controller`, which the robots carry onboard and this workspace
does not: without it the simulated arms' spawner dies at start (four `process has died` lines) and every compliant
part fails at the switch (`controller manager rejected the switch`). The source is cloned in
`src/cartesian_controllers` (fzi `cartesian_controllers`, branch `ros2`; its simulation, tests, handles and
utilities carry a `COLCON_IGNORE`).

```shell
cd ~/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash
python3 -m colcon build --symlink-install --packages-select cartesian_controller_base cartesian_motion_controller \
    cartesian_force_controller cartesian_compliance_controller
```

**0.2b** The simulated arms report no force (the fake hardware reads NaN), so `sim.launch.py` also starts
`sim_wrench`, a near-zero force/torque reading per simulated arm, by default: the F/T plots, the retreat's contact
check and the servo's load guard then read something. `wrench_bias:=0,0,-5,0,0,0` adds a standing reading,
`contact_n:=8` a push along the tool's z while a simulated screw is stalled; keep both small, the compliance
controller reads the same topic and moves the arm by it.

**0.3** Copy the test process into the scratch Drive root. `--fresh` also deletes the runs of an earlier test.

```shell
~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/fake_lab/drive.sh --fresh
```

**Expect:** `drive.sh: …/husky_fake_lab/drive/data_design_study/261008_2ModulesDemo_purged_process is ready (84M)`.

**0.4** Build the workspace from this branch, with crl-husky on `jg/ros2` (78041cf or later): the fake lab runs the
installed relay, fake Motive and simulated tools, and `env.sh` refuses an install without them.

```shell
cd ~/Code/ros2_ws_next && source venv/bin/activate && python3 -m colcon build --symlink-install
```

---

## 1. The shared tmux session

**1.1** In any terminal, run:

```shell
~/Code/ros2_ws_next/src/husky-assembly-teleop/scripts/fake_lab/tmux.sh
```

This opens the tmux session **`fake_lab`** with one window and four panes:
- **top:** the three simulated robots, their tools and the fake Motive (`sim.sh`). They start at once.
- **middle, the monitor pane:** the monitor command (`monitor.sh`), typed in but not started yet.
- **bottom left, small:** the Zenoh router (`router.sh`) that every node meets through. It starts first.
- **bottom right, small:** the mocap relay (`relay.sh`), as on the lab PC.

I read the panes from my side (`tmux capture-pane`). I never type into them unless you ask.

- **Leave the session:** Ctrl-b d. It keeps running.
- **Come back:** run `tmux.sh` again, or `tmux attach -t fake_lab`.

**Expect, in the bottom left pane:** `fake lab: Zenoh router for ROS domain 70, on 127.0.0.1:7470 only`.

**Expect, in the bottom right pane:** `fake lab: mocap relay on ROS domain 70, Zenoh router 127.0.0.1:7470, no Motive`,
then once the robots are up `rigid body 1840: now from the fake Motive, was the simulator's truth`, the same for 1850
and 1860. A `No NatNet frames … reconnecting` line every 30 s is right: no Motive runs here.

**Expect, in the top pane:**
- `fake lab: alice, belle and cindy on ROS domain 70, Zenoh router 127.0.0.1:7470`;
- many `Configured and activated …` lines, one per controller of each arm, `scaffolding_tool_sim`'s `announcing
  left_gripper as simulated (scaffolding tool sim)` and the same for `right_gripper`, and `sim_motive: rigid body
  1840: faking it, noise [1.5e-05, 0.003], marker_error 0.0005`, the same for 1850 and 1860.

If an arm's spawner reports `Failed loading controller joint_state_broadcaster` (three robots starting at once can
starve one; the arm's health chips then stay red), press Ctrl-C in the top pane and start `sim.sh` again.

**1.2** When the top pane goes quiet (about 20 s), click into the monitor pane and press **Enter**.

**Expect:** `fake lab: open http://localhost:8080 …` and the path of the log it saves; about 10 s later, `started in
… ms`. PyBullet's own lines print in this pane too (the known issues say why).

---

## 2. First look

**2.1** Open http://localhost:8080 in Chrome, and keep that one page open.

**Expect:**
- **3D view:** three robots side by side on their park row, y = 1.35 m (Alice at x = 0.3, Belle at 1.7, Cindy at
  3.1), facing +y, arms stowed; the measured lab (cabinet, tables, tripods, trusses, the orange border).
- **Health, top left:** every base shows `signal`, `mocap`, `estop`, `battery 90%`, `ctrl` and `SIM`; every arm
  `signal`, `arm`, `SIM` and its tool: `robotiq` green followed by an amber `robotiq SIM` on Alice and Belle,
  `scaffolding_v3` and `scaffolding_v3 SIM` on each of Cindy's arms. Belle's `mocap` is amber: her fake Motive body
  reports the 1.2 mm marker error measured in the lab (hover it). About `12 PROBLEMS`: the eleven `SIM` chips and
  Belle's `mocap`.
- **Side panel, right:** the tabs **Assembly**, **Collisions**, **Plan** (Base, Motion, Arm) and **Control** (a tab per
  robot); **Obstacles** minimized at the right edge; the **Console** minimized at the bottom. Assembly is open: the
  **PROCESS** section at the top (Folder, Load, the two boxes, an empty action list, the Step slider), then the big
  buttons **Live IK**, **Plan**, **Exec**, **Confirm** | **Cancel**, **Mark done** | **Reopen**, the settings, the
  run, two empty plots.

---

## 3. Load the process, the run and the settings

**3.1** In the **PROCESS** section the folder is filled in. Click **Load** once, and wait 5–10 s.

**Expect:**
- a green chip `261008_2ModulesDemo_purged_process`, `78 actions · 273 movements`, and an amber `robots differ` (this
  export used the old UR frames; the monitor plans with the configured robots);
- **ACTIONS** `1/78`: `1. B20_J · cindy · 4 mv · pending` on top, in blue, then `2. B20_R · cindy · 3 mv`,
  `3. B21_J`, `4. B21_H · alice · 4 mv`, `5. B21_R`, `6. B22_J`, `7. B22_R`, every row ending in `· pending`;
- under the buttons, `current B20_J · B20_J_transfer (4/5)`, `no run yet`, `0/78 done`, and `alice`, `belle`, `cindy`
  each `live now`.

Every Load draws the whole process again, and a page that opens later first receives about 200 bodies (20 s): don't
click Load twice.

**The process panel picks the run's current step.** Click an action row, drag the Step slider or click ◀ mv / mv ▶:
the step under the buttons follows, except while an Exec holds the robots' leases.

**3.2** The two boxes under Load:
- **Transfer workaround** ticked (see Words): B20_J is listed as `J_transit`, `J_M1`, `J_M2`, `J_M4`, 4 mv.
- **Rhino obstacles collide** unticked, with the grey chip `Rhino obstacles: drawn only`: Rhino's cabinet `env39` is
  drawn see-through and left out of every collision check. Leave it: it sits 0.38 m nearer the park row than the
  measured cabinet, and ticked it refuses any base pose on the row.

**3.3** The settings, under Mark done: **Ignore built bars** off, **Insert split mm** 5, **Spiral** off, **Rigid
insert** off, **Spiral s** 20, **Spiral turns** 4, **Servo by bar** off, **Dry run** off. Leave them so.

**3.4** Optional: the simulated screws stall 2 s after a tighten whatever the arms do, so an insert's 5 s path is held
40 % in. To see an insert reach its end and stall in the hold, give the screws more time, in a terminal of the
session (Ctrl-b " splits one), after `source scripts/fake_lab/env.sh`:

```shell
ros2 param set /a200_0806/left_gripper/scaffolding_tool_sim tighten_s 8
ros2 param set /a200_0806/right_gripper/scaffolding_tool_sim tighten_s 8
```

---

## 4. B20: Cindy joints the ground bar, then retreats

**4.1 Cindy's base.** Open **Plan** › **Base**, pick Robot **a200-0806**, click Load **Step**, then **Plan**.

**Expect:** `target +0.527 -3.067 +89.6°`, then after about 2 s `Lattice (hybrid A*) (straight): 5.24 m, 29.0 s`.

Click **Commit**. **Expect:** `sent to a200-0806's follower`; Cindy drives in the 3D view for about 50 s; the monitor
pane says `base path … on a200-0806: done`. **Control** › `0806` › `Base` then reads about `xyz +0.53 -3.06`,
`yaw +88°`: the simulated follower stops within about 1 cm and 2°.

**4.2 B20_J, step 1/2: the transfer, the bar, the grasp, the servo.** Back in **Assembly**, the step line reads
`B20_J_transfer (transfer_free) · then: B20_J_place_bar, B20_J_grasp · skipped: B20_J_load` and the Exec line
`Exec: cindy (a200-0806) moves left_ur_arm, right_ur_arm; then waits for B20_J_place_bar by hand; then tightens the
scaffolding gripper screws on left_ur_arm, right_ur_arm; then servos left_ur_arm, right_ur_arm onto the insert start`.

1. Click **Live IK: insert start** (about 5 s).
   - **Expect:** `check goal B20_J_transfer: goal found (ssik, rank 1 of 64), max |goal - authored| 0.06 rad` in the
     monitor pane, and `run <date>_<time> started`: a run folder is created at the first thing it records. The 0.06
     rad is the base 2° off the design base.
2. Click **Plan** (15–40 s).
   - **Expect:** the chip `plan ready`, `min 16.8 s by the arms' limits`, and in the pane `transfer_free solved by
     tamp.plan_free_dual_arm: 66 waypoints, 1 tries`. The two plots fill: every joint along the preview, and the
     largest joint step per row.
3. Click **Play**: the preview runs as it will move; the slider **t** scrubs it.
4. Click **Exec**.
   - **Expect:** the prompt `cindy (a200-0806): B20_J_transfer — move left_ur_arm, right_ur_arm along the planned path
     (Rhino obstacles not checked), 17 s, largest joint travel 214° (Play it first). Confirm to move.`
5. Click **Confirm** (here or on the phone's Exec tab). Both arms move for 17 s.
   - **Expect:** `B20_J_transfer: arrived, end within 0.00°`, then the prompt `B20_J_place_bar — do Operator places
     the bar by hand, then Confirm …`.
6. Click **Confirm** (the bar is placed, in the simulation's mind).
   - **Expect:** the prompt `B20_J_grasp — GRASP: STOP, then the M1 screw of left_ur_arm, right_ur_arm TIGHTENS until
     STALLED (≤30 s), then STOP. Confirm to send.`
7. Click **Confirm**.
   - **Expect:** after about 2 s, `B20_J_grasp: every gripper screw STALLED: the bar is clamped`. Then the servo runs
     by itself: `[servo] feedback: tool0 (0.2 mm); the MoCap bar bars/B20 is recorded alongside.`, a warning `no
     marker sample of bars/B20 (asset 1020) within 1 s: is Motive streaming it?` (right: no bar markers here),
     `[servo 0] tool0 pos L=0.06 R=0.08 mm`, `servo 1/8 B20_J_transfer: servo: nothing left to plan`, `### bar pose
     at termination (bars/B20): FK-vs-design 0.11 mm/0.007°, MoCap-vs-design n/a, MoCap-vs-FK n/a`, and `Exec
     B20_J_transfer: done — every part done`. The flanges are already within 0.2 mm, so no servo pass moves; a pass
     that would move asks for Confirm first (`SERVO 1/8 onto the insert start (tool0): …`).

**4.3 B20_J, step 2/2: the insert, and the servo after it.** The step line reads `B20_J_insert (insert)`.

1. Click **Plan** (about 3 s). **Expect:** `insert solved by tamp.plan_constrained_dual_arm_linear: 4 waypoints`.
2. Click **Exec**. **Expect:** `B20_J_insert — RIGID insert of a ground bar, no tighten, 5.0 s, largest joint travel
   2.5°.` A ground bar has no mating joint: nothing tightens and the compliance controller is not engaged.
3. Click **Confirm**.
   - **Expect:** `insert: RIGID_DONE: ground bar in, no tighten; the flanges stopped 0.0 mm from the planned end;
     peak |F| not measured`, then the servo after the grounded insert: a warning `no force reading: the servo's load
     guard is OFF. Watch the arms and use Cancel if the bar pushes on the floor.` (no F/T sensor here),
     `[servo 0] tool0 pos L=0.15 R=0.19 mm`, `nothing left to plan`, and `Exec B20_J_insert: done — every part done`.
4. The line under the buttons says `every step of B20_J done: Mark done below`. Click **B20_J done**.
   - **Expect:** `B20_J marked done in run …`, the row `1. B20_J` turns green `· done`, `1/78 done`, and the current
     step becomes `B20_R · B20_R_ungrasp (1/3)`.

**4.4 B20_R, step 1/2: the retreat, with the ungrasp during it.** The step line reads `B20_R_retreat
(retreat_linear) · with: B20_R_ungrasp`.

1. Click **Plan**. **Expect:** `retreat_linear solved by tamp.plan_dual_arm_linear_independent: 12 waypoints`.
2. Click **Exec**. **Expect:** `B20_R_retreat — RETREAT: the M1 screws of left_ur_arm + right_ur_arm OPEN; force
   sensors zeroed; COMPLIANT break-out 5.0 mm (0.5 s); joint control; the rest replanned from where the tools are and
   run at once (4.5 s, no second Confirm). Watch both tools let go and back off along their lines.`
3. Click **Confirm**.
   - **Expect:** `retreat: the gripper screws of left_ur_arm + right_ur_arm OPEN`; `force sensors zeroed`; the
     switch to `cartesian_compliance_controller` (`left_ur_arm, right_ur_arm: cartesian_compliance_controller running
     (0.3 s)`) and the 0.5 s stream; `back under scaled_joint_trajectory_controller (0.1 s)`; the tail replanned
     (`retreat tail B20_R_retreat: … 48 waypoints`); the checks: `PASS start`, `PASS break-out: worst tool0 1.6 mm
     short of its end (limit 3), travelled 3.5 / 3.4 mm`, `PASS contact: |F| left_ur_arm 0.1 N, right_ur_arm 0.0 N
     (limit 20 N)` (the fake wrench), one failing: `check FAIL grippers: left_ur_arm LOOSENING, right_ur_arm LOOSENING
     (want STALLED = fully open)` (the simulated screws loosen until a STOP and never stall), `PASS` for both tool0
     paths (46 mm, ends 0.1 mm from the goal), the joints and the speed; `running the tail now (48 rows, 4.6 s); 1
     check(s) FAILED`; after 5 s `retreat: RETREATED: retreated; 1 check(s) FAILED` and `Exec B20_R_retreat: done —
     every part done`. The ungrasp counts as done with it. The **Plots** tab shows the force and torque of the part.

**4.5 B20_R, step 2/2: home.** Click **mv ▶** in the process section; the step line reads `B20_R_home (free_home)`.

1. Click **Plan** (20–40 s). **Expect:** `free_home solved by tamp.plan_free_dual_arm`, `min 26 s by the arms' limits`.
2. Click **Exec**, then **Confirm** on `B20_R_home — move left_ur_arm, right_ur_arm along the planned path …, 26 s,
   largest joint travel 222°`. **Expect:** `B20_R_home: arrived, end within 0.00°` after 26 s.
3. Click **B20_R done**. **Expect:** `2/78 done`, the current step `B21_J · B21_J_transfer (4/5)`.

Mark done accepts an action with steps left: wait for `every step of … done` before clicking it, or the next base plan
refuses because an arm still stands at the bar.

---

## 5. B21: Cindy joints the bar, Alice holds it, Cindy retreats

**5.1 Cindy's base.** **Plan** › **Base**, Robot **a200-0806**, Load **Step**, **Plan**, **Commit**.

**Expect:** `target +0.947 -3.171 +90.0°`, a path of about `0.77 m, 11 s`, then `done` after about 30 s. The simulated
follower sometimes never starts a short path (`base path … timed out: still following`): click **Plan** and
**Commit** again. **Control** › `0806` › `Base` ends at about `xyz +0.95 -3.18`, `yaw +92°`.

**5.2 B21_J.** As 4.2 and 4.3, with these differences:
- Live IK: `goal found (ssik, rank 1 of 64), max |goal - authored| 0.09 rad`; the transfer's plan takes 19 s.
- The insert has a mating joint on the left: the prompt reads `B21_J_insert — INSERT: rigid 10.0 mm (3.3 s), then
  COMPLIANT 5.0 mm (straight line); the joint screw of left_ur_arm TIGHTENS from the start until it stalls (≤ 30 s),
  then STOP and back to joint control. Watch the bar enter its joint and the held bars stay in their jaws. Confirm to
  start.` After Confirm: `insert: left_ur_arm joint screw TIGHTENING`, the rigid part (3.4 s), `cartesian_compliance_controller
  running (0.3 s)`, and, since the simulated screw stalls 2 s after the tighten (3.4), at the switch itself:
  `left_ur_arm joint screw STALLED at 0.0 of 1.7 s; left_ur_arm back under joint control, right_ur_arm finish the line
  under compliance`, a warning `the arms did NOT reach the line's end: left_ur_arm 0.1 of 8.0 mm, … right_ur_arm 7.8 of
  8.1 mm`, `insert: STALLED: left_ur_arm joint screw STALLED; peak |F| left_ur_arm 0.2 N, right_ur_arm 0.2 N` and `Exec
  B21_J_insert: done`. Once in the pane the first switch back was `rejected` and taken 0.2 s later: a retry, not a
  fault. With `tighten_s 8` (3.4) the left arm runs its compliant line before the stall.
  - Run so far here with **Rigid insert** ticked: `RIGID-ONLY insert: the whole 15.0 mm under joint control (5.0 s);
    the joint screw of left_ur_arm TIGHTENS from the start; the arms hold the end until it stalls (≤ 30 s), then
    STOP. Compliance is never engaged; …`, then after 2 s `ended by the caller: holding every arm` and `insert:
    STALLED: left_ur_arm joint screw STALLED during the motion`. Untick the box again afterwards.
- No servo after this insert: it is not a ground bar.
- Click **B21_J done**: `3/78 done`, the current step `B21_H · B21_H_approach (1/4)`.

**5.3 B21_H: Alice holds B21.**

1. **Alice's base.** **Plan** › **Base**, Robot **a200-0804**, Load **Step**, **Plan**, **Commit**. **Expect:**
   `target +0.638 -1.561 -29.6°`, a path of about 3 m, `done` after about 50 s; `Control` › `0804` › `Base` at about
   `xyz +0.64 -1.56`, `yaw -31°`.
2. **Step 1/4, the approach and its servo.** Click **Live IK: linear start** (the first plan for Alice builds her
   planning cell, about 7 s): `check goal B21_H_approach: goal found (ssik, rank 1 of 8), max |goal - authored| 0.08
   rad`. **Plan**: `support_approach solved by arm.plan_arm: 2 waypoints`. **Exec**, **Confirm** on `alice
   (a200-0804): B21_H_approach — move ur_arm along the planned path …, 16 s, largest joint travel 206°`.
   - **Expect:** `B21_H_approach: arrived, end within 0.00°`, then the servo: `[servo] feedback: tool0 (0.2 mm); no
     MoCap bar recorded (no bar held).`, `[servo 0] tool0 pos arm=0.19 mm`, `nothing left to plan`, `Exec
     B21_H_approach: done — every part done`. The jaws stop short of B21.
3. **Step 2/4, open.** Click **Exec**, then **Confirm** on `B21_H_open — OPEN the gripper (0.426). Confirm to send.`
   - **Expect:** at once, `B21_H_open: at 0.426 rad`: the simulated Robotiq reaches every goal at once (not run yet
     here: without `ros-humble-gripper-controllers` (section 0.1) it is refused with `/a200_0804/gripper: the
     gripper's action server is not available`; the step stays, mv ▶ goes on).
4. **Step 3/4, to grasp.** **Plan** (`support_linear`), **Exec**, **Confirm** on `B21_H_to_grasp — move ur_arm …, 5 s,
   largest joint travel 22°`. **Expect:** `arrived, end within 0.00°`.
5. **Step 4/4, the close: the handoff.** Click **Exec**. **Expect:** `B21_H_close — CLOSE + compliant handoff on
   ur_arm: the arm goes COMPLIANT before the close (gives way ≤ 10 mm); jaws 0.8, effort 0.1, ≤ 30 s; then joint
   control. Watch Cindy's tools are not pulled; hand on the e-stop.` Click **Confirm** (not run yet here: refused
   as the open, `REFUSED: /a200_0804/gripper: the gripper's action server is not available; nothing sent`).
   - **Expect:** in order, the joint controller confirmed, the force sensor zeroed, the anchor gap, the switch to the
     compliance controller, the close, `gripper done`, back under joint control; the arm does not move.
6. Click **B21_H done**: `4/78 done`, the current step `B21_R · B21_R_ungrasp (1/3)`.

**5.4 B21_R.** As 4.4 and 4.5, with Alice holding B21: the retreat's prompt and checks are the same; the tail is
replanned (`retreat tail B21_R_retreat: … 52 waypoints`). Then **mv ▶**, the home move, **B21_R done**: `5/78 done`.

---

## 6. B22: Cindy joints the bar with both screws, then retreats

**6.1 Cindy's base.** As 5.1: `target +1.166 -3.212 +95.8°`, a short path.

**6.2 B22_J.** As 5.2; the insert tightens both screws: `the joint screw of left_ur_arm + right_ur_arm TIGHTENS …`,
and ends when both have stalled. Here the transfer's plan from the retreat pose took up to 110 s; `Plan` says if it
gives up (`no path in 120 s`). Click **B22_J done**: `6/78 done`.

**6.3 B22_R.** As 4.4 and 4.5. Click **B22_R done**: `7/78 done`, and the current step is `B34_J`: the first three
bars are built and the next bar is Belle's.

---

## 7. The run record and a restart

**7.1** Open the run folder:
`~/husky_fake_lab/drive/data_design_study/261008_2ModulesDemo_purged_process/runs/<date>_<time>/`.

**Expect:** `run.json`, `progress.json`, `events.jsonl`; `movements/<action>/<movement>.<n>.json`, one file per part,
the refused and cancelled ones included (an insert's or retreat's file keeps its settings, phases, switches, stall
times and checks); `servo/<action>/<movement>.<n>.json` with a `.png` of each servo's readings.

**7.2** In the monitor pane press **Ctrl-C** once, wait for `(viser) Server stopped` twice, press **Up** and **Enter**,
reload the page and click **Load**.

**Expect:** `continuing run <date>_<time>`; the done actions are still green; the settings are the run's (the
workaround box is on, Rhino obstacles off). The robots keep where they are across the restart.

**7.3** Under the run, **New run** twice within 10 s makes a new run folder (`0/78 done`, every row pending); the old
folder stays.

---

## 8. Optional: interruptions

**8.1 Cancel during a motion.** On any planned motion, Exec, Confirm, then click **Cancel** (red) while the arms move.
**Expect:** `cancelled: holding every arm`, the arms stop mid-path, `Exec …: cancelled — … (operator)`. Plan again
from where they stopped.

**8.2 Stop all during a motion.** Click **Stop all (Esc)** at the top of the page while the arms move. **Expect:**
`robot a200-0806: SOFT STOP: Stop all`, `cancelled: holding every arm`, every arm's `ctrl` chip amber with `last stop:
stopped by operator`; each robot stays stopped until its own **Resume** under Health.

---

## 9. Stop

1. Press Ctrl-C once in the monitor pane and wait for `Server stopped`.
2. Press Ctrl-C in the top pane.
3. Run `tmux kill-session -t fake_lab`. This also stops the router and the relay.

---

## Known issues you will meet

These are not mistakes on your side.

| What you see | Why |
|---|---|
| PyBullet's lines in the monitor pane. | `monitor.sh` imports pytest before the monitor: compas_fab closes stdout for a moment around every model it loads, other threads then fail with `OSError(16, 'Device or resource busy')`, and compas_fab skips that only under pytest. A real fix is your decision. |
| `robots differ` after Load. | The export used the old UR frames; the monitor plans with the configured robots. |
| The grasp's screws stall after 2 s whatever the arms do; an insert's path is held 40 % in. | The simulated tool's `tighten_s`, 2 s by default (3.4 sets it). |
| A retreat reports `1 check(s) FAILED`: grippers. | The simulated screws loosen until a STOP instead of stalling fully open. The contact check reads the fake wrench (section 0.2b) and passes near zero. |
| A base's `xyz` flickers between two poses, the relay pane says `rigid body 18xx: now from …` again and again. | A simulator of an earlier launch is still alive in the same domain (`pgrep -af husky_sim`: two with the same `__ns`); stop the old one. |
| `no marker sample of bars/B20 (asset 1020)`. | The fake Motive streams no bar markers; the servo runs on tool0 and records that the bar could not be measured. |
| A short base path `timed out: still following`. | The simulated follower sometimes never starts a path of a few decimetres. Plan and Commit again. |
| An arm's chips stay red after start. | Its spawners timed out while three robots started at once (`Failed loading controller joint_state_broadcaster` in the top pane). Restart `sim.sh`. |
| Rhino's cabinet `env39` is 0.38 m nearer the park row than the measured one. | A Rhino-side fix; with **Rhino obstacles collide** off (the default) it is drawn only. |
| The ground feet of B20 sit 15.6 mm into the floor at their design pose. | A Rhino-side fact. The planner lets a half mated to the ground touch the lab floor, as the core lets it touch the process's ground bodies; otherwise every ground bar's goal collides with the floor. |
| Mark done accepts an action with steps left. | It marks the action, not its steps. Wait for `every step of … done`. |
| Don't press **Cancel** in base_planner_panel right after a **Commit** while that robot still drives an earlier path. | A race in Jakob's code (review finding M1): within a tick, the earlier path can keep driving unwatched. Stop a driving base with **Stop all**. |

---

## What to send back

Tell me the step numbers that did not go as written, and what you saw. A screenshot helps for anything on the page. I
read the monitor and robot panes myself, and the monitor log is at `~/husky_fake_lab/logs/`. The compliant parts (4.4
step 3, 5.2's insert, 5.3 step 5) are the ones nobody has run in the simulation yet: their log lines matter most.
