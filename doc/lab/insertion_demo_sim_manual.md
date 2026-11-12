# Insertion demo dry run: the demo on a simulated Cindy, on this laptop

Rehearses the demo of [insertion_demo_manual.md](insertion_demo_manual.md) without the robot: the same process, the
same `insertion_demo` plugin, the same `base.sh` and `monitor.sh`. Only Cindy and her router are replaced: her arms
and scaffolding tools are simulated on this laptop (crl-husky's simulation, as in the fake lab), and every node meets
through a Zenoh router of the dry run's own. You click through Grasp, Transit, Insert, Retreat, Compliant and Rigid,
each move planned and previewed before **Execute** runs it, then the interruptions. Plan about 20 minutes.

Replace the placeholder with your own path: `<workspace>` is the ROS workspace (the folder holding `src/`, `venv/`,
`install/`).

## What is the same as in the hall, and what is not

| | In the hall | In the dry run |
|---|---|---|
| Cindy's launch | over ssh: `crl_arms_and_base.launch.py gripper:=scaffolding_v3` | the `sim` pane: `scripts/fake_lab/sim.sh robot:=0806 fake_motive:=false` |
| Zenoh router | Cindy's own, over the cable, ROS domain 80 | the `router` pane, `127.0.0.1:7471` only, ROS domain 71 |
| fixed base | `base.sh` | `base.sh`, the same script |
| monitor | `monitor.sh` on the real Drive | `monitor.sh`, the same script, on a scratch copy of the process |
| the bar, the jig, the rack | real | none: the tools close on nothing, and nothing touches |

**What the dry run cannot show** (the lab checks of insertion_demo_manual.md, section 5, stay):
- **The screws.** A simulated screw stalls a fixed time after a tighten (2 s by default), whatever the arms do, and
  a loosen runs until it is stopped. Whether 6 s of loosening frees the real bar is a lab question.
- **Contact.** The simulated force reading stays near zero: the compliant arms follow their line and are never pushed.
- **The cable and the clocks.** Everything runs on one laptop, on one clock.
- **The rack.** The planner does not know it, here as in the hall.

> ⚠️ **Safety.** Everything runs on ROS domain 71, from a clean environment. Every node connects only to the dry
> run's own Zenoh router, which listens on the loopback only (port 7471; the robots' routers use 7447) and connects
> to no robot. The scripts refuse the robots' domains. The monitor's Drive root is the scratch folder
> `~/husky_insertion_demo_sim/drive`: it never opens the real Drive.

Steps are numbered so you can report them as, for example, "3.4 not OK: …".

---

## 0. Once, before the first dry run

**0.1** The fake lab's packages and the Cartesian controllers built into the workspace, as
fake_lab_test_manual.md 0.1 and 0.2 have them. The insert, the retreat and the Compliant button all switch the arms
to `cartesian_compliance_controller`: without it every one of them fails at the switch (`controller manager rejected
the switch`).

**0.2** The workspace built from this branch, with crl-husky on `jg/ros2`:

```shell
cd <workspace> && source venv/bin/activate && source /opt/ros/humble/setup.bash
python3 -m colcon build --symlink-install
```

**0.3** The demo's process synced in your Drive (`data_design_study/261104_SRD_insertion/process`). The session's
start copies it into the scratch Drive; `DEMO_DRIVE` names your Drive folder if the scripts do not find it.

---

## 1. Start

**1.1** In any terminal, run:

```shell
<workspace>/src/husky-assembly-teleop/scripts/insertion_demo/sim_tmux.sh
```

**Expect:** `sim_drive.sh: …/husky_insertion_demo_sim/drive/data_design_study/261104_SRD_insertion/process is ready
(120M)`, then the tmux session **`insertion_demo_sim`** with five panes, each named in its top border:
- **`monitor`, top left:** `monitor.sh`, typed in but not started.
- **`checks`, bottom left:** a shell with the dry run's ROS settings; `ros2 node list --no-daemon` typed in.
- **`sim`, top right:** the simulated Cindy, in place of her launch. It starts at once.
- **`base`, middle right:** `base.sh`, typed in but not started.
- **`router`, bottom right, small:** the dry run's Zenoh router. It starts first.

**Expect, in the `router` pane:** `fake lab: Zenoh router for ROS domain 71, on 127.0.0.1:7471 only`.

**Expect, in the `sim` pane:** `fake lab: alice, belle and cindy on ROS domain 71 …` (the script's own line: only
Cindy starts here), a `Configured and activated …` line per controller, `announcing left_gripper as simulated
(scaffolding tool sim)` and the same for `right_gripper`, `### ### ### STARTING ALL ARMS`, and last two `sim_wrench`
lines, `a200_0806/left_ur5e: faking its force/torque reading …` and the same for `right_ur5e`.

- **Leave the session:** Ctrl-b d. It keeps running.
- **Come back:** run `sim_tmux.sh` again, or `tmux attach -t insertion_demo_sim`.

**1.2** When the `sim` pane goes quiet (about 20 s), click into the `base` pane and press **Enter**.

**Expect:** `insertion demo: rigid body 1860 (Cindy) fixed at the origin, ROS domain 71 via 127.0.0.1:7471`, then
the warning `FIXED mocap pose: rigid bodies [1860] stand at x 0 y 0 z 0 m, yaw 0°, 20 Hz, whatever the robots do. …`.

**1.3** Click into the `monitor` pane and press **Enter**.

**Expect:** `insertion demo: monitor on ROS domain 71 via 127.0.0.1:7471, Drive root …/husky_insertion_demo_sim/drive`,
`insertion demo: open http://localhost:8080; log: …`, and a few seconds later `started in … ms`. PyBullet's own lines
print in this pane too (a known workaround, `scripts/lab_common.sh`).

---

## 2. First look

**2.1** Open http://localhost:8080 in Chrome, and keep that one page open.

**Expect:**
- **3D view:** Cindy alone at the origin of the floor grid, her arms stowed. The simulation parks her elsewhere; the
  monitor shows her where `base.sh` says, as it will in the hall.
- **Health, top left:** `ALL GREEN`. Her base shows a blue `sim`, then `signal`, `mocap`, `estop` and `ctrl` in
  green; each arm `sim`, `signal`, `arm`, `tcp`, and its tool `sim`, `scaffolding_v3`. The `mocap` chip is green
  from the fixed pose alone: no Motive and no relay run.
- **Side panel, right:** the tabs **Assembly**, **Collisions**, **Plan**, **Insertion demo** and **Control**.

**2.2** In the **Assembly** tab the folder is filled in. Click **Load** once, and wait about 5 s.

**Expect:** a green chip `process`, `7 actions · 17 movements`, an amber `robots differ`, and the actions
`1. B1_P · by hand · 1 mv` … `6. B6_J · cindy · 4 mv`, `7. B6_R · cindy · 3 mv`. In the `monitor` pane: `[process]
loaded …/261104_SRD_insertion/process in 3.5 s … 7 actions, 17 movements`, and a warning `a process robot differs from
the configured one: robots/cindy (a200-0806): left_ur_arm_tool0: TCP off by 106.3 mm …` (Known issues).

**2.3** Open the **Insertion demo** tab.

**Expect:** the status chips `a200-0806` (green), `B6_J` (blue), `bar free` (grey), `tools` (green), `joint` (green),
`no plan` and
`idle` (grey); **Action** `B6_J`; the sliders **Compliant mm** 10, **Loosen s** 6, **Insert mm** 15 and **Retreat mm** 15; the buttons **Save**, **Grasp**,
**Release** | **Tighten**, **Loosen**, **Stop** | the blue **Transit** | **Insert**, **Retreat**, **Return**, **Load**,
**Stow**; under **motion time** the sliders **Transit s**, **Insert s**, **Retreat s**, **Return s**, **Load s** and
**Stow s** (20, 5, 5, 5, 20, 20); under **preview** the slider **Preview time**, the line `min — s by the arms' limits`, the blue **Execute**, then **Play**,
**Cancel** | **Compliant**, **Rigid**, **Zero F/T**; under **autonomous** the slider **Settle s** 0 and the buttons **Preview cycle** and **Cycle**. Hovering
a Move button says what each does. How a step ended, and what a button refused, shows under the buttons and in the
`monitor` pane. At the bottom of the page, three windows minimized to their bars: **Plan plots**, **Exec plots**
and **Force plots**.

Every move takes two clicks: **Transit**, **Insert**, **Retreat** or **Return** plans the move and shows it in the
preview, and nothing moves until **Execute**. **Cycle** alone runs a whole cycle on one click (3.10).

---

## 3. The demo cycle

**3.1 Insert before Transit is refused.** Click **Insert**.

**Expect:** under the buttons, `insert: left_ur_arm's tool0 is 158.1 mm, 178.8° from the insert's start (limit 5 mm,
3°): press Transit first`. Nothing moves.

**3.2 Grasp.** Click **Grasp** (in the hall: after putting the bar into both tools).

**Expect:** in the pane `left_ur_arm + right_ur_arm gripper screws CLOSE`, each gripper `idle -> tightening`, then
`tightening -> stalled`, and after 2 s `gripper screws closed: stalled after left_ur_arm 2.0 s, right_ur_arm 2.1 s`.

**3.3 Transit: plan and preview.** Click **Transit**. The chip `idle` reads `plan transit` while the planner runs.

**Expect:** the first plan builds Cindy's planning cell, then `B6_J_transfer: tamp.plan_free_dual_arm try 1/3: path of
49 waypoints in 2.8 s`, `demo transit: transfer_free solved by tamp.plan_free_dual_arm: 49 waypoints, 1 tries, 4.6 s`,
and `transit planned (tamp.plan_free_dual_arm, 49 waypoints): the motion takes 20.0 s; Execute runs it`, also under
the buttons. Nothing moved. The waypoint counts and times differ from run to run.
- **Chips:** `no plan` turned into a green `transit ready`.
- **3D view:** a see-through green Cindy with her arms at the move's end, the insert's start. The jig's bars on her
  base and the target in blue show too: the demo selected the step in the process panel.
- **Preview time:** at its right end, 20. Drag it to the left: the see-through arms follow the path back to where
  the real arms stand. **Play** runs it from the start.
- **Transit s:** 20, and under **Preview time** `min 11.2 s by the arms' limits`: the move takes the longer of the
  two. Set it to 30: **Preview time** now ends at 30. Set **Insert s** to 30 instead: nothing changes, it is the
  insert's time. Set them back to 20 and 5.
- **Plan plots:** the window opened by itself. `joint values (deg)` has a line per joint over the 20 s, `L pan` …
  `R w3`, the right arm's dashed; `joint step (deg)` the largest step between the plan's rows. With **Transit s**
  at 30 the lines stretch to 30 s. Close the window with the `–` in its bar if it is in the way: the next plan opens
  it again.

**Cancel** now would drop the plan (`the planned transit dropped`, the chip `no plan`); **Transit** plans again.

**3.4 Transit: Execute.** Click **Execute**.

**Expect:** `transit along tamp.plan_free_dual_arm, 49 waypoints, 11.2 s`, `a200-0806: moving left_ur_arm,
right_ur_arm, 11.2 s`; the see-through robot is gone and both arms move at once; then `transit ARRIVED: arrived,
end within 0.00°`. The chip reads `no plan` again and the **Plan plots** are empty: **Execute** once more is refused,
`nothing planned: press Transit, Insert or Retreat first`. The **Exec plots** stay minimized: a transit is a joint
motion, without a compliant part.

To see a cancel, click **Cancel** while the arms move. **Expect:** `cancelled: holding every arm`, the arms stop,
`transit cancelled`. **Transit** again plans from where they stopped.

**3.5 Insert.** Click **Insert**.

**Expect:** `demo insert: insert solved by tamp.plan_constrained_dual_arm_linear: 4 waypoints`, `insert planned
(tamp.plan_constrained_dual_arm_linear, 4 waypoints): the motion takes 5.0 s; Execute runs it`, the chip `insert
ready`, and the see-through tools 15 mm further along the insert. Click **Execute**.

**Expect:**
- the **Exec plots** window opens by itself beside the Plan plots: a chip `compliant: B6_J insert on a200-0806`, and
  under `force and torque` the plots `force [N]` and `torque [Nm]`, three lines per arm (`L x` … `R z`), growing
  while the insert runs. The simulated readings are noise around 0. The curves stay until the next compliant part;
- `INSERT: rigid 5.0 mm (3.4 s), then COMPLIANT 10.0 mm (straight line); the joint screw of left_ur_arm +
  right_ur_arm TIGHTENS from the start until it stalls (≤ 30 s), then STOP and back to joint control. …`;
- `insert: left_ur_arm + right_ur_arm joint screws TIGHTENING`, `moving left_ur_arm, right_ur_arm, 3.4 s`;
- the simulated screws stall 2 s after the tighten, during the rigid part: `joint screw tightening -> stalled`,
  `ended by the caller: holding every arm`, `cartesian_compliance_controller running (0.2 s)`, a warning `the arms did
  NOT reach the line's end: left_ur_arm 0.0 of 11.5 mm, 11.5 mm short, …`;
- `switch back to scaled_joint_trajectory_controller requested`, here two errors `switch to
  'scaled_joint_trajectory_controller' failed: controller manager rejected the switch` and 0.2 s later `left_ur_arm,
  right_ur_arm back under scaled_joint_trajectory_controller`: a retry, not a fault;
- `insert STALLED: left_ur_arm + right_ur_arm joint screws STALLED`, also under the buttons.

**3.6 Retreat.** Click **Retreat**.

**Expect:** `retreat planned (the joint screws loosen 6.0 s, then a straight back-out to the insert's start): the
motion takes 5.0 s; Execute runs it`, the chip `retreat ready`, the see-through tools back at the insert's start.
No planner runs for this one, and `min — s`: the back-out is a straight line the compliant arms follow in **Motion
time**. Click **Execute**.

**Expect:** the **Exec plots** start over with the chip `compliant: B6_J retreat on a200-0806` and run for the 6 s of
loosening and the back-out; `unscrew retreat: force sensors zeroed: |F| left_ur_arm 0.1 N -> 0.1 N, …`,
`cartesian_compliance_controller running (0.2 s)`, `the joint screws of left_ur_arm + right_ur_arm LOOSEN for 6 s`,
each joint screw `idle -> loosening`; 6 s later the motors stop (`loosening -> idle`) and the tools back out:
`back-out: left_ur_arm 3.4 of 3.5 mm, 0.1 mm short, right_ur_arm 3.5 of 3.5 mm, 0.1 mm short`; then `back under
scaled_joint_trajectory_controller (0.1 s)` and `retreat RETREATED: the screws loosened and the tools backed out to
the insert's start, the bar still held`. The back-out is short here because the insert stopped 3.5 mm in (3.5).

**3.7 The whole insert.** Give the simulated screws 12 s, so the arms reach the line's end before the stall. In the
`checks` pane (press **Enter** on the typed `ros2 node list --no-daemon` first, or clear the line):

```shell
ros2 param set --no-daemon /a200_0806/left_gripper/scaffolding_tool_sim tighten_s 12.0
ros2 param set --no-daemon /a200_0806/right_gripper/scaffolding_tool_sim tighten_s 12.0
```

**Expect:** `Set parameter successful` twice. Then click **Insert** and **Execute**: after the rigid part and the
switch, the arms run the compliant line for about 7 s, `insert: tool0 travelled left_ur_arm 10.0 of 10.0 mm, 0.0 mm
short, right_ur_arm 10.0 of 10.0 mm, 0.0 mm short`, no switch error, and `insert STALLED: …` about 12 s after the
start. Click **Retreat** and **Execute**: `back-out: left_ur_arm 14.7 of 14.9 mm, 0.2 mm short, right_ur_arm 14.6 of
14.9 mm, 0.3 mm short`, `retreat RETREATED: …`. The setting lasts until the `sim` pane restarts.

**3.8 Compliant and Rigid.** Click **Compliant**.

**Expect:** `compliant: force sensors zeroed: …`, `cartesian_compliance_controller running (0.2 s)`, `compliant:
COMPLIANT: push the arms; Rigid brings them back under joint control`; the chips `joint` and `idle` both read
`compliant`. The **Exec plots** start over with `compliant: hold on a200-0806` and keep running until **Rigid**.
Nobody pushes here: the arms stand still and the force stays around 0.

Click **Insert** meanwhile. **Expect:** `Insert refused: compliant runs: wait, or Cancel it`.

Click **Rigid**. **Expect:** `compliant: left_ur_arm, right_ur_arm back under scaled_joint_trajectory_controller (0.1
s)`, `both arms back under joint control`; the chips read `joint` and `idle` again.

**3.9 Return.** After a Retreat the tools stand a fraction of a millimetre short of the insert's start. Click
**Return**.

**Expect:** `[motion_planner] demo return: tamp.plan_constrained_dual_arm_linear …` in the `monitor` pane, then
`return planned (tamp.plan_constrained_dual_arm_linear, N waypoints): the motion takes 5.0 s; Execute runs it`, the
chip `return ready`, the **Plan plots** forward with a short path. Click **Execute**: `return ARRIVED: …`, the chip
back to `no plan`. Return refuses when the planned path bends the grasp or jumps a joint (`return: refused: …`): not
expected here.

**3.10 Cycle.** With the arms at the insert's start (after 3.9) and the 12 s screws of 3.7, click **Cycle**.

**Expect:** the chip `idle` reads `cycle: insert`, `cycle: settle`, `cycle: retreat`, `cycle: rest` (a second or
two, until the arms stand still after the switch), `cycle: return` in turn, the `plan …` phases passing quickly. The **Plan plots** come forward with the insert's path, the **Exec plots** with
`compliant: B6_J insert on a200-0806`; `insert STALLED: …` about 12 s after the start, then `insert STALLED …;
settling 0 s before the retreat`; at once the retreat as in 3.6 (the **Exec plots** start over with `… retreat …`);
then `demo return` planned, the **Plan plots** with the way back, `return ARRIVED: …` and the line `cycle done
(return ARRIVED: …): back at the insert's start; Cycle runs the next`. About 50 s in all. Click **Cycle** again at
once: the same again. Click **Cycle** with the arms elsewhere (after a **Compliant** / **Rigid**, say): `Cycle
refused: left_ur_arm's tool0 is … from the insert's start …: press Transit or Return first`.

Open **Force plots** from its bar before a Cycle. **Expect:** two plots, `force [N]` and `torque [Nm]`, with lines
`L x` … `R z` (left red, right green) running on through every move and pause of the cycle, never starting over;
below them `|F| now N` and `|F| peak N` with a number per arm. **Clear** empties them and the peak, and they run on
from there.

**3.11 A longer insert, previewed first.** Set **Insert mm** to 60 and **Retreat mm** to 40, then click **Preview
cycle**. **Expect:** the console `demo preview transit`, `demo preview insert`, `demo preview return`; the line
`cycle preview shown (transit, insert, back-out, return), not to run: …`; the chip `cycle preview`; **Play** shows
the arms going 60 mm back from the joint, in, out 40 mm, and back to the 60 mm start. Nothing moved. **Execute**
is refused. Then **Transit**, **Execute**, and **Cycle**: the same for real, ending `cycle done …`. Click **Save**:
it turns amber `Save?`; click it again: `saved: the insert starts 60 mm before its end in process/actions/B6_J.json;
press Load in the Assembly tab to use it`. This is the dry run's own copy of the process; in the hall it is the Drive.

**3.12 Again.** **Insert**, **Execute**, **Retreat**, **Execute**, as often as you like: each Insert plans from where
the arms stand. In the hall the arms stand wherever a visitor's push left them after Rigid: **Transit** first, then
**Insert**.

**3.13 The bar carried from the loading pose** (the hall manual's 3.10). It needs the saved path in the dry run's
copy of the process, `~/husky_insertion_demo_sim/drive/…/process/solutions/B6_J.json` (planned as the hall manual
says; a new session keeps it). A **Save** in 3.11 makes it stale: start a new session, which brings the Drive's
action file back.

1. Assembly tab: untick **Transfer workaround**, and leave it. **Expect:** the console `Transfer workaround off:
   jointing actions as authored`; in the Insertion demo tab the first Move button reads **Transfer** and the chip
   `bar held`.
2. **Grasp**, then **Load**. **Expect:** `load: the arms do not stand at the insert's start, … the tools were closed
   since their last Release, so they may hold the bar: after a Retreat press Return first. With empty tools: Release,
   then Load plans a free move to the loading pose`.
3. **Release**. **Expect:** 15 s later `not every gripper screw stalled within 15 s of its open; motors stopped` (a
   simulated screw never stalls on a loosen). **Load** (it also plans straight after a monitor start, with no
   Release): the console `demo load: free_to_load solved by tamp.plan_free_dual_arm: …`, the line `load planned
   (tamp.plan_free_dual_arm, … waypoints): … (a free move: the tools must be empty)`, the preview's arms raised over
   the back, no bar in its tools. **Execute**: `load ARRIVED`.
4. **Grasp**, then **Transfer**. **Expect:** at once, with no planner line in the console, `transfer planned (the
   saved carry, 311 waypoints; the tools' relative pose moves at most 0.01 mm, 0.00°): the motion takes 20.0 s;
   Execute runs it`, the chip `transfer ready`. The green preview robot carries a green bar with its two joint halves:
   drag **Preview time** and they move with its tools. In **Plan plots**, below the joints: `tools' relative pose vs
   first waypoint: position (mm)` staying under 0.01, and `… axis angles (deg)` with an x, y and z line under 0.002.
5. **Execute**: `transfer ARRIVED`, the arms at the insert's start.
6. **Insert**, **Retreat** and **Return**, each with **Execute**, as 3.5, 3.6 and 3.9; the preview robot carries the
   bar in each, and the two plots fill.
7. **Load**. **Expect:** `load planned (the saved carry reversed, 311 waypoints; …) …`. **Execute**: `load ARRIVED`,
   the arms over the back again.
8. **Cycle**. **Expect:** the step chip going through `cycle: transfer`, `cycle: insert`, `cycle: retreat`, `cycle:
   return`, `cycle: load`, and the line `cycle done: back at the loading pose; Cycle runs the next`.
9. **Preview cycle**. **Expect:** `cycle preview shown (transfer, insert, back-out, return, transfer back; …), not to
   run: …`.
10. Tick **Transfer workaround** again: the button reads **Transit**, the chip `bar free`. **Transit** plans the free
    move of 3.3: no bar in the preview robot's tools, the two plots of the tools' relative pose empty. Untick it.
11. Set **Insert mm** to 40, click **Transfer**. **Expect:** `transfer: the saved carry does not end at the insert's
    start for Insert mm 40: set Insert mm back to the process's own (30), or Save, Load and plan the carry again`.

---

## 4. Interruptions

**4.1 Cancel during an insert.** With the 12 s screws of 3.7: click **Insert**, **Execute**, and about 7 s later
**Cancel**.

**Expect:** the arms stop, `insert: cancelled; left_ur_arm, right_ur_arm held, still under
cartesian_compliance_controller or switching: switch them back in robot_control`, `insert cancelled`; the first arm
chip reads `compliant`, the second `idle`.

1. Click **Rigid**. **Expect:** `rigid: left_ur_arm, right_ur_arm back under scaled_joint_trajectory_controller`,
   `both arms under joint control`.
2. Click **Insert**. **Expect:** refused, `left_ur_arm's tool0 is 11.0 mm, 0.0° from the insert's start …: press
   Transit first` (your distance differs): the arms stand part way in.
3. Click **Retreat**, then **Execute**. **Expect:** as 3.6: the screws loosen and the tools back out to the insert's
   start, `retreat RETREATED: …`. **Insert** runs again.

**4.2 Stop all while compliant.** Click **Compliant**, then **Stop all (Esc)** at the top left of the page.

**Expect:** `SOFT STOP: stopping every robot and cancelling the plugin tasks that may move one`, `SOFT STOP: cancelled
'insertion_demo: demo compliant'`, `compliant: cancelled; left_ur_arm, right_ur_arm held, still under
cartesian_compliance_controller or switching …`; Health reads `PROBLEMS`, the arms' `arm ctrl` chips amber (`no
controller active; commands will be refused`); the demo's arm chip reads `mixed`.

1. Click **Transit**: it still plans (`transit planned …`), since a plan moves nothing. Click **Execute**.
   **Expect:** `transit REFUSED: a200-0806 refused the trajectory: robot a200-0806 is soft stopped, a multi-arm
   trajectory not sent; press Resume first; nothing sent`.
2. Click **Resume** under Health, then **Rigid**, in this order. **Expect:** `resume arms: resuming` and the arm
   chips green again, `ALL GREEN`: the arms are back under the compliance controller. Then `rigid: left_ur_arm,
   right_ur_arm back under scaled_joint_trajectory_controller`, `both arms under joint control`, the demo's chip
   `joint`. Rigid before Resume switches the arms back for a moment only: the stop takes the controller away again,
   and Rigid is needed once more after Resume.
3. **Transit**, **Insert**, **Retreat**, each with its **Execute**, run again.

**4.3 Cancel during a cycle.** Click **Cycle**, with **Settle s** at 5, and during `cycle: settle` (the 5 s after `insert STALLED`) click
**Cancel**.

**Expect:** `cycle cancelled`, the arms under joint control, the bar still in. Nothing resumes: click **Retreat**,
**Execute** (3.6), then **Return**, **Execute** (3.9), then **Cycle** runs again. A Cancel during `cycle: insert` or
`cycle: retreat` leaves the arms compliant, as 4.1: **Rigid** first.

**4.4 Tighten and Loosen by hand.** With the arms anywhere, click **Tighten**: the chip `tighten` while the simulated
joint screws run, then `joint screws tightened: stalled after left_ur_arm 12.0 s, right_ur_arm 12.0 s`. Click
**Loosen**: `loosen`, then after **Loosen s** `joint screws loosened for 6 s; motors stopped`. Click **Loosen** again and
**Stop** a second later: `loosen cancelled`, both screws stopped. **Stop** with nothing running: `both tools' motors
stopped`. Nothing else moves.

**4.5 Release** (the end of the day). Click **Release**.

**Expect:** `left_ur_arm + right_ur_arm gripper screws OPEN`, and 15 s later `not every gripper screw stalled within
15 s of its open; motors stopped`: a simulated screw never stalls on a loosen.

**4.6 Zero F/T and Stow** (the bar out of the tools). Click **Zero F/T**. **Expect:** `zero: force sensors zeroed:
|F| …` in the console and `force sensors zeroed: |F| now left_ur_arm … N, right_ur_arm … N` under the buttons; nothing
moves. Click **Stow**. **Expect:** the chip `plan stow`, the console `demo stow` on `tamp.plan_free_dual_arm`, then
`stow planned (…)`, the chip `stow ready`, the see-through arms folded in their park pose. Click **Execute**.
**Expect:** `stow ARRIVED` after **Stow s**; both arms stand parked, as the simulator started them.

---

## 5. Stop

1. Press Ctrl-C once in the `monitor` pane and wait for `(viser) Server stopped` twice.
2. Run `tmux kill-session -t insertion_demo_sim`. This also stops the simulation, the fixed base and the router.

Restarting only the monitor (Ctrl-C, **Up**, **Enter**, reload the page, **Load**) keeps the arms where they are.

---

## Known issues you will meet

These are not mistakes on your side.

| What you see | Why |
|---|---|
| `robots differ` after Load, `TCP off by 106.3 mm` in the pane. | This export's scaffolding tools have their TCP 106.3 mm from the configured tools'. The monitor plans with the configured robot, and the demo's motions are tool0 motions. |
| An insert's screws stall after 2 s whatever the arms do; the arms stop a few millimetres in. | The simulated tool's `tighten_s`, 2 s by default (3.7 sets it). |
| Two `controller manager rejected the switch` errors at the end of a short insert. | The switch back came while the switch to compliance was still settling; it is taken on the retry 0.2 s later. |
| Release ends with `not every gripper screw stalled within 15 s`. | The simulated screws loosen until a STOP instead of stalling fully open. |
| The `sim` pane's first line names alice, belle and cindy. | The fake lab's script prints it; `robot:=0806` starts Cindy alone. |
| PyBullet's lines in the `monitor` pane. | The monitor starts with pytest imported, a workaround for a compas_fab bug (`scripts/lab_common.sh`). |
| A plan fails with `can't be planned for: its base is not tracked`; Health reads `PROBLEMS`, `mocap` red, Cindy drawn away from the blue target. | `base.sh` is not running: press **Enter** in the `base` pane (1.2). |
| **Execute** refuses with `the arms are … from the plan's start …: plan again`, the chip `transit stale`. | The arms moved since the plan. Press the Move button again. |
| An arm's chips stay red after start. | Its spawners timed out (`Failed loading controller joint_state_broadcaster` in the `sim` pane). Press Ctrl-C there, **Up**, **Enter**. |
| `sim.sh` refuses: `simulator nodes of an earlier start still run`. | A start that died part way left its nodes running. Run the `kill -INT …` it prints, then start it again. |

A second dry run beside this one (yours and an agent's) takes its own domain, page and folders:
`DEMO_SIM_DOMAIN=73 DEMO_PORT=8097 DEMO_HOME=~/husky_demo_check DEMO_SESSION=my_check sim_tmux.sh`.

---

## What to send back

Tell me the step numbers that did not go as written, and what you saw. A screenshot helps for anything on the page.
The monitor log is at `~/husky_insertion_demo_sim/logs/monitor_<date>.log`, the panes' logs beside it.
