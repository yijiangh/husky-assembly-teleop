# Lab tools test manual: the tool0 jog, the live plots, the base offset, the diagnostics, the compliant parts

The eight operator tools of `yh/lab-tools` (branch note `tasks/2026-10-27_lab_tools.md`), tested one at a time:
first in the simulation (the fake lab of `doc/fake_lab_test_manual.md`), then on one robot. Every step says what to
click, what to expect, and what to do if not. Plan an hour for the simulation and an hour on a robot.

Simulation runs on 2026-10-27 with this code, in a slot of the fake lab (every robot simulated, the fake wrench
on): everything below marked **(sim)** was run as written; the notes under "On the robot" are what to look for.

## Before you start

**In the simulation.** The fake lab as in `doc/fake_lab_test_manual.md` sections 0 and 1, plus:

- the Cartesian controllers built (its section 0.2): the compliant parts need them;
- `ros-humble-gripper-controllers` installed beside the Robotiq packages (its section 0.1 lists it now): the
  simulated Robotiq's action server needs it, or `B21_H_open` is refused and the handoff cannot run;
- the simulation's fake force/torque reading, on by default since this branch: `sim.launch.py` starts `sim_wrench`,
  a near-zero wrench per simulated arm (`wrench_bias:=`, `contact_n:=` add a standing reading and a contact while a
  simulated screw is stalled). **Expect** in the sim pane: `[sim_wrench]: a200_0806/left_ur5e: faking its
  force/torque reading …`, one line per arm, and in **Control** › any arm `F -0.0 -0.1 -0.0 N` instead of `— — —`.

**On the robot.** One robot, its arm under the joint controller, e-stop in hand, the pendant's speed slider at
100 % only for the parts that say so; start with a support robot (Alice or Belle: one arm) for 1–3, Cindy for 5–8.

**A note on browsers.** A held button reports at 20 Hz from a normal browser. A slow page (software WebGL, a
phone's background tab) reports less often, and a jog's press then drops and restarts every second or so (the joint
jog does the same): judge a jog on the laptop's Chrome, not on a remote or headless one.

---

## 1. The tool0 jog (Control › robot › arm › Joint)

The old Teleop window: hold a button and tool0 moves along, or turns about, one of its own axes at the speed set,
until you let go. One joint point is streamed every tick 0.2 s ahead on a straight line from where you pressed, so
the arm never drifts off the axis; one arm jogs at a time.

1. **Control** › `0806` › `Left`: under **Joint**, below **Hold**, the line `tool0 jog: tool0's own axes · moves while
   held · one arm at a time`, the **Tool speed** slider at 5 mm/s, and the **Tool0** folder: `x−  x+  rx−  rx+` /
   `y−  y+  ry−  ry+` / `z−  z+  rz−  rz+`. **(sim)**
2. Hold **x+** for 2 s. **Expect:** the line under the buttons reads `jogging along tool0 x: +9.8 mm at 5 mm/s`
   (counting up), `tcp` in the status moves, and on release the pane says `arm left_ur_arm: hold sent` once.
   **(sim: ~10 mm per 2 s on a normal browser; see the note on browsers.)**
   - **If not:** a chip `jog refused` names why (hover): `the joint controller is not running` (click **Joint**
     under **Run**), `lease refused` (another plugin holds the robot: wait or Cancel it), `no joint state`.
3. Hold **rz+** with the speed at 10: `jogging about tool0 z: +2.0°` after a second (0.2 °/s per mm/s).
4. Hold **x+** on the `Right` tab while the left arm's button is still held (two fingers, or the phone): the second
   is refused, `jog refused: a200-0806 left_ur_arm is jogging; one arm at a time`. **(sim)**
5. With the arm under **Cart** (the compliance controller): any Tool0 button is refused, `the joint controller is not
   running; switch to Joint`.
6. **On the robot:** speed 5, each of the six moves for a second, then each turn; watch the pendant's TCP readout move
   along one axis only (tool0's: the flange frame, not the base's). Hold **x+** for 10 s: the arm keeps a steady 5
   mm/s and stops within 0.2 s of release. At 50 mm/s the point is 10 mm ahead; a joint near a singularity stops the
   jog with `jog stopped: … too near a singularity` and the press is ignored until released.

## 2. The force/torque watch (Control › robot › arm › Live plots)

1. In the arm's tab, the **Live plots** folder at the bottom, closed at start: open it, tab **F/T**: two plots,
   `left_ur_arm_force [N]` (x, y, z) and `left_ur_arm_torque [Nm]`, 30 s wide, redrawn 4 times a second, and the
   note `raw: not zeroed, the tool's weight included`. **(sim: a flat line near 0 with the fake wrench's noise.)**
   - **If not:** `no F/T reading: the arm reports no wrench (fake hardware has none)`: the arm's `rate_limiter/
     ft_sensor_wrench` is silent; in the simulation the fake wrench node is off (`fake_wrench:=false`).
2. **On the robot:** push the flange sideways by hand: the matching line swings and comes back; **FT › Zero** brings
   the lines to zero; a tool mounted shows as a standing offset.

## 3. The mocap base offset (Control › robot › Base › Mocap offset)

The old "MOCAP base XYZ offset": a world shift of one base's mocap pose for this session, for a base whose
calibration is off by a known amount. It is applied as each sample is stored, so everything (the scene, the planners,
the executor, the records) sees the shifted pose; a path sent to the onboard follower is converted back.

1. **Control** › `0806` › `Base`: the **Mocap offset** folder at the bottom, closed; open it: `Δx m`, `Δy m`, `Δz m`
   at 0, **Offset** › **Apply** | **Reset**, and `applied 0.000 0.000 0.000 m` below. **(sim)**
2. Type `0.05` into `Δx m` and click **Apply**. **Expect:** the status line's `xyz` jumps by +0.050 in x, a chip
   `offset 0.050 0.000 0.000 m` on the base's chip line, the pane `base a200-0806: mocap offset 0.050 0.000 0.000 m`,
   and in the 3D view the robot moves 5 cm; a run that is open notes it in `events.jsonl` (`mocap_offset`). **(sim)**
3. **Plan** › **Base**, Load **Robot**, **Plan**, **Commit** a short path: the follower drives to the target where
   the shifted drawing says it is; its reported path on the floor lines up with the drawing (the conversion back).
4. Click **Reset**: the inputs and the pose go back, the chip disappears. **(sim)**
5. **On the robot:** with the robot standing still, Apply 0.05 in x: the drawing moves, the robot does not; a base
   plan from there lands the real robot 5 cm from where the drawing says, as it should (the offset is a correction
   you believe in, not a measurement). Reset before the assembly.

## 4. Rhino bar lines (Assembly › process › View)

1. With a process loaded, tick **Rhino bar lines** under the View buttons. **Expect:** every bar of the process as a
   pink line through its assembly pose with its name (`B20`, `B21`, …) at the middle: a ruler of where the design puts
   the bars. Untick: gone. **(sim)**
2. With `mocap_bars` loaded and a bar tracked, its **Show markers** box also draws that bar's planned centre line and
   name in the same pink next to the fitted (cyan) bar.

## 5. Check tool0 (Assembly › Check tool0)

The old "Draw tool0: live (mocap + FK) vs target (Rhino)".

1. Select a transfer (`B20_J_transfer`) in the process panel, with the robot live. Click **Check tool0**. **Expect:**
   under Exec, `tool0 live minus target: left 0.02 mm (+0.01/+0.02/+0.00) axes 0.00/0.00/0.00° | right …` (the gap
   of each flange's live tool0, by the mocap base and the measured joints, from the step's authored target), the same
   in the pane as `[tool0 check] B20_J_transfer: …`, and in the 3D view per flange the target frame (blue, 15 cm
   axes, label `left target (Rhino)`), the live frame (red, 8 cm, `left live (mocap + FK)`) and a black line between
   them. The run records it (`check_tool0`). **(sim: 0.02 mm with Cindy still at the transfer's end.)**
   - **If not:** `Check tool0: … is not live` (mocap or joints missing), `… has no authored tool0 target` (a step
     without one).
2. Drive the base 10 mm (D-pad, a tap) and click again: the x gap grows by about 10 mm. **(sim: by the base offset.)**
3. The frames go when you change the step, or while another panel's ghosts show.

## 6. Visual diagnosis (Assembly › settings)

The old "Visual diagnosis" box: the real robots see-through, and each insert or retreat plan drawn as its tool0 path.

1. Tick **Visual diagnosis**. **Expect:** every real robot with arms drawn see-through (a single-colour copy at 30 %
   in place of the textured one; the first tick takes about a second per robot). **(sim)**
2. Select `B20_J_insert` (or a retreat) and click **Plan**. **Expect:** in the 3D view per arm the green tool0 path
   (one line per plan row, small axes along it), the planned start and end frames (red, 6 cm, `left_ur_arm plan
   start` / `plan end`), the Rhino start and target frames (blue, `Rhino start` / `Rhino end`) and black lines from
   each Rhino frame to its planned one; in the pane `[visual diagnosis] B20_J_insert left_ur_arm: Rhino start→plan
   start 0.17 mm/0.011°, Rhino end→plan end 0.00 mm/0.000° | right_ur_arm: …; 4 waypoints`. **(sim: a retreat's
   first movement has no Rhino start: only its end is drawn.)**
3. **Clear**, a new step or unticking the box removes the drawing; unticking also draws the robots solid again.
4. A free move's plan draws nothing: only inserts and retreats have a tool0 path worth looking at.

## 7. The Exec plots window (bottom left)

The **Exec plots** window starts minimized to its bar at the bottom left, beside the Plan plots' bar, and opens by
itself when a servo or a compliant part starts, as the old Servoing Live Tracker and Compliant Exec Force windows
did; close it with the control at its bar's right end (nothing closes it for you). It holds a chip line saying
whose curves they are, and the running kind's plots: the servo's five or the compliant part's two.

1. **A servo.** Run B20_J's transfer step (fake lab manual 4.2). **Expect:** when the servo starts, the window
   opens with the chip `servo: B20_J_transfer on a200-0806`, and per pass one point on `tool0_pos_err [mm]` (L
   x/y/z/|d| in reds, R in greens), `tool0_rot_err [deg]`, `base_pos_diff [mm]` and `base_rot_diff [deg]` (blues),
   and `bar` (MoCap vs design, FK vs design) when a bar is tracked; x is the pass number. The points stay until the
   next servo.
2. **A compliant part.** Run an insert with a mating joint (B21_J's, fake lab manual 5.2) or a retreat. **Expect:**
   the window opens, the servo's plots hidden, with the chip `compliant: B21_J_insert on a200-0806`, then `force [N]` and `torque [Nm]` filling every tick, L reds and
   R greens, x in seconds since the part started; in the simulation the fake wrench's noise around zero, and the
   contact force (`contact_n`) while a screw is stalled. Without a wrench the lines are zero and a chip says `no force
   reading: the arms report no wrench`.

## 8. The compliant parts in the simulation

With the Cartesian controllers built and the fake wrench on, the fake lab manual's steps marked "not run yet" run:

- **4.4** B20_R's retreat: the switch to `cartesian_compliance_controller` succeeds, the 5 mm break-out streams
  under it, then the tail. The `contact` check now reads the fake wrench (near zero: `check PASS contact` unless
  `contact_n` is set); the `grippers` check still FAILs (the simulated screws loosen until a STOP and never stall).
- **5.2** B21_J's insert: rigid to the split, the switch to compliance, and since the simulated screw stalls 2 s
  after the tighten (at the switch itself): the left arm straight back under joint control, the right arm finishing
  its line under compliance, `STALLED`. Set `tighten_s 8` (fake lab manual 3.4) to see the compliant line run.
- **5.3 step 5** B21_H's handoff: needs the Robotiq's action server (`ros-humble-gripper-controllers`); refused
  without it as `/a200_0804/gripper: the gripper's action server is not available`.

On the robot these are the fake lab manual's own steps, B20 first (a ground bar: always rigid), then the compliant
insert on B21 with the Exec plots window open.

Run on 2026-10-27 in the simulation: 4.4 and 5.2 as above; 5.3 waits for the package.

## What to send back

The pane's lines for each step, the chip texts you saw instead of the expected ones, and for 1 and 3 on the robot:
the pendant's TCP readout before and after a 10 s hold, and the base's `xyz` before and after Apply.
