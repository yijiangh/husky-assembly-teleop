# Handover: open-loop RTDE execution engine

**Written 2026-09-09.** Context note for picking this task up in a fresh session. Operator
instructions for the network/gripper side are in
[rtde_network_setup.md](rtde_network_setup.md); this file is the state of the WORK — what
is done, what is not, and what will bite you.

The compliant-insertion skill, the part visualization and the grasp verification have their
own work note: [compliant_insertion_handover.md](compliant_insertion_handover.md). This file
keeps the operator's view of them (the flags and the UI flow, below); that one carries their
state, dead ends and what is still untested on hardware. The MATH of both controllers — the
speedJ tracker's law and the insertion skill's — is in [controllers.md](controllers.md).

Keep these files updated as the work moves.

---

## What the task is

Execute the fixtureless-assembly planner's precomputed dual-arm trajectories on Cindy
(`/a200_0806`) open-loop, with real speed/acceleration control and the two arms in
lockstep. The ROS `scaled_joint_trajectory_controller` cannot do that, so the engine
drives the arms one level deeper over **ur_rtde `speedJ`**, as a Python port of the
ReferencePath mode of Valentin's controller
(`~/Code/robot_ipc_control-dev-vh/controller/impedance_controller.cpp:424-445`):

    qd_cmd = clamp(qd_ref(t) + p_gain * (q_ref(t) - q_actual), +-vmax)
    speedJ(qd_cmd, joint_accel, 1/frequency)

One tracker thread per arm, both reading **one shared wall clock** — that shared clock IS
the synchronization. Grippers stay on the existing ROS2 `GripperCommand` action path.

## State: three commits on `yh/fixtureless_assembly`, plus uncommitted work

| Commit | What |
|---|---|
| `9a3faad` | grasp slip-calibration recorder (separate feature, same UI plumbing) |
| `37cd5c8` | the engine: loader, tracker, preview, wiggle generator, network doc |
| `cfc2860` | run-until-sample cutoff + planned approach motion |

**Uncommitted in the tree** (from the parallel pickup-calibration session, see
[pickup_calibration_handover.md](pickup_calibration_handover.md)): `pickup_calib.py`, its two
docs, its `setup.py` entry, and the `--layout-json` integration into
`open_loop_approach.build_obstacles` / `open_loop_engine`. A measured layout replaces the
guessed table and adds one box per part. That session also has an **unanswered question
about what to commit/push** — re-ask before pushing anything.

Also modified and NOT mine: the `husky_assembly_tamp` submodule bump and a `.gitignore`
edit, both predating this work.

## The pieces

| File | Responsibility |
|---|---|
| `husky_assembly_teleop/open_loop_traj.py` | Pure loader for schema `assembly-open-loop-json-v2` (no ROS/UI, so it can be smoke-tested with plain venv python). Per-arm `CubicHermiteSpline(times, q, qd)` — exact at every authored sample. `sample(arm, t, t_end, brake_time)` clamps both ends and brakes after a cutoff. `traj_from_arrays` wraps generated motions as the same type. |
| `husky_assembly_teleop/open_loop_approach.py` | Collision-checked approach from the live pose to the trajectory's sample 0. Native pybullet planning in the viewer's own client. |
| `husky_assembly_teleop/open_loop_engine.py` | The node: DPG UI, PyBullet view, RTDE tracker threads, gripper events, logging. Preview and execute modes in one script. |
| `scripts/make_wiggle_traj.py` | Writes a tiny test trajectory **starting at the arms' live pose** (open/hold + wrist wiggle + close/hold cycles). Refuses a powered-off arm, whose `getActualQ` reads all zeros. |

Run it:

```bash
cd ~/ros2_ws && source venv/bin/activate && source install/setup.bash
export ROS_DOMAIN_ID=86 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
ros2 run husky_assembly_teleop open_loop_engine <traj.json> --swap-arms                       # preview
ros2 run husky_assembly_teleop open_loop_engine <traj.json> --swap-arms --execute \
    --end-sample 640 --err-abort 0.15                                                          # hardware
```

UI flow in execute mode: **Connect RTDE → Plan approach to start → Preview approach →
Execute approach → Check start pose → START TRACKING.** **Space** (or the PAUSE / RESUME
button) coasts both arms to a standstill over half a second and back again, without leaving
the plan. STOP, closing the window and
Ctrl+C all speedStop both arms; a per-joint tracking error over `--err-abort` aborts both.
Every run writes `log.npz` (full-rate q_ref/q_actual/qd_cmd **and per-arm `wrench`/`mode`/
`speed_scale`**), `run_info.json` and `plots.png` next to the input JSON, with the approach
logged separately.

### Execution speed slider (added 2026-09-10)

**`execution speed x`**, under the cutoff slider, plays the trajectory at 0.1× to 2× — drag
it before or *during* a run. It scales the one clock both tracker threads read, so the arms
stay in lockstep however it moves (measured: 3 ms apart at 0.5×); the feed-forward is scaled
with it, so tracking quality is unchanged rather than being left to the P term. Both the
trajectory and the planned approach are scaled.

Three things to know:

- **It is capped at connect** to `--max-joint-vel / max|qd_ref|`, printed in the log and in
  the cutoff readout as `speed x0.50 (max x1.31)`. Above that the reference would be clipped,
  which distorts the path rather than slowing it.
- **Insertions ignore it.** A `--insertions` mate is force-controlled against the world and
  its contact thresholds, stall timer and budget are all in real seconds, so it always runs at
  1×. A change made during one takes effect at the next tracking phase.
- **Everything measured in trajectory seconds stretches with it** — the cutoff brake, the end
  settle, the post-insertion resume blend. At 0.1× a 0.4 s brake is 4 s of wall time. That is
  intended: the whole motion slows down together.

`run_info.json` records `speed_scale_at_start`, every mid-run change with its trajectory time
(`speed_scale_changes`), and the cap. The math is in [controllers.md](controllers.md) §1.3.

### Pause: the Space bar (added 2026-09-10)

**Space**, or the **PAUSE / RESUME** button next to START, walks the shared clock's rate to
zero over `PAUSE_RAMP_S` = 0.5 s. The arms decelerate along their own path and stop; pressing
it again walks the rate back up to whatever the speed slider is asking for and the run
carries on from exactly where it stopped. Everything the speed-slider section above says
applies, because a pause *is* a speed change — lockstep is kept, the feed-forward is scaled
with it, and the trajectory time it covers while braking is half the ramp (0.25 s at x1).

- **A pause is not a stop.** At the bottom of the ramp both arms are still under `speedJ`,
  held on the frozen reference by the tracker's P term, and the status line says so. **STOP**
  (or the pendant) is what actually releases them. Space is the "wait a moment" button, STOP
  is the "something is wrong" button.
- **Tracking phases only** — the trajectory and the planned approach, which both ride the
  shared clock. During a `--insertions` mate Space is refused with a log line: that skill is
  force-controlled against the world with thresholds and timers in real seconds. Use STOP,
  the held-state buttons, or the pendant there.
- **It works from either window.** DearPyGui only sees the key when the control panel has
  focus, so the 3D view's own keyboard is polled as well. Holding the key down does not
  toggle twice: the DPG side ignores auto-repeat until the key is released, and PyBullet's
  side latches the press edge.
- `run_info.json` records every pause as `{'paused_at', 'resumed_at'}` in trajectory seconds,
  alongside `pause_ramp_s`.

! **The ramp lives in `TrajClock`, not in the UI tick, and that is not a stylistic choice.**
DearPyGui blocks on the display's vsync, and when its window is not actually being presented
that is **one frame per second** — measured here, on this machine, with a bare 20-line DPG
script (`render_dearpygui_frame` at 999 ms; 0.1 ms with `set_viewport_vsync(False)`). The
whole 20 Hz supervision tick runs at that rate, so a ramp stepped forward on each tick would
have collapsed into a single jump to zero: the abrupt stop the feature exists to avoid. The
clock instead stores the ramp as its end points and works the rate out from the wall clock on
every read, so both tracker threads see it smoothly at their own 125 Hz. The flow test asserts
this directly — it checks the rate is halfway down after sleeping half the ramp **without
spinning the node once**.

! **The same stall makes gripper events fire late, it is INTERMITTENT, and it is NOT fixed.**
Events, the marking stop and the STOP button are all dispatched from the UI tick
(`_tick_execute`), so on a 1 Hz UI a close can land up to a second after its planned
trajectory time. On 2026-09-10 flow-test case 4 failed on exactly that (`fired_t 3.75`
against a planned `3.35` at x0.5, and a 60 s run for a 4.95 s file) — and failed the same way
with this work reverted, so it is not the pause's doing. **On 2026-09-11 the same suite ran
with a healthy tick and case 4 passed** (`fired_t 3.35`, 13.9 s). Same machine, same code:
whether the window is actually presented is what decides it, so **a case-4 gripper-timing FAIL
is usually this — re-run before treating it as a regression, and do not loosen the
tolerance.** Whether the operator's real session is at 1 Hz was never established; the window
is visible and focused there, which is normally when vsync tracks the display's 60 Hz.
**Check the tick rate on the robot before a run that leans on gripper timing**, and if it is
slow, the cure is `dpg.set_viewport_vsync(False)` in the backend — at the cost of a busy
render loop for every monitor in this repo, which is why it was not done unilaterally.

### Stop after each first pick (`--stop-after-grasp`, added 2026-09-11)

The paper layout is only as good as its fit — the first live run put the real grasp ~4 mm off
the part. Rather than chase that, this mode lets the ROBOT define where the parts are: run the
plan as computed, and right after a gripper closes on a part for the **first** time the run
freezes with the part still in the closed jaws on the sheet. Trace its outline, press
**DONE** (or Space), and the run carries on to the next first pick. From then on the parts go
on the traced outlines and nothing on the planner side changes.

- **Which closes.** `OpenLoopTraj.first_pick_part(ev)`: the part the close grabs (from the
  file's `attached_objects`, at the close sample or the one after) has not been attached to
  ANY robot at any earlier sample. On `open-loop.json` that is 5 of the 13 closes — obj_0
  @6.70 s (right), obj_1 @13.85 (left), obj_2 @34.45 (left), obj_3 @85.80 (left), obj_4
  @105.85 (right). The 4 handover receives (the giver held it earlier), the 3 re-closes on a
  part already in the jaws and the 1 re-pick of the assembly the robot set down itself all run
  through: the robot put those where they are.
- **It IS the pause.** The stop calls the same `_pause` the Space bar uses, so everything in
  the section above applies: both arms stay under `speedJ` on the frozen reference, STOP or
  the pendant releases them, a hard shove past `--err-abort` aborts the run. The one
  difference is the ramp: the freeze is **instant** (`ramp_to(0, 0)`), because the planner
  parks both arms for its `grasp_wait` dwell after every close (verified ≥ 0.40 s on the real
  file, reference speed ≤ 0.009 rad/s at the close) — there is nothing to ramp down, and a
  0.5 s ramp would carry the reference 0.25 s past the grasp pose. Resuming ramps up as usual.
- **Controls.** `--stop-after-grasp` sets the panel checkbox *stop after each first pick*,
  which is read live at every close, so it can be flipped mid-run. **DONE** is the same
  resume as Space; it exists so there is one obvious thing to press. DONE with nothing
  stopped only warns. An operator pause already ramping when a first pick fires wins (the
  stop's label is dropped; the arms are held either way).
- **Record.** `run_info.json` `pauses[]` entries carry a `reason`: `'operator'` for Space, or
  `'grasp of obj_0 (right)'`-style for a marking stop.

! The stop is dispatched from `_fire_gripper_event`, i.e. from the same UI tick as the
gripper command, so the 1 Hz DearPyGui hazard above delays it too. At 20 Hz it lands ≤ 50 ms
after the close, well inside the 0.40 s dwell, and the freeze is at the trajectory time the
close actually fired — but on a stalled UI the close itself is already late and the arm may
have lifted. Check the tick rate on the robot first.

! A `MISSED` verdict (fingers met nothing) still aborts the run while it is frozen, unless
`--no-grasp-abort`. With a part a few millimetres off, expect it — that is what the tracing
is for.

### Zeroing the force sensors (automatic at connect, added 2026-09-11)

**Connect RTDE now zeroes both arms' force/torque sensors** before anything else runs, so
every force the run reports is measured from the parked arms rather than from whatever bias
the sensors woke up with. The log shows what it did, per arm:

```
left FT zeroed (at connect): [ -3.8  1.2 -11.4 ...] -> [0. 0. 0. ...] (base frame)
```

`zeroFtSensor` makes the CURRENT reading the new origin, so what gets subtracted is the tool's
own weight, the cable pull, and anything in the jaws at that moment. `--no-zero-ft` skips it
and leaves the existing bias alone.

**Zero force sensors (both arms)** under the Connect button does it again on demand — after a
tool swap, a moved cable, or a drift you notice in the Wrench window. **The most useful moment
is right before START**, once the approach has run: the arms are then standing in the
trajectory's own start pose, whereas the connect-time zero was taken wherever they happened to
be parked, in a different wrist pose and so under a different tool-gravity load. It is refused while a
run is in progress: the hold guard (`_check_hold_wrench`) measures a *change* since its hold
began, and moving the origin underneath it would either mask a real jump or invent one.

- ! **A zero is only right for the pose it was taken in.** Gravity on the tool changes with
  the wrist, so a zero taken at the park pose drifts as the trajectory moves. This is why a
  compliant insertion re-zeroes for itself, with the part already held, at the start of every
  mate (`open_loop_insertion.py`, the `zero` phase) — that is a separate, later zeroing and it
  overrides the connect-time one from then on.
- Both arms are zeroed together so their numbers stay comparable.
- If a sensor still reads more than 5 N/Nm right after zeroing, the run warns: an arm was
  moving or being touched while it happened.
- `run_info.json` and `insertion_wrench.json` both carry `ft_zeros` — every zeroing with its
  reason, timestamp, and the before/after wrench of both arms, so a force trace can always be
  traced back to the origin it was measured from.

### Insertion wrench log (checkbox, ON by default, added 2026-09-11)

Every mate in `--plan-json` gets **both arms'** force and torque written out at the end of a
run: `insertion_wrench.json` plus one `insertion_wrench_<k>_<part>.png` per mate, in the same
folder as `log.npz`. The window runs from the **funnel mouth** (the pre-insertion pose, where
the straight approach begins) to the **gripper-open that releases the part**, with
`WRENCH_LOG_PAD_S` = 1.0 s of quiet on either side so the baseline is visible. Both boundary
times are in the record, so the pad can be trimmed when parsing.

The panel checkbox *log insertion wrench (json + png)* starts on; `--no-wrench-log` starts it
off. It is read once, when the run is saved.

- **It does NOT need `--insertions`.** The mates are located from the plan's `assemble`
  actions (`find_insertions`), which is pure analysis, so a plain open-loop run — the one that
  actually needs measuring — is logged the same way. `ran_compliant` in each record says which
  kind it was. `--plan-json` is required, since that is what names the mates.
- **Both arms, always.** `role` is `inserting` / `holding`: the inserting arm feels the joint
  going together, the holding arm feels the same push arriving through its own grasp.
- **No cost during the run.** The samples are sliced out of the tracker's own 125 Hz log at
  save time, so nothing extra runs in the control loop.
- **A compliant mate is covered too** — checked: through a `--insertions` press both arms stay
  continuous at the full rate (8.1 ms median step, 11 ms worst). `ran_compliant: true` marks
  those records; the skill's own richer view of the same press — low-passed wrench, depth,
  search radius, phase — stays in `insertions.json` and `insertion_<k>.png`.
- ! **Time can step backwards by a few milliseconds at a phase boundary**, because the shared
  clock is rebased when a phase relaunches (a couple of such steps per mate). Sort by time, or
  segment on the negative steps, if a parser cares.
- **Time base** is trajectory seconds, the same clock as `log.npz`'s `*_t`, so a record lines
  up with the joint and tracking-error traces of the same run. It *freezes* during a pause or
  a marking stop, so wall time and trajectory time are not the same thing here.

```python
import json
d = json.load(open('insertion_wrench.json'))
for rec in d['insertions']:                     # one per mate
    arm = rec['arms'][rec['inserting_arm']]     # or rec['holding_arm']
    t, f = arm['t_s'], arm['force_N']           # [s], [[fx, fy, fz], ...] in N
```

Forces are relative to the last zeroing (see the section above); `ft_zeros` in the same file
says when that was and what was subtracted.

! **The force frame is asserted from UR's docs, not verified on Cindy** — the same caveat as
trap 0b below. `getActualTCPForce` is documented as the wrench at the TCP in that arm's BASE
frame, and a wrong pendant tool offset puts it at the wrong point entirely. Treat the numbers
as relative until somebody pushes each tool along base +x/+y/+z by hand and writes the signs
down here.

### Seeing the parts (`--layout-json`, `--plan-json`)

Give the engine the cell layout and the planner's plan and the 3D view draws the parts
themselves, from their real meshes: grey on the table, **orange** in a gripper, **green**
once mated. Works in preview and execute.

```bash
ros2 run husky_assembly_teleop open_loop_engine <traj.json> --swap-arms \
    --layout-json <layout_nominal.json or the measured one> --plan-json <plan.json>
```

The layout supplies the poses, the plan says which gripper-open is a mate rather than a
set-down. Without `--plan-json` a mated part is simply left where it was released. **The
layout must belong to the same solve as the trajectory** — the startup log prints each
part's timeline and shouts if the gripper closes outside a part's own box, which is what a
table height from another cell looks like.

### Jaw fit: `[jaws]` lines at startup

The fingers used to snap to fully closed on every grasp, which drove the meshes straight
through the part and made the picture useless for judging table clearance. Now each grasp is
fitted once at startup — the knuckle angle is walked shut until the pads meet the part — and
that angle is what the view shows from then on, in preview and execute alike. One line per
grasp:

```
[jaws] left closes on obj_1 @t=6.61s: knuckle 0.531 rad, jaw 27.9 mm, pad tip +6.4 mm above the table
```

`pad tip` is the closest distance from either finger pad to the **layout's** table box at the
fitted angle, so it already contains the four-bar drop (closing swings the inner finger
~13 mm further along the approach axis). A negative number is logged as a WARNING: the
closing fingers would sweep the table. It is a *planned* number against the layout's table
height, so read it alongside the planner's own MuJoCo pad-tip probe, not instead of it —
different table, and neither one sees tracking error.

`opening` is the real pad-to-pad gap, surface to surface (83 mm with the jaws fully open,
against the 2F-85's 85 mm stroke). It reads the pinched thickness plus PyBullet's collision
margin, ~1 mm per side: a leg whose body is 22 mm reads 24.5 mm.

Two caveats worth knowing:

- The parts are **exact triangle meshes**, not convex hulls (changed 2026-09-10). The hull was
  a lie on a leg: its snap-fit lug dragged the hull out along the whole length, so the pads
  stopped up to 4 mm per side early — 30.2 mm of "opening" on a 22 mm leg. The seat's pockets
  are real cavities now too, so a grasp can legitimately close on a 12.6 mm rib inside a
  35 mm-thick part; the part's bounding extents are a ceiling on the opening, never a floor.
- The fit uses the **planned** grasp, like everything else `PartTracker` draws. It does not
  see slip. `closed on nothing` in the log means the jaws reach full close without meeting
  the part at all — the planned grasp holds nothing there.

The command sent to the real gripper is unchanged: still a full close, which stalls on the
part. Only the drawn angle is fitted.

### Compliant insertion (`--insertions`, OFF by default)

Without the flag nothing changes: every mate is replayed open-loop by the speedJ tracker, as
before. With `--insertions` (needs `--plan-json`) each mate is handed to the force-controlled
skill in `open_loop_insertion.py` instead — approach, spiral search, press, one bounded
back-off — while the other arm holds the part being inserted into under a force guard.
A run becomes `track → insert → release → track`.

```bash
ros2 run husky_assembly_teleop open_loop_engine <traj.json> --swap-arms --execute \
    --plan-json <plan.json> --insertions --ins-push-force 10 --ins-guard-force 30
```

Sliders (read live at the start of each insertion): push force, search radius, approach
speed, plus a "skip insertions" toggle. If a mate does NOT seat, both arms park in the
**held** state and wait — nothing opens a gripper on a part that is not in its joint. Three
buttons decide: **Retry insertion**, **Release & continue (UNSEATED)**, **Abort run**.

The Wrench window streams both arms' force and torque all the time; the control panel's role
lines say which arm is INSERTING and which is HOLDING. Each run also writes
`insertions.json` and one `insertion_<k>.png` per mate (depth, forces, search radius, phases).

**Before the engine, use the bench script** — one insertion, one arm, no trajectory:

```bash
python3 scripts/bench_insertion.py --arm left --dry-run     # prints pose, wrench, direction
python3 scripts/bench_insertion.py --arm left --depth-mm 30 --axis tool-z
```

## What will bite you

**-1. The Robotiq mount: a quarter turn AND an 11.2 mm coupling (2026-09-10).** The real
grippers are bolted on a quarter turn, so the pads close along tool0 **X**, not tool0 Y; and
they sit on a Robotiq coupling, which the planner's model left out entirely (`t(0 0 0)`) and
this engine had at a made-up 12 mm. Both now use `utils.ROBOTIQ_COUPLING_M = 0.0112`, derived
in that file from the 2F-85 manual's own figures; the planner has the same number in
`src/rai/husky/calibrated/husky_calibrated.g`, with the hand-typed copy in
`analytic_ik._CAL["husky_*"]["tools"]["two_finger"]` pinned by `tests/test_husky.py`.

**Any plan/mp/open-loop file solved before that date is wrong on both counts and must be
re-solved** — the grasp-DB hash does NOT change, so `--load-plan` will accept the old file
without complaint. The 11.2 mm is also what the `[jaws]` clearance was missing: leg picks read
−10.9 mm (below the table) before and +0.6 mm after, against the planner's own MuJoCo pad
probe at +1.8 mm.

`common.create_end_effector("robotiq_gripper")`, the static closed-mesh proxy the OTHER
monitors draw, was left at the old orientation and with no coupling; it does not feed this
engine.

**-0.7. Every gripper event `SKIPPED-no-server` (2026-09-10).** The engine's action clients
never saw `/a200_0806/<side>_gripper/robotiq_gripper_controller/gripper_cmd`. The Husky's
gripper stacks were running the whole time; the terminal simply lacked
`ROS_DOMAIN_ID=86 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` (see rtde_network_setup.md, "ROS 2 /
DDS to the husky"). Verified the same afternoon: with the env set, `ros2 action send_goal`
closes the left gripper (stalls at 0.789 rad on nothing) and opens it fully (0.0035 rad,
`reached_goal`). The engine now shows the env in its banner and START refuses to run a live
trajectory while a gripper server is missing (`_grippers_reachable`); `--no-gripper` is the
explicit way to run without them.

**-0.5. "the live configuration is already in collision" on a visibly clear arm (2026-09-10).**
The approach planner checks the arms against the layout's part BOXES with the VIEW's finger
state, and both can differ from reality. Ranked by how often they bite:

1. **Model fingers vs real fingers.** The check uses `grip_viz_angle`, i.e. whatever the 3D
   view last showed -- after a finished run that is the last *fitted close* (12-25 mm), not
   the open real jaws; in preview it follows the playhead. Measured at the first leg pick: the
   pose is clear with the jaws open or at 0.426 rad and **15.8 mm deep** with them closed.
   The engine now logs the angle it checked with (`[approach] checking with the model
   fingers at L .. / R .. rad`). If it is not ~0, open the grippers (real and view) and plan
   again.
2. **A footprint box is fatter than the part.** `build_obstacles` boxes a leg at 105 x 30 x
   22 mm (+2 mm float); the leg body is 22 mm wide except at the lug. A fingertip beside a
   leg can read several mm "deep" in the box and be clear of the real leg. Deliberately
   conservative for planning; wrong as a verdict on a parked arm.
3. **Layout / table height.** The box stands where the calibrated layout says the part is
   and on `table.top_z`; a part placed a few mm off its printed outline, or a table height
   off by a few mm, moves the box by exactly that.
4. **Gripper length.** The teleop tip mesh is ~2 mm longer than Robotiq's drawing; a legacy
   coupling on the arm instead of GRP-CPL-062 would make the model 3 mm too long.
5. **The robot model itself** -- checked and CLEAR: at identical joints the PyBullet URDF and
   the planner's rai model put both flanges within 0.0 mm of each other.

Two scripts turn this into numbers, without touching the robot's program:

```bash
# ros2 venv: reads RTDE joints + the UR's own TCP pose, rebuilds the engine's world, reports the
# verdict at open / 0.426 / closed fingers, against boxes AND true meshes, where the tips stand,
# and the UR-controller FK vs PyBullet FK. --traj/--sample stands in for the robot offline.
python3 src/husky-assembly-teleop/scripts/husky/live_pose_check.py --layout-json <layout.json> --out /tmp/live_q.json
# planner venv, from ~/Code/fixtureless-assembly: the same joints in the planner's model --
# its certified collision proxies (what MuJoCo mirrors), joint limits, flange positions, --view.
python3 /home/su/ros2_ws/src/husky-assembly-teleop/scripts/husky/live_pose_in_planner.py --plan <plan.json> --q-json /tmp/live_q.json --view
```

**0. The per-sample `servo_controller` flag does NOT mark the insertions.** It is
`controller_mode != 'trajectory'` in the sim export (`export_open_loop.py:357`), which bundles
the settle, wrench-guard, approach-pacing and place-servo windows. In `open-loop.json` it is
true on 1110 of 3895 samples across 57 windows, and the four leg→seat mates contain **none**
of them. The mates are found from the plan's `assemble` actions instead
(`open_loop_traj.find_insertions`), each tied to the gripper-open that releases that part.
Two further traps found while doing it: a mate's straight line is straight **in the holding
arm's frame**, not the robot base (the other arm is moving too), and the mate belongs to a
part's **last** open — an earlier one is a handover give. Funnel lengths in `open-loop.json`
come out 51 / 8 / 48 / 32 mm, because the planner shortens a funnel when the full 50 mm will
not plan; the 8 mm one is under `--ins-min-depth` and stays on the tracker.

**0b. The FT frame is asserted from UR's docs, not verified here.** `getActualTCPForce` is
documented as the wrench at the TCP in the **base** frame. Nothing in this repo has confirmed
the signs on Cindy. `Connect RTDE` now logs each arm's pendant TCP offset and its raw wrench
— **push each tool by hand along base +x/+y/+z and check the signs before trusting any force
number**, and write the result here. A pendant tool offset that is not the gripper's puts the
forces at the wrong point entirely (see the `ur-pendant-tcp-offset` note).

**1. `--swap-arms` is mandatory for the planner's files.** a1 → RIGHT arm (.41), a2 → LEFT
(.40). The initial assumption was the opposite and it is wrong. Re-verify per new file with
geometry, not eyeballs: at each near-simultaneous open/close pair on opposite arms (a
handover, both grippers on one bar) the two tool0 origins must nearly meet — swapped gives
0.30-0.43 m, unswapped 0.77-1.88 m.

**2. `pybullet_planning` can only ever address the FIRST PyBullet client of the process.**
It copies `CLIENT` into every submodule at import (`get_num_joints` does
`physicsClientId=CLIENT`), so neither `pp.set_client()` nor assigning `pp.CLIENT` retargets
its helpers. Symptom: `ValueError: (4, 'left_ur_arm_shoulder_pan_joint')` from
`joints_from_names`. Corollary: plan natively with pp **in the viewer's own client**
(0.9 s for this 12-DOF search). Do NOT put a compas_fab RobotCell in the GUI client — its
`check_collision` re-pushes the whole cell state per sample, and a 4.6 s headless search
ran past 15 minutes that way. `utils.plan_transit_motion` already has the
`dual_arm_index='both'` branch; wrap the call in `pp.LockRenderer()` because that branch
deliberately leaves the renderer unlocked (`utils.py:425`, a leftover cfab-debug hack).

**3. The approach plan must be re-validated.** BiRRT only tests the configurations its
extend step produces and its shortcutting widens those gaps (one 0.05 rad step is ~45 mm of
tool travel at the shoulder — wider than a bar). `validate_path` re-walks at 0.005 rad and
REJECTS, failing closed on exceptions. Likewise the timed reference is built by densifying
the polyline (0.02 rad) rather than splining through the waypoints: a spline cuts corners
by 0.05-0.27 rad and can leave the checked path, densify+Hermite keeps it within 0.003 rad.

**4. A cutoff usually lands mid-motion.** The reference brakes over 0.4 s instead of
jumping to a hold, so the arm decelerates along its own path. Cutting at sample 600 of
`open-loop.json` (0.73 rad/s there) travels 148 mrad while braking; sample **640** (0.02
rad/s, t=31.95 s) travels 0.3 mrad and keeps the same 4 gripper events. The slider readout
flags a fast cutoff — prefer a still moment.

**5. Ops, every time** (details in [rtde_network_setup.md](rtde_network_setup.md)): stop the
UR drivers AND `multi_arm_safety_sync` (it re-loads `ros_control.urp` and kills RTDE's
uploaded script); relaunch the gripper-only stack with `start_tool_communication:=true`
because the Robotiq Modbus bridge dies with the arm driver; pendants to **Remote**; the
laptop's `cindy` NetworkManager profile (static 192.168.131.19/24, no gateway) reaches all
three IPs on one cable; DDS needs domain 86 + cyclone + `CYCLONEDDS_URI`, and `ros2 daemon
stop` after changing any of those or the CLI keeps using stale settings and silently shows
nothing.

**6. Deferred bug: the grippers run at ~15% speed.** `write()` in the lab's
`ros2_robotiq_gripper` fork folds `kGripperMaxSpeed` into the speed state before scaling to
the 0-255 register, pinning it at 38/255 forever — no config can raise it. Two-line fix and
rebuild recipe are in [rtde_network_setup.md](rtde_network_setup.md); the user chose to
defer it.

## Verification status

- **Committed test scripts** (all headless, all green as of 2026-09-09):
  `scripts/test_open_loop_parts.py` (17 checks: part timelines, re-parenting continuity to
  0.15 um, handover vs mate, layout-mismatch detection, `_draw_parts` itself; `--png` renders
  the cell), `scripts/test_open_loop_insertion.py` (24 checks: the skill against a virtual
  mortise at 0 / 3 / 12 mm error, a fouled pocket, the force guard, plus `find_insertions` on
  the real file), `scripts/test_open_loop_engine_flow.py` (the engine itself through
  track → insert → release → track → done, and the held-on-failure path).
  `scripts/fake_rtde.py` is the shared fake RTDE pair + contact model.
- **Fake-RTDE harness** (integrates commanded velocity perfectly): full execute path —
  state machine, start-pose gate, lockstep logs, rising-edge gripper events, err-abort
  stopping BOTH arms, STOP/abort saving, cutoff braking, and the whole approach flow
  (refuse → plan → preview moves nothing → execute → auto re-check → START → STOP).
- **FT zeroing** (2026-09-11, fake RTDE): flow-test case 7 checks the button records both
  arms, leaves them reading zero, is refused while a run is in progress, and lands in
  `run_info.json`. The automatic zero at Connect has NOT been exercised (the tests attach
  fake arms instead of connecting).
- **Insertion wrench log** (2026-09-11, fake RTDE): flow-test case 3 checks that a plain
  open-loop run writes `insertion_wrench.json` with both arms, ~full-rate samples spanning
  pre-insertion to release, and its figure.
- **Marking stop** (2026-09-11, fake RTDE): `test_open_loop_engine_flow.py` case 1 pins
  `first_pick_part` on the real file (5 of 13 closes), case 6 runs the pick slice (samples
  100..300) through two stops and two DONEs, case 7 the same slice with the mode off, and
  case 5 proves a re-close on an already-held part does not stop the run. Not yet on hardware.
- **On hardware**: the wiggle test ran fine on Cindy with real gripper commands. The
  **approach planner and the cutoff have NOT yet run on the real robot**, and neither has
  any full assembly trajectory.
- ! The harness scripts live in this session's scratchpad
  (`/tmp/claude-1000/.../scratchpad/test_{approach,approach_flow,cutoff,execute_fake}.py`)
  and will be garbage-collected. Worth moving into `scripts/` if this work continues.
- ! Do not pipe long test runs through `grep` — it block-buffers, and a killed run loses
  every line. Redirect to a file and grep the file.

## Next steps

1. On the robot: preview, then Connect → Plan approach → Preview approach → Execute
   approach → Check → START with `--end-sample 640` and a tight `--err-abort`. Watch the
   tracking-error plots; tune `--p-gain` per arm if needed (Valentin used L 0.5 / R 2.0,
   the engine defaults to 1.0/1.0).
2. Then extend the cutoff toward the full 3895 samples.
3. ~~Route the flagged segments to a compliant controller.~~ Done, but as `--insertions`
   driven by the plan's mates rather than the flag (see trap 0). **Not yet run on hardware:**
   `scripts/bench_insertion.py` first, then `--insertions --end-sample` just past the first
   mate, then the whole file.
4. The placeholder table is a guess (top at 0.245 m = half the 0.490 m arm-base height, in
   front of the chassis). Replace it with `--layout-json` from `pickup_calib` once the
   workspace is surveyed.
5. The engine holds the parent-holding arm STILL through a mate, while the plan may move it
   (38 mm on obj_3 of `open-loop.json`). The mate is unaffected, but the assembly ends that
   far from its planned world pose; `find_insertions` warns when it is over 10 mm.
