# Pickup calibration manual — run guide

**Status: 2026-09-10.** Stages 1-6 have run end to end; one real measurement take exists
(2026-09-09, 0.34 mm rms) and stage 3b re-checks it. Stage 7 has run on hardware once, with
the gripper commands never reaching the robot -- stages 7a/7b are the result.

You are telling the planner where the parts really are. The short version: print one A4
sheet, put the parts on their printed outlines, touch four printed crosses with the punch
tip, and re-solve. Each stage says what to run, what you should see, and what to do when it
goes wrong.

Two repos are involved. Set both up once per terminal:

```bash
# PLANNER  ("planner env" below)
cd ~/Code/fixtureless-assembly && source .venv/bin/activate
export E=experiments/real-stool-husky-current      # the 3-part real-bench

# ROBOT SIDE  ("robot env" below)
cd /home/su/ros2_ws && source venv/bin/activate
python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop
source install/setup.bash
```

---

## Stage 1 — make the sheet  (planner env)

```bash
python3 export_layout.py $E/plan.json --out $E/layout_nominal.json --svg $E/layout_outline.svg
```

Expect `PACKED onto one sheet` and `1 A4 page(s) at 1:1`.

**What packing does, and why it matters.** The planner scatters parts ~240 mm apart, wider
than A4 even though the parts are small. So the exporter *re-places* them into a compact
block that fits one sheet. That means **the parts have MOVED and the plan no longer matches
the layout** — stage 4 re-solves for the new positions. `--no-pack` keeps the planner's own
scatter and splits across several sheets instead.

## Stage 2 — print and lay out

**Print `$E/layout_outline.pdf` at 100% / "actual size".** Not "fit to page" — that silently
shrinks it and every part then sits in the wrong place. Measure the printed 100 mm scale bar
with a ruler before trusting the sheet.

Lay the sheet flat with the **"towards the robot" arrow** pointing at the robot, and place
each part on its outline. The outlines are traced from the exact part meshes — snap-fit joint
included — so a part only sits one way. Note one leg is drawn turned 90°; that is deliberate,
it is what makes the set fit one sheet.

Keep the **four red crosses** clean and reachable; they are what you measure next. Nothing is
cut or taped: it is one sheet.

**If the sheet turns out not to be accurate enough** (the first live run grasped ~4 mm off the
printed outlines), let the robot mark the parts instead of chasing the fit: run Stage 7c with
`--stop-after-grasp`. The run freezes right after each part is picked up for the first time,
with the part still in the closed jaws on the sheet — trace around it with a pen, press
**DONE**, and it continues. Handovers and re-grasps do not stop it. From then on place each
part on its traced outline, not the printed one; nothing changes on the planner side.

## Stage 3 — measure  (robot env)

Before you start: the punch is on the arm and its TCP is in
`data/calibration_data/20260622/config.yaml` under `punch_tool.left.offset_xyz` (metres, from
the pendant's 4-point wizard). Current left value: `[-0.00325, 0.00048, 0.11832]`.
**Both pendants stay in LOCAL / free-drive** — this tool only reads.

```bash
ros2 run husky_assembly_teleop pickup_calib \
    ~/Code/fixtureless-assembly/$E/layout_nominal.json --arm left
```

`--offline` rehearses the buttons with joint sliders and no robot.

1. **Connect RTDE (read-only)** — both arms then mirror your free-driving.
2. Choose a **mark**. Its red sphere in the 3D view grows and is labelled `<-- touch this`.
3. Free-drive so the punch tip sits in that cross on the paper. **Record mark point.**
4. Repeat for all four. Three is the minimum for a meaningful error estimate — with two the
   rms is 0 by construction and tells you nothing.
5. **Fit sheet (moves every part)** — reports a shift, a turn and an **rms in mm**. Expect
   ~0.5 mm. Over ~2 mm means a bad touch: **Undo last point** and redo that mark.
6. **Save layout** → writes `layout_nominal_calibrated.json` next to the input.

! **The table height comes from the same touches.** A cross is printed on the sheet and the
sheet lies on the table, so every mark touch measures the table top; `top_z` follows their
mean as you record them (readout: `table top: 0.4126 m from 4 touch(es), spread 2.9 mm`).
Before 2026-09-10 it did NOT -- only the table buttons applied it -- and the first real take
saved the planner's guessed 0.440 m against touches at 0.413 m: **every part 27 mm too high**.
If the readout still shows the planner's number after your touches, something is wrong; stop.

## Stage 3b — verify a saved calibration  (robot env)

The cheapest test of "is the calibration still right": load the calibrated file itself and
touch its crosses again.

```bash
ros2 run husky_assembly_teleop pickup_calib \
    ~/Code/fixtureless-assembly/$E/layout_nominal_calibrated.json --arm left
```

The title says **VERIFY / RE-CALIBRATE**. Touch the four crosses as in stage 3, then press
**Check drift vs saved crosses (changes nothing)**. Per cross it prints the xy distance from
where that cross was measured last time, the z difference from the last touch and from the
modelled table, and the rigid shift/turn/rms the new touches imply:

```
  p1-near-right  drift   0.4 mm  (dx   +0.3, dy   -0.2)  dz vs last touch   +0.1 mm  z vs modelled table   -0.2 mm
  ...
  rigid: shift (+0.3, -0.1) mm, turn +0.02 deg, rms 0.31 mm from 4 mark(s)
  => calibration HOLDS (all within 2.0 mm)
```

`HOLDS` means the sheet, the robot and the punch are where they were; look elsewhere for a
problem. `DRIFT` means press **Fit sheet** and **Save** -- saving overwrites the calibrated
file after keeping the old one as `layout_nominal_calibrated.bak-<timestamp>.json`, and the
re-solve from stage 4 follows. A large `z vs modelled table` with tiny xy drift is the
table-height bug above, not drift.

Reading the 3D view: red spheres are the crosses, solid meshes are current part poses,
translucent ghosts are the nominal ones. After a good fit the solid parts jump to where the
sheet really is.

## Stage 4 — re-solve for the measured cell  (planner env)

```bash
LAYOUT=$E/layout_nominal_calibrated.json

python3 main.py --solve --search-profile standard \
    --assembly real-bench --parts 3 --num-robots 2 --robot husky \
    --wrench --wrench-profile max_robotiq --insert-force 10 --mate-symmetry \
    --grasp-closure-check --grasp-sampler-margin 0.008 --grasp-sampler-pad-support 0.75 \
    --witness-sets 4 --seed 11023 --grasp-sampler-seed 1 \
    --layout-json $LAYOUT --check-init \
    --save $E/plan.json --no-view --quiet-rai
```

! **`--grasp-sampler-seed 1` is not cosmetic.** With the coupling in the model the cell is
tighter, and the default cloud (seed 0) picks a hold that grazes the Husky's own top plate:
the next stage refuses it with `exported attachment chain has 4 collision-invalid keyframe(s)
... obj_1 ~ husky0_top_coll_middle (1.06mm)`. If that message ever comes back, try another
`--grasp-sampler-seed` -- do NOT relax `--mp-collision-tolerance`, which only hides it.

! **Re-solve anything older than 2026-09-10.** Two mount errors were fixed that day: the
Robotiq is rotated 90 deg about the flange axis, and it sits on an 11.2 mm coupling the model
did not have. The grasp-DB hash does not change, so an older plan loads without complaint while
every one of its IK witnesses carries the old wrist roll and sits 11.2 mm too deep. Stage 4
onwards is the re-solve.

! **`$LAYOUT` must be the CALIBRATED file**, not `layout_nominal.json`. They differ by 45-60
mm here. Solving against the nominal sheet while the viewer draws the measured parts sends
the gripper 5 cm away from the part it closes on, and the PyBullet replay then shows the part
floating beside the fingers. `open_loop_parts.GRASP_MARGIN_M` is 50 mm ON TOP of the part's
half-extents, so it does NOT catch a miss this size -- check `layout_json` in the saved plan.

? **`--grasp-standoff M` is available and NOT used here yet.** It sets how far the tool centre
stands off the face it grasps (default 0.010). The Robotiq 2F-85 is a four-bar linkage, so
closing the jaws swings the fingertips ~13 mm further along the approach axis: at the default
the pads end up only 1.9-3.2 mm above the table on a leg pick. 0.015 lifted that to 6.6-6.7
mm on the NOMINAL layout, but that trial was never repeated against the calibrated layout, so
this command stays at the default until it is. Whatever value is used, it is SAVED into the
plan and every later stage rebuilds the same grasp database from it.

**`--check-init` is not optional here.** It reports in seconds whether every part is
collision-free and reachable. Watch for:

```
[flag] --layout-json ACTIVE: 3 measured part pose(s) ...
[scene-check] collision-free: True | graspable (real-IK) by >=1 robot: 3/3 parts
[scene-check] => scene OK
```

If it says `scene NOT USABLE`, **kill the run** — the search cannot succeed and will spend a
quarter of an hour proving it. See Troubleshooting.

## Stage 5 — motion plan and simulate  (planner env)

The layout travels inside the saved plan, so neither command needs the flag again.

```bash
python3 main.py --config $E/plan.json --load-plan $E/plan.json \
    --motion-plan --motion-profile standard --mp-runtime 10 \
    --mp-save $E/mp.json --quiet-rai --no-view

python3 sim.py run $E/plan.json --sim-profile standard --mp-path $E/mp.json \
    --apply-process-wrench --animate-grippers \
    --record $E/replay.npz --report-json $E/replay-report.json

python3 sim.py replay $E/replay.npz 2        # watch it; blocks until you close the window
```

Want `reaches terminal=True` from the motion plan, and `N/N actions ok; plan SUCCEEDED` from
the sim. A failed sim is a stop sign — stage 6 refuses it.

## Stage 6 — export the trajectory  (planner env)

```bash
python3 export_open_loop.py $E/replay.npz $E/open-loop.json --dt 0.05 --grasp-wait 0.5
```

`source success=True` means it accepted the run. `--allow-failed` only ever makes a
diagnostic file — never something to run on hardware.

## Stage 7a — bring up the grippers  (robot env)

The arms are driven over RTDE, so the ROS arm drivers must be OFF and only the two Robotiq
stacks run on the Husky. Both arms must be **powered on** (the grippers' 24 V comes through
the arm). Pendants can stay in LOCAL for this stage.

```bash
scripts/husky/start_gripper_stacks.sh --status     # what is running now (changes nothing)
scripts/husky/start_gripper_stacks.sh              # start both; --restart if they are stale
```

The script ssh's into the Husky itself, refuses to run beside the full `crl_dual_ur5e` stack
(two tool bridges fight over TCP 54321), and ends with a report: want `controllers activated:
3 of 3` for both arms and both `gripper_cmd` action servers listed.

Then, **in the terminal you will run the engine from**, the ROS environment -- all three, or
discovery is silently empty:

```bash
export ROS_DOMAIN_ID=86 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/.cyclonedds.xml
ros2 daemon stop                     # the CLI daemon caches the env of its first run
ros2 action list | grep gripper_cmd
```

You must see exactly these two:

```
/a200_0806/left_gripper/robotiq_gripper_controller/gripper_cmd
/a200_0806/right_gripper/robotiq_gripper_controller/gripper_cmd
```

! Only `CYCLONEDDS_URI` is in `~/.bashrc` today. Without the first export the engine runs in
domain 0 on FastDDS, sees no servers, and on 2026-09-10 executed a whole trajectory logging
`SKIPPED-no-server` for all ten gripper events while the Husky's stacks were up the whole
time. Add the line to `~/.bashrc` next to the `CYCLONEDDS_URI` one and this stage becomes a
check rather than a ritual.

## Stage 7b — prove you can command a gripper from this PC  (robot env)

! **This moves a gripper.** Do it on the arm whose gripper is clear of parts and table.

```bash
scripts/husky/close_grippers.sh left      # or right, or nothing for both
scripts/husky/open_grippers.sh left
```

Both scripts set the ROS env themselves (unless your shell already has it), send the same
GripperCommand goal the engine sends (close 0.8 rad / open 0.0 rad), and print one line per
arm. Expected (measured 2026-09-10 on the left gripper):

```
left close -> position 0.789 rad  stalled true   reached_goal false
left open  -> position 0.003 rad  stalled false  reached_goal true
```

The 0.789 is the empty-close signature -- the fingers met each other with nothing between
them; it is the number the engine uses to tell GRASPED from MISSED. A part between the pads
stops them earlier. `NO ANSWER in 20s` means stage 7a is not done (env or stacks).

Do not start the engine until this works: it now refuses START when a gripper server is
missing, and `--no-gripper` is the only way past that.

## Stage 7c — execute  (robot env)

Pendants to **Remote** for `--execute` (RTDE control); stages 3/3b wanted LOCAL.

```bash
ros2 run husky_assembly_teleop open_loop_engine \
    ~/Code/fixtureless-assembly/$E/open-loop.json \
    --swap-arms --layout-json ~/Code/fixtureless-assembly/$LAYOUT          # preview first
    # ... then add --execute
    # ... and --stop-after-grasp to trace each part where the robot grasps it (Stage 2)
```

**`--swap-arms` is mandatory.** The export lists robots as `["a1", "a2"]` where `a1` is the
RIGHT arm, but the loader maps the first robot to the LEFT arm by default. Without the flag
the arms are exchanged.

`--layout-json` makes the approach planner avoid the measured table and the real parts rather
than a guessed table. Ops prerequisites (stopping the arm drivers, relaunching the gripper
stack) are in [rtde_network_setup.md](rtde_network_setup.md).

The startup banner must read `gripper=ROS [ROS_DOMAIN_ID=86, RMW=rmw_cyclonedds_cpp]`. Button
order: Connect RTDE → Check start pose → Plan approach → Preview approach → Execute approach →
START TRACKING. **Connect RTDE also zeroes both force/torque sensors** — check the two
`FT zeroed` lines in the log. Press **Zero force sensors** again right before START, once the
approach has run and the arms are in the trajectory's own start pose; the connect-time zero
was taken in whatever pose they were parked in. With `--stop-after-grasp` (or the *stop after each first pick* checkbox) the
run freezes after each part's first pick — trace the outline, then **DONE** (or Space); the
arms are still under speedJ meanwhile, so keep the shove gentle. STOP, closing the window and
Ctrl+C all speedStop both arms. Every gripper
event should log `status: 'sent'` and then a `GRASPED` / `OPENED` verdict; a `MISSED` close
stops both arms unless `--no-grasp-abort`.

Every run also writes `insertion_wrench.json` plus a figure per mate next to the trajectory:
both arms' force and torque from the pre-insertion pose to the release. It is on by default
(the *log insertion wrench* checkbox; `--no-wrench-log` starts it off) and needs `--plan-json`
to know where the mates are.

## After the experiment

In this order -- the grippers depend on the arms being powered, so they come down first:

1. **STOP** in the engine (or close its window / Ctrl+C): both arms speedStop. Leave the
   arms where they are; do not free-drive them while the engine is still connected.
2. **Release anything still held**, so nothing drops when power goes: with the engine closed,
   `scripts/husky/open_grippers.sh left` (or `right`) on the arm that holds a part, hand
   under it, and take the part out.
3. **Close the engine.** Its run log is already written to
   `$E/open-loop-trajectory-<timestamp>/` (`run_info.json` has every gripper event and its
   verdict, `plots.png` the tracking); keep those folders, they are the record of the take.
4. **Stop the gripper stacks:**
   ```bash
   scripts/husky/stop_gripper_stacks.sh            # kills only the two gripper sessions
   scripts/husky/stop_gripper_stacks.sh --status   # confirm: no gripper_left/right, no servers
   ```
   It never touches the Clearpath platform node. Do this BEFORE anything that takes an arm
   down, or the tool bridges die noisily and leave a stale `/tmp/ttyUR_*` behind.
5. **Pendants back to LOCAL**, then power the arms and the Husky down the way the lab always
   does (pendant power button; Husky per its own procedure). If the arms are only being
   parked overnight and stay on, step 4 is still the right place to leave things: the stacks
   restart in seconds with stage 7a.
6. **Write it down**: outcome, the run-log folder, and anything that changed (a re-touch,
   a re-solve) in [pickup_calibration_handover.md](pickup_calibration_handover.md).

---

## Troubleshooting

**`scene NOT USABLE` / some parts `NONE` graspable.** The packed positions are out of reach
or too crowded. Raise `PACK_GAP` in `export_layout.py` (currently 0.040 m) and re-export.
Bounding-box gap is *not* the criterion on its own — a 20 mm pack that the spread step later
opened to 43–115 mm was still ungraspable, because what matters is where each part lands
relative to the arms. So change it and re-run the check rather than reasoning about the
number.

**The solve runs for many minutes.** Almost always an unusable scene. Check the
`[scene-check]` lines near the top of the output; if it said NOT USABLE, kill it.

**`saved sampled-grasp database hash does not match this rebuild`.** The plan predates a
change to the grasp sampler, usually a pull from planner master. Solve again; `mp.json` and
any recording are stale too.

**The sim fails every `assemble` as `not_seated`.** The settle gate is not converging — look
for `settle windows: ... err NN mrad` against a 2 mrad tolerance. `--settle-tol` /
`--settle-max` are the knobs; see `FRAGILITY.md` §6.3 in the planner repo.

**`layout has no 'calibration_marks'`.** The layout came from an older exporter. Re-run
stage 1.

## Conventions worth knowing

- Every layout coordinate is in the robot base frame (`base_footprint`, z=0 at the floor),
  which is exactly the planner's rai world frame — a measured point is directly a planner
  coordinate, no conversion anywhere.
- `a1` = RIGHT arm, `a2` = LEFT arm on the planner side; index 0 = left, 1 = right here.
- Quaternions are **wxyz** in the layout and in rai, **xyzw** in PyBullet.
- A part's `yaw` is a turn about world z on top of how the part lies. Height is never
  measured: a part rests on the measured table.
- Legs lie on their wider 30 mm face, so a leg reads 105 x 30 mm with a 22 mm height.

## Accuracy

Bounded by the URDF calibration plus the punch TCP, so roughly 1–2 mm. With 1 mm of touch
noise a four-cross fit gives about 0.1 mm in position and 0.3° in turn — the crosses span
~180–325 mm, and that long baseline is what makes the turn accurate. The reported rms and the
table-top spread are the numbers to judge a take by.

## Known limitation

`pickup_calib` fits **one** sheet: it pools every mark into a single rigid fit. That is
correct while the layout is one sheet, which it is. If a future layout needs several sheets
the tool would average their poses together and misplace everything — the fit must become
per-sheet first. The marks already carry a `page` field for that.

## Files

| What | Where |
|---|---|
| Measuring tool | `husky_assembly_teleop/pickup_calib.py` |
| Sheet exporter / packer | `~/Code/fixtureless-assembly/export_layout.py` |
| Planner side of `--layout-json` | `problem.add_disassembled_obj`, `main.py`, `replay.py`, `plan_io.py` |
| Approach obstacles from a layout | `husky_assembly_teleop/open_loop_approach.py` |
| Gripper stacks on the Husky | `scripts/husky/start_gripper_stacks.sh`, `scripts/husky/stop_gripper_stacks.sh` (both take `--status`) |
| Open / close the grippers by hand | `scripts/husky/open_grippers.sh`, `scripts/husky/close_grippers.sh` (`left`, `right` or both) |
| Live-pose collision diagnosis | `scripts/husky/live_pose_check.py`, `scripts/husky/live_pose_in_planner.py` |
| Punch TCP offsets | `data/calibration_data/20260622/config.yaml` |
| Session handover | [pickup_calibration_handover.md](pickup_calibration_handover.md) |
