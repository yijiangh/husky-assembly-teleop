# Base calibration as a guided plugin (2026-10-20)

Status: **plugin built, sim walkthrough in progress; not committed, not tried on a robot.** Ports the old monitor's
base calibration (`master`, extended on `origin/yh/multi-robot-monitor`) into a `calibration` plugin. Collection,
solve, validation and apply all happen in the monitor, in one guided workflow.

## What is calibrated

One SE(3) per robot: `base_mocap_from_base_footprint`, the URDF root in the base rigid body's frame. The relay
applies it to every pose it publishes for that rigid body (`mocap_relay/node.py:119`). It is measured on one arm; that arm's base comes
from two joint-axis fits, then the URDF chain from the arm base to `base_footprint` is applied. The UR factory
kinematics, the dual-arm mount and the tool marker are not calibrated here (unchanged).

## Decisions

- **No raw base pose needed.** The relay publishes the applied calibration `C_old` latched on
  `/mocap_relay/calibrations`: the full transform, its sha256, source and session. Each sample's base body pose is
  rebuilt as `published base_footprint · inv(C_old)` with the calibration applied at that moment, so the recorded
  takes are the old `raw_data` format in the base body frame, and a relay reload mid-session does no harm. The
  correction shown is `inv(C_old) · C_new`. (Fitting directly in the published frame gives the same result,
  `C_new = C_old · solution`: tested.)
- **Keep the math close to the old pipeline for now.** Port `0_circle_fitting.py`, `1_calibration_analysis.py`
  (`yh/multi-robot-monitor` version, with motion-based axis signs) and `2_convert_and_visualize_transformation.py` as
  they are. Only these change:
  - The hardcoded `'0806'` is gone: with motion-based signs both of its branches give Y = −J1.
  - The URDF-nominal sign test and its PyBullet FK are dropped: the motion signs always override them.
  - Normals are turned to +joint before averaging, so opposite fit signs cannot cancel (no change on good data).
  - The verify metric bug (`‖p‖ − mean‖p‖`) is fixed: distance from the mean position.
  - `circle_fitting_3d` (not in the venv) is reimplemented with the same algorithm.
  - Take files are filed by the joint they turn and deduplicated, not by folder (Belle's `j0/` holds two J1 takes).

  Reproduces every complete Drive session's arm base to under 0.2 µm (20260303 and earlier output files used
  another convention; 20260608 is the axis-sign-bug data). New checks reported: J0/J1 skew (0.03–0.3 mm), axes
  angle (90.00–90.08°), and per take how consistently the joint turned one way.
- **Workflow is new: a step wizard** with Back / Next and live checks. Next stays disabled until the step's
  checks pass. Each step's controls are in one GUI folder.
- **The sweeps are predefined, taken from the old monitor** (`data/calibration/sweeps.yaml`), and collision-checked
  in the plugin.
- **One default tool body for every arm**: `DEFAULT_TOOL_BODY` (1866, the calibration marker) in `sweeps.py`;
  Setup starts from it, the operator may change it, and the run file keeps the one chosen.
- **Both arms are allowed, one at a time.** The arm is chosen in setup, and one session holds one arm.
- **Session-file code moves to `world/`.**
- **The plugin has a relay reload button.**
- **A run survives a crash.** Its progress is saved after every change in a run file next to its take files; a crash
  loses at most the take in progress (a take resumed half-way could mix two tool mountings). A Run dropdown lists the
  arm's runs, newest first; choosing one rebuilds everything from the run file and the take files, stays in Setup (the
  checklist is ticked again), and Next goes through every step whose checks still pass. Collision checks hold only
  while the base stays within 2 cm / 2° of where they were made.

- **Next is never a hard block**: while a check is red, Next is orange; a press refuses once, says why and turns
  Next into "Confirm: go on anyway", which goes on; the run file keeps every override. While a procedure runs, Back
  and Next are grey with the reason in their label (viser shows no tooltip on a button).
- **A calibration is applied only once approved.** Solve writes `calibrated_transformation_<digits>_rhino_in_progress
  .json`, which no relay reads; Apply renames it to the relay's name (an approved one of the session to `archive/`),
  then reloads the relay. The relay itself is unchanged: it only ever reads the approved name.
- **Files a run no longer uses go to `archive/`** below their folder, where neither the relay nor an offline solve
  reads them: a take left out of Solve (at once), a take recorded again, takes and validation samples dropped by
  Refined, cleared validation samples and punch takes.
- **Go, Cancel, Preview and Auto sit below the step's own controls** (Step controls), always shown in the steps that
  run things and grey while they do nothing; Go says what it starts. Record asks Go twice per take: the move to its
  start, then the sweep, each previewed as a ghost with Preview. **Auto** (off by default) skips every Go of moves,
  sweeps and validation poses, and Record then goes on with the next pending take in the shortest order from where
  the arm stands (`shortest_order`); each take runs turned by whole turns and in the direction that starts nearest
  the arm (`nearest_take`), held joints within 225° of stow; validation poses likewise (`nearest_turns`).
- **Chips only say what is not already said**: a procedure that went well leaves no outcome chip, a refused Next none
  (Next itself turns to Confirm), the live base and tool chips only show a problem and never in Setup, and Setup's
  sweeps chip only shows when the file cannot calibrate the arm.
- **Colours**: Sweeps colours takes by the collision check; Record keeps colour for what was measured (grey to
  record, blue recorded, colliding takes not drawn; measured points always blue).
- **Validate checks its fixed poses for collisions first** at each spot and visits only the clear ones, each turned
  by whole turns to the equivalent nearest the arm. It shows "base spot n of 3 · samples of 15", a chip per pose
  (grey, red collides or no move, then green / amber / red by the sample's distance from the mean) and a dot per
  sample where the tool body was, in the same colours.
- **Takes per joint: two needed, four wanted** (amber under four; Su's sessions had six per joint).
- **A finished run resumes at Punch**: the punch replaces the tool body, which needs a monitor restart with the bare
  arm (`tools:=...:none`); choosing the run then skips the earlier steps.
- **Refining the base body warns** (Marker error, the base markers check and the manual): it shifts every consumer's
  base pose until the run is applied.
- **Only the robot's first arm calibrates the base.** Another arm's run (Cindy's right) checks that arm's mount: Solve
  writes `arm_mount_check_<digits>_<arm>.json` with the measured arm base against the URDF, never the relay's file;
  Apply has nothing to do.
- **The calibration marker is an end effector** (`calibration_marker`): no driver, only a collision shape (the flat
  Motive marker plate on tool0 on a 7 mm 3D-printed disk), so the collision check sees what is on the flange.
- **Sweep steps take at least 1 s** (the 5 s default is for moves sent by hand).
- **Clearance: 5 cm for arm motions** during takes (`ARM_CLEARANCE`); driving in Marker error keeps the 10 cm
  `STOP_DISTANCE` made for bases. The collision pre-check (`check_path`) has no margin: it reports contacts in the
  model, so the calibration marker's model must match the real plate.
- **crl-husky: `update_rate: 500` for `scaled_joint_trajectory_controller`.** Unset, the controller samples ahead by
  each cycle's own period; a short cycle after a late one samples earlier than the last, and the forward-only
  trajectory sampler then jumps to the final point (seen as snaps in the sim; would reach the real arms as a servoj
  target). Upstream ur_robot_driver set it in 2.14 (#1894).
- **Kept for now:** the old math and its ~0.5 mm bias on calibrated URDFs (known limit); the old offline scripts until
  after the lab runs. The order and directions of takes may change as long as each take's geometry stays the same.
- **Sim: the real relay on a simulated source.** `crl_husky sim_robot.launch.py mocap_relay:=true` sends husky_sim's
  true base_footprint to `/mocap_sim/truth/id_<n>/pose`; `husky_sim_motive` (`mocap_relay/sim_motive.py`) turns it into
  the raw rigid body through a true calibration (crl-husky's default plus `calibration_error`) and publishes
  calibration markers from the simulated arms' FK, on `/mocap_sim/rigid_body/id_<n>/pose`; the relay with
  `source:=sim` treats those like Motive's bodies. `launch/sim_mocap.launch.py` starts both; the relay reads
  the simulated sessions (`SIM_SESSIONS_FOLDER`: `<Drive>/data_experiment/sim/calibration_data/domain_<id>/`, one per domain, never read by a robot's relay). Without `mocap_relay:=true` nothing changes for other
  sim users. The phone tab comes once the desktop works.

## Steps

The panel follows the phone mock's Steps tab (`phone_mock.py`): title, the step list, the current step (instruction,
check chips, what runs, the live base/tool/lease chips, the question waiting for Go), a "Step" folder (Go, Cancel,
Back, Next) and a "Step controls" folder with the current step's own widgets. Next is refused while a check is red,
and says which. One procedure runs at a time; Cancel ends it and holds the arm during a sweep (a planned move
runs on in the arm planner).

1. **Setup.** Arm (robots with sweeps only), tool body (DEFAULT_TOOL_BODY, tracked at once), checklist
   (tool on flange, area clear, e-stop at hand, Motive anchored). Checks: sweeps valid, relay has announced its
   calibrations, base and tool mocap, arm joints, the sessions folder, no soft stop, checklist ticked.
2. **Marker error.** The D-pad in the step's folder drives (lease and a clearance only while a button is held;
   switches to the velocity controller on a real base); driving only shows Motive new angles. Check: 5 s driven, each
   body's largest marker error amber over 1 mm (the mocap chips' threshold), red over 2 mm. Recheck starts the 5 s
   again. Start refine shows only once a marker error is red, with the consequences as warnings; Start refine / Refined: drive while refining in
   Motive, uncounted; Refined restarts the check and drops the run's takes (the rigid body changed).
3. **Sweeps.** Collisions are checked on entering (when missing or the base moved; the button checks again): the
   arm planner checks every take's waypoints and the moves between them, in the
   world as it is now (`ArmPlannerContract.check_path`). Chips and arcs at the robot colour each take (grey unchecked,
   green clear, red collides, blue recorded); a colliding take cannot be recorded; two clear takes per joint are
   needed, four wanted. A dropdown and a slider preview any waypoint of any take as a ghost; waypoints, and measured points once
   recorded, are translucent spheres on the arcs. The panel docked right, minimized until opened: Live (marker errors,
   base drift) and Results (figures).
4. **Record.** Per take (dropdown of clear takes, advances to the next pending one in the shortest order; Auto Go: no Go prompts and on
   to the next clear take): the arm planner plans the move
   to its start and asks Go, with a Move preview slider scrubbing the plan as a ghost; then under the lease, a clearance and a freeze of the robot's other arm, each waypoint is one joint move,
   then sampled once the tool stands still on the base (spread under 0.2 mm over 10 ticks) with good mocap, 20 ticks
   averaged. The take file is written and its circle fitted. Check: circle error under 1.5 mm and a one-way sweep for
   every take, two good takes per joint (amber under four).
5. **Solve.** The old math on the run's good takes; writes the outputs, the relay's file in progress (not applied
   until Apply). Checks: correction against the applied calibration, axis fits, skew, and the
   relay's own sanity check (red if the relay would refuse it). Writes the old pipeline's files and figures (`old_outputs.py`: same names and
   folders, also after Validate and each punch take), shown in the panel's Results tab.
6. **Validate.** Base moves matter (base-to-tip accuracy across the room): drive with the D-pad (lease only while a
   button is held), then "Sample poses here" visits the arm's validation poses (`validation` in sweeps.yaml, six from
   the old sessions' samples) via the arm planner, a pose with no move skipped. One file per spot. Checks: 15 samples,
   3 spots (0.5 m or 30° apart), 95 % within 2 mm (amber) / 5 mm (red), 95 % of each axis within 0.3° (amber).
   Automating the base moves (base planner) can come later.
7. **Apply.** Tick "No robot is driving", "Apply: approve and reload": the calibration gets the relay's name, then
   the relay's reload service, then wait for its announcement to carry the new file's hash. Check: the relay applies the new calibration.
8. **Punch.** The old procedure's physical check: TCP offset in mm (pendant 4-point), D-pad, free-drive on the
   pendant, "Record punch take" (base and joints still, 1 s averaged, tip = base_footprint · FK · offset). Written in
   the old monitor's format to `punch_validation/`, so `4_punch_validation.py` reads it. Checks: offset 50–300 mm,
   one offset, 4 takes, tip spread 2 / 5 mm, headings over 90°.

A simulated robot's sessions go to `<Drive>/data_experiment/sim/calibration_data/domain_<ROS_DOMAIN_ID>/`, never the
robots' `calibration_data/`: a robot's relay applies the newest session there at its next start or reload.

## Code

```
data/calibration/sweeps.yaml          takes and validation poses per robot and arm
plugins/calibration/
  plugin.py        the guided procedure: steps, tasks that move and sample, panel
  steps.py         steps, instructions, and the checks that gate Next (pure)
  progress.py      the run file: saved after every change, read to resume (pure)
  sampling.py      readings averaged into a sample once still; the record with the calibration taken off (pure)
  fit.py           the old math (circle, axis lines, arm base frame, URDF chain, validation spread)
  sweeps.py        the sweeps file, waypoints, sweep_problems
  session_files.py take files (read by swept joint, never overwritten), validation samples, outputs with mocap_id
robot_interface/mocap_relay.py        MocapRelayInterface, `world.mocap_relay`: the calibrations the relay announces,
                                      and its reload service (closes TODO 7 of 2026-10-16_frames_and_calibration.md)
world/calibration_sessions.py         moved from mocap_relay/calibration.py; the relay imports it from here
shared/teleop/dpad.py                 moved from plugins/robot_control/dpad.py
shared/arm_planner_contract.py        + check_path: a given joint path checked for collisions (arm_planner implements it)
scripts/solve_calibration.py          re-solve a Drive session, compare with its output, check its validation
test/plugins/calibration/             golden fixtures, known-calibration recovery, sweeps, session files, step checks
test/robot_interface/test_mocap_relay_interface.py, test/plugins/arm_planner/test_arm_planner.py (path check)
```

## Session files

`data_experiment/calibration_data/<YYYYMMDD>/`, where the relay looks:

```
calibrated_transformation_<digits>_rhino.json   unchanged keys plus mocap_id (the relay checks it)
base_frame_calibration.json                     as before
<j0|j1>/calibration_<HHMMSS>_<digits>_<arm>_J<j>_take<n>.json   old raw_data format, plus per sample: published
                                                base_footprint, the applied calibration and its hash, marker errors,
                                                spread, time; base_mocap_pose is the rigid body's own pose
validation/calibration_<HHMMSS>_<digits>_<arm>_validation.json
run_<digits>_<arm>_<HHMMSS>.json                the run's progress: sweeps, step reached, drive marker errors,
                                                collision results and base pose, take files, validation file,
                                                output hash, applied
archive/                                        the session's earlier calibration files for the robot
```

`yh/multi-robot-monitor` still writes to the same sessions, so the file names and keys must not change. The plugin
solves from the take files its run file lists; `scripts/solve_calibration.py` reads every take file of a session, so two arms of one robot
on one day mix there.

## Phases

0. **Moves** (done): calibration_sessions to `world/`, D-pad to `shared/teleop/`, the relay interface.
1. **Math** (done): `fit.py` and golden tests, the sweeps file, session files, `scripts/solve_calibration.py`.
2. **Plugin** (done, sim-tested from Setup to Record so far): all seven steps, arm planner `check_path`.
3. **Sim support** (built, see Decisions): relay `source:=sim`, `sim_motive`, `sim_mocap.launch.py`, crl-husky's
   `mocap_relay:=true`. To try in the sim: a run with a `calibration_error` must find it.
4. **Lab run on one robot.** Compare with its latest old-pipeline session.
5. **Cleanup.** Rewrite `doc/calibration_manual.md` for the plugin, delete `calibration_checklist.md` and
   `data/calibration_data/*.py`, update `doc/plugin_roadmap.md` and the README's plugin table. Phone tab: decide whether the wizard also goes on the phone.

## Open

- Cindy's right arm: a right-arm session gives that arm's base; is it compared with `right_arm_mount_joint` and only
  reported, or applied?
- No calibration-tool model: the collision check sees the robot with the tool it was launched with.
- Collision reasons are logged and in the chips' tooltips; the step list only says "collides".
