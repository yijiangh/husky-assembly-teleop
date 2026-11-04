# Base calibration manual

> **⚠ WORK IN PROGRESS.** The `calibration` plugin is tried in simulation, not yet on a robot: watch every move and
> check every result before the relay applies it.

The `calibration` plugin measures where a robot's base rigid body sits on the robot, step by step in the monitor:
from setup to the mocap relay applying the result.

## What is calibrated

- One transform per robot: `base_mocap_from_base_footprint`, the URDF's `base_footprint` in the frame of the base
  rigid body in Motive. The mocap relay applies it to the robot's rigid body pose, so every consumer (monitor, base
  follower) gets `base_footprint` in the world.
- It is measured on one arm. Turning the arm's first two joints (J0 shoulder pan, J1 shoulder lift) moves a tool
  body on the flange on circles around each joint's axis. The two axes give the arm's base frame; the URDF chain
  from the arm base to `base_footprint` gives the calibration.
- Not calibrated here: the UR arms' own kinematics (factory calibration), the dual-arm mount, the tool body.

## What you need

- The robot running with its arm in External Control (crl-husky's README), and someone at the remote e-stop: the
  e-stop on the Husky does not stop the arm.
- The arm's gripper off (onboard launch with `gripper:=none`, see crl-husky's README) and the tool body, the
  calibration marker, on the flange of the arm you calibrate. Setup starts from its Motive Streaming ID, 1866, for
  every arm (`DEFAULT_TOOL_BODY` in `plugins/calibration/robot/sweeps.py`); change it there if another body is mounted.

- Motive streaming the base and tool bodies, with **continuous calibration off or anchor markers placed**: otherwise
  the camera calibration drifts during the run.
- The mocap relay running in the monitor's ROS domain (`ros2 launch husky_assembly_teleop mocap.launch.py`): it
  gives the base pose, says which calibration it applies, and reloads the new one at the end.
- For a real robot, the Drive: `HUSKY_DRIVE_ROOT` (or `-p drive_root:=`) with `data_experiment/calibration_data/`
  synced.
- Room all round the robot: the sweeps turn J0 by up to a full turn.

## Start the monitor

From the workspace root, in the venv, with ROS and the workspace sourced:

```bash
ros2 launch husky_assembly_teleop monitor.launch.py robots:="belle" plugins:="calibration obstacles" \
    tools:="0805:calibration_marker"
```

- `tools` gives the collision checks the marker plate on the flange instead of the robot's default tool. One entry
  per arm: Cindy calibrating her left arm is `0806:calibration_marker:none`.
- For Tip offset and Punch, the punch replaces the tool body: stop the monitor after Apply, swap them, and start it
  again with the bare arm (`'0805:none'`); see [Tip offset](#8-tip-offset).
- `arm_planner` loads with it: it plans the moves and checks the sweeps for collisions. Its first check loads the
  models and takes a while.
- `obstacles` is optional: with it, the collision checks also see the lab's obstacles.

## The panel

From top to bottom: the overview of the steps (each row its state in short, why on hover); a **Step** folder with Back
and Next as small arrows (‹ ›) and Done, the main button, which never move; the current step's chips on named lines
(**result**: whether what the step made is good enough, which sets its state; **ready**: whether its buttons can act
now, red refuses; **now**: always there and one line high, with what runs now, how the last action ended, a base or
tool chip while the mocap of a body the step uses has a problem, and on steps that move the robot a soft stop chip and
a lease chip while another plugin holds the robot; more chips than fit scroll sideways); a **Step controls** folder with the current step's own controls, its readout below
them, and, in the steps that run things, **Auto**, **Preview time**, **Play**, **Go** and **Cancel** (Go only where a
step can wait for you: Record, Validate, Tip offset, Punch; Cancel there, and in Sweeps while the collision check
runs); then any
question waiting for Go and Done's question; and last, under "what to do", the step's instructions. Nothing above a
button changes height while a step shows, so the buttons never jump.

- The overview has a line per step, always ten: its number, its name (bold for the current one, whose number is ringed
  blue), its state and, in short, why (hover for all of it). Steps with a result have a solid number chip: **passes**
  (green), **fails** or **out of date** (red: built from data that changed since, e.g. Record and Solve after the tool
  body changed), **none yet** or **pending** (grey: nothing to judge yet), **accepted** (amber: a failing result you
  went on past with Done; a result missing or out of date is never accepted), **not in use** (amber, Tip offset only:
  its fit is not Punch's TCP offset; no work left). The steps that gather a precondition for later steps (Setup, Marker
  error, Sweeps) have a dashed one: **holds** (green), **expired** (amber; why and since when on hover), **none yet**,
  **fails**. The current step outlines, dotted blue, the precondition steps it needs that have expired.
- Check chips: green passes, amber is a warning, red asks you to stop and look, grey has nothing to judge yet (e.g.
  Solve's axes before solving). Hover a chip for its detail. They judge the step's result as it stands now, and
  carry its figure where there is one, e.g. **samples 9/15** or **position 0.8 mm**.
- A second row of chips says whether the step's buttons can run now, e.g. in Record **Setup** (the checklist holds),
  **Marker error** (measured within the hour), **Sweeps** (the base still where the collisions were checked) and
  **area, e-stop** (Area clear and E-stop at hand ticked within the hour). An amber one only warns: the action asks
  at Go first ("Expired: ... Go acts anyway"), and Go acts. A red one refuses, and only for these reasons:
  - **area, e-stop** red: no robot moves (D-pad, free drive, tool pad, takes, validation poses). A take or a spot asks
    at Go instead whether the area is clear and someone holds the e-stop, and Go ticks both boxes again; Auto never
    skips that question. The D-pad, free drive and the tool pad never ask: they refuse until both are ticked again in
    Setup. Expired preconditions only show amber for them.
  - Record's **Sweeps** red: no collision check where the base stands.
  - what makes an action impossible: no tool body, no clear take, no validation poses, a TCP offset outside 50–300 mm,
    no relay, the wrong tool model for a tip pose or punch take, Punch without a lab frame (none announced yet, not
    trusted, or its floor calibration unreadable); Apply without a solution, or without a validation verdict that
    passes or fails and is accepted.
  - Live safety (lease, soft stop, clearance, base mocap) is enforced below the panel, whatever it shows.
  Solve, Check collisions, the Clear buttons and Apply's reload never refuse on a precondition. While a procedure runs,
  every other action (another arm, run or tool body, Solve, a Clear button, Punch remounted, Target moved) is refused,
  and the status line says so in amber.
- Each step shows the same chips from the start: they change colour as you go, they never appear or vanish.
- **Back** and **Next** move one step, and nothing else: they never save, accept or start anything. Leaving Marker
  error stops driving.
- **Done** says you are done with this step: it saves the run and goes to the work that is left (see [Resuming a
  run](#resuming-a-run) for the rule): the first step whose result is missing, out of date, or failing without an
  acceptance; if that step needs a precondition that has expired, the earliest such precondition step first, then Done
  there goes on to the next one it needs, and then to that step. On a step whose result fails, is missing or out of
  date, or a precondition step that has expired (Setup's boxes, Marker error, Sweeps), Done is orange: a first press
  says why below the step and turns the button into **Accept as is** (a failing result) or **Skip** (one missing, out
  of date or expired) for 5 s; pressed in that time it goes on (to the next step if the
  work left is this one), after it the button is Done again. Accept as is shows the step **accepted** (amber) in the
  overview: the acceptance holds while what the result was calculated from (and, for Punch, the calibration it was
  judged under) stays the same, and the run file keeps it. Skip records nothing: the step still shows none yet or out of
  date.
- The one time Back, Done and Next do nothing is while something runs: they are grey ("Cancel first").
- A second panel, docked right, holds **Live** (marker errors; base drift since the collision check, moved in mm and
  turned in degrees) and **Results** (figures). It starts minimized to a thin rail at the screen's right edge: click
  a tab name there to open it.
- **Go** is enabled when the step waits for you, and says what it starts ("Go: move", "Go: sweep", "Go: try
  again"). Scrub **Preview time** first (seconds from the start up to the move's end), or press **Play** to watch it
  in real time: the ghost shows the arm along what Go starts. A sweep stops at each waypoint for its shortest sample,
  1.5 s; readings that come in moving make the real stops longer. After 10 minutes without Go the procedure fails.
- **Clearance** below Go, ticked, keeps what Go moves clear: a planned move soft stops within 5 mm of anything and is
  refused if the arm already stands that close, a sweep soft stops within 5 cm of the scene (5 mm of the robot
  itself). A red "starts too close" chip warns before the press; untick to move out of a tight spot (the chip turns
  amber: nothing soft stops the arm then).
- With other robots not measured, a Go that moves the arm asks twice, as the planners' Commit: the first press turns
  it amber and says so below; a second within 5 s moves, with those robots checked as stand-ins (any other robot lost
  soft stops the arm). The D-pad and the tool pad do not ask: a robot not measured at the press is a stand-in. Nor
  does Go ask for a robot with its base tracked whose unknown arms are beyond reach: their shoulders about 2.5 m or
  more from the moving arm's (each arm's reach, tool and held bodies included, whatever the joints:
  `world/arm_reach.py`).
- **Auto** goes on without asking Go: the moves, the sweeps and the validation poses; Record then goes on with the
  next take. Off by default. A sample that does not come still asks Go to try again, and a move asks Go (twice)
  while other robots are not measured.
- **Cancel** is enabled while something runs and ends it; the arm holds, and a planned move is cancelled in the arm
  planner too, which gives the robot back. In Validate, Cancel while the arm moves or samples only stops the arm:
  Go goes on with the poses left; Cancel then ends the validation. A take cancelled mid-sweep is lost (a circle
  needs the whole sweep); the takes recorded before stay.
- The D-pad drives while a procedure only waits for Go. A move planned before the base was driven more than 2 cm or
  2° is planned again, and Go asked again.
- While a step's action only waits for Go, pressing an action again starts it afresh: e.g. **Record take** with
  another take chosen plans the move to that one. While something moves or works, an action is refused.
- A procedure that went well leaves no chip: the step's checks say it. One that failed, was refused or cancelled
  leaves a red chip; hover it for why.
- One thing runs at a time. Every motion runs under the robot's lease, so no other plugin commands it meanwhile.
  Stop all ends every calibration procedure.

## The steps

### 1. Setup

1. Pick the **Arm** (only robots and arms with sweeps are listed; the first is chosen at start). The **Tool body**
   starts at the calibration marker's (1866) and is tracked at once; change it if a different body sits on the
   flange. The run file keeps the one chosen. Takes and validation samples belong to the tool body they were measured
   with: another one makes them out of date (red in the overview) until it is set back.
   Below the checklist, **for the record**: tick **On stand** when the base stands on its stand (wheels off the
   floor), and write anything worth knowing later in **Notes**. Both go into the run file for people to read; nothing
   in the calibration uses them. A new run of the same arm keeps On stand, not the notes.
2. If an unfinished run of this arm is offered, see [Resuming a run](#resuming-a-run).
3. Mount the tool body, clear the area, take the e-stop, and tick the checklist: Tool on flange (the tool the
   **tool model** chip names is the one on the flange), Area clear, E-stop at hand, Cameras calibrated
   (Exceptional, under a week old, no camera moved). Whether Motive is anchored the **frame** chip shows: the relay
   checks it. Every later step that acts relies on the checklist, Tip offset
   and Punch included: once it has expired they ask at Go first, and no robot moves while Area clear or E-stop at hand
   is older than an hour. Each tick holds for an hour, also across a restart of the monitor (the run file
   keeps when each box was ticked); Tool on flange holds only for the tool model it was ticked for, so after
   relaunching with the punch only that box asks again. A box that no longer holds shows unticked.

| Check | Red means | What to do |
|---|---|---|
| sweeps | the sweeps file cannot calibrate this arm (too few takes, a waypoint past a joint limit, no validation poses, ...) | fix `data/calibration/sweeps.yaml` |
| relay up | no calibration announcement from the relay (green: which calibration it applies, and its hash) | start the relay in this domain |
| frame | the relay does not hold the lab frame: not trusted (the anchors and truss cameras disagree), or its floor calibration unreadable; amber: not checked yet or a refit pending; grey: no announcement yet, or no floor calibration applied (takes are then compared in Motive's own frame); hover for why and the anchors it does not see | check the anchors and truss cameras in Motive; apply a floor calibration or use the refit in the mocap probe while no robot drives |
| base mocap, tool mocap | no usable pose: no messages, lost, or marked invalid by the relay (amber: high marker error) | check Motive, the markers and the Streaming IDs |
| arm | no joint states | check the arm's driver |
| tool model | the arm's tool model is not `calibration_marker`, so the collision checks would miss the plate (amber once applied with the marker still modelled: only Tip offset and Punch want the bare arm) | restart the monitor with the `-p tools:=...` line it shows |
| Drive | the sessions folder is missing; or the robot's base or arm is simulated and the relay reads Motive, or the other way round | set `HUSKY_DRIVE_ROOT`, sync `data_experiment/calibration_data/`; run a simulated robot with a simulation's relay (`sim:=true`) |
| soft stop | the robot is soft stopped | Resume it (health panel) |

The first tick starts the run: today's session folder and its run file (see [Files](#files)). Before that, a new
run writes nothing; later the first take, sample, tip pose, punch take or solve would start it. Setup's state is the
checklist's alone (dashed in the overview: holds or expired); the other checks are live and only shown. Done goes to
the work that is left.

### 2. Marker error

Driving shows Motive the rigid bodies from new angles; their marker errors must stay small throughout.

1. Drive with the D-pad in Step controls (hold a button; **Speed** is a fraction of full speed), turning both ways.
   The D-pad holds the robot only while a button is held; on a real base it switches to the velocity controller.
   **Clearance** below it, ticked, soft stops the base within 10 cm of anything; untick it to drive out of a tight
   spot (the D-pad turns amber). Every D-pad of the run, on both pages, shows the same box.
2. Finish where the robot is to stand for the sweeps: clear all round.

Check: 5 s driven, and each body's largest marker error while driving: amber over 1 mm (usable, worth a look at
the markers), red over 2 mm. The chips name the level; the seconds and the largest errors are in the readout below
the buttons.

**Recheck** starts the measurement again from zero (5 s of driving), e.g. after cleaning or reseating a marker. A
measurement holds for an hour once the base was driven 5 s and while no body is red: after an hour **drive** turns
amber, and Record and Validate warn and ask at Go before acting. Driven too short or a red body expires it at once,
unless you accept it (Done, then Accept as is): that accepts this measurement, until the next one.

If one is too large, the step says which and shows **Refine base body** or **Refine tool body**; until then there is
no refine button. Look at the markers first and Recheck. If it stays large: press it, then in Motive select that rigid
body, Builder, Refine, Start; drive around with the D-pad; Apply in Motive. The relay asks Motive for its rigid body
definitions every 2 s, so the plugin sees the new definition itself: the refine ends and the check starts again. In a
simulation without the fake Motive (no definitions), Recheck ends it.

Every measurement stores both bodies' definitions as Motive had them. Any data change to a definition (a refine, a
body made again, a moved pivot, another body on the id; not a rename or a new streaming id) makes out of date what was
measured with the old one, whether done from this step or not, and moves their files to `archive/` (the run keeps them,
shown out of date; the old definition back makes them current again):

- after a **base** change: the takes (and with them the solution and Apply), validation samples, punch takes and this
  measurement, since the takes are measured in the base body's frame;
- after a **tool** change: the validation samples and this measurement. The tool body moves by a fixed offset on the
  flange, so the takes still circle the same axes and keep counting (an assumption still to confirm with the calibration
  bench).

> ⚠️ Refining the **base** body changes the pose every consumer of the relay gets for this robot (monitors, the onboard
> follower): the calibration applied no longer fits the refined body. It stays wrong until this run reaches Apply; the
> relay warns about it and the health panel turns the robot's base **mocap** chip amber. Refine the base body only if
> you will finish the run, and stop other users of the robot meanwhile.

Solve writes the base body's definition the takes were recorded with into the calibration file (`body_definition`),
so the relay can tell when Motive's body no longer matches it ([mocap_relay.md](mocap_relay.md)).

### 3. Sweeps

The takes are predefined, taken from the old monitor (`data/calibration/sweeps.yaml`). The arm planner checks every
take's waypoints and the joint moves between them against the robot itself, the other
robots and every scene body, with the robot where it stands now, when you press **Check collisions**; press it again
after driving away. Nothing is checked on its own. **Preview take** and **Waypoint** show any waypoint as a ghost.

- Each take gets a chip, and its tool0 path is drawn at the robot as an arc in the same colour, with its waypoints
  as small translucent spheres: grey not checked, green clear, red collides (the reason is in the chip's tooltip
  and the log). Measured points are blue. A colliding take cannot be recorded.
- **takes**: grey until every take is checked (none yet, or a check cancelled half-way); red under two clear takes
  for J0 or J1, amber under four; the hover names the takes that collide.
- **checked here** (second row): amber once the base moved more than 2 cm or 2° since the check, or a check was
  cancelled half-way: check collisions again. Record refuses to record until then (its **Sweeps** chip is red).
- ! Do not move the base from here on.

### 4. Record

Pick a take in the **Take** dropdown (only clear takes are offered; after each take it moves to the next one not
recorded yet, numbered above it, wrapping round), then **Record take**:

1. The arm planner plans a move to the take's start. Scrub **Preview time** or press **Play** to see the arm's way as
   a ghost, then **Go: move**.
2. At the start, preview again to see the arm's way round the take, then **Go: sweep**.
3. Under the robot's lease, with the robot's base and other arm frozen and a 5 cm clearance to the other robots and
   scene bodies, each waypoint is one joint move (at least 1 s). Recording is refused once the base has moved over
   2 cm or 2° since the collision check, judged again under the lease right before the sweep (the base may have been
   driven while Go waited): check collisions again.
4. At each waypoint the plugin waits until the tool stands still on the base (its position spread under 0.2 mm over
   10 ticks) with both bodies tracked, then averages 20 ticks (1 s) into one sample. If that takes over 20 s, the
   step says why; Go tries again, Cancel stops.
5. After the last waypoint the take file is written and its circle fitted; its chip and arc turn blue (the chip
   amber if the take is left out of Solve).

Record keeps colour for what was measured: the arcs still to record are grey, recorded ones blue (a measured point
red where its sample's marker error was over 2 mm), and colliding takes are not drawn. Every sample keeps the marker
errors the relay reported with it: the **markers** row has a chip per measured take with its largest (red over 2 mm,
amber over 1 mm), the worst sample's on hover. While a take records, its chip is amber and its measured points fill in on its arc as larger,
blue spheres. Each take is run as the same sweep turned by whole turns, forwards or backwards, whichever starts
nearest where the arm stands within the joint limits; held joints stay within 225° of the stow pose so the tool's
cables do not wind up. After each take the take dropdown moves to the next clear one not recorded yet in the
shortest order from where the arm stands (one joint's takes, then the other's); on **Auto** that one starts on its
own. Cancel stops.

Record every clear take: two per joint are the minimum, four or more are wanted, all of them is best.

| Check | Means | What to do |
|---|---|---|
| takes | red under two good takes for J0 or J1, amber under four | record more |
| old takes (amber) | takes recorded with another tool body or before a change to the base body's definition: left out of Solve | record them again |

Recording also needs, in the second row: a **tool body**, a clear take checked where the base stands (**Sweeps**: red
refuses), and **area, e-stop** (Go confirms them). **Setup** (Tool on flange and Cameras calibrated hold) and **Marker
error** (measured within the hour, not red) only warn when expired: the take asks at Go first, and then each
following take on Auto asks again only about what expired since. Done takes you back to the expired step.

An amber take's file moves to `j0/archive/` or `j1/archive/` at once, so no offline solve reads it either. A take
recorded again gets a new file and the old one moves to `archive/`. Any change to the good takes puts the solution out
of date: Solve turns red in the overview, and Solve and Apply must be done again. A cancelled take writes nothing.

After an e-stop or a protective stop during a take: release the stop, Go on the remote, power the arm up and start
External Control again if the pendant stopped it, Resume the robot in the health panel, then record the take again
(the one cut short wrote nothing).

### 5. Solve

**Solve and write** solves from the run's good takes with the old pipeline's math, then writes the session's
calibration file **in progress** (`calibrated_transformation_<digits>_rhino_in_progress.json`) and
`base_frame_calibration.json` (see [Files](#files)). No relay applies a calibration in progress, also not at its next
start: Apply approves it. An earlier `base_frame_calibration.json` moves to `archive/`, and so do this run's
out-of-date take and validation files, while current ones archived before come back, so an offline solve of the
session reads what Solve used. The run keeps every take and spot: setting the tool body back makes them current again.

The solution holds while the good takes it came from stay the same; **solve** is red again once they change.
**change vs applied** compares the result with the calibration the relay applies now: what Apply will change. Which
calibration the relay applied during the takes does not matter, since every take is stored with it taken off.

Only the robot's first arm calibrates the base. On any other arm (Cindy's right) Solve writes
`arm_mount_check_<digits>_<arm>.json` instead: the arm base measured in `base_footprint` and its difference from the
URDF's mount (check **mount**, amber past 1 mm or 0.1°). The relay never reads it, and Apply has nothing to do: the
mount check counts as applied (the run finished) once Solve wrote it.

| Check | Meaning |
|---|---|
| change vs applied | how far the result is from the calibration the relay applies now (blue, a fact; grey if it applies none) |
| axes | angle between J0 and J1 (90° for a perfect arm) and their skew; amber at 1 mm skew or more; the hover also says how well the circles agree on each axis |
| relay limits | amber if the result is over 5 cm or 5° from crl-husky's default: the relay applies it with a warning (wrong robot, a failed run, or a redefined rigid body?) |

- Solve again whenever the takes change: it replaces the calibration in progress.

Solve also writes the old pipeline's files, as its scripts did, and shows the figures in the panel docked right,
tab **Results**; the log gives the headline numbers and the folders. Not for an arm that only checks its mount (its
session's files belong to the base calibration).

| Folder | Files |
|---|---|
| `j0/`, `j1/` | `circle_fitting_log_jN.txt`, `jN_analysis.json`, `jN_all_circles.png`, `jN_projection_errors.png` |
| session | `calibration_analysis_log.txt`, `calibration_analysis_result.png`, `jN_circle_centers.png`, `jN_base_mocap_pos_deviation.png`, `jN_base_mocap_axis_deviation.png`, `compute_tf_log.txt`, `config.yaml` |
| `validation/` | after Validate: `verification_log.txt`, `verification_validation.png`, `diversity_validation.png`, `debug_*_validation.png` |
| `punch_validation/` | after each punch take: `punch_validation_{position,3d,diversity}_<arm>.png` |

`config.yaml` lets the old scripts run on the session too. The panel's **Live** tab plots both bodies' marker errors
and the base's drift from where collisions were checked, over the last minute.

A take whose points stray from its circle, or whose centre sits off the axis, is the one to record again.

### 6. Validate

Under a good calibration the tool body stays put in `tool0` wherever the base stands and whatever the joints. The
relay does not need the new calibration for this, nor does sampling need a solution: spots sampled before Solve are
judged once there is one (**position** and **orientation** grey, the step **pending**, until then). Leave the tool body
mounted as it was for Record. Expired Setup or marker errors only warn: **Sample poses here** asks at Go first.

1. Drive to a spot with the D-pad (it holds the robot only while a button is held). The scene shows where the arm
   will sample: each validation pose's `tool0` as a small frame at the robot, numbered as its chip, with a sphere in
   the chip's colour; they move with the robot, so you can see whether they clear the surroundings.
2. **Sample poses here**: the arm planner first checks the arm's validation poses for collisions where the base
   stands (a fixed set: `validation` in `data/calibration/sweeps.yaml`, see its header for how they were made), then
   samples 3 clear ones as in Record, with Go before each move unless **Auto**: the poses sampled least often so
   far in this run (so each spot adds other poses), the one the arm reaches soonest first. Each pose is
   turned by whole turns to the equivalent nearest where the arm is, keeping every joint within 225° of the stow
   pose where it can, so no joint turns round on the way and the cables do not wind up. A pose
   that collides, or has no collision-free move from here, is skipped.
3. Two rows of chips. **spots**: one per spot sampled, "spot 2 · 6" (its samples), coloured by its worst sample
   (hover for how far they spread), and the next spot. **poses here**: one per pose at this spot, grey until
   visited, then by the sample's distance from the mean in `tool0` (the position check's 2 / 5 mm), or red if it
   collides or has no move (hover for why). **markers**: each pose's sample's marker error at this spot (red
   over 2 mm, amber over 1 mm; a spot chip's hover gives its largest). In the scene, a dot where the
   tool body was at every sample of the run, in the same colours, and a frame at `base_footprint` where the base
   stood for each spot ("spot 1", "spot 2", …; the spot in progress last), also for a resumed run's spots.
4. Repeat at 3 spots or more, far apart (0.5 m) or turned (30°), best with headings all round. Each sample is
   written as it is taken, a file each (`validation_pose<n>`), so nothing sampled is lost however the validation
   ends. Spots are counted by where the base stood, not by file: going on at the same place adds to that spot.
5. You may drive while Go waits, also after a Cancel: past 2 cm or 2° the poses are checked again; past 0.5 m or 30°
   it is another spot, and every pose is sampled afresh there.

| Check | Level | Meaning |
|---|---|---|
| samples | red under 9 (3 at each of 3 spots), grey before the first | samples over all spots |
| spots | red under 3, grey before the first sample | base spots sampled, and the range of headings |
| position | amber over 2 mm, red over 5 mm, grey without a current solution | 95 % of the samples within this distance of their mean |
| orientation | amber over 0.3°, grey without a current solution | 95 % of each tool body axis within this angle of its mean |

**Clear all spots** starts the validation again: every spot's file moves to `validation/archive/`. The samples stay when takes are recorded
again: they are judged under the current solution, and wait while it is out of date. Like takes, they go out of date
with another tool body or a refine. Apply needs this verdict to pass; Done, then Accept as is, on a failing verdict
accepts it, until the samples or the solution change. When position is red: go Back, look at the takes' fits, record the worst again, Solve,
and see the validation again. Its figures join Solve's in **Results**.

### 7. Apply

1. **Apply: approve and reload** needs, in the second row, a written **solution** and the **validation** verdict
   passing or accepted (Done, then Accept as is, on Validate's failing verdict; an accepted verdict shows amber), **relay up**: the relay
   announced its calibrations, and **relay's Drive**: the relay reads its sessions from the same Drive folder this
   monitor writes to (else it would never find the calibration; start both with the same `HUSKY_DRIVE_ROOT`).
2. Press it with the robot standing still: the reload changes the pose it drives from, its onboard follower's too.
   While an arm or the base moves, nothing is approved. There is no Go: Apply moves nothing.
3. The calibration in progress gets the name the relay reads (an approved one of the same session moves to
   `archive/`); from now on every relay applies it at its next start or reload.
4. The plugin calls the relay's reload service for this robot's rigid body (`reload_base_calibration/id_<n>`): the
   other robots keep the calibration they have. It waits up to 5 s for the relay to announce the new file's hash.

Check **applied**: grey until something was applied, red when an earlier solution was applied (Apply again), green once the relay applied the new calibration. The run is then finished, and stays so: the step
follows what this run applied, not the relay now. **relay now** shows the relay live: amber when it applies another
calibration (restarted, or another run applied since), grey before it announced any; it never sends you back to Apply.
If the reload does not get through:

- "does not answer": the relay is not running in this domain.
- "the relay applies … instead": the relay's log says why, e.g. the file does not read or names another rigid body,
  or a newer session of this robot exists (a resumed run from an earlier day).

### 8. Tip offset

Optional, for Punch, in place of the pendant's 4-point TCP: the punch tip touches one fixed target from several
tilts, and the plugin solves where the tip sits in tool0. The pendant stays in Remote mode and its TCP at zero.

1. Stop the monitor, take the tool body off and mount the punch on the flange. Start the monitor again with the bare
   arm (`-p tools:="['0805:none']"`), choose the arm and this run in Setup's **Run**: it opens at Setup, where Tool on
   flange asks to be ticked again (it was ticked for the tool body's model). The run file keeps the other ticks, for an
   hour, also across the restart; older ones need ticking too. Then Done goes on to Tip offset if every step up to
   Apply passes or is accepted; otherwise to the earlier step with work left.
2. Fix the target fixture on the robot itself, where the arm reaches it, so it moves with the base: the poses are
   in the robot's frame, from the joints only, and the base may rock. An A4 sheet behind it helps to see the tip.
3. Per pose: push the tip near the target by hand while holding **Free drive (hold)**, or with **Keep free drive on**
   ticked when you need both hands at the arm; on release (or unticked) the joint controller holds the arm. Put the
   tip on the target with the tool pad, then **Record tip pose**: once the joints stand still for 0.5 s, 1 s of them
   is averaged. Tilt the punch about two different axes, 30° or more each way: 4–6 poses.
4. Once four poses are in and none of the checks below is red, **Done** makes the fit Punch's TCP offset and goes on
   to Punch (with a red check, Done turns into Accept as is, which accepts the fit and makes it the offset).
   Poses refine the fit, never the offset in use: the step shows **not in use** (amber) until Done there again. An
   offset typed in Punch stays in use; Tip offset then only shows amber and leaves no work. **Clear tip poses**
   clears the poses and the fit, and leaves Punch's offset as it is.
5. The monitor cannot see a remount: shown again with poses (e.g. on a resumed run), the step asks "Punch remounted
   since these poses?" below the controls, on the laptop and the phone. If it was taken off or moved on the flange, press **Punch remounted**: the
   poses and the punch takes so far go out of date (the next pose starts afresh). If not, record more poses or press
   Done: one press answers no and goes on, accepting nothing. The question sits below the controls, so no button moves.

The **Tool pad** has a row per axis of its **Frame**, coloured as the scene draws axes (x red, y green, z blue): on
the left shift −/+ along it, on the right turn −/+ about it through the tip, by the **Step size** chosen (0.1 mm and
0.2° up to 5 mm and 5°). **Robot**, at start, is the robot's frame: x forward, y left, z up, whatever the flange's
turn (from the joints only: no mocap); **Tool** is tool0's own (z out of the flange). Step size and Frame are
dropdowns on the laptop and rows of buttons on the phone. Turning keeps the tip in place once the offset is known (until then it
turns about tool0), so after the first pose a new tilt is a few turn clicks and a small shift.
Each step is a short joint move on the joint controller, solved on the URDF; a step that needs more than 10° on a
joint (the wrist near straight) is refused: move the arm away by free drive. Only **area, e-stop** red stops free
drive and the tool pad (as the D-pad in Punch); other chips only warn. They are off while a pose is recorded, hold the
robot's lease only while they move it (a tool step until the arm has stopped), and end with Stop all.
**Clearance** below the pad, ticked, sends a step only once the robot is found 5 mm clear of anything, itself
included, and soft stops it closer; a step from too close is refused, nothing sent. Untick it to put the tip on its
target. Free drive is never checked.

| Check | Level | Meaning |
|---|---|---|
| Setup (second row) | amber | Tool on flange or Cameras calibrated does not hold: unticked, ticked over an hour ago, or Tool on flange ticked for another tool model: tick it there (Record tip pose asks at Go first) |
| area, e-stop (second row) | red | Area clear or E-stop at hand not ticked within the hour: free drive and the tool pad refuse until they are ticked again in Setup |
| tool model (second row) | red | the monitor models a tool on the flange, not the bare arm the punch needs: restart it with the `-p tools:=...` line it shows |
| pendant TCP (second row) | amber, grey while the arm moves | the TCP the arm reports is not tool0 by the kinematics: set the pendant's active TCP to zero (the old punch procedure set it) |
| poses | red under 4 | three solve the offset, the fourth checks it |
| tilt | amber under 15°, red under 5°, grey under three poses | the poses' RMS turn from their mean about the axis they turn least about, about a third of how far they are turned apart: along it the offset is unknown; ±30° about two axes gives about 18° |
| fit | amber over 0.5 mm, red over 1.5 mm, grey under three poses | each pose's tip from the point that fits them all |
| offset | red outside 50–300 mm, grey under three poses | the length of the solved offset |
| in use | amber, grey without a fit | Punch uses another TCP offset than this fit: Done makes it Punch's |

### 9. Punch

Optional, and the old procedure's last check: a punch tip touches one target, fixed in the room (not Tip offset's on
the robot), from several base spots; under a good calibration every take puts the tip at the same point in the world. Each take keeps the base rigid body's own
pose, the joints, the TCP offset used, the calibration the relay applied and the lab frame (`lab_from_raw`), so the
takes are judged under the current solution (or, without one, the calibration the relay applies) and the TCP offset
now: a new solution, another offset or Apply keep them. A floor calibration applied or a refit used moves the lab frame
the tips are compared in: the takes from before go out of date (a move under 1 mm at the lab's corners does not
count). Takes, validation samples and tip poses do not depend on it: each uses the tool or arm relative to the base.

1. **TCP offset (mm)** shows the offset from Tip offset. Without it, enter one in mm (Belle's pendant 4-point gave
   0.58, 1.58, 110.54). The **TCP offset** check is red outside 50–300 mm: a value in metres or a slipped digit.
2. Per take: drive to a new spot with the D-pad, bring the tip near the target with free drive and put it on the
   target with the tool pad (as in Tip offset), then **Record punch take**: once the base and joints stand still for
   0.5 s, 1 s of them is averaged.
3. If the target moved, press **Target moved**: the takes so far go out of date.

Punch measures; it has no marker precondition. Below its buttons, **markers** shows the base body's marker error now
(the **base markers now** chip, red over 2 mm or while the base is not tracked, with the number on its own line
below the chips) and the one stored with each take (unknown, "?", for a take from an older punch
file). A take made while it was red is still recorded, and counted in the **takes** check's detail, e.g. "1 taken with the
base marker error over 2 mm".

| Check | Level | Meaning |
|---|---|---|
| Setup, area, e-stop (second row) | amber, red | as in Tip offset; the D-pad too refuses while area, e-stop is red |
| TCP offset (second row) | red unset or outside 50–300 mm | the length of the offset; Record punch take needs it |
| tool model, pendant TCP (second row) | red, amber | as in Tip offset |
| takes | red under 4 | 4–6 is the old target; takes from before a base refine, a remount, a target move or a lab frame move are out of date and left out; takes made while the base marker error was red are counted (hover) |
| tip spread | amber over 2 mm, red over 5 mm, grey under two takes | every tip's distance from their mean, under the current solution and TCP offset (Belle 20261013, taken with the old scripts: 1.53 mm) |
| headings | amber under 90°, grey under two takes | the range of base headings over the takes |
| as taken | amber over 2 mm, grey under two takes | end to end: the tips as taken, under the calibration the relay applied and the TCP offset used then |
| relay up (second row) | red | the relay announced no calibrations yet; Record punch take needs to know what it applies, not that it applies one |
| frame (second row) | red, amber | as in Setup, but red too before the relay announced a floor calibration: each take stores the lab frame its tip is compared in, and goes out of date once that frame moves. Amber (not checked yet, a refit pending) and grey (no floor calibration applied) do not refuse: the take stores the frame as it is, and a refit used or a floor calibration applied later puts it out of date |

After a take that is out of date, the next take starts a new file, and the old file moves to `archive/`. Takes from a
punch file written before the base body's own pose was kept are judged under the calibration applied when taken.

The takes go to `punch_validation/` in the old monitor's format, so `4_punch_validation.py` still analyses them (each
take with the TCP offset it was taken with; the script refuses a file whose takes used different offsets).

### 10. Overview

The run's result in one place, on the laptop: a row per step with its chips (hover for why), the headline numbers
(change vs applied, the axis fits, the validation spread, the TCP offset) and the session folder with the figures in
each of its folders. Its **required** chip is the worst state of Setup to Apply; its **optional** chip that of Tip
offset and Punch, never worse than amber. It is grey until Apply. **Write figures again** rewrites Solve's,
Validate's and Punch's figures and the analysis from the current solution and takes. Validate writes its own whenever
a sampling ends, also when the next spot's **Sample poses here** replaces it. The figures also show in the
**Results** tab at the right edge.

## On the phone

The phone page (`http://<control PC>:8081`) has a **Calibration** tab with the same run: the overview as one row of
number chips (solid or dashed in the colour of each step's state as on the laptop, the current one ringed blue and
named with its state above its chips, the expired preconditions it needs outlined), Back, Done and Next, the step's
chips, its controls and its instructions. Back, Done and Next on either page move both; a tick in Setup's checklist on
one page ticks the other.

What each step offers on the phone, besides its readout:

| Step | On the phone |
|---|---|
| Setup | the checklist (arm, run and tool body on the laptop) |
| Marker error | the D-pad and Recheck (refining needs Motive: on the laptop) |
| Sweeps | Check collisions, e.g. after moving the base (the hits and the take preview show in 3D on the laptop) |
| Solve, Apply | nothing: on the laptop |
| Record | nothing: each move waits for Go on the laptop, after its preview |
| Validate | the D-pad, to drive to the next spot; Sample poses here on the laptop |
| Tip offset | Free drive (hold), Keep free drive on, the tool pad, Record tip pose, Punch remounted |
| Punch | the D-pad, Free drive (hold), Keep free drive on, the tool pad, Record punch take (the TCP offset on the laptop) |

Go, Auto and Preview are on the laptop only: a move starts where its ghost can be previewed. While a move waits for
Go, the phone shows the question with "Go on the laptop". **Cancel** and **Stop all** work from the phone. The D-pad
and the tool pad have their own speed and step size on each page; their **Clearance** is one setting on both.

## Resuming a run

A run's progress is saved after every change, so a crash or restart loses at most the take in progress. Moving between
steps saves nothing and changes nothing.

- The **Run** dropdown in Setup lists the arm's runs from every session folder, newest first, with their session,
  start time, takes and whether they are finished (applied) or in progress. **Refresh runs** reads the folders again,
  e.g. once a simulator has announced itself.
- Choosing a run rebuilds it from its run file and take files: sweeps, tool body, marker errors, collision results,
  takes, the solution if its file is unchanged, validation. **New run** forgets it; the new run's file starts with the
  first thing recorded.
- A resumed run opens where Done would go, at the work that is left. The goal is fixed: the calibration (Setup to
  Apply), then the punch check (Tip offset, Punch). From the first step, the work left is at the earliest step with a
  result (Record, Solve, Validate, Apply, Tip offset, Punch) that is missing, out of date, failing without an
  acceptance, or pending. The preconditions (Setup, Marker error, Sweeps), Tip offset's **not in use** and live
  conditions (mocap, the relay's live state, soft stop, tool model, pendant TCP) never count as work by themselves.
  If that step needs a precondition that has expired, it opens at the earliest such step instead, and Done there
  goes on to the next one the step needs, then to the step:

  | Step | Needs, still holding |
  |---|---|
  | Marker error, Tip offset, Punch | Setup's checklist (ticked within the hour, Tool on flange for this tool model) |
  | Validate | Setup's checklist, marker errors measured within the hour and not red |
  | Record | Setup's checklist, marker errors within the hour and not red, collisions checked where the base stands |

  A precondition that has expired is amber in its own step and in the second row of each step that needs it, which
  asks at Go before acting; only Record's collision check refuses. After a restart, Setup's checklist is amber once a
  tick is over an hour old (naming the box and when), and red only for a box never ticked. A finished run (every step
  up to Apply passing or accepted) relaunched with the punch within the hour of its ticks opens at Setup with only
  Tool on flange to tick, then Done goes on to Tip offset.
- The checklist's ticks are about the arm, not the run: another run of the same arm keeps them.
- Results depend only on what they were made from: takes on the tool body and the base body's definition, the solution
  on the takes, validation on the tool body and both bodies' definitions (each sample is judged again under whichever
  solution is current, so a new solution needs no new samples, nor does another calibration in the relay), Apply on the
  solution, tip poses on the punch mount (**Punch remounted**), punch takes on the base body's definition, the punch
  mount and the target (**Target moved**), judged again under the current solution and TCP offset. A result accepted
  with Done pressed twice stays accepted while what it was calculated from stays the same.
- The collision results hold only while the base stands within 2 cm and 2° of where they were checked. If it moved,
  Sweeps shows **base** amber and Record refuses: check collisions again, and the takes already recorded still count.
- A resumed run keeps writing into its own session folder, also on a later day.

## Files

Through Motive's relay: `<Drive>/data_experiment/calibration_data/<YYYYMMDD>/`, where the relay looks. Through a
simulation's relay (`sim:=true`): `<Drive>/data_experiment/sim/calibration_data/domain_<ROS_DOMAIN_ID>/<YYYYMMDD>/`,
which only a simulation's relay reads (see below), never the robots'. The relay decides, as it reads Motive or a
simulation, never both. A robot whose base or arm is simulated while the relay reads Motive (a mock arm on a real
base, say), or the other way round, writes nothing: its run would land where the other relay applies it. Setup's
**Drive** chip says why.

```
<YYYYMMDD>/
  run_<digits>_<arm>_<HHMMSS>.json                    the run's progress (below)
  j0/calibration_<HHMMSS>_<digits>_<arm>_J0_take<n>.json   one take each
  j1/calibration_<HHMMSS>_<digits>_<arm>_J1_take<n>.json
  validation/calibration_<HHMMSS>_<digits>_<arm>_validation.json
  base_frame_calibration.json                          the arm base and both axis lines
  calibrated_transformation_<digits>_rhino_in_progress.json   Solve's result, until Apply approves it
  calibrated_transformation_<digits>_rhino.json        what the relay applies, with mocap_id (approved by Apply)
  arm_mount_check_<digits>_<arm>.json                  instead, for an arm that only checks its mount
  archive/<earlier output>_<HHMMSS>.json               earlier results, ignored by the relay
  j0/archive/ j1/archive/ validation/archive/          files the run no longer uses, read by nothing
  punch_validation/punch_validation_<arm>_<YYYYMMDD_HHMM>.json   punch takes ("single", "left" or "right")
  analysis/<digits>_<arm>/solution.json, summary.txt   the plugin's own analysis
```

- `<digits>`: the robot's four serial digits, e.g. 0805.
- Take files use the old pipeline's format (`raw_data`: `joint_conf`, `base_mocap_pose`, `flange_mocap_pose`) plus,
  per sample, the published `base_footprint`, the calibration the relay applied and its hash, marker errors, spread
  and time. `base_mocap_pose` is the rigid body's own pose, the applied calibration taken off.
- Take files are never overwritten.
- The run file holds the sweeps as the run started (editing `sweeps.yaml` later cannot renumber its takes), the marker
  errors and when and with which body definitions they were measured, when each box of the checklist was ticked (and
  Tool on flange for which tool model), the collision results and base pose, its take and validation files with what
  each was measured with (tool body, base and tool body definitions), the hash of its result, the tip offset's poses
  (joints, base and punch mount count), the punch's TCP offset, the punch mount and target counts, each punch take's
  base body definition, mount and target counts, whether it is finished (applied, or a mount check written), the
  acceptances (per step, a stamp of the inputs accepted; older files' `overrides` are kept but used for nothing), On
  stand and the notes.
- Punch files keep the old monitor's keys (`base_pose`, `world_from_punch_tip`, `tool0_from_punch_tip`, `joint_conf`)
  plus, per take, `base_mocap_pose` (the base rigid body's own pose, the applied calibration taken off),
  `applied_calibration` (its hash and pose), `marker_errors` (the base body's, as the relay reported it) and
  `lab_from_raw` (the lab frame its poses are in, as the relay applied it; null if not known).
- The relay passes over a session file measured on another arm than its default's (e.g. an old right-arm session
  of Cindy's), so only the robot's first arm ever calibrates its base.
- The old monitor (`yh/multi-robot-monitor`) writes into the same sessions; both read each other's files.
- Everything but `analysis/` is in the old pipeline's formats, so the old scripts run on the session as it is.
  `analysis/<digits>_<arm>/` is the plugin's own (written at Solve and after each validation spot): its numbers in
  `solution.json` (result, axes, every take's circle and file, validation, mount) and the summary lines. The old
  scripts never read or write it; new analysis goes there.

## Testing in a simulation

The simulated robot publishes what is truly there: husky_sim the base's true base_footprint and, with the marker on an
arm (`tool:=calibration_marker`), that arm's true tool0, from the arm's mount in the URDF and the arm driver's TF. A
fake Motive (`sim_motive`) between them and the relay adds what Motive does: an offset between each body's frame and
the true pose, noise, dropouts, drift and a delay. The relay (`sim:=true`: the simulation's inputs only) and the
monitor run as with the real robots; the simulator and the fake Motive come and go. Three terminals, in a ROS domain of
their own:

```bash
export ROS_DOMAIN_ID=70
ros2 launch husky_assembly_teleop mocap.launch.py sim:=true
ros2 launch husky_assembly_teleop sim.launch.py robot:=0805 tool:=calibration_marker fake_motive:=true \
    offsets:="1850:0.004,-0.003,0.002,0,0,0.5"
ros2 launch husky_assembly_teleop monitor.launch.py robots:="belle" plugins:="calibration" \
    tools:="0805:calibration_marker"
```

- `sim.launch.py` starts crl-husky's `sim_robot.launch.py` and, with `fake_motive:=true`, `sim_mocap.launch.py`
  (`sim_motive`), and takes every argument of both (`--show-args`). Every argument, realistic mocap values, what is
  not simulated: [simulation_manual.md](simulation_manual.md).
- `tool` (dual arm: `tool_left`, `tool_right`) puts the calibration marker on an arm, as rigid body `marker_body`
  (1866 by default). `sim_robot` simulates one robot (base and arms, parked); `sim_all.launch.py` every robot, without
  tools.
- `offsets`: per body, the true pose in Motive's body frame (metres, degrees). For a robot's base it is how far the
  true calibration is from crl-husky's default, which the relay starts with: a run should find it (Solve's
  change vs applied). Zero by default.
- Per-body arguments take values for every body, or `<id>:<values>` per body, space separated; a body's own entry wins.
- The relay runs with `sim:=true` (refused in the robots' domain): per rigid body the fake Motive's body wins over the
  simulator's true pose while it streams; Motive is never read.
- It gives the fake Motive's bodies this domain's simulated sessions only
  (`sim/calibration_data/domain_<ROS_DOMAIN_ID>/` on the Drive), so Apply works as on the real system, simulations in
  other domains never mix in, and no robot's relay ever applies a simulated session. A simulator's true pose gets no
  calibration, so a simulation without the fake Motive never depends on a calibration applied earlier.

Mocap imperfections, all off by default, are further arguments of the fake Motive (`sim.launch.py` with
`fake_motive:=true`, or `sim_mocap.launch.py`), comma separated numbers:

| Argument | Unit | Simulates | Exercises |
|---|---|---|---|
| `delay_sec` | s | Motive's latency: each body reaches the relay late, stamped with when it was true | stamps, still checks |
| `noise` (per body) | m, ° (std per axis) | Motive's jitter on a body | still check while sampling; take fit, validation spread |
| `noise_seed` | integer | the same noise and dropouts again | repeatable runs |
| `marker_error` (per body) | m | a badly defined rigid body in Motive (0.3 mm by default) | Marker error: amber over 1 mm, red over 2 mm |
| `dropout` | chance per sample | a body lost (markers hidden): pose held, marked invalid | sampling waits, then asks Go to retry after 20 s |
| `dropout_burst_s` | s | how long each loss lasts | as `dropout`, with realistic occlusions |
| `drift` (per body) | m/min × 3, °/min × 3 | the body's frame creeping: the tool body sliding on the flange (loose clamp), the base tracker on the robot | take fits, axes, validation spread grow over the run |

For example `noise:="0.00003,0.002 1866:0.00003,0" dropout:=0.002 dropout_burst_s:=0.5`.

The fake Motive also gives each body a definition and announces the lab's cameras, as Motive's model descriptions do
([mocap_relay.md](mocap_relay.md), "Simulated Inputs"). To try a refine: `ros2 param set /sim_motive refines
"1850:1"` (the base; the marker body is 1866). The run's takes, validation and punch takes go out of date and to
`archive/`, a refine started in Marker error ends, and once Applied the relay warns that the calibration was built for
an earlier body (the base's mocap chip turns amber). `refines:="1850:0"` puts the definition back.

The arm itself is the simulated robot's, a `sim_robot` argument (also `sim.launch.py`'s):

| Argument | Unit | Simulates | Exercises |
|---|---|---|---|
| `joint_offsets` | ° × 6, shoulder_pan to wrist_3 | the arm's encoder zeros off: the true arm stands that far on from what it reports, kinematics unlike the URDF (the marker only; the arm's joint states stay) | Validate; Solve's axes |

`"<values>"` for every arm carrying a tool, or `"<arm namespace>:<values>"` per arm (`left_ur5e`), space separated.

- ! Keep noise small: sampling waits until the tool's spread on the base is under 0.2 mm over 10 readings, which
  holds up to about 0.07 mm of combined noise per axis. Base angle noise counts at the tool's distance: 0.004° is
  0.07 mm at a metre; it turns the base about base_footprint, where a real tracker turns about itself.
- Drifts are rotation vectors in the body's frame and start when the fake Motive does.

## Solving again offline

```bash
python3 src/husky-assembly-teleop/scripts/solve_calibration.py 20261013 --robot 0805
```

- Takes a session name under the Drive (`HUSKY_DRIVE_ROOT`) or a folder path. `--robot` and `--arm` default to the
  session's `config.yaml` (old pipeline sessions only); for Cindy without `--arm` it takes the left arm.
- Prints each circle, both axes and the result, compares it with the session's calibration file, and checks it on the
  session's `validation/` samples.
- Reads every take file in `j0/` and `j1/` (by the joint it turns, duplicates once), so two arms of one robot on the
  same day mix.
- `--write` writes the session's outputs, replacing them without archiving.
- `--old-scripts` also runs the old scripts 0 to 4 on a temporary copy of the session (the session is never
  written): the takes the plugin solved, each in the folder of the joint it turns, the validation and punch takes and
  `config.yaml`. It prints how far their result is from the plugin's, and their circles from the plugin's circles.
  Run it after every lab session: the two must agree to well under a micrometre.
- `--truth-error dx,dy,dz,droll,dpitch,dyaw` (metres, degrees): for a simulated session, the base's `offsets` entry
  it ran with (sim_mocap.launch.py); prints how far both results are from the true calibration.

```bash
python3 src/husky-assembly-teleop/scripts/solve_calibration.py \
  "$HUSKY_DRIVE_ROOT/data_experiment/sim/calibration_data/domain_72/20261021" \
  --old-scripts --truth-error 0.004,-0.003,0.002,0,0,0.5
```

## Known limits

- **The old pipeline's math.** It assumes J1 crosses J0 at right angles, as on a stock UR arm. The robots' URDFs
  carry each arm's factory calibration, where they do not quite: the result has a model bias of about 0.5 mm
  (Belle, simulated without noise: 0.48 mm and 0.018° from the true calibration, the old scripts and the plugin
  alike).
- **The calibration marker is the arm's tool only if launched so** (`data/tool_urdf/calibration_marker.urdf`: a
  150 × 140 × 10 mm plate on a 63 × 7 mm disk). Launch the monitor with the marker as the arm's tool
  (`-p tools:="['0805:calibration_marker']"`), or the checks see whatever tool it was launched with.
- Free drive and the tool pad are tried in tests only, not yet on a robot.

## Old pipeline

The old monitor's data collection is on `origin/yh/multi-robot-monitor`. Its offline scripts, `0_circle_fitting.py`
to `4_punch_validation.py` in `data/calibration_data/`, are yh's versions; they read the sessions folder from
`CALIBRATION_DATA_DIR` (default: the Drive's) and the session from `CALIBRATION_DATE` (default: `DEFAULT_DATE_FOLDER`
in `config_loader.py`), and need `circle-fitting-3d` and `scikit-spatial` (`requirements.txt`). Run them with the
venv and ROS sourced, or through `solve_calibration.py --old-scripts`.

On the same takes, the old scripts and the plugin's `fit.py` agree to well under a micrometre
(`test/plugins/calibration/test_calibration_old_scripts.py`). Against the results stored on the Drive, the plugin
differs only on 20261016, by 0.06 mm: its `j0/` holds a take the old analysis left out.
