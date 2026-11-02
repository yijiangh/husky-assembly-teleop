# The relay holds the floor calibration's mocap frame across Motive recalibrations (2026-10-24)

Status: built on `jg/calibration-port`; tested in a simulation (fake Motive recalibration), not yet in the lab.

## Why

A Motive recalibration (or a new ground plane) moves Motive's world frame. Everything measured in it before then (the
lab file's obstacles, probe points, the floor calibration) no longer lines up with the robots. The truss cameras are
bolted in place, so where Motive puts them tells how its frame moved.

## Decisions (user, 24 Oct)

- Reference cameras: the six truss cameras only, the four on top and the two lower ones in the far corners
  (#31321, #31322, #31330, #66076, #72657, #72654). Tripods never.
- Refit every time the reference cameras' poses change (expected only on a recalibration) and log each refit.
- Use a refit only when told to (a relay service, and a button in the mocap probe's floor section). Until then the
  relay publishes as before and warns.
- Warn when cameras move while the relay runs.
- Applying a refit keeps the old frame: the relay corrects every pose it publishes by the fitted motion, so poses stay
  in the frame the floor calibration was measured in.
- The fit compares against the cameras stored with the floor calibration (Export calibration stores them). An adopted
  refit is kept with the cameras it was adopted at.

## The fit (`world/mocap_frame.py`)

- Least squares rigid fit (Kabsch) of the reference cameras' positions, the floor calibration's to Motive's now:
  `reference_from_motive`.
- A camera bumped on its own: fitted without it, its position is off by over BUMP_POSITION or its orientation turned by
  over BUMP_TURN; it is left out, worst first, and named.
- Trusted with at least MIN_CAMERAS cameras left (4 of 6); else not used.
- Simulated with the lab's real positions and 0.5 mm per camera: 0.007° and 0.8 mm at the floor (95 %).

## Files (Drive)

- Floor calibration: `data_experiment/mocap_probe/<stamp>_mocap_floor_calibration.json`, now with `cameras` (every
  camera, Z-up, as the relay announced them). A simulation's go to `sim/mocap_probe/domain_<ROS_DOMAIN_ID>/`.
- Adopted refit: `<floor calibration stem>_frame_<stamp>.json` next to it: `reference_from_motive`, the cameras then.
- The relay uses the newest floor calibration with cameras, and its newest adopted refit.

## Relay

- On start and on `~/reload_calibration`: read the floor calibration and its adopted refit; apply the refit.
- On a change of the reference cameras (vs the adopted refit's cameras, or the floor calibration's): warn that cameras
  moved while running, refit, log it, announce it pending on `~/frame` (latched JSON).
- `~/adopt_frame_fit` (std_srvs/Trigger): use the pending fit if trusted, write the refit file, log.
- The correction applies to Motive's and a fake Motive's raw poses and markers, before the base calibrations.

## Monitor

- `MocapRelayInterface`: `state.frame` from `~/frame`; `adopt_frame_fit()`.
- Mocap probe floor section: the frame's state (held, moved: refit pending, not trusted), and Use refit.
- Health: every base's mocap chip amber while a refit is pending (the poses are in a moved frame).

## Simulation

The fake Motive's `recalibration` (x, y, z m, roll, pitch, yaw deg), settable while it runs, moves its reported frame:
every camera and every body together.
