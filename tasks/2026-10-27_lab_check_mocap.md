# Lab check: floor calibration, lab frame, body definitions, cameras (2026-10-27)

Branch `jg/calibration-port` (501a4d1, not pushed). Every feature here is unit- and simulation-tested; none has run
against the lab's Motive (3.0.3.1, NatNet 4.0, 192.168.0.28) yet. Fill in the **Result** columns as you go.

| Part | What it proves | Time | Needs |
|---|---|---|---|
| [0](#0-before-the-lab) | the right software runs | 15 min | laptop |
| [1](#1-floor-calibration--priority) | **floor calibration works and is right** | 45 min | probe, tape |
| [2](#2-lab-frame-held-across-a-recalibration) | the lab frame survives a Motive recalibration | 60 min | a due recalibration |
| [3](#3-cameras) | cameras shown where they are, named the lab's way | 15 min | — |
| [4](#4-body-definitions) | every body change is seen, nothing else | 30 min | Motive |
| [5](#5-calibration-against-its-body) | a robot calibration knows its body | 60 min | one robot |
| [6](#6-overnight-does-the-relay-see-drift) | whether slow drift is visible | overnight | witness body |

Do 0 and 1 first: the floor calibration is what is needed now, and 2 needs its export.

---

## 0. Before the lab

| # | Do | Pass if | Result |
|---|---|---|---|
| 0.1 | `source venv/bin/activate && source /opt/ros/humble/setup.bash && python3 -m colcon build --symlink-install --packages-select husky_assembly_teleop`, then in the repo `python3 -m pytest -m safety` | build ok, all pass | |
| 0.2 | Stop the lab's relay (one relay per domain), start this one, keeping its log: `ros2 launch husky_assembly_teleop mocap.launch.py 2>&1 \| tee ~/relay_$(date +%Y%m%d_%H%M).log` | log: "Motive defines N rigid bodies", "Motive has 10 cameras", "anchors: … from main_frame.json" (or "no anchors in main_frame.json"), "floor calibration at start: …" | |
| 0.3 | Monitor with the probe and the robots in the lab: `ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice','belle','cindy']" -p plugins:="['mocap_probe']"` (`robots:="[]"` for the probe alone) | probe tab: probe tracked; floor section: chip "frame held" (or "no floor calibration" without one) | |
| 0.4 | Optional: the independent watcher, read-only: `python3 scripts/mocap_body_definitions.py watch --log ~/definitions_$(date +%Y%m%d_%H%M).jsonl` | prints the bodies table | |

Keep the relay log and the watcher log; they go into the results folder at the end.

---

## 1. Floor calibration (priority)

The probe works raw (the relay's floor calibration taken off). It measures floor points and the marks on the floor
(origin, mark 1–3), fits a plane to all of them, and places the frame on it by a least-squares fit of the marks probed
and the anchors read, against the lab frame file `data/lab/main_frame.json` (marks stored since 27 Oct,
anchors to write in 1.6b). Marks only and anchors only are fitted too: the panel lines "anchors mm: A1 … · A2 …" (each
anchor's 3D distance from its stored place) and "marks vs anchors: … mm apart" compare them. It exports `floor_from_raw` with the marks,
anchors and cameras raw; the relay applies it to every pose and follows Motive's frame by the anchors and cameras
(part 2).

**Setup**
- Probe rigid body 1863, its pivot set to the tip in Motive (the recorded point is the rigid body's origin).
- Robots out of the way; the floor clean where you probe (no tape bumps, cables, mats).
- The origin and the three marks on the floor, spread wide (not on a line). A small dent or cone holds the ball tip
  better than a pen cross.

| # | Do | Pass if | Result |
|---|---|---|---|
| 1.1 | **Pivot check.** Put the tip in a small dent (or a cross) and tilt the probe around it in every direction; watch the probe's xyz readout | xyz stays within 0.5 mm. More means the pivot is not at the tip: fix it in Motive first, everything below depends on it | spread: |
| 1.2 | **Floor points.** "Floor point" (laptop or the phone's Floor tab) at 10–12 spots spread over the whole floor: corners, edges, middle | chip "fit good" (hover: tilt); lines "floor: N pts · rms … · max … mm" and "floor normal: ±… per axis", N ≥ 6; record them | rms / max / ±: |
| 1.3 | If max is over ~3 mm: find the point (the list), check the floor there, delete it (list → Delete), probe again nearby | max comes down; else the floor is not flat there: note where | |
| 1.4 | **Marks.** "Mark" → Origin, Mark 1, Mark 2, Mark 3, each on its mark (twice each: it averages) | chip "fixed 4/4" (more with anchors); axes in the 3D view at the origin mark; "marks mm" each under ~1 mm | marks mm: |
| 1.5 | **Hold-out check.** Without recording, hold the tip on 6 new floor spots (2 near the edges) and read "probe … mm above the floor" | each within about ±(max of 1.2); none drifts with the distance from the points | values: |
| 1.6 | **Export floor calibration** (on the phone: tap twice) | below the floor section "exported <file>; press Apply floor calibration to use it"; log "exported the floor calibration (N points, A anchors, 10 cameras) … anchors mm: … ; marks vs anchors: … mm apart"; file `<Drive>/data_experiment/calibration_data/<date>/<date>_<time>_mocap_floor_calibration.json` | file: |
| 1.5b | **Anchors.** Point the tip at each of the six anchor markers (within 10 cm) and press Anchor | each: "anchor A<n>: the marker … mm from the tip"; the export (1.6) keeps them, and the relay follows them ("all 6 anchors seen") | |
| 1.6b | **Store the anchors** (by hand, once, right after 1.6 with "fixed" good): copy each `anchors_in_floor.<name>.floor_xyz` of the export into `"anchors"` of `data/lab/main_frame.json`; commit; restart the monitor | line "anchors mm" each under ~1 mm, "marks vs anchors" under ~1 mm | apart: |
| 1.7 | **Repeat.** Phone's Floor tab: "Delete floor points" (tap twice; it deletes the marks too); probe 10 new spots (not the same ones) and the four marks again; export again | each "marks mm" under ~1 mm; marks vs anchors under ~1 mm; each "anchors mm" under ~1 mm; the two exports agree (command below): normals within ~0.03°, planes within ~1 mm of each other at the origin | marks mm / apart / angle / mm: |
| 1.8 | **The relay takes it on command.** Within 30 s it warns "newer floor calibration on the Drive, not applied: it would move points of the lab's volume up to … mm" and the probe shows the chip "newer floor calibration"; with nothing moving, press **Apply floor calibration** (or `ros2 service call /mocap_relay/reload_floor_calibration std_srvs/srv/Trigger`) | answer "floor calibration <file> applied: points of the lab's volume moved up to … mm"; log "floor calibration at reload: <file> …; Motive's frame followed by A anchors and 6 truss cameras"; probe chip "frame held", Apply greyed | |
| 1.9 | **Sanity check, any time later.** "Delete floor points" (tap twice), probe ~6 floor points spread out and the four marks again (no export) | lines "poses off mm: marks M · relay R" and "marks vs relay: A mm apart": A under ~1 mm (the relay's drift estimate matches the marks); M is how far poses are off now | M / R / A: |

Compare two exports (from the repo, venv active):

```bash
python3 - <Drive>/data_experiment/calibration_data/<date>/<first>.json <Drive>/data_experiment/calibration_data/<date>/<second>.json <<'EOF'
import json, sys
import numpy as np
a, b = (json.load(open(path)) for path in sys.argv[1:3])
na, nb = (np.array(x["plane"]["normal"]) for x in (a, b))
ca, cb = (np.array(x["plane"]["centroid"]) for x in (a, b))
# * Older exports name the marks "fixed_points".
origin = np.array((b.get("marks") or b["fixed_points"])["origin"]["position"])
print(f"normals differ by {np.degrees(np.arccos(min(1.0, abs(na @ nb)))):.4f} deg")
print(f"at the origin the planes are {abs(na @ (origin - ca) - nb @ (origin - cb)) * 1e3:.2f} mm apart")
EOF
```

**Known, not tested here**
- The plane is where the probe's origin was: with a ball tip and the pivot at the ball's centre, the lab frame's
  z = 0 is one ball radius above the floor. Measure the ball and note it: ______ mm.
- The relay applies `floor_from_raw` to every pose: when it first does, every published pose shifts once by the
  floor correction (about 0.25° of tilt for the 16:00 calibration). Restart it with the robots parked.

---

## 2. Lab frame held across a recalibration

When Motive's frame moves (a recalibration), the relay fits how, by the anchors and by the truss cameras, and, only
when told, composes the fit onto floor_from_raw so every pose is back in the lab frame. Do this when a recalibration is due anyway, e.g. when
re-aiming TFL #31321 and TFR #66076 as the camera planner proposes; it costs a re-wand.

Witness: a static rigid body that never moves (screwed down, ≥ 4 markers), and the four crosses of part 1.

| # | Do | Pass if | Result |
|---|---|---|---|
| 2.1 | Part 1 done and held (1.8). Record the four crosses as points (Label "cross"), export them; note the witness's pose | baseline | |
| 2.2 | Recalibrate in Motive: wand, then set the ground plane | relay: "CAMERAS MOVED" box with the truss cameras' move together, and any camera moved against it ("TFL #31321 (truss): …"), then a "REFIT PENDING" box: a row per fit (anchors, truss cameras: moved X mm, turned Y°, lab vol. V ± … mm, …) and "the two fits place the lab's volume up to Z mm apart"; probe chip "refit pending" (hover: the deciding fit, "poses off now … mm; …"); every base's mocap chip amber | X / Y / V / Z: |
| 2.3 | Before Use refit: probe the crosses again, note the witness | they moved by about X / Y | |
| 2.4 | Use refit while a robot drives or an arm moves (the monitor needs the robots, 0.3) | refused: "refit not used: <robot> … ; every pose changes, so only while nothing moves" | |
| 2.5 | Use refit with nothing moving | "refit adopted, in memory: …"; chips green; "frame held"; no file written | |
| 2.6 | Probe the crosses again, note the witness | each cross within ~1 mm of 2.1 | per cross: |
| 2.7 | Restart the relay | log "floor calibration at start: …"; a refit pending again (the file's floor_from_raw); Use refit puts the crosses back | |
| 2.8 | If cameras were re-aimed in 2.2 | the refit names them as left out ("TFL #31321 bumped …") and fits the others (≥ 4) | |

If 2.6 is off by more than ~1 mm: note it per cross. A pattern growing with distance points to a scale change from the
re-wand (the fit is rigid, no scale); one cross off points to the probe.

---

## 3. Cameras

| # | Do | Pass if | Result |
|---|---|---|---|
| 3.1 | Probe: Show cameras | 10 frustums, each at its camera, labelled the lab's way (TFL, TFR, TBL, TBR, BFL, BML, BBL, BFR, BMR, BBR, each with its serial) | |
| 3.2 | Walk to each camera and check its label | the label matches the camera (front = −y, left = +x facing the front). BBR #72653 hangs 1 m back of the centre: confirm the name | wrong ones: |
| 3.3 | Views on the floor, at 0 m | regions by how many cameras see them; compare one camera's footprint with its 2D view in Motive | the fields of view per model are from memory: note if footprints are too big/small |
| 3.4 | Leave everything alone 10 min, anchors on | no "CAMERAS MOVED" warning in the relay log. With continuous calibration an info box "the anchors hold the lab frame: poses unaffected" may come: note the truss cameras' move and which cameras moved against them (12:50 on 27 Oct: frame 1.8 mm, TBL #31330 and BML #72652 2–3 mm against it) | |

---

## 4. Body definitions

The relay reads Motive's body definitions every 2 s. One change at a time; wait 4 s before the next. It keeps no
history: a body missing for one reading comes back as "body gone", then "new body".

| # | Do in Motive | Relay logs | Result |
|---|---|---|---|
| 4.1 | Rename a body | `renamed` (info) | |
| 4.2 | Refine the tool body (Builder, Refine, Apply) | `refined` (warn, "data measured with it is out of date") | |
| 4.3 | Move its pivot, then turn it | `frame moved` (warn), twice | |
| 4.4 | A body made from other markers on an existing id (the probe's on another's id) | `other body` (warn) if swapped within a reading, else `body gone` + `new body` | |
| 4.5 | Make a new body from loose markers | `new body` | |
| 4.6 | Delete it | `body gone` | |
| 4.7 | Change a body's streaming id | nothing (same markers); `/mocap_relay/body_definitions` lists the new id | |
| 4.8 | Save the project, restart Motive, load it | nothing after the reconnect | |

---

## 5. Calibration against its body

Robots parked: a base refine shifts the poses every consumer of the relay gets.

| # | Do | Pass if | Result |
|---|---|---|---|
| 5.1 | Calibrate one robot to Apply (calibration plugin) | calibration file has `body_definition`; relay "its calibration matches the body definition" | |
| 5.2 | Rename its base body | still matches | |
| 5.3 | New run, record two takes, then refine the base body in Motive | within 4 s: takes out of date and in `archive/`; relay box "CALIBRATION FOR AN EARLIER BODY"; that base's mocap chip amber (hover says why); Apply's "relay now" amber | |
| 5.4 | Put the old definition back (load the project saved before 5.3) | matches again; the takes current again | |
| 5.5 | Delete the base body and make it again from its markers, same id | wrong: `redefined` or `refined` | |
| 5.6 | Refine only the tool body | validation samples out of date; takes and the calibration unaffected | |

---

## 6. Overnight: does the relay see drift?

Continuous calibration on, anchor markers removed (the 9 Oct weekend drifted 0.2°/h). Floor calibration held (1.8),
witness in place, nobody in the room. The relay's frame log keeps every look (fits, raw markers, truss camera poses)
in `~/.ros/mocap_frame/<YYYYMMDD>.jsonl` by itself; record only `ros2 bag record /mocap_relay/frame
/mocap/rigid_body/id_<witness>/pose` and the relay log. In the morning: `python3 scripts/mocap_frame_plot.py --file
~/.ros/mocap_frame`, and copy the night's day files with the results.

- **Motive sends live camera updates**: "CAMERAS MOVED" (or the info box) about every 30 min, refits matching the witness's drift;
  morning Use refit puts witness and crosses back.
- **Witness drifts, cameras never change**: Motive 3.0 does not send continuous-calibration updates. The relay then
  catches only recalibrations and bumps, and continuous calibration stays on only with anchors.

---

## Results

Logs, bags and the exports to `<Drive>/data_experiment/mocap_lab_check/<date>/`.

| Part | Pass / fail | Notes |
|---|---|---|
| 0 | | |
| 1 | | |
| 2 | | |
| 3 | | |
| 4 | | |
| 5 | | |
| 6 | | |
