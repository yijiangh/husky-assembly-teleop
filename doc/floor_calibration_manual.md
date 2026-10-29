# Floor calibration manual

The floor calibration puts Motive's poses into the **lab frame**: z up from the floor, x and y on it, the origin and
axes fixed by marks on the floor and Motive's anchor markers, stored in git in `data/lab/main_frame.json`. You make
it with the mocap probe (`mocap_probe` plugin) and export it to the Drive; the mocap relay applies it to every pose it
publishes, but only when told to (**Apply floor calibration**). How the relay applies and follows it:
[mocap_relay.md](mocap_relay.md#lab-frame-layers-anchors-refits).

Make one:

- when the relay has none (its chip says "no floor calibration");
- after a Motive recalibration, to keep the refit (a refit is used in memory only, and a relay restart drops it);
- when a mark or an anchor moved, or the sanity check (section 5) shows "marks vs relay" over about 1 mm.

## 1. What you need

- **The probe**: rigid body 1863 (choose another in the probe's panel), with its pivot set to the tip in Motive. The
  recorded point is the rigid body's origin, so a pivot off the tip puts every point off.
- **Motive** streaming "Unlabeled Markers": the anchors arrive as those.
- **The relay** running against Motive (README, "Start the mocap relay"): the export needs its cameras and its
  calibration folder, and goes nowhere without it.
- **The Drive** mounted on the laptop (`HUSKY_DRIVE_ROOT`): the export goes to today's calibration session there.
- **The monitor** with the probe: `ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="[]" -p
  plugins:="['mocap_probe']"` (add the robots to have Use refit and Apply refuse while one moves). The probe's
  **Floor calibration** folder on the laptop, or the phone's **Mocap › Floor** tab ([phone_manual.md](phone_manual.md)).
- **The floor** clean where you probe (no tape bumps, cables, mats), the robots out of the way, and the four marks
  (origin, mark 1–3) uncovered. A small dent or cone holds the ball tip better than a pen cross.

## 2. Steps

1. **Pivot check.** Put the tip in a dent and tilt the probe around it in every direction, watching its x y z: they
   should stay within 0.5 mm. If not, set the pivot in Motive first; everything below depends on it.
2. **Floor points.** **Floor point** at 10–12 spots spread over the whole floor: corners, edges, middle. Hold the
   probe still on each. The chip turns "fit good" from 6 points with the plane's tilt known to 0.02°.
   If "max" is over about 3 mm, find that point in the list, check the floor there, select only it and **Delete**,
   and probe again nearby.
3. **Marks.** **Mark › Origin**, **Mark 1**, **Mark 2**, **Mark 3**, each on its mark, twice each (probing again
   averages). The chip turns "fixed n/m", the lab frame's axes appear at the origin mark, and "marks mm" shows each
   mark under about 1 mm.
4. **Anchors.** The anchors stored in the lab frame file are found by themselves: each is the unlabeled marker within
   10 cm of its stored place, read 3 s after the probe was last within 0.5 m of it (the probe and your hand hide or
   shift it). Step back and watch "anchors mm" fill in. Point only at one that says "point" (not found near its
   stored place) or at a new anchor: tip near the marker, **Anchor**, then step away.
5. **Read the numbers** (section 4). Good: "fit good", "fixed" green, every mark and anchor under about 1 mm, "marks
   vs anchors" under about 1 mm.
6. **Hold-out check** (optional): without recording, hold the tip on a few new floor spots, some near the edges, and
   read "probe above floor": each should stay within about the plane's max, and not grow with the distance from the
   points.
7. **Export floor calibration.** On the laptop one press; on the phone two taps within 3 s. With a warning (a mark
   off, an anchor not read) it says why, the button turns orange, and a second press within 2 s exports anyway. The
   file goes to `<Drive>/data_experiment/calibration_data/<YYYYMMDD>/<YYYYMMDD>_<HHMMSS>_mocap_floor_calibration.json`
   (a simulation's relay announces its own folder). Nothing changes for anyone yet.
8. **Apply floor calibration.** Within 30 s the relay finds the new file and the chip "newer floor calibration"
   appears, with how far it would move points of the lab's volume (hover). With nothing moving, press **Apply floor
   calibration** in the laptop's Floor calibration folder. Every pose shifts once by that much; the chip turns "frame
   held" once the relay's fit to the anchors confirms it.
9. **Update the lab frame file** only when a mark or an anchor really moved, or you added one: copy its new place
   from the export (`anchors_in_floor.<name>.floor_xyz` for an anchor, `marks.<name>.floor_xy` for a mark) into
   `data/lab/main_frame.json`, commit it, and restart the monitor (the probe reads the file at start). The file
   defines the lab frame: obstacles, the process and every stored position are in it, so never rewrite it to make the
   numbers look better.

The first calibration, with no marks or anchors stored: the origin mark and Motive's x place the frame (chip "frame
not fixed"); write the export's marks and anchors into the file as in step 9 to make that frame the lab's.

## 3. After a Motive recalibration

Recalibrating (wand, ground plane) or continuous calibration moves Motive's frame, and every raw pose with it. The
relay notices by the anchors and the truss cameras, which stay put, and shows **refit pending**: poses are off by the
amount in its hover until you act.

- **Use refit** (laptop, nothing moving): the relay puts every pose back in the lab frame, in memory. Quick, and good
  for the session.
- A relay restart starts from the floor calibration file again, so the refit is pending again: Use refit after each
  restart, or make a new floor calibration (section 2) to keep it.

## 4. What each number means

The floor section shows a line of chips, then readout lines, the same on the laptop and the phone. The relay's frame
chips and **Use refit** / **Apply floor calibration** are on the laptop only. Hover a chip on the laptop for its
detail.

### Chips

| Chip | Means | Do |
|---|---|---|
| no fit (grey) | fewer than four floor or mark points | probe more |
| fit unusable (red) | the points sit in one spot or on a line, or are not on the floor (tilt over 10°) | spread them out |
| fit noisy (amber) | under 6 points, or the plane's tilt known worse than 0.02° per axis | add points farther apart |
| fit good (green) | the plane is well known; hover: its tilt from Motive's ground plane | — |
| no frame (grey) | the frame cannot be placed yet: probe at least two marks (or point at anchors) | probe the marks |
| frame not fixed (amber) | no marks or anchors stored in the lab frame file: the origin mark and Motive's x place it | first calibration only (section 2) |
| fixed n/m (green) | the frame is fitted to n of the m marks and anchors stored | — |
| fixed n/m (amber) | the same, with an anchor left out as moved (over 2 mm from its stored place; hover names it) | export keeps it; update the file if it really moved |
| `<mark>` off (red) | a mark lands over 2 mm from where it is stored: hit badly, or the mark moved | probe it again, or delete its samples; export asks twice |
| `<anchor>` not found (red) | no unlabeled marker within 10 cm of the anchor's stored place | check it in Motive; point at it, Anchor, step away |

The relay's frame (laptop):

| Chip | Means | Do |
|---|---|---|
| frame held (green) | the relay's fit to the anchors (else truss cameras) confirms the frame in use | — |
| frame unchecked (grey) | just started or reloaded, or no anchors and truss cameras seen: no fit has decided yet | wait a few seconds |
| refit pending (amber) | Motive's frame moved: poses are off by the hover's amount | Use refit (section 3) |
| frame untrusted (red) | the fits disagree or are too uncertain: poses may be off | check in Motive that the anchors are seen and no truss camera was bumped |
| no floor calibration (grey) | poses are raw (Motive's, Z-up), not in the lab frame | make one |
| frame: Drive not read (red) | the relay could not read a floor calibration | check the Drive and the relay's log |
| newer floor calibration (amber) | an export the relay has not applied; hover: how far it would move points | Apply floor calibration |
| frame ? (grey) | no relay announcement | start the relay |

### Readout lines

| Line | Means | Good |
|---|---|---|
| `floor: N pts · rms … · max … mm` | the plane fitted to the floor and mark points: how many, their RMS distance from it, the farthest one | max under about 3 mm |
| `floor normal: ±… per axis` | how well the plane's tilt is known | 0.02° or better |
| `marks mm` | each mark's distance from its stored place, on the floor; "new" probed but not stored, "–" not probed | each under about 1 mm (over 2 mm: off) |
| `anchors mm` | each anchor's distance from its stored place, in 3D; "moved" left out of the fit, "new" not stored, "point" not found near its stored place (point at it), "–" not read yet | each under about 1 mm (over 2 mm: moved) |
| `marks vs anchors: … mm apart` | how far the marks alone and the anchors alone place the lab apart, at the lab volume's worst corner | under about 1 mm |
| `poses off mm: marks M · relay R` | how far poses are off now, against the frame the relay applies: by the marks you probed (M) and by the relay's own fit to the anchors (R, its drift estimate) | both small; M is the truth |
| `marks vs relay: A mm apart` | how far those two estimates are apart | under about 1 mm |
| `probe above floor: … mm` | the probe's height above the fitted plane, live | about 0 on the floor |

The lab's volume is the central 3 m cube where the robots work; distances "at its worst corner" count a small turn as
much as a shift. **Show lab marks and anchors** draws the stored marks as rings of 2 mm radius and the anchors as
spheres of 10 cm (found within) and 2 mm (moved beyond), green or red once read, each read anchor labelled with its
distance and noise.

## 5. Sanity check, any time

Without exporting: **Delete floor points** (phone, two taps; on the laptop select them in the list and Delete), probe
about 6 floor points spread out and the four marks. "marks vs relay" under about 1 mm means the relay's frame is
right; "poses off" M says how far poses are off now. If M is over about 1 mm, Use refit when one is pending, else make
a new floor calibration.

## 6. Good to know

- The plane is where the probe's origin was: with the pivot at the ball's centre, the lab frame's z = 0 is one ball
  radius above the floor.
- Everything is solved raw (the relay's floor calibration taken off), so probing works whatever the relay applies.
  Points loaded from an exported file count too: deleting or loading floor samples changes the calibration.
- The export holds `floor_from_raw`, the plane, the marks, the anchors and Motive's cameras (raw, as the relay needs
  them to follow Motive's frame), each mark's and anchor's place in the lab frame, the three fits (marks, anchors,
  both) and the samples. It is written whole, so the relay never reads half a file.
- Two exports can be compared by their `plane.normal` and `plane.centroid`: normals within about 0.03°, planes within
  about 1 mm at the origin.
- Code: `plugins/mocap_probe.py` (panel), `world/floor_calibration.py` (plane, frame, export), `world/lab_frame.py`
  (the lab frame file and its fit).

## 7. When something does not work

| What you see | Likely cause | What to do |
|---|---|---|
| Floor point or Mark refused | the probe has no fix (big status NO FIX) | get it seen by the cameras; NOISY still records |
| "max" large on one point | dirt, a bump, or the probe tilted | delete that point, probe again nearby |
| A mark "off" every time | the mark moved, or the pivot is off | pivot check (step 1); if it really moved, update the file |
| Anchors stay "–" | the probe is near them, or Motive streams no unlabeled markers | step back 0.5 m and wait 3 s; check Motive's streaming |
| An anchor says "point" | nothing within 10 cm of its stored place: moved far, hidden, or gone | check in Motive; point at it, Anchor, step away |
| Export refused: no cameras | the relay is not running, or Motive does not answer | start the relay; check Motive |
| Export refused: no calibration folder | no relay, or the Drive is not found | start the relay; check `HUSKY_DRIVE_ROOT` |
| No "newer floor calibration" after an export | the relay looks every 30 s, or the export went to another relay's folder | wait; check the relay's log |
| Use refit or Apply refused | a robot moves | wait until everything stands still |
