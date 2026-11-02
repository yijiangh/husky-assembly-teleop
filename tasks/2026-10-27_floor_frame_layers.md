# Lab frame in layers: raw Z-up, floor calibration, robot calibrations (2026-10-27)

Replaces the relay's "Motive's frame at the floor calibration" correction (`relay_correction`, refits relative to it)
with plain layers. Branch `jg/calibration-port`.

## Frames

- **Raw**: Motive's poses turned Z-up. The relay always does this, nothing else underneath.
- **Lab frame**: defined by the ideal positions in the lab's frame file `data/lab/<lab>_frame.json`: floor marks
  (x, y on the floor, probed with the probe) and anchors (x, y, z, Motive's continuous calibration markers). Written by
  hand. `data/lab/<lab>.json` keeps only obstacles and the border; Motive's cameras leave `data/lab/` (the fake Motive's
  copy goes to `data/sim/`).
- **Floor calibration**: `floor_from_raw`, a small rigid correction. The relay applies it to every pose, marker,
  camera and anchor it publishes; robot base calibrations are applied on top.

## Floor calibration (mocap probe)

- Works in raw: each floor and mark sample keeps its raw position (the relay's correction then, undone).
- Plane fitted to the floor and mark samples: z and tilt.
- Position and turn on the plane: least squares over the probed marks and the anchors pointed at, against their
  ideal (x, y). **Anchor** records the tip, named after the nearest stored anchor, else the next free A<n>: a hint
  only. The anchor is the unlabeled marker nearest it within 10 cm (the relay's `~/unlabeled_markers`), read 3 s or
  more after the probe was last within 0.5 m of it; the fits and the export use only that marker, all anchors from
  one reading of the relay, live and at export. Logged: marks only, anchors
  only, both (the one used); residual per point; how far the marks-only and anchors-only fits place the lab's volume
  apart; each anchor's height against its ideal z.
- No ideal positions stored yet: origin mark and Motive's x place it (the first calibration; the file is then written
  from its export).
- The file: `floor_from_raw`, the plane, the marks probed (raw), the anchors seen (raw), the cameras (raw), the fits
  compared, the samples.

## Relay

- Applies `floor_from_raw` of the newest floor calibration, or of its newest adopted refit.
- Publishes Motive's unlabeled markers (`~/unlabeled_markers`, lab frame) and finds the floor calibration's anchors
  among them where it saw them, raw, within 2 cm (after a recalibration, through the cameras' fit).
- Both fits are logged side by side whenever either changes, also while held: to judge the cameras against the
  markers.
- Two rigid fits every look, raw then (the floor calibration's or the refit's) to raw now: anchors, and truss cameras.
  Each reports how far adopting it would move a point of the lab's volume (over 1 mm: refit pending, amber chips),
  its largest residual (warping) and outliers (left out). How far
  the two fits place the volume apart is logged too.
- Adopt (`~/adopt_frame_fit`, Use refit): the anchors' fit if trusted, else the cameras'. floor_from_raw in use =
  the file's ∘ fit, in memory only; the relay writes no file. Every fit is made against the file's anchors and
  cameras; a restart starts from the file again. `~/frame_metrics` carries both fits every look for
  `scripts/mocap_frame_plot.py`.
- A newer floor calibration is never applied by itself: warned about once, announced (`newer`, amber chips), applied
  by `~/reload_floor_calibration` or Apply floor calibration in the probe (refused while anything moves).
  `~/reload_calibration` reloads the base calibrations only.
- The lab frame file's ideal anchors are only for the probe's fit; the relay works from the anchors and cameras
  measured with the floor calibration or its last refit.
- Old floor calibrations (`floor_from_mocap`, `relay_correction`) read as `floor_from_raw` = floor_from_mocap ∘
  relay_correction; their old refits are ignored.

## Robot calibrations

They read floor-calibrated poses with the base calibration taken off (`Reading.base`), as they do: nothing to change.

## Thresholds

1 mm anywhere in the lab's volume (`VOLUME`, `VOLUME_MOVED`) for a frame move and a refit pending.

## Rollout

All published poses shift once by the floor correction (16:00 calibration: about 0.25° tilt, 1 mm) when the new relay
starts: restart it with the robots parked.
