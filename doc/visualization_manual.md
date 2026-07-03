# Husky MoCap & Calibration Visualization Manual (Rhino8 / Grasshopper)

This manual covers how to pull MoCap **camera poses** and **calibration data** out of
the robot pipeline and visualise them in **Rhino8 / Grasshopper**, so you can eyeball
camera placement, circle-fit quality, outlier points, and the robot configuration at
each calibration point — all in one shared coordinate frame.

> **Audience**: assumes you have already collected calibration data (see
> [`calibration_manual.md`](calibration_manual.md)) and have Rhino8 installed. Every
> importer runs in **both** the Rhino8 ScriptEditor (Python 3) and a Grasshopper
> GhPython component.

---

## Table of Contents

1. [Overview & Coordinate Frames](#1-overview--coordinate-frames)
2. [Workflow 1 — Visualise MoCap Cameras](#2-workflow-1--visualise-mocap-cameras)
3. [Workflow 2 — Visualise Calibration Points, Circles & Robot Config](#3-workflow-2--visualise-calibration-points-circles--robot-config)
4. [Frames & Origins Reference](#4-frames--origins-reference)
5. [Troubleshooting & Gotchas](#5-troubleshooting--gotchas)

---

## 1. Overview & Coordinate Frames

There are **two independent visualizations** that share the same world frame, so they
overlay directly in one Rhino document:

| # | What you see | Source script(s) | Drive folder |
|---|--------------|------------------|--------------|
| 1 | MoCap cameras (position + orientation) | `export_mocap_cameras.py` → `import_mocap_cameras_rhino.py` | `…/data_experiment/visualise_mocap_camera` |
| 2 | Calibration points + fitted circles + robot skeleton | `visualize_calibration_outliers_rhino.py`, `rhino8_import_outliers.py`, `export_robot_skeleton.py`, `robot_skeleton_viewer.py` | `…/data_experiment/visualise_calibration_to_rhino` |

### 1.1 The one frame everything uses

> **Origin** = the **MoCap (OptiTrack / Motive) origin** — i.e. world `(0, 0, 0)` is
> wherever you set the Motive ground-plane origin.
>
> **Axes** = **Z-up, "rhino" convention**. The raw Motive stream is **Y-up**; the code
> converts it to Z-up with `new = (x, −z, y)` (`mocap_pos_y_up_to_z_up` /
> `mocap_quat_y_up_to_z_up` in
> [`utils.py:37-51`](../husky_assembly_teleop/utils.py#L37), `MOCAP_AXIS_CONVENTION='rhino'`).
>
> **Document units** = **millimeters**. Set this in Rhino once
> (`Tools > Options > Units > Model units = Millimeters`); both workflows assume it.

Why both workflows land in the same frame:

- **Calibration** rigid-body poses are converted to Z-up "rhino" **at capture time** in
  [`husky_monitor.py:4069-4071`](../husky_assembly_teleop/husky_monitor.py#L4069)
  before they are cached and saved — so the saved `base_mocap_pose` / `flange_mocap_pose`
  are already in this frame.
- **Cameras** are stored raw and converted to the same "rhino" frame by the exporter.

So a camera at `(x, y, z)` and a calibration point at `(x, y, z)` refer to the same
physical location. The **only** thing you must keep consistent is units (see
[§5](#5-troubleshooting--gotchas)).

<!-- SCREENSHOT: Rhino viewport showing cameras + calibration circles + robot skeleton overlaid, origin at world (0,0,0) -->

---

## 2. Workflow 1 — Visualise MoCap Cameras

Pipeline: **get camera data → export to JSON/CSV → import into Rhino/GH → view**.

Camera pose data originates from the NatNet **camera descriptions** (Type 5: name +
position + orientation), unpacked in
[`NatNetClient.py:1080`](../husky_assembly_teleop/optitrack/NatNetClient.py#L1080) and
collected by `get_mocap_camera_inventory()`
([`husky_monitor.py:4031`](../husky_assembly_teleop/husky_monitor.py#L4031)).

### 2.1 Where to get camera data

**Option A — live, from the monitor GUI (recommended).** With MoCap connected, in the
PyBullet monitor (calibration mode) click the **`collect cameras data`** button. It
snapshots the cameras, converts them to the Z-up "rhino" frame, and writes a
timestamped `mocap_cameras_<YYYYMMDD_HHMMSS>.json` **and** `.csv` straight into the
`visualise_mocap_camera` Drive folder. Confirm the log line:
`Saved N mocap cameras to …`.

> Handler: `collect_mocap_camera_data`
> ([`husky_monitor.py`](../husky_assembly_teleop/husky_monitor.py)), button registered
> in the calibration block next to `Export calib data to json`.

<!-- SCREENSHOT: PyBullet GUI with the "collect cameras data" button highlighted -->

**Option B — from an existing MoCap experiment take.** Take files under
`data/mocap_experiments/<date>/<session>/takes/*.json` already embed a
`mocap_camera_inventory`. Export it with:

```bash
cd ~/ros2_ws && source venv/bin/activate
python src/husky-assembly-teleop/data/calibration_data/export_mocap_cameras.py <take.json>
# or grab the newest take automatically:
python src/husky-assembly-teleop/data/calibration_data/export_mocap_cameras.py --latest
```

This writes `<stem>_cameras.json` + `<stem>_cameras.csv` into the same Drive folder.

> **Note**: calibration trajectory files (`data/calibration_data/<date>/…`) do **not**
> contain camera inventory — only the GUI button or a mocap_experiments take produce it.

### 2.2 What the export file contains

`*_cameras.json` (the canonical input for Rhino):

```json
{
  "frame": "mocap_origin",
  "axis_convention": "rhino",
  "position_units": "meters",
  "orientation": "quaternion_xyzw",
  "camera_count": 21,
  "cameras": [ { "name": "PrimeX 22 #72300",
                 "position": [x, y, z],
                 "orientation": [qx, qy, qz, qw] }, ... ]
}
```

`*_cameras.csv` has columns `name,x,y,z,qx,qy,qz,qw`. **Positions are in METERS** here;
scaling to the Rhino document happens in the importer.

### 2.3 Import into Rhino8 / Grasshopper

Open
`visualise_mocap_camera/import_mocap_cameras_rhino.py` in the **Rhino8 ScriptEditor**
(or paste into a **GhPython** component) and edit the top of the file:

| Setting | Meaning |
|---------|---------|
| `CAMERAS_JSON` | path to the `*_cameras.json` you exported (use the newest) |
| `UNIT_SCALE` | meters → doc units. **`1000` for a millimeter document** (use `100` for cm, `1.0` for m) |
| `AXIS_LEN`, `VIEW_LEN` | orientation-axis / view-direction line lengths, in **meters** (scaled by `UNIT_SCALE`) |
| `SHOW_LABELS` | text dot with the camera name |

Run it. The script bakes into the active Rhino document (the
`sc.doc = Rhino.RhinoDoc.ActiveDoc` line makes this work in both ScriptEditor and
GhPython).

### 2.4 What you see

On layer **`mocap_cameras`**, per camera:
- a **point** at the camera position;
- the **orientation frame** — red **X**, green **Y**, blue **Z** axis lines;
- a gray **view-direction line** along the camera's local **−Z** (the OptiTrack look
  axis);
- a text dot with the camera name.

Plus a single point at world `(0,0,0)` on layer **`mocap_origin`** = the MoCap origin.

<!-- SCREENSHOT: Rhino showing ~20 cameras as points with RGB axis triads and gray view lines, origin marked -->

---

## 3. Workflow 2 — Visualise Calibration Points, Circles & Robot Config

Pipeline: **produce `_analysis.json` → stage to Drive → import points + circles →
choose which points show by error distance → export robot skeleton → scrub to a
configuration with sliders**.

All geometry here is in the **MoCap world frame (origin = Motive origin), in mm** — the
same frame as the cameras, so everything overlays.

### 3.1 Produce the analysis file

Run the calibration pipeline (see [`calibration_manual.md` §7](calibration_manual.md#7-running-the-calibration-pipeline)).
Step 1 of the pipeline,
[`0_circle_fitting.py`](../data/calibration_data/0_circle_fitting.py), writes each
batch's `{batch}_analysis.json` into `data/calibration_data/<date>/<batch>/`.

Each `_analysis.json` holds, per take:
- `file_name`
- `raw_data[]` — the **unchanged** capture samples: `base_mocap_pose`,
  `flange_mocap_pose` (both `[[x,y,z],[qx,qy,qz,qw]]`, **meters**, Z-up "rhino" frame),
  and `joint_conf` (6 UR joint values, **radians**);
- the fitted circle `center` + `normal` (in the **base_mocap** frame, meters).

> The analysis file stores the circle fit but **not** per-point error — error is
> recomputed in the viewer (§3.3).

### 3.2 Stage the analysis file to the Drive folder

```bash
cd ~/ros2_ws && source venv/bin/activate
cd src/husky-assembly-teleop/data/calibration_data
python visualize_calibration_outliers_rhino.py --batch both --date 20260622
```

This **only copies** `{batch}_analysis.json` into
`visualise_calibration_to_rhino/<date>/` (no CSV is generated — the viewer reads the
raw analysis JSON directly). Expected output:

```
[j0] staged -> …/visualise_calibration_to_rhino/20260622/j0_analysis.json
[j1] staged -> …/visualise_calibration_to_rhino/20260622/j1_analysis.json
Done. 2 analysis file(s) staged.
```

### 3.3 Import points + circles, and choose which points to show

Open [`rhino8_import_outliers.py`](../data/calibration_data/rhino8_import_outliers.py)
in a **GhPython** component (or ScriptEditor). Set the controls at the top:

| Setting | Meaning |
|---------|---------|
| `RUN_MODE` | `"grasshopper"` for GH DataTree outputs; `"terminal"` to just print |
| `DATE`, `BATCH` | which staged file to read (e.g. `20260622`, `j0`) |
| `THRESHOLD` | **how points are chosen**: `"mean"` = per-curve mean error as the cutoff, or a number in **mm** (e.g. `0.5`) — only points with **distance-to-circle error > cutoff** are emitted as outliers |
| `SHOW_ERROR_DISTANCE` | append ` <err>mm` to each point label |
| `POINT_LABELS`, `LABEL_TEXT_SIZE` | per-point label toggle / text height |

**What it computes** (pure Python, no numpy — runs in GhPython): it rebuilds each
flange point in the base frame, re-expresses points + the stored circle fit into the
**Motive world frame** (using the first sample's `base_mocap_pose` as the reference
base), converts to **mm**, then for each point computes its 3D **distance to the fitted
circle** and keeps the ones above the cutoff. So **you decide which points are
visualised by tuning `THRESHOLD`**.

**Grasshopper outputs** (one DataTree branch per curve/take) — name the component
outputs accordingly:

| Output | Type | Notes |
|--------|------|-------|
| `circles` | Circle | one fitted circle per take |
| `points` | Point3d | the outlier points (above cutoff), grouped per curve |
| `point_labels` | str | `jX_tY #<pt_idx>  <err>mm` — wire to a **Text Tag (3D)** |
| `circle_labels` | str | `jX_tY` per circle |
| `colors` | Color | one per branch — wire to a colored preview keyed by branch |
| `origin` | Point3d | world `(0,0,0)` = Motive origin |

In `"terminal"` mode it prints a summary instead — useful to sanity-check without
Rhino:

```
calibration viewer (terminal)  20260622 j0  frame=mocap_world_mm
[circles] 6 curves
  {0}  j0_t1  r= 550.49 mm  center=( -175.4, -1002.8,  675.8)
  ...
[outliers] 87 points (grouped per curve)
  {0} j0_t1 (cutoff 0.27mm, 14 pts): #5 0.56mm, #6 0.30mm, ...
```

> `pt_idx` is the raw_data index = the point index shown during collection, so it lines
> up with the robot skeleton viewer (§3.5).

<!-- SCREENSHOT: Grasshopper canvas wiring circles/points/labels/colors; Rhino showing colored circles + outlier points per trajectory -->

### 3.4 Export the robot skeleton

To see the **robot configuration** at each point, export a lightweight FK skeleton:

```bash
python export_robot_skeleton.py --batch j0 --date 20260622
```

This loads the robot URDF, sets each point's `joint_conf`, runs forward kinematics,
anchors the URDF flange link onto the measured `flange_mocap_pose`, and writes
`<date>_<batch>_robot_skeleton.json` into the **same Drive folder**. It contains, per
take/point: `joint_conf` (radians), the 8 arm-link XYZ positions (`base_link_inertia →
shoulder → upper_arm → forearm → wrist_1 → wrist_2 → wrist_3 → tool0`, in **mm**,
Motive frame), and the `tool0` quaternion.

> **CAVEAT (important)**: this is an **approximate** placement. It anchors the URDF
> flange link directly on the mocap rigid-body pose, ignoring the fixed
> rigid-body↔flange offset — which is exactly what the calibration pipeline solves.
> Good enough to **eyeball the configuration**; it is **not** a calibrated placement.

### 3.5 Preview a configuration with sliders

Open [`robot_skeleton_viewer.py`](../data/calibration_data/robot_skeleton_viewer.py) in
a **GhPython** component. Set `SKELETON_JSON` to the file from §3.4. Add **two integer
sliders** and wire them to component **input** params named **`TAKE`** and **`POINT`**:

- **`TAKE`** selects the trajectory/curve (0 … #takes−1);
- **`POINT`** selects the point within that take (0 … #points−1).

Dragging the sliders scrubs the robot through every captured configuration. Outputs:

| Output | Type | Notes |
|--------|------|-------|
| `joints` | Point3d list | the 8 link positions |
| `bones` | Line list | links between consecutive joints |
| `tool0_plane` | Plane | tool0 frame (orientation from the stored quaternion) |
| `info` | str | take/point indices + `joint_conf` in radians |

Because the skeleton is in the same Motive-origin mm frame, and `TAKE`/`POINT` use the
same indices as §3.3, the previewed robot **overlays the points and circles** — so you
can drag to the exact point whose error you flagged and see the pose that produced it.

`"terminal"` mode prints the selection for a quick check:

```
robot skeleton  20260622 j0 arm=left (mocap_world_mm)
  6 takes; selected take 0 (j0_traj1) has 31 points
j0_traj1 #0  (take 0/5, point 0/30)
joints[rad]: -3.140, -2.120, -2.040, 1.060, 0.810, -0.020
link positions (mm):
  left_ur_arm_base_link_inertia  (   35.1, -1519.1,  489.6)
  ...
  left_ur_arm_tool0              ( -226.6, -1418.6, 1033.2)
```

<!-- SCREENSHOT: Grasshopper TAKE/POINT sliders driving a robot skeleton (joints+bones) overlaid on the calibration circles -->

### 3.6 Overlaying everything

Run §2 (cameras) and §3 (points/circles + skeleton) into the **same Rhino document**
(mm units). All three share the Motive origin, so cameras, circles, outlier points, and
the robot skeleton appear in their true relative positions.

---

## 4. Frames & Origins Reference

| Artifact | Produced by | Frame / origin | Axis convention | Units | Drive folder |
|----------|-------------|----------------|-----------------|-------|--------------|
| Camera poses (`*_cameras.json/.csv`) | `export_mocap_cameras.py` / GUI button | MoCap (Motive) origin | Z-up "rhino" (x,−z,y) | **meters** in file → mm via `UNIT_SCALE=1000` | `visualise_mocap_camera` |
| `{batch}_analysis.json` raw poses | `0_circle_fitting.py` | MoCap origin | Z-up "rhino" (converted at capture) | meters | `visualise_calibration_to_rhino` |
| Circle fit (`center`,`normal`) | `0_circle_fitting.py` | base_mocap frame | Z-up "rhino" | meters | (inside analysis.json) |
| Outlier points + circles (Rhino) | `rhino8_import_outliers.py` | **Motive world**, origin `(0,0,0)` | Z-up "rhino" | **mm** | `visualise_calibration_to_rhino` |
| Robot skeleton | `export_robot_skeleton.py` + `robot_skeleton_viewer.py` | **Motive world**, origin `(0,0,0)` | Z-up "rhino" | **mm** | `visualise_calibration_to_rhino` |
| `joint_conf` | capture | — (joint space) | — | **radians** | — |

**Bottom line**: every visual lands at the **Motive origin**, **Z-up "rhino"** axes.
Cameras are metric and scaled into mm by the importer; calibration geometry is already
mm. Use a **millimeter** Rhino document and they coincide.

---

## 5. Troubleshooting & Gotchas

- **Two different Drive folders.** Cameras → `visualise_mocap_camera`; calibration →
  `visualise_calibration_to_rhino`. Don't cross the paths.
- **Units must match.** Calibration geometry is mm. For cameras to line up, set the
  Rhino document to mm and keep `UNIT_SCALE = 1000` in `import_mocap_cameras_rhino.py`.
  Wrong scale = cameras 1000× too small/large.
- **`RUN_MODE`.** `rhino8_import_outliers.py` and `robot_skeleton_viewer.py` default to
  `"grasshopper"` (need Rhino). Switch to `"terminal"` to sanity-check from a plain
  shell — they print instead of drawing.
- **Robot skeleton is approximate.** It ignores the rigid-body↔flange offset the
  calibration solves; use it to read the *configuration*, not for metric accuracy
  (§3.4 caveat).
- **`joint_conf` is in radians**, not degrees.
- **`convert_to_rhino.py` is NOT a visualization tool.** Despite the name, it has
  nothing to do with importing into Rhino8. It re-aligns a **runtime calibration
  result** file (`calibrated_transformation_*.json`) from the legacy `'rotated'` axis
  convention to `'rhino'` (right-multiplies the base quaternion by `q_z(−90°)`); it
  belongs to the calibration/runtime path. The Rhino **importers** are
  `rhino8_import_outliers.py` and `import_mocap_cameras_rhino.py`.
- **Importers self-copy to Drive.** `rhino8_import_outliers.py`,
  `robot_skeleton_viewer.py`, and `export_robot_skeleton.py` copy themselves into the
  Drive folder when run from a real file path, so collaborators always have the latest
  viewer next to the data.
- **Camera data missing?** Calibration trajectory files don't carry camera inventory —
  use the `collect cameras data` button or a `mocap_experiments` take (§2.1).
