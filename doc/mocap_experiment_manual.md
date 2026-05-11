# MoCap Experiment Manual

This document describes the workflow for:

- collecting raw MoCap takes from the Husky monitor
- attaching workspace and marker-reference images
- capturing webcam timelapse video during each take
- analyzing the collected takes
- generating a markdown report for comparison across configurations

## Scope

The current workflow records:

- raw MoCap rigid-body poses
- the selected robot base rigid body as the analysis target
- one workspace image from the webcam at take start
- a webcam timelapse at `0.5 s` intervals during the take
- a NatNet camera inventory snapshot at take start

The current workflow does not automatically infer which Motive cameras are enabled. The field `take.camera_configuration` is still the authoritative manual label for that experiment condition.

## File Locations

- Main config: [data/mocap_experiments/config.yaml](/home/yijiangh/ros2_ws/src/husky-assembly-teleop/data/mocap_experiments/config.yaml)
- Config template: [data/mocap_experiments/_template/config.yaml](/home/yijiangh/ros2_ws/src/husky-assembly-teleop/data/mocap_experiments/_template/config.yaml)
- Monitor node: [husky_monitor.py](/home/yijiangh/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/husky_monitor.py)
- Analysis/report module: [mocap_experiment.py](/home/yijiangh/ros2_ws/src/husky-assembly-teleop/husky_assembly_teleop/mocap_experiment.py)

Session output is saved under:

```text
data/mocap_experiments/<date>/<session_name>/
```

That session folder contains:

- `takes/`: raw take JSON files
- `analysis/`: plots, CSVs, summary JSON, markdown report
- `photo_library/`: webcam stills and manually added phone photos
- `reference_media/`: per-take copied reference media and timelapse video
- `manifest.json`: session take index

## Environment Setup

### Build terminal

```bash
cd /home/yijiangh/ros2_ws
source venv/bin/activate
python3 -m colcon build --symlink-install
```

### Running terminal

```bash
cd /home/yijiangh/ros2_ws
source venv/bin/activate
source install/setup.bash
ros2 run husky_assembly_teleop husky_monitor
```

## Before Starting a Session

Open [config.yaml](../data/mocap_experiments/config.yaml).

Keep the `experiment` section stable for the full session:

```yaml
experiment:
  name: "mocap_cover_study"
  session_name: "20260311_cover_study"
  operator: "your_name"
  duration_sec: 20.0
  notes: "baseline and deck-cover trials"
```

Recommended rule:

- `experiment.name`: broad project name
- `experiment.session_name`: one session per day or per campaign
- `experiment.duration_sec`: leave at `20.0` unless intentionally changing the protocol

## Per-Take Procedure

Before every take, edit only the `take` section in [config.yaml](/home/yijiangh/ros2_ws/src/husky-assembly-teleop/data/mocap_experiments/config.yaml).

Example:

```yaml
take:
  take_id: "ws_a_mk4_camall_open_t01"
  workspace_position: "A"
  marker_configuration: "mk4"
  camera_configuration: "cam_all"
  cover_configuration: "open"
  trial_index: 1
  notes: "baseline, robot stationary"
  reference_images:
    overview: ""
    workspace: ""
    markers: ""
    camera: ""
    cover: ""
    extra: []
```

### Meaning of the fields

- `take_id`: canonical unique identifier for the take
- `workspace_position`: your workspace placement label
- `marker_configuration`: your marker setup label
- `camera_configuration`: your manual label for the enabled-camera condition
- `cover_configuration`: your cover/deck condition label
- `trial_index`: repeat count within the same condition
- `notes`: anything special about the take

Recommended `take_id` pattern:

```text
ws_<workspace>_<marker>_<camera>_<cover>_t<nn>
```

Example:

```text
ws_a_mk4_camall_open_t01
```

## Webcam Settings

The webcam behavior is configured in:

```yaml
capture:
  workspace_webcam:
    enabled: true
    device_index: 0
    warmup_frames: 8
    role: "workspace"
    timelapse_enabled: true
    timelapse_interval_sec: 0.5
    timelapse_video_fps: 2.0
```

### Test the webcam before the session

In the monitor UI:

- click `Test Webcam Capture`

This writes one test image into the current session `photo_library/`.

Use this button when:

- you changed `device_index`
- you replugged the webcam
- you want to verify framing before data collection

## Collecting a Take

In the monitor UI:

1. Confirm Motive streaming is active.
2. Confirm the correct robot is selected.
3. Update the `take` section in the YAML.
4. Click `Record Raw MoCap Take`.

What happens automatically:

- raw MoCap rigid-body poses are recorded for `20 s`
- the webcam takes one still image at take start
- the webcam captures one frame every `0.5 s`
- those frames are assembled into an `.mp4` after the take ends
- a NatNet camera inventory snapshot is stored with the take

The take stops automatically after the configured duration.

## Images and Media

### Automatically captured workspace media

For each take, the system creates:

- one workspace still image
- one webcam timelapse video

### Manually added phone photos

Use the phone for close-up marker photos.

Copy the phone image into the session `photo_library/` using the same `take_id` prefix.

Recommended filenames:

- `<take_id>__markers.jpg`
- `<take_id>__cover.jpg`
- `<take_id>__camera.jpg`
- `<take_id>__overview.jpg`

Example:

```text
ws_a_mk4_camall_open_t01__markers.jpg
```

The report generator automatically associates files in `photo_library/` with the take if the filename starts with that take’s `take_id` and uses the `__role` convention.

### Optional explicit links in YAML

You can also point `take.reference_images` to reusable image files explicitly:

```yaml
reference_images:
  overview: "photo_library/20260311/overview_a.jpg"
  workspace: ""
  markers: ""
  camera: ""
  cover: ""
  extra: []
```

At record time, those referenced images are copied into that take’s `reference_media/` folder.

## Session Folder Structure

Example:

```text
data/mocap_experiments/20260311/20260311_cover_study/
├── manifest.json
├── takes/
├── photo_library/
├── reference_media/
└── analysis/
```

Typical contents:

- `takes/*.json`: raw MoCap take files
- `photo_library/*.jpg`: webcam stills and manually copied phone images
- `reference_media/<take>/workspace_timelapse.mp4`: take timelapse video
- `analysis/report.md`: final markdown report

## What Is Stored in Each Take

Each take JSON includes:

- raw MoCap frames for all streamed rigid bodies
- target rigid body name
- take metadata from the YAML
- reference image metadata
- webcam timelapse metadata
- MoCap camera inventory snapshot

## Analysis

The analysis uses the selected robot base rigid body only.

Metrics:

- Euclidean distance from each frame position to the first frame position, in mm
- angle between the current X axis and the first-frame X axis, in deg
- angle between the current Y axis and the first-frame Y axis, in deg
- angle between the current Z axis and the first-frame Z axis, in deg

## Generate Analysis Outputs

Run from the workspace root:

```bash
cd /home/yijiangh/ros2_ws
source venv/bin/activate
source install/setup.bash
ros2 run husky_assembly_teleop mocap_experiment_analyze \
  /home/yijiangh/ros2_ws/src/husky-assembly-teleop/data/mocap_experiments/<date>/<session_name>
```

This creates:

- `distance_mm_boxplot.png`
- `angle_x_deg_boxplot.png`
- `angle_y_deg_boxplot.png`
- `angle_z_deg_boxplot.png`
- `combined_boxplots.png`
- `group_metrics.csv`
- `take_metrics.csv`
- `summary.json`

## Generate Markdown Report

```bash
cd /home/yijiangh/ros2_ws
source venv/bin/activate
source install/setup.bash
ros2 run husky_assembly_teleop mocap_experiment_report \
  /home/yijiangh/ros2_ws/src/husky-assembly-teleop/data/mocap_experiments/<date>/<session_name>
```

This creates:

- `analysis/report.md`

The report includes:

- four boxplots
- grouped metric summary
- MoCap camera inventory snapshot
- reference images
- per-take webcam timelapse video entries
- take file index

## Recommended Daily Workflow

1. Build the workspace.
2. Launch the monitor.
3. Set `experiment.session_name` for the day.
4. Use `Test Webcam Capture`.
5. For each take:
   - edit the `take` section
   - click `Record Raw MoCap Take`
   - optionally copy the phone marker photo into `photo_library/` with the matching `take_id`
6. After the full session, run `mocap_experiment_report`.
7. Open `analysis/report.md`.

## Troubleshooting

### Webcam test fails

Check:

- `capture.workspace_webcam.enabled`
- `capture.workspace_webcam.device_index`
- webcam permissions and USB connection

Then use `Test Webcam Capture` again.

### Report does not show a phone image

Check:

- the file is under the session `photo_library/`
- the filename starts with the correct `take_id`
- the role suffix uses `__markers`, `__cover`, `__camera`, or `__overview`

### MoCap camera inventory missing

The camera inventory depends on NatNet model definitions being available. If the report says the inventory is missing:

- confirm Motive was connected and streaming when the take started
- try another take after reconnecting the monitor

### Wrong grouping in plots

The grouping keys come from:

```yaml
report:
  group_by:
    - "take.workspace_position"
    - "take.marker_configuration"
    - "take.camera_configuration"
    - "take.cover_configuration"
```

Adjust those keys only if you intentionally want a different grouping for plots and summaries.
