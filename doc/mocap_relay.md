# Mocap relay

This document explains the mocap relay and gives a migration guide for older code that used direct NatNet
callbacks (for example HuskyMonitor).

The relay lives here (`husky_assembly_teleop/mocap_relay/`, executable `husky_mocap_relay`,
`ros2 launch husky_assembly_teleop mocap.launch.py`), with its NatNet client (OptiTrack's SDK, `mocap_relay/natnet/`);
both moved from crl-husky. crl-husky keeps what the relay publishes (`crl_husky_msgs`) and each robot's default base
calibration (`crl_husky/config/robots/<serial>/mocap_config.json`).

## What Changed Compared To The Old HuskyMonitor Path

Old path (inside HuskyMonitor):
- Started a NatNet client directly in the application process.
- Performed Y-up to Z-up conversion in the monitor code.
- Applied base calibration in the monitor code.
- Updated robot/object state directly from callbacks.

New path (dedicated relay node):
- One ROS2 node (`mocap_relay`) owns the NatNet client.
- Relay does Y-up to Z-up conversion before publishing.
- Relay resolves and applies calibration (`base_mocap_from_base_footprint`) using config files.
- Relay publishes per-ID ROS topics and a streaming-IDs service.
- Consumers subscribe to ROS topics instead of managing NatNet transport.

## How The New System Works

Data flow:

1. OptiTrack/Motive streams NatNet rigid body data.
2. `mocap_relay` receives callbacks at mocap frequency.
3. Relay turns coordinates Z-up ('rhino' convention, below: raw), then into the lab frame by the floor calibration.
4. Relay resolves calibration for each mocap ID and applies it.
5. Relay caches latest pose/validity per rigid body ID.
6. Relay republishes at timer rate (`publish_rate_hz`) on per-ID topics.
7. Consumer nodes subscribe and use `pose_valid` + `tracking_valid` for logic.

Runtime behavior details:
- One relay per ROS domain: at start the relay looks for another relay's `/mocap/get_streaming_ids` service in its domain for 2 s; if it finds one, it logs an error and exits. Two relays started within those 2 s can miss each other.
- Health monitor: if no frames arrive for `frame_timeout_sec`, relay reconnects.
- Reconnect attempts are rate-limited by `reconnect_min_interval_sec`.
- Streaming state is based on `streaming_stale_timeout_sec`.
- Pose validity is computed from freshness, NatNet tracking validity, and marker error threshold.

## What Must Run

Minimum runtime stack:

1. Motive/OptiTrack NatNet server (on mocap PC).
2. Relay node on control PC:
   - `ros2 launch husky_assembly_teleop mocap.launch.py`
3. Any consumer node(s) that need mocap:
   - Example: the base follower (`pure_pursuit.launch.py`), the monitor, or your own subscriber.

! Exactly one relay per ROS domain, running while mocap is in use. Two relays in one domain
would publish interleaved poses with different delays and possibly different calibrations,
so a second relay refuses to start. Stopping the relay stops mocap for every consumer in
that domain.

Recommended startup order:

1. Start relay.
2. Verify topics/service are available.
3. Start downstream consumers.

## Raw Z-up Convention

NatNet streams Y-up poses. The relay converts them to raw Z-up, the convention that
Rhino design files and the PyBullet planners use (the 'rhino' convention, a +90 deg
rotation about the mocap x axis):

- position `(x, y, z) -> (x, -z, y)`
- quaternion `(qx, qy, qz, qw) -> (qx, -qz, qy, qw)`

Consumers never convert axes themselves; everything published by the relay is
already in this frame. Calibration files (`base_mocap_from_base_footprint`) must be
expressed in the same convention, i.e. computed by the `calibration` plugin (or the old pipeline), which
writes `calibrated_transformation_<digits>_rhino.json`.

! Data recorded in the old 'rotated' convention `(z, x, y)` is in the rhino frame yawed -90 deg about Z (its +x
points along rhino -y); it converts to rhino with `(x, y) -> (y, -x)` and `yaw -> yaw - 90 deg`.

## Topics And Service Contract

Per rigid body pose topic pattern:

- `/mocap/rigid_body/id_<mocap_id>/pose`
- Message: `crl_husky_msgs/msg/MocapRigidBodyPose`

Message content:
- `header.stamp`: when the cameras captured the frame, on the relay PC's clock; a pose republished without a new
  frame keeps its stamp.
- `header.frame_id`: `lab` when the pose is in the lab frame (a floor calibration applied, or a simulator's truth),
  `mocap_raw` when it is raw (Motive's, Z-up).
- `pose`: calibrated rigid body pose.
- `pose_valid`: true when all checks pass.
- `tracking_valid`: direct NatNet tracking validity.
- `marker_error`: NatNet marker error.
- `marker_ids`, `marker_positions`, `marker_residuals`, `marker_measured`: the raw markers Motive
  labeled to this rigid body in the same frame, by 1-based member number in the Motive asset.
  Positions are in the lab frame, without a robot calibration: fit your own model (e.g. from CAD)
  to them. Use only markers with `marker_measured`; the others are occluded and filled in from
  Motive's own pose, so fitting to them gives back Motive's solution.

! Raw markers need "Labeled Markers" enabled in Motive's streaming settings; without it the
arrays are empty and the relay warns once.

Motive's setup, asked for every `definitions_period_sec` (NAT_REQUEST_MODELDEF) and published latched when it
changes, as JSON in a `std_msgs/String`, Z-up like the poses:

- `/mocap_relay/body_definitions`: `{"time", "problems", "bodies": [{"id", "name", "offsets", "labels"}]}`, each
  rigid body's marker offsets from its pivot in the body's frame as the poses turn it (Motive's own body frame, not
  calibrated), metres.
- `/mocap_relay/cameras`: `{"time", "cameras": [{"name", "position", "orientation"}]}`, each camera where Motive's
  calibration puts it, in the world.

The relay logs each change: a body new, gone, renamed, refined, redefined, its pivot moved, or another body on its id
(the data changes as warnings, `robot_interface/mocap_definitions.py`). Camera moves are split in two, each since it
was last reported: how the truss cameras moved together, fitted to them (once that moves a point of the lab's volume
over 1 mm), and each camera new, gone, or moved against them by over 1 mm or 0.1°, truss cameras marked. Continuous
calibration re-solving the cameras, a recalibration or a bump all show this way; the cameras alone cannot tell
whether poses moved. While the anchors hold the lab frame they did not, and the box is logged at info ("the anchors
hold the lab frame: poses unaffected"); otherwise it is a warning, "CAMERAS MOVED … poses may have moved with them".
Without at least four truss cameras that fit, each camera's raw move. Cameras are
named the lab's way with their serial, e.g. "BMR #72318": Top/Bottom, Front/Middle/Back (+y is back), Left/Right (+x, facing the front);
the table is `CAMERA_NAMES` in `world/mocap_frame.py`, by serial, so Motive's own names stay as they are. Motive sets no
"tracked models changed" for a refine, so only reading the definitions catches it. The monitor keeps both in
`world.mocap_relay.state`. In a simulation a fake Motive announces them (below).

Streaming IDs service:

- `/mocap/get_streaming_ids`
- Service: `crl_husky_msgs/srv/GetStreamingMocapIds`
- Response: `ids`, `count`, `query_time`.

## Launch Arguments (`launch/mocap.launch.py`)

Most important:
- `server_ip`: Motive/NatNet host IP.
- `client_ip`: local control PC IP used for NatNet sockets.
- `auto_detect_client_ip`: detect `client_ip` automatically from route to `server_ip`.
- `publish_rate_hz`: relay publish frequency.
- `topic_prefix`: prefix for topic generation.

Robustness and validity:
- `streaming_stale_timeout_sec`
- `marker_error_valid_threshold`
- `frame_timeout_sec`
- `reconnect_min_interval_sec`

Transport:
- `use_multicast` (default false/unicast)
- `multicast_address`

Calibration:
- `drive_root`: the project's Drive folder (empty: `HUSKY_DRIVE_ROOT`)
- `use_drive` (default true): prefer the newest calibration session there over the default
- `watch_period_sec` (default 30, 0 off): how often to look for a newer session or floor calibration (logged and
  announced, never applied by itself)
- `sim` (default false): what the relay reads, for its whole run, never both: off, Motive over NatNet with the
  robots' sessions; on (refused in the robots' domain), only a simulation's inputs ([Simulated
  Inputs](#simulated-inputs)) with the simulation's sessions (`sim/calibration_data/domain_<id>/`), base and floor
  calibrations alike. `~/calibrations` says which (`simulated`), `~/frame` names the sessions folder, where the probe
  writes new floor calibrations.

Motive's setup:
- `definitions_period_sec` (default 2, 0 off): how often to ask Motive for its body definitions and cameras
- `truss_cameras` (node parameter, default `31321,31322,31330,66076,72657,72654`): the truss cameras, by serial, that
  tell how Motive's frame moved

Frame log ([Lab Frame](#lab-frame-layers-anchors-refits), Metrics):
- `frame_log` (default `~/.ros/mocap_frame`, `" "`: off): where every look's frame metrics are kept, a file per day
- `frame_log_days` (default 30): how many days of files are kept

## Calibration Ownership

Calibration is relay-side, not consumer-side.

- Default, always present: each robot's crl-husky `config/robots/<serial>/mocap_config.json`, one calibration per mocap id
  (or null). No pinning, no "newest file wins".
- Newer, optional: the newest session on the project's Drive, `data_experiment/calibration_data/<YYYYMMDD>/
  calibrated_transformation_<digits>_rhino.json` (`<digits>`: the serial's last four; only directly in a dated
  folder, not in its `archive/`), used if it is within 5 cm / 5° of the default and names no other rigid body in its
  `mocap_id`; otherwise the relay warns and keeps the default.
- The relay logs what it applies (rigid body, robot, source, sha256, which arm it was measured on) and publishes it
  latched on `/mocap_relay/calibrations`. `/mocap_relay/reload_base_calibrations` (std_srvs/Trigger) applies the newest base
  calibrations, `/mocap_relay/reload_base_calibration/id_<n>` that of one robot's rigid body only, the others keeping
  theirs; the floor calibration has its own, `/mocap_relay/reload_floor_calibration` (below).
- The monitor reads that announcement and calls one robot's reload service through `MocapRelayInterface`
  (`robot_interface/mocap_relay.py`, `world.mocap_relay`); the `calibration` plugin writes new sessions and applies
  them that way (`doc/calibration_manual.md`).
- Rigid bodies of no robot (probe, bars) are published as Motive gives them.
- A session written by the `calibration` plugin names the base body's definition it was built for (`body_definition`:
  streaming id, name, marker offsets, Z-up). The relay compares it with Motive's definition now (below) and announces
  the result with each calibration (`definition_check`: `matches`, `out_of_date` with why, or `unknown` for crl-husky's
  defaults, older sessions and before Motive answered). An out-of-date one, built for an earlier body (refined, redefined,
  pivot moved, another body on the id), is warned about once and still applied: like a high marker error, it costs a
  few millimetres or degrees. The monitor's health panel turns the robot's base **mocap** chip amber (the reason in its
  hover text), and the calibration plugin's Apply step its "relay now" chip.

## Lab Frame: Layers, Anchors, Refits

Three layers, each a small rigid correction on the one below (`world/mocap_frame.py`):

1. **Raw**: Motive's poses turned Z-up. The relay always does this.
2. **Floor calibration**: `floor_from_raw` of the newest floor calibration on the Drive, in the dated calibration
   sessions beside the base calibrations (`data_experiment/calibration_data/<YYYYMMDD>/`; with `sim:=true`, the
   simulation's `sim/calibration_data/domain_<id>/<YYYYMMDD>/`), then the refit in use: together `lab_from_raw`. Every
   pose, marker, camera and anchor the relay publishes is in the lab frame by it. Without one, poses are raw.
3. **Robot base calibrations** on top (above). The calibration plugin takes them back off (`Reading.base`), so it
   works on floor-calibrated body poses.

The **lab frame** is defined by ideal positions in git, `data/lab/main_frame.json` (`world/lab_frame.py`, written by
hand). They only fix the lab frame across floor calibrations, through the probe's fit; the relay never reads them, and
works from what was measured: the anchors and truss cameras stored, raw, with the floor calibration. The file holds the marks on the floor (`"marks": {"origin": [x, y], ...}`) and Motive's anchor markers (`"anchors": {"A1": [x,
y, z], ...}`, z up from the floor). The mocap probe's floor calibration works raw (it takes the relay's floor
calibration off the poses), fits a plane to the floor points, and places the frame on it by a least-squares fit of the
marks probed and the anchors read to their ideal (x, y); marks only and anchors only are fitted too, and the
panel and the export say how far the two place the lab apart, each one's residuals, and each anchor's height against
its ideal z. **Anchor** in the probe records the tip like any point, named after the lab frame's anchor stored
nearest it, else the next free A<n>; it needs no marker in view, as the probe often hides it. The tip is only a hint:
the anchor is the unlabeled marker nearest it, within 10 cm (from `/mocap_relay/unlabeled_markers`), and every fit and
the export use that marker, never the tip. All anchors come together from the relay's newest reading, live and at
export (one rigid set; presses minutes apart under Motive's drifting frame are not), taken 3 s or more after the probe
was last within 0.5 m of each (it and the hand hide or shift the marker). An anchor over 2 mm from its stored place
has moved: it is left out of the fit (one at a time, the farthest first) and still exported, its new place in
`anchors_in_floor`. An anchor not read is left out too, and the export is refused while one is not read, or while the
relay announces no cameras (it passes over a floor calibration without them). The probe takes the markers back to raw by
`lab_from_raw`, so it skips any the relay took (their `time`) before its `~/frame` with a new `lab_from_raw`. Its file holds `floor_from_raw` and, raw as
they were then, the marks, the anchors and the cameras; it is written whole (a temporary file renamed over it).

- **Anchors**: Motive streams them as unlabeled markers (model 0, "Unlabeled Markers" on in its streaming settings),
  with no name. Every `definitions_period_sec` (a look) the relay takes each unlabeled marker's mean over the frames
  since (one frame's noise would tilt the fit by up to 1.6 mm in the lab) and publishes them all on
  `/mocap_relay/unlabeled_markers` (latched JSON: `markers`, in the lab frame). It finds the floor calibration's
  anchors among them as the nearest marker within 2 cm of where it saw each, raw (after a recalibration moved them
  further, of where the truss cameras' fit puts them), and publishes them on `/mocap_relay/anchors` (latched JSON:
  `anchors` by name, in the lab frame, and `missing`: not found in this look). An anchor not seen for 10 s is reported
  ("anchor A3 not seen for 10 s; the other 5 hold the frame", or "only 3 of 6 anchors seen …: the truss cameras hold
  the frame"); one that flickers is not. The lab frame file's anchors are for the probe; the relay follows the floor
  calibration's. `scripts/mocap_anchors.py --json` lists the relay's markers that stay put.
- **Following Motive's frame** (`world/frame_follower.py`): a recalibration, or Motive's continuous calibration, moves
  Motive's frame and every raw pose with it, while the anchors and the six truss cameras (the four on top and the two
  lower ones in the far corners; `truss_cameras`, by serial) stay put. Every look the relay fits, rigidly, where Motive
  puts them now against where the floor calibration stored them: once by the anchors, once by the truss cameras. The
  points used are the largest set, of 4 or more, in which each agrees with a fit of the others; the rest are left out
  and named ("ANCHORS MOVED" for an anchor). Each fit reports its largest residual and its uncertainty at the lab
  volume's worst corner ("± … mm": the layout's leverage times the scatter of those used; points in two tight pairs
  pin no turn, and it grows).
- **Thresholds**, side by side: a camera that moved between two readings by over 1 mm or 0.1° is warned about
  ("CAMERAS MOVED", `CAMERA_MOVED`); within one fit, a camera off the others by 2 mm or 0.1°, or an anchor by 2 mm,
  is left out (`BUMP_POSITION`, `BUMP_TURN`, `ANCHOR_BUMP`), and an anchor left out comes back only within 75 % of
  that (`BACK_SHARE`); a fit is trusted with 4 or more that agree and an
  uncertainty within 2 mm (`MIN_POINTS`, `UNCERTAIN_MAX`); a refit is pending once the deciding fit places a point of
  the lab's volume beyond 1 mm, or twice its uncertainty if larger, from the transform in use (`off_limit`,
  `VOLUME_MOVED`), and is held again only under 75 % of that limit. The lab's volume (`VOLUME`) is the central 3 m
  cube where the robots work, one corner at the lab frame's origin, along x, -y and z; judged at its worst corner, so
  a turn counts as much as a shift.
- **States** (`~/frame` `status`): `none` (no floor calibration), `unchecked` (applied, no fit has decided yet: after
  a start or reload, or with neither anchors nor truss cameras), `held` (a trusted fit confirms it), `refit_pending`
  (the trusted deciding fit is beyond the limit), `untrusted` (the deciding fit is not trusted), `error` (none could
  be read at the start). The anchors' fit decides when trusted, else the truss cameras', else the anchors' untrusted
  one. Each state change is logged once with both fits as a table (used; moved, turned and the lab volume since the
  floor calibration; off now against the transform in use; largest residual; the deciding one marked) and how far
  apart the two place the lab's volume: "frame held by the anchors: poses off now 0.3 mm, within the 1.0 mm limit"
  (info), "REFIT PENDING by …" (warning, with how to use it), "FRAME UNTRUSTED: …" (error). The same table is logged
  again only when the deciding fit's source changes or it moves by over 1 mm. While not held or unchecked, poses may
  be off: the health panel turns every base's mocap chip amber (a simulator's truth excepted).
- **`~/frame`** (latched JSON, announced when the state, the deciding source, the anchors missing, the refit, a newer
  floor calibration or an error changes, or the pending fit moves by over 1 mm): `time`, `status`, `error`,
  `lab_from_raw` ([position, quaternion] or null), `floor_calibration`, `simulated`, `folder`, `refit` (the refit in
  use), `pending` (the deciding fit while not held), `newer` (`floor_calibration`, `moves_volume`, `moves_poses`),
  `decides` (`anchors`, `truss cameras` or null), `anchors_missing` (not seen for 10 s), `anchor_fit`, `camera_fit`,
  `truss_cameras`. Each fit has `raw_cal_from_raw_now`, `used`, `bumped`, `missing`, `residual`, `trusted`, `source`,
  `uncertainty`, `consistent`, `moves_volume` (since the floor calibration), `off_now` (against the transform in use)
  and `describe` ("poses off now … mm; since the floor calibration …").
- **Using a refit**: nothing changes until told: `ros2 service call /mocap_relay/use_refit std_srvs/srv/Trigger`, or
  **Use refit** in the mocap probe (refused while anything moves); logged as "REFIT USED". The fit is composed onto
  the floor calibration's `floor_from_raw`, in memory only: the relay writes no file. Every fit, before and after,
  is made against the floor calibration's anchors and cameras; a restart or another floor calibration starts from
  the file again, unchecked.
- **Metrics**: every look the relay publishes `/mocap_relay/frame_metrics` (JSON: `time`, `floor_calibration`,
  `status`, `decides`, `anchors` and `truss_cameras` as each fit's numbers, `refit`, `off`, `off_limit`, `apart`,
  `anchors_missing`; metres), also without a floor calibration (then without fits). With them, the look's raw inputs
  (Z-up, Motive's frame; `FrameFollower.metrics`):
  - `frames`: the most frames any unlabeled marker was seen in since the last look (a marker needs half to count);
  - `markers`: every unlabeled marker counted, as columns `raw` (its mean, m), `noise_mm` (its RMS distance from that
    mean over the frames, from a sum of squares beside the sums) and `frames`;
  - `anchor_quality`: each of the floor calibration's anchors by name: `raw`, `noise_mm`, `frames`, `residual_mm`
    (against the anchors' fit), `left_out_mm` (against a fit of the other anchors: the agreement check's own fits,
    `FrameFit.left_out_errors`; for an anchor left out, the fit itself) and `status`: `used`, `bumped` (left out),
    `seen` (under 3 found: no fit) or `missing` (not found this look; its numbers null). A large `left_out_mm` is a
    bad anchor;
  - `truss_camera_poses`: each truss camera's [position, quaternion]. With the floor calibration's file (named in
    `floor_calibration`), a sample holds what both fits need to be made again offline.
- **Frame log**: the relay appends every sample, one JSON line as published, to `<frame_log>/<YYYYMMDD>.jsonl` (the
  look's local date; default `~/.ros/mocap_frame/`, local beside ROS's own logs, never on the Drive). Day files older
  than `frame_log_days` (default 30) are deleted at the first look and at each day change; `frame_log:=" "` (a space:
  ROS refuses an empty launch argument) keeps none. A simulation's relay (`sim:=true`) would log to
  `<frame_log>/sim/domain_<id>/`, but makes no looks yet.
  A sample is about 4 kB, so a day of looks every 2 s is about 170 MB, 30 days about 5 GB. Lines are flushed, not
  synced: a power cut may lose the last few; a write error is logged (at most every 10 min) and the relay goes on.
- **Plot**: `scripts/mocap_frame_plot.py` plots the samples live (`--record` keeps them) or from files (`--file`
  takes files, folders and globs, e.g. `--file ~/.ros/mocap_frame` for every day kept): how far each fit moves the
  lab's volume since the floor calibration, with its uncertainty, how far poses are off now against the transform in
  use, the largest residual, how many it uses, and how far apart the two are; beside them, per anchor, its error
  against the others, its noise and the share of frames it was seen in, crossed where left out. With `--file` it
  prints each anchor's median and 95th percentile. Older recordings, without the per-anchor keys, leave those panels
  empty.
- **A new floor calibration** is looked for every `watch_period_sec` and never applied by itself: it is warned about
  once ("newer floor calibration on the Drive, not applied", with how far it would move the lab's volume) and
  announced on `~/frame` (`newer`), which turns the mocap chips amber like a pending refit; one within 1 mm of the
  one in use is only logged as info and leaves the chips as they are. `ros2 service call
  /mocap_relay/reload_floor_calibration std_srvs/srv/Trigger`, or **Apply floor calibration** in the mocap probe
  (refused while anything moves), applies it. Older files (`floor_from_mocap` with `relay_correction`) are read as
  `floor_from_raw` = floor_from_mocap ∘ relay_correction, their cameras and anchors taken back to raw.
- A newest floor calibration that does not read (half-synced, or made in a simulation and in the robots' folder) is
  passed over with a warning, and the one before it applied at a start. A reload that cannot read the Drive, its
  folder or a newer file keeps the frame in use and logs an error; at a start with nothing that reads, `~/frame` says
  `error` and the mocap chips turn amber.
- A fake Motive's `recalibration` (x, y, z m, roll, pitch, yaw deg; `ros2 param set /sim_motive recalibration
  "0.02,-0.01,0,0,0,0.5"`) moves its frame, to try this in a simulation.

## Simulated Inputs

With `sim:=true`, only outside the robots' domain (every domain but those in crl-husky's `robot.yaml` files), the
relay reads simulated inputs only, never Motive, subscribing to each as its topic appears, so the relay and the
monitor keep running while simulations come and go. Per rigid body, the first of these that streams (heard within
`streaming_stale_timeout_sec`) is published:

| Source | Topic | Calibration |
|---|---|---|
| fake Motive (`sim_motive`, `launch/sim_mocap.launch.py`) | `/mocap_sim/motive/id_<n>/pose`: each truth with Motive's imperfections, Z-up; a robot's base relative to crl-husky's default calibration | the default taken off, then the simulated sessions', `sim/calibration_data/domain_<ROS_DOMAIN_ID>/` |
| simulator's truth (crl-husky `husky_sim`, started by `sim_robot` or `sim_all`) | `/mocap_sim/truth/id_<n>/pose`: the true base_footprint, and the true tool0 of an arm carrying the calibration marker | none |

When a source stops, the other comes back (logged). `/mocap_relay/calibrations` lists the fake Motive's calibrations,
each with its definition check, and `"simulated": true`.

The fake Motive also announces, latched, what Motive's model descriptions give: `/mocap_sim/motive/body_definitions`
(five markers per body it fakes, the same for the same id) and `/mocap_sim/motive/cameras` (the lab's ten cameras,
`data/sim/motive_cameras.json`, or `cameras_file:=`). The relay publishes them in place of Motive's.
`refines:="1850:1"`, or `ros2 param set /sim_motive refines "1850:1"` while it runs, nudges a body's markers as a
refine in Motive does: the relay warns, and a calibration built for the body goes out of date.

```shell
export ROS_DOMAIN_ID=70                                               # a domain of its own, never 80
ros2 launch husky_assembly_teleop mocap.launch.py                     # the relay, once
ros2 launch husky_assembly_teleop sim.launch.py robot:=0805 tool:=calibration_marker fake_motive:=true \
    delay_sec:=0.01 noise:=0.0005,0.1                                 # robot:=all for every robot, with its tools
```

`sim.launch.py` starts crl-husky's `sim_robot.launch.py` (or `sim_all.launch.py`) and, with `fake_motive:=true`,
`sim_mocap.launch.py`, and takes every argument of both; each can also run on its own. Workflows and every argument:
[simulation_manual.md](simulation_manual.md).

`sim_motive` knows no robots or arms: it takes every true pose on `/mocap_sim/truth`, moves each body's frame by its
`offsets` entry (for a robot's base: how far the true calibration is from crl-husky's default), adds `delay_sec`
(Motive's latency) and the other imperfections, and announces "mocap" on the `simulated_parts` of each robot whose
base it fakes. Per-body arguments take values for every body, or `<id>:<values>` per body. Its arguments:
`launch/sim_mocap.launch.py`.

## Quick Verification

1. List relay topics:
   - `ros2 topic list | grep /mocap/rigid_body`
2. Echo one rigid body topic:
   - `ros2 topic echo /mocap/rigid_body/id_1004/pose`
3. Check publish rate:
   - `ros2 topic hz /mocap/rigid_body/id_1004/pose`
4. Check active IDs service:
   - `ros2 service call /mocap/get_streaming_ids crl_husky_msgs/srv/GetStreamingMocapIds {}`

## Known Differences And Gaps vs Old Direct NatNet

- Relay publishes rigid body poses with their labeled markers, Motive's body definitions (marker offsets) and camera poses. Unlabeled markers, marker names and other camera metadata are not relayed.
- Only one relay runs per ROS domain (checked in the ROS graph at start), so consumers never receive interleaved poses from two relays.

## Source Files

- Relay node: `husky_assembly_teleop/mocap_relay/node.py`, calibration choice in `world/calibration_sessions.py`,
  model descriptions read in `mocap_relay/body_definitions.py`, compared in `robot_interface/mocap_definitions.py`
- The monitor's side: `husky_assembly_teleop/robot_interface/mocap_relay.py` (`MocapRelayInterface`)
- Relay launch: `launch/mocap.launch.py`
- Following Motive's frame: `world/frame_follower.py` (state, refit, logs, metrics), the fits in
  `world/mocap_frame.py`; the frame log in `mocap_relay/frame_log.py`, plotted by `scripts/mocap_frame_plot.py`
- Tests: `test/mocap_relay/test_mocap_relay_node.py`, `test/mocap_relay/test_frame_log.py`,
  `test/mocap_relay/test_mocap_frame_plot.py`, `test/world/test_frame_follower.py`,
  `test/world/test_mocap_frame.py`, `test/mocap_relay/test_body_definitions.py`,
  `test/world/test_calibration_sessions.py`, `test/robot_interface/test_mocap_relay_interface.py`,
  `test/robot_interface/test_mocap_definitions.py`
- Pose message: crl-husky `crl_husky_msgs/msg/MocapRigidBodyPose.msg`
- Streaming IDs service: crl-husky `crl_husky_msgs/srv/GetStreamingMocapIds.srv`
- Robot config, default calibrations: crl-husky `crl_husky/config/robots/README.md`
