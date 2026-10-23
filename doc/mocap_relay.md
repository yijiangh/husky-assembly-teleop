# Mocap relay

This document explains the mocap relay and gives a migration guide for older code that used direct NatNet
callbacks (for example HuskyMonitor).

The relay lives here (`husky_assembly_teleop/mocap_relay/`, executable `husky_mocap_relay`,
`ros2 launch husky_assembly_teleop mocap.launch.py`); it moved from crl-husky on 16 Oct 2026. crl-husky keeps what
it publishes (`crl_husky_msgs`), the NatNet client (`crl_husky/optitrack/`) and each robot's default base
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
3. Relay converts coordinates to the project world frame (Z-up, 'rhino' convention, see below).
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

## World Frame Convention

NatNet streams Y-up poses. The relay converts them to the Z-up world frame that
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
- `header.frame_id`: configured frame name (default `mocap_world`).
- `pose`: calibrated rigid body pose.
- `pose_valid`: true when all checks pass.
- `tracking_valid`: direct NatNet tracking validity.
- `marker_error`: NatNet marker error.
- `marker_ids`, `marker_positions`, `marker_residuals`, `marker_measured`: the raw markers Motive
  labeled to this rigid body in the same frame, by 1-based member number in the Motive asset.
  Positions are in the Z-up world frame but not calibrated: fit your own model (e.g. from CAD)
  to them. Use only markers with `marker_measured`; the others are occluded and filled in from
  Motive's own pose, so fitting to them gives back Motive's solution.

! Raw markers need "Labeled Markers" enabled in Motive's streaming settings; without it the
arrays are empty and the relay warns once.

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
- `frame_id`: output frame name.

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
- `watch_period_sec` (default 30, 0 off): how often to look for a newer session; it is logged, never applied

## Calibration Ownership

Calibration is relay-side, not consumer-side.

- Default, always present: each robot's crl-husky `config/robots/<serial>/mocap_config.json`, one calibration per mocap id
  (or null). No pinning, no "newest file wins".
- Newer, optional: the newest session on the project's Drive, `data_experiment/calibration_data/<YYYYMMDD>/
  calibrated_transformation_<digits>_rhino.json` (`<digits>`: the serial's last four; only directly in a dated
  folder, not in its `archive/`), used if it is within 5 cm / 5° of the default and names no other rigid body in its
  `mocap_id`; otherwise the relay warns and keeps the default.
- The relay logs what it applies (rigid body, robot, source, sha256, which arm it was measured on) and publishes it
  latched on `/mocap_relay/calibrations`. `/mocap_relay/reload_calibration` (std_srvs/Trigger) applies the newest.
- The monitor reads that announcement and calls the reload service through `MocapRelayInterface`
  (`robot_interface/mocap_relay.py`, `world.mocap_relay`); the `calibration` plugin writes new sessions and applies
  them that way (`doc/calibration_manual.md`).
- Rigid bodies of no robot (probe, bars) are published as Motive gives them.

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

- Relay publishes rigid body poses with their labeled markers. Unlabeled markers, model descriptions (marker names, Motive's marker offsets) and camera metadata are not relayed.
- Only one relay runs per ROS domain (checked in the ROS graph at start), so consumers never receive interleaved poses from two relays.

## Source Files

- Relay node: `husky_assembly_teleop/mocap_relay/node.py`, calibration choice in `world/calibration_sessions.py`
- The monitor's side: `husky_assembly_teleop/robot_interface/mocap_relay.py` (`MocapRelayInterface`)
- Relay launch: `launch/mocap.launch.py`
- Tests: `test/mocap_relay/test_mocap_relay_node.py`, `test/world/test_calibration_sessions.py`,
  `test/robot_interface/test_mocap_relay_interface.py`
- Pose message: crl-husky `crl_husky_msgs/msg/MocapRigidBodyPose.msg`
- Streaming IDs service: crl-husky `crl_husky_msgs/srv/GetStreamingMocapIds.srv`
- Robot config, default calibrations: crl-husky `crl_husky/config/robots/README.md`
