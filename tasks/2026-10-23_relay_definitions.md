# The relay publishes Motive's body definitions and camera poses (2026-10-23)

Status: agreed with the user, building on `jg/calibration-port`.

## Why

A refine changes a rigid body's definition (0.667 mm in the lab test) without Motive setting "tracked models changed",
so only reading the definitions tells. The calibration needs the definitions a measurement was made with
(`2026-10-22_calibration_dependency_model.md`, "Later"); camera poses show Motive's continuous calibration and bumps.
The lab streams NatNet 4.0 (Motive 3.0.3.1), whose model definitions hold one camera description per camera.

## What

- The relay asks Motive for its model definitions every `definitions_period_sec` (default 2 s, 0 off) and publishes,
  latched, when they change:
  - `/mocap_relay/body_definitions`: every rigid body (streaming id, name, marker offsets from the pivot, labels).
  - `/mocap_relay/cameras`: every camera (name, position, orientation).
  JSON in a String, like `/mocap_relay/calibrations` (decided: no crl-husky message yet). Z-up like the poses:
  offsets in the body's frame as the relay's poses turn it, cameras in the world frame.
- The relay logs each change: a body new, gone, renamed, on a new id, refined, redefined, frame moved, other body
  (data changes as warnings); a camera new, gone, or moved by over 1 mm or 0.1°.
- `MocapRelayInterface` keeps both in `world.mocap_relay.state` (`body_definitions`, `cameras`, with arrival times).
- In a simulation the fake Motive announces them on `/mocap_sim/motive/body_definitions` and `/mocap_sim/motive/cameras`
  (the lab's cameras, a five-marker layout per body, `refines` to fake a refine); the relay takes them over Motive's.

## Code

- `robot_interface/mocap_definitions.py`: `BodyDefinition`, `Change`, `Event`, `compare`, `match` (moved from
  `mocap_relay/body_definitions.py`, which `robot_interface` may not import), `MocapCamera`, `camera_changes`, and the
  announcements' JSON both ways.
- `mocap_relay/body_definitions.py`: reading NatNet only; also reads cameras; replies as `Reply`.
- `mocap_relay/node.py`: the poll timer, the Z-up turn, the two publishers, the log.

## Calibrations name their body definition (user, 23 Oct)

- A calibration file names the base body's definition it was built for (`body_definition`); the relay checks each
  applied calibration against Motive's definition now (`definition_check`: matches, wrong, unknown), warns about a wrong
  one and announces the check with the calibration; it still applies it (a refined body's calibration beats crl-husky's
  default). Like a high marker error it costs a few mm or degrees, so the health panel turns the base's mocap chip
  amber (user, 24 Oct), and the calibration's Apply its "relay now" chip.
- The calibration judges out of date by body definitions, not refine counters: they are inputs from the Marker error
  step, stored with every measurement; Solve writes the takes' base definition into the calibration file.

## Later

Check the camera fields of view per model against the lab's cameras (`mocap_probe.CAMERA_FOVS`).
