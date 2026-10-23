# One relay for real and simulated mocap (2026-10-22)

Status: **built** (crl-husky `jg/ros2` a51b5f7, 5552fc5, d6566cb; teleop `jg/viser-cleanup`; not pushed); checked live 22 Oct in
domain 72.

## Goal

The relay and the monitor start once, the same way as with the real robots; a simulation is one launch of its own
that can come and go. The relay takes simulated inputs beside Motive's, and a simulated body overrides Motive's body
with the same id while it streams.

## Decisions (user, 22 Oct)

- `husky_sim` publishes only the true base_footprint, the pose we understand: `/mocap_sim/truth/id_<n>/pose`. No mocap
  delay or noise in husky_sim any more.
- Motive's artifacts come from a fake Motive between husky_sim and the relay (`sim_motive`, optional): raw rigid bodies
  as Motive would send them, `/mocap_sim/motive/id_<n>/pose`, with the true calibration taken off (crl-husky's default
  moved by `calibration_error`), the arm's calibration marker by forward kinematics, noise, dropouts, drift, joint
  offsets, and a delay (Motive's latency).
- The relay is the only publisher of `/mocap/rigid_body/*`. Per id the freshest input of the highest priority wins:
  fake Motive, then truth, then NatNet. A source not heard for `streaming_stale_timeout_sec` gives way to the next, so
  Motive's body comes back when a simulation stops.
- Calibrations per source: NatNet bodies get the robots' sessions (`calibration_data/`), fake Motive bodies the
  simulated ones (`calibration_data_sim/domain_<id>/`), truth bodies none: they already are the base_footprint. So a
  plain simulation never depends on a calibration applied earlier in the domain.
- ! In the fleet's domain the relay reads NatNet only: it never looks for simulated inputs there.
- base_exp records the fake Motive's delay and noise as the sim model's mocap values (0 without a fake Motive), so old
  runs (all 0) keep their setup.
- Known: every simulated base now goes through the relay's 50 Hz republish, 0–20 ms, as the real mocap does.

## Fake Motive knows no robots (user, 22 Oct, later)

- The simulated robot publishes what is truly there: husky_sim the base_footprint and, with sim_robot's
  `tool:=calibration_marker` (`tool_left`, `tool_right`), that arm's true tool0 as the marker's body (`marker_body`,
  1866): the arm's mount from the robot's URDF (husky_urdf beside the checkout, or `husky_urdf:=`), the arm from its UR
  driver's TF (`crl_husky/sim/tool_truth.py`).
- sim_motive is generic: every `/mocap_sim/truth` body, with per-body `offsets`, `noise`, `marker_error`, `drift`, and
  `delay_sec`, `dropout`; per-body text is `<values>` or `<id>:<values>` (`robot_interface/simulated_parts.body_values`).
  A robot's base comes out relative to crl-husky's default: the relay takes the default off, then applies the
  simulated session's. A base's offset is the old `calibration_error`.
- `joint_offsets` moved to sim_robot (husky_sim): each driver link turned by its joint's offset about its z axis
  (`tool_truth.true_arm`), checked against the monitor's FK at the offset joints (test/config/test_sim_tool_truth.py).
- Gone: `robots`, `markers`, `calibration_error`, `marker_offset`, the base/marker variants of noise, marker error and
  drift.
- Checked live (domain 72, Belle with the marker): the marker truth equals the monitor's FK of tool0 to 0.000 mm;
  through the fake Motive the base is off by the domain's 21 Oct sim session (5.2 mm), the marker exact.

## One launch for a simulation (user, 22 Oct)

- `launch/sim.launch.py`: crl-husky's sim_robot (or sim_all with `robot:=all`, no tools) and, with
  `fake_motive:=true`, sim_mocap. It reads both files' argument declarations, so it offers every argument with their
  defaults and never copies them (test/mocap_relay/test_sim_launch.py). Relay and monitor stay separate: started once.

## Changes

- crl-husky: husky_sim (truth only, `mocap_id`, no mocap delay/noise), pure_pursuit_sim and sim_robot (no
  `sim_mocap_topic`, `mocap_relay`, mocap arguments), README.
- teleop relay (`mocap_relay/node.py`): no `source`, `sim_ids`, `exclude_ids`; finds simulated inputs every second
  outside the fleet domain; per-id source choice; simulated samples stored directly (never commit a half-received
  NatNet frame); two calibration sets; `~/calibrations` also lists `sim_calibrations`; NatNet reconnect messages at
  info level while simulated bodies stream.
- `sim_motive`: publishes on the motive prefix, `delay_sec`, announces "mocap" on each robot's `simulated_parts`.
  `sim_mocap.launch.py` starts only it.
- Monitor: `MocapRelayState.sim_calibrations`; the calibration run reads the simulated set for a simulated robot;
  `BaseInterface.mocap_simulation`; base_exp's sim model takes the mocap values from the fake Motive.
- Docs: calibration manual, base_exp and base_planner manuals, READMEs, docstrings.

## Notes

- ! rclpy refuses to change a log call's severity between calls (ValueError, kills the node): the NatNet messages that
  are info while a simulation streams have one call per severity.
- `ros2 launch` refuses an empty `name:=`, so `sim_mocap.launch.py` has no markers by default; calibration tests name
  theirs.

## Todo

- [x] crl-husky changes, build, crl-husky tests (42 pass; its linter tests fail as before, 3060 old errors)
- [x] relay, sim_motive, launch files
- [x] monitor: relay interface, calibration run, base interface, base_exp
- [x] tests, `-m ""` (917 pass)
- [x] docs
- [x] sim run in domain 72: relay first; sim_robot 0804 → relay publishes the truth at 50 Hz, equal to it, ~20 ms old;
      fake Motive (delay 30 ms) → takes over, same pose, ~47 ms old, "mocap" announced; fake Motive stopped → truth
      again; sim stopped → last pose, invalid. NatNet warnings turn info while the simulation streams.
- [ ] the monitor against it (base_exp records the fake Motive's delay and noise)
- [x] commit: crl-husky first, then teleop
- [ ] push: crl-husky first
