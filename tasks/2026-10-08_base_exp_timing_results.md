# Results: why the wheels → motion delay is negative

Answers to `2026-10-08_base_exp_timing.md`. **M** = measured from the recordings, **I** = inferred, **C** = read from code.
Scripts and the regenerated reports: `/tmp/claude-1000/-home-jakob-ra-workspace-src/764ee65f-90b8-494b-ac14-4cd46171bc55/scratchpad/`
(`stamps.py`, `trend.py`, `step.py`, `lag.py`, `analysis/`). The regenerated `robot_model.md` gives the same numbers:
200 / 370 / −150 ms.

## Short answer

The clocks are fine. The wheel odometry is late: it reports a speed about 150–200 ms after the robot moves at that
speed, even though both stamps are taken on the robot. So **commands → motion = 200 ms is right** and `cmd_delay_sec: 0.2`
stays. Commands → wheels is really about 200 ms, not 370, and wheels → motion is about 0.

## 1. Who stamps what (C)

| log | stamped by | clock |
|---|---|---|
| pose (`times[:,0]`) | `mocap_relay` on a control PC: the frame's mid-exposure time (arrival − Motive's transmit−exposure latency). It falls back to arrival time when NatNet sends no hi-res stamps. The monitor uses this stamp since a63848c (10-07 11:23), in `mocap.store_sample`. | control PC (relay host) |
| command (`commands[:,1]`, `times[:,1]`) | `pure_pursuit_node._publish_state` on the robot: `get_clock().now()` when it publishes the state. The cmd_vel goes out in the same timer callback. | robot |
| wheel odometry (`wheel_odometry[:,1]`) | Clearpath `platform_velocity_controller` (`/platform/odom` header) on the robot | robot |
| `received` columns | monitor callback, `get_clock().now()` | monitor |

The comment in `plugin.py` `_measure` ("arrival time stands in … until mocap is stamped") is out of date: the pose is
already stamped with its capture time. I do not know which PC ran the relay on 10-07. It was probably the monitor laptop
itself; in any case it was a chrony-synced control PC.

## 2. `received − stamp` (M)

All 26 real runs of `robot_model_alice_1007` (17:52–18:27) plus 7 real runs of `real_1007` (12:10–12:14):

| log | min | median | p95 | trend in a run (lower envelope) | over the day |
|---|---|---|---|---|---|
| commands | 1–3 ms | 25–31 ms | 48–64 ms | 0.01 ms/s (max 0.7) | min 1–3 ms at 12:10 and at 18:27 |
| wheel odometry | 1–3 ms | 25–31 ms | 48–58 ms | 0.04 ms/s (max 0.9) | same |
| monitor tick − pose capture | 9–10 ms | 21 ms | 31 ms | — | same |
| sim: commands | 0–1 ms | 25 ms | 48 ms | — | — |
| sim: tick − pose capture | 1 ms | 11 ms | 20 ms | — | — |

- The floor is constant at about 2 ms and does not drift over a run or over the day. The spread (median about 26 ms,
  p95 about 50 ms) shows up in the simulator too, so it comes from when the monitor runs the callback (its tick), not
  from the clocks or the WiFi.
- The command stamps come at an even 33.3 ms interval (sd 0.3 ms). The odometry stamps also come at about 30 Hz but
  jitter more (sd 7 ms).

## 3. Clock offset? No (M + I)

- One way, robot → monitor: `received − stamp` is at least 2 ms. **M**
- The other way, relay → robot: I lined up the follower's own yaw error (at its robot stamp) with the monitor's yaw
  error from mocap (at the capture stamp). The lag is 5 ms median (IQR −1 to 15, relative rms 0.05) in alice_1007 and
  25 ms in real_1007. **M**
- Added together, the round trip is about 10–30 ms. So the robot clock is within about ±20 ms of the relay and monitor
  clocks. A 150 ms offset would need 150 ms of one-way WiFi latency with no jitter on every message. **I**
- The step responses (50 % crossing, robot-clock commands vs. robot-clock odometry vs. capture-stamped mocap, **M**):

  | runs | cmd → wheels | wheels → motion | cmd → motion |
  |---|---|---|---|
  | straight (3 usable) | 344–399 ms | −176 to −211 ms | 168–199 ms |
  | turn on the spot (6) | 501–565 ms | −144 to −228 ms | 273–396 ms |

  The wheels lag the mocap motion in every run, by about the same amount. The wheel speed also rises more slowly
  (25 → 75 %: about 130 ms on straights, against about 77 ms for mocap). That points to a delay plus some smoothing
  in the odometry. **I**
- **Likely cause (I):** `/platform/odom` carries a speed that is about 150–200 ms old at its stamp. The stamp is when
  it was published, not when the wheels were measured. Two possible sources, and the recordings cannot separate them:
  the A200 MCU feedback latency, and the diff_drive_controller's `velocity_rolling_window_size` averaging. Check on
  the robot with `ros2 param get …/platform_velocity_controller velocity_rolling_window_size` and the
  controller_manager `update_rate`.
- **True delays (I):** commands → motion **≈ 200 ms** (as identified). Odometry lag ≈ 150–200 ms. Commands → wheels
  ≈ 370 − (150–200) ≈ **170–220 ms**. Wheels → motion **≈ 0** (cannot be resolved to better than about ±30 ms).

## 4. Mocap latency (C + M)

- Since a63848c the pose time is the relay's mid-exposure stamp. OptiTrack does give the capture time: NatNet 3+
  sends `stamp_camera_mid_exposure` and `stamp_transmit`. So **about 0 of the 200 ms is mocap latency**; only the
  sub-millisecond LAN hop is left in it. **C**
- Capture → monitor tick: min 9 ms, median 21 ms. That includes up to 20 ms of the relay's 50 Hz hold and the
  monitor's tick. **M** Even if the relay had fallen back to arrival stamps, this bounds the error at about 10 ms.
- Not determined: whether the relay really used hi-res stamps on 10-07. Its log line "Motive latency
  (mid-exposure to transmit): … ms", or the "NatNet sends no hi-res timestamps" warning, would say.

## 5. Simulator (M + C)

- The simulator publishes no wheel odometry (`base.py`: "none in the simulator"), so its 81 sim runs in this
  experiment cannot show the split.
- Its commands → motion check does pass. Identification returns the configured 200 ms (sim alice_tiles_2) and 0 ms
  (sim ideal) exactly. So on one host the command and pose stamps add no bias, and the 200 ms on the real robot is
  measured the same way.

## Proposed fix

1. **Keep `cmd_delay_sec: 0.2`.** It is measured from robot command stamps to capture stamps, and the clocks agree to
   within about ±20 ms.
2. **Treat the odometry as late, not as a timing reference.** In the report, read wheels → motion as "odometry lag =
   −delay". Either shift `odometry_time` back by that lag in `replay.track` before splitting, or show only commands →
   motion and the wheels efficiencies. With the shift, commands → wheels becomes about 200 ms and wheels → motion
   about 0.
3. **Clock check per session.** Save the joint_states `StreamQuality.delay` (the new chip) and the follower-yaw lag
   from §3 into `experiment.json`. Together they are a round-trip check, so a robot clock running ahead shows up in
   the data, not only on a live chip.
4. Fix the stale comment in `plugin.py` `_measure`.

Side note (M): the delay depends on the template. Turn on the spot is 340 ms, straight 170 ms. That comes from the
robot (skid-steer turning), not from the timing.
