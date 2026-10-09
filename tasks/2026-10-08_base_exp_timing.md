# Task: why the wheels → motion delay is negative

For a subagent. Investigate and report; change no code without asking.

## The finding

`robot_model.md` of experiment `robot_model_alice_1007` (real robot, 26 path runs, 2026-10-07), section "Timing
through the wheels":

| input → output | delay [ms] | fit R² |
|---|---|---|
| commands → motion | 200 | 0.96 / 0.90 / 0.96 |
| commands → wheels | 370 | 0.97 / — / 0.97 |
| wheels → motion | −150 | 0.98 / 0.92 / 0.99 |

A negative wheels → motion delay is not physical: the wheels cannot turn after the robot has moved. Something in
the timing is off by about 150 ms or more. If the clocks are offset, the 200 ms commands → motion delay, which goes
into the robot model (`cmd_delay_sec`), is off too.

Data: `$HUSKY_DRIVE_ROOT/data_experiment/base_exp/robot_model_alice_1007/` (one folder per run, `recording.npz`).
Regenerate the reports with `scripts/analyze_base_exp.py --experiment robot_model_alice_1007 --out <scratch dir>`,
never into the Drive folder.

## What is known about the stamps

- Pose time: the arrival time of the mocap fix on the monitor (`plugin.py` `_measure`, `_times`: "the fix's arrival
  time stands in for its measurement time until mocap is stamped"). Monitor clock; includes the mocap latency.
- Command time: the follower's report stamp (`follower.stamp`; the `commands` log: received, stamp, v, w, state).
  Find out which host and clock stamps it.
- Wheel odometry: the `wheel_odometry` log (received, stamp, v, w), stamped on the robot.
- `replay.track` uses the stamps when a run has them (`clock = "stamps"`), else the monitor's tick.
- `identify.py` fits the delays (`DELAYS`, `WHEEL_DELAYS`: −0.4 to 0.6 s for the wheels).
- The robots and the monitor sync to a Windows w32time reference (no always-on Linux hub). The latest commit,
  "signal chips check clock delay", added a clock-delay check to the monitor's signal chips.

## Questions

1. Which host and clock stamps each of the three: pose, command, wheel odometry?
2. From the recordings: how far apart are each log's `received` and `stamp` columns (median, spread, trend over a
   run and over the day)? A constant offset means clocks; a spread means latency.
3. Does a clock offset between monitor and robot explain the −150 ms? Then what are the true commands → wheels,
   wheels → motion and commands → motion delays?
4. How much of the 200 ms is mocap latency (fix arrival against capture)? Does OptiTrack give a capture time to use?
5. Does the simulator show the same split? It has no clock offset (one host), so its wheels → motion delay should
   be about 0: check with sim runs from the same experiment.

## Deliverable

`tasks/2026-10-08_base_exp_timing_results.md`: answers with numbers, the likely cause, and a proposed fix (e.g. a
stamp to use, a clock check before each session, or a correction in `replay.track`). Keep it short.
