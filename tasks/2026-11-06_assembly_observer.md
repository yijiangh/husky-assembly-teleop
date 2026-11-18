# assembly_observer: record everything that happens

Branch `jg/assembly-observer`, off `core-port` dc350d38.

## Goal

One plugin records the whole session, manual control included, so a lab day can be replayed and questioned later:
the robots at the tick rate (20 Hz max, one row per tick) and every event, on this PC for a week; the panel copies
the ranges an experiment needs to the Drive.

## Decisions

- Recorded on this PC only (`~/.ros/assembly_observer/`), a rolling week; the panel copies a time range to the
  Drive after an experiment (user, 6 Nov).
- Operator clicks and settings logged by the plugin context (`operator: ...`), refusals always logged, Exec's safety
  choices logged, clearance trips with every pair and contact (user, 6 Nov).
- Tick signals only, one table per chunk: no per-message recording. Only continuous values are columns; every
  state that switches (flags, modes, controllers, motor states, tracking) is an event (user, 6 Nov).
- Commands in full: trajectories and paths exactly as sent, each in `commands/<n>_<event>_<robot>.json`, named by
  its event; tool commands, holds and wrenches inline.
- Force: the 20 Hz wrench plus the largest |F| and |T| since the last tick over every reading (the arm decodes every
  wrench message only while something listens); the robot's rate limiter caps that at 50 Hz. Zeroing is an event.
- Every tracked object (`world.tracked_objects`): pose and mocap noise; a new chunk when the set changes. Bases
  carry the same noise columns. Battery charge and voltage.
- Wifi health per robot: rate, largest gap and delay of the platform's and each arm's joint_states and of mocap
  (`StreamStats`; mocap got its own), read twice a second.
- The relay's frame drift: `~/frame_metrics` parsed into `MocapRelayState.drift` (`FrameDrift`), recorded each tick
  and told as an event when it moves by 0.5 mm / 0.1 mm/m.
- Events come from the runtime only (no `requires`): state diffs, command events, clearance trips, relay changes and
  every console log line from INFO up (which carries the guide's, the executor's and the run's messages).
  The assembly run's own log stays where it is; both join by ROS time.
- Chunks of 60 s (a crash loses at most that), saved under a hidden name then renamed; `np.savez_compressed`,
  values float32, times float64 (`Trace.save(compact=True)`). A copy handles one chunk at a time.
- `scripts/observer_logs.py` (list, events, copy) for operators; the panel's copy runs it as a process of its own,
  so a long copy never takes the monitor's memory or GIL. Lines before a copy's range are marked "before_range".
- Each chunk opens with every state ("state" events, motions under way included); a copy starts with the newest
  states before its range and every change since.

## Open

- Not yet run against ROS (sim or lab): check sizes per hour and the tick cost of ~250 columns.
- `data_experiment/assembly_observer` must be made in Drive before the first copy.
- Stale values (TCP, wrench, tool, odometry, follower) keep their last value when a stream stops: fix proposed, not
  done.
- Not in any default plugin list yet.
- exec's `Recorder` stream could be dropped once the observer always runs (part records keep their ROS times).
