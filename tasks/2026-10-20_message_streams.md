# Message signals in the recording system (2026-10-20)

Status: **built, not committed.** The runs under `HUSKY_DRIVE_ROOT` are converted (441: 261 path runs, 180
constant-command runs, all three steps of `scripts/convert_recordings.py`); originals in
`/home/jakob/ra/workspace/backups/recordings_before_message_groups_2026-10-20.tar`.

## Why

`ctx.record` sampled signals once per tick (~20 Hz), stamped with the tick's time. The base's follower reports and
wheel odometry come at ~30–50 Hz, and the analysis needs each message with both its receive time and its robot
stamp. So `BaseInterface` grew its own deques (`follower_log`, `odometry_log`, `follower_history`,
`odometry_history`), the plugins saved them as unlabelled extras (`commands` N×5, `wheel_odometry` N×4), base_exp
built `_twist_log` in the same layout by hand, and readers picked columns by number.

Full-rate data stays a job for `ros2 bag` on the robot: the arms' driver topics reach the monitor through the rate
limiter (50 Hz). Message signals cover what reaches the monitor, per message, next to the monitor's own derived
values in one file.

## Decisions (user, 20 Oct)

Two independent axes:

- **A signal says what is recorded and when.** Its trigger is part of it (`Signal.on`): None for every tick, or an
  interface's `Event` for every message. A message signal reads the record the event hands it (`read(record)`).
  Other monitor state can still be read through a closure, but it is the latest known value, not synchronised with
  the message.
- **A trace says over what time span.** A live trace (`ctx.trace`) runs until untraced, e.g. for plots. A record
  (`with ctx.record`) covers one block.

Following from these:

- **`ctx.trace` and `ctx.record` take any mix of tick and message signals.** A `Trace` keeps one time series per
  trigger, each with its own times; message groups also keep each message's stamp.
- **Interfaces own no recording buffers.** They parse each message into a public record (with `update_time` and
  `stamp`), store it as the latest state, then fire an event with it.
- **Each value is defined once** in `signals.py`, as a value of a record (a private `_Field`), and offered per tick or,
  with `every_message`, per message. A plugin's own signal is a `Signal` with `read()`, or `read(record)` and `on=`.
- **No look-back.** A recording opens its block before what it records. Nothing reads data from before the send, so
  there is no pre-roll either.
- **Old recordings are converted once** (`scripts/convert_recordings.py`), not read in their old layout.
- **Runs are saved and read one way** (`shared/recordings/`, second part, user 20 Oct): a run folder that never clashes,
  experiment.json (what the plugin knew) beside recording.npz (arrays only, with the robot's serial), report.json from
  the analysis; readers pick a channel by its label.

## Built

- `robot_interface/event.py`: `Event` (attach, detach, fire).
- `plugin_api/trace.py`: `Signal.on` and `Signal.key` (`<event>.<name>` for a message signal); `Trace` with a
  series per trigger, `take(event, record)`, `times(key)`, `stamps(key)`; `save` writes "t" for the tick and
  "<event>.t", "<event>.stamp", "<event>.<name>" per event.
- `plugin_api/context.py`: `ctx.trace` attaches each message group to its event, `untrace`, the end of a record block
  and plugin stop detach it. A message signal that raises skips that message; the monitor counts it as a plugin
  failure at the plugin's next step (`_take_signal_failures`).
- `robot_interface/base.py`: `follower_reports` (`<serial>_follower`, `FollowerState`) and `odometry_messages`
  (`<serial>_odometry`, `WheelOdometry`, now also `BaseState.odometry`); both topics keep `MESSAGE_QUEUE` (50)
  messages between ticks. The logs and their accessors are gone.
- `plugin_api/signals.py`: the follower's values as private `_Field`s; `every_message` on each follower signal;
  `follower_state` and `wheel_odometry` new.
- base_exp: path runs record every follower report (command, state) and odometry message; the record block opens
  before the send. Constant-command runs record every odometry message; their commands are the tick signal (one send
  per tick). `LOG_BEFORE` and `_twist_log` are gone.
- base_planner: runs record the same message signals; the send (step 5) comes before the record opens, but before
  any ROS callback, so no report is missed.
- `TracePlot` plots each signal on its own trigger's times.
- Readers: `replay.track` (`follower.stamp`/`follower.command`, `odometry.stamp`/`odometry.velocity`), the
  planner report's odometry distance. The two `load` functions read the new keys unchanged.
- Docs: `trace.py`, `plugin_roadmap.md`, both manuals, the recording example (wheel odometry per message).
- Tests: message signals in `test/plugin_api/test_trace.py`, `test/robot_interface/test_base_events.py`, the planner's
  run recording with reports twice per tick, fixtures in the new layout.

### Second part: what the weak API left behind

- `shared/recordings/runs.py`: `run_folder` (makes the folder; "_2", "_3" on a clash), `write_description` /
  `load_description`, `save_recording` (adds "robot"), `load_recording` → `Recording` (serial dropped from the keys,
  `channel(key, label)`), `write_report`, `plain`. `series.py`: `within`, `held`, `finite_mean`, `span`.
- Mocap per message: `MocapSample`, `subscribe_mocap(event=)`, `BaseInterface.mocap_messages`,
  `signals.base_floor_pose(every_message=)`. Every event's topic keeps `MESSAGE_QUEUE` (now in `event.py`).
- base_exp: records every mocap sample; constant-command runs fire their own `<serial>_twist` event per command sent
  (`SentTwist`). `_times`, `_twist_signals`, `_twist`, `_twist_sent` and the `follower_command` alias are gone. Runs
  are saved through `shared/recordings/`; the setup, path label and outcome are in experiment.json, no longer in the
  npz; the report writes report.json (description and metrics) instead of overwriting experiment.json.
- base_planner: `attempt_folder` is `run_folder`; descriptions and `plain` from shared; the outcome only in
  experiment.json; mocap per message in runs.
- mocap_bars: each take is a run folder (experiment.json, recording.npz), no JSON inside the npz.
- The recording example saves to `data_experiment/example_recording/` on the Drive (was `/tmp`).
- Readers: one loader (`load_recording`) for base_exp, base_planner and the analysis; `tracking_context`,
  `tracking_position`, `follower_progress` and `follower_position_error` read by label; `replay.track` takes poses
  from `mocap.*` (each with the phase of the tick that saw it) or, for older runs, from `times`, and commands from
  `follower.*`, `twist.*` or the older tick signal; the window, hold-last, mean and span helpers from `series.py`.
- `ui/base_markers.py` redraws the trail when its newest point changes (it froze once the trace was full).
- `trace.py` states the tick rate.
- JSON as well as npz (user 20 Oct; npz the default): `recording_format:=json` (`MonitorConfig`) saves every run as
  recording.json, grouped by trigger ({"tick", "events", "extra"}, each signal with labels, units and one row per
  sample, unknown as null). `Trace.save` picks the format by suffix, `load_arrays` reads both to the same keys,
  `load_recording` finds either. A real run: 45 KB npz, 83 KB json, identical after loading.
- One unit per channel (user 20 Oct): `Signal.unit` is one unit for all channels or one per channel, e.g.
  ("m", "m", "rad"); `Signal.units` always lists one per channel, "" for none (a flag, a code, a quaternion part).
  Files store "<key>.units" (npz) or "units" (json); `Recording.unit(key, label)` reads one; `TracePlot` puts a
  shared unit in the title, mixed units on each line. What the codes mean (tracking_context's phase) is in the
  signal's docstring, not its unit. Step 3 of `convert_recordings.py` turned every old ".unit" text into ".units";
  all 441 runs converted, analysis outputs unchanged.
- Commands sent as events (user 20 Oct, group 1 of "what else could be events"): `BaseInterface.twists_sent`
  (`SentTwist`, the stop's zero twist included) and `paths_sent` (`SentPath`: id, waypoints, timed, caps; a stop's
  empty path included); `ArmInterface.trajectories_sent` (`SentTrajectory`: end joints, duration, waypoints, hold),
  `targets_sent` (`SentTarget`, none in test mode) and `wrenches_sent` (`SentWrench`, a hold's zero included). Each
  fires right after its publish, at the monitor's time. `signals.twist_sent`, `path_sent`, `trajectory_sent`,
  `target_sent`, `wrench_sent` read them. base_exp's own twist event is gone (it records `twist_sent`); its path runs
  also record `path_sent`. base_planner sends before its record opens, so it keeps the path and send time in its
  files instead. Still open from that list: state changes as events, per-fix results (tracking error, bar fit) as
  events, and a text log for operator actions.
- Docs: README "Experiment data" (the run layout every plugin uses), both manuals, the design guide, the roadmap.

### Checked

- Both analyses, old code on the old recordings against new code on the converted ones (copies): all 411 text
  outputs (CSV, Markdown, JSON) equal apart from the folder path, constant-command analysis included. Again after
  the second part, from the backup through both conversion steps: all 144 analysis outputs equal; all 412 base_exp
  descriptions equal the old ones without "metrics", every report.json holds the old metrics, and recomputing them
  with the new report gives the same numbers (20 runs sampled).
- Live, against `pure_pursuit_sim.launch.py` (domain 71, a private zenoh router): a real `BaseInterface` and
  `ctx.record` over a 1 m path kept 272 follower reports at 30 Hz and 452 mocap samples at 50 Hz against 97 tick
  samples, idle → following → done, every stamp before its handling time, listeners detached after the block; the
  odometry group empty (no odometry in the simulator); saved and read back through `shared/recordings/`.
- Not run live: the base_exp and base_planner plugins in the monitor, and wheel odometry from a robot.

! Message signals run in step 1, per message: keep them cheap. Other state they read may be up to a tick old, and
  which of this tick's messages they see depends on subscription order.
! `update_time` is when step 1 handled the message, not when it arrived: messages queued between ticks get nearly the
  same time. Time within a stream by its `stamp`.

## Todo

- [ ] Run base_exp and base_planner once in the monitor with the simulator (isolated domain): the recording holds
      `<serial>_follower.*`; on the robot also `<serial>_odometry.*`.
- [ ] `convert_recordings.py` on every other copy of the recordings (the real Drive if it is not this folder).
- [ ] Commit (teleop only; propose grouping first).

## Found in the same scan, not part of this task

- The overview plot is near copied (base_exp report vs planner report).
- What each plugin's experiment.json holds still differs (base_exp: one run; base_planner: a plan attempt and its
  run); the files, their place and how they are read are shared.
- `_excluded.txt` has two parsers with incompatible syntax (`experiments/base_exp/common.py`,
  `experiments/base_planner/analysis.py`).
- `experiments/base_planner/analysis.py` and `scripts/analyze_base_exp.py` create folders under the Drive root,
  which `drive.py` says code never does.
- Stay as they are: `StreamStats` and the mocap `marker_errors` window are health windows, not recordings.
  `marker_errors` also feeds `pose_lost`.
