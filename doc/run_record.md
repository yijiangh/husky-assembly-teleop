# The run record: what an assembly run writes, how, and how the state moves from step to step

For the operator who reads a run back after a lab day, and for whoever changes what is written. The code is in
`husky_assembly_teleop/shared/runs/runs.py` (the files), `plugins/assembly_run/` (who writes them),
`plugins/movement_exec/plugin.py` `_record_part` (one movement's record) and `shared/execution/servo_record.py`
(a servo's). The old monitor's way of doing the same is explained alongside, since the lab notes still speak of
`progress.json` next to the BarAction files.

## 1. One run is one folder

Everything the monitor keeps about one attempt at building the process goes into one folder inside the process
folder:

```
<process>/runs/<YYYYMMDD_HHMMSS>/
    run.json            what this run is: the process files' hashes, the robots, the code, the settings
    progress.json       how far it got: each action's status, where each robot was left
    events.jsonl        one line per thing that happened, in time order
    movements/<action>/<movement>.<n>.json      one file per part run (motion, tool, insert, retreat, ...)
    servo/<action>/<movement>.<n>.json + .png   one file and one picture per servo
```

The folder is made at the first thing the run records (`[assembly_run] started run ...`), so a process you only look
at leaves nothing behind. The timestamp is the run's name; `run.json` repeats it.

**In the old monitor** the unit was the design problem, not the attempt: `progress.json` sat next to
`ActionSchedule.json` and `BarActions/`, one per problem, overwritten as the assembly went on; the planned paths were
written back into the action files as `.live-solved.json` sidecars; the servo loops wrote under
`<drive>/bar_action_live_replan_exe/<date>-servoing/`, the as-built measurements under `<date>-asbuilt/`. Each
monitor (one per robot) wrote into the same `progress.json`. A second attempt at the same problem meant "Reset plan
skeleton to the Rhino export", which threw the first away.

## 2. The files

### run.json

Written once at the start (`save_info`), schema `husky_run/1`:

| key | what |
| --- | --- |
| `name`, `created_at` | the folder's name and when |
| `process`, `process_hash`, `action_hashes` | the process folder and the SHA-256 of `process.json` and of each action file: a run belongs to exactly these files |
| `robots` | per robot `{serial, urdf_sha256, relay_calibration}`: which URDF and which base calibration were in use |
| `commits` | the monitor's, the core's and tamp's git commits |
| `settings` | the run's settings as they were at the start (`RunSettings`: insert split, spiral, rigid insert, the spiral's seconds and turns, servo by bar) |
| `continued_from`, `changed_since` | when the run continues an older one: which, and what changed in the files since |

The hashes are what make "Continue" safe: a run made on other files is never written on, it is continued in a new run
(`continued_from`), and the operator chooses between **Continue newest**, **Continue match** and **New run** when the
newest run was made on another export.

### progress.json

Rewritten (atomically: a temp file, then a rename) whenever something changes, schema `husky_run/1`:

```json
{"current": {"action": "B21_J", "movement": 3},
 "actions": {"B20_J": {"status": "done", "seq": 1, "marked_at": "...", "assumed": false,
                       "ends": {"robots/a200-0806": {"base": [x, y, z, qx, qy, qz, qw],
                                                     "joints": {"left_ur_arm_shoulder_pan_joint": 0.1, ...},
                                                     "source": "live", "at": "...", "after_action": "B20_J"}}}},
 "robots": {"robots/a200-0806": {... the newest end ...}}}
```

- `current` is the step the operator is on; it follows the process panel and is what the monitor selects again at
  the next start.
- An action's `status` is `pending` or `done`; `seq` numbers the marks; `assumed` says the robot was not live at
  Mark done and the export's end was written instead (the second click).
- `ends` is where every live robot stood when the action was marked: its base pose and every joint, as measured
  then (`live_record`). `robots` keeps the newest end per robot: the run's **beliefs** (section 4).

The old `progress.json` had the same three parts (actions, robots, the current index) plus `skipped` and the support
robots' `holds`; the new one drops both: a hold is now a movement like any other, and nothing is skipped.

### events.jsonl

One JSON object per line, appended and flushed at once (a line cut by a crash is ended before the next is written):

```
{"time": "...", "ros_time": 1793100360.3, "event": "mark_done", "action": "B20_J", "movement": null, "assumed": false, "ends": {...}}
```

The events: `session` (a monitor opened the run, new or continued), `settings` (a run setting changed, with the
changes), `mark_done` / `reopen`, `mocap_offset` (a base's mocap offset applied or reset, with its xyz), and per part
`prompt`, `confirm`, `cancel`, `soft_stop`, `error`, `exec` (with the record file it wrote), `servo`, and the planning
events `plan`, `check_goal` and `check_tool0` with their numbers. Reading the lines in order replays the lab session.

### movements/<action>/<movement>.<n>.json

One file per part the executor ran (`n` counts the tries). A motion's record has:

| key | what |
| --- | --- |
| `movement`, `routine`, `robot`, `serial`, `step`, `part` | which movement, how it is run (`transfer_free`, `insert`, ...), which robot, which operator step and which part of it |
| `basis` | what the plan was made from: the start joints, **the base pose when planned**, the other robots' bases and joints, the ROS time, the goal base, the transfer workaround and the run's statuses then (`PlanBasis.as_record`) |
| `plan` | the planner, its attempts and seconds, `joint_names`, **`rows`** (every joint row of the path), the goal joints, the end error per flange, the warnings |
| `command` | what was sent: the retimed positions and velocities per arm, the duration, the largest joint travel |
| `exec` | **the ROS time and the robot's base at the part's start and at its end**, as measured then |
| `outcome` | how it ended: `ARRIVED` / `held` / ..., the end joints and their error, the holds, `result`, `status`, `reason`, `dry_run` |

A compliant part (insert, retreat, handoff) has the same frame with `command = {settings, plan}` and an `outcome`
that keeps its phases with their times, the controller switches, the stall times, the peak force per arm, the base's
largest drift, the end errors, the retreat's checks and replanned tail, and one sample per tick of every arm's
target, TCP and wrench (`stream`, at most 1300 samples). A servo part names its `servo_file`.

So a run keeps every plan that was executed, where the robot stood when it was planned and when it ran, and what
was sent. What it does not keep: plans that were made and then cleared or replaced without an Exec (their `plan`
event in `events.jsonl` has the numbers, not the rows).

### servo/<action>/<movement>.<n>.json and .png

A servo's record (`servo_payload`): the mode (tool0 or the MoCap bar), the bar, every pass's reading (tool0 errors
per side, the base's drift, the wrenches, the bar's deviation from its design pose by MoCap and by FK), the moves
made, the settings and the outcome; the picture plots the readings pass by pass, as the Exec plots window shows them live.

## 3. Who writes, and when

The `assembly_run` plugin owns the run. Every other plugin hands it what to record (`AssemblyRunContract.request_record`)
and never touches the files; `assembly_run` writes on a thread of its own (`writes.py`), so a slow disk never stalls
a tick, and a write that fails is said in the log and tried again. The panel's **Mark done** and **Reopen** are its
own: Mark done takes the snapshot of every live robot (base within 1 s, joints within 0.5 s) and writes the action's
`ends`; a robot not live gets the export's end on a second click, marked `assumed`.

On Load, the plugin reads the runs folder: the newest run made on these very files (the hashes match) is continued
without a question; otherwise the three buttons. Nothing is written until the runs were read, so a half-read folder
is never overwritten.

**In the old monitor** the connected robot's `HuskyMonitor` wrote `progress.json` directly on 'Mark action done'
(`progress_io`), wrote the planned configurations back into the action files on 'Adopt' and 'Save travel-to-load
plan', and appended the servo and as-built files itself; three monitors ran, one per robot, each writing.

## 4. How the state moves from step to step

The data models the two monitors hold, side by side, are in `doc/representation_old_vs_new.md`.

This is the part that changed most. The question is: when the monitor plans the next movement, where does it think
each robot is, and where does a planned movement start?

**The old monitor** kept a "live belief" and wrote it forward into the loaded action: after every executed step,
`_carry_configuration_forward` copied the measured arm joints (and, after a servo, the measured base) into the start
state of every later stationary step and of the next arm movement of the loaded action, so the next plan started
from them; 'Adopt' and 'Save' could write those configurations into the action file for good. For the robots that
were not connected, `progress.json`'s belief (base frame and configuration from their last Mark done, kept if mocap
had seen the base within `LIVE_BELIEF_MAX_AGE_S` = 1 s) placed them in the planning cell, else the export's state,
else parked.

**The new monitor** plans every movement from the live robot and never writes into the process:

1. **The acting robot** is always the snapshot: its measured base (mocap) and joints at the moment Plan is clicked
   (`MotionPlanRequest(snapshot=...)`). A plan goes stale when the robot moves (2° on an arm, 2 mm / 0.5° on the
   base) or another robot moves (1 cm / 1°), and is used by one Exec only (`shared/execution/basis.py`).
2. **The goal of a transfer or an approach** is solved by IK at the live base (**Live IK**), and that goal is reused
   by the plan while the base stays within 2 mm / 0.5° of where it was solved. After a transfer's MoCap-bar servo,
   the insert's goal is the authored move applied from where the servo left the flanges (`carried_insert_goals`).
3. **The other robots** come from the snapshot when they are measured; a robot that is not live comes from the run's
   **beliefs**: the newest `ends` of `progress.json` (`beliefs_of`), which Mark done wrote; a robot without a belief
   is placed where the export has it at that step, else parked (`scene_for`: live, then belief, then design, then
   parked).
4. **The process panel's drawing** is the only place the carried-forward joints still exist: after Live IK or a plan,
   the action's later movements are drawn with the joints found (`Carry`, the "joints from live IK / plan" chip).
   Plans, runs and records never read them; a new action drops them.

So the old monitor's "state propagation" became three things: the live robot for the acting one, Mark done's ends
for the others, and a drawing aid. Nothing is written back into the process files, which is why the hashes in
`run.json` stay valid through a whole run.

## 5. Reading a run back

- `events.jsonl` in order is the session's story; `grep '"event": "exec"'` lists every part run with its record
  file.
- A movement record's `plan.rows` with `basis.base` (the base when planned) reconstructs the planned motion;
  `exec.base_at_start` / `base_at_end` say where the base really was when it ran (they differ from `basis.base` by at
  most the staleness limits above, or the plan would have been refused).
- `progress.json`'s `robots` is where the monitor will place a robot that is not live at the next start.
- Every record is plain JSON; `shared/runs/runs.py` reads them back (`find_runs`, `read_run`).
