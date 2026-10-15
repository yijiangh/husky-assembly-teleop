# Base controller experiments

## Goal and hypotheses

**Goal:** a base controller that follows paths accurately enough for assembly, including tight passages.

**Hypotheses:**

1. **The robot's motion model changes** with speed and turn rate, arm configuration and floor. The model has four
   values: xICR (how far the tracked point lies ahead of the point the robot turns about), speed efficiency,
   steering efficiency, and the delay from command to motion.
2. **Pure pursuit with the ideal model is not good enough.**
3. **Pure pursuit with an adapted model is better, but probably not good enough for tight passages.**

## The three experiments

| | Question | Tests | Runs to collect | Report |
|---|---|---|---|---|
| **A. Robot model** | What is the model of this robot at this speed, under these conditions? | basis for C | standard paths at one speed: real robot; simulator with the new model; ideal simulator | `robot_model.md` |
| **B. Model over conditions** | Does the model change with speed and turn rate, arm and floor? | hypothesis 1 | real robot: constant commands, 3 rounds per condition, all in one experiment | `constant_commands.md` |
| **C. Controllers** | How well does pure pursuit follow paths with the ideal and with the adapted model? | hypotheses 2, 3 | standard paths at A's speed, 3 rounds, per scenario: simulator and real robot, each with both controller models | `summary.md` |

**A and C use only paths at one speed**, so the model they fit and test holds for that speed. B shows whether it
changes with speed; we expect it to, so **each speed gets its own model**: run A and C again per speed (see below).
Order: A, then C, at 0.2 m/s; B per condition, any time.

Runs and reports live on the project's Google Drive:

- runs: `$HUSKY_DRIVE_ROOT/data_experiment/base_exp/<experiment name>/<date>_<time>_<robot>_<path>_<sim|real>/`
  (`recording.npz`, `experiment.json`, `overview.png`);
- reports: `$HUSKY_DRIVE_ROOT/data_experiment/base_exp/<experiment name>/_analysis/`.

**Experiment names:** A and C together under one name per condition and speed, e.g. `alice_tiles_stowed_v020`; B
under one name for all conditions of a robot and floor, e.g. `alice_tiles_commands`: its report compares them. An
experiment's runs are the run folders in its folder, so runs can be moved between experiments.

**Conditions** are recorded with every run from `CONDITIONS` in `plugins/base_exp/record.py` (base, arm, floor).
! They are not in the panel: before collecting under a new condition, edit `CONDITIONS`, set up the robot to match,
and restart the monitor. For path runs the analysis takes the experiment's conditions from its own runs (the ones
most were recorded under) and leaves the others out; constant-command runs are all loaded and grouped by condition.

**Speed of the path runs:** 0.2 m/s with a 30°/s turn-rate cap, geometric (`auto.SPEED`, `auto.MODE`); the follower
slows only to stop at the ends of pieces. ! Not in the panel: for another speed, edit `auto.SPEED`, rebuild, and use
a new experiment name. The analysis loads only path runs at the current `auto.SPEED`. Speed and turn rate vary
freely only in the constant commands (B).

**Tight passages are not driven:** automation keeps every path 0.3 m from everything. Judge hypothesis 3 from the
errors in `summary.md`: a passage needs the end and moving errors well below its clearance.

---

## Setup (every terminal)

From the workspace root:

```bash
cd ~/ra/workspace
source venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
export EXP=alice_tiles_stowed      # the experiment name
export HUSKY_DRIVE_ROOT="$HOME/Insync/<account>/Google Drive - Shared with me/2025-03 Husky Assembly"
```

`HUSKY_DRIVE_ROOT` is your local copy of the project's Drive folder. `data_experiment/base_exp/` must exist in it
and be synced. Create missing folders in Drive, never locally: Drive would get a second folder of the same name.

After pulling or changing code, build with the venv's Python:

```bash
python3 -m colcon build --symlink-install --packages-select crl_husky_msgs crl_husky husky_assembly_teleop
source install/setup.bash
```

> After a `crl_husky_msgs` change, restart every follower, simulator and monitor ("no follower" otherwise).

## Simulator

Its own ROS domain, this computer only. ! Never on the fleet's domain 80 or on a robot: the real base obeys the same
`cmd_vel`. In every simulator terminal, also:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=70          # the simulation domain; never 80
export ROS_LOCALHOST_ONLY=1
```

**Terminal 1: simulator and follower** (Alice, mocap id 1840, starting in open floor):

```bash
ros2 launch crl_husky pure_pursuit_sim.launch.py namespace:=a200_0804 mocap_id:=1840 start_x:=1.0 start_y:=-1.5 \
  sim_model:=ideal controller_model:=ideal
```

- `sim_model`: how the simulated robot moves. `controller_model`: what pure pursuit assumes.
- Models are `crl_husky/config/base_models/<name>.yaml` in crl-husky: `ideal`, and `alice_tiles_2` (Alice, tiles,
  arm stowed; `alice_tiles` is the earlier fit). Single values override the sim model, e.g. `xICR:=-0.10`.

**Terminal 2: monitor**, then open <http://localhost:8095>:

```bash
ros2 run husky_assembly_teleop husky_monitor --ros-args \
  -p robots:="['alice']" -p plugins:="['base_exp', 'obstacles']" -p viser_port:=8095
```

Stop with Ctrl-C, monitor first.

## Real robot

> **Safety.** Clear the floor, keep the e-stop in reach, watch every run. The guard soft-stops the robot within
> 10 cm of anything in the scene, but it runs over Wi-Fi and is no emergency stop.

Use the fleet's settings (`rmw_zenoh_cpp`, `ROS_DOMAIN_ID=80`, no `ROS_LOCALHOST_ONLY`), not the simulator's.

**After a crl-husky change, put the code on the robot.** `upload.sh` copies only files git tracks, so commit or
`git add` new files first:

```bash
cd ~/ra/workspace/src/crl-husky && ./upload.sh 192.168.0.113        # Alice (.114 Belle, .115 Cindy)
ssh administrator@192.168.0.113
cd ~/workspace && colcon build --packages-select crl_husky_msgs crl_husky
```

**Terminal 1 (control PC): mocap relay**, unless running: `ros2 launch crl_husky mocap.launch.py`

**Terminal 2 (robot): follower.** Restart it to switch models; every run records which one it used.

```bash
ssh administrator@192.168.0.113
cd ~/workspace && source install/setup.bash
ros2 launch crl_husky pure_pursuit.launch.py model:=ideal        # or model:=alice_tiles_2
```

**Terminal 3 (control PC): monitor**, then open <http://localhost:8080>. The other robots are listed so they are
avoided:

```bash
ros2 run husky_assembly_teleop husky_monitor --ros-args \
  -p robots:="['alice', 'belle', 'cindy']" -p plugins:="['base_exp', 'obstacles', 'robot_control']"
```

In `robot_control`, switch Alice's `platform_velocity_controller` **on**; in `base_exp`, choose Alice. Drive her to
open floor, at least 1 m from anything.

## Collecting runs (the panel)

In the `base_exp` folder:

1. Status: `tracked` and `follower idle` ("velocity controller off" is fine in the simulator).
2. **Experiment:** `$EXP`.
3. **Automation:**
   - `Controller: standard paths`: the same 10 paths in every scenario (straight, spot turn, wide and tight arc,
     sine, drive-turn-drive; mostly both directions). One round takes 2–3 min.
   - `Robot: constant commands`: no controller; each run stands 0.5 s, holds one command (v, ω) for 4 s open loop,
     stops. 29 commands (`twist.py`): every pair of v 0, 0.03, 0.2 m/s and ω 5, 15, 30°/s, both ways (what the
     follower sends: spot turns, its slow approach to a goal, driving); v 0.1 and 0.3 m/s at 15°/s; straights at
     0.03–0.3 m/s; reversing at 0.2 m/s. One round takes 7–9 min.
   - `Controller: random paths`: any of 44 paths; extra coverage for the breakdowns only, optional.
4. **Auto runs** is set to one round; `0` runs until Stop. For standard paths and constant commands it is the
   **total** for this experiment and setup, earlier runs included: after a stop, Start auto runs only the rest
   (e.g. 30 with 25 done: 5 more). Raise it for more rounds. For random paths it counts per Start.
5. **Start auto.** Both the standard paths and the constant commands resume where this experiment stands for the
   current setup ("paths: round 2: 3/10 paths"), so stopping and restarting is safe.

Controller failures (leaving the path, timing out) are recorded and the series goes on. It pauses for problems
around the controller (guard stop, e-stop, lost mocap, silent follower), with a line in the panel saying why: drive
the robot clear (on the real robot: velocity controller back on, D-pad in `robot_control`) and press Start auto again.
An e-stopped run is saved as "e-stopped" and left out of every report.

---

## A. Robot model

Under the A and C name, e.g. `alice_tiles_stowed_v020`:

1. **Real robot**, follower `model:=ideal`: **standard paths**, 3 rounds.
2. Analyse. `robot_model.md`, section "The model for crl_husky", prints the identified model: save it in crl-husky as
   `crl_husky/config/base_models/<name>.yaml` with the condition and speed in the name (e.g.
   `alice_tiles_stowed_v020.yaml`), rebuild, and upload to the robot.
3. **Simulator** with `sim_model:=<name> controller_model:=ideal`: standard paths, 1 round.
4. **Ideal simulator** (`sim_model:=ideal controller_model:=ideal`): standard paths, 1 round.
5. Analyse. In `robot_model.md`, the robots table lists no missing kinds, and every row of "Checks" says yes.

## B. Model over conditions

One experiment for every condition of a robot and floor, e.g. `alice_tiles_commands`. Per condition (an arm pose):
edit `CONDITIONS`, set the robot up to match, restart the monitor, then:

1. **Real robot: constant commands**, 3 rounds (87 runs, 20–30 min); the rounds count per condition. Start in the middle of the open floor,
   battery full; the robot drives back once it is 0.75 m from the start. The follower must run, for the drives back
   (with the robot's model, so they end at the start instead of circling it). The first time on a robot or after
   changes: 5 runs while watching, then analyse and check `constant_commands.md` has a wheel rise.
2. Analyse. `constant_commands.md` answers one question per section: section 1, speed and turn rate (per condition:
   the values at each speed and turn rate, and the change per m/s with its standard error); section 2, the condition
   (each condition's model, on the commands every condition drove and on all); section 3, the response to a command
   step. `commands_axes.png` shows sections 1 and 2: a line per condition against speed and turn rate. The details
   (per command, per run, runs with a stall, the split at the wheels) are in the CSV files beside it.
3. A new arm pose can change the turn gain a lot (1.6 with the arm fully forward): watch the first runs, and start
   with a few.
4. Where a value changes with speed, the model at A's speed holds only there: repeat **A and C per speed** the
   controller will drive at (`auto.SPEED`, a new name and model per speed, e.g. `..._v010`, `..._v030`). Where it
   changes with arm or floor, the same per condition.

! The delay in `constant_commands.md` includes the Wi-Fi from the monitor to the robot; the other values do not.

## C. Controllers

Standard paths at A's speed, 3 rounds each, in these scenarios (under the same name as A; `<name>` is A's model):

| Scenario | Simulator launch | Real robot follower |
|---|---|---|
| ideal model | `sim_model:=<name> controller_model:=ideal` | `model:=ideal` |
| adapted model | `sim_model:=<name> controller_model:=<name>` | `model:=<name>` |

The simulator runs first: they cost nothing and show what to expect. A's runs already count for the real ideal-model
scenario.

Analyse. `summary.md` "Headline" gives per scenario the success rate and the end and moving errors on the standard
paths (also without the turns on the spot); "Path by path" and "By template" show where they come from, "Failures"
how runs fail.

---

## Analyse

With the setup lines sourced (any ROS domain):

```bash
python3 src/husky-assembly-teleop/scripts/analyze_base_exp.py --experiment $EXP
```

It writes to `$HUSKY_DRIVE_ROOT/data_experiment/base_exp/$EXP/_analysis/`, each report with its CSV files and plots:

| Report | From | Contents |
|---|---|---|
| `summary.md` | path runs | why (hypotheses 2, 3), what was measured, scenarios; headline per robot, path by path, by template, failures, largest end errors |
| `robot_model.md` | path runs | why, what was measured, robots and what is missing; the real robot's model, one model for every run?, prediction 1 s ahead, simulator against real, checks, timing through the wheels, the model for crl-husky |
| `constant_commands.md` | constant-command runs, every condition | why (hypothesis 1), what was measured, what was run; speed and turn rate; the condition (arm pose); response to a command step |

**A bad run** (e.g. something went wrong that the plugin did not notice) stays on disk: list its folder name in
`_excluded.txt` in the experiment folder, one per line with the reason after `#`, and every report leaves it out.

The reports hold numbers only; how they are built: `doc/experiment_report_guide.md`. For written results, ask Claude Code: "Run a subagent with the prompt in
`$HUSKY_DRIVE_ROOT/data_experiment/base_exp/$EXP/_analysis/results_prompt.md`". It writes `results.md` there. Constant commands have their own: `commands_results_prompt.md`, which gives `commands_results.md`.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `base_exp` does not start: "no Drive root" or "... missing under the Drive root" | Set `HUSKY_DRIVE_ROOT`; sync (or create in Drive) `data_experiment/base_exp`. |
| "no follower" | The follower is not running, runs in another ROS domain, or is older than a message change: check `echo $ROS_DOMAIN_ID`, rebuild, restart. |
| "not tracked" | No mocap pose: simulator or relay not running, or wrong mocap id (Alice 1840, Belle 1850, Cindy 1860). |
| The robot jumps between two places | Two simulators publish the same mocap topic: `pgrep -af husky_sim`, stop the old one. |
| "auto not started: no lab border" | Add `obstacles` to the monitor's plugins. |
| The real robot does not move, or a run ends "not driven" | The velocity controller is off: switch it on in `robot_control`, also after every soft stop. |
| The monitor logs "cannot decode messages" | A follower with an older message definition still runs somewhere: rebuild and restart it. |
| A report leaves path runs out "under other conditions" | The experiment holds path runs recorded under different `CONDITIONS`; the path reports use the ones most have. Use one experiment name per condition for A and C. Constant commands are compared across conditions instead. |
| `constant_commands.md` counts runs with a stall | The robot briefly stopped during a constant command (seen with commands sent over Wi-Fi). Only the part without it is used; nothing to do unless most runs stall. `commands_stalls.csv` lists them. |
