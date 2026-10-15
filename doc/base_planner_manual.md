# Base planner experiments

How the base planners are compared: the goals, the tasks, how attempts are recorded, and how to collect and analyse
them. Follows `doc/experiment_design_guide.md` and `doc/experiment_report_guide.md`.

## Goals

**Goal:** a base planner that drives Alice where assembly needs her. Here the goals take the place of hypotheses:
the experiment compares the planners on each.

1. **Narrow passages:** drive into, through and out of passages with a few centimetres to spare.
2. **Around obstacles:** find paths around robots, walls and gaps that only look passable, easily (often and fast).
3. **The ultimate test:** at process step 110, drive into the centre of the bar structure.

Two planners are compared, each with its default settings: **RRT + arcs** and **Lattice (hybrid A\*)**. The better
one on a metric is bold in the report (rules in the report's "What we measured").

## The experiments

| | Question | Goals | Attempts to collect | Report |
|---|---|---|---|---|
| **A. Benchmark** | Which planner solves which task, how fast, with what margin; how does the follower drive its paths in simulation? | 1, 2, 3 | every task offline, by script: the lattice once (it is deterministic), the RRT 5 times | `<experiment>/_analysis/summary.md` |
| **B. Robot** | How does the robot drive each planner's paths? | 1, 2 | the virtual tasks on the real robot in a clean lab, each planner 3 times per task; the live simulator first, once | the same report |
| **C. Step 110** | Can each planner drive into the bar structure, on the robot? | 3 | task `step_110` with the process at step 110, each planner 3 times | the same report |

One experiment name per collection, e.g. `benchmark_1015` (A) and `robot_1016` (B and C together): a report covers
one experiment.

### The tasks

The standard set is `plugins/base_planner/tasks.py`, taken from the planner tests. Each is a start, a target and the
obstacles in the way:

| Task | Goal | What it asks |
|---|---|---|
| free | 2 | nothing in the way |
| around_robot | 2 | a parked robot in the straight line |
| beside_wall | 2 | target facing a wall, 5 cm short of it |
| looks_wide | 2 | a gap 1 cm too narrow for the margin; walls short enough to go around |
| passage | 1 | through a passage 1 m deep, 4 cm to spare |
| narrow_passage | 1 | through 1.5 m with 3 cm to spare, starting beside it facing across |
| tall_walls | 1 | between walls taller than Alice, 4 cm to spare, starting off-centre |
| into_passage | 1 | into the middle of the narrow passage |
| into_facing_out | 1 | into it, ending facing back out |
| out_of_passage | 1 | out of the middle of the narrow passage |
| too_narrow | 1 | narrower than Alice: must fail (a check) |
| step_110 | 3 | into the bar structure at process step 110 |

**Virtual obstacles:** all tasks but `step_110` put their obstacles into the scene as virtual bodies (purple): the
planner avoids them, the floor stays empty. They are placed with the task frame at `TASK_ORIGIN` in `tasks.py`.
! Set `TASK_ORIGIN` to the middle of a clear area about 5 m by 3.5 m before collecting; it is recorded with every
attempt.

**step_110** uses the process plugin's scene at step 110 (`PROCESS`, `PROCESS_STEP`). Its start and target are
`PROCESS_START` and `PROCESS_TARGET` in `tasks.py`; the target is inside the bar structure (confirmed 15 Oct).

### The method

- **Varies:** the planner. **Held fixed:** the tasks, Alice's arm stowed (`ArmConfig.stow_joints`), the planners'
  default settings, the planning speeds, the task origin, the follower model (`alice_tiles`). All are recorded with
  every attempt.
- **Measured independently of the planner:** clearance in the planning world; deviation and end from mocap (robot)
  or the simulated pose against the path sent, the same for both planners.
- **Model groups (offline):** each path found is driven by the simulated follower (crl_husky pure pursuit with
  `alice_tiles`) twice over: **exact model**, the simulated robot exactly as the follower's model (xICR −0.129 m),
  and **deviated models**, the same with the true xICR at −0.20, −0.15 and −0.10 m (the planners' xICR range). The
  report gives finished, deviation and end per group; a deviated group's numbers are the worst of its drives.
- **Unit:** one attempt = one plan from the task's start. An attempt counts for a task only when it starts within
  10 cm and 10° of the task's start and aims at its target (the panel checks); driving to the start is a free
  attempt and left out.
- **Repeats:** the RRT is random, so 5 offline and 3 on the robot per task; the lattice is deterministic offline
  (once), 3 on the robot (the start differs a little each time).
- **Checks with known answers**, in every report: a drive exactly on its path measures 0; `too_narrow` must fail.
- **Effects the system has anyway:** the follower aims for a piece's end within 5 mm but accepts up to 2 cm after
  2 s near it (crl_husky `settle_tolerance`, `settle_time`); commands sent before it reports done still act for the
  command delay (about 0.2 s), so runs are recorded 2 s past done; a skid steer's base_footprint swings on a small
  circle when turning on the spot.
- **Failure:** no path; a run the follower did not finish (aborted, timed out); a simulated drive that did not
  finish. Runs stopped or replaced by the operator, and attempts listed in `<experiment>/_excluded.txt` (folder
  name, then the reason), are left out.

### Where the data lives

`$HUSKY_DRIVE_ROOT/data_experiment/base_planner/<experiment>/<date>_<time>_<robot>_<task|free>_<real|sim|offline>/`:

- `experiment.json`: experiment, task, robot, environment, conditions (task origin, process step, the arm's angle
  from stow, speed caps), planner and settings, start, target; the plan (found or why not, time, length, clearance,
  notes); the run (outcome, controller) once committed; offline, the simulated drives (model group, true xICR,
  finished or why not).
- `plan.npz`: the path and the floor around it. `recording.npz`: the run's samples. `simulated.npz`: offline
  follower traces.
- The report writes `plan.png` per attempt, `overview.png` and `report.json` per run, and `_analysis/`.

---

## Setup (every terminal)

```bash
cd ~/ra/workspace
source venv/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
export EXP=robot_1016               # the experiment name
export HUSKY_DRIVE_ROOT="$HOME/Insync/<account>/Google Drive - Shared with me/2025-03 Husky Assembly"
```

`data_experiment/base_planner/` must exist in the Drive and be synced; experiment folders are made in it.

## A. Benchmark (offline, no robot)

```bash
python3 src/husky-assembly-teleop/scripts/benchmark_base_planner.py --experiment $EXP
python3 src/husky-assembly-teleop/scripts/analyze_base_planner.py --experiment $EXP
```

About 10 minutes: every RRT attempt that finds no path takes the full 10 s. Name tasks to run only those.
`--resimulate` drives an experiment's recorded paths again (e.g. after the model groups change), without planning.

## Simulator (B, first)

Its own ROS domain, this computer only. ! Never on the fleet's domain 80: the real base obeys the same `cmd_vel`.

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=70 ROS_LOCALHOST_ONLY=1
ros2 launch crl_husky pure_pursuit_sim.launch.py namespace:=a200_0804 mocap_id:=1840 \
  controller_model:=alice_tiles sim_model:=alice_tiles xICR:=-0.15
```

and the monitor, in the same domain:

```bash
ros2 run husky_assembly_teleop husky_monitor --ros-args -p robots:="['alice']" \
  -p plugins:="['robot_control', 'base_planner']"
```

## Real robot (B and C)

> **Safety.** The obstacles are virtual: nothing stops Alice but the follower and the e-stop. Clear the area around
> `TASK_ORIGIN`, keep the e-stop in reach, watch every run.

Fleet settings (`rmw_zenoh_cpp`, `ROS_DOMAIN_ID=80`). On the robot, the follower with Alice's model:
`ros2 launch crl_husky pure_pursuit.launch.py model:=alice_tiles`. On the control PC, the mocap relay and the
monitor with `plugins:="['robot_control', 'base_planner']"`; for C also `-p process_directory:=` the process folder.
Switch Alice's `platform_velocity_controller` on in `robot_control`.

## Collecting attempts (the panel)

1. Type the experiment name into **Experiment**, tick **Record**. Stow Alice's arm (**Stow** in `robot_control`):
   the planners plan with it stowed, and the panel warns when it is not.
2. Choose a **Task**: its virtual obstacles appear, the target is set. For `step_110`, select step 110 in the process
   plugin first, and leave the process unloaded for every other task (a loaded process makes a virtual task's scene
   unclean, and the attempt is left out).
3. **Load → Task start**, then **Plan** and **Commit**: Alice drives to the start (a free attempt, left out).
4. **Load → Task target**, choose the **Algorithm**, **Plan**. The attempt is recorded, found or not. **Commit** if
   found; the run is added to the attempt when the follower is done.
5. Repeat 3–4 per planner until each has its attempts on the task, then the next task.

## Analyse

```bash
python3 src/husky-assembly-teleop/scripts/analyze_base_planner.py --experiment $EXP
```

Then have a subagent write `results.md` from `_analysis/summary.md` with `_analysis/results_prompt.md`.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| Record is greyed out | `data_experiment/base_planner/` is missing under `HUSKY_DRIVE_ROOT`: create it in Drive |
| An attempt shows under "free" | it did not start within 10 cm and 10° of the task's start: Load Task start and drive there first |
| A virtual task's attempt is left out, "a process was loaded" | unload the process for virtual tasks |
| `step_110` attempts are free | step 110 is not selected in the process plugin |
| RRT: "the target is too close to something for the search's checks" | the RRT's checks grow near a move's ends; the target has less room than that |
