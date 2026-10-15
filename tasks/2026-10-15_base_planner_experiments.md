# Base planner experiments (2026-10-15)

Spec: `doc/base_planner_manual.md`. Code: `plugins/base_planner/{tasks,recording,report,analysis}.py`,
`scripts/benchmark_base_planner.py`, `scripts/analyze_base_planner.py`.

## Decisions

- **Goals instead of hypotheses** (user): narrow passages in and out; around obstacles easily; process step 110 into
  the bar structure as the ultimate test. The report compares the two planners and bolds the better one.
- **Metrics** (user): plan time, success rate, path length, clearance (margin), deviation and end position, offline
  (simulated follower) and on the robot or live simulator, kept apart.
- **Tasks from the planner tests** (`tasks.py`), with virtual obstacles so the robot runs in a clean lab; the cell
  task uses the process scene. `too_narrow` stays as a check that must fail.
- **Records**: every plan attempt, found or not, with planner, settings, time, path, clearance and the floor; a
  committed run is added to its attempt. One folder per attempt under the experiment (`experiment.json`).
- **An attempt counts for a task** only from the task's start (10 cm, 10°) to its target; driving to the start is a
  free attempt, left out. A virtual task with a process loaded is left out (scene not clean).
- **Plan time** counts attempts that found a path: a planner giving up at once must not look fast.
- **Bold**: gap larger than both spreads and the metric's resolution; success: any difference; overview over the
  tasks where both planners have a value, per-task means first.
- **Model groups** (user): offline drives with the exact follower model (xICR −0.129) and deviated models (true
  xICR −0.20, −0.15, −0.10), reported per group. Unfinished drives count under Finished, not in deviation and end.
- **Repeats**: RRT 5 offline, 3 on the robot; lattice once offline (deterministic), 3 on the robot.
- The RRT's "no path found in 10 s" after 0.01 s was its start/target collision check with its grown end margins;
  `search.connect` now says so.

- **Follower end** (user, crl-husky jg/mpc-wip): aim for 5 mm (`position_tolerance`), accept 2 cm after 2 s near
  the end (`settle_tolerance`, `settle_time`); off to the side up to 6 cm once arrived along the path. The ~5 cm
  "short" in the first benchmark was the simulation cutting the commands still in flight; runs and simulation now
  go 2 s past done, and the report shows the drift after done.

- **Arm stowed** (user): every plan, live and offline, is made with the planned robot's arms at their stow pose;
  the panel warns when the live arm is more than 5° from it, and the ghosts show it stowed. Stowed, Alice is 0.68 m
  wide at every height, so `tall_walls` is sized from that. The planner unit tests keep the arm out (joints 0): the
  harder, lopsided case.
- **Step 110 target** (user, 15 Oct): (0.43, −2.785, 0°), inside the bar structure; the first target was outside.
  Re-run as `benchmark_1016`.
- **step_110_out**: the way back out of the bar structure, its own task.

## Open

- Set `TASK_ORIGIN` to the clean lab area.
- The 9 pilot runs of 15 Oct (`data_experiment/base_planner/2026…_real`, run.json only) predate tasks and
  experiment.json: the report does not read them (`report.py` still draws them one by one).
- Automation (drive to each task's start, plan with each planner, commit, repeat) is not built: the panel flow is
  manual.
