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
- **Task origin** (user, 15 Oct): (0.4, −0.8, 180°), fitted so the benchmark's paths clear the lab (10 cm
  at the tightest, at the ada tripod and Belle).
- **step_110_out**: the way back out of the bar structure, its own task.
- **Pilot runs** (15 Oct, run.json only, before tasks and experiment.json): archived to
  `data_experiment/base_planner/_archive/pilot_1015/`; the report reads experiment runs only.

## Open

- Lattice fine-tuning (user, 15 Oct): prefer open space; a suboptimal path within the time limit beats no path;
  search again when the path fails the follower check (it gives up now). With `alice_tiles_2` the tall walls test
  fails that check at xICR -0.20 (0.3 cm), so it is marked xfail until then.
- Automation (drive to each task's start, plan with each planner, commit, repeat) is not built: the panel flow is
  manual.
