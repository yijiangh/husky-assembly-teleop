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

- **Arm stowed** (user): the experiments hold Alice's arm at its stow pose: the benchmark plans it stowed, and a
  recorded live attempt warns when the arm is more than 5° from it. Stowed, Alice is 0.68 m wide at every height,
  so `tall_walls` is sized from that. The planner unit tests keep the arm out (joints 0): the
  harder, lopsided case.
- **Arm as it is** (user, 19 Oct): live plans are made with the robot as it is: its arms, and every body it holds,
  which moves with the base in every check (`BaseCollisionWorld`; contact between robot and held body is left out).
  The benchmark script still stows the arm (`stowed`), and recorded attempts warn when the live arm is not stowed. A
  plan goes stale once an arm joint moves 2°. Checks turn in steps that move no part more than 5 cm
  (`algorithms.base.yaw_step`): 0.05 rad stowed, less with the arm out or a long body held.
- **Step 110 target** (user, 15 Oct): (0.43, −2.785, 0°), inside the bar structure; the first target was outside.
  Re-run as `benchmark_1016`.
- **Task origin** (user, 15 Oct): (0.4, −0.8, 180°), fitted so the benchmark's paths clear the lab (10 cm
  at the tightest, at the ada tripod and Belle).
- **step_110_out**: the way back out of the bar structure, its own task.
- **Pilot runs** (15 Oct, run.json only, before tasks and experiment.json): archived to
  `data_experiment/base_planner/_archive/pilot_1015/`; the report reads experiment runs only.

- **Lattice prefers room** (user, 16 Oct): the guide checks fits with the margin (squeezes the margin closes no
  longer pull the search in), its way costs up to 3× where few headings fit, and portals found to have no way
  through are skipped (they took the tries from good ones: `passage` failed at the 180° task origin). Checked
  offline against the old lattice on every task and yesterday's live failures: none worse, `passage` and the
  retreat north of step 110 now found.
- **Thin bars** (user, 16 Oct): the 2 cm bars fell between the floor map's rays (one per 5 cm cell), so the guide
  sent the search across the step 110 south row. `floor_heights` casts 3 × 3 rays per cell, the cell taking the
  highest hit (0.8 s instead of 0.1 s per plan); the attempt that failed now plans in 5 s.
- **Follow check faster** (user, 16 Oct): the follower simulation spent its time in crl_husky's command-delay
  prediction (`PurePursuit._predict`, numpy on 3 numbers ~20 times a cycle); now plain floats, ~3× faster, traces
  identical. Next, if needed: line up portals on the fit map (rough stage only, 4 s of 10 near the bar structure),
  a raster "surely clear" test before PyBullet checks, the planner in its own process (live is ~2.4× slower).

## Open

- Lattice: search again when the path fails the follower check (it gives up now). With `alice_tiles_2` the tall
  walls test fails that check at xICR -0.20 (0.3 cm), so it is marked xfail until then. The follower check takes up
  to 3 s of the budget (`looks_wide`).
- Automation (drive to each task's start, plan with each planner, commit, repeat) is not built: the panel flow is
  manual.
