# CDFM planners: merge `yh/holding-prm`, run the benchmark here, two planners in the monitor

Status (2026-11-05): A (merge) done as tamp 3e26057 on `yh/holding-prm` (unpushed; reviewer: faithful, nits only).
B: benchmark + both dashboards run in the ROS venv on the mock root (`pyinstrument` installed); fixes on tamp:
`long_path` import in `export_dashboard.py`, the export's serve message, the `.sh` scripts' exec bit, a README
section. Found: the Drive's roadmaps and cell caches predate the core's tool-contacts change, so the online
planner refuses them (`RoadmapMismatch`, touch bodies differ); cells and roadmaps must be rebuilt with the current
core (`build-cells --force`, then `process_precompute.sh`). C: in progress.

Order of work: A (merge) → B (benchmark + dashboards on the mock root, commit 1 with the submodule pointer) →
C (dropdown + route, commit 2) → fake-lab check. Subagents: an implementer per part with a reviewer after A and C
(memory: planner/implementer/reviewer loop); the merge resolution itself I do by hand, since it needs both
sides' intent. Things to confirm with the user before doing: `pip install pyinstrument` into the ROS venv;
pushing the merged `yh/holding-prm`; nothing is deleted or written on the Drive.

## Context

The tamp submodule (`external/husky_assembly_tamp`) has two branches: `yh/core-port` (checked out, pinned by the
monitor: Alice/Belle ssik solvers, M1 start-derivation diagnostics, linear planners' failure reports) and
`yh/holding-prm` (25 commits, developed on the Windows machine: the CDFM transfer-planner benchmark + its dashboard,
the holding-constraint roadmap planner "holding PRM", `plan_constrained_dual_arm(backend="birrt"|"prm")`).
The user wants `yh/holding-prm` to take in `yh/core-port`, the benchmark and dashboards to run on this Ubuntu
machine for both planners, and the monitor to plan the bar-held transfer (CDFM) with either planner when the
"Transfer workaround" box is unticked, chosen from a dropdown.

Facts found (read-only):
- The monitor already plans the held transfer with the BiRRT when the box is unticked:
  `shared/assembly_planning/tamp_route.py` `Route._transfer_held` (:295) → `held_search` (:464) →
  `plan_constrained_dual_arm(planner, mirror.state, active_bar_id=bar, goal_conf=goal, random_seed=seed, **HELD)`,
  3 tries (`ATTEMPTS`) of `HELD["max_time"]` 120 s. The servo's sampling fallback (`servo_route.py:183–205`)
  calls `held_search` too. The only missing piece is the PRM backend, which is not on `yh/core-port`.
- On `yh/holding-prm`, `backend="prm"` needs `prm_options["roadmap_dir"]`: a roadmap precomputed per bar at
  `<bench_root>/prm/<process folder name>__<bar>/roadmap.json.gz` (`scripts/plan_process_carry.py` default).
  It raises `FileNotFoundError` (no roadmap), `RoadmapMismatch` (built for another bar/grasp/load), `ValueError`
  (bar not attached to left tool0 with an attachment frame); no route → `(None, info)` with
  `failure_reason` in `prm_goal_unholdable | prm_no_start_free | prm_no_route | repair_*`. Not time-limited
  (typical 60–130 s). `save_query=True` by default writes `queries/<tag>.json.gz` into the roadmap dir, and
  `workers>1` writes `scenes/` there unless `scene_cache_dir` is set. No fallback to the BiRRT inside tamp.
- The Drive already holds the benchmark data (`data_experiment/cdfm_planner_benchmark`: 3 cell caches, libraries
  `cdfm_v1` + `smoke`, assets, runs, `prm/` with roadmaps incl. `261008_2ModulesDemo_purged_process__B21`,
  600 nodes, built on Windows at holding-prm 812928b) and the fake lab uses that very process (B20–B22).
- The ROS venv (`ros2_ws_next/venv`, Python 3.10, numpy 1.26.4, ssik 4.1.0, pybullet 3.2.7, networkx 3.4.2,
  scipy 1.15.3, matplotlib, termcolor, trimesh, yourdfpy, pytest, node 24) already has every benchmark dependency
  except `pyinstrument` (profiling) — check at step B1. tamp's `constraints-py311.txt` (numpy 2.x) is for its own
  3.11 venv; `setup.py` says numpy 1.26 stays for ROS2.
- Merge dry run (`git merge-tree`): conflicts in 3 files, `api.py` (2 hunks), `ssik_ik.py` (5), `core.py` (8);
  `keyframe/config.py` merges clean. Details in Part A.

User decisions (2026-11-05): dropdown + roadmap root as **run settings** in the Advanced planning folder (saved in
`run.json`); roadmap dir = Drive convention, with an editable **Roadmap root** text field (empty = convention);
benchmark checked in the **ros2_ws_next venv** first (separate uv py3.11 venv only as fallback); smoke runs on a
**local mock copy** of the bench root, nothing written to the real Drive.

## Branches and commits

- tamp: `git switch yh/holding-prm` (tracking `origin/yh/holding-prm`), `git merge origin/yh/core-port`, resolve,
  one merge commit. Not pushed until the user says (the monitor's submodule pointer then needs it pushed first:
  AGENTS.md "Branches"). Later fixes on the same branch, folded into few commits.
- monitor: new branch `yh/cdfm-planners` off `origin/core-port` (5ac621d5) in this checkout
  (`yh/insertion-demo` keeps its unpushed 4fad3096; `external/tracikpy/` stays untracked, untouched). Commits:
  (1) submodule pointer + benchmark notes/scripts, (2) the planner dropdown and its route, each after `pytest`.
- Mock bench root: `/home/yijiangh/Code/ros2_ws_next/dev/bench_mock/cdfm_planner_benchmark` (outside the repo
  and the Drive; say so in every report), copied from the Drive: `cells/260921_motion_sample-de81ff31`,
  `cells/261008_…-4be92c6a`, `library/smoke`, `assets/<both keys>`, `prm/smoke_prm`, `prm/261008_…__B21`,
  `prm/carry/carry_v1/starts.json`, `prm/dashboards.json`, `runs.json`, `compare/index.json` if present (~300 MB).

## Part A: merge `origin/yh/core-port` into `yh/holding-prm` (tamp submodule)

Resolve by hand, keeping both sides' features; "our" = holding-prm, "their" = core-port.

1. `motion_planner/api.py`
   - imports: keep both `from time import perf_counter` and `import time`.
   - tail of `_derive_constrained_start_for_plan`: core-port's `_with_context` / `stage_times` / `tracked_report`
     structure with holding-prm's key name `direct_path` (not `corridor`): `tracked_report = {k: v … if k !=
     "direct_path"}`, `"planner": "direct_path"`. Check the two auto-merged spots by eye: the direct-connect block
     (`"planner": "direct_path"` next to core-port's `start_conf`/`tracked`) and the main `info` dict
     (`ik_options`/`repair_settings` next to `start_conf`/`context`).
   - core-port's probe passes `max_time=GOAL_BRANCH_PROBE_S` to `derive_constrained_start_tracked`: needs the
     `max_time` parameter from core.py (below).
   - `plan_constrained_dual_arm` keeps holding-prm's `record_search, ik_options, repair, backend, prm_options` and
     core-port's info keys.
2. `motion_planner/ssik_ik.py`: `verify_ssik_joint_order(robot, arms=("left","right"), artifact_set=None)`, cache
   key `(robot, arm, set_name)`, names from `ssik_inprocess.joint_names(arm, artifact_set)`; `ARM_BASE_LINK` with
   alice/belle; `ssik_arm_branches(…, respect_limits=True, artifact_set=None)` calls
   `verify_ssik_joint_order(robot, (arm,), artifact_set=artifact_set)`. holding-prm's `parallel.py:82,113` call
   `verify_ssik_joint_order(scene.robot_puid)`: still fine.
3. `motion_planner/dual_arm_task_space_rrt/core.py`
   - Keep holding-prm's module functions `track_direct_path` (2-tuple return, used by `benchmark/direct_path.py`)
     and `first_blocked_index`; the nested `_track_to` wraps `track_direct_path` and adds core-port's `break_info`
     (4-tuple: `track, ok, n_total, break_info` with `ik_miss` / `branch_flip`), counting `profile["n_ik"]`;
     simplest: give `track_direct_path` an optional `break_out: dict | None` it fills, keep its 2-tuple.
     `_first_blocked_index` calls `first_blocked_index(track, _cc if joint_collision_fn else None)` so core-port's
     timing still counts. Keep `_pose_lists` and holding-prm's `pose_as_lists` both.
   - `derive_constrained_start_tracked(…, max_time=120.0)` from core-port; `candidates`, `profile`, `_finalize`
     from core-port; `direct_path` naming from holding-prm (`record["outcome"] = "corridor"` stays as is: the other
     monitor branch's `run_schema.OUTCOMES` reads it).
   - `update_debug_tree(debug_tree_out, success, iterations, nodes_a, start_pose, goal_pose, *,
     extend_stop_reasons=None, nodes_b=None, confs_a=None, confs_b=None, recorder=None, tree_start=None,
     tree_goal=None, outcome=None, elapsed_s=None, restarts=None)`: writes holding-prm's `tree1/tree2/tree_a/
     tree_b/attempts` (list from the recorder) **and** core-port's `tree_start/tree_goal` (`export_tree`),
     `outcome`, `elapsed_s`; core-port's int `attempts` renamed `restarts` (key collision with the benchmark's
     list, read by `benchmark/cdfm.py:321–368`).
   - `plan_pose_birrt`: holding-prm's `n_ik_solves`, `recorder`, `best_trees = (nodes_a, confs_a, nodes_b,
     confs_b)`; core-port's `stopped_on` and `_give_up(outcome)` (must pass `recorder=recorder` so `attempts == []`
     and set `n_ik_solves = 0`). Every `update_debug_tree` call passes both sides' kwargs.
   - `failure_reason` now reads `profile["outcome"]` (`max_time`, `max_iterations`, `start_in_collision`,
     `goal_in_collision`) instead of `rrt_failed`: fine for the monitor (it only prints it).
4. `keyframe/config.py`: auto-merge (alice/belle in `SSIK_ARM_BUILD` + holding-prm's artifact sets); check.
5. After resolving: `python -c "import husky_assembly_tamp.motion_planner.api, husky_assembly_tamp.benchmark.cli,
   husky_assembly_tamp.motion_planner.holding_prm.online"`, `python -m pytest benchmark/tests -q` (pure tests;
   the data ones need `HUSKY_BENCH_ROOT` = the mock, step B), then the monitor's `pytest -m safety` and
   `test/shared` (tamp imports from the monitor: `report=` on the linear planners, alice/belle ssik, `ik_backend`,
   `rebranch_toward_seed`, `failure_reason`). Reviewer subagent on the merged diff of the three files
   (fresh context, checks the hazards listed above).

## Part B: the benchmark and the dashboards run here, BiRRT and PRM

Environment: `source ros2_ws_next/venv/bin/activate` (no ROS needed); `pip install pyinstrument` if missing
(ask first; it is tamp's `install_requires`). `export HUSKY_BENCH_ROOT=<mock root>`,
`HUSKY_DESIGN_STUDY_DIR=<Drive>/data_design_study` (read only). All commands from `external/husky_assembly_tamp`.

B1. BiRRT benchmark, smoke size, on the smoke library (built from `260921_motion_sample-de81ff31`; the only
    source that must be given as `--source-key` since three caches exist):
    `python scripts/benchmark_cdfm.py run --name smoke_linux --library smoke --source-key 260921_motion_sample-de81ff31 --seeds 1 --max-goals 1 --max-starts-per-goal 2 --workers 2`
    then `summarize --name smoke_linux` and `export-dashboard --name smoke_linux --library smoke`.
    Expect `runs/smoke_linux/{summary.json,dashboard/}` in the mock root.
B2. PRM, smoke: `python scripts/prm_pilot.py info --name smoke_prm` (reads the synced run), then a small build +
    query of our own on the fake-lab bar: `benchmark_cdfm.py build-cells "$HUSKY_DESIGN_STUDY_DIR/261008_2ModulesDemo_purged_process" --bars B21`
    is already cached (`cells/261008_…-4be92c6a`); plan the carry with both backends through the headless script:
    `python scripts/plan_process_carry.py "$HUSKY_DESIGN_STUDY_DIR/261008_2ModulesDemo_purged_process" --bar B21 --cdfm-backend prm --prm-run 261008_2ModulesDemo_purged_process__B21 --save-dir <mock>/prm/plans/linux --no-plot`
    and the same with `--cdfm-backend birrt --max-time 60` (the Drive's saved runs: prm OK in 130 s, birrt
    failed after 737 s; a BiRRT failure here is acceptable, the check is that it runs). Then
    `prm_pilot.py export --name 261008_2ModulesDemo_purged_process__B21` for the PRM dashboard.
    If the mock B21 roadmap refuses (`RoadmapMismatch` from a core/URDF change since 812928b), rebuild a small one:
    `scripts/prm_runs/process_precompute.sh` with `NODES=600 WORKERS=4 TRACK_MM=0` (~2 min).
B3. Dashboards: `python -m http.server --directory "$HUSKY_BENCH_ROOT" 8000`, open
    `http://localhost:8000/runs/smoke_linux/dashboard/` and `http://localhost:8000/prm/<run>/dashboard/` in
    headless Chrome via CDP (the fake-lab driver in the scratchpad) or the Chrome tools; check the page renders
    (robot view, charts, no console errors). `node benchmark/tools/check_modules.mjs <dashboard dir>` and
    `check_charts.mjs <summary.json>` for the self-checks. Note: tamp's export message says to serve the
    dashboard folder; the data root is right (the 3D view reads `../../../assets/`): fix that message (one line).
B4. Write the Linux recipe down: a short "Running here" section in tamp's `benchmark/README.md` (venv, env vars,
    the three commands, the mock root) — not a new doc.

## Part C: the planner dropdown in the monitor

Carrier: two fields on the existing `FreeSearch` (`shared/assembly_planning/route_search.py:50`), which already
travels `MotionPlanRequest.search` → `run_plan` → `plan_movement` → `Route.search`: zero new plumbing. The
process plugin, `ProcessContract`, `basis.py` (staleness) and `servo_route.py` (the servo's sampling fallback
stays BiRRT: a short correction near the insert start, outside the roadmap's carry poses) stay untouched.

C1. Settings, `shared/runs/runs.py`: `CDFM_PLANNERS = ("birrt", "prm")` next to the `PLAN_*_RANGE` constants (no
    tamp import in the runs layer); `RunSettings.plan_cdfm_planner: str = "birrt"` and
    `plan_roadmap_root: str = ""` (docstring: the held transfer's planner; the folder holding
    `prm/<process folder name>__<bar>/`, empty = `<drive_root>/data_experiment/cdfm_planner_benchmark`).
    `read_run` drops unknown names and `asdict` saves them: older runs read the defaults.
    `plugins/assembly_run/writes.py` `_settings_problem` (:354): `CHOICE_SETTINGS = {"plan_cdfm_planner":
    ("the CDFM planner", CDFM_PLANNERS)}` beside `NUMBER_SETTINGS`; one branch before the bool one for
    `SETTING_TYPES[name] is str`: refuse a non-str ("… must be text, not …") and a choice outside its tuple
    ("the CDFM planner must be one of birrt, prm, not …"). `request_settings` already converts with the type.
C2. Panel, `plugins/assembly_steps/panel.py` Advanced planning folder after Swept check (:112):
    `self._plan_cdfm = gui.add_dropdown("CDFM planner", options=list(CDFM_PLANNERS), initial_value=…, hint=
    "How a held transfer is planned (Transfer workaround off): birrt searches; prm follows a precomputed roadmap
    of this process and bar, one try, about 1–2 min, no time limit")` and `self._plan_roadmap = gui.add_text(
    "Roadmap root", initial_value=…, hint="Folder holding prm/<process>__<bar>/; empty: the Drive's
    data_experiment/cdfm_planner_benchmark")`. Both always present (nothing jumps). Add both to the `settings`
    tuple (:135, kind `str`), writing only when the value differs from the run's (the text box calls back per
    edit). `draw` (:196): write both back inside the `settings != self._shown_settings` block.
C3. Thread it, `plugins/assembly_steps/plugin.py` `free_search()` (:339): `root = settings.plan_roadmap_root or
    (str(ctx.config.drive_root / PRM_BENCH) if ctx.config.drive_root else "")`; `FreeSearch(…,
    cdfm_planner=settings.plan_cdfm_planner, roadmap_root=root)`. Resolving the Drive convention on the main
    thread keeps `drive_root` off the worker. Time limit (:545): unchanged, `ATTEMPTS * HELD["max_time"] +
    margin` (420 s+) covers the PRM's one try of 60–130 s; say so in its comment.
C4. Route, `shared/assembly_planning/tamp_route.py`:
    - constants `PRM_BENCH = Path("data_experiment/cdfm_planner_benchmark")`, `PRM_OPTIONS = {"workers": 1,
      "save_query": False, "verbose": False}` with a `!` comment: never write into the Drive (tamp writes
      `scenes/` and `queries/` only with `workers > 1` or `save_query`).
    - `Route.roadmap_dir() -> Path`: `Refused("no roadmap root: set drive_root or Advanced planning's Roadmap
      root")` when empty; else `Path(root) / "prm" / f"{self.process.folder.name}__{bar name}"` (`spec.bar` is
      "bars/B21"; the precompute script names runs `<problem>__<bar>`).
    - `held_search(goal, seed, targets, backend="birrt")` (:464): for "prm", refuse when
      `roadmap_dir / "roadmap.json.gz"` is missing: `Refused(f"no roadmap for {bar} at {folder}: build it with
      tamp's scripts/prm_runs/process_precompute.sh <process folder> {bar}")`; else call
      `plan_constrained_dual_arm(…, **HELD, backend="prm", prm_options={"roadmap_dir": str(folder),
      **PRM_OPTIONS})` in `try/except ValueError as mismatch: raise Refused(f"{self.planner}: {mismatch}")`
      (`RoadmapMismatch` and the "not attached to left tool0" error are `ValueError`s: no new tamp import);
      sync the mirror in a `finally`.
    - `_transfer_held` (:295): `prm = self.search.cdfm_planner == "prm"`; `self.planner =
      "tamp.plan_constrained_dual_arm" + ("/prm" if prm else "")`; `tries = 1 if prm else ATTEMPTS`; the note
      says "roadmap, no time limit" for prm instead of max_time/seed; the final `Refused` text uses `tries`.
      `route.planner` is already recorded in the plan record and the log.
C5. Docs: `doc/lab/plugins.md` assembly_steps paragraph (Advanced planning: CDFM planner, Roadmap root, nothing
    written there); `doc/glossary.md` one entry (CDFM planner / roadmap); `README.md` one clause in the Drive
    paragraph (roadmaps read from `data_experiment/cdfm_planner_benchmark/`); the Transfer-workaround memory
    note gets a line that CDFM now has two planners (the workaround stays default).
C6. Tests (mirroring the existing ones):
    - `test/shared/test_runs.py` (:78): round trip of the two fields through `new_run`/`read_run`; an old
      run.json without them reads the defaults.
    - `test/plugins/assembly_run/test_assembly_run.py` (:994 settings check): `plan_cdfm_planner="astar"` and
      `plan_roadmap_root=3` refused with their texts.
    - `test/plugins/assembly_steps/test_assembly_steps.py` (:314 advanced settings reach the request):
      `request_settings(plan_cdfm_planner="prm", plan_roadmap_root="/roadmaps")` → `request.search.cdfm_planner
      == "prm"`, `roadmap_root == "/roadmaps"`; empty root + `drive_root=Path("/drive")` → the convention; the
      dropdown and text show the values back.
    - `test/shared/test_planning_tamp_route.py` (:353 search settings reach tamp), with a fake
      `tamp_route.plan_constrained_dual_arm` returning `[start, goal]`: prm with the roadmap file made under
      `tmp_path` → one call with `backend="prm"` and the exact `prm_options`, `planner` ends in `/prm`, one
      attempt; file missing → never called, failed with "no roadmap for … process_precompute.sh"; the fake
      raising `ValueError("roadmap built for bars/B21")` → failed with that text; empty root → "no roadmap
      root"; birrt → the 3-try behaviour unchanged.

## Verification

- tamp: `python -m pytest benchmark/tests -q` with `HUSKY_BENCH_ROOT` = mock (pure + smoke groups), Part B runs.
- monitor, while iterating: `python3 -m pytest -n 0 test/shared/assembly_planning test/plugins/assembly_steps
  test/plugins/assembly_run test/plugins/process`; before the commit: `python3 -m pytest` (default set, incl.
  linters and `test/structure/test_layers.py`); `pytest -m safety` before any lab use.
- fake lab (user's tmux `fake_lab` untouched; my own session per the memory note): load
  `261008_2ModulesDemo_purged_process`, untick Transfer workaround, B21: plan with BiRRT (expect a path or a
  3×120 s refusal) and with PRM (roadmap root empty → Drive convention; expect a path in ~1–2 min, the planner
  name in the panel/log, no files written under the Drive's `prm/…__B21/`), then PRM on B20 (no roadmap: the
  refusal names the folder and the build command). Scrub the preview; Exec in the sim.
