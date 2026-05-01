# Software Reorganization for Husky Assembly Mega-Repo (Journal-Version Target)

## Context

We are extending the robarch2026 paper (`doc/robarch2026_robotic_scaffolding_v1.pdf`) into a journal version that adds the multi-agent and chained-MP story sketched in the ITJ papers (`supporting_materials/papers/2025_CAAD_validation.pdf`, `Timber_Assembly_SCF2021.pdf`). The system has accreted into three components developed on different OSs, and the seams are now leaky:

- **A. Rhino design + IK keyframing** (`external/bar_joint_rhino_design_workflow`, Windows): canonical author of bars, joints, grasp poses (`joint_ocf_to_tool0` for Robotiq + Victor's tool), RobotCell, RobotCellState. Replays trajectories for visualization.
- **B. Constrained dual-arm planner** (`external/husky_assembly_tamp`, Linux): hybrid pybullet + Drake. Currently only a `__main__` CLI in `plan_generator.py`. Drake-side constrained bimanual planner already lives behind `docker/constrained_bimanual/planner_server.py` with `scene_exporter.py` as the pybullet→Drake bridge.
- **C. Monitor / control / execute** (this repo, Linux + ROS2 + TracIK + pybullet): replans online via `husky_planning.plan_arm_motion` and friends; consumes design data through hardcoded paths in `common.py`.

Concrete pain points (confirmed by audit):
- 128+ mesh files duplicated MD5-identical between `data/husky_urdf/` and `external/bar_joint_rhino_design_workflow/asset/husky_urdf/`.
- `HUSKY_UR5e_JOINT_NAMES` and `parse_mt_geometric` defined / imported redundantly in C.
- `husky_assembly_teleop/design_interface/` is dead code — the intended A→C bridge is unused; A→C handoff is manual file copy.
- B has no importable Python API; C re-plans rather than consuming B's output, and there is no path today for ITJ-style chained MP to feed Rhino replay.

**Timing**: this refactor is the *post-hardware-tests* target for the journal version. The three hardware tests (RS485+LM replanner, dual-arm planner in monitor, mocap2urdf bar reach) run on the current layout first.

**Intended outcome**: single source-of-truth for URDFs and shared data; one shared planner library used by both B's offline chained-MP CLI and C's online ROS2 replanner; COMPAS FAB types as the lingua franca between A, B, and C; B and C's collision/env state managed via a single pybullet world, with Drake reachable only behind the existing planner_server.

## Target submodule layout

```
bar_joint_rhino_design_workflow (A, Windows + Rhino)        [standalone]
├── data/husky_urdf            (submodule)
└── data/husky_design_study    (submodule)

husky-assembly-teleop (C, Linux + ROS2)                     [this repo]
└── external/husky_assembly_tamp (B, Linux)
    ├── data/husky_urdf            (submodule)
    └── data/husky_design_study    (submodule)
```

Properties:
- A is not a submodule of B or C; it's a peer.
- `husky_urdf` and `husky_design_study` are pinned independently in A and in B (parallel branches off the same upstream).
- C's only deep submodule is B; A and B reach the data submodules at depth 1.
- A's `asset/husky_urdf/` duplicate folder is deleted; A's Rhino code reads from `data/husky_urdf`.

Known cost: A's pin and B's pin of `husky_urdf` (and of `design_study`) can drift. Mitigated by a CI gate on `main` that fails if the SHAs diverge.

## Data contract: COMPAS FAB types as lingua franca

- A exports `RobotCell`, `RobotCellState`, `JointTrajectory`, and a thin `Movement` wrapper (per ITJ chained-MP style) as JSON files into `husky_design_study/`.
- B reads RobotCell + RobotCellState from `husky_design_study`, computes plans for each Movement in the action skeleton, writes `JointTrajectory` + `Movement` results back to `husky_design_study`.
- A reads B's trajectories for Rhino replay (validation + visualization).
- C reads RobotCellState (and optionally B's trajectory for sim2real overlay) at execution time; calls into the shared planner library for online replanning per Movement.

Existing scaffolding to reuse:
- `husky_assembly_teleop/design_interface/load_grasp_targets.py` — `GraspTarget` parser of A-exported grasp JSONs (currently dead; resurrect).
- `husky_assembly_teleop/design_interface/conversions.py` — COMPAS Frame ↔ pybullet pose tuples (currently dead; resurrect).
- `husky_assembly_teleop/design_interface/reconstruct_state_from_json.py` — already exists; wire into the load path.
- `external/compas_fab/` (already a submodule) — host of the canonical types.

## Planner architecture: shared `husky_planner` lib + Drake server

Promote planning code in B into a single Python package:

```
external/husky_assembly_tamp/husky_assembly_tamp/husky_planner/
├── world.py               # pybullet collision/scene; SINGLE source of truth
├── free_motion.py         # BiRRT via pybullet_planning (consolidates B's plan_generator + C's plan_arm_motion)
├── dual_arm_constrained.py# client of Drake planner_server; uses scene_exporter to ship pybullet world
├── ik.py                  # TracIK wrapper (Linux-only); shared with C
└── api.py                 # public entrypoints: plan_movement(movement, world, ...) -> JointTrajectory
```

- **Free motion**: pure pybullet_planning BiRRT. Used by both B's chained-MP CLI and C's online replanner.
- **Constrained dual-arm**: *client* of the existing Drake `planner_server` in `docker/constrained_bimanual/planner_server.py`. Both B's CLI and C are clients; neither imports `pydrake` directly. `scene_exporter.py` becomes the shared bridge utility.
- **Collision / env source of truth**: a single pybullet world owned by the caller (B or C). When the constrained planner is invoked, the caller's pybullet snapshot is shipped to Drake via `scene_exporter`. Same code path online (C) and offline (B).
- **B's CLI**: `plan_generator.py` is refactored into an orchestrator that loops over Movements in an action skeleton, calls `husky_planner.api.plan_movement` for each, and writes JointTrajectory results to `husky_design_study`. CLI entry plus library import — same internals.
- **C's online use**: `husky_planning.py` body is deleted and replaced with a thin wrapper that imports `husky_planner.api`. C continues to own its pybullet world (started by `husky_world.py`); it passes that world into the planner library calls.

## Migration steps (priority order; assume hardware tests are complete)

### Phase 1 — Submodule consolidation
1. Add `data/husky_urdf` as a submodule of A; delete `external/bar_joint_rhino_design_workflow/asset/husky_urdf/`. Refactor A's Rhino/Grasshopper scripts (`scripts/core/*.py`, `support_materials/gh_keyframe_demos/python/*.py`) to read from the new path.
2. Promote `data/husky_assembly_design_study/` to a submodule of A as well; ensure both A and B point to the same upstream `husky_design_study` repo.
3. Add a CI gate (workflow file in C and in A) that fails if A's `husky_urdf` SHA and B's `husky_urdf` SHA do not match on `main`. Same for `husky_design_study`.

### Phase 2 — COMPAS FAB schema as load path in C
4. Resurrect `husky_assembly_teleop/design_interface/`: import it from `husky_world.py` and `husky_monitor.py`. Replace the hardcoded `DESIGN_DATA_DIRECTORY` / `VALIDATION_PROBLEM_NAME` reads in `common.py:216-254` with calls into `design_interface`.
5. Stop redefining grasp poses in C: derive `create_end_effector`'s grasp transforms from A's exported `RobotCell.json` (loaded via `design_interface.load_grasp_targets.GraspTarget`). Keep tool URDF generation in `common.py` only for visualization meshes; geometry source-of-truth is A.

### Phase 3 — Extract `husky_planner` library
6. Create `external/husky_assembly_tamp/husky_assembly_tamp/husky_planner/` with `world.py`, `free_motion.py`, `dual_arm_constrained.py`, `ik.py`, `api.py`. Move BiRRT and TracIK code currently in C's `husky_planning.py:56-234` and B's `motion_planner/` into here. Single canonical implementation; both B's CLI and C call it.
7. Refactor B's `plan_generator.py` from a `__main__` script into a chained-MP orchestrator that imports `husky_planner.api` and writes results to `husky_design_study`.
8. Delete the body of C's `husky_planning.py`; replace with thin re-exports / wrappers around `husky_planner.api`.
9. Promote `docker/constrained_bimanual/scene_exporter.py` to `husky_planner/` so it's importable as a library utility (not just a docker-internal script). The Drake `planner_server.py` itself stays put.

### Phase 4 — Cleanup
10. Delete duplicate `HUSKY_UR5e_JOINT_NAMES` and friends from `husky_assembly_teleop/utils.py:29-48`; keep `common.py:24-42` as canonical.
11. De-duplicate `parse_mt_geometric` imports in C; route through one `design_interface` entry point.
12. Archive `external/husky_assembly_tamp/husky_assembly_tamp/motion_planner/zh_archive/` (it's superseded by `husky_planner/`).
13. Update READMEs in C, B, and A to document the new layout, the data flow through `husky_design_study`, and the cross-platform setup.

## Critical files

| Path | Role | Change |
|---|---|---|
| `external/bar_joint_rhino_design_workflow/asset/husky_urdf/` | duplicate (~150MB) | **delete**; replace with submodule at `data/husky_urdf` |
| `external/bar_joint_rhino_design_workflow/scripts/core/*.py` | hardcoded `asset/...` paths | refactor to read `data/husky_urdf` |
| `external/bar_joint_rhino_design_workflow/support_materials/gh_keyframe_demos/python/GH_export_cell_state_traj.py` | RobotCellState exporter | confirm output lands in `data/husky_design_study` |
| `external/husky_assembly_tamp/husky_assembly_tamp/plan_generator.py` | `__main__` planner | refactor into library + chained-MP CLI orchestrator |
| `external/husky_assembly_tamp/husky_assembly_tamp/husky_planner/` | **new package** | hosts `world.py`, `free_motion.py`, `dual_arm_constrained.py`, `ik.py`, `api.py` |
| `external/husky_assembly_tamp/docker/constrained_bimanual/planner_server.py` | Drake server | stays; client moved into `husky_planner/dual_arm_constrained.py` |
| `external/husky_assembly_tamp/docker/constrained_bimanual/scene_exporter.py` | pybullet→Drake bridge | promote to `husky_planner/` (library use) |
| `husky_assembly_teleop/design_interface/load_grasp_targets.py` | currently dead | **resurrect**; sole grasp loader for C |
| `husky_assembly_teleop/design_interface/conversions.py` | currently dead | **resurrect**; sole COMPAS↔pybullet pose conversion |
| `husky_assembly_teleop/design_interface/reconstruct_state_from_json.py` | currently dead | **resurrect**; RobotCellState loader |
| `husky_assembly_teleop/husky_planning.py` | C's planner | replace body with wrapper around `husky_planner.api` |
| `husky_assembly_teleop/common.py` | canonical robot/EE definitions | drop grasp re-definitions; keep robot-class scaffolding only |
| `husky_assembly_teleop/utils.py:29-48` | duplicate joint-name constants | delete; import from `common.py` |
| `husky_assembly_teleop/husky_world.py` | scene + planner relay | route through `design_interface` and `husky_planner` |

## Verification

End-to-end checks after each phase:

- **After Phase 1**: fresh clone of C with `--recurse-submodules` brings `husky_urdf` and `design_study` at depth 2 (under B). Fresh clone of A brings them at depth 1. `md5sum` on a representative mesh file matches across both clones. Rhino on Windows loads URDFs from the new path. CI submodule-SHA gate triggers on a deliberately-stale PR.
- **After Phase 2**: launch C against a known A-exported `RobotCell.json` from `husky_design_study`. Verify in pybullet GUI that grasp pose, robot configuration, and tool offsets match the pre-refactor baseline within numerical tolerance. No hardcoded paths remain in `common.py`.
- **After Phase 3**: run B's chained-MP CLI on a small action skeleton (e.g. one Movement) in `husky_design_study`; verify the trajectory file written matches a pre-refactor baseline trajectory. Then run C's online replanner on the same scene; verify per-segment plan time and final config are within tolerance of B's offline result. Run the constrained dual-arm planner from both B's CLI and from C's monitor; verify same Drake server response in both cases (planner_server logs identical request).
- **After Phase 4**: re-run the most representative hardware test that was completed pre-refactor (likely Test 2 — dual-arm planner in monitor). Compare end-to-end execution time, plan success rate, and physical assembly outcome against the pre-refactor run.
- **Smoke test on a clean machine**: clone C fresh, run `pip install -e .` in C, in B, and in `husky_planner`. Run the test suite in `external/husky_assembly_tamp/husky_assembly_tamp/test/` and `husky_assembly_teleop/design_interface/tests/`. Both must pass.

## Open items deferred from this plan

- Whether A's `joint_ocf_to_tool0` and friends are pure Python or Rhino-bound. If pure Python, a future iteration could extract them into a small `bar_joint_core` package importable on Linux; until then, JSON via `design_study` is sufficient.
- Whether `husky_design_study` being git-versioned scales for per-run trajectory outputs (commit-spam concern). May need a hybrid: source-of-truth design state in git, per-run planner outputs in a non-versioned data dir keyed by planner SHA.
- ROS2 message wrapping of `Movement` for live trajectory streaming (currently `husky_world.py` calls `send_arm_cmd` directly with `np.ndarray` configs).
- Whether the Drake `planner_server` should run inside the existing docker container on the robot laptop, or be lifted to a beefier offline machine for chained-MP runs.
