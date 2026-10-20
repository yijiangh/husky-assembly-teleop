# Process data in the core, the process plugin's code in the plugin (2026-10-19)

Status: **done 19 Oct, not pushed**: core 0b4a36f (on core main after the other session's 7e1a68a), monitor in
the commit after this note. All tests pass on both sides; Rhino's core tests (jg/rhino-new-core) pass with the new core.

## Why

`shared/loaded_process.py` mixed three things: process logic every consumer needs (a movement's joints with the
assumed start, the scene at a movement's end), the process plugin's own code (loading, mapping to the live scene,
colours), and two wrapper types (`Step`, `LoadedProcess`) that mostly re-bundle the core's `Process`. Rhino's
`plan_steps.py` re-implements the same generic pieces (`step_scene`, `find_movement`, `movement_after`).

## Decisions (user, 19 Oct)

- **Core** (`external/scaffolding_assembly_core`), additions only, no format, plan-check or schema change:
  - `scene_at(process, movement, at_end=False)` and `Process.scene_at(movement, at_end=False)`: `at_end` applies
    the movement's target joints (what `scene_after` already does for the last movement).
  - `carry.joints_at(process, action_id, movement_id, at_end=False, assumed=None)`: robot id -> joints of every robot
    in the movement's start state; the acting robot's assumed start fills in `joints: null`; `at_end` applies the
    target's joints. `assumed` takes a precomputed `assumed_start_all`.
  - `Process.find_movement(action_id, movement_id)` and `Process.movement_after(movement)`, for Rhino's
    `plan_steps.py` (which keeps its own copies until it adopts them).
- **Monitor**: `shared/loaded_process.py` goes.
  - `Step`, `LoadedProcess`: gone. `Selection` (`shared/process_contract.py`) holds the core's `Process`, the
    selected `Action` and `Movement`, its index and the id maps.
  - `displayed_joints` -> the core's `joints_at`; `stands` -> a private helper in `process/drawing.py` (one user).
  - `load_process` -> `plugins/process/loading.py`; `live_ids`, `scene_bodies` -> `plugins/process/scene_bodies.py`;
    colours -> `plugins/process/colors.py` (drawing and scene bodies both use them); navigation (the movement list,
    action counts) private to the plugin.
- **Review from Rhino's side** (19 Oct): nothing breaks. Applied: `joints_at(at_end)` leaves out robots absent from
  the start (as `scene_at`), docstrings on the assumed view vs `scene_at`'s `unmeasured`, identity matching in
  `movement_after`. Rhino's `step_scene` equals `scene_at(at_end=True)` except that Rhino revives a robot the target
  names but the start leaves out.

## Order

1. Core change, its tests (incl. the Python 3.9 import test), a review from Rhino's side, Rhino's core tests on
   `jg/rhino-new-core`. Commit in the core.
2. Bump the core pointer here; the monitor change; `pytest -m ""`. Commit.
3. Push the core first, then this repo (ask before pushing).

## Not in this task

- Moving Rhino's `plan_steps.py` onto the new core helpers (another repo and branch).
