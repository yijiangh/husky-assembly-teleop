# M2: planning foundations in `shared/planning/` (spec)

Part of the core-port plan (milestone M2). These modules are the groundwork every assembly planner and executor
(M3+) builds on:
- movement classification and the drives the monitor really sends;
- operator steps;
- the planning scene on the real robots;
- exact ssik IK at the live base;
- the collision extras;
- joint unwrapping and limits.

Everything is pure, or runs on one `CompasFabMirror` on the planner's worker thread. Nothing keys on class names or ids.

**Abbreviations:**
- Old code is the lab branch at **0e45758**: M = `husky_monitor.py`, W = `husky_world.py`, CFS = `cfab_session.py`,
  BIO = `bar_action_io.py`.
- R = this repo, `husky_assembly_teleop/`.
- C = the core, `external/scaffolding_assembly_core/scaffolding_assembly_core` (c985034).

**Decisions taken:**
- **Tool vs other robots' tools (old CC.6): fixed in the core**, as a separate task (task C below), under the core's
  AGENTS.md review rule. `collision.py` does not rebind compas_fab's `check_collision`.
- **compas_fab's `forward_kinematics` double base transform** (pinned compas_fab 995122d,
  `pybullet_forward_kinematics.py:99–102`): it lives in compas_fab, not our repos, so it is fixed locally with a context
  manager (`world_frame_fk`).
- **"Not measured" robot:** all robots are live in one run (Zenoh), so this means a configured robot whose joints or
  base are not measured right now (`RobotObject.acting_problems()`, C/robot.py:157). A process robot that is not
  configured is left out of the planning scene, with a warning.

Rules for every module:
- Google docstrings with typed signatures, `from x import y` at the top, plain-word comments.
- List the module in `shared/planning/__init__.py`.
- Never import `plugins/` or `panel.py` (it imports viser).
- `test/test_layers.py` stays green.

## Modules

### `joints.py` (~80 lines)

| Function | Behaviour | Port |
|---|---|---|
| `unwrap_to_nearest(goal, start, lower=None, upper=None) -> np.ndarray` | Shifts each goal joint by whole turns to within ±π of its start. With limits, picks the in-range equivalent nearest the start (checks k ∈ {0, ±1, ±2} turns, slack 1e-9). A joint with no in-range equivalent keeps its ±π value. | W:3098 |
| `joint_limits(urdf: Path, names) -> tuple[np.ndarray, np.ndarray]` | URDF lower/upper from `load_urdf(urdf).joint_map[n].limit` (C/urdf.py:50); continuous joints get ±inf. | W:3138; URDF, not PyBullet |
| `outside_limits(labelled, names, lower, upper) -> list[str]` | One `"<label> <joint>=<v> rad (limits [lo, hi])"` per joint outside its limits (tolerance 1e-6). | M:7081 |
| `arm_joint_names(arm) -> list[str]` | `f"{arm}_{n}"` for `UR_JOINT_NAMES` (C/urdf.py:374). | Moved here from `plugins/arm_planner/planner.py:29`, which then imports it (one-line edit). |
| `arm_side(arm) -> str` | `"left"`, `"right"`, else `"arm"`. | Like `config.py:298` |

### `movement.py` (~230 lines)

**Types:**
- `Routine(str, Enum)`: FREE_TO_LOAD, TRANSFER_HELD, TRANSFER_FREE, INSERT, RETREAT_LINEAR, FREE_HOME, SUPPORT_APPROACH,
  SUPPORT_LINEAR, SUPPORT_LEAVE, TOOL_ONLY, OPERATOR, BASE_ONLY.
- `GoalSource(str, Enum)`: LINK_POSES, LINE, JOINTS, NEXT_START, STOW, NONE.
- Frozen `ArmPart`: `flange`, `arm`, `side`, `joints` (6 names), `line: LineSpec | None`, `target: Pose | None`. Built with
  `split_link_id` (C/ids.py:277) and `FLANGE_SUFFIX` (C/robot.py:21).
- Frozen `MatingDrives`: `kept: dict[str, str]`, `dropped: dict[str, str]` (tool → reason), `grounded: bool`.
- Frozen `MovementSpec`: `action`, `movement`, `action_type`, `robot`, `bar`, `routine`, `arms`, `coupled`, `controller`,
  `ends_on`, `goal`, `goal_joints`, `goal_from`, `grips`, `drives: MatingDrives`, `warnings`.
- `UnknownMovement(ValueError)`: its message lists the parts.

**Functions:**
- `classify(process, action, movement, transfer_as_free=False) -> MovementSpec`
- `classify_action(process, action, transfer_as_free=False) -> tuple[MovementSpec, ...]`
- `next_start(process, action, movement) -> tuple[dict | None, str]`: walks forward in schedule order.
  - It skips later movements of the same action where no arm moves and the acting robot has no joints.
  - It returns the first start that gives the acting robot's joints.
  - It returns `(None, <id>)` at an arm movement whose start joints are null, or when the robot is absent.
- `mating_drives(process, movement) -> MatingDrives`

**Classification (first match wins).** Parts come from `Movement` (C/process/types.py:260) and `Target`
(C/process/types.py:198). The action type is only a cross-check: a mismatch goes into `warnings`, it never changes the
routine.

| # | Parts | Routine | Goal |
|---|---|---|---|
| 1 | `ends_on == "operator"`, no arms | OPERATOR | NONE |
| 2 | `base` set, no arms | BASE_ONLY | NONE |
| 3 | no arms, a tool part | TOOL_ONLY | NONE |
| 4 | linear, coupled, and (a tighten drive, or `target.built` adds `action.bar`) | INSERT | LINE |
| 5 | linear, uncoupled, ≥2 arms | RETREAT_LINEAR | LINE |
| 6 | linear, uncoupled, 1 arm | SUPPORT_LINEAR | LINE |
| 7 | free, coupled | TRANSFER_HELD, or TRANSFER_FREE when `transfer_as_free` | LINK_POSES |
| 8 | free, uncoupled, ≥2 arms, target null | FREE_TO_LOAD | NEXT_START |
| 9 | free, uncoupled, ≥2 arms, target joints and no link_poses | FREE_HOME | JOINTS |
| 10 | free, uncoupled, 1 arm, target has link_poses | SUPPORT_APPROACH | LINK_POSES |
| 11 | free, uncoupled, 1 arm, otherwise | SUPPORT_LEAVE | JOINTS, else NEXT_START, else STOW |
| — | anything else | raises `UnknownMovement` | |

**`mating_drives`** ports BIO:854 and BIO:885 onto the process mates instead of body names.

A **tighten** on tool T is kept only if all of these hold, with `on = start.tools[T].on`:
- `on` is a `joints/` half;
- a mate `(on, other)` exists whose `other` is a `joints/` half;
- `bar_of(other)` (C/process/relations.py:231) is in `start.built` and `start.present`.

Otherwise the tighten is dropped, with one of these reasons:
- "ground half": the half is mated to `ground/`; this also sets `grounded=True`;
- "no mate (one-sided bar)";
- "receiver not built";
- "tool on nothing".

As a diagnostic only, log a warning when `bodies[on].part` lacks "Male".

A **loosen** is kept only if `end_state(movement).built` (C/process/plan_check.py:133) loses the bar of `on`. Otherwise
it is dropped with "loosen on release: never sent".

An export whose drives are already correct passes through unchanged.

### `steps.py` (~90 lines)

- Frozen `OperatorStep`: `primary: int`, `with_motion: tuple[int, ...] = ()`, `in_pause: tuple[int, ...] = ()`, and a
  property `indices` (all of them, in execution order).
- `operator_steps(process, action, transfer_as_free=False) -> list[OperatorStep]` (port of BIO:740):
  - **Ungrasp.** A TOOL_ONLY movement whose grips all go "open", directly followed by a RETREAT_LINEAR, becomes that
    retreat's `with_motion`.
  - **Mount pause.** With `transfer_as_free`, the run of OPERATOR / TOOL_ONLY movements directly before a TRANSFER_FREE
    becomes its `in_pause` (old W:5852–5912).
  - **Everything else** is its own step. Every index appears exactly once.
- `step_index_of(steps, movement_index) -> int | None` (BIO:780).

### `scene_for.py` (~200 lines)

**Types:**
- Frozen `RobotBelief`: `base: Pose | None = None`, `joints: Mapping[str, float] | None = None`.
- Frozen `PlanningScene`: `scene: Scene`, `acting: str` (live id), `ids: IdMap`, `process_ids: IdMap`,
  `sources: dict[str, str]`, `warnings: tuple[str, ...]`.

**Signature:**
```python
scene_for(loaded: LoadedProcess, step: Step, snapshot: Scene, robots: tuple[RobotConfig, ...], *,
          prefix="process/", beliefs: Mapping[str, RobotBelief] | None = None,
          hide_held=False, ignore_built=False, keep=("obstacles/",)) -> PlanningScene
```

It is pure: take the snapshot on the main thread, run this on the worker.

**Steps:**
1. `ids, process_ids = live_ids(process, robots, prefix)` (R/shared/process.py:210). ValueError if the acting robot does
   not map or is not in the snapshot.
2. Make a new `Scene`. Robots are `.copy()` of the snapshot's robots. Bodies are copies of the snapshot bodies whose id
   starts with something in `keep`.
3. Put `scene_bodies(loaded, step, robots, prefix)` (R/shared/process.py:244), so the ids and Geometry objects are the
   process plugin's and the mirror never rebuilds.
4. For each non-acting configured robot with `acting_problems()`, field by field:
   - base: live if tracked, else the belief, else the design base;
   - unmeasured joints: the belief, else the design joints;
   - "design" means `scene_at(step.movement).robots[process_ids(id)]`;
   - a known design base with null joints uses `stow_joints` (R/world/kinematics.py:171).

   Apply the fill with `seeded` (C/process/scenes.py:185), but only when every unknown gets a value. Otherwise the robot
   is "parked", with a warning. Sources are one of: live, belief, design, design (stowed arms), parked.
5. Disable bodies attached to a parked robot (warning).
6. `hide_held`: disable bodies whose placement is an `Attachment` to the acting robot.
7. `ignore_built`: disable the bars in `start.built` and their halves (`halves_of`, C/process/types.py:342), except those
   held by the acting robot (old M:3739).
8. `scene.world_poses = world_poses(...)` (C/scene.py:135, C/kinematics.py:90). Remove any body with no world pose, with a
   warning; otherwise `CompasFabCell.state(strict)` raises KeyError (C/mirrors/cell.py:212).

### `collision.py` (~120 lines, after the core fix)

| API | Behaviour | Port |
|---|---|---|
| `@contextmanager world_frame_fk(planner)` | Instance-level `forward_kinematics` without the second base transform. Restores the method on exit. | CFS:702–760 |
| `collisions(mirror, joints=None, full_report=False) -> list[tuple[str, str]]` | `mirror.collisions(...)`, whose pairs include tool against other tool once task C is in. | new |
| Frozen `SweptReport` | `ok`, `samples`, `bad_segments`, `error` | |
| `swept_check(mirror, joint_names, path, step=0.01) -> SweptReport` | Checks the interior samples of each segment with `mirror.search_check`, and names the pairs once per segment with `collisions(full_report=True)`. Any exception fails closed. | M:6932–7079 |
| `in_process_terms(pair, process_ids) -> tuple[str, str]` | Each side through `IdMap.get` (C/ids.py:321); unmapped sides stay as they are. | new |

### `ik.py` (~200 lines)

**Formulas:**
- `A_s = link_pose(urdf, Pose(), {}, f"{arm}_base_link")`
- `F_s(q) = link_pose(urdf, B, q, f"{arm}_tool0")`
- ssik input `T_s = (B·A_s)⁻¹·F*_s`

All are computed from the model URDF, with no PyBullet.

**Constants:** `SSIK_SOLVERS = {"left_ur_arm": ("left",), "right_ur_arm": ("right",), "ur_arm": ("alice", "belle")}`,
`MAX_BRANCHES = 8`, `FIT_TOL = 1e-6`. `FIT_JOINTS` = zeros, all 0.5, and `[1.0, -1.2, 1.4, -0.8, 0.6, -0.4]` (W:3477).

**Functions and types:**
- `ssik_unavailable() -> str | None` (W:3508–3537).
- `@lru_cache fitting_solver(urdf, arm) -> str | None`: the first solver whose `ssik_inprocess.fk` matches `inv(A_s)·FK`
  within FIT_TOL at all three joint sets (W:3448). Measured: 0804 fits alice, 0805 fits belle, 0806 fits left/right.
- `branches(urdf, base, arm, solver, target, reference) -> list[np.ndarray]`:
  `ssik_inprocess.solve(solver, T, max_solutions=8, q_seed=ref)`, then `rebranch_toward_seed` (tamp
  `motion_planner/ssik_ik.py:123`), sorted by L∞ to the reference. Not tamp's `ssik_arm_branches`, which reads PyBullet.
- Frozen `IkResult`: `joints`, `max_change`, `rank`, `combinations`, `backend`, `solvers`, `colliding`,
  `colliding_joints`, `reason`.
- `solve_ik(mirror, robot, targets, reference=None, check_collision=True, log=print) -> IkResult` (W:3540–3613):
  - Takes the product of the per-arm branches, sorted by the largest move.
  - The first combination with no collisions wins.
  - If all collide, returns `joints=None` with rank 1's colliding pair.
- **PyBullet fallback**, when there is no fitting solver or ssik is unavailable:
  - log `"! IK falls back to PyBullet …"` once per (urdf, arm);
  - inside `mirror.lend()` with `world_frame_fk`, run `inverse_kinematics` per planning group (W:3616–3749).
- **Threading:** worker only, after `mirror.sync(scene)`. Sync again after IK before building a search check.

## Tasks

| Task | Content | Depends on |
|---|---|---|
| A | `joints.py` (+ the import in `arm_planner/planner.py`), `test_planning_joints.py` | — |
| B | `movement.py`, `steps.py`, `test_planning_movement.py`, `test_planning_steps.py` | — |
| C | **Core:** tool vs other tools (CC.6) in `CompasFabMirror.collisions` and `SearchCheck`, bound in `_connect` so `lend()` borrowers get it. Branch in the core repo, its own tests, a consumer-side review (Rhino IK hold scenes!), Rhino consumer tests, then a pointer bump here. | — |
| D | `scene_for.py`, `test_planning_scene_for.py` | — |
| E | `collision.py`, `ik.py`, `test_planning_collision.py`, `test_planning_ik.py` | A, C |
| F | `__init__` docstring, full test run | all |

## Tests (assertions)

**Fixtures:**
- `purged` = `load_process(drive_root() / "data_design_study/261008_2ModulesDemo_purged_process", DATA)`; skipped when
  missing (`HUSKY_DRIVE_ROOT`).
- Configs come from `robot_config_from_serial` (as in `test/test_compas_fab_mirror.py:32`).

**movement:**
- **Golden table.** All 70 movements of the 18 actions B20–B37 classify. The routines per action type are:
  - J = (FREE_TO_LOAD, OPERATOR, TOOL_ONLY, TRANSFER_HELD, INSERT)
  - R = (TOOL_ONLY, RETREAT_LINEAR, FREE_HOME)
  - H = (SUPPORT_APPROACH, TOOL_ONLY, SUPPORT_LINEAR, TOOL_ONLY)
  - HR = (TOOL_ONLY, SUPPORT_LINEAR, SUPPORT_LEAVE)
- **Workaround.** `transfer_as_free=True` changes only J_transfer (to TRANSFER_FREE). Id suffixes are cross-checked in
  the test only.
- **Goals:** B20_J_load is NEXT_START (joints None); J_transfer is LINK_POSES; J_insert is LINE; R_home is JOINTS;
  B21_H_approach is LINK_POSES; B21_HR_leave is STOW.
- **Sides:** cindy gives left/right; alice gives arm.
- **Unknown shapes:** a linear coupled movement without a build, and a movement with both base and arms, raise.
- **mating_drives:**
  - B20 insert: kept `{}`, both tools "ground half", `grounded`.
  - B21 insert: kept `{AT3L: tighten}`, AT3R "no mate".
  - B22 insert: both kept.
  - B20 R_ungrasp: kept `{}`.
  - An already-correct B21 passes through unchanged.

The old `DISPATCH_EXPECTED` (smoke:713) maps over as follows:
- J0 → load;
- J1 / J2 → place_bar / grasp;
- J3 → transfer;
- J4 tighten → moved onto the insert;
- J5 → insert;
- R0 untighten → the dropped loosen;
- R1 / R2 → ungrasp folded into the retreat;
- R3 → home.

**steps:**
- J without the workaround: `[0],[1],[2],[3],[4]`. J with it: `[0], OperatorStep(3, in_pause=(1,2)), [4]`.
- R: `OperatorStep(1, with_motion=(0,)), [2]`.
- H: 4 own steps; HR: 3; a synthetic `bar_placing` action: 1.
- Every index appears exactly once. Also test `step_index_of`.

**scene_for:**
- **Held bar.** At B21_J_transfer (0806), `process/bars/B21` is `Attachment("robots/a200-0806", "left_ur_arm_tool0", grasp)`.
- **Alice's B21_H_close** (0804 + 0806; Alice measured, Cindy not):
  - Cindy's source is "design";
  - B21 is a Pose equal to its assembly pose;
  - Cindy's real left tool0 · grasp matches it within 1e-4.
- **Sources.** A belief beats the design. An unmeasured 0805 with no design is "parked", with a warning.
- **Options.** `hide_held` disables B21 and its halves. `ignore_built` at B22_J_insert disables B20, B21 and their halves,
  but keeps B22 and `ground/`.
- **Kept bodies.** Only `obstacles/` come over from the snapshot.
- **Geometry identity.** The Geometry objects are identical (`is`) to the plugin's.
- **Slow:** `CompasFabMirror(CINDY).sync(scene)` succeeds.

**ik:**
- FK-fit selection as listed. The process alice URDF gives `None`.
- **Round trip** at 20 seeded random bases, for alice, belle, left and right:
  - the first branch equals q within 1e-6;
  - every branch reproduces F*;
  - at most 8 branches, sorted.
- **Slow `solve_ik`, Cindy stowed:** target FK(stow) gives rank 1 and `max_change < 1e-6`. A box at the rank-1 wrist
  gives rank > 1, or `colliding` names `obstacles/box`.
- **Slow fallback:** with `ssik_unavailable` monkeypatched, the backend is "pybullet" and a "!" line is logged.

**collision** (slow):
- `world_frame_fk` at base (2, 1, 0), yaw 0.7, equals `link_pose` within 1e-6, and differs without it.
- `swept_check` finds a box placed midway between two clear waypoints; the sample count is right.
- `in_process_terms` mapping.
- After task C: Belle placed so that only her tool overlaps Cindy's tool is named by `collisions()`.

**joints:**
- +2π + 0.1 → 0.1.
- Elbow 2.26 → −1.06 within ±π stays −1.06.
- No in-range equivalent: keeps the nearest.
- A ±π tie follows `np.round`.
- A goal exactly at a limit.
- `joint_limits` (0806): elbow ±π, pan ±2π.
- `outside_limits` text.

## Commands
```
cd /home/yijiangh/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/husky-assembly-teleop
export HUSKY_DRIVE_ROOT="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly"
python3 -m pytest test -q -p no:cacheprovider            # baseline 336 passed, 2 skipped, plus the new tests
python3 -m pytest test -q -p no:cacheprovider -m slow    # baseline 14 passed, 1 xpassed, plus the new slow tests
python3 -m pytest test/test_layers.py -q -p no:cacheprovider
python3 -m pytest test -q -p no:cacheprovider -m linter -k "flake8 or pep257"
```

## Risks
- **Mirror rebuilds** (~2 s) happen if planning-scene ids or Geometry objects drift from the process plugin's, or if one
  mirror alternates between the full snapshot and `scene_for` scenes.
- **Parked or unconfigured robots** vanish as obstacles. They must show up in `warnings`.
- **Retreat.** The purged process has a position-controlled retreat and a form-B ungrasp with a `loosen`. The old flow
  was a compliant retreat. `operator_steps` keeps the old grouping, and the controller choice is M4's.
- **Stale IK in the purged process.** IK tests use targets made from FK. The process URDFs for alice/belle fit no ssik
  solver, so always solve on the live model.
- **Threading.** The mirror and `pp` are single-thread and global: worker only, then `sync` again.

## Status (2026-10-19)

Tasks A, B, D, E and F done on `yh/core-port`. Task C (core CC.6) is in: the core pointer is at 439bc18 (core branch
`yh/cc6-tool-pairs`, 7cd5a8a + 439bc18), so `test_planning_collision.py::test_tool_against_another_robots_tool` passes
(no longer xfail); `test_only_the_tools_overlap` keeps its scene honest.

Review fixes; these replace the body above on two points:
- **Mount pause order.** With the transfer workaround the transfer runs first, with empty tools; the operator mounts
  and grasps the bar after its motion, at the insertion start (old W:5896–5911, M:448–453, M:6191–6192). So
  `OperatorStep.in_pause` is now `after_motion`, run after the primary in this order, and `indices` is in execution
  order (primary, then `with_motion`, then `after_motion`). The travel to load is not run with the workaround (old
  M:8844): it is the transfer step's `skipped`, left out of `indices` but still counted once. B21_J with the
  workaround: `OperatorStep(3, after_motion=(1, 2), skipped=(0,))`, `OperatorStep(4)`.
- **`ignore_built`** never disables the action's own bar or its halves, built or not (old M:3780): Alice going for
  B21, or Cindy retreating from it, still sees it.
- `swept_check` also fails closed on waypoints whose width is not the number of joint names.

Where the code differs from this spec:
- **Golden table.** The 18 actions are the first 18 of the schedule (B20_J … B35_HR): bars B20–B22 and B34–B37. B25
  and B26 come later. All 306 movements of the purged process classify, with no warnings.
- **`MatingDrives.warnings`.** The "not a male half" diagnostic goes there (no logger in the repo; the function stays
  pure), and `classify` copies it into `MovementSpec.warnings`.
- **`next_start`** walks the whole schedule; after the last movement it returns `(None, "")`.
- **Action-type cross-check.** OPERATOR and BASE_ONLY are accepted in any action type.
- **`scene_for`.** The design is read from the step's start state (the same values `scene_at` gives, without building
  the process scene twice). A robot filled from several sources reports the most guessed one (order: live, belief,
  design, design (stowed arms), parked). A live robot that is disabled is reported parked.
- **`ik`.** Branches are turned toward the reference within the URDF limits (`joint_limits`). `IkResult.colliding` is
  rank 1's pair whenever rank 1 collides, also when a later rank wins. The PyBullet fallback uses the core's
  `planning_group` and `iter_inverse_kinematics` (compas_fab's `inverse_kinematics` hands out the next solution on a
  repeated call).
- **IK test.** A box at the stowed wrist blocks all 64 combinations (the wrist is fixed to tool0), so the result is
  "all collide"; a box at the forearm is added, where a later rank wins.
