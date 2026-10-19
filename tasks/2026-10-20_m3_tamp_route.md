# M3: the tamp route — plan one movement, and a headless check of the whole run (spec)

Part of the core-port plan (milestone M3). M2 (`shared/planning/{joints,movement,steps,scene_for,collision,ik}.py`)
classifies movements, builds planning scenes, solves IK and checks collisions; M3 is the first code that **plans**:
one movement → a core `MovementResult` through tamp / single-arm / linear planners on the core mirror, ported from the
old monitor's dispatch, plus a headless CLI that plans every arm movement of the lab run (the first 18 scheduled actions:
B20_J … B35_HR) at the authored bases and at ±2 cm/2° — M3's acceptance gate before any lab work.

Abbreviations: old code at **0e45758** (read with `git show`): M = husky_monitor.py, CFS = cfab_session.py,
MAN = doc/three_robot_assembly_test_manual.md, SMOKE = scripts/headless_plan_skeleton_smoke.py. C = core
(`external/scaffolding_assembly_core/scaffolding_assembly_core`). API = tamp `motion_planner/api.py` (2306271).
R = `husky_assembly_teleop/`.

Assumes the core's CC.6 (tools vs other tools, core 7cd5a8a) lands first; M3 does not depend on it (it only adds
tool–tool pairs; tamp's linear loops pass `_skip_cc3/4/5`, so CC.6 still runs there).

Probe results already known: at B21_H_to_grasp Alice's goal collides with the bar she grasps
(`tools/a200-0804/ur_arm` vs `process/bars/B21`) because the core derives allowed contacts from the START state only →
`end_contacts` below. All 7 R_home targets in the purged process equal the old `HUSKY_DUAL_ARM_HOME_CONF_12` exactly.

## 1. Files

### 1.1 `shared/planning/arm.py` (new; a move) + `plugins/arm_planner/planner.py`
- Move verbatim from planner.py: `JOINT_STEP`, `PREVIEW_SPEED`, `ArmPath` (planner.py:31–55), `_extend` (126–130),
  `_distance` (133–135), and the search as `plan_arm(mirror, arm_name, goal, abort, *, label: Callable[[str], str] = str,
  time_limit: float = TIME_LIMIT) -> PlanResult[ArmPath]` (body planner.py:152–175, `world.label` → `label`, pass
  `time_limit` to `connect`, search.py:63).
- planner.py keeps `ArmPlanningWorld` and a 2-line wrapper `plan_arm(world, mirror, arm_name, goal, abort)` →
  `arm.plan_arm(..., label=world.label)`; re-export `ArmPath`, `_extend`, `arm_joint_names` in `__all__` (imported by
  plugin.py:27, test_arm_planner.py:13, test_arm_path_send.py:10 — unchanged).

### 1.2 `shared/planning/scene_for.py`: option `end_contacts: bool = False`
When set, for each `(a, b)` in `allowed_contacts(process, end_state(step.movement))` (C/process/relations.py:98;
C/process/plan_check.py:133) with both sides in `ids`, add the other side's live id to the `touches` of each side's body
(bodies present in the scene). Touches change the mirror state, not the cell (no rebuild). Docstring: "contacts allowed
at the movement's start or end (a linear move into a grasp)". This is the old per-movement ACM behaviour; it lives in
the monitor (not a core rule). Test: B21_H_to_grasp with `end_contacts=True` → `process/bars/B21`.touches contains
Alice's live tool id; without → not.

### 1.3 `shared/planning/single_arm.py` (new, ~150 lines)
| API | Behaviour |
|---|---|
| `interpolate_poses(start: Pose, goal: Pose, max_step_distance=0.005, max_step_angle=0.05) -> list[Pose]` | `n = max(1, ceil(d/step_d), ceil(θ/step_a))`; lerp + scipy Slerp; exactly `start … goal`, n+1 poses. Pure. |
| frozen `LinearPath`: `joint_names`, `points: np.ndarray \| None`, `backend` ("ssik"/"compas_fab"), `reason=""`, `waypoint=0`, `colliding: tuple[str, str] \| None = None` | |
| `plan_linear(mirror, robot: RobotObject, arm: str, goal: Pose, *, log=print) -> LinearPath` | worker thread, after `sync`. Per pose after the first: `ik.branches(urdf, robot.base, arm, solver, pose, previous)` nearest first; keep the first with L∞ step ≤ `MAX_JOINT_STEP = radians(10)` and `search_check(q) is None`; else fail with `waypoint=i` and a reason ("unreachable" / "joint jump" / "collides", pair from `collisions(..., full_report=True)`). |

Fallback (no fitting solver or ssik unavailable): port CFS:763–823 — `FrameWaypoints([frame_from_pose(goal)],
TargetMode.ROBOT, 1e-3, 1e-3)`, options `{max_step_distance, max_step_angle, max_jump_revolute: 0.35,
check_collision: True, verbose: False, max_descend_iterations: 200}`, `plan_cartesian_motion(...)` inside
`with mirror.lend() as planner, world_frame_fk(planner):`, catching `(BackendError, ValueError)`.
Why ssik first: exact (1e-6 vs 1 mm), the same algorithm tamp uses for Cindy's linear moves (API:1459–1607).

### 1.4 `shared/planning/tamp_route.py` (new, ~300 lines; result helpers to `route_result.py` if > 350)
```python
@dataclass(frozen=True)
class RouteResult:
    result: MovementResult        # process robot ids; status solved | failed | not_planned
    routine: Routine
    planner: str                  # "tamp.plan_free_dual_arm", "arm.plan_arm", "single_arm.plan_linear/ssik", …
    attempts: int
    seconds: float                # scene_for + syncs + IK + search + checks
    waypoints: int
    end_error: dict[str, tuple[float, float]]   # process flange id -> (m, rad) vs the goal pose
    joint_error: float | None     # max |end - goal joints| for JOINTS/NEXT_START/STOW goals
    goal_joints: dict[str, float] | None        # goal actually used (M4 caches it)
    colliding: tuple[tuple[str, str], ...]      # process terms (in_process_terms)
    swept: SweptReport | None
    ik: IkResult | None
    warnings: tuple[str, ...]
    @property ok -> bool          # result.status == "solved"

def plan_movement(mirror, loaded, step, snapshot, robots, *, transfer_as_free=False, beliefs=None,
                  ignore_built=False, goal_joints=None, abort: threading.Event | None = None,
                  log: Callable[[str], None] = print) -> RouteResult
```
- Start = the acting robot's joints in `snapshot` (the caller decides: live, previous plan end, CLI chain).
- `spec = classify(...)`; TOOL_ONLY/OPERATOR/BASE_ONLY → `not_planned` ("no arm moves"); ValueError if
  `mirror.robot_id != planning.acting`. Every planning scene uses `scene_for(..., end_contacts=True)`.
- Pure helpers (fast tests): `pose_error(a, b) -> (m, rad)` (port M:9505–9526, |quaternion dot|),
  `line_goal(start: Pose, line: LineSpec) -> Pose` (p + d·u, same R), `aims_at_line(movement, spec) -> bool` (True when
  the acting tool is `on` a body at the start: the arm leaves a contact).

Per routine (tamp calls inside `with mirror.lend() as planner, world_frame_fk(planner):`; `mirror.sync(scene)` before
every lend and after it; `goal_dict` is a plain `{name: value}` dict — a numpy goal crashes tamp):

| Routine | Scene / goal | Planner call | Attempts / post | Old code |
|---|---|---|---|---|
| TRANSFER_FREE | IK scene `hide_held=False` (bar held at the goal = insert start): `solve_ik(mirror, robot, {p.arm: p.target}, reference=spec.goal_joints)` (or the `goal_joints` override); `unwrap_to_nearest(goal, start, *joint_limits(...))`; refuse on `outside_limits`. Plan scene `hide_held=True`. | `plan_free_dual_arm(planner, mirror.state, goal_dict, max_time=120.0, max_iterations=50, joint_resolution=0.1)` (API:832) | ≤3 on `birrt_failed` / swept failure; `swept_check` | M:4640–4785, 7099–7165, 6932–7079 |
| TRANSFER_HELD | no hide; goal as above (IK with the bar held) | `plan_constrained_dual_arm(planner, mirror.state, active_bar_id=planning.ids(spec.bar), goal_conf=goal_dict, stage=3, position_res=0.01, rotation_res=0.025, max_time=120.0, derive_start=False, random_seed=9973*k)` (API:909; 1081–1129) | 3; warn if the right flange misses > 2 mm (grasp mismatch, M:9639–9661) | M:9247–9345 |
| INSERT | no hide; `goal_ee_frames = {p.side: frame_from_pose(p.target)}`, else `line_goal` from start FK | `plan_constrained_dual_arm_linear(planner, mirror.state, active_bar_id=..., goal_ee_frames=..., skip_env_collisions=False, report=report)` (API:1610) | 1 | M:9347–9380 |
| RETREAT_LINEAR | `goal_ee_frames = {p.side: frame_from_pose(line_goal(FK_start(p), p.line))}` (FK by `link_pose`) | `plan_dual_arm_linear_independent(planner, mirror.state, goal_ee_frames=..., skip_env_collisions=False, report=report)` (API:1732) | 1 | M:9680–9789 |
| FREE_HOME | `spec.goal_joints` exactly (no unwrap: the process target wins) | `plan_free_dual_arm(..., max_time=30.0, joint_resolution=0.1)` | 3, swept check | M:9791–9821; MAN:1194 |
| FREE_TO_LOAD | `spec.goal_joints` if resolved, else `not_planned` ("needs the transfer start, after M3") | as TRANSFER_FREE | 3, swept | M:7691–7708 |
| SUPPORT_APPROACH | `solve_ik(..., {arm: target}, reference=spec.goal_joints)`; sync | `arm.plan_arm(mirror, arm, goal6, abort, time_limit=30.0)` (outside lend) | 3, swept | M:4430–4582 |
| SUPPORT_LINEAR | `aims_at_line` → `line_goal(FK_start, line)`, else `p.target` | `single_arm.plan_linear(...)` | 1 | M:4456–4479, CFS:763 |
| SUPPORT_LEAVE | JOINTS/NEXT_START → `spec.goal_joints`; STOW → `stow_joints(config, names)` (world/kinematics.py:171) | `arm.plan_arm(..., time_limit=30.0)` | 3, swept | — |

Path column names: `api._arm_joint_names(planner.client.robot_cell)` (left + right, the 12-vector order tamp returns);
`JointTrajectory` results → `jt.joint_names`, `pt.joint_values`.

Held bars in the mirror's `RobotCellState` (C/process/scenes.py:118–121, R/shared/process.py:273, C/mirrors/cell.py:227–262):
acting robot holds → bar + halves `Attachment(acting, "left_ur_arm_tool0", grasp)` (first holder), carried in
`collisions`, CC.3/CC.4, SearchCheck `_carried`, `touch_bodies` = tools on the part + pending-mate halves; tamp finds it
as `rigid_bodies_puids[ids(bar)]` (API:163). TRANSFER_FREE plan → `enabled=False` via `hide_held` → `is_hidden=True`
(moved to HIDDEN_POSITION, skipped by checks). Another robot holds → static at its pose, `touch_bodies += holder`.
Built bar → static; tools on it touch it (start contacts), the to_grasp gripper via `end_contacts`.

Failure reporting (fail closed; any exception → `failed` with `repr`, then re-sync): start/goal collision via
`collisions(..., full_report=True)` mapped by `in_process_terms`; `birrt_failed` ("no path in N s × k"); swept segment +
pairs; IK `reason` + `colliding`; linear `report["reason"]`, `waypoint/n_waypoints`, pairs from
`report["first_colliding_state"]`. A start collision is checked once for every routine (free → fail; linear → warning).

`MovementResult` (C/process/solutions.py:60–91): process ids (`spec.robot`, its 6 or 12 arm joints); `start`, `end`,
`trajectory = Trajectory(spec.robot, names, rows, times=None)`, `start_overridden` when authored start joints exist and
differ by > `CHAIN_TOLERANCE`; `path_poses`/`times` empty (M5).

Threading: worker only, on the thread that created the mirror; lend only around foreign planners (tamp, compas_fab
cartesian); our own `solve_ik`, `plan_arm`, `plan_linear`, `swept_check`, `collisions` run on a freshly synced mirror
outside the lend; re-sync after `solve_ik` before `search_check`; one pybullet_planning user per process; `abort`
checked between attempts (tamp itself can't be cancelled).

### 1.5 `experiments/assembly_plan_check.py` (new, ~250 lines; `experiments` may import shared and plugins, not old)
CLI: `python -m husky_assembly_teleop.experiments.assembly_plan_check PROCESS [--actions 18|B20_J,B20_R] [--only id,…]
[--workaround on|off (default on)] [--perturb-base M,DEG] [--seed 0] [--write-solutions DIR] [--data DIR] [--verbose]`.
`OLD_HOME` = the 12 values of old utils.py:90 (0e45758), keyed by name.

`run_check(loaded, robots, actions, *, transfer_as_free=True, perturb=None, seed=0, only=None, extra_bodies=(),
mirrors=None, log=print) -> list[Row]`:
1. seed `random`, `np.random`, `rng = np.random.default_rng(seed)`;
2. walk actions in schedule order; per action `operator_steps(...)`; `skipped` indices → status "skipped";
3. per arm movement build the **design snapshot** (no ROS): for every configured process robot present in
   `movement.start.robots`: base = perturbed design base (offset drawn once per (robot, design base rounded to
   1 mm/0.1°): dx, dy ~ U(±m), dyaw ~ U(±deg) about world z); joints = the run's `current[robot]` if it stands at the
   same base, else the authored start joints, else `stow_joints`; `RobotObject(robot_id(serial), model, base, joints)`
   (measured). Robots absent from the start are left out. `Scene(robots=..., bodies=extra_bodies, world_poses=...)`.
   An acting robot with no design base → `not_planned`;
4. `plan_movement(mirrors[acting], ...)` (mirrors created lazily, closed in `finally` unless passed in);
5. on success `current[robot] = (base, end)`; pop it when the robot's arm movement failed or was left out by `--only`
   (a workaround skip does not pop).
Also: `home_check(loaded, specs)` (FREE_HOME goals vs `OLD_HOME`, max |Δ| per movement); `format_table(rows)` (movement,
routine, ok, attempts, s, end err mm/deg or joint err, waypoints, note); `write_solutions(loaded, rows, folder,
settings)` — refuses a folder inside the source; copies the process without `solutions/`/`runs/`, reads the copy,
writes one `Solution` per action with `solved_against(copy, action)` (workaround-skipped loads `not_planned`), prints
`len(solution_warnings(...))` + first 10 lines (stale IK expected). `main(argv) -> int`: 0 when every row is solved /
skipped / not_planned; 1 on any `failed`; 2 on bad input. Without `--verbose`, capture each route's stdout and print its
last 20 lines only on failure.

### 1.6 Docstrings
List arm.py, single_arm.py, tamp_route.py in `shared/planning/__init__.py`; `assembly_plan_check` in
`experiments/__init__.py`.

## 2. Tasks
| # | Task | Files | Depends |
|---|---|---|---|
| A | arm search move | shared/planning/arm.py, plugins/arm_planner/planner.py | — |
| B | `end_contacts` | scene_for.py, test_planning_scene_for.py | — |
| C | single-arm linear | single_arm.py, test_planning_single_arm.py | B (slow test) |
| D | route | tamp_route.py, test_planning_tamp_route.py | A, B, C |
| E | CLI | experiments/assembly_plan_check.py, test_assembly_plan_check.py | D |
| F | docstrings, full test run, CLI runs, this spec's status section | — | all |

## 3. Verification
Fast (no planning): `pose_error` (identical → 0; sign-flipped quaternion → 0; 90° about z → π/2); `line_goal` moves
exactly d·u, keeps R; `interpolate_poses` ends exact, count = ceil(max(d/0.005, θ/0.05)) + 1, steps within limits, zero
move → 2 poses; `aims_at_line` (B21_H_to_grasp → False, B21_HR_retreat → True); `perturbed` (exact dx, dy; yaw adds dyaw;
z unchanged; same seed → same offsets, within bounds); selection (18 actions B20_J…B35_HR; workaround → 7 skipped
loads, 36 arm movements); `home_check` all 7 R_home ≤ 1e-12; argparse defaults.

Slow (`test_planning_tamp_route.py`, one module-scoped mirrors dict; tolerances: ssik routes 1e-4 m / 1e-3 rad, joint
goals 1e-9 rad):
| Movement (`run_check(only=…)`) | Assertions |
|---|---|
| B20_J_transfer | TRANSFER_FREE; ok; `ik.backend == "ssik"`; `swept.ok`; first row = stow; both flanges vs target link_poses within tol.; 12 names |
| B20_J_insert (chained) | INSERT; ok; start = transfer end (1e-9); flanges vs insert link_poses within tol.; 2 ≤ waypoints ≤ 10 |
| B20_R_retreat | each flange moved 0.05 m along (0, −1, 0) within 1e-4; orientation within 1e-3 |
| B20_R_home | end = target joints (1e-9); attempts ≤ 3; `swept.ok` |
| B21_H_approach | `ik.solvers == ("alice",)`; end vs link_poses within tol. |
| B21_H_to_grasp | aims at link_poses; ok (needs `end_contacts`); waypoints 21 ± 1 |
| B21_HR_leave (chain broken → authored start) | STOW; end = stow (1e-9) |
| box failure | `extra_bodies=(Body("obstacles/box", box((0.3,)*3), Pose(FK of HOME left tool0)),)`, only B20_R_home → failed, attempts 1, reason mentions the goal, a pair has "obstacles/box" and a "robots/cindy/"/"tools/cindy/" side |
| perturbed | `perturb=(0.02, 2.0), seed=0`, B21_H_approach + B21_H_to_grasp → ok; base offset non-zero; end vs world link_poses within tol. |
| write_solutions | original has no `solutions/` afterwards; the copy reads back (`read_solutions`), `is_stale` False |

Commands:
```
cd /home/yijiangh/Code/ros2_ws_next && source venv/bin/activate && source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/husky-assembly-teleop
export HUSKY_DRIVE_ROOT="/home/yijiangh/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/2025-03 Husky Assembly"
P="$HUSKY_DRIVE_ROOT/data_design_study/261008_2ModulesDemo_purged_process"
python3 -m pytest test -q -p no:cacheprovider            # baseline 383 passed, 2 skipped (+ new)
python3 -m pytest test -q -p no:cacheprovider -m slow    # baseline 25 passed, 1 xfailed, 1 xpassed (+ new)
python3 -m pytest test/test_layers.py test/test_arm_planner.py test/test_arm_path_send.py -q -p no:cacheprovider -m ""
python3 -m pytest test -q -p no:cacheprovider -m linter -k "flake8 or pep257"
python3 -m husky_assembly_teleop.experiments.assembly_plan_check "$P"
for s in 0 1 2; do python3 -m husky_assembly_teleop.experiments.assembly_plan_check "$P" --perturb-base 0.02,2 --seed $s; done
python3 -m husky_assembly_teleop.experiments.assembly_plan_check "$P" --workaround off --only B20_J_load,B20_J_transfer
python3 -m husky_assembly_teleop.experiments.assembly_plan_check "$P" --write-solutions <scratch dir>
```

Comparison with the old headless run (different data → compare outcomes, not numbers): the smoke ran on 260920_RobArch
and "does not check planning outcomes" (SMOKE:15–23); its dispatch table is M2's golden test. From MAN:
| Old (MAN line) | New | Acceptance |
|---|---|---|
| H_M0 free approach ≈65/63 wp (919, 1103) | B21/B35_H_approach | ok (counts not comparable: dense pp path vs RRT corners) |
| H_M2 to grasp ≈22 wp (962, 1121) | B21/B35_H_to_grasp | ok, 21 ± 1 |
| HR retreat ≈21 wp (1226, 1252) | B21/B35_HR_retreat | ok, 21 ± 1 |
| B36 transfer swept rejection, ground (1162–1167, 1284) | B36_J_transfer | ok within 3 attempts |
| B37 home failed twice, right pan ≈5.05 rad (1194–1199, 1344) | B37_R_home | ok within 3; print the right-pan change |
| 2π gap transfer end ↔ insert start (1163) | chain | insert start = transfer end exactly |

Gate: every arm movement solved or skipped at the authored bases, and at ±2 cm/2° for seeds 0–2; a failure there is
reported to the user, not patched. Timing: each movement within its budget (attempts × max_time + IK); the table flags
> 60 s. HOME: `home_check` max |Δ| (0 now; rerun on the Windows export).

## 4. Risks
1. Contacts at the end of a grasp: without `end_contacts`, to_grasp fails at its goal (probe). The option allows the
   gripper to touch its bar along the whole line — the old ACM's behaviour. Candidate to move into the core later
   (Rhino's planner meets the same case).
2. Mirror after lend / IK: tamp and IK leave the PyBullet world anywhere → always `sync` before `search_check`,
   `swept_check`, `collisions`.
3. Goals must be dicts (numpy crashes tamp). FREE_HOME is not unwrapped (B37 home turns ≈5 rad).
4. `--workaround off`: no derived load start in M3 → TRANSFER_HELD starts from the chained HOME with the bar on; expect
   start collisions and exit 1 — reported, not hidden.
5. Stale IK (B15): goals are link_poses (IK with reference = target joints); "end ≠ target joints" warnings expected.
6. After the CC.6 bump, perturbed bases may report real tool–tool hits (nearest 12 mm at B36); name them in process terms.

## Status (2026-10-20)

Tasks A–F done on `yh/core-port` (commits 0760976 arm, 10215be end_contacts, d6b5556 single_arm, 95a82f8 tamp_route,
07605b7 CLI, 6ebe75b CLI note, and this spec with F). Also, asked during the work, e033452: a straight move holding a
bar is an INSERT whatever its drives or build (newer exports give a ground bar's insert no drives and build it in a
later `<bar>_J_fix_foundation` operator movement); such an insert is `grounded` from its tools' halves.

Tests: default 383 → 400 passed (2 skipped); `-m slow` 25 → 37 passed, 1 xfailed (CC.6, strict, unchanged),
1 xpassed; linters clean.

### Gate: not met — reported, not patched

! These results are from before the follow-ups below (arms carried across base drives, the core's CC.6); the
re-run is at the end of this section.

| Run | Solved / failed / skipped | Failed |
|---|---|---|
| authored bases | 35 / 1 / 7 | B35_J_transfer |
| ±2 cm/2°, seed 0 | 32 / 4 / 7 | B35_J_transfer, B22_J_transfer, B37_J_insert, B37_R_home |
| ±2 cm/2°, seed 1 | 35 / 1 / 7 | B35_J_transfer |
| ±2 cm/2°, seed 2 | 35 / 1 / 7 | B35_J_transfer |

1. **B35_J_transfer (every run): the whole-turn unwrap sends the left arm through the robot.** The goal by IK is the
   authored target exactly (rank 1 of 64, change 0). Its left shoulder_lift is −3.210 rad; the transfer starts from the
   stowed arms (lift −0.038), from which +3.073 is 0.06 rad nearer, so `unwrap_to_nearest` picks it: the upper arm
   swings down through `dual_arm_bulkhead_link` and `base_link` (41 of 49 samples on the straight line hit). No path in
   3 × 120 s. Kept as solved (no unwrap), it plans in 1 try, 16 s, 69 waypoints (seeds 0 and 1); started from the old
   HOME — where Cindy's arms really are after B34_R_home — the unwrap keeps −3.210 and it plans in 1 try, 8 s.
   Open choice: carry a robot's joints across its base drives (its arms do not move while the base drives) instead
   of "else the authored start, else stowed"; and/or turn a joint by whole turns only where that does not cross
   the robot (or keep IK's own value, already turned toward the target joints, when it is within the limits).
2. **B37_J_insert (seed 0, base +(4.6, −4.7) mm +2.0°): out of reach.** The left tool0 target is 1.0596 m from
   `left_ur_arm_base_link`; at the authored base ssik finds 2 branches only, at this base none
   (`left_unreachable at waypoint 3/3`). B37_R_home then starts from the retreat's authored joints (chain broken) and
   finds no path in 3 × 30 s.
3. **B22 (seed 0, base +(17.4, 12.6) mm −2.0°): swept hits near the structure.** The transfer's 3 paths each sweep
   AT3R through `bars/B20` or `joints/J20-22_mocap` near their end (segments 72–76 of 74–99). The way home is marginal:
   in a first seed-0 run its 3 paths swept AT3R through `joints/J20-22_male` in segment 1–2, just after the retreat's
   end; in the run above it passed at its 3rd try (93 s), as in seed 2 (98 s). The paths graze the structure: see
   (b) under "Re-run after the follow-ups" for the measurements.

Timing: besides the failures, rows over 60 s are ways home needing 3 tries (B22_R_home seeds 0 and 2, B36_R_home
seed 1: 86–98 s) and, in some runs, B22/B36 transfers (59–110 s). A failed transfer costs 300–360 s (3 × 120 s).

### Where the code differs from this spec

- **Files.** The route is three files: `tamp_route.py` (258 lines: `plan_movement`, the routines), `route_search.py`
  (225: `RouteSearch`, the searches, checks and results), `route_result.py` (111: `RouteResult`, `pose_error`,
  `line_goal`, `aims_at_line`). The CLI is 446 lines (`Row`, `BaseOffsets`, `perturbed`, `base_key`,
  `arm_movements`, `select_actions` besides the functions named here).
- **`aims_at_line(process, movement, spec)`**: the process finds the tool on a moving flange.
- **compas_fab fallback**: `FrameWaypoints(..., tolerance_position=1e-3, tolerance_orientation=1e-3)` by keyword (the
  third positional argument is `native_scale`).
- **Limits** are checked for every tamp free search (transfer, load, home), as the old `_plan_free_and_validate` did.
  The approach's goal is not unwrapped (IK already turns branches toward the target joints).
- **`goal_joints` override** applies to the transfers, the travel to load and the approach; the way home keeps the
  process's target.
- **`attempts`** counts the search's tries: 0 when refused before one (start collides, no IK).
- **Box test**: a 0.1 m box at home's left tool0 fails the way home at its first try ("the goal collides", the box with
  Cindy's wrist and AT3L). A 0.3 m box also covers her upper arm at the start (the retreat's end): refused at the
  start check, 0 tries; both are tested.
- **`home_check(specs)`** takes the classified specs only. **`run_check(..., verbose)`**: without it each route's
  output is kept at the file descriptor (`redirect_stdout` breaks PyBullet's helpers: `UnsupportedOperation:
  fileno`). The CLI prints one progress line per movement to stderr; `--write-solutions DIR` writes into
  `DIR/<process folder>` and refuses an existing copy.
- The slow route tests use `run_check`, so they came with the CLI's commit.

### Comparison with the old headless run

| Old (MAN) | New (authored) |
|---|---|
| H_M0 approach ≈65 wp | B21/B35_H_approach ok, 2 waypoints (the straight move is free) |
| H_M2 to grasp ≈22 wp | B21/B35_H_to_grasp ok, 21 |
| HR retreat ≈21 wp | B21/B35_HR_retreat ok, 22 / 21 |
| B36 transfer swept rejection | B36_J_transfer ok in 1–2 tries over all runs (29–110 s) |
| B37 home failed twice, right pan ≈5.05 rad | B37_R_home ok in 1–2 tries (21–63 s; it fails in seed 0 after the insert there); right pan −1.23, largest turn left elbow −2.67 |
| 2π gap transfer end ↔ insert start | insert start = transfer end exactly. But the unwrap puts the right pan 2π off the process's target joints on 5 of 7 transfers (B20, B22, B34, B36, B37); insert and retreat follow, and the way home turns it back (right pan −2.6 to −3.25 rad) |

HOME: all 7 R_home targets within 4.9e-13 rad of the old `HUSKY_DUAL_ARM_HOME_CONF_12`.

`--write-solutions`: 18 solutions in the copy, none stale, the original without `solutions/`. 21 warnings: 15 "6.28
from the process's target joints" (the unwrapped right pan above, not stale IK), 5 "ends <R_home> and starts <next
J_transfer> 0.439 apart" (the next transfer starts from the stowed arms at its new base, by this spec's chaining rule),
1 "B34_R_home … B35_J_insert 4.19 apart" (B35's transfer failed, so its insert starts from its authored joints).

`--workaround off --only B20_J_load,B20_J_transfer`: the load is not planned (no transfer start in M3); the held
transfer, from the stowed arms with the bar on, is not a start collision as risk 4 guessed: tamp's constrained
search never grows a tree (`tree_a=1, tree_b=1`, "max_iterations" × 3, 294 s). Exit 1.

Other: tamp writes `husky_assembly_tamp/logs/dual_arm_task_space_rrt_core.log` under `external/` (gitignored) whenever
its constrained planners run.

### Follow-ups after the review

1. **The CLI carries a robot's arm joints across its base drives** (decision on open question 1). The arms do not
   move while the base drives, and the lab's live start is where the arms really are (home after R_home). In
   `run_check` a robot's joints are its last planned end wherever its base stands; the authored start joints only
   when it has none yet (its first movement, or after a failed or `--only`-left-out arm movement), else its stowed
   arms. This replaces step 3's "current only on the same base, else authored, else stowed" (§1.5).
   `unwrap_to_nearest` stays as it is (old W:3098), question 2 left open.
2. **Core pinned to the reviewed CC.6 change** (439bc18, core branch `yh/cc6-tool-pairs`, 7cd5a8a + 439bc18; the
   core branch must be pushed before this one). The CC.6 test passes (no longer xfail); the design-cell comparison
   and `scripts/legacy_equivalence.py` wrap their plain planner with `with_tool_pairs` (like for like).

### Re-run after the follow-ups

Tests: default 401 passed, 2 skipped; `-m slow` 38 passed, 1 xpassed (the base planner's lattice test, not strict);
layers + arm planner 13 passed; flake8 and pep257 clean; the core's own tests 129 passed, 3 skipped;
`test_compas_fab_mirror` against the design cell passes on the 260814 export. `test_legacy_equivalence` does not
run on the 260814 or 261016 exports: `convert_export` refuses their Cindy cell (`right_ur_arm_base_link-
base_fixed_joint` has another origin than our URDF), before any collision check.

Solved / failed / skipped, and the failures: (a) B37's insert at the left arm's reach, (b) tamp's 0.1 rad checks vs
the swept check's 0.01 rad, (c) caught by CC.6, (d) new.

| Run | Chaining only (core c985034) | Chaining and CC.6 (core 439bc18) |
|---|---|---|
| authored bases | 35 / 1 / 7: B22_R_home (b) | **36 / 0 / 7** |
| ±2 cm/2°, seed 0 | 34 / 2 / 7: B22_J_transfer (b), B37_J_insert (a) | 33 / 3 / 7: B22_J_transfer (b), B37_J_insert (a), B37_R_home (c) |
| ±2 cm/2°, seed 1 | 35 / 1 / 7: B22_R_home (b) | **36 / 0 / 7** |
| ±2 cm/2°, seed 2 | 35 / 1 / 7: B22_J_transfer (b) | 35 / 1 / 7: B22_J_transfer (b) |

Gate: met at the authored bases and for seed 1; not for seeds 0 and 2. B35_J_transfer plans in every run now (its
transfer starts from home, where the unwrap keeps the authored shoulder lift).

- **(a) B37_J_insert, seed 0**: as before; the left target is 1.0596 m from the arm's base link, 2 ssik branches at
  the authored base, none at +(4.6, −4.7) mm +2.0°.
- **(b) B22_J_transfer, seeds 0 and 2: the paths graze the structure.** Each of the 3 paths sweeps AT3R through
  `bars/B20` or `joints/J20-22_mocap` on its way in (segments 72–80 of 74–96), not at the goal: there AT3R is 46.8 mm
  from B20 and 37.0 mm from J20-22_mocap (seed 0). It does not "step over" the bar: the hit segments are short (largest
  joint step 0.045–0.061 rad, below tamp's 0.1 rad), and their waypoints, which tamp checked, are clear by only 0 to
  22 mm; between them the 0.01 rad samples go 2 to 14 mm into the bar or half (e.g. AT3R and J20-22_mocap: waypoints
  +5.2 / +1.9 mm, −14.0 mm between, 6 of 8 samples in). Measured by the reviewer: `/tmp/claude-1000/-home-yijiangh-Code-ros2-ws-src-husky-assembly-teleop/b293830d-c2c5-4642-b9b4-6739881709bf/scratchpad/m3_review/b22_run.txt`. With tamp's
  `joint_resolution` at 0.1 rad 1 of 3 paths hit, at 0.05 and 0.02 rad 3 of 3 were clear (48–61 s and 103–119 s
  per path), and a hit path re-planned at 0.02 or 0.01 rad around its bad segment came out clear
  (`/tmp/claude-1000/-home-yijiangh-Code-ros2-ws-src-husky-assembly-teleop/b293830d-c2c5-4642-b9b4-6739881709bf/scratchpad/m3_review/b22_options.txt`); the resolution is the user's decision, unchanged here. B22_R_home is marginal: 2–3 tries in
  every run after the bump (64–96 s); before it, it failed twice (authored, seed 1), AT3R through `joints/J20-22_male`
  in segment 1–2.
- **(c) B37_R_home, seed 0: `tools/cindy/AT3R` against `robots/belle`.** A path planned with CC.6 off sweeps
  Cindy's right tool through Belle on 59 of its 1281 samples (start and goal clear); with CC.6 on there is no path
  in 3 × 30 s (twice), without it one in 1–3 tries (twice). It follows (a): the chain broke, so the retreat starts
  from its authored joints at the moved base (touching `joints/J35-37_mocap`). Before the bump, seed 0 "solved" it
  at its 3rd try with the right pan turning +5.05 rad, through Belle, unseen.
- **(d)**: none.

Timing after the bump, besides failures: B36_J_transfer 123 s (3 tries, authored) and 166 s (3 tries, seed 1);
B22_R_home 64–96 s; B36_R_home 72–73 s (2 tries). Home check 4.89e-13 rad in every run.

The `--only` reproducibility gap noted here before is closed by 4d90bb9 (below); the probes above planned the full
walk with stand-ins instead.

### Review (approve with nits) and its fixes

- e96f294: `end_contacts` lets a tool touch only the body it is on at the movement's end (before: every contact
  allowed at the end, so the gripper also touched every half on its bar). B21/B35_H_to_grasp still plan in 21 steps.
- 4d90bb9: base offsets drawn from (seed, the stand's base key) and each movement's searches seeded from (seed,
  movement id): `--only` and `--actions` give the same offsets as the full run.
- 7f3ee16: tests for the failures a route reports (single-arm "unreachable" and "joint jump"; tamp's linear report
  with its waypoint and nearest pair, on B37's insert out of reach and on a box behind B20's retreat), fail closed
  (an error inside tamp's borrow, the mirror synced after it), `abort` ("cancelled"), and the workaround's hidden bar.
- 3499f36: a colliding start told from a colliding goal; `--write-solutions` checked before planning; Args/Returns
  on the route's methods.
- The B22 diagnosis above is corrected: the paths graze the structure, they do not step over it.

Unchanged, pending the user: `unwrap_to_nearest`, B37's insert at the left arm's reach, tamp's `joint_resolution`.

Re-run: default 402 passed, 2 skipped; `-m slow` 45 passed, 1 xpassed; layers + arm planner 13 passed; flake8 and
pep257 clean. CLI at the authored bases: 36 solved, 0 failed, 7 skipped, exit 0 (slowest B36_J_transfer 63 s, 1
try; total 243 s); home check 4.89e-13 rad.
