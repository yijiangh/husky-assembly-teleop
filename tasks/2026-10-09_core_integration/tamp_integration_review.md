# Tamp × shared core × schema 2: integration review (tamp's advocate)

2026-10-09. Reviewed:

- **Spec:** `specs/schema2_proposal.md`, `specs/schema2_implementation_status.md`, `specs/shared_core_report.md` and `specs/earlier_review_tamp.md`.
- **Core:** the `husky-assembly-teleop` working tree on `jg/viser-cleanup` (HEAD e328f5b plus uncommitted changes). Cited below as "core WT".
- **Tamp:**
  - `main` 517d692, which Rhino pins;
  - teleop's pin 2fce15f;
  - `origin/yh/holding-prm` 5143725, `origin/yh/ssik-speedup` 147d767 and `origin/yh/vamp_integration` 93e6010. `git ls-remote` shows no newer commits.
- **Rhino:** `origin/hs/mocap-experiment` 069275f, where it calls tamp.

Paths in the evidence column are relative to each repo. Spec lines are from `specs/schema2_proposal.md`.

## 1. Verdict

Schema 2 and the core fit tamp's in-process motion API well.

- **What worked.** A converted design reached all three tamp refs (517d692, 2fce15f, 5143725) with no changes to tamp: `read` → `scene_at` → `CompasFabMirror.sync` → `lend()` → tamp.
  - Planned: the coupled transfer (with tamp's start derivation), the insert, the retreat and home.
  - Keyframe IK solved the approach, assembled and retreat keyframes.
  - The results were written as valid `solutions/` files.
  - The earlier blocker (cell layout, group names, `bar_<id>` names) is gone for the API: the mirror builds exactly the layout tamp expects.
- **One blocker remains: derived allowed contacts.** A tool may touch only the half it is on. compas_fab loads each tool as one convex hull, which overlaps the carried bar by 1.3 mm, while the real meshes are 1.01 mm apart. So every carried-bar state "collides", and the coupled transfer fails until the contacts are patched.
- **Smaller gaps, mostly about format and policy rather than code:**
  - null start joints that tamp would derive offline;
  - no status for movements that have nothing to plan;
  - a 1e-6 chaining tolerance that tamp's IK cannot meet;
  - who owns keyframes (design `target.joints` or `solutions/ keyframe_only`);
  - an uninstallable core;
  - no single-arm planner for the 24 support movements.

## 2. What I ran

All prototype code and logs are in `prototypes/tamp/` next to this file (the design copy stayed in a temporary scratchpad). Every script ran in the teleop venv (Python 3.10) with `PYTHONPATH=<teleop root>[:<tamp worktree>]`, on a copy of the converted 260814 design (`tamp_review/design_260814`). No repository was modified.

| Script / log | What it does |
| --- | --- |
| `try_tamp.py` → `run_2fce15f.log`, `run_main.log`, `run_prm.log` | Reads the design and runs `plan_check`. Builds a mirror for movements of `B10_J_joint` and `B10_R_release`, then calls tamp through `lend()`: the motion API, keyframe IK and the solutions writer. Also checks the ssik artifact against the design URDF. |
| `run_nopatch.log` | `J_M3` with the derived contacts only. |
| `diag_home.py` → `diag_home.log` | Why `R_M3` (home) failed at first. |
| `diag_vhacd.py`, `diag_exact.py`, `diag_exact2.py` → `diag_*.log` | Tool-to-bar clearance: one hull, VHACD parts and exact meshes. |
| `diag_hidden.py`, `diag_export.py` → logs | Hidden bodies in tamp's obstacle list. Cell JSON export for PRM workers. `pp.LockRenderer` inside `lend()`. |

**Results** (identical on 517d692, 2fce15f and 5143725 unless noted):

| Step | Result |
| --- | --- |
| `read` + `check_plan(geometry=False)` | OK, 0.6 s. Known findings only: 2 B11 errors and 4 B5 warnings. |
| `mirror.sync(scene_at(B10_J_M4))` | OK, first build 0.4 s.<br>• Tools: `tools/AT3L`, `tools/AT3R`, `robots/alice`, `robots/belle`.<br>• 94 rigid bodies, keyed `bars/…` and `joints/…`; 47 hidden.<br>• The bar is attached to `left_ur_arm_tool0`, and its halves ride on the same link. |
| Collision check at the authored insert start | **2 pairs: (`tools/AT3L`, `bars/B10`) and (`tools/AT3R`, `bars/B10`).** None once bar↔own-tool contacts are allowed, as the old export allowed them (`touch_bodies: ['AT3L','AT3R']`). |
| Tamp's view of the mirror's cell | OK.<br>• `_require_loaded_cell` passes.<br>• `resolve_arm_groups` gives `base_left/right_arm_manipulator`, the same groups the mirror attached the tools to and that `api.py` hardcodes.<br>• 12 arm joints found. |
| `J_M4` insert: `plan_constrained_dual_arm_linear(goal_conf=target.joints)` | Solved, 5 waypoints in 1.0 s. Its end is **7.7e-4 rad** from the design's target joints.<br>It also "solves" with the derived contacts only, because linear planners skip CC3–5 (environment collisions) by default. |
| `R_M2` retreat: `plan_dual_arm_linear_independent` | Solved, 5 waypoints in 0.6 s. Its start still has the two tool↔bar "collisions". |
| `R_M3` home: `plan_free_dual_arm` | Failed (`birrt_failed`) at tamp's default `max_iterations=20` within 5 s and 20 s.<br>Solved with `max_iterations=200`: 459 waypoints in 30 s at 2fce15f (341 at 517d692, 268 at 5143725). This is a budget setting, not an integration problem. |
| `J_M3` coupled transfer, design start joints `null` | `mirror.sync` refuses ("joints not known").<br>After seeding the core's `assumed_start_all` joints and allowing bar↔tool contacts, `plan_constrained_dual_arm(derive_start=True)` solved: 46 waypoints and `path_poses`, in 10.6 s (20 s on main).<br>**With the derived contacts only: `goal_in_collision`** (`run_nopatch.log`). |
| Keyframe IK: `solve_keyframe_chain` over approach (`J_M3` target), assembled (`J_M4` target) and retreat (`R_M2` target), one base | Solved in 1–11 s with the gradient backend. Two adapters were needed: `SimpleNamespace(start_state=mirror.state, target_ee_frames={left,right})`, and the mm base matrix.<br>Each solution sits on another IK branch than the design's (Rhino's) targets: one joint differs by 2π, and the rest differ by up to 2.4–3.1 rad modulo 2π. |
| Teleop pin 2fce15f without `HUSKY_IK_BACKEND=gradient` | **Every IK-based call fails:** ssik is the default backend, and the teleop venv has no ssik.<br>The ssik keyframe cold solve also needs `home_conf_12`. |
| `write_solution` for `B10_J_joint` and `B10_R_release`, then `read_solutions` and `solution_warnings` | Valid files: 16 kB, and 62–103 kB because of home's 459 rows. Statuses: `not_planned` for the 4 movements without arm planning, `solved` for the rest.<br>**3 chain warnings:** 3.3e-6, 7.7e-4 and 8.7e-4 rad, all above 1e-6. |
| ssik artifacts (`asset/ssik/{left,right}_ur_arm_ik.py`, KinBody hash e1e0725d821f) vs the design's `robots/cindy/robot.urdf` | FK identical over 20 random configurations (max \|dp\| 5.6e-16 m), so the design copy is the non-Stock calibrated URDF the artifacts were built for. Nothing records or checks this at run time. |
| Exact clearance, all 20 inserts (40 tool/bar pairs) | Exact meshes: min 1.01 mm, median 1.01 mm.<br>PyBullet signed distance: one hull −1.3 mm, VHACD with 4 parts −2.0 mm, so **convex decomposition does not fix it**. |
| B19 and B21 inserts and retreats | Besides the tool↔bar pairs: bar↔mated female halves (`J17-19_female`, `J18-19_female`, …) and tool↔female half. |
| Hidden bodies | 47 of the 98 PyBullet bodies in tamp's `_collect_obstacle_puids` are hidden (absent), parked at the world origin. compas_fab's own check skips them, but VAMP's point cloud export includes them. |
| Mirror cell → JSON → fresh client (PRM worker style) | Cell 10 MB and state 54 kB, dumped in 0.2 s and rebuilt in 1.0 s. Same collisions as the mirror. |
| `pp.LockRenderer()` inside `lend()` | **`KeyError: 0`**: `pp_client` does not register the client in `pp.CLIENTS`. |

## 3. Mapping tables

### 3a. Tamp input → core source

| Tamp needs | Core source | Status |
| --- | --- | --- |
| Loaded `PyBulletPlanner`; cell with the robot plus `ToolModel`s on groups | `CompasFabMirror.sync(scene)` then `lend()` | **Works** |
| Group names `base_{left,right}_arm_manipulator` | The mirror's `planning_group` picks the same groups (`mirrors/compas.py:219-235`) | **Works**, but tamp hardcodes them (`api.py:53-56` @517d692) |
| Tool links `*_ur_arm_tool0`, 12 arm joints by suffix | URDF names are unchanged | **Works** (Cindy only) |
| Body names (`active_bar_id`) | The API takes the id as a parameter, so `bars/B10` works | **Works** for the API. The headless planner builds `bar_<id>` itself. |
| Start `RobotCellState` | `mirror.state` | **Works** when the design's start joints are known. **Adapter needed** for `null` (40/40 `J_M3`, 8/8 `H_M0`): the mirror refuses them (`mirrors/compas_fab.py:144-147`, core WT). |
| Attachment of the carried bar, with two holders | First holder only (`left_ur_arm_tool0`, with its grasp); halves composed onto the same link | **Works**: tamp derives both grasps by FK. The second holder's grasp is not used by tamp; B14 checks it. |
| Allowed contacts | Derived: `relations.allowed_contacts` puts them in `Body.touches`, and the mirror turns them into `touch_bodies`/`touch_links` | **Blocker**: tool↔carried bar is missing (see P1) |
| Tool mount contacts (`touch_links`) | `RobotModel.tool_touches`: wrist 2, wrist 3, flange, tool0 | **Works** (the old export had wrist 2 and 3 only) |
| Goals: joints or tool0 frames `{left, right}` | `target.joints[robot]`; `target.links[link id]` → `frame_from_pose` | **Adapter needed**: flange id → "left"/"right" |
| Keyframe IK inputs: mm 4x4 base and targets; `(role, movement)` with `start_state` and `target_ee_frames` | `Pose.matrix()` ×1000; `scene_at(movement)` per keyframe | **Adapter needed**. The chain spans the J and R actions (retreat = `R_M2`). |
| `home_conf_12` (ssik cold solve, start derivation) | None. The adapter used `R_M3`'s target. | **Missing** (tamp setting or spec) |
| Base when the design has `null`; ground meshes in mm (`WalkableGround.json`) | `ground/` bodies with geometry, in metres | **Adapter needed** (mm shim) |
| `line` (direction, distance) | `Movement.line` | **Not used by tamp**: its linear planners take goal frames or joints. The adapter must build frames from FK(start) + `line` when the target is absent. |
| ssik URDF identity | Design URDF = non-Stock calibrated; FK-identical to the artifacts | **Works**, but **unrecorded** |
| PRM worker scene (trimmed cell and state as JSON) | `json_dumps(mirror.cell / mirror.state)` | **Works** (tested). Needs a core helper and hash-keyed caches. |
| VAMP scene (point cloud and spheres in the robot root frame) | `Scene` (enabled bodies, geometry, `world_poses`) | **Adapter needed**. Today it is Windows→WSL only. |
| Single-arm robot (Alice, Belle) | Mirror for `robots/alice` | **Missing in tamp**: `_arm_joint_names` asserts 6+6 (`api.py:84`) |

### 3b. Schema 2 movement parts → tamp routine

Counts are over both converted designs (364 movements). "Tamp" means planning tamp can do today.

| Parts (`arms`, `path`, `coupled`, `controller`, tool part, `ends_on`, target, start) | Count | Example | Tamp routine | Status |
| --- | --- | --- | --- | --- |
| 2, free, –, position, –, target, **`target: null`**, start null | 40 | `J_M0` free to load | `plan_free_dual_arm`. The goal is the next arm move's solved start (`J_M3`); the start is the live robot. | **Planned at execution only** (`not_planned` offline). Goal resolution needs a rule (C3). |
| 2, free, coupled, position, –, target, joints+links, **start null** | 40 | `J_M3` transfer | `plan_constrained_dual_arm(derive_start=True)` | **Works with adapter** (seeded start). Blocked by P1. |
| 2, linear, coupled, compliant, drive tighten, `ends_on: tools`, joints+links | 40 | `J_M4` insert | `plan_constrained_dual_arm_linear` | **Works**. Drives and `ends_on` do not affect the path. CC3–5 are skipped by default. |
| 2, linear, –, position, –, target, joints+links | 40 | `R_M2` retreat | `plan_dual_arm_linear_independent` | **Works** |
| 2, free, –, position, –, target, joints | 40 | `R_M3` home | `plan_free_dual_arm` | **Works** (needs `max_iterations` ≫ 20) |
| 1, free / linear, position | 24 | `H_M0`, `H_M2`, `HR_M1` (Alice, Belle) | none | **Missing**: tamp is dual-arm only. Support IK lives in Rhino (`core/robot_cell_support.py`). |
| 0 arms: grip change (`ends_on: tools`) or `ends_on: operator` | 140 | `J_M1` mount, `J_M2` grasp, `R_M1` ungrasp, `H_M3` and `HR_M0` grip | Nothing to plan; start = end | **Status missing** (P5) |
| Untighten (linear, coupled, compliant, drive loosen; bar attached and built) | 0 | — | `plan_constrained_dual_arm_linear`, reversed | **Adapter needed**, untested (no data) |
| Form-A ungrasp (linear, –, compliant, grip → open) | 0 | — | `plan_dual_arm_linear_independent` | **Adapter needed**, untested |

**Picking a routine from parts, not from class names or `_M<n>_` ids:**

```
if not arms:                                   -> no motion (start = end), never a planner call
n = len(arms); acting robot = robot of arms[0]
if n == 1:                                     -> single-arm free / linear (missing)
if path == "free" and not coupled:             -> plan_free_dual_arm
if path == "free" and coupled:                 -> plan_constrained_dual_arm(active_bar = the bar held by both arms
                                                  in start.attached; derive_start = start joints are null)
if path == "linear" and coupled:               -> plan_constrained_dual_arm_linear(active_bar as above)
if path == "linear" and not coupled:           -> plan_dual_arm_linear_independent
goal: target.joints[robot] if given, else target.links[flange] (left/right by flange id), else FK(start) + line,
      else (target null) the solved start of the robot's next arm movement in schedule order
```

`controller`, `drives` and `ends_on` never select a routine. They set collision allowances (pending mates) and execution behaviour.

The headless planner's `_ROLE_RE = r"_M([0-9])_"` (`headless_bar_action_planner.py:240` @517d692, `:291` @2fce15f) is now actively wrong. The converted insert keeps the id `B10_J_M4_tool_tighten_joint`, so the regex calls it role "M4", which is home.

### 3c. Tamp output → `solutions/` field

| `solutions/` field | Tamp output | Status |
| --- | --- | --- |
| `status` `solved` / `failed` / `not_planned` | Path or `None`, plus `info["failure_reason"]` | **Works** |
| `status` `keyframe_only` | `solve_keyframe_chain` → `{role: RobotCellState}` | **Adapter needed**: the chain spans J and R, and ownership is unclear (C4) |
| `reason` | `info["failure_reason"]` from the free and constrained planners | **Works**. The linear planners return only `None`, so the adapter writes a generic reason. |
| `bases` | Keyframe base search returns a mm 4x4 | **Adapter needed** (mm → `Pose`) |
| `start` / `end` | `path[0]` / `path[-1]`; `info["derived_start_conf"]` | **Works** |
| `start_overridden` | Tamp re-derives starts on request | **Awkward**: at 1e-6, every chained linear move would count as overridden (P4) |
| `trajectory` (robot, 12 `joint_names`, `positions`) | List of 12-vectors, or `JointTrajectory`, left arm then right arm | **Works**. Names are explicit; the core's `movable_joints` order differs, which does no harm. |
| `trajectory.times` | Tamp never sets times: no `time_from_start` in any branch | **Missing** (optional). Waypoint density varies: 5 rows for 15 mm, 459 for home. The executor must retime. |
| `path_poses` (link id → flange poses) | `info["path_poses"]` holds **bar** poses (`world_from_bar`) | **Adapter needed**, or let the spec allow a body id (C5) |
| `planner` block (name, repo, commit, `ik_backend`, artifacts, settings) | Tamp knows all of these; `artifacts` is free text | **Adapter needed**. Record the ssik KinBody hash. |
| `solved_against` | `solutions.solved_against(design, action)` | **Works** for a design on disk. In-process callers with an unwritten design cannot write solutions. |
| Partial chains; write after each movement | Tamp plans movement by movement | **Awkward**: `write_solution` replaces the whole file, and there is no merge helper |
| Chaining across J→R | `solution_warnings` checks across the schedule | **Works**. It reported every IK-level gap. |

## 4. Pain points, missing features, contradictions

| # | Issue | Evidence | Severity |
| --- | --- | --- | --- |
| P1 | **Derived contacts plus compas_fab convex hulls: every carried bar collides with Cindy's tools.**<br>• The spec allows a tool to touch only "the body it is on", which is the male half (spec `:308`).<br>• The core joins all of a tool's collision shapes into one mesh, so PyBullet tests one hull.<br>• The real clearance is 1.01 mm. The hull overlaps by 1.3 mm, and VHACD parts still by 2.0 mm, so neither a single hull nor a decomposition can represent it.<br>• Effect: the insert, ungrasp and retreat starts of all 20 bars collide, and `J_M3` fails with `goal_in_collision`.<br>• The spec's own advice ("margin below 1 mm", `:319`) cannot be met with hulls.<br>• Extra pairs at the B19 and B21 inserts: the bar↔mated female halves, and the tool↔female half at their retreats. | `mirrors/compas.py:193-196` (core WT); `relations.py:358-360`; `run_*.log`, `run_nopatch.log`, `diag_exact*.log`, `diag_vhacd.log`; old export `B10__J.json` M3 `touch_bodies: ['AT3L','AT3R']` | **Blocker** |
| P2 | **Null start joints on the coupled transfer** (40/40 `J_M3`). The mirror refuses the robot. Tamp's `derive_start` exists exactly for this case, but it needs a seeded state and an attached bar. Spec rule 3 says such movements stay `not_planned` until execution (C1). | `mirrors/compas_fab.py:144-147`; `scenes.py:404-409`; `api.py:484-560` @517d692; `try_tamp.py` `J_M3` step | Major |
| P3 | **Tamp plans only dual-arm Cindy.**<br>• Groups and links are hardcoded, and `_arm_joint_names` asserts 6+6.<br>• 24 support movements (Alice, Belle) have no tamp routine.<br>• The ssik artifacts exist only for Cindy's two arms. | `api.py:53-66,84` @517d692; `keyframe/config.py:85-89` | Major |
| P4 | **The chaining tolerance of 1e-6 rad does not match planner output.**<br>• The linear IK loop ends 7.7e-4 rad from the target joints, and RRT paths end 3.3e-6 away.<br>• The IK tolerances used are 1 mm and 0.01 rad (`api.py:201,205` @517d692).<br>• So every chain warns, and `start_overridden` would be set everywhere. | spec `:484`; `solutions.py:26` (core WT); `run_*.log` | Major |
| P5 | **No status for "nothing to plan".**<br>• 140 of the 364 movements have no arm motion.<br>• `not_planned` would block an executor that wants a complete chain.<br>• `solved` with no trajectory is not specified, and neither is `start = end`. | spec `:473`; `solutions.py:174` | Major |
| P6 | **Keyframe ownership.**<br>• Rhino writes keyframe IK into the design's `target.joints`.<br>• The spec also gives planners `keyframe_only`.<br>• Tamp's re-solve lands on other branches (2π, and up to 3.1 rad modulo 2π).<br>• Which one wins is not stated, and the chain spans two actions (approach and assembled in J, retreat in R) with one base. | spec `:473`; `ik_keyframe.py:70-170` @517d692; `try_tamp.py` keyframe step | Major |
| P7 | **Linear planners skip environment collisions (CC3–5) by default**, so the insert and retreat are never checked against the structure. This hides P1 for those moves, and it also means derived pending-mate contacts never reach tamp's linear moves. | `api.py:820,937,1048` @517d692; `:1151,1277,1392,1503` @2fce15f | Major |
| P8 | **The core is not installable on its own.** `bar_assembly_core` ships inside `husky_assembly_teleop`'s ROS `setup.py` (`find_packages`), so tamp cannot depend on it without pulling in teleop. | teleop `setup.py:8`; `bar_assembly_core/requirements.txt` | Major |
| P9 | **The default IK backend is ssik on the branches the hosts run.**<br>• At 2fce15f the motion planners default to ssik, which the teleop venv lacks, so every IK call fails.<br>• The ssik keyframe cold solve needs `home_conf_12`. Schema 2 has no home, and Rhino keeps it in its own config. | `api.py:1170` @2fce15f; `dual_arm_ik.py:526` @517d692; Rhino `rs_ik_keyframe.py:1545` | Major (env and config) |
| P10 | **The headless planner cannot read schema 2.**<br>• It reads `RobotCell.json` and `BarActions/` through `rs_data_structure` classes.<br>• Its roles come from `_M<n>_` (M0–M4 of the pre-J/R era), and it fails on `__J`.<br>• Under schema 2 ids, the insert maps to "M4" (home). | `headless_bar_action_planner.py:91-96,240-272,913-1036` @517d692 | Major |
| P11 | `line` is ignored: tamp needs target frames or joints. All 96 linear moves in the data have targets, so it only matters when the target is absent. | `api.py:926-1117` @517d692 | Minor |
| P12 | `path_poses` from tamp are bar poses, not flange poses. | `api.py:713,745` @517d692; spec `:480` | Minor |
| P13 | No trajectory timing anywhere in tamp, and the density differs per planner. | grep `time_from_start` over all four refs: no hits | Minor |
| P14 | `pp_client` does not register `pp.CLIENTS[client_id]`, so `pp.LockRenderer()` raises `KeyError` inside `lend()`. The headless planner registers it itself (`:330-331`). | `mirrors/pp_client.py:30-40` (core WT); `diag_export.log` | Minor |
| P15 | Tamp's `_collect_obstacle_puids` does not filter hidden bodies: 47 of 98, parked at the origin. Harmless for the cfab-checked paths, but they enter VAMP's point cloud. | `api.py:143-156` @517d692; vamp `plan_m1_with_vamp.py:188,217-257` @93e6010 | Minor |
| P16 | PRM workers load a trimmed cell from a JSON cache keyed by `source_key`, without the design hashes the spec requires (`:110`). The mirror's cell exports fine. | `holding_prm/parallel.py:59-86`, `benchmark/scene.py:674-700` @5143725; `diag_export.log` | Minor |
| P17 | The VAMP client only launches `wsl -d …` (Windows host), and its script reads `RobotCell` and `BarActions`. | `vamp_backend/client.py:58`, `plan_m1_with_vamp.py:160-186` @93e6010 | Minor |
| P18 | `write_solution` replaces the whole file, with no per-movement merge. `writer.library` still says `design_io`. | `solutions.py:111-132` (core WT) | Minor |
| P19 | The ssik artifact identity is unrecorded. Artifacts and the design URDF match today, but the monitor plans on StockUrFrames, which the artifacts do not fit. | `asset/ssik/left_ur_arm_ik.py:8` @5143725; `try_tamp.py` ssik step; shared report #7 | Minor |
| P20 | Tamp bugs and defaults hit during the integration:<br>• `_conf12_from_target` raises `IndexError` on numpy goals.<br>• `plan_free_dual_arm` defaults to `max_iterations=20`, too low for home. | `api.py:96-109,413` @517d692 | Minor |
| P21 | Dependencies:<br>• `rs_data_structure` is pinned at ce01ca0, but only the headless script (and the PRM benchmark's provenance) imports it.<br>• compas_fab floats on `wip_process`; the hosts use 995122d.<br>• networkx is imported but undeclared.<br>• main's config says ssik needs 3.11, while the branches install ssik ≥4.1 on 3.10. | `setup.py:16,21` @517d692; `holding_prm/search.py:31`, `pilot.py:40`, `setup.py:37` @5143725; `keyframe/config.py:66` @517d692 | Minor (but urgent) |
| P22 | The branches have diverged:<br>• `yh/holding-prm` has 2fce15f but not 147d767.<br>• `yh/ssik-speedup` has 147d767.<br>• `yh/vamp_integration` has neither.<br>Rhino pins 517d692 and teleop pins 2fce15f. | `git merge-base` | Coordination |

**Contradictions**

| # | Between | What | Evidence |
| --- | --- | --- | --- |
| C1 | Spec rule 3 vs principle 6 vs tamp | Rule 3: a movement that starts unknown is planned at execution and stays `not_planned`. Principle 6: `null` means "not decided by the design", so the planner may decide. Tamp derives the `J_M3` start offline; that is its main M1 feature (147d767, 02ea290, 55b585a). The `J_M1`/`J_M2` nulls follow a manual mount and a grasp that do not move the arms, so nothing is unknown there. | spec `:42,:143`; `try_tamp.py` |
| C2 | Spec contact rule vs the core's collision backend | The spec narrows contacts to the half (`:308`) and warns about margins (`:319`). The core's own mirror then reports 120 false pairs in 260814 (60 movements). The draft's broad rule, "tool with its whole part", had hidden them. | teleop `tasks/2026-10-08_shared_core.md:364-370`; `specs/schema2_implementation_status.md`; `diag_exact2.log` |
| C3 | `target: null` vs a planner | "End where the next movement starts", but for `J_M0` the next start (`J_M1`) is null too. The goal is the solved start of the next *arm* movement (`J_M3`), which the spec does not say. | spec `:76,:144`; data census |
| C4 | Design `target.joints` vs `solutions/ keyframe_only` | Both claim the keyframe joints. There is no precedence rule. | spec `:32-35` (principle 3) vs `:473` |
| C5 | `path_poses` "flange path" vs tamp | Tamp has the bar path. Showing the attached bar is the stated purpose, and the bar path serves it directly. | spec `:480`; `api.py:713,745` |
| C6 | Shared report step 5 vs schema 2 | "Tamp keeps reading `RobotCell.json`/`BarActions/` written from the mirror's cell". No such writer exists: the core converts old exports to schema 2, never back. Tamp must read design folders. | `specs/shared_core_report.md` step 5 |
| C7 | Shared report vs tamp branches | "ssik via a Python 3.11 sidecar" holds only for Rhino. Tamp branches run ssik ≥4.1 in-process on 3.10 and default to it. | `setup.py:37` @5143725 |
| C8 | Shared report blocker #3 vs now | Resolved for the API: the mirror builds `ToolModel`s on groups, and tamp accepts `bars/…` ids. Only the headless planner still builds `bar_<id>`. | `try_tamp.py` cell checks |

## 5. Change requests (prioritized)

| # | To | Change | Reason | Smallest fix |
| --- | --- | --- | --- | --- |
| CR1 | Spec + core `relations` | **A tool may touch the part it is on (bar plus mounted halves).** Also measure whether the B19/B21 bar↔female-half pairs are hull artifacts too, and if so allow pending mates' bar↔half. | P1/C2: 1.01 mm clearance cannot be represented by hulls or VHACD. Tamp otherwise fails every carried state. | In `allowed_contacts`, for `on` add the pair (tool, `bar_of(on)`) and (tool, each half of that bar). Spec `:308`: "body it is on" → "part it is on". |
| CR2 | Spec rule 3 + core | **Separate "planner decides" from "measured at execution".** A null start that the previous non-arm movements carry over (mount, grasp) is the planner's to choose. The planner writes `start` (and `solved`) offline. Rule 3 then applies only after steps that may move the robot (operator relocation). | P2/C1: tamp's start derivation. | Spec: one sentence. Core: a `seeded(scene, robot_id, joints)` helper that sets joints and clears `unmeasured`, fed from `carry.assumed_start_all`, so adapters stop hand-patching `RobotObject`. |
| CR3 | Spec + `solutions` | **No-motion results:** a movement without `arms` is `solved` with `start = end` and `trajectory: null`, written by whoever plans the action. | P5: complete chains for the executor. | Spec sentence plus a `MovementResult.still(joints)` constructor. |
| CR4 | Spec + `solutions` | **Chaining tolerance 1e-3 rad**, matching the IK tolerances, or a per-planner tolerance in the planner block. When the design gives target joints, a planner snaps its last waypoint to them. `start_overridden` uses the same tolerance. | P4 | `CHAIN_TOLERANCE = 1e-3`; spec `:484`. |
| CR5 | Spec | **Keyframe precedence:** the design's `target.joints` and bases win when present. `keyframe_only` and `bases` in `solutions/` only fill what the design leaves `null`. A keyframe chain may span the J and R actions of one bar and shares one base. | P6/C4 | Two sentences in "Solutions and runs". |
| CR6 | Core packaging | **Make `bar_assembly_core` its own distribution** (pyproject with extras `[mirrors]` = compas, compas_fab, pybullet, pybullet_planning), versioned and pinnable. | P8: tamp `[design]` must be able to depend on it. | `bar_assembly_core/pyproject.toml` (or its own repo) and a CI install job on 3.9 and 3.10. |
| CR7 | Spec `target: null` | "The solved start of the robot's next arm movement in schedule order." | C3 | One sentence. |
| CR8 | Core `pp_client` | Register `pp.CLIENTS.setdefault(client_id, None)` for the block and restore it after. | P14 | 2 lines. |
| CR9 | Core | **Exports for out-of-process planners:**<br>• `CompasFabMirror.export()` → (cell JSON, state JSON) for PRM workers;<br>• `scene_obstacles(scene, robot_id)` → enabled bodies (mesh, pose in the robot root frame) and carried bodies (link, offset) for VAMP.<br>Cache keys use `solved_against`. | P15, P16, P17; spec `:110` | Two small functions; JSON export already tested. |
| CR10 | `solutions` | `update_solution(design, action_id, results, planner)`: merge per movement, keep the others. `path_poses` keyed by a link **or body** id. `writer.library` → `bar_assembly_core`. | P12, P18, C5 | Small. |
| CR11 | Spec planner block | `artifacts` names the IK artifact identity: ssik KinBody hash per arm plus the content hash of the URDF it was checked against. | P19 | Spec wording; tamp fills it. |
| CR12 | Tamp (own repo) | • Declare networkx and ssik; pin compas_fab 995122d.<br>• Move `rs_data_structure` to a `[legacy]` extra.<br>• Raise `plan_free_dual_arm`'s default `max_iterations`.<br>• Fix the `_conf12_from_target` `IndexError`.<br>• Skip hidden bodies in `_collect_obstacle_puids`.<br>• Turn env checks on for retreats, or document why not. | P7, P15, P20, P21 | Small PRs. |

## 6. Integration proposal

**Dependency direction.** Tamp's planners stay independent of the core. `motion_planner/` and `keyframe/` already take the right inputs: a loaded compas_fab planner, a `RobotCellState` and goals. They import neither the core nor `rs_data_structure`.

- Add one optional subpackage, `husky_assembly_tamp.design`, behind the extra `[design]`, which depends on `bar_assembly_core[mirrors]` (CR6).
- The core never imports tamp: it must stay Python 3.9-clean for Rhino and planner-agnostic.
- The adapter lives in tamp, not in the core or the hosts. Tamp owns which routine plans which parts, its settings and failure reasons, start derivation and IK backends, and it is the writer of its `solutions/`.
- Hosts (the monitor's bar_action, Rhino later) call the adapter in-process with a mirror they own. Batch runs use the CLI.

**Entry points** (in `husky_assembly_tamp.design`):

1. `plan_movement(mirror, design, action_id, movement_id, *, start=None, settings) -> MovementResult`
   - Builds `scene_at`, seeds a null start (from `start`, the previous solved end, or `assumed_start_all`), then runs `mirror.sync` and `lend()`.
   - Picks the routine from the parts (table 3b) and builds the goal from `target.joints`, `target.links`, `line`, or the next solved start.
   - Returns a `MovementResult`: `still()` for movements without arms, `failed` with a reason, or `solved` with a 12-joint trajectory and `path_poses`.
2. `plan_action(design, action_id, *, mirror=None, settings, chain_from=None) -> Solution`. Plans the movements in order, chaining starts, and writes after each movement (`update_solution`). It plans `J_M3` before `J_M0`: `J_M0` stays `not_planned` (live) but its goal is now known.
3. `solve_keyframes(design, bar, *, base=None, settings) -> {movement id: MovementResult(keyframe_only)}`. Runs the approach, assembled and retreat chain across J and R. It samples a base from `ground/` bodies only where the design's base is null, and contains the mm shim.
4. CLI `python -m husky_assembly_tamp.design plan <design folder> [--action ID | --all] [--keyframes]`. This replaces the headless planner's I/O.

**What stays in tamp:** the motion planners, the task-space RRT and start derivation, ssik and gradient IK with their artifacts, base sampling, home configurations (as settings, recorded in `planner.settings`), holding PRM, VAMP, benchmarks and the replay tools.

**Headless planner migration.**

- Freeze `scripts/headless_bar_action_planner.py` as legacy for old exports. Old data goes through the core's `scripts/convert_design.py` instead.
- The new CLI reads design folders, selects routines by parts (no `_M<n>_`, no rs classes), and writes `solutions/` in place of the `.solved_keyframe.json`, `.solved_motion.json` and `.live-solved.json` sidecars.
- `replay_bar_action_plan.py` reads `solutions/` through `read_solutions` and `scene_at`.

**Order of steps**

| # | Step | Done when | Effort |
| --- | --- | --- | --- |
| 0 | Tamp hygiene (CR12): pins, extras, defaults, hidden-body filter. Agree on one integration branch, since `holding-prm`, `ssik-speedup` and `vamp_integration` have diverged. | `pip install -e .[design,ssik]` works in the teleop venv, and `try_tamp.py` passes without `HUSKY_IK_BACKEND` | 1 d |
| 1 | Core: CR1 (part contacts), CR8, CR6 (packaging) | `B10_J_M4` start has 0 collisions with derived contacts; `J_M3` plans without a touch patch; `pip install bar_assembly_core[mirrors]` works on 3.9 and 3.10 | 1–2 d |
| 2 | Spec decisions CR2–CR5, CR7, CR11 | `doc/design_format.md` updated; `solution_warnings` uses the new tolerance | 0.5 d plus review |
| 3 | `plan_movement` for Cindy's five arm kinds plus no-motion | All 40 J/R actions of 260814 and 260920 give a result per movement. Failures are only real planning failures, each with a reason. | 2–3 d |
| 4 | `plan_action` plus `update_solution` plus chaining | `solution_warnings` is empty for all Cindy actions of 260814. `not_planned` appears only for `J_M0` and the support moves. | 1–2 d |
| 5 | `solve_keyframes` across J→R with the mm shim and base search | For bars whose base the design gives, the results match Rhino's targets within tolerance, or the branch difference is reported. Null bases get `bases`. | 2 d |
| 6 | New CLI; legacy headless frozen; replay reads `solutions/` | `--all` runs on both converted designs headless | 1–2 d |
| 7 | PRM and VAMP fed from `mirror.export()` and `scene_obstacles` (CR9), with hash-keyed caches | The PRM pilot runs from a design folder; VAMP builds its cloud with no hidden bodies | 2–3 d |
| 8 | Decide on single-arm support planning (tamp or Rhino) | A decision recorded. If tamp: an N-arm generalisation of `api.py` (groups from `arms`) | 3–5 d, if chosen |

**Rough total:** about 2 to 2.5 weeks for one person for steps 0–6, plus about a week for 7–8.

**Housekeeping.** I created four tamp worktrees in the scratchpad (`tamp_wt_main`, `tamp_wt_prm`, `tamp_wt_ssik`, `tamp_wt_vamp`), which added entries under the tamp repo's `.git/worktrees`. Remove them with `git -C …/husky_assembly_tamp worktree remove <path>` when done.
