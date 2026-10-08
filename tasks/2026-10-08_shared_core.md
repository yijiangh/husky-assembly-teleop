# 2026-10-08 Shared core: `bar_assembly_core`

Status: **steps 1–5 and the refactor (§10) implemented**; report in §9. Brief: the core extraction prompt (decisions 1–16 there are fixed).

## 1. Goal

Extract `design_io`, the scene and the mirrors into a top-level package `bar_assembly_core/` next to
`husky_assembly_teleop/`, and move the monitor onto it. Later the Rhino plugin and `husky_assembly_tamp` use the
same package; its move to its own repository must be a copy.

## 2. Answers from the user (planning)

| # | Question | Answer |
|---|---|---|
| A1 | Package name | `bar_assembly_core` |
| A2 | Equivalence exports | `$HUSKY_DRIVE_ROOT/data_design_study/260814_RobArch_support_ik` and `.../260920_RobArch_demo_revamp_backup` |
| A3 | `pp.set_client` does not redirect pybullet_planning: 17 of its modules bind `CLIENT` at import (`from pybullet_planning.utils import CLIENT`); checked: after `set_client(b)`, `pp.get_bodies()` still reads client 0. Today's `PyBulletMirror.active()` (`pp.CLIENT = …`) has the same flaw. | `lend()` calls `set_client` **and** rebinds `CLIENT` in every loaded `pybullet_planning` module, restoring both after. No change to `external/`. |
| A4 | FK provider needs yourdfpy (not in the numpy/scipy/trimesh list) | yourdfpy, imported lazily by `bar_assembly_core.kinematics`; `import bar_assembly_core.design_io` stays numpy/scipy/trimesh only. |

## 3. Rules for the package

| # | Rule | Checked by |
|---|---|---|
| R1 | Imports nothing from `husky_assembly_teleop`, `rclpy`, `viser`, `crl_husky` (also not under `TYPE_CHECKING`). | `test/test_core_isolation.py` (AST scan) |
| R2 | Python 3.9: every module has `from __future__ import annotations`; no `match`, no runtime `X \| Y`, no `slots=`/`kw_only=`, no `zip(strict=)`. | `test/test_core_py39.py` (syntax for all files; `uv run --python 3.9` import of `bar_assembly_core`, `.design_io`, `.mirrors`, `.mirrors.pybullet`, `.mirrors.compas_fab`) |
| R3 | Core dependencies numpy, scipy, trimesh==4.12.2. Extras: yourdfpy (`kinematics`, so `scene_at`), compas, compas_fab, compas_robots, pybullet, pybullet_planning (`mirrors`, `design_io.compas_fab`, `design_io.legacy`). Listed in `bar_assembly_core/requirements.txt`. | `import bar_assembly_core.design_io` without compas (existing T1) |

## 4. Package layout (steps 1–5; superseded by the layout in §10)

```
bar_assembly_core/
├── __init__.py            package rules (R1–R3); imports nothing heavy
├── requirements.txt
├── design_io/             moved from husky_assembly_teleop/design_io (step 1)
│   ├── types.py           + Design.scene_at(movement), Design.scene_after(bar)  (delegate to scenes.py)
│   ├── scenes.py          scene_at / scene_after, design_model(design, robot_id) cached by content (step 4)
│   └── compas_fab.py      to_robot_cell / to_cell_state: legacy test harness only
├── ur.py                  UR_JOINT_NAMES, TOOL_TOUCHES_ARM_LINKS, stock_frame_problem  (ROS-free)
├── robot.py               RobotModel (frozen, identity), robot_model(...), RobotObject (validity flags)
├── scene.py               Attachment, Body, SceneSnapshot, same_source, world_poses(...)
├── kinematics.py          ForwardKinematics: link_pose(urdf, base, joints, link)
├── ids.py                 IdMap (explicit; refuses unmapped), retarget(attachments, robot_map, models)
├── hold.py                release_bar(design, action_id), hold_scene(scene, bar_id)
└── mirrors/
    ├── __init__.py        check_display
    ├── compas_convert.py  frame_from_pose, pose_from_frame, load_model, rigid_body, subtree, filled,
    │                      PARKED_POSITION, FileMesh (step 1); tool_model, robot_as_tool, planning_group (step 3)
    ├── pp_client.py       pp_client(client_id): pybullet_planning pointed at one world (step 5)
    ├── pybullet.py        PyBulletMirror
    └── compas_fab.py      CompasFabMirror (+ lend), SearchCheck
```

Stays in the monitor: `RobotConfig` (gains `model: RobotModel`), `robot_interface`, the tick, `Scene` /
`PluginScene` / `take_snapshot` (in `world/scene.py`), `Kinematics` (live, stitched URDF), plugins, viser drawing,
`config.py`, `tool_urdfs.py` (stitching for drawing and the live model; builds live tools as `ToolSpec`s).

## 5. Model decisions made while planning

| # | Decision | Why |
|---|---|---|
| P1 | `RobotModel(name, urdf, srdf, flanges, tools: {flange: ToolSpec}, tool_touches: {flange: link names}, stock_ur_frames)`; frozen, `eq=False`. `urdf` never contains tools. | Decision 7/8: mirrors build tools as separate objects. Reuses `ToolSpec` (geometry and TCP in the flange frame). |
| P2 | Design models are cached by content (`urdf`, `srdf`, flange→`ToolSpec`); `ToolSpec` hashes its `Geometry` by identity. | Decision 3: an unchanged model keeps its object across `scene_at` calls and `dataclasses.replace(design, …)`. |
| P3 | A live robot's model: the bare calibrated `_StockUrFrames` URDF (meshes made absolute by `stitch_tools(…, tools={})`) plus each mounted tool as a `ToolSpec` built from its tool URDF (collision meshes at zero joints, TCP at tool0). | Decision 8: compas_fab cells (tamp) need `ToolModel` tools; the stitched URDF stays for drawing and `Kinematics`. |
| P4 | `PyBulletMirror` loads a robot from a URDF it writes once per `RobotModel`: the model's URDF plus one fixed link per tool, built from the tool's collision shapes. | Planners move robots with raw `p` calls (base planner); tools must stay links of the robot body. |
| P5 | `RobotObject(id, model, base, joints, enabled=True, base_tracked=True, unmeasured=frozenset(), base_time=None, joints_time=None, label="")`. `unmeasured` only lists the model's movable joints (SRDF-passive wheels and mimic joints excluded). `acting_problems(robot)` lists why it can't act. | Decision 5. Wheels and gripper fingers are never measured; counting them would refuse every real robot. |
| P6 | Planned robot with `joints: null` in the design: joints at zero, all movable joints `unmeasured`. The cell plugin fills its assumed joints (`assumed_start_all`) as a scene edit. | `scene_at` reports the design as authored; a planner refuses an acting robot with unknown joints. |
| P7 | Scene type keeps the name `SceneSnapshot(tick, time, bodies, world_poses, robots)`; `robots` keyed by robot id (`robots/a200-0806`), tracked objects are `Body`s `tracked/<name>`. `world_poses(bodies, robots, link_pose)` resolves attachments to robot links or to other bodies. | Smallest diff; decision 2 and the "tracked objects as Body" step. |
| P8 | Mounted tools keep their `ToolSpec.id` as their id in collision results and `touches` (`tools/AT3L`). `retarget` swaps robot ids; the cell plugin maps a planned tool id to the real model's tool on the same flange. | Matches the design's ids and compas_fab cell keys. |
| P9 | `scene_after(bar)`: the start state of the first movement after the last action whose `bar` is `bar`; for the schedule's last action, its last movement's start with the target joints applied. | "Once that bar is built, following the schedule". |
| P10 | `lend()` is a context manager yielding the `PyBulletPlanner`; it sets the dirty flag, and the next `sync` rebuilds the cell if compas_fab's cell is no longer ours, writes the full state and recomputes static contacts. | Decision 15; tamp may call `set_robot_cell_state` or leave joints anywhere. |

## 6. Steps

| # | Work | Done when |
|---|---|---|
| 1 | `git mv husky_assembly_teleop/design_io bar_assembly_core/design_io`; move mirror helpers to `bar_assembly_core/mirrors/compas_convert.py`; fix imports in monitor, scripts and tests. | Quick + slow tests pass; nothing imports `husky_assembly_teleop.design_io`. |
| 2 | `ur.py` (UR_JOINT_NAMES, TOOL_TOUCHES_ARM_LINKS); `kinematics.py` from `world/kinematics.py`'s yourdfpy code; move scene data types and both mirrors (duck-typed on `RobotConfig` for now); R1 test; R2 test with Python 3.9 via uv. | `python3.9 -c "import bar_assembly_core, bar_assembly_core.mirrors"` works without ROS; quick + slow pass. |
| 3 | `RobotModel` / `RobotObject`; `RobotConfig.model`; snapshot by robot id; tracked objects as `Body`; one identity test in both mirrors; mirrors and planners refuse invalid acting robots; tools as `ToolModel`s in compas_fab cells (P3/P4). | Monitor runs `robot_control`, `cell`, `base_planner`, `arm_planner` as before; planner tests pass; quick + slow pass. |
| 4 | `scene_at`, `scene_after`, held attachments, `retarget`, absent robots, `hold.py`; cell plugin puts design scenes; equivalence script gets a `mirror` loader. | Equivalence on both exports: mirror pairs = design pairs except floor pairs (and pairs `static_contacts` hides, reported apart); quick + slow pass. |
| 5 | `CompasFabMirror.lend()` (A3, P10); `PyBulletMirror.active()` uses the same rebinding. | Slow test: mirror on a client other than 0, `sync`, tamp `plan_free_dual_arm` on the lent planner, second `sync` writes the full state, pp client restored everywhere. |

## 7. Schema 2 candidates (collected while working)

- `connections` (decision 10).
- Tool state per `State` (a tool's opening, which tool is mounted).
- Typed `notes` (format Q8).
- `solutions/` files.

## 8. Report

§9, written for the maintainers of `bar_joint_rhino_design_workflow` and `husky_assembly_tamp`.

## 9. Report: the shared core, for the Rhino and tamp reviewers

`bar_assembly_core/` sits next to `husky_assembly_teleop/` in this repository. It imports nothing from the monitor,
ROS, viser or crl_husky (`test/test_core_isolation.py`), and it runs on Python 3.9 (`test/test_core_py39.py`).
Moving it to its own repository is a copy of the folder.

### 9.1 Public API

Layers, each importing only the ones above it (`bar_assembly_core/__init__.py`). The core layers need numpy, scipy
and trimesh==4.12.2. **yourdfpy** is needed by `kinematics` and `design.scenes`, so by `Design.scene_at` and
`scene_after`. `mirrors` needs pybullet, pybullet_planning, compas, compas_fab 1.1.0 and compas_robots, and
`legacy` needs compas and rs_data_structure. The list is in `bar_assembly_core/requirements.txt`.

| Module | Main types and functions |
|---|---|
| `geometry` | `Pose(position, orientation)` (frozen, by value), `compose(a, b)`, `invert(a)`; `TriMesh`, `BoxShape`, `CylinderShape`, `Geometry(visual, collision)` (by identity), `box_geometry`, `cylinder_geometry`, `shape_mesh(shape)`. |
| `ids` | `ID_PATTERN`, `check_id`, `ROBOTS`, `TOOLS`, `TRACKED`, `robot_id(name)`, `tracked_id(name)`, `link_id(robot, link)`, `split_link_id(id)`; `IdMap(pairs)`: `ids(id)` (KeyError if unmapped), `id in ids`, `ids.get(id)`, a mapped robot maps its links. |
| `urdf` | `urdf_links`, `urdf_joints`, `movable_joints(urdf, srdf)`, `srdf_group_tips`, `resolved_urdf_text`, `copy_robot`; UR conventions `UR_JOINT_NAMES`, `TOOL_TOUCHES_ARM_LINKS`, `stock_frame_problem(urdf, arm)`. |
| `kinematics` | `ForwardKinematics().link_pose(urdf, base, joints, link) -> Pose`; `load_urdf(path)`. One instance per thread. |
| `robot` | `Tool(id, geometry, tcp, kind, touches)` (geometry and TCP in the flange frame); `RobotModel(name, urdf, srdf, flanges, tools, tool_touches, stock_ur_frames)`: frozen, by identity; `.links`, `.movable_joints`. `robot_model(name, urdf, srdf, tools=None, touches=None)`. `RobotObject(id, model, base, joints, enabled=True, base_tracked=True, unmeasured=frozenset(), base_time=None, joints_time=None, label="")`, `.copy()`, `.acting_problems() -> list[str]`. |
| `scene` | `Attachment(parent, link, grasp)` (parent: robot id or body id); `Body(id, geometry, placement, touches=(), label="", color=None, enabled=True)`, `.copy()`; `Scene(tick, time, bodies, world_poses, robots)`, `.copy()`, `.label(id)`; `world_poses(bodies, robots, link_pose)`; `same_source(new, old)`; `retarget(attachments, robot_map, models)`. |
| `design` | The format: `read(folder)`, `write(design, folder)`, `validate(design)`, `Design` (`.scene_at(movement)`, `.scene_after(bar)`, `.movements()`), `RobotSpec`, `BodySpec`, `State`, `Movement`, `Action`, `Target`, `Attached`, `DesignError`, `SchemaMismatch`; `assumed_joints`, `assumed_start_all` (`design.carry`). |
| `design.scenes` | `scene_at(design, movement)`, `scene_after(design, bar)`, `design_model(design, robot_id) -> RobotModel` (cached by content). |
| `design.hold` | `release_bar(design, action_id)`, `hold_scene(scene, bar)`, `hold_scene_for(design, action_id)`. |
| `mirrors.compas_fab` | `CompasFabMirror(robot_id, log=None)`: `.sync(scene)`, `.collisions(joints=None, full_report=False)`, `.search_check(joint_names)`, `.lend()`, `.state`, `.cell`, `.static_contacts`, `.configuration(joints)`, `.state_at(joints)`, `.set_gui(gui)`, `.close()`. |
| `mirrors.pybullet` | `PyBulletMirror()`: `.sync(scene)`, `.collisions(robot_id, margin=0.0, candidates=None)`, `.allowed(a, b)`, `.robot(robot_id)`, `.robots`, `.body_ids(id)`, `.id_of(pybullet_id)`, `.obstacle_ids()`, `.active()`, `.set_gui(gui)`, `.close()`. |
| `mirrors.compas` | `frame_from_pose`, `pose_from_frame`, `rigid_body(geometry)`, `load_model(urdf, visual=True)`, `tool_model(tool, name)`, `robot_as_tool(urdf, tools, name, visual=True)`, `planning_group(cell, flange)`, `filled`, `subtree`, `PARKED_POSITION`. |
| `mirrors.pp_client` | `pp_client(client_id)`: pybullet_planning acts on that world inside the block; restored after. |
| `legacy` | `export.read_legacy(folder, robot_files)`, `export.load_export`, `conversion.convert_export`; `compas_fab.to_robot_cell` / `to_cell_state` (the reference for `scripts/legacy_equivalence.py`). |

Ids: one canonical id per object. Robots are `robots/cindy` when planned, `robots/a200-0806` when measured. Links are
`robots/<name>/<link>`. Mounted tools use their tool id (`tools/AT3L`; a live one is `tools/a200-0806/left_ur_arm`).
Bodies use design ids (`bars/B1`). Measured objects are `tracked/<name>`.

### 9.2 What a host does

```python
from bar_assembly_core.design import read
from bar_assembly_core.design.hold import hold_scene_for
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror

design = read(folder)                                     # 1. models: design robots get theirs inside scene_at
scene = design.scene_at(movement)                         # 2. the world at one movement (or design.scene_after(bar))
robot = scene.robots["robots/cindy"]                      # 3. edit the scene, never the design
robot.joints.update(seed); robot.unmeasured = frozenset() #    e.g. fill joints the design leaves null
scene.bodies["bars/B3"].enabled = False                   #    e.g. disable the held bar for a plan
mirror = CompasFabMirror("robots/cindy")                  # 4. one mirror per acting robot, one thread
mirror.sync(scene)                                        #    rebuilds the cell only for new models or geometry
with mirror.lend() as planner:                            #    pybullet_planning points at the mirror's world here
    path, info = plan_free_dual_arm(planner, mirror.state, goal)
mirror.sync(next_scene)                                   #    writes the full state again
```

- **Build a `RobotModel`**: from a design robot, `design.scenes.design_model(design, robot_id)`, which `scene_at`
  already uses. Otherwise `robot_model(name, urdf, srdf, tools={flange: Tool}, touches={flange: [link, …]})`: the
  URDF without tools; each tool's geometry and TCP are in its flange frame. Build each model once and share it:
  mirrors compare models by identity, and a new object means a rebuild (~2 s).
- **Get a scene**: `design.scene_at(movement)`, `design.scene_after(bar)`, or `hold_scene_for(design, action_id)`.
  Scenes share nothing mutable with the design, only `Geometry` and `RobotModel` objects.
- **Edit it**: change `RobotObject` and `Body` fields in place, or start from `scene.copy()`. After moving a robot or
  changing a placement, recompute `scene.world_poses = world_poses(scene.bodies, scene.robots, link_pose)`.
- **Sync and lend**: `mirror.sync(scene)`, then `collisions`, `search_check` or `lend`. A robot with
  `acting_problems()` (absent, base untracked, unknown joints) is refused with `ValueError`. Inside `lend()` only
  one pybullet_planning user may run in the process; after it, sync before using the mirror again.

### 9.3 Decisions not followed as written

| # | Decision | What was done | Why |
|---|---|---|---|
| 15 | `lend()` calls `pybullet_planning.set_client` and restores the previous client. | It calls `set_client`, **and** sets `CLIENT` in every loaded `pybullet_planning` module, then restores each one (`mirrors/pp_client.py`). The user approved this. | 17 pybullet_planning modules copy `CLIENT` at import (`from pybullet_planning.utils import CLIENT`). After `set_client(b)`, `pp.get_bodies()` still reads client 0. Tamp has worked so far only because its world happened to be client 0. The old `PyBulletMirror.active()` (`pp.CLIENT = …`) had the same flaw and now uses `pp_client` too. A fix upstream (`get_client()` at call time) would make the rebinding unnecessary. |
| 9 | `retarget(attachments, robot_map)` | `retarget(attachments, robot_map, models)` | Refusing a link the target lacks needs the target's model. `robot_map` is an `IdMap`. |
| 7 | Move `TOOL_TOUCHES_ARM_LINKS` onto the model. | The constant moved to `urdf.py`. `robot_model()` resolves it into `RobotModel.tool_touches` (per flange, link names), together with the design tool's own `touches`. | The model carries the result; the convention stays shared. |
| 5 | Refuse an acting robot whose joints are unmeasured. | Only the model's movable joints count (`RobotModel.movable_joints`: not fixed, not SRDF-passive, no mimic). | Wheels and gripper fingers are never measured; counting them would refuse every real robot. |
| 1 | The design stays an ordinary editable object. | Unchanged: the frozen dataclasses edited with `dataclasses.replace`, as before. No versioning. | — |
| Facts | "`<arm>_base_link` differs by 90°" between the URDF variants. | Measured: Alice differs at `ur_arm_base_link` (90°); Cindy only at `right_ur_arm_base` (90°). All other links agree (`test/test_core_scene.py::test_frame_convention_and_fk_comparison`). | Cindy's turn sits in another joint. `RobotModel.stock_ur_frames` records the convention. |
| Step 1 | Move `design_io` unchanged. | Moved, then split by layer in §10: the format is `design/`, the old export `legacy/`, poses and shapes `geometry.py`. They now live with the converter in `legacy/`. | The brief asked for the helpers to move. Only the legacy modules depend on `mirrors`. |
| — | Scene type name. | `Scene` (the monitor's live store is `LiveScene`). | Renamed in §10, before anyone outside depends on it. |

Further changes reviewers should know about:
- **Tools are separate everywhere in planning.** A live robot's `RobotModel` holds its tools as `Tool`s read from
  the tool URDFs (collision meshes at zero joints, TCP at tool0, because the monitor knows no TCP). In compas_fab they
  are `ToolModel`s attached to the SRDF group ending at their flange. `PyBulletMirror` writes one URDF per model with
  each tool as a fixed link, so raw `p` moves keep it on the robot. `SearchCheck` now carries attached tools like
  held bodies (tested equal to `check_collision`). The stitched tool URDFs remain only for drawing and the monitor's
  `Kinematics`.
- **The monitor's base planner no longer plans from an untracked base** (decision 5); it reports why instead.
- **The cell plugin**: held bodies now follow the configured real robot (`retarget`), and touches name the real robots
  and tools. A grasp the real robot can't take (a missing link) is refused, reported in the panel, and the bodies are
  put without the robots.

### 9.4 Flag: `static_contacts` hides contacts

`CompasFabMirror` still allows any pair of a *stationary* tool (another robot) and a body that already touch at
`sync`. One change: the acting robot's own tools are never allowed this way, because they move with it. With real
robots in a scene, a real robot pressing into a body would be hidden from every other robot's plans. The equivalence
runs list these pairs separately ("pairs allowed by static_contacts"); counts are in §9.6. Nothing else was changed.

### 9.5 Schema 2 candidates and deferred work

Schema 2 candidates:
- `connections` (decision 10): bar with joint halves, mated halves.
- Tool state per `State` (opening; which tool is mounted).
- Typed `notes` (`lm_distance_mm`, `approach_axis`, …).
- `solutions/` (planner results).
- Tool `touches` are robot-specific ids (`robots/cindy/left_ur_arm_wrist_2_link`), so one tool spec can't be shared
  between robots. Link names relative to the mounting robot would remove that.
- The design's robot URDFs are the non-stock variants. Either require stock UR frames in designs, or record the
  convention in `design.json`.

Deferred or stubbed:
- Commit is still a stub in both planners.
- No limit on the skew between `base_time` and `joints_time` yet.
- After an edit, the owner must recompute `world_poses`; the core does not track edits.
- Moving touches onto real robots lives in the cell plugin (`scene_bodies`); the core has `retarget` for attachments only.
- The core FK takes a URDF path; a model whose tools move would need more.
- The monitor was started with `robot_control`, `cell`, `base_planner` and `arm_planner` and ticks without errors
  (first tick 9 ms), but not on hardware: no plans were run through the UI.
- `uv` is not installed on this machine, so `test_core_py39.py` skips without it. The runs below used uv 0.12.23
  installed to a scratch folder.
- Not touched: the Rhino and tamp repositories, `external/` and its pins, VAMP/PRM export, IK, the file format.
- Tamp: `_conf12_from_target` catches `TypeError`/`KeyError` but not the `IndexError` a numpy goal raises, so pass
  goals as lists or `Configuration`s.

### 9.6 Test results

| Check | Result |
|---|---|
| Quick set and linters (`pytest -m "not slow"`, with `HUSKY_DESIGN_DIRECTORY` = 260814 export and uv on PATH) | 193 passed, 2 skipped (copyright stub; equivalence test needs `DESIGN_IO_EQUIVALENCE_EXPORT`, run by hand below) |
| Slow set (`pytest -m slow`) | 7 passed, including `test_compas_fab_lend.py` (tamp `plan_free_dual_arm` on a lent planner, on a client other than 0) |
| Python 3.9 (`uv run --python 3.9`, no PYTHONPATH, so no ROS) | `import bar_assembly_core, bar_assembly_core.mirrors` (and every core module) works: prints `3.9.25`, rclpy not loaded. The pure core imports with numpy, scipy and trimesh alone; `Design.scene_at` runs under 3.9. |
| Equivalence, 260814_RobArch_support_ik | 0 unexpected kinds. Mirror (`scene_at` → `CompasFabMirror`) vs `to_cell_state`: 137 of 140 movements identical; the other 3 differ only by pairs `static_contacts` allows (`bars/B5`–`robots/cindy` in Alice's `B3_H_M1..M3`). No floor-pair differences. Pairs: mirror 22, design 25. |
| Equivalence, 260920_RobArch_demo_revamp_backup | Same: 0 unexpected, 137/140 identical, the same 3 `static_contacts` pairs, no floor differences. Pairs: mirror 20, design 23. |
| Monitor | Starts with `robots:=['0804','0806']` and the four plugins, ticks without errors (no hardware). |
| Timing (260814, 94 bodies) | `scene_at` 3.0 ms per movement (run only when the design or step changes); scene copy 0.055 ms; `scene_after` 0.2 ms. |

## 10. Refactor before the hand-off (user request, 2026-10-08)

Goal: a minimal core whose layout reads as its layering. Answers: layered layout; legacy converter inside the core as
`legacy/`; `SceneSnapshot` → `Scene` (monitor's store → `LiveScene`), `ToolSpec` → `Tool`; `design_io` → `design`
(the `library` value written into design.json stays "design_io": no format change).

```
bar_assembly_core/              each layer imports only the ones above it
├── geometry.py    Pose, compose, invert, TriMesh, BoxShape, CylinderShape, Geometry, shape_mesh   (pose + geometry)
├── ids.py         ID_PATTERN, check_id, link_id, split_link_id, robot_id, tracked_id, IdMap
├── urdf.py        URDF/SRDF reading and copying, UR conventions                                (robot_files + ur)
├── kinematics.py  ForwardKinematics (yourdfpy)
├── robot.py       Tool, RobotModel, robot_model, RobotObject
├── scene.py       Attachment, Body, Scene, world_poses, same_source, retarget
├── design/        the file format: types, read, write, validate, meshes, version; carry (assumed joints),
│                  scenes (scene_at, scene_after; yourdfpy), hold
├── mirrors/       compas.py (conversions), compas_fab.py, pybullet.py, pp_client.py
└── legacy/        export.py (old export → Design), conversion.py, compas_fab.py (to_cell_state harness), timing.py
```

| # | Work | Done when |
|---|---|---|
| R1 | Moves and merges with `git mv`; split names to their layer. | Files in place. |
| R2 | Rewrite every import (core, monitor, tests, scripts) by name; rename the types. | Quick, slow and linters pass; 3.9 check passes. |
| R3 | Docs, report §9, AGENTS.md. | No stale paths (`design_io.`, `compas_convert`, `SceneSnapshot`, `ToolSpec`). |
