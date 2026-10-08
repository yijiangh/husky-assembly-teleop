# 2026-10-08 Shared core: `bar_assembly_core`

Status: **planned**. Brief: `husky_assembly_teleop/core_extraction_prompt.md` (decisions 1–16 there are fixed).

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

## 4. Package layout (target)

```
bar_assembly_core/
├── __init__.py            package rules (R1–R3); imports nothing heavy
├── requirements.txt
├── design_io/             moved from husky_assembly_teleop/design_io (step 1)
│   ├── types.py           + Design.scene_at(movement), Design.scene_after(bar)  (delegate to scenes.py)
│   ├── scenes.py          scene_at / scene_after: design -> scene (step 4)
│   ├── models.py          robot_model(design, robot_id) -> RobotModel, cached by content (step 4)
│   └── compas_fab.py      to_robot_cell / to_cell_state / planning_group: legacy test harness only
├── ur.py                  UR_JOINT_NAMES, TOOL_TOUCHES_ARM_LINKS, stock_frame_problem  (ROS-free)
├── robot.py               RobotModel (frozen, identity), RobotObject (validity flags), acting_problems
├── scene.py               Attachment, Body, SceneSnapshot, world_poses(...)
├── kinematics.py          ForwardKinematics: link_pose(model, base, joints, link); fk_difference(a, b, links, joints)
├── ids.py                 IdMap (explicit; refuses unmapped), retarget(attachments, robot_map, models)
├── hold.py                release_bar(design, action_id), hold_scene(scene, bar_id)
└── mirrors/
    ├── __init__.py        check_display
    ├── compas_convert.py  frame_from_pose, pose_from_frame, load_model, rigid_body, subtree, filled,
    │                      PARKED_POSITION, FileMesh (moved out of design_io/compas_fab.py in step 1)
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

Filled in at the end (§9 of this file): public API, host checklist, decisions not followed, deferred work, test results.
