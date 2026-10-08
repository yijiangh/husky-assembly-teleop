# Design file format, schema 2

Status: **implemented** (schema 2). Read and written by `bar_assembly_core.design`, part of the shared core
`bar_assembly_core` (plan: `tasks/2026-10-08_shared_core.md` §11). The design for schema 2, with its reasons and
examples from the 260814 export, is the doc "Husky design format: schema 2 proposal". The core also turns a design into
scenes (`Design.scene_at`, `scene_after`) for its mirrors; the format does not depend on them.

Sections 1–9 define the format. Section 10 gives the reasons. Appendices map it to compas_fab and to the old export,
and give an example.

---

## 1. Scope

A **design** is the plan for one assembly, with no other input: the robots, their tools and every body; the schedule;
and per movement its start state, its target and the parts that say what moves and how it ends.

Not part of a design: planner results (`solutions/`, §8), measurements and executions (`runs/`, §8), the Rhino
document (`source/` keeps a copy), and modeling-only objects (Rhino "fake bars").

Three products, each with one writer: the design folder (Rhino), `solutions/` (planners), `runs/` (the monitor).

## 2. Folder layout

```
<design>/
├── design.json                  manifest: robots, tools, bodies, connections, schedule
├── actions/<action id>.json     one file per scheduled action
├── meshes/<any path>.obj|.stl|.glb
├── robots/<robot>/robot.urdf, robot.srdf, meshes/…
├── source/                      the .3dm, joint_pairs.json, robotic_tools.json it came from (not read by the library)
├── solutions/<action id>.json   planner results (§8); never written by the exporter
└── runs/<yyyy-mm-dd_hhmmss>/    executions (§8); written by the monitor only
```

## 3. Conventions

| Item | Rule |
|---|---|
| Encoding | UTF-8 JSON. No comments, no `NaN`/`Infinity`. |
| Units | Metres and radians. Fixed by the schema. |
| Pose | `[x, y, z, qx, qy, qz, qw]`: position, then unit quaternion (x, y, z, w). World frame unless stated otherwise. |
| Robot base pose | Pose of the URDF root link in world. |
| Id | `[A-Za-z0-9_.-]+` segments joined by `/`. Case sensitive. Unique and valid within one export: a renumber is a design change. |
| `null` | Not decided by the design (bases, joints, tool states), or unknown after a manual step. No placeholder values. |
| File references | Paths relative to the design folder, with `/`. |

### 3.1 Id namespaces

| Prefix | Object | Example |
|---|---|---|
| `robots/<robot>` | robot | `robots/cindy` |
| `robots/<robot>/<link>` | one URDF link of a robot | `robots/cindy/left_ur_arm_tool0` |
| `tools/<tool>`, `tools/<robot>/<tool>` | tool mounted on a robot | `tools/AT3L`, `tools/alice/SupportGripper` |
| `bars/<bar>` | bar | `bars/B3` |
| `joints/<joint>` | joint half | `joints/J1-3_male` |
| `ground/<ground>` | walkable ground surface | `ground/WG0` |
| `obstacles/<name>` | other static body | `obstacles/column_A` |

Action and movement ids are plain names (`B3_H_hold`, `B3_H_M0_free_to_approach`), unique per design.

## 4. `design.json`

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `format` | `"husky_design"` | yes | File kind. |
| `writer` | Writer (§7) | yes | The library that wrote the file. |
| `producer` | Producer (§7) | no | The code that made the design, e.g. Rhino's `RSExportAllBarActions`. |
| `robots` | map robot id → Robot | yes | Every robot. |
| `tools` | map tool id → Tool | yes | Every tool; may be empty. |
| `bodies` | map id → Body | yes | Every body. |
| `connections` | array of [body id, body id] | no | Bodies joined in the design (§5.5). |
| `schedule` | array of action ids | yes | Execution order across all robots. |

### 4.1 Robot

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `urdf`, `srdf` | path | yes | Robot description; planning groups, disabled collisions. |
| `serial` | string | no | Hardware serial (`"0806"`). |
| `tools` | map link name → tool id | no | Tools mounted on this robot, by flange link. |
| `ground_links` | array of link names | no | Links the robot stands on (its wheels): they may touch any `ground/` body. |

### 4.2 Tool

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `collision` | array of Shape | yes | Collision shapes in the flange link frame. `[]`: never collides. |
| `visual` | array of Shape | no | Drawn shapes. Absent: same as `collision`. |
| `tcp` | Pose | yes | Tool centre point in the flange link frame. |
| `kind` | string | yes | Hardware type, from the tool state vocabulary (§5.4). |
| `mount_contacts` | array of link ids | no | Robot links the tool may always touch (how it is mounted), e.g. the wrist links. |

### 4.3 Body

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `pose` | Pose | yes | Design pose in world: where the body is once placed. |
| `collision` | array of Shape | yes | Collision shapes in the body frame. `[]`: never collides. |
| `visual` | array of Shape | no | Drawn shapes. Absent: same as `collision`. |
| `label` | string | no | Display text. Absent: the id. |
| `part` | string | no | Catalogue reference, e.g. `"T20/Female"`. |
| `markers` | map label → [x, y, z] | no | Marker points in the body frame, metres: placement checks, mocap registration. |

A body under `ground/` is walkable: its collision shapes are the surface robot bases stand on.

### 4.4 Shape

Exactly one geometry key, plus an optional `origin`: `{"mesh": <path>}` (§6), `{"box": [sx, sy, sz]}` centred at
`origin`, `{"cylinder": [radius, length]}` along the Z axis of `origin`, centred at it. `"origin"`: the shape's pose in
the owner's frame; absent: identity.

## 5. Action file `actions/<action id>.json`

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `format` | `"husky_design/action"` | yes | File kind. |
| `writer` | Writer (§7) | yes | |
| `id` | action id | yes | Equals the file name without `.json`. |
| `type` | enum | yes | `bar_jointing`, `bar_release`, `bar_holding`, `bar_holding_release`. |
| `robot` | robot id | yes | The acting robot. |
| `bar` | body id | yes | The bar the action is about. |
| `ground` | array of body ids | no | Ground surfaces the base may stand on. |
| `supports_until` | array of body ids | no | `bar_holding` only: bars that must be built before release. |
| `label` | string | no | Display text. |
| `notes` | object | no | For people (§5.6). |
| `movements` | array of Movement | yes | In execution order. |

### 5.1 Movement

A movement is one segment that runs without stopping, described by independent parts. Its kind is not stored: an arm
move has arms only, a tool move a tool change only, a combined move both (the insertion; an ungrasp that backs off),
a manual step neither and `ends_on: operator`.

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `id` | movement id | yes | |
| `label` | string | no | Display text. |
| `arms` | array of link ids | no | Flange links of the arms that move. Absent: no arm moves. |
| `path` | `free`, `linear` | when arms move | Path shape. |
| `coupled` | bool | no | The arms keep their relative pose (they hold one bar). Default `false`. |
| `controller` | `position`, `compliant` | when arms move | `compliant`: the arm may stop short of its target, following forces. |
| `line` | map flange link id → {`direction`, `distance`} | no | `linear` only, per moving arm: unit direction (world) and distance (m). |
| `ends_on` | `target`, `tools`, `operator` | no | Default `target`: every moving arm reached its target and every tool transition finished. `tools`: every tool transition finished; the arms stop wherever they are. `operator`: the operator confirmed. |
| `start` | State (§5.2) | yes | |
| `target` | Target (§5.3) or `null` | no | `null` or absent: end where the next movement starts. |
| `notes` | object | no | For people (§5.6). |

The design states the parts and what "done" means; the controller owns the motor start order, rates, gains, force,
stall detection and timeouts. A tool change is the difference between the start and the target tool states. A movement
that starts unknown (`null` joints, base or tool states) is planned at execution, from the measured robot.

### 5.2 State

Complete and readable alone: every robot, every present body, every tool.

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `robots` | map robot id → RobotState or `null` | yes | Every robot. `null`: not in the scene. |
| `present` | array of body ids | yes | Bodies that exist now. |
| `poses` | map body id → Pose | yes | The world pose of every present body that is not carried. |
| `carried` | map body id → Carried | no | Present bodies that move with a robot link. |
| `tools` | map tool id → ToolState or `null` | yes | Every mounted tool. `null`: its robot is absent, or its state unknown. |

RobotState: `base` (Pose or `null`) and `joints` (map joint → value over every non-passive joint, or `null`).

Carried: `to` (link id `robots/<robot>/<link>`) and `offset` (the body pose in that link's frame). A body carried through
a tool is carried by the flange with the composed offset.

ToolState: one value per channel of the tool's kind (§5.4), each a value or `null`, plus `on`: the body the tool sits
on, or absent.

### 5.3 Target

| Key | Type | Meaning |
|---|---|---|
| `joints` | map robot id → (map joint → value) | Target joint values; a subset of joints is allowed. |
| `links` | map link id → Pose | Target world pose of a link, e.g. a flange. |
| `tools` | map tool id → (map channel → value) | The tool channels the movement changes, at their end value. |

### 5.4 Tool state vocabulary

Each kind has named channels with two planned values (`bar_assembly_core.design.vocabulary`). A value says what the tool
does to the body once the movement is done; motor activity (`IDLE`, `TIGHTENING`, `STALLED`, …) is execution data for
`runs/`. Adding a kind, channel or value raises the schema.

| Kind | Channel | Values | How the transition ends (controller) |
|---|---|---|---|
| `scaffolding_v3` | `grip` | `open`, `closed` | `closed`: motor stall; `open`: timeout |
| `scaffolding_v3` | `joint` | `loose`, `tight` | `tight`: motor stall; `loose`: timeout |
| `scaffolding_v1` | `grip` | `open`, `closed` | output switched |
| `robotiq` | `grip` | `open`, `closed` | goal reached, or stalled on the bar |

### 5.5 Carried, on and connected bodies

| Relation | Between | Means | Stored as |
|---|---|---|---|
| Carried | body → robot link | the body moves with the robot | `State.carried` |
| On | tool → body | the tool sits on that body | `on` in the tool state |
| Connected | body ↔ body | joined in the design: a joint half and its bar, mated halves, a ground connector and its ground | `connections` |

A **part** is a bar plus the joint halves connected to it; mates connect parts but never merge them. `on` is set when
the tool arrives at the body (the operator mounts the bar, or an approach ends) and cleared when it leaves (a retreat
ends).

**Allowed contacts are derived, never stored** (`bar_assembly_core.design.relations`):

| Rule | Derived from |
|---|---|
| A tool may touch the part it sits on, and the halves mated to that part | `on`, `connections` |
| Connected bodies may touch; a mated half may touch its mate's part | `connections` |
| A robot's ground links may touch any `ground/` body | `ground_links`, the body's prefix |
| A tool may touch its mount contacts | `mount_contacts` |

? The mate rules (a half with its mate's part, a tool with the halves mated to its part) are an addition to the
proposal: in 260814 a male half touches its mate's bar after insertion (36 contacts), which the proposal's rules alone
report as collisions.

### 5.6 Notes

`notes` on actions and movements are for people: a key maps to a string, number or boolean, nothing nested. The library
never reads them; a value a program needs is a missing field, to be proposed as a schema change.

## 6. Mesh files

| Rule | |
|---|---|
| Formats | `.obj` (text), `.stl` (binary), `.glb`. Chosen by extension. |
| Content | One mesh per file, in metres, in the frame of the shape that references it. |
| Sharing | Any number of shapes may reference one file. Readers load it once. |
| Robot meshes | URDF mesh references are paths relative to the URDF file; readers resolve them against its folder. |

## 7. Writer, producer and content hash

```json
"writer": {"schema": 2, "library": "design_io", "commit": "3f9c2e71b0ad", "dirty": false},
"producer": {"repo": "bar_joint_rhino_design_workflow", "commit": "9a1b…", "dirty": false, "command": "RSExportAllBarActions"}
```

`writer` names the library that wrote the file: `schema` (incremented on every incompatible change), `library`,
`commit` (or `"unknown"`), `dirty`. A reader accepts only its own `schema` and names `commit` otherwise; there is no
conversion between schemas inside the library. `producer` names the code that made the design.

**Content hash** (`design.content_hash`): the SHA-256 of a file's JSON without `writer` and `producer`, keys sorted,
floats rounded to 12 decimals. Re-exporting an unchanged design gives the same hash, also from a newer library commit.
Files outside the design that refer to it record the hashes of what they refer to; a mismatch marks them stale.

## 8. Solutions and runs

Neither ever changes the design, and the design never refers to them.

- `solutions/<action id>.json`, written by planners: per movement a status (`solved`, `keyframe_only`, `failed`,
  `not_planned`), solved bases, start and end joints, a trajectory per robot; `solved_against` holds the content hashes
  of `design.json` and the action file. The format is in the proposal; the library implements only the hash so far.
- `runs/<yyyy-mm-dd_hhmmss>/`, written by the monitor: reserved, specified later.

## 9. Validation

A reader rejects a design that breaks any rule, listing every problem.

1. `format` and `writer.schema` match in every file.
2. Every id matches §3 and is unique; body keys use a §3.1 prefix.
3. Every file referenced exists; every action file is scheduled, and every scheduled action has a file.
4. Every robot, tool, body and link id referenced exists; link ids name a link in that robot's URDF.
5. Every tool is mounted on exactly one robot; mount links, `ground_links` and `mount_contacts` exist.
6. Joint names exist in the URDF; a non-`null` `joints` in a RobotState lists every non-passive joint.
7. Each State lists every robot and every mounted tool (a tool of an absent robot is `null`), and a pose for exactly
   the present bodies that are not carried.
8. A carried body's robot is present in that state.
9. Quaternions and line directions have unit length (tolerance 1e-6).
10. `arms` name links of the acting robot, each the tip of an SRDF group.
11. Every mesh reference in a robot's URDF is a relative path to an existing file.
12. `path` and `controller` exactly when arms move; `line` only on a `linear` path, for moving arms, distance > 0;
    `ends_on: tools` needs a tool change, and with arms `controller: compliant`; `ends_on: operator` has no arms and no
    tool change; a movement with neither arms nor a tool change ends on `operator`.
13. Tool kinds, channels and values come from the vocabulary; every channel is listed; `on` names a present body.
14. Every carried body belongs to a part a tool of the carrying robot is on. An arm motion carrying bodies needs
    `grip: closed` on those tools. A closed tool on a body the robot does not carry pins its arm, except in a
    compliant movement that opens that tool. Manual steps and tool moves are exempt.
15. Notes are flat: strings, numbers, booleans.
16. A connection joins two different bodies.

## 10. Rationale

Schema 2's reasons are in the proposal. Kept from schema 1:

| # | Decision | Reason |
|---|---|---|
| R1 | One world, not one cell per robot | Three robots share one scene; compas_fab's one-robot limit belongs to its adapter (App. A). |
| R2 | One id per object, path-shaped | Paths give grouping without a `kind` field. |
| R3 | `null` for an absent or undecided value | A placeholder value is indistinguishable from a real one. |
| R4 | Full state per movement | Any movement is readable without replaying earlier ones (schema 2: also every pose and tool). |
| R6 | Standard assets by reference | Robots as URDF + SRDF, meshes as files. |
| R7 | Primitives | Bars are cylinders: exact collision, no mesh data. |
| R8 | Copies of robot files in the design | Self-contained and reproducible although calibration changes later. |
| R10 | `schedule` is the only order | Old exports kept order in three places that disagreed. |
| R12 | Carry by links only | A tool is fixed to its flange, so "carried by a tool" is carried by the flange with the composed offset. |
| R15 | Schema number plus commit, no conversion | Old designs are read with old code. |
| R17 | Arms named by flange link | Which SRDF group to plan with is a planner choice. |
| R18 | Tool `id` and `kind` both | The id names a geometry variant, the kind the hardware. |
| R20 | Relative robot mesh paths | The design folder can move. |

## Appendix A. Mapping to compas_fab

Planners get a compas_fab cell by syncing a scene (`design.scene_at(movement)`) into a `CompasFabMirror` for the acting
robot (`bar_assembly_core/mirrors/compas_fab.py`):

| Design / scene | compas_fab |
|---|---|
| Acting robot URDF + SRDF | `RobotCell.robot_model`, `robot_semantics` |
| Tool of the acting robot | `ToolModel` keyed by its tool id, `frame` = `tcp`; `ToolState` attached to the SRDF group ending at its flange (nearest the URDF root), `touch_links` = mount contacts and the arm's wrist and flange links |
| Other robot | One `ToolModel` of its URDF, its tools welded to the flanges; `ToolState` at its base with its joints, or parked when absent |
| Body | `RigidBody`; primitives triangulated; hidden when absent |
| Carried by the acting robot | `attached_to_link`, `attachment_frame` = offset |
| Carried by another robot | stationary at its world pose; that robot in `touch_bodies` |
| Derived contacts (§5.5) | `touch_links` (acting robot links) and `touch_bodies` (bodies, tools, other robots) |

## Appendix B. Old export → schema 2 (`bar_assembly_core.legacy`)

| Old export | Schema 2 |
|---|---|
| `RobotCell.json`, `RobotCell_<Name>.json` | `robots`, `tools` (`mount_contacts` from the tool's touch links), `bodies` |
| `bar_B1`, `env_bar_B1`; `joint_…` | `bars/B1`; `joints/…` |
| Wheel links touching the ground | `ground_links` |
| `ObstacleRobot<Name>` at `(50, 50, 0)` | `null` |
| Body–body contacts, halves carried with a bar | `connections`: each half with the bar it is carried with, mated halves, ground connectors with their action's ground. A half's contact with its mate's bar is not a connection. |
| `notes.bar_pose_is_placeholder` | the bar is not present |
| Unattached rigid bodies | `poses`, every one |
| `attached_to_link` | `carried` |
| `tool_action` along the schedule | tool states: `grasp`/`close` → `grip: closed`, `ungrasp`/`open` → `grip: open`, `tighten` → `joint: tight`, `untighten` → `joint: loose`; start released |
| Tool contacts | `on`: a scaffolding tool on the male or ground half it touches; a support gripper on its action's bar from its `close`; cleared after a retreat |
| A robot's bodies missing in another robot's states | still carried by it (states are complete) |
| The supported bar missing in its hold release | present at its design pose |
| Movement class | `arms`, `path`, `coupled`, `controller` (`joint_tracking` → `position`, `cartesian_compliant` → `compliant`) |
| `tighten` with `overlaps_next` + the insert | one movement under the tighten's id, `target.tools: joint tight`, `ends_on: tools` |
| `ungrasp` | a tool move (the export has no line for a back-off) |
| `lm_axis`, `lm_distance_mm`, `retreat_axes_world` | `line`: the axes and distance, else start to target |
| `ends_on`, `constraint`, `bar_arm_side`, `planner_fills`, `start_config_is_none`, `unplanned_offline`, `goal_backfilled_from` | dropped (fields or planner status) |
| other notes | `notes`, flat values only |
| `trajectory` | refused (`solutions/`) |
| `WalkableGround.json` (mm) | bodies under `ground/` (m) |

## Appendix C. Example

`design.json` (abridged):

```json
{
  "format": "husky_design",
  "writer": {"schema": 2, "library": "design_io", "commit": "3f9c2e71b0ad", "dirty": false},
  "robots": {
    "robots/cindy": {"urdf": "robots/cindy/robot.urdf", "srdf": "robots/cindy/robot.srdf", "serial": "0806",
                     "tools": {"left_ur_arm_tool0": "tools/AT3L", "right_ur_arm_tool0": "tools/AT3R"},
                     "ground_links": ["front_left_wheel_link", "front_right_wheel_link", "rear_left_wheel_link",
                                      "rear_right_wheel_link"]}
  },
  "tools": {
    "tools/AT3L": {"collision": [{"mesh": "meshes/tools/AT3L.obj"}], "tcp": [-0.07, 0, 0.08, 0, 0, 0, 1],
                   "kind": "scaffolding_v3", "mount_contacts": ["robots/cindy/left_ur_arm_wrist_3_link"]}
  },
  "bodies": {
    "bars/B1":          {"pose": [0.10, 2.20, 0.05, 0, 0.7071, 0, 0.7071], "collision": [{"cylinder": [0.0125, 0.90]}]},
    "joints/J1-3_male": {"pose": [0.10, 2.35, 0.05, 0, 0, 0, 1], "collision": [{"mesh": "meshes/joints/T20_Male.obj"}],
                         "part": "T20/Male"}
  },
  "connections": [["bars/B3", "joints/J1-3_male"], ["joints/J1-3_female", "joints/J1-3_male"]],
  "schedule": ["B1_J_joint", "B1_R_release", "B3_J_joint", "B3_H_hold"]
}
```

The insertion, one combined movement (abridged):

```json
{"id": "B1_J_M4", "label": "Insert",
 "arms": ["robots/cindy/left_ur_arm_tool0", "robots/cindy/right_ur_arm_tool0"],
 "path": "linear", "coupled": true, "controller": "compliant", "ends_on": "tools",
 "line": {"robots/cindy/left_ur_arm_tool0": {"direction": [0, 0, -1], "distance": 0.015},
          "robots/cindy/right_ur_arm_tool0": {"direction": [0, 0, -1], "distance": 0.015}},
 "start": {"robots": {"…": "…"}, "present": ["…"], "poses": {"…": "…"},
           "carried": {"bars/B1": {"to": "robots/cindy/left_ur_arm_tool0", "offset": [0, 0, 0.12, 0, 0, 0, 1]}},
           "tools": {"tools/AT3L": {"grip": "closed", "joint": "loose", "on": "joints/G1-T20Ground-0_ground"},
                     "tools/AT3R": {"grip": "closed", "joint": "loose", "on": "joints/G1-T20Ground-1_ground"},
                     "tools/alice/SupportGripper": null}},
 "target": {"links": {"…": "…"}, "tools": {"tools/AT3L": {"joint": "tight"}, "tools/AT3R": {"joint": "tight"}}}}
```
