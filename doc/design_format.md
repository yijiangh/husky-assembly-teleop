# Design file format, schema 1

Status: **proposal**. Replaces the compas_fab JSON export (`RobotCell*.json`, `BarActions/`,
`ActionSchedule.json`, `WalkableGround.json`). Read and written by the `design_io` library
(plan: `tasks/2026-10-01_design_io_library.md`).

Sections 1–9 define the format. Section 10 gives the reasons. Appendices map it to compas_fab
and to the current export, and give an example.

---

## 1. Scope

A **design** is everything needed to plan and execute one assembly, with no other input:

- the robots, their tools, and every body (bars, joints, ground, obstacles);
- the schedule: the order of actions across all robots;
- per movement: the authored start state and target.

Not part of a design: planner results (section 8), live measurements, the Rhino document, and
modeling-only objects that do not exist physically (Rhino "fake bars" and their joint halves).

## 2. Folder layout

```
<design>/
├── design.json                  manifest: robots, tools, bodies, schedule
├── actions/
│   └── <action id>.json         one file per scheduled action
├── meshes/
│   └── <any path>.obj|.stl|.glb one mesh per file, referenced from design.json
├── robots/
│   └── <robot id>/
│       ├── robot.urdf           mesh references relative to this file
│       ├── robot.srdf
│       └── meshes/…
└── solutions/                   planner results (section 8); never written by the exporter
    └── <action id>.json
```

## 3. Conventions

| Item | Rule |
|---|---|
| Encoding | UTF-8 JSON. No comments, no `NaN`/`Infinity`. |
| Length | Metres. Fixed by the schema; not stated per file. |
| Angle | Radians. |
| Pose | Array of 7 numbers `[x, y, z, qx, qy, qz, qw]`: position, then unit quaternion (x, y, z, w). In world frame unless stated otherwise. |
| Robot base pose | Pose of the URDF root link in world. |
| Id | `[A-Za-z0-9_.-]+` segments joined by `/`. Case sensitive. Unique across the design. |
| Absent value | `null`. No sentinel values (e.g. far-away poses). |
| File references | Paths relative to the design folder, with `/` separators. |

### 3.1 Id namespaces

| Prefix | Object | Example |
|---|---|---|
| `robots/<robot>` | robot | `robots/cindy` |
| `robots/<robot>/<link>` | one URDF link of a robot | `robots/cindy/left_ur_arm_tool0` |
| `tools/<tool>` | tool mounted on a robot | `tools/AT3L` |
| `bars/<bar>` | bar | `bars/B3` |
| `joints/<joint>` | joint half | `joints/J1-3_male` |
| `ground/<ground>` | walkable ground surface | `ground/WG0` |
| `obstacles/<name>` | other static body | `obstacles/column_A` |

Action and movement ids are plain names (`B3_H_hold`, `B3_H_M0_free_to_approach`), unique per design.

## 4. `design.json`

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `format` | `"husky_design"` | yes | File kind. |
| `writer` | Writer (§7) | yes | Schema version and writing code. |
| `robots` | map robot id → Robot | yes | Every robot, keyed by its full id (`robots/cindy`). |
| `tools` | map tool id → Tool | yes | Every tool, keyed by its full id (`tools/AT3L`); may be empty. |
| `bodies` | map id → Body | yes | Every body, keyed by its full id (`bars/B1`). |
| `schedule` | array of action ids | yes | Execution order across all robots. |

### 4.1 Robot

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `urdf` | path | yes | Robot description. |
| `srdf` | path | yes | Planning groups, disabled collision pairs, named states. |
| `serial` | string | no | Hardware serial (`"0806"`). Absent for a robot that exists only in the design. |
| `tools` | map link name → tool id | no | Tools mounted on this robot, by flange link (`left_ur_arm_tool0`). |

### 4.2 Tool

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `collision` | array of Shape | yes | Collision shapes in the flange link frame. `[]`: never collides. |
| `visual` | array of Shape | no | Drawn shapes. Absent: same as `collision`. |
| `tcp` | Pose | yes | Tool centre point in the flange link frame. |
| `kind` | string | yes | Hardware and driver type for execution (`"scaffolding_v3"`, `"robotiq"`). Several tools may share a kind. |
| `touches` | array of ids | no | Always allowed to touch this tool (e.g. wrist links). |

### 4.3 Body

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `pose` | Pose | yes | Design pose in world: where the body is once placed. |
| `collision` | array of Shape | yes | Collision shapes in the body frame. `[]`: never collides. |
| `visual` | array of Shape | no | Drawn shapes. Absent: same as `collision`. |
| `touches` | array of ids | no | Always allowed to touch this body. Symmetric. |
| `label` | string | no | Display text. Absent: the id. |

A body under `ground/` is walkable: its collision shapes are the surface robot bases stand on.

### 4.4 Shape

Exactly one of the geometry keys, plus an optional `origin`.

| Form | Meaning |
|---|---|
| `{"mesh": <path>}` | Triangle mesh from a file (§6). |
| `{"box": [sx, sy, sz]}` | Box centred at `origin`, side lengths along its axes. |
| `{"cylinder": [radius, length]}` | Cylinder along the Z axis of `origin`, centred at it. |
| `"origin": Pose` | Shape pose in the owner's frame. Absent: identity. |

## 5. Action file `actions/<action id>.json`

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `format` | `"husky_design/action"` | yes | File kind. |
| `writer` | Writer (§7) | yes | |
| `id` | action id | yes | Equals the file name without `.json`. |
| `type` | enum | yes | `bar_jointing`, `bar_release`, `bar_holding`, `bar_holding_release`. |
| `robot` | robot id | yes | The acting robot, e.g. `robots/alice`. |
| `bar` | body id | yes | The bar the action is about. |
| `ground` | array of body ids | no | Ground surfaces the base may stand on. |
| `supports_until` | array of body ids | no | `bar_holding` only: bars that must be built before release. |
| `label` | string | no | Display text. |
| `movements` | array of Movement | yes | In execution order. |

### 5.1 Movement

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `id` | movement id | yes | |
| `type` | enum | yes | `free`, `linear`, `manual`, `tool`. |
| `arms` | array of link ids | yes for `free`, `linear` | Flange links of the arms that move, e.g. `robots/cindy/left_ur_arm_tool0`. |
| `coupled` | bool | no | `true`: the arms hold one object together (end-effector constrained). Default `false`. |
| `controller` | enum | yes | `joint_tracking`, `cartesian_compliant`, `none`. |
| `tools` | array of tool ids | yes for `tool` | Tools that act. |
| `tool_action` | string | yes for `tool` | E.g. `grasp`, `tighten`, `release`, `open`, `close`. |
| `overlaps_next` | bool | no | `tool` only: runs on through the next movement. Default `false`. |
| `start` | State (§5.2) | yes | Authored start state. |
| `target` | Target (§5.3) | no | Where the movement should end. |
| `label` | string | no | Display text. |
| `notes` | object | no | Planning hints from the producer (`lm_distance_mm`, `approach_axis`, …), passed through unchanged. Temporary: to be replaced by typed fields. |

### 5.2 State

A complete description of one moment: every robot, every body. No state refers to another.

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `robots` | map robot id → RobotState or `null` | yes | Every robot of the design. `null`: not in the scene. |
| `present` | array of body ids | yes | Bodies that exist at this moment. Others are absent. |
| `poses` | map body id → Pose | no | Present, unattached bodies not at their design pose. |
| `attached` | map body id → Attachment | no | Present bodies held by a robot link. |
| `touches` | array of [id, id] | no | Contacts allowed in this state, in addition to the design-level `touches`. |
| `placeholder` | array of ids | no | Robots or bodies whose pose here is a placeholder, not a real pose. |

RobotState:

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `base` | Pose | yes | Base pose (§3). |
| `joints` | map joint → value, or `null` | yes | Every non-passive URDF joint. `null`: not fixed by the design; a planner or the live robot decides. |

Attachment:

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `to` | link id | yes | `robots/<robot>/<link>`. |
| `grasp` | Pose | yes | Body pose in that link's frame. |

### 5.3 Target

At least one key.

| Key | Type | Meaning |
|---|---|---|
| `joints` | map robot id → (map joint → value) | Target joint values; a subset of joints is allowed. |
| `links` | map link id → Pose | Target world pose of a link, e.g. a flange. |

## 6. Mesh files

| Rule | |
|---|---|
| Formats | `.obj` (text), `.stl` (binary), `.glb`. Chosen by extension. |
| Content | One mesh per file, in metres, in the frame of the shape that references it. |
| Sharing | Any number of shapes may reference one file. Readers load it once. |
| Faces | Triangles or polygons; readers triangulate. |
| Robot meshes | URDF mesh references are paths relative to the URDF file. Readers resolve them against the URDF's folder, never the working directory. |

## 7. Writer and versioning

```json
"writer": {"schema": 1, "library": "design_io", "commit": "a1b2c3d", "dirty": false}
```

| Key | Meaning |
|---|---|
| `schema` | Format version. Incremented on every incompatible change. |
| `library` | Name of the writing library. |
| `commit` | Its git commit, or `"unknown"`. |
| `dirty` | Written from uncommitted library changes. |

A reader accepts only its own `schema`. On mismatch it stops and names `commit`: to read an old
design, check out that commit. There is no conversion between schemas inside the library.

## 8. Solutions (planner results)

Sketch; not part of schema 1. `solutions/<action id>.json` holds, per movement id, solved start
and end joints, a solved base pose and a trajectory (`joint_names`, `times`, `positions`), plus a
`planner` block (name, commit). Design files never contain planner results.

## 9. Validation

A reader rejects a design that breaks any rule.

1. `format` and `writer.schema` match in every file.
2. Every id matches §3 and is unique; body keys use a §3.1 prefix.
3. Every file referenced exists; every file in `actions/` is in `schedule`, and every scheduled action has a file.
4. Every robot id, tool id, body id and link id referenced exists. Link ids name a link in that robot's URDF.
5. Every tool is mounted on exactly one robot; the mount link exists.
6. `joints` maps name every joint they list in the robot's URDF; a non-`null` `joints` in a RobotState lists every non-passive joint.
7. Each State lists every robot. A body is in `attached` or `poses` only if it is in `present`, and never in both.
8. An attachment's robot is present (not `null`) in that state.
9. Quaternions have unit length (tolerance 1e-6).
10. `arms` name links of the acting robot, and its SRDF has a group ending at each of them.
11. Every mesh reference in a robot's URDF is a relative path to an existing file.

## 10. Rationale

| # | Decision | Reason |
|---|---|---|
| R1 | One world, not one cell per robot | Three robots share one scene. compas_fab's one-robot-per-planner limit is a property of that backend, handled by its adapter (App. A), not by the file. |
| R2 | One id per object, path-shaped | Current exports name one bar `bar_B1` or `env_bar_B1` depending on the cell. Paths give grouping without a `kind` field, and match the monitor's scene ids. |
| R3 | `null` for an absent robot | The parking pose `(50, 50, 0)` is indistinguishable from a real pose to every reader that does not know the convention. |
| R4 | Full state per movement | Kept from the producer (`bar_action.py`): any movement is readable without replaying earlier ones. States hold no geometry, so repetition is small. |
| R5 | `joints: null` has one meaning | "Not fixed at design time." The producer's two readings (live current, planner fills) are decided by the consumer, not the file. |
| R6 | Standard assets by reference | Robots as URDF + SRDF, meshes as files. The producer already sources robots, tools and joints from these files; the current export only re-embeds them (≈1 GB for ≈40 MB of geometry, ~14 s to load). |
| R7 | Primitives | Bars are cylinders: exact collision, no mesh data. Backends use them natively. |
| R8 | Copies of robot files in the design | The design is self-contained and reproducible although calibration changes later. Execution compares the copy with the live URDF. |
| R9 | No derived data | World poses of attached bodies follow from base, joints and grasp; storing them lets them disagree. |
| R10 | `schedule` is the only order | Current exports keep order and holds in three places that disagree. |
| R11 | Movement kind as fields | `type` + `arms` + `coupled` + `controller` replace ten movement classes and need no class path in the file. |
| R12 | Attach to links only | A tool is fixed to its flange, so "attached to a tool" is an attachment to the flange with the composed grasp. One way to express one fact. |
| R13 | Fixed units | A units field with one legal value adds nothing; unit bugs came from inconsistent exporters (`WalkableGround.json` in mm). |
| R14 | `visual` defaults to `collision` | Most bodies draw what they collide with; no duplicate lists. |
| R15 | Schema number plus commit, no conversion | Old designs are read with old code. The number keeps one reader working across commits that did not change the format; the commit says which code to use otherwise. |
| R16 | Planner results in `solutions/` | The exported design never changes after export; each planning run is attributable. |
| R17 | Arms named by flange link, not SRDF group | The arm-only and base-rooted groups (`Left arm`, `base_left_arm_manipulator`) move the same six joints and end at the same link; they differ only in the root frame for IK targets. Which group to plan with is a planner choice; the file states only what moves. |
| R18 | Tool `id` and `kind` both | The id names a geometry variant (Rhino's candidates `AT3L`, `AT3_E1L`, …); the kind names the hardware driver (`scaffolding_v3`). Several variants share one driver, so neither can replace the other. |
| R19 | No modeling-only objects | Rhino's fake bars give a real bar's male joint a partner to be placed against; they are excluded from every collision scene and never assembled. Only the Rhino document needs them. |
| R20 | Relative robot mesh paths | The design folder can move. compas_robots resolves a plain path against the working directory, so resolving against the URDF folder is the reader's job. |

## Appendix A. Mapping to compas_fab

`to_robot_cell(design, robot)` builds one `RobotCell` for one acting robot; `to_cell_state` builds
its `RobotCellState` for one State.

| Design | compas_fab |
|---|---|
| Acting robot URDF + SRDF | `robot_model`, `robot_semantics`; mesh paths made absolute before loading |
| `arms` | Planning group: the SRDF group ending at that link, rooted at the URDF root (as Rhino uses `base_left_arm_manipulator`) |
| Other robot | `ToolModel` from its URDF, its tool meshes welded to the flange |
| Tool of the acting robot | `ToolModel`: collision meshes, `frame` = `tcp` |
| Body | `RigidBody`; primitives triangulated |
| Acting robot `base`, `joints` | `robot_base_frame`, `robot_configuration` (`None` for `null`) |
| Other robot present | `ToolState`: `frame` = base, `configuration` = joints |
| Other robot `null` | `ToolState` at a parking pose (compas_fab needs every tool in every state) |
| Body not in `present` | `RigidBodyState.is_hidden = True` |
| `attached` to the acting robot | `attached_to_link`, `attachment_frame` = grasp |
| `attached` to another robot | stationary `frame` = resolved world pose; that robot in `touch_bodies` |
| `touches` entry naming an acting-robot link | `touch_links` |
| other `touches` entry | `touch_bodies` |

`from_compas_fab` reads the other way. Lossy: primitives stay meshes; `attached_to_tool` becomes a
link attachment with the composed grasp.

## Appendix B. Current export → schema 1

| Current | Schema 1 |
|---|---|
| `RobotCell.json`, `RobotCell_<Name>.json` | `design.json` `robots`, `tools`, `bodies`; files in `robots/`, `meshes/` |
| `bar_B1`, `env_bar_B1` | `bars/B1` |
| `joint_J1-3_male`, `env_joint_J1-3_male` | `joints/J1-3_male` |
| `ObstacleRobot<Name>` + `ToolState` | `robots/<name>` RobotState |
| Tool at `(50, 50, 0)` | `null` |
| `ActionSchedule.json` | `schedule`; holds from action `type` and order |
| `BarActions/<bar>__<kind>.json` | `actions/<action id>.json` |
| `assembly_seq` per action | removed (schedule) |
| Movement class | `type`, `arms`, `coupled`, `controller`, `tools`, `tool_action` |
| `assembly_seq` entries of fake bars | removed (§1) |
| `notes.bar_pose_is_placeholder` | `placeholder` |
| other `notes` | `notes`, unchanged |
| `trajectory` | `solutions/` |
| `WalkableGround.json` (mm) | bodies under `ground/` (m) |
| `dtype`, `guid` | removed |

## Appendix C. Example

`design.json` (abridged):

```json
{
  "format": "husky_design",
  "writer": {"schema": 1, "library": "design_io", "commit": "a1b2c3d", "dirty": false},
  "robots": {
    "robots/cindy": {"urdf": "robots/cindy/robot.urdf", "srdf": "robots/cindy/robot.srdf", "serial": "0806",
                     "tools": {"left_ur_arm_tool0": "tools/AT3L", "right_ur_arm_tool0": "tools/AT3R"}},
    "robots/alice": {"urdf": "robots/alice/robot.urdf", "srdf": "robots/alice/robot.srdf", "serial": "0804",
                     "tools": {"ur_arm_tool0": "tools/SupportGripper"}}
  },
  "tools": {
    "tools/AT3L": {"collision": [{"mesh": "meshes/tools/AT3_E1L.obj"}], "tcp": [0, 0, 0.12, 0, 0, 0, 1],
                   "kind": "scaffolding_v3", "touches": ["robots/cindy/left_ur_arm_wrist_3_link"]}
  },
  "bodies": {
    "bars/B1":          {"pose": [0.10, 2.20, 0.05, 0, 0.7071, 0, 0.7071], "collision": [{"cylinder": [0.0125, 0.90]}]},
    "joints/J1-3_male": {"pose": [0.10, 2.35, 0.05, 0, 0, 0, 1], "collision": [{"mesh": "meshes/joints/T20_Male.obj"}]},
    "ground/WG0":       {"pose": [0, 0, 0, 0, 0, 0, 1], "collision": [{"mesh": "meshes/ground/WG0.obj"}],
                         "touches": ["robots/cindy/front_left_wheel_link", "robots/cindy/front_right_wheel_link"]}
  },
  "schedule": ["B1_J_joint", "B1_R_release", "B3_J_joint", "B3_H_hold"]
}
```

`actions/B3_H_hold.json`, first movement:

```json
{
  "format": "husky_design/action",
  "writer": {"schema": 1, "library": "design_io", "commit": "a1b2c3d", "dirty": false},
  "id": "B3_H_hold", "type": "bar_holding", "robot": "robots/alice", "bar": "bars/B3",
  "ground": ["ground/WG0"], "supports_until": ["bars/B4", "bars/B9"],
  "movements": [
    {
      "id": "B3_H_M0_free_to_approach", "type": "free", "arms": ["robots/alice/ur_arm_tool0"],
      "controller": "joint_tracking",
      "start": {
        "robots": {
          "robots/alice": {"base": [-4.51, 1.76, -0.016, 0, 0, 0.0397, 0.9992], "joints": null},
          "robots/cindy": {"base": [-3.90, 1.20, -0.016, 0, 0, 0.7071, 0.7071],
                           "joints": {"left_ur_arm_shoulder_pan_joint": -1.18, "…": 0.0}},
          "robots/belle": null
        },
        "present": ["bars/B1", "bars/B3", "joints/J1-3_male", "ground/WG0"],
        "attached": {"bars/B3": {"to": "robots/cindy/left_ur_arm_tool0", "grasp": [0, 0, 0.12, 0, 0, 0, 1]}},
        "touches": [["bars/B3", "tools/AT3L"], ["bars/B3", "tools/AT3R"]]
      },
      "target": {"joints": {"robots/alice": {"ur_arm_shoulder_pan_joint": 0.59, "…": 0.0}},
                 "links": {"robots/alice/ur_arm_tool0": [-4.43, 2.44, 0.56, 0, 0, 0.3, 0.954]}}
    }
  ]
}
```

`"…"` stands for the remaining joints; a real file lists them all (§9 rule 6).
