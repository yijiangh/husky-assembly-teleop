# Design file format, schema 2

Status: **implemented** (schema 2, final revision, with the decisions of 2026-10-09). Read and written by
`bar_assembly_core.design`, part of the shared core `bar_assembly_core` (plan: husky-assembly-teleop
`tasks/2026-10-08_shared_core.md` §12; decisions: `tasks/2026-10-09_core_integration/README.md`). This is the in-repo copy of the doc "Husky
design format: schema 2 proposal", which also gives the reasons and the examples from the 260814 export. Where the
proposal leaves a point open, the choice made here is marked **Resolved**.

Sections 1–10 define the format. Section 11 lists the checks. Appendices map it to compas_fab and to the old export,
and give an example.

---

## 1. Scope and principles

A **design** is the plan for one assembly: the robots, their tools and every body; the schedule; and per movement its
start state, its target and the parts that say what moves and how it ends.

1. **The file is the in-memory `Design`, one to one.** Scenes, mirrors and host details never appear in it.
2. **The design is the plan:** planned poses, what moves, and how each movement ends. No planner status, no
   measurements.
3. **Three products, each with one writer:** the design folder (Rhino), `solutions/` (planners), `runs/` (the
   monitor).
4. **Every state reads alone, together with the design:** bases, joints, attachments with their grasps, built bars and
   tool states. A pose is written only where it differs from what the design implies.
5. **One rule says where a bar is** (§8.2). Mounted halves follow their bar.
6. **`null` means "not decided by the design"**, for bases, joints and tool states alike. No placeholder values.
7. **Ids are readable and valid within one export.** Anything outside the design that refers to it records the
   content hash of what it refers to (§3.2).
8. **Anything a machine reads is a typed field.** `notes` is for people.

## 2. Folder layout

```
<design>/
├── design.json                  manifest: robots, tools, bodies, mates, schedule
├── actions/<action id>.json     one file per scheduled action
├── meshes/<content hash>.obj       written by the library; read: any path, .obj|.stl|.glb
├── robots/<robot>/robot.urdf, robot.srdf, meshes/…
├── source/                      optional: the .3dm, joint_pairs.json, robotic_tools.json it came from (never read)
├── solutions/<action id>.json   planner results (§10); never written by the exporter
└── runs/<yyyy-mm-dd_hhmmss>/    reserved for the monitor (§10)
```

The library's `write` leaves `source/`, `solutions/` and `runs/` as they are.

## 3. Conventions

| Item | Rule |
|---|---|
| Encoding | UTF-8 JSON. No comments, no `NaN`/`Infinity`. |
| Units | Metres and radians. |
| Pose | `[x, y, z, qx, qy, qz, qw]`: position, then unit quaternion (x, y, z, w). World frame unless stated otherwise. |
| Robot base pose | Pose of the URDF root link in world. |
| `null` | Not decided by the design (bases, joints, tool states), or unknown after a manual step. |
| File references | Paths relative to the design folder, with `/`. |

### 3.1 Ids

Ids are the names Rhino writes: `[A-Za-z0-9_.-]+` segments joined by `/`, case sensitive, unique within one export.
A renumber or rename is a design change: re-export, then re-plan. Bodies get no separate label; `label` is optional
display text and contains no `/`.

| Prefix | Object | Example |
|---|---|---|
| `robots/<robot>` | robot | `robots/cindy` |
| `robots/<robot>/<link>` | one URDF link of a robot | `robots/cindy/left_ur_arm_tool0` |
| `tools/<tool>`, `tools/<robot>/<tool>` | tool mounted on a robot | `tools/AT3L`, `tools/alice/SupportGripper` |
| `bars/<bar>` | bar | `bars/B7` |
| `joints/<joint>` | connector half | `joints/J3-10_male`, `joints/G1-T20Ground-0_ground` |
| `ground/<ground>` | walkable ground surface | `ground/WG0` |
| `obstacles/<name>` | other static body | `obstacles/column_A` |

Action and movement ids are plain names without `/`, unique per design. Producers name them by role, so people and
programs find a movement without counting: an action is `<bar>_<code>` (`J` jointing, `R` release, `H` holding, `HR`
holding release), a movement `<action>_<role>`, with `_2`, `_3`, … when a role repeats.

| Action | Roles, in order |
|---|---|
| `B10_J` | `load`, `mount`, `grasp`, `transfer`, `insert` |
| `B10_R` | `ungrasp`, `retreat`, `home` |
| `B3_H` | `approach`, `open`, `to_grasp`, `close` |
| `B3_HR` | `open`, `retreat`, `leave` |

The role is a naming convention: what a movement does is in its typed fields.

### 3.2 Content hashes

The content hash of a file is the SHA-256 of its JSON with `writer` and `producer` removed, keys sorted, floats rounded
to 12 decimals (`design.content_hash`). The writer is deterministic (ids sorted, the same rounding), so re-exporting an
unchanged design gives the same hashes, also from a newer library commit. `design.json` covers every file it refers
to: meshes are named by their content, and each robot records the hash of its files (`files_hash`).
`design.design_hashes(design)` gives the hashes a `write` would give without writing, so a host can tell a stale
solution while the design is still in memory. Readers never match by id alone:

| File | Records | On a mismatch |
|---|---|---|
| `solutions/<action>.json` | `solved_against`: hashes of `design.json` and of the action file | Stale: shown, never executed |
| `runs/<run>/run.json` | Hashes of `design.json`, of each executed action file and of each solution | Read against the design version it executed |
| Planner caches | The same two hashes in the cache key | Rebuilt |

## 4. `design.json`

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `format` | `"husky_design"` | yes | File kind. |
| `writer` | Writer (§9) | yes | The library that wrote the file. |
| `producer` | Producer (§9) | no | The code that made the design, e.g. Rhino's `RSExportAllBarActions`. |
| `robots` | map robot id → Robot | yes | Every robot. |
| `tools` | map tool id → Tool | yes | Every tool; may be empty. |
| `bodies` | map id → Body | yes | Every body. |
| `mates` | array of [body id, body id] | no | Joints of the finished structure (§8.3): a male and a female half, or a ground half and a ground body. |
| `parts` | map part name → Part | no | The catalogue parts the producer knows (§4.5). |
| `schedule` | array of action ids | yes | Execution order across all robots: the only order. |

### 4.1 Robot

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `urdf`, `srdf` | path | yes | Robot description; planning groups, passive joints, disabled collisions. |
| `files_hash` | string | no | SHA-256 of the URDF, the SRDF and every mesh the URDF names, as copied into the design. Written by the library; a mismatch on read is an A4 error. |
| `serial` | string or `null` | no | Hardware serial (`"0806"`). |
| `tools` | map link name → tool id | no | Tools mounted on this robot, by flange link. |
| `ground_links` | array of link names | no | The links the robot stands on (its wheels): they may touch any `ground/` body. |

### 4.2 Tool

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `collision` | array of Shape | yes | Collision shapes in the flange link frame. `[]`: never collides. |
| `visual` | array of Shape | no | Drawn shapes. Absent: same as `collision`. |
| `tcp` | Pose | yes | Tool centre point in the flange link frame. |
| `kind` | string | yes | Hardware type, from the tool vocabulary (§7). |
| `mount_contacts` | array of link ids | no | Robot links the tool may always touch (how it is mounted, like the SRDF's disabled collisions). |

### 4.3 Body

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `pose` | Pose | yes | Design pose in world: where the body is once built. |
| `collision` | array of Shape | yes | Collision shapes in the body frame. `[]`: never collides. |
| `visual` | array of Shape | no | Drawn shapes. Absent: same as `collision`. |
| `label` | string | no | Display text. Absent: the id. |
| `part` | string | no | Catalogue reference, e.g. `"T20/Female"`, `"T20/MoCap"`, `"T20/Ground"`. |
| `markers` | map label → [x, y, z] | no | Marker points in the body frame, metres: placement checks, mocap registration. |
| `mount` | bar id | halves only | The bar a connector half is fixed to: rigid and static for the whole design; the half follows its bar. Required on every `joints/` body. |

The offset of a half on its bar follows from the two design poses. Bars do not list their halves:
`Design.halves_of(bar)`. A part is a bar plus its mounted halves; it is not stored. A body under `ground/` is walkable.

### 4.4 Shape

Exactly one geometry key, plus an optional `origin`: `{"mesh": <path>}` (§6), `{"box": [sx, sy, sz]}` centred at
`origin`, `{"cylinder": [radius, length]}` along the Z axis of `origin`, centred at it. `"origin"`: the shape's pose in
the owner's frame; absent: identity.

### 4.5 Part

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `seat` | Pose | yes | The half's pose in the frame of the tool centre point acting on it. B14 checks grasps against it. |

Rhino writes the catalogue from `joint_pairs.json` and `robotic_tools.json`. Without an entry, the plan checks fall back
to their built-in seats (`T20/Male` at the TCP, `T20/Ground` turned 180° about its z axis), and warn for other parts.

## 5. Action file `actions/<action id>.json`

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `format` | `"husky_design/action"` | yes | File kind. |
| `writer` | Writer (§9) | yes | |
| `id` | action id | yes | Equals the file name without `.json`. |
| `type` | enum | yes | `bar_jointing`, `bar_release`, `bar_holding`, `bar_holding_release`. |
| `robot` | robot id | yes | The acting robot. |
| `bar` | bar id | yes | The bar the action is about. |
| `ground` | array of ground ids | no | Ground surfaces the base may stand on. |
| `supports_until` | array of bar ids | no | `bar_holding` only: bars that must be built before the hold is released. |
| `label` | string | no | Display text. |
| `notes` | object | no | For people (§5.6). |
| `movements` | array of Movement | yes | In execution order. |

### 5.1 Movement

A movement is one segment that runs without stopping, described by independent parts. Its kind follows from the parts
and is not stored: an **arm move** has arms only; a **tool move** a grip change or a drive only; a **combined move**
both (insert, the compliant forms of ungrasp and untighten); a **manual step** neither, and `ends_on: operator`.

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `id` | movement id | yes | |
| `label` | string | no | Display text. |
| `arms` | array of link ids | no | Flange links of the arms that move. Absent: no arm moves. |
| `path` | `free`, `linear` | when arms move | Path shape. |
| `coupled` | bool | no | The arms keep their relative pose, because they hold one bar. Default `false`. |
| `controller` | `position`, `compliant` | when arms move | `compliant`: the arm may stop short of its target. |
| `line` | map flange link id → {`direction`, `distance`} | on linear paths | One per moving arm: unit direction (world frame) and distance (m). |
| `drives` | map tool id → `tighten`, `loosen` | no | Run the jointing screw during the movement. |
| `ends_on` | `target`, `tools`, `operator` | no | Default `target`. |
| `start` | State (§5.2) | yes | |
| `target` | Target (§5.3) or `null` | no | `null` or absent: end where the next movement starts. |
| `notes` | object | no | For people (§5.6). |

**Rules**

1. When a movement ends: `target`: every moving arm reached its target and every tool part finished; `tools`: every
   tool part finished, the arms stop wherever they are; `operator`: the operator confirmed.
2. `ends_on: tools` needs a tool part; with arm motion it also needs `controller: compliant`.
3. `null` bases, joints and tool states are allowed anywhere: the design does not decide them. A planner may choose
   them offline and writes what it chose into its solution (`bases`, `start`); the monitor fills them from
   measurements at execution. Planning mirrors park other robots whose base or joints are `null`;
   `scenes.seeded` fills in the robot being planned for.
4. `target: null` means "end where the next movement starts".
5. `attached` and `built` may change within a movement; start and target say which: a manual mount or a grasp
   attaches the bar, an insert ends with it built, an ungrasp ends with it no longer attached, an untighten ends with
   it no longer built.
6. The design says what happens; the controller says how: motor start order, streaming rate, gains, force, stall
   detection and timeout lengths, the STOP that clears a stalled motor.

**Cindy's four screw operations.** Each is one movement, or two when the arms move only after the tool part (form B).

| Operation | Bar at start | Arms | Tool part | Ends on | Bar at end |
|---|---|---|---|---|---|
| Grasp | attached (resting in the tools), not built | none | grip → closed | stall | attached, not built |
| Insert | attached, not built | `linear`, `coupled`, `compliant`, forward | drive `tighten` | stall | attached and **built**: Cindy is frozen |
| Ungrasp, after an insert | attached and built | Form A: `linear`, `compliant`, back. Form B: none, then a careful retreat | grip → open, and drive `loosen` to back the jointing screw off | timeout | built, **no longer attached** |
| Untighten, to remove a bar | attached and built | Form A: `linear`, `coupled`, `compliant`, back. Form B: none, then a retreat that moves the bar | drive `loosen` | timeout | attached, **no longer built** |

### 5.2 State

| Key | Type | Req. | Meaning |
|---|---|---|---|
| `robots` | map robot id → RobotState or `null` | yes | Every robot. `null`: not in the scene. |
| `present` | array of body ids | yes | Bars and ground that exist now. Halves are present with their bar; obstacles always. |
| `attached` | map bar id → array of Holder | no | Bars held by a robot, each holder with its grasp (§8.1). |
| `built` | array of bar ids | no | Bars fixed in the structure: they take their design pose. |
| `poses` | map bar id → Pose | no | Only present bars that are neither built nor attached and away from their design pose (a pickup rack). |
| `tools` | map tool id → ToolState or `null` | yes | Every mounted tool. `null`: its robot is absent and the state is not given. |

- RobotState: `base` (Pose or `null`) and `joints` (map joint → value over every non-passive joint, or `null`).
- Holder: `to` (link id `robots/<robot>/<link>`) and `grasp` (the bar's pose in that link's frame). A bar held through
  a tool is held by the flange with the composed grasp. The first holder of an unbuilt bar sets its pose; the others
  must agree.
- ToolState: `grip` (`open`, `closed` or `null`) and `on` (the body the tool is acting on, or `null`), both always
  written.

### 5.3 Target

| Key | Type | Meaning |
|---|---|---|
| `joints` | map robot id → (map joint → value) | Target joint values; a subset of joints is allowed. |
| `links` | map link id → Pose | Target world pose of a link, e.g. a flange. |
| `tools` | map tool id → {`grip`, `on`} | Each key optional, at least one. `grip`: the grip the tool is commanded to (commanding the grip it has is allowed: the hardware repeats it). `on`: the body it is on at the end, or `null` when it leaves; only where `on` changes. |
| `attached` | map bar id → array of Holder | **Resolved:** the whole `attached` map at the end, written only when the movement changes it. |
| `built` | array of bar ids | **Resolved:** the whole `built` list at the end, written only when the movement changes it. |

### 5.4 Notes

`notes` on actions and movements are for people: a key maps to a string, number or boolean, nothing nested. The library
never reads them (a core test fails if a core module does). A note a program needs is a missing field: propose a schema
change. Where the 14 keys of the old export go:

| Key | Schema 2 |
|---|---|
| `lm_axis`, `lm_distance_mm`, `retreat_axes_world` | `Movement.line` |
| `ends_on` | `Movement.ends_on` |
| `constraint` (`fixed_relative_ee_transform`) | `coupled: true` |
| `bar_arm_side` | follows from `attached` |
| `planner_fills`, `start_config_is_none` | `joints: null` |
| `unplanned_offline`, `goal_backfilled_from` | planner status: `solutions/` |
| `approach_axis`, `approach_offset_mm`, `retreat_target_config`, `after` | stay as notes |

## 6. Mesh files

| Rule | |
|---|---|
| Formats | `.obj` (text), `.stl` (binary), `.glb`. Chosen by extension. |
| Content | One mesh per file, in metres, in the frame of the shape that references it. |
| Sharing | Any number of shapes may reference one file. Readers load it once. |
| Robot meshes | URDF mesh references are paths relative to the URDF file. |

## 7. Tool vocabulary

Every tool has one lasting state, its grip; Cindy's tools also have one drive, the jointing screw
(`bar_assembly_core.design.vocabulary`). The vocabulary is part of the schema: adding a kind, a grip value or a drive
raises the schema number. Motor activity (`IDLE`, `TIGHTENING`, `LOOSENING`, `STALLED`) is execution data for `runs/`.

| Kind | Channel | Values | Hardware | How a change ends (controller) |
|---|---|---|---|---|
| `scaffolding_v3` | `grip` | `open`, `closed` | Gripper screws, motor M1 | `closed`: stall; `open`: timeout |
| `scaffolding_v1` | `grip` | `open`, `closed` | UR tool digital output (`set_io`) | output switched |
| `robotiq` | `grip` | `open`, `closed` | 2F-85 (`GripperCommand`) | goal reached, or stalled on the bar |

| Kind | Drive | Directions | Hardware | Ends on (controller) |
|---|---|---|---|---|
| `scaffolding_v3` | jointing screw | `tighten`, `loosen` | Motor M2 | `tighten`: stall; `loosen`: timeout |

A grip change is a `grip` in `target.tools`; it names the tools the movement commands. The tightness belongs to the
mate, not the tool. A `loosen` drive leaves the bar built, unless the target clears `built` (an untighten).

## 8. Bar states and relations

### 8.1 Bar states

| `attached` | `built` | Meaning | The robot holding it |
|---|---|---|---|
| yes | no | Carried: the bar follows the robot | Moves freely, carrying it |
| yes | yes | Fixed and held (after an insert, while a support robot grips it, before an untighten) | Frozen, except in the compliant movement that releases it |
| no | yes | Part of the structure | — |
| no | no | Staged: its pose is in `State.poses`; without one it floats at its design pose (warning) | — |

- `attached` means a robot holds the bar: set when a grip closes on it or the operator mounts it, cleared when the grip
  opens. Several holders: one robot's flanges moving `coupled` for an unbuilt bar (Cindy's left and right); different
  robots for a built bar (Cindy and Alice). Only bars are attached; halves follow by mount.
- `built` means the bar is fixed in the structure: set when one of its halves is mated (the end of an insert), cleared
  by an untighten.

### 8.2 Where a body is (`relations.placement`)

- A built bar takes its design pose and is never written in `poses`.
- An attached, unbuilt bar follows its first holding link's pose times the grasp.
- Any other present bar takes its pose in `State.poses`, else its design pose.
- A mounted half follows its bar by the mount offset (its own design pose when the bar is at its design pose).
- Ground and obstacles take their design pose.

### 8.3 Relations and mate status

| Relation | Between | Moves bodies? | Stored as |
|---|---|---|---|
| Attached | bar → robot link(s) | Only if the bar is not built | `State.attached` |
| Mounted | connector half → its bar | Yes, rigidly, for the whole design | `mount` on the half |
| On | tool → body | No: collision information only | `on` in `State.tools` |
| Mated | half ↔ half, or ground half ↔ ground | No | `mates` in `design.json` |

`on` is set by the target of the movement that brings the tool to the body (the operator's mount, or a support's
linear approach) and cleared by the target of the movement it leaves in (a retreat or a compliant back-off); it never
changes between movements (B11). A Cindy tool is on the half at its TCP (the male or a ground half); a
support gripper is on the bar it supports.

A mate's status follows from the `built` and `attached` flags of its two sides (`relations.mate_status`); a ground
body counts as built.

| Status | When | Contact |
|---|---|---|
| Engaged | Both sides built (or a built bar and the ground) | allowed |
| Pending | One side built, the other present, attached and not built (being joined or removed) | allowed between these two halves |
| Open | One side built, the other not present; **Resolved:** also every other combination | none: an ordinary obstacle |
| Not relevant | Neither side present | not in the scene |

### 8.4 Allowed contacts

Derived, never stored (`relations.allowed_contacts`):

| Rule | Derived from |
|---|---|
| A tool may touch the whole part it is on: the bar and every half mounted on it | `on`, `mount` |
| A half may touch the bar it is mounted on | `mount` |
| The halves of a pending or engaged mate may touch each other, and each other's bar | `mates`, `built`, `attached` |
| A half mated to a ground body may touch every present `ground/` body | `mates`, the body's prefix |
| A present robot's ground links may touch any `ground/` body | `ground_links`, the body's prefix |
| A tool may touch its mount contacts | `mount_contacts` (on the robot model) |

The part rules are as wide as the parts because collision models are coarser than the parts: compas_fab checks each
tool as one convex hull, which overlaps the bar it holds by about 1 mm (the real meshes are 1.0 mm apart).

## 9. Writer and producer

```json
"writer": {"schema": 2, "library": "bar_assembly_core", "commit": "3f9c2e71b0ad", "dirty": false},
"producer": {"repo": "bar_joint_rhino_design_workflow", "commit": "9a1b…", "dirty": false, "command": "RSExportDesign"}
```

`writer` names the library that wrote the file: `schema`, `library`, `commit` (or `"unknown"`), `dirty`. A reader
accepts only its own `schema` and names `commit` otherwise; the library converts nothing between schemas. `producer`
names the code that made the design. Neither is part of the content hash.

## 10. Solutions and runs

Neither ever changes the design, and the design never refers to them.

### 10.1 `solutions/<action id>.json` (`design.solutions`)

| Key | Type | Meaning |
|---|---|---|
| `format` | `"husky_design/solution"` | File kind. |
| `writer` | Writer | As in the design files. |
| `planner` | object | `name` (required), `repo`, `commit`, `ik_backend` (e.g. `ssik`), `artifacts` (the ssik artifact set or URDF it was built for), `settings` (any JSON). |
| `action` | action id | The action solved; equals the file name. |
| `solved_against` | `{design, action}` | Content hashes of `design.json` and of the action file as read. A mismatch marks the solution stale. |
| `movements` | movement id → Result | One entry per movement of the action. |

| Result key | Type | Meaning |
|---|---|---|
| `status` | enum | `solved`, `keyframe_only` (base and end joints, no trajectory), `failed`, `not_planned`. |
| `reason` | string | Why it failed; required for `failed`. |
| `bases` | robot id → Pose | Bases the planner chose where the design had `null`. |
| `start` | robot id → joints | Solved start joints. |
| `start_overridden` | bool | The planner moved a start the design had fixed; neighbours must agree with it. |
| `end` | robot id → joints | Solved end joints. |
| `trajectory` | object or `null` | `robot`, `joint_names`, `positions`, optional `times`: one trajectory per robot over all its moving joints. |
| `path_poses` | link or body id → [Pose] | Optional: a flange's or the attached bar's path, for showing the bar. |

- **Chaining.** Each movement starts where the previous one ended, within 1e-3 (radians or metres: the planners' IK
  tolerance), across actions in `schedule` order; the reader reports every break (`solution_warnings`).
- **The design wins.** Where the design gives target joints or a base, a solution's end must reach them (1e-3; a
  planner snaps its last waypoint to them); `bases`, `start` and `keyframe_only` only fill what the design leaves
  `null`. A keyframe chain may span one bar's J and R actions, on one base.
- **No arm motion.** A movement no arm moves in (a grip, a drive, a manual step) is `solved` with `start` equal to
  `end` and no trajectory (`MovementResult.still`), written by whoever plans the action.
- **Latest only.** One file per action holds the latest result; history lives in dated copies or in git.
- **Partial chains are normal.** Unplanned movements say `not_planned`.

### 10.2 `runs/<yyyy-mm-dd_hhmmss>/`

Reserved for the monitor; specified later. Planners never read it.

## 11. Validation

Two groups of checks, run at different times. Every message starts with its check, e.g. `A9: …` or `B7: …`.

| Group | Runs in | When | On failure |
|---|---|---|---|
| A, file checks | `bar_assembly_core.design` (`read`, `write`, `validate`; `read_solutions`, `write_solution` for A14) | On every read and write | The file is rejected; every problem is reported at once |
| B, plan checks | `bar_assembly_core.design.plan_check.check_plan` | On demand: Rhino before export, planners before planning, the monitor before execution | Errors block execution; warnings are reported |

### 11.1 A: file checks

| # | Check |
|---|---|
| A1 | Every file is UTF-8 JSON without `NaN` or `Infinity`, and `format` and `writer.schema` match in every file |
| A2 | Every required key is present with the right type, and no unknown key appears outside `notes` |
| A3 | Ids match the id pattern, are unique and use a known prefix; labels contain no `/` |
| A4 | Every reference resolves: robot, tool, body, half and action ids, and every file under the design folder (meshes, URDF, SRDF, and the URDF's mesh paths, relative to the URDF). A robot's `files_hash` matches its files |
| A5 | `schedule` and `actions/` match one to one, and each action file's `id` equals its file name |
| A6 | Each tool is mounted on exactly one robot, at a link its URDF declares |
| A7 | Every pose has 7 numbers and a unit quaternion (tolerance 1e-6) |
| A8 | Each state lists every robot, as a `RobotState` or `null`. A `joints` map names only joints the robot's URDF declares, and lists all non-passive ones |
| A9 | In each state, `present` lists only bars and ground; `attached`, `built` and `poses` name only present bars; `poses` never names a built or attached bar; every holder has a link id `to` and a `grasp` |
| A10 | Every state lists every mounted tool. `grip` values and `drives` directions come from the vocabulary of that tool's kind, and `on` (in states and targets) names an existing body or is `null` |
| A11 | `path` and `controller` are present exactly when `arms` is not empty; `ends_on` is one of its values; `line` appears only on linear paths, with one entry per moving arm (unit direction, positive distance) |
| A12 | Every half's `mount` names an existing bar (only halves have one). `mates` pairs two halves, or a half and a ground body, and each half is in at most one mate |
| A13 | `notes` values are strings, numbers or booleans |
| A14 | Every `solutions/` file names an existing action (and its movements, with known statuses) and carries `solved_against`. A hash mismatch makes the solution stale, which is reported as a warning |

### 11.2 B: plan checks

| # | Check | Severity |
|---|---|---|
| | **Structure, in each state** | |
| B1 | A present bar that is neither attached nor built has a pose in `State.poses`; otherwise it floats at its design pose | warning |
| B2 | Every built bar has at least one engaged mate, and the built bars reach the ground through engaged mates | error |
| B3 | Several holders of an unbuilt bar all belong to one robot | error |
| B4 | Every holder's robot has a tool on the bar's part: on the bar or on one of its halves | error |
| B5 | Mates that never become engaged anywhere in the schedule are listed, and halves in no mate (their partner is on a bar the design leaves out) | warning |
| | **Procedure, per movement and across the schedule** | |
| B6 | `ends_on: tools` comes with a tool part (a grip change or a drive); with arm motion it also needs `controller: compliant` | error |
| B7 | An arm motion that moves an attached, unbuilt bar has `grip: closed` on its holders' tools, and several holders move `coupled` | error |
| B8 | A robot that holds a built bar moves only in a compliant movement whose tool part releases that bar | error |
| B9 | A tool on a body its arm does not hold allows only a `linear` retreat, until `on` clears | error |
| B10 | `attached` and `built` change only as the operations allow: a mount or grasp attaches, an insert builds, an ungrasp releases, an untighten unbuilds | error |
| B11 | Each movement ends where the next one starts (by definition for `target: null`), across actions in `schedule` order | error |
| B12 | A hold is released only after every bar in its `supports_until` is built | error |
| | **Geometry, with robot models** | |
| B13 | `arms` name flange links that end an SRDF group of the acting robot | error |
| B14 | Grasps agree with the geometry. For a tool on a connector half, the grasp is TCP × part seat × mount offset; for several holders, forward kinematics agrees whenever joints are known | error |

**Resolved** in the implementation:
- B11 compares joints, grips, tool `on`, present bars, `attached` (holders and grasps) and `built`; bases only within one action,
  because robots drive between actions. Values left open (`null`) are not compared, nor is the acting robot's joints
  after an arm motion whose target gives none.
- B14 compares within 0.1 mm and 1 mrad. The part seats come from `parts` (§4.5), else `plan_check.PART_SEATS`
  (`T20/Male` at the TCP; `T20/Ground` turned 180° about the TCP's z axis). A tool on a half of a part with no seat is
  reported as a B14 warning.
  For a built bar, every holder's forward kinematics must give the design pose.
- B9 checks that the arm's path is linear; that it moves away is not checked.
- B10 checks within a movement; between movements B11 refuses any change.

## 12. Rationale kept from schema 1

| # | Decision | Reason |
|---|---|---|
| R1 | One world, not one cell per robot | Three robots share one scene; compas_fab's one-robot limit belongs to its adapter (App. A). |
| R2 | One id per object, path-shaped | Paths give grouping without a `kind` field. |
| R3 | `null` for an absent or undecided value | A placeholder value is indistinguishable from a real one. |
| R4 | Full state per movement | Any movement is readable with the design, without replaying earlier ones. |
| R6 | Standard assets by reference | Robots as URDF + SRDF, meshes as files. |
| R7 | Primitives | Bars are cylinders: exact collision, no mesh data. |
| R8 | Copies of robot files in the design | Self-contained and reproducible although calibration changes later. |
| R10 | `schedule` is the only order | Old exports kept order in three places that disagreed. |
| R12 | Attach by links only | A tool is fixed to its flange, so "held by a tool" is held by the flange with the composed grasp. |
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
| Tool of the acting robot | `ToolModel` keyed by its tool id, `frame` = `tcp`; attached to the SRDF group ending at its flange; `touch_links` = mount contacts and the arm's wrist and flange links |
| Other robot | One `ToolModel` of its URDF, its tools welded to the flanges; at its base with its joints, or parked when absent or when its base or joints are `null` (`mirror.parked`) |
| Body | `RigidBody`; primitives triangulated; hidden when absent (a half with its bar) |
| Attached, unbuilt bar and its halves, held by the acting robot | `attached_to_link` = the first holder's link, `attachment_frame` = grasp (× mount offset for a half) |
| Held by another robot, or built | stationary at its world pose |
| Derived contacts (§8.4) | `touch_links` (acting robot links) and `touch_bodies` (bodies, tools, other robots) |

## Appendix B. Old export → schema 2 (`bar_assembly_core.legacy`)

The converter replays the bars' `attached` and `built` flags and the tool states along the schedule; it does not copy
the export's per-state attachments and contacts.

| Old export | Schema 2 |
|---|---|
| `RobotCell.json`, `RobotCell_<Name>.json` | `robots`, `tools` (`mount_contacts` from the tool's touch links), `bodies` |
| `bar_B1`, `env_bar_B1`; `joint_…` | `bars/B1`; `joints/…` |
| Wheel links | `ground_links` |
| `ObstacleRobot<Name>` at `(50, 50, 0)` | `null` |
| Halves carried with a bar in its jointing action | `mount` on each half |
| Half ids | `part`: `T20/Male`, `T20/Female`, `T20/Ground` (every half in the exports is type T20) |
| `J<a>-<b>_male` and `J<a>-<b>_female` | a mate, when both are in the design (4 male halves in each export have no female: their partner sits on a fake bar) |
| `G<n>-…_ground` | a mate with the ground body whose walkable polygons are nearest its design position seen from above (0 when above one; the nearest in height wins a tie) |
| Manual mount (`ManualMovement`) | `ends_on: operator`; the bar becomes attached to every flange carrying it or one of its halves (grasps from the export's attachment frames); its target puts the scaffolding tools on the male or ground half at their TCP |
| `grasp` / `close` | grip → `closed`; a support gripper is on its bar from its close, and the bar becomes attached to it (grasp by forward kinematics) |
| Action and movement ids | by role (§3.1): `B10_J_joint` → `B10_J`, its insert → `B10_J_insert` |
| `tighten` (`overlaps_next`) + the insert | one movement, `<action>_insert`: linear, coupled, compliant, `drives: tighten`, `ends_on: tools`; the bar is built at its end |
| `untighten` opening every release | folded into the ungrasp after it: `drives: loosen`, the bar stays built |
| `ungrasp` / `open` | form B: a tool move, grip → `open`, the flange's holders removed; the tools stay on their body until the following retreat, whose target clears `on` |
| A support's `close` | the movement before it (the linear approach) ends with the gripper `on` the bar |
| A tool move setting the grip a tool already has (two support `open`s) | kept: the hardware repeats it |
| Part seats | `parts` for `T20/Male` and `T20/Ground`, from `plan_check.PART_SEATS` |
| Movement class | `arms`, `path`, `coupled`, `controller` (`joint_tracking` → `position`, `cartesian_compliant` → `compliant`) |
| `lm_axis`, `lm_distance_mm`, `retreat_axes_world` | `line`: the axes and distance, else start to target |
| Notes | per §5.4; planner status dropped |
| Placeholder bar poses | the bar is not present until mounted |
| Robot states (`robot_configuration`, `ObstacleRobot…`) | as in the export |
| `trajectory` | refused (`solutions/`) |
| `WalkableGround.json` (mm) | bodies under `ground/` (m) |

## Appendix C. Example

`design.json` (abridged):

```json
{
  "format": "husky_design",
  "writer": {"schema": 2, "library": "bar_assembly_core", "commit": "3f9c2e71b0ad", "dirty": false},
  "robots": {
    "robots/cindy": {"urdf": "robots/cindy/robot.urdf", "srdf": "robots/cindy/robot.srdf", "files_hash": "9f2c…",
                     "serial": "0806",
                     "tools": {"left_ur_arm_tool0": "tools/AT3L", "right_ur_arm_tool0": "tools/AT3R"},
                     "ground_links": ["front_left_wheel_link", "front_right_wheel_link", "rear_left_wheel_link",
                                      "rear_right_wheel_link"]}
  },
  "tools": {
    "tools/AT3L": {"collision": [{"mesh": "meshes/3e1f0c9a7b2d4e61.obj"}], "tcp": [-0.07, 0, 0.08, 0, 0, 0, 1],
                   "kind": "scaffolding_v3", "mount_contacts": ["robots/cindy/left_ur_arm_wrist_3_link"]}
  },
  "bodies": {
    "bars/B3":             {"pose": [0.10, 2.20, 0.05, 0, 0.7071, 0, 0.7071], "collision": [{"cylinder": [0.0125, 0.90]}]},
    "joints/J3-10_female": {"pose": [0.10, 2.35, 0.05, 0, 0, 0, 1], "collision": [{"mesh": "meshes/a07c55e2d9f81b3c.obj"}],
                            "part": "T20/Female", "mount": "bars/B3"},
    "joints/J3-10_male":   {"pose": [0.10, 2.36, 0.05, 0, 0, 0, 1], "collision": [{"mesh": "meshes/c49d2b80e6a17f05.obj"}],
                            "part": "T20/Male", "mount": "bars/B10"}
  },
  "mates": [["ground/WG0", "joints/G1-T20Ground-0_ground"], ["joints/J3-10_female", "joints/J3-10_male"]],
  "parts": {"T20/Male": {"seat": [0, 0, 0, 0, 0, 0, 1]}, "T20/Ground": {"seat": [0, 0, 0, 0, 0, 1, 0]}},
  "schedule": ["B1_J", "B1_R", "B3_J", "B3_H", "B3_R"]
}
```

The insertion of B10, one combined movement (abridged):

```json
{"id": "B10_J_insert", "label": "Insert",
 "arms": ["robots/cindy/left_ur_arm_tool0", "robots/cindy/right_ur_arm_tool0"],
 "path": "linear", "coupled": true, "controller": "compliant",
 "line": {"robots/cindy/left_ur_arm_tool0": {"direction": [-1, 0, 0], "distance": 0.015},
          "robots/cindy/right_ur_arm_tool0": {"direction": [-1, 0, 0], "distance": 0.015}},
 "drives": {"tools/AT3L": "tighten", "tools/AT3R": "tighten"}, "ends_on": "tools",
 "start": {"robots": {"…": "…"}, "present": ["bars/B1", "bars/B10", "…", "ground/WG0"],
           "attached": {"bars/B10": [{"to": "robots/cindy/left_ur_arm_tool0", "grasp": ["…"]},
                                     {"to": "robots/cindy/right_ur_arm_tool0", "grasp": ["…"]}]},
           "built": ["bars/B1", "bars/B3", "…"],
           "tools": {"tools/AT3L": {"grip": "closed", "on": "joints/J3-10_male"},
                     "tools/AT3R": {"grip": "closed", "on": "joints/J7-10_male"},
                     "tools/alice/SupportGripper": {"grip": "open", "on": null},
                     "tools/belle/SupportGripper": {"grip": "open", "on": null}}},
 "target": {"joints": {"…": "…"}, "links": {"…": "…"}, "built": ["bars/B1", "bars/B10", "bars/B3", "…"]}}
```

The grasp, a tool move:

```json
{"id": "B10_J_grasp", "label": "Grasp", "ends_on": "tools", "start": {"…": "…"},
 "target": {"tools": {"tools/AT3L": {"grip": "closed"}, "tools/AT3R": {"grip": "closed"}}}}
```

The retreat after the ungrasp, which takes the tools off the halves:

```json
{"id": "B10_R_retreat", "arms": ["…", "…"], "path": "linear", "controller": "position", "line": {"…": "…"},
 "start": {"…": "…"}, "target": {"links": {"…": "…"}, "tools": {"tools/AT3L": {"on": null}, "tools/AT3R": {"on": null}}}}
```
