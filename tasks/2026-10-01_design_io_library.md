# 2026-10-01 `design_io`: first version of the design library

Status: **M1–M5 implemented** (M5 without the scene bodies, see §6). Format: `doc/design_format.md` (schema 1).

## 1. Goal

A library that reads, writes and validates schema 1 designs, and converts them to and from
compas_fab. Version 1 lives in the monitor as `husky_assembly_teleop/design_io/` and moves to its
own repository afterwards (candidate: `rs_data_structure`).

## 2. Constraints

| # | Constraint |
|---|---|
| C1 | Imports nothing from `husky_assembly_teleop` outside `design_io/`. |
| C2 | Runs on Python 3.9 (Rhino 8): no `match`, no `X \| Y` evaluated at runtime, no `slots=` / `kw_only=` dataclasses, no `zip(strict=)`. Every module starts with `from __future__ import annotations`. |
| C3 | Core dependencies: `numpy`, `scipy`, `trimesh==4.12.2` (newest release for 3.9). The monitor pins the same version: its 72 tests pass with 4.12.2 and 5.1, and viser (`<6,>=3.21.7`) and yourdfpy accept it. |
| C4 | `compas`, `compas_fab`, `compas_robots` are imported only by `compas_fab.py` and `legacy.py`. `design_io/__init__.py` imports neither. |
| C5 | Data types are frozen dataclasses. Collections inside them are never mutated after construction. |
| C6 | No conversion between schemas (format §7). |

## 3. Package layout

```
husky_assembly_teleop/design_io/
├── __init__.py      public API (§5); no compas imports
├── pose.py          Pose, compose, ID_PATTERN, check_id           (moved from world/scene.py)
├── geometry.py      TriMesh, BoxShape, CylinderShape, Shape,
│                    Geometry, shape_mesh, box_geometry,
│                    cylinder_geometry                             (moved from world/geometry.py)
├── types.py         Design and its parts (§4)
├── version.py       SCHEMA, LIBRARY, writer_info()
├── meshes.py        read_mesh(path) -> TriMesh, write_mesh(TriMesh, path); cache by path
├── read.py          read(folder) -> Design
├── write.py         write(design, folder)
├── validate.py      validate(design) -> None; raises DesignError with every problem found
├── robot_files.py   URDF/SRDF reading (xml only) and copying into a design
├── carry.py         assumed_joints(design, action_id, movement_id) -> (joints, source)
├── compas_fab.py    to_robot_cell, to_cell_state, planning_group, compas_link_pose;
│                    helpers shared with world/mirrors/compas_fab.py
└── legacy.py        read_legacy(folder, robot_files, serials) -> Design   (current compas_fab export)

scripts/convert_design.py          legacy folder -> schema 1 folder
test/test_design_io_*.py           §7
```

`world/scene.py` and `world/geometry.py` re-export the moved names, so existing imports keep working.
`Geometry.from_rigid_body` stays on `Geometry`: it only calls methods of the body it is given, so it
needs no compas import.

## 4. Memory layout

```python
# types.py
MovementType = Literal["free", "linear", "manual", "tool"]
ActionType   = Literal["bar_jointing", "bar_release", "bar_holding", "bar_holding_release"]
Controller   = Literal["joint_tracking", "cartesian_compliant", "none"]

@dataclass(frozen=True)
class Writer:
    schema: int
    library: str
    commit: str                              # "unknown" if not found
    dirty: bool

@dataclass(frozen=True)
class RobotSpec:
    id: str                                  # "robots/cindy"
    urdf: Path                               # absolute, resolved at read
    srdf: Path
    serial: str | None
    tools: dict[str, str]                    # flange link name -> tool id

@dataclass(frozen=True)
class ToolSpec:
    id: str                                  # "tools/AT3L"
    geometry: Geometry                       # in the flange link frame
    tcp: Pose                                # in the flange link frame
    kind: str | None
    touches: tuple[str, ...]

@dataclass(frozen=True)
class BodySpec:
    id: str                                  # "bars/B1"
    pose: Pose                               # design pose, world
    geometry: Geometry                       # in the body frame
    touches: tuple[str, ...]
    label: str

@dataclass(frozen=True)
class RobotState:
    base: Pose
    joints: dict[str, float] | None          # every non-passive joint, or None

@dataclass(frozen=True)
class Attached:
    to: str                                  # "robots/<robot>/<link>"
    grasp: Pose                              # in that link's frame

@dataclass(frozen=True)
class State:
    robots: dict[str, RobotState | None]     # every robot; None = not in the scene
    present: frozenset[str]
    poses: dict[str, Pose]
    attached: dict[str, Attached]
    touches: frozenset[tuple[str, str]]      # each pair sorted
    placeholder: frozenset[str]

@dataclass(frozen=True)
class Target:
    joints: dict[str, dict[str, float]]      # robot id -> joint -> value
    links: dict[str, Pose]                   # link id -> world pose

@dataclass(frozen=True)
class Movement:
    id: str
    type: MovementType
    arms: tuple[str, ...]                    # flange link ids
    coupled: bool
    controller: Controller
    tools: tuple[str, ...]
    tool_action: str | None
    overlaps_next: bool
    start: State
    target: Target | None
    label: str

@dataclass(frozen=True)
class Action:
    id: str
    type: ActionType
    robot: str
    bar: str
    ground: tuple[str, ...]
    supports_until: tuple[str, ...]
    label: str
    movements: tuple[Movement, ...]

@dataclass(frozen=True)
class Design:
    folder: Path | None                      # None until read or written
    writer: Writer
    robots: dict[str, RobotSpec]
    tools: dict[str, ToolSpec]
    bodies: dict[str, BodySpec]
    schedule: tuple[str, ...]
    actions: dict[str, Action]               # keyed by action id
```

## 5. Public API

| Function | Module | Result |
|---|---|---|
| `read(folder)` | `read.py` | `Design`. Validates. Raises `SchemaMismatch` (names the writing commit) or `DesignError`. |
| `write(design, folder)` | `write.py` | Writes all files; `writer` from `writer_info()`. Refuses a non-empty folder without `overwrite=True`; with it, removes stale `actions/*.json`. |
| `validate(design)` | `validate.py` | Format §9 rules 2–10. Collects all problems, then raises once. |
| `world_pose(design, state, body_id)` | `types.py` | Resolved world pose of a present body (design pose, override or attachment). Needs forward kinematics: takes a `link_pose(robot_id, link, joints, base)` callable. |
| `assumed_joints(design, action_id, movement_id)` | `carry.py` | Joints for display or as a planner seed when the authored joints are `None`, with their source movement id. Logic of `plugins/cell/design.carry_forward`. |
| `to_robot_cell(design, robot_id, names=None, include=None)` | `compas_fab.py` | `RobotCell` for one acting robot (format App. A). URDF mesh paths made absolute before loading; models cached per URDF file. `names` renames ids in the cell; `include` filters bodies. |
| `planning_group(cell, link)` | `compas_fab.py` | The SRDF group ending at that flange link, rooted at the URDF root if there is one. |
| `to_cell_state(design, robot_id, state, cell, names=None, joints=None)` | `compas_fab.py` | `RobotCellState` matching that cell; `joints` stands in for missing start joints. |
| `compas_link_pose(design)` | `compas_fab.py` | Forward kinematics for `types.world_pose`. |
| `read_legacy(folder, robot_files, serials=None)` | `legacy.py` | `Design` from the current export folder. The Rhino exporter's in-memory objects are the same types, so its switch can reuse the same conversion. |

## 6. Milestones

| # | Work | Done when |
|---|---|---|
| M1 | Move `pose.py`, `geometry.py` into `design_io/`; re-exports in `world/`. | All existing tests pass unchanged. |
| M2 | `types.py`, `version.py`, `meshes.py`, `read.py`, `write.py`, `validate.py`. | A hand-written two-robot design reads, writes and reads back equal; each §9 rule has a failing case. |
| M3 | `legacy.py`, `from_compas_fab`, `scripts/convert_design.py`. | `260814_RobArch_support_ik` converts: 3 robots, 48 actions, 224 movements, one id per body, no parked robots, ground in metres. |
| M4 | `to_robot_cell`, `to_cell_state`; `world/mirrors/compas_fab.py` uses the shared helpers. | Round trip on the 260814 export (§7, T5) passes for all 224 movements. |
| M5 | `cell` plugin reads schema 1 through `design_io`; `carry.py` replaces `design.carry_forward`; the base and arm planners read the step's `design_io` values; tool mismatch warning (§10). Drawing and stepping use only the `Design` (robot URDFs via yourdfpy, tools at their flanges, bodies from their shapes); compas_fab cells and states are built only in planners' mirrors. | The plugin shows the converted design as it showed the legacy one. |
| M5b | Design bodies into `ctx.scene` (scene plan phase 4), so live planners collide with the built structure. | Split from M5: it changes what live planners see, and attachments there follow the measured robot. |
| M6 | Extraction to its own repository; monitor uses it as a submodule under `external/`. | C1 test green before the move; monitor tests green after. |

Not in version 1: `solutions/` files, GLB writing, changes to the Rhino exporter.

## 7. Tests

| # | Test | Checks |
|---|---|---|
| T1 | `test_design_io_isolation` | No module in `design_io/` imports `husky_assembly_teleop` outside it (AST scan); `import design_io` does not import `compas`. |
| T2 | `test_design_io_py39` | `design_io/` compiles and imports under Python 3.9 (`uv run --python 3.9`); skipped if unavailable. |
| T3 | `test_design_io_roundtrip` | `read(write(d)) == d` for a hand-written design, including primitives, shared mesh files, `null` robots and `null` joints. |
| T4 | `test_design_io_validate` | One failing design per format §9 rule. |
| T5 | `test_design_io_compas_fab` | For every movement of the converted 260814 design and each acting robot: every rigid-body and tool frame of `to_cell_state` equals the original start state within 1e-6 m and 1e-6 rad; `is_hidden` flags equal; `check_collision` reports the same pairs. Skipped if the data folder is absent. |
| T6 | `test_design_io_schema` | A file with another `schema` raises `SchemaMismatch` naming its commit. |

## 8. Legacy conversion rules

| Current | Rule |
|---|---|
| Robot model / `ObstacleRobot<Name>` | Robot id `robots/<name lowercase>` from `ActionSchedule.json` `robots`. URDF and SRDF from `robot_files` (husky_urdf copies); joint names and origins verified against the embedded model. |
| `bar_X`, `env_bar_X` | `bars/X`. `joint_X`, `env_joint_X` → `joints/X`. `obstacle_X` → `obstacles/X`. |
| Rigid-body meshes | Written once per distinct mesh (hash of vertices and faces) under `meshes/`. |
| Design pose of a body | Its frame in the last movement where it is present and not attached. Other movements: `poses` entry if different by more than 1e-6. |
| Tool state at `(50, 50, 0)` | `null`. |
| `robot_configuration` | All non-passive joints; joints missing from a partial configuration are taken from the previous movement of the same robot, else zero, and counted in the report. `None` stays `null`. |
| `attached_to_tool` | Attachment to the tool's flange link, grasp composed with the attachment frame. |
| `touch_links`, `touch_bodies` | `touches` pairs with the owner. |
| `notes.bar_pose_is_placeholder` | The action's bar in `placeholder`. |
| Movement class | `type`, `arms`, `coupled`: dual-arm classes → both flange links, single-arm → the one flange link; `EndEffectorConstrained*` → `coupled`. |
| Fake bars | Not exported. The current export already leaves them out of cells and actions; their `assembly_seq` entries go with that field. |
| Tools | Id from the Rhino registry name (`AT3L`); `kind` by table: `AT3*` → `scaffolding_v3`, `SupportGripper` → `robotiq`. `tcp` from `M_tcp_from_block`, millimetres to metres. |
| `WalkableGround.json` | `ground/<id>` bodies; millimetres to metres; polygon extruded downward 0.05 m (as `old/cfab_session._slab_mesh_from_polygon`). |

## 9. Open questions

| # | Question | Status |
|---|---|---|
| Q1 | Which SRDF groups movements name. | Resolved: none; movements name flange links (format R17). |
| Q2 | Tool ids. | Resolved: Rhino's id plus a required `kind` (format R18). |
| Q3 | Fake bars. | Resolved: not exported (format R19). |
| Q4 | trimesh version. | Resolved: 4.12.2 everywhere (C3). |
| Q5 | Final home: `rs_data_structure` (retiring its Movement classes) or a new repository. | Open; decide at M6. |
| Q6 | Which tool geometry the monitor uses. | Resolved: §10. |
| Q8 | Typed fields for the producer's planning hints now in movement `notes` (`lm_distance_mm`, `lm_axis`, `approach_axis`, `approach_offset_mm`, `retreat_axes_world`, `constraint`, `ends_on`, …). | Open; passed through unchanged in schema 1. |

## 10. Tools in the monitor

| Use | Tool geometry and driver |
|---|---|
| Measured robot (drawing, live collision checks, execution) | The monitor's configuration for that robot: `tools` parameter, stitched tool URDF, driver by `EndEffectorKind`. |
| Planned states (from the design) | The design's tool: its `collision`/`visual` shapes and `tcp`. |
| Mismatch | When the `cell` plugin loads a design, it compares each design tool's `kind` with the end effector configured on the same robot's flange link, for every robot that has a `serial` and is loaded. Different, missing or extra: one warning per robot, in the log and as a chip in the panel. Loading continues. |

## 11. Rationale

| # | Decision | Reason |
|---|---|---|
| D1 | Start inside the monitor | Existing scene types, mirrors, tests and data are here; the package boundary is enforced by T1, so the later move is a copy. |
| D2 | Move `Pose` and `Geometry` into the library | One definition shared by file and scene; otherwise the library and the monitor drift apart. |
| D3 | numpy, scipy, trimesh only in the core | All install in Rhino 8's CPython 3.9 and in the monitor venv; compiled or heavy dependencies stay out. |
| D4 | compas behind two modules | Rhino and planners need compas_fab objects, the monitor core does not; loading the design must not pay for compas. |
| D5 | Share helpers with `CompasFabMirror` | It already maps a scene snapshot to a compas_fab cell; a second mapping would diverge. |
| D6 | Legacy reader through `from_compas_fab` | The current export can only be read with compas; the same function serves the Rhino exporter later. |
| D7 | T5 on the producer's own data | Equal frames and equal collision results on their export is the evidence that the new format loses nothing they use. |
| D8 | `write` removes stale action files | The current export leaves files from earlier runs in `BarActions/`. |
| D9 | Measured robot keeps the monitor's tools; plans keep the design's | The measurement describes the robot as it is; the plan was made with the design's geometry, and replacing it would check a different plan than the one authored. A warning, not a refusal: tool variants of one kind (`AT3L`, `AT3_E1L`) differ by design. |
