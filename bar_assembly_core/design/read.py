"""
Read a design folder (doc/design_format.md) into a `Design`: check every file (A1, A2), parse, validate (A3–A13).

Files written by different commits of the same schema read fine; another schema raises `SchemaMismatch`.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ..geometry import BoxShape, CylinderShape, Geometry, Pose, Shape, TriMesh
from ..robot import Tool
from .meshes import MeshCache
from ..urdf import robot_files_hash
from .types import (Action, BodySpec, Design, DesignError, Holder, LineSpec, Movement, PartSpec, Producer, RobotSpec,
                    RobotState, SchemaMismatch, State, Target, ToolState, Writer)
from .validate import validate
from .version import SCHEMA

DESIGN_FORMAT, ACTION_FORMAT = "husky_design", "husky_design/action"

#: Every object of the format (A2): key -> type, a leading "?" marking an optional key. Types: "str", "bool", "num",
#: "nums" (a list of numbers), "pose" (7 numbers, else A7), "notes", "json" (anything), another object's name;
#: "{t" is a map of t, "[t" a list of t, "null|t" also allows null.
KEYS: Dict[str, Dict[str, str]] = {
    "design": {"format": "str", "writer": "writer", "?producer": "producer", "robots": "{robot", "tools": "{tool",
               "bodies": "{body", "?mates": "[[str", "?parts": "{part", "schedule": "[str"},
    "writer": {"schema": "num", "library": "str", "commit": "str", "dirty": "bool"},
    "producer": {"repo": "str", "commit": "str", "dirty": "bool", "command": "str"},
    "robot": {"urdf": "str", "srdf": "str", "?files_hash": "str", "?serial": "null|str", "?tools": "{str",
              "?ground_links": "[str"},
    "part": {"seat": "pose"},
    "tool": {"collision": "[shape", "?visual": "[shape", "tcp": "pose", "kind": "str", "?mount_contacts": "[str"},
    "body": {"pose": "pose", "collision": "[shape", "?visual": "[shape", "?label": "str", "?part": "str",
             "?markers": "{nums", "?mount": "str"},
    "shape": {"?mesh": "str", "?box": "nums", "?cylinder": "nums", "?origin": "pose"},
    "action": {"format": "str", "writer": "writer", "id": "str", "type": "str", "robot": "str", "bar": "str",
               "?ground": "[str", "?supports_until": "[str", "?label": "str", "?notes": "notes",
               "movements": "[movement"},
    "movement": {"id": "str", "?label": "str", "?arms": "[str", "?path": "str", "?coupled": "bool",
                 "?controller": "str", "?line": "{line", "?drives": "{str", "?ends_on": "str", "start": "state",
                 "?target": "null|target", "?notes": "notes"},
    "line": {"direction": "nums", "distance": "num"},
    "state": {"robots": "{null|robot_state", "present": "[str", "?attached": "{[holder", "?built": "[str",
              "?poses": "{pose", "tools": "{null|tool_state"},
    "robot_state": {"base": "null|pose", "joints": "null|{num"},
    "holder": {"to": "str", "grasp": "pose"},
    "tool_state": {"grip": "null|str", "on": "null|str"},
    "target": {"?joints": "{{num", "?links": "{pose", "?tools": "{tool_change", "?attached": "{[holder",
               "?built": "[str"},
    "tool_change": {"?grip": "str", "?on": "null|str"},
    # * solutions/<action id>.json
    "solution": {"format": "str", "writer": "writer", "planner": "planner", "action": "str",
                 "solved_against": "solved_against", "movements": "{result"},
    "planner": {"name": "str", "?repo": "str", "?commit": "str", "?ik_backend": "str", "?artifacts": "str",
                "?settings": "json"},
    "solved_against": {"design": "str", "action": "str"},
    "result": {"status": "str", "?reason": "str", "?bases": "{pose", "?start": "{{num", "?start_overridden": "bool",
               "?end": "{{num", "?trajectory": "null|trajectory", "?path_poses": "{[pose"},
    "trajectory": {"robot": "str", "joint_names": "[str", "positions": "[nums", "?times": "nums"},
}


def read(folder: Path) -> Design:
    """Read and validate a design folder.

    Args:
        folder: The design folder (holds `design.json`).

    Returns:
        Design: With `folder` and robot file paths absolute.

    Raises:
        SchemaMismatch: If any file has another schema; names the commit that wrote it.
        DesignError: If a file is missing or malformed, or the design breaks a file check (every problem found is
            listed, each starting with its check, e.g. "A4: ...").
    """
    folder = Path(folder).resolve()
    manifest_path = folder / "design.json"
    if not manifest_path.is_file():
        raise DesignError([f"A4: no design.json in {folder}"])

    # * 1. Load every file and check its kind and schema (A1), then its keys and types (A2), before parsing.
    problems: List[str] = []
    manifest = load_json(manifest_path, "design.json", DESIGN_FORMAT, problems)
    raw_actions = {}
    for path in sorted((folder / "actions").glob("*.json")):
        raw_actions[path.stem] = load_json(path, f"actions/{path.name}", ACTION_FORMAT, problems)
    if manifest is not None:
        check_keys(manifest, "design", "design.json", problems)
    for name, raw in raw_actions.items():
        if raw is not None:
            check_keys(raw, "action", f"actions/{name}.json", problems)
    if problems:
        raise DesignError(problems)

    # * 2. Parse. File-only problems (a missing mesh, a wrong action id) are reported with what `validate` finds.
    meshes = _Meshes(folder, problems)
    design = Design(folder=folder, writer=_writer(manifest["writer"]),
                    robots={key: _robot(key, value, folder) for key, value in manifest["robots"].items()},
                    tools={key: _tool(key, value, meshes) for key, value in manifest["tools"].items()},
                    bodies={key: _body(key, value, meshes) for key, value in manifest["bodies"].items()},
                    schedule=tuple(manifest["schedule"]),
                    actions={name: _action(raw) for name, raw in raw_actions.items()},
                    mates=frozenset(tuple(sorted(pair)) for pair in manifest.get("mates", ())),
                    producer=_producer(manifest["producer"]) if "producer" in manifest else None,
                    parts={name: PartSpec(_pose(raw["seat"])) for name, raw in manifest.get("parts", {}).items()})
    for name, action in design.actions.items():
        if action.id != name:
            problems.append(f"A5: actions/{name}.json has id {action.id!r}, not its file name")
    for pair in manifest.get("mates", ()):
        if len(pair) != 2:
            problems.append(f"A12: design.json mates: {pair!r} is not a pair")
    for robot_id, raw in manifest["robots"].items():
        robot = design.robots[robot_id]
        if "files_hash" in raw and robot.urdf.is_file() and robot.srdf.is_file():
            try:
                found = robot_files_hash(robot.urdf, robot.srdf)
            except FileNotFoundError as error:
                found = None
                problems.append(f"A4: {robot_id}: {error}")
            if found is not None and found != raw["files_hash"]:
                problems.append(f"A4: {robot_id}: the robot files changed since the design was written "
                                f"(files_hash does not match)")

    # * 3. Validate: every problem at once.
    try:
        validate(design)
    except DesignError as error:
        problems.extend(error.problems)
    if problems:
        raise DesignError(problems)
    return design


# --- --- --- --- --- FILES --- --- --- --- ---

def _refuse_constant(name: str) -> None:
    """Refuse NaN and Infinity (A1): `json` would read them."""
    raise ValueError(f"{name} is not JSON")


def load_json(path: Path, name: str, kind: str, problems: List[str]) -> Optional[Dict[str, Any]]:
    """Load one JSON file and check its `format` and `writer.schema` (A1).

    Args:
        path: The file.
        name: Its path in the design folder, for messages.
        kind: The `format` it must have.
        problems: Gets one line per problem.

    Returns:
        dict | None: The parsed file, or None if it is not a JSON object of this kind.

    Raises:
        SchemaMismatch: If `writer.schema` is not this library's.
    """
    try:
        data = json.loads(path.read_bytes().decode("utf-8"), parse_constant=_refuse_constant)
    except (UnicodeDecodeError, ValueError) as error:
        problems.append(f"A1: {name}: not UTF-8 JSON ({error})")
        return None
    if not isinstance(data, dict) or data.get("format") != kind:
        found = data.get("format") if isinstance(data, dict) else None
        problems.append(f"A1: {name}: format is {found!r}, expected {kind!r}")
        return None
    writer = data.get("writer")
    if not isinstance(writer, dict) or "schema" not in writer:
        problems.append(f"A1: {name}: no writer.schema")
        return None
    if writer["schema"] != SCHEMA:
        raise SchemaMismatch(name, writer["schema"], str(writer.get("commit", "unknown")), SCHEMA)
    return data


def check_keys(value: Any, kind: str, where: str, problems: List[str]) -> None:
    """Check a JSON value against `KEYS` (A2): required keys present, no unknown key, every type right.

    Args:
        value: The value.
        kind: A type as in `KEYS`, e.g. "design" or "[str".
        where: Where it is, for messages, e.g. "design.json bodies bars/B1".
        problems: Gets one line per problem.
    """
    if kind.startswith("null|"):
        if value is None:
            return
        kind = kind[len("null|"):]
    if kind.startswith(("{", "[")):
        expected = dict if kind[0] == "{" else list
        if not isinstance(value, expected):
            problems.append(f"A2: {where}: expected {'an object' if expected is dict else 'a list'}")
            return
        items = value.items() if expected is dict else enumerate(value)
        for key, item in items:
            check_keys(item, kind[1:], f"{where} {key}", problems)
    elif kind in KEYS:
        if not isinstance(value, dict):
            problems.append(f"A2: {where}: expected an object")
            return
        keys = {key.lstrip("?"): type_ for key, type_ in KEYS[kind].items()}
        missing = [key for key in KEYS[kind] if not key.startswith("?") and key not in value]
        unknown = [key for key in value if key not in keys]
        if missing:
            problems.append(f"A2: {where}: missing {missing}")
        if unknown:
            problems.append(f"A2: {where}: unknown keys {unknown}")
        if kind == "shape" and sum(key in value for key in ("mesh", "box", "cylinder")) != 1:
            problems.append(f"A2: {where}: a shape has exactly one of mesh, box, cylinder")
        if kind == "tool_change" and not value:
            problems.append(f"A2: {where}: a tool change has a grip, an on, or both")
        for key, item in value.items():
            if key in keys:
                check_keys(item, keys[key], f"{where} {key}", problems)
    elif kind == "pose" and _scalar_ok(value, "nums") and len(value) != 7:
        problems.append(f"A7: {where}: a pose has 7 numbers, not {len(value)}")
    elif not _scalar_ok(value, "nums" if kind == "pose" else kind):
        problems.append(f"A2: {where}: expected {kind}, got {value!r}"[:200])


def _number(value: Any) -> bool:
    """Whether a JSON value is a number (bools are not)."""
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _scalar_ok(value: Any, kind: str) -> bool:
    """Whether a value has a scalar type of `KEYS`."""
    if kind == "str":
        return isinstance(value, str)
    if kind == "bool":
        return isinstance(value, bool)
    if kind == "num":
        return _number(value)
    if kind == "nums":
        return isinstance(value, list) and all(_number(item) for item in value)
    if kind == "notes":
        return isinstance(value, dict)  # ? its values are A13's
    return kind == "json"


class _Meshes:
    """Mesh shapes of one read: one object per file (and per file + origin)."""

    def __init__(self, folder: Path, problems: List[str]):
        self.folder, self.problems = folder, problems
        self.cache = MeshCache()
        self.moved: Dict[Tuple[str, Pose], TriMesh] = {}

    def get(self, reference: str, origin: Optional[Pose], owner: str) -> Optional[TriMesh]:
        """The mesh of a `{"mesh": ...}` shape, moved by its `origin`; None (and a problem noted) if missing.

        Args:
            reference: The file, relative to the design folder.
            origin: The shape's `origin`, or None.
            owner: Id of the tool or body, for messages.

        """
        path = self.folder / reference
        if not path.is_file():
            self.problems.append(f"A4: {owner}: mesh file {reference!r} does not exist")
            return None
        mesh = self.cache.get(path)
        if origin is None or origin == Pose():
            return mesh
        key = (reference, origin)
        if key not in self.moved:
            matrix = origin.matrix()
            self.moved[key] = TriMesh.from_arrays(mesh.vertices @ matrix[:3, :3].T + matrix[:3, 3], mesh.faces)
        return self.moved[key]


# --- --- --- --- --- DESIGN.JSON --- --- --- --- ---

def _pose(values) -> Pose:
    """A pose from `[x, y, z, qx, qy, qz, qw]` (its length is checked by `check_keys`)."""
    return Pose.from_arrays(values[:3], values[3:])


def _writer(raw: Dict[str, Any]) -> Writer:
    """The `writer` block."""
    return Writer(int(raw["schema"]), str(raw["library"]), str(raw["commit"]), bool(raw["dirty"]))


def _producer(raw: Dict[str, Any]) -> Producer:
    """The `producer` block."""
    return Producer(str(raw["repo"]), str(raw["commit"]), bool(raw["dirty"]), str(raw["command"]))


def _robot(key: str, raw: Dict[str, Any], folder: Path) -> RobotSpec:
    """A robot entry; its file paths made absolute."""
    return RobotSpec(id=key, urdf=(folder / raw["urdf"]).resolve(), srdf=(folder / raw["srdf"]).resolve(),
                     serial=raw.get("serial"), tools=dict(raw.get("tools", {})),
                     ground_links=tuple(raw.get("ground_links", ())))


def _shapes(raw_shapes, meshes: _Meshes, owner: str) -> Tuple[Shape, ...]:
    """A list of shapes (format §4.4). Mesh shapes with a missing file are left out (a problem is noted)."""
    shapes = []
    for raw in raw_shapes:
        origin = _pose(raw["origin"]) if "origin" in raw else None
        if "box" in raw:
            shapes.append(BoxShape(tuple(float(v) for v in raw["box"]), origin or Pose()))
        elif "cylinder" in raw:
            radius, length = raw["cylinder"]
            shapes.append(CylinderShape(float(radius), float(length), origin or Pose()))
        elif "mesh" in raw:
            mesh = meshes.get(raw["mesh"], origin, owner)
            if mesh is not None:
                shapes.append(mesh)
        else:
            raise ValueError(f"{owner}: shape {raw!r} has none of mesh, box, cylinder")
    return tuple(shapes)


def _geometry(raw: Dict[str, Any], meshes: _Meshes, owner: str) -> Geometry:
    """`collision` and `visual` of a tool or body; absent `visual` is the same tuple as `collision`."""
    collision = _shapes(raw["collision"], meshes, owner)
    visual = _shapes(raw["visual"], meshes, owner) if "visual" in raw else collision
    return Geometry(visual, collision)


def _tool(key: str, raw: Dict[str, Any], meshes: _Meshes) -> Tool:
    """A tool entry."""
    return Tool(id=key, geometry=_geometry(raw, meshes, key), tcp=_pose(raw["tcp"]), kind=str(raw["kind"]),
                mount_contacts=tuple(raw.get("mount_contacts", ())))


def _body(key: str, raw: Dict[str, Any], meshes: _Meshes) -> BodySpec:
    """A body entry."""
    return BodySpec(id=key, pose=_pose(raw["pose"]), geometry=_geometry(raw, meshes, key),
                    label=str(raw.get("label", "")), part=str(raw.get("part", "")),
                    markers={name: tuple(float(v) for v in point) for name, point in raw.get("markers", {}).items()},
                    mount=raw.get("mount"))


# --- --- --- --- --- ACTION FILES --- --- --- --- ---

def _joints(raw) -> Optional[Dict[str, float]]:
    """A joint map, or None for `null`."""
    return None if raw is None else {name: float(value) for name, value in raw.items()}


def _holders(raw: Dict[str, Any]) -> Dict[str, Tuple[Holder, ...]]:
    """An `attached` map: bar -> its holders."""
    return {bar: tuple(Holder(value["to"], _pose(value["grasp"])) for value in holders)
            for bar, holders in raw.items()}


def _state(raw: Dict[str, Any]) -> State:
    """A State (format §5.2)."""
    robots = {}
    for robot, value in raw["robots"].items():
        robots[robot] = None if value is None else RobotState(None if value["base"] is None else _pose(value["base"]),
                                                              _joints(value["joints"]))
    return State(robots=robots,
                 present=frozenset(raw["present"]),
                 attached=_holders(raw.get("attached", {})),
                 built=frozenset(raw.get("built", ())),
                 poses={body: _pose(pose) for body, pose in raw.get("poses", {}).items()},
                 tools={tool: None if value is None else ToolState(value["grip"], value["on"])
                        for tool, value in raw["tools"].items()})


def _target(raw: Optional[Dict[str, Any]]) -> Optional[Target]:
    """A Target (format §5.3), or None for `null` or absent."""
    if raw is None:
        return None
    return Target(joints={robot: _joints(values) for robot, values in raw.get("joints", {}).items()},
                  links={link: _pose(pose) for link, pose in raw.get("links", {}).items()},
                  tools={tool: value["grip"] for tool, value in raw.get("tools", {}).items() if "grip" in value},
                  on={tool: value["on"] for tool, value in raw.get("tools", {}).items() if "on" in value},
                  attached=_holders(raw["attached"]) if "attached" in raw else None,
                  built=frozenset(raw["built"]) if "built" in raw else None)


def _movement(raw: Dict[str, Any]) -> Movement:
    """A Movement (format §5.1)."""
    return Movement(id=raw["id"], start=_state(raw["start"]), arms=tuple(raw.get("arms", ())), path=raw.get("path"),
                    coupled=bool(raw.get("coupled", False)), controller=raw.get("controller"),
                    line={flange: LineSpec(tuple(float(v) for v in value["direction"]), float(value["distance"]))
                          for flange, value in raw.get("line", {}).items()},
                    drives=dict(raw.get("drives", {})),
                    ends_on=str(raw.get("ends_on", "target")), target=_target(raw.get("target")),
                    label=str(raw.get("label", "")), notes=dict(raw.get("notes", {})))


def _action(raw: Dict[str, Any]) -> Action:
    """An action file (format §5)."""
    return Action(id=raw["id"], type=raw["type"], robot=raw["robot"], bar=raw["bar"],
                  movements=tuple(_movement(movement) for movement in raw["movements"]),
                  ground=tuple(raw.get("ground", ())), supports_until=tuple(raw.get("supports_until", ())),
                  label=str(raw.get("label", "")), notes=dict(raw.get("notes", {})))
