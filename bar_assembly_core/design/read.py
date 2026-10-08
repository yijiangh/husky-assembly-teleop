"""
Read a design folder (doc/design_format.md) into a `Design`: check every file's format and schema, parse, validate.

Files written by different commits of the same schema read fine; another schema raises `SchemaMismatch`.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from ..geometry import BoxShape, CylinderShape, Geometry, Pose, Shape, TriMesh
from ..robot import Tool
from .meshes import MeshCache
from .types import (Action, BodySpec, Carried, Design, DesignError, LineSpec, Movement, Producer, RobotSpec, RobotState,
                    SchemaMismatch, State, Target, Writer)
from .validate import validate
from .version import SCHEMA

DESIGN_FORMAT, ACTION_FORMAT = "husky_design", "husky_design/action"


def read(folder: Path) -> Design:
    """Read and validate a design folder.

    Args:
        folder: The design folder (holds `design.json`).

    Returns:
        Design: With `folder` and robot file paths absolute.

    Raises:
        SchemaMismatch: If any file has another schema; names the commit that wrote it.
        DesignError: If a file is missing or malformed, or the design breaks a format rule
            (every problem found is listed).
    """
    folder = Path(folder).resolve()
    manifest_path = folder / "design.json"
    if not manifest_path.is_file():
        raise DesignError([f"no design.json in {folder}"])

    # * 1. Load every file and check its kind and schema before looking at anything else.
    manifest = _load(manifest_path, "design.json", DESIGN_FORMAT)
    raw_actions = {}
    for path in sorted((folder / "actions").glob("*.json")):
        raw_actions[path.stem] = _load(path, f"actions/{path.name}", ACTION_FORMAT)

    # * 2. Parse. File-only problems (a missing mesh, a wrong action id) are reported with what `validate` finds.
    problems: List[str] = []
    meshes = _Meshes(folder, problems)
    try:
        writer = _writer(manifest["writer"])
        producer = _producer(manifest["producer"]) if "producer" in manifest else None
        robots = {key: _robot(key, value, folder) for key, value in manifest["robots"].items()}
        tools = {key: _tool(key, value, meshes) for key, value in manifest["tools"].items()}
        bodies = {key: _body(key, value, meshes) for key, value in manifest["bodies"].items()}
        connections = frozenset(_pair(pair) for pair in manifest.get("connections", ()))
        schedule = tuple(manifest["schedule"])
    except (KeyError, TypeError, ValueError, AttributeError) as error:
        raise DesignError([f"design.json: malformed ({type(error).__name__}: {error})"]) from error
    actions = {}
    for name, raw in raw_actions.items():
        try:
            actions[name] = _action(raw)
        except (KeyError, TypeError, ValueError, AttributeError) as error:
            raise DesignError([f"actions/{name}.json: malformed ({type(error).__name__}: {error})"]) from error
        if actions[name].id != name:
            problems.append(f"rule 3: actions/{name}.json has id {actions[name].id!r}, not its file name")

    design = Design(folder=folder, writer=writer, robots=robots, tools=tools, bodies=bodies, schedule=schedule,
                    actions=actions, connections=connections, producer=producer)
    # * 3. Validate: every problem at once.
    try:
        validate(design)
    except DesignError as error:
        problems.extend(error.problems)
    if problems:
        raise DesignError(problems)
    return design


# --- --- --- --- --- FILES --- --- --- --- ---

def _load(path: Path, name: str, kind: str) -> Dict[str, Any]:
    """Load one JSON file and check its `format` and `writer.schema`.

    Args:
        path: The file.
        name: Its path in the design folder, for messages.
        kind: The `format` it must have.

    Returns:
        dict: The parsed file.

    Raises:
        SchemaMismatch: If `writer.schema` is not this library's.
        DesignError: If it is not JSON, or not a file of this kind.
    """
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except ValueError as error:
        raise DesignError([f"{name}: not valid JSON ({error})"]) from error
    if not isinstance(data, dict) or data.get("format") != kind:
        found = data.get("format") if isinstance(data, dict) else None
        raise DesignError([f"rule 1: {name}: format is {found!r}, expected {kind!r}"])
    writer = data.get("writer")
    if not isinstance(writer, dict) or "schema" not in writer:
        raise DesignError([f"rule 1: {name}: no writer.schema"])
    if writer["schema"] != SCHEMA:
        raise SchemaMismatch(name, writer["schema"], str(writer.get("commit", "unknown")), SCHEMA)
    return data


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
            self.problems.append(f"rule 3: {owner}: mesh file {reference!r} does not exist")
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
    """A pose from `[x, y, z, qx, qy, qz, qw]`."""
    if len(values) != 7:
        raise ValueError(f"a pose has 7 numbers, got {values!r}")
    return Pose.from_arrays(values[:3], values[3:])


def _writer(raw: Dict[str, Any]) -> Writer:
    """The `writer` block."""
    return Writer(int(raw["schema"]), str(raw["library"]), str(raw["commit"]), bool(raw["dirty"]))


def _producer(raw: Dict[str, Any]) -> Producer:
    """The `producer` block."""
    return Producer(str(raw["repo"]), str(raw["commit"]), bool(raw["dirty"]), str(raw["command"]))


def _pair(raw) -> Tuple[str, str]:
    """A pair of ids, sorted."""
    first, second = raw
    return tuple(sorted((str(first), str(second))))


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
                    markers={name: tuple(float(v) for v in point) for name, point in raw.get("markers", {}).items()})


# --- --- --- --- --- ACTION FILES --- --- --- --- ---

def _joints(raw) -> Optional[Dict[str, float]]:
    """A joint map, or None for `null`."""
    return None if raw is None else {name: float(value) for name, value in raw.items()}


def _state(raw: Dict[str, Any]) -> State:
    """A State (format §5.2)."""
    robots = {}
    for robot, value in raw["robots"].items():
        robots[robot] = None if value is None else RobotState(None if value["base"] is None else _pose(value["base"]),
                                                              _joints(value["joints"]))
    return State(robots=robots,
                 present=frozenset(raw["present"]),
                 poses={body: _pose(pose) for body, pose in raw.get("poses", {}).items()},
                 carried={body: Carried(value["to"], _pose(value["offset"]))
                          for body, value in raw.get("carried", {}).items()},
                 tools={tool: None if value is None else {key: None if v is None else str(v)
                                                          for key, v in value.items()}
                        for tool, value in raw.get("tools", {}).items()})


def _target(raw: Optional[Dict[str, Any]]) -> Optional[Target]:
    """A Target (format §5.3), or None for `null` or absent."""
    if raw is None:
        return None
    return Target(joints={robot: _joints(values) for robot, values in raw.get("joints", {}).items()},
                  links={link: _pose(pose) for link, pose in raw.get("links", {}).items()},
                  tools={tool: {channel: str(value) for channel, value in channels.items()}
                         for tool, channels in raw.get("tools", {}).items()})


def _movement(raw: Dict[str, Any]) -> Movement:
    """A Movement (format §5.1)."""
    return Movement(id=raw["id"], start=_state(raw["start"]), arms=tuple(raw.get("arms", ())), path=raw.get("path"),
                    coupled=bool(raw.get("coupled", False)), controller=raw.get("controller"),
                    line={flange: LineSpec(tuple(float(v) for v in value["direction"]), float(value["distance"]))
                          for flange, value in raw.get("line", {}).items()},
                    ends_on=str(raw.get("ends_on", "target")), target=_target(raw.get("target")),
                    label=str(raw.get("label", "")), notes=dict(raw.get("notes", {})))


def _action(raw: Dict[str, Any]) -> Action:
    """An action file (format §5)."""
    return Action(id=raw["id"], type=raw["type"], robot=raw["robot"], bar=raw["bar"],
                  movements=tuple(_movement(movement) for movement in raw["movements"]),
                  ground=tuple(raw.get("ground", ())), supports_until=tuple(raw.get("supports_until", ())),
                  label=str(raw.get("label", "")), notes=dict(raw.get("notes", {})))
