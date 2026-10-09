"""
Write a `Design` into a design folder (doc/design_format.md), then read it back; and the content hash of a file.

The writer is deterministic: ids sorted, floats rounded to FLOAT_DIGITS, each mesh in `meshes/<content hash>.obj`, so
equal designs give equal files and `design_hashes` can tell the hashes without writing. Keys at their default, and
`visual` equal to `collision`, are left out.
"""

from __future__ import annotations

import json
import re
import shutil
from hashlib import sha1, sha256
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from ..geometry import BoxShape, CylinderShape, Geometry, Pose, Shape, TriMesh
from ..urdf import copy_robot, robot_files_hash
from .meshes import write_mesh
from .read import ACTION_FORMAT, DESIGN_FORMAT, read
from .types import Action, Design, Holder, Movement, Producer, RobotSpec, State, Target, Writer
from .validate import validate
from .version import writer_info

#: Decimals kept in written floats: a picometre, far below any measurement or calibration.
FLOAT_DIGITS = 12


def write(design: Design, folder: Path, *, overwrite: bool = False, package_dirs: Sequence[Path] = ()) -> Design:
    """Write a design into a folder; `source/`, `solutions/` and `runs/` in it are left as they are.

    Args:
        design: The design; validated first (A3–A13; the robot meshes only on the read-back).
        folder: The design folder. Created if missing.
        overwrite: Write into a non-empty folder, deleting its old action files and meshes first.
        package_dirs: Where to find ROS packages named by `package://` mesh references in robot URDFs.

    Returns:
        Design: The design as read back from the folder.

    Raises:
        DesignError: If the design breaks a format rule.
        FileExistsError: If the folder is not empty and `overwrite` is False.
        FileNotFoundError: If a mesh referenced by a robot URDF is missing.
    """
    # ? Robot meshes are checked on the read-back: source URDFs may use `package://`; only the copies must be relative.
    validate(design, check_robot_meshes=False)
    folder = Path(folder).resolve()
    if folder.exists() and any(folder.iterdir()):
        if not overwrite:
            raise FileExistsError(f"{folder} is not empty; pass overwrite=True to replace the design in it")
        for stale in (folder / "actions").glob("*.json"):
            stale.unlink()
        shutil.rmtree(folder / "meshes", ignore_errors=True)
    folder.mkdir(parents=True, exist_ok=True)

    writer = _writer(writer_info())
    robots = {robot_id: _robot(robot, folder, package_dirs) for robot_id, robot in sorted(design.robots.items())}
    _write_json(folder / "design.json", _manifest(design, writer, robots, _MeshFiles(folder)))
    (folder / "actions").mkdir(exist_ok=True)
    for action_id in design.schedule:
        _write_json(folder / "actions" / f"{action_id}.json", _action(design.actions[action_id], writer))
    return read(folder)


def _manifest(design: Design, writer: Dict[str, Any], robots: Dict[str, Any], meshes: "_MeshFiles") -> Dict[str, Any]:
    """The content of `design.json`, given the robot entries and where meshes go."""
    return {
        "format": DESIGN_FORMAT,
        "writer": writer,
        **({"producer": _producer(design.producer)} if design.producer is not None else {}),
        "robots": robots,
        "tools": {tool_id: _drop_empty({**_geometry(tool.geometry, meshes), "tcp": _pose(tool.tcp),
                                        "kind": tool.kind, "mount_contacts": list(tool.mount_contacts)})
                  for tool_id, tool in sorted(design.tools.items())},
        "bodies": {body_id: _drop_empty({"pose": _pose(body.pose), **_geometry(body.geometry, meshes),
                                         "label": body.label, "part": body.part,
                                         "markers": {name: [_float(v) for v in point]
                                                     for name, point in sorted(body.markers.items())},
                                         "mount": body.mount})
                   for body_id, body in sorted(design.bodies.items())},
        **_drop_empty({"mates": [list(pair) for pair in sorted(design.mates)],
                       "parts": {name: {"seat": _pose(part.seat)} for name, part in sorted(design.parts.items())}}),
        "schedule": list(design.schedule),
    }


def design_hashes(design: Design, package_dirs: Sequence[Path] = ()) -> Tuple[str, Dict[str, str]]:
    """The content hashes `write` would give, without writing: of `design.json` and of every action file.

    Compare them with a solution's `solved_against` to tell whether it is stale, e.g. while the design is open in Rhino.

    Args:
        design: The design, in memory or read from a folder.
        package_dirs: As for `write`: where to find packages named by `package://` in the robot URDFs.

    Returns:
        tuple[str, dict[str, str]]: The hash of `design.json`, and action id -> the hash of its file.
    """
    robots = {robot_id: _robot_entry(robot, f"robots/{robot.name}/robot.urdf", f"robots/{robot.name}/robot.srdf",
                                     robot_files_hash(robot.urdf, robot.srdf, package_dirs))
              for robot_id, robot in sorted(design.robots.items())}
    manifest = _manifest(design, {}, robots, _MeshFiles(None))
    return _value_hash(manifest), {action_id: _value_hash(_action(design.actions[action_id], {}))
                                   for action_id in design.schedule}


# --- --- --- --- --- ROBOTS AND MESHES --- --- --- --- ---

def _inside(path: Path, folder: Path) -> bool:
    """Whether `path` is inside `folder`."""
    try:
        Path(path).resolve().relative_to(folder.resolve())
        return True
    except ValueError:
        return False


def _robot(robot: RobotSpec, folder: Path, package_dirs: Sequence[Path]) -> Dict[str, Any]:
    """A robot entry; its files copied into `robots/<name>/` unless they are there already."""
    robot_dir = folder / "robots" / robot.name
    if _inside(robot.urdf, robot_dir) and _inside(robot.srdf, robot_dir):
        urdf, srdf = Path(robot.urdf).resolve(), Path(robot.srdf).resolve()
    else:
        # ? A fresh copy: remove what an earlier write left in this robot's folder.
        shutil.rmtree(robot_dir, ignore_errors=True)
        urdf, srdf = copy_robot(robot.urdf, robot.srdf, robot_dir, package_dirs)
    return _robot_entry(robot, _relative(urdf, folder), _relative(srdf, folder), robot_files_hash(urdf, srdf))


def _robot_entry(robot: RobotSpec, urdf: str, srdf: str, files_hash: str) -> Dict[str, Any]:
    """A robot entry with its file paths in the design and the hash of those files."""
    return _drop_empty({"urdf": urdf, "srdf": srdf, "files_hash": files_hash, "serial": robot.serial,
                        "tools": dict(sorted(robot.tools.items())), "ground_links": list(robot.ground_links)})


def _relative(path: Path, folder: Path) -> str:
    """A path inside the design folder, relative to it, with `/`."""
    return Path(path).resolve().relative_to(folder).as_posix()


class _MeshFiles:
    """The mesh files of one write, each named by its content: `meshes/<hash>.obj`, written on first use."""

    def __init__(self, folder: Optional[Path]):
        """Write into `folder`; None: only name the files (`design_hashes`)."""
        self.folder = folder
        self.by_object: Dict[int, str] = {}
        self.written: set = set()
        # ? Keep the meshes alive: `by_object` is keyed by object identity.
        self.kept: List[TriMesh] = []

    def reference(self, mesh: TriMesh) -> str:
        """The file of a mesh, relative to the design folder; written on first use."""
        if id(mesh) not in self.by_object:
            content = sha1(np.ascontiguousarray(mesh.vertices, dtype=np.float64).tobytes()
                           + b"|" + np.ascontiguousarray(mesh.faces, dtype=np.int64).tobytes()).hexdigest()
            name = f"meshes/{content[:16]}.obj"
            if self.folder is not None and name not in self.written:
                write_mesh(mesh, self.folder / name)
                self.written.add(name)
            self.by_object[id(mesh)] = name
            self.kept.append(mesh)
        return self.by_object[id(mesh)]


# --- --- --- --- --- TO JSON VALUES --- --- --- --- ---

def _drop_empty(entry: Dict[str, Any]) -> Dict[str, Any]:
    """Leave out optional keys at their default: None, False, "", and empty lists or maps."""
    return {key: value for key, value in entry.items()
            if not (value is None or value is False or (isinstance(value, (str, list, dict)) and not value))}


def _writer(writer: Writer) -> Dict[str, Any]:
    """The `writer` block."""
    return {"schema": writer.schema, "library": writer.library, "commit": writer.commit, "dirty": writer.dirty}


def _producer(producer: Producer) -> Dict[str, Any]:
    """The `producer` block."""
    return {"repo": producer.repo, "commit": producer.commit, "dirty": producer.dirty, "command": producer.command}


def _float(value: float) -> float:
    """A float rounded to FLOAT_DIGITS, so noise like -3.4e-20 is written as 0.0 (`+ 0.0` turns -0.0 into 0.0)."""
    return round(float(value), FLOAT_DIGITS) + 0.0


def _pose(pose: Pose) -> List[float]:
    """A pose as `[x, y, z, qx, qy, qz, qw]`."""
    return [_float(v) for v in (*pose.position, *pose.orientation)]


def _shape(shape: Shape, meshes: _MeshFiles) -> Dict[str, Any]:
    """One shape (format §4.4); `origin` left out at identity."""
    if isinstance(shape, TriMesh):
        return {"mesh": meshes.reference(shape)}
    if isinstance(shape, BoxShape):
        entry: Dict[str, Any] = {"box": [_float(v) for v in shape.size]}
    elif isinstance(shape, CylinderShape):
        entry = {"cylinder": [_float(shape.radius), _float(shape.height)]}
    else:
        raise TypeError(f"unknown shape {shape!r}")
    if shape.origin != Pose():
        entry["origin"] = _pose(shape.origin)
    return entry


def _geometry(geometry: Geometry, meshes: _MeshFiles) -> Dict[str, Any]:
    """`collision`, plus `visual` when it differs (same shapes in the same order counts as equal)."""
    entry = {"collision": [_shape(shape, meshes) for shape in geometry.collision]}
    # ? TriMesh compares by object and primitives by value, so this is "same shapes, same order".
    if tuple(geometry.visual) != tuple(geometry.collision):
        entry["visual"] = [_shape(shape, meshes) for shape in geometry.visual]
    return entry


def _joints(joints: Optional[Dict[str, float]]) -> Optional[Dict[str, float]]:
    """A joint map as plain floats, or None."""
    return None if joints is None else {name: _float(value) for name, value in joints.items()}


def _holders(attached: Dict[str, Tuple[Holder, ...]]) -> Dict[str, Any]:
    """An `attached` map, bars sorted; each bar's holders keep their order (the first sets the pose)."""
    return {bar: [{"to": holder.to, "grasp": _pose(holder.grasp)} for holder in holders]
            for bar, holders in sorted(attached.items())}


def _state(state: State) -> Dict[str, Any]:
    """A State (format §5.2), sets sorted so equal states give equal files."""
    robots = {robot: None if value is None else {"base": None if value.base is None else _pose(value.base),
                                                 "joints": _joints(value.joints)}
              for robot, value in state.robots.items()}
    return {"robots": robots, "present": sorted(state.present), **_drop_empty({
        "attached": _holders(state.attached), "built": sorted(state.built),
        "poses": {body: _pose(pose) for body, pose in sorted(state.poses.items())}}),
        "tools": {tool: None if value is None else {"grip": value.grip, "on": value.on}
                  for tool, value in sorted(state.tools.items())}}


def _target(target: Target) -> Dict[str, Any]:
    """A Target (format §5.3); `on`, `attached` and `built` only when the movement changes them."""
    tools: Dict[str, Dict[str, Any]] = {tool: {"grip": grip} for tool, grip in target.tools.items()}
    for tool, body in target.on.items():
        tools.setdefault(tool, {})["on"] = body
    entry = _drop_empty({"joints": {robot: _joints(joints) for robot, joints in target.joints.items()},
                         "links": {link: _pose(pose) for link, pose in target.links.items()},
                         "tools": dict(sorted(tools.items()))})
    if target.attached is not None:
        entry["attached"] = _holders(target.attached)
    if target.built is not None:
        entry["built"] = sorted(target.built)
    return entry


def _movement(movement: Movement) -> Dict[str, Any]:
    """A Movement (format §5.1): only the parts it has; `ends_on` left out at "target"."""
    entry: Dict[str, Any] = {"id": movement.id, **_drop_empty({"label": movement.label})}
    entry.update(_drop_empty({
        "arms": list(movement.arms), "path": movement.path, "coupled": movement.coupled,
        "controller": movement.controller,
        "line": {flange: {"direction": [_float(v) for v in line.direction], "distance": _float(line.distance)}
                 for flange, line in movement.line.items()},
        "drives": dict(sorted(movement.drives.items()))}))
    if movement.ends_on != "target":
        entry["ends_on"] = movement.ends_on
    entry["start"] = _state(movement.start)
    if movement.target is not None:
        entry["target"] = _target(movement.target)
    entry.update(_drop_empty({"notes": dict(movement.notes)}))
    return entry


def _action(action: Action, writer: Dict[str, Any]) -> Dict[str, Any]:
    """An action file (format §5)."""
    return {"format": ACTION_FORMAT, "writer": writer, "id": action.id, "type": action.type,
            "robot": action.robot, "bar": action.bar,
            **_drop_empty({"ground": list(action.ground), "supports_until": list(action.supports_until),
                           "label": action.label, "notes": dict(action.notes)}),
            "movements": [_movement(movement) for movement in action.movements]}


# --- --- --- --- --- JSON TEXT --- --- --- --- ---

#: A list of numbers as `json.dumps(indent=2)` spreads it over lines; it can only start outside a string.
_NUMBER_LIST = re.compile(r"\[\n[-+.\deE,\s]+\]")


def _write_json(path: Path, value: Any) -> None:
    """Write one JSON file indented by 2, with each list of numbers (a pose, joints) on one line."""
    text = json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False)
    # * "[\n  1.0,\n  2.0\n]" -> "[1.0, 2.0]"
    text = _NUMBER_LIST.sub(lambda match: "[" + " ".join(match.group()[1:-1].split()) + "]", text)
    path.write_text(text + "\n", encoding="utf-8")


# --- --- --- --- --- CONTENT HASH --- --- --- --- ---

def content_hash(path: Path) -> str:
    """The SHA-256 of a JSON file's content, without `writer` and `producer`.

    Keys sorted and floats rounded to FLOAT_DIGITS, so re-exporting an unchanged design gives the same hash from any
    library commit. Files outside the design (solutions, runs, planner caches) record it to notice a changed design.

    Args:
        path: `design.json`, an action file, or any other JSON file.

    Returns:
        str: 64 hex digits.
    """
    return _value_hash(json.loads(Path(path).read_text(encoding="utf-8")))


def _value_hash(data: Dict[str, Any]) -> str:
    """`content_hash` of a JSON object as it would be written."""
    data = {key: value for key, value in json.loads(json.dumps(data)).items() if key not in ("writer", "producer")}
    text = json.dumps(_canonical(data), sort_keys=True, separators=(",", ":"), ensure_ascii=False, allow_nan=False)
    return sha256(text.encode("utf-8")).hexdigest()


def _canonical(value: Any) -> Any:
    """A JSON value with every float rounded as the writer rounds it."""
    if isinstance(value, float):
        return _float(value)
    if isinstance(value, list):
        return [_canonical(item) for item in value]
    if isinstance(value, dict):
        return {key: _canonical(item) for key, item in value.items()}
    return value
