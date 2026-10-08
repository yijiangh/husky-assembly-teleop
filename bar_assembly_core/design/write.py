"""
Write a `Design` into a design folder (doc/design_format.md), then read it back.

Equal meshes share one file `meshes/<first owner id>.obj`; keys at their default, and `visual` equal to
`collision`, are left out.
"""

from __future__ import annotations

import json
import re
import shutil
from hashlib import sha1
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from ..geometry import BoxShape, CylinderShape, Geometry, Shape, TriMesh
from .meshes import write_mesh
from ..geometry import Pose
from .read import ACTION_FORMAT, DESIGN_FORMAT, read
from ..urdf import copy_robot
from .types import Action, Design, Movement, RobotSpec, State, Target, Writer
from .validate import validate
from .version import writer_info

#: Decimals kept in written floats: a picometre, far below any measurement or calibration.
FLOAT_DIGITS = 12


def write(design: Design, folder: Path, *, overwrite: bool = False, package_dirs: Sequence[Path] = ()) -> Design:
    """Write a design into a folder.

    Args:
        design: The design; validated first (rule 11 only on the read-back).
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
    # ? Rule 11 is left to the read-back: source URDFs may use `package://`; only the copies must be relative.
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
    robots = {robot_id: _robot(robot, folder, package_dirs) for robot_id, robot in design.robots.items()}
    meshes = _MeshFiles(folder)
    manifest = {
        "format": DESIGN_FORMAT,
        "writer": writer,
        "robots": robots,
        "tools": {tool_id: _drop_empty({**_geometry(tool.geometry, meshes, tool_id), "tcp": _pose(tool.tcp),
                                        "kind": tool.kind, "touches": list(tool.touches)})
                  for tool_id, tool in design.tools.items()},
        "bodies": {body_id: _drop_empty({"pose": _pose(body.pose), **_geometry(body.geometry, meshes, body_id),
                                         "touches": list(body.touches), "label": body.label})
                   for body_id, body in design.bodies.items()},
        "schedule": list(design.schedule),
    }
    _write_json(folder / "design.json", manifest)
    (folder / "actions").mkdir(exist_ok=True)
    for action_id in design.schedule:
        _write_json(folder / "actions" / f"{action_id}.json", _action(design.actions[action_id], writer))
    return read(folder)


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
    return _drop_empty({"urdf": _relative(urdf, folder), "srdf": _relative(srdf, folder),
                        "serial": robot.serial, "tools": dict(robot.tools)})


def _relative(path: Path, folder: Path) -> str:
    """A path inside the design folder, relative to it, with `/`."""
    return Path(path).resolve().relative_to(folder).as_posix()


class _MeshFiles:
    """The mesh files of one write: one file per mesh object, and per distinct content."""

    def __init__(self, folder: Path):
        self.folder = folder
        self.by_object: Dict[int, str] = {}
        self.by_content: Dict[str, str] = {}
        self.used: set = set()
        # ? Keep the meshes alive: `by_object` is keyed by object identity.
        self.kept: List[TriMesh] = []

    def reference(self, mesh: TriMesh, owner: str) -> str:
        """The file of a mesh, written on first use.

        Args:
            mesh: The mesh.
            owner: Id of the tool or body using it; names the file if this is its first use.

        Returns:
            str: The file, relative to the design folder.
        """
        if id(mesh) in self.by_object:
            return self.by_object[id(mesh)]
        content = sha1(np.ascontiguousarray(mesh.vertices, dtype=np.float64).tobytes()
                       + b"|" + np.ascontiguousarray(mesh.faces, dtype=np.int64).tobytes()).hexdigest()
        if content not in self.by_content:
            name, number = f"meshes/{owner}.obj", 0
            while name in self.used:
                number += 1
                name = f"meshes/{owner}_{number}.obj"
            self.used.add(name)
            write_mesh(mesh, self.folder / name)
            self.by_content[content] = name
        self.by_object[id(mesh)] = self.by_content[content]
        self.kept.append(mesh)
        return self.by_content[content]


# --- --- --- --- --- TO JSON VALUES --- --- --- --- ---

def _drop_empty(entry: Dict[str, Any]) -> Dict[str, Any]:
    """Leave out optional keys at their default: None, False, "", and empty lists or maps."""
    return {key: value for key, value in entry.items()
            if not (value is None or value is False or (isinstance(value, (str, list, dict)) and not value))}


def _writer(writer: Writer) -> Dict[str, Any]:
    """The `writer` block."""
    return {"schema": writer.schema, "library": writer.library, "commit": writer.commit, "dirty": writer.dirty}


def _float(value: float) -> float:
    """A float rounded to FLOAT_DIGITS, so noise like -3.4e-20 is written as 0.0 (`+ 0.0` turns -0.0 into 0.0)."""
    return round(float(value), FLOAT_DIGITS) + 0.0


def _pose(pose: Pose) -> List[float]:
    """A pose as `[x, y, z, qx, qy, qz, qw]`."""
    return [_float(v) for v in (*pose.position, *pose.orientation)]


def _shape(shape: Shape, meshes: _MeshFiles, owner: str) -> Dict[str, Any]:
    """One shape (format §4.4); `origin` left out at identity."""
    if isinstance(shape, TriMesh):
        return {"mesh": meshes.reference(shape, owner)}
    if isinstance(shape, BoxShape):
        entry: Dict[str, Any] = {"box": [_float(v) for v in shape.size]}
    elif isinstance(shape, CylinderShape):
        entry = {"cylinder": [_float(shape.radius), _float(shape.height)]}
    else:
        raise TypeError(f"{owner}: unknown shape {shape!r}")
    if shape.origin != Pose():
        entry["origin"] = _pose(shape.origin)
    return entry


def _geometry(geometry: Geometry, meshes: _MeshFiles, owner: str) -> Dict[str, Any]:
    """`collision`, plus `visual` when it differs (same shapes in the same order counts as equal)."""
    entry = {"collision": [_shape(shape, meshes, owner) for shape in geometry.collision]}
    # ? TriMesh compares by object and primitives by value, so this is "same shapes, same order".
    if tuple(geometry.visual) != tuple(geometry.collision):
        entry["visual"] = [_shape(shape, meshes, owner) for shape in geometry.visual]
    return entry


def _joints(joints: Optional[Dict[str, float]]) -> Optional[Dict[str, float]]:
    """A joint map as plain floats, or None."""
    return None if joints is None else {name: _float(value) for name, value in joints.items()}


def _state(state: State) -> Dict[str, Any]:
    """A State (format §5.2), sets sorted so equal states give equal files."""
    robots = {robot: None if value is None else {"base": _pose(value.base), "joints": _joints(value.joints)}
              for robot, value in state.robots.items()}
    return {"robots": robots, "present": sorted(state.present), **_drop_empty({
        "poses": {body: _pose(pose) for body, pose in state.poses.items()},
        "attached": {body: {"to": value.to, "grasp": _pose(value.grasp)} for body, value in state.attached.items()},
        "touches": [list(pair) for pair in sorted(tuple(sorted(pair)) for pair in state.touches)],
        "placeholder": sorted(state.placeholder)})}


def _target(target: Target) -> Dict[str, Any]:
    """A Target (format §5.3)."""
    return _drop_empty({"joints": {robot: _joints(joints) for robot, joints in target.joints.items()},
                        "links": {link: _pose(pose) for link, pose in target.links.items()}})


def _movement(movement: Movement) -> Dict[str, Any]:
    """A Movement (format §5.1). `arms` and `tools` are written whenever their movement type needs them."""
    entry: Dict[str, Any] = {"id": movement.id, "type": movement.type}
    if movement.arms or movement.type in ("free", "linear"):
        entry["arms"] = list(movement.arms)
    entry.update(_drop_empty({"coupled": movement.coupled}))
    entry["controller"] = movement.controller
    if movement.tools or movement.type == "tool":
        entry["tools"] = list(movement.tools)
    entry.update(_drop_empty({"tool_action": movement.tool_action, "overlaps_next": movement.overlaps_next}))
    entry["start"] = _state(movement.start)
    if movement.target is not None:
        entry["target"] = _target(movement.target)
    entry.update(_drop_empty({"label": movement.label, "notes": dict(movement.notes)}))
    return entry


def _action(action: Action, writer: Dict[str, Any]) -> Dict[str, Any]:
    """An action file (format §5)."""
    return {"format": ACTION_FORMAT, "writer": writer, "id": action.id, "type": action.type,
            "robot": action.robot, "bar": action.bar,
            **_drop_empty({"ground": list(action.ground), "supports_until": list(action.supports_until),
                           "label": action.label}),
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
