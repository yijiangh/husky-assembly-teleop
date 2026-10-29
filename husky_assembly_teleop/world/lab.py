"""
The lab's obstacles as data: shapes on the floor and the border of the usable area, read from and written to a file.

A lab file (`data/lab/<name>.json`, in git) holds resolved shapes: type, name, centre, size, yaw and a free `source`
note, and optionally meshes drawn in place of the shape (`MeshPart`, files in `meshes/` beside the lab file); collisions
always use the shape. Temporary obstacles live in a local file instead (`TEMPORARY_FILE`), never in the lab file.
The floor is not in the file: a slab under the border (`floor_body`). Pure data, files and scene bodies; no UI.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, replace
from functools import lru_cache
from pathlib import Path

import numpy as np

from scaffolding_assembly_core.geometry import (BoxShape, Geometry, Pose, TriMesh, box_geometry, cylinder_geometry,
                                                read_mesh)
from scaffolding_assembly_core.kinematics import chain_joints
from scaffolding_assembly_core.robot import RobotModel
from scaffolding_assembly_core.scene import Body

#: Lab files, under the data directory; `-p lab:=<name>` picks `<name>.json`.
LAB_DIRECTORY = "lab"
#: The lab loaded without a `lab` parameter.
DEFAULT_LAB = "main"
#: Obstacle meshes, beside the lab file: `meshes/<name>.obj`. Rhino exports in millimetres, Y up, each with its origin
#: at the centre of where it stands on the floor.
MESH_DIRECTORY = "meshes"
#: Where temporary obstacles are kept between runs, outside the repo.
TEMPORARY_FILE = Path.home() / "husky_obstacles_temporary.json"
#: Where the debug gizmo kept them before; read once if TEMPORARY_FILE is missing.
OLD_TEMPORARY_FILE = Path.home() / "husky_debug_gizmo.json"

#: Shape types, and the id group of each ("obstacles/<group>/<name>").
GROUPS = {"box": "boxes", "cylinder": "cylinders", "truss": "trusses"}
#: Ground plate under a truss column: side and thickness, metres.
TRUSS_PLATE = 0.8
TRUSS_PLATE_THICKNESS = 0.02
#: The floor: thickness of the slab under z = 0, and its side around the origin while the lab has no border, metres.
FLOOR_THICKNESS = 0.05
FLOOR_WITHOUT_BORDER = 20.0
#: File format name and version.
FORMAT, VERSION = "husky_lab", 1
UNITS = ("lab frame (as the mocap relay publishes poses), metres; yaw in degrees about Z. size: box sides; cylinder "
         "(diameter, diameter, height); truss column (side, side, height) on a 0.8 m ground plate. meshes: drawn "
         "only, never collide; at: metres from the shape's bottom centre in its frame, yaw in degrees")


def slug(name: str) -> str:
    """Turn a display name into an id segment, e.g. "main pc table" -> "main_pc_table"."""
    return re.sub(r"[^A-Za-z0-9_.\-]+", "_", name)


@dataclass(frozen=True)
class MeshPart:
    """A mesh drawn for an obstacle in place of its shape; collisions still use the shape.

    Attributes:
        mesh: File name in MESH_DIRECTORY, without ".obj".
        at: (x, y, z) of the mesh origin from the shape's bottom centre, metres, in the shape's frame.
        yaw: Turn about Z in the shape's frame, radians.
    """

    mesh: str
    at: tuple[float, float, float] = (0.0, 0.0, 0.0)
    yaw: float = 0.0


@dataclass(frozen=True)
class Shape:
    """One obstacle standing in the lab.

    Attributes:
        type: "box", "cylinder" or "truss".
        name: Label, shown in collision messages; its slug is the last part of the id.
        center: (x, y, z) of the shape's middle, metres, lab frame.
        size: Box sides; a cylinder's (diameter, diameter, height); a truss column's (side, side, height). Metres.
        yaw: Turn about Z, radians.
        source: Where the numbers came from, e.g. "probe 2026-09-30" or "by hand".
        meshes: What is drawn in place of the shape; none draws the shape.
    """

    type: str
    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0
    source: str = ""
    meshes: tuple[MeshPart, ...] = ()

    @property
    def pose(self) -> Pose:
        """Pose: The shape's centre, turned by `yaw`."""
        return Pose(self.center, (0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)))

    @property
    def square(self) -> bool:
        """bool: Whether its x and y size must stay equal (a cylinder's diameter, a truss column)."""
        return self.type != "box"

    def geometry(self, mesh_directory: Path | None = None) -> Geometry:
        """The shape for collisions, drawn as its meshes if it has any. ! A new object per call: keep it while it holds.

        Args:
            mesh_directory: Where its meshes are (`mesh_directory_of` the lab file); unused without meshes.
        """
        sx, sy, sz = self.size
        if self.type == "box":
            geometry = box_geometry(self.size)
        elif self.type == "cylinder":
            geometry = cylinder_geometry(sx / 2, sz)
        else:
            shapes = (BoxShape((sx, sy, sz)),
                      BoxShape((TRUSS_PLATE, TRUSS_PLATE, TRUSS_PLATE_THICKNESS),
                               Pose((0.0, 0.0, (TRUSS_PLATE_THICKNESS - sz) / 2))))
            geometry = Geometry(shapes, shapes)
        if not self.meshes:
            return geometry
        visual = tuple(placed_mesh(Path(mesh_directory) / f"{part.mesh}.obj", part, sz) for part in self.meshes)
        # * A mesh read empty (a Git LFS pointer, in a clone without Git LFS) leaves the shape drawn.
        if not all(len(mesh.faces) for mesh in visual):
            return geometry
        return Geometry(visual, geometry.collision)

    def moved(self, pose: Pose) -> Shape:
        """The same shape at `pose`, upright: roll and pitch are dropped, only the yaw is kept."""
        x, y, z, w = pose.orientation
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return replace(self, center=tuple(float(v) for v in pose.position), yaw=yaw)

    def resized(self, size: tuple[float, float, float], minimum: float = 0.01) -> Shape:
        """The same shape with a new size, each side at least `minimum`; x and y made equal if `square`."""
        size = tuple(max(float(v), minimum) for v in size)
        if self.square:
            size = (size[0], size[0], size[2])
        return replace(self, size=size)


@dataclass(frozen=True)
class Border:
    """Walls just outside a floor rectangle, drawn as a shaded band on the floor but colliding up to `height`.

    Attributes:
        x: (min, max) of the free area, metres, lab frame.
        y: (min, max) of the free area, metres, lab frame.
        width: Band width, metres, outward from the free area.
        height: Collision height, metres, from the floor.
        source: Where the numbers came from.
    """

    x: tuple[float, float]
    y: tuple[float, float]
    width: float = 0.2
    height: float = 2.0
    source: str = ""

    #: Drawn thickness of the band, metres.
    SHADE = 0.005

    def walls(self) -> list[tuple[str, Geometry, Pose]]:
        """One wall per side as (name, geometry, pose), drawn SHADE high; the x walls cover the corners."""
        (x0, x1), (y0, y1), w, h = self.x, self.y, self.width, self.height
        sides = (("x min", (x0 - w / 2, (y0 + y1) / 2), (w, y1 - y0 + 2 * w)),
                 ("x max", (x1 + w / 2, (y0 + y1) / 2), (w, y1 - y0 + 2 * w)),
                 ("y min", ((x0 + x1) / 2, y0 - w / 2), (x1 - x0, w)),
                 ("y max", ((x0 + x1) / 2, y1 + w / 2), (x1 - x0, w)))
        walls = []
        for side, (cx, cy), (sx, sy) in sides:
            band = (BoxShape((sx, sy, self.SHADE), Pose((0.0, 0.0, (self.SHADE - h) / 2))),)
            walls.append((f"border {side}", Geometry(band, box_geometry((sx, sy, h)).collision), Pose((cx, cy, h / 2))))
        return walls


@dataclass(frozen=True)
class Lab:
    """A lab's obstacles and the border of its usable floor.

    Attributes:
        shapes: The obstacles, in file order.
        border: The usable floor, or None for none.
    """

    shapes: tuple[Shape, ...] = ()
    border: Border | None = None


def shape_id(plugin: str, shape: Shape) -> str:
    """The scene id of a lab shape: "<plugin>/<group>/<slug of its name>"."""
    return f"{plugin}/{GROUPS[shape.type]}/{slug(shape.name)}"


def border_id(plugin: str, name: str) -> str:
    """The scene id of a border wall: "<plugin>/boxes/border_x_min" and so on."""
    return f"{plugin}/boxes/{slug(name)}"


def floor_id(plugin: str) -> str:
    """The scene id of the floor: "<plugin>/floor"."""
    return f"{plugin}/floor"


def lab_file(data_directory: Path, lab: str) -> Path:
    """The file of lab `lab`: a path to a .json file as given, else `<data>/lab/<lab>.json`."""
    path = Path(lab).expanduser()
    return path if path.suffix == ".json" else data_directory / LAB_DIRECTORY / f"{lab}.json"


def mesh_directory_of(path: Path) -> Path:
    """The meshes of the lab file at `path`: MESH_DIRECTORY beside it."""
    return Path(path).parent / MESH_DIRECTORY


# --- --- --- --- --- MESHES --- --- --- --- ---

@lru_cache(maxsize=None)
def read_obstacle_mesh(path: Path) -> TriMesh:
    """A mesh file of MESH_DIRECTORY in metres, Z up. ? Cached: each file is read once per run."""
    mesh = read_mesh(path)
    vertices = mesh.vertices / 1000.0
    return TriMesh.from_arrays(np.column_stack((vertices[:, 0], -vertices[:, 2], vertices[:, 1])), mesh.faces)


@lru_cache(maxsize=None)
def placed_mesh(path: Path, part: MeshPart, height: float) -> TriMesh:
    """The mesh of `part` in the frame of a shape `height` high, whose origin is its centre. ? Cached."""
    mesh, c, s = read_obstacle_mesh(path), math.cos(part.yaw), math.sin(part.yaw)
    turn = np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
    offset = np.array(part.at) - (0.0, 0.0, height / 2)
    return TriMesh.from_arrays(mesh.vertices @ turn.T + offset, mesh.faces)


# --- --- --- --- --- FILES --- --- --- --- ---

def shape_from_json(data: dict) -> Shape:
    """A shape as the files write it (yaw in degrees).

    Raises:
        ValueError: If a field is missing or wrong.
    """
    try:
        meshes = tuple(MeshPart(str(item["mesh"]), tuple(float(v) for v in item.get("at", (0.0, 0.0, 0.0))),
                                math.radians(float(item.get("yaw", 0.0)))) for item in data.get("meshes", ()))
        shape = Shape(type=str(data["type"]), name=str(data["name"]),
                      center=tuple(float(v) for v in data["center"]), size=tuple(float(v) for v in data["size"]),
                      yaw=math.radians(float(data.get("yaw", 0.0))), source=str(data.get("source", "")),
                      meshes=meshes)
    except (KeyError, TypeError, ValueError, AttributeError) as error:
        raise ValueError(f"bad obstacle {data!r}: {error}") from error
    if shape.type not in GROUPS or len(shape.center) != 3 or len(shape.size) != 3:
        raise ValueError(f"bad obstacle {data!r}: type must be one of {sorted(GROUPS)}, center and size 3 numbers")
    if any(len(part.at) != 3 or part.mesh != slug(part.mesh) for part in meshes):
        raise ValueError(f"bad obstacle {data!r}: a mesh needs a plain file name and `at` 3 numbers")
    return shape


def shape_to_json(shape: Shape) -> dict:
    """A shape as the files write it: millimetre and centidegree precision."""
    meshes = [{"mesh": part.mesh, **({"at": [round(v, 4) for v in part.at]} if any(part.at) else {}),
               **({"yaw": round(math.degrees(part.yaw), 2)} if part.yaw else {})} for part in shape.meshes]
    return {"type": shape.type, "name": shape.name, "center": [round(v, 4) for v in shape.center],
            "size": [round(v, 4) for v in shape.size], "yaw": round(math.degrees(shape.yaw), 2),
            **({"source": shape.source} if shape.source else {}), **({"meshes": meshes} if meshes else {})}


def read_lab(path: Path) -> Lab:
    """Read a lab file.

    Raises:
        OSError: If it cannot be read.
        ValueError: If it is not a lab file, an entry is wrong or a mesh file is missing.
    """
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict) or data.get("format") != FORMAT:
        raise ValueError(f"{path} is not a lab file (format {FORMAT!r})")
    border = data.get("border")
    try:
        border = None if border is None else Border(
            x=tuple(float(v) for v in border["x"]), y=tuple(float(v) for v in border["y"]),
            width=float(border.get("width", 0.2)), height=float(border.get("height", 2.0)),
            source=str(border.get("source", "")))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{path}: bad border: {error}") from error
    shapes = tuple(shape_from_json(item) for item in data.get("obstacles", []))
    names = [slug(shape.name) for shape in shapes]
    if len(set(names)) != len(names):
        raise ValueError(f"{path}: two obstacles share a name")
    missing = sorted({part.mesh for shape in shapes for part in shape.meshes
                      if not (mesh_directory_of(path) / f"{part.mesh}.obj").is_file()})
    if missing:
        raise ValueError(f"{path}: no mesh file for {missing} in {mesh_directory_of(path)}")
    return Lab(shapes, border)


def write_lab(lab: Lab, path: Path) -> None:
    """Write a lab file, one obstacle per line so a change is one line in git; replaced whole, never half written."""
    lines = [json.dumps(shape_to_json(shape)) for shape in lab.shapes]
    border = "null" if lab.border is None else json.dumps(
        {"x": list(lab.border.x), "y": list(lab.border.y), "width": lab.border.width, "height": lab.border.height,
         **({"source": lab.border.source} if lab.border.source else {})})
    text = (f'{{\n  "format": "{FORMAT}",\n  "version": {VERSION},\n  "units": {json.dumps(UNITS)},\n'
            f'  "obstacles": [\n    ' + ",\n    ".join(lines) + f'\n  ],\n  "border": {border}\n}}\n')
    replace_text(Path(path), text)


def read_temporary(path: Path | None = None, old_path: Path | None = None) -> dict[int, Shape]:
    """The temporary obstacles by number; from the debug gizmo's old file if `path` does not exist yet.

    Args:
        path: The file; None for TEMPORARY_FILE.
        old_path: The debug gizmo's file; None for OLD_TEMPORARY_FILE.

    Raises:
        OSError: If a file exists but cannot be read.
        ValueError: If it is not a temporary obstacles file.
    """
    path, old_path = Path(path or TEMPORARY_FILE), Path(old_path or OLD_TEMPORARY_FILE)
    if path.exists():
        return {int(item["number"]): shape_from_json(item) for item in json.loads(Path(path).read_text())["shapes"]}
    if not Path(old_path).exists():
        return {}
    shapes = {}
    for item in json.loads(Path(old_path).read_text())["shapes"]:
        # * The old file stored a box or cylinder by kind, centre and quaternion.
        number, kind = int(item["number"]), str(item["kind"])
        shape = Shape(kind, f"tmp {kind} {number}", tuple(float(v) for v in item["position"]),
                      tuple(float(v) for v in item["size"]), source="debug gizmo")
        shapes[number] = shape.moved(Pose.from_arrays(item["position"], item["orientation"]))
    return shapes


def write_temporary(shapes: dict[int, Shape], path: Path | None = None) -> None:
    """Write the temporary obstacles to `path` (None for TEMPORARY_FILE); replaced whole, never half written."""
    data = {"shapes": [{"number": number, **shape_to_json(shape)} for number, shape in sorted(shapes.items())]}
    replace_text(Path(path or TEMPORARY_FILE), json.dumps(data, indent=2) + "\n")


def replace_text(path: Path, text: str) -> None:
    """Write `text` beside `path`, then swap it in: a kill mid-write cannot leave half a file."""
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text)
    temporary.replace(path)


# --- --- --- --- --- SCENE BODIES --- --- --- --- ---

#: Colours, (r, g, b, a) from 0 to 1: muted grey furniture, the border as orange floor tape, temporary ones purple.
LAB_COLOR = (150 / 255, 150 / 255, 160 / 255, 1.0)
BORDER_COLOR = (230 / 255, 140 / 255, 40 / 255, 1.0)
TEMPORARY_COLOR = (0.6, 0.35, 0.9, 1.0)


def border_bodies(plugin: str, border: Border | None) -> list[Body]:
    """The border walls as scene bodies."""
    if border is None:
        return []
    return [Body(border_id(plugin, name), geometry, pose, label=name, color=BORDER_COLOR)
            for name, geometry, pose in border.walls()]


def floor_body(plugin: str, border: Border | None, touches: tuple[str, ...]) -> Body:
    """The floor: a slab just under z = 0, out to the border walls' outer edge; never drawn, only collides.

    Args:
        plugin: The owning plugin's name.
        border: The lab's border; None for a FLOOR_WITHOUT_BORDER square around the origin.
        touches: What may touch it: the robots' platform links (`platform_links`).
    """
    if border is None:
        (x0, x1), (y0, y1) = ((-FLOOR_WITHOUT_BORDER / 2, FLOOR_WITHOUT_BORDER / 2),) * 2
    else:
        (x0, x1), (y0, y1) = ((low - border.width, high + border.width) for low, high in (border.x, border.y))
    geometry = Geometry((), box_geometry((x1 - x0, y1 - y0, FLOOR_THICKNESS)).collision)
    return Body(floor_id(plugin), geometry, Pose(((x0 + x1) / 2, (y0 + y1) / 2, -FLOOR_THICKNESS / 2)),
                touches=touches, label="floor")


def platform_links(model: RobotModel) -> tuple[str, ...]:
    """The links of a robot no arm joint moves (wheels, chassis): they may touch the floor, driving never changes it."""
    movable = set(model.movable_joints)
    return tuple(sorted(link for link in model.links if not movable.intersection(chain_joints(model.urdf, link))))
