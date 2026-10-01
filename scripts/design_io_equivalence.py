"""Check that the old app, the export loader and design_io hand the planner the same compas_fab objects.

    python scripts/design_io_equivalence.py <export> [--json out.json] [--no-collisions]
                                            [--timing [--runs 3] [--no-compare]]

Loaders, each giving one `RobotCell` per acting robot and one `RobotCellState` per movement start:
  old     the pre-refactor monitor (husky_assembly_teleop/old): RobotCell.json plus its floor body, and
          Cindy's scheduled actions with the floor state added. It never loaded the Alice/Belle cells.
  new     `design_io.legacy.load_export`, as loaded.
  design  `convert_export` into a temporary folder, `read`, then `compas_fab.to_robot_cell` / `to_cell_state`.

Names are mapped to design ids before comparing (`bar_B1`, `env_bar_B1` -> `bars/B1`); the floors (old
`obstacle_ground`, design `ground/*`) are compared on their own. Each difference kind is listed with its count
and examples, and marked as explained (KNOWN) or not.
! Loads seven cells of ~350 MB: ~4 GB of memory and ~5 minutes, a third of it PyBullet collision checks.
"""

from __future__ import annotations

import contextlib
import io
import json
import subprocess
import sys
import time
from argparse import ArgumentParser
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median
from tempfile import TemporaryDirectory
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
import trimesh
from compas.data import json_load
from compas.geometry import Frame, Transformation
from compas_fab.backends import CollisionCheckError, PyBulletClient, PyBulletPlanner
from compas_fab.robots import RigidBodyState
from compas_robots.model import MeshDescriptor
from scipy.spatial import ConvexHull, cKDTree
from scipy.spatial.transform import Rotation

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))
DATA = REPO / "data"

import rs_data_structure  # noqa: E402,F401  (registers the action dtypes for json_load)
from husky_assembly_teleop.design_io import read  # noqa: E402
from husky_assembly_teleop.design_io.compas_fab import PARKED_POSITION, to_cell_state, to_robot_cell  # noqa: E402
from husky_assembly_teleop.design_io.conversion import convert_export  # noqa: E402
from husky_assembly_teleop.design_io.legacy import body_id, load_export  # noqa: E402
from husky_assembly_teleop.design_io.timing import Stopwatch  # noqa: E402

#: Tolerances: joints (rad or m), frame positions (m) and rotations (rad), mesh points (m), areas/volumes (relative).
JOINT_TOL, FRAME_TOL, MESH_TOL, RELATIVE_TOL = 1e-9, 1e-6, 1e-6, 1e-6
#: Canonical name of any floor body in collision pairs.
GROUND = "<ground>"
#: The old app's floor body (old/cfab_session.py:75).
OLD_GROUND = "obstacle_ground"
LOADERS = ("old", "new", "design")
PAIRS = (("old", "new"), ("new", "design"), ("old", "design"))

#: Difference kinds that are explained, with the reason and code reference. Any other kind is UNEXPECTED.
KNOWN = {
    "floor: only in old": "the export has no floor; the old app adds one (old/cfab_session.py:254)",
    "floor: only in design": "the export has no floor; design_io adds WalkableGround slabs (design_io/legacy.py:761)",
    "floor geometry: differs": "old drops each WalkableGround vertex's z, so every slab top is at z=0 "
                               "(old/cfab_session.py:149-152); design keeps the 3D polygon (design_io/legacy.py:781)",
    "floor state: touch differs": "old lets only the acting robot's four wheel links touch the floor "
                                  "(old/husky_monitor.py:202); design lets every robot's wheel links touch it "
                                  "(design_io/legacy.py:772), so the other robots are touch_bodies",
    "floor pairs: only in old": "pairs with the old app's floor; the export has none (old/cfab_session.py:254)",
    "floor pairs: only in design": "pairs with design_io's floor; the export has none (design_io/legacy.py:761)",
    "floor pairs: differ": "old flags the other robots standing on its floor (no touch allowed, see floor state), "
                           "and catches contacts between z=0 and the design floor's real top (z=-0.0156 m)",
    "tool state: group name differs, same flange": "Alice/Belle exports attach the gripper to the arm-only group "
                                                   "`manipulator`; design picks the base-rooted `base_arm_manipulator` "
                                                   "(design_io/compas_fab.py:193, format App. A). compas_fab attaches "
                                                   "to the same last link",
    "tool state: attached tool has a frame": "the Alice/Belle exports set `frame` on the attached gripper; compas_fab "
                                             "ignores it for attached tools (pybullet_set_robot_cell_state.py:88)",
    "tool state: attachment_frame None vs identity": "compas_fab reads None as identity "
                                                     "(pybullet_set_robot_cell_state.py:235)",
    "body state: touch_bodies mirrored": "design lists a body-body touch on both bodies (design_io/compas_fab.py:235); "
                                         "the export on one. compas_fab CC.4 reads both lists, so it checks the same",
    "other robot: welded tool link named differently": "export `<flange>_obstacle_tool`, design_io `<flange>_tool` "
                                                       "(design_io/compas_fab.py:163); the links are matched by name "
                                                       "and compared, and collisions report the whole robot",
    "geometry: visual differs": "the export holds compas copies of the visual meshes; design reads the URDF's files",
}


# --- --- --- --- --- REPORT --- --- --- --- ---

@dataclass
class Category:
    """Results of one comparison category for one pair of loaders."""

    name: str
    compared: int = 0
    equal: int = 0
    deviations: Dict[str, float] = field(default_factory=dict)
    kinds: Dict[str, List[str]] = field(default_factory=lambda: defaultdict(list))
    counts: Dict[str, int] = field(default_factory=lambda: defaultdict(int))

    def add(self, item: str, problems: List[Tuple[str, str]]) -> None:
        """Count one compared item; `problems` are (kind, detail) pairs, empty when equal."""
        self.compared += 1
        if not problems:
            self.equal += 1
        for kind, detail in problems:
            self.counts[kind] += 1
            if len(self.kinds[kind]) < 500:
                self.kinds[kind].append(f"{item}: {detail}")

    def deviation(self, measure: str, value: float) -> None:
        """Track the largest deviation of one measure."""
        self.deviations[measure] = max(self.deviations.get(measure, 0.0), float(value))

    def as_dict(self) -> dict:
        """Plain JSON value."""
        return {"compared": self.compared, "equal": self.equal, "max_deviation": self.deviations,
                "differences": {kind: {"count": self.counts[kind], "known": KNOWN.get(kind),
                                       "examples": self.kinds[kind]} for kind in self.counts}}


class Report:
    """Categories per loader pair."""

    def __init__(self):
        self.pairs: Dict[str, Dict[str, Category]] = defaultdict(dict)
        self.notes: List[str] = []

    def category(self, pair: Tuple[str, str], name: str) -> Category:
        """The category `name` of a loader pair, created on first use."""
        key = f"{pair[0]} vs {pair[1]}"
        if name not in self.pairs[key]:
            self.pairs[key][name] = Category(name)
        return self.pairs[key][name]

    def unexpected(self) -> List[str]:
        """Every difference kind not in KNOWN, as "pair / category / kind"."""
        return [f"{pair} / {name} / {kind}" for pair, categories in self.pairs.items()
                for name, category in categories.items() for kind in category.counts if kind not in KNOWN]

    def as_dict(self) -> dict:
        """Plain JSON value."""
        return {"notes": self.notes,
                "pairs": {pair: {name: c.as_dict() for name, c in cats.items()} for pair, cats in self.pairs.items()},
                "unexpected": self.unexpected()}

    def print(self) -> None:
        """Print a compact table per pair, then every difference kind."""
        for note in self.notes:
            print(f"* {note}")
        for pair, categories in self.pairs.items():
            print(f"\n=== {pair}")
            print(f"  {'category':28}{'equal/compared':>16}   max deviation")
            for name, c in categories.items():
                devs = ", ".join(f"{k} {v:.2e}" for k, v in sorted(c.deviations.items())) or "-"
                print(f"  {name:28}{c.equal:>8}/{c.compared:<7}   {devs}")
            for name, c in categories.items():
                for kind, count in c.counts.items():
                    mark = "known" if kind in KNOWN else "UNEXPECTED"
                    print(f"    [{mark}] {name} / {kind}: {count}")
                    for example in c.kinds[kind][:3]:
                        print(f"        {example}")
        unexpected = self.unexpected()
        print(f"\n{len(unexpected)} unexpected difference kind(s)" + (":" if unexpected else ""))
        for line in unexpected:
            print(f"  {line}")


# --- --- --- --- --- LOADERS --- --- --- --- ---

@dataclass
class Loaded:
    """One loader's planning objects.

    Attributes:
        cells: Robot id -> cell.
        states: (action id, movement id) -> (acting robot id, start state).
        keys: Robot id -> cell key -> design id (GROUND for a floor).
    """

    label: str
    cells: Dict[str, object]
    states: Dict[Tuple[str, str], Tuple[str, object]]
    keys: Dict[str, Dict[str, str]] = field(default_factory=dict)


def quiet(_text: str) -> None:
    """Swallow progress messages."""


def _schedule(export: Path) -> dict:
    return json.loads((export / "ActionSchedule.json").read_text())


def _import_old():
    """The old app's cfab_session module, after giving it the package constants it imports (old/__init__.py:50)."""
    import husky_assembly_teleop
    husky_assembly_teleop.DATA_DIRECTORY = str(DATA)
    husky_assembly_teleop.DESIGN_DATA_DIRECTORY = ""
    with contextlib.redirect_stdout(io.StringIO()):
        from husky_assembly_teleop.old import cfab_session
    return cfab_session


#: Copied from old/husky_monitor.py:202 (importing the monitor needs ROS nodes and hardware clients).
GROUND_TOUCH_LINKS = (
    'front_left_wheel_link', 'front_right_wheel_link',
    'rear_left_wheel_link', 'rear_right_wheel_link',
)


def _inject_ground_rigid_body_state(cell, state):
    """old/husky_monitor.py:2552 `_inject_ground_rigid_body_state`, verbatim but `self.cfab.robot_cell` -> `cell`."""
    GROUND_RIGID_BODY_NAME = OLD_GROUND
    if state is None:
        return
    if cell is None or GROUND_RIGID_BODY_NAME not in (cell.rigid_body_models or {}):
        return
    rb_states = getattr(state, 'rigid_body_states', None)
    if rb_states is None or GROUND_RIGID_BODY_NAME in rb_states:
        return
    rb_states[GROUND_RIGID_BODY_NAME] = RigidBodyState(
        frame=Frame.worldXY(),
        touch_links=list(GROUND_TOUCH_LINKS),
    )


def _lap(watch: Optional[Stopwatch], name: str) -> None:
    """Record a lap, if timing."""
    if watch is not None:
        watch.lap(name)


def load_old(export: Path, watch: Optional[Stopwatch] = None) -> Loaded:
    """The old monitor's planning objects: `CfabSession.__init__` without PyBullet, then `load_bar_action_file`.

    * Reproduces old/cfab_session.py:240-255 (RobotCell.json + `build_ground_rigid_body`) and, per scheduled action
      of that cell's robot, old/bar_action_io.py:36 `parse_bar_action` + old/husky_monitor.py:2520 (floor state).
    ? `parse_bar_action`'s isinstance check is left out: `BarAssemblyAction` no longer exists in rs_data_structure.
    ? Not reproduced: the live arm joints written into M0 states (husky_monitor.py:2518), which need the robot.
    """
    cfab_session = _import_old()
    cfab_session.DESIGN_DATA_DIRECTORY = str(export.parent)
    cell = json_load(str(export / "RobotCell.json"))
    with contextlib.redirect_stdout(io.StringIO()):
        cell.rigid_body_models[OLD_GROUND] = cfab_session.build_ground_rigid_body(export.name)
    _lap(watch, "cell (RobotCell.json + floor)")
    schedule = _schedule(export)
    robot = next(f"robots/{name.lower()}" for name, entry in schedule["robots"].items()
                 if entry["robot_id"] == cell.robot_model.name)
    states = {}
    for entry in schedule["schedule"]:
        if f"robots/{entry['robot'].lower()}" != robot:
            continue
        action = json_load(str(export / entry["file"]))
        for movement in action.movements:
            _inject_ground_rigid_body_state(cell, movement.start_state)
            states[(entry["action_id"], movement.movement_id)] = (robot, movement.start_state)
    _lap(watch, "states (action files)")
    return Loaded("old", {robot: cell}, states)


def load_new(export: Path, watch: Optional[Stopwatch] = None) -> Loaded:
    """`design_io.legacy.load_export`: every cell and every scheduled movement's start state, as loaded."""
    loaded = load_export(export, quiet, watch)
    robots = {entry["robot_id"]: f"robots/{name.lower()}" for name, entry in loaded.schedule["robots"].items()}
    cells = {robots[name]: cell for name, cell in loaded.cells.items()}
    states = {}
    for entry, action in loaded.actions:
        for movement in action.movements:
            robot = f"robots/{entry['robot'].lower()}"
            states[(entry["action_id"], movement.movement_id)] = (robot, movement.start_state)
    return Loaded("new", cells, states)


def load_design(design_folder: Path, robots: Optional[Tuple[str, ...]] = None, watch: Optional[Stopwatch] = None):
    """`read` a converted design, then one cell per robot and one state per movement (cell keys are design ids).

    Args:
        design_folder: The converted design.
        robots: Only these acting robots (e.g. Cindy's, to match the old app); all by default.
        watch: Gets a lap for reading, the cells and the states, if given.

    Returns:
        tuple[Loaded, Design]
    """
    design = read(design_folder)
    _lap(watch, "read")
    cells = {robot: to_robot_cell(design, robot) for robot in design.robots if robots is None or robot in robots}
    _lap(watch, "cells (URDF, SRDF, meshes)")
    states = {(action.id, movement.id): (action.robot, to_cell_state(design, action.robot, movement.start,
                                                                     cells[action.robot]))
              for action, movement in design.movements() if action.robot in cells}
    _lap(watch, "states (to_cell_state)")
    return Loaded("design", cells, states), design


def convert(export: Path, destination: Path) -> Path:
    """`design_io.conversion.convert_export` with the repository's robot files."""
    return convert_export(export, destination, DATA, report=quiet)


def name_maps(loaded: Loaded, design) -> None:
    """Fill `loaded.keys`: each cell key as a design id."""
    for robot, cell in loaded.cells.items():
        tools = {tool_id.split("/")[-1]: tool_id for tool_id in design.robots[robot].tools.values()}
        keys = {}
        for key in [*cell.tool_models, *cell.rigid_body_models]:
            if loaded.label == "design" or key.startswith("ground/"):
                keys[key] = GROUND if key.startswith("ground/") else key
            elif key == OLD_GROUND:
                keys[key] = GROUND
            elif key.startswith("ObstacleRobot"):
                keys[key] = f"robots/{key[len('ObstacleRobot'):].lower()}"
            elif key in tools:
                keys[key] = tools[key]
            else:
                keys[key] = body_id(key)
        loaded.keys[robot] = keys


# --- --- --- --- --- MEASURES --- --- --- --- ---

def frame_deviation(a, b) -> Tuple[float, float]:
    """(position, rotation angle) between two compas frames."""
    rotation_a = np.array([list(a.xaxis), list(a.yaxis), list(a.zaxis)]).T
    rotation_b = np.array([list(b.xaxis), list(b.yaxis), list(b.zaxis)]).T
    angle = Rotation.from_matrix(rotation_a.T @ rotation_b).magnitude()
    return float(np.abs(np.subtract(list(a.point), list(b.point))).max()), float(angle)


def compare_frames(category: Category, a, b, what: str) -> List[Tuple[str, str]]:
    """Problems between two optional frames, tracking `<what> m` and `<what> rad` deviations."""
    if a is None and b is None:
        return []
    if (a is None) != (b is None):
        return [(f"{what} None in one", f"{a} vs {b}")]
    position, angle = frame_deviation(a, b)
    category.deviation(f"{what} m", position)
    category.deviation(f"{what} rad", angle)
    if position > FRAME_TOL or angle > FRAME_TOL:
        return [(f"{what} differs", f"{position:.3g} m, {angle:.3g} rad")]
    return []


def _matrix(frame) -> np.ndarray:
    return np.eye(4) if frame is None else np.asarray(frame.to_transformation().matrix, dtype=float)


@dataclass
class Part:
    """One collision or visual element: a triangle mesh (in the owner's frame) or a primitive."""

    kind: str
    vertices: Optional[np.ndarray] = None
    faces: Optional[np.ndarray] = None
    parameters: Tuple[float, ...] = ()
    matrix: Optional[np.ndarray] = None


def _mesh_part(mesh, matrix: np.ndarray, scale=(1.0, 1.0, 1.0)) -> Part:
    vertices, faces = mesh.to_vertices_and_faces(triangulated=True)
    points = np.asarray(vertices, dtype=float).reshape(-1, 3) * np.asarray(scale, dtype=float)
    points = points @ matrix[:3, :3].T + matrix[:3, 3]
    return Part("mesh", points, np.asarray(faces, dtype=np.int64).reshape(-1, 3))


#: id(link or body), which -> its parts; the objects stay alive in the loaders, so ids are not reused.
_PARTS: Dict[Tuple[int, str], List[Part]] = {}


def link_parts(link, which: str) -> List[Part]:
    """A link's `visual` or `collision` elements in the link frame (cached)."""
    if (id(link), which) not in _PARTS:
        _PARTS[(id(link), which)] = _link_parts(link, which)
    return _PARTS[(id(link), which)]


def _link_parts(link, which: str) -> List[Part]:
    parts = []
    for item in getattr(link, which):
        shape = item.geometry.shape
        origin = _matrix(item.origin)
        if isinstance(shape, MeshDescriptor):
            parts.extend(_mesh_part(mesh, origin, shape.scale) for mesh in shape.meshes or [])
            continue
        name = type(shape).__name__.replace("Proxy", "").lower()
        values = tuple(float(getattr(shape, attr)) for attr in ("xsize", "ysize", "zsize", "radius", "height")
                       if hasattr(shape, attr))
        own = getattr(shape, "frame", None)
        parts.append(Part(name, parameters=values, matrix=origin @ _matrix(own)))
    return parts


def body_parts(rigid_body, which: str) -> List[Part]:
    """A rigid body's `visual` or `collision` meshes in metres, in the body frame (cached)."""
    if (id(rigid_body), which) in _PARTS:
        return _PARTS[(id(rigid_body), which)]
    meshes = rigid_body.collision_meshes if which == "collision" else rigid_body.visual_meshes
    scale = rigid_body.native_scale or 1.0
    _PARTS[(id(rigid_body), which)] = [_mesh_part(mesh, np.eye(4), (scale,) * 3) for mesh in meshes or []]
    return _PARTS[(id(rigid_body), which)]


def _trimesh(part: Part):
    return trimesh.Trimesh(part.vertices, part.faces, process=False)


def _hull_volume(part: Part) -> float:
    """Convex hull volume: what PyBullet checks for each mesh (compas_fab loads meshes with concavity off)."""
    if len(part.vertices) < 4:
        return 0.0
    try:
        return float(ConvexHull(part.vertices).volume)
    except Exception:  # flat or degenerate
        return 0.0


def _point_distance(a: np.ndarray, b: np.ndarray) -> float:
    """Symmetric largest nearest-vertex distance between two point sets."""
    if len(a) == 0 or len(b) == 0:
        return 0.0 if len(a) == len(b) else float("inf")
    return float(max(cKDTree(b).query(a)[0].max(), cKDTree(a).query(b)[0].max()))


def _surface_distance(a: Part, b: Part, samples: int = 2000) -> float:
    """Largest distance from (up to `samples`) vertices of each mesh to the other's surface."""
    worst = 0.0
    for source, target in ((a, b), (b, a)):
        points = source.vertices
        if len(points) > samples:
            points = points[np.random.default_rng(0).choice(len(points), samples, replace=False)]
        _, distance, _ = trimesh.proximity.closest_point(_trimesh(target), points)
        worst = max(worst, float(distance.max()))
    return worst


def compare_parts(category: Category, a: List[Part], b: List[Part], label: str) -> List[Tuple[str, str]]:
    """Problems between two element lists: counts and kinds, primitives by value, meshes up to triangulation.

    Meshes are equal when their arrays are identical, or when area, volume, convex hull volume and bounding box
    agree (relative RELATIVE_TOL, MESH_TOL) and every vertex lies within MESH_TOL of the other mesh.
    """
    problems = []
    kind = f"geometry: {label} differs"
    if [p.kind for p in a] != [p.kind for p in b]:
        return [(kind, f"elements {[p.kind for p in a]} vs {[p.kind for p in b]}")]
    for index, (x, y) in enumerate(zip(a, b)):
        if x.kind != "mesh":
            deviation = max(np.abs(np.subtract(x.parameters, y.parameters)).max(initial=0.0),
                            np.abs(x.matrix - y.matrix).max())
            category.deviation(f"{label} primitive", deviation)
            if len(x.parameters) != len(y.parameters) or deviation > MESH_TOL:
                problems.append((kind, f"#{index} {x.kind} {x.parameters} vs {y.parameters}"))
            continue
        if x.vertices.shape == y.vertices.shape and x.faces.shape == y.faces.shape \
                and np.array_equal(x.vertices, y.vertices) and np.array_equal(x.faces, y.faces):
            category.deviation(f"{label} identical meshes", 0.0)
            continue
        mx, my = _trimesh(x), _trimesh(y)
        size = max(float(np.ptp(np.vstack([x.vertices, y.vertices]), axis=0).max()), 1e-9)
        relative = {
            "area": abs(mx.area - my.area) / max(mx.area, my.area, 1e-12),
            "volume": abs(mx.volume - my.volume) / max(abs(mx.volume), abs(my.volume), 1e-12),
            "hull": abs(_hull_volume(x) - _hull_volume(y)) / max(_hull_volume(x), _hull_volume(y), 1e-12),
        }
        bbox = float(np.abs(mx.bounds - my.bounds).max())
        points = _point_distance(x.vertices, y.vertices)
        surface = points if points <= MESH_TOL else _surface_distance(x, y)
        for name, value in relative.items():
            category.deviation(f"{label} {name} rel", value)
        category.deviation(f"{label} bbox m", bbox)
        category.deviation(f"{label} surface m", surface)
        same = all(v <= RELATIVE_TOL for v in relative.values()) and bbox <= MESH_TOL and surface <= MESH_TOL
        if not same:
            problems.append((kind, f"#{index} {len(x.faces)} vs {len(y.faces)} triangles, area rel "
                                   f"{relative['area']:.2g}, hull rel {relative['hull']:.2g}, bbox {bbox:.2g} m, "
                                   f"surface {surface:.2g} m (size {size:.2g} m)"))
    return problems


# --- --- --- --- --- CELL COMPARISONS --- --- --- --- ---

#: The export names another robot's welded tool link `<flange>_obstacle_tool`; design_io `<flange>_tool`.
WELDED_EXPORT, WELDED_DESIGN = "_obstacle_tool", "_tool"


def _same_name(name: str) -> str:
    """A link or joint name with the export's welded-tool suffix written as design_io's."""
    return name.replace(WELDED_EXPORT, WELDED_DESIGN)


def compare_model(report: Report, pair, a, b, owner: str, category_prefix: str) -> None:
    """Links, joints and per-link geometry of two robot or tool models (welded tool links matched by `_same_name`)."""
    kinematics = report.category(pair, f"{category_prefix} kinematics")
    links_a = {_same_name(link.name): link for link in a.links}
    links_b = {_same_name(link.name): link for link in b.links}
    problems = []
    renamed = sorted(link.name for link in [*a.links, *b.links] if WELDED_EXPORT in link.name)
    if renamed and {link.name for link in a.links} != {link.name for link in b.links}:
        problems.append(("other robot: welded tool link named differently", f"{renamed} vs {WELDED_DESIGN}"))
    if a.root.name != b.root.name:
        problems.append(("root differs", f"{a.root.name} vs {b.root.name}"))
    if set(links_a) != set(links_b):
        problems.append(("link names differ", f"only first {sorted(set(links_a) - set(links_b))}, "
                                              f"only second {sorted(set(links_b) - set(links_a))}"))
    kinematics.add(f"{owner} links", problems)
    joints_a = {_same_name(j.name): j for j in a.joints}
    joints_b = {_same_name(j.name): j for j in b.joints}
    for name in sorted(set(joints_a) | set(joints_b)):
        if name not in joints_a or name not in joints_b:
            kinematics.add(f"{owner}/{name}", [("joint only in one", "first" if name in joints_a else "second")])
            continue
        x, y = joints_a[name], joints_b[name]
        problems = []
        if (x.type, _same_name(x.parent.link), _same_name(x.child.link)) != \
                (y.type, _same_name(y.parent.link), _same_name(y.child.link)):
            problems.append(("joint type or links differ", f"{(x.type, x.parent.link, x.child.link)} vs "
                                                           f"{(y.type, y.parent.link, y.child.link)}"))
        problems += [(f"joint {kind}", detail) for kind, detail in compare_frames(kinematics, x.origin, y.origin,
                                                                                  "origin")]
        axis_a = list(x.axis.vector) if x.axis else [0, 0, 0]
        axis_b = list(y.axis.vector) if y.axis else [0, 0, 0]
        axis = float(np.abs(np.subtract(axis_a, axis_b)).max())
        kinematics.deviation("axis", axis)
        if axis > FRAME_TOL:
            problems.append(("joint axis differs", f"{axis_a} vs {axis_b}"))
        limit_a = [getattr(x.limit, k, None) for k in ("lower", "upper", "effort", "velocity")] if x.limit else []
        limit_b = [getattr(y.limit, k, None) for k in ("lower", "upper", "effort", "velocity")] if y.limit else []
        if len(limit_a) != len(limit_b) or any(
                (p is None) != (q is None) or (p is not None and abs(float(p) - float(q)) > JOINT_TOL)
                for p, q in zip(limit_a, limit_b)):
            problems.append(("joint limits differ", f"{limit_a} vs {limit_b}"))
        if x.limit and y.limit:
            kinematics.deviation("limits", max(abs(float(p) - float(q)) for p, q in zip(limit_a, limit_b)
                                               if p is not None and q is not None))
        kinematics.add(f"{owner}/{name}", problems)
    for which in ("collision", "visual"):
        geometry = report.category(pair, f"{category_prefix} {which}")
        for name in sorted(set(links_a) & set(links_b)):
            geometry.add(f"{owner}/{name}", compare_parts(geometry, link_parts(links_a[name], which),
                                                          link_parts(links_b[name], which), which))


def compare_semantics(report: Report, pair, a, b, robot: str) -> None:
    """SRDF groups, passive joints, end effectors, disabled collisions and group states."""
    category = report.category(pair, "robot semantics")
    sa, sb = a.robot_semantics, b.robot_semantics
    for field_name in ("main_group_name", "passive_joints", "end_effectors", "group_states"):
        x, y = getattr(sa, field_name), getattr(sb, field_name)
        category.add(f"{robot} {field_name}", [] if x == y else [(f"{field_name} differs", f"{x} vs {y}")])
    for group in sorted(set(sa.groups) | set(sb.groups)):
        x, y = sa.groups.get(group), sb.groups.get(group)
        category.add(f"{robot} group {group}", [] if x == y else [("group differs", f"{x} vs {y}")])
    da = {tuple(sorted(p)) for p in sa.disabled_collisions}
    db = {tuple(sorted(p)) for p in sb.disabled_collisions}
    category.add(f"{robot} disabled_collisions ({len(da)})",
                 [] if da == db else [("disabled collisions differ", f"only first {sorted(da - db)[:5]}, "
                                                                     f"only second {sorted(db - da)[:5]}")])


def compare_cells(report: Report, pair, a: Loaded, b: Loaded, robot: str) -> None:
    """Robot model, semantics, tools (own and other robots) and bodies of one robot's cells."""
    cell_a, cell_b = a.cells[robot], b.cells[robot]
    compare_model(report, pair, cell_a.robot_model, cell_b.robot_model, robot, "robot")
    compare_semantics(report, pair, cell_a, cell_b, robot)

    tools_a = {a.keys[robot][k]: m for k, m in cell_a.tool_models.items()}
    tools_b = {b.keys[robot][k]: m for k, m in cell_b.tool_models.items()}
    keys = report.category(pair, "cell keys")
    bodies_a = {a.keys[robot][k]: m for k, m in cell_a.rigid_body_models.items() if a.keys[robot][k] != GROUND}
    bodies_b = {b.keys[robot][k]: m for k, m in cell_b.rigid_body_models.items() if b.keys[robot][k] != GROUND}
    for what, x, y in (("tools", tools_a, tools_b), ("bodies", bodies_a, bodies_b)):
        keys.add(f"{robot} {what}", [] if set(x) == set(y) else [(f"{what} differ", f"only first "
                                                                  f"{sorted(set(x) - set(y))}, only second "
                                                                  f"{sorted(set(y) - set(x))}")])
    for tool in sorted(set(tools_a) & set(tools_b)):
        prefix = "other robot" if tool.startswith("robots/") else "tool"
        compare_model(report, pair, tools_a[tool], tools_b[tool], f"{robot}:{tool}", prefix)
        frame = report.category(pair, f"{prefix} kinematics")
        frame.add(f"{robot}:{tool} tool frame", compare_frames(frame, tools_a[tool].frame, tools_b[tool].frame,
                                                               "tool frame"))
    for which in ("collision", "visual"):
        category = report.category(pair, f"body {which}")
        for body in sorted(set(bodies_a) & set(bodies_b)):
            category.add(f"{robot}:{body}", compare_parts(category, body_parts(bodies_a[body], which),
                                                          body_parts(bodies_b[body], which), which))
    compare_floor(report, pair, a, b, robot)


def floor_parts(loaded: Loaded, robot: str) -> List[Part]:
    """Every floor mesh of a cell, in world."""
    cell = loaded.cells[robot]
    return [part for key, body in cell.rigid_body_models.items() if loaded.keys[robot][key] == GROUND
            for part in body_parts(body, "collision")]


def floor_summary(parts: List[Part]) -> dict:
    """Slab count, total area, volume and bounding box of floor meshes."""
    if not parts:
        return {"slabs": 0}
    meshes = [_trimesh(p) for p in parts]
    points = np.vstack([p.vertices for p in parts])
    return {"slabs": len(parts), "area m2": round(sum(m.area for m in meshes), 6),
            "volume m3": round(sum(abs(m.volume) for m in meshes), 6),
            "bbox": np.round([points.min(axis=0), points.max(axis=0)], 4).tolist()}


def compare_floor(report: Report, pair, a: Loaded, b: Loaded, robot: str) -> None:
    """The floors, by geometry (they are separate bodies with other names in each loader)."""
    category = report.category(pair, "floor geometry")
    fa, fb = floor_parts(a, robot), floor_parts(b, robot)
    if not fa and not fb:
        category.add(robot, [])
        return
    if not fa or not fb:
        category.add(robot, [(f"floor: only in {a.label if fa else b.label}", str(floor_summary(fa or fb)))])
        return
    sa, sb = floor_summary(fa), floor_summary(fb)
    category.add(robot, [] if sa == sb else [("floor geometry: differs", f"{sa} vs {sb}")])


# --- --- --- --- --- STATE COMPARISONS --- --- --- --- ---

def _joint_map(configuration) -> Dict[str, float]:
    return {} if configuration is None else dict(zip(configuration.joint_names, map(float, configuration.joint_values)))


def compare_joints(category: Category, a, b, extra_kind: str) -> List[Tuple[str, str]]:
    """Problems between two configurations: None in one, joints only in one, values beyond JOINT_TOL."""
    if a is None and b is None:
        return []
    if (a is None) != (b is None):
        return [("configuration None in one", "first None" if a is None else "second None")]
    x, y = _joint_map(a), _joint_map(b)
    problems = []
    if set(x) != set(y):
        problems.append((extra_kind, f"only first {sorted(set(x) - set(y))}, only second {sorted(set(y) - set(x))}"))
    common = set(x) & set(y)
    worst = max((abs(x[n] - y[n]) for n in common), default=0.0)
    category.deviation("joint", worst)
    if worst > JOINT_TOL:
        problems.append(("joint values differ", f"{worst:.3g}"))
    return problems


def _flange(cell, group: Optional[str]) -> Optional[str]:
    """The link compas_fab attaches a tool of `group` to (pybullet_set_robot_cell_state.py:93)."""
    return cell.get_link_names(group)[-1] if group else None


def compare_states(report: Report, pair, a: Loaded, b: Loaded, key) -> None:
    """One movement's start state: base, configuration, tool states, body states, floor state."""
    robot, sa = a.states[key]
    _, sb = b.states[key]
    cell_a, cell_b = a.cells[robot], b.cells[robot]
    item = f"{key[1]}"

    base = report.category(pair, "state base+joints")
    problems = compare_frames(base, sa.robot_base_frame, sb.robot_base_frame, "base")
    problems += compare_joints(base, sa.robot_configuration, sb.robot_configuration, "configuration joints differ")
    base.add(item, problems)

    tools = report.category(pair, "state tools")
    ta = {a.keys[robot][k]: s for k, s in sa.tool_states.items()}
    tb = {b.keys[robot][k]: s for k, s in sb.tool_states.items()}
    if set(ta) != set(tb):
        tools.add(f"{item} keys", [("tool keys differ", f"{sorted(set(ta) ^ set(tb))}")])
    for tool in sorted(set(ta) & set(tb)):
        x, y = ta[tool], tb[tool]
        problems = []
        if x.is_hidden != y.is_hidden:
            problems.append(("tool state: hidden differs", f"{x.is_hidden} vs {y.is_hidden}"))
        if tool.startswith("robots/"):
            parked = [s.frame is not None and np.allclose(list(s.frame.point), PARKED_POSITION, atol=1e-3)
                      for s in (x, y)]
            if all(parked):
                tools.deviation("parked", 0.0)
            else:
                problems += [(f"other robot: {k}", d) for k, d in compare_frames(tools, x.frame, y.frame, "base")]
            problems += [(f"other robot: {k}" if not k.startswith("configuration joints") else
                          "other robot: configuration lists more joints", d)
                         for k, d in compare_joints(tools, x.configuration, y.configuration,
                                                    "configuration joints differ")]
            if x.attached_to_group or y.attached_to_group or set(x.touch_links) != set(y.touch_links):
                problems.append(("other robot: attachment or touches differ",
                                 f"{x.attached_to_group}/{x.touch_links} vs {y.attached_to_group}/{y.touch_links}"))
        else:
            if x.attached_to_group != y.attached_to_group:
                same = _flange(cell_a, x.attached_to_group) == _flange(cell_b, y.attached_to_group)
                kind = "tool state: group name differs, same flange" if same else "tool state: attached elsewhere"
                problems.append((kind, f"{x.attached_to_group} vs {y.attached_to_group}"))
            if x.attached_to_group and y.attached_to_group:
                if (x.frame is None) != (y.frame is None):
                    problems.append(("tool state: attached tool has a frame", f"{x.frame} vs {y.frame}"))
                fa, fb = x.attachment_frame, y.attachment_frame
                if (fa is None) != (fb is None):
                    if compare_frames(tools, fa or Frame.worldXY(), fb or Frame.worldXY(), "attachment"):
                        problems.append(("tool state: attachment differs", f"{fa} vs {fb}"))
                    else:
                        problems.append(("tool state: attachment_frame None vs identity", f"{fa} vs {fb}"))
                else:
                    problems += [(f"tool state: {k}", d) for k, d in compare_frames(tools, fa, fb, "attachment")]
            else:
                problems += [(f"tool state: {k}", d) for k, d in compare_frames(tools, x.frame, y.frame, "frame")]
            la, lb = set(x.touch_links), set(y.touch_links)
            if la != lb:
                kind = ("tool state: touch_links superset" if lb > la and b.label == "design" else
                        "tool state: touch_links differ")
                problems.append((kind, f"only first {sorted(la - lb)}, only second {sorted(lb - la)}"))
        tools.add(f"{item} {tool}", problems)

    bodies = report.category(pair, "state bodies")
    ra = {a.keys[robot][k]: s for k, s in sa.rigid_body_states.items() if a.keys[robot][k] != GROUND}
    rb = {b.keys[robot][k]: s for k, s in sb.rigid_body_states.items() if b.keys[robot][k] != GROUND}
    if set(ra) != set(rb):
        bodies.add(f"{item} keys", [("body keys differ", f"{sorted(set(ra) ^ set(rb))[:6]}")])
    mirrored_a, mirrored_b = _mirrored(a, robot, ra), _mirrored(b, robot, rb)
    for body in sorted(set(ra) & set(rb)):
        mirrored = (mirrored_a.get(body, set()), mirrored_b.get(body, set()))
        bodies.add(f"{item} {body}", compare_body_state(bodies, a, b, robot, ra[body], rb[body], mirrored))

    floor = report.category(pair, "floor state")
    ga = [s for k, s in sa.rigid_body_states.items() if a.keys[robot][k] == GROUND]
    gb = [s for k, s in sb.rigid_body_states.items() if b.keys[robot][k] == GROUND]
    if ga or gb:
        floor.add(item, compare_floor_states(a, b, robot, ga, gb))


def _touch_bodies(loaded: Loaded, robot: str, state) -> set:
    return {loaded.keys[robot].get(name, name) for name in state.touch_bodies}


def _mirrored(loaded: Loaded, robot: str, states: dict) -> Dict[str, set]:
    """Body id -> the bodies that list it in their own `touch_bodies` (compas_fab CC.4 checks both lists)."""
    result: Dict[str, set] = defaultdict(set)
    for body, state in states.items():
        for other in _touch_bodies(loaded, robot, state):
            result[other].add(body)
    return result


def compare_body_state(category: Category, a: Loaded, b: Loaded, robot: str, x, y,
                       mirrored: Tuple[set, set] = (set(), set())) -> List[Tuple[str, str]]:
    """Problems between two states of one body.

    Args:
        mirrored: Per loader, the bodies listing this one in their `touch_bodies`; a `touch_bodies` difference
            that this closes is reported as "mirrored".
    """
    if x.is_hidden != y.is_hidden:
        return [("body state: hidden differs", f"{x.is_hidden} vs {y.is_hidden}")]
    hidden = x.is_hidden
    problems = []
    if (x.attached_to_link, x.attached_to_tool) != (y.attached_to_link, y.attached_to_tool):
        problems.append(("body state: attachment differs" if not hidden else "body state: hidden body frame differs",
                         f"{(x.attached_to_link, x.attached_to_tool)} vs {(y.attached_to_link, y.attached_to_tool)}"))
    elif x.attached_to_link or x.attached_to_tool:
        problems += [(f"body state: {k}", d) for k, d in compare_frames(category, x.attachment_frame,
                                                                        y.attachment_frame, "grasp")]
        if (x.frame is None) != (y.frame is None):
            problems.append(("body state: attached body has a frame", f"{x.frame} vs {y.frame}"))
    else:
        found = compare_frames(category, x.frame, y.frame, "hidden frame" if hidden else "frame")
        problems += [("body state: hidden body frame differs" if hidden else f"body state: {k}", d) for k, d in found]
    for what, la, lb in (("touch_links", set(x.touch_links), set(y.touch_links)),
                         ("touch_bodies", _touch_bodies(a, robot, x), _touch_bodies(b, robot, y))):
        if la == lb:
            continue
        if hidden:
            kind = "body state: hidden body touches differ"
        elif what == "touch_bodies" and la | mirrored[0] == lb | mirrored[1]:
            kind = "body state: touch_bodies mirrored"
        elif lb > la and b.label == "design":
            kind = f"body state: {what} superset"
        else:
            kind = f"body state: {what} differ"
        problems.append((kind, f"only first {sorted(la - lb)}, only second {sorted(lb - la)}"))
    return problems


def compare_floor_states(a: Loaded, b: Loaded, robot: str, ga: list, gb: list) -> List[Tuple[str, str]]:
    """The floor's allowed touches, merged over every floor body (all stationary at identity, never hidden)."""
    if not ga or not gb:
        return [(f"floor: only in {a.label if ga else b.label}", "")]
    problems = []
    for states in (ga, gb):
        for state in states:
            if state.is_hidden or state.attached_to_link or state.frame is None or \
                    max(frame_deviation(state.frame, Frame.worldXY())) > FRAME_TOL:
                problems.append(("floor state: not stationary at identity", str(state)))
    la = set().union(*(s.touch_links for s in ga))
    lb = set().union(*(s.touch_links for s in gb))
    ba = set().union(*(_touch_bodies(a, robot, s) for s in ga))
    bb = set().union(*(_touch_bodies(b, robot, s) for s in gb))
    if (la, ba) != (lb, bb):
        problems.append(("floor state: touch differs", f"links {sorted(la)} bodies {sorted(ba)} vs links "
                                                       f"{sorted(lb)} bodies {sorted(bb)}"))
    return problems


def flange_frames(cell, state) -> Dict[str, np.ndarray]:
    """World pose (4x4) of each SRDF group's last link (the arms' tool0), by forward kinematics of the cell's model.

    ? Matrices, not `RobotModel.forward_kinematics`: its `Frame.transformed` rebuilds the axes with ~2e-8 rad error.
    """
    if state.robot_configuration is None:
        return {}
    model = cell.robot_model
    transformations = model.compute_transformations(_joint_map(state.robot_configuration))
    base = np.asarray(Transformation.from_frame(state.robot_base_frame).matrix)
    frames = {}
    for group in cell.group_names:
        link = cell.get_link_names(group)[-1]
        joint = model.get_link_by_name(link).parent_joint
        frames[link] = base @ np.asarray(transformations[joint.name].matrix) @ _matrix(joint.current_origin)
    return frames


def compare_kinematics(report: Report, pair, a: Loaded, b: Loaded, key) -> None:
    """Forward kinematics of each arm flange in world, per state."""
    category = report.category(pair, "flange FK")
    robot, sa = a.states[key]
    _, sb = b.states[key]
    fa, fb = flange_frames(a.cells[robot], sa), flange_frames(b.cells[robot], sb)
    if not fa and not fb:
        return
    problems = [] if set(fa) == set(fb) else [("flanges differ", f"{sorted(fa)} vs {sorted(fb)}")]
    for link in sorted(set(fa) & set(fb)):
        position = float(np.abs(fa[link][:3, 3] - fb[link][:3, 3]).max())
        angle = float(Rotation.from_matrix(fa[link][:3, :3].T @ fb[link][:3, :3]).magnitude())
        category.deviation("FK m", position)
        category.deviation("FK rad", angle)
        if position > FRAME_TOL or angle > FRAME_TOL:
            problems.append(("flange FK differs", f"{link} {position:.3g} m, {angle:.3g} rad"))
    category.add(key[1], problems)


# --- --- --- --- --- COLLISIONS --- --- --- --- ---

def collision_pairs(loaded: Loaded, robot: str, keys: List[Tuple[str, str]]) -> Dict[Tuple[str, str], set]:
    """compas_fab's colliding pairs (`check_collision`, full report) per movement, as design ids.

    ? Movements without a configuration (M0 starts) are left out: compas_fab cannot place the robot.
    """
    cell = loaded.cells[robot]
    names = {id(model): loaded.keys[robot][key] for key, model in [*cell.tool_models.items(),
                                                                   *cell.rigid_body_models.items()]}

    def name(model) -> str:
        return names.get(id(model), f"{robot}/{model.name}")

    result = {}
    with PyBulletClient("direct", verbose=False) as client:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(cell)
        for key in keys:
            state = loaded.states[key][1]
            try:
                planner.check_collision(state, {"full_report": True})
                result[key] = set()
            except CollisionCheckError as error:
                result[key] = {tuple(sorted((name(x), name(y)))) for x, y in error.collision_pairs}
    return result


def compare_collisions(report: Report, pair, found: Dict[str, dict]) -> None:
    """Colliding pairs per movement; pairs with a floor are told apart when only one loader has a floor."""
    category = report.category(pair, "collision pairs")
    a, b = found[pair[0]], found[pair[1]]
    for key in sorted(set(a) & set(b)):
        x, y = a[key], b[key]
        floor_x = {p for p in x if GROUND in p}
        floor_y = {p for p in y if GROUND in p}
        problems = []
        if x - floor_x != y - floor_y:
            problems.append(("pairs differ", f"only first {sorted((x - floor_x) - y)}, "
                                             f"only second {sorted((y - floor_y) - x)}"))
        if floor_x != floor_y:
            if floor_x and not floor_y and pair[1] == "new":
                kind = "floor pairs: only in old"
            elif floor_y and not floor_x and pair[0] == "new":
                kind = "floor pairs: only in design"
            else:
                kind = "floor pairs: differ"
            problems.append((kind, f"only first {sorted(floor_x - floor_y)}, only second {sorted(floor_y - floor_x)}"))
        category.add(key[1], problems)
        category.deviation("pairs per state", max(len(x), len(y)))


# --- --- --- --- --- RUN --- --- --- --- ---

def run(export: Path, collisions: bool = True, log: Callable[[str], None] = print) -> Report:
    """Load the export with all three loaders and compare every pair."""
    export = Path(export).expanduser().resolve()
    report = Report()
    with TemporaryDirectory() as tmp:
        log("converting")
        convert(export, Path(tmp) / "design")
        log("loading: design_io")
        design_loaded, design = load_design(Path(tmp) / "design")
    log("loading: export loader")
    loaded = {"new": load_new(export), "design": design_loaded}
    log("loading: old app")
    loaded["old"] = load_old(export)
    for each in loaded.values():
        name_maps(each, design)
    report.notes.append(f"{export.name}: old covers {sorted(loaded['old'].cells)} "
                        f"({len(loaded['old'].states)} movements); new and design cover {sorted(loaded['new'].cells)} "
                        f"({len(loaded['new'].states)} movements)")
    report.notes.append(f"tolerances: joints {JOINT_TOL}, frames {FRAME_TOL} m / rad, mesh points {MESH_TOL} m, "
                        f"areas and volumes {RELATIVE_TOL} relative")
    for pair in PAIRS:
        a, b = loaded[pair[0]], loaded[pair[1]]
        for robot in sorted(set(a.cells) & set(b.cells)):
            log(f"comparing {pair} {robot}: cells")
            compare_cells(report, pair, a, b, robot)
        log(f"comparing {pair}: states")
        for key in sorted(set(a.states) & set(b.states)):
            compare_states(report, pair, a, b, key)
            compare_kinematics(report, pair, a, b, key)
    if collisions:
        found: Dict[str, dict] = {label: {} for label in LOADERS}
        for label, each in loaded.items():
            for robot in each.cells:
                keys = [key for key, (r, state) in each.states.items()
                        if r == robot and state.robot_configuration is not None]
                log(f"collisions: {label} {robot} ({len(keys)} states)")
                found[label].update(collision_pairs(each, robot, keys))
        skipped = sum(1 for _, state in loaded["new"].states.values() if state.robot_configuration is None)
        report.notes.append(f"collisions: {skipped} movements without a configuration are not checked")
        for pair in PAIRS:
            compare_collisions(report, pair, found)
    return report


# --- --- --- --- --- TIMING (each step in a fresh process) --- --- --- --- ---

def _timed(load: Callable[[Stopwatch], object]) -> dict:
    """Seconds and laps of one load."""
    watch = Stopwatch()
    start = time.perf_counter()
    load(watch)
    return {"seconds": time.perf_counter() - start, "laps": watch.laps}


def step_old(export: str) -> dict:
    """Time the old app's loading."""
    return _timed(lambda watch: load_old(Path(export), watch))


def step_new(export: str) -> dict:
    """Time `load_export`."""
    return _timed(lambda watch: load_new(Path(export), watch))


def step_design(design: str) -> dict:
    """Time `read` + every cell + every movement state."""
    return _timed(lambda watch: load_design(Path(design), watch=watch))


def step_design_cindy(design: str) -> dict:
    """Time `read` + Cindy's cell + Cindy's movement states: the old app's scope."""
    return _timed(lambda watch: load_design(Path(design), robots=("robots/cindy",), watch=watch))


def step_convert(export: str, design: str) -> dict:
    """Time `convert_export`."""
    return _timed(lambda watch: convert_export(Path(export), Path(design), DATA, report=quiet, watch=watch))


def run_step(name: str, *args: str) -> Tuple[dict, float]:
    """Run one step in a fresh Python process.

    Returns:
        tuple[dict, float]: The step's seconds and laps, and the whole process's seconds (start-up and imports).
    """
    start = time.perf_counter()
    done = subprocess.run([sys.executable, "-W", "ignore", __file__, "--step", name, *args],
                          capture_output=True, text=True)
    process = time.perf_counter() - start
    lines = [line for line in done.stdout.splitlines() if line.startswith("RESULT ")]
    if done.returncode != 0 or not lines:
        raise RuntimeError(f"step {name} failed:\n{done.stderr[-3000:]}")
    return json.loads(lines[-1][len("RESULT "):]), process


def timing(export: Path, runs: int) -> dict:
    """Median, min and max seconds per loader and for the conversion, `runs` fresh processes each."""
    export = Path(export).expanduser().resolve()
    times = defaultdict(list)
    with TemporaryDirectory() as tmp:
        for i in range(runs):
            print(f"timing run {i + 1}/{runs}", file=sys.stderr)
            steps = (("convert", str(export), str(Path(tmp) / f"design_{i}")), ("old", str(export)),
                     ("new", str(export)), ("design", str(Path(tmp) / f"design_{i}")),
                     ("design_cindy", str(Path(tmp) / f"design_{i}")))
            for name, *args in steps:
                step, process = run_step(name, *args)
                times[name].append(step["seconds"])
                times[f"{name} (process)"].append(process)
                for lap, seconds in step["laps"]:
                    times[f"{name} · {lap}"].append(seconds)
    return {name: {"median": median(v), "min": min(v), "max": max(v), "runs": v} for name, v in times.items()}


def main() -> None:
    """Compare the three loaders on one export; optionally time them."""
    parser = ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("export", type=Path, help="folder with ActionSchedule.json and RobotCell*.json")
    parser.add_argument("--json", type=Path, help="also write the results to this file")
    parser.add_argument("--no-collisions", action="store_true", help="skip the PyBullet collision checks")
    parser.add_argument("--no-compare", action="store_true", help="only time (with --timing)")
    parser.add_argument("--timing", action="store_true", help="also time each loader in fresh processes")
    parser.add_argument("--runs", type=int, default=3, help="timing runs per loader")
    options = parser.parse_args()
    result = {"export": str(options.export)}
    if not options.no_compare:
        report = run(options.export, collisions=not options.no_collisions,
                     log=lambda line: print(f"[{time.strftime('%H:%M:%S')}] {line}", file=sys.stderr))
        report.print()
        result["comparison"] = report.as_dict()
    if options.timing:
        result["timing"] = timing(options.export, options.runs)
        print("\nwall time until every cell and movement state is available (fresh process each):")
        for name, t in result["timing"].items():
            print(f"  {name:20}{t['median']:8.2f} s   [{t['min']:.2f} – {t['max']:.2f}]")
    if options.json:
        options.json.write_text(json.dumps(result, indent=1, default=str))
        print(f"\nwritten {options.json}")


if __name__ == "__main__":
    if len(sys.argv) > 2 and sys.argv[1] == "--step":
        print("RESULT " + json.dumps(globals()[f"step_{sys.argv[2]}"](*sys.argv[3:])))
    else:
        main()
