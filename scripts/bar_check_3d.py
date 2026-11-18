#!/usr/bin/env python3
"""
A 3D page beside the bar check: the finished structure as a ghost, the chosen bar solid and its marked joints in colour.

Clicking a joint picks it and clicking another bar goes to that bar; the bar check decides what a pick means and sends
the selection and its distances back through `Bar3DView.show`. Scene updates run on a thread of their own, so `show`
returns at once and only the newest selection is drawn.
"""

from __future__ import annotations

import logging
import socket
import threading
import webbrowser
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from scaffolding_assembly_core.geometry import CylinderShape, shape_mesh
from scaffolding_assembly_core.process import Process

GROUND = "ground"

BAR_COLOR, JOINT_COLOR, MARK_COLOR = "#9aa5b1", "#52606d", "#c2410c"
GHOST_COLOR, GROUND_COLOR = "#7b8794", "#cbd2d9"
GHOST_OPACITY, GROUND_OPACITY = 0.2, 0.15
#: The page's light: its strength (pi shows an upward face in its exact colour) and its colour on downward faces.
LIGHT_INTENSITY, LIGHT_FROM_BELOW = np.pi, (150, 150, 150)

#: Vertical field of view of the page's camera, radians.
CAMERA_FOV = np.radians(50.0)
#: Room left around the framed bar: a share of its length on each side, plus a fixed distance, metres.
FRAME_MARGIN_SHARE, FRAME_MARGIN = 0.15, 0.2
#: How far a marked joint's label stands off the bar, metres; each further label stands further off so they do not meet.
LABEL_REACH, LABEL_REACH_STEP = 0.12, 0.07

#: How far from the bar's axis the distances are drawn, and how far along the bar their labels part, metres.
DIMENSION_REACH, DIMENSION_LABEL_SHIFT = 0.2, 0.05
#: Half the width of the square marking a measured edge, metres.
EDGE_MARK = 0.045
#: A click this close to a bar's axis counts as on the bar, which is thin on screen, metres.
BAR_PICK_REACH = 0.02

#: The layers a body can be drawn in, as indices into `_Batch.handles`.
GHOST, SOLID, MARKED = 0, 1, 2

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class Dimension:
    """A distance to draw along the chosen bar.

    Attributes:
        start: Where it starts, mm from the bar's -z end.
        end: Where it ends, likewise.
        text: What it says, e.g. "543.4 mm outer edges".
        color: Its colour, "#rrggbb".
    """

    start: float
    end: float
    text: str
    color: str


@dataclass(frozen=True)
class _BarView:
    """What the page needs to know about one bar.

    Attributes:
        body: Its body id, e.g. "bars/B21".
        center: Its middle in the world, metres.
        axis: Unit vector along it.
        length: Its length, metres.
        joints: Its joints' body ids.
    """

    body: str
    center: np.ndarray
    axis: np.ndarray
    length: float
    joints: frozenset[str]

    def point(self, along: float) -> np.ndarray:
        """The point on its axis this many mm from its -z end, in the world, metres."""
        return self.center + self.axis * (along * 1e-3 - self.length / 2.0)


@dataclass
class _Batch:
    """All bodies sharing one mesh, drawn once per layer; a body shows in one layer, shrunk to nothing in the others.

    Attributes:
        bodies: The bodies, in instance order.
        handles: The scene handles of the ghost, solid and marked layers.
        layers: The layer each body shows in now.
    """

    bodies: list[str]
    handles: tuple[Any, Any, Any]
    layers: np.ndarray

    def paint(self, solid: set[str], marked: set[str]) -> None:
        """Move each body to its layer, sending only the layers that changed."""
        layers = np.array([MARKED if body in marked else SOLID if body in solid else GHOST for body in self.bodies])
        for layer, handle in enumerate(self.handles):
            if not np.array_equal(layers == layer, self.layers == layer):
                handle.batched_scales = (layers == layer).astype(np.float32)
        self.layers = layers


def _rgb(color: str) -> tuple[int, int, int]:
    """The red, green and blue of a "#rrggbb" colour."""
    return int(color[1:3], 16), int(color[3:5], 16), int(color[5:7], 16)


def _batched_rgb(color: str) -> tuple[int, int, int]:
    """The red, green and blue to give batched meshes for this colour: viser shows their colours brighter than given."""
    red, green, blue = (round(255.0 * ((channel / 255.0 + 0.055) / 1.055) ** 2.4) for channel in _rgb(color))
    return red, green, blue


def _free_port() -> int:
    """A port nothing listens on, from the range the system hands out, far from those a monitor uses."""
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def _body_mesh(body: Any) -> tuple[np.ndarray, np.ndarray]:
    """The vertices and faces of all of a body's collision shapes, in the body's frame."""
    meshes = [shape_mesh(shape) for shape in body.geometry.collision]
    offsets = np.cumsum([0] + [len(mesh.vertices) for mesh in meshes[:-1]])
    return (np.vstack([mesh.vertices for mesh in meshes]),
            np.vstack([mesh.faces + offset for mesh, offset in zip(meshes, offsets)]))


def _short(body_id: str) -> str:
    """A bar's short name, or GROUND for any ground body."""
    return GROUND if body_id.startswith("ground/") else body_id.removeprefix("bars/")


def _mate_labels(process: Process) -> dict[str, str]:
    """Each mated joint's label, as the bar check names it: the bar it meets, or "ground-<foot number>"."""
    labels = {}
    for first, second in process.mates:
        for half, other in ((first, second), (second, first)):
            owner = _short(process.bodies[other].mount if other.startswith("joints/") else other)
            foot = half.removesuffix("_ground").rpartition("-")[2]
            labels[half] = f"{GROUND}-{foot}" if owner == GROUND else owner
    return labels


def _bar_view(process: Process, bar_id: str) -> _BarView:
    """Where the bar is and which joints it carries."""
    body = process.bodies[bar_id]
    cylinder = next(shape for shape in body.geometry.collision if isinstance(shape, CylinderShape))
    frame = body.assembly_pose.matrix() @ cylinder.origin.matrix()
    axis = frame[:3, 2] / np.linalg.norm(frame[:3, 2])
    return _BarView(bar_id, frame[:3, 3], axis, cylinder.height, frozenset(process.halves_of(bar_id)))


def _across(vector: np.ndarray, axis: np.ndarray) -> np.ndarray:
    """The part of `vector` square to the unit vector `axis`."""
    return vector - axis * (vector @ axis)


def _unit(vector: np.ndarray) -> np.ndarray | None:
    """`vector` scaled to length one, or None when it is too short to have a direction."""
    length = np.linalg.norm(vector)
    return vector / length if length > 1e-6 else None


def _view_direction(bar: _BarView, middle: np.ndarray) -> np.ndarray:
    """The unit direction from the bar to the camera: side-on to it, from outside the structure, a little above."""
    outward = (bar.center - middle) * (1.0, 1.0, 0.0)
    side = _unit(_across(outward, bar.axis))
    if side is None:
        side = _unit(_across(np.array((1.0, 0.0, 0.0)), bar.axis))
    if side is None:
        side = _unit(_across(np.array((0.0, 1.0, 0.0)), bar.axis))
    above = _unit(_across(np.array((0.0, 0.0, 1.0)), bar.axis))
    return _unit(side if above is None else side + 0.6 * above)


def _ray_depth(origin: np.ndarray, direction: np.ndarray, triangles: np.ndarray) -> float | None:
    """How far along a ray of unit direction its first hit on these (n, 3, 3) triangles lies, or None for a miss."""
    edge_a, edge_b = triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0]
    across = np.cross(direction, edge_b)
    det = np.sum(edge_a * across, axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        inverse = 1.0 / det
        offset = origin - triangles[:, 0]
        u = np.sum(offset * across, axis=1) * inverse
        turned = np.cross(offset, edge_a)
        v = np.sum(direction * turned, axis=1) * inverse
        depth = np.sum(edge_b * turned, axis=1) * inverse
        hit = (np.abs(det) > 1e-12) & (u >= 0.0) & (v >= 0.0) & (u + v <= 1.0) & (depth > 0.0)
    return float(depth[hit].min()) if hit.any() else None


def _ray_near_bar(origin: np.ndarray, direction: np.ndarray, bar: _BarView) -> float | None:
    """How far along a ray of unit direction it passes within BAR_PICK_REACH of the bar's axis, or None."""
    offset = origin - bar.center
    cosine = direction @ bar.axis
    square = max(1.0 - cosine * cosine, 1e-9)
    half = bar.length / 2.0
    along_bar = np.clip((offset @ bar.axis - cosine * (offset @ direction)) / square, -half, half)
    depth = along_bar * cosine - offset @ direction
    if depth <= 0.0:
        return None
    miss = origin + direction * depth - (bar.center + bar.axis * along_bar)
    return float(depth) if np.linalg.norm(miss) <= BAR_PICK_REACH else None


class Bar3DView:
    """The page: one viser server showing the process's finished structure with one bar picked out."""

    def __init__(self, process: Process, on_pick: Callable[[str, str | None], None] | None = None) -> None:
        """Build the scene and start the server on a free port.

        Args:
            process: The process to show.
            on_pick: Called with (bar, joint id) for a clicked joint and (bar, None) for a clicked bar or for clearing
                the chosen bar's selection. ! It runs on a server thread: hand the work to your own thread.
        """
        # Imported here because viser is slow to import and the bar check should start fast without this page.
        from viser import ViserServer

        self._process = process
        self._on_pick = on_pick
        self._labels = _mate_labels(process)
        self._bars = {_short(body): _bar_view(process, body) for body in process.bodies if body.startswith("bars/")}
        centers = [bar.center for bar in self._bars.values()]
        self._middle = np.mean(centers, axis=0) if centers else np.zeros(3)
        self._marks: list[Any] = []
        self._framed: str | None = None
        self._wanted: tuple[str, tuple[str, ...], tuple[Dimension, ...]] | None = None
        self._closed = False
        self._wake = threading.Event()

        self._server = ViserServer(host="127.0.0.1", port=_free_port(), verbose=False)
        self._server.scene.set_up_direction("+z")
        # Soft light from all around: parts keep their true colours and no side of a part turns black.
        self._server.scene.configure_environment_map(hdri=None)
        self._server.scene.configure_default_lights(enabled=False)
        self._server.scene.add_light_hemisphere("/light", ground_color=LIGHT_FROM_BELOW, intensity=LIGHT_INTENSITY,
                                                position=(0.0, 0.0, 1.0))
        self._server.gui.configure_theme(control_layout="floating", show_logo=False, show_share_button=False)
        self._heading = self._server.gui.add_markdown("Choose a bar.")
        self._server.gui.add_button("Clear selection").on_click(lambda _event: self._clear())
        self._batches = self._build_batches()
        self._triangles = self._world_triangles()
        self._server.scene.on_click()(self._clicked)
        self._add_ground()
        self._server.initial_camera.fov = CAMERA_FOV
        self._server.initial_camera.up = (0.0, 0.0, 1.0)
        self._server.initial_camera.position = self._middle + (3.0, -3.0, 2.5)
        self._server.initial_camera.look_at = self._middle

        self._worker = threading.Thread(target=self._run, name="bar-3d-view", daemon=True)
        self._worker.start()

    @property
    def url(self) -> str:
        """str: The page's address."""
        return f"http://127.0.0.1:{self._server.get_port()}"

    def open(self) -> None:
        """Open the page in the browser, on a thread because starting a browser can take a while."""
        threading.Thread(target=webbrowser.open, args=(self.url,), daemon=True).start()

    def show(self, bar: str, joints: Sequence[str] = (), dimensions: Sequence[Dimension] = ()) -> None:
        """Pick out a bar, mark some of its joints and draw distances; returns at once, the page follows shortly after.

        Args:
            bar: The bar's short name, e.g. "B21".
            joints: Full body ids of the joints to mark, e.g. "joints/J20-21_male".
            dimensions: The distances to draw along the bar.

        Raises:
            KeyError: The process has no such bar, or a joint is not on it.
        """
        strangers = set(joints) - self._bars[bar].joints
        if strangers:
            raise KeyError(f"{sorted(strangers)} not on {bar}")
        wanted = (bar, tuple(sorted(set(joints))), tuple(dimensions))
        if self._closed or wanted == self._wanted:
            return
        self._wanted = wanted
        self._wake.set()

    def close(self) -> None:
        """Stop the server; calling it again does nothing."""
        if self._closed:
            return
        self._closed = True
        self._wake.set()
        self._worker.join(timeout=2.0)
        self._server.stop()

    # --- --- --- --- --- CLICKS --- --- --- --- ---

    def _world_triangles(self) -> dict[str, np.ndarray]:
        """Every bar's and joint's mesh triangles in the world, to find what a click hit."""
        triangles = {}
        for body_id, body in self._process.bodies.items():
            if body_id.startswith(("bars/", "joints/")) and body.geometry.collision:
                vertices, faces = _body_mesh(body)
                pose = body.assembly_pose.matrix()
                triangles[body_id] = (vertices @ pose[:3, :3].T + pose[:3, 3])[faces]
        return triangles

    def _pick(self, origin: np.ndarray, direction: np.ndarray) -> tuple[str, str | None] | None:
        """What a click's ray picks: the chosen bar's joints win, even behind ghosts; then whatever it hits first.

        Returns:
            tuple: The bar and the joint picked (None for the bar itself), or None for nothing or the chosen bar's tube.
        """
        chosen = self._wanted[0] if self._wanted is not None else None
        own, others = [], []
        for body_id, triangles in self._triangles.items():
            if not body_id.startswith("joints/"):
                continue
            depth = _ray_depth(origin, direction, triangles)
            if depth is not None:
                bar = _short(self._process.bodies[body_id].mount)
                (own if bar == chosen else others).append((depth, bar, body_id))
        if own:
            return min(own)[1:]
        for name, bar in self._bars.items():
            depth = _ray_near_bar(origin, direction, bar)
            if depth is not None:
                others.append((depth, name, ""))
        if not others:
            return None
        _depth, bar, joint = min(others)
        return None if bar == chosen and not joint else (bar, joint or None)

    def _clicked(self, event: Any) -> None:
        """Report what a click on the page picked."""
        direction = _unit(np.asarray(event.ray_direction, dtype=float))
        hit = self._pick(np.asarray(event.ray_origin, dtype=float), direction) if direction is not None else None
        if hit is not None and self._on_pick is not None:
            self._on_pick(*hit)

    def _clear(self) -> None:
        """Report that the page's Clear selection was pressed."""
        if self._wanted is not None and self._on_pick is not None:
            self._on_pick(self._wanted[0], None)

    # --- --- --- --- --- SCENE --- --- --- --- ---

    def _build_batches(self) -> list[_Batch]:
        """Add every bar and joint, one set of layers per distinct mesh, all in the ghost layer."""
        groups: dict[bytes, tuple[np.ndarray, np.ndarray, list[str]]] = {}
        for body_id, body in self._process.bodies.items():
            if body_id.startswith(("bars/", "joints/")) and body.geometry.collision:
                vertices, faces = _body_mesh(body)
                groups.setdefault(vertices.tobytes() + faces.tobytes(), (vertices, faces, []))[2].append(body_id)
        return [self._add_batch(index, *group) for index, group in enumerate(groups.values())]

    def _add_batch(self, index: int, vertices: np.ndarray, faces: np.ndarray, bodies: list[str]) -> _Batch:
        """Add the ghost, solid and marked layers of the bodies sharing one mesh."""
        poses = [self._process.bodies[body].assembly_pose for body in bodies]
        positions = np.array([pose.position for pose in poses], dtype=np.float32)
        wxyzs = np.array([pose.orientation for pose in poses], dtype=np.float32)[:, [3, 0, 1, 2]]
        solid_color = BAR_COLOR if bodies[0].startswith("bars/") else JOINT_COLOR
        looks = ((GHOST_COLOR, GHOST_OPACITY), (solid_color, None), (MARK_COLOR, None))
        handles = tuple(
            self._server.scene.add_batched_meshes_simple(
                f"/structure/{index}/{layer}", vertices, faces, wxyzs, positions,
                batched_scales=np.full(len(bodies), float(layer == GHOST), dtype=np.float32),
                batched_colors=_batched_rgb(color), opacity=opacity, lod="off", flat_shading=True,
                cast_shadow=False, receive_shadow=False)
            for layer, (color, opacity) in enumerate(looks))
        return _Batch(bodies, handles, np.full(len(bodies), GHOST))

    def _add_ground(self) -> None:
        """Add the ground bodies, very faint."""
        for body_id, body in self._process.bodies.items():
            if body_id.startswith("ground/") and body.geometry.collision:
                vertices, faces = _body_mesh(body)
                pose = body.assembly_pose
                self._server.scene.add_mesh_simple(
                    f"/{body_id}", vertices, faces, color=_rgb(GROUND_COLOR), opacity=GROUND_OPACITY, side="double",
                    position=pose.position, wxyz=np.asarray(pose.orientation)[[3, 0, 1, 2]],
                    cast_shadow=False, receive_shadow=False)

    def _joint_center(self, joint_id: str) -> np.ndarray:
        """The middle of a joint's mesh in the world, metres."""
        body = self._process.bodies[joint_id]
        vertices = _body_mesh(body)[0]
        pose = body.assembly_pose.matrix()
        return pose[:3, :3] @ ((vertices.min(axis=0) + vertices.max(axis=0)) / 2.0) + pose[:3, 3]

    def _label(self, joint_id: str) -> str:
        """What a joint is called on the page: what it mates with, or its own name when it mates with nothing."""
        return self._labels.get(joint_id, joint_id.removeprefix("joints/"))

    def _draw_marks(self, bar: _BarView, joints: tuple[str, ...]) -> None:
        """Replace the labels: each stands off the bar on its joint's side, tied to the joint by a line."""
        for handle in self._marks:
            handle.remove()
        self._marks = []
        for index, joint in enumerate(joints):
            center = self._joint_center(joint)
            side = _unit(_across(center - bar.center, bar.axis))
            if side is None:
                side = _view_direction(bar, self._middle)
            tip = center + side * (LABEL_REACH + index * LABEL_REACH_STEP)
            self._marks.append(self._server.scene.add_line_segments(
                f"/marks/{index}/line", np.array([[center, tip]]), colors=_rgb(MARK_COLOR), line_width=2.0))
            self._marks.append(self._server.scene.add_label(
                f"/marks/{index}/label", self._label(joint), position=tip, anchor="bottom-center",
                font_screen_scale=1.3))

    def _draw_dimensions(self, bar: _BarView, dimensions: tuple[Dimension, ...]) -> None:
        """Draw the distances on either side of the bar as first seen, a square on each measured edge."""
        side = _unit(np.cross(bar.axis, _view_direction(bar, self._middle)))
        square = np.cross(bar.axis, side)
        corners = EDGE_MARK * np.array([side + square, side - square, -side - square, -side + square])
        for index, dimension in enumerate(dimensions):
            sign = 1.0 if index % 2 == 0 else -1.0
            reach = side * DIMENSION_REACH * sign
            start, end = bar.point(dimension.start), bar.point(dimension.end)
            lines = [[start, start + reach], [end, end + reach], [start + reach, end + reach]]
            for edge in (start, end):
                lines += [[edge + corners[i], edge + corners[(i + 1) % 4]] for i in range(4)]
            self._marks.append(self._server.scene.add_line_segments(
                f"/dimensions/{index}/lines", np.array(lines), colors=_rgb(dimension.color), line_width=3.0))
            label_at = (start + end) / 2.0 + reach + bar.axis * DIMENSION_LABEL_SHIFT * sign
            self._marks.append(self._server.scene.add_label(
                f"/dimensions/{index}/label", dimension.text, position=label_at,
                anchor="bottom-center", font_screen_scale=1.5))

    def _write_heading(self, bar: str, joints: tuple[str, ...], dimensions: tuple[Dimension, ...]) -> None:
        """Name the bar, its marked joints and their distances at the top of the page, and say what a click does."""
        marked = ", ".join(f"`{joint.removeprefix('joints/')}`" + (f" to {self._labels[joint]}" if joint in self._labels
                                                                   else " (mates with nothing)") for joint in joints)
        distances = "  \n".join(f"**{dimension.text}**" for dimension in dimensions)
        next_step = "" if dimensions else f"Click {'a second joint' if joints else 'two joints'} to measure."
        self._heading.content = "\n\n".join(part for part in (f"### {bar}", marked, distances, next_step) if part)

    def _frame(self, bar: _BarView) -> None:
        """Point every open page at the bar, and any page opened later too."""
        half_view = bar.length * (0.5 + FRAME_MARGIN_SHARE) + FRAME_MARGIN
        distance = half_view / np.tan(CAMERA_FOV / 2.0)
        position = bar.center + _view_direction(bar, self._middle) * distance
        self._server.initial_camera.position = position
        self._server.initial_camera.look_at = bar.center
        for client in self._server.get_clients().values():
            with client.atomic():
                client.camera.fov = CAMERA_FOV
                client.camera.up_direction = (0.0, 0.0, 1.0)
                client.camera.position = position
                # ! After the position: setting the position alone carries the point looked at along with it.
                client.camera.look_at = bar.center

    def _apply(self, name: str, joints: tuple[str, ...], dimensions: tuple[Dimension, ...]) -> None:
        """Bring the page to this selection; the camera moves only when the bar changed."""
        bar = self._bars[name]
        with self._server.atomic():
            for batch in self._batches:
                batch.paint({bar.body, *bar.joints}, set(joints))
            self._draw_marks(bar, joints)
            self._draw_dimensions(bar, dimensions)
            self._write_heading(name, joints, dimensions)
        if name != self._framed:
            self._framed = name
            self._frame(bar)

    def _run(self) -> None:
        """Draw the newest selection whenever it changes, until the view is closed."""
        drawn = None
        while True:
            self._wake.wait()
            self._wake.clear()
            if self._closed:
                return
            wanted = self._wanted
            if wanted is None or wanted == drawn:
                continue
            try:
                self._apply(*wanted)
                drawn = wanted
            except Exception:  # ! A failed drawing must not end the thread: the next selection may well work.
                logger.exception("Could not show %s", wanted)
