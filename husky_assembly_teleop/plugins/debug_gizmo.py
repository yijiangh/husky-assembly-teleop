"""
Quick debug tool: click any scene body with a world pose, drag it with a gizmo, and log where it ends up.

Clicking in the 3D view picks the nearest movable body under the cursor; the dropdown does the same.
Position and roll, pitch, yaw can also be typed in. "Add" puts temporary boxes and cylinders into the scene, which
planners avoid like any obstacle; they are resized with a second gizmo on one corner, or by typing their size.
Untick "Active" to hide the gizmos and stop picking by click; viser does not tell the server whether a folder is open.

! Writes other plugins' live bodies directly, which plugins normally must not do. Debug only: the owner may put
  the body back at any time, and nothing is saved.

Run with:  -p plugins:="['obstacles', 'debug_gizmo']"
"""

from __future__ import annotations

from html import escape

import numpy as np
import viser
from scipy.spatial.transform import Rotation

from ..design_io.geometry import Geometry, box_geometry, cylinder_geometry, shape_mesh
from ..design_io.pose import Pose
from ..plugin_api.context import PluginContext
from ..plugin_api.plugin import HuskyPlugin, register
from ..ui.quaternion import quaternion_to_wxyz
from ..ui.style import SECTION_CTRL, block, note, numbers, section, values
from ..world.scene import Body

#: Dropdown entry while no body can be moved.
NONE_ENTRY = "(none)"
#: Size of a new temporary obstacle, metres: box sides, or a cylinder's (diameter, diameter, height).
TMP_SIZE = (0.5, 0.5, 1.0)
#: Smallest side of a temporary obstacle, metres.
TMP_MIN = 0.01
#: Colour of temporary obstacles, (r, g, b, a) from 0 to 1.
TMP_COLOR = (0.6, 0.35, 0.9, 1.0)


def _rpy(pose: Pose) -> tuple[float, float, float]:
    """Roll, pitch, yaw of `pose`, degrees."""
    return tuple(Rotation.from_quat(pose.orientation).as_euler("xyz", degrees=True))


def _tmp_geometry(kind: str, size: tuple[float, float, float]) -> Geometry:
    """The geometry of a temporary "box" or "cylinder"; a cylinder takes size x as its diameter."""
    return box_geometry(size) if kind == "box" else cylinder_geometry(size[0] / 2, size[2])


def ray_distance(origin: np.ndarray, direction: np.ndarray, vertices: np.ndarray, faces: np.ndarray) -> float | None:
    """Distance along the ray to the nearest triangle it hits, or None (Möller–Trumbore, both faces count)."""
    a, b, c = (vertices[faces[:, i]] for i in range(3))
    ab, ac = b - a, c - a
    p = np.cross(direction, ac)
    det = np.einsum("ij,ij->i", ab, p)
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / det
        t_vec = origin - a
        u = np.einsum("ij,ij->i", t_vec, p) * inv
        q = np.cross(t_vec, ab)
        v = (q @ direction) * inv
        t = np.einsum("ij,ij->i", ac, q) * inv
        hit = (np.abs(det) > 1e-12) & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 0)
    return float(t[hit].min()) if hit.any() else None


@register
class DebugGizmoPlugin(HuskyPlugin):
    """Moves any world-placed scene body with a gizmo and logs its pose; adds and resizes temporary obstacles."""

    name = "debug_gizmo"

    def __init__(self):
        """Start with nothing selected and no temporary obstacles."""
        # Each body's pose when first seen, by id; for Reset and the shown offset.
        self._original: dict[str, Pose] = {}
        self._gizmo: viser.TransformControlsHandle | None = None
        self._corner: viser.TransformControlsHandle | None = None
        # The body the gizmos were last put on.
        self._gizmo_on: str | None = None
        # Temporary obstacles by id: ("box" or "cylinder", size).
        self._tmp: dict[str, tuple[str, tuple[float, float, float]]] = {}
        self._tmp_count = 0
        # World point the corner gizmo resizes against: the corner opposite it, fixed while dragging.
        self._anchor = np.zeros(3)

    def setup(self, ctx: PluginContext) -> None:
        """Build the widgets and the (hidden) gizmos.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            gui.add_html(section("move body", SECTION_CTRL))
            self._active = gui.add_checkbox("Active", True, hint="Show the gizmos and pick bodies by clicking in 3D.")
            self._select = gui.add_dropdown("Body", [NONE_ENTRY])
            self._position = gui.add_vector3("Position m", (0.0, 0.0, 0.0), step=0.001)
            self._rpy = gui.add_vector3("RPY °", (0.0, 0.0, 0.0), step=0.1,
                                        hint="Roll, pitch, yaw about fixed X, Y, Z, degrees.")
            self._size = gui.add_vector3("Size m", TMP_SIZE, step=0.01, min=(TMP_MIN,) * 3, visible=False,
                                         hint="Box sides; a cylinder takes x as its diameter and z as its height.")
            actions = gui.add_button_group("Selected", ["Log", "Reset", "Remove"])
            add = gui.add_button_group("Add", ["Box", "Cylinder"])
            self._status: viser.GuiHtmlHandle = gui.add_html("")
        self._gizmo = ctx.view.scene.add_transform_controls(f"{ctx.view.scene_root}/gizmo", scale=0.6,
                                                            depth_test=False, visible=False)
        self._corner = ctx.view.scene.add_transform_controls(f"{ctx.view.scene_root}/corner", scale=0.25,
                                                             disable_rotations=True, depth_test=False, visible=False)

        async def _clicked(event: viser.SceneClickEvent) -> None:
            origin, direction = np.array(event.ray_origin), np.array(event.ray_direction)
            ctx.submit("pick body", lambda: self._pick(ctx, origin, direction))

        # * A scene-wide callback, not one of our nodes: teardown must remove it.
        self._clicked = _clicked
        ctx.view.scene.on_click()(_clicked)

        # ! Callbacks run on a viser thread: hand every action to the main thread.
        actions.on_click(ctx.defer_value("selected body", lambda clicked: {
            "Log": self._log, "Reset": self._reset, "Remove": self._remove}[clicked](ctx)))
        add.on_click(ctx.defer_value("add obstacle", lambda clicked: self._add(ctx, clicked.lower())))

        async def _typed(event: viser.GuiEvent) -> None:
            # * Skip our own writes from `draw`: viser calls back for those too, without a client.
            if event.client is None:
                return
            body_id, position, rpy = self._select.value, self._position.value, self._rpy.value
            ctx.submit("type pose", lambda: self._set_pose(ctx, body_id, position, rpy))

        self._position.on_update(_typed)
        self._rpy.on_update(_typed)

        async def _typed_size(event: viser.GuiEvent) -> None:
            if event.client is None:
                return
            body_id, size = self._select.value, self._size.value
            ctx.submit("type size", lambda: self._set_size(ctx, body_id, size))

        self._size.on_update(_typed_size)

        @self._gizmo.on_update
        async def _dragged(event: viser.TransformControlsEvent) -> None:
            # * Read the pose here: by the time the intent runs the gizmo may have moved on.
            body_id, position, wxyz = self._select.value, tuple(event.target.position), tuple(event.target.wxyz)
            ctx.submit("drag body", lambda: self._move(ctx, body_id, position, wxyz))

        @self._corner.on_update
        async def _resized(event: viser.TransformControlsEvent) -> None:
            body_id, position = self._select.value, np.array(event.target.position)
            ctx.submit("resize body", lambda: self._resize(ctx, body_id, position))

    def teardown(self, ctx: PluginContext) -> None:
        """Remove the scene click callback; the core removes the temporary obstacles.

        Args:
            ctx: This plugin's context.
        """
        ctx.view.scene.remove_click_callback(self._clicked)

    # --- --- --- --- --- ACTIONS (run as intents) --- --- --- --- ---

    def _pick(self, ctx: PluginContext, origin: np.ndarray, direction: np.ndarray) -> None:
        """Select the nearest shown, movable body the click ray hits; a miss, or not Active, keeps the selection."""
        if not self._active.value:
            return
        snapshot = ctx.scene.snapshot
        nearest, best = None, np.inf
        for body_id, body in snapshot.bodies.items():
            if not body.enabled or self._pose(ctx, body_id) is None:
                continue
            # * Into the body frame, so the cached shape meshes need no transform.
            inverse = np.linalg.inv(snapshot.world_poses[body_id].matrix())
            local_origin = inverse[:3, :3] @ origin + inverse[:3, 3]
            local_direction = inverse[:3, :3] @ direction
            for shape in body.geometry.visual:
                mesh = shape_mesh(shape)
                distance = ray_distance(local_origin, local_direction, mesh.vertices, mesh.faces)
                if distance is not None and distance < best:
                    nearest, best = body_id, distance
        if nearest is not None and nearest in self._select.options:
            self._select.value = nearest

    def _add(self, ctx: PluginContext, kind: str) -> None:
        """Add a temporary "box" or "cylinder" standing at the origin, and select it."""
        self._tmp_count += 1
        body_id = f"{self.name}/tmp/{kind}_{self._tmp_count}"
        ctx.scene.put(Body(body_id, _tmp_geometry(kind, TMP_SIZE), Pose((0.0, 0.0, TMP_SIZE[2] / 2)),
                           label=f"tmp {kind} {self._tmp_count}", color=TMP_COLOR))
        self._tmp[body_id] = (kind, TMP_SIZE)
        self._sync_options(ctx)
        self._select.value = body_id

    def _remove(self, ctx: PluginContext) -> None:
        """Remove the selected body if it is a temporary obstacle."""
        body_id = self._select.value
        if body_id in self._tmp:
            del self._tmp[body_id]
            ctx.scene.remove(body_id)
            self._sync_options(ctx)

    def _pose(self, ctx: PluginContext, body_id: str) -> Pose | None:
        """The world pose of body `body_id`, or None if it is gone or attached."""
        body = ctx.scene.bodies.get(body_id)
        return body.placement if body is not None and isinstance(body.placement, Pose) else None

    def _place_gizmo(self, ctx: PluginContext, body_id: str) -> None:
        """Put the gizmo on body `body_id`, and the corner gizmo on its +x +y +z corner if it is temporary."""
        pose = self._pose(ctx, body_id)
        self._gizmo_on = body_id
        if pose is None:
            return
        self._gizmo.position = pose.position
        self._gizmo.wxyz = quaternion_to_wxyz(pose.orientation)
        self._place_corner(body_id, pose)

    def _place_corner(self, body_id: str, pose: Pose) -> None:
        """Put the corner gizmo on a temporary obstacle's +x +y +z corner, and its anchor on the opposite one."""
        if body_id in self._tmp:
            half = np.array(self._tmp[body_id][1]) / 2
            matrix = pose.matrix()
            self._corner.position = matrix[:3, :3] @ half + matrix[:3, 3]
            self._corner.wxyz = quaternion_to_wxyz(pose.orientation)
            self._anchor = matrix[:3, :3] @ -half + matrix[:3, 3]

    def _move(self, ctx: PluginContext, body_id: str, position: tuple[float, ...], wxyz: tuple[float, ...]) -> None:
        """Set body `body_id` to the gizmo pose; the corner gizmo follows."""
        if self._pose(ctx, body_id) is not None:
            w, x, y, z = wxyz
            pose = Pose.from_arrays(position, (x, y, z, w))
            ctx.scene.bodies[body_id].placement = pose
            self._place_corner(body_id, pose)

    def _resize(self, ctx: PluginContext, body_id: str, corner: np.ndarray) -> None:
        """Resize temporary obstacle `body_id` so its corner is at `corner` and the opposite one stays put."""
        pose = self._pose(ctx, body_id)
        if pose is None or body_id not in self._tmp:
            return
        kind, _ = self._tmp[body_id]
        rotation = pose.matrix()[:3, :3]
        span = rotation.T @ (corner - self._anchor)
        size = np.maximum(np.abs(span), TMP_MIN)
        if kind == "cylinder":
            size[:2] = size[:2].max()
        # * The centre is half a size from the anchor, toward the dragged corner.
        center = self._anchor + rotation @ (np.where(span < 0, -1.0, 1.0) * size / 2)
        self._apply_size(ctx, body_id, tuple(size), Pose.from_arrays(center, pose.orientation))

    def _set_size(self, ctx: PluginContext, body_id: str, size: tuple[float, float, float]) -> None:
        """Set a temporary obstacle's typed size, keeping its centre, and move the gizmos with it."""
        pose = self._pose(ctx, body_id)
        if pose is not None and body_id in self._tmp:
            size = tuple(max(float(v), TMP_MIN) for v in size)
            if self._tmp[body_id][0] == "cylinder":
                size = (size[0], size[0], size[2])
            self._apply_size(ctx, body_id, size, pose)
            self._place_gizmo(ctx, body_id)

    def _apply_size(self, ctx: PluginContext, body_id: str, size: tuple[float, ...], pose: Pose) -> None:
        """Give temporary obstacle `body_id` a new size and pose; a new Geometry, as the scene requires."""
        kind, _ = self._tmp[body_id]
        self._tmp[body_id] = (kind, size)
        body = ctx.scene.bodies[body_id]
        body.geometry = _tmp_geometry(kind, size)
        body.placement = pose
        self._gizmo.position = pose.position

    def _set_pose(self, ctx: PluginContext, body_id: str, position: tuple[float, ...], rpy: tuple[float, ...]) -> None:
        """Set body `body_id` to a typed position and roll, pitch, yaw (degrees), and move the gizmos with it."""
        if self._pose(ctx, body_id) is not None:
            orientation = Rotation.from_euler("xyz", rpy, degrees=True).as_quat()
            ctx.scene.bodies[body_id].placement = Pose.from_arrays(position, orientation)
            self._place_gizmo(ctx, body_id)

    def _log(self, ctx: PluginContext) -> None:
        """Log the selected body's pose: position, quaternion (x, y, z, w), roll, pitch, yaw, and a temporary size."""
        body_id = self._select.value
        pose = self._pose(ctx, body_id)
        if pose is None:
            return
        size = ""
        if body_id in self._tmp:
            kind, sides = self._tmp[body_id]
            size = f" {kind} size=({', '.join(f'{v:.4f}' for v in sides)})"
        ctx.log_info(f"{body_id}: position=({', '.join(f'{v:.4f}' for v in pose.position)}) "
                     f"orientation=({', '.join(f'{v:.4f}' for v in pose.orientation)}) "
                     f"rpy_deg=({', '.join(f'{v:.2f}' for v in _rpy(pose))}){size}")

    def _reset(self, ctx: PluginContext) -> None:
        """Put the selected body back where it was when first seen."""
        body_id = self._select.value
        if self._pose(ctx, body_id) is not None and body_id in self._original:
            ctx.scene.bodies[body_id].placement = self._original[body_id]
            self._place_gizmo(ctx, body_id)

    # --- --- --- --- --- DRAW --- --- --- --- ---

    def _sync_options(self, ctx: PluginContext) -> None:
        """List every movable body in the dropdown, and remember the pose of new ones."""
        ids = sorted(body_id for body_id in ctx.scene.bodies if self._pose(ctx, body_id) is not None)
        for body_id in ids:
            self._original.setdefault(body_id, self._pose(ctx, body_id))
        options = tuple(ids) or (NONE_ENTRY,)
        if tuple(self._select.options) != options:
            self._select.options = options

    def draw(self, ctx: PluginContext) -> None:
        """Keep the body list current, and show the selected body's pose (and size) in the fields and its offset.

        Args:
            ctx: This plugin's context.
        """
        self._sync_options(ctx)
        body_id = self._select.value
        if body_id != self._gizmo_on:
            self._place_gizmo(ctx, body_id)
        pose = self._pose(ctx, body_id)
        self._size.visible = body_id in self._tmp
        shown = self._active.value and pose is not None
        self._gizmo.visible = shown
        self._corner.visible = shown and body_id in self._tmp
        if pose is None:
            self._status.content = note("no movable body: add one, or load a plugin that puts bodies, e.g. obstacles")
            return
        original = self._original[body_id]
        offset = tuple(a - b for a, b in zip(pose.position, original.position))
        # ? Rounded, so the fields are only rewritten when the pose really changes.
        self._position.value = tuple(round(v, 4) for v in pose.position)
        self._rpy.value = tuple(round(v, 2) for v in _rpy(pose))
        if body_id in self._tmp:
            self._size.value = tuple(round(v, 4) for v in self._tmp[body_id][1])
        self._status.content = block(escape(ctx.scene.snapshot.label(body_id)) + values(
            f"Δ   {numbers(offset, 3, 8, 3)} m"))
