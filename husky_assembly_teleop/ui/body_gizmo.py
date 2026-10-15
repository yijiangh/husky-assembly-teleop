"""
Gizmos to move and resize one scene body, and picking a body by clicking it in the 3D view.

The plugin keeps the selection and changes the body; this widget only turns operator input into calls on the main
thread: `on_pick(body_id)`, `on_move(body_id, pose)` and `on_resize(body_id, pose, size)`. Call `place` to put the
gizmos on the selected body, e.g. from `draw` when the selection changed.

! Call `remove` in teardown: the click callback is scene-wide, not one of the plugin's nodes.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Iterable

import numpy as np
import viser

from scaffolding_assembly_core.geometry import Pose, shape_mesh
from .quaternion import quaternion_to_wxyz

if TYPE_CHECKING:
    from scaffolding_assembly_core.scene import Scene
    from ..plugin_api.context import PluginContext

#: Smallest side a resize leaves, metres.
MIN_SIZE = 0.01


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


def pick(snapshot: Scene, origin: np.ndarray, direction: np.ndarray, candidates: Iterable[str]) -> str | None:
    """The nearest enabled candidate body the ray hits, by its drawn shapes, or None."""
    nearest, best = None, np.inf
    for body_id in candidates:
        body = snapshot.bodies.get(body_id)
        if body is None or not body.enabled or body_id not in snapshot.world_poses:
            continue
        # * Into the body frame, so the cached shape meshes need no transform.
        inverse = np.linalg.inv(snapshot.world_poses[body_id].matrix())
        local_origin, local_direction = inverse[:3, :3] @ origin + inverse[:3, 3], inverse[:3, :3] @ direction
        for shape in body.geometry.visual:
            mesh = shape_mesh(shape)
            distance = ray_distance(local_origin, local_direction, mesh.vertices, mesh.faces)
            if distance is not None and distance < best:
                nearest, best = body_id, distance
    return nearest


def resize_from_corner(pose: Pose, anchor: np.ndarray, corner: np.ndarray,
                       square: bool = False) -> tuple[Pose, tuple[float, float, float]]:
    """The centre and size that put a box's corner at `corner` while the opposite corner stays at `anchor`.

    Args:
        pose: The body's pose; its orientation is kept.
        anchor: The fixed opposite corner, world frame.
        corner: Where the dragged corner is now, world frame.
        square: Keep x and y equal (a cylinder's diameter), using the larger.

    Returns:
        tuple[Pose, tuple[float, float, float]]: The new pose (centre) and size.
    """
    rotation = pose.matrix()[:3, :3]
    span = rotation.T @ (np.asarray(corner) - anchor)
    size = np.maximum(np.abs(span), MIN_SIZE)
    if square:
        size[:2] = size[:2].max()
    # * The centre is half a size from the anchor, toward the dragged corner.
    center = anchor + rotation @ (np.where(span < 0, -1.0, 1.0) * size / 2)
    return Pose.from_arrays(center, pose.orientation), tuple(float(v) for v in size)


class BodyGizmo:
    """A move gizmo, an optional corner gizmo for resizing, and picking by click, for one plugin's selection."""

    def __init__(self, ctx: PluginContext, name: str, candidates: Callable[[], Iterable[str]],
                 on_pick: Callable[[str], None], on_move: Callable[[str, Pose], None],
                 on_resize: Callable[[str, Pose, tuple[float, float, float]], None] | None = None):
        """Add the hidden gizmos below the plugin's scene root and start listening for clicks.

        Args:
            ctx: The owning plugin's context.
            name: Scene path segment for the gizmos, e.g. "gizmo".
            candidates: The body ids a click may pick; called on the main thread.
            on_pick: Called with the picked body's id. A miss keeps the selection.
            on_move: Called with the body id and its new pose while the move gizmo is dragged.
            on_resize: Called with the body id, its new pose and size while the corner gizmo is dragged; None for
                no corner gizmo.
        """
        self._ctx, self._candidates = ctx, candidates
        root = f"{ctx.view.scene_root}/{name}"
        self._move = ctx.view.scene.add_transform_controls(f"{root}/move", scale=0.6, depth_test=False, visible=False)
        self._corner = None if on_resize is None else ctx.view.scene.add_transform_controls(
            f"{root}/corner", scale=0.25, disable_rotations=True, depth_test=False, visible=False)
        #: Whether clicks pick and the gizmos show; e.g. from an "Active" checkbox.
        self.active = True
        # The body the gizmos are on, its pose and size, and the fixed corner a resize keeps.
        self._target: str | None = None
        self._pose: Pose | None = None
        self._square = False
        self._anchor = np.zeros(3)

        async def _clicked(event: viser.SceneClickEvent) -> None:
            origin, direction = np.array(event.ray_origin), np.array(event.ray_direction)
            ctx.submit("pick body", lambda: self._pick(origin, direction, on_pick))

        self._clicked = _clicked
        ctx.view.scene.on_click()(_clicked)

        @self._move.on_update
        async def _dragged(event: viser.TransformControlsEvent) -> None:
            # * Read here: by the time the intent runs the gizmo may have moved on.
            target, position, (w, x, y, z) = self._target, tuple(event.target.position), tuple(event.target.wxyz)
            if target is not None:
                ctx.submit("move body", lambda: on_move(target, Pose.from_arrays(position, (x, y, z, w))))

        if self._corner is not None:
            @self._corner.on_update
            async def _resized(event: viser.TransformControlsEvent) -> None:
                target, corner = self._target, np.array(event.target.position)
                if target is not None:
                    ctx.submit("resize body", lambda: self._resize(target, corner, on_resize))

    def place(self, body_id: str | None, pose: Pose | None, size: tuple[float, float, float] | None = None,
              square: bool = False) -> None:
        """Put the gizmos on a body, or hide them for None; the corner gizmo only with a size.

        Args:
            body_id: The body the drags apply to.
            pose: Its world pose.
            size: Its box size, for the corner gizmo on its +x +y +z corner; None hides that gizmo.
            square: Keep its x and y size equal while resizing.
        """
        self._target, self._pose, self._square = body_id, pose, square
        shown = self.active and body_id is not None and pose is not None
        self._move.visible = shown
        if shown:
            self._move.position, self._move.wxyz = pose.position, quaternion_to_wxyz(pose.orientation)
        if self._corner is not None:
            self._corner.visible = shown and size is not None
            if self._corner.visible:
                self._place_corner(pose, size)

    def follow(self, pose: Pose, size: tuple[float, float, float] | None = None) -> None:
        """Move the corner gizmo with a body being dragged or resized; the move gizmo follows the drag itself."""
        self._pose = pose
        if self._corner is not None and size is not None and self._corner.visible:
            self._place_corner(pose, size)

    def remove(self) -> None:
        """Stop listening for clicks. Call in teardown."""
        self._ctx.view.scene.remove_click_callback(self._clicked)

    def _place_corner(self, pose: Pose, size: tuple[float, float, float]) -> None:
        """Put the corner gizmo on the +x +y +z corner, and the anchor on the opposite one."""
        half, matrix = np.array(size) / 2, pose.matrix()
        self._corner.position = matrix[:3, :3] @ half + matrix[:3, 3]
        self._corner.wxyz = quaternion_to_wxyz(pose.orientation)
        self._anchor = matrix[:3, :3] @ -half + matrix[:3, 3]

    def _pick(self, origin: np.ndarray, direction: np.ndarray, on_pick: Callable[[str], None]) -> None:
        """Pick the nearest candidate under the click, if active (intent)."""
        if not self.active:
            return
        found = pick(self._ctx.scene.snapshot, origin, direction, self._candidates())
        if found is not None:
            on_pick(found)

    def _resize(self, body_id: str, corner: np.ndarray, on_resize) -> None:
        """Turn a corner drag into a new pose and size for the body (intent)."""
        if body_id != self._target or self._pose is None:
            return
        pose, size = resize_from_corner(self._pose, self._anchor, corner, self._square)
        self._pose = pose
        self._move.position = pose.position
        on_resize(body_id, pose, size)
