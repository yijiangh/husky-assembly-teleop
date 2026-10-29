"""
The 3D view of the scene, drawn from each tick's snapshot.

Bodies go under /scene/<id>; tracked objects (ids "tracked/<name>") get axes too.

At most `build_budget` meshes are built per tick, so a large cell fills in over a few ticks; moves and
removes always apply in full, so a body is never shown at an old pose. Switching between visual and collision
shapes rebuilds everything, at the same budget.
"""

from __future__ import annotations

from dataclasses import dataclass

import viser

from scaffolding_assembly_core.geometry import BoxShape, CylinderShape, Geometry, Shape
from scaffolding_assembly_core.ids import TRACKED
from scaffolding_assembly_core.scene import Scene
from scaffolding_assembly_core.geometry import Pose
from .quaternion import quaternion_to_wxyz

#: Colour of a body without one of its own, (r, g, b, a) from 0 to 1.
DEFAULT_COLOR = (0.7, 0.7, 0.7, 1.0)


@dataclass
class _DrawnBody:
    """What one body was drawn with, to tell what changed.

    Attributes:
        geometry: The Geometry object its meshes were built from.
        color: The colour they were built with.
        pose: The last world pose assigned to the frame.
        frame: Frame carrying the pose; its meshes are its children.
    """

    geometry: Geometry
    color: tuple[float, float, float, float] | None
    pose: Pose
    frame: viser.FrameHandle


class SceneView:
    """Draws the scene's bodies, tracked objects included, from each tick's snapshot. Main thread only."""

    def __init__(self, server: viser.ViserServer, build_budget: int = 50) -> None:
        """Create the root frames.

        Args:
            server: The viser server to draw in.
            build_budget: Shapes built at most per `sync`; the rest wait for the next one.
        """
        self._server = server
        self._build_budget = build_budget
        server.scene.add_frame("/scene", show_axes=False)
        self._bodies: dict[str, _DrawnBody] = {}
        self._collision = False

    def sync(self, snapshot: Scene, collision: bool = False) -> None:
        """Bring the view in line with one snapshot. Call once per tick, inside `server.atomic()`.

        Args:
            snapshot: The tick's copy of the world.
            collision: Draw the collision shapes instead of the visual ones.
        """
        if collision != self._collision:
            self._collision = collision
            for drawn in self._bodies.values():
                drawn.frame.remove()
            self._bodies.clear()
        budget = self._build_budget
        # Gone, or changed shape or colour: remove now, rebuild below when the budget allows.
        for body_id, drawn in list(self._bodies.items()):
            body = snapshot.bodies.get(body_id)
            if body is None or body.geometry is not drawn.geometry or body.color != drawn.color:
                drawn.frame.remove()
                del self._bodies[body_id]

        for body_id, body in snapshot.bodies.items():
            pose = snapshot.world_poses[body_id]
            drawn = self._bodies.get(body_id)
            if drawn is None:
                if budget <= 0:
                    continue  # not drawn yet; built in a later tick
                # * Tracked objects show their frame: some are a frame only (the mocap probe).
                axes = body_id.startswith(f"{TRACKED}/")
                frame = self._server.scene.add_frame(f"/scene/{body_id}", show_axes=axes, axes_length=0.1,
                                                     axes_radius=0.004, visible=body.enabled, position=pose.position,
                                                     wxyz=quaternion_to_wxyz(pose.orientation))
                budget -= len(self._add_meshes(f"/scene/{body_id}", body.geometry, body.color))
                self._bodies[body_id] = _DrawnBody(body.geometry, body.color, pose, frame)
            elif pose != drawn.pose:
                # ? Compared first: viser sends a message for every assignment.
                drawn.frame.position, drawn.frame.wxyz = pose.position, quaternion_to_wxyz(pose.orientation)
                drawn.pose = pose
            if drawn is not None and drawn.frame.visible != body.enabled:
                drawn.frame.visible = body.enabled  # disabled: hidden, kept for when it comes back

    def _add_meshes(self, parent: str, geometry: Geometry,
                    color: tuple[float, float, float, float] | None) -> list[viser.SceneNodeHandle]:
        """Add a geometry's visual shapes, or its collision ones while drawing those, below a frame.

        Args:
            parent: Name of the frame.
            geometry: The shapes to draw.
            color: (r, g, b, a) from 0 to 1, or None for DEFAULT_COLOR.

        Returns:
            list[viser.SceneNodeHandle]: One handle per shape.
        """
        # ! Distinct names per mode: viser keeps a removed node's pose for a new node of the same name, so a
        #   collision shape would inherit an offset visual's position (e.g. floor marks, drawn 1 m underground).
        kind, shapes = ("collision", geometry.collision) if self._collision else ("visual", geometry.visual)
        return [add_shape(self._server.scene, f"{parent}/{kind}_{i}", shape, color or DEFAULT_COLOR)
                for i, shape in enumerate(shapes)]


def add_labelled_frame(scene, name: str, pose: Pose, text: str, axes_length: float,
                       color: tuple[int, int, int]) -> tuple[viser.FrameHandle, viser.LabelHandle]:
    """A frame at `pose` with `text` at its origin, both under `name`; the origin ball takes `color` (labels have none).

    Args:
        scene: The scene api, e.g. `ctx.view.scene`.
        name: The frame's scene path; the label goes at `<name>/label`.
        pose: Where, in the world.
        text: The label.
        axes_length: The axes' length, metres.
        color: RGB 0-255 of the origin ball.

    Returns:
        tuple[viser.FrameHandle, viser.LabelHandle]: The frame and its label.
    """
    frame = scene.add_frame(name, position=tuple(float(v) for v in pose.position),
                            wxyz=quaternion_to_wxyz(pose.orientation), axes_length=axes_length,
                            axes_radius=axes_length / 30.0, origin_radius=axes_length / 10.0, origin_color=color)
    label = scene.add_label(f"{name}/label", text)
    return frame, label


def add_shape(scene, name: str, shape: Shape, color: tuple[float, ...]) -> viser.SceneNodeHandle:
    """Add one shape: primitives natively (exact, lit correctly), meshes as triangles.

    Args:
        scene: A viser scene API (`server.scene`, or a PluginView's `scene`).
        name: Scene node name.
        shape: What to draw, in its parent frame's coordinates.
        color: (r, g, b) or (r, g, b, a), from 0 to 1.

    Returns:
        viser.SceneNodeHandle: Its handle; its `color` and `opacity` can be changed later.
    """
    r, g, b, a = (*color, 1.0)[:4]
    look = dict(color=(r, g, b), opacity=None if a >= 1.0 else a)
    if isinstance(shape, BoxShape):
        return scene.add_box(name, dimensions=shape.size, position=shape.origin.position,
                             wxyz=quaternion_to_wxyz(shape.origin.orientation), **look)
    if isinstance(shape, CylinderShape):
        # ? viser turns its cylinders onto Z, as PyBullet and trimesh do.
        return scene.add_cylinder(name, radius=shape.radius, height=shape.height,
                                  position=shape.origin.position,
                                  wxyz=quaternion_to_wxyz(shape.origin.orientation), **look)
    return scene.add_mesh_simple(name, shape.vertices, shape.faces, **look)
