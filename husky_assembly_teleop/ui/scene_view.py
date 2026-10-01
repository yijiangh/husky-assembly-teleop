"""
The 3D view of the scene: bodies under /scene/<id> and tracked objects under /tracked/<name>, drawn from
each tick's snapshot.

At most `build_budget` meshes are built per tick, so a large cell fills in over a few ticks; moves and
removes always apply in full, so a body is never shown at an old pose.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import viser

from ..design_io.geometry import BoxShape, CylinderShape, Geometry, Shape
from ..world.scene import SceneSnapshot
from ..design_io.pose import Pose
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


@dataclass
class _DrawnTracked:
    """One tracked object's frame and meshes.

    Attributes:
        frame: Frame with axes, carrying the pose.
        pose: The last pose assigned to the frame.
        geometry: The Geometry object the shapes were built from; None if not built (yet).
        meshes: The shape handles, children of `frame`.
    """

    frame: viser.FrameHandle
    pose: Pose
    geometry: Geometry | None = None
    meshes: list[viser.SceneNodeHandle] = field(default_factory=list)


class SceneView:
    """Draws the scene's bodies and the tracked objects from each tick's snapshot. Main thread only."""

    def __init__(self, server: viser.ViserServer, build_budget: int = 50) -> None:
        """Create the root frames.

        Args:
            server: The viser server to draw in.
            build_budget: Shapes built at most per `sync`; the rest wait for the next one.
        """
        self._server = server
        self._build_budget = build_budget
        server.scene.add_frame("/scene", show_axes=False)
        server.scene.add_frame("/tracked", show_axes=False)
        self._bodies: dict[str, _DrawnBody] = {}
        self._tracked: dict[str, _DrawnTracked] = {}

    def sync(self, snapshot: SceneSnapshot) -> None:
        """Bring the view in line with one snapshot. Call once per tick, inside `server.atomic()`.

        Args:
            snapshot: The tick's copy of the world.
        """
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
                frame = self._server.scene.add_frame(f"/scene/{body_id}", show_axes=False,
                                                     position=pose.position, wxyz=quaternion_to_wxyz(pose.orientation))
                budget -= len(self._add_meshes(f"/scene/{body_id}", body.geometry, body.color))
                self._bodies[body_id] = _DrawnBody(body.geometry, body.color, pose, frame)
            elif pose != drawn.pose:
                # ? Compared first: viser sends a message for every assignment.
                drawn.frame.position, drawn.frame.wxyz = pose.position, quaternion_to_wxyz(pose.orientation)
                drawn.pose = pose

        self._sync_tracked(snapshot, budget)

    def _sync_tracked(self, snapshot: SceneSnapshot, budget: int) -> None:
        """Move, show and hide the tracked objects' frames, and build their meshes. Missing ones are hidden.

        Args:
            snapshot: The tick's copy of the world.
            budget: Shapes left to build this tick.
        """
        for name, drawn in self._tracked.items():
            drawn.frame.visible = name in snapshot.tracked
        for name, entry in snapshot.tracked.items():
            drawn = self._tracked.get(name)
            if drawn is None:
                frame = self._server.scene.add_frame(f"/tracked/{name}", axes_length=0.1, axes_radius=0.004,
                                                     position=entry.pose.position,
                                                     wxyz=quaternion_to_wxyz(entry.pose.orientation))
                drawn = self._tracked[name] = _DrawnTracked(frame, entry.pose)
            elif entry.pose != drawn.pose:
                drawn.frame.position, drawn.frame.wxyz = entry.pose.position, quaternion_to_wxyz(entry.pose.orientation)
                drawn.pose = entry.pose

            geometry = entry.description.geometry
            if geometry is not drawn.geometry:
                for mesh in drawn.meshes:
                    mesh.remove()
                drawn.geometry, drawn.meshes = None, []
                if geometry is not None and budget > 0:
                    drawn.geometry, drawn.meshes = geometry, self._add_meshes(f"/tracked/{name}", geometry, None)
                    budget -= len(drawn.meshes)

    def _add_meshes(self, parent: str, geometry: Geometry,
                    color: tuple[float, float, float, float] | None) -> list[viser.SceneNodeHandle]:
        """Add a geometry's visual shapes below a frame, in the frame's coordinates.

        Args:
            parent: Name of the frame.
            geometry: The shapes to draw.
            color: (r, g, b, a) from 0 to 1, or None for DEFAULT_COLOR.

        Returns:
            list[viser.SceneNodeHandle]: One handle per shape.
        """
        return [self._add_shape(f"{parent}/mesh_{i}", shape, color or DEFAULT_COLOR)
                for i, shape in enumerate(geometry.visual)]

    def _add_shape(self, name: str, shape: Shape,
                   color: tuple[float, float, float, float]) -> viser.SceneNodeHandle:
        """Add one shape: primitives natively (exact, lit correctly), meshes as triangles.

        Args:
            name: Scene node name.
            shape: What to draw, in its parent frame's coordinates.
            color: (r, g, b, a) from 0 to 1.

        Returns:
            viser.SceneNodeHandle: Its handle.
        """
        r, g, b, a = color
        look = dict(color=(r, g, b), opacity=None if a >= 1.0 else a)
        scene = self._server.scene
        if isinstance(shape, BoxShape):
            return scene.add_box(name, dimensions=shape.size, position=shape.origin.position,
                                 wxyz=quaternion_to_wxyz(shape.origin.orientation), **look)
        if isinstance(shape, CylinderShape):
            # ? viser turns its cylinders onto Z, as PyBullet and trimesh do.
            return scene.add_cylinder(name, radius=shape.radius, height=shape.height,
                                      position=shape.origin.position,
                                      wxyz=quaternion_to_wxyz(shape.origin.orientation), **look)
        return scene.add_mesh_simple(name, shape.vertices, shape.faces, **look)
