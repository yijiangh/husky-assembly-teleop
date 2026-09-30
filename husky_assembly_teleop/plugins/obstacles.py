"""
Fixed lab boxes (tables, cabinets), drawn in the 3D view and added to the shared PyBullet scene.

* Plugins with their own world (e.g. the base planner) read the `boxes` list and
  declare `requires = ("obstacles",)`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import pybullet_planning as pp

from ..context import PluginContext
from ..plugin import HuskyPlugin, register
from ..pose_input import yaw_to_wxyz

#: Muted grey, so the boxes read as furniture.
BOX_COLOR = (150, 150, 160)


@dataclass(frozen=True)
class Box:
    """One box standing on the floor.

    Attributes:
        name: Label, shown in collision messages.
        center: (x, y, z) of the box's middle, metres, world frame.
        size: (x, y, z) side lengths, metres, before turning.
        yaw: Turn about Z, radians.
    """

    name: str
    center: tuple[float, float, float]
    size: tuple[float, float, float]
    yaw: float = 0.0

    @property
    def orientation(self) -> tuple[float, float, float, float]:
        """tuple[float, float, float, float]: `yaw` as an (x, y, z, w) quaternion."""
        return (0.0, 0.0, math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0))

    @classmethod
    def from_top_points(cls, name: str, points: tuple[tuple[float, float, float], ...]) -> Box:
        """Fit an axis-aligned box from floor to points measured on its top surface.

        The XY footprint is the points' bounding rectangle, so take them at opposite corners.
        The top height is their mean Z.

        Args:
            name: Label, shown in collision messages.
            points: (x, y, z) points on the top surface, metres, world frame.

        Returns:
            Box: The fitted box, with no yaw.
        """
        xs, ys, zs = zip(*points)
        height = sum(zs) / len(zs)
        return cls(name, center=((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, height / 2),
                   size=(max(xs) - min(xs), max(ys) - min(ys), height))


#: The lab's static obstacles, fitted from mocap pointer samples (2026-09-30) on each top surface.
BOXES = (
    Box.from_top_points("building cabinet", ((-0.412, 2.012, 1.094), (3.788, 2.001, 1.136), (-0.410, 2.612, 1.096))),
    Box.from_top_points("operator table A", ((-1.584, 0.462, 1.339), (-2.287, 0.456, 1.274), (-1.519, 2.116, 1.369))),
    Box.from_top_points("operator table B", ((-0.400, 2.756, 1.265), (-1.427, 1.421, 1.329), (-0.481, 1.562, 1.305))),
)


@register
class ObstaclesPlugin(HuskyPlugin):
    """Adds the fixed boxes to the shared scene and the 3D view."""

    name = "obstacles"

    def __init__(self):
        """Start with no boxes added."""
        self.boxes: tuple[Box, ...] = BOXES
        self._bodies: list[int] = []

    def setup(self, ctx: PluginContext) -> None:
        """Add every box to the shared PyBullet scene and to the 3D view.

        Args:
            ctx: This plugin's context.
        """
        for index, box in enumerate(self.boxes):
            body = pp.create_box(*box.size, color=tuple(c / 255 for c in BOX_COLOR) + (1.0,))
            pp.set_pose(body, (box.center, box.orientation))
            self._bodies.append(body)
            ctx.view.scene.add_box(f"{ctx.view.scene_root}/box_{index}", color=BOX_COLOR, dimensions=box.size,
                                   position=box.center, wxyz=yaw_to_wxyz(box.yaw))

    def teardown(self, ctx: PluginContext) -> None:
        """Remove the boxes from the shared PyBullet scene.

        Args:
            ctx: This plugin's context.
        """
        for body in self._bodies:
            pp.remove_body(body)
        self._bodies.clear()
