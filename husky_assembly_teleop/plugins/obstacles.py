"""
Fixed lab boxes (tables, cabinets), drawn in the 3D view and added to the shared PyBullet scene.

! Placeholder layout. Replace BOXES with the measured lab furniture.

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


#: The lab's static obstacles. Clear of the robots' default row along Y at x = 0.
BOXES = (
    Box("table 1", center=(3.0, 0.0, 0.375), size=(1.6, 0.8, 0.75), yaw=math.pi / 2),
    Box("table 2", center=(0.0, 4.0, 0.375), size=(1.6, 0.8, 0.75)),
    Box("cabinet", center=(-3.0, -1.5, 0.5), size=(0.6, 1.2, 1.0)),
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
