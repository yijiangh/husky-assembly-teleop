"""
Validate in the scene: the samples, the validation poses' targets at the robot, and where the base stood per spot.

Each drawing is redone only when what it shows changed; the targets hang on the base pose, so they move with the
robot while it drives to a spot.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import trimesh

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.kinematics import link_pose

from ....ui.quaternion import quaternion_to_wxyz
from ....world.checks import BAD, GOOD, WARN
from ..analysis.checks import validation_level
from ..robot.sweeps import named
from .takes_view import MEASURED_OPACITY, MEASURED_RADIUS, place_on

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext
    from ..run import Run, Validation

#: Validation dots by level: green, amber, red; a target not sampled yet grey.
LEVEL_RGB = {GOOD: (40, 160, 70), WARN: (230, 150, 30), BAD: (220, 60, 60)}
PENDING_RGB = (134, 142, 150)
#: A target's axes (length, radius) and sphere radius, metres.
TARGET_AXES, TARGET_RADIUS = (0.06, 0.004), 0.012
#: A spot's base_footprint axes: length, radius, metres.
SPOT_AXES = (0.3, 0.01)


class ValidationView:
    """Draws Validate's samples, targets and spots.

    Attributes:
        targets_drawn: The run and colours the targets were last drawn with; None while hidden.
    """

    def __init__(self):
        """Nothing drawn."""
        self.targets_drawn: tuple | None = None
        self._dots_drawn: tuple | None = None
        #: Each validation pose's tool0 in base_footprint, for the run they were computed for.
        self._targets: tuple[Run, list[Pose]] | None = None
        self._targets_frame = None
        self._targets_placed: tuple | None = None
        self._spots_drawn: tuple | None = None
        self._spots_frame = None

    def hide(self, ctx: PluginContext) -> None:
        """Hide the dots, targets and spots."""
        self.draw_dots(ctx, None)
        self.draw_targets(ctx, None, [])
        self.draw_spots(ctx, [])

    def draw_spots(self, ctx: PluginContext, spots: list[Pose]) -> None:
        """Where the base stood for each spot sampled, the spot in progress last: its base_footprint, numbered."""
        key = tuple((tuple(np.round(pose.position, 3)), tuple(np.round(pose.orientation, 3))) for pose in spots)
        if key == self._spots_drawn:
            return
        self._spots_drawn = key
        root = f"{ctx.view.scene_root}/validation_spots"
        if self._spots_frame is not None:
            # * Drops the old spots with their frame.
            self._spots_frame.remove()
        self._spots_frame = ctx.view.scene.add_frame(root, show_axes=False, visible=bool(spots))
        for index, pose in enumerate(spots):
            ctx.view.scene.add_frame(f"{root}/{index}", axes_length=SPOT_AXES[0], axes_radius=SPOT_AXES[1],
                                     position=pose.position, wxyz=quaternion_to_wxyz(pose.orientation))
            ctx.view.scene.add_label(f"{root}/{index}/name", f"spot {index + 1}", position=(0.0, 0.0, 0.05))

    def draw_targets(self, ctx: PluginContext, run: Run | None, colors: list[tuple[int, int, int]]) -> None:
        """Where the arm will sample: each validation pose's tool0 at the robot, numbered; none for no colours."""
        root = f"{ctx.view.scene_root}/validation_targets"
        if run is not None and colors and (self._targets is None or self._targets[0] is not run):
            self._targets = (run, [link_pose(run.urdf, Pose(), named(run.arm, pose), f"{run.arm}_tool0")
                                   for pose in run.validation_poses])
            self.targets_drawn = self._targets_placed = None
        shown = bool(colors) and run is not None
        if self._targets_frame is None:
            self._targets_frame = ctx.view.scene.add_frame(root, show_axes=False, visible=False)
        self._targets_placed = place_on(self._targets_frame, ctx.kinematics.base_pose(run.serial) if shown else None,
                                        self._targets_placed)
        key = (id(run), tuple(colors)) if shown else None
        if not shown or key == self.targets_drawn:
            return
        self.targets_drawn = key
        for index, (pose, rgb) in enumerate(zip(self._targets[1], colors)):
            ctx.view.scene.add_frame(f"{root}/{index}", axes_length=TARGET_AXES[0], axes_radius=TARGET_AXES[1],
                                     position=pose.position, wxyz=quaternion_to_wxyz(pose.orientation))
            ctx.view.scene.add_icosphere(f"{root}/{index}/dot", radius=TARGET_RADIUS, color=rgb, subdivisions=2,
                                         cast_shadow=False, receive_shadow=False)
            ctx.view.scene.add_label(f"{root}/{index}/number", str(index + 1), position=(0.0, 0.0, 0.03))

    def draw_dots(self, ctx: PluginContext, validation: Validation | None) -> None:
        """A dot where the tool body was at each sample, coloured by its distance from the mean; none for None."""
        name = f"{ctx.view.scene_root}/validation"
        shown = validation is not None and bool(validation.tools)
        key = (shown, len(validation.tools) if validation else 0, id(validation))
        if key == self._dots_drawn:
            return
        self._dots_drawn = key
        if not shown:
            ctx.view.scene.add_frame(name, show_axes=False, visible=False)
            return
        points = np.array([tool.position for tool in validation.tools])
        distances = validation.spread[0] if validation.spread is not None else np.zeros(len(points))
        colors = np.array([LEVEL_RGB[validation_level(distance)] for distance in distances[:len(points)]])
        sphere = trimesh.creation.icosphere(subdivisions=1, radius=MEASURED_RADIUS)
        ctx.view.scene.add_batched_meshes_simple(
            name, sphere.vertices, sphere.faces, batched_wxyzs=np.tile((1.0, 0.0, 0.0, 0.0), (len(points), 1)),
            batched_positions=points, batched_colors=colors, batched_opacities=np.full(len(points), MEASURED_OPACITY),
            cast_shadow=False, receive_shadow=False)
