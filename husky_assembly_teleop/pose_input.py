"""
A floor-pose input: number fields in the panel plus a 3D gizmo, always showing
the same value. Build one in `setup`, read `pose` when needed.

! Planar only (x, y, yaw): the gizmo is flattened to the floor. Make a separate
  6-DOF input if height or tilt is needed.

! Threading: widget callbacks run on viser threads and only queue the new value
  (ctx.submit). `pose` is written on the main thread only, so reads need no lock.
"""

from __future__ import annotations

import math
from typing import Callable

import numpy as np
import viser

from .context import PluginContext

#: Poses closer than this (metres, radians) count as equal, so a value written
#: into one widget does not bounce back through the other.
SAME_POSE_TOLERANCE = 1e-3


def yaw_to_wxyz(yaw: float) -> tuple[float, float, float, float]:
    """Convert a yaw to a (w, x, y, z) quaternion.

    Args:
        yaw: Angle about Z, radians.

    Returns:
        tuple[float, float, float, float]: The rotation as (w, x, y, z).
    """
    return (math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def yaw_from_xyzw(quaternion) -> float:
    """Return the yaw (turn about Z) of an (x, y, z, w) quaternion, ignoring tilt.

    Args:
        quaternion: Orientation as (x, y, z, w).

    Returns:
        float: Yaw in (-pi, pi].
    """
    x, y, z, w = (float(value) for value in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PlanarPoseInput:
    """x, y, yaw number fields and an optional drag gizmo, kept in sync.

    Attributes:
        pose: Current (x, y, yaw), metres and radians, world frame.
    """

    def __init__(self, ctx: PluginContext, gui: viser.GuiApi, scene_path: str, label: str = "Target",
                 on_change: Callable[[], None] | None = None, gizmo_scale: float = 0.5):
        """Build the fields and a gizmo that stays hidden until "Gizmo" is ticked.

        Call inside `ctx.view.ui()` so the fields land in the plugin's folder.

        Args:
            ctx: The owning plugin's context.
            gui: GUI api to add the fields to.
            scene_path: Gizmo path; must be under `ctx.view.scene_root`.
            label: Label of the number row.
            on_change: Called on the main thread when the operator edits the
                pose. Not called for `set`.
            gizmo_scale: Gizmo size, metres.
        """
        self._ctx = ctx
        self._on_change = on_change
        self.pose = (0.0, 0.0, 0.0)

        # * One row for all three numbers; yaw is typed in degrees.
        self._fields = gui.add_vector3(f"{label} x y yaw°", initial_value=(0.0, 0.0, 0.0), step=0.01,
                                       hint="Target pose on the floor: x and y in metres, yaw in degrees")
        self._show = gui.add_checkbox("Gizmo", initial_value=False,
                                      hint="Show a handle in the 3D view to drag the target with")

        # * Floor axes only (x, y, and the ring about Z); no height or tilt.
        self._gizmo = ctx.view.scene.add_transform_controls(
            scene_path, scale=gizmo_scale, active_axes=(True, True, False),
            disable_sliders=False, depth_test=False, visible=False)

        # ! Callbacks only queue the value; it is applied on the main thread.
        self._fields.on_update(ctx.defer_value(f"{label} typed", self._on_fields))
        self._show.on_update(ctx.defer_value(f"{label} gizmo shown", self._on_show))

        async def on_drag(event: viser.TransformControlsEvent) -> None:
            position, wxyz = tuple(event.target.position), tuple(event.target.wxyz)
            ctx.submit(f"{label} dragged", lambda: self._on_drag(position, wxyz))

        self._gizmo.on_update(on_drag)

    # --- --- --- --- --- WHAT THE PLUGIN CALLS --- --- --- --- ---

    def set(self, x: float, y: float, yaw: float) -> None:
        """Set the pose from code, without calling `on_change`. main thread only.

        Args:
            x: Metres, world frame.
            y: Metres, world frame.
            yaw: Radians about Z.
        """
        self._apply((float(x), float(y), _wrap(float(yaw))), notify=False)

    # --- --- --- --- --- CALLBACKS (intents, on the main thread) --- --- --- --- ---

    def _on_fields(self, value) -> None:
        """Apply a number-row edit (yaw in degrees)."""
        x, y, yaw_degrees = value
        self._apply((float(x), float(y), _wrap(math.radians(float(yaw_degrees)))), notify=True)

    def _on_drag(self, position, wxyz) -> None:
        """Apply a gizmo drag, flattened to the floor."""
        w, x, y, z = wxyz
        self._apply((float(position[0]), float(position[1]), yaw_from_xyzw((x, y, z, w))), notify=True)

    def _on_show(self, shown) -> None:
        """Show or hide the gizmo."""
        self._gizmo.visible = bool(shown)

    def _apply(self, pose: tuple[float, float, float], notify: bool) -> None:
        """Store a pose and copy it into both widgets.

        ! Writing a widget queues its callback back here; the tolerance check
          below ends that loop.

        Args:
            pose: (x, y, yaw), metres and radians.
            notify: Call `on_change` (True for operator edits).
        """
        if np.allclose(pose, self.pose, atol=SAME_POSE_TOLERANCE):
            return
        self.pose = pose
        x, y, yaw = pose
        self._fields.value = (round(x, 4), round(y, 4), round(math.degrees(yaw), 2))
        # Gizmo height is left as it was.
        self._gizmo.position = (x, y, float(self._gizmo.position[2]))
        self._gizmo.wxyz = yaw_to_wxyz(yaw)
        if notify and self._on_change is not None:
            self._on_change()


def _wrap(angle: float) -> float:
    """Fold an angle into (-pi, pi].

    Args:
        angle: Radians.

    Returns:
        float: The wrapped angle.
    """
    return math.atan2(math.sin(angle), math.cos(angle))
