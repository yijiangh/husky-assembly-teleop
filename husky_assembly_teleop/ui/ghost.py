"""
See-through robot copies in a plugin's 3D view, for targets, planned paths and previews.

Build them in `setup` with `robot_ghosts` (slow), then `show` / `hide` them in every `draw`. Show a panel's
ghosts only while `RecentUse.active`: only the panel used last shows its ghosts.

! Main thread only.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Iterable, Mapping

import numpy as np

from .quaternion import quaternion_to_wxyz
from .visualization import FastViserUrdf, shared_urdf

if TYPE_CHECKING:
    from ..config import RobotConfig
    from ..plugin_api.context import PluginContext
    from bar_assembly_core.design_io.pose import Pose

#: Colours (r, g, b, alpha from 0 to 1) shared by the plugins.
TARGET_COLOR = (1.0, 0.78, 0.0, 0.35)   # yellow: where it should go
PATH_COLOR = (0.35, 0.6, 1.0, 0.35)     # blue: where the plan has it at the slider time
#: Colour per kind; the kind is also the scene subcategory.
KIND_COLORS = {"target": TARGET_COLOR, "path": PATH_COLOR}


class RecentUse:
    """Whether the operator used something recently and last of all; gates showing its ghosts."""

    #: The one touched last, across every plugin.
    _latest: RecentUse | None = None

    def __init__(self, timeout: float):
        """Start unused.

        Args:
            timeout: Seconds it counts as in use after `touch`; 0 or less: from then on.
        """
        self.timeout = timeout
        self._at: float | None = None

    def touch(self) -> None:
        """Note an input now; this becomes the one in use, and every other stops being."""
        self._at = time.monotonic()
        RecentUse._latest = self

    @property
    def active(self) -> bool:
        """bool: Whether this was touched last of all, within `timeout` seconds if one is set."""
        return (RecentUse._latest is self and self._at is not None
                and (self.timeout <= 0 or time.monotonic() - self._at < self.timeout))


class RobotGhost:
    """One see-through robot, hidden until shown."""

    def __init__(self, ctx: PluginContext, config: RobotConfig, kind: str):
        """Load the robot's meshes at "<plugin root>/ghosts/<kind>/<serial>".

        Args:
            ctx: The owning plugin's context; the ghost lives in its view.
            config: The robot; its stitched URDF is drawn.
            kind: A key of KIND_COLORS: "target" or "path". Sets the colour.
        """
        path = f"{ctx.view.scene_root}/ghosts/{kind}/{config.serial}"
        color = KIND_COLORS[kind]
        self._frame = ctx.view.scene.add_frame(path, show_axes=False, visible=False)
        self._urdf = FastViserUrdf(ctx.view, shared_urdf(config.urdf_file), root_node_name=path,
                                   mesh_color_override=color)
        self._names = self._urdf.get_actuated_joint_names()
        # What is shown now: (position, orientation, joint values), or None while hidden.
        self._shown: tuple | None = None
        # Link-name prefixes of the parts drawn, or None for the whole robot.
        self._parts: tuple[str, ...] | None = None

    def show(self, base: Pose, joints: Mapping[str, float], parts: tuple[str, ...] | None = None) -> None:
        """Show the robot with its base at `base` and these joint values (0 for any missing).

        Args:
            base: World pose of the robot's root link.
            joints: Values by joint name.
            parts: Draw only links whose name starts with one of these, e.g. ("left_ur_arm_",)
                for one arm and its tool; None for the whole robot.
        """
        if parts != self._parts:
            self._parts = parts
            # ? A mesh's name is its path of links from the root; it belongs to a part if any link on it does.
            for mesh in self._urdf._meshes:
                mesh.visible = parts is None or any(segment.startswith(parts) for segment in mesh.name.split("/"))
        values = tuple(float(joints.get(name, 0.0)) for name in self._names)
        shown = (base.position, base.orientation, values)
        if shown == self._shown:
            return
        self._shown = shown
        self._frame.position = tuple(float(v) for v in base.position)
        self._frame.wxyz = quaternion_to_wxyz(base.orientation)
        self._urdf.update_cfg(np.array(values))
        self._frame.visible = True

    def hide(self) -> None:
        """Hide the robot."""
        if self._shown is not None:
            self._shown = None
            self._frame.visible = False


def robot_ghosts(ctx: PluginContext, robots: Iterable[RobotConfig], kind: str) -> dict[str, RobotGhost]:
    """One ghost of a kind per robot, by serial. Call in `setup`: loading meshes is too slow for a tick.

    Args:
        ctx: The owning plugin's context.
        robots: The robots to build a ghost for.
        kind: A key of KIND_COLORS: "target" or "path".

    Returns:
        dict[str, RobotGhost]: The ghosts, hidden, by robot serial.
    """
    return {config.serial: RobotGhost(ctx, config, kind) for config in robots}
