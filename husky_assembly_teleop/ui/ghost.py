"""
A see-through copy of a robot in a plugin's 3D view, posed by hand: for targets, planned paths and previews.

* Build ghosts in `setup` with `robot_ghosts` (loading the meshes takes a moment), then
  `show` or `hide` them in `draw`. `show` does nothing when nothing changed, so it can run every tick.
* Every ghost lives at "<plugin root>/ghosts/<kind>/<serial>", so the debug view shows each
  plugin's ghosts as one "ghosts" category with the same subcategories everywhere.
* Context: show a plugin's ghosts only while it is in use (`RecentUse`): after the
  operator's input in it, for `MonitorConfig.ghost_timeout` seconds (0: from then on).
  Only the last one used anywhere is active, so ghosts of one panel show at a time:
  editing the base target hides the arm's ghosts, scrubbing the arm path brings them back.
! Main thread only.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Iterable, Mapping

import numpy as np
import viser.extras

from .quaternion import quaternion_to_wxyz
from .visualization import load_urdf

if TYPE_CHECKING:
    from ..config import RobotConfig
    from ..plugin_api.context import PluginContext
    from ..world.scene import Pose

#: Colours (r, g, b, alpha from 0 to 1) shared by the plugins, so a target and a path look the same everywhere.
TARGET_COLOR = (1.0, 0.78, 0.0, 0.35)   # yellow: where it should go
PATH_COLOR = (0.35, 0.6, 1.0, 0.35)     # blue: where the plan has it at the slider time
#: The kinds of ghost, by scene subcategory, with their colours.
KIND_COLORS = {"target": TARGET_COLOR, "path": PATH_COLOR}


class RecentUse:
    """Whether the operator used something recently, and last of all: for showing its ghosts only then."""

    #: The one touched last, across every plugin. ? Main thread only, like everything here.
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
        # The plugin's view is passed so the ghost stays in the plugin's subtree.
        self._urdf = viser.extras.ViserUrdf(ctx.view, load_urdf(config.urdf_file), root_node_name=path,
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
            # ? viser names each mesh by its path of links from the root, so a mesh belongs
            #   to a part if any link on its path does. ViserUrdf keeps its meshes in `_meshes`.
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
    """One ghost of a kind per robot, by serial. Call in `setup`: loading the meshes is too slow for the tick.

    Args:
        ctx: The owning plugin's context.
        robots: The robots to build a ghost for.
        kind: A key of KIND_COLORS: "target" or "path".

    Returns:
        dict[str, RobotGhost]: The ghosts, hidden, by robot serial.
    """
    return {config.serial: RobotGhost(ctx, config, kind) for config in robots}
