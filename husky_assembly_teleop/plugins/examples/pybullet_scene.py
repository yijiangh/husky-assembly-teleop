"""
Example 3: a plugin that adds a box to the shared PyBullet scene and checks it against the robots.

`ctx.scene` is a PyBullet client holding the live robots, posed from this tick's
measurements before any plugin runs. Each tick the plugin checks the box for
collision with every robot, reads each arm's tool0 pose by forward kinematics,
and mirrors both into the 3D view. Buttons move the box.

! PyBullet rules for plugins:
  - Call it only on the ROS thread (setup, update, draw, intents, jobs), never
    directly in a widget callback. There, `pp` already targets `ctx.scene`.
  - Remove in teardown whatever you add; nothing tracks it for you.
  - Every other plugin sees what you add (the box is an obstacle for them too).

* Look up link names once in setup, so a wrong name fails at startup, not every tick.

Run with:  -p plugins:="['example_pybullet']" -p robots:="['0806']"
"""

from __future__ import annotations

import numpy as np
import pybullet_planning as pp
import viser

from ...context import PluginContext
from ...plugin import HuskyPlugin, register
from ...ui_style import FAIL, NONE, OK, SECTION_CTRL, block, chip, numbers, section, values
from ...visualization import quaternion_to_wxyz

BOX_SIZE = (0.3, 0.3, 0.3)       # m
BOX_START = (1.5, 0.0, 0.15)     # m, standing on the floor
BOX_COLOR = (0.9, 0.5, 0.1)      # rgb, 0..1; viser and PyBullet both take this
BOX_STEP = 0.1                   # m per button press
MOVES = {"−x": (-1, 0), "+x": (1, 0), "−y": (0, -1), "+y": (0, 1)}


@register
class ExamplePybulletPlugin(HuskyPlugin):
    """A movable box checked for collision with the robots, plus the arms' tool frames."""

    name = "example_pybullet"

    def __init__(self):
        """Start empty; setup adds the box."""
        self._box: int | None = None
        self._box_position = np.array(BOX_START)
        self._colliding: list[str] = []
        #: Each arm's tool0 link, as (robot body id, link id), keyed by (serial, arm name).
        self._tool_links: dict[tuple[str, str], tuple[int, int]] = {}
        #: Each arm's tool0 pose from forward kinematics, same keys.
        self._tool_poses: dict[tuple[str, str], tuple] = {}
        self._tool_frames: dict[tuple[str, str], viser.FrameHandle] = {}

    def setup(self, ctx: PluginContext) -> None:
        """Add the box to PyBullet and to the 3D view, and build the buttons.

        Args:
            ctx: This plugin's context.

        Raises:
            ValueError: If a robot's URDF has no tool0 link for one of its arms.
        """
        self._box = pp.create_box(*BOX_SIZE, color=(*BOX_COLOR, 1.0))
        pp.set_pose(self._box, pp.Pose(point=self._box_position))
        for serial, body in ctx.scene.robots.items():
            for arm_name in ctx.world.robots[serial].arms:
                # URDF link name: arm name + "_tool0".
                link = pp.link_from_name(body, f"{arm_name}_tool0")
                self._tool_links[(serial, arm_name)] = (body, link)

        # * 3D view: nodes under ctx.view.scene_root are cleaned up for us.
        root = ctx.view.scene_root
        self._box_view = ctx.view.scene.add_box(f"{root}/box", color=BOX_COLOR,
                                                dimensions=BOX_SIZE, position=self._box_position)
        for serial, arm_name in self._tool_links:
            self._tool_frames[(serial, arm_name)] = ctx.view.scene.add_frame(
                f"{root}/{serial}/{arm_name}_tool0", axes_length=0.1, axes_radius=0.005)

        with ctx.view.ui() as gui:
            self._status: viser.GuiHtmlHandle = gui.add_html("")
            gui.add_html(section("box", SECTION_CTRL))
            move = gui.add_button_group("Move", list(MOVES))
            move.on_click(ctx.defer_value("move box", lambda clicked: self._move_box(MOVES[clicked])))

    def _move_box(self, direction: tuple[int, int]) -> None:
        """Move the box one step (runs as an intent, so PyBullet is safe here).

        Args:
            direction: Unit step in x and y.
        """
        self._box_position[:2] += np.array(direction) * BOX_STEP
        pp.set_pose(self._box, pp.Pose(point=self._box_position))

    def update(self, ctx: PluginContext) -> None:
        """Check the box for collision and read tool0 poses from this tick's robots.

        Args:
            ctx: This plugin's context.
        """
        self._colliding = [serial for serial, body in ctx.scene.robots.items()
                           if pp.pairwise_collision(self._box, body)]
        self._tool_poses = {key: pp.get_link_pose(body, link)
                            for key, (body, link) in self._tool_links.items()}

    def draw(self, ctx: PluginContext) -> None:
        """Update the 3D view and the status text.

        Args:
            ctx: This plugin's context.
        """
        self._box_view.position = self._box_position
        lines = [f"box           {numbers(self._box_position, 3, 6, 2)} m"]
        for (serial, arm_name), (position, orientation) in self._tool_poses.items():
            frame = self._tool_frames[(serial, arm_name)]
            frame.position = position
            # ! PyBullet quaternions are xyzw, viser wants wxyz.
            frame.wxyz = quaternion_to_wxyz(orientation)
            lines.append(f"{serial} {arm_name:<8} {numbers(position, 3, 6, 2)} m")
        if not ctx.scene.robots:
            state = chip("no robots", NONE)
        elif self._colliding:
            state = chip("collision " + " ".join(self._colliding), FAIL)
        else:
            state = chip("clear", OK)
        self._status.content = block(state + values(*lines))

    def teardown(self, ctx: PluginContext) -> None:
        """Remove the box from PyBullet (the 3D view and widgets clean up themselves).

        May run after a partial setup, so check the box exists.

        Args:
            ctx: This plugin's context.
        """
        if self._box is not None:
            pp.remove_body(self._box)
            self._box = None
