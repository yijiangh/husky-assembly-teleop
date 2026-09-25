"""
Example 3: using the shared PyBullet scene.

`ctx.scene` is a PyBullet client holding the live robots, posed from their
measurements before any plugin runs each tick. So forward kinematics and
collision checks against it answer for *this* tick's reality.

What this plugin does:
  - adds one box of its own to the scene, and removes it in teardown
  - lets the operator move the box with buttons
  - every tick, checks the box against every robot for collision, and reads
    each arm's tool0 pose by forward kinematics
  - mirrors the box and the tool frames into the 3D view (viser)

! Three rules for PyBullet in a plugin:
  1. Only on the ROS thread: in setup, update, draw, an intent or a job --
     never directly in a widget callback. There, `pp` already talks to
     `ctx.scene`: the monitor points it there before every hook.
  2. Whatever you add, you remove in teardown. Nothing tracks it for you.
  3. What you add, every other plugin sees. The box below is an obstacle in
     their collision checks too. Add only what really belongs in the world.

* Look names up once, in setup. A link name that is not in the URDF then
  stops this plugin at startup with one clear error, instead of failing every
  tick.

Run with:  -p plugins:="['example_pybullet']" -p robots:="['0806']"
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
import pybullet_planning as pp

from ...context import PluginContext
from ...plugin import HuskyPlugin, register
from ...ui_style import FAIL, NONE, OK, SECTION_CTRL, block, chip, numbers, section, values
from ...visualization import quaternion_to_wxyz

if TYPE_CHECKING:
    import viser

BOX_SIZE = (0.3, 0.3, 0.3)       # m
BOX_START = (1.5, 0.0, 0.15)     # m, standing on the floor
BOX_COLOR = (0.9, 0.5, 0.1)      # rgb, 0..1; viser and PyBullet both take this
BOX_STEP = 0.1                   # m per button press
MOVES = {"−x": (-1, 0), "+x": (1, 0), "−y": (0, -1), "+y": (0, 1)}


@register
class ExamplePybulletPlugin(HuskyPlugin):
    """A movable box, checked for collision with the robots, and the arms' tool frames."""

    name = "example_pybullet"

    def __init__(self):
        """Nothing in the scene yet; setup adds the box."""
        self._box: int | None = None
        self._box_position = np.array(BOX_START)
        self._colliding: list[str] = []
        #: Each arm's tool0 link, as (robot body id, link id), keyed by (serial, arm name).
        self._tool_links: dict[tuple[str, str], tuple[int, int]] = {}
        #: Each arm's tool0 pose from forward kinematics, same keys.
        self._tool_poses: dict[tuple[str, str], tuple] = {}
        self._tool_frames: dict[tuple[str, str], "viser.FrameHandle"] = {}

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
                # Link names come from the URDF: the arm's name + "_tool0".
                link = pp.link_from_name(body, f"{arm_name}_tool0")
                self._tool_links[(serial, arm_name)] = (body, link)

        # * 3D view: everything under ctx.view.scene_root, which is removed for
        #   us on teardown. Only PyBullet needs manual cleanup.
        root = ctx.view.scene_root
        self._box_view = ctx.view.scene.add_box(f"{root}/box", color=BOX_COLOR,
                                                dimensions=BOX_SIZE, position=self._box_position)
        for serial, arm_name in self._tool_links:
            self._tool_frames[(serial, arm_name)] = ctx.view.scene.add_frame(
                f"{root}/{serial}/{arm_name}_tool0", axes_length=0.1, axes_radius=0.005)

        with ctx.view.ui() as gui:
            self._status: "viser.GuiHtmlHandle" = gui.add_html("")
            gui.add_html(section("box", SECTION_CTRL))
            move = gui.add_button_group("Move", list(MOVES))
            move.on_click(ctx.defer_value("move box", lambda clicked: self._move_box(MOVES[clicked])))

    def _move_box(self, direction: tuple[int, int]) -> None:
        """Move the box one step. An intent, so PyBullet is safe to touch here.

        Args:
            direction: Unit step in x and y.
        """
        self._box_position[:2] += np.array(direction) * BOX_STEP
        pp.set_pose(self._box, pp.Pose(point=self._box_position))

    def update(self, ctx: PluginContext) -> None:
        """Collision and forward kinematics against this tick's measured robots.

        Args:
            ctx: This plugin's context.
        """
        self._colliding = [serial for serial, body in ctx.scene.robots.items()
                           if pp.pairwise_collision(self._box, body)]
        self._tool_poses = {key: pp.get_link_pose(body, link)
                            for key, (body, link) in self._tool_links.items()}

    def draw(self, ctx: PluginContext) -> None:
        """Mirror the box and tool frames into the 3D view, and show the numbers.

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
        """Remove the box from PyBullet. The 3D view and widgets go by themselves.

        Also runs when setup failed partway, so check what actually exists.

        Args:
            ctx: This plugin's context.
        """
        if self._box is not None:
            pp.remove_body(self._box)
            self._box = None
