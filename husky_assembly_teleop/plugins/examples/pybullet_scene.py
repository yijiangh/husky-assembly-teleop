"""
Example 3: put a box in the scene and check the robots for collision in our own PyBullet world.

The pattern every planner follows:
  1. Put a body in `ctx.scene`; the core draws it and removes it when the plugin closes.
     Buttons move it in place (`body.placement = Pose(...)`).
  2. Read link poses from `ctx.kinematics`: here each arm's tool0, drawn as a frame.
  3. Each tick a task hands `ctx.scene.snapshot` to one worker thread, which syncs its
     own `PyBulletMirror` from it and asks which objects each robot touches.

! Rules for plugins:
  - Collision objects go in `ctx.scene` as `Body`s under "<plugin name>/…", never straight into PyBullet.
  - Use `p` / `pp` only inside your own mirror, created, synced, queried and closed on one worker thread.
  - Report results with our ids ("example_pybullet/box", "robots/0806"), never PyBullet body ids:
    those exist only inside one mirror and get reused.

Run with:  -p plugins:="['example_pybullet']" -p robots:="['0806']"
"""

from __future__ import annotations

import asyncio
from concurrent.futures import ThreadPoolExecutor

import numpy as np
import viser

from ...plugin_api.context import PluginContext
from bar_assembly_core.design_io.geometry import box_geometry
from bar_assembly_core.mirrors.pybullet import PyBulletMirror
from ...plugin_api.plugin import HuskyPlugin, register
from bar_assembly_core.scene import Body, SceneSnapshot
from bar_assembly_core.design_io.pose import Pose
from ...ui.style import FAIL, NONE, OK, SECTION_CTRL, block, chip, numbers, section, values
from ...ui.quaternion import quaternion_to_wxyz

BOX_ID = "example_pybullet/box"  # ! must start with the plugin's name
BOX_SIZE = (0.3, 0.3, 0.3)       # m
BOX_START = (1.5, 0.0, 0.15)     # m, standing on the floor
BOX_COLOR = (0.9, 0.5, 0.1)      # rgb, 0..1
BOX_STEP = 0.1                   # m per button press
MOVES = {"−x": (-1, 0), "+x": (1, 0), "−y": (0, -1), "+y": (0, 1)}


@register
class ExamplePybulletPlugin(HuskyPlugin):
    """A movable box in the scene, a collision check on a worker thread, plus the arms' tool frames."""

    name = "example_pybullet"

    def __init__(self):
        """Start empty; setup puts the box in the scene and starts the checks."""
        #: One worker thread, the only one that touches the mirror.
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="example-pybullet")
        #: Our own PyBullet world, created on the worker at the first check.
        self._mirror: PyBulletMirror | None = None
        self._check_task: asyncio.Task | None = None
        #: Latest result: ids each robot collides with, by serial, and the snapshot's tick.
        self._collisions: dict[str, list[str]] = {}
        self._checked_tick = -1
        #: Each arm's tool0 world pose, keyed by (serial, arm name).
        self._tool_poses: dict[tuple[str, str], Pose] = {}
        self._tool_frames: dict[tuple[str, str], viser.FrameHandle] = {}

    def setup(self, ctx: PluginContext) -> None:
        """Put the box in the scene, look up the tool0 links, build the view and start the checks.

        Args:
            ctx: This plugin's context.

        Raises:
            KeyError: If a robot's URDF has no tool0 link for one of its arms.
        """
        ctx.scene.put(Body(id=BOX_ID, geometry=box_geometry(BOX_SIZE), placement=Pose(BOX_START),
                           color=(*BOX_COLOR, 1.0), label="example box"))

        # * 3D view: only our own extras; nodes under ctx.view.scene_root are cleaned up for us.
        root = ctx.view.scene_root
        for serial, robot in ctx.world.robots.items():
            for arm_name in robot.arms:
                # * Look up link names once here, so a wrong name fails setup, not every tick.
                ctx.kinematics.link_pose(serial, f"{arm_name}_tool0")
                self._tool_frames[(serial, arm_name)] = ctx.view.scene.add_frame(
                    f"{root}/{serial}/{arm_name}_tool0", axes_length=0.1, axes_radius=0.005)

        with ctx.view.ui() as gui:
            self._status: viser.GuiHtmlHandle = gui.add_html("")
            gui.add_html(section("box", SECTION_CTRL))
            move = gui.add_button_group("Move", list(MOVES))
            move.on_click(ctx.defer_value("move box", lambda clicked: self._move_box(ctx, MOVES[clicked])))

        self._check_task = ctx.spawn("collision checks", self._check_loop(ctx))

    def _move_box(self, ctx: PluginContext, direction: tuple[int, int]) -> None:
        """Move the box one step, in place (intent, on the main thread).

        Args:
            ctx: This plugin's context.
            direction: Unit step in x and y.
        """
        body = ctx.scene.bodies[BOX_ID]
        position = np.array(body.placement.position)
        position[:2] += np.array(direction) * BOX_STEP
        # * The next snapshot has the new pose; the core view and our mirror follow.
        body.placement = Pose.from_arrays(position, body.placement.orientation)

    async def _check_loop(self, ctx: PluginContext) -> None:
        """Every tick, check this tick's snapshot on the worker and keep the result.

        ! Cancelling this task abandons the await, but a check already on the worker runs to the end.

        Args:
            ctx: This plugin's context.
        """
        loop = asyncio.get_running_loop()
        while True:
            snapshot = ctx.scene.snapshot  # main thread
            self._collisions = await loop.run_in_executor(self._executor, self._check, snapshot)
            self._checked_tick = snapshot.tick
            await ctx.next_tick()

    def _check(self, snapshot: SceneSnapshot) -> dict[str, list[str]]:
        """Sync the mirror to a snapshot and list what each robot collides with (worker thread).

        Args:
            snapshot: The world to check. Read-only.

        Returns:
            dict[str, list[str]]: Our ids of the objects each robot collides with, by serial.
        """
        if self._mirror is None:
            self._mirror = PyBulletMirror()  # ! on the worker: the mirror belongs to this thread
        self._mirror.sync(snapshot)
        return {serial: self._mirror.collisions(serial) for serial in snapshot.robots}

    def update(self, ctx: PluginContext) -> None:
        """Read the tool0 poses, and restart the checks after a soft stop cancelled them.

        Args:
            ctx: This plugin's context.
        """
        self._tool_poses = {(serial, arm_name): ctx.kinematics.link_pose(serial, f"{arm_name}_tool0")
                            for serial, arm_name in self._tool_frames}
        # ? Soft stop cancels every task; a failed task stays down.
        if self._check_task is not None and self._check_task.cancelled():
            self._check_task = ctx.spawn("collision checks", self._check_loop(ctx))

    def draw(self, ctx: PluginContext) -> None:
        """Move the tool frames and update the status text.

        Args:
            ctx: This plugin's context.
        """
        box_position = ctx.scene.bodies[BOX_ID].placement.position
        lines = [f"box           {numbers(box_position, 3, 6, 2)} m"]
        for (serial, arm_name), pose in self._tool_poses.items():
            frame = self._tool_frames[(serial, arm_name)]
            frame.position = pose.position
            # ! Pose quaternions are xyzw, viser wants wxyz.
            frame.wxyz = quaternion_to_wxyz(pose.orientation)
            lines.append(f"{serial} {arm_name:<8} {numbers(pose.position, 3, 6, 2)} m")
        hits = [f"{serial}: {', '.join(ids)}" for serial, ids in self._collisions.items() if ids]
        if not ctx.world.robots:
            state = chip("no robots", NONE)
        elif hits:
            state = chip("collision", FAIL)
            lines.extend(hits)
        else:
            state = chip("clear", OK)
        lines.append(f"checked tick  {self._checked_tick}")
        self._status.content = block(state + values(*lines))

    def teardown(self, ctx: PluginContext) -> None:
        """Close the mirror on its worker thread, then stop the worker. May run after a partial setup.

        Args:
            ctx: This plugin's context.
        """
        # ? Queued after any running check, so the mirror closes last, on its own thread.
        self._executor.submit(self._close_mirror)
        self._executor.shutdown(wait=True)

    def _close_mirror(self) -> None:
        """Close the mirror if a check created one (worker thread).

        ? Checked here, not in teardown: a running check may create it first.
        """
        if self._mirror is not None:
            self._mirror.close()
            self._mirror = None
