"""
Example 2: reading robot configuration and state.

Every robot is in `ctx.world.robots`, keyed by serial. Reach its parts the
same way every time:

  robot.config               fixed for the run: serial, which arms, which tool.
                             Read it in setup, to decide what to build.
  robot.base.state           measured base: mocap pose and whether it is live.
  robot.arms[name].state     measured arm: joints, controller, tool, wrench.
  robot.arms[name].joint_vector()
                             the six joint angles in UR order, or None until
                             all six have arrived.

Measured state is written by ROS callbacks between ticks, so read it in
update or draw, every tick. Within one tick it does not change.

! Read only. A plugin never writes into state; only ROS callbacks do. To make
  a robot do something, call a method on the part (see robot_control).

Also shows reacting to a change: `update` remembers each arm's controller from
the last tick and logs when it changes -- the usual way to turn state into
events without callbacks.

Run with:  -p plugins:="['example_robot_state']" -p robots:="['0806']"
"""

from __future__ import annotations

import numpy as np
import viser

from ...context import PluginContext
from ...plugin import HuskyPlugin, register
from ...ui_style import NONE, OK, SECTION_SENSOR, block, chip, freshness_chip, numbers, section, values


@register
class ExampleRobotStatePlugin(HuskyPlugin):
    """One status block per robot, plus a log line whenever a controller changes."""

    name = "example_robot_state"

    def __init__(self):
        """No widgets yet; setup builds one per robot."""
        self._rows: dict[str, viser.GuiHtmlHandle] = {}
        #: Each arm's controller as seen last tick, keyed by (serial, arm name).
        self._last_controller: dict[tuple[str, str], str] = {}

    def setup(self, ctx: PluginContext) -> None:
        """Build one row per robot. Configuration decides what exists.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            if not ctx.world.robots:
                gui.add_html(block(chip("no robots", NONE, "start with -p robots:=[...]")))
            for serial, robot in ctx.world.robots.items():
                # * Configuration: fixed, so it can go into the section label once.
                arms = ", ".join(f"{name}:{arm.config.end_effector or '-'}" for name, arm in robot.arms.items())
                gui.add_html(section(serial, SECTION_SENSOR, arms))
                self._rows[serial] = gui.add_html("")

    def update(self, ctx: PluginContext) -> None:
        """Log controller changes. Compare with last tick; there is no callback for it.

        Args:
            ctx: This plugin's context.
        """
        for serial, robot in ctx.world.robots.items():
            for arm_name, arm in robot.arms.items():
                key = (serial, arm_name)
                active = arm.state.controllers.active
                previous = self._last_controller.get(key)
                if previous is not None and previous != active:
                    ctx.log_info(f"{serial} {arm_name}: controller {previous or 'none'} -> {active or 'none'}")
                self._last_controller[key] = active

    def draw(self, ctx: PluginContext) -> None:
        """Show each robot's measured state.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        for serial, robot in ctx.world.robots.items():
            # ! Base pose. Until mocap has sent one valid fix (`has_fix`),
            #   `position` is only a default, so show dashes. After that, keep
            #   showing the last pose but grey it out while mocap does not track
            #   the robot (`tracked` is False).
            base = robot.base.state
            pose = base.position if base.has_fix else None
            chips = freshness_chip("mocap", base.last_update_time, now)
            base_lines = values(f"base {numbers(pose, 3, 7, 3)} m", dim=not base.tracked)

            arm_lines = []
            for arm_name, arm in robot.arms.items():
                controller = arm.state.controllers.active
                chips += chip(f"{arm_name} {controller or 'no ctrl'}", OK if controller else NONE)
                joints = arm.joint_vector()  # None until all six joints have arrived
                arm_lines.append(arm_name)
                arm_lines.append(f"  q {numbers(None if joints is None else np.degrees(joints), 6, 6, 1)} °")

            # * Plain assignment: viser only sends it to the browser if it changed.
            self._rows[serial].content = block(chips + base_lines + values(*arm_lines))
