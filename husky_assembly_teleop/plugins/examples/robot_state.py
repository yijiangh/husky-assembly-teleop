"""
Example 2: reading robot configuration and state from `ctx.world.robots` (keyed by serial).

  robot.config               fixed for the run (serial, arms, tool); read it in setup.
  robot.base.state           measured base: mocap pose and whether it is live.
  robot.arms[name].state     measured arm: joints, controller, tool, wrench.
  robot.arms[name].joint_vector()
                             the six joint angles in UR order, or None until all arrive.

Measured state is written by ROS callbacks between ticks, so it is constant within
a tick. Also shows turning a state change into an event by comparing with last tick.

! Read only. To move a robot, call a method on the part (see robot_control).

Run with:  -p plugins:="['example_robot_state']" -p robots:="['0806']"
"""

from __future__ import annotations

import numpy as np
import viser

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...ui.style import NONE, OK, SECTION_SENSOR, block, chip, freshness_chip, numbers, section, values


@register
class ExampleRobotStatePlugin(HuskyPlugin):
    """One status block per robot, plus a log line whenever a controller changes."""

    name = "example_robot_state"

    def __init__(self):
        """Start with no widgets; setup builds one per robot."""
        self._rows: dict[str, viser.GuiHtmlHandle] = {}
        #: Each arm's controller last tick, keyed by (serial, arm name).
        self._last_controller: dict[tuple[str, str], str] = {}

    def setup(self, ctx: PluginContext) -> None:
        """Build one row per robot.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            if not ctx.world.robots:
                gui.add_html(block(chip("no robots", NONE, "start with -p robots:=[...]")))
            for serial, robot in ctx.world.robots.items():
                # * Config is fixed, so it goes into the section label once.
                arms = ", ".join(f"{name}:{arm.config.end_effector or '-'}" for name, arm in robot.arms.items())
                gui.add_html(section(serial, SECTION_SENSOR, arms))
                self._rows[serial] = gui.add_html("")

    def update(self, ctx: PluginContext) -> None:
        """Log controller changes by comparing with last tick.

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
            # ! Pose is None (dashes) until the first valid fix, then keeps the last, greyed while not `tracked`.
            base = robot.base.state
            pose = base.position
            chips = freshness_chip("mocap", base.last_update_time, now)
            base_lines = values(f"base {numbers(pose, 3, 7, 3)} m", dim=not base.tracked)

            arm_lines = []
            for arm_name, arm in robot.arms.items():
                controller = arm.state.controllers.active
                chips += chip(f"{arm_name} {controller or 'no ctrl'}", OK if controller else NONE)
                joints = arm.joint_vector()  # None until all six joints have arrived
                arm_lines.append(arm_name)
                arm_lines.append(f"  q {numbers(None if joints is None else np.degrees(joints), 6, 6, 1)} °")

            # * Viser sends the content only if it changed.
            self._rows[serial].content = block(chips + base_lines + values(*arm_lines))
