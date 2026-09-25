"""
The health panel: one glance to see whether every robot and tracked object is fine.

Each robot and each tracked object gets one row of chips, one chip per thing
that can go wrong. Green means fine, amber means it works but look at it, red
means it is broken or silent. Hover a chip for the detail. The banner on top is
green only when every chip is.

What is checked, and why it matters:

  mocap          the base pose is only ever from mocap. Silent relay, body lost
                 by NatNet (hidden markers), or a pose the relay marked invalid
                 are red; a high marker error is amber.
  <part> ctrl    the base's and each arm's controller manager. Never answered or
                 stopped answering is red (driver down, or wifi); no controller
                 running or a failed switch is amber, as commands will refuse.
  <arm> joints   joint_states from the arm's rate limiter. Missing or old is red.
  <arm> tool     robotiq: its action client has not connected to the server.
                 scaffolding_v3: tool_status missing or old.
  objects        every entry in world.tracked_objects: whether it is tracked
                 and how old its last sample is.

! Read only. Every check is a function of measured state and the clock, so it
  never commands anything and never blocks. The checks are pure functions below
  the plugin, which keeps them easy to read and to test on their own.

Loaded by default (config.DEFAULT_PLUGINS); --no-default turns that off.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from html import escape

import viser

from ..context import PluginContext
from ..plugin import HuskyPlugin, register
from ..robot_interface import ArmInterface, HuskyRobotInterface, RobotiqGripper, ScaffoldingV1, ScaffoldingV3
from ..robot_interface.base import BaseState
from ..robot_interface.controller_manager import REFRESH_PERIOD, ControllerManagerState
from ..ui_style import BUSY, FAIL, OK, STALE_AFTER, block, chip, section
from ..world_state import TrackedObject

# --- --- --- --- --- THRESHOLDS --- --- --- --- ---

#: Mean marker error, metres, above which the mocap chip turns amber.
#: ? A guess from typical OptiTrack numbers (well under a millimetre when the
#:   rigid body is healthy). The relay has its own, much looser, threshold
#:   (`marker_error_valid_threshold`) past which it marks the pose invalid.
MARKER_ERROR_WARN = 1e-3
#: Seconds without a list_controllers answer before a controller manager counts
#: as gone. It is polled every REFRESH_PERIOD, so this allows two missed polls.
CONTROLLER_STALE_AFTER = 3 * REFRESH_PERIOD

# Severity levels. Ordered, so the worst of several is simply `max`.
GOOD, WARN, BAD = 0, 1, 2
#: Chip colour for each severity.
LEVEL_COLORS = {GOOD: OK, WARN: BUSY, BAD: FAIL}


@dataclass(frozen=True)
class Check:
    """The outcome of one health check, shown as one chip.

    Attributes:
        label: Short chip text, e.g. "mocap" or "left_ur_arm joints".
        level: GOOD, WARN or BAD.
        detail: What is wrong, or a short fact when all is well. Shown as the
            chip's tooltip.
    """

    label: str
    level: int
    detail: str = ""


@register
class HealthPlugin(HuskyPlugin):
    """One panel listing every robot and tracked object, green when all is well."""

    name = "health"

    def __init__(self):
        """No widgets yet; setup builds them."""
        self._panel: viser.GuiHtmlHandle | None = None
        #: This tick's checks, as (row title, checks) pairs, in display order.
        self._rows: list[tuple[str, list[Check]]] = []

    def setup(self, ctx: PluginContext) -> None:
        """Build the single HTML widget the whole panel is drawn into.

        ? One widget rather than one per robot, because tracked objects can
          appear while running and a single block of HTML handles that for free.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            self._panel = gui.add_html("")

    def update(self, ctx: PluginContext) -> None:
        """Run every check against this tick's state.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        rows = [(serial, robot_checks(robot, now)) for serial, robot in ctx.world.robots.items()]
        if ctx.world.tracked_objects:
            rows.append(("objects", [object_check(obj, now)
                                     for obj in ctx.world.tracked_objects.values()]))
        self._rows = rows

    def draw(self, ctx: PluginContext) -> None:
        """Render the banner and one section per row.

        Args:
            ctx: This plugin's context.
        """
        self._panel.content = render(self._rows)


# --- --- --- --- --- CHECKS --- --- --- --- ---
# Pure functions of state and the clock. `now` is ROS time, seconds.

def robot_checks(robot: HuskyRobotInterface, now: float) -> list[Check]:
    """Every check for one robot: mocap, base controllers, then each arm.

    Args:
        robot: The robot to check.
        now: Current ROS time, seconds.

    Returns:
        list[Check]: One per chip, in display order.
    """
    checks = [mocap_check(robot.config.mocap_id, robot.base.state, now),
              controller_check("base ctrl", robot.base.state.controllers, now)]
    for arm_name, arm in robot.arms.items():
        checks.append(controller_check(f"{arm_name} ctrl", arm.state.controllers, now))
        checks.append(age_check(f"{arm_name} joints", "joint_states", arm.state.last_update_time, now))
        tool = tool_check(arm, now)
        if tool is not None:
            checks.append(tool)
    return checks


def mocap_check(mocap_id: int | None, state: BaseState, now: float) -> Check:
    """Whether the base pose is live, and how good the fix is.

    Args:
        mocap_id: The base's rigid-body id, or None if it has none configured.
        state: The base's measured state.
        now: Current ROS time, seconds.

    Returns:
        Check: The mocap chip. Its label carries the marker error once known.
    """
    if mocap_id is None:
        return Check("mocap", WARN, "no mocap id configured; the base pose is never measured")
    if state.last_update_time == 0.0:
        return Check("mocap", BAD, f"no message for rigid body {mocap_id} yet; is mocap_relay running?")
    age = now - state.last_update_time
    if age > STALE_AFTER:
        return Check("mocap", BAD, f"relay silent for {age:.0f}s (rigid body {mocap_id})")

    error_mm = state.marker_error * 1e3
    # ! The relay sends an infinite error for a body it has no sample of.
    label = f"mocap {error_mm:.2f}mm" if math.isfinite(error_mm) else "mocap"
    # * Why the relay marked it invalid, in the order it decides: NatNet lost
    #   the body, or the marker error is past the relay's threshold, or the
    #   relay's own cache went stale.
    if not state.tracking_valid:
        return Check(label, BAD, f"rigid body {mocap_id} lost by mocap; markers hidden?")
    if not state.tracked:
        return Check(label, BAD, f"relay marks the pose invalid (marker error {error_mm:.2f} mm)")
    if state.marker_error > MARKER_ERROR_WARN:
        return Check(label, WARN, f"marker error {error_mm:.2f} mm is high; "
                                  f"check the markers and the rigid body definition")
    return Check(label, GOOD, f"rigid body {mocap_id}, marker error {error_mm:.2f} mm")


def controller_check(label: str, state: ControllerManagerState, now: float) -> Check:
    """Whether a controller manager answers, and a controller is running.

    Args:
        label: Chip text, e.g. "base ctrl".
        state: That controller manager's state.
        now: Current ROS time, seconds.

    Returns:
        Check: The controller manager's chip.
    """
    if state.last_update_time == 0.0:
        return Check(label, BAD, "controller manager never answered; driver down or not discovered")
    age = now - state.last_update_time
    if age > CONTROLLER_STALE_AFTER:
        return Check(label, BAD, f"controller manager silent for {age:.0f}s")
    if state.switch_error:
        return Check(label, WARN, state.switch_error)
    if not state.active:
        return Check(label, WARN, "no controller active; commands will be refused")
    return Check(label, GOOD, state.active)


def age_check(label: str, topic: str, last_update_time: float, now: float) -> Check:
    """Whether a topic has arrived, and recently.

    Args:
        label: Chip text.
        topic: The topic's name, for the detail text.
        last_update_time: ROS time of its last message, or 0.0 if none came.
        now: Current ROS time, seconds.

    Returns:
        Check: BAD if missing or older than STALE_AFTER, else GOOD.
    """
    if last_update_time == 0.0:
        return Check(label, BAD, f"no {topic} yet")
    age = now - last_update_time
    if age > STALE_AFTER:
        return Check(label, BAD, f"{topic} is {age:.0f}s old")
    return Check(label, GOOD, f"{topic} {age:.1f}s old")


def tool_check(arm: ArmInterface, now: float) -> Check | None:
    """Whether the arm's end effector driver is reachable.

    Args:
        arm: The arm the tool is mounted on.
        now: Current ROS time, seconds.

    Returns:
        Check | None: The tool's chip, or None for a bare arm.
    """
    tool = arm.end_effector
    label = f"{arm.config.name} tool"
    if isinstance(tool, RobotiqGripper):
        # ? The gripper only reports through its action, so the connection
        #   and the last result are all there is to check.
        if not tool.server_is_ready():
            return Check(label, BAD, "gripper action server not connected")
        if tool.state.last_result_ok is False:
            return Check(label, WARN, "last gripper command did not reach its target")
        return Check(label, GOOD, "robotiq connected")
    if isinstance(tool, ScaffoldingV3):
        return age_check(label, "tool_status", tool.state.last_update_time, now)
    if isinstance(tool, ScaffoldingV1):
        return Check(label, WARN, "scaffolding_v1 is not implemented; commands will raise")
    return None


def object_check(obj: TrackedObject, now: float) -> Check:
    """Whether a tracked object's pose is live.

    Args:
        obj: The object.
        now: Current ROS time, seconds.

    Returns:
        Check: The object's chip, labelled with its name.
    """
    if obj.last_update_time == 0.0:
        return Check(obj.name, BAD, "never seen by mocap")
    age = now - obj.last_update_time
    if age > STALE_AFTER:
        return Check(obj.name, BAD, f"last seen {age:.0f}s ago")
    if not obj.tracked:
        return Check(obj.name, BAD, "not tracked; markers hidden?")
    return Check(obj.name, GOOD, "tracked")


# --- --- --- --- --- DRAWING --- --- --- --- ---

def render(rows: list[tuple[str, list[Check]]]) -> str:
    """The whole panel as HTML: a banner, then one section per row.

    Args:
        rows: (title, checks) pairs, in display order.

    Returns:
        str: HTML for the panel's widget.
    """
    problems = [check for _, checks in rows for check in checks if check.level > GOOD]
    worst = max((check.level for check in problems), default=GOOD)
    if not rows:
        banner = chip("NOTHING TO CHECK", BUSY, "no robots and no tracked objects")
    elif not problems:
        banner = chip("ALL GREEN", OK)
    else:
        banner = chip(f"{len(problems)} PROBLEM{'S' if len(problems) > 1 else ''}", LEVEL_COLORS[worst])
    html = block(banner)

    for title, checks in rows:
        row_worst = max((check.level for check in checks), default=GOOD)
        html += section(title, LEVEL_COLORS[row_worst])
        # * Chips only; the detail is in each chip's tooltip.
        html += block("".join(chip(check.label, LEVEL_COLORS[check.level], escape(check.detail))
                              for check in checks))
    return html
