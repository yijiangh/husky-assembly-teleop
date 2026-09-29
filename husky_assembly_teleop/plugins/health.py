"""
The health panel: one glance to see whether every robot and tracked object is fine.

Each robot and each tracked object gets one row of chips. Green means fine,
amber means it works but look at it, red means it is broken or silent. Hover a
chip for the detail. The banner on top is green only when every chip is.

What is checked, and why it matters:

  mocap          the base pose is only ever from mocap. Silent relay, body lost
                 by NatNet (hidden markers), or a pose the relay marked invalid
                 are red; a high marker error is amber.
  estop          the platform's emergency stop. Engaged is red; no message yet
                 is amber, since then nobody knows.
  battery        charge from the BMS: amber below BATTERY_WARN, red below
                 BATTERY_BAD or when the BMS reports a health problem.
  base ctrl      the base's controller manager. Never answered or stopped
                 answering is red (driver down, or wifi); no controller running
                 or a failed switch is amber, as commands will refuse.
  <arm>          one chip per arm, named after its worst part; the tooltip
                 lists every part:
                   sync     status from multi_arm_safety_sync on the robot.
                            Missing is red.
                   dash     the arm's dashboard. Not connected is red; no
                            dashboard_client at all (fake hardware) is amber.
                   safety   anything but NORMAL is red, as the sync then stops
                            every arm. A protective stop can be unlocked below.
                   running  robot mode RUNNING (brakes released) and the
                            program playing. Anything else is red: the arm will
                            not move, however healthy the rest looks.
                   ctrl     the arm's controller manager, as for the base.
                   joints   joint_states from the rate limiter. Missing or old is red.
                   tool     robotiq: action server not connected.
                            scaffolding_v3: tool_status missing or old.
  objects        every entry in world.tracked_objects: whether it is tracked
                 and how old its last sample is.

Each robot also has
  - Unlock protective stop, like "Enable robot" on the teach pendant, through
    multi_arm_safety_sync. The sync restarts ros_control.urp by itself once
    every arm is operational. ! Clear the cause first; the robot refuses it
    for 5 s after the stop.
  - Resume arms, which lifts the soft stop ("Stop all" / Esc) of its arms.
    The sync restarts them once every arm is operational.
  - Reconnect, which rebuilds all its topics, services and actions
    (HuskyRobotInterface.reconnect). Its state is kept, so the chips stay red
    until fresh data arrives.

! The checks never command anything and never block. Each is a function of
  measured state and the clock, written as a pure function below the plugin,
  which keeps them easy to read and to test on their own.

Loaded by default (config.DEFAULT_PLUGINS); --no-default turns that off.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from html import escape

import viser
from sensor_msgs.msg import BatteryState
from ur_dashboard_msgs.msg import RobotMode, SafetyMode

from ..context import PluginContext
from ..plugin import HuskyPlugin, register
from ..robot_interface import ArmInterface, HuskyRobotInterface, RobotiqGripper, ScaffoldingV1, ScaffoldingV3
from ..robot_interface.arm import SYNC_STATUS_MAX_AGE, ArmState
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
#: Battery charge, 0 to 1, below which the battery chip turns amber, and red.
BATTERY_WARN = 0.3
BATTERY_BAD = 0.15
#: Seconds without a battery message before the reading counts as old.
#: ? Generous, because the BMS publish rate is not pinned down.
BATTERY_STALE_AFTER = 10.0

#: BMS health values that are fine. Anything else (overheat, dead, ...) is red.
BATTERY_HEALTH_OK = (BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN, BatteryState.POWER_SUPPLY_HEALTH_GOOD)
#: RobotMode constant -> readable name, e.g. 5 -> "idle".
ROBOT_MODE_NAMES = {getattr(RobotMode, name): name.lower().replace("_", " ")
                    for name in dir(RobotMode) if name.isupper() and isinstance(getattr(RobotMode, name), int)}
#: SafetyMode constant -> readable name, e.g. 3 -> "protective stop".
SAFETY_MODE_NAMES = {getattr(SafetyMode, name): name.lower().replace("_", " ")
                     for name in dir(SafetyMode) if name.isupper() and isinstance(getattr(SafetyMode, name), int)}

# ! Chip labels and tooltips hold no fast-changing values: no ages, no marker
#   error, no voltage. viser replaces a row's whole HTML whenever its text
#   changes, which closes any open tooltip and drops a text selection, so one
#   ticking number makes every tooltip in its row unusable. Text should change
#   only when the state it describes changes.

#: Robot button label -> the HuskyRobotInterface method it calls.
ROBOT_ACTIONS = {
    "Unlock": "unlock_protective_stop",
    "Resume": "resume_arms",
    "Reconnect": "reconnect",
}

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
        self._banner: viser.GuiHtmlHandle | None = None
        #: Per robot, by serial: its section bar and chips. Its Reconnect button
        #: is a separate widget below.
        self._robot_rows: dict[str, viser.GuiHtmlHandle] = {}
        #: One chip row for all tracked objects. One widget, since objects can
        #: appear while running.
        self._objects_row: viser.GuiHtmlHandle | None = None
        #: This tick's checks per robot, by serial, and for the tracked objects.
        self._robot_checks: dict[str, list[Check]] = {}
        self._object_checks: list[Check] = []

    def setup(self, ctx: PluginContext) -> None:
        """Build the banner, one chip row and action buttons per robot, and the objects row.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            self._banner = gui.add_html("")
            for serial in ctx.world.robots:
                self._robot_rows[serial] = gui.add_html("")
                # ? One button group, so the three sit side by side. viser 1.1 has
                #   no other side-by-side layout; the price is small buttons with
                #   no icons and one tooltip for all three.
                actions = gui.add_button_group(
                    "Robot", list(ROBOT_ACTIONS),
                    hint="Unlock: like 'Enable robot' on the teach pendant; clear the cause first. "
                         "Resume: undo 'Stop all' for this robot's arms (the base comes back with its "
                         "controller button in the robot panel). "
                         "Reconnect: rebuild every topic, service and action of this robot.")
                # ! Default argument, so each group keeps its own serial.
                actions.on_click(ctx.defer_value(
                    f"robot action {serial}",
                    lambda clicked, serial=serial: getattr(ctx.world.robots[serial], ROBOT_ACTIONS[clicked])()))
            self._objects_row = gui.add_html("")

    def update(self, ctx: PluginContext) -> None:
        """Run every check against this tick's state.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        self._robot_checks = {serial: robot_checks(robot, now) for serial, robot in ctx.world.robots.items()}
        self._object_checks = [object_check(obj, now) for obj in ctx.world.tracked_objects.values()]

    def draw(self, ctx: PluginContext) -> None:
        """Copy this tick's checks into the banner and the rows.

        Args:
            ctx: This plugin's context.
        """
        every_check = [check for checks in self._robot_checks.values() for check in checks]
        self._banner.content = render_banner(every_check + self._object_checks)
        for serial, checks in self._robot_checks.items():
            self._robot_rows[serial].content = (section(serial, LEVEL_COLORS[worst_level(checks)])
                                                + render_chips(checks))
        self._objects_row.content = (section("objects", LEVEL_COLORS[worst_level(self._object_checks)])
                                     + render_chips(self._object_checks)) if self._object_checks else ""


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
              estop_check(robot.base.state),
              battery_check(robot.base.state, now),
              controller_check("base ctrl", robot.base.state.controllers, now)]
    return checks + [arm_check(arm, now) for arm in robot.arms.values()]


def arm_check(arm: ArmInterface, now: float) -> Check:
    """Every check of one arm, folded into one chip.

    Args:
        arm: The arm.
        now: Current ROS time, seconds.

    Returns:
        Check: The arm's chip, labelled with its worst part.
    """
    state = arm.state
    parts = [sync_check(state, now)]
    # The dashboard's parts are only known while the sync's status is fresh.
    if parts[0].level < BAD:
        parts += [dashboard_check(state)]
        if state.dashboard_up:
            parts += [safety_check(state.safety_mode), running_check(state)]
    parts += [controller_check("ctrl", state.controllers, now),
              age_check("joints", "joint_states", state.last_update_time, now)]
    tool = tool_check(arm, now)
    if tool is not None:
        parts.append(tool)
    return combine(arm.config.name, parts)


def combine(label: str, parts: list[Check]) -> Check:
    """Fold several checks into one chip: the worst level, every detail in the tooltip.

    Args:
        label: The chip's name, e.g. the arm's.
        parts: The checks to fold, in tooltip order.

    Returns:
        Check: Labelled `label` when all are good, else `label` and the worst part's label.
    """
    level = worst_level(parts)
    worst = next(part for part in parts if part.level == level)
    detail = "\n".join(f"{part.label}: {part.detail}" for part in parts)
    return Check(label if level == GOOD else f"{label} {worst.label}", level, detail)


def mocap_check(mocap_id: int | None, state: BaseState, now: float) -> Check:
    """Whether the base pose is live, and how good the fix is.

    Args:
        mocap_id: The base's rigid-body id, or None if it has none configured.
        state: The base's measured state.
        now: Current ROS time, seconds.

    Returns:
        Check: The mocap chip.
    """
    if mocap_id is None:
        return Check("mocap", WARN, "no mocap id configured; the base pose is never measured")
    if state.last_update_time is None:
        return Check("mocap", BAD, f"no message for rigid body {mocap_id} yet; is mocap_relay running?")
    age = now - state.last_update_time
    if age > STALE_AFTER:
        return Check("mocap", BAD, f"relay silent for over {STALE_AFTER:g}s (rigid body {mocap_id})")

    # ! No marker error value in the label or tooltip: it changes with every
    #   sample, and each change rebuilds the row, closing any open tooltip. The
    #   chip's colour carries the quality instead (amber above MARKER_ERROR_WARN).
    label = "mocap"
    warn_mm = MARKER_ERROR_WARN * 1e3
    # * Why the relay marked it invalid, in the order it decides: NatNet lost
    #   the body, or the marker error is past the relay's threshold, or the
    #   relay's own cache went stale.
    if not state.tracking_valid:
        return Check(label, BAD, f"rigid body {mocap_id} lost by mocap; markers hidden?")
    if not state.tracked:
        return Check(label, BAD, "relay marks the pose invalid (marker error past its threshold, or stale)")
    if state.marker_error > MARKER_ERROR_WARN:
        return Check(label, WARN, f"marker error above {warn_mm:g} mm; "
                                  f"check the markers and the rigid body definition")
    return Check(label, GOOD, f"rigid body {mocap_id}, marker error under {warn_mm:g} mm")


def estop_check(state: BaseState) -> Check:
    """Whether the platform's emergency stop is engaged.

    Args:
        state: The base's measured state.

    Returns:
        Check: The e-stop chip.
    """
    if state.estopped is None:
        return Check("estop", WARN, "no emergency_stop message yet; state unknown")
    if state.estopped:
        return Check("ESTOP", BAD, "platform emergency stop is engaged")
    return Check("estop", GOOD, "released")


def battery_check(state: BaseState, now: float) -> Check:
    """How full the battery is, and whether the BMS reports a problem.

    Args:
        state: The base's measured state.
        now: Current ROS time, seconds.

    Returns:
        Check: The battery chip, labelled with the charge when known.
    """
    if state.battery_update_time is None:
        return Check("battery", WARN, "no bms/state message yet")
    percentage = state.battery_percentage
    known = percentage is not None and math.isfinite(percentage)
    # * In 5 % steps and without the voltage: the BMS reading jitters under
    #   load, and every change rebuilds the row, closing any open tooltip.
    label = f"battery {round(percentage * 20) * 5}%" if known else "battery"
    detail = "charging" if state.battery_charging else "discharging"
    if now - state.battery_update_time > BATTERY_STALE_AFTER:
        return Check(label, WARN, f"no bms/state for over {BATTERY_STALE_AFTER:g}s")
    if state.battery_health not in BATTERY_HEALTH_OK:
        return Check(label, BAD, f"BMS reports health problem {state.battery_health}; {detail}")
    if known and percentage < BATTERY_BAD:
        return Check(label, BAD, f"battery nearly empty; {detail}")
    if known and percentage < BATTERY_WARN:
        return Check(label, WARN, f"battery low; {detail}")
    return Check(label, GOOD, detail)


def sync_check(state: ArmState, now: float) -> Check:
    """Whether multi_arm_safety_sync reports, and whether it lets the arms run.

    Args:
        state: The arm's measured state.
        now: Current ROS time, seconds.

    Returns:
        Check: The sync part of the arm's chip.
    """
    if state.status_update_time is None:
        return Check("SYNC", BAD, "no status from multi_arm_safety_sync; is it running on the robot?")
    if now - state.status_update_time > SYNC_STATUS_MAX_AGE:
        return Check("SYNC", BAD, f"multi_arm_safety_sync silent for over {SYNC_STATUS_MAX_AGE:g}s")
    return Check("sync", GOOD, f"last stop: {state.stop_reason}" if state.stop_reason else "reporting")


def dashboard_check(state: ArmState) -> Check:
    """Whether the sync reaches the arm's dashboard.

    Args:
        state: The arm's measured state.

    Returns:
        Check: The dashboard part of the arm's chip.
    """
    if not state.dashboard_up:
        return Check("NO DASHBOARD", WARN, "no dashboard_client; fake hardware, or the driver is down")
    if not state.dashboard_connected:
        return Check("DASHBOARD", BAD, state.problem or "dashboard not connected to the arm")
    return Check("dash", GOOD, "connected")


def safety_check(mode: int | None) -> Check:
    """Whether the arm is in NORMAL safety mode. Anything else stops every arm.

    Args:
        mode: The UR SafetyMode constant, or None if unknown.

    Returns:
        Check: The safety part of the arm's chip, labelled with the mode when not normal.
    """
    if mode is None:
        return Check("safety", WARN, "safety mode not known yet")
    if mode == SafetyMode.NORMAL:
        return Check("safety", GOOD, "normal")
    name = SAFETY_MODE_NAMES.get(mode, f"mode {mode}")
    return Check(name.upper(), BAD, f"{name}; clear it on the teach pendant")


def running_check(state: ArmState) -> Check:
    """Whether the arm can follow commands: brakes released and external control playing.

    ? Separate from safety because safety NORMAL does not mean the arm can
      move. After power-on it sits in IDLE with the brakes locked, and even
      when RUNNING it ignores ROS until ros_control.urp plays. The controller
      manager and joint_states look healthy through all of this.

    Args:
        state: The arm's measured state.

    Returns:
        Check: The running part of the arm's chip, labelled with what is missing.
    """
    if state.robot_mode is None:
        return Check("running", WARN, "robot mode not known yet")
    if state.robot_mode != RobotMode.RUNNING:
        name = ROBOT_MODE_NAMES.get(state.robot_mode, f"mode {state.robot_mode}")
        return Check(name.upper(), BAD, f"robot mode is {name}; power on and release the brakes on the teach pendant")
    if not state.program_playing:
        program = state.program_name or "no program"
        return Check("NO PROGRAM", BAD, f"{program} is not playing; the arm ignores ROS until ros_control.urp plays"
                                        + (f" ({state.problem})" if state.problem else ""))
    return Check("running", GOOD, f"{state.program_name} playing")


def controller_check(label: str, state: ControllerManagerState, now: float) -> Check:
    """Whether a controller manager answers, and a controller is running.

    Args:
        label: Chip text, e.g. "base ctrl".
        state: That controller manager's state.
        now: Current ROS time, seconds.

    Returns:
        Check: The controller manager's chip.
    """
    if state.last_update_time is None:
        return Check(label, BAD, "controller manager never answered; driver down or not discovered")
    age = now - state.last_update_time
    if age > CONTROLLER_STALE_AFTER:
        return Check(label, BAD, f"controller manager silent for over {CONTROLLER_STALE_AFTER:g}s")
    if state.switch_error:
        return Check(label, WARN, state.switch_error)
    if state.active is None:
        return Check(label, WARN, "no controller active; commands will be refused")
    return Check(label, GOOD, state.active)


def age_check(label: str, topic: str, last_update_time: float | None, now: float) -> Check:
    """Whether a topic has arrived, and recently.

    Args:
        label: Chip text.
        topic: The topic's name, for the detail text.
        last_update_time: ROS time of its last message, or None if none came.
        now: Current ROS time, seconds.

    Returns:
        Check: BAD if missing or older than STALE_AFTER, else GOOD.
    """
    if last_update_time is None:
        return Check(label, BAD, f"no {topic} yet")
    age = now - last_update_time
    if age > STALE_AFTER:
        return Check(label, BAD, f"no {topic} for over {STALE_AFTER:g}s")
    return Check(label, GOOD, f"{topic} arriving")


def tool_check(arm: ArmInterface, now: float) -> Check | None:
    """Whether the arm's end effector driver is reachable.

    Args:
        arm: The arm the tool is mounted on.
        now: Current ROS time, seconds.

    Returns:
        Check | None: The tool's chip, or None for a bare arm.
    """
    tool = arm.end_effector
    label = "tool"
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
    if obj.last_update_time is None:
        return Check(obj.name, BAD, "never seen by mocap")
    age = now - obj.last_update_time
    if age > STALE_AFTER:
        return Check(obj.name, BAD, f"not seen for over {STALE_AFTER:g}s")
    if not obj.tracked:
        return Check(obj.name, BAD, "not tracked; markers hidden?")
    return Check(obj.name, GOOD, "tracked")


# --- --- --- --- --- DRAWING --- --- --- --- ---

def render_banner(checks: list[Check]) -> str:
    """The banner: green when every check is, else how many are not.

    Args:
        checks: Every check on the panel.

    Returns:
        str: HTML for the banner widget.
    """
    problems = [check for check in checks if check.level > GOOD]
    if not checks:
        return block(chip("NOTHING TO CHECK", BUSY, "no robots and no tracked objects"))
    if not problems:
        return block(chip("ALL GREEN", OK))
    return block(chip(f"{len(problems)} PROBLEM{'S' if len(problems) > 1 else ''}",
                      LEVEL_COLORS[worst_level(problems)]))


def worst_level(checks: list[Check]) -> int:
    """The most severe level among `checks`, GOOD when there are none."""
    return max((check.level for check in checks), default=GOOD)


def render_chips(checks: list[Check]) -> str:
    """One row of chips, one per check. The detail is in each chip's tooltip.

    Args:
        checks: The row's checks.

    Returns:
        str: HTML for the row's widget.
    """
    return block("".join(chip(check.label, LEVEL_COLORS[check.level], escape(check.detail)) for check in checks))
