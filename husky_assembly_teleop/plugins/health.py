"""
The health panel: a banner, one row of chips per robot (with Unlock, Resume, Reconnect) and one for tracked objects.

Green is fine, amber needs a look, red is broken or silent; hover a chip for detail.
Loaded by default (config.DEFAULT_PLUGINS).

! Checks never command or block: each is a pure function of measured state and the clock.
"""

from __future__ import annotations

import math

import viser
from sensor_msgs.msg import BatteryState
from ur_dashboard_msgs.msg import RobotMode, SafetyMode

from ..plugin_api.context import PluginContext
from ..world.mocap import mocap_check
from ..plugin_api.plugin import HuskyPlugin, register
from ..robot_interface.arm import ArmInterface
from ..robot_interface.end_effectors import RobotiqGripper, ScaffoldingV1, ScaffoldingV3
from ..robot_interface.robot import HuskyRobotInterface
from ..robot_interface.arm import JOINT_STATES_RATE, SYNC_STATUS_MAX_AGE, ArmState
from ..robot_interface.base import BaseState
from ..robot_interface.controller_manager import REFRESH_PERIOD, ControllerManagerState
from ..robot_interface.stream_stats import WINDOW, StreamQuality
from ..ui.style import BUSY, LEVEL_COLORS, OK, block, check_chip, chip, section
from ..world.checks import BAD, GOOD, STALE_AFTER, WARN, Check

# --- --- --- --- --- THRESHOLDS --- --- --- --- ---

#: Seconds without a list_controllers answer before a controller manager counts as gone.
CONTROLLER_STALE_AFTER = 3 * REFRESH_PERIOD
#: Battery charge (0 to 1) below which the chip turns amber, then red.
BATTERY_WARN = 0.3
BATTERY_BAD = 0.15
#: Seconds without a battery message before the reading counts as old. Generous: the BMS rate is unknown.
BATTERY_STALE_AFTER = 10.0

#: Share of JOINT_STATES_RATE arriving below which the signal chip turns amber, then red.
SIGNAL_RATE_WARN = 0.9
SIGNAL_RATE_BAD = 0.5
#: Largest gap between joint_states, seconds, above which the signal chip turns amber, then red.
#: ? At 50 Hz, 100 ms is about four samples lost in a row.
SIGNAL_GAP_WARN = 0.1
SIGNAL_GAP_BAD = 0.2

#: BMS health values that are fine; anything else is red.
BATTERY_HEALTH_OK = (BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN, BatteryState.POWER_SUPPLY_HEALTH_GOOD)
#: RobotMode constant -> readable name, e.g. 5 -> "idle".
ROBOT_MODE_NAMES = {getattr(RobotMode, name): name.lower().replace("_", " ")
                    for name in dir(RobotMode) if name.isupper() and isinstance(getattr(RobotMode, name), int)}
#: SafetyMode constant -> readable name, e.g. 3 -> "protective stop".
SAFETY_MODE_NAMES = {getattr(SafetyMode, name): name.lower().replace("_", " ")
                     for name in dir(SafetyMode) if name.isupper() and isinstance(getattr(SafetyMode, name), int)}

# ! Keep fast-changing values (ages, voltage) out of chip text: a change rebuilds the row and closes tooltips.

#: Robot button label -> HuskyRobotInterface method it calls.
ROBOT_ACTIONS = {
    "Unlock": "unlock_protective_stop",
    "Resume": "resume_arms",
    "Reconnect": "reconnect",
}


@register
class HealthPlugin(HuskyPlugin):
    """One panel listing every robot and tracked object, green when all is well."""

    name = "health"

    def __init__(self):
        """Start with no widgets; setup builds them."""
        self._banner: viser.GuiHtmlHandle | None = None
        #: Per robot, by serial: its section bar and chips.
        self._robot_rows: dict[str, viser.GuiHtmlHandle] = {}
        #: One row for all tracked objects.
        self._objects_row: viser.GuiHtmlHandle | None = None
        #: This tick's checks per robot (by serial) and for tracked objects.
        self._robot_checks: dict[str, list[Check]] = {}
        self._object_checks: list[Check] = []
        #: Last level per (row, chip position), to log when a chip changes colour.
        self._levels: dict[tuple[str, int], int] = {}

    def setup(self, ctx: PluginContext) -> None:
        """Build the banner, one chip row and action buttons per robot, and the objects row.

        Args:
            ctx: This plugin's context.
        """
        with ctx.view.ui() as gui:
            self._banner = gui.add_html("")
            for serial in ctx.world.robots:
                self._robot_rows[serial] = gui.add_html("")
                # ? A button group: the only way in viser 1.1 to put buttons side by side.
                actions = gui.add_button_group(
                    "Robot", list(ROBOT_ACTIONS),
                    hint="Unlock: like 'Enable robot' on the teach pendant; clear the cause first. "
                         "Resume: undo 'Stop all' for this robot's arms (the base comes back with its "
                         "controller button in the robot panel). "
                         "Reconnect: rebuild every topic, service and action of this robot.")
                # ! Bind this loop's serial as a default argument.
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
        self._object_checks = [mocap_check(obj.name, obj.mocap_id, obj, now)
                               for obj in ctx.world.tracked_objects.values()]
        for row, checks in [*self._robot_checks.items(), ("objects", self._object_checks)]:
            self._log_changes(ctx, row, checks)

    def _log_changes(self, ctx: PluginContext, row: str, checks: list[Check]) -> None:
        """Log every chip that changed colour since the last tick, so short outages can be traced later.

        A chip's first level is not logged: at start every chip is still waiting for its robot.

        Args:
            ctx: This plugin's context.
            row: The row, a robot's serial or "objects".
            checks: That row's checks this tick.
        """
        # * Keyed by position, not label: combined chips change their label with their colour.
        for index, check in enumerate(checks):
            key = (row, index)
            before = self._levels.get(key)
            self._levels[key] = check.level
            if before is None or before == check.level:
                continue
            message = f"health {row} {check.label}: {LEVEL_NAMES[before]} -> {LEVEL_NAMES[check.level]}: {check.detail}"
            if check.level == BAD:
                ctx.log_error(message)
            elif check.level > before:
                ctx.log_warn(message)
            else:
                ctx.log_info(message)

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


#: Chip colours by level, for the log.
LEVEL_NAMES = {GOOD: "green", WARN: "amber", BAD: "red"}


# --- --- --- --- --- CHECKS (`now` is ROS time, seconds) --- --- --- --- ---

def robot_checks(robot: HuskyRobotInterface, now: float) -> list[Check]:
    """Every check for one robot: wifi signal, mocap, base controllers, then each arm.

    Args:
        robot: The robot to check.
        now: Current ROS time, seconds.

    Returns:
        list[Check]: One per chip, in display order.
    """
    checks = [signal_check(robot, now),
              mocap_check("mocap", robot.config.mocap_id, robot.base.state, now),
              estop_check(robot.base.state),
              battery_check(robot.base.state, now),
              controller_check("base ctrl", robot.base.state.controllers, now)]
    for arm in robot.arms.values():
        checks.append(arm_check(arm, now))
        # * The tool gets its own chip: its driver fails independently of the arm.
        tool = tool_check(arm, now)
        if tool is not None:
            checks.append(tool)
    return checks


def signal_check(robot: HuskyRobotInterface, now: float) -> Check:
    """How well the wifi carries the robot's streams, judged by each arm's joint_states.

    Args:
        robot: The robot to check.
        now: Current ROS time, seconds.

    Returns:
        Check: One chip for the robot, labelled with the worst arm when not all are good.
    """
    if not robot.arms:
        return Check("signal", WARN, "no arm, so no stream to judge the link by")
    return combine("signal", [stream_check(arm.config.name, arm.state.joint_states_stats.quality(now))
                              for arm in robot.arms.values()])


def stream_check(label: str, quality: StreamQuality) -> Check:
    """Whether joint_states arrive at the rate limiter's rate without gaps.

    ! Only thresholds in the text, never the measured values: they change every tick and would
      rebuild the row, closing its tooltips.

    Args:
        label: Chip text, e.g. the arm's name.
        quality: The stream's recent rate and largest gap.

    Returns:
        Check: BAD when silent or far off, WARN when a little off, else GOOD.
    """
    if quality.rate is None:
        return Check(label, WARN, f"joint_states seen for under {WINDOW:g}s; still measuring")
    if quality.rate == 0.0:
        return Check(label, BAD, f"no joint_states in the last {WINDOW:g}s")
    rate_warn, rate_bad = SIGNAL_RATE_WARN * JOINT_STATES_RATE, SIGNAL_RATE_BAD * JOINT_STATES_RATE
    gap = quality.max_gap or 0.0
    problems = []
    if quality.rate < rate_bad:
        problems.append((BAD, f"joint_states below {rate_bad:g} Hz"))
    elif quality.rate < rate_warn:
        problems.append((WARN, f"joint_states below {rate_warn:g} Hz"))
    if gap > SIGNAL_GAP_BAD:
        problems.append((BAD, f"a gap over {SIGNAL_GAP_BAD * 1e3:g} ms"))
    elif gap > SIGNAL_GAP_WARN:
        problems.append((WARN, f"a gap over {SIGNAL_GAP_WARN * 1e3:g} ms"))
    if problems:
        text = " and ".join(text for _level, text in problems)
        return Check(label, max(level for level, _text in problems),
                     f"{text} in the last {WINDOW:g}s; samples lost on the wifi?")
    unknown = "; gaps unknown (unstamped)" if quality.max_gap is None else ""
    return Check(label, GOOD, f"joint_states at {rate_warn:g} Hz or more, "
                              f"no gap over {SIGNAL_GAP_WARN * 1e3:g} ms{unknown}")


def arm_check(arm: ArmInterface, now: float) -> Check:
    """Every check of one arm, folded into one chip labelled with its worst part.

    Args:
        arm: The arm.
        now: Current ROS time, seconds.
    """
    state = arm.state
    parts = [sync_check(state, now)]
    # Dashboard parts are known only while the sync's status is fresh.
    if parts[0].level < BAD:
        parts += [dashboard_check(state)]
        if state.dashboard_up:
            parts += [safety_check(state.safety_mode), running_check(state)]
    parts += [controller_check("ctrl", state.controllers, now),
              age_check("joints", "joint_states", state.last_update_time, now)]
    return combine(arm.config.name, parts)


def combine(label: str, parts: list[Check]) -> Check:
    """Fold several checks into one chip: the worst level, every detail in the tooltip.

    Args:
        label: The chip's name, e.g. the arm's.
        parts: The checks to fold, in tooltip order.

    Returns:
        Check: Labelled `label`, plus the worst part's label when not all are good.
    """
    level = worst_level(parts)
    worst = next(part for part in parts if part.level == level)
    detail = "\n".join(f"{part.label}: {part.detail}" for part in parts)
    return Check(label if level == GOOD else f"{label} {worst.label}", level, detail)


def estop_check(state: BaseState) -> Check:
    """Whether the platform's emergency stop is engaged.

    Args:
        state: The base's measured state.
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
        Check: Labelled with the charge when known.
    """
    if state.battery_update_time is None:
        return Check("battery", WARN, "no bms/state message yet")
    percentage = state.battery_percentage
    known = percentage is not None and math.isfinite(percentage)
    # * 5 % steps, no voltage: a jittering label would rebuild the row.
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
    """
    if not state.dashboard_up:
        return Check("NO DASHBOARD", WARN, "no dashboard_client; fake hardware, or the driver is down")
    if not state.dashboard_connected:
        return Check("DASHBOARD", BAD, state.problem or "dashboard not connected to the arm")
    return Check("dash", GOOD, "connected")


def safety_check(mode: int | None) -> Check:
    """Whether the arm is in NORMAL safety mode; anything else stops every arm.

    Args:
        mode: The UR SafetyMode constant, or None if unknown.

    Returns:
        Check: Labelled with the mode when not normal.
    """
    if mode is None:
        return Check("safety", WARN, "safety mode not known yet")
    if mode == SafetyMode.NORMAL:
        return Check("safety", GOOD, "normal")
    name = SAFETY_MODE_NAMES.get(mode, f"mode {mode}")
    return Check(name.upper(), BAD, f"{name}; clear it on the teach pendant")


def running_check(state: ArmState) -> Check:
    """Whether the arm can follow commands: brakes released and ros_control.urp playing.

    ? Separate from safety: in NORMAL safety the brakes may still be locked or the program stopped.

    Args:
        state: The arm's measured state.

    Returns:
        Check: Labelled with what is missing.
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
        last_update_time: ROS time of its last message, or None.
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
        Check | None: Labelled "<arm> <tool kind>", or None for a bare arm.
    """
    tool = arm.end_effector
    label = f"{arm.config.name} {arm.config.end_effector}"
    if isinstance(tool, RobotiqGripper):
        if not tool.server_is_ready():
            return Check(label, BAD, "gripper action server not connected")
        joints = age_check(label, "gripper joint_states", tool.state.last_update_time, now)
        if joints.level != GOOD:
            return joints
        if tool.state.reactivate_error:
            return Check(label, WARN, f"reactivation failed: {tool.state.reactivate_error}")
        if tool.state.last_result_ok is False:
            return Check(label, WARN, "last gripper command did not reach its target")
        return Check(label, GOOD, "robotiq connected")
    if isinstance(tool, ScaffoldingV3):
        # ? A stalled motor is not checked: the screws stall on purpose once tight.
        return age_check(label, "tool_status", tool.state.last_update_time, now)
    if isinstance(tool, ScaffoldingV1):
        # ? The tool reports nothing; only the arm's set_io service and io_states can fail.
        if not tool.service_is_ready():
            return Check(label, BAD, "set_io service not available")
        return age_check(label, "io_states", tool.state.last_update_time, now)
    return None


# --- --- --- --- --- DRAWING --- --- --- --- ---

def render_banner(checks: list[Check]) -> str:
    """The banner's HTML: green when every check is, else how many are not.

    Args:
        checks: Every check on the panel.
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
    """One row's HTML: a chip per check, its detail in the tooltip."""
    return block("".join(check_chip(check) for check in checks))
