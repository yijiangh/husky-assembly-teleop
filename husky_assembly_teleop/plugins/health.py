"""
The health panel: a banner, a row per robot (with Unlock, Resume, Reconnect) and one for tracked objects.

The banner sits under Stop all (`ctx.view.header()`), so it shows whichever tabs are open: an overall chip, then
one per robot and one for tracked objects.

A robot's row has a line of chips for its base and one for each arm with its tool.

Green is fine, amber needs a look, red is broken or silent, blue is a fact to know (a "sim" chip before the simulated
part's chips); hover a chip for detail.
Loaded by default (config.DEFAULT_PLUGINS).

! Checks never command or block: each is a pure function of measured state and the clock.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

import viser
from sensor_msgs.msg import BatteryState
from ur_dashboard_msgs.msg import RobotMode, SafetyMode

from ..plugin_api.context import PluginContext
from ..world.mocap import mocap_check
from ..world.tcp import tcp_check, tool0_by_kinematics
from ..plugin_api.plugin import HuskyPlugin, register
from ..robot_interface.arm import ArmInterface
from ..robot_interface.end_effectors import RobotiqGripper, ScaffoldingV1, ScaffoldingV3
from ..robot_interface.robot import HuskyRobotInterface
from ..robot_interface.simulated_parts import SimulatedPart
from ..robot_interface.arm import JOINT_STATES_RATE as ARM_RATE
from ..robot_interface.arm import SYNC_STATUS_MAX_AGE, ArmState
from ..robot_interface.base import JOINT_STATES_RATE as BASE_RATE
from ..robot_interface.base import BaseState
from ..robot_interface.controller_manager import REFRESH_PERIOD, ControllerManagerState
from ..robot_interface.mocap_definitions import OUT_OF_DATE
from ..robot_interface.mocap_relay import (FRAME_REFIT_PENDING, FRAME_UNCHECKED, FRAME_UNTRUSTED, TRUTH,
                                           AppliedCalibration, FloorCalibrationState, MocapRelayState)
from ..robot_interface.stream_stats import WINDOW, StreamQuality
from ..ui.phone import phone_text
from ..ui.style import BUSY, LEVEL_COLORS, NONE, block, check_chip, chip, chip_line, section
from ..world.checks import BAD, GOOD, INFO, STALE_AFTER, WARN, Check

# --- --- --- --- --- THRESHOLDS --- --- --- --- ---

#: Seconds without a list_controllers answer before a controller manager counts as gone.
CONTROLLER_STALE_AFTER = 3 * REFRESH_PERIOD
#: Battery charge (0 to 1) below which the chip turns amber, then red.
BATTERY_WARN = 0.3
BATTERY_BAD = 0.15
#: Seconds without a battery message before the reading counts as old. Generous: the BMS rate is unknown.
BATTERY_STALE_AFTER = 10.0

#: Share of a stream's nominal rate arriving below which the signal chip turns amber, then red.
SIGNAL_RATE_WARN = 0.9
SIGNAL_RATE_BAD = 0.5
#: Largest gap between joint_states, seconds, above which the signal chip turns amber, then red.
#: ? At 50 Hz, 100 ms is about four samples lost in a row; at the base's 30 Hz, two.
SIGNAL_GAP_WARN = 0.1
SIGNAL_GAP_BAD = 0.2
#: Delay (`StreamQuality.delay`), seconds: above these the chip turns amber, then red (robot clock behind, or a slow
#: network); below these negative ones too (robot clock ahead, for certain). ? Synced, it reads 1 to 5 ms.
SIGNAL_DELAY_WARN = 0.03
SIGNAL_DELAY_BAD = 0.1
SIGNAL_AHEAD_WARN = -0.005
SIGNAL_AHEAD_BAD = -0.02

#: Seconds (wall clock) a chip must keep a new colour before the change is logged, so a flickering chip makes one
#: line, not many.
LOG_HOLD = 1.5
#: The log topic of the colour changes: their source is "health/chips", which the console's default view leaves out.
CHIPS_TOPIC = "chips"

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
    "Resume": "resume",
    "Reconnect": "reconnect",
}
#: Refused while another plugin has the robot's lease: Unlock lets its arms move again, and Reconnect drops for a
#: moment the links that plugin's motion is watched and held through. Resume needs no lease (`robot_lease.py`).
LEASED_ACTIONS = ("Unlock", "Reconnect")


def robot_action(ctx: PluginContext, serial: str, clicked: str) -> None:
    """Run a robot button's action (a key of ROBOT_ACTIONS); LEASED_ACTIONS are refused while the robot is leased.

    Args:
        ctx: health's context.
        serial: The robot.
        clicked: The button's label.
    """
    robot = ctx.world.robots[serial]
    lease = robot.lease_slot.lease
    if clicked in LEASED_ACTIONS and lease is not None:
        ctx.log_warn(f"{serial} {clicked} refused: robot {serial} is leased by {lease.holder}")
        return
    getattr(robot, ROBOT_ACTIONS[clicked])()


@register
class HealthPlugin(HuskyPlugin):
    """One panel listing every robot and tracked object, green when all is well."""

    name = "health"
    icon = viser.Icon.HEART_RATE_MONITOR

    def __init__(self):
        """Start with no widgets; setup builds them."""
        self._banner: viser.GuiHtmlHandle | None = None
        #: Per robot, by serial: its section bar and chips.
        self._robot_rows: dict[str, viser.GuiHtmlHandle] = {}
        #: One row for all tracked objects.
        self._objects_row: viser.GuiHtmlHandle | None = None
        #: On the phone: the banner above the tabs, and every row in one html in the Health tab.
        self._phone_banner: viser.GuiHtmlHandle | None = None
        self._phone_rows: viser.GuiHtmlHandle | None = None
        #: This tick's checks per robot (by serial), as (line name, checks), and for tracked objects.
        self._robot_checks: dict[str, list[tuple[str, list[Check]]]] = {}
        self._object_checks: list[Check] = []
        #: Last level per (row, chip position), to log when a chip changes colour.
        self._levels: dict[tuple[str, int], int] = {}
        #: Colour changes not logged yet, by (row, chip position); see `_log_changes`.
        self._changes: dict[tuple[str, int], _Change] = {}
        #: Each arm's last TCP verdict by (serial, arm name), kept while the arm moves (`tcp_check`).
        self._tcp_checks: dict[tuple[str, str], Check] = {}

    def setup(self, ctx: PluginContext) -> None:
        """Build the banner, one chip row and action buttons per robot, the objects row, and the same on the phone.

        Args:
            ctx: This plugin's context.
        """
        # * The banner sits under Stop all, so a problem shows whichever tabs are open.
        with ctx.view.header() as gui:
            self._banner = gui.add_html("")
        # * Starts collapsed: the banner above says enough until something needs a look.
        with ctx.view.folder(expanded=False) as gui:
            for serial in ctx.world.robots:
                self._robot_rows[serial] = gui.add_html("")
                # ? A button group: the only way in viser 1.1 to put buttons side by side.
                actions = gui.add_button_group(
                    "Robot", list(ROBOT_ACTIONS),
                    hint="Unlock: like 'Enable robot' on the teach pendant; clear the cause first. "
                         "Resume: undo 'Stop all' for this robot: it takes motion commands again, its arms' "
                         "programs play again and its base's velocity controller is switched on. "
                         "Reconnect: rebuild every topic, service and action of this robot. "
                         "Unlock and Reconnect are refused while another plugin has the robot's lease.")
                # ! Bind this loop's serial as a default argument.
                actions.on_click(ctx.defer_value(f"robot action {serial}",
                                                 lambda clicked, serial=serial: robot_action(ctx, serial, clicked)))
            self._objects_row = gui.add_html("")
        # * The phone shows the same banner and rows, without the actions: those stay on the laptop.
        with ctx.view.phone_header() as phone:
            self._phone_banner = phone.add_html("")
        with ctx.view.phone("Health", viser.Icon.HEART_RATE_MONITOR) as phone:
            self._phone_rows = phone.add_html("")

    def update(self, ctx: PluginContext) -> None:
        """Run every check against this tick's state.

        Args:
            ctx: This plugin's context.
        """
        now = ctx.now()
        for serial, robot in ctx.world.robots.items():
            for name, arm in robot.arms.items():
                check = tcp_check(arm, tool0_by_kinematics(ctx.kinematics, serial, name), now)
                if check is not None:
                    self._tcp_checks[(serial, name)] = check
        relay = ctx.world.mocap_relay.state if ctx.world.mocap_relay is not None else None
        self._robot_checks = {serial: robot_checks(robot, now, self._tcp_checks, applied_calibration(relay, robot),
                                                   applied_frame(relay, robot))
                              for serial, robot in ctx.world.robots.items()}
        self._object_checks = [mocap_check(obj.name, obj.mocap_id, obj, now)
                               for obj in ctx.world.tracked_objects.values()]
        lines = [(f"{serial} {name}", checks) for serial, robot_lines in self._robot_checks.items()
                 for name, checks in robot_lines]
        # * The hold runs on the wall clock: a paused simulation or a bag jumping back must not hold a change.
        held = time.monotonic()
        for row, checks in [*lines, ("objects", self._object_checks)]:
            self._note_changes(row, checks, held)
        self._log_changes(ctx, held)

    def _note_changes(self, row: str, checks: list[Check], now: float) -> None:
        """Note every chip of `row` ("<serial> <base or arm>", or "objects") that changed colour since the last tick.

        `now` is the wall clock (`time.monotonic`), seconds.

        A chip's first level is not noted: at start every chip is still waiting for its robot.
        """
        # * Keyed by position, not label: combined chips change their label with their colour. Sim chips, which come
        #   and go in front of a part, are left out, so they shift no other chip.
        for index, check in enumerate(check for check in checks if check.label != SIM_LABEL):
            key = (row, index)
            before = self._levels.get(key)
            self._levels[key] = check.level
            if before is None or before == check.level:
                continue
            change = self._changes.setdefault(key, _Change(before, check.level))
            change.check, change.since, change.count = check, now, change.count + 1
            # * The furthest from where it started, the worse on a tie: amber to green and back is "flickered green".
            change.furthest = max(change.furthest, check.level, key=lambda level: (abs(level - change.first), level))

    def _log_changes(self, ctx: PluginContext, now: float) -> None:
        """Log the colour changes that held for LOG_HOLD seconds, one line per row, so outages can be traced later."""
        rows: dict[str, list[_Change]] = {}
        for key, change in list(self._changes.items()):
            if now - change.since >= LOG_HOLD:
                rows.setdefault(key[0], []).append(self._changes.pop(key))
        for row, changes in rows.items():
            message = f"{row}: " + "; ".join(change.describe() for change in changes)
            level = max(change.log_level() for change in changes)
            if level == BAD:
                ctx.log_error(message, CHIPS_TOPIC)
            elif level == WARN:
                ctx.log_warn(message, CHIPS_TOPIC)
            else:
                ctx.log_info(message, CHIPS_TOPIC)

    def draw(self, ctx: PluginContext) -> None:
        """Copy this tick's checks into the banner and the rows.

        Args:
            ctx: This plugin's context.
        """
        stops = {serial: stop for serial in self._robot_checks
                 if (stop := soft_stop_check(ctx.world.robots[serial])) is not None}
        # * The banner: the soft stop first, so it heads the robot's hover text.
        banner = render_banner({serial: [*([("", [stops[serial]])] if serial in stops else []), *lines]
                                for serial, lines in self._robot_checks.items()}, self._object_checks)
        self._banner.content = banner
        rows = []
        for serial, lines in self._robot_checks.items():
            robot = [check for _name, checks in lines for check in checks]
            stop = stops.get(serial)
            # * Next to the name: whatever else is green, nothing moves this robot until Resume.
            heading = serial + (" " + check_chip(stop) if stop is not None else "")
            rows.append(section(heading, LEVEL_COLORS[worst_level(robot + ([stop] if stop is not None else []))])
                        + "".join(render_line(name, checks) for name, checks in lines))
            self._robot_rows[serial].content = rows[-1]
        rows.append((section("objects", LEVEL_COLORS[worst_level(self._object_checks)])
                     + render_chips(self._object_checks)) if self._object_checks else "")
        self._objects_row.content = rows[-1]
        self._phone_banner.content = phone_text(banner)
        self._phone_rows.content = phone_text("".join(rows))


#: Chip colours by level, for the log.
LEVEL_NAMES = {INFO: "blue", GOOD: "green", WARN: "amber", BAD: "red"}


@dataclass
class _Change:
    """A chip's colour changes since its last logged colour `first`, the last at time `since`, to `check`'s.

    `furthest` is the colour furthest from `first` it had, the worse on a tie.
    """

    first: int
    furthest: int
    check: Check | None = None
    since: float = 0.0
    count: int = 0

    def describe(self) -> str:
        """E.g. "mocap green -> amber (2 changes, red for a moment): <detail>" or "mocap flickered red, green again"."""
        label, last = self.check.label, self.check.level
        changes = f" ({self.count} changes)" if self.count > 1 else ""
        if last == self.first:
            return f"{label} flickered {LEVEL_NAMES[self.furthest]}{changes}, {LEVEL_NAMES[last]} again"
        if self._red_for_a_moment():
            changes = f" ({self.count} changes, red for a moment)"
        return f"{label} {LEVEL_NAMES[self.first]} -> {LEVEL_NAMES[last]}{changes}: {self.check.detail}"

    def log_level(self) -> int:
        """BAD for an error (turned red), WARN for a warning (turned worse, or red for a moment), else GOOD."""
        last = self.check.level
        if last == self.first:
            return WARN if self.furthest == BAD else GOOD
        if last == BAD:
            return BAD
        return WARN if last > self.first or self._red_for_a_moment() else GOOD

    def _red_for_a_moment(self) -> bool:
        """Whether it was red in between, neither starting nor ending red."""
        return self.furthest == BAD and BAD not in (self.first, self.check.level)


# --- --- --- --- --- CHECKS (`now` is ROS time, seconds) --- --- --- --- ---

def robot_checks(robot: HuskyRobotInterface, now: float, tcp_checks: dict[tuple[str, str], Check] | None = None,
                 calibration: AppliedCalibration | None = None,
                 frame: FloorCalibrationState | None = None) -> list[tuple[str, list[Check]]]:
    """Every check for one robot: a line for the base, then one per arm with its tool and TCP.

    Args:
        robot: The robot to check.
        now: Current ROS time, seconds.
        tcp_checks: Each arm's latest `tcp_check` by (serial, arm name); an arm without one gets an amber chip.
        calibration: The base calibration the relay applies to its rigid body, or None; built for an earlier body
            definition, it turns the mocap chip amber, as a high marker error does.
        frame: The floor calibration the relay applies, or None; with poses off (`poses_off`), the mocap chip turns
            amber.

    Returns:
        list[tuple[str, list[Check]]]: (line name, one check per chip), in display order.
    """
    state = robot.base.state
    base = state.joint_states_stats.quality(now)
    # * Each part's sim chip comes first, then its own chips: base; arm up to its tcp; then the tool.
    lines = [("base", simulated_check(robot.base.simulation)
              + [stream_check("signal", base, BASE_RATE) if base.seen else
                 Check("signal", WARN, "no platform joint_states yet"),
                 in_frame(calibrated(mocap_check("mocap", robot.config.mocap_id, state, now), calibration),
                          frame),
                 estop_check(state),
                 battery_check(state, now),
                 controller_check("ctrl", state.controllers, now)])]
    for arm in robot.arms.values():
        checks = simulated_check(arm.simulation) + [
            stream_check("signal", arm.state.joint_states_stats.quality(now), ARM_RATE), arm_check(arm, now)]
        checks.append((tcp_checks or {}).get((robot.config.serial, arm.config.name),
                                             Check("tcp", WARN, "not checked yet: waiting for the arm to be still")))
        # * Only when out of date: the monitor uses the calibration, but the URDF file other tools read does not.
        stale = [c for c in robot.config.ur_calibration if c.arm == arm.config.name and c.out_of_date]
        if stale:
            checks.append(Check("URDF", WARN, stale[0].describe()))
        # * The tool gets its own chips: its driver fails independently of the arm.
        tool = tool_check(arm, now)
        if tool is not None:
            checks += simulated_check(robot.simulated.get(arm.config.end_effector_namespace)) + [tool]
        lines.append((arm.config.label.lower(), checks))
    return lines


def applied_calibration(relay: MocapRelayState | None, robot: HuskyRobotInterface) -> AppliedCalibration | None:
    """What the relay applies to the robot's base body (a fake Motive's from the simulated sessions); a truth none."""
    if relay is None or relay.source(robot.config.mocap_id) == TRUTH:
        return None
    return relay.calibrations.get(robot.config.mocap_id)


def applied_frame(relay: MocapRelayState | None, robot: HuskyRobotInterface) -> FloorCalibrationState | None:
    """The floor calibration the relay applies to the robot's base body; none to a simulator's truth (in the lab)."""
    if relay is None or relay.source(robot.config.mocap_id) == TRUTH:
        return None
    return relay.frame


def calibrated(mocap: Check, calibration: AppliedCalibration | None) -> Check:
    """The base's mocap chip, at least amber while the relay finds its calibration built for an earlier body."""
    if calibration is None or calibration.definition_status != OUT_OF_DATE:
        return mocap
    return Check(mocap.label, max(mocap.level, WARN),
                 f"{mocap.detail}; calibration {calibration.definition_detail}: a few mm or degrees off, calibrate "
                 f"again")


def in_frame(mocap: Check, frame: FloorCalibrationState | None) -> Check:
    """The base's mocap chip, at least amber while poses may be off the floor calibration's frame (`poses_off`)."""
    if frame is None or not frame.poses_off:
        return mocap
    if frame.error and not frame.floor_calibration:
        why = f"the relay could not read its floor calibration ({frame.error})"
    elif frame.status == FRAME_UNCHECKED:
        why = "the relay has not checked Motive's frame against the floor calibration yet (no anchors or truss cameras)"
    elif frame.status in (FRAME_REFIT_PENDING, FRAME_UNTRUSTED):
        why = (f"Motive's frame moved since the floor calibration ({frame.pending or frame.status}): poses are off "
               f"until the refit is used (mocap probe, Use refit)")
    else:
        why = (f"a newer floor calibration ({frame.newer}) is not applied yet: poses move by it when it is (mocap "
               f"probe, Apply floor calibration)")
    return Check(mocap.label, max(mocap.level, WARN), f"{mocap.detail}; {why}")


#: The label of a simulated part's chip.
SIM_LABEL = "sim"


def simulated_check(simulation: SimulatedPart | None) -> list[Check]:
    """A blue "sim" chip while a part announces itself as simulated, else none: checks on it are relaxed."""
    if simulation is None:
        return []
    return [Check(SIM_LABEL, INFO,
                  f"{simulation.kind} ({simulation.node}): simulated, so some safety checks are skipped")]


def stream_check(label: str, quality: StreamQuality, rate: float = ARM_RATE) -> Check:
    """Whether joint_states arrive at their rate, without gaps, and with stamps the monitor's clock agrees with.

    ! Only thresholds in the text, never the measured values: they change every tick and would
      rebuild the row, closing its tooltips.

    Args:
        label: Chip text, e.g. the arm's name.
        quality: The stream's recent rate, largest gap and delay.
        rate: The stream's nominal rate, Hz.

    Returns:
        Check: BAD when silent or far off, WARN when a little off, else GOOD.
    """
    if quality.rate is None:
        return Check(label, WARN, f"joint_states seen for under {WINDOW:g}s; still measuring")
    if quality.rate == 0.0:
        return Check(label, BAD, f"no joint_states in the last {WINDOW:g}s")
    rate_warn, rate_bad = SIGNAL_RATE_WARN * rate, SIGNAL_RATE_BAD * rate
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
    delay = quality.delay
    if delay is not None and delay < SIGNAL_AHEAD_WARN:
        level, limit = (BAD, SIGNAL_AHEAD_BAD) if delay < SIGNAL_AHEAD_BAD else (WARN, SIGNAL_AHEAD_WARN)
        problems.append((level, f"stamps over {-limit * 1e3:g} ms in the future (robot clock ahead; time sync?)"))
    elif delay is not None and delay > SIGNAL_DELAY_WARN:
        level, limit = (BAD, SIGNAL_DELAY_BAD) if delay > SIGNAL_DELAY_BAD else (WARN, SIGNAL_DELAY_WARN)
        problems.append((level, f"every message over {limit * 1e3:g} ms late (robot clock behind, or a slow link)"))
    if problems:
        text = " and ".join(text for _level, text in problems)
        return Check(label, max(level for level, _text in problems), f"{text} in the last {WINDOW:g}s")
    unknown = "; gaps and delay unknown (unstamped)" if quality.delay is None else ""
    return Check(label, GOOD, f"joint_states at {rate_warn:g} Hz or more, no gap over {SIGNAL_GAP_WARN * 1e3:g} ms, "
                              f"clock within {-SIGNAL_AHEAD_WARN * 1e3:g}/{SIGNAL_DELAY_WARN * 1e3:g} ms{unknown}")


def arm_check(arm: ArmInterface, now: float) -> Check:
    """Every check of one arm, folded into one chip labelled with its worst part.

    Args:
        arm: The arm.
        now: Current ROS time, seconds.
    """
    state = arm.state
    parts = [sync_check(state, now)]
    # Dashboard parts are known only while the sync's status is fresh; a simulated arm has none (its SIM chip says so).
    if parts[0].level < BAD and arm.simulation is None:
        parts += [dashboard_check(state)]
        if state.dashboard_up:
            parts += [safety_check(state.safety_mode), running_check(state)]
    parts += [controller_check("ctrl", state.controllers, now),
              age_check("joints", "joint_states", state.last_update_time, now)]
    return combine("arm", parts)


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
        return Check("bat", WARN, "no bms/state message yet")
    percentage = state.battery_percentage
    known = percentage is not None and math.isfinite(percentage)
    # * 5 % steps, no voltage: a jittering label would rebuild the row.
    label = f"bat {round(percentage * 20) * 5}%" if known else "bat"
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
        return Check("NO DASHBOARD", BAD, "no dashboard_client: the driver is down, so the arm does not move; "
                                          "for fake hardware, launch with use_fake_hardware:=true")
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
        Check | None: Labelled with the tool kind, or None for a bare arm.
    """
    tool = arm.end_effector
    label = arm.config.end_effector
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

def soft_stop_check(robot: HuskyRobotInterface) -> Check | None:
    """A red SOFT STOPPED chip while the robot is soft stopped, the reason as its hover text; None otherwise."""
    latch = robot.soft_stop_latch
    if not latch.stopped:
        return None
    return Check("SOFT STOPPED", BAD, f"{latch.reason or 'soft stopped'}; press Resume to move it again")


def render_banner(robots: dict[str, list[tuple[str, list[Check]]]], objects: list[Check]) -> str:
    """The banner's HTML, one line: an overall chip, then one per robot and one for tracked objects (if any).

    Each chip is red if any of its checks is, else amber if any is, else green; its hover text lists what is not green.

    Args:
        robots: Per serial, its lines as (part name, checks), e.g. from `robot_checks`.
        objects: The tracked objects' checks.
    """
    if not robots and not objects:
        return chip_line(chip("NOTHING TO CHECK", BUSY, "no robots and no tracked objects"))
    groups = {serial.split("-")[-1]: [(f"{name} " if name else "", check) for name, checks in lines
                                      for check in checks]
              for serial, lines in robots.items()}
    if objects:
        groups["objects"] = [("", check) for check in objects]
    every = [(f"{label} {part}", check) for label, checks in groups.items() for part, check in checks]
    level = worst_level([check for _part, check in every])
    chips = chip(BANNER_LABELS[level], LEVEL_COLORS[level], _not_green(every) or "every chip is green")
    for label, checks in groups.items():
        chips += chip(label, LEVEL_COLORS[worst_level([check for _part, check in checks])],
                      _not_green(checks) or "all green")
    return chip_line(chips)


#: The overall chip's text by level.
BANNER_LABELS = {GOOD: "ALL GREEN", WARN: "CHECK", BAD: "PROBLEMS"}


def _not_green(checks: list[tuple[str, Check]]) -> str:
    """One hover line per check that is not green: "<part> <label>: <detail>", worst first."""
    shown = sorted((item for item in checks if item[1].level > GOOD), key=lambda item: -item[1].level)
    return "\n".join(f"{part}{check.label}: {check.detail}" for part, check in shown)


def worst_level(checks: list[Check]) -> int:
    """The most severe level among `checks`, GOOD when there are none."""
    return max((check.level for check in checks), default=GOOD)


def render_chips(checks: list[Check]) -> str:
    """One row's HTML: a chip per check, its detail in the tooltip."""
    return block("".join(check_chip(check) for check in checks))


def render_line(name: str, checks: list[Check]) -> str:
    """One line of a robot's row, one line high: its part's name in gray, then a chip per check."""
    label = (f'<span style="display:inline-block;width:42px;font-size:11px;color:{NONE}">'
             f'{name}</span>')
    return chip_line(label + "".join(check_chip(check) for check in checks))
