"""
What the observer records every tick: continuous values only.

A state that switches (a flag, a mode, a controller, a motor state) is an event instead (`watch.py`).

Signal names start with the robot's short serial ("0806_base_pose", "0806_ur_arm_joints"), a tracked object's with
"object_" ("object_B20_pose"), so everything fits one trace. Every value is the newest at the tick: one row per tick.
The names and channels the viewer reads are in `names.py`.

! A tick signal that raises fails the observer's tick: every read here goes through `_safe`.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, Callable, Mapping

import numpy as np

from ...config import short_serial
from ...plugin_api.trace import Signal
from ...robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER
from ...robot_interface.end_effectors import RobotiqState, ScaffoldingV3State
from .names import (BASE_MOCAP, BASE_POSE, BASE_STREAMS, BATTERY, FOLLOWER, FOLLOWER_CHANNELS, JOINT_CHANNELS,
                    JOINTS, LATE_SHARE, NOISE, OBJECT_MOCAP, OBJECT_POSE, ODOMETRY, POSE, RELAY_DRIFT, STREAM, STREAMS,
                    TARGET, TCP, TOOL, TWIST_SENT, WRENCH, WRENCH_PEAK, object_key, signal)

if TYPE_CHECKING:
    from ...robot_interface.arm import ArmInterface
    from ...robot_interface.mocap_relay import MocapRelayInterface
    from ...robot_interface.mocap_state import MocapBody
    from ...robot_interface.robot import HuskyRobotInterface
    from ...robot_interface.stream_stats import StreamStats

POSE_UNITS = ("m",) * 3 + ("",) * 4
NOISE_UNITS = ("m", "", "", "m")
#: A stream's channels' units (`StreamQuality`).
STREAM_UNITS = ("1/s", "s", "s")
#: Seconds between two readings of a stream's quality: it covers the last 2 s, so more often adds nothing but load.
STREAM_PERIOD = 0.5
#: A twist older than this no longer moves the base (its cmd_vel timeout), seconds: its column is then unknown.
TWIST_HELD = 0.3


def robot_signals(robot: HuskyRobotInterface, now: Callable[[], float],
                  twists: Mapping[str, tuple[float, float, float]],
                  peak: Callable[[str, str], tuple | None]) -> list[Signal]:
    """Every tick signal of one robot.

    Args:
        robot: The robot.
        now: ROS time, seconds.
        twists: Serial -> the last twist sent (v, w, ROS time), kept by the observer's listener.
        peak: (serial, arm) -> the largest |force| and |torque| read since the last call, or None for no reading.
    """
    serial, base = robot.config.serial, robot.base
    prefix = short_serial(serial)
    state = base.state
    signals = [
        Signal(signal(prefix, BASE_POSE), _safe(lambda: _pose(state.position, state.orientation)), POSE, POSE_UNITS),
        Signal(signal(prefix, BASE_MOCAP), _safe(lambda: _noise(state)), NOISE, NOISE_UNITS),
        # * `x`, `y`, `yaw`: the pose the follower steered from, as it got it over wifi, to compare with the base's.
        Signal(signal(prefix, FOLLOWER), _safe(lambda: _follower(state.follower)), FOLLOWER_CHANNELS,
               ("", "m", "m", "rad", "m/s", "rad/s", "m", "m", "rad", "m", "m", "rad", "m", "m")),
        Signal(signal(prefix, ODOMETRY), _safe(lambda: None if state.odometry is None else state.odometry.velocity),
               ("v", "w"), ("m/s", "rad/s")),
        Signal(signal(prefix, TWIST_SENT), _safe(lambda: _twist(twists.get(serial), now())), ("v", "w"),
               ("m/s", "rad/s")),
        Signal(signal(prefix, BATTERY), _safe(lambda: _battery(state)), ("charge", "voltage"), ("", "V")),
    ]
    streams = dict(zip(BASE_STREAMS, (state.joint_states_stats, state.mocap_stats)))
    streams.update({name: arm.state.joint_states_stats for name, arm in robot.arms.items()})
    labels = tuple(f"{name}_{channel}" for name in streams for channel in STREAM)
    signals.append(Signal(signal(prefix, STREAMS), _safe(_Every(STREAM_PERIOD, now, lambda: _streams(streams, now()))),
                          labels, STREAM_UNITS * len(streams)))
    for name, arm in robot.arms.items():
        signals += arm_signals(signal(prefix, name), arm, lambda name=name: peak(serial, name))
    return signals


def arm_signals(prefix: str, arm: ArmInterface, peak: Callable[[], tuple | None]) -> list[Signal]:
    """Every tick signal of one arm, named "<prefix>_<what>"; the tool's only for a tool with a continuous value."""
    state = arm.state
    signals = [
        Signal(signal(prefix, JOINTS), _safe(arm.joint_vector), JOINT_CHANNELS, "rad"),
        Signal(signal(prefix, TCP), _safe(lambda: _pose(state.tcp_position, state.tcp_orientation)), POSE, POSE_UNITS),
        Signal(signal(prefix, WRENCH), _safe(lambda: state.wrench), ("fx", "fy", "fz", "tx", "ty", "tz"),
               ("N",) * 3 + ("N m",) * 3),
        # * Over every reading since the last tick (up to the robot's rate limiter, 50 Hz), so a short peak shows.
        Signal(signal(prefix, WRENCH_PEAK), _safe(peak), ("force", "torque"), ("N", "N m")),
        Signal(signal(prefix, TARGET), _safe(lambda: _target(state)), POSE, POSE_UNITS),
    ]
    tool = state.end_effector
    if isinstance(tool, ScaffoldingV3State):
        signals.append(Signal(signal(prefix, TOOL), _safe(lambda: (_number(tool.current), _number(tool.pwm_pct))),
                              ("current", "pwm"), ("", "%")))
    elif isinstance(tool, RobotiqState):
        signals.append(Signal(signal(prefix, TOOL), _safe(lambda: tool.position), ("position",), "rad"))
    return signals


def object_signals(objects: Mapping[str, MocapBody]) -> list[Signal]:
    """Each tracked object's pose and how well mocap sees it, named "object_<name>_pose" and "object_<name>_mocap"."""
    signals, keys = [], set()
    for name, body in objects.items():
        key = object_key(name)
        # * Two names alike once made safe ("B-20", "B 20") stay apart by their mocap id.
        key = signal(key, str(body.mocap_id)) if key in keys else key
        keys.add(key)
        signals += [Signal(signal(key, OBJECT_POSE), _safe(lambda body=body: _pose(body.position, body.orientation)),
                           POSE, POSE_UNITS),
                    Signal(signal(key, OBJECT_MOCAP), _safe(lambda body=body: _noise(body)), NOISE, NOISE_UNITS)]
    return signals


def relay_signals(relay: MocapRelayInterface | None, now: Callable[[], float]) -> list[Signal]:
    """The relay's frame drift by its last look, and how old that look is; none without a relay."""
    if relay is None:
        return []
    labels = ("off", "anchors_moved", "cameras_moved", "apart", "scale_ppm", "anchors_residual")
    units = ("m", "m", "m", "m", "ppm", "m")
    return [Signal(RELAY_DRIFT, _safe(lambda: _drift(relay)), labels, units)]


def monitor_signals(late_share: Callable[[], float]) -> list[Signal]:
    """The monitor's own health: the share of late ticks over the last minute."""
    return [Signal(LATE_SHARE, _safe(late_share))]


# --- --- --- --- --- READERS --- --- --- --- ---

class _Every:
    """A reader called at most every `period` seconds; in between, its last value again."""

    def __init__(self, period: float, now: Callable[[], float], read: Callable[[], Any]):
        """Read with `read`, at most every `period` seconds of `now`."""
        self._period, self._now, self._read = period, now, read
        self._at, self._value = -math.inf, None

    def __call__(self) -> Any:
        """The newest value, read again once `period` passed."""
        if self._now() - self._at >= self._period:
            self._at, self._value = self._now(), self._read()
        return self._value


def _safe(read: Callable[[], Any]) -> Callable[[], Any]:
    """`read`, giving None (unknown) instead of raising."""
    def guarded() -> Any:
        try:
            return read()
        except Exception:
            return None
    return guarded


def _pose(position, orientation) -> tuple | None:
    """A pose as 7 numbers, or None without a position."""
    if position is None or orientation is None:
        return None
    return (*position, *orientation)


def _number(value: float | None) -> float:
    """A number, NaN for unknown."""
    return math.nan if value is None else float(value)


def _battery(state) -> tuple[float, float]:
    """The battery's charge (0 to 1) and voltage; NaN where unknown."""
    return _number(state.battery_percentage), _number(state.battery_voltage)


def _noise(body: MocapBody) -> tuple | None:
    """Motive's mean marker error, the markers labeled, how many filled in, the largest residual; None before one."""
    markers = body.markers
    if markers is None:
        return None if body.marker_error is None else (body.marker_error, math.nan, math.nan, math.nan)
    residual = math.nan if markers.residuals is None or not len(markers.residuals) else \
        float(np.max(markers.residuals))
    return (_number(body.marker_error), float(len(markers.ids)), float(np.count_nonzero(~markers.measured)), residual)


def _follower(follower) -> tuple | None:
    """The follower's progress, errors, command, the pose it used, its reference and lookahead; None before one."""
    if follower is None:
        return None
    pose = follower.pose if follower.pose is not None else (math.nan,) * 3
    return (follower.progress, follower.cross_track_error, follower.along_track_error, follower.yaw_error,
            *follower.command, *pose, *follower.reference, *follower.lookahead)


def _twist(sent: tuple[float, float, float] | None, now: float) -> tuple | None:
    """The twist still moving the base, or None: none sent, or sent over TWIST_HELD ago."""
    return None if sent is None or now - sent[2] > TWIST_HELD else (sent[0], sent[1])


def _target(state) -> tuple | None:
    """The Cartesian target the arm holds: the last one sent while the compliance controller runs, else None."""
    if state.controllers.active != CARTESIAN_COMPLIANCE_CONTROLLER:
        return None
    return _pose(state.sent_target_position, state.sent_target_orientation)


def _streams(streams: Mapping[str, StreamStats], now: float) -> list[float]:
    """Every stream's rate, largest gap and delay over the last few seconds; NaN where unknown."""
    values = []
    for stats in streams.values():
        quality = stats.quality(now)
        values += [_number(value) for value in (quality.rate, quality.max_gap, quality.delay)]
    return values


def _drift(relay: MocapRelayInterface) -> tuple | None:
    """The relay's drift numbers by its last look (`FrameDrift`), or None before one."""
    drift = relay.state.drift
    if drift is None:
        return None
    return tuple(_number(value) for value in (drift.off, drift.anchors_moved, drift.cameras_moved, drift.apart,
                                              drift.scale_ppm, drift.anchors_residual))
