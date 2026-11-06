"""
The observer's events: what changed since the last tick, and every discrete command sent, as plain JSON dicts.

`Watcher.look` compares each robot's state with the last tick's; the listeners (`attach`) note every command the
monitor sends, whoever sent it. A motion says who held the robot's lease when it started and whether anything was
sent: a move with neither came from outside the monitor (free drive, the pendant, a push, a joystick).

- ! Main thread only: `look` from a hook, the listeners fire from the command senders.
- Each event is {"event", "ros_time", ...}; the robot's serial is "robot" where one applies. A command sent in full
  (a trajectory, a path) carries its message under FILE_KEY, for the plugin to write to a file of its own.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Callable, Iterable

import numpy as np
from scipy.spatial.transform import Rotation

from ...robot_interface.end_effectors import RobotiqState, ScaffoldingV1State, ScaffoldingV3, ScaffoldingV3State

if TYPE_CHECKING:
    from ...robot_interface.mocap_relay import MocapRelayInterface
    from ...robot_interface.mocap_state import MocapBody
    from ...robot_interface.robot import HuskyRobotInterface
    from ...world.clearance import ClearanceTrip

#: An arm moves while a joint moved slightly this recently (`ArmState.last_slight_motion_time`), seconds.
ARM_STILL_AFTER = 0.5
#: A base moves while it went this far within BASE_WINDOW: metres, radians.
BASE_MOVE, BASE_TURN = 0.01, math.radians(1.0)
BASE_WINDOW = 0.5
#: A command sent this recently, seconds, counts as the cause of a motion starting.
COMMAND_RECENT = 1.0
#: A stream counts as lost after this long without a message, and back once it ran this long again, seconds.
STREAM_LOST, STREAM_BACK = 1.0, 2.0
#: Mocap tracking counts as lost once it stayed lost this long, and back once it held this long, seconds: a grasp's
#: flicker is counted, not told line by line.
TRACKING_LOST, TRACKING_BACK = 0.5, 0.2
#: The relay's drift is told again when it changed this much: metres, ppm.
DRIFT_STEP, SCALE_STEP = 0.0005, 100.0
#: An event's key for a message sent in full (the message itself), which the plugin writes to a file of its own.
FILE_KEY = "_message"


@dataclass
class _Flag:
    """A good or bad state told only once it held: bad after `bad_after`, good again after `good_after` seconds.

    Each change says how often the state flickered without being told since the last one; a return to good also says
    how long it was bad.
    """

    bad_after: float
    good_after: float
    told: bool | None = None
    told_at: float = 0.0
    since: float | None = None
    flickers: int = 0

    def update(self, good: bool | None, now: float) -> dict | None:
        """The change to tell (value "lost" or "back"), or None; `good` None (unknown yet) changes nothing."""
        if good is None:
            return None
        if self.told is None:
            self.told, self.told_at = good, now
            return None
        if good == self.told:
            self.flickers += self.since is not None
            self.since = None
            return None
        self.since = now if self.since is None else self.since
        if now - self.since < (self.good_after if good else self.bad_after):
            return None
        # * `since`: when the change began; the event comes once it held.
        change = {"value": "back" if good else "lost", "since": self.since, "flickers": self.flickers}
        if good:
            change["lost_for"] = round(self.since - self.told_at, 3)
        self.told, self.told_at, self.since, self.flickers = good, self.since, None, 0
        return change


@dataclass
class _Moving:
    """A motion going on: when it started, where from, and what started it."""

    since: float
    start: Any
    by: str | None
    commanded: bool


@dataclass
class _RobotSeen:
    """What `look` saw of one robot last tick."""

    status: dict = field(default_factory=dict)
    flags: dict[str, _Flag] = field(default_factory=dict)
    arms_moving: dict[str, _Moving | None] = field(default_factory=dict)
    base_moving: _Moving | None = None
    base_poses: deque = field(default_factory=deque)


class Watcher:
    """Turns the robots' and the relay's changes into events, and keeps the commands the listeners heard.

    Attributes:
        twists: Serial -> the last twist sent (v, w, ROS time), for the twist column.
    """

    def __init__(self, robots: dict[str, HuskyRobotInterface], relay: MocapRelayInterface | None,
                 now: Callable[[], float], objects: dict[str, MocapBody] | None = None):
        """Watch `robots`, `relay` and the tracked `objects` (read live); nothing is told before the first `look`."""
        self._robots, self._relay, self._now = robots, relay, now
        self._objects = objects if objects is not None else {}
        self._objects_seen: dict[str, _Flag] | None = None
        #: (serial, arm) -> the largest |force| and |torque| read since the last `take_peak`.
        self._peaks: dict[tuple[str, str], list[float]] = {}
        self._seen = {serial: _RobotSeen() for serial in robots}
        self._relay_seen: dict = {}
        self._trips_seen: set[tuple] = set()
        #: (serial, arm or None) -> ROS time the last command sent to it runs until, and who held the lease then.
        self._commanded: dict[tuple[str, str | None], tuple[float, str | None]] = {}
        self._relay_drift: dict | None = None
        self._heard: list[dict] = []
        self._last_wrench: dict[tuple[str, str], tuple] = {}
        self._listeners: list[tuple[Any, Callable]] = []
        self.twists: dict[str, tuple[float, float, float]] = {}

    # --- --- --- --- --- LISTENERS --- --- --- --- ---

    def attach(self) -> None:
        """Listen to every command event of every robot."""
        for serial, robot in self._robots.items():
            base = robot.base
            self._listen(base.twists_sent, lambda sent, s=serial: self._on_twist(s, sent))
            self._listen(base.paths_sent, lambda sent, s=serial: self._on_path(s, sent))
            for name, arm in robot.arms.items():
                self._listen(arm.trajectories_sent, lambda sent, s=serial, a=name: self._on_trajectory(s, a, sent))
                self._listen(arm.targets_sent, lambda sent, s=serial, a=name: self._note(s, a, sent.update_time))
                self._listen(arm.wrenches_sent, lambda sent, s=serial, a=name: self._on_wrench(s, a, sent))
                self._listen(arm.wrench_messages, lambda reading, s=serial, a=name: self._on_reading(s, a, reading))
                self._listen(arm.ft_zeros_sent, lambda sent, s=serial, a=name: self._on_zero(s, a, sent))
                if arm.end_effector is not None:
                    self._listen(arm.end_effector.commands_sent,
                                 lambda sent, s=serial, a=name: self._on_tool(s, a, sent))

    def detach(self) -> None:
        """Stop listening."""
        for event, listener in self._listeners:
            event.detach(listener)
        self._listeners.clear()

    def _listen(self, event, listener: Callable) -> None:
        """Attach `listener` to `event`, guarded: a listener runs inside the sender and must never raise."""
        def guarded(record) -> None:
            try:
                listener(record)
            except Exception:
                pass
        event.attach(guarded)
        self._listeners.append((event, guarded))

    def take_peak(self, serial: str, arm: str) -> tuple[float, float] | None:
        """The largest |force| and |torque| read on an arm since the last call, or None for no reading."""
        peak = self._peaks.pop((serial, arm), None)
        return None if peak is None else (peak[0], peak[1])

    def _on_reading(self, serial: str, arm: str, reading) -> None:
        """One force/torque reading: kept in the arm's peak."""
        force, torque = float(np.linalg.norm(reading.wrench[:3])), float(np.linalg.norm(reading.wrench[3:]))
        peak = self._peaks.setdefault((serial, arm), [force, torque])
        peak[0], peak[1] = max(peak[0], force), max(peak[1], torque)

    def _on_zero(self, serial: str, arm: str, sent) -> None:
        """The force sensor zeroed, with the reading it tares away."""
        self._heard.append({"event": "force sensor zeroed", "ros_time": sent.update_time, "robot": serial, "arm": arm,
                            "by": self._holder(serial), "wrench_before": _numbers(sent.wrench)})

    def _holder(self, serial: str) -> str | None:
        """Who holds a robot's lease now, the sender of a command being sent."""
        lease = self._robots[serial].lease_slot.lease
        return None if lease is None else lease.holder

    def _note(self, serial: str, arm: str | None, when: float, lasts: float = 0.0) -> str | None:
        """Note a command sent to a base (arm None) or an arm, running `lasts` seconds; returns the lease holder."""
        holder = self._holder(serial)
        self._commanded[(serial, arm)] = (when + lasts, holder)
        return holder

    def _on_twist(self, serial: str, sent) -> None:
        """A twist: kept for the column, noted as a command."""
        self.twists[serial] = (float(sent.command[0]), float(sent.command[1]), sent.update_time)
        self._note(serial, None, sent.update_time)

    def _on_path(self, serial: str, sent) -> None:
        """A path, or a stop (no waypoints)."""
        holder = self._note(serial, None, sent.update_time)
        # * A stop is an empty path: nothing more to keep than this line.
        self._heard.append({"event": "path sent" if sent.waypoints else "path stop sent", "ros_time": sent.update_time,
                            "robot": serial, "by": holder, "path_id": sent.path_id, "waypoints": sent.waypoints,
                            "timed": bool(sent.timed), "speed_caps": _numbers(sent.speed_caps),
                            **(_full(sent.message) if sent.waypoints else {})})

    def _on_trajectory(self, serial: str, arm: str, sent) -> None:
        """A joint trajectory, or the braking one of a hold."""
        holder = self._note(serial, arm, sent.update_time, float(sent.duration))
        # * A hold or a one-point step (a jog, every tick) is all in its target: no file of its own.
        self._heard.append({"event": "hold sent" if sent.hold else "trajectory sent", "ros_time": sent.update_time,
                            "robot": serial, "arm": arm, "by": holder, "duration": float(sent.duration),
                            "waypoints": int(sent.waypoints), "target": _numbers(sent.target),
                            **({} if sent.hold or sent.waypoints <= 1 else _full(sent.message))})

    def _on_wrench(self, serial: str, arm: str, sent) -> None:
        """A target wrench, told only when it changed."""
        values = tuple(_numbers(sent.force) + _numbers(sent.torque))
        if self._last_wrench.get((serial, arm)) == values:
            return
        self._last_wrench[(serial, arm)] = values
        self._heard.append({"event": "wrench sent", "ros_time": sent.update_time, "robot": serial, "arm": arm,
                            "by": self._holder(serial), "force": list(values[:3]), "torque": list(values[3:])})

    def _on_tool(self, serial: str, arm: str, sent) -> None:
        """A tool command."""
        holder = self._note(serial, arm, sent.update_time)
        self._heard.append({"event": "tool command sent", "ros_time": sent.update_time, "robot": serial, "arm": arm,
                            "by": holder, "command": sent.command, **sent.values})

    # --- --- --- --- --- LOOKING --- --- --- --- ---

    def look(self, trips: Iterable[ClearanceTrip] = ()) -> tuple[list[dict], list[str]]:
        """Every event since the last call: the commands heard, then what changed, oldest first; and the errors.

        Each robot, the relay, the objects and the trips are looked at on their own: one that raises leaves the others'
        events in, and its error (repr) in the second list.
        """
        now = self._now()
        events, self._heard, errors = self._heard, [], []
        parts = [lambda serial=serial, robot=robot: self._robot_changes(serial, robot, self._seen[serial], now)
                 for serial, robot in self._robots.items()]
        parts += [lambda: self._relay_changes(now), lambda: self._object_changes(now), lambda: self._trips(trips)]
        for part in parts:
            try:
                events += part()
            except Exception as error:
                errors.append(repr(error))
        return events, errors

    def snapshot(self) -> tuple[list[dict], list[str]]:
        """Every state now as "state" events (robots, relay, objects' tracking); changes are told against these.

        Guarded part by part as `look`: returns the events and the errors (repr).
        """
        now, events, errors = self._now(), [], []
        parts = [lambda serial=serial, robot=robot: [self._robot_state(serial, robot, now)]
                 for serial, robot in self._robots.items()]
        parts += [lambda: self._relay_state(now), lambda: self._objects_state(now)]
        for part in parts:
            try:
                events += part()
            except Exception as error:
                errors.append(repr(error))
        return events, errors

    def _robot_state(self, serial: str, robot: HuskyRobotInterface, now: float) -> dict:
        """One robot's states now, its motions under way included; the baseline of later changes."""
        seen = self._seen[serial]
        seen.status = _status(robot, now)
        flags = {key: flag.told for key, flag in seen.flags.items()} or _flags(robot, now)
        moving = {name: was is not None for name, was in seen.arms_moving.items()}
        moving["base"] = seen.base_moving is not None
        return {"event": "state", "ros_time": now, "robot": serial, "status": seen.status, "flags": flags,
                "moving": moving}

    def _relay_state(self, now: float) -> list[dict]:
        """The relay's states and drift now, as the baseline of later changes; none without a relay."""
        if self._relay is None:
            return []
        current = self._relay_current()
        self._relay_seen.update(current)
        drift = self._relay.state.drift
        self._relay_drift = None if drift is None else _drift_values(drift)
        return [{"event": "relay state", "ros_time": now, **current, "drift": self._relay_drift}]

    def _objects_state(self, now: float) -> list[dict]:
        """Each tracked object's mocap tracking now; none without objects."""
        if not self._objects:
            return []
        return [{"event": "objects state", "ros_time": now, "tracked": {
            name: body.tracked if body.last_update_time is not None else None for name, body in self._objects.items()}}]

    def _trips(self, trips: Iterable[ClearanceTrip]) -> list[dict]:
        """The clearance trips not told yet, each with every pair and its contact."""
        events = []
        for trip in trips:
            key = (trip.time, trip.serial, trip.task)
            if key not in self._trips_seen:
                self._trips_seen.add(key)
                events.append({"event": "clearance trip", "ros_time": trip.time, "robot": trip.serial,
                               "task": trip.task, "problem": trip.problem,
                               "hit": None if trip.hit is None else trip.hit.as_record()})
        return events

    def _robot_changes(self, serial: str, robot: HuskyRobotInterface, seen: _RobotSeen, now: float) -> list[dict]:
        """One robot's changes: its status, its streams, its arms' and base's motions."""
        events: list[dict] = []
        status = _status(robot, now)
        for key, value in status.items():
            old = seen.status.get(key, value if not seen.status else None)
            if value != old:
                events.append(_changed(key, value, old, serial, now))
        seen.status = status
        for key, good in _flags(robot, now).items():
            flag = seen.flags.setdefault(key, _Flag(*((TRACKING_LOST, TRACKING_BACK) if key == "mocap tracking"
                                                      else (0.0, STREAM_BACK))))
            change = flag.update(good, now)
            if change is not None:
                events.append({"event": key, "ros_time": now, "robot": serial, **change})
        holder = status["lease"]
        for name, arm in robot.arms.items():
            moving = arm.state.last_slight_motion_time is not None and \
                now - arm.state.last_slight_motion_time < ARM_STILL_AFTER
            was = seen.arms_moving.get(name)
            joints = arm.joint_vector()
            if moving and was is None:
                commanded = self._sent_since(serial, name, now - COMMAND_RECENT)
                by = holder or self._sent_by(serial, name, commanded)
                seen.arms_moving[name] = _Moving(now, joints, by, commanded)
                events.append({"event": "arm moving", "ros_time": now, "robot": serial, "arm": name, "by": by,
                               "commanded": commanded, "cause": _cause(by, commanded), "joints": _numbers(joints)})
            elif not moving and was is not None:
                seen.arms_moving[name] = None
                moved = None if joints is None or was.start is None else float(np.max(np.abs(joints - was.start)))
                events.append({"event": "arm still", "ros_time": now, "robot": serial, "arm": name,
                               "seconds": now - was.since, "largest_joint_change": moved,
                               "commanded": was.commanded or self._sent_since(serial, name, was.since),
                               "joints": _numbers(joints)})
        events += self._base_motion(serial, robot, seen, holder, now)
        return events

    def _base_motion(self, serial: str, robot: HuskyRobotInterface, seen: _RobotSeen, holder: str | None,
                     now: float) -> list[dict]:
        """The base starting or stopping, by its mocap pose over the last BASE_WINDOW."""
        state = robot.base.state
        if state.position is None or not state.tracked:
            return []
        pose = (np.array(state.position, dtype=float), np.array(state.orientation, dtype=float))
        poses = seen.base_poses
        poses.append((now, pose))
        while poses and poses[0][0] < now - BASE_WINDOW:
            poses.popleft()
        oldest = poses[0][1]
        moving = _apart(oldest, pose)
        if moving and seen.base_moving is None:
            # * A path runs as long as the follower follows it, pauses between pieces included.
            following = state.follower is not None and state.follower.state == "following"
            commanded = following or self._sent_since(serial, None, now - COMMAND_RECENT)
            by = holder or self._sent_by(serial, None, commanded)
            seen.base_moving = _Moving(now, oldest, by, commanded)
            return [{"event": "base moving", "ros_time": now, "robot": serial, "by": by, "commanded": commanded,
                     "cause": _cause(by, commanded), "pose": _pose(oldest)}]
        if not moving and seen.base_moving is not None:
            was, seen.base_moving = seen.base_moving, None
            distance = float(np.linalg.norm(pose[0][:2] - was.start[0][:2]))
            return [{"event": "base still", "ros_time": now, "robot": serial, "seconds": now - was.since,
                     "distance": distance, "turn": _turn(was.start, pose),
                     "commanded": was.commanded or self._sent_since(serial, None, was.since), "pose": _pose(pose)}]
        return []

    def _sent_since(self, serial: str, arm: str | None, since: float) -> bool:
        """Whether anything was sent to the robot's base (arm None) or that arm since `since`."""
        return self._commanded.get((serial, arm), (-math.inf, None))[0] >= since

    def _sent_by(self, serial: str, arm: str | None, commanded: bool) -> str | None:
        """Who held the lease when the last command went to the base or arm, if one went recently."""
        return self._commanded.get((serial, arm), (0.0, None))[1] if commanded else None

    def _object_changes(self, now: float) -> list[dict]:
        """Tracked objects added or removed, and each one's mocap tracking switching."""
        events = []
        first, flags = self._objects_seen is None, self._objects_seen or {}
        for name in self._objects.keys() - flags.keys():
            flags[name] = _Flag(TRACKING_LOST, TRACKING_BACK)
            if not first:
                events.append({"event": "object tracked from now on", "ros_time": now, "object": name,
                               "mocap_id": self._objects[name].mocap_id})
        for name in flags.keys() - self._objects.keys():
            del flags[name]
            events.append({"event": "object no longer tracked", "ros_time": now, "object": name})
        for name, flag in flags.items():
            body = self._objects[name]
            change = flag.update(body.tracked if body.last_update_time is not None else None, now)
            if change is not None:
                events.append({"event": "object mocap tracking", "ros_time": now, "object": name, **change})
        self._objects_seen = flags
        return events

    def _relay_changes(self, now: float) -> list[dict]:
        """The relay's floor calibration, refit, frame status, base calibrations and drift, when they change."""
        if self._relay is None:
            return []
        current = self._relay_current()
        events = []
        for key, value in current.items():
            if key in self._relay_seen and self._relay_seen[key] != value:
                events.append({"event": f"relay {key}", "ros_time": now, "value": value, "was": self._relay_seen[key]})
            elif key not in self._relay_seen and value not in (None, {}):
                events.append({"event": f"relay {key}", "ros_time": now, "value": value, "was": None})
            self._relay_seen[key] = value
        drift = self._relay.state.drift
        if drift is not None:
            values = _drift_values(drift)
            if _drift_moved(self._relay_drift, values):
                events.append({"event": "relay drift", "ros_time": now, "value": values, "was": self._relay_drift})
                self._relay_drift = values
        return events

    def _relay_current(self) -> dict:
        """The relay's switching states now: frame status, calibrations, refit, deciding fit, scale flag, anchors."""
        state = self._relay.state
        frame = state.frame
        current = {
            "frame status": frame.status if frame else None,
            "floor calibration": frame.floor_calibration if frame else None,
            "refit": frame.refit if frame else None,
            "newer floor calibration": frame.newer if frame else None,
            "scale": frame.scale if frame else None,
            "base calibrations": {str(rb): _calibration(calibration) for rb, calibration in
                                  sorted(state.calibrations.items())},
        }
        drift = state.drift
        if drift is not None:
            current["deciding fit"] = drift.decides
            current["scale changed"] = drift.scale_changed
            current["anchors used"] = drift.anchors_used
        return current


# --- --- --- --- --- HELPERS --- --- --- --- ---

def _status(robot: HuskyRobotInterface, now: float) -> dict:
    """What `look` compares tick to tick for one robot: plain values, each change one event."""
    base = robot.base.state
    lease = robot.lease_slot.lease
    status = {
        "lease": lease.holder if lease is not None else None,
        "soft stop": robot.soft_stop_latch.reason or "stopped" if robot.soft_stopped else None,
        "e-stop": base.estopped,
        "follower": None if base.follower is None else f"{base.follower.state} {base.follower.reason}".strip(),
        "battery charging": base.battery_charging if base.battery_update_time is not None else None,
        "battery health": base.battery_health,
        **_controllers("base", base.controllers),
    }
    for name, arm in robot.arms.items():
        state = arm.state
        status[f"{name} operational"] = state.operational
        status[f"{name} safety mode"] = state.safety_mode
        status[f"{name} robot mode"] = state.robot_mode
        status[f"{name} program playing"] = state.program_playing
        status[f"{name} problem"] = state.problem or None
        status[f"{name} stop reason"] = state.stop_reason or None
        status.update(_controllers(name, state.controllers))
        tool = state.end_effector
        if isinstance(tool, ScaffoldingV3State):
            status[f"{name} gripper motor"] = tool.gripper_motor
            status[f"{name} joint motor"] = tool.joint_motor
            status[f"{name} tool buttons"] = [button for button, held in zip(ScaffoldingV3.BUTTONS, tool.buttons_held)
                                              if held]
        elif isinstance(tool, RobotiqState):
            status[f"{name} gripper result"] = tool.failure or tool.last_result_ok
        elif isinstance(tool, ScaffoldingV1State):
            status[f"{name} gripper closed"] = tool.gripper_closed
    return status


def _flags(robot: HuskyRobotInterface, now: float) -> dict[str, bool | None]:
    """A robot's good-or-bad states, told debounced (`_Flag`): its mocap tracking and each stream running."""
    base = robot.base.state
    flags = {"mocap tracking": base.tracked if base.last_update_time is not None else None,
             "base stream": _running(base.joint_states_update_time, now),
             "mocap stream": _running(base.last_update_time, now)}
    flags.update({f"{name} stream": _running(arm.state.last_update_time, now) for name, arm in robot.arms.items()})
    return flags


def _controllers(name: str, state) -> dict:
    """A controller manager's running controller (one event per switch) and its last switch error."""
    return {f"{name} controller": state.active, f"{name} controller switch error": state.switch_error}


def _full(message) -> dict:
    """A message sent in full, under FILE_KEY for the plugin to write; nothing without one."""
    return {} if message is None else {FILE_KEY: message}


def _changed(key: str, value: Any, old: Any, serial: str, now: float) -> dict:
    """One status change as an event: "lease" taken or given back, "soft stop" set or cleared, ...; else "<key>"."""
    if key == "lease":
        name = "lease taken" if value is not None else "lease released"
        return {"event": name, "ros_time": now, "robot": serial, "holder": value if value is not None else old}
    return {"event": key, "ros_time": now, "robot": serial, "value": value, "was": old}


def _running(update_time: float | None, now: float) -> bool | None:
    """Whether a stream had a message within STREAM_LOST; None before its first."""
    return None if update_time is None else now - update_time <= STREAM_LOST


def _cause(holder: str | None, commanded: bool) -> str:
    """Who or what started a motion, for people."""
    if commanded:
        return holder or "the monitor, with no lease held"
    return "no command sent: free drive, the pendant, a push or a joystick" if holder is None else \
        f"no command sent while {holder} held the lease"


def _apart(a: tuple, b: tuple) -> bool:
    """Whether two poses are BASE_MOVE or BASE_TURN apart."""
    return float(np.linalg.norm(a[0] - b[0])) >= BASE_MOVE or _turn(a, b) >= BASE_TURN


def _turn(a: tuple, b: tuple) -> float:
    """The angle between two poses' orientations, radians."""
    return float((Rotation.from_quat(a[1]).inv() * Rotation.from_quat(b[1])).magnitude())


def _pose(pose: tuple) -> list[float]:
    """A pose as [x, y, z, qx, qy, qz, qw]."""
    return _numbers(pose[0]) + _numbers(pose[1])


def _numbers(values) -> list[float] | None:
    """Numbers as a plain list, rounded to a micrometre (or a millionth); None for none."""
    if values is None:
        return None
    return [round(float(value), 6) for value in np.asarray(values, dtype=float).reshape(-1)]


def _calibration(calibration) -> dict:
    """A base calibration as the relay announced it: where from and its hash."""
    return {"source": calibration.source, "session": calibration.session, "sha256": calibration.sha256[:12],
            "definition": calibration.definition_status}


def _drift_values(drift) -> dict:
    """The relay's drift numbers, as an event keeps them."""
    return {name: getattr(drift, name) for name in ("off", "off_limit", "anchors_moved", "cameras_moved", "apart",
                                                    "scale_ppm", "anchors_residual")}


def _drift_moved(told: dict | None, now: dict) -> bool:
    """Whether the drift moved by DRIFT_STEP (or the scale by SCALE_STEP) from what was last told, or appeared."""
    if told is None:
        return True
    for name, step in (("off", DRIFT_STEP), ("anchors_moved", DRIFT_STEP), ("scale_ppm", SCALE_STEP)):
        before, after = told.get(name), now.get(name)
        if (before is None) != (after is None) or (after is not None and abs(after - before) >= step):
            return True
    return False
