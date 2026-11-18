"""
Every rest pose and every stop of each robot: kept in memory, appended to a day file, read back at startup.

A pose is all of a robot's arms at rest, logged when the robot comes to rest after moving while it ran (operational).
! What an arm does while it is not running (an e-stopped arm sagging on its brakes) is never logged as a pose, so
  the newest pose stays the one from before the stop.
* Each line says whether the robot was simulated: a real monitor never offers a pose from a simulation, nor the
  other way round. Order is by wall time, since ROS time may be a simulation's.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timedelta
from pathlib import Path
from typing import Iterable

#: Where the day files go, one JSON line per pose or stop: <folder>/<yyyymmdd>.jsonl.
LOG_FOLDER = Path.home() / ".ros" / "stop_recovery"
#: Poses and stops kept in memory per robot (the panel shows the last few).
KEPT = 50
#: A rest pose this close to the newest one on every joint, radians, is not logged again.
SAME_POSE = 1e-4


@dataclass(frozen=True)
class ArmPose:
    """One arm at rest.

    Attributes:
        joints: Joint angles, radians, UR order.
        tcp: tool0 position in the arm's base_link, metres, or None if not reported.
        force: The force sensor's force (fx, fy, fz), newtons, raw, or None if not reported.
    """

    joints: tuple[float, ...]
    tcp: tuple[float, float, float] | None = None
    force: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class RobotPose:
    """All arms of one robot at rest.

    Attributes:
        serial: The robot, e.g. "a200-0805".
        ros_time: ROS time it was logged, seconds.
        wall: Local wall time it was logged, "2026-11-10T19:39:20".
        sim: Whether the robot was simulated.
        arms: Arm name -> its pose.
        go_back: Whether a Go back ended there (finished, held or stopped).
    """

    serial: str
    ros_time: float
    wall: str
    sim: bool
    arms: dict[str, ArmPose] = field(default_factory=dict)
    go_back: bool = False

    @property
    def clock(self) -> str:
        """Its wall time as "19:39:20"."""
        return self.wall[11:19]

    @property
    def order(self) -> tuple[str, float]:
        """Its place among poses and stops: wall time, then ROS time within a second."""
        return self.wall, self.ros_time

    def same_joints(self, other: RobotPose) -> bool:
        """Whether every arm of both is within SAME_POSE on every joint."""
        if set(self.arms) != set(other.arms):
            return False
        return all(abs(a - b) <= SAME_POSE for name in self.arms
                   for a, b in zip(self.arms[name].joints, other.arms[name].joints))


@dataclass(frozen=True)
class Stop:
    """A robot stopping: one of its arms left the running state (e-stop, protective stop, power off).

    Attributes:
        serial: The robot.
        ros_time: ROS time, seconds.
        wall: Local wall time, "2026-11-10T19:47:26".
        sim: Whether the robot was simulated.
        reason: What the safety sync said, e.g. "safety mode ROBOT_EMERGENCY_STOP".
        forces: Arm name -> force (fx, fy, fz) when it stopped, newtons, or None where not reported.
    """

    serial: str
    ros_time: float
    wall: str
    sim: bool
    reason: str
    forces: dict[str, tuple[float, float, float] | None] = field(default_factory=dict)

    @property
    def clock(self) -> str:
        """Its wall time as "19:47:26"."""
        return self.wall[11:19]

    @property
    def order(self) -> tuple[str, float]:
        """Its place among poses and stops, as `RobotPose.order`."""
        return self.wall, self.ros_time


def _vector(value) -> tuple[float, ...] | None:
    """A JSON list as a tuple, None kept."""
    return None if value is None else tuple(float(each) for each in value)


class PoseLog:
    """The poses and stops of every robot, oldest first, and the lines still to write to the day file.

    Attributes:
        folder: Where the day files are.
        poses: Serial -> its poses, oldest first, at most KEPT.
        stops: Serial -> its stops, oldest first, at most KEPT.
        unwritten: JSON lines not yet appended to the day file (`take_unwritten`).
    """

    def __init__(self, folder: Path = LOG_FOLDER):
        """Start empty; `load` reads the day files."""
        self.folder = folder
        self.poses: dict[str, list[RobotPose]] = {}
        self.stops: dict[str, list[Stop]] = {}
        self.unwritten: list[str] = []

    def add_pose(self, pose: RobotPose) -> bool:
        """Log a rest pose unless it is its robot's newest one again (sim or real alike); True if logged."""
        kept = self.poses.setdefault(pose.serial, [])
        newest = next((each for each in reversed(kept) if each.sim == pose.sim), None)
        if newest is not None and newest.same_joints(pose):
            return False
        kept.append(pose)
        del kept[:-KEPT]
        self.unwritten.append(json.dumps({"kind": "pose", **asdict(pose)}))
        return True

    def add_stop(self, stop: Stop) -> None:
        """Log a stop."""
        kept = self.stops.setdefault(stop.serial, [])
        kept.append(stop)
        del kept[:-KEPT]
        self.unwritten.append(json.dumps({"kind": "stop", **asdict(stop)}))

    def last(self, serial: str, count: int, sim: bool) -> list[RobotPose]:
        """The newest `count` poses of a robot, simulated or real as `sim`, newest first."""
        return [pose for pose in reversed(self.poses.get(serial, [])) if pose.sim == sim][:count]

    def stop_after(self, pose: RobotPose) -> Stop | None:
        """The first stop of its robot (sim or real alike) after `pose` and before its next pose, or None."""
        later = [each.order for each in self.poses.get(pose.serial, [])
                 if each.sim == pose.sim and each.order > pose.order]
        until = min(later, default=("~", 0.0))
        return next((stop for stop in self.stops.get(pose.serial, [])
                     if stop.sim == pose.sim and pose.order < stop.order < until), None)

    def take_unwritten(self) -> list[str]:
        """The lines still to write, now counted as written."""
        lines, self.unwritten = self.unwritten, []
        return lines

    def day_file(self, now: datetime) -> Path:
        """The day file for a wall time."""
        return self.folder / f"{now:%Y%m%d}.jsonl"

    def load(self, now: datetime, serials: Iterable[str]) -> None:
        """Read yesterday's and today's day files for these robots; a line that does not parse is skipped."""
        wanted = set(serials)
        for day in (now - timedelta(days=1), now):
            path = self.day_file(day)
            if not path.is_file():
                continue
            for line in path.read_text().splitlines():
                try:
                    entry = json.loads(line)
                    if entry.get("serial") not in wanted:
                        continue
                    serial, ros_time, wall, sim = (entry["serial"], float(entry["ros_time"]), entry["wall"],
                                                   bool(entry["sim"]))
                    if entry.get("kind") == "pose":
                        arms = {name: ArmPose(tuple(arm["joints"]), _vector(arm.get("tcp")),
                                              _vector(arm.get("force")))
                                for name, arm in entry["arms"].items()}
                        self.poses.setdefault(serial, []).append(
                            RobotPose(serial, ros_time, wall, sim, arms, bool(entry.get("go_back", False))))
                    elif entry.get("kind") == "stop":
                        forces = {name: _vector(force) for name, force in entry.get("forces", {}).items()}
                        self.stops.setdefault(serial, []).append(
                            Stop(serial, ros_time, wall, sim, entry.get("reason", ""), forces))
                except (ValueError, KeyError, TypeError):
                    continue
        for kept in (*self.poses.values(), *self.stops.values()):
            kept.sort(key=lambda each: each.order)
            del kept[:-KEPT]


def append_lines(path: Path, lines: list[str]) -> None:
    """Append JSON lines to a day file, making its folder. Blocking: run it on a thread."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as file:
        file.writelines(line + "\n" for line in lines)
