"""
Every rest pose and every stop of each robot: kept in memory, appended to a day file, read back at startup.

A pose is all of a robot's arms at rest, logged when the robot comes to rest after moving while it ran (operational).
! What an arm does while it is not running (an e-stopped arm sagging on its brakes) is never logged as a pose, so
  the newest pose stays the one from before the stop.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from datetime import datetime, timedelta
from pathlib import Path
from typing import Iterable

#: Where the day files go, one JSON line per pose or stop: <folder>/<yyyymmdd>.jsonl.
LOG_FOLDER = Path.home() / ".ros" / "stop_recovery"
#: Poses kept in memory per robot (the panel shows the last few).
KEPT = 50
#: A rest pose this close to the newest one on every joint, radians, is not logged again.
SAME_POSE = 1e-4


@dataclass(frozen=True)
class ArmPose:
    """One arm at rest.

    Attributes:
        joints: Joint angles, radians, UR order.
        tcp: tool0 position in the arm's base_link, metres, or None if not reported.
        force: The force sensor's force, newtons (magnitude, raw), or None if not reported.
    """

    joints: tuple[float, ...]
    tcp: tuple[float, float, float] | None = None
    force: float | None = None


@dataclass(frozen=True)
class RobotPose:
    """All arms of one robot at rest.

    Attributes:
        serial: The robot, e.g. "a200-0805".
        ros_time: ROS time it was logged, seconds.
        wall: Local wall time it was logged, "2026-11-10T19:39:20".
        arms: Arm name -> its pose.
    """

    serial: str
    ros_time: float
    wall: str
    arms: dict[str, ArmPose] = field(default_factory=dict)

    @property
    def clock(self) -> str:
        """Its wall time as "19:39:20"."""
        return self.wall[11:19]

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
        reason: What the safety sync said, e.g. "safety mode ROBOT_EMERGENCY_STOP".
        forces: Arm name -> force, newtons, when it stopped (None where not reported).
    """

    serial: str
    ros_time: float
    wall: str
    reason: str
    forces: dict[str, float | None] = field(default_factory=dict)

    @property
    def clock(self) -> str:
        """Its wall time as "19:47:26"."""
        return self.wall[11:19]


class PoseLog:
    """The poses and stops of every robot, newest last, and the lines still to write to the day file.

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
        """Log a rest pose unless it is the newest one again; True if logged."""
        kept = self.poses.setdefault(pose.serial, [])
        if kept and kept[-1].same_joints(pose):
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

    def last(self, serial: str, count: int) -> list[RobotPose]:
        """The newest `count` poses of a robot, newest first."""
        return list(reversed(self.poses.get(serial, [])[-count:]))

    def stop_after(self, pose: RobotPose) -> Stop | None:
        """The first stop of its robot after `pose` and before the robot's next pose, or None."""
        poses = self.poses.get(pose.serial, [])
        later = [each.ros_time for each in poses if each.ros_time > pose.ros_time]
        until = min(later, default=float("inf"))
        return next((stop for stop in self.stops.get(pose.serial, [])
                     if pose.ros_time < stop.ros_time < until), None)

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
                    if entry.get("kind") == "pose":
                        arms = {name: ArmPose(tuple(arm["joints"]), None if arm.get("tcp") is None
                                              else tuple(arm["tcp"]), arm.get("force"))
                                for name, arm in entry["arms"].items()}
                        self.poses.setdefault(entry["serial"], []).append(
                            RobotPose(entry["serial"], float(entry["ros_time"]), entry["wall"], arms))
                    elif entry.get("kind") == "stop":
                        self.stops.setdefault(entry["serial"], []).append(
                            Stop(entry["serial"], float(entry["ros_time"]), entry["wall"], entry.get("reason", ""),
                                 dict(entry.get("forces", {}))))
                except (ValueError, KeyError, TypeError):
                    continue
        for kept in (*self.poses.values(), *self.stops.values()):
            kept.sort(key=lambda each: each.ros_time)
            del kept[:-KEPT]


def append_lines(path: Path, lines: list[str]) -> None:
    """Append JSON lines to a day file, making its folder. Blocking: run it on a thread."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a") as file:
        file.writelines(line + "\n" for line in lines)
