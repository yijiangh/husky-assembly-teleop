"""
A robot's soft-stop latch: while it is set, the robot's base, arms and tools refuse every command that could move it.

Stopping (`hold`, `stop`, `stop_path`, a zero twist, a tool's `stop`) always goes through. Only `resume` clears it.
"""

from __future__ import annotations

from ..config import short_serial

#: The latch's reason for the operator's Stop all (`HuskyMonitor._soft_stop`); a clearance trip gives its problem.
STOP_ALL_REASON = "Stop all"


class MotionRefused(Exception):
    """A motion refused for an operator's reason, not a bug: soft stop, no lease, or a broken freeze.

    The monitor logs it as a normal end of a task, intent or hook, not as a plugin failure.
    """

    #: Short enough for a chip; the message says the rest.
    label = "refused"


class RobotStopped(MotionRefused):
    """A command to a soft-stopped robot: nothing was sent. The operator must press Resume for that robot first."""

    label = "soft stopped"


class SoftStopLatch:
    """Whether one robot is soft stopped; set by `HuskyRobotInterface.soft_stop`, cleared by its `resume`.

    Attributes:
        serial: The robot, for messages.
        stopped: True from a soft stop until the next resume.
        reason: Why it was stopped, e.g. STOP_ALL_REASON; "" when not stopped or not said.
    """

    def __init__(self, serial: str):
        """Start not stopped."""
        self.serial = serial
        self.stopped = False
        self.reason = ""

    def problem(self) -> str | None:
        """Why nothing may move this robot now, or None."""
        return f"robot {short_serial(self.serial)} is soft stopped; press Resume first" if self.stopped else None

    def check(self) -> None:
        """Raise RobotStopped if the robot is soft stopped.

        Raises:
            RobotStopped: If the robot is soft stopped; nothing may be sent. Its message says only why (`problem`).
        """
        if problem := self.problem():
            raise RobotStopped(problem)
