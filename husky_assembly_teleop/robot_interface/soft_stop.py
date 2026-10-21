"""
A robot's soft-stop latch: while it is set, the robot's base, arms and tools refuse every command that could move it.

Stopping (`hold`, `stop`, `stop_path`, a zero twist, a tool's `stop`) always goes through. Only `resume` clears it.
"""

from __future__ import annotations


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
        reason: Why it was stopped, e.g. "Stop all"; "" when not stopped or not said.
    """

    def __init__(self, serial: str):
        """Start not stopped."""
        self.serial = serial
        self.stopped = False
        self.reason = ""

    def problem(self) -> str | None:
        """Why nothing may move this robot now, or None."""
        return f"robot {self.serial} is soft stopped; press Resume first" if self.stopped else None

    def check(self, command: str) -> None:
        """Raise RobotStopped if the robot is soft stopped.

        Args:
            command: What would be sent, for the message, e.g. "a joint trajectory to left".

        Raises:
            RobotStopped: If the robot is soft stopped; nothing may be sent.
        """
        if self.stopped:
            raise RobotStopped(f"robot {self.serial} is soft stopped, {command} not sent; press Resume first")
