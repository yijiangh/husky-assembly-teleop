"""
Which parts of a robot are simulated: each announces itself once a second on /<robot>/simulated_parts.

The simulators send it (crl_husky.simulated_parts): husky_sim for the base, multi_arm_safety_sync for arms on fake
hardware (it runs them as simulated when launched with use_fake_hardware:=true), each simulated tool by its namespace
("gripper", "left_gripper"), a fake Motive (sim_motive) for "mocap".
! A part counts as simulated only while its announcements keep coming; one not heard from is real, with every check.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

#: The topic, under the robot's namespace.
TOPIC = "simulated_parts"
#: A part counts as simulated while its last announcement is this recent, seconds; it announces every second.
SIMULATED_FOR = 3.0


@dataclass(frozen=True)
class SimulatedPart:
    """One part's last announcement.

    Attributes:
        part: "base", "mocap", or an arm or a tool by its namespace under the robot's, e.g. "left_ur5e",
            "left_gripper".
        node: The node that simulates it, e.g. "husky_sim"; its parameters describe the simulation.
        kind: How it is simulated, e.g. "base simulator", "ur fake hardware".
        heard_at: ROS time it was last heard, seconds.
    """

    part: str
    node: str
    kind: str
    heard_at: float


class SimulatedParts:
    """The parts of one robot that announce themselves as simulated, by part."""

    def __init__(self, now: Callable[[], float]):
        """Nothing heard yet; `now` gives the ROS time, seconds."""
        self._now = now
        self._heard: dict[str, SimulatedPart] = {}

    def heard(self, message) -> None:
        """Keep an announcement (crl_husky_msgs/SimulatedPart)."""
        self._heard[message.part] = SimulatedPart(message.part, message.node, message.kind, self._now())

    def get(self, part: str) -> SimulatedPart | None:
        """`part`'s announcement while it is simulated, else None: never heard, or silent for SIMULATED_FOR."""
        heard = self._heard.get(part)
        return heard if heard is not None and self._now() - heard.heard_at <= SIMULATED_FOR else None

    def all(self) -> list[SimulatedPart]:
        """Every part simulated now, by part name."""
        return [heard for part, heard in sorted(self._heard.items()) if self.get(part) is not None]


def body_values(text: str, body: int, count: int) -> list[float]:
    """A fake Motive's per-body parameter for one body: `count` numbers, zeros if none are given.

    The text holds values for every body ("0.00003,0.002") and, space separated, `<id>:<values>` for single bodies
    ("1866:0.00003,0"); a body's own entry wins.

    Raises:
        ValueError: If an entry does not have `count` numbers.
    """
    every, own = [0.0] * count, None
    for entry in text.split():
        target, _, numbers = entry.rpartition(":")
        parsed = [float(number) for number in numbers.split(",")]
        if len(parsed) != count:
            raise ValueError(f"{entry!r}: {count} numbers expected")
        if not target:
            every = parsed
        elif int(target) == body:
            own = parsed
    return own if own is not None else every
