"""
The insert's spiral search: the bar circles out around its straight line during the compliant part (W:4703-4822).

Both arms get the same sideways offset each tick, so the grasp is kept: the bar shifts, it does not turn. The radius
grows with the line's progress, from 0 at the handover to SPIRAL_RADIUS at the end, where the arms hold until a stall
or the operator's answer (`run_spiral`).

- ! A spiral over SPIRAL_MAX_SPEED at full radius is refused before anything moves (`spiral_problem`).
- ! Main thread, from a plugin task.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, Collection, Protocol, Sequence

import numpy as np

from scaffolding_assembly_core.geometry import Pose

from .stream import HOLD_FOR_STALL, Segment, stream

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.robot import HuskyRobotInterface
    from .stream import Recorder

#: The spiral's radius at the line's end, metres; keeps the target well inside the arm's 5 cm check. Its seconds and
#: turns are run settings (`RunSettings`); 4 turns make a pitch of 3.75 mm, under the joint's ~5 mm catch (M:12947).
SPIRAL_RADIUS = 0.015
#: Seconds from the spiral's end back to the line's end, after the operator's Confirm (W:4718).
SPIRAL_RETURN = 2.0
#: The fastest a spiral may move at full radius, metres per second (the retreat checks' speed limit).
SPIRAL_MAX_SPEED = 0.05
#: The warning the operator answers when the spiral ends without a stall.
SPIRAL_END_TEXT = (f"Spiral complete without a stall: HOLDING its end under compliance. Confirm = back to the "
                   f"straight-line end over {SPIRAL_RETURN:.0f} s, then hold for the stall (≤ {HOLD_FOR_STALL:.0f} s); "
                   f"Cancel = end the insert here.")


class Gate(Protocol):
    """Whether every mating screw stalled (`tools.StallGate`)."""

    text: str

    def all_stalled(self) -> bool:
        """Whether every mating screw counts as stalled."""


class Operator(Protocol):
    """A question to the operator that does not wait: open it, poll it each tick, close it once no longer needed."""

    def open(self, text: str, warn: bool = False) -> Any:
        """Ask `text`, as a warning if `warn`; returns the handle to poll."""

    def poll(self, handle: Any) -> bool | None:
        """The answer: True for Confirm, False for Cancel, None while there is none; an answered prompt is closed."""

    def close(self, handle: Any) -> None:
        """Take the question away unanswered."""


def lateral_basis(axis: Sequence[float]) -> tuple[np.ndarray, np.ndarray]:
    """Two unit vectors across `axis`: u = â × z (â × x for a vertical axis), v = â × u (W:4721-4747).

    Raises:
        ValueError: For a zero axis.
    """
    length = float(np.linalg.norm(axis))
    if length < 1e-12:
        raise ValueError("the spiral's axis is a zero vector")
    along = np.asarray(axis, dtype=float) / length
    across = np.cross(along, [0.0, 0.0, 1.0])
    if np.linalg.norm(across) < 1e-6:
        across = np.cross(along, [1.0, 0.0, 0.0])
    across /= np.linalg.norm(across)
    return across, np.cross(along, across)


class Spiral:
    """The bar's sideways offset from its line at each moment: radius and angle grow with f = elapsed / seconds.

    Attributes:
        u, v: Unit vectors across the axis, world.
        seconds: Seconds to full radius (the compliant part's duration).
        turns: Whole turns made on the way out.
        radius: Radius at the end, metres.
    """

    def __init__(self, axis: Sequence[float], seconds: float, turns: int, radius: float = SPIRAL_RADIUS):
        """A spiral around `axis`, the insert's direction in the world (any length)."""
        self.u, self.v = lateral_basis(axis)
        self.seconds, self.turns, self.radius = float(seconds), int(turns), float(radius)

    def frac(self, elapsed: float) -> float:
        """How far out the spiral is, 0 to 1."""
        return min(max(elapsed / self.seconds, 0.0), 1.0)

    def offset(self, elapsed: float) -> np.ndarray:
        """The offset, world, metres: radius f R, angle 2π n f."""
        fraction = self.frac(elapsed)
        angle = 2.0 * math.pi * self.turns * fraction
        return fraction * self.radius * (math.cos(angle) * self.u + math.sin(angle) * self.v)

    def done(self, elapsed: float) -> bool:
        """Whether the spiral reached full radius."""
        return elapsed >= self.seconds

    def text(self, elapsed: float) -> str:
        """Where the spiral stands, e.g. "spiral 63% (12.6/20.0 s, radius 9.4 mm, 2.5 of 4 turns)"."""
        fraction = self.frac(elapsed)
        return (f"spiral {100.0 * fraction:.0f}% ({min(elapsed, self.seconds):.1f}/{self.seconds:.1f} s, radius "
                f"{fraction * self.radius * 1e3:.1f} mm, {fraction * self.turns:.1f} of {self.turns} turns)")

    @property
    def pitch(self) -> float:
        """float: How much the radius grows per turn, metres."""
        return self.radius / self.turns

    @property
    def speed(self) -> float:
        """float: How fast the offset moves at full radius, metres per second."""
        return _speed(self.seconds, self.turns, self.radius)

    def as_record(self) -> dict:
        """The spiral as JSON values."""
        return {"seconds": self.seconds, "turns": self.turns, "radius_mm": self.radius * 1e3,
                "pitch_mm": self.pitch * 1e3, "speed_mm_s": self.speed * 1e3}


def _speed(seconds: float, turns: int, radius: float) -> float:
    """A spiral's speed at full radius, metres per second: (R / T) √(1 + (2π n)²)."""
    return radius / seconds * math.sqrt(1.0 + (2.0 * math.pi * turns) ** 2)


def spiral_fits(mating: Collection[str]) -> bool:
    """Whether an insert may spiral: it needs two mating joints; about one joint the spiral skews the bar."""
    return len(mating) >= 2


def spiral_problem(seconds: float, turns: int) -> str:
    """Why a spiral of SPIRAL_RADIUS with these settings may not run (faster than SPIRAL_MAX_SPEED), or "".

    Returns:
        str: E.g. "spiral too fast: 75 mm/s at 15 mm, limit 50; 4 turns need ≥ 7.6 s" (rounded up to 0.1 s), or "".
    """
    speed = _speed(seconds, turns, SPIRAL_RADIUS)
    if speed <= SPIRAL_MAX_SPEED:
        return ""
    shortest = math.ceil(round(_speed(1.0, turns, SPIRAL_RADIUS) / SPIRAL_MAX_SPEED * 10.0, 6)) / 10.0
    return (f"spiral too fast: {speed * 1e3:.0f} mm/s at {SPIRAL_RADIUS * 1e3:.0f} mm, limit "
            f"{SPIRAL_MAX_SPEED * 1e3:.0f}; {turns} turns need ≥ {shortest:.1f} s")


async def run_spiral(ctx: PluginContext, robot: HuskyRobotInterface, segment: Segment, spiral: Spiral, gate: Gate,
                     operator: Operator, recorder: Recorder) -> str:
    """Spiral out along the line and hold its end (A); on Confirm go back to the line's end and hold for the stall (B).

    - A: the line over the spiral's seconds plus its offset, held at the end with no time limit. It ends on a stall;
      once the spiral is complete a warning prompt opens, polled each tick before sending: Cancel ends here, Confirm
      goes on to B (W:4962-5051).
    - B: from the spiral's end to the line's end over SPIRAL_RETURN, then held until a stall, or HOLD_FOR_STALL.
    - ! As after `stream`, the caller holds the arms and switches them back on every ending.

    Args:
        ctx: The plugin's context.
        robot: The robot.
        segment: Each arm's line: from the measured TCP at the handover to the end; its seconds and wait are not used.
        spiral: The spiral around the insert's axis.
        gate: Whether every mating screw stalled.
        operator: Asks the warning at the spiral's end.
        recorder: Gets a sample each tick; its phase is set to "spiral", then "spiral return".

    Returns:
        str: "stalled", "cancelled" (the operator's Cancel at the spiral's end), "no_stall" (B's time ran out), or
            why a stream stopped (`stream`).
    """
    serial = robot.config.serial
    prompt, answer, last = None, None, 0.0

    def searching(elapsed: float) -> bool:
        """No stall yet, and no answer to the prompt; opens it once the spiral is complete."""
        nonlocal prompt, answer, last
        last = elapsed
        if gate.all_stalled():
            return False
        if prompt is None:
            if spiral.done(elapsed):
                ctx.log_warn(f"{serial}: {spiral.text(elapsed)} without a stall: holding its end until Confirm or "
                             f"Cancel")
                prompt = operator.open(SPIRAL_END_TEXT, warn=True)
            return True
        answer = operator.poll(prompt)
        return answer is None

    recorder.phase = "spiral"
    try:
        ended = await stream(ctx, robot, Segment(segment.start, segment.end, spiral.seconds, math.inf),
                             keep_going=searching, lateral=spiral.offset, recorder=recorder)
    finally:
        if prompt is not None and answer is None:
            operator.close(prompt)
    if ended != "ended":
        return ended
    if answer is None:
        ctx.log_info(f"{serial}: {gate.text} STALLED at {spiral.text(last)}")
        return "stalled"
    if not answer:
        ctx.log_warn(f"{serial}: Cancel at the spiral's end; the insert ends here")
        return "cancelled"
    ctx.log_info(f"{serial}: Confirm: back to the line's end over {SPIRAL_RETURN:.0f} s, then holding for the stall "
                 f"(≤ {HOLD_FOR_STALL:.0f} s)")
    back = spiral.offset(spiral.seconds)
    returning = Segment({name: Pose.from_arrays(np.add(end.position, back), end.orientation)
                         for name, end in segment.end.items()}, segment.end, SPIRAL_RETURN, HOLD_FOR_STALL)
    recorder.phase = "spiral return"
    ended = await stream(ctx, robot, returning, keep_going=lambda _elapsed: not gate.all_stalled(), recorder=recorder)
    if ended == "ended":
        ctx.log_info(f"{serial}: {gate.text} STALLED after the return to the line's end")
        return "stalled"
    if ended == "time":
        ctx.log_warn(f"{serial}: no stall within {HOLD_FOR_STALL:.0f} s of the return to the line's end")
        return "no_stall"
    return ended
