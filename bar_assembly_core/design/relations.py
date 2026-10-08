"""
Relations between bodies, robots and tools that the format stores once and readers derive from (format §5.5).

A part is a bar plus the joint halves connected to it; mates (two connected halves) connect parts but never merge
them. Allowed contacts are derived, never stored:
- a tool with the part it sits on, and with the halves mated to that part;
- connected bodies, and a mated half with its mate's part (a male half sits against its mate's bar);
- a robot's ground links with ground bodies;
- a tool with its mount contacts (the robot model's, `RobotModel.tool_touches`).
"""

from __future__ import annotations

from typing import Dict, Iterable, Set, Tuple

from .types import Design, State
from .vocabulary import ON

GROUND_PREFIX, BAR_PREFIX, HALF_PREFIX = "ground/", "bars/", "joints/"


def part_of(design: Design) -> Dict[str, str]:
    """The bar each body's part belongs to: a bar maps to itself, a body connected to a bar to that bar.

    Returns:
        dict[str, str]: Body id -> bar id. Bodies in no part (ground, obstacles, loose halves) are left out.
    """
    parts = {body: body for body in design.bodies if body.startswith(BAR_PREFIX)}
    for a, b in design.connections:
        for bar, other in ((a, b), (b, a)):
            if bar.startswith(BAR_PREFIX) and not other.startswith((BAR_PREFIX, GROUND_PREFIX)):
                parts.setdefault(other, bar)
    return parts


def members(parts: Dict[str, str], body: str) -> Set[str]:
    """Every body of the part `body` belongs to; just `body` if it is in no part."""
    bar = parts.get(body)
    return {other for other, owner in parts.items() if owner == bar} if bar is not None else {body}


def allowed_contacts(design: Design, state: State) -> Set[Tuple[str, str]]:
    """Every allowed contact in a state, each pair sorted (mount contacts excepted: they are the robot model's).

    Args:
        design: The design.
        state: The state; only present bodies and the tools' `on` matter.

    Returns:
        set[tuple[str, str]]: Pairs of ids: bodies, tools, and robot links "robots/<robot>/<link>".
    """
    pairs: Set[Tuple[str, str]] = set()

    def add(a: str, others: Iterable[str]) -> None:
        pairs.update(tuple(sorted((a, b))) for b in others if b != a)

    parts = part_of(design)
    mates: Dict[str, Set[str]] = {}
    for a, b in design.connections:
        add(a, (b,))
        if a.startswith(HALF_PREFIX) and b.startswith(HALF_PREFIX):
            mates.setdefault(a, set()).add(b)
            mates.setdefault(b, set()).add(a)
    for half, others in mates.items():
        for other in others:
            add(half, members(parts, other))
    for tool, tool_state in state.tools.items():
        on = tool_state.get(ON) if tool_state else None
        if on is not None:
            part = members(parts, on)
            add(tool, part | {mate for body in part for mate in mates.get(body, ())})
    grounds = [body for body in state.present if body.startswith(GROUND_PREFIX)]
    for robot_id, robot in design.robots.items():
        if state.robots.get(robot_id) is not None:
            for link in robot.ground_links:
                add(f"{robot_id}/{link}", grounds)
    return pairs
