"""Arm joints for planners: names, sides, URDF limits, and goals moved by whole turns next to a start. Any thread."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

from scaffolding_assembly_core.urdf import UR_JOINT_NAMES, load_urdf

#: How far outside a limit a goal joint may sit and still count as inside, radians: rounding only.
TURN_SLACK = 1e-9
#: How far outside a limit a joint may be before `outside_limits` reports it, radians.
LIMIT_TOLERANCE = 1e-6
#: Whole turns tried either side of the nearest equivalent value.
_TURNS = (0, -1, 1, -2, 2)


def arm_joint_names(arm: str) -> list[str]:
    """The URDF names of one arm's six joints (e.g. arm "left_ur_arm"), in the UR driver's order."""
    return [f"{arm}_{name}" for name in UR_JOINT_NAMES]


def arm_side(arm: str) -> str:
    """An arm's side from its name: "left", "right", or "arm" for a one-arm robot's "ur_arm"."""
    side = arm.split("_", 1)[0]
    return side if side in ("left", "right") else "arm"


def unwrap_to_nearest(goal: Sequence[float], start: Sequence[float], lower: Sequence[float] | None = None,
                      upper: Sequence[float] | None = None) -> np.ndarray:
    """Shift each goal joint by whole turns to the equivalent value nearest its start (same flange pose).

    - Without limits, each joint ends within ±π of its start.
    - ! With limits, only in-range values are picked: the UR elbow turns ±π only, so its nearest value may be out of
      range (start 2.26, goal -1.06 -> 5.22); it then keeps its in-range value and turns the long way.
    - A joint with no in-range equivalent at all keeps its ±π value; `outside_limits` reports it.

    Args:
        goal: Goal joint values, radians.
        start: Start joint values, same order.
        lower: Lower limits, same order (`joint_limits`), or None.
        upper: Upper limits, same order, or None.

    Returns:
        np.ndarray: The shifted goal.
    """
    goal, start = np.asarray(goal, dtype=float), np.asarray(start, dtype=float)
    turn = 2.0 * np.pi
    nearest = goal - np.round((goal - start) / turn) * turn
    if lower is None or upper is None:
        return nearest
    for index, (low, high) in enumerate(zip(lower, upper)):
        in_range = [value for value in (nearest[index] + k * turn for k in _TURNS)
                    if low - TURN_SLACK <= value <= high + TURN_SLACK]
        if in_range:
            nearest[index] = min(in_range, key=lambda value: abs(value - start[index]))
    return nearest


def joint_limits(urdf: Path, names: Sequence[str]) -> tuple[np.ndarray, np.ndarray]:
    """URDF lower and upper limits of some joints; a continuous joint, or one without limits, gets ±inf.

    Args:
        urdf: The robot's URDF.
        names: Joint names.

    Returns:
        tuple[np.ndarray, np.ndarray]: Lower and upper limits, radians, in the order of `names`.

    Raises:
        KeyError: If the URDF has no joint of one of the names.
    """
    joints = load_urdf(urdf).joint_map
    lower, upper = [], []
    for name in names:
        joint = joints[name]
        free = joint.type == "continuous" or joint.limit is None
        lower.append(-np.inf if free or joint.limit.lower is None else joint.limit.lower)
        upper.append(np.inf if free or joint.limit.upper is None else joint.limit.upper)
    return np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)


def outside_limits(labelled: Sequence[tuple[str, Sequence[float]]], names: Sequence[str], lower: Sequence[float],
                   upper: Sequence[float]) -> list[str]:
    """Joints outside their limits, for one or more labelled configurations.

    Args:
        labelled: (label, joint values) pairs, values in the order of `names`, e.g. [("start", q0), ("goal", q1)].
        names: The joint names.
        lower: Lower limits, same order.
        upper: Upper limits, same order.

    Returns:
        list[str]: One "<label> <joint>=<value> rad (limits [lo, hi])" per joint outside its limits; empty if none.
    """
    return [f"{label} {name}={value:+.3f} rad (limits [{low:+.3f}, {high:+.3f}])"
            for label, values in labelled
            for name, value, low, high in zip(names, values, lower, upper)
            if not low - LIMIT_TOLERANCE <= value <= high + LIMIT_TOLERANCE]
