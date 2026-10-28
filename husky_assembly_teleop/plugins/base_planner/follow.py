"""
Where the base really goes when the onboard follower drives a path, simulated (crl_husky `follow_sim.followed`).

`model_groups` drives a path with the robot exactly as the follower's model says, and with deviated models.
"""

from __future__ import annotations

import math

import numpy as np
from crl_husky.follow_sim import followed

#: How far the true turning centre may be from the follower model's, either way, metres, as crl_husky's xICR. The
#: planners' default; the deviated models put the true xICR at both ends.
XICR_SPREAD = 0.05
#: The model groups, in report order.
EXACT, DEVIATED = "exact model", "deviated models"


def thinned(trace: np.ndarray, position_step: float, yaw_step: float) -> np.ndarray:
    """The poses of a trace at least `position_step` metres or `yaw_step` radians apart, with the last one."""
    kept, last = [trace[0]], trace[0]
    for pose in trace[1:]:
        if math.hypot(pose[0] - last[0], pose[1] - last[1]) >= position_step or abs(pose[2] - last[2]) >= yaw_step:
            kept.append(pose)
            last = pose
    if kept[-1] is not trace[-1]:
        kept.append(trace[-1])
    return np.array(kept)


def xicr_range(x_model: float, spread: float) -> tuple[float, float]:
    """The lowest and highest xICR a path is checked for: `spread` metres either side of the follower model's."""
    return x_model - spread, x_model + spread


def model_groups(poses, model: dict, linear_speed: float, angular_speed: float,
                 spread: float = XICR_SPREAD) -> list[tuple]:
    """Drive `poses` with the robot exactly as the follower's model says, then with the true xICR `spread` either side.

    Args:
        poses: (x, y, yaw) waypoints as sent to the follower.
        model: The follower's robot model (crl_husky `load_model`).
        linear_speed: The path's speed cap, m/s.
        angular_speed: The path's turn rate cap, rad/s.
        spread: How far the deviated models' xICR is from the model's, metres.

    Returns:
        list: (group, true xICR, trace, "" if finished else why not) per drive: EXACT first, then DEVIATED low and high.
    """
    out = []
    deviated = [(DEVIATED, value) for value in xicr_range(model["xICR"], spread)]
    for group, x_icr in [(EXACT, model["xICR"])] + deviated:
        trace, why = followed(poses, x_icr, model, linear_speed, angular_speed)
        out.append((group, float(x_icr), trace, why))
    return out
