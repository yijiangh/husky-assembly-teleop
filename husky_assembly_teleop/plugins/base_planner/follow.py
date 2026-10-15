"""
Where the base really goes when the onboard follower drives a path, simulated (crl_husky `follow_sim.followed`).

`model_groups` drives a path with the robot exactly as the follower's model says, and with deviated models.
"""

from __future__ import annotations

import math

import numpy as np
from crl_husky.follow_sim import followed

#: Where the turning centre may be along the body, metres, as crl_husky's xICR (negative: ahead of base_footprint);
#: around Alice's fitted -0.13 m (config/base_models in crl_husky). The planners' default range.
XICR_RANGE = (-0.20, -0.10)
#: The deviated models: the follower's model with the true xICR at the ends and middle of XICR_RANGE, metres.
DEVIATED_XICRS = (XICR_RANGE[0], sum(XICR_RANGE) / 2.0, XICR_RANGE[1])
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


def model_groups(poses, model: dict, linear_speed: float, angular_speed: float) -> list[tuple]:
    """Drive `poses` with the robot exactly as the follower's model says, then with each deviated xICR.

    Args:
        poses: (x, y, yaw) waypoints as sent to the follower.
        model: The follower's robot model (crl_husky `load_model`).
        linear_speed: The path's speed cap, m/s.
        angular_speed: The path's turn rate cap, rad/s.

    Returns:
        list: (group, true xICR, trace, "" if finished else why not) per drive: EXACT first, then DEVIATED_XICRS.
    """
    out = []
    for group, x_icr in [(EXACT, model["xICR"])] + [(DEVIATED, value) for value in DEVIATED_XICRS]:
        trace, why = followed(poses, x_icr, model, linear_speed, angular_speed)
        out.append((group, float(x_icr), trace, why))
    return out
