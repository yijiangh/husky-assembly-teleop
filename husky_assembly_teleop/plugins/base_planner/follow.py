"""
Where the base really goes when the onboard follower drives a path: a closed-loop simulation, no ROS.

Runs the follower's own code (crl_husky `PurePursuit`, with its robot model from config/base_models) against crl_husky's
motion model (`BaseModel`) with the base's true xICR, at the follower's control rate and with the model's command
delay, as crl_husky `husky_sim` does. Slip, noise and mocap delay are not modelled.

`model_groups` drives a path with the robot exactly as the follower's model says, and with deviated models.
"""

from __future__ import annotations

import math
from collections import deque

import numpy as np
from crl_husky.base_model import BaseModel
from crl_husky.pure_pursuit import DONE, PurePursuit, PurePursuitParams

#: The follower's control rate, Hz (crl_husky pure_pursuit_node `control_rate_hz`).
CONTROL_RATE = 30.0
#: Give up after this many times the time the path takes at full speed, plus GRACE seconds.
PATIENCE = 3.0
GRACE = 10.0
#: Seconds simulated after the follower is done, as the plugin records them: the last commands still act.
SETTLE = 2.0
#: Where the turning centre may be along the body, metres, as crl_husky's xICR (negative: ahead of base_footprint);
#: around Alice's fitted -0.13 m (config/base_models in crl_husky). The planners' default range.
XICR_RANGE = (-0.20, -0.10)
#: The deviated models: the follower's model with the true xICR at the ends and middle of XICR_RANGE, metres.
DEVIATED_XICRS = (XICR_RANGE[0], sum(XICR_RANGE) / 2.0, XICR_RANGE[1])
#: The model groups, in report order.
EXACT, DEVIATED = "exact model", "deviated models"


def followed(poses, x_icr: float, model: dict, linear_speed: float, angular_speed: float) -> tuple[np.ndarray, str]:
    """Simulate the follower driving `poses`, with the base's true turning centre at `x_icr`.

    Args:
        poses: (x, y, yaw) waypoints as sent to the follower (`BasePath`), the first being where the base starts.
        x_icr: The base's true xICR, metres, as in crl_husky's base models.
        model: The follower's robot model (crl_husky `load_model`); the base has its efficiencies and delay too.
        linear_speed: The path's speed cap, m/s.
        angular_speed: The path's turn rate cap, rad/s.

    Returns:
        tuple[np.ndarray, str]: The base's (x, y, yaw) every control cycle, from the start until SETTLE seconds
        after the follower was done; and "" if the follower finished, else why not.
    """
    follower = PurePursuit(PurePursuitParams(
        x_icr=model["xICR"], speed_efficiency=model["speed_efficiency"],
        steering_efficiency=model["steering_efficiency"], cmd_delay_sec=model["cmd_delay_sec"]))
    base = BaseModel(x_icr, model["speed_efficiency"], model["steering_efficiency"])
    poses = np.asarray(poses, dtype=float)
    follower.start(follower.make_path(*poses.T), 0.0, linear_speed, angular_speed)
    steps = np.diff(poses, axis=0)
    limit = PATIENCE * float(np.sum(np.linalg.norm(steps[:, :2], axis=1)) / linear_speed
                             + np.sum(np.abs(steps[:, 2])) / angular_speed) + GRACE
    dt = 1.0 / CONTROL_RATE
    # Commands waiting out the delay, (time they act, v, w); the one acting now.
    waiting, acting = deque(), (0.0, 0.0)
    state, t = poses[0].copy(), 0.0
    trace = [state]
    while follower.following:
        if t > limit:
            return np.array(trace), f"not done after {limit:.0f} s"
        v, w = follower.step(*state, t, dt, sent_at=t)
        waiting.append((t + model["cmd_delay_sec"], v, w))
        while waiting and waiting[0][0] <= t + 1e-9:
            acting = waiting.popleft()[1:]
        state = base.step(state, *acting, dt)
        t += dt
        trace.append(state)
    # * The follower is done, but the commands it sent last still act for the model's delay: settle as recorded.
    waiting.append((t + model["cmd_delay_sec"], 0.0, 0.0))
    for _ in range(int(round(SETTLE / dt))):
        while waiting and waiting[0][0] <= t + 1e-9:
            acting = waiting.popleft()[1:]
        state = base.step(state, *acting, dt)
        t += dt
        trace.append(state)
    status = follower.status
    return np.array(trace), "" if status.state == DONE else status.reason or "stopped"


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
