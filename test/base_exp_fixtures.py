"""Shared builders for the base_exp tests: a run driven through a known robot model, recorded like the plugin does."""

from __future__ import annotations

import numpy as np
from crl_husky.base_model import BaseModel

#: Rates as on the robot: follower commands, the monitor's tick (poses), wheel odometry; Hz.
COMMAND_RATE, TICK_RATE, ODOMETRY_RATE = 30.0, 20.0, 28.0
#: Integration steps per command.
FINE = 20


def model_run(delay=0.2, speed=0.9, steering=0.95, x_icr=-0.1, seconds=60.0, seed=0, stamps=True,
              wheel_delay=None) -> dict[str, np.ndarray]:
    """A run driven by smooth random commands through the simulator's model.

    The robot holds each command until the next one, `delay` seconds late; the recording samples it at the tick.

    Args:
        delay: Seconds from a command to the motion.
        speed: Speed efficiency.
        steering: Steering efficiency.
        x_icr: Turning-centre offset, metres.
        seconds: Length of the run.
        seed: Seed of the commands.
        stamps: Whether the run has `times` and the follower's command log (`commands`).
        wheel_delay: With a value, the run has wheel odometry: the commands this many seconds late, unscaled.
    """
    rng = np.random.default_rng(seed)
    sent = np.arange(0.0, seconds, 1.0 / COMMAND_RATE)
    knots = np.arange(0.0, seconds + 2.0, 2.0)
    command = np.column_stack([np.interp(sent, knots, rng.uniform(-0.2, 0.3, len(knots))),
                               np.interp(sent, knots, rng.uniform(-0.6, 0.6, len(knots)))])

    def held(times, seconds_late):
        """The command acting at `times`, sent `seconds_late` before; zero before the first."""
        index = np.searchsorted(sent, times - seconds_late, side="right") - 1
        return np.where((index >= 0)[:, None], command[np.clip(index, 0, None)], 0.0)

    model, step = BaseModel(x_icr, speed, steering), 1.0 / (COMMAND_RATE * FINE)
    fine = np.arange(0.0, seconds, step)
    acting = held(fine, delay)
    pose = np.zeros((len(fine), 3))
    for i in range(1, len(fine)):
        pose[i] = model.step(pose[i - 1], *acting[i - 1], step)

    # * The tick samples the latest pose and the latest command; ticks are not in step with the commands.
    t = np.arange(0.0, seconds - 0.1, 1.0 / TICK_RATE) + 0.013
    at = np.searchsorted(fine, t, side="right") - 1
    latest = np.searchsorted(sent, t, side="right") - 1
    context = np.zeros((len(t), 6))
    context[:, 5] = 1
    run = {"t": t, "floor_pose": pose[at], "tracking_context": context, "follower_command": command[latest]}
    if stamps:
        run["times"] = np.column_stack([fine[at], sent[latest]])
        run["commands"] = np.column_stack([sent, sent, command, np.ones(len(sent))])
    if wheel_delay is not None:
        odometry = np.arange(0.0, seconds, 1.0 / ODOMETRY_RATE)
        run["wheel_odometry"] = np.column_stack([odometry, odometry, held(odometry, wheel_delay)])
    return run
