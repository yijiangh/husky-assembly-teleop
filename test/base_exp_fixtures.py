"""Shared builders for the base_exp tests: a run driven through a known robot model, recorded like the plugin does."""

from __future__ import annotations

import numpy as np
from crl_husky.base_model import PLATFORM_CYCLE, BaseModel

from husky_assembly_teleop.shared.recordings.runs import Recording

#: Rates as on the robot: follower commands, the monitor's tick (poses), wheel odometry, mocap; Hz.
COMMAND_RATE, TICK_RATE, ODOMETRY_RATE, MOCAP_RATE = 30.0, 20.0, 28.0, 50.0
#: Integration steps per command.
FINE = 20
#: The tracking context's channels, as base_exp records them.
CONTEXT = ("turning", "curvature", "progress", "speed", "turn_rate", "phase")


def labelled(run: dict) -> Recording:
    """`run` as a loaded recording, with the tracking context's labels."""
    return Recording({**run, "tracking_context.labels": np.array(CONTEXT)})


def saved(run: dict, robot: str = "x") -> dict:
    """`run`'s arrays as recording.npz keeps them: the robot's serial before the pose's key, and "robot"."""
    return {**{f"{robot}_{k}" if k == "floor_pose" else k: value for k, value in run.items()}, "robot": np.array(robot)}


def model_run(delay=0.2, speed=0.9, steering=0.95, x_icr=-0.1, seconds=60.0, seed=0, stamps=True,
              wheel_delay=None, mocap=False, platform: BaseModel | None = None) -> Recording:
    """A run driven by smooth random commands through the simulator's model.

    The robot holds each command until the next one, `delay` seconds late; the recording samples it at the tick.

    Args:
        delay: Seconds from a command to the motion.
        speed: Speed efficiency.
        steering: Steering efficiency.
        x_icr: Turning-centre offset, metres.
        seconds: Length of the run.
        seed: Seed of the commands.
        stamps: Whether the run has its stamps: every follower report (`follower.*`), and `times` (runs recorded
            before mocap was) or, with `mocap`, every mocap sample.
        wheel_delay: With a value, the run has wheel odometry: the commands this many seconds late, unscaled.
        mocap: Whether poses come per mocap sample (`mocap.*`, MOCAP_RATE, handled 5 ms after capture) as recorded
            now, instead of `times`.
        platform: With one, the commands pass its limits every PLATFORM_CYCLE before they act, as on the robot.
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
    if platform is not None:
        cycles = np.arange(0.0, seconds, PLATFORM_CYCLE)
        wheels, limited = held(cycles, 0.0), (0.0, 0.0)
        for i, (v, w) in enumerate(wheels):
            limited, wheels[i] = platform.limit(limited, v, w, PLATFORM_CYCLE)
        index = np.searchsorted(cycles, fine - delay, side="right") - 1
        acting = np.where((index >= 0)[:, None], wheels[np.clip(index, 0, None)], 0.0)
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
    if stamps and mocap:
        captured = np.arange(0.0, seconds - 0.1, 1.0 / MOCAP_RATE)
        run.update({"mocap.t": captured + 0.005, "mocap.stamp": captured,
                    "mocap.floor_pose": pose[np.searchsorted(fine, captured, side="right") - 1]})
    elif stamps:
        run["times"] = np.column_stack([fine[at], sent[latest]])
    if stamps:
        run.update({"follower.t": sent, "follower.stamp": sent, "follower.command": command,
                    "follower.state": np.ones((len(sent), 1))})
    if wheel_delay is not None:
        odometry = np.arange(0.0, seconds, 1.0 / ODOMETRY_RATE)
        run.update({"odometry.t": odometry, "odometry.stamp": odometry,
                    "odometry.velocity": held(odometry, wheel_delay)})
    return labelled(run)
