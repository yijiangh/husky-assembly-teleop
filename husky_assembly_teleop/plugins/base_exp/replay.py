"""
Replay recorded commands through a motion model and compare with where mocap saw the robot.

A model is the simulator's (`crl_husky.base_model.BaseModel`) plus a delay from command to motion (`Model`). Its
quality is how far it predicts wrong `horizon` seconds after starting at a measured pose, driven by the run's own
commands (`errors`); 1 s is about how long pure pursuit takes to correct (0.2 m lookahead at 0.2 m/s). A replay of
a whole run from its start (`free_run`) drifts, so it is for plots only.

Commands come from the follower's log (`commands`: every report, stamped) when recorded, else from the
tick-sampled `follower_command`. Poses and commands are timed by their stamps (`times`) when recorded, else by the
monitor's tick, which adds the latencies to the delay.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np
from crl_husky.base_model import BaseModel

from .report import segment_types

#: Integration step, seconds.
STEP = 0.01
#: Horizons scored, seconds; HORIZON is the headline one.
HORIZONS = (0.25, 0.5, 1.0, 2.0)
HORIZON = 1.0
#: Seconds between the start poses of the predictions.
EVERY = 0.2
#: Run phases kept: the follower following (start poses come from these) and the robot settling.
FOLLOWING, PHASES = 1, (1, 2)
#: Fewer poses than this leave a run out.
FEWEST_POSES = 5


@dataclass(frozen=True)
class Model:
    """A motion model: the simulator's, with commands acting `delay` seconds after they were sent."""

    base: BaseModel = field(default_factory=BaseModel)
    delay: float = 0.0

    @classmethod
    def of(cls, x_icr: float, speed_efficiency: float, steering_efficiency: float, delay: float) -> Model:
        """A model from identified values; an unknown (NaN) value is the ideal one."""
        def known(value, ideal):
            return ideal if value is None or math.isnan(value) else float(value)

        return cls(BaseModel(known(x_icr, 0.0), known(speed_efficiency, 1.0), known(steering_efficiency, 1.0)),
                   known(delay, 0.0))


#: The robot the controllers assume: no delay, gains of one, turning about the tracked point.
IDEAL = Model()


@dataclass
class Track:
    """One run's measured poses, and the commands and wheel odometry around them, on one clock (seconds).

    Attributes:
        pose_time: Capture time of each pose.
        pose: (N, 3) x, y and unwrapped yaw; while following or settling only.
        moving: Per pose, whether the follower was following.
        kinds: Per pose, its segment type (`report.SEGMENTS`).
        command_time: When each command was sent, ascending.
        command: (M, 2) commanded v and ω.
        odometry_time: Stamp of each wheel odometry message, ascending; empty without it.
        odometry: (K, 2) v and ω from the wheels.
        clock: "stamps" (capture and command stamps) or "tick" (the monitor's tick time).
    """

    pose_time: np.ndarray
    pose: np.ndarray
    moving: np.ndarray
    kinds: np.ndarray
    command_time: np.ndarray
    command: np.ndarray
    odometry_time: np.ndarray
    odometry: np.ndarray
    clock: str


@dataclass
class Errors:
    """Prediction errors, one row per start pose and one column per horizon.

    Attributes:
        horizons: Seconds ahead of each column.
        position: Metres between predicted and measured position.
        heading: Radians between predicted and measured heading, absolute.
        kinds: Segment type at each start pose.
    """

    horizons: tuple
    position: np.ndarray
    heading: np.ndarray
    kinds: np.ndarray

    @classmethod
    def join(cls, parts: list[Errors], horizons=HORIZONS) -> Errors:
        """Several runs' errors as one."""
        parts = [p for p in parts if len(p.kinds)]
        if not parts:
            return cls(tuple(horizons), np.empty((0, len(horizons))), np.empty((0, len(horizons))),
                       np.empty(0, dtype=object))
        return cls(parts[0].horizons, np.concatenate([p.position for p in parts]),
                   np.concatenate([p.heading for p in parts]), np.concatenate([p.kinds for p in parts]))

    def at(self, horizon: float = HORIZON) -> tuple[np.ndarray, np.ndarray]:
        """Position and heading errors `horizon` seconds ahead (one of `horizons`)."""
        column = self.horizons.index(horizon)
        return self.position[:, column], self.heading[:, column]


def track(run: dict[str, np.ndarray]) -> Track | None:
    """A loaded recording (`report.load`) as a Track; None if it has too few poses."""
    context = run["tracking_context"]
    phase = context[:, 5]
    keep = np.isin(phase, PHASES)
    stamped = "times" in run
    times = run["times"] if stamped else np.column_stack([run["t"]] * 2)
    pose = run["floor_pose"]
    finite = keep & np.isfinite(pose).all(axis=1) & np.isfinite(times[:, 0])
    pose_time, first = np.unique(times[finite, 0], return_index=True)
    if len(pose_time) < FEWEST_POSES:
        return None
    pose = pose[finite][first].copy()
    pose[:, 2] = np.unwrap(pose[:, 2])
    moving = (phase[finite] == FOLLOWING)[first]
    kinds = segment_types(context[finite])[first]

    log = run.get("commands")
    if stamped and log is not None and len(log):
        command_time, command = log[:, 1], log[:, 2:4]
    else:
        command_time, command = times[keep, 1], run["follower_command"][keep]
    command_time, command = _ascending(command_time, command)

    odometry = run.get("wheel_odometry")
    if stamped and odometry is not None and len(odometry):
        odometry_time, odometry = _ascending(odometry[:, 1], odometry[:, 2:4])
    else:
        odometry_time, odometry = np.empty(0), np.empty((0, 2))
    return Track(pose_time, pose, moving, kinds, command_time, command, odometry_time, odometry,
                 "stamps" if stamped else "tick")


def commands_at(track: Track, model: Model, times: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """The v and ω acting at `times`: the latest command sent `model.delay` before; zero before the first."""
    index = np.searchsorted(track.command_time, np.asarray(times) - model.delay, side="right") - 1
    known = index >= 0
    command = track.command[np.clip(index, 0, None)] if len(track.command) else np.zeros((*np.shape(times), 2))
    return np.where(known, command[..., 0], 0.0), np.where(known, command[..., 1], 0.0)


def measured_at(track: Track, times: np.ndarray) -> np.ndarray:
    """The measured pose at `times` (linear between poses; yaw unwrapped), (S, 3)."""
    return np.column_stack([np.interp(times, track.pose_time, track.pose[:, i]) for i in range(3)])


def predict(track: Track, model: Model, start: np.ndarray, t0: np.ndarray, duration: float) -> np.ndarray:
    """Poses from each start pose (S, 3) at times t0 (S,), every STEP for `duration` seconds: (steps + 1, S, 3)."""
    steps = int(math.ceil(duration / STEP - 1e-9))
    out = np.empty((steps + 1, *np.shape(start)))
    out[0] = pose = np.asarray(start, dtype=float)
    for k in range(steps):
        v, w = commands_at(track, model, t0 + (k + 0.5) * STEP)  # * the command at mid-step: no half-step lag
        out[k + 1] = pose = model.base.step(pose, v, w, STEP)
    return out


def errors(track: Track, model: Model, horizons=HORIZONS) -> Errors:
    """How far the model predicts wrong, from start poses EVERY seconds apart while following."""
    longest = max(horizons)
    candidates = np.flatnonzero(track.moving & (track.pose_time + longest <= track.pose_time[-1]))
    starts, last = [], -math.inf
    for index in candidates:
        if track.pose_time[index] - last >= EVERY:
            starts.append(index)
            last = track.pose_time[index]
    if not starts:
        return Errors.join([], horizons)
    starts = np.array(starts)
    t0 = track.pose_time[starts]
    poses = predict(track, model, track.pose[starts], t0, longest)
    position, heading = [], []
    for horizon in horizons:
        k = int(round(horizon / STEP))
        predicted, measured = poses[k], measured_at(track, t0 + k * STEP)
        position.append(np.hypot(*(predicted[:, :2] - measured[:, :2]).T))
        heading.append(np.abs(np.remainder(predicted[:, 2] - measured[:, 2] + math.pi, 2 * math.pi) - math.pi))
    return Errors(tuple(horizons), np.column_stack(position), np.column_stack(heading), track.kinds[starts])


def score(tracks: list[Track], model: Model, horizons=HORIZONS) -> Errors:
    """`errors` of several runs together."""
    return Errors.join([errors(t, model, horizons) for t in tracks], horizons)


def free_run(track: Track, model: Model) -> tuple[np.ndarray, np.ndarray]:
    """The whole run replayed from its first pose while following, never corrected.

    Returns:
        tuple[np.ndarray, np.ndarray]: The pose times from that pose on, and the predicted pose at each (N, 3).
    """
    first = int(np.argmax(track.moving)) if track.moving.any() else 0
    t0 = track.pose_time[first]
    poses = predict(track, model, track.pose[first][None], np.array([t0]), track.pose_time[-1] - t0)[:, 0]
    grid = t0 + STEP * np.arange(len(poses))
    times = track.pose_time[first:]
    return times, np.column_stack([np.interp(times, grid, poses[:, i]) for i in range(3)])


def synthetic(track: Track, model: Model) -> Track:
    """The run as if the robot were exactly `model`: its poses replaced by the model's replay of its commands.

    Checks the evaluation itself: identifying such runs must give `model` back, and replaying them with it no error.
    """
    times, poses = free_run(track, model)
    first = len(track.pose_time) - len(times)
    return replace(track, pose_time=times, pose=poses, moving=track.moving[first:], kinds=track.kinds[first:])


def _ascending(times: np.ndarray, values: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Finite rows in time order, one per distinct time."""
    ok = np.isfinite(times) & np.isfinite(values).all(axis=1)
    times, first = np.unique(times[ok], return_index=True)
    return times, values[ok][first]
