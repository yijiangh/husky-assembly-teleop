"""
How the robot responds to commands, identified from recorded runs.

Delay, speed and steering efficiency, turning-centre offset. Plain numpy; experiment code, which may later move onto
the robot.

The model is the simulator's (`crl_husky.base_model`), so the values can be given to it as they are. In the frame of
the tracked point, with each input acting `delay` seconds after it was sent:

- forward speed  = speed_efficiency · v
- sideways speed = xICR · ω   (xICR: how far the tracked point lies ahead of the point the robot turns about;
  negative when it lies behind)
- turn rate      = steering_efficiency · ω

The input is the follower's commands, or the wheel odometry (`SOURCES`); with both, the delay splits into command to
wheels and wheels to motion. Runs are read as `replay.Track`s, so the timing rules there apply.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .replay import HORIZONS, Errors, Model, Track, score, track

#: Delays tried, seconds; the one that fits best is taken. From the wheels also negative ones: odometry that arrives
#: after the motion it measures (filtered or stamped late) shows as a negative delay.
DELAYS = np.arange(0.0, 0.601, 0.01)
WHEEL_DELAYS = np.arange(-0.4, 0.601, 0.01)
#: Samples that excite a channel: driving faster than this (m/s), turning faster than this (rad/s).
DRIVING, TURNING = 0.05, 0.1
#: Fewer exciting samples than this leave a value unknown (NaN).
ENOUGH = 20
#: What drives what: input -> output. "motion" is the mocap-measured motion of the tracked point.
SOURCES = {"commands → motion": ("commands", "motion"), "commands → wheels": ("commands", "wheels"),
           "wheels → motion": ("wheels", "motion")}


@dataclass
class Response:
    """The identified response; NaN where the runs did not excite it enough (or, for wheels, where it has no meaning).

    Attributes:
        delay: Seconds from an input to the motion it causes.
        speed_efficiency: Forward speed per input speed.
        steering_efficiency: Turn rate per input turn rate.
        x_icr: Distance of the tracked point ahead of the turning centre, metres; NaN for the wheels as output.
        fit: Share of the variance explained (R²) per channel: "forward", "sideways", "turn".
        samples: Samples used; driving and turning: those exciting each channel.
        clock: "stamps" when every run had capture and command stamps, else "tick" (delay includes latencies).
    """

    delay: float
    speed_efficiency: float
    steering_efficiency: float
    x_icr: float
    fit: dict
    samples: int
    driving: int
    turning: int
    clock: str

    @property
    def model(self) -> Model:
        """As a replay model; unknown values are the ideal ones."""
        return Model.of(self.x_icr, self.speed_efficiency, self.steering_efficiency, self.delay)


def identify(runs: list, source: str = "commands → motion") -> Response | None:
    """Fit the model to some runs together, at the delay that explains the output best.

    Args:
        runs: Loaded recordings (`report.load`) or Tracks.
        source: One of SOURCES.

    Returns:
        Response | None: The identified values; None if no run has the data or enough samples.
    """
    tracks = _tracks(runs)
    data = [d for d in (_pairs(t, *SOURCES[source]) for t in tracks) if d is not None]
    if not data:
        return None
    best = None
    for delay in WHEEL_DELAYS if source.startswith("wheels") else DELAYS:
        stacked = _stack(data, delay)
        result = _fit(*stacked)
        fits = [value for value in result[3].values() if not math.isnan(value)]
        explained = np.mean(fits) if fits else -math.inf
        if best is None or explained > best[0]:
            best = (explained, delay, result, stacked)
    _, delay, (speed, steering, x_icr, fit), stacked = best
    v, w = stacked[3], stacked[4]
    return Response(delay=float(delay), speed_efficiency=speed, steering_efficiency=steering, x_icr=x_icr, fit=fit,
                    samples=len(v), driving=int((np.abs(v) > DRIVING).sum()),
                    turning=int((np.abs(w) > TURNING).sum()),
                    clock="stamps" if all(t.clock == "stamps" for t in tracks) else "tick")


def cross_validated(runs: list, horizons=HORIZONS) -> Errors | None:
    """Replay errors of models identified on the other half of the runs (alternate runs), so no run scores its own fit.

    Returns:
        Errors | None: Both halves' errors; None with fewer than two runs or when a half identifies nothing.
    """
    tracks = _tracks(runs)
    halves = (tracks[0::2], tracks[1::2])
    if not all(halves):
        return None
    parts = []
    for scored, fitted in (halves, halves[::-1]):
        found = identify(fitted)
        if found is None:
            return None
        parts.append(score(scored, found.model, horizons))
    return Errors.join(parts, horizons)


def _tracks(runs: list) -> list[Track]:
    """Runs as Tracks, the ones with too few poses left out."""
    tracks = [run if isinstance(run, Track) else track(run) for run in runs]
    return [t for t in tracks if t is not None]


def _pairs(t: Track, source: str, output: str) -> tuple[np.ndarray, ...] | None:
    """One run's output velocities (times, forward, sideways, turn), input (times, v, ω), and if the input is held."""
    input_time, values = (t.command_time, t.command) if source == "commands" else (t.odometry_time, t.odometry)
    if len(input_time) == 0:
        return None
    if output == "motion":
        yaw = t.pose[:, 2]
        vx, vy, turn = (np.gradient(c, t.pose_time) for c in (t.pose[:, 0], t.pose[:, 1], yaw))
        forward = np.cos(yaw) * vx + np.sin(yaw) * vy
        sideways = -np.sin(yaw) * vx + np.cos(yaw) * vy
        output_time = t.pose_time
    else:
        if len(t.odometry_time) == 0:
            return None
        # * Wheels only while the run was followed or settling, like the poses.
        inside = (t.odometry_time >= t.pose_time[0]) & (t.odometry_time <= t.pose_time[-1])
        output_time = t.odometry_time[inside]
        forward, turn = t.odometry[inside, 0], t.odometry[inside, 1]
        sideways = np.full(len(output_time), np.nan)
    return output_time, forward, sideways, turn, input_time, values[:, 0], values[:, 1], source == "commands"


def _stack(data: list[tuple], delay: float) -> tuple[np.ndarray, ...]:
    """Output velocities of every run, each with the input acting on it `delay` seconds earlier.

    * Commands are held until the next one, as the robot holds them and as `replay` does; wheel odometry measures a
      continuous speed, so it is interpolated.
    """
    columns = [[], [], [], [], []]
    for output_time, forward, sideways, turn, input_time, v, w, held in data:
        acted = output_time - delay
        if held:
            index = np.searchsorted(input_time, acted, side="right") - 1
            valid = index >= 0
            inputs = v[index[valid]], w[index[valid]]
        else:
            valid = (acted >= input_time[0]) & (acted <= input_time[-1])
            inputs = np.interp(acted[valid], input_time, v), np.interp(acted[valid], input_time, w)
        for column, values in zip(columns, (forward[valid], sideways[valid], turn[valid], *inputs)):
            column.append(values)
    return tuple(np.concatenate(column) for column in columns)


def _fit(forward, sideways, turn, v, w) -> tuple[float, float, float, dict]:
    """Least-squares gains through the origin: forward on v, sideways and turn on ω; NaN without excitation."""
    driving, turning = np.abs(v) > DRIVING, np.abs(w) > TURNING
    speed, r2_forward = _gain(v, forward, driving)
    x_icr, r2_sideways = _gain(w, sideways, turning)
    steering, r2_turn = _gain(w, turn, turning)
    return speed, steering, x_icr, {"forward": r2_forward, "sideways": r2_sideways, "turn": r2_turn}


def _gain(command: np.ndarray, measured: np.ndarray, exciting: np.ndarray) -> tuple[float, float]:
    """The gain k in measured = k · command, and the share of variance it explains; NaN if not excited."""
    known = np.isfinite(measured)
    if (exciting & known).sum() < ENOUGH:
        return math.nan, math.nan
    command, measured = command[known], measured[known]
    k = float(np.dot(command, measured) / max(np.dot(command, command), 1e-12))
    residual = measured - k * command
    spread = np.sum((measured - measured.mean()) ** 2)
    return k, float(1.0 - np.sum(residual ** 2) / spread) if spread > 0 else math.nan
