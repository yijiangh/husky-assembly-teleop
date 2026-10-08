"""
The report of one base experiment run: `experiment.json` and `overview.png` beside its `recording.npz`. No ROS.

- experiment.json: the run's properties (sim or real, sim model, controller and its parameters, template), its
  outcome and tracking numbers.
- overview.png: the path sent and where the robot went, then the monitor's position and heading errors and the
  commands over time; turns on the spot are shaded grey, settling yellow.

! Uses matplotlib's `Figure` directly, not pyplot, so it is safe on a worker thread.

Redo reports by hand:  python3 -m husky_assembly_teleop.plugins.base_exp.report <run folder or .npz>...
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from crl_husky.follower_path import FollowerPath
from matplotlib.figure import Figure

#: Heading arrows drawn per line in the top view.
ARROWS = 15
PATH_COLOR, ROBOT_COLOR = "tab:blue", "tab:orange"
TURN_SHADE = "0.9"          # turns on the spot
SETTLE_SHADE = "#fff3bf"    # settling after the follower ended


#: Segment types the errors are split into, by the path where the robot is: turns on the spot, and drives by their
#: curvature (1/m): straight below CURVE, tight from TIGHT_CURVE on (radius 0.67 m and less).
SEGMENTS = ("straight", "curve", "tight curve", "spot turn")
CURVE, TIGHT_CURVE = 0.2, 1.5
#: Phases of a run (see the plugin).
FOLLOWING, SETTLING = 1, 2


@dataclass
class RunMetrics:
    """Tracking numbers of one run, from the monitor's measurement (the same for every controller).

    "Motion" means while the follower followed the path; "end" is the pose after the robot settled.

    Attributes:
        duration: Seconds the follower followed the path.
        expected: Seconds the run should take at the speeds set.
        settle_time: Seconds from the follower's end until the robot stood still (or recording gave up).
        motion_position: Mean position error while moving, metres.
        motion_heading: Mean heading error while moving, degrees.
        max_position: Largest position error while moving, metres.
        max_heading: Largest heading error while moving, degrees.
        rms_along_track: Root mean square of the lag behind schedule, metres; NaN for a geometric path.
        end_position: Distance from the path's last pose after settling, metres.
        end_heading: Heading difference to the path's last pose after settling, degrees.
        segments: Per segment type (SEGMENTS): mean position error (m), mean heading error (°) and sample count
            while moving.
    """

    duration: float
    expected: float
    settle_time: float
    motion_position: float
    motion_heading: float
    max_position: float
    max_heading: float
    rms_along_track: float
    end_position: float
    end_heading: float
    segments: dict


def write_report(recording: str | Path) -> tuple[Path, Path]:
    """Write experiment.json and overview.png beside a run's recording.

    Args:
        recording: The run's `recording.npz`, or the folder holding it.

    Returns:
        tuple[Path, Path]: The JSON and the PNG written.
    """
    recording = Path(recording)
    if recording.is_dir():
        recording = recording / "recording.npz"
    run = load(recording)
    report = {**properties(run), "outcome": str(run["outcome"]), "metrics": asdict(metrics(run))}
    data = recording.with_name("experiment.json")
    data.write_text(json.dumps(report, indent=2, default=_plain) + "\n")
    return data, plot(recording, recording.with_name("overview.png"))


def properties(run: dict[str, np.ndarray]) -> dict:
    """The run's properties as saved by the plugin; {} for recordings from before they were saved."""
    return json.loads(str(run["experiment"])) if "experiment" in run else {}


def load(path: str | Path) -> dict[str, np.ndarray]:
    """The recording's arrays, with the robot's serial dropped from the signal names."""
    with np.load(path) as data:
        pose_key = next(key for key in data.files if key.endswith("_floor_pose"))
        prefix = pose_key[:-len("floor_pose")]
        return {key.removeprefix(prefix): data[key] for key in data.files}


def metrics(run: dict[str, np.ndarray]) -> RunMetrics:
    """Tracking numbers of a loaded recording."""
    t, pose = run["t"], run["floor_pose"]
    position = np.abs(run["tracking_position"][:, 0])
    heading = np.abs(np.degrees(run["tracking_heading"][:, 0]))
    context = run["tracking_context"]
    phase = context[:, 5]
    moving = phase == FOLLOWING
    settling = phase == SETTLING
    known = pose[np.isfinite(pose).all(axis=1)]
    end = known[-1] if len(known) else np.full(3, np.nan)
    goal = run["path_poses"][-1]
    kinds = segment_types(context)
    segments = {}
    for kind in SEGMENTS:
        pick = moving & (kinds == kind)
        segments[kind] = {"position": _mean(position[pick]), "heading": _mean(heading[pick]),
                          "samples": int(pick.sum())}
    return RunMetrics(
        duration=_span(t[moving]),
        expected=float(run.get("expected_duration", np.nan)),
        settle_time=_span(np.concatenate([t[moving][-1:], t[settling]])),
        motion_position=_mean(position[moving]),
        motion_heading=_mean(heading[moving]),
        max_position=_max_abs(position[moving]),
        max_heading=_max_abs(heading[moving]),
        rms_along_track=_rms(run["tracking_position"][moving, 1]),
        end_position=float(np.hypot(*(end[:2] - goal[:2]))),
        end_heading=math.degrees(abs(math.remainder(float(end[2] - goal[2]), 2 * math.pi))),
        segments=segments,
    )


def segment_types(context: np.ndarray) -> np.ndarray:
    """Per sample of `tracking_context`, its segment type (one of SEGMENTS)."""
    turning, curvature = context[:, 0] > 0.5, np.abs(context[:, 1])
    kinds = np.where(curvature >= TIGHT_CURVE, "tight curve", np.where(curvature >= CURVE, "curve", "straight"))
    return np.where(turning, "spot turn", kinds).astype(object)


def plot(path: str | Path, out: str | Path | None = None) -> Path:
    """Draw the overview of a recording.

    Args:
        path: The recording.
        out: The PNG to write; `<recording>.png` if None.

    Returns:
        Path: The PNG written.
    """
    path = Path(path)
    run = load(path)
    numbers = metrics(run)
    t = run["t"] - run["t"][0]
    pose, poses = run["floor_pose"], run["path_poses"]
    timed = run["path_t"].size > 0
    followed = FollowerPath.from_poses(*poses.T, run["path_t"] if timed else None)

    figure = Figure(figsize=(8, 12), layout="constrained")
    grid = figure.add_gridspec(4, 1, height_ratios=(3.0, 1.0, 1.0, 1.0))
    top = figure.add_subplot(grid[0])
    error = figure.add_subplot(grid[1])
    yaw = figure.add_subplot(grid[2], sharex=error)
    command = figure.add_subplot(grid[3], sharex=error)

    # * Top view.
    line = followed.polyline()
    top.plot(line[:, 0], line[:, 1], "--", color=PATH_COLOR, label="path sent")
    _arrows(top, line, PATH_COLOR)
    known = pose[np.isfinite(pose).all(axis=1)]
    if len(known):
        top.plot(known[:, 0], known[:, 1], "-", color=ROBOT_COLOR, label="robot")
        _arrows(top, known, ROBOT_COLOR)
        top.plot(*known[0, :2], "o", color=ROBOT_COLOR, label="start")
        top.plot(*known[-1, :2], "s", color=ROBOT_COLOR, label="end")
    top.plot(*poses[-1, :2], "x", color=PATH_COLOR, markersize=10, label="goal")
    top.set_aspect("equal", adjustable="datalim")
    top.set_xlabel("x [m]")
    top.set_ylabel("y [m]")
    top.grid(True, alpha=0.35)
    top.legend(loc="best", fontsize=8)
    top.set_title(f"{run['path_label']}  ·  {str(run['outcome']).split(' → ')[0]}\n{_setup(properties(run))}\n"
                  f"{numbers.duration:.1f} s (expected {numbers.expected:.1f} s)  ·  "
                  f"end {numbers.end_position * 100:.1f} cm, {numbers.end_heading:.1f}° after settling", fontsize=10)

    # * Errors and commands over time, measured by the monitor.
    position_error = run["tracking_position"]
    error.plot(t, position_error[:, 0] * 100, color="tab:red", label="position (signed while driving)")
    if timed:
        error.plot(t, position_error[:, 1] * 100, color="tab:purple", label="along-track (behind schedule)")
    error.set_ylabel("position error [cm]")
    error.set_title(f"while moving: mean {numbers.motion_position * 100:.1f} cm, "
                    f"max {numbers.max_position * 100:.1f} cm", fontsize=9)
    error.legend(loc="upper right", fontsize=8)

    yaw.plot(t, np.degrees(run["tracking_heading"][:, 0]), color="tab:green")
    yaw.set_ylabel("heading error [°]")
    yaw.set_title(f"while moving: mean {numbers.motion_heading:.1f}°, max {numbers.max_heading:.1f}°", fontsize=9)

    commands = run["follower_command"]
    command.plot(t, commands[:, 0], color="tab:green", label="v [m/s]")
    command.set_ylabel("v [m/s]", color="tab:green")
    command.tick_params(axis="y", colors="tab:green")
    turn_rate = command.twinx()
    turn_rate.plot(t, np.degrees(commands[:, 1]), color="tab:orange", label="ω [°/s]")
    turn_rate.set_ylabel("ω [°/s]", color="tab:orange")
    turn_rate.tick_params(axis="y", colors="tab:orange")
    command.set_xlabel("time [s]")
    command.set_title(_caps(run), fontsize=9)

    context = run["tracking_context"]
    for axis in (error, yaw, command):
        axis.grid(True, alpha=0.35)
        axis.axhline(0.0, color="0.3", linewidth=0.8)
        for start, end in _spans(t, (context[:, 0] > 0.5) & (context[:, 5] == FOLLOWING)):
            axis.axvspan(start, end, color=TURN_SHADE, zorder=0)
        for start, end in _spans(t, context[:, 5] == SETTLING):
            axis.axvspan(start, end, color=SETTLE_SHADE, zorder=0)

    out = Path(out) if out is not None else path.with_suffix(".png")
    figure.savefig(out, dpi=150)
    return out


def _setup(props: dict) -> str:
    """One line on where and with what the run was made, e.g. "sim (xICR 0, speed 1, steering 1) · pure_pursuit"."""
    if not props:
        return "setup not recorded"
    where = props.get("environment", "?")
    model = props.get("sim_model")
    if where == "sim" and isinstance(model, dict):
        where += (f" (xICR {model.get('xICR', '?')}, speed {model.get('speed_efficiency', '?')}, "
                  f"steering {model.get('steering_efficiency', '?')})")
    controller = props.get("controller", {})
    name = controller.get("name", "?")
    parameters = controller.get("parameters")
    if isinstance(parameters, dict) and "lookahead" in parameters:
        name += f" (lookahead {parameters['lookahead']} m)"
    return f"{where}  ·  {name}"


def _plain(value):
    """JSON fallback: numpy numbers and arrays as plain Python."""
    return value.item() if isinstance(value, np.generic) else np.asarray(value).tolist()


def _arrows(axis, poses: np.ndarray, color: str) -> None:
    """About ARROWS heading arrows along `poses` (n x 3)."""
    pick = np.unique(np.linspace(0, len(poses) - 1, min(ARROWS, len(poses))).astype(int))
    span = np.ptp(poses[:, :2], axis=0).max() if len(poses) > 1 else 0.0
    length = max(0.05, 0.06 * span)
    axis.quiver(poses[pick, 0], poses[pick, 1], length * np.cos(poses[pick, 2]), length * np.sin(poses[pick, 2]),
                color=color, alpha=0.6, angles="xy", scale_units="xy", scale=1.0, width=0.004)


def _spans(t: np.ndarray, flags: np.ndarray) -> list[tuple[float, float]]:
    """Time spans where `flags` holds."""
    spans, start = [], None
    for time, flag in zip(t, flags):
        if flag and start is None:
            start = time
        elif not flag and start is not None:
            spans.append((start, time))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


def _caps(run: dict[str, np.ndarray]) -> str:
    """The speed caps sent with the path, as a title."""
    linear, angular = (float(v) for v in run["speed_caps"])
    if linear <= 0 and angular <= 0:
        return "commands (follower's own limits)"
    return f"commands (caps {linear:.2f} m/s, {math.degrees(angular):.0f}°/s)"


def _mean(values: np.ndarray) -> float:
    """Mean of the finite values, NaN if there are none."""
    finite = values[np.isfinite(values)]
    return float(finite.mean()) if finite.size else float("nan")


def _span(times: np.ndarray) -> float:
    """Seconds from the first to the last time, 0 for fewer than two."""
    return float(times[-1] - times[0]) if len(times) > 1 else 0.0


def _max_abs(values: np.ndarray) -> float:
    """Largest magnitude of the finite values, NaN if there are none."""
    finite = values[np.isfinite(values)]
    return float(np.max(np.abs(finite))) if finite.size else float("nan")


def _rms(values: np.ndarray) -> float:
    """Root mean square of the finite values, NaN if there are none."""
    finite = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(finite ** 2))) if finite.size else float("nan")


if __name__ == "__main__":
    for name in sys.argv[1:]:
        target = Path(name)
        if target.is_dir() or target.name == "recording.npz":
            print(*write_report(target))
        else:
            print(plot(target))  # a recording from before runs had their own folder
