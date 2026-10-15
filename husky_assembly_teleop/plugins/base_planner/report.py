"""
The report of one recorded base planner run, and the picture of one plan attempt. No ROS.

- A run: `report.json` and `overview.png` beside its `recording.npz` (`write_report`).
- A plan attempt: `plan.png` beside its `plan.npz` (`plot_plan`), found or not.
- An offline attempt's simulated runs: `SimulatedMetrics` of each trace in its `simulated.npz`.

Measured by mocap (or the simulated pose) against the path sent, the same for every planner and controller:

- report.json: the attempt's description (experiment.json) and the run's metrics.
- overview.png: the path sent and where the robot went, then its distance from the path, its heading error, and the
  commands over time; turns on the spot are shaded grey.

! Uses matplotlib's `Figure` directly, not pyplot, so it is safe on a worker thread.

Redo reports by hand:  python3 -m husky_assembly_teleop.plugins.base_planner.report <run folder>...
"""

from __future__ import annotations

import json
import math
import re
import sys
import textwrap
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
from matplotlib.figure import Figure

from ..base_exp.report import heading_arrows, time_spans
from .path import densify
from .recording import load_description, load_plan_arrays

PATH_COLOR, ROBOT_COLOR = "tab:blue", "tab:orange"
TURN_SHADE = "0.9"
#: Follower states in the progress signal.
FOLLOWING, DONE_STATE = 1, 2
#: A command is a stop below STILL m/s and rad/s, a turn on the spot below STILL m/s and from TURNING rad/s.
STILL, TURNING = 0.01, 0.05
#: A stop counts once it lasts STOP_TIME seconds.
STOP_TIME = 0.2
#: Spacing of the path the robot is measured against, metres and radians.
STEP = 0.01


@dataclass
class RunMetrics:
    """Numbers of one run, from mocap (base_footprint) against the path sent.

    "While following" is the span the follower reported following.

    Attributes:
        turns_about: How the path's turns on the spot were sent: "base_footprint" (on one spot, as the RRT sends them)
            or "turning centre" (base_footprint swings on a small circle, as the lattice sends them); "none".
        planned_length: Metres the path drives.
        driven_length: Metres the robot drove while following.
        planned_duration: Seconds the path takes at the planning speeds.
        duration: Seconds the follower followed.
        pieces: Drive and turn pieces the follower cut the path into.
        stops: Times the commands stood still for STOP_TIME or longer while following.
        spot_turn_time: Seconds the commands turned on the spot while following.
        mean_deviation: Mean distance from the path while following, metres.
        p95_deviation: 95th percentile of that distance, metres.
        max_deviation: Largest distance from the path while following, metres.
        mean_heading: Mean heading error against the nearest path pose while following, degrees.
        max_heading: Largest such heading error, degrees.
        follower_cross_track: The follower's own mean absolute cross-track error, of the point it steers, metres.
        end_position: Distance of the last recorded pose from the target, metres.
        end_heading: Heading difference of the last recorded pose to the target, degrees.
        settle_drift: How far the robot moved after the follower reported done, metres; NaN if it never did.
        speed_ratio: Distance driven (mocap) over distance commanded while following.
        odometry_ratio: Distance driven (mocap) over distance the wheel odometry reports while following.
    """

    turns_about: str
    planned_length: float
    driven_length: float
    planned_duration: float
    duration: float
    pieces: int
    stops: int
    spot_turn_time: float
    mean_deviation: float
    p95_deviation: float
    max_deviation: float
    mean_heading: float
    max_heading: float
    follower_cross_track: float
    end_position: float
    end_heading: float
    settle_drift: float
    speed_ratio: float
    odometry_ratio: float


@dataclass
class SimulatedMetrics:
    """Numbers of one simulated drive of a path (`follow.followed`) against the path.

    Attributes:
        finished: Whether the simulated follower finished.
        max_deviation: Largest distance of base_footprint from the path, metres.
        end_position: Distance of the last pose from the target, metres.
        end_heading: Heading difference of the last pose to the target, degrees.
        duration: Seconds the drive took.
    """

    finished: bool
    max_deviation: float
    end_position: float
    end_heading: float
    duration: float


def load(folder: str | Path) -> dict:
    """A run's arrays, with the robot's serial dropped from the signal names, and its description as "run".

    "run" holds the attempt's setup, its run's outcome and controller, and the path's goal, length and planned
    duration; "plan" the plan's record (None for runs recorded before plans were) and "plan_arrays" its plan.npz.
    """
    folder = Path(folder)
    with np.load(folder / "recording.npz") as data:
        pose_key = next(key for key in data.files if key.endswith("_floor_pose"))
        prefix = pose_key[:-len("floor_pose")]
        run = {key.removeprefix(prefix): data[key] for key in data.files}
    description = load_description(folder)
    if description is None:
        raise FileNotFoundError(f"{folder} has no experiment.json")
    poses, times = run["path_poses"], run["path_t"]
    length = float(np.sum(np.hypot(*np.diff(poses[:, :2], axis=0).T)))
    run["run"] = {**description, **(description.get("run") or {}), "length": length,
                  "planned_duration": float(times[-1]) if len(times) else math.nan,
                  "plan_seconds": (description.get("plan") or {}).get("seconds")}
    run["plan"] = description.get("plan")
    run["plan_arrays"] = load_plan_arrays(folder) or (load_plan_arrays(folder.parent / description["plan_folder"])
                                                      if description.get("plan_folder") else None)
    return run


def simulated_metrics(poses: np.ndarray, goal, trace: np.ndarray, finished: bool,
                      rate: float = 30.0) -> SimulatedMetrics:
    """The numbers of one simulated drive: `trace` (base_footprint every 1 / `rate` s) against path `poses`."""
    distances, _ = deviation({"path_poses": poses, "floor_pose": trace})
    end = trace[-1]
    return SimulatedMetrics(finished=finished, max_deviation=float(np.nanmax(distances)),
                            end_position=float(np.hypot(end[0] - goal[0], end[1] - goal[1])),
                            end_heading=math.degrees(abs(math.remainder(float(end[2] - goal[2]), 2 * math.pi))),
                            duration=len(trace) / rate)


def planner(record: dict | None) -> str:
    """The planner a plan or run names, as a label: "Lattice (hybrid A*)", or "not recorded"."""
    return (record or {}).get("algorithm") or "not recorded"


def draw_floor(axis, arrays: dict | None) -> None:
    """Shade what a plan's floor map has on the floor, below everything else drawn; nothing without a map."""
    if arrays is None or "floor_heights" not in arrays:
        return
    occupied = np.isfinite(arrays["floor_heights"]).T
    low, cell = arrays["floor_low"], float(arrays["floor_cell"])
    extent = (low[0] - cell / 2, low[0] + (occupied.shape[1] - 0.5) * cell,
              low[1] - cell / 2, low[1] + (occupied.shape[0] - 0.5) * cell)
    axis.imshow(np.ma.masked_where(~occupied, occupied), origin="lower", extent=extent, cmap="Greys", vmin=0, vmax=3,
                interpolation="nearest", zorder=0)


def plot_plan(folder: str | Path) -> Path:
    """Draw a plan attempt over its floor map: start, target, the path if found, and what came of it.

    Returns:
        Path: The plan.png written.
    """
    folder = Path(folder)
    description, arrays = load_description(folder), load_plan_arrays(folder)
    record = {**description, **description["plan"]}
    figure = Figure(figsize=(7, 7), layout="constrained")
    axis = figure.add_subplot()
    draw_floor(axis, arrays)
    poses = arrays["path_poses"]
    if len(poses):
        axis.plot(poses[:, 0], poses[:, 1], "-", color=PATH_COLOR, label="path")
        heading_arrows(axis, poses, PATH_COLOR)
    for pose, marker, label in ((record["start"], "o", "start"), (record["goal"], "x", "target")):
        axis.plot(*pose[:2], marker, color=ROBOT_COLOR, markersize=9, label=label)
        axis.annotate("", (pose[0] + 0.3 * math.cos(pose[2]), pose[1] + 0.3 * math.sin(pose[2])), pose[:2],
                      arrowprops={"arrowstyle": "->", "color": ROBOT_COLOR})
    if record.get("clearance_at"):
        axis.plot(*record["clearance_at"][:2], "v", color="tab:red", label="least clearance")
    # * A failed search may say where it got closest: "closest ... at (x, y, yaw°)".
    closest = re.search(r"closest .*? at \((-?[\d.]+), (-?[\d.]+), (-?[\d.]+)°\)", record.get("reason", ""))
    if closest:
        x, y, yaw = (float(value) for value in closest.groups())
        axis.plot(x, y, "D", color="tab:purple", label="closest the search got")
        axis.annotate("", (x + 0.3 * math.cos(math.radians(yaw)), y + 0.3 * math.sin(math.radians(yaw))), (x, y),
                      arrowprops={"arrowstyle": "->", "color": "tab:purple"})
    axis.set_aspect("equal", adjustable="datalim")
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.grid(True, alpha=0.35)
    axis.legend(loc="best", fontsize=8)
    outcome = (f"found in {record['seconds']:.1f} s: {record['length']:.2f} m, {record['duration']:.1f} s, clearance "
               f"{record['clearance'] * 100:.1f} cm" if record["found"] and record.get("clearance") is not None
               else f"found in {record['seconds']:.1f} s" if record["found"]
               else "\n".join(textwrap.wrap(f"no path after {record['seconds']:.1f} s: {record['reason']}", 100)))
    task = f"task {record['task']}" if record.get("task") else "no task"
    axis.set_title(f"{folder.name}  ·  {task}  ·  {planner(record)}\n{settings_line(record.get('settings', {}))}\n"
                   f"{outcome}", fontsize=9)
    out = folder / "plan.png"
    figure.savefig(out, dpi=130)
    return out


def settings_line(settings: dict) -> str:
    """A plan's settings as one short line, e.g. "x_icr -0.2..-0.1  follower_model alice_tiles  margin 0.05"."""
    def value(item):
        if isinstance(item, (list, tuple)):
            return "..".join(f"{part:g}" if isinstance(part, float) else str(part) for part in item)
        return f"{item:g}" if isinstance(item, float) else str(item)
    return "  ".join(f"{key} {value(item)}" for key, item in settings.items())


def deviation(run: dict) -> tuple[np.ndarray, np.ndarray]:
    """Per sample, the distance from the path sent (m) and the heading error against its nearest pose (rad)."""
    poses = run["path_poses"]
    reference = np.array([poses[0]] + densify([tuple(pose) for pose in poses], STEP, STEP))
    pose = run["floor_pose"]
    distances = np.full(len(pose), np.nan)
    headings = np.full(len(pose), np.nan)
    for i, sample in enumerate(pose):
        if not np.isfinite(sample).all():
            continue
        gaps = np.hypot(*(reference[:, :2] - sample[:2]).T)
        nearest = int(np.argmin(gaps))
        distances[i] = gaps[nearest]
        headings[i] = math.remainder(float(sample[2] - reference[nearest, 2]), 2.0 * math.pi)
    return distances, headings


def metrics(run: dict) -> RunMetrics:
    """The numbers of a loaded run."""
    t, pose = run["t"], run["floor_pose"]
    following = run["follower_progress"][:, 2] == FOLLOWING
    distances, headings = deviation(run)
    moving = following & np.isfinite(distances)
    known = pose[np.isfinite(pose).all(axis=1)]
    goal = np.array(run["run"]["goal"])
    commands = run["follower_command"]
    still = (np.abs(commands[:, 0]) < STILL) & (np.abs(commands[:, 1]) < STILL)
    spot = (np.abs(commands[:, 0]) < STILL) & (np.abs(commands[:, 1]) >= TURNING)
    dt = np.diff(t, append=t[-1])
    driven = _length(pose[following])
    return RunMetrics(
        turns_about=turns_about(run["path_poses"]),
        planned_length=float(run["run"]["length"]),
        driven_length=driven,
        planned_duration=float(run["run"]["planned_duration"]),
        duration=_span(t[following]),
        pieces=int(np.nanmax(run["follower_progress"][:, 1])) + 1,
        stops=sum(1 for start, end in time_spans(t, following & still) if end - start >= STOP_TIME),
        spot_turn_time=float(np.sum(dt[following & spot])),
        mean_deviation=_mean(distances[moving]),
        p95_deviation=float(np.percentile(distances[moving], 95)) if moving.any() else math.nan,
        max_deviation=float(np.max(distances[moving])) if moving.any() else math.nan,
        mean_heading=math.degrees(_mean(np.abs(headings[moving]))),
        max_heading=math.degrees(float(np.max(np.abs(headings[moving])))) if moving.any() else math.nan,
        follower_cross_track=_mean(np.abs(run["follower_position_error"][following, 0])),
        end_position=float(np.hypot(*(known[-1, :2] - goal[:2]))) if len(known) else math.nan,
        end_heading=math.degrees(abs(math.remainder(float(known[-1, 2] - goal[2]), 2 * math.pi)))
        if len(known) else math.nan,
        settle_drift=_settle_drift(t, pose, run["follower_progress"][:, 2]),
        speed_ratio=_ratio(driven, float(np.sum(np.abs(commands[following, 0]) * dt[following]))),
        odometry_ratio=_ratio(driven, _logged_distance(run["wheel_odometry"], t[following])),
    )


def turns_about(poses: np.ndarray) -> str:
    """How a path's turns on the spot were sent: about "base_footprint", the "turning centre", or "none"."""
    steps = np.hypot(*np.diff(poses[:, :2], axis=0).T)
    turns = np.abs(np.diff(poses[:, 2]))
    if np.any((steps < 1e-6) & (turns > 1e-6)):
        return "base_footprint"
    # A swing: base_footprint moves a few centimetres while the heading changes a lot.
    if np.any((steps > 1e-6) & (turns > 0.03) & (steps < 0.5 * turns)):
        return "turning centre"
    return "none"


def write_report(folder: str | Path) -> tuple[Path, Path]:
    """Write report.json and overview.png into a run's folder.

    Returns:
        tuple[Path, Path]: The JSON and the PNG written.
    """
    folder = Path(folder)
    run = load(folder)
    numbers = metrics(run)
    data = folder / "report.json"
    data.write_text(json.dumps({**run["run"], "metrics": asdict(numbers)}, indent=2) + "\n")
    return data, plot(run, numbers, folder / "overview.png", folder.name)


def plot(run: dict, numbers: RunMetrics, out: Path, title: str) -> Path:
    """Draw the overview of a run; returns `out`."""
    t = run["t"] - run["t"][0]
    pose, poses = run["floor_pose"], run["path_poses"]
    following = run["follower_progress"][:, 2] == FOLLOWING
    distances, headings = deviation(run)
    commands = run["follower_command"]
    spot = following & (np.abs(commands[:, 0]) < STILL) & (np.abs(commands[:, 1]) >= TURNING)

    figure = Figure(figsize=(8, 12), layout="constrained")
    grid = figure.add_gridspec(4, 1, height_ratios=(3.0, 1.0, 1.0, 1.0))
    top = figure.add_subplot(grid[0])
    error = figure.add_subplot(grid[1])
    yaw = figure.add_subplot(grid[2], sharex=error)
    command = figure.add_subplot(grid[3], sharex=error)

    # * Top view, over the floor as the plan saw it.
    draw_floor(top, run.get("plan_arrays"))
    top.plot(poses[:, 0], poses[:, 1], "--", color=PATH_COLOR, label="path sent")
    heading_arrows(top, poses, PATH_COLOR)
    known = pose[np.isfinite(pose).all(axis=1)]
    if len(known):
        top.plot(known[:, 0], known[:, 1], "-", color=ROBOT_COLOR, label="robot (mocap)")
        heading_arrows(top, known, ROBOT_COLOR)
        top.plot(*known[0, :2], "o", color=ROBOT_COLOR, label="start")
        top.plot(*known[-1, :2], "s", color=ROBOT_COLOR, label="end")
    top.plot(*poses[-1, :2], "x", color=PATH_COLOR, markersize=10, label="target")
    top.set_aspect("equal", adjustable="datalim")
    top.set_xlabel("x [m]")
    top.set_ylabel("y [m]")
    top.grid(True, alpha=0.35)
    top.legend(loc="best", fontsize=8)
    setup = run["run"]
    planned_in = f", planned in {setup['plan_seconds']:.1f} s" if setup.get("plan_seconds") is not None else ""
    top.set_title(f"{title}  ·  {setup.get('outcome', '?')}\n{setup.get('environment', '?')}  ·  "
                  f"{planner(setup)}{planned_in}  ·  {setup.get('controller', '?')}  ·  turns about "
                  f"{numbers.turns_about}\n"
                  f"{numbers.duration:.1f} s (planned {numbers.planned_duration:.1f} s)  ·  "
                  f"end {numbers.end_position * 100:.1f} cm, {numbers.end_heading:.1f}°, drift after done "
                  f"{numbers.settle_drift * 100:.1f} cm", fontsize=10)

    # * Errors and commands over time.
    error.plot(t, distances * 100, color="tab:red", label="distance from the path (mocap)")
    error.plot(t, np.abs(run["follower_position_error"][:, 0]) * 100, color="tab:purple", alpha=0.7,
               label="follower's own cross-track (point it steers)")
    error.set_ylabel("distance [cm]")
    error.set_title(f"while following: mean {numbers.mean_deviation * 100:.1f} cm, "
                    f"p95 {numbers.p95_deviation * 100:.1f} cm, max {numbers.max_deviation * 100:.1f} cm", fontsize=9)
    error.legend(loc="upper right", fontsize=8)

    yaw.plot(t, np.degrees(headings), color="tab:green")
    yaw.set_ylabel("heading error [°]")
    yaw.set_title(f"against the nearest path pose: mean {numbers.mean_heading:.1f}°, max {numbers.max_heading:.1f}°",
                  fontsize=9)

    command.plot(t, commands[:, 0], color="tab:green")
    command.set_ylabel("v [m/s]", color="tab:green")
    command.tick_params(axis="y", colors="tab:green")
    turn_rate = command.twinx()
    turn_rate.plot(t, np.degrees(commands[:, 1]), color="tab:orange")
    turn_rate.set_ylabel("ω [°/s]", color="tab:orange")
    turn_rate.tick_params(axis="y", colors="tab:orange")
    command.set_xlabel("time [s]")
    command.set_title(f"commands: {numbers.pieces} pieces, {numbers.stops} stops, "
                      f"{numbers.spot_turn_time:.1f} s turning on the spot (grey)", fontsize=9)
    for axis in (error, yaw, command):
        axis.grid(True, alpha=0.35)
        axis.axhline(0.0, color="0.3", linewidth=0.8)
        for start, end in time_spans(t, spot):
            axis.axvspan(start, end, color=TURN_SHADE, zorder=0)

    figure.savefig(out, dpi=150)
    return out


def _settle_drift(t: np.ndarray, pose: np.ndarray, states: np.ndarray) -> float:
    """Metres between the pose when the follower reported done and the last pose; NaN if it never did.

    ! At first the follower may still report the last path as done: take the first done after following.
    """
    following = np.flatnonzero(states == FOLLOWING)
    done = np.flatnonzero(states == DONE_STATE)
    done = done[done > following[-1]] if len(following) else done[:0]
    known = np.isfinite(pose).all(axis=1)
    if not len(done) or not known[done[0]:].any():
        return math.nan
    after = pose[done[0]:][known[done[0]:]]
    return float(np.hypot(*(after[-1, :2] - after[0, :2])))


def _length(pose: np.ndarray) -> float:
    """Metres along the finite poses."""
    known = pose[np.isfinite(pose).all(axis=1)]
    return float(np.sum(np.hypot(*np.diff(known[:, :2], axis=0).T))) if len(known) > 1 else 0.0


def _logged_distance(log: np.ndarray, times: np.ndarray) -> float:
    """Metres a log of (time, stamp, v, w) reports within the span of `times`."""
    if len(times) < 2 or not len(log):
        return math.nan
    inside = log[(log[:, 0] >= times[0]) & (log[:, 0] <= times[-1])]
    return float(np.sum(np.abs(inside[:-1, 2]) * np.diff(inside[:, 0]))) if len(inside) > 1 else math.nan


def _mean(values: np.ndarray) -> float:
    """Mean of the finite values, NaN if there are none."""
    finite = values[np.isfinite(values)]
    return float(finite.mean()) if finite.size else math.nan


def _span(times: np.ndarray) -> float:
    """Seconds from the first to the last time, 0 for fewer than two."""
    return float(times[-1] - times[0]) if len(times) > 1 else 0.0


def _ratio(a: float, b: float) -> float:
    """The ratio of `a` to `b`, NaN when `b` is not positive."""
    return a / b if b > 0 else math.nan


if __name__ == "__main__":
    for name in sys.argv[1:]:
        print(*write_report(name))
