"""
The constant-command report, constant_commands.md: how the robot's response changes with speed and turn rate.

Each constant-command run holds one command (v, ω) open loop, a cell of the command grid (`twist.py`). Per run:

- Steady state, from STEADY_AFTER seconds after the step to the stop: mean forward speed, sideways speed and turn rate
  of the tracked point (mocap), so speed efficiency = forward / v, steering efficiency = turn / ω, xICR = sideways / ω.
  With wheel odometry, the same through the wheels.
- Step response, of the turn (or of the distance on a straight):
  - delay (mocap): where the steady line of the angle turned (or distance driven), extended back, meets the start.
    That is the delay a pure-delay model needs: exact for a pure delay, plus τ for a lag, plus half the ramp for an
    acceleration limit.
  - wheel delay and rise (wheel odometry, a speed measured directly): seconds to 10 % of the steady speed, and from
    10 % to 90 %. A rise that grows with the step means an acceleration limit, a fixed one a lag.

Then per cell (mean and spread over its runs), trends over the grid, and left/right and forward/reverse.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import pandas as pd
from matplotlib.figure import Figure

from ..replay import track
from ..report import load
from .common import Experiment, markdown, robots, save

#: Seconds after the step before the steady window starts: delay (~0.25 s) and the ramp, with a margin.
STEADY_AFTER = 1.2
#: A steady window shorter than this leaves the run out, seconds.
SHORTEST_STEADY = 1.5
#: Commands that excite a channel: speed (m/s) and turn rate (rad/s) above these.
DRIVING, TURNING = 0.05, 0.1
#: The values per run, with label, factor to the unit, and how many decimals.
QUANTITIES = {"speed_efficiency": ("speed efficiency", 1.0, 2), "steering_efficiency": ("steering efficiency", 1.0, 2),
              "x_icr": ("xICR [cm]", 100.0, 1), "delay": ("delay [ms]", 1000.0, 0),
              "wheel_delay": ("wheel delay [ms]", 1000.0, 0), "wheel_rise": ("wheel rise [ms]", 1000.0, 0)}
#: Smallest colour or axis span per plotted value (in its unit), so a constant value is not stretched into noise.
MIN_SPAN = {"x_icr": 2.0, "steering_efficiency": 0.05, "speed_efficiency": 0.05}


@dataclass
class Steady:
    """One constant-command run's response; NaN where the command did not excite it.

    Attributes:
        forward, sideways, turn: Steady mean velocities of the tracked point, m/s and rad/s.
        speed_efficiency, steering_efficiency, x_icr: As in the model (`identify.Response`).
        wheel_v, wheel_w: Steady wheel odometry, m/s and rad/s; NaN without odometry.
        delay: The delay a pure-delay model needs, from mocap (see the module), seconds.
        wheel_delay, wheel_rise: From the wheel odometry: seconds to 10 % of the steady speed, and from 10 % to 90 %.
        steady: Seconds of the steady window.
    """

    forward: float
    sideways: float
    turn: float
    speed_efficiency: float
    steering_efficiency: float
    x_icr: float
    wheel_v: float
    wheel_w: float
    delay: float
    wheel_delay: float
    wheel_rise: float
    steady: float


def write(experiment: Experiment) -> int:
    """Write constant_commands.md, its CSV files and plots from the experiment's constant-command runs.

    Returns:
        int: Runs analysed.
    """
    out = experiment.out
    runs = responses(experiment.runs)
    runs.to_csv(out / "commands_runs.csv", index=False)
    tables = {"commands_cells": cell_table(runs), "commands_trends": trend_table(runs),
              "commands_symmetry": symmetry_table(runs), "commands_wheels": wheel_table(runs)}
    for name, table in tables.items():
        table.to_csv(out / f"{name}.csv", index=False)
    plots = [p for p in (_heatmaps(runs, out), _trends(runs, out), _steps(runs, out)) if p]
    left = runs[~runs.used]
    lines = [f"# Constant commands: {experiment.name}\n",
             experiment.intro()
             + (f" Not analysed: {len(left)} ({', '.join(sorted(set(left.why)))})." if len(left) else "") + "\n",
             "Each run holds one command (v, ω) open loop, sent from the monitor: a cell of the command grid. Steady "
             f"values from {STEADY_AFTER:g} s after the step to the stop. xICR: how far the tracked point lies ahead "
             "of the point the robot turns about (negative: behind). The delay includes the network from the monitor "
             "to the robot.\n",
             "## Per cell\n",
             "Mean ± standard deviation over the cell's runs; n runs.\n",
             markdown(tables["commands_cells"], "robot"), "",
             "## Does the model change over the grid?\n",
             "Per value: the spread of the cell means across the grid against the spread between runs of one cell "
             "(noise). A ratio well above 1 means the value depends on the command; the fit says how: the change "
             "across the grid's range of |v| and |ω| (value = a + b·|v| + c·|ω|).\n",
             markdown(tables["commands_trends"], "robot"), "",
             "## Left and right, forward and reverse\n",
             markdown(tables["commands_symmetry"], "robot"), ""]
    if not tables["commands_wheels"].empty:
        lines += ["## Through the wheels\n",
                  "Steady wheel odometry per command (commands → wheels), and motion per wheel motion (wheels → "
                  "motion: how much the skid steer loses).\n",
                  markdown(tables["commands_wheels"]), ""]
    lines += ["## Plots\n",
              "commands_steps.png: a wheel rise that grows with the step means an acceleration limit, a flat one a "
              "lag.\n", *[f"![{name}]({name})\n" for name in plots]]
    (out / "constant_commands.md").write_text("\n".join(lines))
    return int(runs.used.sum())


# --- --- --- --- --- RUNS --- --- --- --- ---

def responses(runs: pd.DataFrame) -> pd.DataFrame:
    """The constant-command runs (`common.load_runs` rows), one row each: robot, cell, and their `Steady` values."""
    rows = []
    for _, run in runs.iterrows():
        row = {"run": run.run, "folder": run.folder, "started": run.started, "robot": run.robot,
               "v": round(float(run.p_v), 3), "w": round(float(run.p_w), 3), "battery": run.battery,
               "outcome": run.outcome}
        found = steady(load(Path(run.folder) / "recording.npz")) if run.success else None
        row["used"] = found is not None
        row["why"] = "" if found is not None else ("not done" if not run.success else "too short or no mocap")
        row.update(asdict(found) if found is not None else {})
        rows.append(row)
    found = pd.DataFrame(rows)
    # * Every robot in the reports' order.
    return found.assign(order=found.robot.map({r: i for i, r in enumerate(robots(runs))})).sort_values(
        ["order", "v", "w"], kind="stable").drop(columns="order")


def steady(run: dict[str, np.ndarray]) -> Steady | None:
    """One recorded run's response (see `Steady`); None if it has no clear step or too short a steady part."""
    tr = track(run)
    if tr is None or len(tr.command_time) < 2:
        return None
    moving = np.abs(tr.command).max(axis=1) > 1e-9
    if not moving.any():
        return None
    step_at = tr.command_time[np.argmax(moving)]
    after = np.flatnonzero(~moving & (tr.command_time > step_at))
    stop_at = tr.command_time[after[0]] if len(after) else tr.command_time[-1]
    v, w = tr.command[np.argmax(moving)]
    if stop_at - step_at - STEADY_AFTER < SHORTEST_STEADY:
        return None

    t, yaw = tr.pose_time, tr.pose[:, 2]
    vx, vy, turn = (np.gradient(c, t) for c in (tr.pose[:, 0], tr.pose[:, 1], yaw))
    forward = np.cos(yaw) * vx + np.sin(yaw) * vy
    sideways = -np.sin(yaw) * vx + np.cos(yaw) * vy
    window = (t >= step_at + STEADY_AFTER) & (t <= stop_at)
    if window.sum() < 5:
        return None
    mean_forward, mean_sideways = float(forward[window].mean()), float(sideways[window].mean())
    mean_turn = float(np.polyfit(t[window], yaw[window], 1)[0])

    wheel_v = wheel_w = math.nan
    wheel_delay = wheel_rise = math.nan
    turning = abs(w) > TURNING
    if len(tr.odometry_time):
        steady = (tr.odometry_time >= step_at + STEADY_AFTER) & (tr.odometry_time <= stop_at)
        if steady.sum() >= 3:
            wheel_v, wheel_w = (float(x) for x in tr.odometry[steady].mean(axis=0))
            channel = tr.odometry[:, 1] if turning else tr.odometry[:, 0]
            wheel_delay, wheel_rise = _step(tr.odometry_time, channel, step_at, wheel_w if turning else wheel_v)

    # * Angle turned, or distance driven along the start heading: grows linearly once steady.
    start = t < step_at
    if not start.any():
        return None
    first = tr.pose[start][-1]
    if turning:
        progress = yaw - first[2]
    else:
        progress = (tr.pose[:, 0] - first[0]) * math.cos(first[2]) + (tr.pose[:, 1] - first[1]) * math.sin(first[2])
    slope, offset = np.polyfit(t[window], progress[window], 1)
    delay = float(-offset / slope - step_at) if abs(slope) > 1e-9 else math.nan
    return Steady(
        forward=mean_forward, sideways=mean_sideways, turn=mean_turn,
        speed_efficiency=mean_forward / v if abs(v) > DRIVING else math.nan,
        steering_efficiency=mean_turn / w if turning else math.nan,
        x_icr=mean_sideways / w if turning else math.nan,
        wheel_v=wheel_v, wheel_w=wheel_w, delay=delay, wheel_delay=wheel_delay, wheel_rise=wheel_rise,
        steady=float(stop_at - step_at - STEADY_AFTER))


def _step(t: np.ndarray, values: np.ndarray, step_at: float, steady: float) -> tuple[float, float]:
    """Seconds from the step to 10 % of `steady`, and from 10 % to 90 % (crossings interpolated); NaN if not reached."""
    if not math.isfinite(steady) or abs(steady) < 1e-6:
        return math.nan, math.nan
    share, after = values / steady, t >= step_at
    crossings = []
    for level in (0.1, 0.9):
        reached = np.flatnonzero(after & (share >= level))
        if len(reached) == 0:
            return math.nan, math.nan
        i = reached[0]
        if i == 0 or share[i] == share[i - 1]:
            crossings.append(t[i])
        else:
            crossings.append(t[i - 1] + (level - share[i - 1]) / (share[i] - share[i - 1]) * (t[i] - t[i - 1]))
    return float(crossings[0] - step_at), float(crossings[1] - crossings[0])


# --- --- --- --- --- TABLES --- --- --- --- ---

def cell_table(runs: pd.DataFrame) -> pd.DataFrame:
    """Per robot and cell: n, and mean ± spread of each quantity."""
    rows = []
    for (robot, v, w), group in _used(runs).groupby(["robot", "v", "w"], sort=False):
        row = {"robot": robot, "v [m/s]": v, "ω [°/s]": round(math.degrees(w)), "n": len(group)}
        for key, (label, factor, digits) in QUANTITIES.items():
            row[label] = _mean_spread(group[key] * factor, digits)
        rows.append(row)
    return pd.DataFrame(rows)


def trend_table(runs: pd.DataFrame) -> pd.DataFrame:
    """Per robot and quantity: grid spread against noise, and the change across the grid's |v| and |ω| range."""
    rows = []
    for robot, mine in _used(runs).groupby("robot", sort=False):
        span_v, span_w = mine.v.abs().max() - mine.v.abs().min(), mine.w.abs().max() - mine.w.abs().min()
        for key, (label, factor, digits) in QUANTITIES.items():
            values = mine[["v", "w", key]].dropna()
            if len(values) < 4:
                continue
            means = values.groupby(["v", "w"])[key]
            between = float(means.mean().std()) * factor
            within = float(np.sqrt(np.nanmean(means.var().to_numpy()))) * factor if (means.size() > 1).any() \
                else math.nan
            a, b, c = _plane(values.v.abs(), values.w.abs(), values[key] * factor)
            rows.append({"robot": robot, "value": label, "mean": f"{float(values[key].mean() * factor):.{digits}f}",
                         "across cells": _round(between, digits + 1),
                         "within a cell": _round(within, digits + 1) if math.isfinite(within) else math.nan,
                         "ratio": round(between / within, 1) if math.isfinite(within) and within > 0 else math.nan,
                         f"change over |v| 0–{span_v:g} m/s": _round(b * span_v, digits + 1),
                         f"change over |ω| range ({math.degrees(span_w):.0f}°/s)": _round(c * span_w, digits + 1)})
    return pd.DataFrame(rows)


def symmetry_table(runs: pd.DataFrame) -> pd.DataFrame:
    """Per robot: left against right turns (same |v|, |ω|), and reverse against forward at the reverse speeds."""
    rows = []
    for robot, mine in _used(runs).groupby("robot", sort=False):
        for name, pick_a, pick_b in (("left − right", mine.w > 0, mine.w < 0),
                                     ("reverse − forward", mine.v < 0, mine.v > 0)):
            a, b = mine[pick_a], mine[pick_b]
            if name == "reverse − forward":
                b = b[b.v.isin(-a.v.unique())]
            a, b = a.assign(av=a.v.abs(), aw=a.w.abs()), b.assign(av=b.v.abs(), aw=b.w.abs())
            common = set(zip(a.av, a.aw)) & set(zip(b.av, b.aw))
            if not common:
                continue
            row = {"robot": robot, "comparison": name, "cells": len(common)}
            for key in ("steering_efficiency", "x_icr", "speed_efficiency"):
                label, factor, digits = QUANTITIES[key]
                diffs = [a[(a.av == v) & (a.aw == w)][key].mean() - b[(b.av == v) & (b.aw == w)][key].mean()
                         for v, w in common]
                diffs = [d for d in diffs if math.isfinite(d)]
                row[label] = _round(float(np.mean(diffs)) * factor, digits + 1) if diffs else math.nan
            rows.append(row)
    return pd.DataFrame(rows)


def wheel_table(runs: pd.DataFrame) -> pd.DataFrame:
    """Per robot: commands → wheels and wheels → motion gains, and wheel step response, pooled over cells."""
    used = _used(runs).dropna(subset=["wheel_v"])
    rows = []
    for robot, mine in used.groupby("robot", sort=False):
        driving, turning = mine[mine.v.abs() > DRIVING], mine[mine.w.abs() > TURNING]
        rows.append({
            "robot": robot,
            "wheels / command, speed": _ratio(driving.wheel_v, driving.v),
            "wheels / command, turn": _ratio(turning.wheel_w, turning.w),
            "motion / wheels, speed": _ratio(driving.forward, driving.wheel_v),
            "motion / wheels, turn": _ratio(turning.turn, turning.wheel_w),
            "xICR from wheels [cm]": round(float(np.mean(turning.sideways / turning.wheel_w)) * 100, 1)
            if len(turning) else math.nan,
            "wheel delay [ms]": _round(float(mine.wheel_delay.mean()) * 1000, 0),
            "wheel rise [ms]": _round(float(mine.wheel_rise.mean()) * 1000, 0),
            "mocap delay [ms]": _round(float(mine.delay.mean()) * 1000, 0)})
    return pd.DataFrame(rows)


def _round(value: float, digits: int) -> float | int:
    """Rounded to `digits`: a whole number when 0, never -0.0; NaN stays NaN."""
    if not math.isfinite(value):
        return math.nan
    rounded = round(float(value), digits) + 0.0
    return int(rounded) if digits == 0 else rounded


def _used(runs: pd.DataFrame) -> pd.DataFrame:
    """The runs analysed."""
    return runs[runs.used]


def _mean_spread(values: pd.Series, digits: int) -> str:
    """The values as "mean ± sd", the mean alone for one value, or "—"."""
    values = values.dropna()
    if values.empty:
        return "—"
    if len(values) == 1:
        return f"{values.iloc[0]:.{digits}f}"
    return f"{values.mean():.{digits}f} ± {values.std():.{digits}f}"


def _plane(x: pd.Series, y: pd.Series, z: pd.Series) -> tuple[float, float, float]:
    """Least squares z = a + b·x + c·y; NaN slopes where x or y does not vary."""
    columns = [np.ones(len(z))]
    varied = [np.ptp(x) > 1e-9, np.ptp(y) > 1e-9]
    columns += [c.to_numpy(float) for c, ok in zip((x, y), varied) if ok]
    solution = np.linalg.lstsq(np.column_stack(columns), z.to_numpy(float), rcond=None)[0]
    slopes = iter(solution[1:])
    return float(solution[0]), *(float(next(slopes)) if ok else math.nan for ok in varied)


def _ratio(measured: pd.Series, commanded: pd.Series) -> float:
    """Least squares gain measured = k · commanded, two decimals; NaN without samples."""
    m, c = measured.to_numpy(float), commanded.to_numpy(float)
    ok = np.isfinite(m) & np.isfinite(c)
    return round(float(np.dot(c[ok], m[ok]) / np.dot(c[ok], c[ok])), 2) if ok.sum() else math.nan


# --- --- --- --- --- PLOTS --- --- --- --- ---

def _heatmaps(runs: pd.DataFrame, out: Path) -> str:
    """Per robot a row of grids, v down and signed ω across: xICR, steering and speed efficiency, cell means."""
    used = _used(runs)
    keys = ("x_icr", "steering_efficiency", "speed_efficiency")
    robots = list(dict.fromkeys(used.robot))
    if not robots:
        return ""
    figure = Figure(figsize=(4.2 * len(keys), 3.4 * len(robots) + 0.4), layout="constrained")
    axes = np.array(figure.subplots(len(robots), len(keys), squeeze=False))
    for row, robot in enumerate(robots):
        mine = used[used.robot == robot]
        for column, key in enumerate(keys):
            label, factor, digits = QUANTITIES[key]
            table = (mine.groupby(["v", "w"])[key].mean() * factor).unstack("w").sort_index(ascending=False)
            axis = axes[row, column]
            image = axis.imshow(table.to_numpy(float), cmap="viridis", aspect="auto",
                                **_limits(table.to_numpy(float), MIN_SPAN[key], ("vmin", "vmax")))
            axis.set_xticks(range(len(table.columns)), [f"{math.degrees(w):.0f}" for w in table.columns], fontsize=7)
            axis.set_yticks(range(len(table.index)), [f"{v:g}" for v in table.index], fontsize=7)
            for (i, j), value in np.ndenumerate(table.to_numpy(float)):
                if math.isfinite(value):
                    axis.text(j, i, f"{value:.{digits}f}", ha="center", va="center", fontsize=6, color="white")
            axis.set_xlabel("ω [°/s]")
            axis.set_ylabel(f"{robot}\nv [m/s]" if column == 0 else "v [m/s]")
            axis.set_title(label, fontsize=9)
            figure.colorbar(image, ax=axis, shrink=0.8)
    figure.suptitle("Per cell (mean over its runs)")
    return save(figure, out, "commands_cells.png")


def _trends(runs: pd.DataFrame, out: Path) -> str:
    """Plot xICR and steering efficiency against |ω| and against sideways acceleration |v·ω|, coloured by |v|."""
    used = _used(runs).dropna(subset=["x_icr"])
    if used.empty:
        return ""
    figure = Figure(figsize=(10, 6.5), layout="constrained")
    axes = figure.subplots(2, 2)
    speed = used.v.abs()
    for row, key in enumerate(("x_icr", "steering_efficiency")):
        label, factor, _ = QUANTITIES[key]
        for column, (x, xlabel) in enumerate(((np.degrees(used.w.abs()), "|ω| [°/s]"),
                                              ((used.v * used.w).abs(), "|v·ω| [m/s²]"))):
            points = axes[row, column].scatter(x, used[key] * factor, c=speed, cmap="plasma", s=18,
                                               marker="o", edgecolors="none")
            reverse = used.v < 0
            axes[row, column].scatter(x[reverse], used[key][reverse] * factor, facecolors="none",
                                      edgecolors="black", s=40, label="reverse")
            axes[row, column].set_ylim(**_limits(used[key].to_numpy(float) * factor, MIN_SPAN[key],
                                                 ("bottom", "top")))
            axes[row, column].set_xlabel(xlabel)
            axes[row, column].set_ylabel(label)
            axes[row, column].grid(True, alpha=0.35)
    figure.colorbar(points, ax=axes, label="|v| [m/s]", shrink=0.6)
    figure.suptitle("Does the model depend on the command?")
    return save(figure, out, "commands_trends.png")


def _limits(values: np.ndarray, span: float, names: tuple[str, str]) -> dict:
    """Lower and upper limit around the finite values, at least `span` apart, under the given keyword names."""
    values = values[np.isfinite(values)]
    if not len(values):
        return {}
    low, high = float(values.min()), float(values.max())
    middle, half = (low + high) / 2, max(high - low, span) / 2
    return dict(zip(names, (middle - half, middle + half)))


def _steps(runs: pd.DataFrame, out: Path) -> str:
    """Step response against the step's size (turn rate, or speed on a straight): delays, and the wheels' rise.

    A panel only for the steps driven: turning ones, straight ones, or both.
    """
    used = _used(runs)
    if used.delay.isna().all():
        return ""
    turning = used.w.abs() > TURNING
    panels = [(pick, size, label) for pick, size, label in (
        (turning, np.degrees(used.w.abs()), "turn rate step [°/s]"),
        (~turning, used.v.abs() * 100, "speed step on a straight [cm/s]")) if pick.any()]
    figure = Figure(figsize=(5 * len(panels), 3.8), layout="constrained")
    for axis, (pick, size, xlabel) in zip(np.atleast_1d(figure.subplots(1, len(panels))), panels):
        for key, color, label in (("delay", "tab:red", "delay (mocap)"), ("wheel_delay", "tab:blue", "wheel delay"),
                                  ("wheel_rise", "tab:green", "wheel rise 10–90 %")):
            if used[key][pick].notna().any():
                axis.scatter(size[pick], used[key][pick] * 1000, color=color, s=18, label=label)
        axis.set_xlabel(xlabel)
        axis.set_ylabel("[ms]")
        axis.grid(True, alpha=0.35)
    figure.axes[0].legend(fontsize=7)
    figure.suptitle("Step response")
    return save(figure, out, "commands_steps.png")
