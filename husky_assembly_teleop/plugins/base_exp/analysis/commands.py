"""
The constant-command report, constant_commands.md: how the robot's model changes with the command and the condition.

Each constant-command run holds one command (v, ω) open loop, a cell of the command grid (`twist.py`), under the
conditions it was recorded with (arm pose, floor, base). The report reads why, what was measured, then numbers only,
a section and one or two short tables per question: does the model change with speed and turn rate; with the
condition (the runs' conditions that differ, e.g. the arm pose); how the robot responds to a command step. The
details (per command, per run, stalls, through the wheels) go to the CSV files only.
Runs are grouped by robot and condition (`responses`). Per run:

- Steady state, from STEADY_AFTER seconds after the step to the stop: mean forward speed, sideways speed and turn rate
  of the tracked point (mocap), so speed efficiency = forward / v, steering efficiency = turn / ω, xICR = sideways / ω.
  With wheel odometry, the same through the wheels.
- Stalls: where the wheels fall below STALL of the command inside the steady part (the robot briefly stops; seen
  with commands sent over Wi-Fi), only the longest stall-free stretch counts; a run without SHORTEST_STEADY seconds of
  it is left out.
- Step response, of the turn (or of the distance on a straight):
  - delay (mocap): where the steady line of the angle turned (or distance driven), extended back, meets the start.
    That is the delay a pure-delay model needs: exact for a pure delay, plus τ for a lag, plus half the ramp for an
    acceleration limit.
  - wheel delay and rise (wheel odometry, a speed measured directly): seconds to 10 % of the steady speed, and from
    10 % to 90 %. A rise that grows with the step means an acceleration limit, a fixed one a lag.
"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import matplotlib
import numpy as np
import pandas as pd
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from ..replay import track
from ..report import load
from ..twist import SEGMENT, grid
from .common import Experiment, markdown, save

#: Seconds after the step before the steady window starts: delay (~0.25 s) and the ramp, with a margin.
STEADY_AFTER = 1.2
#: A steady window shorter than this leaves the run out, seconds.
SHORTEST_STEADY = 1.5
#: A stall: the wheels below this share of the command for two odometry messages or more; the steady stretch keeps
#: STALL_MARGIN seconds away from it (the motion lags the wheels' readings).
STALL, STALL_MARGIN = 0.6, 0.3
#: Commands that excite a channel: speed (m/s) and turn rate (rad/s) above these.
DRIVING, TURNING = 0.02, 0.05
#: The speed the fit over all runs is also given at, m/s: the path experiments' (`auto.SPEED`).
REFERENCE_SPEED = 0.2
#: The prompt for the written results of a constant-command experiment, written beside the report; a subagent gets it.
RESULTS_PROMPT = """\
You write the results section of one constant-command experiment of the base-controller study, from its analysis
folder:
{folder}

Read constant_commands.md and the CSV files it is made of (commands_overview.csv, commands_fit.csv,
commands_by_speed.csv, commands_by_turn_rate.csv, commands_conditions.csv, commands_wheels.csv, commands_steps.csv,
commands_cells.csv, commands_stalls.csv, commands_runs.csv). Look at commands_axes.png, commands_cells.png and
commands_steps.png. Ignore other files.

The report starts with why and what was measured: use its definitions and terms, and keep its ! notes in mind.
Everything after that is numbers only; reading them is your job. Write commands_results.md in the same folder with
exactly these sections, in this order:

# Results: <experiment name>
One line: date range, number of runs, the conditions compared with their runs and runs per command, what was held
fixed.

## 1. Speed and turn rate
Per condition. With speed: for each value, the change per m/s and how many standard errors it is; say "changes" only
above three, and say how large the change is over the driven speeds. With turn rate: the values at each turn rate
against the spread between runs. Left against right and forward against reverse where they differ.

## 2. Arm pose
The model per condition (steering efficiency, turning centre, xICR, speed efficiency) and each condition's
difference to the first one, in the value's unit and against their standard errors: on the commands every condition
drove, then on all commands. Say which values the condition changes and which it leaves alone, and from
commands_wheels.csv whether the change arises between command and wheels or between wheels and motion.

## 3. Response to a command step
The delay, and the wheels' rise from 10 % to 90 %: whether the rise grows with the step (acceleration limit) or
stays the same (lag), for turns and for straights; whether it differs between conditions.

## Caveats
Conditions with fewer than 3 runs per command, commands not driven, runs with a stall or not analysed, single runs
far from their repeats, anything else that limits the answers.

## Next steps
2 to 4 concrete experiments or changes the data points to.

Rules: state only what the data shows, with the numbers that show it; a difference within the spread between repeats
or below three standard errors is no difference; efficiencies with two decimals, everything else rounded to one;
plain language, short sentences; no new plots; do not change any other file.
"""
#: The column that names a group of runs in the tables: the robot and the condition (see `responses`).
GROUP = "condition"
#: Line colours of the groups in commands_axes.png, in table order.
COLOURS = ("tab:blue", "tab:orange", "tab:red", "tab:green", "tab:purple", "tab:brown")
#: Heatmap colour of a cell no run was driven in.
NOT_DRIVEN = "0.8"
#: The values per run, with label, factor to the unit, and how many decimals.
QUANTITIES = {"speed_efficiency": ("speed efficiency", 1.0, 2), "steering_efficiency": ("steering efficiency", 1.0, 2),
              "x_icr": ("xICR [cm]", 100.0, 1), "delay": ("delay [ms]", 1000.0, 0),
              "wheel_delay": ("wheel delay [ms]", 1000.0, 0), "wheel_rise": ("wheel rise [ms]", 1000.0, 0)}
#: The quantities of the step response (QUANTITIES keys).
STEP = ("delay", "wheel_delay", "wheel_rise")
#: The model's values fitted per group: label, factor to the unit, decimals (`_model`).
FITTED = {"steering efficiency": (1.0, 2), "turning centre [cm]": (100.0, 1), "xICR [cm]": (100.0, 1),
          "speed efficiency": (1.0, 2)}
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
        steady: Seconds of the steady stretch used.
        stalled: Seconds the wheels stalled inside the steady part (0 without wheel odometry).
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
    stalled: float


def write(experiment: Experiment) -> int:
    """Write constant_commands.md, its CSV files, plots and results prompt from the experiment's constant-command runs.

    Returns:
        int: Runs analysed.
    """
    out = experiment.out
    fixed, varying = _conditions(experiment.runs)
    runs = responses(experiment.runs)
    runs.to_csv(out / "commands_runs.csv", index=False)
    tables = {"commands_overview": overview_table(runs), "commands_fit": fit_table(runs),
              "commands_by_speed": level_table(runs, "v"), "commands_by_turn_rate": level_table(runs, "w"),
              "commands_conditions": condition_table(runs), "commands_wheels": wheel_table(runs),
              "commands_steps": step_table(runs), "commands_cells": cell_table(runs),
              "commands_stalls": stall_table(runs)}
    for name, table in tables.items():
        table.to_csv(out / f"{name}.csv", index=False)
    plots = [p for p in (_axes(runs, out), _heatmaps(runs, out), _steps(runs, out)) if p]
    left = runs[~runs.used]
    groups = list(dict.fromkeys(runs.robot))
    shared = _shared_cells(_used(runs))
    compared = ("the conditions (" + ", ".join(varying) + "): " + "; ".join(groups)) if varying else "one condition"

    # * The report's tables read across the command: a row per value, a column per speed, turn rate or step.
    by_speed = _across(tables["commands_by_speed"], "v [m/s]", groups, "{} m/s",
                       ("turning centre [cm]", "steering efficiency", "speed efficiency"))
    if "change per m/s" in tables["commands_fit"]:
        slopes = tables["commands_fit"].set_index(["robot", "value"])["change per m/s"]
        by_speed["change per m/s"] = [slopes.get(key, math.nan) for key in zip(by_speed.robot, by_speed.value)]
    by_turn_rate = _across(tables["commands_by_turn_rate"], "ω [°/s]", groups, "{}°/s",
                           ("turning centre [cm]", "steering efficiency"))
    steps = _across(tables["commands_steps"], "step", groups, "{}",
                    tuple(QUANTITIES[key][0] for key in ("delay", "wheel_rise")))

    def show(table: pd.DataFrame, group: str = GROUP) -> str:
        return markdown(table.rename(columns={"robot": GROUP}), group)

    def plot(name: str, goal: str) -> list[str]:
        return [f"{name}: {goal}\n", f"![{name}]({name})\n"] if name in plots else []

    lines = [f"# Constant commands: {experiment.name}\n",
             experiment.intro()
             + (f" Not analysed: {len(left)} ({', '.join(sorted(set(left.why)))})." if len(left) else "") + "\n",
             "## Why\n",
             "A controller steers better when it knows how the robot responds to its commands. **Hypothesis 1:** "
             "that response, the robot's motion model, changes with speed and turn rate, and with the arm pose "
             "(anything that moves the robot's weight, a payload included). This report answers:\n",
             "1. Do speed and turn rate change the model? (section 1)",
             "2. Does the arm pose change it? (section 2)",
             "3. How long does the robot take to respond to a command? (section 3)\n",
             "## What we measured\n",
             "- **Held fixed**: " + ", ".join(f"{key} {value}" for key, value in fixed.items()) + f". **Compared**: "
             f"{compared}. The first, the one with the most runs, is the reference.",
             f"- **A run** holds one command for {SEGMENT:g} s open loop: a speed v (negative: reverse) and a turn "
             f"rate ω (positive: left). Its values are steady ones, from {STEADY_AFTER:g} s after the command starts "
             "until it stops.",
             "- **Steering efficiency**: turn rate per commanded turn rate.",
             "- **Turning centre**: how far the tracked point lies ahead of the point the robot turns about "
             "(negative: behind).",
             "- **xICR**: turning centre × steering efficiency, the form the model takes. ! It changes when either "
             "of the two does.",
             "- **Speed efficiency**: forward speed per commanded speed, on straights. ! On a turn the forward speed "
             "also depends on the turn rate; commands_cells.png shows it there too.",
             "- **Delay** (mocap): from the command to the motion, as a pure delay. ! It includes the Wi-Fi from the "
             "monitor to the robot. **Wheel rise** (wheel odometry): the time the wheels take from 10 % to 90 % of "
             "their steady speed. A rise that grows with the step means an acceleration limit, a flat one a lag.",
             f"- **A stall**: the wheels fall below {STALL:.0%} of the command: the robot briefly stopped (seen with "
             "commands sent over Wi-Fi). Only the longest steady stretch without it is used, and the run gives no "
             f"delay; with less than {SHORTEST_STEADY:g} s of it the run is not analysed.",
             "- **±** after a mean: the standard deviation between its runs; a single run shows its value alone. "
             "After a fitted value: its standard error; a change or difference of more than about three is real.\n",
             f"What was run. Commands: those with a run analysed, out of the grid's {len(grid())}.\n",
             show(tables["commands_overview"]), "",
             "## 1. Speed and turn rate\n",
             "Does the model change with the command? Each condition on its own.\n",
             *plot("commands_axes.png", "turning centre and steering efficiency against speed (left; solid: left "
                   "turns, dashed: right turns) and against turn rate (right), a colour per condition. Lines: mean ± "
                   "standard deviation of the runs; dots: single runs. A sloped line is a change with the command."),
             "**By speed**: the mean of the runs at each speed (the turning ones; speed efficiency: the straights). "
             "Change per m/s: fitted over all runs, forward and reverse alike, ± standard error.\n",
             show(by_speed), "",
             "**By turn rate**: the mean of the runs at each turn rate.\n",
             show(by_turn_rate), "",
             *plot("commands_cells.png", "every command on its own, speed down and turn rate across. Grey: not "
                   "driven; –: the value does not exist there."),
             "## 2. Arm pose\n",
             "Does the arm pose change the model? Each condition's model, fitted over its runs, ± standard error. "]
    if len(groups) < 2:
        lines[-1] += "One condition: nothing to compare.\n"
    elif shared:
        lines[-1] += (f"First on the {len(shared)} commands every condition drove, so that like is compared with "
                      "like; then on all commands of each condition.\n")
    else:
        lines[-1] += "No command was driven under every condition: each on its own commands.\n"
    lines += [show(tables["commands_conditions"], "commands"), "",
              "## 3. Response to a command step\n",
              "How long does the robot take to respond, and does that change with the size of the step or the arm "
              "pose? A turn counts by its turn rate, a straight by its speed; the mean of the runs.\n",
              show(steps), "",
              *plot("commands_steps.png", "the same, every run a dot, all conditions together, with the wheel delay."),
              "## More numbers\n",
              "In the CSV files only: runs per value, and left and right turns apart (commands_by_speed.csv, "
              "commands_by_turn_rate.csv, commands_fit.csv); every command (commands_cells.csv) and run "
              "(commands_runs.csv); the split at the wheels (commands_wheels.csv); the runs with a stall "
              "(commands_stalls.csv).", ""]
    (out / "constant_commands.md").write_text("\n".join(lines))
    (out / "commands_results_prompt.md").write_text(RESULTS_PROMPT.format(folder=out.resolve()))
    return int(runs.used.sum())


def _across(table: pd.DataFrame, column: str, groups: list[str], header: str, values: tuple) -> pd.DataFrame:
    """A table of levels turned for the report: a row per group and value, a column per level; "—" where none.

    A value a group has at no level gets no row.

    Args:
        table: Rows per group ("robot") and level, e.g. `level_table`.
        column: The column of the levels, e.g. "v [m/s]"; numbers come sorted, other levels as they appear.
        groups: The groups, in order.
        header: The header of a level's column, e.g. "{} m/s".
        values: The columns of `table` that become rows.
    """
    if table.empty:
        return pd.DataFrame(columns=["robot", "value"])
    levels = list(dict.fromkeys(table[column]))
    try:
        levels.sort(key=float)
    except ValueError:
        pass
    rows = []
    for robot in groups:
        mine = table[table.robot == robot].set_index(column)
        rows += [{"robot": robot, "value": value,
                  **{header.format(level): mine[value].get(level, "—") for level in levels}}
                 for value in values if len(mine) and (mine[value] != "—").any()]
    return pd.DataFrame(rows)


# --- --- --- --- --- RUNS --- --- --- --- ---

def responses(runs: pd.DataFrame) -> pd.DataFrame:
    """The constant-command runs (`common.load_runs` rows), one row each: group, cell, and their `Steady` values.

    The group (column `robot`) is the robot and the run's condition, where the runs differ in them: e.g. "stowed" and
    "forward" for one robot in two arm poses. Groups come by their number of runs, the largest (the reference) first.
    """
    _, varying = _conditions(runs)
    several = runs.robot.nunique() > 1
    rows = []
    for _, run in runs.iterrows():
        conditions = json.loads(run.conditions)
        label = ", ".join(str(conditions.get(key, "?")) for key in varying)
        group = " · ".join(part for part in (run.robot if several or not label else "", label) if part)
        row = {"run": run.run, "folder": run.folder, "started": run.started, "robot": group,
               "v": round(float(run.p_v), 3), "w": round(float(run.p_w), 3), "battery": run.battery,
               "outcome": run.outcome}
        found = steady(load(Path(run.folder) / "recording.npz")) if run.success else None
        row["used"] = found is not None
        row["why"] = "" if found is not None else ("not done" if not run.success else
                                                   "stalled, too short or no mocap")
        row.update(asdict(found) if found is not None else {})
        rows.append(row)
    found = pd.DataFrame(rows)
    order = {group: i for i, group in enumerate(found.robot.value_counts().index)}
    return found.assign(order=found.robot.map(order)).sort_values(["order", "v", "w"], kind="stable").drop(
        columns="order")


def _conditions(runs: pd.DataFrame) -> tuple[dict, list[str]]:
    """The conditions every run shares, and the keys the runs differ in (e.g. ["arm"])."""
    each = [json.loads(text) for text in runs.conditions.unique()]
    keys = list(dict.fromkeys(key for conditions in each for key in conditions))
    varying = [key for key in keys if len({json.dumps(c.get(key)) for c in each}) > 1]
    return {key: each[0][key] for key in keys if key not in varying}, varying


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
    begin, end, stalled = _stall_free(tr, step_at + STEADY_AFTER, stop_at, v, w)
    window = (t >= begin) & (t <= end)
    if end - begin < SHORTEST_STEADY or window.sum() < 5:
        return None
    mean_forward, mean_sideways = float(forward[window].mean()), float(sideways[window].mean())
    mean_turn = float(np.polyfit(t[window], yaw[window], 1)[0])

    wheel_v = wheel_w = math.nan
    wheel_delay = wheel_rise = math.nan
    turning = abs(w) > TURNING
    if len(tr.odometry_time):
        steady = (tr.odometry_time >= begin) & (tr.odometry_time <= end)
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
    # * After a stall the line no longer starts at the step.
    delay = float(-offset / slope - step_at) if abs(slope) > 1e-9 and stalled == 0.0 else math.nan
    return Steady(
        forward=mean_forward, sideways=mean_sideways, turn=mean_turn,
        speed_efficiency=mean_forward / v if abs(v) > DRIVING else math.nan,
        steering_efficiency=mean_turn / w if turning else math.nan,
        x_icr=mean_sideways / w if turning else math.nan,
        wheel_v=wheel_v, wheel_w=wheel_w, delay=delay, wheel_delay=wheel_delay, wheel_rise=wheel_rise,
        steady=float(end - begin), stalled=stalled)


def _stall_free(tr, start: float, end: float, v: float, w: float) -> tuple[float, float, float]:
    """The longest stretch of [start, end] without a stall, and the seconds stalled; all of it without odometry."""
    inside = (tr.odometry_time >= start) & (tr.odometry_time <= end)
    if inside.sum() < 3:
        return start, end, 0.0
    turning = abs(w) > TURNING
    times = tr.odometry_time[inside]
    slow = tr.odometry[inside, 1 if turning else 0] / (w if turning else v) < STALL
    # * One low reading is noise at slow commands: a stall is two in a row or more.
    slow &= np.r_[slow[1:], False] | np.r_[False, slow[:-1]]
    if not slow.any():
        return start, end, 0.0
    edges = np.flatnonzero(np.diff(np.r_[False, slow, False].astype(int)))
    stalls = [(times[a], times[b - 1]) for a, b in zip(edges[::2], edges[1::2])]
    starts = [start] + [last + STALL_MARGIN for _, last in stalls]
    ends = [first - STALL_MARGIN for first, _ in stalls] + [end]
    longest = max(zip(starts, ends), key=lambda stretch: stretch[1] - stretch[0])
    return float(longest[0]), float(longest[1]), float(sum(last - first for first, last in stalls) + 0.0)


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
        row = {"robot": robot, "v [m/s]": f"{v:g}", "ω [°/s]": round(math.degrees(w)), "n": len(group)}
        for key, (label, factor, digits) in QUANTITIES.items():
            row[label] = _mean_spread(group[key] * factor, digits)
        rows.append(row)
    return pd.DataFrame(rows)


def fit_table(runs: pd.DataFrame) -> pd.DataFrame:
    """Per robot: each model value at standstill and its change per m/s of |v|, fitted over all runs, ± standard error.

    "change [se]" counts the standard errors of the change.

    Least squares on the steady velocities: sideways and turn rate on ω and ω·|v| (xICR, steering efficiency), sideways
    on the measured turn rate likewise (turning centre), forward speed of the straights on v and v·|v|.
    """
    rows = []
    for robot, mine in _used(runs).groupby("robot", sort=False):
        turning, straight = mine[mine.w.abs() > TURNING], mine[(mine.w.abs() <= TURNING) & (mine.v.abs() > DRIVING)]
        left, right = turning[turning.w > 0], turning[turning.w < 0]
        fits = (("turning centre [cm]", turning, turning.sideways, turning.turn, 100.0, 1),
                ("turning centre, left turns [cm]", left, left.sideways, left.turn, 100.0, 1),
                ("turning centre, right turns [cm]", right, right.sideways, right.turn, 100.0, 1),
                ("xICR [cm]", turning, turning.sideways, turning.w, 100.0, 1),
                ("steering efficiency", turning, turning.turn, turning.w, 1.0, 2),
                ("speed efficiency", straight, straight.forward, straight.v, 1.0, 2))
        for label, used, output, by, factor, digits in fits:
            speeds = used.v.abs()
            if len(used) < 3:
                continue
            varied = speeds.nunique() > 1
            columns = [by.to_numpy(float)] + ([(by * speeds).to_numpy(float)] if varied else [])
            values, errors = _least_squares(output.to_numpy(float), columns)
            row = {"robot": robot, "value": label, "runs": len(used),
                   "at standstill": f"{values[0] * factor:.{digits}f} ± {errors[0] * factor:.{digits}f}"}
            if varied:
                row["change per m/s"] = (f"{_round(values[1] * factor, digits):+.{digits}f} ± "
                                         f"{errors[1] * factor:.{digits}f}")
                row["change [se]"] = _round(abs(values[1]) / errors[1], 1) if errors[1] > 0 else math.nan
                row[f"at {REFERENCE_SPEED:g} m/s"] = f"{(values[0] + values[1] * REFERENCE_SPEED) * factor:.{digits}f}"
            rows.append(row)
    return pd.DataFrame(rows)


def level_table(runs: pd.DataFrame, axis: str) -> pd.DataFrame:
    """Per group and signed level of v ("v") or ω ("w"): turning centre and steering efficiency of the turning runs.

    By speed, left and right turns also apart, and the straights' speed efficiency; by turn rate the sign is the
    direction (positive: left).
    """
    rows = []
    used = _used(runs)
    used = used.assign(turning=used.w.abs() > TURNING)
    label = "v [m/s]" if axis == "v" else "ω [°/s]"
    for (robot, level), mine in (used if axis == "v" else used[used.turning]).groupby(["robot", axis], sort=False):
        turning, straight = mine[mine.turning], mine[~mine.turning & (mine.v.abs() > DRIVING)]
        centre = turning.sideways / turning.turn * 100
        row = {"robot": robot, label: level if axis == "v" else round(math.degrees(level)), "n": len(turning),
               "turning centre [cm]": _mean_spread(centre, 1)}
        if axis == "v":
            row.update({"left turns [cm]": _mean_spread(centre[turning.w > 0], 1),
                        "right turns [cm]": _mean_spread(centre[turning.w < 0], 1)})
        row["steering efficiency"] = _mean_spread(turning.turn / turning.w, 2)
        if axis == "v":
            row.update({"n straights": len(straight),
                        "speed efficiency": _mean_spread(straight.forward / straight.v, 2)})
        rows.append(row)
    table = pd.DataFrame(rows)
    if table.empty:
        return table
    order = {group: i for i, group in enumerate(dict.fromkeys(runs.robot))}
    table = table.assign(order=table.robot.map(order)).sort_values(["order", label], kind="stable").drop(
        columns="order")
    return table.assign(**{label: table[label].map("{:g}".format)})


def overview_table(runs: pd.DataFrame) -> pd.DataFrame:
    """Per group: runs, runs analysed and with a stall, commands with a run analysed (of the grid's), runs of each."""
    rows = []
    for robot, mine in runs.groupby("robot", sort=False):
        per_command = mine[mine.used].groupby(["v", "w"]).size()
        repeats = "—" if per_command.empty else (f"{per_command.min()}" if per_command.min() == per_command.max()
                                                 else f"{per_command.min()} to {per_command.max()}")
        rows.append({"robot": robot, "runs": len(mine), "analysed": int(mine.used.sum()),
                     "of them with a stall": int((mine.used & (mine.stalled > 0)).sum()),
                     "commands": f"{len(per_command)} of {len(grid())}", "runs per command": repeats})
    return pd.DataFrame(rows)


def condition_table(runs: pd.DataFrame) -> pd.DataFrame:
    """Per set of commands and group: the model fitted over the group's runs (no speed term) ± standard error.

    The sets: the commands every group drove (only with several groups), then all commands of each group.
    """
    used = _used(runs)
    groups = list(dict.fromkeys(used.robot))
    cells = set(_shared_cells(used))
    sets = [("all of each condition", used)]
    if len(groups) > 1 and cells:
        sets.insert(0, (f"the {len(cells)} every condition drove", used[[c in cells for c in zip(used.v, used.w)]]))
    rows = []
    for name, chosen in sets:
        for robot in groups:
            mine = chosen[chosen.robot == robot]
            found = _model(mine)
            rows.append({"commands": name, "robot": robot, "runs": len(mine)} | {
                label: f"{found[label][0] * factor:.{digits}f} ± {found[label][1] * factor:.{digits}f}"
                if label in found else "—" for label, (factor, digits) in FITTED.items()})
    return pd.DataFrame(rows)


def _model(mine: pd.DataFrame) -> dict[str, tuple[float, float]]:
    """One group's model without a speed term: per FITTED label the least-squares value and its standard error.

    Steering efficiency, turning centre and xICR from the runs that turn, speed efficiency from the straights; a value
    with fewer than two such runs is left out.
    """
    turning, straight = mine[mine.w.abs() > TURNING], mine[(mine.w.abs() <= TURNING) & (mine.v.abs() > DRIVING)]
    fits = {"steering efficiency": (turning.turn, turning.w), "turning centre [cm]": (turning.sideways, turning.turn),
            "xICR [cm]": (turning.sideways, turning.w), "speed efficiency": (straight.forward, straight.v)}
    found = {}
    for label, (output, by) in fits.items():
        if len(output) >= 2:
            (value,), (error,) = _least_squares(output.to_numpy(float), [by.to_numpy(float)])
            found[label] = (float(value), float(error))
    return found


def step_table(runs: pd.DataFrame) -> pd.DataFrame:
    """Per step and group: the step response's times, mean ± spread over the runs.

    A turn counts by its turn rate, whatever its speed and direction; a straight by its speed, forward or reverse.
    """
    used = _used(runs)
    order = {group: i for i, group in enumerate(dict.fromkeys(runs.robot))}
    on_straight = used.w.abs() <= TURNING
    used = used.assign(order=used.robot.map(order), on_straight=on_straight,
                       step_size=np.where(on_straight, used.v.abs(), used.w.abs()))
    rows = []
    for (straight, size, _), mine in used.groupby(["on_straight", "step_size", "order"]):
        row = {"robot": mine.robot.iloc[0], "n": len(mine),
               "step": f"straight {size:g} m/s" if straight else f"turn {math.degrees(size):.0f}°/s"}
        for key in STEP:
            label, factor, digits = QUANTITIES[key]
            row[label] = _mean_spread(mine[key] * factor, digits)
        rows.append(row)
    return pd.DataFrame(rows, columns=["robot", "step", "n", *(QUANTITIES[key][0] for key in STEP)])


def stall_table(runs: pd.DataFrame) -> pd.DataFrame:
    """The runs with a stall: group, command, seconds stalled, and the steady seconds used (or that it was left out)."""
    rows = []
    for _, run in runs.iterrows():
        left_out = not run.used and run.why.startswith("stalled")
        if left_out or (run.used and run.stalled > 0):
            rows.append({"robot": run.robot, "v [m/s]": f"{run.v:g}", "ω [°/s]": round(math.degrees(run.w)),
                         "run": run.run, "stalled [s]": "—" if left_out else f"{run.stalled:.1f}",
                         "used [s]": "not analysed" if left_out else f"{run.steady:.1f}"})
    return pd.DataFrame(rows)


def _shared_cells(used: pd.DataFrame) -> list[tuple[float, float]]:
    """The commands (v, ω) every group drove, in grid order."""
    per_group = [set(zip(mine.v, mine.w)) for _, mine in used.groupby("robot")]
    return sorted(set.intersection(*per_group)) if per_group else []


def _least_squares(output: np.ndarray, columns: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Least-squares gains of `output` on `columns` (through the origin), and their standard errors."""
    matrix = np.column_stack(columns)
    values = np.linalg.lstsq(matrix, output, rcond=None)[0]
    residual = output - matrix @ values
    variance = residual @ residual / max(len(output) - len(values), 1)
    return values, np.sqrt(np.diag(variance * np.linalg.pinv(matrix.T @ matrix)))


def wheel_table(runs: pd.DataFrame) -> pd.DataFrame:
    """Per robot: the steady gains commands → wheels and wheels → motion, and xICR per wheel turn rate."""
    used = _used(runs).dropna(subset=["wheel_v"])
    rows = []
    for robot, mine in used.groupby("robot", sort=False):
        driving, turning = mine[mine.v.abs() > DRIVING], mine[mine.w.abs() > TURNING]
        rows.append({
            "robot": robot,
            "wheels per command, speed": _ratio(driving.wheel_v, driving.v),
            "wheels per command, turn": _ratio(turning.wheel_w, turning.w),
            "motion per wheels, speed": _ratio(driving.forward, driving.wheel_v),
            "motion per wheels, turn": _ratio(turning.turn, turning.wheel_w),
            "xICR per wheel turn rate [cm]": _ratio(turning.sideways * 100, turning.wheel_w, 1)})
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


def _ratio(measured: pd.Series, commanded: pd.Series, digits: int = 2) -> str:
    """Least squares gain measured = k · commanded, as text with `digits` decimals; "—" without samples."""
    m, c = measured.to_numpy(float), commanded.to_numpy(float)
    ok = np.isfinite(m) & np.isfinite(c)
    return f"{float(np.dot(c[ok], m[ok]) / np.dot(c[ok], c[ok])):.{digits}f}" if ok.sum() else "—"


# --- --- --- --- --- PLOTS --- --- --- --- ---

def _heatmaps(runs: pd.DataFrame, out: Path) -> str:
    """Per robot a row of grids, v down and signed ω across: xICR, steering and speed efficiency, cell means.

    Cells without a run analysed are grey; cells whose runs cannot give the value (xICR on a straight) are white with
    a dash.
    """
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
            cells = mine.groupby(["v", "w"])[key]
            table = (cells.mean() * factor).unstack("w").sort_index(ascending=False)
            driven = cells.size().unstack("w").reindex_like(table).notna().to_numpy()
            axis = axes[row, column]
            colours = matplotlib.colormaps["viridis"].with_extremes(bad=NOT_DRIVEN)
            image = axis.imshow(np.ma.masked_invalid(table.to_numpy(float)), cmap=colours, aspect="auto",
                                **_limits(table.to_numpy(float), MIN_SPAN[key], ("vmin", "vmax")))
            axis.set_xticks(range(len(table.columns)), [f"{math.degrees(w):.0f}" for w in table.columns], fontsize=7)
            axis.set_yticks(range(len(table.index)), [f"{v:g}" for v in table.index], fontsize=7)
            for (i, j), value in np.ndenumerate(table.to_numpy(float)):
                if math.isfinite(value):
                    axis.text(j, i, f"{value:.{digits}f}", ha="center", va="center", fontsize=6, color="white")
                elif driven[i, j]:
                    axis.add_patch(Rectangle((j - 0.5, i - 0.5), 1.0, 1.0, facecolor="white", edgecolor="none"))
                    axis.text(j, i, "–", ha="center", va="center", fontsize=7, color="0.4")
            axis.set_xlabel("ω [°/s]")
            axis.set_ylabel(f"{robot}\nv [m/s]" if column == 0 else "v [m/s]")
            axis.set_title(label, fontsize=9)
            figure.colorbar(image, ax=axis, shrink=0.8)
    figure.suptitle("Per command (mean over its runs)\ngrey: not driven · –: driven, but the value does not exist "
                    "there (no turn on a straight, no speed on a spot turn)", fontsize=10)
    return save(figure, out, "commands_cells.png")


def _axes(runs: pd.DataFrame, out: Path) -> str:
    """Turning centre and steering efficiency against signed v and signed ω: lines per group, mean ± sd per level.

    Against speed, left and right turns are separate lines (negative speed: reverse); against turn rate the sign is
    the direction, and the line breaks at zero, where no turning run lies.
    """
    used = _used(runs)
    used = used[used.w.abs() > TURNING]
    if used.empty:
        return ""
    used = used.assign(centre=used.sideways / used.turn * 100, steering=used.turn / used.w,
                       rate=np.degrees(used.w).round())
    groups = list(dict.fromkeys(runs.robot))
    colours = dict(zip(groups, COLOURS))
    left, right = used.w > 0, used.w < 0
    # * Per panel: the x column, its label, the shift between groups, and the lines (rows, line style, marker).
    panels = (("v", "speed v [m/s] (negative: reverse)", 0.004, ((left, "-", "o"), (right, "--", "s"))),
              ("rate", "turn rate ω [°/s] (positive: left, negative: right)", 0.5,
               ((right, "-", "o"), (left, "-", "o"))))
    figure = Figure(figsize=(11.5, 7.5), layout="constrained")
    axes = figure.subplots(2, 2, sharey="row")
    for row, (key, label) in enumerate((("centre", "turning centre [cm]"), ("steering", "steering efficiency"))):
        for column, (axis_key, xlabel, shift, lines) in enumerate(panels):
            axis = axes[row, column]
            for i, group in enumerate(groups):
                offset = (i - (len(groups) - 1) / 2) * shift
                for pick, style, marker in lines:
                    mine = used[(used.robot == group) & pick]
                    if mine.empty:
                        continue
                    axis.scatter(mine[axis_key] + offset, mine[key], s=9, color=colours[group], alpha=0.25,
                                 edgecolors="none")
                    levels = mine.groupby(axis_key)[key].agg(["mean", "std"]).fillna(0.0)
                    axis.errorbar(levels.index + offset, levels["mean"], yerr=levels["std"], color=colours[group],
                                  linestyle=style, marker=marker, markersize=4, linewidth=1.5, capsize=3)
            axis.axvline(0.0, color="0.6", linewidth=0.8)
            axis.set_xlabel(xlabel)
            axis.grid(True, alpha=0.35)
        axes[row, 0].set_ylabel(label)
    handles = [Line2D([], [], color=colours[group], marker="o", markersize=4, label=group) for group in groups]
    handles += [Line2D([], [], color="0.3", linestyle="-", marker="o", markersize=4, label="against speed: left turns"),
                Line2D([], [], color="0.3", linestyle="--", marker="s", markersize=4, label="right turns")]
    figure.legend(handles=handles, loc="outside lower center", ncols=min(len(handles), 5), fontsize=9)
    figure.suptitle("The model against speed and turn rate, per condition\n"
                    "line: mean ± standard deviation of the runs at that level · dots: single runs", fontsize=10)
    return save(figure, out, "commands_axes.png")


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
