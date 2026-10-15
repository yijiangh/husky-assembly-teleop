"""
The base planner report's charts: one chart per question, tasks as rows grouped by goal, planners by colour. No ROS.

Each function draws one PNG into the report's `_analysis/` from the report's attempts (`analysis.Attempt`):

- success.png: which tasks each planner solves (a scorecard).
- plan_time.png: how long each attempt planned, failures marked.
- paths.png: length and clearance of the paths found.
- driving.png: the simulated follower's deviation and end, exact and deviated models.

! Uses matplotlib's `Figure` directly, not pyplot, so it is safe on a worker thread.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import FancyBboxPatch

from .follow import DEVIATED, EXACT
from .tasks import TASKS

#: Planner colours, fixed per planner (reference palette slots 1 and 2), and the rest in order.
PLANNER_COLORS = {"Lattice (hybrid A*)": "#2a78d6", "RRT + arcs": "#eb6834"}
OTHER_COLORS = ("#1baf7a", "#6250d6")
#: Chart ink, light surface.
SURFACE, INK, INK_2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
#: Gap between goal groups, in rows; vertical offset of the planners within a row.
GROUP_GAP = 0.8
OFFSET = 0.17
#: The planners' planning budget, seconds (planning.search.TIME_LIMIT).
BUDGET = 10.0


def colour(planner: str, planners: list[str]) -> str:
    """A planner's colour: fixed for the known ones, the next free one for others."""
    if planner in PLANNER_COLORS:
        return PLANNER_COLORS[planner]
    others = [name for name in planners if name not in PLANNER_COLORS]
    return OTHER_COLORS[others.index(planner) % len(OTHER_COLORS)]


def draw_all(attempts: list, planners: list[str], out: Path) -> list[str]:
    """Draw every chart; returns the names written (charts without data are left out)."""
    written = []
    for name, draw in (("success.png", _success), ("plan_time.png", _plan_time), ("paths.png", _paths),
                       ("driving.png", _driving)):
        if draw(attempts, planners, out / name):
            written.append(name)
    return written


# --- --- --- --- --- CHARTS --- --- --- --- ---


def _success(attempts: list, planners: list[str], out: Path) -> bool:
    """Scorecard: a cell per task and planner, filled as far as its attempts found a path, "found / attempts"."""
    rows = _rows(attempts)
    if not rows:
        return False
    figure, axis = _figure(len(rows), width=2.2 + 1.6 * len(planners))
    for j, planner in enumerate(planners):
        for task, y in rows.items():
            members = [a for a in attempts if a.task == task and a.planner == planner]
            if not members:
                axis.text(j, y, "not run", ha="center", va="center", fontsize=8, color=MUTED)
                continue
            found = sum(bool(a.plan.get("found")) for a in members)
            share = found / len(members)
            axis.add_patch(_cell(j, y, SURFACE, GRID))
            if share:
                axis.add_patch(_cell(j, y, colour(planner, planners), None, width=0.8 * share, alpha=0.9))
            axis.text(j, y, f"{found}/{len(members)}", ha="center", va="center", fontsize=9,
                      color=INK if share < 0.5 else "white", fontweight="bold")
    axis.set_xlim(-0.6, len(planners) - 0.4)
    axis.set_xticks(range(len(planners)), [_short(name) for name in planners], fontsize=10, color=INK)
    axis.xaxis.tick_top()
    axis.tick_params(axis="x", length=0)
    _task_axis(axis, rows)
    axis.grid(False)
    axis.set_title("Paths found per task: found / attempts", fontsize=11, color=INK, loc="left", pad=24)
    figure.savefig(out, dpi=130, facecolor=SURFACE)
    return True


def _plan_time(attempts: list, planners: list[str], out: Path) -> bool:
    """Every attempt's planning time as a dot (× where it found no path), per task, against the budget."""
    rows = _rows(attempts)
    if not rows:
        return False
    figure, axis = _figure(len(rows), width=8.0)
    rng = np.random.default_rng(0)
    for j, planner in enumerate(planners):
        dy = (j - (len(planners) - 1) / 2) * 2 * OFFSET
        for task, y in rows.items():
            for a in (a for a in attempts if a.task == task and a.planner == planner):
                seconds = a.plan.get("seconds")
                if seconds is None:
                    continue
                jitter = rng.uniform(-0.05, 0.05)
                if a.plan.get("found"):
                    axis.plot(seconds, y + dy + jitter, "o", ms=6, color=colour(planner, planners),
                              markeredgecolor=SURFACE, markeredgewidth=1, zorder=3)
                else:
                    axis.plot(seconds, y + dy + jitter, "x", ms=6, mew=1.6, color=colour(planner, planners),
                              zorder=3)
    axis.axvline(BUDGET, color=INK_2, linewidth=1, linestyle=(0, (4, 3)), zorder=1)
    axis.text(BUDGET, -0.9, f"budget {BUDGET:.0f} s", ha="right", va="bottom", fontsize=8, color=INK_2)
    axis.set_xlim(0, BUDGET * 1.08)
    axis.set_xlabel("planning time [s]", fontsize=9, color=INK_2)
    _task_axis(axis, rows)
    _legend(figure, planners, [("o", "found a path"), ("x", "no path")])
    axis.set_title("Planning time per attempt", fontsize=11, color=INK, loc="left")
    figure.savefig(out, dpi=130, facecolor=SURFACE)
    return True


def _paths(attempts: list, planners: list[str], out: Path) -> bool:
    """Two panels on the same task rows: path length (m) and clearance (cm), mean with sd, per planner."""
    rows = _rows(attempts)
    if not rows or not any(a.plan.get("found") for a in attempts):
        return False
    figure = Figure(figsize=(10.0, _height(len(rows))), layout="constrained", facecolor=SURFACE)
    axes = figure.subplots(1, 2, sharey=True)
    for axis, (title, metric, unit) in zip(axes, (("Path length", "length", "m"),
                                                  ("Clearance (least distance to anything)", "clearance", "cm"))):
        _style(axis)
        for task, y in rows.items():
            if not any(a.task == task and a.plan.get("found") for a in attempts):
                axis.text(0.2 if metric == "length" else 0.5, y, "no path", va="center", fontsize=8, color=MUTED)
        for j, planner in enumerate(planners):
            dy = (j - (len(planners) - 1) / 2) * 2 * OFFSET
            for task, y in rows.items():
                values = [a.value(metric) for a in attempts if a.task == task and a.planner == planner]
                values = [v for v in values if v is not None]
                if values:
                    _dot(axis, np.mean(values), np.std(values), y + dy, colour(planner, planners))
        axis.set_xlabel(f"{title.split(' (')[0].lower()} [{unit}]", fontsize=9, color=INK_2)
        axis.set_title(title, fontsize=10, color=INK, loc="left")
        axis.set_xlim(left=0)
    axes[1].set_xlim(0, 32)
    axes[1].axvspan(30, 32, color=GRID, zorder=0)
    axes[1].text(31, max(rows.values()) + 0.6, "measured\nup to 30", ha="center", va="bottom", fontsize=7,
                 color=MUTED)
    _task_axis(axes[0], rows)
    axes[1].tick_params(axis="y", length=0)
    _legend(figure, planners, [("o", "mean, line: sd")])
    figure.savefig(out, dpi=130, facecolor=SURFACE)
    return True


def _driving(attempts: list, planners: list[str], out: Path) -> bool:
    """Two panels on the same task rows: the simulated follower's deviation and end error (cm), per planner.

    Exact model hollow, deviated models filled; paths whose drives did not finish are counted beside the row.
    """
    rows = _rows([a for a in attempts if a.simulated])
    if not rows:
        return False
    figure = Figure(figsize=(10.0, _height(len(rows))), layout="constrained", facecolor=SURFACE)
    axes = figure.subplots(1, 2, sharey=True)
    for axis, (title, metric) in zip(axes, (("Largest distance from the path", "deviation"),
                                            ("End: distance from the target", "end"))):
        _style(axis)
        notes = []
        for j, planner in enumerate(planners):
            dy = (j - (len(planners) - 1) / 2) * 2 * OFFSET
            color = colour(planner, planners)
            for task, y in rows.items():
                members = [a for a in attempts if a.task == task and a.planner == planner and a.simulated]
                if not members:
                    continue
                for group, filled, shift in ((EXACT, False, -0.06), (DEVIATED, True, 0.06)):
                    values = [a.simulated_value(metric, group) for a in members]
                    values = [v for v in values if v is not None]
                    if values:
                        _dot(axis, np.mean(values), np.std(values), y + dy + shift, color, filled=filled)
                stuck = sum(not a.finished(DEVIATED) for a in members if DEVIATED in a.simulated)
                if stuck and metric == "deviation":
                    shown = [a.simulated_value(metric, group) for a in members for group in (EXACT, DEVIATED)]
                    right = max([v for v in shown if v is not None] or [0.0])
                    notes.append((right, y + dy, f"{stuck}/{len(members)} stuck"))
        axis.set_xlabel(f"{metric} [cm]", fontsize=9, color=INK_2)
        axis.set_title(title, fontsize=10, color=INK, loc="left")
        low, high = axis.get_xlim()
        axis.set_xlim(0, high * (1.3 if notes else 1.05))
        for right, y, text in notes:
            axis.text(right + 0.03 * high, y, text, va="center", fontsize=7, color=INK_2)
    _task_axis(axes[0], rows)
    axes[1].tick_params(axis="y", length=0)
    _legend(figure, planners, [("o_hollow", "exact model"), ("o", "deviated models (worst)"),
                               ("none", "n/m stuck: drives that did not finish at a deviated model")])
    figure.suptitle("Simulated follower on the paths found: exact and deviated robot models", fontsize=11, color=INK,
                    x=0.01, ha="left")
    figure.savefig(out, dpi=130, facecolor=SURFACE)
    return True


# --- --- --- --- --- PARTS --- --- --- --- ---


def _rows(attempts: list) -> dict[str, float]:
    """Task -> y position, in TASKS order from the top, a gap between goals; only tasks with attempts."""
    rows, y, goal = {}, 0.0, None
    for task in TASKS:
        if not any(a.task == task.name for a in attempts):
            continue
        if goal is not None and task.goal != goal:
            y += GROUP_GAP
        rows[task.name], goal, y = y, task.goal, y + 1.0
    return rows


def _task_axis(axis, rows: dict[str, float]) -> None:
    """Task names down the left, top first, and the goals as group labels."""
    goals = {task.name: task.goal for task in TASKS}
    axis.set_yticks(list(rows.values()), list(rows), fontsize=9, color=INK)
    axis.set_ylim(max(rows.values()) + 0.7, -1.0)
    axis.tick_params(axis="y", length=0)
    previous = None
    for task, y in rows.items():
        if goals[task] != previous:
            axis.annotate(goals[task].upper(), (0, y - 0.55), xycoords=("axes fraction", "data"), fontsize=7,
                          color=MUTED, fontweight="bold", xytext=(-4, 0), textcoords="offset points", ha="right")
            if previous is not None:
                axis.axhline(y - GROUP_GAP / 2 - 0.5, color=AXIS, linewidth=0.8, zorder=1)
            previous = goals[task]


def _figure(rows: int, width: float):
    figure = Figure(figsize=(width, _height(rows)), layout="constrained", facecolor=SURFACE)
    axis = figure.add_subplot()
    _style(axis)
    return figure, axis


def _height(rows: int) -> float:
    return 1.4 + 0.36 * rows


def _style(axis) -> None:
    """Recessive chrome: surface, hairline vertical grid, no frame but the baseline."""
    axis.set_facecolor(SURFACE)
    axis.grid(True, axis="x", color=GRID, linewidth=0.8)
    axis.set_axisbelow(True)
    for side in ("top", "right", "left"):
        axis.spines[side].set_visible(False)
    axis.spines["bottom"].set_color(AXIS)
    axis.tick_params(colors=INK_2, labelsize=8)


def _dot(axis, value: float, spread: float, y: float, color: str, filled: bool = True) -> None:
    """A mean as a dot with an sd line, filled or hollow."""
    if spread > 0:
        axis.plot([value - spread, value + spread], [y, y], color=color, linewidth=2, solid_capstyle="round",
                  alpha=0.55, zorder=2)
    axis.plot(value, y, "o", ms=7, color=color if filled else SURFACE, markeredgecolor=color, markeredgewidth=1.6,
              zorder=3)


def _cell(x: float, y: float, face: str, edge: str | None, width: float = 0.8, alpha: float = 1.0):
    """A rounded scorecard cell centred on (x, y), `width` of it filled."""
    return FancyBboxPatch((x - 0.4, y - 0.36), width, 0.72, boxstyle="round,pad=0,rounding_size=0.06",
                          facecolor=face, edgecolor=edge or "none", linewidth=0.8, alpha=alpha, zorder=2)


def _legend(figure, planners: list[str], markers: list[tuple[str, str]]) -> None:
    """One legend row: the planners by colour, then what the marker shapes mean, in ink."""
    handles = [Line2D([], [], marker="o", linestyle="", ms=7, color=colour(name, planners), label=_short(name))
               for name in planners]
    for shape, label in markers:
        if shape == "none":
            handles.append(Line2D([], [], linestyle="", label=label))
            continue
        hollow = shape == "o_hollow"
        handles.append(Line2D([], [], marker="o" if hollow else shape, linestyle="", ms=7, color=MUTED,
                              markerfacecolor=SURFACE if hollow else MUTED, markeredgewidth=1.6, label=label))
    figure.legend(handles=handles, loc="outside lower center", ncol=len(handles), frameon=False, fontsize=8,
                  labelcolor=INK_2)


def _short(name: str) -> str:
    return name.split()[0]
