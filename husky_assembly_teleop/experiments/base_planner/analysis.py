"""
The base planner experiment's report (doc/base_planner_manual.md): the planners side by side on the standard tasks.

Reads every attempt folder of one experiment (`recording.py`) and writes into its `_analysis/`: summary.md, one CSV per
table, the charts (`figures.py`) and tasks.png, and results_prompt.md; per attempt plan.png, and per run overview.png
and report.json. No ROS. Follows doc/experiment_report_guide.md: numbers only, no interpretation.
"""

from __future__ import annotations

import csv
import math
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from matplotlib.figure import Figure

from ...plugins.base_planner.follow import DEVIATED, EXACT
from ...plugins.base_planner.path import densify
from ...plugins.base_planner.recording import load_plan_arrays
from ...plugins.base_planner.tasks import BY_NAME, TASK_ORIGIN, TASKS
from ...shared.recordings.runs import load_description, recording_path, write_report
from .figures import colour, draw_all, draw_lab
from .report import (SimulatedMetrics, RunMetrics, draw_floor, load, metrics, plot, plot_plan, planner,
                     settings_line, simulated_metrics)

#: Where attempts were made, in report order.
ENVIRONMENTS = ("offline", "sim", "real")
#: Run outcomes the operator caused: the run is left out (its plan still counts).
OPERATOR = ("stopped", "replaced by a new commit")
#: Better-than rules per metric: which way is better, and the least gap that counts (beside the spread).
BETTER = {"success": (+1, 0.0), "seconds": (-1, 0.2), "length": (-1, 0.10), "clearance": (+1, 1.0),
          "deviation": (-1, 1.0), "end": (-1, 1.0), "drift": (-1, 1.0), "ratio": (-1, 0.05)}
#: A plan from within this far of its target, metres and radians, is left out of the pooled numbers: already there.
THERE_POSITION = 0.05
THERE_YAW = math.radians(5.0)
#: Decimals per metric, the same in every table.
DECIMALS = {"seconds": 1, "length": 2, "clearance": 1, "deviation": 1, "end": 1, "drift": 1, "ratio": 2}


@dataclass
class Attempt:
    """One recorded attempt and what was measured of it.

    Attributes:
        number: Its number in the report, in time order.
        folder: Its folder.
        description: Its experiment.json.
        run: The live run's metrics; None if not committed.
        simulated: The offline follower simulations' metrics per model group (`follow.EXACT`, `follow.DEVIATED`),
            one per drive; empty if none.
        drives: The simulated drives as recorded: group, true xICR, why not finished.
        left_out: Why it is left out of the comparisons; "" if it counts.
        run_left_out: Why its run is left out; "" if it counts or there is none.
    """

    number: int
    folder: Path
    description: dict
    run: RunMetrics | None = None
    simulated: dict[str, list[SimulatedMetrics]] = field(default_factory=dict)
    drives: list[dict] = field(default_factory=list)
    left_out: str = ""
    run_left_out: str = ""

    @property
    def task(self) -> str | None:
        """The task it counts for; None for a free attempt."""
        return self.description.get("task")

    @property
    def planner(self) -> str:
        """The planner's label."""
        return planner(self.description)

    @property
    def environment(self) -> str:
        """Where it was made: offline, sim or real."""
        return self.description.get("environment", "?")

    @property
    def plan(self) -> dict:
        """The plan's record; {} if none."""
        return self.description.get("plan") or {}

    def value(self, metric: str) -> float | None:
        """The attempt's number for a plan metric (success; seconds, length, clearance in cm of a path found)."""
        plan = self.plan
        if metric == "success":
            return 1.0 if plan.get("found") else 0.0
        if metric == "clearance":
            return None if plan.get("clearance") is None else plan["clearance"] * 100
        return plan.get(metric) if plan.get("found") else None

    def finished(self, group: str) -> float | None:
        """1 if every simulated drive of a model group finished, else 0; None if the group was not simulated."""
        drives = self.simulated.get(group)
        return None if not drives else float(all(m.finished for m in drives))

    def simulated_value(self, metric: str, group: str) -> float | None:
        """The worst over a model group's simulated drives: deviation or end in cm.

        None if not simulated, or if a drive did not finish (a failure, counted under Finished).
        """
        if not self.finished(group):
            return None
        values = [m.max_deviation if metric == "deviation" else m.end_position for m in self.simulated[group]]
        return max(values) * 100

    def run_value(self, metric: str) -> float | None:
        """The live run's deviation or end in cm, or its duration over the planned one; None without a run."""
        if self.run is None or self.run_left_out:
            return None
        m = self.run
        return {"deviation": m.max_deviation * 100, "end": m.end_position * 100,
                "drift": m.settle_drift * 100 if math.isfinite(m.settle_drift) else None,
                "ratio": m.duration / m.planned_duration if m.planned_duration else None}[metric]


def analyse(experiment: Path) -> Path:
    """Write the experiment's report into `<experiment>/_analysis/`; returns summary.md."""
    attempts = load_attempts(experiment)
    if not attempts:
        raise ValueError(f"no attempts under {experiment}")
    out = experiment / "_analysis"
    out.mkdir(exist_ok=True)
    planners = sorted({attempt.planner for attempt in attempts if not attempt.left_out})
    counted = [attempt for attempt in attempts if not attempt.left_out]
    charts = draw_all(counted, planners, out)
    lines = _header(experiment, attempts, planners)
    lines += _overview(counted, planners, out)
    lines += _pooled(attempts, planners, out)
    lines += _planning(counted, planners, out)
    lines += _charts(charts, ("success.png", "Which tasks each planner solves."),
                     ("plan_time.png", "How long each attempt planned; × found no path."),
                     ("paths.png", "How long and how roomy the paths found are."))
    lines += _simulation(counted, planners, out)
    lines += _charts(charts, ("driving.png", "How closely the simulated follower drives the paths found, and how "
                                             "close it ends, with the exact and with deviated robot models."))
    for environment in ("real", "sim"):
        lines += _runs(counted, planners, environment, out)
        name = f"drives_{environment}.png"
        if draw_lab([a for a in attempts if not a.left_out.startswith("excluded")], planners, out / name,
                    environment):
            lines += _charts([name], (name, "Where every committed run went in the lab, tasks or not: the path sent "
                                            "and the path driven."))
    lines += _process(counted, planners, out)
    lines += _failures(counted, out)
    lines += _all(attempts, out)
    lines += _checks(counted)
    lines += _plots(counted, planners, out)
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    (out / "results_prompt.md").write_text(RESULTS_PROMPT)
    return out / "summary.md"


def _charts(written: list[str], *charts: tuple[str, str]) -> list[str]:
    """Embed the charts written, each under the one line it answers."""
    lines = []
    for name, question in charts:
        if name in written:
            lines += [question, "", f"![{question}]({name})", ""]
    return lines


def load_attempts(experiment: Path) -> list[Attempt]:
    """Every attempt folder of an experiment, measured, with what is left out and why; draws their pictures."""
    excluded = _excluded(experiment)
    attempts = []
    for folder in sorted(path for path in experiment.iterdir() if path.is_dir() and not path.name.startswith("_")):
        description = load_description(folder)
        if description is None:
            continue
        attempt = Attempt(len(attempts) + 1, folder, description)
        if (folder / "plan.npz").is_file():
            plot_plan(folder)
        if recording_path(folder) is not None:
            data = load(folder)
            attempt.run = metrics(data)
            plot(data, attempt.run, folder / "overview.png", folder.name)
            write_report(folder, description, asdict(attempt.run))
            outcome = (description.get("run") or {}).get("outcome", "")
            if outcome.startswith(OPERATOR):
                attempt.run_left_out = f"run {outcome}"
        if (folder / "simulated.npz").is_file():
            arrays = load_plan_arrays(folder)
            attempt.drives = (description.get("simulated") or {}).get("drives", [])
            with np.load(folder / "simulated.npz") as data:
                traces = [data[f"trace_{k}"] for k in range(len(attempt.drives))]
            for drive, trace in zip(attempt.drives, traces):
                attempt.simulated.setdefault(drive["group"], []).append(
                    simulated_metrics(arrays["path_poses"], description["goal"], trace, not drive["not_finished"]))
        attempt.left_out = _left_out(attempt, excluded)
        attempts.append(attempt)
    return attempts


def _left_out(attempt: Attempt, excluded: dict[str, str]) -> str:
    """Why an attempt is left out of the comparisons; "" if it counts."""
    if attempt.folder.name in excluded:
        return f"excluded: {excluded[attempt.folder.name]}"
    task = BY_NAME.get(attempt.task or "")
    if task is None:
        return "no task (free attempt, e.g. driving to a task's start)"
    process_step = (attempt.description.get("conditions") or {}).get("process_step")
    if task.virtual and process_step is not None and attempt.environment != "offline":
        return "a process was loaded: the scene was not clean"
    return ""


# --- --- --- --- --- SECTIONS --- --- --- --- ---


def _header(experiment: Path, attempts: list[Attempt], planners: list[str]) -> list[str]:
    counts = {environment: sum(attempt.environment == environment for attempt in attempts)
              for environment in ENVIRONMENTS}
    runs = sum(attempt.run is not None for attempt in attempts)
    left = [attempt for attempt in attempts if attempt.left_out]
    names = [attempt.folder.name for attempt in attempts]
    left_line = (f"{len(left)} left out (listed under [All attempts](#all-attempts))" if left else "Nothing left out")
    lines = [
        f"# Base planners: experiment {experiment.name}",
        "",
        f"{len(attempts)} plan attempts ({', '.join(f'{n} {env}' for env, n in counts.items() if n)}), {runs} runs, "
        f"{names[0][:8]} {names[0][9:13]} to {names[-1][:8]} {names[-1][9:13]}. {left_line}.",
        "",
        "## Why",
        "",
        "Goals: the base planner drives Alice **into and out of narrow passages**, **finds paths around obstacles "
        "easily**, and, as the ultimate test, **drives into the centre of the bar structure at process step 110**. "
        "This report compares the planners on those goals.",
        "",
        "1. Which planner finds a path more often, and faster? → [Overview](#overview), [Planning](#planning)",
        "2. Which makes shorter paths with more margin? → [Planning](#planning)",
        "3. Which planner's paths does the follower drive closer to plan and end closer to the target, in "
        "simulation, with the exact model and with deviated models? → [Driving, simulated](#driving-simulated)",
        "4. And on the robot, and in the live simulator? → [Driving, real robot](#driving-real-robot), "
        "[Driving, live simulator](#driving-live-simulator)",
        "5. How do they do at process step 110? → [Process step 110](#process-step-110)",
        "6. Why did attempts fail? → [Failures](#failures)",
        "7. Over every plan and run, tasks or not, how does each planner do? → "
        "[All plans and runs, per planner](#all-plans-and-runs-per-planner)",
        "",
        "## What we measured",
        "",
        "Each task (below) is a start and a target for Alice and the obstacles in the way. An attempt plans the task "
        "from its start with one planner and its settings in the panel; committed, the follower drives it. "
        "Offline attempts (scripts/benchmark_base_planner.py) plan in the same planning world and drive each path "
        "found in a simulation of the onboard follower (crl_husky pure pursuit with its robot model) in two model "
        "groups: **exact model**, the simulated robot exactly as the follower's model; **deviated models**, the same "
        "with the true xICR the plan's xICR spread either side of the model's (values under "
        "[Driving, simulated](#driving-simulated)), one drive each.",
        "",
        f"Held fixed: the task frame at {_pose(TASK_ORIGIN)} in the lab (virtual tasks), the planning speeds, "
        "each planner's default settings (listed under [Planning](#planning)).",
        "",
        "| Metric | Unit | Definition |",
        "| --- | --- | --- |",
        "| Success | found / attempts | Attempts that returned a path |",
        "| Plan time | s | From the search's start to a path found, the planner's own checks included |",
        "| Length | m | Of the path found |",
        "| Clearance | cm | Least distance from Alice, arm included, to anything else along the path found, in the "
        "planning world, up to 30 cm |",
        "| Finished | finished / paths | Paths whose simulated drives all finished, per model group |",
        "| Deviation | cm | Largest distance of base_footprint from the path while following: mocap (runs), or the "
        "worst drive of a model group whose drives all finished (offline) |",
        "| End | cm | Distance of the last pose from the target: 2 s after the follower was done, or the worst "
        "drive of a model group |",
        "| Drift after done | cm | How far the robot moved after the follower reported done, until the recording "
        "ended 2 s later (runs) |",
        "| Drive / planned | | Time the follower followed over the path's planned duration |",
        "",
        "A failure is an attempt without a path, a run the follower did not finish (aborted, timed out), or a "
        "simulated drive that did not finish. Runs the operator stopped or replaced count neither way.",
        "",
        "**Bold**: the better planner's value in a pair, where the gap is larger than the spread (sd) of both and "
        "than the metric's resolution: plan time 0.2 s, length 0.10 m, clearance, deviation and end 1 cm, drive / "
        "planned 0.05; success: any difference. Only on the same task; in the overview, over the tasks where both "
        "planners have a value. Mean ± sd over a task's attempts; the overview averages per task first. n is "
        "attempts unless stated.",
        "",
        "! The follower aims for the end within 5 mm, but accepts up to 2 cm after 2 s near it, and up to 6 cm "
        "to the side once it has arrived along the path (crl_husky pure pursuit).",
        "! The RRT is random and is attempted several times per task; the lattice is deterministic and is attempted "
        "once offline.",
        "! Virtual obstacles do not exist on the floor: the robot runs only show how the robot drives the paths.",
        "",
        "### Tasks",
        "",
        "| Task | Goal | What it asks | Room cm | Solvable |",
        "| --- | --- | --- | --- | --- |",
    ]
    for task in TASKS:
        room = "open" if task.room is None else f"{task.room * 100:+.1f}"
        lines.append(f"| {task.name} | {task.goal} | {task.title} | {room} | {'yes' if task.solvable else 'no'} |")
    lines += ["", "### Planners", "", "| Planner | Settings |", "| --- | --- |"]
    for name in planners:
        settings = sorted({settings_line(attempt.description.get("settings", {})) for attempt in attempts
                           if attempt.planner == name})
        lines.append(f"| {name} | {'<br>'.join(settings)} |")
    return lines + [""]


def _overview(attempts: list[Attempt], planners: list[str], out: Path) -> list[str]:
    columns = [("Success", "success", "plan"), ("Plan time s", "seconds", "plan"), ("Length m", "length", "plan"),
               ("Clearance cm", "clearance", "plan"),
               ("Finished exact", "finished", EXACT), ("Deviation exact cm", "deviation", EXACT),
               ("End exact cm", "end", EXACT), ("Finished deviated", "finished", DEVIATED),
               ("Deviation deviated cm", "deviation", DEVIATED), ("End deviated cm", "end", DEVIATED),
               ("Deviation real cm", "deviation", "real"), ("End real cm", "end", "real")]
    solvable = [task.name for task in TASKS if task.solvable]
    cells, notes = {name: [] for name in planners}, []
    for title, metric, source in columns:
        per_task = {name: {} for name in planners}
        for task in solvable:
            for name in planners:
                values = [_value(attempt, metric, source) for attempt in attempts
                          if attempt.task == task and attempt.planner == name]
                values = [value for value in values if value is not None]
                if values:
                    per_task[name][task] = float(np.mean(values))
        shared = sorted(set.intersection(*(set(values) for values in per_task.values()))) if planners else []
        means = {name: (float(np.mean([per_task[name][task] for task in shared])) if shared else None)
                 for name in planners}
        kind = "success" if metric in ("success", "finished") else metric
        best = _best(kind, {name: (mean, 0.0) for name, mean in means.items()})
        for name in planners:
            cells[name].append(_cell(means[name], kind, name == best, percent=kind == "success"))
        notes.append(f"{title}: {len(shared)} tasks")
    lines = ["## Overview", "", "Each planner over the solvable tasks both have a value for: success rate, then the "
             "means of the per-task means.", "",
             "| Planner | " + " | ".join(title for title, _, _ in columns) + " |",
             "| --- | " + " | ".join("---" for _ in columns) + " |"]
    lines += [f"| {name} | " + " | ".join(cells[name]) + " |" for name in planners]
    lines += ["", "Tasks per column: " + "; ".join(notes) + ".", ""]
    _csv(out / "overview.csv", ["planner"] + [title for title, _, _ in columns],
         [[name] + [_strip(cell) for cell in cells[name]] for name in planners])
    return lines


def _pooled(attempts: list[Attempt], planners: list[str], out: Path) -> list[str]:
    """Every plan and run per planner, task or not (drives to a task's start too): more numbers, mixed problems."""
    pooled = [attempt for attempt in attempts if not attempt.left_out.startswith("excluded")
              and not _already_there(attempt)]
    columns = (("Attempts", "count"), ("Success", "success"), ("Plan time s", "seconds"), ("Length m", "length"),
               ("Clearance cm", "clearance"), ("Runs", "count"), ("Done", "done"), ("Deviation cm", "deviation"),
               ("End cm", "end"), ("Drift after done cm", "drift"), ("Drive / planned", "ratio"))
    header = ["Planner", "Where"] + [title for title, _ in columns]
    lines = ["## All plans and runs, per planner", "",
             "Every attempt pooled per planner, with or without a task, so drives to a task's start count too; only "
             f"attempts excluded by hand and plans already at their target (within {THERE_POSITION * 100:.0f} cm and "
             f"{math.degrees(THERE_YAW):.0f}°) are left out. The problems differ between planners, so this is about "
             "how each does in general, not a comparison on equal tasks. Mean ± sd over attempts; runs as in "
             "Driving.", "", "| " + " | ".join(header) + " |", "| " + " | ".join("---" for _ in header) + " |"]
    rows = []
    for environment in ENVIRONMENTS:
        group = {name: [a for a in pooled if a.planner == name and a.environment == environment] for name in planners}
        if not any(group.values()):
            continue
        runs = {name: [a for a in members if a.run is not None and not a.run_left_out]
                for name, members in group.items()}
        stats = []
        for title, metric in columns:
            values = {}
            for name in planners:
                members = runs[name] if metric in ("done", "deviation", "end", "drift", "ratio") else group[name]
                if metric == "count":
                    values[name] = (float(len(runs[name] if title == "Runs" else group[name])), 0.0)
                    continue
                found = [(1.0 if a.description["run"]["outcome"] == "done" else 0.0) if metric == "done"
                         else a.run_value(metric) if metric in ("deviation", "end", "drift", "ratio")
                         else a.value(metric) for a in members]
                found = [value for value in found if value is not None]
                values[name] = (float(np.mean(found)), float(np.std(found))) if found else (None, 0.0)
            kind = "success" if metric == "done" else metric
            stats.append((kind, values, None if metric == "count" else _best(kind, values)))
        if rows:
            lines.append("| " + " | ".join("" for _ in header) + " |")
        for name in planners:
            if not group[name]:
                continue
            cells = [f"{values[name][0]:.0f}" if kind == "count" else
                     _cell(values[name][0], kind, best == name, spread=values[name][1], percent=kind == "success")
                     for kind, values, best in stats]
            row = [name, environment] + cells
            rows.append(row)
            lines.append("| " + " | ".join(row) + " |")
    _csv(out / "pooled.csv", header, [[_strip(cell) for cell in row] for row in rows])
    return lines + [""]


def _already_there(attempt: Attempt) -> bool:
    """Whether the attempt planned from where it already was: within THERE_POSITION and THERE_YAW of its target."""
    start, goal = attempt.description.get("start"), attempt.description.get("goal")
    if not start or not goal:
        return False
    return (math.hypot(start[0] - goal[0], start[1] - goal[1]) < THERE_POSITION
            and abs(math.remainder(start[2] - goal[2], math.tau)) < THERE_YAW)


def _planning(attempts: list[Attempt], planners: list[str], out: Path) -> list[str]:
    metrics_ = (("Success", "success"), ("Plan time s", "seconds"), ("Length m", "length"),
                ("Clearance cm", "clearance"))
    lines = ["## Planning", "", "Per task, how often each planner found a path, how fast, and the path's length and "
             "clearance, over every environment's attempts.", ""]
    rows = _pair_table(attempts, planners, metrics_, lambda attempt, metric: attempt.value(metric))
    return lines + rows + [""] + _table_csv(out / "planning.csv", rows)


def _simulation(attempts: list[Attempt], planners: list[str], out: Path) -> list[str]:
    offline = [attempt for attempt in attempts if attempt.environment == "offline" and attempt.plan.get("found")]
    lines = ["## Driving, simulated", ""]
    if not offline:
        return lines + ["Not run: no offline attempts.", ""]
    for group, file in ((EXACT, "simulation_exact.csv"), (DEVIATED, "simulation_deviated.csv")):
        members = [attempt for attempt in offline if group in attempt.simulated]
        values = sorted({f"{drive['xicr']:+.3f}" for attempt in members for drive in attempt.drives
                         if drive["group"] == group})
        lines += [f"### {group.capitalize()}", ""]
        if not members:
            lines += ["Not run: no drives in this group.", ""]
            continue
        lines += [f"Per task, the follower simulated on each path found offline with the true xICR at "
                  f"{', '.join(values)} m: whether it finished, and its deviation and end"
                  + (" at the worst of them." if len(values) > 1 else "."), ""]
        rows = _pair_table(members, planners, (("Finished", "finished"), ("Deviation cm", "deviation"),
                                               ("End cm", "end")),
                           lambda attempt, metric, group=group: attempt.finished(group) if metric == "finished"
                           else attempt.simulated_value(metric, group))
        lines += rows + [""] + _table_csv(out / file, rows)
    return lines


def _runs(attempts: list[Attempt], planners: list[str], environment: str, out: Path) -> list[str]:
    title = {"real": "Driving, real robot", "sim": "Driving, live simulator"}[environment]
    runs = [attempt for attempt in attempts if attempt.environment == environment and attempt.run is not None
            and not attempt.run_left_out]
    lines = [f"## {title}", ""]
    if not runs:
        return lines + [f"Not run: no {'robot' if environment == 'real' else 'live simulator'} runs on tasks yet.",
                        ""]
    lines += [f"Per task, the committed paths as driven ({'mocap' if environment == 'real' else 'simulated pose'}): "
              "whether the follower finished, deviation, end, and time against plan.", ""]
    rows = _pair_table(runs, planners, (("Done", "done"), ("Deviation cm", "deviation"), ("End cm", "end"),
                                        ("Drift after done cm", "drift"), ("Drive / planned", "ratio")),
                       lambda attempt, metric: (1.0 if attempt.description["run"]["outcome"] == "done" else 0.0)
                       if metric == "done" else attempt.run_value(metric))
    return lines + rows + [""] + _table_csv(out / f"runs_{environment}.csv", rows)


def _process(attempts: list[Attempt], planners: list[str], out: Path) -> list[str]:
    tasks = [task for task in TASKS if not task.virtual]
    lines = [f"## Process step {tasks[0].process_step}", ""]
    if not any(attempt.task == task.name for attempt in attempts for task in tasks):
        return lines + ["Not run: no attempts on these tasks yet.", ""]
    columns = (("Success", "success", "plan"), ("Plan time s", "seconds", "plan"), ("Length m", "length", "plan"),
               ("Clearance cm", "clearance", "plan"), ("Deviation exact cm", "deviation", EXACT),
               ("End exact cm", "end", EXACT), ("Deviation deviated cm", "deviation", DEVIATED),
               ("End deviated cm", "end", DEVIATED), ("Deviation run cm", "deviation", "run"),
               ("End run cm", "end", "run"))
    header = ["Task", "Planner", "Where", "Attempts"] + [title for title, _, _ in columns]
    lines += ["Every number of the ultimate test, into the bar structure and back out, per task, planner and where "
              "it was attempted.", "",
              "| " + " | ".join(header) + " |", "| " + " | ".join("---" for _ in header) + " |"]
    rows = []
    for task, environment in ((task, environment) for task in tasks for environment in ENVIRONMENTS):
        mine = [attempt for attempt in attempts if attempt.task == task.name]
        group = {name: [a for a in mine if a.planner == name and a.environment == environment] for name in planners}
        if not any(group.values()):
            continue
        if rows:
            lines.append("| " + " | ".join("" for _ in header) + " |")
        stats = []
        for _, metric, source in columns:
            source = environment if source == "run" else source
            values = {}
            for name, members in group.items():
                found = [_value(a, metric, source) for a in members]
                found = [value for value in found if value is not None]
                values[name] = (float(np.mean(found)), float(np.std(found))) if found else (None, 0.0)
            stats.append((metric, values, _best(metric, values)))
        for name in planners:
            if not group[name]:
                continue
            cells = [_cell(values[name][0], metric, best == name, spread=values[name][1],
                           percent=metric == "success") for metric, values, best in stats]
            row = [task.name, name, environment, str(len(group[name]))] + cells
            rows.append(row)
            lines.append("| " + " | ".join(row) + " |")
    _csv(out / "process.csv", header, [[_strip(cell) for cell in row] for row in rows])
    return lines + [""]


def _failures(attempts: list[Attempt], out: Path) -> list[str]:
    failed = [attempt for attempt in attempts if not attempt.plan.get("found")
              or (attempt.run is not None and not attempt.run_left_out
                  and attempt.description["run"]["outcome"] != "done")
              or any(not m.finished for drives in attempt.simulated.values() for m in drives)]
    lines = ["## Failures", ""]
    if not failed:
        return lines + ["None.", ""]
    lines += ["Attempts without a path, runs not done and simulated drives not finished, grouped by task, planner "
              "and what happened; numbers link to the attempt's picture.", "",
              "| Task | Planner | Where | What happened | Count | Attempts |", "| --- | --- | --- | --- | --- | --- |"]
    groups: dict[tuple, list[Attempt]] = {}
    for attempt in failed:
        groups.setdefault((attempt.task, attempt.planner, attempt.environment, _what(attempt)), []).append(attempt)
    rows = []
    for (task, name, environment, what), members in sorted(groups.items()):
        links = ", ".join(_link(attempt) for attempt in members)
        rows.append([task, name, environment, what, str(len(members)), links])
        lines.append("| " + " | ".join(rows[-1]) + " |")
    _csv(out / "failures.csv", ["task", "planner", "where", "what", "count", "attempts"],
         [row[:5] + [" ".join(str(a.number) for a in groups[tuple(row[:4])])] for row in rows])
    return lines + [""]


def _all(attempts: list[Attempt], out: Path) -> list[str]:
    lines = ["## All attempts", "", "Every attempt in time order, left-out ones included; numbers link to the plan's "
             "picture, Run to the run's overview.", "",
             "| # | Task | Planner | Where | Found | Plan time s | Length m | Clearance cm | Deviation exact cm | "
             "End exact cm | Deviation deviated cm | End deviated cm | Run | Deviation run cm | End run cm | "
             "Left out |", "| " + " | ".join("---" for _ in range(16)) + " |"]
    rows = []
    for attempt in attempts:
        run = (f"[{attempt.description['run']['outcome'].split(':')[0]}]({_rel(attempt.folder / 'overview.png')})"
               if attempt.run is not None else "not committed")
        row = [_link(attempt), attempt.task or "—", attempt.planner, attempt.environment,
               "yes" if attempt.plan.get("found") else "no", _cell(attempt.value("seconds"), "seconds"),
               _cell(attempt.value("length"), "length"), _cell(attempt.value("clearance"), "clearance"),
               _cell(attempt.simulated_value("deviation", EXACT), "deviation"),
               _cell(attempt.simulated_value("end", EXACT), "end"),
               _cell(attempt.simulated_value("deviation", DEVIATED), "deviation"),
               _cell(attempt.simulated_value("end", DEVIATED), "end"), run,
               _cell(attempt.run_value("deviation"), "deviation"), _cell(attempt.run_value("end"), "end"),
               attempt.left_out or attempt.run_left_out or ""]
        rows.append(row)
        lines.append("| " + " | ".join(row) + " |")
    _csv(out / "attempts.csv", ["number", "folder", "task", "planner", "where", "found", "plan_time_s", "length_m",
                                "clearance_cm", "deviation_exact_cm", "end_exact_cm", "deviation_deviated_cm",
                                "end_deviated_cm", "run_outcome", "deviation_run_cm", "end_run_cm", "left_out"],
         [[str(a.number), a.folder.name, a.task or "", a.planner, a.environment, str(bool(a.plan.get("found"))),
           *(_strip(cell) for cell in row[5:12]), (a.description.get("run") or {}).get("outcome", ""),
           *(_strip(cell) for cell in row[13:15]), a.left_out or a.run_left_out] for a, row in zip(attempts, rows)])
    return lines + [""]


def _checks(attempts: list[Attempt]) -> list[str]:
    poses = np.array([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 0.0, math.pi / 2), (1.0, 1.0, math.pi / 2)])
    on_path = np.array([poses[0]] + densify([tuple(pose) for pose in poses], 0.01, 0.01))
    synthetic = simulated_metrics(poses, poses[-1], on_path, True)
    unsolvable = [task.name for task in TASKS if not task.solvable]
    found = [attempt for attempt in attempts if attempt.task in unsolvable and attempt.plan.get("found")]
    tried = [attempt for attempt in attempts if attempt.task in unsolvable]
    return ["## Checks", "", "The measurement against known answers, computed with every report.", "",
            "| Check | Expected | Got |", "| --- | --- | --- |",
            f"| A drive exactly on its path: deviation, end | 0.0 cm, 0.0 cm | {synthetic.max_deviation * 100:.1f} cm, "
            f"{synthetic.end_position * 100:.1f} cm |",
            f"| Unsolvable tasks ({', '.join(unsolvable)}): paths found | 0 of the attempts | "
            + (f"{len(found)} of {len(tried)} |" if tried else "not run |"), ""]


def _plots(attempts: list[Attempt], planners: list[str], out: Path) -> list[str]:
    _tasks_plot(attempts, planners, out / "tasks.png")
    (out / "metrics.png").unlink(missing_ok=True)  # replaced by the charts above
    return ["## Where the paths go", "", "Each planner's paths over the floor of the task's first attempt.", "",
            "![Each planner's paths per task](tasks.png)", "",
            "## Files", "", "- One CSV per table: overview, pooled, planning, simulation_exact, simulation_deviated, "
            "runs_real, runs_sim, process, "
            "failures, attempts.", "- `results_prompt.md`: the prompt for results.md.", ""]


# --- --- --- --- --- TABLES --- --- --- --- ---


def _pair_table(attempts: list[Attempt], planners: list[str], metrics_, value_of) -> list[str]:
    """Per task (grouped by goal, empty rows between), each metric for each planner side by side, better in bold."""
    short = {name: _short(name) for name in planners}
    header = ["Task", "n " + "/".join(short[name] for name in planners)] + [f"{title} {short[name]}"
                                                                            for title, _ in metrics_
                                                                            for name in planners]
    lines = ["| " + " | ".join(header) + " |", "| " + " | ".join("---" for _ in header) + " |"]
    goal = None
    for task in TASKS:
        members = [attempt for attempt in attempts if attempt.task == task.name]
        if not members:
            continue
        if goal is not None and task.goal != goal:
            lines.append("| " + " | ".join("" for _ in header) + " |")
        goal = task.goal
        counts = "/".join(str(sum(a.planner == name for a in members)) for name in planners)
        cells = []
        for _, metric in metrics_:
            stats = {}
            for name in planners:
                values = [value_of(a, metric) for a in members if a.planner == name]
                values = [value for value in values if value is not None]
                stats[name] = (float(np.mean(values)), float(np.std(values))) if values else (None, 0.0)
            kind = "success" if metric in ("success", "finished", "done") else metric
            best = _best(kind, stats)
            cells += [_cell(stats[name][0], kind, name == best, spread=stats[name][1],
                            percent=kind == "success") for name in planners]
        lines.append(f"| {task.name} | {counts} | " + " | ".join(cells) + " |")
    return lines


def _best(metric: str, stats: dict) -> str | None:
    """The planner whose value is better beyond the spread and the resolution (BETTER); None if none is."""
    sign, resolution = BETTER[metric]
    known = {name: value for name, value in stats.items() if value[0] is not None}
    if len(known) < 2:
        return None
    ranked = sorted(known, key=lambda name: -sign * known[name][0])
    first, second = known[ranked[0]], known[ranked[1]]
    gap = abs(first[0] - second[0])
    if metric == "success":
        return ranked[0] if gap > 1e-9 else None
    return ranked[0] if gap > max(first[1], second[1], resolution) else None


def _cell(value: float | None, metric: str, bold: bool = False, spread: float = 0.0, percent: bool = False) -> str:
    """A table cell: the value with the metric's decimals (± spread when there is one), "—" when missing."""
    if value is None or not math.isfinite(value):
        return "—"
    if percent:
        text = f"{value * 100:.0f} %"
    else:
        decimals = DECIMALS.get(metric, 2)
        text = f"{value:.{decimals}f}" + (f" ± {spread:.{decimals}f}" if spread > 0 else "")
    return f"**{text}**" if bold else text


def _value(attempt: Attempt, metric: str, source: str) -> float | None:
    """One number of an attempt, by metric and where it comes from.

    `source` is "plan", a simulated model group (EXACT, DEVIATED: finished, deviation, end), or a run's environment
    ("real", "sim": deviation, end).
    """
    if source == "plan" or metric in ("success", "seconds", "length", "clearance"):
        return attempt.value(metric)
    if source in (EXACT, DEVIATED):
        return attempt.finished(source) if metric == "finished" else attempt.simulated_value(metric, source)
    return attempt.run_value(metric) if attempt.environment == source else None


def _table_csv(path: Path, lines: list[str]) -> list[str]:
    """Write a Markdown table's rows as a CSV (separator rows left out); returns nothing to add to the report."""
    rows = [[_strip(cell.strip()) for cell in line.strip("|").split("|")] for line in lines
            if not line.startswith("| ---") and line.replace("|", "").strip()]
    _csv(path, rows[0], rows[1:])
    return []


def _csv(path: Path, header: list[str], rows: list[list[str]]) -> None:
    with path.open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(header)
        writer.writerows(rows)


# --- --- --- --- --- PLOTS --- --- --- --- ---


def _tasks_plot(attempts: list[Attempt], planners: list[str], out: Path) -> None:
    tasks = [task for task in TASKS if any(attempt.task == task.name for attempt in attempts)]
    if not tasks:
        return
    columns = 3
    rows = math.ceil(len(tasks) / columns)
    figure = Figure(figsize=(4.2 * columns, 3.8 * rows), layout="constrained")
    for k, task in enumerate(tasks):
        axis = figure.add_subplot(rows, columns, k + 1)
        members = [attempt for attempt in attempts if attempt.task == task.name]
        draw_floor(axis, load_plan_arrays(members[0].folder))
        for name in planners:
            color = colour(name, planners)
            for attempt in members:
                arrays = load_plan_arrays(attempt.folder) if attempt.planner == name else None
                if arrays is not None and len(arrays["path_poses"]):
                    axis.plot(*arrays["path_poses"][:, :2].T, "-", color=color, linewidth=1.2, alpha=0.8)
            axis.plot([], [], "-", color=color, label=_short(name))
        start, target = members[0].description["start"], members[0].description["goal"]
        axis.plot(*start[:2], "o", color="tab:green")
        axis.plot(*target[:2], "x", color="tab:purple", markersize=8)
        axis.set_aspect("equal", adjustable="datalim")
        axis.set_title(task.name, fontsize=10)
        axis.tick_params(labelsize=7)
        if k == 0:
            axis.legend(fontsize=7, loc="best")
    figure.suptitle("Paths per task: start ●, target ×, planners by colour", fontsize=11)
    figure.savefig(out, dpi=110)


# --- --- --- --- --- SMALL HELPERS --- --- --- --- ---


def _what(attempt: Attempt) -> str:
    """What failed in an attempt, short, without numbers that differ between attempts."""
    if not attempt.plan.get("found"):
        first = attempt.plan.get("reason", "").split(";")[0].split(" at (")[0].replace("|", "/")
        return "no path: " + " ".join("N" if any(c.isdigit() for c in word) else word for word in first.split())
    if attempt.run is not None and attempt.description["run"]["outcome"] != "done":
        return "run: " + attempt.description["run"]["outcome"].split(":")[0]
    parts = []
    for group in (EXACT, DEVIATED):
        stuck = [f"{drive['xicr']:+.2f}" for drive in attempt.drives
                 if drive["group"] == group and drive["not_finished"]]
        if stuck:
            parts.append(f"{group} (xICR {', '.join(stuck)})")
    return "simulated drive did not finish: " + "; ".join(parts)


def _short(name: str) -> str:
    """A planner's short name for table headers: its first word."""
    return name.split()[0]


def _link(attempt: Attempt) -> str:
    """The attempt's number, linking to its plan picture."""
    return f"[{attempt.number}]({_rel(attempt.folder / 'plan.png')})"


def _rel(path: Path) -> str:
    """A path relative to the _analysis folder, for links."""
    return f"../{path.parent.name}/{path.name}"


def _excluded(experiment: Path) -> dict[str, str]:
    """`_excluded.txt`: one attempt folder per line, then its reason."""
    path = experiment / "_excluded.txt"
    if not path.is_file():
        return {}
    out = {}
    for line in path.read_text().splitlines():
        if line.strip() and not line.startswith("#"):
            name, _, reason = line.strip().partition(" ")
            out[name] = reason.strip() or "no reason given"
    return out


def _pose(pose) -> str:
    return f"({pose[0]:.2f} m, {pose[1]:.2f} m, {math.degrees(pose[2]):.0f}°)"


def _strip(cell: str) -> str:
    return cell.replace("**", "")


RESULTS_PROMPT = """# Prompt for results.md

Read `summary.md` in this folder (and the CSVs it names if a number needs checking) and write `results.md` beside it,
following doc/experiment_report_guide.md. Sections:

1. **Answer per question**: one short paragraph per numbered question in summary.md's Why, each with the numbers that
   answer it, naming the planner that is better where a value is bold, and saying "no difference" where none is.
2. **The goals**: for each goal (narrow passages, around obstacles, process step 110), whether the data shows a
   planner meeting it, and which tasks still fail.
3. **Caveats**: what limits these numbers (attempts per task, tasks not run, virtual obstacles, follower effects).
4. **Next steps**: at most five, each tied to a number above.

Rules: only what the data shows, with its numbers; a difference within the spread is no difference; no new metrics.
"""
