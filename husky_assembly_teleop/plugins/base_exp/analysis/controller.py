"""
The controller report, summary.md: how well each scenario's controller follows the paths, from the path runs.

It reads why (the hypotheses), what was measured, then numbers only; the written results come from RESULTS_PROMPT.

- Comparisons stay within one robot, on the paths all its controllers finished, each path once.
- The ideal simulator is a reference: its own group in the headline, a column per path, left out of the rest.
- No breakdown by segment type: a segment's errors include what the one before left, and every path ends in a turn
  piece that takes the final approach. Templates start at the robot's pose, so they are clean.
"""

from __future__ import annotations

import math
import warnings
from pathlib import Path

import numpy as np
import pandas as pd
from matplotlib.figure import Figure
from matplotlib.lines import Line2D

from ..auto import MODE, SPEED, STANDARD_SET
from ..record import CONDITIONS
from .common import (ERRORS, NUMBERS, PROGRESS_BINS, TEMPLATES, Experiment, link, markdown, robots, save,
                     scenario_row, scenarios, shared_paths)

COLORS = dict(zip(TEMPLATES, ("tab:blue", "tab:orange", "tab:green", "tab:red", "tab:purple")))
#: The template whose position error is partly the skid steer's own (see "What we measured").
SPOT_TURN = "Turn on the spot"
#: Columns of the headline and by-template tables, in order.
COLUMNS = ["robot", "scenario", "attempts", "done", "success [%]", "paths compared", *ERRORS]

#: The prompt for the written results, saved beside every analysis. A subagent gets it with the analysis folder.
RESULTS_PROMPT = """\
You write the results section of one base-controller experiment, from its analysis folder:
{folder}

Read summary.md (the controllers) and robot_model.md (the robot) and the CSV files they are made of (summary.md:
headline*.csv, per_path.csv, by_template.csv, failures.csv, largest_end_errors.csv; robot_model.md: model.csv,
model_stability.csv, model_by_template.csv, prediction*.csv, sim_vs_real*.csv, checks.csv, wheels.csv). Look at
templates.png, along_path.png, outcomes.png, model_stability.png and model_replay.png. Ignore other files.

Each report starts with why and what was measured: use its definitions and terms. Everything after that is numbers
only; reading them is your job. Write results.md in the same folder with exactly these sections, in this order:

# Results: <experiment name>
One line: date range, number of runs, scenarios, fixed conditions.

## Hypotheses
Per hypothesis in summary.md: supported, not supported, or not yet tested, on which robot (simulator or real), with
the numbers that show it: success rate first, then the end error, then the error while moving.

## The robot model
2 to 4 sentences: the real robot's model, whether one model fits every run, how much better it predicts than the
ideal model, whether the simulator with it drives like the real robot, and whether every check passes.

## Where the errors come from
3 to 6 bullets, each naming a cause with numbers: a template, a path, or a part of the path (along_path.png). Say
which scenario each holds for, and relate it to the robot model (delay, efficiencies, xICR). On a turn on the spot,
say how much of the position error the skid steer leaves to any controller.

## Failures
How runs failed, what the failures have in common and the likely reason; "none" if there are none. A robot that
circles the goal and never settles is a controller failure: say so where it happens.

## Caveats
Small n (fewer than 5 runs per path or template and scenario), scenarios not collected, failed checks, anything else
that limits the conclusions.

## Next steps
2 to 4 concrete experiments or changes the data points to.

Rules: state only what the data shows, with the numbers that show it; a difference within the spread between repeats
is no difference; round to one decimal; plain language, short sentences; no new plots; do not change any other file.
"""


def write(experiment: Experiment) -> None:
    """Write summary.md, its CSV files and plots, and results_prompt.md into the experiment's analysis folder."""
    runs, out = experiment.runs, experiment.out
    standard = runs[runs.standard >= 0]
    compared = standard if not standard.empty else runs
    # * Breakdowns and plots: the ideal simulator follows every path exactly, so it adds only zeros.
    shown = runs[runs.robot != "sim ideal"]
    tables = {"headline": headline(compared), "per_path": per_path(standard), "by_template": by_template(shown),
              "failures": failures(runs, experiment.profiles, out), "largest_end_errors": largest(shown, out)}
    no_spot = compared[compared.template != SPOT_TURN]
    if not no_spot.empty and len(no_spot) < len(compared):
        tables["headline_no_spot_turns"] = headline(no_spot)
    runs.to_csv(out / "runs.csv", index=False)
    for name, table in tables.items():
        table.drop(columns="comparable", errors="ignore").to_csv(out / f"{name}.csv", index=False)
    plots = [p for p in (_templates(tables["by_template"], shown, out), _along_path(shown, experiment.profiles, out),
                         _outcomes(shown, out)) if p]

    fixed = ", ".join([f"{SPEED[0]} m/s and {SPEED[1]:.0f}°/s", MODE.lower()]
                      + [f"{key} {value}" for key, value in CONDITIONS.items()])
    which = (f"the standard paths, the same {len(STANDARD_SET)} in every scenario" if not standard.empty
             else "every path (no standard-path runs)")
    errors = dict(lower=ERRORS, higher=("success [%]",))
    lines = [f"# Controller evaluation: {experiment.name}\n",
             experiment.intro() + "\n",
             "## Why\n",
             "Goal: a base controller that follows paths accurately enough for assembly. This report tests two "
             "hypotheses:\n",
             "- **Hypothesis 2:** pure pursuit with the ideal robot model is not good enough.",
             "- **Hypothesis 3:** pure pursuit with the robot's identified model is better.\n",
             "For each robot (the real one, or the simulator with a robot model) it answers:\n",
             "1. How often does each controller finish a path, and how far off does the robot end? (section 1)",
             "2. On which paths and kinds of path does each controller fail or do worst? (sections 2, 3)",
             "3. How do runs fail, and which runs ended furthest off? (sections 4, 5)\n",
             "## What we measured\n",
             f"Held fixed: {fixed}. The monitor measures every run against the path sent, the same way for every "
             "controller.\n",
             "- **Outcome**: done (the controller finished and the robot settled), timed out, soft stopped (too close "
             "to an obstacle), or stopped by the operator (left out of the success rate).",
             "- **Success**: done runs per attempt, each path once (the mean of the paths' rates).",
             "- **End**: distance and heading from the path's last pose after the robot settled. **Moving**: mean "
             "distance and heading from the path while the controller followed it; on a turn on the spot, distance "
             "from the spot. Position in cm, heading in degrees.",
             "- **Mean ± spread**: the mean over the paths, each once, and the standard deviation between repeats of "
             "one path. Bold marks the better controller on one robot, on the same paths, where the gap exceeds the "
             "spread.",
             "- ! On a turn on the spot the tracked point lies |xICR| from the point the robot turns about, so it "
             "moves on a circle around it: part of the position error there is the skid steer's, whatever the "
             "controller. Section 1 also gives the headline without these paths.",
             "- The ideal simulator follows its commands exactly: a reference for the controller alone.\n",
             "Scenarios: a robot and a controller with the robot model it assumes, \"<robot> · <controller>\" "
             "(PP: pure pursuit).\n",
             markdown(_scenario_table(runs)), "",
             "## 1. Headline\n",
             f"Each scenario on {which}. Errors over the paths every controller of that robot finished.\n",
             markdown(tables["headline"], "robot", **errors), ""]
    if "headline_no_spot_turns" in tables:
        lines += [f"The same without \"{SPOT_TURN}\":\n",
                  markdown(tables["headline_no_spot_turns"], "robot", **errors), ""]
    if not standard.empty:
        lines += ["## 2. Path by path\n",
                  "Which standard paths each scenario finishes. Cell: done/attempts · end position [cm] · moving "
                  "position [cm], means over the done runs.\n",
                  markdown(tables["per_path"]), ""]
    lines += ["## 3. By template\n",
              "Every path run but the ideal simulator's. Errors over the paths of the template every controller of "
              "that robot finished; where they share none, over each scenario's own done paths (\"own\"), without "
              "bold.\n",
              markdown(tables["by_template"], "robot", **errors, within=["robot", "template"],
                       comparable="comparable"), "",
              "## 4. Failures\n",
              "Runs that did not end done, by scenario, path and outcome. Reached: how far along the path the robot "
              "got; max off: the largest distance from the path while moving.\n",
              markdown(tables["failures"], "scenario"), "",
              "## 5. Largest end errors\n",
              "Per scenario and template, the done run that ended furthest off.\n",
              markdown(tables["largest_end_errors"], "scenario"), "",
              "## 6. Plots\n",
              "Without the ideal simulator. templates.png: section 3 as bars (\"none done\" where no run finished); "
              "along_path.png: the errors along the path, mean over the done runs per template; outcomes.png: how "
              "every run ended.\n",
              *[f"![{name}]({name})\n" for name in plots]]
    (out / "summary.md").write_text("\n".join(lines))
    (out / "results_prompt.md").write_text(RESULTS_PROMPT.format(folder=out.resolve()))


# --- --- --- --- --- TABLES --- --- --- --- ---

def headline(runs: pd.DataFrame) -> pd.DataFrame:
    """Per robot and scenario: success, and errors on the paths every controller of that robot finished.

    A controller run on another robot but not on this one gets an empty row, so the missing scenario shows (not for
    the ideal simulator, where a robot model has nothing to adapt to).
    """
    if runs.empty:
        return pd.DataFrame()
    first = runs.drop_duplicates("scenario").set_index("scenario")
    controllers = list(dict.fromkeys(first.controller_label[s] for s in scenarios(runs)))
    rows = []
    for robot in robots(runs):
        mine = [s for s in scenarios(runs) if first.robot[s] == robot]
        shared = shared_paths(runs, mine)
        for controller in controllers:
            scenario = next((s for s in mine if first.controller_label[s] == controller), None)
            if scenario is not None:
                rows.append({"robot": robot, "scenario": scenario,
                             **scenario_row(runs[runs.scenario == scenario], shared)})
            elif robot != "sim ideal":
                rows.append({"robot": robot, "scenario": f"{robot} · {controller} (not collected)", "attempts": 0}
                            | dict.fromkeys(COLUMNS[3:], "—"))
    return pd.DataFrame(rows, columns=COLUMNS)


def per_path(runs: pd.DataFrame) -> pd.DataFrame:
    """The standard paths one by one, a column per scenario: "done/attempts · end cm · moving cm"."""
    if runs.empty:
        return pd.DataFrame()
    rows = []
    for index, (name, values) in enumerate(STANDARD_SET):
        # * The label the runs carry, so every table names a path the same way.
        labels = runs.path[runs.standard == index]
        row = {"path": labels.iloc[0] if len(labels) else f"{name} {_numbers(values)}"}
        for scenario in scenarios(runs):
            mine = runs[(runs.standard == index) & (runs.scenario == scenario) & runs.counted]
            done = mine[mine.success]
            cell = f"{len(done)}/{len(mine)}" if len(mine) else "not run"
            if len(done):
                cell += f" · {done.end_position.mean() * 100:.1f} · {done.motion_position.mean() * 100:.1f}"
            row[scenario] = cell
        rows.append(row)
    return pd.DataFrame(rows)


def by_template(runs: pd.DataFrame) -> pd.DataFrame:
    """Per robot, template and scenario: success, errors on the template's shared paths, duration over expected.

    Column "comparable" is False where the robot's controllers share no done path of the template: each row then
    shows its own done paths.
    """
    rows = []
    for robot in robots(runs) if not runs.empty else []:
        for template in TEMPLATES:
            group = runs[(runs.robot == robot) & (runs.template == template)]
            mine = [s for s in scenarios(runs) if s in set(group.scenario)]
            shared = shared_paths(group, mine)
            for scenario in mine:
                own = group[group.scenario == scenario]
                paths = shared or set(own.path[own.success])
                done = own[own.success & own.path.isin(paths)]
                ratio = done.groupby("path").duration_ratio.mean().mean() if len(done) else math.nan
                row = {"robot": robot, "template": template, "scenario": scenario, **scenario_row(own, paths),
                       "duration / expected": round(float(ratio), 2), "comparable": bool(shared)}
                if not shared:
                    row["paths compared"] = f"{len(paths)} (own)"
                rows.append(row)
    columns = ["robot", "template", *COLUMNS[1:], "duration / expected", "comparable"]
    return pd.DataFrame(rows).reindex(columns=columns) if rows else pd.DataFrame()


def failures(runs: pd.DataFrame, profiles: dict, out: Path) -> pd.DataFrame:
    """Runs that did not end done, per scenario, path and outcome: how many, how far they got and strayed, links."""
    rows = []
    failed = runs[~runs.success]
    for scenario in scenarios(runs):
        mine = failed[failed.scenario == scenario]
        for (path, reason), group in mine.groupby(["path", "reason"], sort=False):
            attempts = int((runs.counted & (runs.scenario == scenario) & (runs.path == path)).sum())
            reached = [_reached(profiles[name]) for name in group.run]
            rows.append({"scenario": scenario, "path": path, "outcome": reason,
                         "failed / attempts": f"{len(group)}/{attempts}", "reached [%]": _range(reached),
                         "max off [cm]": _range(group.max_position * 100),
                         "runs": ", ".join(link(row, out, _time(row)) for _, row in group.iterrows())})
    return pd.DataFrame(rows)


def largest(runs: pd.DataFrame, out: Path) -> pd.DataFrame:
    """Per scenario and template, the done run with the largest end position error, with its four errors."""
    rows = []
    done = runs[runs.success]
    for scenario in scenarios(runs):
        for template in TEMPLATES:
            mine = done[(done.scenario == scenario) & (done.template == template)]
            if mine.empty:
                continue
            row = mine.loc[mine.end_position.idxmax()]
            rows.append({"scenario": scenario, "template": template, "path": row.path,
                         **{label: round(row[column] * factor, 1) for column, label, factor in NUMBERS},
                         "run": link(row, out, _time(row))})
    return pd.DataFrame(rows)


def _scenario_table(runs: pd.DataFrame) -> pd.DataFrame:
    """Each scenario spelt out: the robot (and the simulator's model), the controller and the model it assumes."""
    rows = []
    for scenario in scenarios(runs):
        mine = runs[runs.scenario == scenario]
        first = mine.iloc[0]
        robot = "the real robot" if first.environment == "real" else f"simulator, {first.model or 'ideal'}"
        rows.append({"scenario": scenario, "robot": robot,
                     "controller": f"{first.controller}, assuming {first.controller_model}", "runs": len(mine)})
    return pd.DataFrame(rows)


def _reached(profile: np.ndarray) -> float:
    """How far along the path a run got, percent, from its error profile; 0 if it never moved."""
    reached = np.flatnonzero(np.isfinite(profile[:, 0]))
    return 100 * (reached[-1] + 1) / PROGRESS_BINS if len(reached) else 0.0


def _range(values) -> str:
    """Whole numbers as "18" or "18 to 20"; "—" when none is known."""
    values = np.asarray(values, dtype=float)
    if not np.isfinite(values).any():
        return "—"
    low, high = round(float(np.nanmin(values))), round(float(np.nanmax(values)))
    return f"{low}" if low == high else f"{low} to {high}"


def _time(row: pd.Series) -> str:
    """When a run started, "17:54:32", for its link; the folder name when unknown."""
    return row.started[11:19] if len(row.started) >= 19 else row.run


def _numbers(values: dict) -> str:
    """Template numbers in short, e.g. "radius=0.5, angle=-90"."""
    return ", ".join(f"{key}={value:g}" for key, value in values.items())


# --- --- --- --- --- PLOTS --- --- --- --- ---

def _templates(table: pd.DataFrame, runs: pd.DataFrame, out: Path) -> str:
    """Grouped bars: the four errors per template, one bar per scenario; "none done" where it ran but never finished."""
    if table.empty:
        return ""
    templates = [t for t in TEMPLATES if t in set(table.template)]
    names = scenarios(runs)
    width = 0.8 / max(len(names), 1)
    figure = Figure(figsize=(12, 6), layout="constrained")
    for axis, label in zip(figure.subplots(2, 2).ravel(), ERRORS):
        for i, scenario in enumerate(names):
            rows = table[table.scenario == scenario].set_index("template").reindex(templates)
            values = pd.to_numeric(rows[label].astype(str).str.split().str[0], errors="coerce")
            x = np.arange(len(templates)) + (i - (len(names) - 1) / 2) * width
            axis.bar(x, values.fillna(0.0), width, label=scenario)
            for position, ran, value in zip(x, rows.scenario.notna(), values):
                if ran and not np.isfinite(value):
                    axis.text(position, 0.0, "none done", rotation=90, ha="center", va="bottom", fontsize=7,
                              color="0.3")
        axis.set_xticks(range(len(templates)), [t.replace(", ", ",\n") for t in templates], fontsize=8)
        axis.set_ylabel(label)
        axis.grid(True, axis="y", alpha=0.35)
    figure.legend(*figure.axes[0].get_legend_handles_labels(), loc="outside lower center",
                  ncols=min(len(names), 6), fontsize=8)
    figure.suptitle("Error by template")
    return save(figure, out, "templates.png")


def _along_path(runs: pd.DataFrame, profiles: dict, out: Path) -> str:
    """Mean |position| and |heading| error along path progress: a row per scenario, a line per template."""
    done = runs[runs.success]
    if done.empty:
        return ""
    centers = (np.arange(PROGRESS_BINS) + 0.5) / PROGRESS_BINS
    names = [s for s in scenarios(runs) if s in set(done.scenario)]
    figure = Figure(figsize=(10, 2.6 * len(names) + 0.6), layout="constrained")
    axes = np.atleast_2d(figure.subplots(len(names), 2, sharex=True))
    for row, scenario in enumerate(names):
        for template, group in done[done.scenario == scenario].groupby("template"):
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", RuntimeWarning)  # a bin no run reached
                mean = np.nanmean(np.stack([profiles[name] for name in group.run]), axis=0)
            axes[row, 0].plot(centers, mean[:, 0] * 100, "-o", markersize=3, color=COLORS[template])
            axes[row, 1].plot(centers, mean[:, 1], "-o", markersize=3, color=COLORS[template])
        axes[row, 0].set_ylabel(f"{scenario}\nposition [cm]")
        axes[row, 1].set_ylabel("heading [°]")
    for axis in axes.ravel():
        axis.grid(True, alpha=0.35)
    for axis in axes[-1]:
        axis.set_xlabel("progress along the path")
    handles = [Line2D([], [], color=COLORS[t], label=t) for t in TEMPLATES if t in set(done.template)]
    figure.legend(handles=handles, loc="outside lower center", ncols=len(handles), fontsize=8)
    figure.suptitle("Error along the path")
    return save(figure, out, "along_path.png")


def _outcomes(runs: pd.DataFrame, out: Path) -> str:
    """Stacked bars: how runs ended, a group per template with the scenarios side by side."""
    if runs.empty:
        return ""
    counts = runs.groupby(["template", "scenario", "outcome"]).size().unstack(fill_value=0)
    order = [(t, s) for t in TEMPLATES for s in scenarios(runs) if (t, s) in counts.index]
    counts = counts.loc[order]
    # * A gap between templates, so the scenarios of one template read as one comparison.
    templates = [template for template, _ in order]
    x = np.arange(len(order)) + np.cumsum([i > 0 and t != templates[i - 1] for i, t in enumerate(templates)]) * 0.8
    figure = Figure(figsize=(max(7, 0.45 * len(counts) + 2), 4.5), layout="constrained")
    axis = figure.add_subplot()
    bottom = np.zeros(len(counts))
    for outcome in counts.columns:
        axis.bar(x, counts[outcome], bottom=bottom, label=outcome, color="tab:green" if outcome == "done" else None)
        bottom += counts[outcome].to_numpy()
    for template in dict.fromkeys(templates):
        middle = x[[t == template for t in templates]].mean()
        axis.text(middle, 1.01, template, transform=axis.get_xaxis_transform(), ha="center", va="bottom", fontsize=8)
    axis.set_xticks(x, [scenario for _, scenario in order], fontsize=7, rotation=60, ha="right")
    axis.yaxis.get_major_locator().set_params(integer=True)
    axis.set_ylabel("runs")
    axis.legend(fontsize=8)
    figure.suptitle("Outcomes")
    return save(figure, out, "outcomes.png")
