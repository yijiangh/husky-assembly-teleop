"""
The robot model report, robot_model.md: the real robot's model, whether one fits all runs, how well it predicts, how
close the simulator with it comes, and checks of the method.

It reads why, what was measured, then numbers only. Each robot's path runs are pooled over controllers, since the
response belongs to the robot; constant-command runs are not here (constant_commands.md).
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
from matplotlib.figure import Figure
from matplotlib.lines import Line2D

from ..auto import MODE, SPEED
from ..identify import SOURCES, identify
from ..record import CONDITIONS
from ..replay import HORIZON, IDEAL, Errors, Model, free_run, score, synthetic
from ..replay import errors as run_errors
from .common import (ERRORS, TEMPLATES, Experiment, markdown, parameters, robots, run_track, save, scenario_row,
                     scenarios, shared_paths, tracks)

#: The input → output that defines the model; the others split its delay through the wheels.
MOTION = next(iter(SOURCES))
#: What each kind of robot is for, and where the manual says how to collect it.
ROLES = {"real": ("the robot to model (sections 1 to 4)", "base_exp_manual.md A, step 1"),
         "sim ideal": ("checks the method: no offset, no delay (section 5)", "base_exp_manual.md A, step 4"),
         "sim real model": ("checks the method on the real robot's model, and stands in for it (sections 4, 5)",
                            "base_exp_manual.md A, step 3")}
#: How close an identified value must be to a known one: delay (s), efficiencies, xICR (m).
TOLERANCE = {"delay": 0.02, "speed_efficiency": 0.03, "steering_efficiency": 0.03, "x_icr": 0.01}
#: Largest median replay error, 1 s ahead, of a model that should predict exactly, metres.
EXACT = 0.005
#: The model's values: label, factor to the unit, decimals.
VALUES = {"delay": ("delay [ms]", 1000.0, 0), "speed_efficiency": ("speed efficiency", 1.0, 2),
          "steering_efficiency": ("steering efficiency", 1.0, 2), "x_icr": ("xICR [cm]", 100.0, 1)}
#: Motion models replayed, in table order, with their plot colours.
MODEL_COLORS = {"ideal": "0.5", "identified": "tab:red", "sim settings": "tab:blue"}
#: Prediction columns, all lower is better.
PREDICTION = ("position median [cm]", "position p95 [cm]", "heading median [°]", "heading p95 [°]")
#: Runs overlaid in model_replay.png, at most.
REPLAYED_RUNS = 16


def write(experiment: Experiment) -> None:
    """Write robot_model.md, its CSV files and plots into the experiment's analysis folder (path runs only)."""
    runs, out = experiment.runs, experiment.out
    names = robots(runs)
    found = {(robot, source): identify(tracks(runs[runs.robot == robot]), source)
             for robot in names for source in SOURCES}
    found = {key: value for key, value in found.items() if value is not None}
    known = {robot: _settings(runs, robot) for robot in names}
    replays = replay_errors(runs, known)
    single = per_run(runs)
    twin = _twin(found.get(("real", MOTION)), known)
    noise = twin or next((r for r in names if known[r] is not None and r != "sim ideal"), None)
    real = runs[runs.robot == "real"]
    tables = {"robots": robots_table(runs, known, twin), "model": model_table(found),
              "model_stability": stability_table(real, single, found, noise),
              "model_by_template": template_table(real), "prediction": prediction_table(replays, "real"),
              "prediction_template": prediction_template_table(replays, "real"),
              "sim_vs_real_open": open_loop_table(replays, twin), "sim_vs_real_closed": closed_loop_table(runs, twin),
              "checks": checks_table(runs, found, known, replays), "wheels": wheel_table(found)}
    single.to_csv(out / "model_runs.csv", index=False)
    for name, table in tables.items():
        table.to_csv(out / f"{name}.csv", index=False)
    plots = [p for p in (_stability(single, found, out), _horizon(replays, out), _replay(runs, found, out)) if p]

    tick = any(response.clock == "tick" for response in found.values())
    fixed = ", ".join([f"{SPEED[0]} m/s and {SPEED[1]:.0f}°/s", MODE.lower()]
                      + [f"{key} {value}" for key, value in CONDITIONS.items()])
    tolerance = ", ".join(f"{_short(key)} {value * VALUES[key][1]:g}{_unit(key)}" for key, value in TOLERANCE.items())
    lines = [f"# Robot model: {experiment.name}\n",
             experiment.intro() + "\n",
             "## Why\n",
             "A controller steers better when it knows how the robot responds to its commands. This report answers:\n",
             "1. What are the real robot's model values? (section 1)",
             "2. Does one model fit every run: over the runs, over time, over the kinds of path? (section 2)",
             "3. Does it predict the real robot better than the ideal model? (section 3)",
             "4. Does the simulator with that model drive like the real robot? (section 4)",
             "5. Is the method right? In a simulator the model is known, so identification must give it back. "
             "(section 5)\n",
             "## What we measured\n",
             f"Held fixed: {fixed}.\n",
             "- **The model**, how the robot responds to a command (v, ω): **delay** from command to motion; "
             "**speed efficiency**, forward speed per commanded speed; **steering efficiency**, turn rate per "
             "commanded turn rate; **xICR**, how far the tracked point lies ahead of the point the robot turns about "
             "(negative: behind).",
             "- **Identification** fits the four values to the commands sent and the motion mocap measured, over all "
             "path runs of one robot with every controller. It holds for the commands those runs sent (table below)."
             + (" Clock \"tick\": some runs had no stamps, so the delay includes the latencies." if tick else ""),
             f"- **Prediction**: every 0.2 s along a run, the model starts at the measured pose, is driven by the "
             f"commands sent, and is compared with mocap {HORIZON:g} s later. *ideal*: what a plain controller "
             "assumes; *identified*: fitted on the other half of the robot's runs, so no run grades its own fit. "
             "Median: a typical prediction; p95: the worst 5 %. Bold marks the better model on the same starts.",
             f"- **Tolerance**: values closer than {tolerance} count as the same.\n",
             "Robots: the real one, or the simulator with a model (\"sim ideal\", \"sim <model name>\").\n",
             markdown(tables["robots"]), "",
             "## 1. The model\n",
             "The real robot's values, fitted on all its runs. R²: the share of the forward speed, sideways speed and "
             "turn rate the model explains.\n",
             markdown(tables["model"]), "",
             "## 2. One model for every run?\n",
             "The real robot, each run fitted on its own (empty where a run does not excite a value, e.g. steering on "
             "a straight): median, spread (standard deviation) and n. *spread in sim*: the same in a simulator, whose "
             "model is fixed: the method's own noise. *first / second half*: the runs in time order, fitted in two "
             "halves.\n",
             markdown(tables["model_stability"]), "",
             "Per template, all its runs fitted together:\n",
             markdown(tables["model_by_template"]), "",
             f"## 3. Prediction ({HORIZON:g} s ahead)\n",
             "The real robot, every run:\n",
             markdown(tables["prediction"], lower=PREDICTION), "",
             "By the template of the run:\n",
             markdown(tables["prediction_template"], "template", lower=PREDICTION), "",
             "## 4. Simulator against real\n",
             "The simulator running the real robot's model (within the tolerance) against the real robot. Open loop: "
             f"how far each departs from the ideal model, {HORIZON:g} s ahead. Closed loop: the same controller on "
             "both, on the paths both finished, each path once; mean ± spread between repeats of one path. No bold: "
             "the question is whether they are close.\n"]
    if twin is None:
        lines += ["(no simulator runs the real robot's model)\n"]
    else:
        lines += [markdown(tables["sim_vs_real_open"]), "",
                  markdown(tables["sim_vs_real_closed"], "template"), ""]
    lines += ["## 5. Checks\n",
              "Where the answer is known: in the simulators, and in synthetic runs (each run's commands replayed "
              "through the identified model). A failed check means the recording or the method is off, not the "
              "robot.\n",
              markdown(tables["checks"]), ""]
    if not tables["wheels"].empty:
        lines += ["## 6. Timing through the wheels\n",
                  "The real robot's delay split at the wheel odometry: commands → wheels and wheels → motion. The "
                  "wheels → motion efficiencies: how much of the wheel motion becomes motion of the robot. ! A delay "
                  "below zero is not physical: the odometry stamps or the clocks are off, and commands → wheels "
                  "reads that much too long.\n",
                  markdown(tables["wheels"]), ""]
    lines += ["## 7. The model for crl_husky\n", *_model_lines(found.get(("real", MOTION))), "",
              "## Plots\n",
              "model_stability.png: each run fitted on its own, in time order, coloured by template; the line is the "
              "fit of all runs. model_horizon.png: the median prediction error over how far ahead it predicts. "
              "model_replay.png: whole real runs replayed from their start, never corrected.\n",
              *[f"![{name}]({name})\n" for name in plots]]
    (out / "robot_model.md").write_text("\n".join(lines))


# --- --- --- --- --- TABLES --- --- --- --- ---

def robots_table(runs: pd.DataFrame, known: dict, twin: str | None) -> pd.DataFrame:
    """One row per robot: how it moves, its controllers and runs, the commands sent, and what it is for.

    Kinds of robot the report needs but has no runs of get a row saying how to collect them.
    """
    rows, present = [], set()
    for robot in robots(runs):
        mine = runs[runs.robot == robot]
        role = robot if robot in ("real", "sim ideal") else "sim real model" if robot == twin else None
        present.add(role)
        sent = np.vstack([t.command for t in tracks(mine)] or [np.zeros((1, 2))])
        controllers = [s.split(" · ", 1)[1] for s in scenarios(runs) if s in set(mine.scenario)]
        rows.append({"robot": robot, "moves as": "the real robot" if known[robot] is None else _text(known[robot]),
                     "controllers": ", ".join(controllers), "runs": len(mine),
                     "commands sent": f"v up to ±{np.abs(sent[:, 0]).max():.2f} m/s, "
                                      f"ω up to ±{math.degrees(np.abs(sent[:, 1]).max()):.0f}°/s",
                     "used for": ROLES[role][0] if role else "simulator with another model (section 5)"})
    for role, (purpose, how) in ROLES.items():
        if role not in present:
            rows.append({"robot": f"**missing**: {role}", "moves as": "", "controllers": "", "runs": 0,
                         "commands sent": "", "used for": f"{purpose}; collect: {how}"})
    return pd.DataFrame(rows)


def model_table(found: dict) -> pd.DataFrame:
    """The real robot's identified commands → motion response, with its fit."""
    response = found.get(("real", MOTION))
    if response is None:
        return pd.DataFrame()
    return pd.DataFrame([{"robot": "real", **_columns(_values(response)),
                          "fit R² forward / sideways / turn": _fit(response)}])


def wheel_table(found: dict) -> pd.DataFrame:
    """The real robot's response through the wheels: commands → wheels and wheels → motion."""
    return pd.DataFrame([{"input → output": source, **_columns(_values(response)), "fit R²": _fit(response)}
                         for (robot, source), response in found.items() if robot == "real" and source != MOTION])


def replay_errors(runs: pd.DataFrame, known: dict) -> dict[tuple[str, str], list[tuple[str, Errors]]]:
    """Per (robot, model), each run's template and replay errors. Models: ideal, identified (cross-validated), sim
    settings."""
    replays = {}
    for robot in robots(runs):
        pairs = [(row.template, track) for row in runs[runs.robot == robot].itertuples()
                 if (track := run_track(row.folder)) is not None]
        if not pairs:
            continue
        replays[robot, "ideal"] = [(template, run_errors(track, IDEAL)) for template, track in pairs]
        crossed = _crossed(pairs)
        if crossed is not None:
            replays[robot, "identified"] = crossed
        if known[robot] is not None:
            model = _model(known[robot])
            replays[robot, "sim settings"] = [(template, run_errors(track, model)) for template, track in pairs]
    return replays


def _crossed(pairs: list) -> list[tuple[str, Errors]] | None:
    """Each run's errors under the model identified on the other half of the runs (alternate runs); None if a half
    is empty or identifies nothing."""
    halves = (pairs[0::2], pairs[1::2])
    if not all(halves):
        return None
    out = []
    for scored, fitted in (halves, halves[::-1]):
        found = identify([track for _, track in fitted])
        if found is None:
            return None
        out += [(template, run_errors(track, found.model)) for template, track in scored]
    return out


def _joined(parts: list[tuple[str, Errors]], template: str | None = None) -> Errors:
    """The errors of all runs, or of one template's."""
    return Errors.join([e for t, e in parts if template is None or t == template])


def _prediction(errors: Errors) -> dict:
    """Median and 95th percentile of position and heading error HORIZON seconds ahead, and the number of starts."""
    position, heading = errors.at(HORIZON)
    position, heading = position * 100, np.degrees(heading)
    return dict(zip(PREDICTION, (_stat(np.median, position), _p95(position), _stat(np.median, heading),
                                 _p95(heading)))) | {"starts": len(position)}


def prediction_table(replays: dict, robot: str) -> pd.DataFrame:
    """One robot, per model: prediction errors over every run."""
    return pd.DataFrame([{"model": model, **_prediction(_joined(parts))}
                         for (name, model), parts in replays.items() if name == robot and model != "sim settings"])


def prediction_template_table(replays: dict, robot: str) -> pd.DataFrame:
    """One robot, per template and model: prediction errors over that template's runs."""
    rows = []
    for template in TEMPLATES:
        for (name, model), parts in replays.items():
            errors = _joined(parts, template)
            if name == robot and model != "sim settings" and len(errors.kinds):
                rows.append({"template": template, "model": model, **_prediction(errors)})
    return pd.DataFrame(rows)


def open_loop_table(replays: dict, twin: str | None) -> pd.DataFrame:
    """The ideal model's prediction errors on the simulator running the real robot's model, and on the real robot."""
    if twin is None:
        return pd.DataFrame()
    return pd.DataFrame([{"robot": robot, "model": "ideal", **_prediction(_joined(replays[robot, "ideal"]))}
                         for robot in (twin, "real") if (robot, "ideal") in replays])


def closed_loop_table(runs: pd.DataFrame, twin: str | None) -> pd.DataFrame:
    """Per controller run on both the twin simulator and the real robot, per template: success and errors on the
    paths both finished."""
    if twin is None:
        return pd.DataFrame()
    rows = []
    first = runs.drop_duplicates("scenario").set_index("scenario")
    for controller in dict.fromkeys(first.controller_label[s] for s in scenarios(runs)):
        pair = [s for s in scenarios(runs) if first.robot[s] in (twin, "real") and first.controller_label[s] ==
                controller]
        if len(pair) < 2:
            continue
        for template in ("all standard paths", *TEMPLATES):
            group = runs[runs.standard >= 0] if template == "all standard paths" else runs[runs.template == template]
            group = group[group.scenario.isin(pair)]
            if group.empty:
                continue
            shared = shared_paths(group, pair)
            for scenario in pair:
                rows.append({"template": template, "scenario": scenario,
                             **scenario_row(group[group.scenario == scenario], shared)})
    columns = ["template", "scenario", "attempts", "done", "success [%]", "paths compared", *ERRORS]
    return pd.DataFrame(rows).reindex(columns=columns) if rows else pd.DataFrame()


def checks_table(runs: pd.DataFrame, found: dict, known: dict, replays: dict) -> pd.DataFrame:
    """The checks where the answer is known: per robot, expected against measured, and whether it holds."""
    rows = []

    def check(name, robot, expected, result, ok):
        rows.append({"check": name, "robot": robot, "expected": expected, "result": result,
                     "ok": "yes" if ok else "**no**"})

    for robot in robots(runs):
        settings, response = known[robot], found.get((robot, MOTION))
        if settings is not None and response is not None:
            check("identification gives the simulator's model back", robot, _text(settings),
                  _text(_values(response)), _close(_values(response), settings))
        if (robot, "sim settings") in replays:
            median = float(np.median(_joined(replays[robot, "sim settings"]).at(HORIZON)[0]))
            check(f"the simulator's model predicts its runs ({HORIZON:g} s)", robot,
                  f"≤ {EXACT * 100:.1f} cm", f"{median * 100:.2f} cm", median <= EXACT)
        if response is not None:
            # * Each run's own commands drive the identified model: identifying and replaying those must be exact.
            made = [synthetic(t, response.model) for t in tracks(runs[runs.robot == robot])]
            recovered = identify(made)
            exact = float(np.median(score(made, response.model).at(HORIZON)[0])) if made else math.nan
            check("synthetic runs give their model back", robot, _text(_values(response)),
                  f"{_text(_values(recovered))}; replay {exact * 100:.2f} cm" if recovered else "nothing identified",
                  recovered is not None and _close(_values(recovered), _values(response)) and exact <= EXACT)
    return pd.DataFrame(rows)


def per_run(runs: pd.DataFrame) -> pd.DataFrame:
    """Each run fitted on its own: robot, scenario, template, start time, and the four values (NaN if not excited)."""
    rows = []
    for _, run in runs.iterrows():
        measured = run_track(run.folder)
        response = identify([measured]) if measured is not None else None
        rows.append({"robot": run.robot, "scenario": run.scenario, "run": run.run, "template": run.template,
                     "started": run.started,
                     **(_values(response) if response is not None else {key: math.nan for key in VALUES})})
    return pd.DataFrame(rows).sort_values(["robot", "started", "run"], kind="stable")


def stability_table(runs: pd.DataFrame, single: pd.DataFrame, found: dict, noise: str | None) -> pd.DataFrame:
    """The real robot per value: all runs together, run by run (median, spread, n), the spread in the `noise`
    simulator, and by halves in time."""
    if runs.empty:
        return pd.DataFrame()
    response = found.get(("real", MOTION))
    mine = runs.sort_values(["started", "run"], kind="stable")
    middle = len(mine) // 2
    halves = [identify(tracks(mine.iloc[:middle])), identify(tracks(mine.iloc[middle:]))] if middle else [None] * 2
    fitted, reference = single[single.robot == "real"], single[single.robot == noise]
    rows = []
    for key, (label, factor, digits) in VALUES.items():
        def show(value):
            return f"{value * factor:.{digits}f}" if value is not None and math.isfinite(value) else "—"

        values = fitted[key].dropna()
        rows.append({"value": label, "all runs": show(getattr(response, key) if response else None),
                     "run by run: median": show(values.median() if len(values) else None),
                     "spread": show(_spread(values)), "n": len(values),
                     f"spread in {noise or 'sim'}": show(_spread(reference[key])),
                     "first half": show(getattr(halves[0], key) if halves[0] else None),
                     "second half": show(getattr(halves[1], key) if halves[1] else None)})
    return pd.DataFrame(rows)


def template_table(runs: pd.DataFrame) -> pd.DataFrame:
    """Per template, in the usual order: its runs fitted together."""
    rows = []
    for template in TEMPLATES:
        group = runs[runs.template == template]
        response = identify(tracks(group)) if len(group) else None
        if response is not None:
            rows.append({"template": template, "runs": len(group), **_columns(_values(response))})
    return pd.DataFrame(rows)


def _twin(real, known: dict) -> str | None:
    """The simulator that runs the real robot's identified model within the tolerance; None if none does."""
    if real is None:
        return None
    return next((robot for robot, settings in known.items() if settings and _close(_values(real), settings)), None)


def _short(key: str) -> str:
    """A value's name without its unit, e.g. "delay"."""
    return VALUES[key][0].split(" [")[0]


def _unit(key: str) -> str:
    """A value's unit with a leading space, e.g. " ms"; "" for a ratio."""
    label = VALUES[key][0]
    return f" {label.split('[')[1].rstrip(']')}" if "[" in label else ""


def _spread(values: pd.Series) -> float:
    """Standard deviation of the finite values; NaN with fewer than two."""
    values = values.dropna()
    return float(values.std()) if len(values) > 1 else math.nan


def _settings(runs: pd.DataFrame, robot: str) -> dict | None:
    """The model a simulator ran with, as identified values (`_values`); None for the real robot."""
    settings = json.loads(runs.loc[runs.robot == robot, "sim_settings"].iloc[0])
    if settings is None:
        return None
    return {"delay": settings["cmd_delay_sec"], "speed_efficiency": settings["speed_efficiency"],
            "steering_efficiency": settings["steering_efficiency"], "x_icr": settings["xICR"]}


def _values(response) -> dict[str, float]:
    """An identified response's values, by VALUES key."""
    return {key: getattr(response, key) for key in VALUES}


def _close(found: dict, expected: dict) -> bool:
    """Whether every known value lies within TOLERANCE of the expected one."""
    return all(math.isnan(found[key]) or abs(found[key] - expected[key]) <= TOLERANCE[key] for key in TOLERANCE)


def _model(values: dict) -> Model:
    """A replay model from values (`_values`)."""
    return Model.of(values["x_icr"], values["speed_efficiency"], values["steering_efficiency"], values["delay"])


def _columns(values: dict) -> dict:
    """Values as table columns, each with its unit's decimals; "—" where not identified."""
    return {label: f"{values[key] * factor:.{digits}f}" if math.isfinite(values[key]) else "—"
            for key, (label, factor, digits) in VALUES.items()}


def _text(values: dict) -> str:
    """Values in one line, e.g. "220 ms, 0.88, 1.01, -12.9 cm" (delay, speed, steering, xICR)."""
    delay, speed, steering, x_icr = _columns(values).values()
    return f"{delay} ms, {speed}, {steering}, {x_icr} cm"


def _fit(response) -> str:
    """Share of the variance explained per channel, e.g. "0.90 / 0.90 / 0.96"."""
    return " / ".join("—" if math.isnan(response.fit[k]) else f"{response.fit[k]:.2f}"
                      for k in ("forward", "sideways", "turn"))


def _model_lines(real) -> list[str]:
    """The real robot's identified response as a robot model for crl_husky, and as simulator launch values."""
    if real is None:
        return ["No real runs: nothing to model."]
    values = _values(real)

    def number(value, factor=1.0, ideal="0"):
        return ideal if math.isnan(value) else f"{value * factor:.3g}"

    model = {"xICR": number(values["x_icr"]), "speed_efficiency": number(values["speed_efficiency"], ideal="1"),
             "steering_efficiency": number(values["steering_efficiency"], ideal="1"),
             "cmd_delay_sec": number(values["delay"])}
    return ["Save it as crl_husky `config/base_models/<name>.yaml` and rebuild; then `sim_model:=<name>` runs the "
            "simulator with it, `controller_model:=<name>` (sim) or `model:=<name>` (robot) gives it to the "
            "follower.\n",
            "```yaml\n" + "".join(f"{key}: {value}\n" for key, value in model.items()) + "```\n",
            "Or as single simulator values:\n",
            "```\nros2 launch crl_husky pure_pursuit_sim.launch.py ... "
            + " ".join(f"{key}:={value}" for key, value in model.items()) + "\n```"]


def _stat(function, values: np.ndarray) -> float:
    """`function` of the values, one decimal; NaN when empty."""
    return _round(function(values), 1) if len(values) else math.nan


def _p95(values: np.ndarray) -> float:
    """95th percentile, one decimal; NaN when empty."""
    return _round(np.percentile(values, 95), 1) if len(values) else math.nan


def _round(value: float, digits: int) -> float:
    """Rounded to `digits`; NaN stays NaN."""
    return math.nan if math.isnan(value) else round(float(value), digits)


# --- --- --- --- --- PLOTS --- --- --- --- ---

def _stability(single: pd.DataFrame, found: dict, out: Path) -> str:
    """Each run's own fit in time order, a row per robot and a column per value, coloured by template.

    The line is the fit of all the robot's runs together: points scattered evenly around it mean one model; groups by
    colour, or a trend along the runs, mean several.
    """
    names = [robot for robot in dict.fromkeys(single.robot)
             if single.loc[single.robot == robot, list(VALUES)].notna().any().any()]
    if not names:
        return ""
    templates = [t for t in TEMPLATES if t in set(single.template)]
    colors = dict(zip(templates, (f"C{i}" for i in range(len(templates)))))
    figure = Figure(figsize=(14, 2.6 * len(names) + 0.8), layout="constrained")
    # * One scale per value across robots, so a simulator's spread (the method's noise) compares with the real one.
    axes = np.array(figure.subplots(len(names), len(VALUES), squeeze=False, sharey="col"))
    for row, robot in enumerate(names):
        mine = single[single.robot == robot].reset_index(drop=True)
        response = found.get((robot, MOTION))
        for column, (key, (label, factor, _)) in enumerate(VALUES.items()):
            axis = axes[row, column]
            for template, group in mine.groupby("template", sort=False):
                axis.plot(group.index, group[key] * factor, "o", markersize=4, color=colors[template])
            if response is not None and math.isfinite(getattr(response, key)):
                axis.axhline(getattr(response, key) * factor, color="0.3", linewidth=1.0)
            axis.set_title(label, fontsize=9)
            axis.grid(True, alpha=0.35)
        axes[row, 0].set_ylabel(robot)
    for axis in axes[-1]:
        axis.set_xlabel("run (time order)")
    handles = [Line2D([], [], color=colors[t], marker="o", linestyle="", label=t) for t in templates]
    handles.append(Line2D([], [], color="0.3", label="all runs together"))
    figure.legend(handles=handles, loc="outside lower center", ncols=min(len(handles), 6), fontsize=8)
    figure.suptitle("Each run fitted on its own")
    return save(figure, out, "model_stability.png")


def _horizon(replays: dict, out: Path) -> str:
    """Median replay error against how far ahead it predicts: a column per robot, a line per model."""
    if not replays:
        return ""
    names = list(dict.fromkeys(robot for robot, _ in replays))
    figure = Figure(figsize=(3.4 * len(names) + 0.6, 5.2), layout="constrained")
    axes = np.array(figure.subplots(2, len(names), sharex=True, squeeze=False))
    for column, robot in enumerate(names):
        for (name, model), parts in replays.items():
            errors = _joined(parts)
            if name != robot or not len(errors.kinds):
                continue
            horizons = [0.0, *errors.horizons]
            axes[0, column].plot(horizons, [0.0, *np.median(errors.position, axis=0) * 100], "-o", markersize=3,
                                 color=MODEL_COLORS[model])
            axes[1, column].plot(horizons, [0.0, *np.degrees(np.median(errors.heading, axis=0))], "-o", markersize=3,
                                 color=MODEL_COLORS[model])
        axes[0, column].set_title(robot, fontsize=10)
        axes[1, column].set_xlabel("seconds ahead")
    axes[0, 0].set_ylabel("position [cm]")
    axes[1, 0].set_ylabel("heading [°]")
    for axis in axes.ravel():
        axis.grid(True, alpha=0.35)
    handles = [Line2D([], [], color=color, marker="o", markersize=3, label=model)
               for model, color in MODEL_COLORS.items() if any(m == model for _, m in replays)]
    figure.legend(handles=handles, loc="outside lower center", ncols=len(handles), fontsize=8)
    figure.suptitle("Replay error (median)")
    return save(figure, out, "model_horizon.png")


def _replay(runs: pd.DataFrame, found: dict, out: Path) -> str:
    """Whole runs of the real robot (else the last robot) replayed from their start, never corrected.

    With the ideal and the identified model (fitted on all the robot's runs), failed runs too. The measured path is
    drawn wide underneath; a dot marks where each ends (a replay that does not move, e.g. the ideal model on a turn on
    the spot, is only a dot).
    """
    names = robots(runs)
    robot = "real" if "real" in names else names[-1]
    chosen = runs[runs.robot == robot].sort_values(["template", "run"]).head(REPLAYED_RUNS)
    if chosen.empty:
        return ""
    response = found.get((robot, MOTION))
    models = {"ideal": IDEAL, **({"identified": response.model} if response is not None else {})}
    columns = min(len(chosen), 4)
    rows = math.ceil(len(chosen) / columns)
    figure = Figure(figsize=(4 * columns, 4 * rows + 0.6), layout="constrained")
    axes = np.array(figure.subplots(rows, columns, squeeze=False)).ravel()
    for axis, (_, run) in zip(axes, chosen.iterrows()):
        measured = run_track(run.folder)
        if measured is None:
            continue
        axis.plot(*measured.pose[:, :2].T, color="0.75", linewidth=4.0, solid_capstyle="round")
        axis.plot(*measured.pose[-1, :2], "o", color="0.4", markersize=5)
        for model, motion in models.items():
            _, poses = free_run(measured, motion)
            axis.plot(*poses[:, :2].T, "--", color=MODEL_COLORS[model], linewidth=1.3)
            axis.plot(*poses[-1, :2], "o", color=MODEL_COLORS[model], markersize=4)
        axis.set_aspect("equal", adjustable="datalim")
        outcome = "" if run.success else f" ({run.outcome})"
        axis.set_title(f"{run.template}{outcome}\n{parameters(run)}", fontsize=8)
        axis.grid(True, alpha=0.35)
    for axis in axes[len(chosen):]:
        axis.set_visible(False)
    handles = [Line2D([], [], color="0.75", linewidth=4.0, label="measured")] + [
        Line2D([], [], color=MODEL_COLORS[m], linestyle="--", label=f"{m} model") for m in models]
    figure.legend(handles=handles, loc="outside lower center", ncols=len(handles), fontsize=8)
    figure.suptitle(f"Replay from the start, uncorrected: {robot}")
    return save(figure, out, "model_replay.png")
