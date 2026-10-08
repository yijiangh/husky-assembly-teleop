"""
What the reports share: the runs of an experiment, loaded once, with their robot and scenario labels; Markdown tables.

- A run drove a **path** (standard or random paths, with the follower) or one **constant command** (`twist.py`).
- Its **robot** is "real", "sim ideal", "sim <model name>" (or "sim model A", B, ... for unnamed models).
- Its **scenario** is "<robot> · <controller and the model it assumes>", e.g. "sim alice_tiles · PP ideal".
- Only path runs at the fixed speed and mode, and runs under the fixed conditions, are loaded; the others are counted
  in `left_out`. So are e-stopped runs, and runs listed in the experiment folder's EXCLUDED file, which stay on disk.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, replace
from functools import lru_cache
from pathlib import Path

import numpy as np
import pandas as pd
from matplotlib.figure import Figure

from ..auto import GRID, MODE, SPEED, cell_key, standard_set
from ..record import CONDITIONS, MODEL_PARAMETERS, environment_label, setup_key, sim_model
from ..replay import Track, track
from ..report import FOLLOWING, load, metrics
from ..twist import cell_of

#: Robots in table order, with their short label.
ENVIRONMENTS = {"sim ideal": "sim ideal", "sim model": "sim", "real": "real"}
#: Short controller labels; others keep their node name.
CONTROLLERS = {"pure_pursuit": "PP", "husky_mpc": "MPC", "mpc": "MPC"}
#: A controller's robot-model parameters, with the ideal values (crl_husky PurePursuitParams).
CONTROLLER_MODEL = {"x_icr": 0.0, "speed_efficiency": 1.0, "steering_efficiency": 1.0, "cmd_delay_sec": 0.0}
TEMPLATES = list(GRID)
#: Run numbers: (column, label with unit, factor to that unit).
NUMBERS = (("end_position", "end position [cm]", 100.0), ("end_heading", "end heading [°]", 1.0),
           ("motion_position", "moving position [cm]", 100.0), ("motion_heading", "moving heading [°]", 1.0))
ERRORS = tuple(label for _, label, _ in NUMBERS)
PROGRESS_BINS = 20
#: Short labels of the sim model's parameters, with the factor to their unit.
MODEL_LABELS = {"xICR": ("xICR", 1.0, "m"), "speed_efficiency": ("speed efficiency", 1.0, ""),
                "steering_efficiency": ("steering efficiency", 1.0, ""),
                "cmd_delay_sec": ("command delay", 1000.0, "ms"), "mocap_delay_sec": ("mocap delay", 1000.0, "ms"),
                "mocap_noise_position_m": ("mocap noise", 1000.0, "mm"),
                "mocap_noise_yaw_rad": ("mocap yaw noise", 180.0 / math.pi, "°")}
#: Runs to leave out, in the experiment's folder: one run folder name per line, "# reason" after it.
EXCLUDED = "_excluded.txt"
#: What a run drove: a path with the follower (standard or random paths), or one constant command (`twist.py`).
PATHS, COMMANDS = "path", "constant-command"
#: The standard paths, by `cell_key`, with their position in the set.
STANDARD = {cell_key(settings): i for i, settings in enumerate(standard_set())}


@dataclass
class Experiment:
    """The loaded runs of one experiment (or of all).

    Attributes:
        name: The experiment name, or "all".
        runs: One row per run (see `load_runs`).
        profiles: Per run folder name, mean |position| (m) and |heading| (°) error in PROGRESS_BINS bins of progress.
        left_out: What was not loaded, e.g. "2 at another speed or mode", or "none".
        out: The folder the reports go to.
    """

    name: str
    runs: pd.DataFrame
    profiles: dict
    left_out: str
    out: Path

    def only(self, kind: str) -> Experiment:
        """The same experiment with only the runs of one kind (PATHS or COMMANDS)."""
        return replace(self, runs=self.runs[self.runs.kind == kind])

    def intro(self) -> str:
        """One line: runs of each kind, dates, how many done, what was left out."""
        started = self.runs.started[self.runs.started != ""]
        span = f"{started.min()[:16]} to {started.max()[:16]}" if len(started) else "unknown dates"
        kinds = self.runs.kind.value_counts()
        count = " and ".join(f"{n} {kind} runs" for kind, n in kinds.items())
        return (f"Experiment `{self.name}`: {count} ({span}), {int(self.runs.success.sum())} done. "
                f"Left out: {self.left_out}.")


def load_runs(recordings: Path, experiment: str | None) -> tuple[pd.DataFrame, dict, str]:
    """The runs that follow the protocol, one row each, and per run its errors in bins of path progress.

    Returns:
        tuple[pd.DataFrame, dict, str]: The runs; their profiles (see `Experiment`); what was left out.
    """
    rows, profiles = [], {}
    skipped = {"e-stopped": 0, f"listed in {EXCLUDED}": 0, "without the monitor's tracking": 0,
               "at another speed or mode": 0, "under other conditions": 0}
    for data in sorted(recordings.glob("**/experiment.json")):
        if any(part.startswith("_") for part in data.relative_to(recordings).parts):
            continue  # _analysis, _archive
        description = json.loads(data.read_text())
        if experiment is not None and description.get("experiment_name") != experiment:
            continue
        # * The robot stood while the follower still commanded: neither the controller's result nor the robot's.
        if description.get("outcome", "").startswith("e-stopped"):
            skipped["e-stopped"] += 1
            continue
        if data.parent.name in _excluded(data.parent.parent):
            skipped[f"listed in {EXCLUDED}"] += 1
            continue
        settings = description.get("template")
        # * Constant-command runs follow no path: no tracking errors, and their own speeds.
        commands = settings is not None and cell_of(settings) is not None
        run = load(data.with_name("recording.npz"))
        if settings is None or (not commands and "tracking_position" not in run):
            skipped["without the monitor's tracking"] += 1
            continue
        if not commands and (settings["mode"] != MODE
                             or not math.isclose(settings["linear_speed"], SPEED[0], abs_tol=1e-6)):
            skipped["at another speed or mode"] += 1
            continue
        if description.get("conditions", CONDITIONS) != CONDITIONS:
            skipped["under other conditions"] += 1
            continue
        row = {
            "kind": COMMANDS if commands else PATHS, "run": data.parent.name, "folder": str(data.parent),
            "started": description.get("started", ""),
            "environment": environment_label(description), "model": _model_text(description),
            "sim_model_name": (description.get("sim_model") or {}).get("model") or "",
            "sim_settings": json.dumps(sim_model(description)),
            "controller": description.get("controller", {}).get("name", "?"),
            "controller_label": _controller_label(description),
            "controller_model": _controller_model_text(description), "key": str(setup_key(description)),
            "template": settings["name"], "standard": STANDARD.get(cell_key(settings), -1),
            "outcome": description["outcome"].split(" →")[0].split(":")[0],
            "reason": description["outcome"].split(" →")[0],
            "settled": description.get("settled", False),
            "battery": (description.get("battery") or {}).get("percentage", math.nan),
            **{f"p_{k}": v for k, v in settings["parameters"].items()},
        }
        if not commands:
            numbers = metrics(run)
            row.update({k: v for k, v in numbers.__dict__.items() if k != "segments"})
            row["duration_ratio"] = row["duration"] / row["expected"] if row["expected"] else math.nan
            for kind, values in numbers.segments.items():
                for key, value in values.items():
                    row[f"{slug(kind)}_{key}"] = value
            profiles[row["run"]] = _profile(run)
        row["success"] = row["outcome"] == "done"
        # * Stopped by the operator says nothing about the controller: left out of the success rate.
        row["counted"] = row["outcome"] != "stopped"
        rows.append(row)
    runs = pd.DataFrame(rows)
    if not runs.empty:
        runs["robot"] = _robot_labels(runs)
        runs["scenario"] = _scenario_labels(runs)
        runs["path"] = [f"{row.template} {parameters(row)}" if row.kind == PATHS else ""
                        for _, row in runs.iterrows()]
    left_out = ", ".join(f"{n} {why}" for why, n in skipped.items() if n) or "none"
    return runs, profiles, left_out


@lru_cache(maxsize=None)
def _excluded(folder: Path) -> frozenset[str]:
    """The run folder names listed in `folder`'s EXCLUDED file; none without one."""
    listing = folder / EXCLUDED
    if not listing.is_file():
        return frozenset()
    names = (line.split("#")[0].strip() for line in listing.read_text().splitlines())
    return frozenset(name for name in names if name)


def scenarios(runs: pd.DataFrame) -> list[str]:
    """The scenario labels in table order: ideal sim, sims with models, real; per robot the "ideal" controller first."""
    keys = runs[["scenario", "environment", "robot", "controller_label"]].drop_duplicates()
    keys = keys.assign(rank=keys.environment.map({name: i for i, name in enumerate(ENVIRONMENTS)}),
                       baseline=~keys.controller_label.str.endswith(" ideal"))
    return list(keys.sort_values(["rank", "robot", "baseline", "controller_label"]).scenario)


def robots(runs: pd.DataFrame) -> list[str]:
    """The robot labels in table order: ideal sim, sims with models, real."""
    return list(dict.fromkeys(runs.set_index("scenario").loc[scenarios(runs), "robot"]))


def parameters(row: pd.Series) -> str:
    """The template's numbers of one run, e.g. "length=1, angle=90"."""
    return ", ".join(f"{column[2:]}={row[column]:g}" for column in row.index
                     if column.startswith("p_") and pd.notna(row[column]))


@lru_cache(maxsize=None)
def run_track(folder: str) -> Track | None:
    """One run folder's recording as a replay Track (loaded once)."""
    return track(load(Path(folder) / "recording.npz"))


def tracks(runs: pd.DataFrame) -> list[Track]:
    """The runs as replay Tracks, in run order; runs with too few poses left out."""
    found = [run_track(folder) for folder in runs.folder]
    return [t for t in found if t is not None]


def shared_paths(runs: pd.DataFrame, names: list[str]) -> set[str]:
    """The paths every one of the scenarios `names` finished at least once."""
    done = runs[runs.success]
    return set.intersection(*(set(done.path[done.scenario == name]) for name in names)) if names else set()


def scenario_row(runs: pd.DataFrame, paths: set[str]) -> dict:
    """One scenario's runs: attempts, done, success (each path once) and the errors on `paths` as mean ± spread.

    Runs stopped by the operator count neither way.
    """
    counted = runs[runs.counted]
    rates = counted.groupby("path").success.mean()
    row = {"attempts": len(counted), "done": int(counted.success.sum()),
           "success [%]": round(100 * rates.mean()) if len(rates) else math.nan, "paths compared": len(paths)}
    done = counted[counted.success & counted.path.isin(paths)]
    return row | {label: mean_spread(done, column, factor) for column, label, factor in NUMBERS}


def mean_spread(done: pd.DataFrame, column: str, factor: float) -> str:
    """Mean ± spread: the mean of the path means (each path once), and how much repeats of one path differ.

    The spread is the standard deviation between runs of the same path, pooled over the paths driven more than once;
    over all runs it would mostly show that paths differ (a spot turn against a straight).
    """
    if done.empty:
        return "—"
    by_path = done.groupby("path")[column]
    mean = by_path.mean().mean() * factor
    variances = by_path.var().dropna()
    spread = math.sqrt(variances.mean()) * factor if len(variances) else math.nan
    return f"{mean:.1f}" + (f" ± {spread:.1f}" if math.isfinite(spread) else "")


def link(row: pd.Series, out: Path, text: str | None = None) -> str:
    """A Markdown link from `out` to the run's overview.png, shown as `text` (default the run's folder name)."""
    return f"[{text or row.run}]({os.path.relpath(Path(row.folder) / 'overview.png', out)})"


def markdown(table: pd.DataFrame, group: str | None = None, lower: tuple = (), higher: tuple = (),
             within: list[str] | None = None, comparable: str | None = None) -> str:
    """A table as Markdown: blocks separated by an empty row, and the best value in bold within each comparison.

    The best value is bold only where it beats every other row of its comparison by more than their spread: a cell
    like "9.6 ± 2.1" compares by its mean, against the larger of the two spreads.

    Args:
        table: The table, its rows in block order.
        group: Column whose equal values form a block (e.g. "robot"); None: one block.
        lower: Columns where lower is better (errors); bold only where given.
        higher: Columns where higher is better (success rate).
        within: Columns whose equal values form one comparison for the bold, e.g. ["robot", "template"]; default
            the group.
        comparable: Boolean column, not shown: a comparison with any False row gets no bold (e.g. other paths).
    """
    if table.empty:
        return "(none)"
    blocks = [table.index] if group is None else [rows.index for _, rows in table.groupby(group, sort=False)]
    keys = within or group
    compared = [table.index] if keys is None else [rows.index for _, rows in table.groupby(keys, sort=False)]
    best = pd.DataFrame(False, index=table.index, columns=table.columns)
    for index in compared:
        if len(index) < 2 or (comparable and not table.loc[index, comparable].all()):
            continue
        for column in (c for c in (*lower, *higher) if c in table.columns):
            text = table.loc[index, column].astype(str)
            values = pd.to_numeric(text.str.split().str[0], errors="coerce")
            spreads = pd.to_numeric(text.str.partition("± ")[2], errors="coerce").fillna(0.0)
            if values.notna().sum() < 2:
                continue
            target = values.min() if column in lower else values.max()
            winners = values == target
            others = values.notna() & ~winners
            # * No bold when every row ties, or when the gap to any other row is within the spread.
            margin = np.maximum(spreads[others], spreads[winners].max())
            if others.any() and ((values[others] - target).abs() > margin).all():
                best.loc[index, column] = winners
    if comparable:
        table, best = table.drop(columns=comparable), best.drop(columns=comparable)

    def cell(value, bold):
        text = "—" if isinstance(value, float) and math.isnan(value) else str(value)
        return f"**{text}**" if bold else text

    # * The grouping columns show their value once per run of equal values, so each comparison reads as one.
    named = [c for c in dict.fromkeys([*([group] if group else []), *(within or [])]) if c in table.columns]
    header = "| " + " | ".join(str(c) for c in table.columns) + " |"
    rule = "|" + "---|" * len(table.columns)
    rows = []
    for number, index in enumerate(blocks):
        if number:
            # * Markdown tables have no row borders: an empty row separates the blocks.
            rows.append("|" + " |" * len(table.columns))
        previous = None
        for i in index:
            key = tuple(table.at[i, c] for c in named)
            shown = [("" if previous is not None and column in named
                      and key[:named.index(column) + 1] == previous[:named.index(column) + 1] else value)
                     for column, value in table.loc[i].items()]
            previous = key
            rows.append("| " + " | ".join(cell(v, b) for v, b in zip(shown, best.loc[i])) + " |")
    return "\n".join([header, rule, *rows])


def save(figure: Figure, out: Path, name: str) -> str:
    """Write a figure; returns its file name."""
    figure.savefig(out / name, dpi=130)
    return name


def slug(name: str) -> str:
    """A name for files and columns, e.g. "tight_curve"."""
    return "".join(c if c.isalnum() else "_" for c in name.lower()).strip("_")


def values_text(model: dict[str, float]) -> str:
    """Sim model values that differ from the ideal robot, e.g. "xICR -0.11 m, command delay 250 ms"; "ideal" if none."""
    parts = []
    for key, value in model.items():
        if value != MODEL_PARAMETERS[key]:
            label, factor, unit = MODEL_LABELS[key]
            parts.append(f"{label} {value * factor:.3g}" + (f" {unit}" if unit else ""))
    return ", ".join(parts) or "ideal"


def _robot_labels(runs: pd.DataFrame) -> pd.Series:
    """What the robot was: "real", "sim ideal", "sim <model name>", or "sim model A", B, ... for unnamed models.

    A run without a model name takes the name another run gave the same values. Two different models under one
    name are numbered like scenarios: "sim alice_tiles", "sim alice_tiles #2".
    """
    sims = runs[runs.environment == "sim model"]
    named = sims[~sims.sim_model_name.isin(["", "ideal"])]
    names = {}
    for name, group in named.groupby("sim_model_name", sort=False):
        for i, model in enumerate(dict.fromkeys(m for m in group.model if m not in names)):
            names[model] = f"sim {name}" + (f" #{i + 1}" if i else "")
    unnamed = [model for model in dict.fromkeys(sims.model) if model not in names]
    names.update({model: f"sim model {chr(ord('A') + i)}" for i, model in enumerate(sorted(unnamed))})
    return pd.Series([names[model] if environment == "sim model" else ENVIRONMENTS.get(environment, environment)
                      for environment, model in zip(runs.environment, runs.model)], index=runs.index)


def _scenario_labels(runs: pd.DataFrame) -> pd.Series:
    """Scenario labels "<robot> · <controller>", e.g. "sim alice_tiles · PP ideal"; numbered if two share one.

    Constant-command runs use no controller: "<robot> · constant commands".
    """
    labels = runs.robot + " · " + runs.controller_label.where(runs.kind == PATHS, "constant commands")
    for label in labels.unique():
        keys = list(dict.fromkeys(runs.key[labels == label]))
        for i, key in enumerate(keys[1:], start=2):
            labels[(labels == label) & (runs.key == key)] = f"{label} #{i}"
    return labels


def _controller_label(description: dict) -> str:
    """Short controller label with the robot model it assumes, e.g. "PP ideal" or "PP alice_tiles"."""
    controller = description.get("controller", {})
    name = controller.get("name", "?")
    label = CONTROLLERS.get(name, name)
    assumed = _controller_model(description)
    if assumed is None:
        return label
    if assumed == CONTROLLER_MODEL:
        return f"{label} ideal"
    model = controller["parameters"].get("model")
    return f"{label} {model if model and model != 'ideal' else 'custom model'}"


def _controller_model(description: dict) -> dict[str, float] | None:
    """The robot model the controller assumes (CONTROLLER_MODEL keys); None if its parameters were not read."""
    parameters = description.get("controller", {}).get("parameters")
    if not isinstance(parameters, dict):
        return None
    return {key: float(parameters.get(key, ideal)) for key, ideal in CONTROLLER_MODEL.items()}


def _controller_model_text(description: dict) -> str:
    """The controller's assumed model, e.g. "ideal" or "alice_tiles: xICR -0.129 m, ..., command delay 220 ms"."""
    assumed = _controller_model(description)
    if assumed is None:
        return "unknown"
    if assumed == CONTROLLER_MODEL:
        return "ideal"
    name = description["controller"]["parameters"].get("model")
    values = values_text(dict(zip(MODEL_PARAMETERS, assumed.values())))
    return f"{name}: {values}" if name and name != "ideal" else values


def _model_text(description: dict) -> str:
    """The sim model's parameters that differ from the ideal robot, e.g. "xICR -0.11 m, command delay 250 ms"."""
    model = sim_model(description)
    return "" if model is None else values_text(model)


def _profile(run: dict[str, np.ndarray]) -> np.ndarray:
    """Mean |position| (m) and |heading| (°) error in PROGRESS_BINS bins of progress, while following."""
    context = run["tracking_context"]
    moving = context[:, 5] == FOLLOWING
    values = (np.abs(run["tracking_position"][:, 0]), np.abs(np.degrees(run["tracking_heading"][:, 0])))
    bins = np.clip((np.nan_to_num(context[:, 2]) * PROGRESS_BINS).astype(int), 0, PROGRESS_BINS - 1)
    profile = np.full((PROGRESS_BINS, 2), np.nan)
    for b in range(PROGRESS_BINS):
        for column, value in enumerate(values):
            chosen = value[moving & (bins == b)]
            chosen = chosen[np.isfinite(chosen)]
            if chosen.size:
                profile[b, column] = chosen.mean()
    return profile
