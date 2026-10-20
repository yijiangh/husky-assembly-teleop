"""
Experiment runs on the Drive, saved and read the same way by every plugin and by the offline analysis.

A run (also a plan attempt or a take) is a folder `<recordings>/<experiment>/<date>_<time>_<parts>/` (`run_folder`):

- experiment.json: what the plugin knows of the run, e.g. setup, conditions and outcome (`write_description`).
- recording.npz, or recording.json if asked for (`MonitorConfig.recording_format`): its trace (`save_recording`),
  with the robot's serial as "robot"; `load_recording` reads either back with the serial dropped from the keys and
  picks a signal's channel by label.
- report.json and pictures: what the analysis computes from these (`write_report`).

! The recordings folder of each plugin (its RECORDING_FOLDER under the Drive root) is made in Drive, never here
  (`drive.py`); the experiment and run folders below it are made here.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from ...drive import stamped_name
from ...plugin_api.trace import FORMATS, load_arrays

if TYPE_CHECKING:
    from ...plugin_api.trace import Trace

#: The files of a run folder; the recording is "recording.<format>", one of FORMATS.
DESCRIPTION, RECORDING, REPORT = "experiment.json", "recording", "report.json"


def run_folder(recordings: Path, experiment: str, *parts: str) -> Path:
    """Make a new run folder under the experiment, named by time and `parts`; "_2", "_3", … if the name is taken.

    Args:
        recordings: The plugin's recordings folder; it must exist.
        experiment: The experiment's folder name, made if new.
        parts: What names the run, e.g. the robot, task and "sim".

    Raises:
        OSError: If the recordings folder is missing or the folder cannot be made.
    """
    if not recordings.is_dir():
        raise FileNotFoundError(f"no recordings folder {recordings}; make it in Drive")
    name = stamped_name(*parts)
    folder, count = recordings / experiment / name, 2
    while True:
        try:
            folder.mkdir(parents=True)
            return folder
        except FileExistsError:
            folder, count = recordings / experiment / f"{name}_{count}", count + 1


def write_description(folder: Path, description: dict) -> Path:
    """Write a run's experiment.json; returns it."""
    path = folder / DESCRIPTION
    path.write_text(json.dumps(description, indent=2, default=plain) + "\n")
    return path


def load_description(folder: Path) -> dict | None:
    """A run's experiment.json; None if the folder has none."""
    path = folder / DESCRIPTION
    return json.loads(path.read_text()) if path.is_file() else None


def write_report(folder: Path, description: dict, metrics: dict) -> Path:
    """Write a run's report.json: its description and the numbers the analysis computed; returns it."""
    path = folder / REPORT
    path.write_text(json.dumps({**description, "metrics": metrics}, indent=2, default=plain) + "\n")
    return path


def save_recording(folder: Path, rec: Trace, robot: str | None, extra: dict[str, Any] | None = None,
                   file_format: str = "npz") -> Path:
    """Write a run's recording: the trace, `extra` arrays such as the path sent, and the robot's serial.

    Args:
        folder: The run folder.
        rec: The run's trace.
        robot: The serial its signals are named after ("<serial>_floor_pose"), or None for none.
        extra: More arrays; keys must not clash with the trace's.
        file_format: "npz" (compact, the default) or "json" (readable, several times larger).

    Returns:
        Path: The file written.
    """
    extra = {**(extra or {}), **({} if robot is None else {"robot": np.array(robot)})}
    return rec.save(folder / f"{RECORDING}.{file_format}", extra)


def recording_path(folder: Path) -> Path | None:
    """The run's recording file, in whichever format it was saved; None if it has none."""
    return next((path for path in (folder / f"{RECORDING}.{f}" for f in FORMATS) if path.is_file()), None)


class Recording(dict):
    """A loaded recording: its arrays by the .npz keys (`Trace.save`), the robot's "<serial>_" dropped from them."""

    def channel(self, key: str, label: str) -> np.ndarray:
        """One channel of a signal, picked by its label: for example "phase" of "tracking_context".

        Raises:
            KeyError: If the signal or the label is not in the recording.
        """
        return self[key][:, self._index(key, label)]

    def unit(self, key: str, label: str) -> str:
        """The unit of one channel of a signal, by its label; "" for none.

        Raises:
            KeyError: If the signal or the label is not in the recording.
        """
        return str(self[f"{key}.units"][self._index(key, label)])

    def _index(self, key: str, label: str) -> int:
        """The column of the channel `label` in signal `key`."""
        labels = [str(name) for name in self[f"{key}.labels"]]
        if label not in labels:
            raise KeyError(f"{key} has no channel {label!r}; it has {labels}")
        return labels.index(label)


def load_recording(path: str | Path) -> Recording:
    """A run's recording, .npz or .json (or the run folder holding it), with "<robot>_" dropped from its keys.

    Raises:
        FileNotFoundError: If the folder has no recording.
    """
    path = Path(path)
    if path.is_dir():
        found = recording_path(path)
        if found is None:
            raise FileNotFoundError(f"{path} has no {RECORDING}.{' or .'.join(FORMATS)}")
        path = found
    arrays = load_arrays(path)
    prefix = f"{arrays['robot']}_" if "robot" in arrays else ""
    return Recording({key.removeprefix(prefix): value for key, value in arrays.items()})


def plain(value):
    """JSON fallback: numpy numbers and arrays as plain Python."""
    return value.item() if isinstance(value, np.generic) else np.asarray(value).tolist()
