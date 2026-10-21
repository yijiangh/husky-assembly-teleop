"""Run folders on the Drive: names that never clash, the description beside the recording, labels on reading."""

import json

import numpy as np
import pytest

from husky_assembly_teleop.plugin_api.trace import Signal, Trace
from husky_assembly_teleop.robot_interface.event import Event
from husky_assembly_teleop.shared.recordings.runs import (load_description, load_recording, run_folder,
                                                          save_recording, write_description, write_report)


def test_run_folders_never_clash(tmp_path):
    """Two runs in the same second get their own folders; a missing recordings folder is an error, not made here."""
    first = run_folder(tmp_path, "trial", "alice", "free")
    second = run_folder(tmp_path, "trial", "alice", "free")
    assert first.is_dir() and second.is_dir()
    assert first != second
    with pytest.raises(FileNotFoundError):
        run_folder(tmp_path / "not_in_drive", "trial", "alice")


def test_recording_loads_without_the_serial_and_by_label(tmp_path):
    """Keys lose "<robot>_", message groups included; a channel is picked by its label."""
    reports = Event("a200-0804_follower")
    trace = Trace([Signal("a200-0804_follower_progress", lambda: (0.5, 2.0, 1.0), ("progress", "piece", "state")),
                   Signal("command", lambda r: r.command, ("v", "w"), on=reports)])
    trace.sample(1.0)
    trace.take(reports, type("Report", (), {"update_time": 0.9, "stamp": 0.8, "command": (0.2, 0.0)})())
    save_recording(tmp_path, trace, "a200-0804", {"path_poses": np.zeros((2, 3))})
    run = load_recording(tmp_path)
    assert run.channel("follower_progress", "state").tolist() == [1.0]
    assert run["follower.command"].tolist() == [[0.2, 0.0]] and run["follower.stamp"].tolist() == [0.8]
    assert str(run["robot"]) == "a200-0804" and run["path_poses"].shape == (2, 3)
    with pytest.raises(KeyError):
        run.channel("follower_progress", "speed")


def test_a_json_recording_loads_like_npz(tmp_path):
    """A run saved as .json loads into the same keys; a folder without a recording says so."""
    trace = Trace([Signal("a200-0804_floor_pose", lambda: (1.0, 2.0, 0.5), ("x", "y", "yaw"), ("m", "m", "rad"))])
    trace.sample(1.0)
    path = save_recording(tmp_path, trace, "a200-0804", {"path_t": np.array([])}, "json")
    run = load_recording(tmp_path)
    assert path.name == "recording.json" and run.channel("floor_pose", "yaw").tolist() == [0.5]
    assert run.unit("floor_pose", "yaw") == "rad" and run.unit("floor_pose", "x") == "m"
    assert run["path_t"].shape == (0,) and str(run["robot"]) == "a200-0804"
    with pytest.raises(FileNotFoundError):
        load_recording(tmp_path / "trial")


def test_description_and_report_sit_beside_the_recording(tmp_path):
    """experiment.json is what the plugin knew; report.json adds the numbers; numpy values are written plainly."""
    write_description(tmp_path, {"outcome": "done", "caps": np.array([0.2, 0.5]), "count": np.int64(3)})
    assert load_description(tmp_path) == {"outcome": "done", "caps": [0.2, 0.5], "count": 3}
    assert load_description(tmp_path / "missing") is None
    report = json.loads(write_report(tmp_path, load_description(tmp_path), {"error": np.float64(0.1)}).read_text())
    assert report["outcome"] == "done" and report["metrics"] == {"error": 0.1}
