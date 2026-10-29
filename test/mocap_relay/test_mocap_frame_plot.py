"""The frame plot reads the relay's samples, old and new, from files, folders and globs, and draws them headless."""

from __future__ import annotations

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest
from matplotlib import pyplot
from mocap_frame_fixtures import LAB_ANCHORS, LAB_CAMERAS, LOOSE
from repo_paths import REPO

from scaffolding_assembly_core.geometry import Pose
from husky_assembly_teleop.robot_interface.mocap_definitions import MocapAnchor
from husky_assembly_teleop.world.frame_follower import FrameFollower
from husky_assembly_teleop.world.mocap_frame import FloorCalibration

SCRIPT = REPO / "scripts" / "mocap_frame_plot.py"
#: A sample as `--record` kept it before the per-anchor quality was published.
OLD = {"time": 100.0, "floor_calibration": "20261024_090000_mocap_floor_calibration.json", "status": "held",
       "decides": "anchors", "anchors": None, "truss_cameras": None, "refit": None, "off": 0.0, "off_limit": 0.001,
       "apart": None, "anchors_missing": []}


@pytest.fixture(scope="module")
def script():
    """The plot script as a module, drawing headless."""
    matplotlib = pytest.importorskip("matplotlib")
    matplotlib.use("Agg")
    spec = importlib.util.spec_from_file_location("mocap_frame_plot", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def new_samples(count: int, start: float) -> list[dict]:
    """`count` looks 2 s apart from `start`, as the relay publishes them: A2 3 mm off from the second, A6 never seen."""
    frame = FrameFollower()
    frame.use_floor_calibration(FloorCalibration(Path(OLD["floor_calibration"]), Pose(), dict(LAB_CAMERAS),
                                                 {name: MocapAnchor(name, at) for name, at in LAB_ANCHORS.items()}))
    frame.cameras = dict(LAB_CAMERAS)
    samples = []
    for index in range(count):
        moved = {**LAB_ANCHORS, "A2": tuple(np.add(LAB_ANCHORS["A2"], (0.003 if index else 0.0, 0.0, 0.0)))}
        markers = [at for name, at in moved.items() if name != "A6"] + [LOOSE]
        frame.look(markers, start + 2.0 * index, [(0.0001, 100)] * len(markers))
        samples.append(json.loads(json.dumps(frame.metrics(start + 2.0 * index))))
    return samples


def write(path: Path, samples: list[dict]) -> Path:
    """`samples` as JSON lines in `path`."""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(sample) + "\n" for sample in samples))
    return path


def test_files_folders_and_globs_read_in_time_order_and_a_cut_line_is_skipped(script, tmp_path):
    """Two day files in a folder (its sim/ left out), an old recording by glob; the last day's line cut by a crash."""
    write(tmp_path / "log" / "20261027.jsonl", new_samples(2, 1000.0))
    day = write(tmp_path / "log" / "20261028.jsonl", new_samples(2, 2000.0))
    day.write_text(day.read_text() + '{"time": 30')
    write(tmp_path / "log" / "sim" / "domain_70" / "20261028.jsonl", new_samples(1, 5000.0))
    write(tmp_path / "old" / "frame.jsonl", [OLD])
    files = script.sample_files([str(tmp_path / "log"), str(tmp_path / "old" / "*.jsonl"), str(day)])
    assert [path.name for path in files] == ["20261027.jsonl", "20261028.jsonl", "frame.jsonl"]
    samples, bad = script.read_samples(files)
    assert [sample["time"] for sample in samples] == [100.0, 1000.0, 1002.0, 2000.0, 2002.0] and bad == 1


def test_each_anchors_error_and_noise_are_series_with_gaps_and_left_out_marked(script):
    """An old sample has none (NaN); A2 left out from the second look; A6 never found: all NaN."""
    samples = [OLD, *new_samples(3, 1000.0)]
    errors = script.anchor_series(samples, "left_out_mm")
    assert sorted(errors) == [f"A{n}" for n in range(1, 7)]
    values, bumped = errors["A2"]
    assert np.isnan(values[0]) and values[2] == pytest.approx(3.0, abs=0.05)
    assert list(bumped) == [False, False, True, True]
    assert np.all(np.isnan(errors["A6"][0])) and script.anchor_series(samples, "noise_mm")["A1"][0][1] == 0.1
    assert script.anchor_series(samples, "seen")["A1"][0][1] == 1.0
    summary = "\n".join(script.anchor_summary(samples))
    assert "A2" in summary and script.anchor_summary([OLD]) == []


@pytest.mark.parametrize("which", ["old", "new", "both"])
def test_old_and_new_samples_draw_without_a_window(script, which):
    """Old samples leave the anchors' panels empty; nothing crashes."""
    samples = {"old": [OLD, {**OLD, "time": 160.0}], "new": new_samples(3, 1000.0),
               "both": [OLD, *new_samples(3, 1000.0)]}[which]
    figure = script.plot(samples)
    # ! Closed before asserting: a figure left open fails other tests on the same worker that count open figures.
    axes = figure.axes
    pyplot.close(figure)
    assert len(axes) == 8
    assert (len(axes[5].lines) == 0) == (which == "old")
