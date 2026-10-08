"""Constant commands: the command grid, its chooser, and the per-run analysis recovering a known response."""

import json
import math
import random

import numpy as np
import pytest
from crl_husky.base_model import BaseModel

from husky_assembly_teleop.plugins.base_exp.analysis.commands import responses, steady
from husky_assembly_teleop.plugins.base_exp.analysis.common import COMMANDS, EXCLUDED, load_runs
from husky_assembly_teleop.plugins.base_exp.twist import (MAX_WHEEL_SPEED, SEGMENT, WHEEL_BASE, TwistGrid, cell_of,
                                                          grid, settings_of, twist_plan)


def _segment_run(v, w, delay=0.2, speed=0.9, steering=0.95, x_icr=-0.12, wheel_delay=0.1, rate=20.0):
    """A constant-command run through the simulator model, as the plugin records it: 0.5 s still, SEGMENT s, 1.5 s."""
    model, fine, lead = BaseModel(x_icr, speed, steering), 20, 0.5
    seconds = lead + SEGMENT + 1.5
    t = np.arange(0.0, seconds, 1.0 / rate)
    on = (t >= lead) & (t < lead + SEGMENT)
    command = np.column_stack([np.where(on, v, 0.0), np.where(on, w, 0.0)])
    t_fine = np.arange(0.0, seconds, 1.0 / (rate * fine))
    acting = (t_fine - delay >= lead) & (t_fine - delay < lead + SEGMENT)
    pose = np.zeros((len(t_fine), 3))
    for i in range(1, len(t_fine)):
        pose[i] = model.step(pose[i - 1], v * acting[i - 1], w * acting[i - 1], 1.0 / (rate * fine))
    context = np.zeros((len(t), 6))
    context[:, 5] = np.where(t < lead + SEGMENT, 1, 2)
    wheels = (t - wheel_delay >= lead) & (t - wheel_delay < lead + SEGMENT)
    return {"t": t, "floor_pose": pose[::fine], "tracking_context": context, "follower_command": command,
            "times": np.column_stack([t, t]), "commands": np.column_stack([t, t, command, np.ones(len(t))]),
            "wheel_odometry": np.column_stack([t, t, v * wheels, w * wheels])}


def test_grid_keeps_the_wheel_limit_and_both_directions():
    """Every cell within the wheel speed limit; every turning cell also the other way round."""
    cells = grid()
    assert all(abs(v) + abs(w) * WHEEL_BASE / 2 <= MAX_WHEEL_SPEED + 1e-9 for v, w in cells)
    assert all((v, -w) in cells for v, w in cells)
    assert cell_of(settings_of(0.2, -0.3)) == (0.2, -0.3)


def test_chooser_repeats_the_grid_evenly():
    """The least attempted cell comes first, so a second round starts only once the first is complete."""
    chooser = TwistGrid([], random.Random(0))
    seen = []
    for _ in range(len(grid()) + 3):
        settings = next(chooser.candidates())
        seen.append(cell_of(settings))
        chooser.count(settings, True)
    assert len(set(seen[:len(grid())])) == len(grid())
    assert chooser.summary.startswith("round 2: 3/")


def test_plan_predicts_the_ideal_arc():
    """The drawn run is the ideal robot's: on an arc of radius v / ω."""
    plan = twist_plan("a", 0.2, 0.5, (0.0, 0.0, 0.0))
    centre = np.array([0.0, 0.2 / 0.5])
    assert np.allclose(np.hypot(*(plan.poses[:, :2] - centre).T), 0.4, atol=1e-2)
    assert plan.expected == SEGMENT


@pytest.mark.parametrize("v, w", [(0.2, 0.5), (0.0, -0.3), (0.3, 0.0), (-0.2, 0.3)])
def test_steady_recovers_the_response(v, w):
    """Steady values give back the model; the step delay is the command delay (within a sample)."""
    found = steady(_segment_run(v, w))
    if abs(v) > 0.05:
        assert found.speed_efficiency == pytest.approx(0.9, abs=0.02)
    if abs(w) > 0.1:
        assert found.steering_efficiency == pytest.approx(0.95, abs=0.02)
        assert found.x_icr == pytest.approx(-0.12, abs=0.005)
        assert found.wheel_w == pytest.approx(w, abs=1e-6)
    assert found.delay == pytest.approx(0.2, abs=0.06)
    assert found.wheel_delay == pytest.approx(0.1, abs=0.06)


def test_constant_command_runs_load_with_the_others(tmp_path):
    """Constant-command runs are told by their settings and load without tracking; responses() analyses them."""
    for name, settings in (("twist", settings_of(0.2, 0.5)), ("arc", {"name": "Arc", "parameters": {}})):
        folder = tmp_path / "exp" / name
        folder.mkdir(parents=True)
        (folder / "experiment.json").write_text(json.dumps(
            {"experiment_name": "exp", "template": settings, "outcome": "done", "environment": "real"}))
        arrays = {f"x_{k}" if k == "floor_pose" else k: value for k, value in _segment_run(0.2, 0.5).items()}
        np.savez(folder / "recording.npz", **arrays)
    runs, _, left_out = load_runs(tmp_path, "exp")
    assert list(runs.run) == ["twist"] and runs.kind.iloc[0] == COMMANDS
    assert left_out == "1 without the monitor's tracking"  # the arc: a path run needs the tracking
    found = responses(runs)
    assert bool(found.used.iloc[0]) and found.robot.iloc[0] == "real"
    assert found.x_icr.iloc[0] == pytest.approx(-0.12, abs=0.005) and not math.isnan(found.delay.iloc[0])


def test_e_stopped_and_listed_runs_are_left_out(tmp_path):
    """An e-stopped run, and one listed in the experiment's EXCLUDED file, stay on disk but out of the analysis."""
    for name, outcome in (("kept", "done"), ("estop", "e-stopped"), ("listed", "done")):
        folder = tmp_path / "exp" / name
        folder.mkdir(parents=True)
        (folder / "experiment.json").write_text(json.dumps(
            {"experiment_name": "exp", "template": settings_of(0.2, 0.5), "outcome": outcome, "environment": "real"}))
        np.savez(folder / "recording.npz", **{f"x_{k}" if k == "floor_pose" else k: value
                                              for k, value in _segment_run(0.2, 0.5).items()})
    (tmp_path / "exp" / EXCLUDED).write_text("# bad runs\nlisted  # e-stopped before the plugin knew\n")
    runs, _, left_out = load_runs(tmp_path, "exp")
    assert list(runs.run) == ["kept"]
    assert left_out == f"1 e-stopped, 1 listed in {EXCLUDED}"


def test_two_models_under_one_name_are_numbered(tmp_path):
    """Robots are named like scenarios: one model name with two sets of values gives "sim m" and "sim m #2"."""
    for name, x_icr in (("first", -0.12), ("second", -0.10)):
        folder = tmp_path / "exp" / name
        folder.mkdir(parents=True)
        (folder / "experiment.json").write_text(json.dumps(
            {"experiment_name": "exp", "template": settings_of(0.2, 0.5), "outcome": "done", "environment": "sim",
             "sim_model": {"model": "m", "xICR": x_icr}}))
        np.savez(folder / "recording.npz", **{f"x_{k}" if k == "floor_pose" else k: value
                                              for k, value in _segment_run(0.2, 0.5).items()})
    runs, _, _ = load_runs(tmp_path, "exp")
    assert list(runs.robot) == ["sim m", "sim m #2"]
    assert list(runs.scenario) == ["sim m · constant commands", "sim m #2 · constant commands"]
