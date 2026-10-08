"""Replaying recorded commands: the true model predicts the measured motion, the ideal one does not."""

import numpy as np
import pytest
from base_exp_fixtures import model_run

from husky_assembly_teleop.plugins.base_exp.identify import cross_validated, identify
from husky_assembly_teleop.plugins.base_exp.replay import HORIZON, IDEAL, Model, errors, free_run, synthetic, track

TRUE = Model.of(-0.1, 0.9, 0.95, 0.2)


def test_true_model_predicts_the_motion():
    """One second ahead, the true model is within millimetres; the ideal one is centimetres off."""
    run = track(model_run())
    true, _ = errors(run, TRUE).at(HORIZON)
    ideal, _ = errors(run, IDEAL).at(HORIZON)
    assert len(true) > 100
    assert np.median(true) < 0.005 and np.median(ideal) > 0.02


def test_errors_grow_with_the_horizon():
    """A wrong model drifts further the longer it runs."""
    found = errors(track(model_run()), IDEAL)
    medians = np.median(found.position, axis=0)
    assert np.all(np.diff(medians) > 0)


def test_cross_validated_model_is_close_to_the_truth():
    """Fitted on the other runs, the model still predicts within millimetres."""
    runs = [model_run(seed=seed) for seed in range(4)]
    position, _ = cross_validated(runs).at(HORIZON)
    assert np.median(position) < 0.005


def test_free_run_follows_the_measured_path():
    """Without corrections, the true model stays close over a short run."""
    run = track(model_run(seconds=20.0))
    times, poses = free_run(run, TRUE)
    measured = run.pose[-len(times):]
    assert np.max(np.hypot(*(poses[:, :2] - measured[:, :2]).T)) < 0.05


def test_synthetic_runs_give_their_model_back():
    """The evaluation check: runs made to follow a model exactly identify as that model."""
    made = [synthetic(track(model_run(seed=seed, delay=0.0, speed=1.0, steering=1.0, x_icr=0.0)), TRUE)
            for seed in range(2)]
    found = identify(made)
    assert found.delay == pytest.approx(0.2, abs=0.011) and found.x_icr == pytest.approx(-0.1, abs=0.002)
    assert found.speed_efficiency == pytest.approx(0.9, rel=0.01)
