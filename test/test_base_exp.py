"""The base experiment's test paths, as the onboard follower will cut them into pieces, and its reports."""

import json
import math

import numpy as np
import pytest
from crl_husky.follower_path import FollowerPath

from husky_assembly_teleop.plugin_api.trace import Signal, Trace
from husky_assembly_teleop.plugins.base_exp.templates import (PARAMETERS, TEMPLATES, arc, drive_turn_drive, place,
                                                              straight, timing, turn)

DEFAULTS = {p.name: (math.radians(p.default) if p.name == "angle" else p.default) for p in PARAMETERS}


@pytest.mark.parametrize("name", list(TEMPLATES))
def test_every_template_makes_a_followable_path(name):
    """Each template's poses start at the origin and build a path that ends at its last pose."""
    poses = TEMPLATES[name].make(DEFAULTS)
    assert np.allclose(poses[0], 0.0)
    path = FollowerPath.from_poses(*poses.T, timing(poses, 0.2, 0.5))
    assert path.timed and path.duration > 0
    end = path.polyline()[-1]
    assert np.allclose(end[:2], poses[-1, :2], atol=1e-6)
    assert end[2] == pytest.approx(poses[-1, 2])


def test_drive_turn_drive_has_a_turn_on_the_spot():
    """The turn between the straights is a piece of its own, in the given direction."""
    path = FollowerPath.from_poses(*drive_turn_drive(1.0, -math.pi / 2, 0.5).T)
    kinds = [piece.turning for piece in path.pieces]
    assert kinds == [False, True, False, True]
    assert path.pieces[1].yaw1 - path.pieces[1].yaw0 == pytest.approx(-math.pi / 2)


def test_negative_length_drives_backwards():
    """A negative straight keeps the heading and reverses."""
    path = FollowerPath.from_poses(*straight(-1.0).T)
    assert path.pieces[0].reverse and path.pieces[0].length == pytest.approx(1.0)


def test_full_turn_keeps_its_sense():
    """Turning 360 degrees stays a full turn, not zero."""
    poses = turn(2 * math.pi)
    path = FollowerPath.from_poses(*poses.T)
    assert path.pieces[0].yaw1 - path.pieces[0].yaw0 == pytest.approx(2 * math.pi)


def test_place_moves_into_the_world():
    """Placing turns and shifts the poses by the start pose, keeping yaw unwrapped."""
    world = place(arc(1.0, math.pi), (1.0, 2.0, math.pi / 2))
    assert world[0] == pytest.approx([1.0, 2.0, math.pi / 2])
    assert world[-1, :2] == pytest.approx([-1.0, 2.0], abs=1e-9)
    assert world[-1, 2] == pytest.approx(1.5 * math.pi)


def test_timing_takes_the_slower_of_driving_and_turning():
    """A 1 m straight at 0.2 m/s takes 5 s; a quarter turn at 0.5 rad/s takes pi seconds."""
    assert timing(straight(1.0), 0.2, 0.5)[-1] == pytest.approx(5.0)
    assert timing(turn(math.pi / 2), 0.2, 0.5)[-1] == pytest.approx(math.pi)


def test_trace_save_stores_extra_arrays(tmp_path):
    """Extra arrays go into the same file; clashing names are refused."""
    trace = Trace([Signal("p", lambda: 1.0)])
    trace.sample(0.0)
    data = np.load(trace.save(tmp_path / "run.npz", {"path_poses": np.zeros((2, 3))}))
    assert data["path_poses"].shape == (2, 3) and data["p"].tolist() == [[1.0]]
    with pytest.raises(ValueError):
        trace.save(tmp_path / "bad.npz", {"p": np.zeros(1)})


def test_report_plots_a_recording(tmp_path):
    """A run folder gets experiment.json (setup, outcome, numbers by segment) and overview.png."""
    from husky_assembly_teleop.plugins.base_exp.report import load, metrics, write_report
    poses = drive_turn_drive(1.0, math.pi / 2, 0.5)
    t = np.arange(0.0, 10.0, 0.05)
    n = len(t)
    driven = np.column_stack([np.linspace(0, 1, n), np.zeros(n), np.zeros(n)])
    turning = (np.arange(n) > 100) & (np.arange(n) < 120)
    phase = np.where(np.arange(n) < 180, 1, 2)
    context = np.column_stack([turning, np.zeros(n), np.linspace(0, 1, n), np.full(n, 0.2), np.zeros(n), phase])
    names = [("a200-0804_floor_pose", ("x", "y", "yaw")), ("tracking_position", ("position", "along")),
             ("tracking_heading", ()), ("tracking_context", ("turning", "curvature", "progress", "speed",
                                                             "turn_rate", "phase")),
             ("a200-0804_follower_command", ("v", "w"))]
    trace = Trace([Signal(name, None, labels) for name, labels in names])
    for i, time in enumerate(t):
        trace.add(time, driven[i], (0.01, np.nan), 0.02, context[i], (0.2, 0.0))
    setup = {"environment": "sim", "sim_model": {"xICR": 0.0}, "controller": {"name": "pure_pursuit"}}
    saved = trace.save(tmp_path / "recording.npz", {
        "path_poses": poses, "path_t": np.array([]), "path_label": np.array("Drive, turn, drive"),
        "outcome": np.array("done"), "speed_caps": np.array([0.2, 0.5]), "expected_duration": np.array(9.0),
        "experiment": np.array(json.dumps(setup))})
    numbers = metrics(load(saved))
    assert numbers.motion_position == pytest.approx(0.01) and numbers.motion_heading == pytest.approx(1.146, 1e-3)
    assert numbers.segments["spot turn"]["samples"] == 19 and numbers.segments["straight"]["samples"] == 161
    assert numbers.end_position == pytest.approx(math.hypot(1.0 - poses[-1, 0], poses[-1, 1]))
    assert numbers.duration == pytest.approx(179 * 0.05) and numbers.settle_time == pytest.approx(20 * 0.05)
    data, png = write_report(tmp_path)
    report = json.loads(data.read_text())
    assert report["environment"] == "sim" and report["metrics"]["segments"]["spot turn"]["samples"] == 19
    assert png.name == "overview.png" and png.stat().st_size > 10_000


def test_setup_key_ignores_topics_and_start_pose():
    """Runs differ in setup only by environment, model and controller tuning; not by topics or where the sim starts."""
    from husky_assembly_teleop.plugins.base_exp.record import setup_key
    run = {"environment": "sim", "sim_model": {"xICR": 0.0, "speed_efficiency": 1.0, "steering_efficiency": 1.0,
                                               "start_x": 0.0, "mocap_topic": "/a"},
           "controller": {"name": "pure_pursuit", "parameters": {"lookahead": 0.2, "mocap_topic": "/a"}}}
    moved = json.loads(json.dumps(run))
    moved["sim_model"].update(start_x=2.0, mocap_topic="/b")
    moved["controller"]["parameters"]["mocap_topic"] = "/b"
    tuned = json.loads(json.dumps(run))
    tuned["controller"]["parameters"]["lookahead"] = 0.3
    assert setup_key(run) == setup_key(moved) != setup_key(tuned)
