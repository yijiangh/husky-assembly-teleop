"""A calibration run's saved progress: it reads back as written, and runs list newest first."""

import json

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from repo_paths import DATA
from husky_assembly_teleop.plugins.calibration.storage.progress import Progress, new_run, runs
from husky_assembly_teleop.plugins.calibration.robot.sampling import Reading, Sampler
from husky_assembly_teleop.plugins.calibration.storage.session_files import read_take, write_take
from husky_assembly_teleop.plugins.calibration.robot.sweeps import load_sweeps
from scaffolding_assembly_core.geometry import Pose, compose

pytestmark = pytest.mark.safety

SERIAL, ARM = "a200-0805", "ur_arm"


def _run(session, time_of_day="120000") -> Progress:
    session.mkdir(parents=True, exist_ok=True)
    return new_run(session, SERIAL, ARM, load_sweeps(DATA, "0805", ARM), time_of_day)


def test_a_run_reads_back_as_written(tmp_path):
    """Every field survives a save and a load, integer keys included; no temporary file is left."""
    run = _run(tmp_path / "20261020")
    run.drive_seconds, run.marker_max = 6.5, [0.001, 0.0005]
    run.collisions, run.collision_base = {0: "", 3: "waypoint 4 is in collision"}, [[1.0, 2.0, 0.0], [0, 0, 0, 1]]
    run.takes, run.validations, run.output_sha = {0: "j0/calibration_a.json"}, ["validation/v.json"], "abc"
    run.punch_file, run.punch_offset = "punch_validation/p.json", [0.0006, 0.0016, 0.1105]
    run.save()
    loaded = Progress.load(run.file)
    assert loaded == run
    assert [path.name for path in run.session.iterdir()] == [run.file.name]


def test_an_older_run_file_with_the_step_reached_still_loads(tmp_path):
    """Older run files kept the furthest step reached: it is read past, not used."""
    run = _run(tmp_path / "20261020")
    data = json.loads(run.file.read_text())
    run.file.write_text(json.dumps({**data, "step": 4}))
    assert Progress.load(run.file) == run


def test_runs_list_newest_first(tmp_path):
    """An arm's runs in every session folder, newest first; other arms and junk files don't count."""
    old = _run(tmp_path / "20261019")
    newer = _run(tmp_path / "20261020", "090000")
    (tmp_path / "20261020" / "run_0805_ur_arm_100000.json").write_text("not json")
    _run(tmp_path / "20261020", "110000").file.rename(tmp_path / "20261020" / "run_0805_left_ur_arm_110000.json")
    assert [run.file for run in runs(tmp_path, SERIAL, ARM)] == [newer.file, old.file]
    assert runs(tmp_path / "missing", SERIAL, ARM) == []


def test_a_take_reads_back_from_its_file(tmp_path):
    """A take written from samples reads back with the same points and the calibration applied while recording."""
    calibration = Pose((0.05, 0.01, -0.48), tuple(Rotation.from_euler("z", 90, degrees=True).as_quat()))
    body = Pose((1.0, -2.0, 0.47))
    footprint = compose(body, calibration)
    records = []
    for angle in np.radians(np.arange(0, 360, 30)):
        tool = compose(footprint, Pose((0.5 * np.cos(angle), 0.5 * np.sin(angle), 0.9)))
        sampler = Sampler()
        sampler.add(Reading(footprint, tool, np.array([angle, 0, 0, 0, 0, 0]), 1e-3, 1e-3))
        records.append(sampler.sample(calibration, "abc", 1.0).record())
    take, applied = read_take(write_take(tmp_path, "t", records), 0)
    assert len(take.points) == 12 and take.angles[1] == pytest.approx(np.radians(30))
    assert applied.position == pytest.approx(calibration.position)
    # * Points are in the base rigid body: the tool's circle sits where it was put on base_footprint, moved by it.
    assert np.linalg.norm(take.points - take.points.mean(axis=0), axis=1) == pytest.approx(np.full(12, 0.5))
