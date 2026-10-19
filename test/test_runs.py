"""The `runs/` record (shared.runs) and the rules of marking actions (shared.assembly_run), on a small process."""

from __future__ import annotations

import json
from dataclasses import replace
from datetime import datetime, timezone
from hashlib import sha256
from pathlib import Path
from types import SimpleNamespace

import pytest
from process_fixtures import build_process, with_action, with_start, write_robot_files

from husky_assembly_teleop.shared import runs
from husky_assembly_teleop.shared.assembly_run import exported_end, first_pending, is_live
from husky_assembly_teleop.shared.process import LoadedProcess
from husky_assembly_teleop.shared.runs import (ASSUMED, DONE, LIVE, PENDING, SCHEMA, RobotRecord, RunError, RunSettings,
                                               continue_run, file_hashes, find_runs, mark_done, new_run, read_events,
                                               read_run, record_event, record_movement, reopen, save_progress)
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.process import RobotState, write
from scaffolding_assembly_core.robot import RobotObject

NOW = datetime(2026, 10, 21, 9, 30, 0, tzinfo=timezone.utc)
ALICE, CINDY = "robots/a200-0804", "robots/a200-0806"


@pytest.fixture
def process(tmp_path):
    """The small process, written to `tmp_path/p`."""
    return write(build_process(tmp_path), tmp_path / "p")


@pytest.fixture
def robots(tmp_path):
    """Two configured robots: a serial and a URDF each is all a run records."""
    urdf, _ = write_robot_files(tmp_path / "robots")
    return tuple(SimpleNamespace(serial=serial, model=SimpleNamespace(urdf=urdf), source_urdf=None)
                 for serial in ("a200-0804", "a200-0806"))


def record(x: float, source: str = LIVE) -> RobotRecord:
    """A robot record at base x."""
    return RobotRecord(Pose((x, 0.0, 0.0)), {"left_joint1": x}, source, "t")


def test_new_run_reads_back(process, robots, tmp_path):
    """A new run's files read back equal, with the schema and the process's file hashes, every action pending."""
    run = new_run(process, robots, RunSettings(spiral=True), now=NOW, root=tmp_path)
    assert run.folder == Path(process.folder) / "runs" / "20261021_093000"
    assert read_run(run.folder) == run
    data = json.loads((run.folder / "run.json").read_text())
    assert (data["schema"], data["kind"], data["process"]) == (SCHEMA, "run", "p")
    assert (run.info.process_hash, run.info.action_hashes) == file_hashes(process)
    assert set(run.info.robots) == {ALICE, CINDY} and len(run.info.robots[CINDY]["urdf_sha256"]) == 64
    assert set(run.info.commits) == {"monitor", "core", "tamp"}
    assert {record.status for record in run.progress.actions.values()} == {PENDING}
    assert list(run.progress.actions) == list(process.schedule)
    assert [event["event"] for event in read_events(run.folder)] == ["new_run"]


def test_continue_run_picks_the_newest_matching(process, robots):
    """Of the runs on these files the newest is continued; a run on other files is not, another schema is a problem."""
    older = new_run(process, robots, RunSettings(), now=NOW)
    newer = new_run(process, robots, RunSettings(), now=NOW.replace(minute=40))
    other = new_run(process, robots, RunSettings(), now=NOW.replace(minute=50))
    info = json.loads((other.folder / "run.json").read_text())
    (other.folder / "run.json").write_text(json.dumps({**info, "process_hash": "0" * 64}))
    alien = Path(process.folder) / "runs" / "20261021_095500"
    alien.mkdir()
    (alien / "run.json").write_text(json.dumps({**info, "schema": "husky_run/9"}))

    found, problems = find_runs(process.folder)
    assert [run.info.name for run in found] == [other.info.name, newer.info.name, older.info.name]
    assert len(problems) == 1 and problems[0].startswith("20261021_095500") and "husky_run/9" in problems[0]
    assert continue_run(found, file_hashes(process)) == newer


def test_mark_done_records_ends_and_beliefs(process, robots):
    """A mark keeps its ends, believes each robot there and counts on; a later mark changes only its own robots."""
    run = new_run(process, robots, RunSettings(), now=NOW)
    run = mark_done(run, "B1_J_joint", {CINDY: record(1.0), ALICE: record(2.0)}, False, now=NOW)
    run = mark_done(run, "B1_H_hold", {ALICE: record(3.0, ASSUMED)}, True, now=NOW)
    first, second = run.progress.actions["B1_J_joint"], run.progress.actions["B1_H_hold"]
    assert (first.status, first.seq, first.assumed) == (DONE, 1, False)
    assert (second.status, second.seq, second.assumed) == (DONE, 2, True)
    assert first.ends[CINDY].after_action == "B1_J_joint"
    assert run.progress.robots[CINDY] == replace(record(1.0), after_action="B1_J_joint")
    assert run.progress.robots[ALICE] == replace(record(3.0, ASSUMED), after_action="B1_H_hold")
    assert first_pending(run.progress, process.schedule) == "B1_R_release"
    with pytest.raises(RunError):
        mark_done(run, "B1_J_joint", {}, False)


def test_reopen_restores_the_earlier_end_or_drops_it(process, robots):
    """Reopening the latest mark puts a robot back at its latest earlier end; a robot with none loses its belief."""
    run = new_run(process, robots, RunSettings(), now=NOW)
    run = mark_done(run, "B1_J_joint", {CINDY: record(1.0)}, False)
    run = mark_done(run, "B1_H_hold", {CINDY: record(2.0), ALICE: record(3.0)}, False)
    run = mark_done(run, "B1_R_release", {CINDY: record(4.0)}, False)

    reopened = reopen(run, "B1_H_hold")
    assert reopened.progress.actions["B1_H_hold"] == runs.ActionRecord()
    assert ALICE not in reopened.progress.robots
    assert reopened.progress.robots[CINDY].after_action == "B1_R_release"  # not from the reopened action

    reopened = reopen(reopened, "B1_R_release")
    assert reopened.progress.robots[CINDY] == replace(record(1.0), after_action="B1_J_joint")
    with pytest.raises(RunError, match="not done"):
        reopen(reopened, "B1_H_hold")


def test_new_run_keeps_the_old_folder(process, robots):
    """A new run leaves the old run's files as they were; one started in the same second is named `_2`."""
    old = new_run(process, robots, RunSettings(), now=NOW)
    save_progress(mark_done(old, "B1_J_joint", {CINDY: record(1.0)}, False))
    before = {path.name: path.read_bytes() for path in old.folder.iterdir()}
    same_second = new_run(process, robots, RunSettings(), now=NOW)
    assert same_second.folder.name == "20261021_093000_2"
    assert {path.name: path.read_bytes() for path in old.folder.iterdir()} == before
    assert find_runs(process.folder)[0][0].folder == same_second.folder


def test_re_export_keeps_runs_and_carries_by_id(process, robots, tmp_path):
    """Writing the process again keeps runs/; the old run no longer matches, and continuing it carries what still is."""
    run = new_run(process, robots, RunSettings(), now=NOW)
    run = mark_done(run, "B1_J_joint", {CINDY: record(1.0)}, False)
    run = mark_done(run, "B1_HR_hold_release", {ALICE: record(2.0)}, False)
    run = replace(run, progress=replace(run.progress, current=("B1_H_hold", 1)))
    save_progress(run)

    changed = with_action(build_process(tmp_path), "B1_H_hold", label="hold B1 tight")
    changed = replace(changed, schedule=changed.schedule[:-1],
                      actions={key: value for key, value in changed.actions.items() if key != "B1_HR_hold_release"})
    again = write(changed, process.folder, overwrite=True)
    found, _ = find_runs(again.folder)
    assert [each.folder for each in found] == [run.folder]
    assert continue_run(found, file_hashes(again)) is None

    carried = new_run(again, robots, RunSettings(), continued_from=found[0], now=NOW.replace(hour=10))
    assert carried.info.continued_from == run.info.name
    assert carried.info.changed_since == ("process.json", "B1_H_hold")  # the schedule is in process.json
    assert carried.progress.actions["B1_J_joint"].status == DONE
    assert "B1_HR_hold_release" not in carried.progress.actions
    assert carried.progress.current == ("B1_H_hold", 1)
    assert set(carried.progress.robots) == {CINDY, ALICE}
    event, = read_events(carried.folder)
    assert (event["event"], event["dropped"]) == ("continue", ["B1_HR_hold_release"])
    assert event["changed"] == ["process.json", "B1_H_hold"]


def test_crash_safety(process, robots, monkeypatch):
    """A failed rename keeps the old progress.json readable; a stray .tmp is ignored; a cut last event is skipped."""
    run = new_run(process, robots, RunSettings(), now=NOW)

    def crash(_source, _target):
        raise OSError("power cut")

    monkeypatch.setattr(runs, "replace_file", crash)
    with pytest.raises(OSError):
        save_progress(mark_done(run, "B1_J_joint", {CINDY: record(1.0)}, False))
    monkeypatch.undo()
    assert (run.folder / "progress.json.tmp").is_file()
    assert read_run(run.folder) == run
    assert find_runs(process.folder) == ([run], [])

    record_event(run.folder, "mark_done", 12.5, action="B1_J_joint")
    with open(run.folder / "events.jsonl", "a") as handle:
        handle.write('{"time": "2026-10-21T09:3')
    events = read_events(run.folder)
    assert [event["event"] for event in events] == ["new_run", "mark_done"]
    assert (events[1]["ros_time"], events[1]["action"], events[1]["movement"]) == (12.5, "B1_J_joint", None)


def test_record_movement_numbers_its_files(process, robots):
    """Records of one movement are numbered .1, .2; servo records go to servo/."""
    folder = new_run(process, robots, RunSettings(), now=NOW).folder
    first = record_movement(folder, "B1_J_joint", "B1_M2_insert", {"ok": True})
    second = record_movement(folder, "B1_J_joint", "B1_M2_insert", {"ok": False})
    servo = record_movement(folder, "B1_J_joint", "B1_M2_insert", {"error": 0.1}, kind="servo")
    assert (first.name, second.name) == ("B1_M2_insert.1.json", "B1_M2_insert.2.json")
    assert servo == folder / "servo" / "B1_J_joint" / "B1_M2_insert.1.json"
    assert json.loads(second.read_text()) == {"schema": SCHEMA, "kind": "movement", "ok": False}
    assert json.loads(servo.read_text())["kind"] == "servo"


def test_is_live_at_the_age_limits():
    """Live: tracked and measured, joints at most 0.5 s and the base at most 1.0 s old."""
    robot = RobotObject("robots/a200-0806", None, Pose(), {}, base_time=9.0, joints_time=9.5)
    assert is_live(robot, 10.0)
    assert not is_live(robot, 10.001)
    assert not is_live(replace(robot, base_time=8.99), 10.0)
    assert not is_live(replace(robot, base_tracked=False), 10.0)
    assert not is_live(replace(robot, unmeasured=frozenset({"left_joint1"})), 10.0)
    assert not is_live(replace(robot, joints_time=None), 10.0)


def test_exported_end(process, tmp_path):
    """The export's end of an action's robot; None where its base or joints are null."""
    loaded = LoadedProcess(process.folder, process, ())
    end = exported_end(loaded, "B1_J_joint", at="now")
    assert (end.source, end.at, end.joints["left_joint1"]) == (ASSUMED, "now", 0.5)
    assert exported_end(loaded, "B1_R_release") is None  # the retreat's start has no base
    start = process.actions["B1_H_hold"].movements[1].start
    unknown = with_start(process, "B1_H_hold", 1,
                         robots={**start.robots, "robots/alice": RobotState(start.robots["robots/alice"].base, None)})
    assert exported_end(LoadedProcess(process.folder, unknown, ()), "B1_H_hold") is None


def test_a_cut_line_is_ended_and_skipped(tmp_path):
    """A line cut by a crash is ended before the next append; every unreadable line is skipped and counted once."""
    record_event(tmp_path, "new_run", None)
    with open(tmp_path / "events.jsonl", "a") as handle:
        handle.write('{"time": "2026-10-21T09:3')
    record_event(tmp_path, "session", 1.0)
    record_event(tmp_path, "mark_done", 2.0, action="B20_J")
    told = []
    assert [event["event"] for event in read_events(tmp_path, log=told.append)] == ["new_run", "session", "mark_done"]
    assert told == [f"{tmp_path / 'events.jsonl'}: skipped 1 unreadable line"]


def test_a_robot_is_recorded_by_its_source_urdf(process, tmp_path):
    """The URDF hash is of the file in the data folder, not of the model's copy with this PC's mesh paths."""
    source, copy = tmp_path / "source.urdf", tmp_path / "copy.urdf"
    source.write_text("<robot name='fake'/>")
    copy.write_text("<robot name='fake'><!-- /home/someone/meshes --></robot>")
    robots = (SimpleNamespace(serial="a200-0806", model=SimpleNamespace(urdf=copy), source_urdf=source),)
    run = new_run(process, robots, RunSettings(), now=NOW)
    assert run.info.robots[CINDY]["urdf_sha256"] == sha256(source.read_bytes()).hexdigest()


def test_a_carried_movement_the_action_lacks_becomes_its_first_step(process, robots, tmp_path):
    """Continuing on an export whose action lost the saved movement: its first step, said in the first event."""
    run = new_run(process, robots, RunSettings(), now=NOW)
    save_progress(replace(run, progress=replace(run.progress, current=("B1_H_hold", 1))))
    base = build_process(tmp_path)
    shorter = with_action(base, "B1_H_hold", movements=base.actions["B1_H_hold"].movements[:1])
    again = write(shorter, process.folder, overwrite=True)
    carried = new_run(again, robots, RunSettings(), continued_from=read_run(run.folder), now=NOW.replace(hour=10))
    assert carried.progress.current == ("B1_H_hold", 0)
    assert read_events(carried.folder)[0]["current_moved"] == [["B1_H_hold", 1], ["B1_H_hold", 0]]
