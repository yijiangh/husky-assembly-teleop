"""The observer's recordings in Rerun: what a time range becomes (`rerun_view.py`)."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from importlib.util import find_spec

import numpy as np
import pytest

from husky_assembly_teleop.plugins.assembly_observer import rerun_view
from husky_assembly_teleop.plugins.assembly_observer.export import copied_sessions, copy_range
from husky_assembly_teleop.plugins.assembly_observer.rerun_view import (
    LATE_TICKS, SINCE_START, TIMELINE, _entity, _entries, _held, _run_events, _state_changes, _target_error,
    _text_line, _yaw, log_range)
from husky_assembly_teleop.shared.runs.runs import record_event

from repo_paths import DATA, REPO

# ! Not `pytest.importorskip`: with the ROS pytest plugins it skips the whole session, not this module.
pytestmark = pytest.mark.skipif(find_spec("rerun") is None, reason="rerun-sdk not installed")

#: The session's ROS time at its start, and wall time minus ROS time.
ROS_START, OFFSET = 1_000.0, 1_794_000_000.0


@pytest.fixture
def session(tmp_path):
    """A session of one robot with one arm: 100 ticks at 20 Hz, a lease taken and released, an arm move, a log line.

    Before its first states, the process loaded.
    """
    folder = tmp_path / "20261109" / "20261109_100000_session"
    folder.mkdir(parents=True)
    robots = {"a200-0806": {"arms": {"left_ur_arm": "ScaffoldingV3"}}}
    (folder / "experiment.json").write_text(json.dumps({"kind": "observer session", "ros_time": ROS_START,
                                                        "wall_time": ROS_START + OFFSET, "robots": robots}))
    t = ROS_START + 0.05 * np.arange(100)
    pose = np.tile([1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0], (100, 1)).astype(np.float32)
    pose[40:50] = np.nan  # * mocap lost for half a second
    tcp = np.tile([0.3, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0], (100, 1)).astype(np.float32)
    target = np.full((100, 7), np.nan, np.float32)
    target[30:] = [0.3, 0.004, 0.5, 1.0, 0.0, 0.0, 0.0]  # * the compliance controller holds a target from 1.5 s
    labels = np.array(["x", "y", "z", "qx", "qy", "qz", "qw"])
    np.savez_compressed(folder / "chunk_001.npz", t=t,
                        **{"0806_base_pose": pose, "0806_base_pose.labels": np.array(["x", "y", "z", "qx", "qy", "qz",
                                                                                      "qw"]),
                           "0806_base_pose.units": np.array(["m"] * 3 + [""] * 4),
                           "0806_left_ur_arm_tcp": tcp, "0806_left_ur_arm_tcp.labels": labels,
                           "0806_left_ur_arm_target": target, "0806_left_ur_arm_target.labels": labels,
                           "0806_left_ur_arm_wrench_peak": np.ones((100, 2), np.float32),
                           "0806_left_ur_arm_wrench_peak.labels": np.array(["force", "torque"]),
                           "0806_left_ur_arm_wrench_peak.units": np.array(["N", "N m"])})
    events = [
        {"event": "log", "ros_time": ROS_START - 0.5, "level": "INFO", "source": "process",
         "text": f"loaded {tmp_path / 'process'} in 1.0 s; 1 actions"},
        {"event": "state", "ros_time": ROS_START, "robot": "a200-0806", "status": {"lease": None, "e-stop": False},
         "moving": {"base": False, "left_ur_arm": False}},
        {"event": "lease taken", "ros_time": ROS_START + 0.5, "robot": "a200-0806", "holder": "guide"},
        {"event": "left_ur_arm moving", "ros_time": ROS_START + 2.0, "robot": "a200-0806", "arm": "left_ur_arm"},
        {"event": "log", "ros_time": ROS_START + 3.0, "level": "WARN", "source": "guide", "text": "slow plan"},
        {"event": "lease released", "ros_time": ROS_START + 9.0, "robot": "a200-0806", "holder": "guide"},
    ]
    (folder / "events.jsonl").write_text("".join(json.dumps(event) + "\n" for event in events) + '{"cut by a cr')
    for path in folder.iterdir():  # * a session ends at its last write (`sessions`): this one at 5 s
        os.utime(path, (ROS_START + OFFSET + 5.0,) * 2)
    return tmp_path


def test_a_range_is_saved_with_its_ticks_and_events_but_not_the_snapshots(session, tmp_path):
    """Only the range's ticks and events are logged; the states before it only set where things stood."""
    saved = tmp_path / "range.rrd"
    shown = log_range(session, ROS_START + OFFSET + 1.0, ROS_START + OFFSET + 4.0, save=saved)
    assert shown["sessions"] == ["20261109_100000_session"]
    assert shown["rows"] == 61  # * ticks 20 to 80: 1 s to 4 s at 20 Hz, both ends in
    assert shown["events"] == 2  # * the arm move and the log line; the lease before the range is a state, not a line
    assert shown["notes"] == ["no data folder: no robot models, arms not placed in 3D"]
    assert saved.stat().st_size > 0


def test_a_signal_is_one_entity_on_both_timelines_and_held_values_are_logged_twice(session, tmp_path):
    """One plot entity per signal (a pose: position, yaw) on both timelines; a constant run keeps its two ends."""
    from rerun.chunk import RrdReader

    saved = tmp_path / "range.rrd"
    start = ROS_START + OFFSET + 1.0
    log_range(session, start, ROS_START + OFFSET + 4.0, save=saved)
    reader = RrdReader(saved)
    rows: dict[str, list] = {}
    for chunk in reader.stream(store=reader.recordings()[0]).to_chunks():
        batch = chunk.to_record_batch()
        if "Scalars:scalars" in batch.schema.names:
            assert {TIMELINE, SINCE_START} <= set(chunk.timeline_names)
            rows.setdefault(chunk.entity_path, []).extend(zip(batch.column(SINCE_START).to_pylist(),
                                                              batch.column("Scalars:scalars").to_pylist()))
    assert set(rows) == {"/0806/base_pose/position", "/0806/base_pose/yaw", "/0806/left_ur_arm/tcp/position",
                         "/0806/left_ur_arm/target/position", "/0806/left_ur_arm/target_error",
                         "/0806/left_ur_arm/wrench_peak"}
    since, values = zip(*rows["/0806/left_ur_arm/wrench_peak"])
    assert [value for value in values] == [[1.0, 1.0]] * 2 and since[0].total_seconds() == 0.0
    assert since[-1].total_seconds() == pytest.approx(3.0)
    # * Unknown before 1.5 s, then 4 mm and no angle: each run is its first and last tick.
    errors = [value for _, value in rows["/0806/left_ur_arm/target_error"]]
    assert np.isnan(errors[0]).all() and errors[-1] == pytest.approx([4.0, 0.0], abs=1e-3) and len(errors) == 4


def test_yaw_and_tool0_error_come_from_the_quaternions():
    """Yaw in degrees from (x, y, z, w); tool0's error is a distance in mm and a rotation angle, q and -q alike."""
    half = np.radians(10.0) / 2
    turned = [0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4)]
    assert _yaw(np.array([turned, [np.nan] * 4])) == pytest.approx([90.0, np.nan], nan_ok=True)
    tcp = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]] * 3)
    target = np.array([[0.003, 0.004, 0.0, np.sin(half), 0.0, 0.0, np.cos(half)],
                       [0.0, 0.0, 0.0, -np.sin(half), 0.0, 0.0, -np.cos(half)], [np.nan] * 7])
    assert _target_error(tcp, target) == pytest.approx(np.array([[5.0, 10.0], [0.0, 10.0], [np.nan, np.nan]]),
                                                       nan_ok=True)


def test_only_ticks_inside_a_constant_run_are_left_out():
    """A row equal to both neighbours (NaN equal to NaN) is held; the ends of every run stay."""
    values = np.array([1.0, 1.0, 1.0, 2.0, np.nan, np.nan, np.nan])[:, None]
    assert _held(values).tolist() == [False, True, False, False, False, True, False]


def test_with_the_data_folder_each_robot_gets_its_urdf_model(session, tmp_path):
    """Cindy's URDF loads, so nothing is noted and the file holds her meshes (a few MB, not just the ticks)."""
    saved = tmp_path / "range.rrd"
    shown = log_range(session, ROS_START + OFFSET + 1.0, ROS_START + OFFSET + 4.0, save=saved, data=DATA)
    assert shown["notes"] == []
    assert saved.stat().st_size > 1_000_000


def test_no_session_in_the_range_says_so(session):
    """A range without recordings logs nothing and says why."""
    shown = log_range(session, ROS_START + OFFSET + 3600.0, ROS_START + OFFSET + 7200.0, spawn=False)
    assert shown["sessions"] == [] and "no recordings" in shown["notes"][0]


def test_tick_signals_land_under_their_robot_arm_or_object(session):
    """Signal keys map to /<serial>[/<arm>]/<signal>, objects/<name>/<what> or lab/<key>."""
    found = rerun_view._sessions(session, ROS_START + OFFSET, ROS_START + OFFSET + 5.0)
    assert _entity("0806_left_ur_arm_wrench_peak", found[0]) == ("0806/left_ur_arm/wrench_peak", "a200-0806",
                                                                 "left_ur_arm")
    assert _entity("0806_base_pose", found[0]) == ("0806/base_pose", "a200-0806", None)
    assert _entity("object_fitted_bar_B20_pose", found[0]) == ("objects/fitted_bar_B20/pose", None, None)
    assert _entity("relay_drift", found[0]) == ("lab/relay_drift", None, None)


def test_states_come_from_snapshots_and_switch_events_alike():
    """Snapshots, value events, leases and motion events all feed the same state timelines."""
    snapshot = {"event": "state", "robot": "a200-0806", "status": {"lease": None, "e-stop": False},
                "flags": {"mocap tracking": "lost"}, "moving": {"base": True}}
    assert _state_changes(snapshot) == {"states/0806/lease": "none", "states/0806/e-stop": "false",
                                        "states/0806/mocap_tracking": "lost", "states/0806/base_motion": "moving"}
    assert _state_changes({"event": "lease taken", "robot": "a200-0806", "holder": "guide"}) == \
        {"states/0806/lease": "guide"}
    assert _state_changes({"event": "left_ur_arm still", "robot": "a200-0806"}) == \
        {"states/0806/left_ur_arm_motion": "still"}
    assert _state_changes({"event": "relay frame status", "value": "held", "was": None}) == \
        {"states/lab/relay_frame_status": "held"}
    assert _state_changes({"event": "path sent", "robot": "a200-0806", "waypoints": 3}) == {}


def test_stops_and_lost_streams_are_warnings_and_log_lines_keep_their_level():
    """Stops and lost streams are WARN; log lines keep their own level and go under their source."""
    assert _text_line({"event": "soft stop", "robot": "a200-0806", "value": "too close"})[1] == "WARN"
    assert _text_line({"event": "base stream", "robot": "a200-0806", "value": "lost"})[1] == "WARN"
    assert _text_line({"event": "base stream", "robot": "a200-0806", "value": "back"})[1] == "INFO"
    assert _text_line({"event": "log", "level": "ERROR", "source": "guide", "text": "x"}) == \
        ("log/guide", "ERROR", "[guide] x")


@pytest.fixture
def copy(session, tmp_path):
    """The session's 1 s to 4 s copied as the panel's Copy to Drive does: `<name>/<stamp>/<session>/`."""
    # * A copy looks for the session under its day's folder: the fixture's is named for another day.
    (session / "20261109").rename(session / time.strftime("%Y%m%d", time.localtime(ROS_START + OFFSET)))
    destination = tmp_path / "drive" / "b20_insert" / "20261109_120000"
    destination.mkdir(parents=True)
    start, end = ROS_START + OFFSET + 1.0, ROS_START + OFFSET + 4.0
    copied = copy_range(session, start, end, destination, {"name": "b20_insert"})
    assert list(copied) == ["20261109_100000_session"]
    return destination


def test_a_copy_is_found_from_its_folder_a_session_of_it_or_a_folder_of_copies(copy):
    """A copy's sessions span the range copied, not their last write (the copy's), whichever folder is given."""
    for folder in (copy, copy / "20261109_100000_session", copy.parent):
        found = copied_sessions(folder)
        assert [session["folder"].name for session in found] == ["20261109_100000_session"], folder
        assert (found[0]["start"], found[0]["end"]) == (ROS_START + OFFSET + 1.0, ROS_START + OFFSET + 4.0)
    assert copied_sessions(copy / "20261109_100000_session" / "commands") == []


def test_a_folder_of_copies_holds_them_by_name_and_stamp(copy):
    """The copies' root, `<to>/<name>/<stamp>/<session>`, finds them too."""
    found = copied_sessions(copy.parent.parent)
    assert [session["folder"].name for session in found] == ["20261109_100000_session"]


def test_a_copy_keeps_the_process_loaded_before_its_states(copy, tmp_path):
    """The process's "loaded" line, older than the newest states before the range, is copied: runs and bars need it."""
    lines = [json.loads(line) for line in (copy / "20261109_100000_session" / "events.jsonl").read_text().splitlines()]
    assert [line["event"] for line in lines[:2]] == ["log", "state"] and lines[0]["before_range"]
    shown = log_range(copy, ROS_START + OFFSET + 1.0, ROS_START + OFFSET + 4.0, save=tmp_path / "copy.rrd", copy=True)
    assert not [note for note in shown["notes"] if "no process loaded" in note]


def test_a_copy_opens_whole_from_the_script_without_a_range(copy, tmp_path):
    """`observer_logs.py rerun --folder <copy>` shows every tick and event copied; a start and end narrow it."""
    def call(*args: str) -> dict:
        done = subprocess.run([sys.executable, str(REPO / "scripts" / "observer_logs.py"), "rerun", "--folder",
                               str(copy), "--save", str(tmp_path / "copy.rrd"), *args], capture_output=True, text=True)
        return json.loads(done.stdout.splitlines()[-1])

    whole = call()
    assert whole["sessions"] == ["20261109_100000_session"] and whole["rows"] == 61 and whole["events"] == 2
    assert (tmp_path / "copy.rrd").stat().st_size > 0
    narrowed = call(time.strftime("%H:%M:%S", time.localtime(ROS_START + OFFSET + 3.0)))
    assert narrowed["rows"] == 21 and narrowed["events"] == 1  # * 3 s to 4 s: the log line at 3 s only


def test_common_events_read_as_short_sentences():
    """Moves, leases, stops and log lines read as short lines; an unknown kind shows its fields, long ones cut."""
    by = "movement_exec: exec B20_J_transfer: transfer"
    moving = {"event": "arm moving", "robot": "a200-0806", "arm": "left_ur_arm", "by": by, "commanded": True}
    assert _text_line(moving)[2] == "0806 left_ur_arm moving by movement_exec: B20_J_transfer"
    assert _text_line({"event": "lease taken", "robot": "a200-0806", "holder": "guide"})[2] == "0806 lease → guide"
    assert _text_line({"event": "soft stop", "robot": "a200-0806", "value": "too close", "was": None}) == \
        ("events/0806", "WARN", "0806 soft stop: too close")
    assert _text_line({"event": "base still", "robot": "a200-0804", "seconds": 30.72, "distance": 2.13, "turn": 1.0,
                       "commanded": True, "pose": [0.0] * 7})[2] == "0804 base still after 30.7 s, 2.13 m, 57°"
    assert _text_line({"event": "relay frame status", "value": "held", "was": None}) == \
        ("events/lab", "INFO", "relay frame status → held")
    late = {"event": "log", "level": "WARN", "source": "ticks", "text": "tick 3 started 50 ms late\n  more"}
    assert _text_line(late) == ("log/ticks", "WARN", "[ticks] tick 3 started 50 ms late …")
    unknown = _text_line({"event": "new thing", "robot": "a200-0806", "n": 3, "note": "x" * 100})[2]
    assert unknown.startswith("0806 new thing: n=3, note=xxx") and unknown.endswith("…")
    assert _text_line({"event": "state", "robot": "a200-0806", "status": {}}) is None


def test_late_ticks_become_a_state_and_other_repeats_one_line_with_a_count():
    """Late-tick reports switch a state with a line at each switch; a repeating log line shows once, then its count."""
    def log(at, source, text, count=1):
        return {"event": "log", "ros_time": at, "level": "WARN", "source": source, "text": text, "count": count}
    events = [log(t, "ticks", f"tick {int(t)} started {t:.0f} ms late") for t in (0.0, 5.0, 10.0, 40.0)] + \
        [log(1.0, "guide", "slow plan 1.2 s"), log(2.0, "guide", "slow plan 3.4 s"),
         log(2.5, "guide", "slow plan 3.4 s", count=3), log(20.0, "guide", "slow plan 9 s")]
    entries = list(_entries(sorted(events, key=lambda event: event["ros_time"]), 0.0))
    late = [(at, changes[LATE_TICKS]) for at, changes, _ in entries if LATE_TICKS in changes]
    assert late == [(0.0, "late"), (25.0, "ok"), (40.0, "late"), (55.0, "ok")]
    lines = [(at, line[2]) for at, _, line in entries if line is not None and line[0] == "log/guide"]
    assert lines == [(1.0, "[guide] slow plan 1.2 s"), (2.5, "[guide] slow plan 3.4 s (×3 more)"),
                     (20.0, "[guide] slow plan 9 s")]


def test_a_flood_across_the_range_shows_its_first_line_inside_and_its_count_up_to_the_end():
    """Lines before the range never hide those in it; a flood still going at the range's end is counted up to it."""
    flood = [{"event": "log", "ros_time": float(t), "level": "WARN", "source": "relay", "text": f"gap {t % 7} frames"}
             for t in range(1000, 2200)]

    def lines(start, end):
        return [(at, line[2]) for at, _, line in _entries(flood, 0.0, start, end)
                if line is not None and start <= at <= end]
    assert lines(1300.0, 1900.0) == [(1300.0, "[relay] gap 5 frames"), (1900.0, "[relay] gap 3 frames (×600 more)")]
    assert lines(900.0, 1500.0) == [(1000.0, "[relay] gap 6 frames"), (1500.0, "[relay] gap 2 frames (×500 more)")]


def test_a_run_of_the_loaded_process_adds_its_steps_and_their_outcomes(tmp_path):
    """The runs under a process the session loaded give step states and lines; their events outside it are left out."""
    run = tmp_path / "process" / "runs" / "20261109_100000"
    run.mkdir(parents=True)
    record_event(run, "plan", ROS_START - 50.0, action="B20_J", movement="B20_J_transfer", data={"waypoints": 3})
    record_event(run, "guide", ROS_START + 1.0, action="B20_J", movement="B20_J_transfer",
                 data={"event": "start", "part": "B20_J_transfer: transfer", "kind": "move", "mode": "God"})
    record_event(run, "exec", ROS_START + 2.0, action="B20_J", movement="B20_J_transfer",
                 data={"status": "FAILED", "result": "failed"})
    record_event(run, "mark_done", ROS_START + 3.0, action="B20_J", assumed=False)
    events = [{"event": "session start", "ros_time": ROS_START},
              {"event": "log", "ros_time": ROS_START + 0.5, "level": "INFO", "source": "process",
               "text": f"loaded {tmp_path / 'process'} in 1.0 s: read process 0.2 s; 3 actions"},
              {"event": "session end", "ros_time": ROS_START + 4.0}]
    found = _run_events(events)
    assert [event["event"] for event in found] == ["run guide", "run exec", "run mark_done"]
    entries = list(_entries(sorted(events + found, key=lambda event: event["ros_time"]), 0.0))
    assert entries[2][1] == {"states/run/run": "20261109_100000", "states/run/action": "B20_J",
                             "states/run/step": "B20_J_transfer", "states/run/guide_mode": "God",
                             "states/run/guide": "B20_J_transfer: transfer"}
    assert entries[2][2] is None  # * the guide's start and done are only states
    assert entries[3][2] == ("events/run", "WARN", "B20_J_transfer failed")
    assert entries[4][1]["states/run/action"] == "B20_J done" and entries[4][2][2] == "B20_J done"


#: A Cindy trajectory's joints at its end, in the order the trajectory files list them.
UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
             "wrist_3_joint"]


@pytest.fixture
def sent(session):
    """The session plus a path sent at 1 s and followed until 3 s, a 2 s trajectory at 1.5 s and a trip at 2 s."""
    folder = session / "20261109" / "20261109_100000_session"
    (folder / "commands").mkdir()
    (folder / "commands" / "00001_path_sent_a200-0806.json").write_text(json.dumps(
        {"path": {"header": {"frame_id": "mocap_world"}, "x": [0.0, 1.0, 2.0], "y": [0.0, 0.5, 0.5],
                  "yaw": [0.0, 0.0, 0.0], "t": []}}))
    (folder / "commands" / "00002_trajectory_sent_a200-0806_left_ur_arm.json").write_text(json.dumps(
        {"joint_names": UR_JOINTS, "points": [{"positions": [0.0] * 6, "time_from_start": {"sec": 0, "nanosec": 0}},
                                              {"positions": [0.5] * 6,
                                               "time_from_start": {"sec": 2, "nanosec": 0}}]}))
    hit = {"robot": "robots/a200-0806", "where": "move", "margin": 0.01,
           "pairs": [{"a": "robots/a200-0806/left_ur_arm_wrist_3_link", "b": "lab/boxes/main_pc_table",
                      "contact": {"point_a": [1.0, 1.0, 0.8], "point_b": [1.0, 1.05, 0.8], "depth": 0.004}}]}
    events = [json.loads(line) for line in (folder / "events.jsonl").read_text().splitlines()[:-1]] + [
        {"event": "path sent", "ros_time": ROS_START + 1.0, "robot": "a200-0806", "by": "base_planner",
         "file": "commands/00001_path_sent_a200-0806.json"},
        {"event": "follower", "ros_time": ROS_START + 1.1, "robot": "a200-0806", "value": "following"},
        {"event": "trajectory sent", "ros_time": ROS_START + 1.5, "robot": "a200-0806", "arm": "left_ur_arm",
         "file": "commands/00002_trajectory_sent_a200-0806_left_ur_arm.json"},
        {"event": "clearance trip", "ros_time": ROS_START + 2.0, "robot": "a200-0806", "task": "x", "problem": "y",
         "hit": hit},
        {"event": "follower", "ros_time": ROS_START + 3.0, "robot": "a200-0806", "value": "done"},
        {"event": "log", "ros_time": ROS_START + 0.2, "level": "INFO", "source": "process",
         "text": f"loaded {session / 'process'} in 1.0 s; 1 actions, 2 movements"},
    ]
    (folder / "events.jsonl").write_text("".join(json.dumps(event) + "\n" for event in events))
    os.utime(folder / "events.jsonl", (ROS_START + OFFSET + 5.0,) * 2)
    return session


def _entities(path) -> set[str]:
    """The entity paths a saved recording holds."""
    from rerun.chunk import RrdReader
    return {chunk.entity_path for chunk in RrdReader(path).stream()}


def test_sent_paths_trajectories_and_trips_show_while_they_last(sent):
    """A path shows from its send until its follower is done, a ghost until its last point, a trip for 2 s."""
    found = rerun_view._sessions(sent, ROS_START + OFFSET, ROS_START + OFFSET + 5.0)
    shown = {item.entity: item for item in rerun_view._sent(found[0], ROS_START + OFFSET, ROS_START + OFFSET + 5.0)}
    wall = ROS_START + OFFSET
    assert set(shown) == {"world/plans/0806", "ghost_0806/left_ur_arm", "world/trips/0806"}
    assert (shown["world/plans/0806"].start, shown["world/plans/0806"].end) == (wall + 1.0, wall + 3.0)
    assert np.allclose(shown["world/plans/0806"].what[0][:, :2], [[0.0, 0.0], [1.0, 0.5], [2.0, 0.5]])
    assert shown["ghost_0806/left_ur_arm"].end == pytest.approx(wall + 3.5)
    assert shown["ghost_0806/left_ur_arm"].what == {f"left_ur_arm_{joint}": 0.5 for joint in UR_JOINTS}
    assert shown["world/trips/0806"].labels == ["left_ur_arm_wrist_3_link / main_pc_table"]


def test_a_thing_replaced_before_its_end_keeps_no_end():
    """The next send on the same entity replaces the last one; only those overlapping the range are kept."""
    first, second, early = (rerun_view.Shown("world/plans/0806", 1.0, 5.0, None),
                            rerun_view.Shown("world/plans/0806", 3.0, None, None),
                            rerun_view.Shown("world/plans/0804", 0.0, 0.5, None))
    assert rerun_view._in_range([second, first, early], 1.0, 10.0) == [first, second]
    assert first.end is None


def test_the_scene_has_the_lab_the_sent_paths_the_arm_ghost_and_the_trip(sent, tmp_path):
    """With the data folder the lab's obstacles, the ghost's arm joints, the path and the trip line are logged."""
    saved = tmp_path / "range.rrd"
    shown = log_range(sent, ROS_START + OFFSET + 0.5, ROS_START + OFFSET + 4.5, save=saved, data=DATA)
    assert not [note for note in shown["notes"] if "ghost" in note or "lab" in note]
    entities = _entities(saved)
    assert {"/world/lab/main_pc_table/0", "/world/lab/border_x_min/0", "/world/plans/0806", "/world/trips/0806",
            "/ghost_0806/joints/left_ur_arm_shoulder_pan_joint"} <= entities
    assert not any(path.startswith("/ghost_0806/joints/right_ur_arm") for path in entities)  # * no trajectory
    assert not any("collision_geometries" in path or "wheel" in path for path in entities if
                   path.startswith("/ghost_"))


def test_a_tracked_bar_gets_its_process_shape_and_its_halves(sent, tmp_path, monkeypatch):
    """The bar of a fitted_bar object comes from the process the log says was loaded; others are noted."""
    import rerun as rr
    from types import SimpleNamespace
    from scaffolding_assembly_core.geometry import Pose, box_geometry, cylinder_geometry
    bodies = {"bars/B20": SimpleNamespace(assembly_pose=Pose((1.0, 0.0, 0.0)), geometry=cylinder_geometry(0.01, 1.0)),
              "joints/J20": SimpleNamespace(assembly_pose=Pose((1.0, 0.0, 0.2)), geometry=box_geometry((0.05,) * 3))}
    process = SimpleNamespace(bodies=bodies, halves_of=lambda bar: ("joints/J20",) if bar == "bars/B20" else ())
    folders = []
    monkeypatch.setattr(rerun_view, "read", lambda folder: folders.append(folder) or process)
    found = rerun_view._sessions(sent, ROS_START + OFFSET, ROS_START + OFFSET + 5.0)
    saved, notes = tmp_path / "bars.rrd", []
    rr.init("test_bars")
    rr.save(saved)
    rerun_view._log_scene(rr, found, None, ["fitted_bar_B20", "fitted_bar_B99"], ROS_START + OFFSET,
                          ROS_START + OFFSET + 5.0, notes)
    rr.get_global_data_recording().flush()
    assert folders == [sent / "process"]
    assert notes == ["not in the process, axes only: fitted_bar_B99"]
    assert {"/world/objects/fitted_bar_B20/bar/0", "/world/objects/fitted_bar_B20/bar/joints_J20/0"} <= \
        _entities(saved)
    # * Hidden from the range's start until its pose is known: else it stands at the lab's origin.
    from rerun.chunk import RrdReader
    scales = [(chunk.timeline_names, chunk.to_record_batch().column("Transform3D:scale").to_pylist())
              for chunk in RrdReader(saved).stream() if chunk.entity_path == "/world/objects/fitted_bar_B20/bar"
              and "Transform3D:scale" in chunk.to_record_batch().schema.names]
    assert len(scales) == 1 and {TIMELINE, SINCE_START} <= set(scales[0][0])
    assert scales[0][1] == [[[pytest.approx(rerun_view.HIDDEN_SCALE)] * 3]]


def test_the_objects_mocap_plot_lists_each_object():
    """Rerun takes a `*` only as a trailing `/**`: the objects' mocap plot names each object's entity."""
    import rerun.blueprint as rrb

    def views(item):
        if isinstance(item, rrb.View):
            yield item
        for inner in getattr(item, "contents", None) or () if not isinstance(item, rrb.View) else ():
            yield from views(inner)
    blueprint = rerun_view._blueprint([], ["fitted_bar_B20", "pipe"], {}, {})
    plot = next(view for view in views(blueprint.root_container) if view.name == "objects mocap")
    assert plot.contents == ["+ $origin/fitted_bar_B20/mocap", "+ $origin/pipe/mocap"]


def test_a_hold_ends_its_arms_ghost_and_the_session_end_ends_a_path_still_followed(sent):
    """A hold cuts a trajectory short, so its ghost goes; a path followed when the session ended ends with it."""
    folder = sent / "20261109" / "20261109_100000_session"
    events = [event for event in map(json.loads, (folder / "events.jsonl").read_text().splitlines())
              if not (event["event"] == "follower" and event["value"] == "done")]
    events.append({"event": "hold sent", "ros_time": ROS_START + 2.5, "robot": "a200-0806", "arm": "left_ur_arm",
                   "duration": 0.2, "waypoints": 1})
    (folder / "events.jsonl").write_text("".join(json.dumps(event) + "\n" for event in events))
    for path in [folder, *folder.rglob("*")]:  # * the session's end is its last write: 5 s
        os.utime(path, (ROS_START + OFFSET + 5.0,) * 2)
    wall = ROS_START + OFFSET
    found = rerun_view._sessions(sent, wall, wall + 10.0)
    shown = {item.entity: item for item in rerun_view._sent(found[0], wall, wall + 10.0)}
    assert shown["ghost_0806/left_ur_arm"].end == pytest.approx(wall + 2.5)
    assert shown["world/plans/0806"].end == pytest.approx(wall + 5.0)
