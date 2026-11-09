"""The observer's recordings in Rerun: what a time range becomes (`rerun_view.py`)."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time

import numpy as np
import pytest

from husky_assembly_teleop.plugins.assembly_observer import rerun_view
from husky_assembly_teleop.plugins.assembly_observer.export import copied_sessions, copy_range
from husky_assembly_teleop.plugins.assembly_observer.rerun_view import _entity, _state_changes, _text_line, log_range

from repo_paths import DATA, REPO

pytest.importorskip("rerun")

#: The session's ROS time at its start, and wall time minus ROS time.
ROS_START, OFFSET = 1_000.0, 1_794_000_000.0


@pytest.fixture
def session(tmp_path):
    """A session of one robot with one arm: 100 ticks at 20 Hz, a lease taken and released, an arm move, a log line."""
    folder = tmp_path / "20261109" / "20261109_100000_session"
    folder.mkdir(parents=True)
    robots = {"a200-0806": {"arms": {"left_ur_arm": "ScaffoldingV3"}}}
    (folder / "experiment.json").write_text(json.dumps({"kind": "observer session", "ros_time": ROS_START,
                                                        "wall_time": ROS_START + OFFSET, "robots": robots}))
    t = ROS_START + 0.05 * np.arange(100)
    pose = np.tile([1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0], (100, 1)).astype(np.float32)
    pose[40:50] = np.nan  # * mocap lost for half a second
    np.savez_compressed(folder / "chunk_001.npz", t=t,
                        **{"0806_base_pose": pose, "0806_base_pose.labels": np.array(["x", "y", "z", "qx", "qy", "qz",
                                                                                      "qw"]),
                           "0806_base_pose.units": np.array(["m"] * 3 + [""] * 4),
                           "0806_left_ur_arm_wrench_peak": np.ones((100, 2), np.float32),
                           "0806_left_ur_arm_wrench_peak.labels": np.array(["force", "torque"]),
                           "0806_left_ur_arm_wrench_peak.units": np.array(["N", "N m"])})
    events = [
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
