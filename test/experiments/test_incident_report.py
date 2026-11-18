"""The incident report: the real line forms classified, bursts merged, a log's end read, and the page written."""

from __future__ import annotations

from datetime import datetime
from os import utime
from pathlib import Path

import pytest

from husky_assembly_teleop.experiments.incidents.classify import (ALL_ROBOTS, MONITOR, Incident, LogLine, add_events,
                                                                  classify_monitor, classify_pane, count_matrix, clean,
                                                                  events_incidents, log_ending, merge_bursts,
                                                                  monitor_incidents, pane_incidents, read_log)
from husky_assembly_teleop.experiments.incidents.page import build_report, read_incidents, render

STAMP = datetime(2026, 11, 4, 15, 0, 0).timestamp()


def line(text: str, level: str = "WARN", t: float = STAMP) -> LogLine:
    """A stamped monitor line as `read_log` gives it."""
    return LogLine(t, level, text, f"[{level}] [{t}] [husky_monitor]: {text}", "monitor_x.log")


@pytest.mark.parametrize("text, kind, robot", [
    ("[movement_exec] Exec B20_J_transfer: failed — B20_J_transfer: ran all 8 passes without converging", "step",
     None),
    ("[movement_exec] Exec B20_J_insert: cancelled — B20_J_insert: cancelled (Stop all (the soft stop))", "step", None),
    ("[movement_exec] Exec B21_H_approach: refused — B21_H_approach: alice (a200-0804) ur_arm: the arm is not "
     "operational: safety mode ROBOT_EMERGENCY_STOP", "step", "Alice"),
    ("[movement_exec] a200-0806: insert: the arms did NOT reach the line's end: left_ur_arm 3.5 of 4.8 mm", "step",
     "Cindy"),
    ("[movement_exec] a200-0806: retreat: check FAIL grippers: left_ur_arm LOOSENING", "gripper", "Cindy"),
    ("[movement_exec] ### SERVOING stopped at iter 1: a200-0804 ur_arm moved within the last 2 s", "servo", "Alice"),
    ("[movement_exec] ### SERVOING ran all 8 iterations without converging.", "servo", None),
    ("[movement_exec] [as-built] B20_J_insert start: B20 fit FAILED (the marker snapshot is 1824.9 s old) | no ref",
     "mocap", None),
    ("[movement_exec] [servo] no marker sample of bars/B20 (asset 1020) within 1 s: is Motive streaming it?", "mocap",
     None),
    ("SOFT STOP: stopping every robot and cancelling the plugin tasks that may move one", "stop_all", ALL_ROBOTS),
    ("robot a200-0804: SOFT STOP: Stop all", "stop_all", ALL_ROBOTS),
    ("robot a200-0806: SOFT STOP: robot a200-0806 came within 10 cm of main pc table", "clearance", "Cindy"),
    ("[robot_control] task 'robot_control: clearance a200-0806': robot a200-0806 came within 10 cm of floor at its "
     "start; refused, nothing sent", "clearance", "Cindy"),
    ("robot a200-0805: cannot stop arms, MultiArmSafetySync not reachable", "no_answer", "Belle"),
    ("/a200_0805: switch_controller is not available, cannot stop", "no_answer", "Belle"),
    ("/a200_0806/right_ur5e: not starting free_drive_controller: robot a200-0806 is soft stopped; press Resume first",
     "controller", "Cindy"),
    ("arm right_ur_arm: not sending a trajectory: scaled_joint_trajectory_controller is not active", "controller", "–"),
    ("[assembly_steps] B20_J_transfer: no goal at the live base: no IK for the goal (ssik)", "plan", "–"),
    ("HOLD NOT SENT on left_ur_arm: stop a200-0806 with the teach pendant or the e-stop (why)", "hold", "Cindy"),
    ("[health/chips] a200-0804 arm: arm ROBOT EMERGENCY STOP green -> red: sync: last stop: ur5e: safety mode "
     "ROBOT_EMERGENCY_STOP", "estop", "Alice"),
])
def test_the_real_monitor_lines_get_their_kind_and_robot(text, kind, robot):
    """Each line form of the logs: its kind, and its robot from its words or from the Exec that ran."""
    found = classify_monitor(line(text))
    assert found is not None and found[0] == kind
    if robot is not None:
        assert found[1] == robot


@pytest.mark.parametrize("text", [
    "[health/chips] a200-0804 base: signal green -> amber: a gap over 200 ms in the joint_states",
    "[health/chips] objects: fitted_bar/B20 flickered amber (3 changes), green again",
    "[health/chips] a200-0804 arm: arm NO PROGRAM green -> red (2 changes): sync: last stop: stopped by operator",
    "[health/chips] a200-0804 arm: arm red -> green: sync: last stop: ur5e: safety mode ROBOT_EMERGENCY_STOP",
    "[ticks] tick 2 started 72 ms late; the 122 ms since the previous tick started went to:",
    "[movement_exec] Exec B20_J_transfer: done — every part done",
    "[assembly_run] New run refused: a200-0806 is leased by movement_exec: exec B20_J_insert: wait until it is done",
    "[movement_exec] waiting for Confirm: cindy (a200-0806): B20_J_insert — SERVO by bar: no usable mocap fit of B20",
    "[process] a process robot differs from the configured one: robots/alice (a200-0804): arm base frames turn",
])
def test_chip_flicker_and_ordinary_lines_are_no_incidents(text):
    """A chip's change other than a stop, a late tick, a step done, a click refused."""
    assert classify_monitor(line(text)) is None


def test_a_step_belongs_to_the_robot_it_ran_on_not_the_one_its_reason_names():
    """The reason names Alice; the Exec before it ran on Cindy."""
    lines = [line("[movement_exec] Exec B20_J_transfer on cindy (a200-0806): B20_J_transfer", "INFO"),
             line("[movement_exec] Exec B20_J_transfer: refused — robots/a200-0804 moved since planning: base 35.0 cm")]
    found = monitor_incidents(lines)
    assert [(item.kind, item.robot) for item in found] == [("step", "Cindy")]
    assert found[0].what.startswith("Exec B20_J_transfer: refused") and found[0].raw == [lines[1].raw]


def test_read_log_cleans_colours_and_a_line_without_a_stamp_takes_the_last_one():
    """Terminal colours gone; times from the stamps; before the first stamp, the start given."""
    text = ("pybullet build time\n\x1b[33m[WARN] [1793800060.5] [husky_monitor]: hello\x1b[0m\r\n\n"
            "ROBOT EMERGENCY STOP: clear it\n"
            "[ur_ros2_control_node-5] [WARN] [1793800070.2] [UR_Client_Library:]: Lost\n"
            "[urscript_interface-9] [1793800071.1] WARN ./include/producer.h 88: Lost connection\n")
    lines = read_log(text, "x.log", started=5.0)
    assert [item.t for item in lines] == [5.0, 1793800060.5, 1793800060.5, 1793800070.2, 1793800071.1]
    assert [item.stamped for item in lines] == [False, True, False, True, True]
    assert lines[1].text == "hello" and lines[3].text == "UR_Client_Library:: Lost"
    assert clean("\x1b[01;32msu@pc\x1b[00m:~$\r") == "su@pc:~$"


def test_a_burst_of_the_same_line_is_one_incident_with_a_count_and_a_time_range():
    """Many identical lines within the gap merge; after the gap a new one starts; raw lines are kept short."""
    text = "/a200_0806/right_ur5e: not starting free_drive_controller: robot a200-0806 is soft stopped"
    many = [Incident("controller", "Cindy", text, STAMP + k, STAMP + k, raw=[f"line {k}"], source="m")
            for k in range(32)]
    later = Incident("controller", "Cindy", text, STAMP + 500, STAMP + 500, raw=["later"], source="m")
    merged = merge_bursts([*many, later])
    assert len(merged) == 2
    assert merged[0].count == 32 and merged[0].start == STAMP and merged[0].end == STAMP + 31
    assert len(merged[0].raw) == 11 and merged[0].dropped == 21
    assert merged[0].raw[0] == "line 0" and merged[0].raw[-1] == "line 31"


def test_different_words_stay_apart_unless_the_kind_merges_any():
    """Two different refusals are two; a numbers-only difference is one; Stop all lines of a burst are one."""
    steps = [Incident("step", "Cindy", "Exec A: refused — first", STAMP, STAMP),
             Incident("step", "Cindy", "Exec A: refused — second", STAMP + 1, STAMP + 1),
             Incident("step", "Cindy", "Exec A: refused — second", STAMP + 2, STAMP + 2)]
    assert [item.count for item in merge_bursts(steps)] == [1, 2]
    numbers = [Incident("mocap", "Cindy", "B20 fit FAILED: 1273.3 s", STAMP, STAMP),
               Incident("mocap", "Cindy", "[servo 0] not usable: 1301.3 s", STAMP + 5, STAMP + 5)]
    assert len(merge_bursts(numbers)) == 1
    stops = [Incident("stop_all", ALL_ROBOTS, "SOFT STOP: stopping", STAMP, STAMP),
             Incident("stop_all", ALL_ROBOTS, "robot a200-0804: SOFT STOP: Stop all", STAMP, STAMP)]
    assert len(merge_bursts(stops)) == 1


def stamped_tail(*texts: str) -> list[LogLine]:
    """Lines as they end a log: stamped ones in brackets, others as printed."""
    out = []
    for text in texts:
        if text.startswith("[INFO] "):
            out.append(line(text[len("[INFO] "):], "INFO"))
        else:
            out.append(LogLine(STAMP, "", text, text, "monitor_x.log", stamped=False))
    return out


def test_a_log_ending_on_the_monitors_shutdown_is_a_stop_by_hand():
    """The stopping line and "Server stopped": marked inferred, as they do not say who stopped it."""
    found = log_ending(stamped_tail("[INFO] stopping: cancelling plugin tasks and tearing down",
                                    "(viser) Server stopped", "(viser) Server stopped"))
    assert [(item.kind, item.robot) for item in found] == [("hand", MONITOR)]
    assert "inferred" in found[0].inferred


def test_a_keyboard_interrupt_in_the_shutdown_is_a_stop_by_hand_not_a_crash():
    """The real form: a traceback from rclpy's shutdown ending in KeyboardInterrupt."""
    found = log_ending(stamped_tail("[INFO] stopping: cancelling plugin tasks and tearing down",
                                    "Traceback (most recent call last):", 'File "<string>", line 6, in <module>',
                                    "rclpy.try_shutdown()", "KeyboardInterrupt"))
    assert [item.kind for item in found] == ["hand"]


def test_a_traceback_or_a_process_death_is_a_crash_with_its_lines():
    """A traceback ending in another error, and the printed death of a process."""
    found = log_ending(stamped_tail("(viser) Server stopped", "Traceback (most recent call last):",
                                    'File "monitor.py", line 676, in main', "asyncio.run(_run_monitor())",
                                    "RuntimeError: Port 8080 is taken: is another monitor running?"))
    assert [item.kind for item in found] == ["crash"]
    assert found[0].what.startswith("RuntimeError: Port 8080") and "Traceback" in found[0].raw[0]
    assert [item.kind for item in log_ending(stamped_tail("Segmentation fault (core dumped)"))] == ["crash"]
    assert [item.kind for item in log_ending(stamped_tail("X connection to :1 broken (explicit kill)"))] == ["crash"]


def test_a_log_ending_in_silence_is_an_inferred_crash_unless_it_is_still_being_written():
    """No stop line at all: a crash, inferred; the same log still changing is a running monitor."""
    lines = stamped_tail("[INFO] [movement_exec] Exec B20_J_insert: done — every part done")
    found = log_ending(lines)
    assert [item.kind for item in found] == ["crash"] and "inferred" in found[0].inferred
    assert log_ending(lines, still_written=True) == []


def test_robot_pane_lines_and_their_kinds():
    """The launch's errors, the sync's safety mode, ssh; a process stopped on purpose and a shutdown's close are not."""
    pane = (lambda text: LogLine(STAMP, "", text, text, "cindy.log", stamped=False))
    sync = "a200_0806.MultiArmSafetySync: ### left_ur5e: safety mode changed: "
    assert classify_pane(pane(sync + "SYSTEM_EMERGENCY_STOP")) == "estop"
    assert classify_pane(pane(sync + "NORMAL")) is None
    tool = "a200_0806.left_gripper.scaffolding_tool_driver: "
    assert classify_pane(pane(tool + "Unexpected response to command 'STATUS'")) == "gripper"
    assert classify_pane(pane("UR_Client_Library:: Lost connection to stream, attempting to reconnect")) == "no_answer"
    assert classify_pane(pane("ssh: connect to host 192.168.0.113 port 22: No route to host")) == "no_answer"
    died = "[ERROR] [ur_ros2_control_node-5]: process has died [pid 1, exit code {}, cmd '/opt/ros/lib/ur_node --args"
    assert classify_pane(pane(died.format(-11))) == "process"
    assert classify_pane(pane(died.format(-2))) is None
    found = pane_incidents([pane("administrator@cpr$ sudo shutdown now"), pane("[sudo] password for administrator:"),
                            pane("Connection to 192.168.0.113 closed by remote host.")], "Cindy")
    assert found == []
    found = pane_incidents([pane(died.format(-11))], "Cindy")
    assert found[0].what == "ur_node died (exit code -11)" and "last stamp" in found[0].inferred


def test_a_runs_events_join_the_logs_incident_or_stand_alone():
    """The exec's end joins the log's line nearby; with no log line it is an incident of its own."""
    events = [{"time": "2026-11-04T15:00:00+01:00", "event": "check_goal", "action": "B20_J",
               "movement": "B20_J_transfer", "data": {"ok": True, "acting": "robots/a200-0806"}},
              {"time": "2026-11-04T15:00:05+01:00", "event": "exec", "action": "B20_J", "movement": "B20_J_transfer",
               "data": {"status": "refused", "result": "refused"}},
              {"time": "2026-11-04T15:00:06+01:00", "event": "exec", "action": "B20_J", "movement": "B20_J_transfer",
               "data": {"status": "ARRIVED", "result": "done"}},
              {"time": "2026-11-04T15:30:00+01:00", "event": "soft_stop", "action": None, "movement": None, "data": {}}]
    found = events_incidents(events, "run1")
    assert [(item.kind, item.robot) for item in found] == [("step", "Cindy"), ("stop_all", ALL_ROBOTS)]
    t = found[0].start + 1
    logged = Incident("step", "Cindy", "Exec B20_J_transfer: refused", t, t, raw=["log"])
    merged = add_events([logged], found)
    assert len(merged) == 2 and merged[0].raw[0] == "log" and "refused" in merged[0].raw[1]
    assert merged[1].kind == "stop_all" and merged[1].source == "run1/events.jsonl"


def test_the_matrix_counts_a_burst_once_per_day_and_robot():
    """Day and robot rows, kinds as columns."""
    items = [Incident("step", "Cindy", "a", STAMP, STAMP, count=9), Incident("step", "Cindy", "b", STAMP, STAMP),
             Incident("gripper", "Alice", "c", STAMP, STAMP)]
    assert count_matrix(items) == {("20261104", "Cindy"): {"step": 2}, ("20261104", "Alice"): {"gripper": 1}}


LOG = """pybullet build time
[WARN] [{t}.0] [husky_monitor]: /a200_0805: controller manager is not available; the active controller is unknown
[INFO] [{t}.5] [husky_monitor]: [movement_exec] Exec B20_J_transfer on cindy (a200-0806): B20_J_transfer
[WARN] [{t2}.0] [husky_monitor]: [movement_exec] Exec B20_J_transfer: failed — <b>ran all 8 passes</b>
[INFO] [{t2}.5] [husky_monitor]: [health/chips] a200-0805 base: signal green -> amber: a gap
[INFO] [{t3}.0] [husky_monitor]: stopping: cancelling plugin tasks and tearing down
(viser) Server stopped
"""


def write_logs(folder: Path) -> None:
    """A monitor's log of 4 Nov, its stamps real, and a robot pane's."""
    folder.mkdir()
    t = int(STAMP)
    (folder / "monitor_20261104_150000.log").write_text(LOG.format(t=t, t2=t + 600, t3=t + 1200))
    (folder / "cindy_20261104_150000.log").write_text(
        f"[ur_ros2_control_node-5] [WARN] [{t + 700}.0] [UR_Client_Library:]: Lost connection to stream\n")
    # * Last written that day: `read_incidents` tells a running monitor from the file's date, on any day the test runs.
    for log in folder.glob("*.log"):
        utime(log, (t + 1200, t + 1200))


def test_the_page_lists_the_days_incidents_from_a_log_folder(tmp_path):
    """Counts, kinds, raw lines escaped, nothing fetched; it says it is generated and what the data cannot show."""
    logs = tmp_path / "logs"
    write_logs(logs)
    out = build_report(logs, "20261104", out=tmp_path / "out" / "incidents.html")
    found = read_incidents(logs, "20261104", "20261104", now=STAMP + 10_000)
    assert [(item.kind, item.robot) for item in found["incidents"]] == [
        ("no_answer", "Belle"), ("step", "Cindy"), ("no_answer", "Cindy"), ("hand", MONITOR)]
    assert found["health_lines"] == 1 and found["days_without_log"] == [] and found["running"] == []
    html = out.read_text()
    assert "Crashes and failures, 4 Nov 2026" in html and "Log lines" in html and "Stopped by hand" in html
    assert "&lt;b&gt;ran all 8 passes&lt;/b&gt;" in html and "<b>ran all" not in html
    assert "generated" in html and "What the data cannot show" in html and "http://" not in html
    assert "https://" not in html


def test_lines_of_another_day_are_left_out_and_a_day_without_log_is_said(tmp_path):
    """A day with no line in the folder: no incident, and the page says it has no log."""
    logs = tmp_path / "logs"
    write_logs(logs)
    found = read_incidents(logs, "20261105", "20261106")
    assert found["incidents"] == [] and found["days_without_log"] == ["20261105", "20261106"]
    assert "No incident on these days" in render(found, "20261105", "20261106")
