"""The accuracy report: its numbers agree with the 3D page's collector, old records read like new, no conclusions."""

from __future__ import annotations

import json
from datetime import datetime

import numpy as np
from planning_fixtures import purged_process
from test_as_built_page import BAR, DAY, bar_entry, drive, record  # noqa: F401 (`drive` is a fixture)

from husky_assembly_teleop.experiments.as_built.accuracy import (bar_error_mm, bars_measured, bars_without_markers,
                                                                 fit_quality, placed_motion, read_day, read_servo_runs,
                                                                 servo_run, settings_in_force, step_changes)
from husky_assembly_teleop.experiments.as_built.accuracy_page import build_report, settings_section
from husky_assembly_teleop.experiments.as_built.collect import _three_places, collect_assembly
from husky_assembly_teleop.experiments.as_built.old_records import old_bar_lengths, upgrade_record
from husky_assembly_teleop.shared.runs.as_built import append_record, as_built_path

PROBLEM = "261008_2ModulesDemo_purged_process"


def shaped(time: str, epoch: float, phase: str, routine: str | None, bars: list[dict], movement: str = "M",
           outcome: str | None = "ok", part: str | None = None, run: str = "r1") -> dict:
    """A shaped record, as `collect.shape_records` gives it, with the fields the numbers read."""
    return {"time": time, "epoch": epoch, "run_id": run, "robot": "Cindy", "bars": bars,
            "step": {"movement_id": movement, "kind": routine, "part": part, "phase": phase, "outcome": outcome},
            "options": []}


def bar(name: str, placement: float | None = None, ends=None, milestone: str | None = None,
        seat: float = 1000.0) -> dict:
    """A shaped bar with its placement and its two ends in metres."""
    return {"bar": name, "ok": True, "placement": placement, "fitted": ends, "milestone": milestone,
            "placement_seat": seat}


def ends_at(x_mm: float) -> list[list[float]]:
    """A bar's two ends with its start `x_mm` along x and its end twice that."""
    return [[x_mm / 1e3, 0.0, 0.0], [2 * x_mm / 1e3, 1.0, 0.0]]


def test_step_changes_compare_records_of_one_action_and_number_the_tries():
    """An insert's start to end, the wait before it, a second try; a run's first and a pair with no number are out."""
    action = {"action_id": "B20_J", "bar_id": "B20", "records": [
        shaped("10:00:00", 0, "end", "transfer_free", [bar("B20", 0.2)], "transfer"),
        shaped("10:01:00", 60, "start", "insert", [bar("B20", 0.3)], "insert"),
        shaped("10:01:10", 70, "end", "insert", [bar("B20", 0.8)], "insert"),
        shaped("10:02:00", 120, "end", "insert", [bar("B20", 0.1)], "insert", part="servo"),
        shaped("11:00:00", 3600, "start", "insert", [bar("B20", 0.5)], "insert", run="r2"),
        shaped("11:00:10", 3610, "end", "insert", [bar("B20", 0.7)], "insert", run="r2"),
        shaped("11:00:20", 3620, "manual", None, [bar("B20", None)], "insert", None, run="r2"),
    ]}
    rows = step_changes([action])
    assert [(row["group"], row["try"], row["tries"]) for row in rows] == [
        ("wait", 1, 1), ("insert", 1, 2), ("insert", 2, 2)]
    assert rows[1]["before"] == 0.3 and rows[1]["after"] == 0.8
    assert abs(rows[1]["change"] - 0.5) < 1e-9 and rows[1]["seconds"] == 10


def test_two_records_with_another_actions_record_between_them_are_no_step():
    """The release ran between the two records of the jointing: no wait there, whatever their phases say."""
    jointing = {"action_id": "B20_J", "bar_id": "B20", "records": [
        shaped("10:00:00", 0, "end", "insert", [bar("B20", 0.2)], "insert"),
        shaped("10:20:00", 1200, "start", "insert", [bar("B20", 3.0)], "insert")]}
    release = {"action_id": "B20_R", "bar_id": "B20", "records": [
        shaped("10:10:00", 600, "end", "retreat_linear", [bar("B20", 1.0)], "retreat")]}
    assert step_changes([jointing, release]) == []


def test_placed_motion_counts_a_bar_from_its_release_until_it_is_built_again():
    """Before its release a bar's motion does not count; each event measures from the record before it."""
    actions = [
        {"action_id": "B20_J", "bar_id": "B20", "kind": "J", "records": [
            shaped("09:00:00", 0, "end", "insert", [bar("B20", 1.0, ends_at(0.0))], "insert")]},
        {"action_id": "B20_R", "bar_id": "B20", "kind": "R", "records": [
            shaped("10:00:00", 3600, "end", "retreat_linear", [bar("B20", 2.0, ends_at(0.0), "after_retreat")],
                   "retreat")]},
        {"action_id": "B21_J", "bar_id": "B21", "kind": "J", "records": [
            shaped("10:10:00", 4200, "start", "insert", [bar("B20", 2.1, ends_at(1.0), "bystander")], "ins"),
            shaped("10:10:30", 4230, "end", "insert", [bar("B20", 2.5, ends_at(3.0), "bystander")], "ins")]},
        {"action_id": "B20_J2", "bar_id": "B20", "kind": "J", "records": [
            shaped("11:00:00", 7200, "end", "insert", [bar("B20", 9.0, ends_at(500.0))], "again")]},
    ]
    motion = placed_motion(actions)
    assert motion["bars"] == ["B20"] and len(motion["events"]) == 1
    event = motion["events"][0]
    assert event["title"] == "ins" and event["from"] == "10:00:00" and event["to"] == "10:10:30"
    assert np.allclose(event["moved"]["B20"], [3.0, 6.0])
    assert motion["since"]["B20"]["from"] == "10:00:00" and motion["since"]["B20"]["to"] == "10:10:30"
    assert motion["since"]["B20"]["error_from"] == 2.0 and motion["since"]["B20"]["error_to"] == 2.5
    assert motion["still"]["seconds"] == 600 and np.allclose(motion["still"]["moved"]["B20"], [1.0, 2.0])


def test_a_bystander_at_its_seat_is_in_place_and_one_on_its_loading_spot_is_not():
    """First seen near its seat: counted from then on; metres away: never."""
    actions = [{"action_id": "B21_J", "bar_id": "B21", "kind": "J", "records": [
        shaped("10:00:00", 0, "start", "insert", [bar("B20", 1.0, ends_at(0.0), "bystander", seat=2.0),
                                                  bar("B19", 1.0, ends_at(0.0), "bystander", seat=2000.0)], "ins"),
        shaped("10:00:10", 10, "end", "insert", [bar("B20", 1.0, ends_at(1.0), "bystander", seat=2.0),
                                                 bar("B19", 1.0, ends_at(9.0), "bystander", seat=2000.0)], "ins")]}]
    motion = placed_motion(actions)
    assert motion["bars"] == ["B20"] and list(motion["events"][0]["moved"]) == ["B20"]


def pose_at(x: float) -> list[list[float]]:
    """A 4x4 pose of a bar along world z, `x` metres along x."""
    T = np.eye(4)
    T[0, 3] = x
    return T.tolist()


def test_a_servo_files_readings_become_the_placement_form_flanges_forces_and_outcome():
    """Bar error as the 3D page's worst of three; not usable gives no number and keeps the reason."""
    T_design, T_mocap = np.asarray(pose_at(0.0)), np.asarray(pose_at(0.003))
    good = {"iter": 0, "tool0": {"left": {"pos_norm_mm": 1.5}, "right": {"pos_norm_mm": 2.5}},
            "wrench": {"left": {"force_N": 60.0}, "right": {"force_N": 61.0}},
            "bar": {"usable": True, "T_mocap": T_mocap.tolist(), "T_design": T_design.tolist()}}
    bad = {"iter": 1, "tool0": {"left": {"pos_norm_mm": 0.5}}, "wrench": {},
           "bar": {"usable": False, "reason": "fit failed", "T_mocap": None, "T_design": T_design.tolist()}}
    run = servo_run({"bar_name": "bars/B20", "movement_id": "B20_J_transfer", "kind": "transfer",
                     "servo_feedback": "tool0", "servoing_data": [good, bad],
                     "outcome": {"status": "failed", "reason": "ran all", "passes": 1}},
                    datetime(2026, 11, 4, 15, 0, 0).timestamp(), 1.8, old=False, file="x.json")
    assert run["bar"] == "B20" and run["corrections"] == 1 and run["time"] == "15:00:00"
    assert run["outcome"] == "failed" and run["reason"] == "ran all" and run["ended"] == ""
    assert abs(run["passes"][0]["bar_mm"] - _three_places(T_mocap, T_design, 1.8)["worst"]) < 1e-9
    assert abs(run["passes"][0]["bar_mm"] - 3.0) < 1e-9
    assert run["passes"][0]["flanges"] == {"left": 1.5, "right": 2.5} and run["passes"][0]["forces"]["right"] == 61.0
    assert run["passes"][1]["bar_mm"] is None and run["passes"][1]["note"] == "fit failed"
    old = servo_run({"bar_name": "bar_B20", "servoing_data": [good]}, None, 1.8, old=True)
    assert old["bar"] == "B20" and old["outcome"] is None and "old file" in old["ended"]


def test_the_old_frame_starts_at_the_bar_so_a_tilted_bar_gets_the_same_number_as_in_the_new_frame():
    """A tilt about the old origin moves the far end most; shifted to the middle, the worst distance is at that end."""
    length = 2.0
    tilt = np.eye(4)
    angle = 0.001
    tilt[:3, :3] = [[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]]
    reading = {"bar": {"usable": True, "T_mocap": tilt.tolist(), "T_design": np.eye(4).tolist()}}
    assert abs(bar_error_mm(reading, length, old=True) - angle * length * 1e3) < 1e-3
    assert bar_error_mm(reading, None, old=True) is None


def old_entry(T: np.ndarray, T_reference: np.ndarray) -> dict:
    """An old monitor's bar entry: markers under their set's key, frames from the bar's start."""
    return {"bar_B20": {"1": {"pos": [0, 0, 0.5], "error": 0.0}, "2": {"pos": [0, 0.05, 0.5], "error": 0.0}},
            "markerset_key": "bar_B20", "bar_id": "B20", "milestone": "before_insert", "held_by": None,
            "T_link_from_bar": None, "reference_kind": "resting", "marker_source": "asset", "marker_age_s": 0.0,
            "design_markers": [{"name": "M1", "bar_m": [0, 0, 0.5]}, {"name": "M2", "bar_m": [0, 0.05, 0.5]}],
            "bar_fit": {"ok": True, "T_world_from_bar": T.tolist(), "n_matched": 2, "n_design": 2, "rms_m": 0.001,
                        "correspondences": {"0": 0, "1": 1}, "sensed_ids": [1, 2]},
            "T_reference": T_reference.tolist(), "T_assembled": T_reference.tolist()}


def test_an_old_record_upgrades_to_the_new_form():
    """Action id, routine from the movement's name, bar under its id, frames moved half a bar, markers with them."""
    T = np.eye(4)
    upgraded = upgrade_record(
        {"schema_version": 1, "action": {"action_id": "B20_J_joint", "kind": "J", "bar_id": "B20"},
         "step": {"movement_id": "B20_J_M4_LM_insert", "phase": "end", "outcome": "ok"},
         "bars": {"bar_B20": old_entry(T, T), "bar_B99": {"bar_id": "B99"}}}, {"B20": 2.0})
    assert upgraded["action"]["id"] == "B20_J_joint" and upgraded["step"]["routine"] == "insert"
    assert list(upgraded["bars"]) == ["B20"]
    entry = upgraded["bars"]["B20"]
    assert entry["bar"] == "bars/B20" and "bar_B20" not in entry
    assert entry["sensed"]["ids"] == [1, 2] and entry["layout"]["length"] == 2.0
    assert np.allclose(np.asarray(entry["T_reference"])[:3, 3], [0, 0, 1.0])
    assert np.allclose(np.asarray(entry["bar_fit"]["T_world_from_bar"])[:3, 3], [0, 0, 1.0])
    assert np.allclose(entry["layout"]["points"][0], [0, 0, -0.5])


def test_an_old_exports_bar_lengths_come_from_its_cell_file(tmp_path):
    """Each bar's collision mesh extent along z, not the blocks beside it."""
    cell = tmp_path / "RobotCell.json"
    cell.write_text('{\n            "bar_B20": {\n"z": 0.0,\n"z": 1.8\n            },\n            "bar_B21": {\n'
                    '"z": 0.5,\n"z": 2.5\n            },\n            "table": {\n"z": 100.0\n            }\n}')
    assert old_bar_lengths(cell) == {"B20": 1.8, "B21": 2.0}


def fit_entry(ok: bool, matched: int = 8, rms_mm: float = 0.5, reason: str = "") -> dict:
    """A raw entry's fit."""
    return {"bar_fit": {"ok": ok, "n_matched": matched, "n_design": 8, "rms_m": rms_mm / 1e3 if ok else None,
                        "reason": reason}}


def test_fit_quality_flags_fits_under_n_or_above_the_rms_limit_and_counts_failures():
    """Matched range, rms median and worst, counts of the flags, reasons with their numbers folded."""
    raw = [{"bars": {"B20": fit_entry(True)}}, {"bars": {"B20": fit_entry(True, 6, 2.0)}},
           {"bars": {"B20": fit_entry(False, reason="the marker snapshot is 12.5 s old")}},
           {"bars": {"B20": fit_entry(False, reason="the marker snapshot is 99.1 s old")}}]
    row = fit_quality(raw)[0]
    assert row["records"] == 4 and row["ok"] == 2 and row["failed"] == 2
    assert sorted(row["matched"]) == [6, 8] and row["under_n"] == 1 and row["over_rms"] == 1
    assert row["rms_median"] == 1.25 and row["rms_max"] == 2.0
    assert row["reasons"] == {"the marker snapshot is N s old": 2}


def test_which_bars_were_measured_and_which_have_no_markers():
    """The action's bar counts; a bar with markers but never fitted is neither."""
    new_form = [{"bar": "bars/B20", "bar_id": "B20", "mocap_id": 1020}, {"bar": "bars/B21", "bar_id": "B21"}]
    raw = [{"action": {"bar_id": "B20"}, "instrumented_bars": new_form,
            "bars": {"B20": fit_entry(True), "B21": fit_entry(False)}},
           {"action": {"bar_id": "B22"}, "instrumented_bars": new_form, "bars": {}},
           {"action": {"bar_id": "B21"}, "instrumented_bars": ["bar_B20", "bar_B21"], "bars": {}}]
    assert bars_measured(raw) == ["B20"] and bars_without_markers(raw) == ["B22"]


def test_settings_in_force_are_spans_of_time_per_value():
    """A value that changes opens a new span; dict options are not settings."""
    raw = [{"stamp": "2026-11-04T10:00:00", "settings": {"spiral": False},
            "options": {"insert_mode": None, "spiral": {}}},
           {"stamp": "2026-11-04T10:05:00", "settings": {"spiral": False}, "options": {"insert_mode": "rigid"}},
           {"stamp": "2026-11-04T10:09:00", "settings": {"spiral": True}, "options": {}}]
    found = {item["key"]: item["spans"] for item in settings_in_force(raw)}
    assert found["spiral"] == [{"value": False, "from": "10:00:00", "to": "10:05:00"},
                               {"value": True, "from": "10:09:00", "to": "10:09:00"}]
    assert [span["value"] for span in found["insert_mode"]] == [None, "rigid"]


def test_the_page_shows_the_3d_pages_numbers_and_says_it_draws_no_conclusions(drive, tmp_path):  # noqa: F811
    """Placement numbers equal the collector's; offline; the generated note; every section; nothing written as cause."""
    scene = collect_assembly(DAY, PROBLEM, drive, purged_process().source, with_robots=False)
    expected = next(bar_ for action in scene["actions"] for shown in action["records"] for bar_ in shown["bars"]
                    if bar_["milestone"] == "before_insert")["placement"]
    day = read_day(drive, DAY, PROBLEM, purged_process().source)
    mine = next(bar_ for action in day["actions"] for shown in action["records"] for bar_ in shown["bars"]
                if bar_["milestone"] == "before_insert")["placement"]
    assert mine == expected
    out = build_report(drive, DAY, PROBLEM, purged_process().source, tmp_path / "out" / "accuracy.html")
    html = out.read_text()
    for words in ("gives measurements without conclusions", "What each step did to the bar being built",
                  "Bars already in place", "The servo, correction by correction", "Which settings each step used",
                  "Fit quality per bar", "How the numbers are made", PROBLEM, "Servo feedback: MoCap bar markers"):
        assert words in html, words
    assert "http://" not in html and "https://" not in html
    for claim in ("because of", "caused by", "points to", "explains"):
        assert claim not in html.lower(), claim


def test_the_settings_table_writes_each_action_once_with_its_movements_under_it():
    """B20_J twice over, with B20_R between its records: one heading, and short movement names."""
    def step(time: str, epoch: float, movement: str) -> dict:
        return shaped(time, epoch, "end", "transfer_free", [], movement=movement, part="motion")
    actions = [{"action_id": "B20_J", "records": [step("14:00:21", 1.0, "B20_J_transfer"),
                                                  step("14:20:00", 3.0, "B20_J_insert")]},
               {"action_id": "B20_R", "records": [step("14:14:50", 2.0, "B20_R_retreat")]}]
    html = settings_section({"old": False, "actions": actions, "raw": []})
    assert html.count("<b>B20_J</b>") == 1 and html.count("<b>B20_R</b>") == 1
    assert html.index("<b>B20_J</b>") < html.index("14:20:00") < html.index("<b>B20_R</b>")
    assert ">transfer<" in html and "B20_J_transfer" not in html


def test_the_default_output_sits_in_the_days_result_folder_and_the_day_defaults_to_the_newest(drive):  # noqa: F811
    """`<day>-asbuilt-result/accuracy_<day>.html`, as the monitor's button serves that folder."""
    out = build_report(drive, None, PROBLEM, purged_process().source)
    folder = drive / "data_experiment/bar_action_live_replan_exe" / f"{DAY}-asbuilt-result"
    assert out == folder / f"accuracy_{DAY}.html"


def test_a_servo_file_of_the_run_shows_on_the_page_with_its_time(drive, tmp_path):  # noqa: F811
    """A copy of the process with a run: the run's servo file read, its time from the run's events."""
    process = purged_process()
    path = as_built_path(drive, PROBLEM, DAY)
    assert path.exists()
    run = tmp_path / "process" / "runs" / "20261030_101010"
    (run / "servo" / "B20_J").mkdir(parents=True)
    pose = pose_at(0.0)
    moved = pose_at(0.002)
    data = {"bar_name": BAR, "movement_id": "B20_J_transfer", "kind": "transfer", "servo_feedback": "mocap_bar",
            "outcome": {"status": "done", "reason": "converged", "passes": 1},
            "servoing_data": [{"iter": 0, "tool0": {"left": {"pos_norm_mm": 0.3}}, "wrench": {},
                               "bar": {"usable": True, "T_mocap": moved, "T_design": pose}}]}
    (run / "servo" / "B20_J" / "B20_J_transfer.1.json").write_text(json.dumps(data))
    (run / "events.jsonl").write_text(json.dumps({"time": "2026-10-30T14:01:00+01:00", "event": "servo",
                                                  "file": "servo/B20_J/B20_J_transfer.1.json"}) + "\n")
    # * The process is read from its own folder and the runs from the folder given: the records name this run.
    day = read_day(drive, DAY, PROBLEM, process.source)
    day["process_folder"] = tmp_path / "process"
    runs = read_servo_runs(day, drive)
    assert len(runs) == 1 and runs[0]["bar"] == "B20" and runs[0]["time"] == "14:01:00"
    assert abs(runs[0]["passes"][0]["bar_mm"] - 2.0) < 1e-9


def test_the_same_record_appended_later_counts_by_its_time_not_its_place_in_the_file(drive):  # noqa: F811
    """Several monitors append to one file: read in time order."""
    process = purged_process().process
    path = as_built_path(drive, PROBLEM, DAY)
    early = datetime(2026, 10, 30, 13, 0, 0)
    append_record(path, record(process, early, 0, "B20_J", "J", "B20_J_transfer", "transfer_free", "motion", "end",
                               "ok", bar_entry(process, BAR, "assembled", None, "bystander", slide_m=0.5)))
    day = read_day(drive, DAY, PROBLEM, purged_process().source)
    stamps = [entry["stamp_epoch"] for entry in day["raw"]]
    assert stamps == sorted(stamps) and stamps[0] == early.timestamp()
