"""The as-built record's pure pieces: the file's path, the milestones, where a bar should be, the record, its file."""

from __future__ import annotations

import json
from dataclasses import replace
from datetime import datetime
from pathlib import Path

import numpy as np
import pytest
from process_fixtures import build_process, joints as JOINTS_AT, with_action

from husky_assembly_teleop.shared.assembly_planning.movement import Routine
from husky_assembly_teleop.shared.runs.as_built import (KIND, MILESTONES, PHASES, append_record, as_built_path,
                                                        bar_lines, build_record, day_of, milestone_of, read_records,
                                                        reference_of, servo_block, with_saved_robots)
from husky_assembly_teleop.shared.runs.runs import RobotRecord
from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.process.types import Target


def drive(tmp_path: Path) -> Path:
    """A Drive root with the runs folder the records go under."""
    (tmp_path / "data_experiment" / "bar_action_live_replan_exe").mkdir(parents=True)
    return tmp_path


def test_the_file_lives_in_the_days_folder_on_the_drive(tmp_path):
    """<drive>/data_experiment/bar_action_live_replan_exe/<day>-asbuilt/as_built_<problem>.jsonl; no Drive raises."""
    root = drive(tmp_path)
    path = as_built_path(root, "261008_2ModulesDemo_purged_process", "20261030")
    assert path == (root / "data_experiment/bar_action_live_replan_exe/20261030-asbuilt"
                    / "as_built_261008_2ModulesDemo_purged_process.jsonl")
    assert day_of("2026-10-30T14:15:01") == "20261030"
    with pytest.raises(FileNotFoundError):
        as_built_path(None, "p", "20261030")
    with pytest.raises(FileNotFoundError):
        as_built_path(tmp_path / "elsewhere", "p", "20261030")


@pytest.mark.parametrize("action_type, routine, part_kind, phase, last, after_insert, expected", [
    ("bar_jointing", Routine.TRANSFER_FREE, "servo", "end", True, False, "before_insert"),
    ("bar_jointing", Routine.TRANSFER_FREE, "motion", "end", False, False, None),
    ("bar_jointing", Routine.INSERT, "insert", "end", True, False, "after_insert"),
    ("bar_jointing", Routine.INSERT, "servo", "end", True, False, "after_insert"),
    ("bar_jointing", Routine.OPERATOR, "operator", "end", True, True, "after_fix"),
    ("bar_jointing", Routine.OPERATOR, "operator", "end", True, False, None),
    ("bar_release", Routine.RETREAT_LINEAR, "retreat", "start", True, False, "before_retreat"),
    ("bar_release", Routine.RETREAT_LINEAR, "retreat", "end", True, False, "after_retreat"),
    ("bar_holding", Routine.SUPPORT_APPROACH, "servo", "end", True, False, "hold_approach"),
    ("bar_holding", Routine.SUPPORT_LINEAR, "motion", "end", True, False, "hold_at_grasp"),
    ("bar_holding", Routine.TOOL_ONLY, "handoff", "end", True, False, "hold_grasped"),
    ("bar_holding_release", Routine.TOOL_ONLY, "robotiq_open", "end", True, False, "hold_released"),
    ("bar_holding_release", Routine.SUPPORT_LINEAR, "motion", "end", True, False, "hold_retreated"),
    ("bar_jointing", Routine.INSERT, "insert", "start", True, False, None),
    ("bar_jointing", Routine.INSERT, "insert", "manual", True, False, None),
])
def test_the_milestones_of_the_active_bar(action_type, routine, part_kind, phase, last, after_insert, expected):
    """The old table, in routines: each moment of the action's own bar, and nothing at the other moments."""
    assert milestone_of(action_type, routine, part_kind, phase, True, last, after_insert) == expected
    assert expected is None or expected in MILESTONES


def test_every_other_bar_is_a_bystander_and_a_click_has_no_milestone():
    """A bar that is not the action's is a bystander at any moment; a manual record of the active bar has none."""
    assert milestone_of("bar_jointing", Routine.INSERT, "insert", "end", False, True, False) == "bystander"
    assert milestone_of(None, None, None, "manual", False, False, False) == "bystander"
    assert milestone_of("bar_jointing", Routine.INSERT, "insert", "manual", True, True, False) is None
    assert milestone_of(None, Routine.INSERT, "insert", "end", True, True, False) is None


def test_reference_of_a_held_a_resting_and_a_built_bar(tmp_path):
    """A held bar: the nearest authored flange pose composed with the grasp; staged: its pose; built: the seat."""
    process = build_process(tmp_path)
    joint = process.actions["B2_J_joint"]
    transfer = next(k for k, movement in enumerate(joint.movements) if movement.id == "B2_M2_transfer")
    insert = transfer + 1
    holder = joint.movements[insert].start.attached["bars/B2"][0]
    # The fixture's transfer gives joints only: give it the flange's pose too, as a real export does.
    flange = Pose((0.5, 0.5, 0.5))
    movements = list(joint.movements)
    movements[transfer] = replace(movements[transfer], target=Target.of(joints={"robots/cindy": JOINTS_AT(0.3)},
                                                                        link_poses={holder.to: flange}))
    process = with_action(process, "B2_J_joint", movements=tuple(movements))
    # At the insert's start B2 hangs in Cindy's tool: the transfer's target, with the grasp.
    T, kind, held_by, T_grasp = reference_of(process, "B2_J_joint", insert, "bars/B2", "start")
    assert kind == "held_authored:B2_M2_transfer" and held_by == holder.to
    np.testing.assert_allclose(T, compose(flange, holder.grasp).matrix(), atol=1e-9)
    np.testing.assert_allclose(T_grasp, holder.grasp.matrix(), atol=1e-12)
    # At the transfer's end its own target counts; at the grasp's start nothing earlier is authored.
    assert reference_of(process, "B2_J_joint", transfer, "bars/B2", "end")[1] == "held_authored:B2_M2_transfer"
    assert reference_of(process, "B2_J_joint", 1, "bars/B2", "start")[1:3] == ("held_no_authored_pose", holder.to)
    # At the insert's end B2 is built: the seat.
    T, kind, held_by, _grasp = reference_of(process, "B2_J_joint", insert, "bars/B2", "end")
    assert kind == "assembled" and held_by is None
    np.testing.assert_allclose(T, process.bodies["bars/B2"].assembly_pose.matrix())
    # B2 waits on its rack while B1 is built: resting at that pose.
    b1_insert = next(k for k, movement in enumerate(process.actions["B1_J_joint"].movements)
                     if movement.id == "B1_M2_insert")
    T, kind, held_by, _grasp = reference_of(process, "B1_J_joint", b1_insert, "bars/B2", "start")
    assert kind == "resting" and held_by is None
    np.testing.assert_allclose(T, Pose((2.0, 0.0, 0.1)).matrix())
    # A servo aiming B2 wins over everything; nothing loaded: the seat.
    design = Pose((0.0, 1.0, 2.0))
    T, kind, held_by, T_grasp = reference_of(process, "B2_J_joint", insert, "bars/B2", "end",
                                             servo_bar=("bars/B2", design, (holder.to, holder.grasp)))
    assert kind == "servo_target" and held_by == holder.to and np.allclose(T, design.matrix())
    assert reference_of(process, None, None, "bars/B2", "manual")[1] == "assembled"


def test_the_record_is_json_safe_and_refuses_an_unknown_phase(tmp_path):
    """Numpy, poses and `to_json` objects become plain types; the phase must be one of PHASES."""
    record = build_record(problem="p", run_id="20261030_101010", robot="cindy", serial="a200-0806", action=None,
                          step={"phase": "start", "outcome": None}, instrumented_bars=[],
                          bars={"B20": {"T_reference": np.eye(4), "deviation_vs_reference": None}},
                          robot_state={"base_pose": Pose((1.0, 2.0, 0.0))}, tool0={}, load=None, servo=None,
                          options={"dry_run": False}, robots={}, settings={"spiral": False},
                          now=datetime(2026, 10, 30, 14, 15, 1))
    assert record["stamp"] == "2026-10-30T14:15:01" and record["schema_version"] == 2
    assert record["bars"]["B20"]["T_reference"][0] == [1.0, 0.0, 0.0, 0.0]
    assert record["robot_state"]["base_pose"] == [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    json.dumps(record)
    assert build_record(problem="p", run_id="", robot="cindy", serial="s", action=None, step=None,
                        instrumented_bars=[], bars={}, robot_state={}, tool0={}, load=None, servo=None, options={},
                        robots={}, settings={})["step"] == {"phase": "manual", "outcome": None}
    with pytest.raises(ValueError):
        build_record(problem="p", run_id="", robot="cindy", serial="s", action=None, step={"phase": "middle"},
                     instrumented_bars=[], bars={}, robot_state={}, tool0={}, load=None, servo=None, options={},
                     robots={}, settings={})
    assert PHASES == ("start", "end", "manual") and KIND == "as_built"


def test_append_and_read_round_trip_and_a_torn_last_line(tmp_path):
    """Two records come back in order; a cut-off last line is skipped with a note, a cut line elsewhere raises."""
    path = tmp_path / "20261030-asbuilt" / "as_built_p.jsonl"
    first = {"stamp": "2026-10-30T10:00:00", "bars": {}, "step": {"phase": "start"}}
    second = {"stamp": "2026-10-30T10:00:05", "bars": {"B20": {"T": np.eye(2)}}, "step": {"phase": "end"}}
    assert append_record(path, first) == path and append_record(path, second) == path
    assert path.read_text().count("\n") == 2
    records = read_records(path)
    assert [record["stamp"] for record in records] == [first["stamp"], second["stamp"]]
    assert records[1]["bars"]["B20"]["T"] == [[1.0, 0.0], [0.0, 1.0]]
    with open(path, "a") as file:
        file.write('{"stamp": "2026-10-30T10:00:10", "ba')
    assert len(read_records(path)) == 2
    path.write_text('{"a": 1}\n{"b": \n{"c": 3}\n')
    with pytest.raises(ValueError):
        read_records(path)
    assert read_records(tmp_path / "missing.jsonl") == []


def test_saved_robots_fill_only_the_missing_ones():
    """A robot the record measured live stays; the run's saved ends fill the others as assumed."""
    record = {"robots": {"robots/a200-0806": {"name": "cindy", "source": "live"}}}
    saved = {"robots/a200-0806": RobotRecord(Pose((9.0, 9.0, 0.0)), {"j": 1.0}, "live", "t0", "B20_J"),
             "robots/a200-0804": RobotRecord(Pose((1.0, 2.0, 0.0)), {"j": 0.5}, "assumed", "t1", "B21_H")}
    filled = with_saved_robots(record, saved)
    assert filled["robots"]["robots/a200-0806"] == {"name": "cindy", "source": "live"}
    alice = filled["robots"]["robots/a200-0804"]
    assert alice["source"] == "assumed" and alice["after_action"] == "B21_H" and alice["joints"] == {"j": 0.5}
    assert alice["base"] == [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0] and alice["serial"] == "a200-0804"


def test_the_servo_block_takes_the_last_reading_and_the_bar_lines_read_the_old_way():
    """The page's servo numbers from a servo record; the log lines per bar, fit and reference."""
    payload = {"movement_id": "B20_J_transfer", "servo_feedback": "mocap_bar", "planner_mode": "transfer+mocap_bar",
               "servoing_data": [{"iter": 0, "tool0": {"left": {"pos_norm_mm": 3.0, "rot_err_deg": [0.1, 0.2, 0.3]}}},
                                 {"iter": 1, "tool0": {"left": {"pos_norm_mm": 0.1, "rot_err_deg": [0.1, -0.4, 0.0]},
                                                       "right": {"pos_norm_mm": 0.3, "rot_err_deg": [0, 0, 0]}},
                                  "bar": {"mocap_vs_design": {"dpos_mm": 0.2}, "fk_vs_design": {"dpos_mm": 1.0},
                                          "mocap_vs_fk": {"dpos_mm": 1.1}, "usable": True}}],
               "moves": [{}], "settings": {"by_bar": True, "why_tool0": ""},
               "outcome": {"status": "done", "reason": "converged on the MoCap bar"}}
    block = servo_block(payload, "servo/B20_J/B20_J_transfer.1.json")
    assert block["iterations"] == 1 and block["feedback"] == "mocap_bar" and block["engaged"]
    assert block["arms"]["left"] == {"pos_norm_mm": 0.1, "rot_max_deg": 0.4} and block["worst"] == 0.3
    assert block["bar"]["mocap_vs_design"] == {"dpos_mm": 0.2} and block["status"] == "done"
    assert servo_block({"servoing_data": []}, None) is None
    record = {"bars": {"B20": {"bar_fit": {"ok": True, "n_matched": 8, "n_design": 8, "rms_m": 0.0006},
                               "reference_kind": "held_authored:B20_J_transfer", "milestone": "before_insert",
                               "deviation_vs_reference": {"dpos_mm": 0.4, "dang_deg": 0.03}},
                       "B21": {"bar_fit": {"ok": False, "reason": "only 5 of 8 design markers matched (need 6)"},
                               "reference_kind": "assembled", "milestone": "bystander",
                               "deviation_vs_reference": None}}}
    assert bar_lines(record) == [
        "B20 [before_insert] fit ok 8/8, rms 0.6 mm | vs held_authored:B20_J_transfer dpos 0.4 mm, dang 0.030 deg",
        "B21 [bystander] fit FAILED (only 5 of 8 design markers matched (need 6)) | no reference (assembled)"]
    assert bar_lines({"bars": {}}) == ["no bar measured"]
