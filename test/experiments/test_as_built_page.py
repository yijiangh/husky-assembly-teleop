"""The as-built page built from records of the purged process: the scene data's shape and the written .html."""

from __future__ import annotations

import json
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np
import pytest
from planning_fixtures import purged_process

from husky_assembly_teleop.experiments.as_built.build import build_page
from husky_assembly_teleop.experiments.as_built.collect import collect_assembly, options_of, timeline
from husky_assembly_teleop.experiments.as_built.page import THREE_CORE, pack_link_meshes
from husky_assembly_teleop.shared.runs.as_built import append_record, as_built_path, build_record
from husky_assembly_teleop.world.bar_pose import pose_deviation
from husky_assembly_teleop.world.marker_layouts import bar_layouts, fake_sensed_markers, fit_bar
from scaffolding_assembly_core.geometry import Pose

BAR, DAY = "bars/B20", "20261030"


def bar_entry(process, bar: str, reference_kind: str, held_by: str | None, milestone: str | None,
              slide_m: float = 0.001) -> dict:
    """A measured bar as `as_built_record` writes it: fitted a little off its seat, the seat as its reference."""
    layout = bar_layouts(process)[bar]
    seat = process.bodies[bar].assembly_pose.matrix()
    sensed = fake_sensed_markers(seat, layout.points, slide_m=slide_m, roll_deg=0.05, noise_m=0.0)
    fit = fit_bar(sensed, layout)
    assert fit.ok
    pose = Pose.from_matrix(fit.T_world_from_bar)
    return {
        "bar": bar, "bar_id": bar[5:], "mocap_id": 1020, "bar_fit": fit.to_json(),
        "T_world_from_bar": [*pose.position, *pose.orientation],
        "sensed": {"points": sensed.points.tolist(), "ids": list(sensed.ids), "residuals": sensed.residuals.tolist(),
                   "age": 0.0, "source": "asset", "dropped": 0, "occluded": 0, "notes": []},
        "layout": {"names": list(layout.names), "points": layout.points.tolist(), "length": layout.length},
        "marker_source": "asset", "marker_age_s": 0.0, "measured_at": 10.0, "held_by": held_by,
        "T_link_from_bar": None, "reference_kind": reference_kind, "T_reference": seat.tolist(),
        "T_assembled": seat.tolist(),
        "deviation_vs_reference": pose_deviation(fit.T_world_from_bar, seat).to_json(),
        "deviation_vs_assembled": pose_deviation(fit.T_world_from_bar, seat).to_json(), "milestone": milestone}


def record(process, when: datetime, action_index: int, action_id: str, kind: str, movement: str, routine: str,
           part_kind: str, phase: str, outcome: str | None, bar: dict, servo: dict | None = None) -> dict:
    """One record of Cindy, B20 measured, at `when`."""
    base = Pose((1.0, -1.5, 0.0))
    joints = {f"left_ur_arm_{name}": 0.1 for name in ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                                      "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")}
    joints.update({name.replace("left", "right"): 0.2 for name in list(joints)})
    return build_record(
        problem="261008_2ModulesDemo_purged_process", run_id="20261030_101010", robot="cindy", serial="a200-0806",
        action={"index": action_index, "id": action_id, "type": "bar_jointing" if kind == "J" else "bar_release",
                "kind": kind, "bar": BAR, "bar_id": "B20", "robot": "robots/cindy"},
        step={"index": 0, "movement_id": movement, "routine": routine,
              "part": {"index": 0, "kind": part_kind, "also": [], "tools": [], "text": "", "last": True},
              "step": {"primary": 0, "indices": [0]}, "phase": phase, "outcome": outcome, "status": "", "reason": "",
              "dry_run": False},
        instrumented_bars=[{"bar": BAR, "bar_id": "B20", "mocap_id": 1020}], bars={"B20": bar},
        robot_state={"base_tracked": True, "base_pose": base, "fk_note": None, "arms": {
            "left": {"arm": "left_ur_arm", "flange": "robots/cindy/left_ur_arm_tool0",
                     "joint_names": [n for n in joints if n.startswith("left")], "joints": [0.1] * 6,
                     "tool0_fk": Pose((1.5, -1.0, 0.5)), "tool0_target_end": Pose((1.5, -1.0, 0.5))},
            "right": {"arm": "right_ur_arm", "flange": "robots/cindy/right_ur_arm_tool0",
                      "joint_names": [n for n in joints if n.startswith("right")], "joints": [0.2] * 6,
                      "tool0_fk": Pose((0.5, -1.0, 0.5)), "tool0_target_end": None}}},
        tool0={"left": {"pos_err_mm": [0.1, 0.2, 0.0], "pos_norm_mm": 0.22, "rot_err_deg": [0, 0, 0.1],
                        "observed_pose": [[1.5, -1.0, 0.5], [0, 0, 0, 1]]}},
        load={"arms": {"left": {"force_N": 12.0, "torque_Nm": 1.0, "wrench": [12, 0, 0, 1, 0, 0]},
                       "right": {"force_N": 10.0, "torque_Nm": 0.9, "wrench": [10, 0, 0, 0.9, 0, 0]}},
              "zero": {"at": 5.0, "reason": "travel to load (tools empty)"}},
        servo=servo,
        options={"transfer_as_free": {"box": True, "applies": routine == "transfer_free",
                                      "used": routine == "transfer_free"},
                 "servo_by_bar": {"box": True, "applies": part_kind == "servo",
                                  "used": bool(servo and servo["engaged"]), "why": ""},
                 "spiral": {"box": False, "fits": False, "applies": False, "used": False}, "insert_mode": None,
                 "dry_run": False, "ignore_built": False, "obstacles_collide": True, "insert_split_mm": 5.0,
                 "spiral_seconds": 20.0, "spiral_turns": 4},
        robots={"robots/a200-0806": {"name": "cindy", "serial": "a200-0806", "base": base, "joints": joints,
                                     "source": "live", "at": when.isoformat(), "after_action": None}},
        settings={"servo_by_bar": True}, now=when)


@pytest.fixture
def drive(tmp_path) -> Path:
    """A Drive root with one day of records: B20's transfer (start, end with a servo), its insert's end, its retreat."""
    loaded = purged_process()
    process = loaded.process
    root = tmp_path / "drive"
    (root / "data_experiment" / "bar_action_live_replan_exe").mkdir(parents=True)
    path = as_built_path(root, "261008_2ModulesDemo_purged_process", DAY)
    t0 = datetime(2026, 10, 30, 14, 0, 0)
    servo = {"file": "servo/B20_J/B20_J_transfer.1.json", "movement_id": "B20_J_transfer", "feedback": "mocap_bar",
             "planner_mode": "transfer+mocap_bar", "iterations": 2,
             "arms": {"left": {"pos_norm_mm": 0.3, "rot_max_deg": 0.1},
                      "right": {"pos_norm_mm": 0.5, "rot_max_deg": 0.2}},
             "bar": {"mocap_vs_design": {"dpos_mm": 0.3, "dang_deg": 0.02}, "fk_vs_design": None, "mocap_vs_fk": None},
             "worst": 0.5, "engaged": True, "by_bar": True, "why_tool0": "", "status": "done", "reason": "converged"}
    rows = [
        record(process, t0, 0, "B20_J", "J", "B20_J_transfer", "transfer_free", "motion", "start", None,
               bar_entry(process, BAR, "assembled", None, "bystander", slide_m=0.5)),  # on its loading spot
        record(process, t0 + timedelta(seconds=60), 0, "B20_J", "J", "B20_J_transfer", "transfer_free", "servo", "end",
               "ok", bar_entry(process, BAR, "held_authored:B20_J_transfer", "robots/cindy/left_ur_arm_tool0",
                               "before_insert"), servo),
        record(process, t0 + timedelta(seconds=120), 0, "B20_J", "J", "B20_J_insert", "insert", "insert", "end", "ok",
               bar_entry(process, BAR, "assembled", None, "after_insert", slide_m=0.002)),
        record(process, t0 + timedelta(seconds=200), 1, "B20_R", "R", "B20_R_retreat", "retreat_linear", "retreat",
               "end", "ok", bar_entry(process, BAR, "assembled", None, "after_retreat", slide_m=0.0015)),
    ]
    for row in rows:
        append_record(path, row)
    return root


def test_the_collector_shapes_the_day_as_the_old_page_expects(drive):
    """Actions in the schedule's order, each record's bars from their first milestone on, the card's numbers."""
    loaded = purged_process()
    scene = collect_assembly(DAY, "261008_2ModulesDemo_purged_process", drive, loaded.source, with_robots=False)
    assert scene["problem"] == "261008_2ModulesDemo_purged_process" and scene["day"] == DAY
    assert set(scene) >= {"assembled", "actions", "history", "ball_check", "robot_models", "source_file"}
    assert "B20" in scene["assembled"] and scene["assembled"]["B20"]["length"] == pytest.approx(1.8)
    actions = scene["actions"]
    assert [a["action_id"] for a in actions[:3]] == ["B20_J", "B20_R", "B21_J"]
    assert actions[0]["kind"] == "J" and actions[0]["robot"] == "Cindy" and actions[0]["exists"] == ["B20"]
    # * As the old page: Cindy holds a bar between its J and its R, not at the R itself; a support robot from its H.
    assert actions[1]["held"] == {} and "B21" in actions[2]["exists"]
    assert actions[4]["held"] == {"B21": "Alice"}, "B21_R: Alice holds B21 from B21_H until B21_HR"
    joint = actions[0]["records"]
    assert [r["step"]["phase"] for r in joint] == ["start", "end", "end"]
    assert joint[0]["bars"] == [] and scene["history"]["B20"]["skipped"] == 1, "the loading spot is not drawn"
    end = joint[1]
    assert end["milestone"] == "before_insert" and end["label"] == "before insert" and end["robot"] == "Cindy"
    bar = end["bars"][0]
    assert bar["bar"] == "B20" and bar["ok"] and bar["placement"] == pytest.approx(1.0, abs=0.2)
    assert bar["reference_kind"].startswith("held_authored") and bar["held_by"].endswith("left_ur_arm_tool0")
    assert len(bar["markers_sensed"]) == 9 and sum(m["state"] == "stray" for m in bar["markers_sensed"]) == 1
    assert len(bar["markers_authored"]) == 8 and all(m["seen"] for m in bar["markers_authored"])
    assert len(bar["pairs"]["joints"]) == 4 and len(bar["pairs"]["spacing"]) == 3
    assert bar["pair_length"] == pytest.approx(0.0, abs=0.05) and bar["findings"]
    assert end["servo"]["worst"] == 0.5 and end["servo"]["feedback"] == "mocap_bar" and "note" not in end["servo"]
    assert end["load"] == {"left_N": 12.0, "right_N": 10.0, "imbalance": 2.0,
                           "zero": {"at": "5.0", "note": "travel to load (tools empty)"}}
    assert end["tool0"]["left"]["fk_vs_target"]["dpos_mm"] == pytest.approx(0.0)
    assert end["tool0_worst"] == pytest.approx(0.0) and end["tool0"]["right"]["fk_vs_target"] is None
    options = {o["key"]: o for o in end["options"]}
    assert options["free_move"]["used"] and options["servo_bar"]["used"] and not options["spiral"]["applies"]
    assert options["servo_bar"]["why"] == "the servo aimed the bar's MoCap fit at its Rhino pose"
    assert joint[0]["options"] == end["options"], "the start record takes the step's options from its end"
    assert actions[1]["records"][0]["milestone"] == "after_retreat" and actions[1]["worst"] is not None
    history = scene["history"]["B20"]["entries"]
    assert [h["milestone"] for h in history] == ["before_insert", "after_insert", "after_retreat"]
    assert scene["ball_check"]["B20"]["n_records"] == 3


def test_build_page_writes_an_offline_html_with_the_scene_and_the_viewer(drive):
    """The page carries the scene data, the viewer and three.js; the Drive's result folder is where it goes."""
    if not THREE_CORE.exists():
        pytest.skip("three.js is not fetched (scripts/fetch_three_js.sh)")
    loaded = purged_process()
    out = build_page(drive, DAY, process_folder=loaded.source, with_robots=True)
    assert out == drive / "data_experiment/bar_action_live_replan_exe/20261030-asbuilt-result/assembly_20261030.html"
    html = out.read_text()
    assert '<script type="application/json" id="scene-data">' in html and "function buildStep" in html
    assert "OrbitControls" in html and "http://" not in html.split("scene-data")[0]
    data = json.loads(html.split('id="scene-data">', 1)[1].split("</script>", 1)[0])
    assert data["title"] == f"As-built 261008_2ModulesDemo_purged_process · {DAY}" and data["floor"] is None
    assert [m["key"] for m in data["metrics"]][:2] == ["placement", "rotation"]
    robots = data["actions"][0]["records"][1]["robots"]
    assert robots and robots[0]["name"] == "Cindy" and robots[0]["live"] and "base_link" in robots[0]["links"]
    assert "Cindy" in data["robot_models"] and data["robot_models"]["Cindy"]["footprint"][0] > 0.5


def test_options_of_reads_the_record_not_a_log():
    """A straight compliant insert of a one-mate bar: the spiral box applies, not used, and says why."""
    record = {"options": {"spiral": {"box": True, "fits": False, "applies": True, "used": False},
                          "insert_mode": "compliant-one-mate", "transfer_as_free": {}, "servo_by_bar": {}},
              "step": {"routine": "insert", "part": {"kind": "insert"}}, "action": {"kind": "J"}, "servo": None}
    spiral = next(o for o in options_of(record) if o["key"] == "spiral")
    assert spiral["applies"] and not spiral["used"] and spiral["box"] and "one mating joint" in spiral["why"]
    assert spiral["label"] == "Insert compliant part: SPIRAL search"


def test_timeline_and_meshes_on_the_fixture_process(tmp_path):
    """The schedule's bars and holders; a link's box and mesh shapes pack into one mesh."""
    from process_fixtures import build_process
    actions = timeline(build_process(tmp_path))
    assert [a["kind"] for a in actions] == ["J", "H", "R", "J", "HR"]
    assert actions[1]["held"] == {"B1": "Alice"} and actions[2]["held"] == {"B1": "Alice"}
    assert actions[3]["held"] == {"B1": "Alice"} and actions[3]["exists"] == ["B1", "B2"]
    packed = pack_link_meshes({"base_link": (np.zeros((3, 3), dtype=np.float32), np.array([[0, 1, 2]]))})
    assert packed["base_link"]["bits"] == 16 and packed["base_link"]["positions"]
