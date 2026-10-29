"""The servo record: every key of spec §2.9 in the payload, its labels per kind, and the four-panel PNG."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from husky_assembly_teleop.shared.execution.servo import AFTER_INSERT, APPROACH, TRANSFER, ServoPlan
from husky_assembly_teleop.shared.execution.servo_record import plot_servo, servo_payload
from husky_assembly_teleop.world.marker_layouts import BarLayout
from scaffolding_assembly_core.geometry import Pose, compose

LEFT, RIGHT = "robots/cindy/left_ur_arm_tool0", "robots/cindy/right_ur_arm_tool0"
#: The record's keys (spec §2.9): the old monitor's five and the new ones.
KEYS = {"planner_mode", "movement_id", "servo_feedback", "bar_name", "servoing_data", "robot", "serial", "action",
        "kind", "bar_frame", "settings", "moves", "outcome", "dry_run"}
MOVE = {"planner": "tamp.plan_constrained_dual_arm_linear", "attempts": 1, "rows": 12,
        "ik": {"backend": "ssik", "rank": 0, "combinations": 4, "max_change": 0.02}, "correction_mm": 3.1,
        "excursion_deg": 1.2, "net_deg": 1.1, "traj_time": 3.0, "duration": 3.4, "prompted": True,
        "outcome": "arrived"}


def plan(kind: str = TRANSFER, layout: bool = True) -> ServoPlan:
    """A plan of `kind`: two arms holding B20 (one arm on an approach), with B20's layout unless `layout` is off."""
    grasp = Pose((-0.495, 0.0, 0.08), (0.0, -0.7071067811865476, 0.0, -0.7071067811865476))
    if kind == APPROACH:
        targets = {"robots/alice/ur_arm_tool0": Pose((1.0, 0.0, 1.0))}
        return ServoPlan(kind, "B21_H", 0, "B21_H_approach", 5, tuple(targets), ("ur_arm",), targets, False, None,
                         None, None, None, False, False, "this loop plans free moves (no bar held)", False, None)
    targets = {LEFT: Pose((1.0, 0.0, 1.0)), RIGHT: Pose((1.0, -1.0, 1.0))}
    points = np.array([(0.025, 0.024, z) for z in (-0.8, -0.5, 0.25, 0.78)] * 2)
    return ServoPlan(kind, "B20_J", 3, "B20_J_transfer", 4, (LEFT, RIGHT), ("left_ur_arm", "right_ur_arm"),
                     targets, True, "bars/B20", (LEFT, grasp),
                     BarLayout("bars/B20", tuple("abcdefgh"), points, 1.8) if layout else None,
                     compose(targets[LEFT], grasp), True, True, "", kind == AFTER_INSERT,
                     20.0 if kind == AFTER_INSERT else None)


def reading(k: int, sides: tuple = ("left", "right"), bar: bool = True) -> dict:
    """Reading `k`: the tool0 errors shrinking with k, a base drift, and the bar as measured, usable from k = 1."""
    tool0 = {side: {"pos_err_mm": [1.0 / (k + 1), 0.5 / (k + 1), -0.2], "pos_norm_mm": 1.2 / (k + 1),
                    "rot_err_deg": [0.1, 0.2, 0.3], "observed_pose": ((1.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0))}
             for side in sides}
    base = {"pos_diff_mm": [0.1 * k, 0.0, 0.0], "pos_norm_mm": 0.1 * k, "rot_diff_deg": [0.01, 0.02, 0.0]}
    out = {"iter": k, "tool0": tool0, "base": base, "wrench": None, "force_change_N": {side: 0.0 for side in sides},
           "track_mm": {side: 0.1 for side in sides}}
    if bar:
        deviation = {"dpos_mm": 0.4, "dang_deg": 0.05, "dpos_world_mm": [0.4, 0.0, 0.0], "dpos_bar_mm": [0.4, 0.0, 0.0],
                     "rotvec_bar_deg": [0.05, 0.0, 0.0]}
        out["bar"] = {"bar": "bars/B20", "grounded": True, "source": "asset", "fit": None,
                      "T_design": np.eye(4).tolist(), "T_fk": np.eye(4).tolist(), "T_mocap": None, "T_corr": None,
                      "fk_vs_design": deviation, "mocap_vs_design": deviation if k else None,
                      "mocap_vs_fk": deviation if k else None, "usable": k > 0,
                      "reason": "" if k else "fit failed: no sample"}
    return out


def payload(kind: str = TRANSFER, engaged: bool = True, sides: tuple = ("left", "right"), bar: bool = True,
            layout: bool = True) -> dict:
    """A two-reading, one-move record of `kind`."""
    return servo_payload(plan(kind, layout), robot="robots/a200-0806", serial="a200-0806",
                         readings=[reading(0, sides, bar), reading(1, sides, bar)], moves=[MOVE], status="done",
                         reason="converged on the MoCap bar", engaged=engaged, dry_run=False)


def test_payload_has_every_key_of_the_spec():
    """The old keys and the new ones, plain JSON values, the bar frame note and the constants used."""
    record = payload()
    assert set(record) == KEYS
    json.dumps(record)
    assert record["planner_mode"] == "transfer+mocap_bar" and record["servo_feedback"] == "mocap_bar"
    assert record["bar_name"] == "bars/B20" and record["movement_id"] == "B20_J_transfer"
    assert (record["robot"], record["serial"], record["action"], record["kind"]) == (
        "robots/a200-0806", "a200-0806", "B20_J", TRANSFER)
    assert record["bar_frame"] == {"origin": "middle", "length_m": 1.8, "measured_at": "start (T·Trans(0,0,−L/2))"}
    assert record["outcome"] == {"status": "done", "reason": "converged on the MoCap bar", "passes": 1}
    assert record["moves"] == [MOVE] and len(record["servoing_data"]) == 2 and record["dry_run"] is False
    settings = record["settings"]
    assert settings["max_passes"] == 8 and abs(settings["tool0_tol_mm"] - 0.2) < 1e-12
    assert np.allclose(settings["bar_tol_mm_deg"], [0.5, 0.1]) and np.allclose(settings["bar_gate_mm_deg"], [30.0, 3.0])
    assert settings["max_force_change_N"] is None and settings["by_bar"] and not settings["linear_only"]
    assert payload(layout=False)["bar_frame"]["length_m"] is None
    assert payload(bar=False)["bar_name"] is None


@pytest.mark.parametrize("kind, engaged, mode, feedback", [
    (TRANSFER, False, "transfer", "tool0"),
    (TRANSFER, True, "transfer+mocap_bar", "mocap_bar"),
    (AFTER_INSERT, True, "transfer+mocap_bar", "mocap_bar"),
    (APPROACH, False, "single_arm_joint", "tool0"),
])
def test_planner_mode_labels(kind: str, engaged: bool, mode: str, feedback: str):
    """The old labels: a held servo is a transfer, '+mocap_bar' only when the loop steered on the bar."""
    record = payload(kind, engaged, sides=("arm",) if kind == APPROACH else ("left", "right"), bar=kind != APPROACH)
    assert record["planner_mode"] == mode and record["servo_feedback"] == feedback
    assert record["settings"]["max_force_change_N"] == (20.0 if kind == AFTER_INSERT else None)


def test_plot_writes_a_png(tmp_path: Path):
    """Two arms with bar readings (a gap where the fit failed), and one arm without: both draw to a PNG."""
    path = tmp_path / "B20_J_transfer.1.png"
    plot_servo(payload(), path)
    assert path.stat().st_size > 1000
    single = tmp_path / "B21_H_approach.1.png"
    plot_servo(payload(APPROACH, False, sides=("arm",), bar=False), single)
    assert single.stat().st_size > 1000
