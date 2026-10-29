"""The compliant parts end to end through movement_exec, as the operator runs them: Exec, the outcome.

B21's insert with its screw stalling 1 s into the compliant part (the right arm finishing its line and settling before
it goes back),
B20's ground-bar insert (always rigid), and B20's retreat with its tail replanned to the plan's goals, on the
fake lab of `test_movement_exec` (Cindy following what
she is sent, her tools' firmware answering, the clock the fake context's). Each checks the record the run writes and
that every arm ends under the joint controller.
"""

from __future__ import annotations

import asyncio
import json
from dataclasses import replace

import numpy as np
import pytest
from test_movement_exec import (INSERT_STEP, RETREAT_STEP, TailPlanner, along, commands, compliant_lab, execute,
                                prompts, said)

from husky_assembly_teleop.robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER as COMPLIANCE
from husky_assembly_teleop.robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER as JOINT
from husky_assembly_teleop.robot_interface.end_effectors import ScaffoldingV3
from husky_assembly_teleop.shared.execution.retreat import TAIL_STEP
from husky_assembly_teleop.shared.execution.stream import RETREATED, RIGID_DONE, SETTLE_WINDOW, STALLED
from husky_assembly_teleop.shared.movement_exec_contract import DONE, SERVO_INDEX
from husky_assembly_teleop.shared.runs.runs import RunSettings

pytestmark = [pytest.mark.slow, pytest.mark.safety]

STOP, TIGHTEN_SCREW, OPEN_GRIPPER = (0, 0), (ScaffoldingV3.JOINT_MOTOR, 1), (ScaffoldingV3.GRIPPER_MOTOR, -1)
#: Seconds into the compliant part the left screw stalls: inside B21's 1.7 s line (the insert asked over 5 s), so
#: the right arm finishes it.
STALL_AFTER = 1.0


def screw_driver(lab, arm: str):
    """The firmware of one of Cindy's tools."""
    return next(driver for driver in lab.drivers if getattr(driver, "tool", None) is lab.tools[arm])


def record_of(lab) -> dict:
    """The last movement record the run wrote, as it lands in `runs/<run>/movements/`."""
    return json.loads(json.dumps(lab.run.files()[-1]))


def back_under_joint_control(lab) -> bool:
    """Every arm of Cindy runs the joint controller, its controller manager saying so."""
    return all(arm.controllers.running == JOINT and arm.controllers.is_active(JOINT)
               for arm in lab.cindy.arms.values())


def test_b21s_insert_ends_when_the_left_screw_stalls_1_s_into_the_compliant_part():
    """B21_J_insert: rigid, compliant to the stall, the right arm's line finished and settled; DONE, STALLED, back.

    The insert is asked over 5 s (the lab's requests take 2 s), so its line lasts 1.7 s and the stall at 1 s is inside
    it. The record holds the split achieved, the stall time (from TIGHTEN to the stall), the left arm's early way back,
    the right arm's rest of the line and its settle, the switch to compliance and back; the left screw alone tightened.
    """
    lab = compliant_lab(("B21_J", 4))
    left = screw_driver(lab, "left_ur_arm")
    left.screw_stall_after = None
    seen = {}

    async def stall_into_the_compliant_part(lab):
        while lab.plugin.active:
            recorder = lab.plugin._job.recorder
            if recorder is not None and recorder.phase == "compliant":
                break
            await asyncio.sleep(0)
        seen["compliant"] = lab.ctx.t
        # * A hair under: the clock is summed in 0.1 s steps, and a rounding error must not cost an extra tick.
        while lab.ctx.t < seen["compliant"] + STALL_AFTER - 1e-6 and lab.plugin.active:
            await asyncio.sleep(0)
        left.screw = ScaffoldingV3.STALLED

    request = replace(along(lab, "B21_J_insert", INSERT_STEP), traj_time=5.0)
    result, _held = execute(lab, request, during=stall_into_the_compliant_part)
    assert result.status == DONE and result.done == (4,), result.reason
    assert not prompts(lab) and said(lab)[0].startswith("cindy (a200-0806): B21_J_insert — INSERT: rigid ")
    record = record_of(lab)
    outcome = record["outcome"]
    assert record["part"]["kind"] == "insert" and outcome["status"] == STALLED and "STALLED" in outcome["reason"]
    plan = outcome["plan"]
    assert plan["mating"] == ["left_ur_arm"] and not plan["grounded"]
    assert plan["split_mm"] == 5.0 and plan["achieved_mm"] == pytest.approx(5.0) and 14.0 < plan["total_mm"] < 15.5
    assert plan["rigid_s"] > 0.0 and plan["compliant_s"] >= 0.5
    rigid, switch, compliant, back, finish, settle = outcome["phases"]
    assert [phase["name"] for phase in outcome["phases"]] == [
        "rigid", "switch", "compliant", "left_ur_arm back", "compliant right_ur_arm", "settle right_ur_arm"]
    assert rigid["end"] in ("ARRIVED", "ENDED") and compliant["end"] == "stalled"
    assert STALL_AFTER <= compliant["t1"] - compliant["t0"] <= STALL_AFTER + 0.3
    assert back["end"] == "back" and finish["end"] == "ended" and settle["end"] == "settled"
    assert finish["t1"] - finish["t0"] >= plan["compliant_s"] - STALL_AFTER - 0.3
    assert settle["t1"] - settle["t0"] >= SETTLE_WINDOW and outcome["settle"]["settled"]
    # * The gate is made in the tick TIGHTEN goes out, the rigid part's first; the stall is counted in the tick that
    #   ends the compliant part.
    assert list(outcome["stall_times"]) == ["left_ur_arm"]
    assert outcome["stall_times"]["left_ur_arm"] == pytest.approx(compliant["t1"] - rigid["t0"], abs=2e-3)
    assert outcome["switch"]["ok"] and outcome["switch"]["back_ok"] and outcome["switch"]["to_s"] >= 0.0
    assert switch["t0"] <= switch["t1"] <= compliant["t0"]
    assert commands(lab, "left_ur_arm") == [STOP, TIGHTEN_SCREW, STOP] and commands(lab, "right_ur_arm") == [STOP, STOP]
    for arm in lab.cindy.arms.values():
        assert arm.controllers.switches == [COMPLIANCE, JOINT]
        assert arm._target_frame.messages and arm._target_wrench.messages
    assert back_under_joint_control(lab)
    assert set(sample["phase"] for sample in outcome["stream"]) >= {
        "rigid", "compliant", "compliant right_ur_arm", "settle right_ur_arm"}
    # * The left arm stops about 1.1 s into its 1.7 s line, some 2 mm short; the right arm finishes its line.
    assert all(error["remaining_mm"] < 2.5 for error in outcome["end_error_mm"].values())
    assert outcome["end_error_mm"]["right_ur_arm"]["remaining_mm"] < 0.1
    assert outcome.get("base_drift_max_mm", 0.0) == pytest.approx(0.0, abs=1e-6)
    assert result.end is not None and len(result.end) == 12


def test_b20s_ground_bar_insert_is_rigid_with_no_tighten():
    """B20_J_insert: a ground bar, so RIGID_DONE whatever the split and the spiral; no TIGHTEN, no switch, DONE.

    Its line says so, the record keeps the end gap; the servo after it (M7) finds nothing left.
    """
    lab = compliant_lab(("B20_J", 4), RunSettings(spiral=True))
    result, _held = execute(lab, along(lab, "B20_J_insert", INSERT_STEP))
    assert result.status == DONE and result.done == (4, SERVO_INDEX), result.reason
    assert said(lab)[0].startswith("cindy (a200-0806): B20_J_insert — RIGID insert of a ground bar, no tighten, ")
    # * The servo's records come after the insert's.
    record = json.loads(json.dumps(next(file for file in reversed(lab.run.files())
                                        if file.get("part", {}).get("kind") == "insert")))
    outcome = record["outcome"]
    assert record["part"]["kind"] == "insert" and outcome["status"] == RIGID_DONE
    assert "rigid_only" not in outcome["settings"] and outcome["settings"]["spiral"] is True
    plan = outcome["plan"]
    assert plan["grounded"] and plan["mating"] == [] and plan["spiral"] is None
    assert plan["achieved_mm"] == 0.0 and plan["compliant_s"] == 0.0 and plan["rigid_s"] > 0.0
    assert [(phase["name"], phase["end"]) for phase in outcome["phases"]] == [("rigid", "ARRIVED")]
    assert outcome["end_gap_mm"] < 0.5 and outcome["stall_times"] == {}
    for arm in lab.tools:
        assert commands(lab, arm) == []  # * no tool is driven for a ground bar, not even a STOP
    for arm in lab.cindy.arms.values():
        assert arm.controllers.switches == [] and not arm._target_frame.messages
    assert "to_s" not in outcome["switch"] and back_under_joint_control(lab)
    # * One rigid motion for both arms; every ending holds each arm (one single-point trajectory each), nothing more.
    assert len(lab.cindy._multi_arm.messages) == 1 and lab.published() == 3
    assert all([len(message.points) for message in arm._trajectory.messages] == [1]
               for arm in lab.cindy.arms.values())
    assert lab.run.files()[-1]["outcome"]["status"] == "nothing left"  # * the servo part after the ground bar


def test_b20s_retreat_breaks_out_compliant_then_runs_the_tail_replanned_to_the_goals():
    """B20_R_retreat: the M1 screws open, a compliant break-out, then the tail planned to the plan's goals; DONE.

    The replan asks for the plan's goals in 1 mm steps; the record has every phase and the tail; the ungrasp folded
    into the step is done; every arm ends under the joint controller at the tail's end.
    """
    lab = compliant_lab(("B20_R", 1))
    request = along(lab, "B20_R_retreat", RETREAT_STEP, goals=True)
    start = np.array(request.route.result.trajectory.positions[0])
    planner = lab.ctx.plugins["motion_planner"] = TailPlanner(lab, start + RETREAT_STEP)
    result, _held = execute(lab, request)
    assert result.status == DONE and result.done == (1, 0), result.reason
    assert not prompts(lab) and said(lab) == [said(lab)[0]] and said(lab)[0].startswith(
        "cindy (a200-0806): B20_R_retreat — RETREAT: the M1 screws of left_ur_arm + right_ur_arm OPEN; force sensors "
        "zeroed; COMPLIANT break-out ")
    asked, = planner.requests
    assert asked.goal_poses == request.route.goals and asked.linear_step == TAIL_STEP
    retreat, ungrasp = [json.loads(json.dumps(file)) for file in lab.run.files()]
    outcome = retreat["outcome"]
    assert retreat["part"]["kind"] == "retreat" and outcome["status"] == RETREATED
    plan = outcome["plan"]
    assert plan["split_mm"] == 5.0 and plan["compliant_mm"] == pytest.approx(5.0, abs=1e-3)
    assert plan["total_mm"] > 5.5
    assert plan["compliant_s"] >= 0.5 and plan["tail_s"] >= 0.5
    assert [phase["name"] for phase in outcome["phases"]] == [
        "open", "force zero", "break-out", "joint control", "replan", "tail"]
    assert [phase["end"] for phase in outcome["phases"]] == ["sent", "done", "done", "back", "solved", "ARRIVED"]
    tail = outcome["tail"]
    assert tail["planner"] == "fake" and tail["waypoints"] == 20 and tail["seconds"] >= plan["tail_s"] - 1e-9
    assert tail["outcome"]["status"] == "ARRIVED"
    assert outcome["checks"] and all(line.startswith(("PASS ", "FAIL ")) for line in outcome["checks"])
    assert outcome["switch"]["ok"] and outcome["switch"]["back_ok"]
    assert all(error["remaining_mm"] <= 2.0 for error in outcome["end_error_mm"].values())
    assert {sample["phase"] for sample in outcome["stream"]} >= {"break-out", "tail"}
    assert outcome.get("base_drift_max_mm", 0.0) == pytest.approx(0.0, abs=1e-6)
    for arm in lab.tools:
        assert commands(lab, arm)[:2] == [STOP, OPEN_GRIPPER] and commands(lab, arm)[-1] == STOP
    for arm in lab.cindy.arms.values():
        assert arm.controllers.switches == [COMPLIANCE, JOINT]
    assert back_under_joint_control(lab)
    live = np.concatenate([arm.joint_vector() for arm in lab.cindy.arms.values()])
    np.testing.assert_allclose(live, start + RETREAT_STEP, atol=1e-3)
    assert ungrasp["movement"] == "B20_R_ungrasp" and ungrasp["outcome"]["status"] == "opened"
