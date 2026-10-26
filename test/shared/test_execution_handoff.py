"""The compliant Robotiq close on a fake Alice with her real frames: anchor, close, drift stop and endings."""

from __future__ import annotations

import asyncio
import json
import math
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import ALICE, robot_config
from plugin_fakes import (ArmDriver, FakeContext, MocapDriver, RobotiqDriver, fake_arm, fake_node, fake_robot,
                          fake_robotiq, lease_to_test)
from robot_lab import SUPPORT
from test_execution_stream import BASE, MOCAP_ID, moved

from husky_assembly_teleop.robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER as COMPLIANCE
from husky_assembly_teleop.robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER as JOINT
from husky_assembly_teleop.robot_interface.frames import HUSKY_FRAME, MOCAP_FRAME, fixed_transform
from husky_assembly_teleop.shared.execution.handoff import MAX_DRIFT, TIMEOUT, handoff_text, run_handoff
from husky_assembly_teleop.shared.execution.stream import CLOSED, DRY_RUN, FAILED, REFUSED, STIFF
from husky_assembly_teleop.shared.execution.tools import ROBOTIQ_CLOSE, ROBOTIQ_EFFORT

pytestmark = pytest.mark.safety


class Lab:
    """Alice with her URDF frames under joint control at BASE, her Robotiq open, leased to the test.

    Mocap sees her base, the arm and the gripper report, every tick. Under compliance the TCP sits on the last target
    (lag 1), plus `push` from ROS time `push_at` on: a hand on the arm (or, pushed from the start, frames that
    disagree). `events` keeps every target, wrench and gripper goal, in order.
    """

    def __init__(self, reject_goals: bool = False, **gripper):
        """Build Alice; `gripper` options go to the RobotiqDriver; `reject_goals`: the gripper rejects every goal."""
        self.ctx = FakeContext(step=self.tick)
        node = fake_node(self.ctx.now, self.ctx.logs)
        config = robot_config(ALICE)
        self.gripper = fake_robotiq(node)
        self.arm = fake_arm(node, "ur_arm", SUPPORT, end_effector=self.gripper)
        self.arm.urdf = config.model.urdf
        self.robot = lease_to_test(fake_robot(node, "a200-0804", [self.arm]))
        self.robot.config.model = config.model
        self.robot._husky_in_mocap_frame = fixed_transform(config.model.urdf, MOCAP_FRAME, HUSKY_FRAME)
        self.mocap = MocapDriver(self.robot, BASE, MOCAP_ID)
        self.drivers = [ArmDriver(self.arm, self.robot, lag=1.0), RobotiqDriver(self.gripper, **gripper)]
        self.push, self.push_at = np.zeros(3), math.inf
        self.events: list[tuple[str, object]] = []
        self.arm.targets_sent.attach(lambda target: self.events.append(("target", target)))
        self.arm.wrenches_sent.attach(lambda wrench: self.events.append(("wrench", wrench)))
        send = self.gripper._action.send_goal_async

        def goal(message):
            """Keep the goal as an event; the gripper accepts it unless `reject_goals`."""
            self.events.append(("goal", message.command.position))
            answer = send(message)
            self.gripper._action.goals[-1].accepted = not reject_goals
            return answer

        self.gripper._action.send_goal_async = goal
        self.tick(self.ctx.t)

    def tick(self, t: float) -> None:
        """Mocap sees the base, the arm and the gripper move and report; the push is added to the TCP."""
        self.mocap.tick(t)
        for driver in self.drivers:
            driver.tick(t)
        if t >= self.push_at - 1e-9:
            self.arm.state.tcp_position = self.arm.state.tcp_position + self.push

    def run(self, dry_run: bool = False):
        """Run the handoff to its end."""
        return asyncio.run(run_handoff(self.ctx, self.robot, "ur_arm", self.gripper, dry_run=dry_run))

    def sent(self, kind: str) -> list:
        """Every event of one kind: "target", "wrench" or "goal"."""
        return [value for each, value in self.events if each == kind]

    def kinds(self) -> list[str]:
        """The kinds of every event, in order."""
        return [each for each, _value in self.events]


def test_the_gripper_closes_on_the_bar_with_the_arm_compliant_around_its_anchor():
    """F/T zeroed, compliance, the anchor and a zero wrench BEFORE the close; stalled on the bar → CLOSED, JTC."""
    lab = Lab(reach=lambda target: target - 0.3)
    anchor = lab.arm.state.tcp_position.copy()
    outcome = lab.run()
    assert outcome.status == CLOSED and outcome.ok and "0.500 rad" in outcome.reason
    assert len(lab.arm._zero_ft.calls) == 1
    assert lab.arm.controllers.switches == [COMPLIANCE, JOINT] and lab.arm.state.controllers.active == JOINT
    assert lab.kinds()[:3] == ["target", "wrench", "goal"] and lab.sent("goal") == [ROBOTIQ_CLOSE]
    held = [target for target in lab.sent("target") if not target.hold]
    assert len(held) > 3 and all(np.allclose(target.position, anchor, atol=1e-12) for target in held)
    assert all(not np.any(wrench.force) and not np.any(wrench.torque) for wrench in lab.sent("wrench"))
    assert lab.sent("target")[-1].hold  # the way out: hold at the TCP, then joint control
    record = outcome.record
    assert record["status"] == CLOSED and record["fk_tcp_gap"]["mm"] < 1e-3 and record["drift_max_mm"] < 1e-6
    assert record["switch"]["ok"] and record["switch"]["back_ok"] and record["gripper"]["result_ok"]
    assert record["stream"] and record["stream"][0]["phase"] == "close"
    json.dumps(record)


def test_the_record_keeps_the_furthest_the_base_moved():
    """The base moves 1.5 mm during the close: `base_drift_max_mm` says so (the anchor is in base_link: no stop)."""
    lab = Lab(delay=2.0)
    shifted = moved(BASE, (0.0015, 0.0, 0.0))
    lab.drivers.append(SimpleNamespace(tick=lambda t: setattr(lab.mocap, "pose", shifted) if t > 11.0 else None))
    outcome = lab.run()
    assert outcome.status == CLOSED and outcome.record["base_drift_max_mm"] == pytest.approx(1.5, abs=1e-6)


@pytest.mark.parametrize("push_mm", [10.5, 9.5])
def test_the_hold_ends_once_the_arm_moved_over_10_mm_from_its_anchor(push_mm):
    """Pushed 10.5 mm mid-close: FAILED, the fingers stopped, joint control. Pushed 9.5 mm: it gives way and closes."""
    lab = Lab(delay=2.0)
    lab.push, lab.push_at = np.array([push_mm * 1e-3, 0.0, 0.0]), 11.0
    outcome = lab.run()
    if push_mm * 1e-3 > MAX_DRIFT:
        assert outcome.status == FAILED and f"moved {push_mm:.1f} mm" in outcome.reason
        assert lab.gripper._action.goals[0].cancelled
    else:
        assert outcome.status == CLOSED and outcome.record["drift_max_mm"] == pytest.approx(push_mm)
        assert not lab.gripper._action.goals[0].cancelled
    assert lab.arm.state.controllers.active == JOINT


def test_an_anchor_11_mm_from_the_urdf_tool0_closes_stiff():
    """Measured TCP 11 mm off the URDF's tool0: no compliance, no target; the gripper closes stiff → STIFF, not ok."""
    lab = Lab()
    lab.push, lab.push_at = np.array([0.0, 0.0, 0.011]), -math.inf
    lab.tick(lab.ctx.t)
    outcome = lab.run()
    assert outcome.status == STIFF and not outcome.ok and "11.0 mm" in outcome.reason
    assert lab.sent("goal") == [ROBOTIQ_CLOSE] and lab.sent("target") == []
    assert lab.arm.controllers.switches == [] and outcome.record["stiff_close"]["status"] == "reached"


def test_no_answer_within_30_s_fails_and_stops_the_fingers():
    """The gripper never answers: FAILED after 30 s, its goal cancelled once accepted, joint control."""
    lab = Lab(answer=False)
    outcome = lab.run()
    assert outcome.status == FAILED and f"within {TIMEOUT:.0f} s" in outcome.reason
    assert TIMEOUT < outcome.seconds < TIMEOUT + 1.0
    assert lab.gripper._cancel_on_accept and lab.arm.state.controllers.active == JOINT


def test_a_closed_gripper_whose_arm_is_not_brought_back_fails():
    """The controller manager stops answering once the arm is compliant: closed, but FAILED, said loudly."""
    lab = Lab()
    lab.drivers.append(SimpleNamespace(tick=lambda t: setattr(lab.arm.controllers, "delay", None) if t > 10.7 else 0))
    outcome = lab.run()
    assert outcome.status == FAILED and "may be left under" in outcome.reason
    assert outcome.record["gripper"]["result_ok"] and not outcome.record["switch"]["back_ok"]
    assert any("ARMS MAY BE LEFT UNDER" in line for line in lab.ctx.lines("error"))


def test_a_gripper_that_reports_no_success_fails():
    """The close goal rejected: FAILED, not CLOSED."""
    lab = Lab(reject_goals=True)
    outcome = lab.run()
    assert outcome.status == FAILED and "did not report success" in outcome.reason
    assert lab.arm.state.controllers.active == JOINT


def test_a_close_cancelled_from_outside_is_not_closed():
    """The last goal succeeded; this close is cancelled from outside (a Stop in robot_control): FAILED, not CLOSED."""
    lab = Lab(delay=2.0)
    lab.gripper.state.last_result_ok = True
    lab.drivers.append(SimpleNamespace(tick=lambda t: lab.gripper.stop() if t > 10.85 else None))
    outcome = lab.run()
    assert outcome.status == FAILED and "did not report success" in outcome.reason
    assert lab.gripper._action.goals[0].cancelled and lab.arm.state.controllers.active == JOINT


def test_a_cancel_stops_the_fingers_holds_the_arm_and_brings_it_back():
    """Cancelled mid-close: the goal cancelled, a hold at the TCP, joint control, then CancelledError."""
    lab = Lab(delay=10.0)

    async def scenario():
        task = asyncio.ensure_future(run_handoff(lab.ctx, lab.robot, "ur_arm", lab.gripper))
        while lab.ctx.t < 11.5 and not task.done():
            await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    assert lab.gripper._action.goals[0].cancelled
    assert lab.sent("target")[-1].hold and lab.arm.state.controllers.active == JOINT
    assert lab.arm.controllers.switches == [COMPLIANCE, JOINT]


def test_a_refused_switch_to_compliance_closes_nothing():
    """The controller manager rejects the switch: FAILED, no target, no close goal."""
    lab = Lab()
    lab.arm.controllers.rejects = True
    outcome = lab.run()
    assert outcome.status == FAILED and "nothing closed" in outcome.reason
    assert lab.sent("goal") == [] and lab.sent("target") == [] and lab.arm.state.controllers.active == JOINT


def test_a_refused_anchor_target_closes_nothing():
    """Stop all lands as the arm goes compliant: the anchor target is refused → FAILED, no close goal, joint control."""
    lab = Lab()
    switch = lab.arm.controllers.switch

    def stop_all_on_compliance(controller: str, unguarded: bool = False) -> bool:
        """The real switch; the robot soft stopped right after the one to compliance."""
        sent = switch(controller, unguarded)
        if controller == COMPLIANCE:
            lab.robot.soft_stop_latch.stopped = True
        return sent

    lab.arm.controllers.switch = stop_all_on_compliance
    outcome = lab.run()
    assert outcome.status == FAILED and "soft stopped" in outcome.reason and "nothing closed" in outcome.reason
    assert lab.sent("goal") == [] and all(target.hold for target in lab.sent("target"))
    assert lab.arm.state.controllers.active == JOINT


@pytest.mark.parametrize("setup, why", [
    (lambda lab: setattr(lab.gripper._action, "ready", False), "not available"),
    (lambda lab: setattr(lab.gripper.state, "moving", True), "still runs"),
    (lambda lab: setattr(lab.arm.controllers, "running", COMPLIANCE) or setattr(
        lab.arm.state.controllers, "active", COMPLIANCE), "is not under"),
    (lambda lab: setattr(lab.arm, "urdf_problem", "tool0 turned"), "URDF"),
    (lambda lab: setattr(lab.arm, "config", replace(lab.arm.config, cartesian_test_mode=True)), "Cartesian test mode"),
    (lambda lab: lab.robot.base.state.marker_errors.clear(), "base pose lost"),
])
def test_a_refusal_sends_nothing(setup, why):
    """Gripper not ready or busy, arm not under joint control, URDF problem, test mode, mocap lost: nothing sent."""
    lab = Lab()
    setup(lab)
    outcome = lab.run()
    assert outcome.status == REFUSED and why in outcome.reason
    assert lab.events == [] and lab.arm._zero_ft.calls == [] and lab.arm.controllers.switches == []


def test_a_dry_run_sends_nothing():
    """Every check passes: DRY_RUN; no zero, no switch, no target, wrench or goal."""
    lab = Lab()
    outcome = lab.run(dry_run=True)
    assert outcome.status == DRY_RUN and not outcome.ok
    assert lab.events == [] and lab.arm._zero_ft.calls == [] and lab.arm.controllers.switches == []


def test_the_prompt_says_compliant_before_the_close():
    """The question names the arm, the give, the jaws and the ceiling."""
    text = handoff_text("ur_arm")
    assert text.startswith("CLOSE + compliant handoff on ur_arm: the arm goes COMPLIANT before the close")
    assert "≤ 10 mm" in text and f"jaws {ROBOTIQ_CLOSE}, effort {ROBOTIQ_EFFORT}, ≤ 30 s" in text
