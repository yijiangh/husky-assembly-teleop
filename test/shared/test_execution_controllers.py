"""Switching fake arms between joint and compliance control and back: refusals, failures, late answers, cancels."""

from __future__ import annotations

import asyncio

import numpy as np
import pytest
from geometry_msgs.msg import PoseStamped
from plugin_fakes import FakeContext, fake_arm, fake_node, fake_robot, lease_to_test

from husky_assembly_teleop.robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER as COMPLIANCE
from husky_assembly_teleop.robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER as JOINT
from husky_assembly_teleop.shared.execution.controllers import (QUERY_TIMEOUT, RESTORE_TIMEOUT, SWITCH_TIMEOUT,
                                                                leave_compliance, restore_joint_control, switch_arms)

pytestmark = pytest.mark.safety

START = np.array([0.0, -1.5, 1.5, 0.0, 0.5, 0.0])
LABEL = "Cindy (a200-0806): B21 insert"
#: A FakeContext tick, seconds: a wait may end one tick late.
TICK = 0.1


class Lab:
    """Cindy's two arms under `active`, leased to the test, reporting every tick; their controller managers tick too."""

    def __init__(self, active: str = JOINT):
        """Build the arms; their controller managers answer at once until a test says otherwise."""
        self.ctx = FakeContext(step=self.tick)
        node = fake_node(self.ctx.now, self.ctx.logs)
        self.robot = lease_to_test(fake_robot(node, "a200-0806", [fake_arm(node, name, START, active)
                                                                  for name in ("left_ur_arm", "right_ur_arm")]))
        self.arms = list(self.robot.arms.values())
        self.left, self.right = (arm.controllers for arm in self.arms)
        self.tick(self.ctx.t)
        # * Their last poll was answered half a second ago, so only a new answer counts as one.
        for arm in self.arms:
            arm.state.controllers.last_update_time = self.ctx.t - 0.5

    def tick(self, t: float) -> None:
        """Every arm reports its joints and TCP; every controller manager answers what is due."""
        tcp = PoseStamped()
        tcp.pose.position.x, tcp.pose.position.z, tcp.pose.orientation.w = 0.4, 0.3, 1.0
        for arm in self.arms:
            arm.state.last_update_time = t
            arm._on_tcp_pose(tcp)
            arm.controllers.tick(t)

    def delay(self, ticks: int | None) -> None:
        """Both controller managers answer `ticks` ticks after each request (None: never)."""
        self.left.delay = self.right.delay = ticks

    def running(self) -> list[str | None]:
        """What each controller manager runs."""
        return [self.left.running, self.right.running]


def test_a_switch_waits_until_every_arm_runs_the_controller():
    """Asked what runs, then switched: ok once both answered, each switched once."""
    lab = Lab()
    lab.delay(3)
    outcome = asyncio.run(switch_arms(lab.ctx, lab.arms, COMPLIANCE))
    assert outcome.ok and outcome.active == {"left_ur_arm": COMPLIANCE, "right_ur_arm": COMPLIANCE}
    assert outcome.seconds == pytest.approx(0.6) and lab.running() == [COMPLIANCE, COMPLIANCE]
    assert lab.left.switches == lab.right.switches == [COMPLIANCE]
    assert any(f"{COMPLIANCE} running" in line for line in lab.ctx.lines("info"))


def test_arms_already_under_the_controller_are_not_switched():
    """Both already run it: ok, no switch asked. Only the right one: only the left is switched."""
    lab = Lab(active=COMPLIANCE)
    outcome = asyncio.run(switch_arms(lab.ctx, lab.arms, COMPLIANCE))
    assert outcome.ok and "already under" in outcome.reason and lab.left.switches == lab.right.switches == []

    lab = Lab()
    lab.right.running = COMPLIANCE  # the controller manager says so when asked
    assert asyncio.run(switch_arms(lab.ctx, lab.arms, COMPLIANCE)).ok
    assert lab.left.switches == [COMPLIANCE] and lab.right.switches == []


def test_a_switch_in_flight_refuses_before_anything_is_asked():
    """The right arm still switching: refused at once; nothing asked of either controller manager."""
    lab = Lab()
    lab.arms[1].state.controllers.switch_in_flight = JOINT
    outcome = asyncio.run(switch_arms(lab.ctx, lab.arms, COMPLIANCE))
    assert not outcome.ok and f"still switching right_ur_arm to {JOINT}" in outcome.reason
    assert outcome.seconds == 0.0 and lab.left.switches == lab.right.switches == []
    assert lab.left._list_client.calls == lab.right._list_client.calls == []
    assert outcome.reason in lab.ctx.lines("error")[-1]


def test_a_controller_manager_that_is_down_fails_at_once():
    """No switch service: nothing goes out, FAILED at once, without waiting for an answer."""
    lab = Lab()
    lab.left.ready = lab.right.ready = False
    outcome = asyncio.run(switch_arms(lab.ctx, lab.arms, COMPLIANCE))
    assert not outcome.ok and "switch_controller is not available" in outcome.reason and outcome.seconds == 0.0
    assert lab.running() == [JOINT, JOINT] and lab.left._switch_client.calls == []


def test_a_switch_rejected_partway_fails_and_leaving_brings_the_switched_arm_back():
    """Left accepted, right rejected: FAILED at the answer, not after 5 s; `leave_compliance` restores the left."""
    lab = Lab()
    lab.delay(2)
    lab.right.rejects = True
    outcome = asyncio.run(switch_arms(lab.ctx, lab.arms, COMPLIANCE))
    assert not outcome.ok and "rejected" in outcome.reason and outcome.seconds == pytest.approx(0.4)
    assert lab.running() == [COMPLIANCE, JOINT]

    assert asyncio.run(leave_compliance(lab.ctx, lab.arms, label=LABEL))
    assert lab.running() == [JOINT, JOINT]
    assert lab.left.switches == [COMPLIANCE, JOINT] and lab.right.switches == [COMPLIANCE]
    assert [len(arm._target_frame.messages) for arm in lab.arms] == [1, 0]  # held at its TCP before the switch
    assert len(lab.arms[1]._trajectory.messages) == 1  # the right one held under joint control


def test_a_switch_never_answered_fails_after_the_timeout_and_a_late_answer_is_switched_back():
    """No answer: FAILED after the 2 s query and the 5 s switch. The answer comes during the restore: switched back."""
    lab = Lab()
    lab.delay(None)

    async def scenario():
        outcome = await switch_arms(lab.ctx, lab.arms, COMPLIANCE)
        in_flight = [arm.state.controllers.switch_in_flight for arm in lab.arms]
        lab.delay(3)
        return outcome, in_flight, await leave_compliance(lab.ctx, lab.arms, label=LABEL)

    outcome, in_flight, restored = asyncio.run(scenario())
    assert not outcome.ok and f"no answer within {SWITCH_TIMEOUT:.0f} s" in outcome.reason
    assert outcome.seconds == pytest.approx(QUERY_TIMEOUT + SWITCH_TIMEOUT, abs=TICK) and in_flight == [COMPLIANCE] * 2
    assert restored and lab.running() == [JOINT, JOINT]
    assert lab.left.switches == lab.right.switches == [COMPLIANCE, JOINT]
    warnings = lab.ctx.lines("warn")
    assert f"not asked, a switch in flight: left_ur_arm (to {COMPLIANCE}), right_ur_arm (to {COMPLIANCE})" in warnings
    late = [line for line in warnings if f"runs {COMPLIANCE}; switch back to {JOINT} requested" in line]
    assert len(late) == 2 and all(line.startswith(LABEL) for line in late)


def test_leave_compliance_still_requests_joint_control_when_cancelled_twice():
    """Cancelled, the part's `finally` holds and asks at once; a second cancel ends the part, not the restore."""
    lab = Lab(active=COMPLIANCE)
    lab.delay(5)

    async def scenario():
        async def part():
            try:
                await lab.ctx.wait_until(lambda: False)
            finally:
                await leave_compliance(lab.ctx, lab.arms, label=LABEL)

        task = asyncio.ensure_future(part())
        await asyncio.sleep(0)
        task.cancel()
        await asyncio.sleep(0)
        asked = [arm.state.controllers.switch_in_flight for arm in lab.arms]
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        restore, = (task for task in lab.ctx._tasks if task.get_name() == f"restore joint control: {LABEL}")
        return asked, await restore

    asked, restored = asyncio.run(scenario())
    assert asked == [JOINT, JOINT] and restored and lab.running() == [JOINT, JOINT]
    assert [len(arm._target_frame.messages) for arm in lab.arms] == [1, 1]
    assert any(line.startswith(f"{LABEL}: left_ur_arm, right_ur_arm back under {JOINT}")
               for line in lab.ctx.lines("info"))


def test_two_stop_alls_with_a_switch_in_flight_leave_the_restore_running():
    """Stop all twice during a pending switch to compliance: the part ends; the restore survives, switches it back."""
    lab = Lab()
    lab.delay(None)

    def restores() -> list[asyncio.Task]:
        return [task for task in lab.ctx._tasks if task.get_name() == f"restore joint control: {LABEL}"]

    async def scenario():
        async def part():
            try:
                await switch_arms(lab.ctx, lab.arms, COMPLIANCE)
            finally:
                await leave_compliance(lab.ctx, lab.arms, label=LABEL)

        task = lab.ctx.spawn("insert", part())
        while lab.right.switches != [COMPLIANCE] and not task.done():
            await asyncio.sleep(0)
        lab.robot.soft_stop_latch.stopped = True
        first = lab.ctx._cancel_for_soft_stop()
        while not restores() and not task.done():
            await asyncio.sleep(0)
        second = lab.ctx._cancel_for_soft_stop()
        with pytest.raises(asyncio.CancelledError):
            await task
        lab.delay(3)  # the switch to compliance is answered late
        restore, = restores()
        return first, second, await restore

    first, second, restored = asyncio.run(scenario())
    assert "insert" in first and "insert" in second and f"restore joint control: {LABEL}" not in second
    assert restored and lab.running() == [JOINT, JOINT]
    assert lab.left.switches == lab.right.switches == [COMPLIANCE, JOINT]
    assert not any("ARMS MAY BE LEFT" in line for line in lab.ctx.lines("error"))


def test_a_cancelled_restore_logs_the_loud_error():
    """Cancelled (plugin stop, shutdown) before the arms are back: the loud error for each one not back, then raised."""
    lab = Lab(active=COMPLIANCE)
    lab.delay(None)

    async def scenario():
        task = asyncio.ensure_future(restore_joint_control(lab.ctx, lab.arms, label=LABEL))
        while lab.ctx.t < 11.0 and not task.done():
            await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    error = lab.ctx.lines("error")[-1]
    assert error.startswith(f"{LABEL}: ARMS MAY BE LEFT UNDER {COMPLIANCE}: left_ur_arm (runs {COMPLIANCE}, "
                            f"switching to {JOINT}), right_ur_arm")
    assert "when the wait for them was cancelled" in error


def test_a_restore_that_never_lands_ends_in_the_loud_error():
    """Controller manager down: asked about once a second, then after 10 s the error to switch them by hand."""
    lab = Lab(active=COMPLIANCE)
    lab.left.ready = lab.right.ready = False
    started = lab.ctx.t
    assert not asyncio.run(leave_compliance(lab.ctx, lab.arms, label=LABEL))
    assert lab.ctx.t - started == pytest.approx(RESTORE_TIMEOUT, abs=TICK)
    assert 2 <= len(lab.left.switches) <= 12 and set(lab.left.switches) == {JOINT}
    error = lab.ctx.lines("error")[-1]
    assert error.startswith(f"{LABEL}: ARMS MAY BE LEFT UNDER {COMPLIANCE}: left_ur_arm (runs {COMPLIANCE})")
    assert error.endswith("switch them in robot_control before anything else")
    # * Nothing went out, and no line says it did.
    claims = [line for line in lab.ctx.lines("info") + lab.ctx.lines("warn") if "switch back to" in line]
    assert claims and all("NOT SENT" in line and "requested" not in line for line in claims)


def test_a_refused_hold_is_named_and_leaving_is_not_clean():
    """The left TCP stale as the part ends: its hold is refused, the one warn names it, and leaving says False.

    Both arms are asked for joint control and get it all the same.
    """
    lab = Lab(active=COMPLIANCE)
    lab.arms[0].state.tcp_update_time = lab.ctx.t - 1.0
    assert not asyncio.run(leave_compliance(lab.ctx, lab.arms, label=LABEL))
    assert lab.running() == [JOINT, JOINT] and lab.left.switches == lab.right.switches == [JOINT]
    assert [len(arm._target_frame.messages) for arm in lab.arms] == [0, 1]
    named = [line for line in lab.ctx.lines("warn") if line.startswith("the hold was refused on left_ur_arm (")]
    assert len(named) == 1 and f"left_ur_arm, right_ur_arm: switch back to {JOINT} requested" in named[0]
    assert any("HOLD NOT SENT, TCP pose missing or stale" in line for line in lab.ctx.lines("error"))


def test_leaving_while_the_plugin_closes_requests_and_says_it_is_not_confirmed():
    """No task can start: held and asked at once all the same, and the log says it is not confirmed."""
    lab = Lab(active=COMPLIANCE)
    lab.delay(None)
    lab.ctx.closed = True
    assert not asyncio.run(leave_compliance(lab.ctx, lab.arms, label=LABEL))
    assert [arm.state.controllers.switch_in_flight for arm in lab.arms] == [JOINT, JOINT]
    assert [len(arm._target_frame.messages) for arm in lab.arms] == [1, 1] and not lab.ctx._tasks
    error = lab.ctx.lines("error")[-1]
    assert f"not confirmed, the plugin is closing (left_ur_arm, right_ur_arm: switch back to {JOINT} requested)" \
        in error
