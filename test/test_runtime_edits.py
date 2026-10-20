"""The runtime edits for the executor: a hold under compliance, both arms' velocities, an awaited search, the lease."""

from __future__ import annotations

import asyncio
import dataclasses
import threading

import numpy as np
import pytest
from plugin_fakes import FakeContext, fake_arm, fake_node, fake_robot

from husky_assembly_teleop.robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER
from husky_assembly_teleop.robot_interface.robot import HuskyRobotInterface, Lease
from husky_assembly_teleop.shared.planning.panel import PlannerPlugin, Search
from husky_assembly_teleop.shared.planning.path import TimedPath
from husky_assembly_teleop.shared.planning.search import PlanResult

HOME = [0.0, -1.5, 1.5, 0.0, 0.0, 0.0]


def compliant_arm(ctx: FakeContext):
    """An arm under the compliance controller, its TCP measured now, a target sent 20 mm away 0.1 s ago."""
    arm = fake_arm(fake_node(ctx.now, ctx.logs), "ur_arm", HOME, active=CARTESIAN_COMPLIANCE_CONTROLLER)
    state = arm.state
    state.tcp_position, state.tcp_orientation = np.array([0.4, 0.1, 0.4]), np.array([0.0, 0.0, 0.0, 1.0])
    state.tcp_update_time = ctx.t
    state.sent_target_position, state.sent_target_orientation = np.array([0.42, 0.1, 0.4]), state.tcp_orientation
    state.sent_target_time = ctx.t - 0.1
    return arm


def test_a_hold_is_sent_though_the_last_target_was_further_than_a_step():
    """E1: a target 20 mm from the last one 0.1 s earlier is refused; a hold at the measured TCP is sent."""
    ctx = FakeContext()
    arm = compliant_arm(ctx)
    assert not arm.send_cartesian_target(arm.state.tcp_position, arm.state.tcp_orientation)
    assert "jumps 20.0 mm" in ctx.lines("error")[-1] and not arm._target_frame.messages
    assert arm.hold()
    sent, = arm._target_frame.messages
    assert (sent.pose.position.x, sent.pose.position.y, sent.pose.position.z) == (0.4, 0.1, 0.4)


def test_both_arms_carry_their_velocities():
    """E2: the multi-arm message's points carry each arm's velocities; velocities for other arms are refused."""
    ctx = FakeContext()
    node = fake_node(ctx.now, ctx.logs)
    robot = fake_robot(node, "a200-0806", [fake_arm(node, "left_ur_arm", HOME), fake_arm(node, "right_ur_arm", HOME)])
    positions = np.array([HOME, np.add(HOME, 0.01), np.add(HOME, 0.02)])
    velocities = {"left_ur_arm": np.full((3, 6), 0.01), "right_ur_arm": np.full((3, 6), 0.02)}
    velocities["left_ur_arm"][-1] = velocities["right_ur_arm"][-1] = 0.0
    assert robot.send_multi_arm_trajectory({"left_ur_arm": positions, "right_ur_arm": positions}, 4.0, velocities)
    message, = robot._multi_arm.messages
    assert list(message.trajectory1.points[0].velocities) == [0.01] * 6
    assert list(message.trajectory2.points[1].velocities) == [0.02] * 6
    assert list(message.trajectory2.points[2].velocities) == [0.0] * 6
    assert not robot.send_multi_arm_trajectory({"left_ur_arm": positions, "right_ur_arm": positions}, 4.0,
                                               {"left_ur_arm": velocities["left_ur_arm"]})
    assert len(robot._multi_arm.messages) == 1 and "velocities for" in ctx.lines("error")[-1]


class Panel(PlannerPlugin):
    """The planner panel with nothing of its own."""

    def forget_plan(self) -> None:
        """Nothing kept."""


PATH = TimedPath(np.array([0.0, 1.0]), np.zeros((2, 6)))


def search(work=None, run=None, time_limit: float = 5.0) -> Search:
    """A search by `work` or by `run`."""
    return Search("a200-0806", "plan", "planning", work, time_limit, lambda result: ("found", False), run=run)


def test_a_search_can_be_awaited_on_the_main_thread():
    """E3: a `run` result makes the plan ready; a `run` that never answers times out; `work` is as before."""
    async def found():
        return PlanResult(PATH)

    async def never():
        await asyncio.get_running_loop().create_future()

    panel = Panel()
    asyncio.run(panel._run(search(run=found), threading.Event()))
    assert panel.plan_state == "ready" and panel.path is PATH and panel._message == "found"

    panel = Panel()
    abort = threading.Event()
    panel.plan_state = "planning"
    asyncio.run(panel._run(search(run=never, time_limit=0.05), abort))
    assert panel.plan_state == "none" and panel._message.startswith("timed out") and abort.is_set()

    panel = Panel()
    asyncio.run(panel._run(search(work=lambda: PlanResult(PATH)), threading.Event()))
    assert panel.plan_state == "ready" and panel.path is PATH
    panel._executor.shutdown()


def test_the_lease_is_none_until_taken():
    """E4: a robot holds no lease at first; a lease cannot be changed in place."""
    assert HuskyRobotInterface.lease is None
    assert HuskyRobotInterface.__new__(HuskyRobotInterface).lease is None
    lease = Lease("movement_exec", "B20_R_home", 1.0, lambda: None)
    with pytest.raises(dataclasses.FrozenInstanceError):
        lease.holder = "robot_control"
