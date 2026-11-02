"""The runtime edits for the executor: a hold under compliance, both arms' velocities."""

from __future__ import annotations

import numpy as np
import pytest
from plugin_fakes import FakeContext, fake_arm, fake_node, fake_robot, lease_to_test

from husky_assembly_teleop.robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER

pytestmark = pytest.mark.safety

HOME = [0.0, -1.5, 1.5, 0.0, 0.0, 0.0]


def compliant_arm(ctx: FakeContext):
    """An arm under the compliance controller, its TCP measured now, a target sent 20 mm away 0.1 s ago."""
    arm = lease_to_test(fake_arm(fake_node(ctx.now, ctx.logs), "ur_arm", HOME, active=CARTESIAN_COMPLIANCE_CONTROLLER))
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
    robot = lease_to_test(fake_robot(node, "a200-0806", [fake_arm(node, "left_ur_arm", HOME),
                                                         fake_arm(node, "right_ur_arm", HOME)]))
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
