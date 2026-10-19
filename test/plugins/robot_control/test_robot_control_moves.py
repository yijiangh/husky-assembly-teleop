"""robot_control's moves on an arm with its real checks: joint moves, streamed Cartesian moves, Hold and the D-pad."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import numpy as np
import pytest
from robot_fakes import TCP_ORIENTATION, TCP_POSITION, fake_arm

from husky_assembly_teleop.plugin_api.concurrency import WaitTimeout
from husky_assembly_teleop.plugins.robot_control.plugin import RobotControlPlugin, _BaseWidgets
from husky_assembly_teleop.robot_interface.arm import (CARTESIAN_COMPLIANCE_CONTROLLER,
                                                       SCALED_JOINT_TRAJECTORY_CONTROLLER, UR_JOINT_NAMES)
from husky_assembly_teleop.robot_interface.base import PLATFORM_VELOCITY_CONTROLLER

pytestmark = pytest.mark.safety

#: Seconds per tick of the fake context.
TICK = 0.05


class Context:
    """A plugin context whose clock is the arm's node clock and advances one TICK per `next_tick`.

    `follow` runs every tick, to play the arm moving.
    """

    def __init__(self, arm, follow=lambda: None):
        """Share `arm`'s clock."""
        self.arm, self.follow, self.lines = arm, follow, []

    def now(self) -> float:
        """The arm's ROS time."""
        return self.arm._node.now

    async def next_tick(self) -> None:
        """Advance time by one tick, and let the arm move."""
        self.arm._node.now += TICK
        self.follow()
        await asyncio.sleep(0)

    async def wait_until(self, predicate, timeout_s=None, description="condition") -> None:
        """Tick until `predicate` holds, or raise WaitTimeout after `timeout_s`."""
        end = self.now() + timeout_s
        while not predicate():
            if self.now() > end:
                raise WaitTimeout(f"timed out waiting for {description}")
            await self.next_tick()

    def spawn(self, label, work):
        """Start `work` as a task."""
        return asyncio.get_running_loop().create_task(work, name=label)

    def log_info(self, message: str) -> None:
        """Keep it."""
        self.lines.append(("info", message))

    def log_warn(self, message: str) -> None:
        """Keep it."""
        self.lines.append(("warn", message))

    def text(self) -> str:
        """Everything logged."""
        return "\n".join(message for _level, message in self.lines)


def tracking(arm):
    """An arm that reaches each Cartesian target by the next tick, measured at once."""
    def follow():
        if arm.state.sent_target_position is not None:
            arm.state.tcp_position = arm.state.sent_target_position
            arm.state.tcp_orientation = arm.state.sent_target_orientation
        arm.state.tcp_update_time = arm.state.last_update_time = arm.state.controllers.last_update_time = arm._node.now
        arm.state.status_update_time = arm._node.now
    return follow


def test_a_streamed_cartesian_move_passes_every_check_and_arrives():
    """10 cm and 10 degrees: every target is within the step limits of the one before, and the last is the goal."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    ctx = Context(arm, tracking(arm))
    goal = TCP_POSITION + [0.1, 0.0, 0.0]
    turned = np.array([0.0, 0.0, np.sin(np.radians(5.0)), np.cos(np.radians(5.0))])
    asyncio.run(RobotControlPlugin()._stream_cartesian_move(ctx, "0804", arm, goal, turned))
    assert "cartesian move done" in ctx.text() and not arm._node.log.text("error")
    assert len(arm._target_frame.sent) > 10
    np.testing.assert_allclose(arm.state.sent_target_position, goal, atol=1e-9)


def test_a_streamed_move_the_arm_refuses_stops_and_holds():
    """When a target is refused mid-move (here the controller changed), the move ends and the arm is held."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER)
    follow, ticks = tracking(arm), [0]

    def then_switch():
        follow()
        ticks[0] += 1
        if ticks[0] == 3:
            arm.state.controllers.active = SCALED_JOINT_TRAJECTORY_CONTROLLER

    ctx = Context(arm, then_switch)
    asyncio.run(RobotControlPlugin()._stream_cartesian_move(ctx, "0804", arm, TCP_POSITION + [0.1, 0, 0],
                                                            TCP_ORIENTATION))
    assert "stopped early" in ctx.text() and len(arm._target_frame.sent) == 3
    assert len(arm._trajectory.sent) == 1  # the hold, under the joint controller now running


def test_test_mode_checks_one_target_and_streams_nothing():
    """cartesian_test_mode: one target is checked at the TCP, nothing is published, and the log says so."""
    arm = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER, cartesian_test_mode=True)
    ctx = Context(arm)
    asyncio.run(RobotControlPlugin()._stream_cartesian_move(ctx, "0804", arm, TCP_POSITION + [0.1, 0, 0],
                                                            TCP_ORIENTATION))
    assert not arm._target_frame.sent and "TEST MODE" in ctx.text()


@pytest.mark.parametrize("arrives", [True, False])
def test_a_joint_move_waits_for_the_arm_and_never_raises(arrives):
    """The move goes out through every check; arrival is logged, and so is a timeout, without raising."""
    arm = fake_arm()
    target = np.radians([20.0, -30.0, 40.0, 0.0, 10.0, 0.0])

    def follow():
        arm.state.last_update_time = arm._node.now
        if arrives:
            for name, value in zip(UR_JOINT_NAMES, target):
                arm.state.joint_positions[f"ur_arm_{name}"] = float(value)

    ctx = Context(arm, follow)
    asyncio.run(RobotControlPlugin()._execute_move(ctx, "0804", arm, target))
    assert len(arm._trajectory.sent) == 1 and not arm._node.log.text("error")
    assert ("arrived" in ctx.text()) is arrives and ("timed out" in ctx.text()) is not arrives


@pytest.mark.parametrize("controller", [SCALED_JOINT_TRAJECTORY_CONTROLLER, CARTESIAN_COMPLIANCE_CONTROLLER])
def test_a_cancelled_move_holds_the_arm(controller):
    """Stop all, the plugin stopping or shutdown cancel the task: the trajectory or the targets must not run on."""
    arm = fake_arm(controller)
    ctx = Context(arm, tracking(arm))
    plugin = RobotControlPlugin()

    async def run():
        if controller == CARTESIAN_COMPLIANCE_CONTROLLER:
            move = plugin._stream_cartesian_move(ctx, "0804", arm, TCP_POSITION + [0.05, 0.0, 0.0], TCP_ORIENTATION)
        else:
            move = plugin._execute_move(ctx, "0804", arm, np.radians([20.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        task = asyncio.create_task(move)
        for _ in range(3):
            await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(run())
    assert "hold sent" in arm._node.log.text("info")


def test_one_move_per_arm_at_a_time_and_hold_cancels_it():
    """A second Start while a move runs is ignored; Hold cancels the running move and holds the arm."""
    arm, plugin = fake_arm(), RobotControlPlugin()
    ctx = Context(arm)

    async def run():
        never = asyncio.Event()

        async def move(_ctx, _serial, _arm):
            await never.wait()

        plugin._start_move(ctx, "0804", arm, "move", move)
        plugin._start_move(ctx, "0804", arm, "move", move)
        task = plugin._trajectory_tasks[("0804", "ur_arm")]
        plugin._hold_arm("0804", arm)
        await asyncio.sleep(0)
        return task

    task = asyncio.run(run())
    assert "already running" in ctx.text() and task.cancelled() and len(arm._trajectory.sent) == 1


def test_hold_also_removes_an_applied_force():
    """After Hold the compliance controller must no longer push with the force set before."""
    arm, plugin = fake_arm(CARTESIAN_COMPLIANCE_CONTROLLER), RobotControlPlugin()
    plugin._apply_force(("0804", "ur_arm"), arm, np.array([0.0, 0.0, 10.0]))
    plugin._hold_arm("0804", arm)
    assert arm._target_wrench.sent[-1].wrench.force.z == 0.0 and not plugin._applied_force


def _driven_base():
    """A plugin with one base whose velocity controller runs, recording its twists."""
    sent, controller = [], [PLATFORM_VELOCITY_CONTROLLER]
    base = SimpleNamespace(controllers=SimpleNamespace(is_active=lambda name: name == controller[0]),
                           send_twist=lambda linear, angular: sent.append((linear, angular)))
    plugin = RobotControlPlugin()
    plugin._bases = [_BaseWidgets(serial="0804", base=base, panel=None, status=None,
                                  speed=SimpleNamespace(value=1.0), phone_speed=SimpleNamespace(value=1.0))]
    return plugin, sent, controller


def test_a_held_base_stops_when_its_controller_goes_off():
    """The hold is dropped as soon as the velocity controller stops, and one stop goes out."""
    plugin, sent, controller = _driven_base()
    ctx = SimpleNamespace(now=lambda: 0.0)
    plugin._on_hold(ctx, "0804", False, 1.0, 0.0)
    plugin.update(ctx)
    controller[0] = None
    plugin.update(ctx)
    plugin.update(ctx)
    assert sent[1:] == [(0.0, 0.0)]


def test_teardown_stops_a_base_that_is_being_driven():
    """Shutting down mid-drive sends a stop."""
    plugin, sent, _controller = _driven_base()
    ctx = SimpleNamespace(now=lambda: 0.0)
    plugin._on_hold(ctx, "0804", False, 0.0, 1.0)
    plugin.update(ctx)
    plugin.teardown(ctx)
    assert sent[-1] == (0.0, 0.0)
