"""The stop recovery plugin: rest poses logged, a sag while stopped ignored, Go back slow, limited and force-watched."""

from __future__ import annotations

import asyncio
from datetime import datetime

import numpy as np
import pytest
from robot_lab import RobotLab

from husky_assembly_teleop.plugins.stop_recovery.plugin import GO_BACK, StopRecoveryPlugin
from husky_assembly_teleop.plugins.stop_recovery.pose_log import PoseLog
from husky_assembly_teleop.ui.second_press import DOUBLE_CLICK_SECONDS

BELLE, CINDY = "a200-0805", "a200-0806"
ESTOP = "safety mode ROBOT_EMERGENCY_STOP"


def started(tmp_path) -> tuple[RobotLab, StopRecoveryPlugin]:
    """The three robots and the plugin, logging to `tmp_path`, set up and ticked once."""
    lab = RobotLab()
    plugin = StopRecoveryPlugin()
    plugin.log = PoseLog(tmp_path)
    plugin.setup(lab.ctx)
    for robot in lab.robots.values():
        for arm in robot.arms.values():
            arm.state.wrench = np.array([0.0, 2.0, 0.0, 0.0, 0.0, 0.0])
    return lab, plugin


async def ticks(lab: RobotLab, plugin: StopRecoveryPlugin, seconds: float) -> None:
    """Run the plugin's hooks every tick for `seconds`."""
    until = lab.ctx.t + seconds
    while lab.ctx.t < until:
        await lab.ctx.next_tick()
        plugin.update(lab.ctx)
        plugin.draw(lab.ctx)
        await asyncio.sleep(0)


def sag(arm, degrees: float, force: float) -> None:
    """Move an arm's shoulder lift and elbow by `degrees`, as its brakes would let it, and set its force."""
    for joint in ("shoulder_lift_joint", "elbow_joint"):
        arm.state.joint_positions[f"{arm.config.name}_{joint}"] += np.radians(degrees)
    arm.state.wrench = np.array([0.0, force, 0.0, 0.0, 0.0, 0.0])


def estop(robot, on: bool) -> None:
    """Trip or release an e-stop on every arm of a robot, as the safety sync reports it."""
    for arm in robot.arms.values():
        arm.state.operational, arm.state.problem = (not on), (ESTOP if on else "")


async def press_twice(lab: RobotLab, plugin: StopRecoveryPlugin, serial: str) -> None:
    """Press Go back, then again once a double click would no longer count."""
    go = plugin._widgets[serial]["go"]
    go.callback(None)
    await ticks(lab, plugin, DOUBLE_CLICK_SECONDS + 0.1)
    go.callback(None)
    await ticks(lab, plugin, 0.2)  # the move's task starts


def message(plugin: StopRecoveryPlugin, serial: str) -> str:
    """The text under a robot's buttons."""
    return plugin._messages.get(serial, ("", ""))[0]


async def stop_and_sag(lab: RobotLab, plugin: StopRecoveryPlugin, serial: str, degrees: float = 0.5,
                       force: float = 27.0) -> np.ndarray:
    """Log the robot's pose, e-stop it, let its arms sag, release and resume; the joints from before."""
    robot = lab.robots[serial]
    await ticks(lab, plugin, 0.5)
    before = {name: arm.joint_vector().copy() for name, arm in robot.arms.items()}
    estop(robot, True)
    await ticks(lab, plugin, 0.3)
    for arm in robot.arms.values():
        sag(arm, degrees, force)
    await ticks(lab, plugin, 2.0)
    estop(robot, False)
    await ticks(lab, plugin, 2.0)
    return before


def test_the_pose_before_a_stop_stays_the_newest_and_the_sag_is_never_logged(tmp_path):
    """The sag while e-stopped is not a pose; the stop is logged and named in the newest pose's label."""
    lab, plugin = started(tmp_path)

    async def scenario():
        before = await stop_and_sag(lab, plugin, BELLE)
        poses = plugin.log.poses[BELLE]
        assert len(poses) == 1 and np.allclose(poses[0].arms["ur_arm"].joints, before["ur_arm"])
        assert plugin.log.stops[BELLE][0].reason == ESTOP
        assert plugin.log.stops[BELLE][0].forces["ur_arm"] == pytest.approx(2.0)  # at the trip, before the sag
        label = plugin._widgets[BELLE]["pick"].value
        assert "newest" in label and "before stop" in label

    asyncio.run(scenario())


@pytest.mark.safety
def test_go_back_asks_first_then_moves_slowly_back_and_reports_the_force(tmp_path):
    """The first press asks; the second sends one slow move back, and the force is reported."""
    lab, plugin = started(tmp_path)

    async def scenario():
        before = await stop_and_sag(lab, plugin, BELLE)
        arm = lab.belle.arms["ur_arm"]
        go = plugin._widgets[BELLE]["go"]
        go.callback(None)
        assert "again" in message(plugin, BELLE) and lab.published() == 0
        await ticks(lab, plugin, DOUBLE_CLICK_SECONDS + 0.1)
        go.callback(None)
        await ticks(lab, plugin, 0.2)
        assert len(arm._trajectory.messages) == 1
        sent = arm._trajectory.messages[-1]
        assert sent.points[-1].time_from_start.sec >= 8  # slow, whatever the size
        arm.state.wrench = np.array([0.0, 2.5, 0.0, 0.0, 0.0, 0.0])  # the load comes off
        await ticks(lab, plugin, 10.0)
        assert np.allclose(arm.joint_vector(), before["ur_arm"], atol=1e-6)
        assert message(plugin, BELLE).startswith("Back at") and "27.0 → 2.5 N" in message(plugin, BELLE)

    asyncio.run(scenario())


@pytest.mark.safety
def test_a_two_arm_robot_moves_both_arms_back_together(tmp_path):
    """Cindy's two arms go back in one multi-arm trajectory."""
    lab, plugin = started(tmp_path)

    async def scenario():
        before = await stop_and_sag(lab, plugin, CINDY)
        await press_twice(lab, plugin, CINDY)
        assert len(lab.cindy._multi_arm.messages) == 1
        await ticks(lab, plugin, 10.0)
        for name, arm in lab.cindy.arms.items():
            assert np.allclose(arm.joint_vector(), before[name], atol=1e-6)

    asyncio.run(scenario())


@pytest.mark.safety
def test_go_back_is_refused_while_stopped_or_too_far(tmp_path):
    """Nothing is sent beyond the step limit or while the arm is not operational."""
    lab, plugin = started(tmp_path)

    async def scenario():
        await stop_and_sag(lab, plugin, BELLE, degrees=5.0)
        await press_twice(lab, plugin, BELLE)
        assert "use the arm planner" in message(plugin, BELLE) and lab.published() == 0
        estop(lab.belle, True)
        await ticks(lab, plugin, 0.2)
        await press_twice(lab, plugin, BELLE)
        assert "not operational" in message(plugin, BELLE) and lab.published() == 0

    asyncio.run(scenario())


@pytest.mark.safety
def test_a_rising_force_holds_the_robot(tmp_path):
    """A force 5 N over its start value holds the robot and ends the move."""
    lab, plugin = started(tmp_path)

    async def scenario():
        await stop_and_sag(lab, plugin, BELLE)
        arm = lab.belle.arms["ur_arm"]
        await press_twice(lab, plugin, BELLE)
        await ticks(lab, plugin, 1.0)
        arm.state.wrench = np.array([0.0, 33.0, 0.0, 0.0, 0.0, 0.0])
        await ticks(lab, plugin, 0.3)
        assert message(plugin, BELLE).startswith("Held") and len(arm._trajectory.messages) == 2  # the move, a hold
        assert plugin._moves[BELLE].done()

    asyncio.run(scenario())


def test_a_robot_stopped_at_start_logs_no_pose_until_it_moves(tmp_path):
    """A monitor started after a stop must not take the sagged pose for the one to go back to."""
    lab = RobotLab()
    estop(lab.belle, True)
    plugin = StopRecoveryPlugin()
    plugin.log = PoseLog(tmp_path)
    plugin.setup(lab.ctx)

    async def scenario():
        estop(lab.belle, False)
        await ticks(lab, plugin, 2.0)
        assert BELLE not in plugin.log.poses
        arm = lab.belle.arms["ur_arm"]
        sag(arm, 0.5, 2.0)  # now it moves while running
        await ticks(lab, plugin, 2.0)
        assert len(plugin.log.poses[BELLE]) == 1

    asyncio.run(scenario())


def test_poses_and_stops_are_read_back_from_the_day_file(tmp_path):
    """What was logged comes back from the day file."""
    lab, plugin = started(tmp_path)

    async def scenario():
        await stop_and_sag(lab, plugin, BELLE)
        while plugin.log.unwritten or not plugin._writer.done():  # the writer's thread
            await asyncio.sleep(0.05)
            plugin.update(lab.ctx)

    asyncio.run(scenario())
    again = PoseLog(tmp_path)
    again.load(datetime.now(), [BELLE])
    assert again.poses[BELLE] == plugin.log.poses[BELLE]
    assert again.stops[BELLE][0].reason == ESTOP
    assert again.stop_after(again.poses[BELLE][0]) is not None


def test_the_button_waits_amber_after_the_first_press(tmp_path):
    """Go back shows it waits for its second press."""
    lab, plugin = started(tmp_path)

    async def scenario():
        await stop_and_sag(lab, plugin, BELLE)
        plugin._widgets[BELLE]["go"].callback(None)
        plugin.draw(lab.ctx)
        assert plugin._widgets[BELLE]["go"].label == f"{GO_BACK}?"

    asyncio.run(scenario())
