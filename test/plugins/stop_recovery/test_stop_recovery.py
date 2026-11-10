"""The stop recovery plugin: rest poses logged, a sag while stopped ignored, Go back slow, limited and force-watched."""

from __future__ import annotations

import asyncio
from datetime import datetime

import numpy as np
import pytest
from robot_lab import RobotLab

from husky_assembly_teleop.plugins.stop_recovery.plugin import GO_BACK, StopRecoveryPlugin
from husky_assembly_teleop.plugins.stop_recovery.pose_log import ArmPose, PoseLog, RobotPose
from husky_assembly_teleop.ui.second_press import DOUBLE_CLICK_SECONDS

BELLE, CINDY = "a200-0805", "a200-0806"
ESTOP = "safety mode ROBOT_EMERGENCY_STOP"
#: The force every arm reads at first, newtons: the sensor's offset and the tool's weight.
REST = 2.0


def started(tmp_path, lab: RobotLab | None = None) -> tuple[RobotLab, StopRecoveryPlugin]:
    """The three robots and the plugin, logging to `tmp_path`, set up; each arm reads REST N along y and has a TCP."""
    lab = lab or RobotLab()
    for robot in lab.robots.values():
        for arm in robot.arms.values():
            load(arm, REST)
            arm.state.tcp_position = np.array([0.4, 0.1, 0.5])
    plugin = StopRecoveryPlugin()
    plugin.log = PoseLog(tmp_path)
    plugin.setup(lab.ctx)
    return lab, plugin


def load(arm, fy: float) -> None:
    """Set an arm's measured force, along y."""
    arm.state.wrench = np.array([0.0, fy, 0.0, 0.0, 0.0, 0.0])


async def ticks(lab: RobotLab, plugin: StopRecoveryPlugin, seconds: float) -> None:
    """Run the plugin's hooks every tick for `seconds`."""
    until = lab.ctx.t + seconds
    while lab.ctx.t < until:
        await lab.ctx.next_tick()
        plugin.update(lab.ctx)
        plugin.draw(lab.ctx)
        await asyncio.sleep(0)


def sag(arm, degrees: float, force: float, tcp_mm: float = 3.0) -> None:
    """Move an arm's shoulder lift and elbow by `degrees` and its TCP by `tcp_mm`, as its brakes would, and load it."""
    for joint in ("shoulder_lift_joint", "elbow_joint"):
        arm.state.joint_positions[f"{arm.config.name}_{joint}"] += np.radians(degrees)
    arm.state.tcp_position = arm.state.tcp_position + np.array([0.0, 0.0, -tcp_mm / 1000])
    load(arm, force)


def estop(robot, on: bool) -> None:
    """Trip or release an e-stop on every arm of a robot, as the safety sync reports it."""
    for arm in robot.arms.values():
        arm.state.operational, arm.state.problem = (not on), (ESTOP if on else "")


async def press(lab: RobotLab, plugin: StopRecoveryPlugin, serial: str) -> None:
    """Press Go back once, and wait until a double click would no longer count."""
    plugin._widgets[serial]["go"].callback(None)
    await ticks(lab, plugin, DOUBLE_CLICK_SECONDS + 0.1)


async def press_twice(lab: RobotLab, plugin: StopRecoveryPlugin, serial: str) -> None:
    """Press Go back twice, then let the move's task start."""
    await press(lab, plugin, serial)
    await press(lab, plugin, serial)


def message(plugin: StopRecoveryPlugin, serial: str) -> str:
    """The text under a robot's buttons."""
    return plugin._messages.get(serial, ("", ""))[0]


def sent(lab: RobotLab, serial: str) -> int:
    """How many trajectories a robot was sent, multi-arm messages included."""
    robot = lab.robots[serial]
    return sum(len(arm._trajectory.messages) for arm in robot.arms.values()) + \
        (len(robot._multi_arm.messages) if robot._multi_arm is not None else 0)


async def stop_and_sag(lab: RobotLab, plugin: StopRecoveryPlugin, serial: str, degrees: float = 0.5,
                       force: float = 27.0, tcp_mm: float = 3.0, sag_first: bool = False) -> dict:
    """Log the robot's pose, e-stop it, let it sag, release and resume; its joints from before, by arm.

    Args:
        lab: The robots.
        plugin: The plugin, ticked.
        serial: The robot to stop.
        degrees: How far its shoulder lift and elbow sag.
        force: The force each arm reads after the sag, newtons, along y.
        tcp_mm: How far its tool0 sags.
        sag_first: Sag one tick before the safety sync reports the stop (its status comes late over Wi-Fi).
    """
    robot = lab.robots[serial]
    await ticks(lab, plugin, 0.5)
    before = {name: arm.joint_vector().copy() for name, arm in robot.arms.items()}
    if sag_first:
        for arm in robot.arms.values():
            sag(arm, degrees, force, tcp_mm)
        await ticks(lab, plugin, 0.1)
    estop(robot, True)
    await ticks(lab, plugin, 0.3)
    if not sag_first:
        for arm in robot.arms.values():
            sag(arm, degrees, force, tcp_mm)
    await ticks(lab, plugin, 2.0)
    estop(robot, False)
    await ticks(lab, plugin, 2.0)
    return before


def picked(plugin: StopRecoveryPlugin, serial: str) -> RobotPose | None:
    """The pose picked for a robot."""
    return plugin._picked(serial)


@pytest.mark.safety
@pytest.mark.parametrize("sag_first", [False, True], ids=["sag after the stop shows", "sag before the stop shows"])
def test_the_pose_before_a_stop_is_picked_and_the_sag_is_never_logged(tmp_path, sag_first):
    """The sag is not a pose, even when the brakes drop before the sync reports the stop; the stop is named."""
    lab, plugin = started(tmp_path)

    async def scenario():
        before = await stop_and_sag(lab, plugin, BELLE, sag_first=sag_first)
        poses = plugin.log.poses[BELLE]
        assert len(poses) == 1 and np.allclose(poses[0].arms["ur_arm"].joints, before["ur_arm"])
        assert picked(plugin, BELLE) == poses[0]
        assert "before stop" in plugin._widgets[BELLE]["pick"].value
        assert plugin.log.stops[BELLE][0].reason == ESTOP

    asyncio.run(scenario())


@pytest.mark.safety
def test_go_back_asks_first_then_moves_slowly_back_and_reports_how_far_the_force_is_off(tmp_path):
    """The first press asks; the second sends one slow move back; arrived, the force is reported against then."""
    lab, plugin = started(tmp_path)

    async def scenario():
        before = await stop_and_sag(lab, plugin, BELLE)
        arm = lab.belle.arms["ur_arm"]
        await press(lab, plugin, BELLE)
        assert "again" in message(plugin, BELLE) and sent(lab, BELLE) == 0
        await press(lab, plugin, BELLE)
        assert len(arm._trajectory.messages) == 1
        assert arm._trajectory.messages[-1].points[-1].time_from_start.sec >= 8  # slow, whatever the size
        load(arm, REST + 0.5)  # the load comes off
        await ticks(lab, plugin, 10.0)
        assert np.allclose(arm.joint_vector(), before["ur_arm"], atol=1e-6)
        assert message(plugin, BELLE) == f"Back at {picked(plugin, BELLE).clock}. Force off by ur_arm 25.0 → 0.5 N"

    asyncio.run(scenario())


@pytest.mark.safety
def test_a_two_arm_robot_moves_both_arms_back_together(tmp_path):
    """Cindy's two arms go back in one multi-arm trajectory."""
    lab, plugin = started(tmp_path)

    async def scenario():
        before = await stop_and_sag(lab, plugin, CINDY)
        await press_twice(lab, plugin, CINDY)
        assert len(lab.cindy._multi_arm.messages) == 1 and sent(lab, CINDY) == 1
        for arm in lab.cindy.arms.values():
            load(arm, REST)
        await ticks(lab, plugin, 10.0)
        for name, arm in lab.cindy.arms.items():
            assert np.allclose(arm.joint_vector(), before[name], atol=1e-6)
        assert message(plugin, CINDY).startswith("Back at")

    asyncio.run(scenario())


@pytest.mark.safety
def test_a_two_arm_robot_is_held_when_one_arm_s_force_goes_off(tmp_path):
    """One arm's force going further off holds both arms."""
    lab, plugin = started(tmp_path)

    async def scenario():
        await stop_and_sag(lab, plugin, CINDY)
        await press_twice(lab, plugin, CINDY)
        await ticks(lab, plugin, 1.0)
        load(lab.cindy.arms["right_ur_arm"], 40.0)
        await ticks(lab, plugin, 0.3)
        assert message(plugin, CINDY).startswith("Held: right_ur_arm")
        assert all(len(arm._trajectory.messages) == 1 for arm in lab.cindy.arms.values())  # each its hold

    asyncio.run(scenario())


@pytest.mark.safety
@pytest.mark.parametrize("degrees, tcp_mm, refusal", [(5.0, 3.0, "° away"), (0.5, 30.0, "mm away")],
                         ids=["joint", "tool0"])
def test_go_back_is_refused_too_far(tmp_path, degrees, tcp_mm, refusal):
    """Nothing is sent beyond the joint or the tool0 limit."""
    lab, plugin = started(tmp_path)

    async def scenario():
        await stop_and_sag(lab, plugin, BELLE, degrees=degrees, tcp_mm=tcp_mm)
        await press_twice(lab, plugin, BELLE)
        assert refusal in message(plugin, BELLE) and "use the arm planner" in message(plugin, BELLE)
        assert sent(lab, BELLE) == 0

    asyncio.run(scenario())


@pytest.mark.safety
@pytest.mark.parametrize("what", ["stopped", "free drive", "no force"])
def test_go_back_is_refused_while_the_arm_cannot_be_moved_or_watched(tmp_path, what):
    """Not operational, another controller, or no force reading: nothing is sent."""
    lab, plugin = started(tmp_path)

    async def scenario():
        await stop_and_sag(lab, plugin, BELLE)
        arm = lab.belle.arms["ur_arm"]
        if what == "stopped":
            estop(lab.belle, True)
        elif what == "free drive":
            arm.state.controllers.active = "free_drive_controller"
        else:
            arm.state.wrench = None
        await ticks(lab, plugin, 0.2)
        await press_twice(lab, plugin, BELLE)
        expected = {"stopped": "not operational", "free drive": "Controller: Joint", "no force": "no force reading"}
        assert expected[what] in message(plugin, BELLE) and sent(lab, BELLE) == 0

    asyncio.run(scenario())


@pytest.mark.safety
def test_a_force_going_further_off_holds_the_robot_even_at_the_same_magnitude(tmp_path):
    """Forces are compared as vectors: pushed the other way at no more |F| than a 5 N rise, the robot is held."""
    lab, plugin = started(tmp_path)

    async def scenario():
        await stop_and_sag(lab, plugin, BELLE)  # 27 N along +y: 25 N off its 2 N then
        arm = lab.belle.arms["ur_arm"]
        await press_twice(lab, plugin, BELLE)
        await ticks(lab, plugin, 1.0)
        load(arm, -32.0)  # |F| 32 N: 5 N over 27, not more; but 34 N off its force then
        await ticks(lab, plugin, 0.3)
        assert message(plugin, BELLE).startswith("Held") and len(arm._trajectory.messages) == 2  # the move, a hold
        assert not plugin._moving_back(BELLE)

    asyncio.run(scenario())


@pytest.mark.safety
def test_after_a_held_go_back_the_pose_before_the_stop_stays_picked(tmp_path):
    """Where a Go back ended is logged, marked, and not picked: the pose it aimed for is."""
    lab, plugin = started(tmp_path)

    async def scenario():
        await stop_and_sag(lab, plugin, BELLE)
        aimed = picked(plugin, BELLE)
        await press_twice(lab, plugin, BELLE)
        await ticks(lab, plugin, 1.0)
        load(lab.belle.arms["ur_arm"], -32.0)
        await ticks(lab, plugin, 3.0)
        newest = plugin.log.poses[BELLE][-1]
        assert newest.go_back and newest != aimed
        assert picked(plugin, BELLE) == aimed

    asyncio.run(scenario())


@pytest.mark.safety
def test_an_arm_that_does_not_arrive_is_held(tmp_path):
    """A move that stops short (a slow pendant slider, a dropped goal) is held and reported, never called back."""
    lab, plugin = started(tmp_path, RobotLab(follow=0.5))

    async def scenario():
        await stop_and_sag(lab, plugin, BELLE)
        await press_twice(lab, plugin, BELLE)
        await ticks(lab, plugin, 25.0)
        assert message(plugin, BELLE).startswith("Not back after") and not plugin._moving_back(BELLE)
        assert len(lab.belle.arms["ur_arm"]._trajectory.messages) == 2  # the move, a hold

    asyncio.run(scenario())


@pytest.mark.safety
def test_stop_holds_the_robot_and_ends_go_back(tmp_path):
    """Stop cancels a running Go back and holds the robot."""
    lab, plugin = started(tmp_path)

    async def scenario():
        await stop_and_sag(lab, plugin, BELLE)
        await press_twice(lab, plugin, BELLE)
        await ticks(lab, plugin, 1.0)
        plugin._stop(lab.ctx, BELLE)
        await ticks(lab, plugin, 0.3)
        assert not plugin._moving_back(BELLE) and len(lab.belle.arms["ur_arm"]._trajectory.messages) >= 2
        assert message(plugin, BELLE) == "Go back stopped (Stop or Stop all): held where it is"

    asyncio.run(scenario())


@pytest.mark.safety
def test_another_pick_between_the_presses_asks_again(tmp_path):
    """The second press counts only for the pose the first asked about."""
    lab, plugin = started(tmp_path)

    async def scenario():
        before = await stop_and_sag(lab, plugin, BELLE)
        older = RobotPose(BELLE, 1.0, "2026-01-01T08:00:00", False,
                          {"ur_arm": ArmPose(tuple(before["ur_arm"] + np.radians(0.2)), None, (0.0, REST, 0.0))})
        plugin.log.poses[BELLE].insert(0, older)
        await ticks(lab, plugin, 0.2)
        await press(lab, plugin, BELLE)
        widgets = plugin._widgets[BELLE]
        widgets["pick"].value = next(label for label in widgets["options"] if label.startswith("08:00:00"))
        await press(lab, plugin, BELLE)
        assert "again" in message(plugin, BELLE) and sent(lab, BELLE) == 0

    asyncio.run(scenario())


@pytest.mark.safety
def test_a_real_robot_is_never_offered_a_simulated_pose(tmp_path):
    """Sim and real poses share the day file, but a real robot is offered only real ones."""
    lab = RobotLab()
    lab.belle.arms["ur_arm"].state.operational = False  # nothing real logged at start
    lab, plugin = started(tmp_path, lab)
    plugin.log.add_pose(RobotPose(BELLE, 1.0, "2026-11-10T08:00:00", True,
                                  {"ur_arm": ArmPose(tuple(lab.belle.arms["ur_arm"].joint_vector()))}))
    plugin.draw(lab.ctx)
    assert plugin._picked(BELLE) is None


@pytest.mark.safety
def test_a_go_back_that_never_moves_marks_no_later_pose(tmp_path):
    """A goal the controller drops leaves no mark: the next pose, after a jog, is still picked after the next stop."""
    lab, plugin = started(tmp_path, RobotLab(follow=0.0))

    async def scenario():
        await stop_and_sag(lab, plugin, BELLE)
        await press_twice(lab, plugin, BELLE)
        await ticks(lab, plugin, 25.0)
        assert message(plugin, BELLE).startswith("Not back after") and BELLE not in plugin._ended_go_back
        arm = lab.belle.arms["ur_arm"]
        sag(arm, -0.3, 27.0, tcp_mm=0.0)  # a jog by hand, no Go back
        await ticks(lab, plugin, 2.0)
        jogged = plugin.log.poses[BELLE][-1]
        assert not jogged.go_back
        await stop_and_sag(lab, plugin, BELLE, degrees=0.2)
        assert picked(plugin, BELLE) == jogged

    asyncio.run(scenario())


@pytest.mark.safety
def test_a_robot_stopped_at_start_logs_no_pose_until_it_moves(tmp_path):
    """A monitor started after a stop must not take the sagged pose for the one to go back to."""
    lab = RobotLab()
    estop(lab.belle, True)
    lab, plugin = started(tmp_path, lab)

    async def scenario():
        estop(lab.belle, False)
        await ticks(lab, plugin, 2.0)
        assert BELLE not in plugin.log.poses
        sag(lab.belle.arms["ur_arm"], 0.5, REST)  # now it moves while running
        await ticks(lab, plugin, 2.0)
        assert len(plugin.log.poses[BELLE]) == 1

    asyncio.run(scenario())


def test_poses_and_stops_are_read_back_from_the_day_file(tmp_path):
    """What was logged comes back from the day file, forces as vectors."""
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
    assert again.poses[BELLE][0].arms["ur_arm"].force == (0.0, REST, 0.0)
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
