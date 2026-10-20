"""The mounted tools' commands: Robotiq gripper goals, scaffolding v3 screw motors, scaffolding v1 tool output."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from action_msgs.msg import GoalStatus
from robot_fakes import Client, Future, Node, Publisher
from ur_msgs.srv import SetIO

from husky_assembly_teleop.robot_interface.end_effectors import (RobotiqGripper, RobotiqState, ScaffoldingV1,
                                                                 ScaffoldingV1State, ScaffoldingV3, ScaffoldingV3State)
from husky_assembly_teleop.robot_interface.soft_stop import RobotStopped, SoftStopLatch

pytestmark = pytest.mark.safety


class ActionClient:
    """A gripper action client that keeps every goal; `ready` says whether the server is up."""

    def __init__(self, ready: bool = True):
        """No goals yet."""
        self.ready, self.goals = ready, []

    def server_is_ready(self) -> bool:
        """Whether the server is up."""
        return self.ready

    def send_goal_async(self, goal) -> Future:
        """Keep the goal and return the future of its acceptance."""
        self.goals.append(Future(goal))
        return self.goals[-1]


def accept(goal: Future) -> Future:
    """Accept `goal` and return the future of its result; the result future's `cancels` counts cancel requests."""
    result = Future(None)
    result.cancels = 0

    def cancel() -> Future:
        result.cancels += 1
        return Future(None)

    goal.answer(SimpleNamespace(accepted=True, get_result_async=lambda: result, cancel_goal_async=cancel))
    return result


def finish(result: Future, reached: bool = False, stalled: bool = False) -> None:
    """End a goal at 0.4 rad."""
    result.answer(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED,
                                  result=SimpleNamespace(reached_goal=reached, stalled=stalled, position=0.4)))


def cancelled(result: Future) -> None:
    """End a goal as cancelled, with the empty result the controller sends."""
    result.answer(SimpleNamespace(status=GoalStatus.STATUS_CANCELED,
                                  result=SimpleNamespace(reached_goal=False, stalled=False, position=0.0)))


def robotiq(ready: bool = True) -> RobotiqGripper:
    """A Robotiq gripper whose action server is up if `ready`."""
    gripper = RobotiqGripper.__new__(RobotiqGripper)
    gripper._node, gripper.namespace, gripper.state, gripper._action = Node(), "/a200/gripper", RobotiqState(), \
        ActionClient(ready)
    gripper._latch = SoftStopLatch("0804")
    gripper._goal, gripper._cancel_on_accept = None, False
    return gripper


def v3() -> ScaffoldingV3:
    """A scaffolding v3 tool with both motors idle."""
    tool = ScaffoldingV3.__new__(ScaffoldingV3)
    tool._node, tool.namespace, tool.state, tool._command = Node(), "/a200/tool", ScaffoldingV3State(), Publisher()
    tool._latch = SoftStopLatch("0804")
    tool.state.gripper_motor = tool.state.joint_motor = ScaffoldingV3.IDLE
    return tool


def v1(ready: bool = True) -> ScaffoldingV1:
    """A scaffolding v1 tool whose arm's set_io service is up if `ready`."""
    tool = ScaffoldingV1.__new__(ScaffoldingV1)
    tool._node, tool.namespace, tool.arm_namespace = Node(), "/a200/tool", "/a200/ur5e"
    tool.state, tool._set_io, tool._latch = ScaffoldingV1State(), Client(ready), SoftStopLatch("0804")
    return tool


# --- --- --- --- --- Robotiq --- --- --- --- ---

@pytest.mark.parametrize("position, effort, sent_position, sent_effort", [
    (0.4, 0.5, 0.4, 0.5),
    (-1.0, -5.0, RobotiqGripper.OPEN_POSITION, 0.0),
    (9.0, 1e6, RobotiqGripper.CLOSED_POSITION, RobotiqGripper.MAX_EFFORT),
])
def test_robotiq_goals_are_clipped_to_the_gripper(position, effort, sent_position, sent_effort):
    """Position and effort are clipped to the gripper's range; the command is remembered and it counts as moving."""
    gripper = robotiq()
    gripper.move(position, effort)
    goal, = gripper._action.goals
    assert goal.request.command.position == pytest.approx(sent_position)
    assert goal.request.command.max_effort == pytest.approx(sent_effort)
    assert gripper.state.moving and gripper.state.commanded_position == pytest.approx(sent_position)


def test_robotiq_open_and_close_go_to_the_ends():
    """Open and close send one goal each, to the ends."""
    gripper = robotiq()
    gripper.open()
    gripper.close()
    assert [goal.request.command.position for goal in gripper._action.goals] == pytest.approx(
        [RobotiqGripper.OPEN_POSITION, RobotiqGripper.CLOSED_POSITION])


def test_robotiq_stop_cancels_the_running_goal():
    """Stop cancels the goal instead of sending a new one; its cancelled result ends the move without a failure."""
    gripper = robotiq()
    gripper.close()
    result = accept(gripper._action.goals[0])
    gripper.stop()
    assert result.cancels == 1 and len(gripper._action.goals) == 1
    cancelled(result)
    assert not gripper.state.moving and gripper.state.last_result_ok is None
    assert "gripper goal cancelled" in gripper._node.log.text("info")


def test_robotiq_stop_before_acceptance_cancels_once_accepted():
    """Stop while the goal is still on its way: it is cancelled as soon as the server accepts it."""
    gripper = robotiq()
    gripper.close()
    gripper.stop()
    assert accept(gripper._action.goals[0]).cancels == 1


def test_robotiq_stop_when_idle_sends_nothing():
    """No goal running: nothing to cancel, no goal sent."""
    gripper = robotiq()
    gripper.close()
    finish(accept(gripper._action.goals[0]), reached=True)
    gripper.stop()
    assert len(gripper._action.goals) == 1


def test_robotiq_replaced_goal_is_not_a_failure():
    """A goal replaced by a newer one ends cancelled: logged as replaced, health untouched, newer goal still moving."""
    gripper = robotiq()
    gripper.close()
    close_result = accept(gripper._action.goals[0])
    gripper.open()
    accept(gripper._action.goals[1])
    cancelled(close_result)
    assert gripper.state.moving and gripper.state.last_result_ok is None
    assert "gripper goal replaced" in gripper._node.log.text("info")


def test_robotiq_without_its_server_sends_nothing():
    """The action server down: a warning, no goal, not moving."""
    gripper = robotiq(ready=False)
    gripper.close()
    assert not gripper._action.goals and not gripper.state.moving
    assert "not available" in gripper._node.log.text("warning")


@pytest.mark.parametrize("reached, stalled, ok", [(True, False, True), (False, True, True), (False, False, False)])
def test_robotiq_result_says_whether_it_got_there(reached, stalled, ok):
    """Reached or stalled on a part is a success; falling short is not. Either way it stops moving."""
    gripper = robotiq()
    gripper.close()
    finish(accept(gripper._action.goals[0]), reached, stalled)
    assert not gripper.state.moving and gripper.state.last_result_ok is ok


@pytest.mark.parametrize("answer", [lambda goal: goal.answer(SimpleNamespace(accepted=False)),
                                    lambda goal: goal.fail(RuntimeError("server gone"))])
def test_robotiq_rejected_goal_stops_moving(answer):
    """A rejected or failed goal never leaves `moving` set."""
    gripper = robotiq()
    gripper.close()
    answer(gripper._action.goals[0])
    assert not gripper.state.moving and gripper.state.last_result_ok is False


def test_robotiq_preempted_goal_does_not_end_the_newer_one():
    """Close, then open before the close finished: the close's result must not say the gripper stopped."""
    gripper = robotiq()
    gripper.close()
    close_result = accept(gripper._action.goals[0])
    gripper.open()
    accept(gripper._action.goals[1])
    finish(close_result)
    assert gripper.state.moving


# --- --- --- --- --- scaffolding v3 --- --- --- --- ---

def test_v3_commands_drive_the_right_motor_the_right_way():
    """Gripper: close tightens, open loosens M1. Joint screw: M2 either way. Stop: motor 0, direction 0."""
    tool = v3()
    tool.close()
    tool.open()
    tool.drive_screw(1)
    tool.drive_screw(-1)
    tool.drive_screw(0)
    sent = [(message.motor, message.direction) for message in tool._command.sent]
    assert sent == [(1, 1), (1, -1), (2, 1), (2, -1), (0, 0)]


@pytest.mark.parametrize("stalled", ["gripper_motor", "joint_motor"])
def test_v3_stalled_motor_refuses_until_stopped(stalled):
    """A stalled motor gets nothing, either way; Stop still goes out."""
    tool = v3()
    setattr(tool.state, stalled, ScaffoldingV3.STALLED)
    run = tool.close if stalled == "gripper_motor" else (lambda: tool.drive_screw(-1))
    run()
    assert not tool._command.sent and "stalled" in tool._node.log.text("warning")
    tool.stop()
    assert [(message.motor, message.direction) for message in tool._command.sent] == [(0, 0)]


# --- --- --- --- --- scaffolding v1 --- --- --- --- ---

def test_v1_switches_the_gripper_output():
    """Close sets tool output 1 to GRIPPER_CLOSED_WHEN_ON, open to the other."""
    tool = v1()
    tool.close()
    tool.open()
    closed, opened = (call.request for call in tool._set_io.calls)
    assert closed.pin == opened.pin == SetIO.Request.PIN_TOOL_DOUT1
    on, off = float(SetIO.Request.STATE_ON), float(SetIO.Request.STATE_OFF)
    assert (closed.state, opened.state) == ((on, off) if ScaffoldingV1.GRIPPER_CLOSED_WHEN_ON else (off, on))


@pytest.mark.parametrize("ready, answer", [(False, None), (True, SimpleNamespace(success=False))])
def test_v1_failed_switch_is_reported(ready, answer):
    """No set_io service, or a refusal: a warning, and the request counts as failed."""
    tool = v1(ready)
    tool.close()
    if answer is not None:
        tool._set_io.calls[0].answer(answer)
        assert tool.state.last_request_ok is False
    assert tool._node.log.text("warning")


# --- --- --- --- --- soft stop --- --- --- --- ---

def test_a_soft_stopped_tool_refuses_every_command_but_stop():
    """Robotiq, v3 and v1: every command that moves the tool raises RobotStopped, sending nothing; stop goes out."""
    gripper, screw, switched = robotiq(), v3(), v1()
    gripper.close()
    closing = accept(gripper._action.goals[0])
    gripper._latch.stopped = screw._latch.stopped = switched._latch.stopped = True
    for command in (gripper.open, gripper.close, lambda: gripper.move(0.4), gripper.reactivate,
                    screw.open, screw.close, lambda: screw.drive_screw(1), switched.open, switched.close):
        with pytest.raises(RobotStopped, match="soft stopped"):
            command()
    assert len(gripper._action.goals) == 1 and not screw._command.sent and not switched._set_io.calls
    gripper.stop()
    screw.drive_screw(0)
    assert closing.cancels == 1
    assert [(message.motor, message.direction) for message in screw._command.sent] == [(0, 0)]
