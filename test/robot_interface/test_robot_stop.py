"""A robot's soft stop, resume and unlock through multi_arm_safety_sync, without ROS."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from robot_fakes import Client, Node

from husky_assembly_teleop.robot_interface.base import PLATFORM_VELOCITY_CONTROLLER
from husky_assembly_teleop.robot_interface.robot import HuskyRobotInterface
from husky_assembly_teleop.robot_interface.soft_stop import RobotStopped, SoftStopLatch

pytestmark = pytest.mark.safety


def robot(arms: int = 1, ready: bool = True):
    """A robot with `arms` arms whose sync services are up if `ready`; its base records stops."""
    interface = HuskyRobotInterface.__new__(HuskyRobotInterface)
    interface._node, interface.config = Node(), SimpleNamespace(serial="0804")
    interface.soft_stop_latch = SoftStopLatch("0804")
    interface.base = SimpleNamespace(stops=0, switched=[])
    interface.base.stop = lambda: setattr(interface.base, "stops", interface.base.stops + 1)
    interface.base.controllers = SimpleNamespace(switch=lambda name: interface.base.switched.append(name) or ready)
    interface.arms = {f"arm{index}": SimpleNamespace(config=SimpleNamespace(name=f"arm{index}"),
                                                     end_effector=SimpleNamespace(stops=0))
                      for index in range(arms)}
    for arm in interface.arms.values():
        arm.end_effector.stop = lambda tool=arm.end_effector: setattr(tool, "stops", tool.stops + 1)
    interface._unlock, interface._stop_arms, interface._resume_arms = Client(ready), Client(ready), Client(ready)
    return interface


def test_soft_stop_stops_the_base_and_asks_the_sync_to_stop_the_arms():
    """One base stop, one stop call to the sync."""
    interface = robot()
    interface.soft_stop()
    assert interface.base.stops == 1 and len(interface._stop_arms.calls) == 1
    assert "SOFT STOP" in interface._node.log.text("warning")


def test_soft_stop_stops_every_tool_even_with_the_sync_down():
    """A running screw motor or gripper move stops with the robot; one tool raising does not keep the next going."""
    interface = robot(arms=2, ready=False)

    def broken():
        raise RuntimeError("driver gone")

    interface.arms["arm0"].end_effector.stop = broken
    interface.soft_stop()
    assert interface.arms["arm1"].end_effector.stops == 1 and "driver gone" in interface._node.log.text("error")


def test_soft_stop_without_arms_only_stops_the_base():
    """A robot without arms has no sync to call."""
    interface = robot(arms=0)
    interface.soft_stop()
    assert interface.base.stops == 1 and not interface._stop_arms.calls


def test_a_base_that_raises_does_not_keep_the_arms_from_stopping():
    """A base whose stop raises (e.g. its publisher destroyed by a reconnect) is logged; the arms still stop."""
    interface = robot()

    def broken():
        raise RuntimeError("publisher destroyed")

    interface.base.stop = broken
    interface.soft_stop()
    assert len(interface._stop_arms.calls) == 1 and "publisher destroyed" in interface._node.log.text("error")
    assert interface.soft_stopped


def test_soft_stop_with_the_sync_down_still_stops_the_base_and_says_so():
    """The base stops even when the arms cannot be reached; the log says the arms were not stopped."""
    interface = robot(ready=False)
    interface.soft_stop()
    assert interface.base.stops == 1 and not interface._stop_arms.calls
    assert "cannot stop arms" in interface._node.log.text("error")


def test_soft_stop_latches_the_robot_until_resume():
    """From the soft stop the robot refuses motion; resume accepts commands again before it switches the base on."""
    interface = robot(arms=2)
    assert not interface.soft_stopped
    interface.soft_stop()
    assert interface.soft_stopped
    with pytest.raises(RobotStopped, match="robot 0804 is soft stopped"):
        interface.send_multi_arm_trajectory({}, 1.0)
    seen = []
    interface.base.controllers.switch = lambda _name: seen.append(interface.soft_stopped) or True
    assert interface.resume() and seen == [False] and not interface.soft_stopped


def test_resume_switches_the_base_controller_on_and_asks_the_sync_to_resume_the_arms():
    """Both parts of a soft stop are undone; a robot without arms only gets its base back."""
    interface = robot()
    assert interface.resume()
    assert interface.base.switched == [PLATFORM_VELOCITY_CONTROLLER] and len(interface._resume_arms.calls) == 1
    bare = robot(arms=0)
    assert bare.resume() and bare.base.switched == [PLATFORM_VELOCITY_CONTROLLER] and not bare._resume_arms.calls


def test_resume_with_the_sync_down_still_resumes_the_base_and_says_so():
    """The base comes back even when the arms cannot; the result says not everything went out."""
    interface = robot(ready=False)
    assert not interface.resume()
    assert interface.base.switched == [PLATFORM_VELOCITY_CONTROLLER]


@pytest.mark.parametrize("command, client", [("resume", "_resume_arms"),
                                             ("unlock_protective_stop", "_unlock")])
def test_sync_answers_are_logged(command, client):
    """A refusal is an error in the log; an acceptance is information."""
    interface = robot()
    assert getattr(interface, command)()
    future, = getattr(interface, client).calls
    future.answer(SimpleNamespace(success=False, message="arm not in remote control"))
    assert "arm not in remote control" in interface._node.log.text("error")
    getattr(interface, command)()
    getattr(interface, client).calls[-1].answer(SimpleNamespace(success=True, message="done"))
    assert "done" in interface._node.log.text("info")
