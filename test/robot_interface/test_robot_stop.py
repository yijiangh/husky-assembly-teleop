"""A robot's soft stop, resume and unlock through multi_arm_safety_sync, without ROS."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from robot_fakes import Client, Node

from husky_assembly_teleop.robot_interface.robot import HuskyRobotInterface

pytestmark = pytest.mark.safety


def robot(arms: int = 1, ready: bool = True):
    """A robot with `arms` arms whose sync services are up if `ready`; its base records stops."""
    interface = HuskyRobotInterface.__new__(HuskyRobotInterface)
    interface._node, interface.config = Node(), SimpleNamespace(serial="0804")
    interface.base = SimpleNamespace(stops=0)
    interface.base.stop = lambda: setattr(interface.base, "stops", interface.base.stops + 1)
    interface.arms = {f"arm{index}": object() for index in range(arms)}
    interface._unlock, interface._stop_arms, interface._resume_arms = Client(ready), Client(ready), Client(ready)
    return interface


def test_soft_stop_stops_the_base_and_asks_the_sync_to_stop_the_arms():
    """One base stop, one stop call to the sync."""
    interface = robot()
    interface.soft_stop()
    assert interface.base.stops == 1 and len(interface._stop_arms.calls) == 1
    assert "SOFT STOP" in interface._node.log.text("warning")


def test_soft_stop_without_arms_only_stops_the_base():
    """A robot without arms has no sync to call."""
    interface = robot(arms=0)
    interface.soft_stop()
    assert interface.base.stops == 1 and not interface._stop_arms.calls


def test_soft_stop_with_the_sync_down_still_stops_the_base_and_says_so():
    """The base stops even when the arms cannot be reached; the log says the arms were not stopped."""
    interface = robot(ready=False)
    interface.soft_stop()
    assert interface.base.stops == 1 and not interface._stop_arms.calls
    assert "cannot stop arms" in interface._node.log.text("error")


@pytest.mark.parametrize("command, client", [("resume_arms", "_resume_arms"),
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
