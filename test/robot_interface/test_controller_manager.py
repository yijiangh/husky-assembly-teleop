"""Switching an arm's or base's controllers: one at a time, read back from the controller manager, never assumed."""

from __future__ import annotations

from types import SimpleNamespace

import pytest
from controller_manager_msgs.srv import SwitchController
from robot_fakes import Client, Node

from husky_assembly_teleop.robot_interface.controller_manager import (ControllerManagerInterface,
                                                                      ControllerManagerState)

pytestmark = pytest.mark.safety

JOINTS, CARTESIAN, FREE = "joints", "cartesian", "free_drive"


def manager(active: str | None = None, others: dict[str, str] | None = None, ready: bool = True):
    """A manager of JOINTS, CARTESIAN and FREE with `active` running and `others` in their reported states."""
    interface = ControllerManagerInterface.__new__(ControllerManagerInterface)
    interface._node, interface.namespace, interface.switchable = Node(), "/a200_0804/ur5e", (JOINTS, CARTESIAN, FREE)
    interface.state = ControllerManagerState(active=active, last_update_time=100.0)
    interface.state.controllers = {name: "inactive" for name in interface.switchable} | (others or {})
    if active is not None:
        interface.state.controllers[active] = "active"
    interface._reported_unavailable, interface._guard = False, None
    interface._list_client, interface._switch_client = Client(ready), Client(ready)
    interface._configure_client = Client(ready)
    return interface


def listing(**states: str):
    """A list_controllers answer with these controller states."""
    return SimpleNamespace(controller=[SimpleNamespace(name=name, state=state) for name, state in states.items()])


def test_switch_stops_every_other_running_controller_strictly():
    """Activate the one asked for; deactivate each switchable one reported active, even one `active` missed."""
    interface = manager(JOINTS, others={FREE: "active"})
    assert interface.switch(CARTESIAN)
    request = interface._switch_client.calls[0].request
    assert list(request.activate_controllers) == [CARTESIAN]
    assert set(request.deactivate_controllers) == {JOINTS, FREE}
    assert request.strictness == SwitchController.Request.STRICT
    assert interface.state.switch_in_flight == CARTESIAN and interface.state.requested == CARTESIAN


def test_an_accepted_switch_is_read_back():
    """The answer sets `active`, then the list is asked for again, which has the last word."""
    interface = manager(JOINTS)
    interface.switch(CARTESIAN)
    interface._switch_client.calls[0].answer(SimpleNamespace(ok=True))
    assert interface.state.active == CARTESIAN and interface.state.switch_in_flight is None
    interface._list_client.calls[0].answer(listing(joints="inactive", cartesian="inactive", free_drive="inactive"))
    assert interface.state.active is None and interface.state.requested is None


@pytest.mark.parametrize("finish", [lambda future: future.answer(SimpleNamespace(ok=False)),
                                    lambda future: future.fail(RuntimeError("timeout"))])
def test_a_rejected_switch_keeps_the_running_controller(finish):
    """Rejected or raising: `active` stays, `switch_error` says why, and the state is read back."""
    interface = manager(JOINTS)
    interface.switch(CARTESIAN)
    finish(interface._switch_client.calls[0])
    assert interface.state.active == JOINTS and "failed" in interface.state.switch_error
    assert interface.state.switch_in_flight is None and len(interface._list_client.calls) == 1


@pytest.mark.parametrize("controller, active, ready, in_flight, why", [
    ("unknown", None, True, None, "is not one of"),
    (CARTESIAN, None, False, None, "is not available"),
    (CARTESIAN, None, True, FREE, "still switching"),
])
def test_switch_refused(controller, active, ready, in_flight, why):
    """Unknown controller, no service, or a switch still pending: nothing is sent, and the error says why."""
    interface = manager(active, ready=ready)
    interface.state.switch_in_flight = in_flight
    assert not interface.switch(controller)
    assert not interface._switch_client.calls and why in interface.state.switch_error


def test_a_guard_refuses_only_what_it_names():
    """The arm's guard keeps free drive off while the safety sync does not vouch for the arm; others still switch."""
    interface = manager(JOINTS)
    interface._guard = lambda controller: "no recent sync status" if controller == FREE else None
    assert not interface.switch(FREE) and not interface._switch_client.calls
    assert "no recent sync status" in interface.state.switch_error
    assert interface.switch(CARTESIAN)


def test_switching_to_the_running_controller_sends_nothing():
    """Already running: True, and it counts as ours."""
    interface = manager(CARTESIAN)
    assert interface.switch(CARTESIAN)
    assert not interface._switch_client.calls and interface.state.requested == CARTESIAN


def test_deactivate_all_goes_out_even_while_a_switch_is_pending():
    """The soft stop deactivates every switchable controller, best effort, and nothing is ours any more."""
    interface = manager(JOINTS)
    interface.switch(CARTESIAN)
    assert interface.deactivate_all()
    request = interface._switch_client.calls[-1].request
    assert set(request.deactivate_controllers) == {JOINTS, CARTESIAN, FREE} and not request.activate_controllers
    assert request.strictness == SwitchController.Request.BEST_EFFORT and interface.state.requested is None


@pytest.mark.parametrize("ready, answer, why", [
    (False, None, "cannot stop"),
    (True, SimpleNamespace(ok=False), "STOP FAILED"),
])
def test_a_failed_stop_is_loud(ready, answer, why):
    """No service, or a rejected stop: `switch_error` and the log say the controllers still run."""
    interface = manager(JOINTS, ready=ready)
    interface.deactivate_all()
    if answer is not None:
        interface._switch_client.calls[0].answer(answer)
    assert why in interface.state.switch_error and why in interface._node.log.text("error")


def test_a_switch_from_outside_is_no_longer_ours():
    """When the list shows another controller than the one we switched to, `requested` is cleared."""
    interface = manager(CARTESIAN)
    interface.state.requested = CARTESIAN
    interface.refresh()
    interface._list_client.calls[0].answer(listing(joints="active", cartesian="inactive"))
    assert interface.state.active == JOINTS and interface.state.requested is None
    assert interface.state.last_update_time == interface._node.now


def test_a_missing_controller_manager_is_reported_once():
    """The first refresh with the service down warns, later ones do not; one says when it is back."""
    interface = manager(ready=False)
    assert not interface.refresh() and not interface.refresh()
    assert interface._node.log.text("warning").count("not available") == 1
    interface._list_client.ready = True
    assert interface.refresh() and "is available" in interface._node.log.text("info")


@pytest.mark.parametrize("finish, afresh", [(lambda future: future.answer(SimpleNamespace(ok=True)), True),
                                            (lambda future: future.answer(SimpleNamespace(ok=False)), False),
                                            (lambda future: future.fail(RuntimeError("timeout")), False)])
def test_restart_configures_the_controller_afresh_then_switches_to_it(finish, afresh):
    """Configure first, which clears what the controller kept from its stop; then switch either way, warning if not."""
    interface = manager()
    assert interface.restart(JOINTS)
    configure, = interface._configure_client.calls
    assert configure.request.name == JOINTS and not interface._switch_client.calls
    finish(configure)
    switch, = interface._switch_client.calls
    assert list(switch.request.activate_controllers) == [JOINTS]
    assert ("not configured afresh" in interface._node.log.text("warning")) != afresh


def test_restart_switches_at_once_without_a_configure_service():
    """No configure service (an older manager, the simulator): the controller is switched to as it is."""
    interface = manager()
    interface._configure_client = Client(False)
    assert interface.restart(JOINTS) and len(interface._switch_client.calls) == 1
