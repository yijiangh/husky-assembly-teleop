"""What a run records by itself: a stop, its end, a gripper's failure, each once."""

from __future__ import annotations

from types import SimpleNamespace

from plugin_fakes import fake_node, fake_robotiq
from ur_dashboard_msgs.msg import SafetyMode

from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.end_effectors import NO_RESULT
from husky_assembly_teleop.robot_interface.refusal import Refusal
from husky_assembly_teleop.shared.runs.faults import BASE_ESTOP, GRIPPER_FAILED, SAFETY_MODE, FaultWatch

BELLE = "a200-0805"


def belle(mode=SafetyMode.NORMAL, tool=None) -> SimpleNamespace:
    """Belle as the watch reads her."""
    arm = SimpleNamespace(state=SimpleNamespace(safety_mode=mode), end_effector=tool)
    return SimpleNamespace(arms={"ur_arm": arm}, base=SimpleNamespace(state=BaseState(estopped=False)))


def robotiq():
    """A real gripper without ROS."""
    return fake_robotiq(fake_node(lambda: 0.0, []), position=0.5)


def test_every_estop_and_its_end_is_an_event_of_its_own():
    """Two e-stops seconds apart stay two, with the return to normal between them: Belle at 17:04 and 17:05."""
    watch, robot = FaultWatch(), belle()
    robots = {BELLE: robot}
    assert watch.changes(robots) == []
    modes = []
    for mode in (SafetyMode.ROBOT_EMERGENCY_STOP, SafetyMode.NORMAL, SafetyMode.ROBOT_EMERGENCY_STOP):
        robot.arms["ur_arm"].state.safety_mode = mode
        (event, data), = watch.changes(robots)
        assert event == SAFETY_MODE and (data["serial"], data["arm"]) == (BELLE, "ur_arm")
        modes.append((data["was"], data["mode"]))
        assert watch.changes(robots) == []
    assert modes == [("normal", "robot emergency stop"), ("robot emergency stop", "normal"),
                     ("normal", "robot emergency stop")]


def test_a_stop_already_there_at_the_first_look_is_an_event():
    """A monitor started on an e-stopped robot says so."""
    (event, data), = FaultWatch().changes({BELLE: belle(SafetyMode.ROBOT_EMERGENCY_STOP)})
    assert event == SAFETY_MODE and data["was"] is None and data["mode"] == "robot emergency stop"


def test_a_tools_failure_from_before_the_first_look_is_no_event():
    """An old refusal or failed command is not this run's; a joint screw's stall never is the gripper's failure."""
    gripper = robotiq()
    gripper.state.last_result_ok, gripper.state.failure = False, NO_RESULT
    gripper.state.reactivate_error = "timed out"
    gripper.last_refusal = Refusal("gripper refused", "not available", 1.0)
    watch, robots = FaultWatch(), {BELLE: belle(tool=gripper)}
    assert watch.changes(robots) == []
    gripper.last_refusal = Refusal("joint screw stalled", "stop it first", 2.0)
    assert watch.changes(robots) == []


def test_an_unknown_mode_says_nothing_and_changes_nothing():
    """The status gone for a moment is no change: the stop's event does not repeat when it comes back."""
    watch, robot = FaultWatch(), belle(SafetyMode.ROBOT_EMERGENCY_STOP)
    robots = {BELLE: robot}
    assert len(watch.changes(robots)) == 1
    robot.arms["ur_arm"].state.safety_mode = None
    assert watch.changes(robots) == []
    robot.arms["ur_arm"].state.safety_mode = SafetyMode.ROBOT_EMERGENCY_STOP
    assert watch.changes(robots) == []


def test_the_platforms_estop_engaged_and_released():
    """Each change once."""
    watch, robot = FaultWatch(), belle()
    robots = {BELLE: robot}
    watch.changes(robots)
    robot.base.state.estopped = True
    assert watch.changes(robots) == [(BASE_ESTOP, {"serial": BELLE, "engaged": True})]
    assert watch.changes(robots) == []
    robot.base.state.estopped = False
    assert watch.changes(robots) == [(BASE_ESTOP, {"serial": BELLE, "engaged": False})]


def test_a_robotiq_command_that_failed_is_an_event_with_where_the_fingers_are():
    """Once per failed command: the same failure again needs a new command between."""
    gripper, watch = robotiq(), FaultWatch()
    robots = {BELLE: belle(tool=gripper)}
    state = gripper.state
    state.commanded_position, state.commanded_effort = 0.8, 0.1
    assert watch.changes(robots) == []
    state.last_result_ok, state.failure = False, NO_RESULT
    (event, data), = watch.changes(robots)
    assert event == GRIPPER_FAILED and data["failure"] == "no result"
    assert (data["serial"], data["commanded"], data["effort"], data["position"]) == (BELLE, 0.8, 0.1, 0.5)
    assert watch.changes(robots) == []
    state.last_result_ok, state.failure = None, ""
    assert watch.changes(robots) == []
    state.last_result_ok, state.failure = False, NO_RESULT
    assert len(watch.changes(robots)) == 1


def test_a_tool_command_refused_and_a_reactivation_that_failed_are_events():
    """A refusal before anything was sent, and Reactivate's own error."""
    gripper, watch = robotiq(), FaultWatch()
    robots = {BELLE: belle(tool=gripper)}
    watch.changes(robots)
    gripper.last_refusal = Refusal("gripper refused", "action server is not available", 1.0)
    (event, data), = watch.changes(robots)
    assert event == GRIPPER_FAILED and (data["failure"], data["reason"]) == ("gripper refused",
                                                                             "action server is not available")
    gripper.state.reactivate_error = "reactivate_gripper timed out"
    (event, data), = watch.changes(robots)
    assert data["failure"] == "reactivate failed" and data["reason"] == "reactivate_gripper timed out"
    assert watch.changes(robots) == []
