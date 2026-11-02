"""robot_control's readouts keep their height: chips on one fixed line, numbers on a fixed count of rows apart."""

from types import SimpleNamespace

import numpy as np
from robot_fakes import fake_arm

from husky_assembly_teleop.plugins.robot_control.arm_panel import pose_plan_line
from husky_assembly_teleop.plugins.robot_control.status import (arm_status, base_status, phone_arm_status,
                                                                refusal_chip, tool_status)
from husky_assembly_teleop.robot_interface.base import BaseState
from husky_assembly_teleop.robot_interface.end_effectors import NO_RESULT, RobotiqState
from husky_assembly_teleop.robot_interface.refusal import Refusal
from husky_assembly_teleop.ui.style import FAIL, OK, chip_line


class _Slider:
    """Stands in for a pose slider: just its value."""

    def __init__(self, value: float):
        """Hold `value`."""
        self.value = value


def test_arm_chips_sit_on_the_chip_line_also_with_a_refusal():
    """The arm's chips, a refusal included, go on the chip line, so the numbers below keep their place."""
    arm = fake_arm()
    refused = refusal_chip([Refusal("jog refused", "test", 100.0)], 100.0)
    for extra in ("", refused):
        chips, numbers = arm_status(arm, 100.0, extra)
        assert chips.count('class="chip-line"') == 1 and "q   " in numbers
        assert phone_arm_status(arm, 100.0, extra).startswith(chip_line("")[:40])


def test_chips_keep_their_html_while_the_numbers_change():
    """The joints, the TCP and the gripper move: only the numbers' html changes, so a hovered chip keeps its text."""
    arm = fake_arm()
    before = arm_status(arm, 100.0)
    arm.state.tcp_position = arm.state.tcp_position + 0.01
    after = arm_status(arm, 100.0)
    assert before[0] == after[0] and before[1] != after[1]
    gripper = RobotiqState(position=0.1, last_update_time=100.0)
    chips = tool_status(gripper, 100.0)[0]
    gripper.position = 0.2
    assert tool_status(gripper, 100.0)[0] == chips


def test_a_base_shows_the_mocap_chip_and_numbers_apart():
    """The base: its controller and mocap chips on the chip line, pose, mocap error and wifi in the html below.

    Pose and marker error change every fix: only the numbers' html changes, so a hovered chip keeps its text.
    """
    state = BaseState(position=np.array([1.0, 2.0, 0.0]), orientation=np.array([0.0, 0.0, 0.0, 1.0]), tracked=True,
                      tracking_valid=True, marker_error=0.0004, last_update_time=99.99, last_fix_time=99.99)
    base = SimpleNamespace(state=state, mocap_id=1850,
                           controllers=fake_arm(controller="platform_velocity_controller").controllers)
    chips, numbers = base_status(base, 100.0)
    assert ">mocap<" in chips and "xyz" not in chips and "xyz" in numbers and "wifi" in numbers
    state.position, state.marker_error = np.array([1.02, 2.01, 0.0]), 0.0006
    state.last_update_time = state.last_fix_time = 100.02
    after_chips, after_numbers = base_status(base, 100.03)
    assert after_chips == chips and after_numbers != numbers


def test_a_second_run_tap_shows_next_to_the_running_controller():
    """A switch refused (another one pending): the running controller stays green, a refusal chip beside it."""
    arm = fake_arm()
    arm.controllers.last_refusal = Refusal("switch refused", "still switching to 'cartesian'", 100.0)
    chips, _numbers = arm_status(arm, 101.0)
    assert f"background:{OK}" in chips.split("switch refused")[0] and ">switch refused<" in chips
    assert ">switch refused<" not in arm_status(arm, 100.0 + 5.0)[0], "gone after REFUSAL_SHOWN"


def test_a_refused_switch_goes_last_so_no_chip_moves():
    """The switch refusal comes after every other chip and the caller's refusals: the chips before keep their place."""
    arm = fake_arm()
    plain = arm_status(arm, 101.0)[0]
    arm.controllers.last_refusal = Refusal("switch refused", "still switching to 'cartesian'", 100.0)
    refused = refusal_chip([Refusal("jog refused", "test", 100.0)], 101.0)
    chips = arm_status(arm, 101.0, refused)[0]
    line = '<div class="chip-line">'
    assert chips.split(line)[1].startswith(plain.split(line)[1].split("</div>")[0] + refused + "<span")
    assert ">switch refused<" in chips.split(refused)[1]


def test_a_robotiq_goal_without_answer_is_not_short_of_its_target():
    """No result: its own red chip, not "short of target", which means the fingers stopped with nothing gripped."""
    chips, _numbers = tool_status(RobotiqState(last_update_time=100.0, last_result_ok=False, failure=NO_RESULT),
                                  100.0)
    assert ">no result<" in chips and "short of target" not in chips and FAIL in chips


def test_the_pose_plan_has_two_rows_before_and_after_the_first_tcp():
    """Before the first TCP the plan shows the same two rows, with gray dashes."""
    arm = fake_arm()
    pose = [_Slider(value) for value in (0.4, 0.0, 0.4, 180.0, 0.0, 0.0)]
    with_tcp = pose_plan_line(arm, pose)
    arm.state.tcp_position = None
    without = pose_plan_line(arm, pose)
    assert with_tcp.count("<div>") == without.count("<div>") == 2 and "—" in without
