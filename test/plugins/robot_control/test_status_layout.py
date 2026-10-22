"""robot_control's readouts keep their height: chips on one fixed line, numbers on a fixed count of rows."""

from robot_fakes import fake_arm

from husky_assembly_teleop.plugins.robot_control.arm_panel import pose_plan_line
from husky_assembly_teleop.plugins.robot_control.status import arm_status, phone_arm_status, refusal_chip
from husky_assembly_teleop.robot_interface.refusal import Refusal
from husky_assembly_teleop.ui.style import chip_line


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
        html = arm_status(arm, 100.0, extra)
        assert html.count('class="chip-line"') == 1 and html.index("chip-line") < html.index("q   ")
        assert phone_arm_status(arm, extra).startswith(chip_line("")[:40])


def test_the_pose_plan_has_two_rows_before_and_after_the_first_tcp():
    """Before the first TCP the plan shows the same two rows, with gray dashes."""
    arm = fake_arm()
    pose = [_Slider(value) for value in (0.4, 0.0, 0.4, 180.0, 0.0, 0.0)]
    with_tcp = pose_plan_line(arm, pose)
    arm.state.tcp_position = None
    without = pose_plan_line(arm, pose)
    assert with_tcp.count("<div>") == without.count("<div>") == 2 and "—" in without
