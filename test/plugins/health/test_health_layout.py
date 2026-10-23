"""Health's robot rows keep their height: each part's chips stay on one line, however many or long they get."""

from husky_assembly_teleop.plugins.health import render_line
from husky_assembly_teleop.world.checks import BAD, Check


def test_a_robot_line_is_a_chip_line_with_the_name_first():
    """The part's name, then its chips, on one chip line."""
    html = render_line("arm", [Check("arm PROTECTIVE STOP", BAD), Check("ESTOP", BAD)])
    assert html.count('class="chip-line"') == 1
    assert html.index("chip-line") < html.index(">arm</span>") < html.index("PROTECTIVE STOP")
