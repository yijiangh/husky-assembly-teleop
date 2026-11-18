"""The row of robot buttons: icon and short id, the chosen one highlighted, whichever way it was chosen."""

from plugin_fakes import FakeContext

from husky_assembly_teleop.colors import CURRENT, NONE, rgb
from husky_assembly_teleop.ui.robot_buttons import RobotButtons

SERIALS = ["a200-0804", "a200-0805", "a200-0806"]


def colors(row: RobotButtons) -> list[tuple[int, int, int]]:
    """The buttons' colours, left to right."""
    return [button.color for button in row.buttons.values()]


def test_a_button_per_robot_side_by_side_with_the_short_id():
    """The labels are as Control's tabs, and the row's CSS names each button, so the row is one line high."""
    gui = FakeContext().view.gui
    row = RobotButtons(gui, SERIALS, SERIALS[0])
    assert [button.label for button in row.buttons.values()] == ["0804", "0805", "0806"]
    style = gui.of_kind("html")[-1].content
    assert all(f'button[id="{button._impl.uuid}"]' in style for button in row.buttons.values())


def test_the_highlight_follows_the_robot_shown():
    """Chosen by a press or by the panel itself: the same call moves it."""
    row = RobotButtons(FakeContext().view.gui, SERIALS, SERIALS[0])
    assert colors(row) == [rgb(CURRENT), rgb(NONE), rgb(NONE)]
    row.show(SERIALS[2])
    assert colors(row) == [rgb(NONE), rgb(NONE), rgb(CURRENT)]
