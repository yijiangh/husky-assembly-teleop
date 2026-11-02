"""A button acting on its second press: the window, the double-click guard, keys, no time limit, and its look."""

from types import SimpleNamespace

from husky_assembly_teleop.colors import WARN, rgb
from husky_assembly_teleop.ui.second_press import (DOUBLE_CLICK_SECONDS, SECOND_PRESS_SECONDS, SecondPress,
                                                   show_waiting, waiting_group_css)


def test_the_second_press_counts_within_the_window_but_not_as_a_double_click():
    """The first press asks; one within DOUBLE_CLICK_SECONDS is ignored; one later in the window acts; then it rests."""
    press = SecondPress()
    assert not press.press(10.0) and press.waiting(10.0) == "go"
    assert not press.press(10.0 + DOUBLE_CLICK_SECONDS / 2), "a double click"
    assert press.press(11.0) and press.waiting(11.0) == ""
    assert not press.press(20.0) and press.waiting(20.0 + SECOND_PRESS_SECONDS) == "go"
    assert press.waiting(20.1 + SECOND_PRESS_SECONDS) == ""
    assert not press.press(21.0 + SECOND_PRESS_SECONDS), "too late: it asks again"


def test_another_key_asks_anew_and_no_limit_waits_until_forgotten():
    """A press of another key forgets the first and waits itself; with no limit it waits until `forget`."""
    press = SecondPress(seconds=None)
    assert not press.press(0.0, "new run") and not press.press(1.0, "mark B20_J")
    assert press.waiting(1e6) == "mark B20_J" and press.press(1e6, "mark B20_J")
    press.press(0.0, "new run")
    press.forget()
    assert press.waiting(0.1) == "" and "press it again to go on" in press.hint


def test_a_waiting_button_is_amber_and_says_press_again():
    """Amber and its own label with a "?" (or the one given) while waiting; its own label and colour once it stops."""
    button = SimpleNamespace(label="Export", color=None)
    show_waiting(button, True, "Export", None)
    assert (button.label, button.color) == ("Export?", rgb(WARN))
    show_waiting(button, True, "Export", None, "Export anyway?")
    assert button.label == "Export anyway?"
    show_waiting(button, False, "Export", None)
    assert (button.label, button.color) == ("Export", None)
    group = SimpleNamespace(_impl=SimpleNamespace(uuid="g1"))
    assert waiting_group_css(group, 2).startswith('div:has(> div > p > label[for="g1"]) button:nth-child(3)')
