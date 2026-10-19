"""The relay's log text: wrapped to the width, boxes around warnings, no escape codes without colour."""

from husky_assembly_teleop.mocap_relay.log_text import WIDTH, box, wrap


def test_wrap_keeps_lines_and_long_words():
    """Lines wrap to the width, short lines stay, a path longer than the width stays whole."""
    text = "short line\n" + "word " * 60 + "\n" + "a/" * 70
    lines = wrap(text).splitlines()
    assert lines[0] == "short line"
    assert all(len(line) <= WIDTH for line in lines[1:-1]) and lines[2].startswith("  ")
    assert lines[-1] == "a/" * 70  # a path is never cut


def test_box_without_colour(monkeypatch):
    """A box has its title on top, every line framed, and no escape codes when ROS does not colour."""
    monkeypatch.setenv("RCUTILS_COLORIZED_OUTPUT", "0")
    lines = box("title", "body " * 40).splitlines()
    assert lines[0] == "title" and "\033" not in "".join(lines)
    assert lines[1].startswith("┌") and lines[-1].startswith("└") and all(line.startswith("│ ") for line in lines[2:-1])
    assert all(len(line) <= WIDTH + 1 for line in lines)
