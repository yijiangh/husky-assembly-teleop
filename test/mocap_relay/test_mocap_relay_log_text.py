"""The relay's log text: wrapped to the width, boxes around warnings, no escape codes without colour."""

from husky_assembly_teleop.mocap_relay.log_text import WIDTH, WRAP_WIDTH, box, wrap


def test_wrap_keeps_lines_and_long_words():
    """Only lines over the wrap width wrap, short lines stay, a path longer than it stays whole."""
    text = "short line\n" + "word " * 60 + "\n" + "a/" * 100
    lines = wrap(text).splitlines()
    assert lines[0] == "short line"
    assert all(len(line) <= WRAP_WIDTH for line in lines[1:-1]) and lines[2].startswith("  ")
    assert lines[-1] == "a/" * 100  # a path is never cut


def test_box_without_colour(monkeypatch):
    """A box has its title on top, a rule of WIDTH, every line framed, no escape codes when ROS does not colour."""
    monkeypatch.setenv("RCUTILS_COLORIZED_OUTPUT", "0")
    lines = box("title", "body " * 60).splitlines()
    assert lines[0] == "title" and "\033" not in "".join(lines)
    assert lines[1].startswith("┌") and lines[-1].startswith("└") and all(line.startswith("│ ") for line in lines[2:-1])
    assert len(lines[1]) == WIDTH + 1 and all(len(line) <= WRAP_WIDTH for line in lines)
