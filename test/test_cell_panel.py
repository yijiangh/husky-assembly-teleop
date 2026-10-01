"""The cell panel's action list: a window of the schedule that keeps the selected action in its middle row."""

from __future__ import annotations

from husky_assembly_teleop.plugins.cell.plugin import action_window


def test_action_window_centres_the_selection():
    """The selected action sits in the middle row, except near either end; short schedules start at the top."""
    assert action_window(10, 48, rows=7) == 7  # rows 7..13, action 10 in the middle
    assert action_window(1, 48, rows=7) == 0
    assert action_window(47, 48, rows=7) == 41
    assert action_window(2, 4, rows=7) == 0
