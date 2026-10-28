"""The process panel: its action list keeps the selected action in its middle row, and its texts keep their height."""

from __future__ import annotations

import re

from plugin_fakes import FakeContext
from process_fixtures import build_process

from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.plugins.process.plugin import NO_MOVEMENT, ProcessPlugin, action_window
from scaffolding_assembly_core.process import write

#: The markers of a cut one-line row (`_one_line`) and of the two-line description.
ONE_LINE, DESCRIPTION = "white-space:pre;overflow:hidden", "-webkit-line-clamp:2"


def _chip_rows(html: str) -> list[tuple[bool, bool]]:
    """Per one-line row: whether it holds a plain chip, and a bordered (swatch) chip, which is 2 px taller."""
    rows = re.findall(r'white-space:pre;overflow:hidden;text-overflow:ellipsis">(.*?)</div>', html)
    swatches = [row.count("border:1px") for row in rows]
    return [(row.count("display:inline-block") > swatch, swatch > 0) for row, swatch in zip(rows, swatches)]


def test_action_window_centres_the_selection():
    """The selected action sits in the middle row, except near either end; short schedules start at the top."""
    assert action_window(10, 48, rows=7) == 7  # rows 7..13, action 10 in the middle
    assert action_window(1, 48, rows=7) == 0
    assert action_window(47, 48, rows=7) == 41
    assert action_window(2, 4, rows=7) == 0


def test_the_details_and_the_error_line_keep_their_height(tmp_path):
    """Before a Load the details have the rows they have after one; the error line is one line, even empty.

    The details sit above and below the movement buttons, and the Assembly tab's buttons below all of them, so none of
    them may change height.
    """
    ctx = FakeContext()
    ctx.config.process_directory = None
    plugin = ProcessPlugin()
    plugin.setup(ctx)
    plugin.draw(ctx)
    before = plugin._which.content + plugin._details.content
    assert "no process loaded" in before and plugin._moves.options == (NO_MOVEMENT,)
    assert plugin._error_text.content.count(ONE_LINE) == 1 and "&nbsp;" in plugin._error_text.content
    write(build_process(tmp_path), tmp_path / "process")
    plugin._take(load_process(tmp_path / "process", tmp_path))
    plugin._error = "a long error\nover two lines"
    plugin.draw(ctx)
    after = plugin._which.content + plugin._details.content
    assert (before.count(ONE_LINE), before.count(DESCRIPTION)) == (after.count(ONE_LINE), after.count(DESCRIPTION))
    assert _chip_rows(before) == _chip_rows(after)
    error = plugin._error_text.content
    assert error.count(ONE_LINE) == 1 and "\n" not in error and 'title="a long error over two lines"' in error


def test_a_movement_button_selects_that_movement_of_the_action(tmp_path):
    """The buttons are the selected action's movements by their codes, the selected one ringed; a click selects one."""
    ctx = FakeContext()
    ctx.config.process_directory = None
    plugin = ProcessPlugin()
    plugin.setup(ctx)
    write(build_process(tmp_path), tmp_path / "process")
    plugin._take(load_process(tmp_path / "process", tmp_path))
    plugin.draw(ctx)
    action, movement = plugin.selected
    codes = tuple(label.code for label in plugin._labels(action))
    assert len(codes) > 1 and movement is action.movements[0] and plugin._moves.options == codes
    style = plugin._moves_style.content
    assert style.count("box-shadow") == 1 and re.search(r"nth-child\(1\) \{[^}]*box-shadow", style)

    plugin._on_movement(codes[-1])
    assert plugin.selected == (action, action.movements[-1])
    plugin.draw(ctx)
    assert re.search(rf"nth-child\({len(codes)}\) \{{[^}}]*box-shadow", plugin._moves_style.content)
    plugin._on_movement("not a code of this action")
    assert plugin.selected == (action, action.movements[-1])


def test_the_rhino_bar_lines_are_off_at_start(tmp_path):
    """Only the operator turns the pink lines on: the box starts unticked, and so does the switch the drawing reads."""
    ctx = FakeContext()
    ctx.config.process_directory = None
    plugin = ProcessPlugin()
    plugin.setup(ctx)
    box, = [widget for widget in ctx.gui.of_kind("checkbox") if widget.label == "Rhino bar lines"]
    assert box.value is False and plugin._bar_lines is False


def test_the_step_slider_follows_only_the_operator():
    """Draw writes the Step slider back, and viser calls back for that too, late: only an operator's drag selects."""
    ctx, asked = FakeContext(), {}
    ctx.config.process_directory = None

    def defer_value(label, run, operator_only=False):
        asked[label] = operator_only
        return run

    ctx.defer_value = defer_value
    ProcessPlugin().setup(ctx)
    assert asked["select step"] is True
