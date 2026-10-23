"""The process panel: its action list keeps the selected action in its middle row, and its texts keep their height."""

from __future__ import annotations

import re

from plugin_fakes import FakeContext
from process_fixtures import build_process

from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.plugins.process.plugin import ProcessPlugin, action_window
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

    The Assembly tab's buttons sit below both, so neither may change height.
    """
    ctx = FakeContext()
    ctx.config.process_directory = None
    plugin = ProcessPlugin()
    plugin.setup(ctx)
    plugin.draw(ctx)
    before = plugin._details.content
    assert "no process loaded" in before
    assert plugin._error_text.content.count(ONE_LINE) == 1 and "&nbsp;" in plugin._error_text.content
    write(build_process(tmp_path), tmp_path / "process")
    plugin._take(load_process(tmp_path / "process", tmp_path))
    plugin._error = "a long error\nover two lines"
    plugin.draw(ctx)
    after = plugin._details.content
    assert (before.count(ONE_LINE), before.count(DESCRIPTION)) == (after.count(ONE_LINE), after.count(DESCRIPTION))
    assert _chip_rows(before) == _chip_rows(after)
    error = plugin._error_text.content
    assert error.count(ONE_LINE) == 1 and "\n" not in error and 'title="a long error over two lines"' in error
