"""The process panel: the action list, the texts' heights, and a movement's old name, kinds and sentence."""

from __future__ import annotations

import re
from dataclasses import replace

import pytest
from planning_fixtures import purged_process
from plugin_fakes import FakeContext
from process_fixtures import build_process

from husky_assembly_teleop.plugins.process.loading import load_process, process_folders
from husky_assembly_teleop.plugins.process.plugin import (NO_FOLDER, NO_MOVEMENT, ProcessPlugin, _movement_css,
                                                          action_window)
from husky_assembly_teleop.shared.assembly_planning.labels import INSERT_TIGHTEN
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


def test_the_folder_lists_the_processes_and_the_configured_one(tmp_path):
    """Processes and old exports two levels below the Drive root, newest name first, and the monitor's own folder."""
    for path in ("study/260101_a/process.json", "study/260202_b/ActionSchedule.json", "study/notes/readme.txt",
                 "study/deep/c/process.json"):
        (tmp_path / path).parent.mkdir(parents=True, exist_ok=True)
        (tmp_path / path).write_text("{}")
    assert process_folders(tmp_path) == ["study/260202_b", "study/260101_a"] and process_folders(None) == []

    def folder(configured: str | None, root=tmp_path):
        ctx = FakeContext(drive_root=root)
        ctx.config.process_directory = configured
        ProcessPlugin().setup(ctx)
        widget = next(widget for widget in ctx.gui.widgets if widget.label == "Folder")
        return widget.value, widget.options["initial_value"]

    assert folder("study/260101_a") == (["study/260202_b", "study/260101_a"], "study/260101_a")
    assert folder(None) == (["study/260202_b", "study/260101_a"], "study/260202_b")
    assert folder("/elsewhere/p") == (["study/260202_b", "study/260101_a", "/elsewhere/p"], "/elsewhere/p")
    assert folder(None, root=None) == ([NO_FOLDER], NO_FOLDER)


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
    labels = plugin._labels(action)
    assert "nth-child(1):hover" not in style, "the selected movement's details are below the buttons: no tip"
    tip = f'content: "{labels[1].old_name}\\A {labels[1].text}"'
    assert f"nth-child(2):hover)::after {{ display: block; {tip}; }}" in style

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


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process()


@pytest.fixture
def small(tmp_path):
    """The small synthetic process (cindy and alice, five actions), written and loaded as the monitor loads it."""
    write(build_process(tmp_path), tmp_path / "process")
    return load_process(tmp_path / "process", tmp_path)


def test_the_panel_says_which_drives_are_never_sent(purged):
    """A ground bar's authored tightens show as not sent in the moves line; a screwed bar's do not."""
    panel = ProcessPlugin()
    panel._take(purged.process)
    for movement_id, marked in (("B20_J_insert", True), ("B22_J_insert", False)):
        panel.select(next(index for index, (_, one) in enumerate(panel._movements) if one.id == movement_id))
        assert ("tighten (not sent)" in _details(panel)) == marked, movement_id


def test_the_process_panel_shows_the_old_name_the_row_and_the_sentence(purged):
    """At a screwed insert: its kinds' chips, the old name and the sentence; the action's codes, this one ringed.

    The transfer workaround is on at start: the row is in run order, the transit first, no travel to load.
    """
    panel = ProcessPlugin()
    panel._take(purged.process)
    panel.select(next(index for index, (_, one) in enumerate(panel._movements) if one.id == "B22_J_insert"))
    html = _details(panel)
    assert f"old name {INSERT_TIGHTEN}" in html and "Straight-line insert 15 mm" in html
    assert "1px dashed #666666" in html and "linear-gradient(90deg,#d5e8d4,#e6d0de)" in html
    action, movement = panel.selected
    labels = panel._labels(action)
    codes = [label.code for label in labels]
    assert codes[0] == "J_transit" and {"J_M1", "J_M2", "J_M4+5"} <= set(codes)
    assert "J_M0" not in codes and "J_M3" not in codes
    # * Only the selected movement's button is ringed, and every button keeps one weight, so nothing moves.
    css = _movement_css("u", [label.kinds[0] for label in labels], action.movements.index(movement),
                        [None] * len(labels))
    assert css.count("box-shadow") == 1 and css.count("font-weight") == 1


def test_the_panel_escapes_rhino_labels(small):
    """A movement's label from Rhino is text, never HTML, in the chips' hover text and the sentence."""
    first_id = small.schedule[0]
    first = small.actions[first_id]
    odd = replace(first, movements=(replace(first.movements[0], label="<b>mount</b> & go"), *first.movements[1:]))
    panel = ProcessPlugin()
    panel._take(replace(small, actions={**small.actions, first_id: odd}))
    panel.select(0)
    html = _details(panel)
    assert "<b>mount</b>" not in html and "&lt;b&gt;mount&lt;/b&gt; &amp; go" in html


def _details(panel: ProcessPlugin) -> str:
    """The selected movement's details as the panel shows them, above and below its movement buttons."""
    action, movement = panel.selected
    return "".join(panel._details_html(action, movement, panel._labels(action)))
