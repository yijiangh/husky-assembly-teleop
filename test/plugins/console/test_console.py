"""The console shows plugin lines by default, the rest on request, newest first, at a fixed height."""

from contextlib import nullcontext
from types import SimpleNamespace

from plugin_fakes import FakeContext
from rclpy.logging import LoggingSeverity

from husky_assembly_teleop.plugin_api.console_log import ROBOTS, RUNTIME, TICKS, ConsoleLog, Entry
from husky_assembly_teleop.plugins.console import (EVERY_SOURCE, HIDDEN_MARK, LIST_HEIGHT, SHOW_ALL, SHOW_PLUGINS,
                                                   SHOW_WARNINGS, SHOWN_MARK, ConsolePlugin, render, shown)

DEBUG, INFO, WARN, ERROR = (LoggingSeverity.DEBUG, LoggingSeverity.INFO, LoggingSeverity.WARN,
                            LoggingSeverity.ERROR)


def _shown_sources(show: str, source: str = EVERY_SOURCE, hide: tuple[str, ...] = ()) -> list[tuple[str, str]]:
    """Which of a mixed set of lines `show` lets through, as (source, level name)."""
    entries = [Entry(0.0, level, name, "text") for name in ("robot_control", RUNTIME, ROBOTS, "health", "health/chips",
                                                            TICKS)
               for level in (DEBUG, INFO, WARN, ERROR)]
    return [(entry.source, entry.level.name) for entry in entries
            if shown(entry, show, source, hide)]


def test_plugins_leaves_out_health_changes_and_ticks_but_never_an_error():
    """Info and up from the plugins (health's own messages too), the runtime and the robots; else only errors."""
    assert _shown_sources(SHOW_PLUGINS) == [
        (name, level) for name in ("robot_control", RUNTIME, ROBOTS, "health") for level in ("INFO", "WARN", "ERROR")
    ] + [("health/chips", "ERROR"), (TICKS, "ERROR")]


def test_warnings_shows_warnings_from_all_but_the_ticks_and_all_shows_everything():
    """Warnings: warnings and errors except tick reports (errors always). All: every line."""
    assert _shown_sources(SHOW_WARNINGS) == [
        (name, level) for name in ("robot_control", RUNTIME, ROBOTS, "health", "health/chips")
        for level in ("WARN", "ERROR")
    ] + [(TICKS, "ERROR")]
    assert len(_shown_sources(SHOW_ALL)) == 24


def test_a_source_narrows_to_its_own_lines():
    """Source picks one source; the Show rules still apply."""
    assert _shown_sources(SHOW_ALL, TICKS) == [(TICKS, level) for level in ("DEBUG", "INFO", "WARN", "ERROR")]


def test_hide_leaves_out_its_sources_even_their_errors_and_a_name_covers_its_topics():
    """All but health/chips and the runtime, nothing of theirs; "health" hides health and health/chips."""
    assert _shown_sources(SHOW_ALL, hide=("health/chips", RUNTIME)) == [
        (name, level) for name in ("robot_control", ROBOTS, "health", TICKS)
        for level in ("DEBUG", "INFO", "WARN", "ERROR")]
    assert _shown_sources(SHOW_PLUGINS, hide=("health",)) == [
        (name, level) for name in ("robot_control", RUNTIME, ROBOTS) for level in ("INFO", "WARN", "ERROR")
    ] + [(TICKS, "ERROR")]


def test_the_list_is_newest_first_escaped_at_a_fixed_height_with_the_whole_message_wrapped():
    """Newest on top; HTML in a message is shown, not run; a traceback shows whole, wrapped, and on hover too."""
    html = render([Entry(1.0, INFO, "a", "first"), Entry(2.0, ERROR, "b", "failed <x>\nTraceback\n  line")])
    assert html.index("failed &lt;x&gt;") < html.index("first")
    assert ">failed &lt;x&gt;\nTraceback\n  line<" in html and "white-space:pre-wrap" in html
    assert 'title="failed &lt;x&gt;\nTraceback\n  line"' in html
    assert f"height:{LIST_HEIGHT}px" in html and "nothing to show" in render([])
    assert ".message {white-space:pre-wrap" in html and 'class="message"' in html, "the message wraps"


def _console() -> tuple[ConsolePlugin, FakeContext, ConsoleLog, list]:
    """A console set up in a fake context with an empty log; the list records each `minimize`."""
    ctx = FakeContext(name="console")
    log = ctx.console_log = ConsoleLog()
    minimized = []
    panel = SimpleNamespace(add_tab=lambda *_tab: nullcontext(), float=lambda **_place: None,
                            minimize=lambda: minimized.append(True))
    ctx.view = SimpleNamespace(panel=lambda: panel, gui=ctx.gui)
    console = ConsolePlugin()
    console.setup(ctx)
    ctx.gui.of_kind("dropdown")[0].value = EVERY_SOURCE  # * The fake takes the options for the value.
    return console, ctx, log, minimized


def test_the_plugin_draws_the_filtered_lines_and_offers_each_source():
    """It starts minimized; at Plugins health's colour change counts as hidden; each source is a Source choice."""
    console, ctx, log, minimized = _console()
    assert minimized, "the window starts minimized"
    log.add(INFO, "robot_control", "drive done")
    log.add(INFO, "health/chips", "0804 base: mocap green -> amber")
    console.draw(ctx)
    status, lines = ctx.gui.of_kind("html")
    assert "drive done" in lines.content and "mocap" not in lines.content
    assert "1 hidden" in status.content and "late ticks 0%" in status.content
    assert ctx.gui.of_kind("dropdown")[0].options == [EVERY_SOURCE, "health/chips", "robot_control"]
    ctx.gui.of_kind("button_group")[0].callback(SHOW_ALL)
    console.draw(ctx)
    assert "mocap" in lines.content
    hide = ctx.gui.of_kind("dropdown")[1]
    hide.callback(f"{SHOWN_MARK}health/chips")
    console.draw(ctx)
    assert "mocap" not in lines.content and "1 hidden" in status.content


def test_hide_starts_with_ticks_and_the_runtime_and_toggles_any_source_picked():
    """Hide's first choice says what it hides; picking a source ticks it, picking it again unticks it."""
    console, ctx, log, _minimized = _console()
    hide = ctx.gui.of_kind("dropdown")[1]
    assert hide.value == "hiding ticks, runtime"
    assert hide.options == ["hiding ticks, runtime", f"{HIDDEN_MARK}runtime", f"{HIDDEN_MARK}ticks"]
    log.add(INFO, RUNTIME, "plugin loaded")
    log.add(WARN, TICKS, "tick 9 started 40 ms behind")
    log.add(INFO, "robot_control", "drive done")
    ctx.gui.of_kind("button_group")[0].callback(SHOW_ALL)
    console.draw(ctx)
    lines = ctx.gui.of_kind("html")[1]
    assert "drive done" in lines.content and "plugin loaded" not in lines.content and "behind" not in lines.content
    assert f"{SHOWN_MARK}robot_control" in hide.options
    hide.callback(f"{HIDDEN_MARK}runtime")
    hide.callback(f"{SHOWN_MARK}robot_control")
    hide.callback("hiding ticks, robot_control")  # * the summary written back comes back too: it changes nothing
    console.draw(ctx)
    assert hide.value == "hiding ticks, robot_control"
    assert "plugin loaded" in lines.content and "drive done" not in lines.content and "behind" not in lines.content


def test_a_line_added_while_the_list_is_drawn_shows_at_the_next_draw():
    """A line another thread adds between reading the version and the lines is never left out for good."""
    console, ctx, log, _minimized = _console()
    read = log.entries

    def entries_then_a_line_comes_in():
        lines = read()
        log.add(ERROR, "robots", "switch failed")
        return lines

    log.entries = entries_then_a_line_comes_in
    console.draw(ctx)
    log.entries = read
    console.draw(ctx)
    assert "switch failed" in ctx.gui.of_kind("html")[1].content
