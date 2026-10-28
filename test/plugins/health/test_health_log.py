"""Health logs a chip's colour change once it has held for LOG_HOLD seconds, one line per row."""

from types import SimpleNamespace

from plugin_fakes import FakeContext

from husky_assembly_teleop.plugins.health import CHIPS_TOPIC, LOG_HOLD, SIM_LABEL, HealthPlugin
from husky_assembly_teleop.world.checks import BAD, GOOD, INFO, WARN, Check


def _run(levels_at: list[tuple[float, list[int]]], until: float) -> FakeContext:
    """Feed the chip levels of row "0804 base" at each time, then tick to `until`; returns the context with its logs."""
    health, ctx = HealthPlugin(), FakeContext()
    times = [t for t, _ in levels_at] + [until]
    for t, levels in levels_at:
        health._note_changes("0804 base", [Check(f"chip{index}", level, f"why {level}")
                                           for index, level in enumerate(levels)], t)
        health._log_changes(ctx, t)
    health._log_changes(ctx, max(times))
    return ctx


def test_the_first_levels_are_not_logged():
    """At start every chip is still waiting for its robot."""
    assert _run([(0.0, [BAD, GOOD])], until=10.0).logs == []


def test_a_change_is_logged_once_it_held():
    """Not while it is newer than LOG_HOLD, then once: an error when the chip turned red."""
    held = [(0.0, [GOOD]), (1.0, [BAD])]
    assert _run(held, until=1.0 + LOG_HOLD / 2).logs == []
    assert _run(held, until=1.0 + LOG_HOLD).logs == [("error", "0804 base: chip0 green -> red: why 2")]


def test_a_flickering_chip_makes_one_line():
    """Changes within LOG_HOLD of each other add up into one line with their count."""
    flicker = [(0.0, [GOOD]), (1.0, [WARN]), (1.2, [GOOD]), (1.4, [WARN])]
    assert _run(flicker, until=5.0).logs == [("warn", "0804 base: chip0 green -> amber (3 changes): why 1")]


def test_a_chip_back_to_its_colour_says_it_flickered_and_warns_if_it_was_red():
    """Red for a moment is a warning; amber for a moment only information."""
    assert _run([(0.0, [GOOD]), (1.0, [BAD]), (1.1, [GOOD])], until=5.0).logs == [
        ("warn", "0804 base: chip0 flickered red (2 changes), green again")]
    assert _run([(0.0, [GOOD]), (1.0, [WARN]), (1.1, [GOOD])], until=5.0).logs == [
        ("info", "0804 base: chip0 flickered amber (2 changes), green again")]


def test_a_red_chip_green_for_a_moment_flickered_green_not_red():
    """The flicker names the colour furthest from where the chip started: a red chip briefly green is only news."""
    assert _run([(0.0, [BAD]), (1.0, [GOOD]), (1.1, [BAD])], until=5.0).logs == [
        ("info", "0804 base: chip0 flickered green (2 changes), red again")]
    assert _run([(0.0, [WARN]), (1.0, [GOOD]), (1.1, [WARN])], until=5.0).logs == [
        ("info", "0804 base: chip0 flickered green (2 changes), amber again")]


def test_red_for_a_moment_on_the_way_to_a_better_colour_is_a_warning():
    """Amber, red for a moment, then green: better in the end, but the red moment is said and warned about."""
    assert _run([(0.0, [WARN]), (1.0, [BAD]), (1.1, [GOOD])], until=5.0).logs == [
        ("warn", "0804 base: chip0 amber -> green (2 changes, red for a moment): why 0")]


def test_colour_changes_are_logged_under_the_chips_topic():
    """So the console's default view can leave them out, and keep health's other messages."""
    health, logged = HealthPlugin(), []
    ctx = SimpleNamespace(log_error=lambda message, topic="": logged.append(topic))
    health._note_changes("0804 base", [Check("chip0", GOOD)], 0.0)
    health._note_changes("0804 base", [Check("chip0", BAD)], 1.0)
    health._log_changes(ctx, 1.0 + LOG_HOLD)
    assert logged == [CHIPS_TOPIC]


def test_changes_of_one_row_share_a_line_at_the_level_of_the_worst():
    """Two chips of the row changing together: one line, an error when one of them turned red."""
    assert _run([(0.0, [GOOD, GOOD]), (1.0, [WARN, BAD])], until=5.0).logs == [
        ("error", "0804 base: chip0 green -> amber: why 1; chip1 green -> red: why 2")]


def test_a_sim_chip_appearing_in_front_shifts_no_other_chip():
    """The sim chip comes and goes before a part's chips: no other chip counts as changed."""
    health, ctx = HealthPlugin(), FakeContext()
    signal = Check("signal", GOOD)
    health._note_changes("0804 base", [signal, Check("mocap", BAD, "lost")], 0.0)
    health._note_changes("0804 base", [Check(SIM_LABEL, INFO, "husky_sim"), signal, Check("mocap", BAD, "lost")], 1.0)
    health._log_changes(ctx, 10.0)
    assert ctx.logs == []
