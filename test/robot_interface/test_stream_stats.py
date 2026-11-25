"""A stream's rate, largest gap and delay, as the signal chip reads them."""

from __future__ import annotations

import pytest

from husky_assembly_teleop.robot_interface.stream_stats import PROMPT_WINDOW, StreamStats


def _stream(stamps: list[float], late: float = 0.0) -> StreamStats:
    """A stream whose messages arrive `late` seconds after their stamps, from 100 s on."""
    stats = StreamStats(window=2.0)
    for stamp in stamps:
        stats.add(100.0 + stamp + late, 100.0 + stamp)
    return stats


def _steady(seconds: float, rate: float = 100.0) -> list[float]:
    """Stamps of a stream at `rate` for `seconds`."""
    return [i / rate for i in range(int(seconds * rate))]


def test_steady_stream_reads_its_rate_and_period():
    """100 Hz for 5 s: about 100 Hz, gaps of one period."""
    quality = _stream(_steady(5.0)).quality(now=105.0)
    assert quality.rate == pytest.approx(100.0, rel=0.02)
    assert quality.max_gap == pytest.approx(0.01)


def test_lost_samples_open_a_gap_and_lower_the_rate():
    """Dropping 30 samples in a row: a 0.31 s gap, 85 Hz."""
    stamps = [stamp for stamp in _steady(5.0) if not 4.0 <= stamp < 4.3]
    quality = _stream(stamps).quality(now=105.0)
    assert quality.max_gap == pytest.approx(0.31)
    assert quality.rate == pytest.approx(85.0, rel=0.02)


def test_delay_is_the_smallest_lateness():
    """Messages 3 ms late, one batch held 40 ms by the monitor's tick: the delay is 3 ms."""
    stats = _stream(_steady(5.0), late=0.003)
    stats.add(105.0, 104.96)
    assert stats.quality(now=105.0).delay == pytest.approx(0.003)


def test_clock_ahead_gives_a_negative_delay():
    """Stamps 30 ms ahead of the monitor's clock, 2 ms network: -28 ms."""
    assert _stream(_steady(5.0), late=-0.028).quality(now=105.0).delay == pytest.approx(-0.028)


def test_never_seen_stream_is_marked():
    """No message ever: not seen, rate 0, nothing else known."""
    quality = StreamStats(window=2.0).quality(now=105.0)
    assert (quality.seen, quality.rate, quality.max_gap, quality.delay) == (False, 0.0, None, None)


def test_rate_is_unknown_until_a_full_window():
    """One second after the first message the rate is not judged yet."""
    assert _stream(_steady(1.0)).quality(now=101.0).rate is None


def test_unstamped_stream_has_no_gap():
    """Stamp 0.0 everywhere: the rate still counts, the gap is unknown."""
    stats = StreamStats(window=2.0)
    for arrival in _steady(5.0):
        stats.add(100.0 + arrival, 0.0)
    quality = stats.quality(now=105.0)
    assert quality.rate == pytest.approx(100.0, rel=0.02)
    assert quality.max_gap is None
    assert quality.delay is None


def test_silent_stream_reads_zero():
    """Nothing for longer than the window: rate 0."""
    assert _stream(_steady(5.0)).quality(now=110.0).rate == 0.0


@pytest.mark.parametrize("clock", [0.0, 233000.0, -5.0])
def test_lateness_is_against_the_promptest_message_whatever_the_clocks_say(clock):
    """A sender's clock days behind or ahead changes nothing; a message 2 s later than the rest reads 2 s late."""
    stats = StreamStats()
    for stamp in _steady(5.0):
        stats.add(300000.0 + stamp + 0.003, 300000.0 + stamp - clock)
    assert stats.lateness(300005.0 + 0.003, 300005.0 - clock) == pytest.approx(0.0, abs=1e-6)
    assert stats.lateness(300007.0 + 0.003, 300005.0 - clock) == pytest.approx(2.0)


def test_lateness_forgets_the_promptest_message():
    """The sender's clock stepped back 10 s: late at first, the new normal after two PROMPT_WINDOWs."""
    stats = StreamStats()
    stats.add(100.0, 100.0)
    stats.add(101.0, 91.0)
    assert stats.lateness(101.0, 91.0) == pytest.approx(10.0)
    for k in range(1, 5):
        stats.add(101.0 + k * PROMPT_WINDOW, 91.0 + k * PROMPT_WINDOW)
    assert stats.lateness(101.0 + 4 * PROMPT_WINDOW, 91.0 + 4 * PROMPT_WINDOW) == pytest.approx(0.0)


def test_an_unstamped_or_new_stream_is_not_late():
    """No stamp, or nothing to compare with yet: 0."""
    stats = StreamStats()
    assert stats.lateness(100.0, 97.0) == 0.0
    stats.add(100.0, 99.0)
    assert stats.lateness(103.0, 0.0) == 0.0
