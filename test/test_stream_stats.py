"""The signal chip: stream rate and largest gap, and how the health panel judges them."""

from __future__ import annotations

import pytest

from husky_assembly_teleop.plugins.health import stream_check
from husky_assembly_teleop.robot_interface.stream_stats import StreamQuality, StreamStats
from husky_assembly_teleop.world.checks import BAD, GOOD, WARN


def _stream(stamps: list[float]) -> StreamStats:
    """A stream whose messages arrive at their own stamps, from 100 s on."""
    stats = StreamStats(window=2.0)
    for stamp in stamps:
        stats.add(100.0 + stamp, 100.0 + stamp)
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


def test_silent_stream_reads_zero():
    """Nothing for longer than the window: rate 0."""
    assert _stream(_steady(5.0)).quality(now=110.0).rate == 0.0


@pytest.mark.parametrize("quality, level", [
    (StreamQuality(rate=None, max_gap=None), WARN),
    (StreamQuality(rate=0.0, max_gap=None), BAD),
    (StreamQuality(rate=49.5, max_gap=0.025), GOOD),
    (StreamQuality(rate=40.0, max_gap=0.025), WARN),
    (StreamQuality(rate=49.5, max_gap=0.15), WARN),
    (StreamQuality(rate=15.0, max_gap=0.025), BAD),
    (StreamQuality(rate=47.5, max_gap=0.5), BAD),
])
def test_stream_check_levels(quality: StreamQuality, level: int):
    """Thresholds at 50 Hz joint_states: 45 / 25 Hz and 100 / 200 ms."""
    assert stream_check("left", quality).level == level


def test_stream_check_text_does_not_change_with_the_numbers():
    """Two different amber readings give the same tooltip, so the row is not rebuilt every tick."""
    assert stream_check("left", StreamQuality(80.0, 0.01)) == stream_check("left", StreamQuality(70.0, 0.02))
