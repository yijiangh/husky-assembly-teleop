"""How the health panel judges a stream's rate, largest gap and delay (`stream_check`)."""

from __future__ import annotations

import pytest

from husky_assembly_teleop.plugins.health import stream_check
from husky_assembly_teleop.robot_interface.stream_stats import StreamQuality
from husky_assembly_teleop.world.checks import BAD, GOOD, WARN


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


@pytest.mark.parametrize("delay, level", [
    (0.002, GOOD), (-0.003, GOOD), (0.05, WARN), (0.2, BAD), (-0.01, WARN), (-0.05, BAD), (None, GOOD)])
def test_stream_check_delay_levels(delay: float | None, level: int):
    """Clock thresholds: 30 / 100 ms late, 5 / 20 ms ahead; unstamped is not judged."""
    assert stream_check("left", StreamQuality(rate=49.5, max_gap=0.025, delay=delay)).level == level


def test_stream_check_uses_the_given_rate():
    """28 Hz is fine for the base's 28 Hz stream, amber for the arms' 50 Hz."""
    quality = StreamQuality(rate=28.0, max_gap=0.04, delay=0.003)
    assert stream_check("base", quality, rate=28.0).level == GOOD
    assert stream_check("left", quality).level == WARN


def test_stream_check_text_does_not_change_with_the_numbers():
    """Two different amber readings give the same tooltip, so the row is not rebuilt every tick."""
    assert stream_check("left", StreamQuality(80.0, 0.01)) == stream_check("left", StreamQuality(70.0, 0.02))
