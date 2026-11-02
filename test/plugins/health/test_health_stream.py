"""How the health panel judges a stream's rate, largest gap and delay (`stream_check`), and folds an arm's checks."""

from __future__ import annotations

import pytest

from husky_assembly_teleop.plugins.health import LOG_HOLD, HealthPlugin, age_check, combine, stream_check
from husky_assembly_teleop.robot_interface.stream_stats import WINDOW, StreamQuality
from husky_assembly_teleop.world.checks import BAD, GOOD, PENDING, STALE_AFTER, WARN, Check


@pytest.mark.parametrize("quality, level", [
    (StreamQuality(rate=None, max_gap=None), PENDING),
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


def test_a_flapping_signal_shows_the_worse_level_until_the_better_one_held():
    """Marginal wifi: amber shows at once and stays while green comes and goes; green shows once it held LOG_HOLD."""
    health = HealthPlugin()
    good, warn = Check("signal", GOOD, "on time"), Check("signal", WARN, "a gap over 100 ms")
    shown = [health._hold_signal("0804 base", check, t).level
             for t, check in [(0.0, good), (0.5, warn), (0.6, good), (0.9, warn), (1.0, good), (1.0 + LOG_HOLD, good)]]
    assert shown == [GOOD, WARN, WARN, WARN, WARN, GOOD]


def test_the_arm_chip_lists_only_what_is_not_green_worst_first():
    """Green parts stay out of the tooltip; too long for HOVER_MAX, the worst stays whole and the others by name."""
    parts = [Check("sync", GOOD, "reporting"), Check("ctrl", WARN, "no controller active"),
             Check("NO PROGRAM", BAD, "x" * 120), Check("joints", WARN, "no joint_states for over 1s")]
    chip = combine("arm", parts)
    assert chip.label == "arm NO PROGRAM" and chip.detail == f"NO PROGRAM: {'x' * 120}\nalso ctrl, joints"
    assert combine("arm", parts[:2]).detail == "ctrl: no controller active"
    assert combine("arm", parts[:1]).detail == "all fine: sync"


def test_the_sync_last_stop_stays_in_the_arm_chip_tooltip():
    """After Stop all the sync is green but says why it stopped the arms: kept after the amber parts, or all green."""
    sync, ctrl = Check("sync", GOOD, "last stop: stopped by operator"), Check("ctrl", WARN, "no controller active")
    assert combine("arm", [sync, ctrl], noted=[sync]).detail == (
        "ctrl: no controller active\nsync: last stop: stopped by operator")
    assert combine("arm", [sync], noted=[sync]).detail == "all fine: sync\nsync: last stop: stopped by operator"


@pytest.mark.parametrize("age, level", [(0.5, GOOD), (STALE_AFTER + 0.1, WARN), (WINDOW + 0.1, BAD)])
def test_a_topic_turns_amber_when_stale_then_red_when_silent(age: float, level: int):
    """A tool's topic has no signal chip behind it: a dead driver must end red, not stay amber."""
    assert age_check("gripper", "gripper joint_states", 100.0 - age, 100.0).level == level
