"""Tests for the chip helpers: hover texts stay readable."""

from __future__ import annotations

import re

from husky_assembly_teleop.ui.style import HOVER_MAX, chip, freshness_chip, hover, swatch_chip
from husky_assembly_teleop.world.checks import STALE_AFTER


def _title(html: str) -> str:
    """The chip's hover text."""
    return re.search(r'title="([^"]*)"', html).group(1)


def test_a_long_hover_text_is_cut_at_a_word_and_a_short_one_kept():
    """A tooltip longer than about three lines is unreadable."""
    long = "word " * 100
    for html in (chip("x", "#000000", long), swatch_chip("x", "#ffffff", "#000000", long)):
        title = _title(html)
        assert len(title) <= HOVER_MAX and title.endswith("word…")
    assert _title(chip("x", "#000000", "short")) == "short"


def test_a_stale_chip_hover_text_does_not_change_with_its_age():
    """The handle would change every second and drop the tooltip."""
    assert freshness_chip("joints", 0.0, 2 * STALE_AFTER) == freshness_chip("joints", 0.0, 50 * STALE_AFTER)


def test_hover_keeps_its_ending_within_the_limit():
    """The chip's backstop would otherwise cut "full text in …" off again."""
    text = hover("word " * 100, "the console")
    assert len(text) <= HOVER_MAX and text.endswith("full text in the console")
