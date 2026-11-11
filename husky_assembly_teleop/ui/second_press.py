"""
A button that acts on its second press: the first press only asks, and the button turns amber with a "?" ("Commit?").

One look and one timing for every such button (Commit, Exec, Start, Export floor calibration, New run, Done, ...):

- The second press counts within SECOND_PRESS_SECONDS of the first (or with no limit), not sooner than
  DOUBLE_CLICK_SECONDS: a double click never answers the question it raised.
- While it waits the button is amber (WARN) and reads `again_label(label)`: its own label, about as wide; the question
  goes on the panel's message line.
- A press counts only for what it asked about (`key`): ask about a new plan or new reasons and it asks again.
"""

from __future__ import annotations

import math

import viser

from ..colors import WARN, WHITE, rgb
from .css import group_buttons

#: Seconds a first press waits for its second.
SECOND_PRESS_SECONDS = 5.0
#: A second press sooner than this after the first is a double click: ignored, the first still waits.
DOUBLE_CLICK_SECONDS = 0.25


def again_label(label: str) -> str:
    """The label of a button waiting for its second press: its own with a question mark, e.g. "Commit?"."""
    return f"{label}?"


class SecondPress:
    """The first press of a button (or of one of several buttons, told apart by `key`) waiting for its second.

    Attributes:
        seconds: How long a first press waits; None for no limit (then any other press forgets it).
        key: What waits, e.g. "Start" or "mark B20_J"; "" when nothing waits (see `waiting`).
    """

    def __init__(self, seconds: float | None = SECOND_PRESS_SECONDS):
        """Wait for nothing yet."""
        self.seconds = seconds
        self.key = ""
        self._pressed_at = -math.inf

    def press(self, now: float, key: str = "go") -> bool:
        """Whether this press of `key` is the second; else it waits for one (a double click is ignored).

        ! Call it last, after every other refusal, so the second press is never asked for in vain.

        Args:
            now: The press's time, seconds (`ctx.now()`).
            key: Which button or question; a press of another key forgets the one waiting and waits itself.

        Returns:
            bool: True to act now (nothing waits any more); False when the press only asked.
        """
        if self.waiting(now) == key:
            if now - self._pressed_at < DOUBLE_CLICK_SECONDS:
                return False
            self.forget()
            return True
        self.key, self._pressed_at = key, now
        return False

    def waiting(self, now: float) -> str:
        """The key whose first press waits at `now`, or ""."""
        if self.key and (self.seconds is None or now - self._pressed_at <= self.seconds):
            return self.key
        return ""

    def forget(self) -> None:
        """Wait for nothing."""
        self.key, self._pressed_at = "", -math.inf

    @property
    def hint(self) -> str:
        """For a button's hover text: how soon the second press must come."""
        return ("Asks first: press it again to go on." if self.seconds is None
                else f"Asks first: press it again within {self.seconds:g} s to go on.")


def show_waiting(button: viser.GuiButtonHandle, waiting: bool, label: str, color: tuple[int, int, int] | str | None,
                 again: str = "") -> None:
    """Draw `button` as waiting for its second press (amber, `again` or `again_label(label)`) or at rest."""
    label, color = (again or again_label(label), rgb(WARN)) if waiting else (label, color)
    if button.label != label:
        button.label = label
    if button.color != color:
        button.color = color


def waiting_group_css(group: viser.GuiButtonGroupHandle, index: int) -> str:
    """CSS making option `index` of a button group amber, for an html widget's `<style>`, while it waits."""
    return f"{group_buttons(group._impl.uuid, index)} {{ background: {WARN} !important; color: {WHITE} !important; }}"
