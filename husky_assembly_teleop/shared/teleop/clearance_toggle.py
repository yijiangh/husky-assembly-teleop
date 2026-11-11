"""The Clearance box below every D-pad, jog and commit: while ticked, its moves keep a clearance (`ctx.clearance`)."""

from __future__ import annotations

from typing import Any

import viser

from ...plugin_api.context import PluginContext
from ...ui.style import BUSY, FAIL, chip
from ...world.clearance import ClearanceWatch

#: The chip while the robot already stands too close for what the buttons start (`ClearanceToggle.start_chip`).
STARTS_TOO_CLOSE = "starts too close"
#: The chip while a plan keeps less than usual, so its clearance is smaller too (`close_chip`).
CLOSE_PLAN = "small clearance"


def close_chip(note: str) -> str:
    """An amber chip saying why a plan's clearance is below the usual one (`MoveKind.close_note`), or "" without."""
    return chip(CLOSE_PLAN, BUSY, f"{note}.\nA path followed a little off may still trip it.") if note else ""


class ClearanceToggle:
    """One Clearance setting, ticked at start, shown as a box below each set of buttons it guards.

    An operator's tick on any of its boxes ticks them all; unticking lets the robot move out of a tight spot. A
    commit's toggle also watches the robot it would move (`watch`), to warn before a press (`start_chip`).

    Attributes:
        hint: The boxes' hover text: what the moves keep clear of, and when to untick.
        boxes: Every box showing it.
    """

    def __init__(self, hint: str):
        """Ticked, with no box yet (`add`) and no watch."""
        self.hint = hint
        self.boxes: list[viser.GuiCheckboxHandle] = []
        self._value = True
        self._watch: ClearanceWatch | None = None

    @property
    def value(self) -> bool:
        """bool: Whether the moves keep their clearance; setting it ticks or unticks every box."""
        return self._value

    @value.setter
    def value(self, value: bool) -> None:
        self._value = bool(value)
        for box in self.boxes:
            if box.value != self._value:
                box.value = self._value

    def add(self, ctx: PluginContext, gui: viser.GuiApi, label: str) -> viser.GuiCheckboxHandle:
        """Add a box showing it, below the buttons it guards; `label` names what they move, for intent names."""
        box = gui.add_checkbox("Clearance", initial_value=self._value, hint=self.hint)
        box.on_update(ctx.defer_value(f"clearance {label}", lambda value: setattr(self, "value", value),
                                      operator_only=True))
        self.boxes.append(box)
        return box

    def watch(self, ctx: PluginContext, serial: str | None, margin: float, **options: Any) -> None:
        """Watch robot `serial` at the margin its moves keep (`ctx.watch_clearance`); None stops watching.

        Call it every tick: a new robot or margin replaces the watch. `options` go to `ctx.watch_clearance`.
        """
        watch = self._watch
        if watch is not None and serial is not None and (watch.serials, watch.margin) == ((serial,), margin):
            return
        if watch is not None:
            ctx.unwatch_clearance(watch)
        self._watch = None if serial is None else ctx.watch_clearance([serial], margin, **options)

    def start_chip(self, buttons: str) -> str:
        """A chip while the watched robot already stands too close: red while ticked, `buttons` refused; else amber."""
        watch = self._watch
        found = None if watch is None else watch.found.get(watch.serials[0])
        if found is None:
            return ""
        if self._value:
            return chip(STARTS_TOO_CLOSE, FAIL, f"{found}.\nClearance refuses {buttons}; untick it to move out.")
        return chip(STARTS_TOO_CLOSE, BUSY, f"{found}.\nClearance is off: {buttons} moves anyway, nothing soft "
                                            f"stops it.")
