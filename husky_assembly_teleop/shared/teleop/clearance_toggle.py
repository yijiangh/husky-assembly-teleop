"""The Clearance box below every D-pad and jog: while ticked, its moves keep a clearance (`ctx.clearance`)."""

from __future__ import annotations

import viser

from ...plugin_api.context import PluginContext


class ClearanceToggle:
    """One Clearance setting, ticked at start, shown as a box below each set of buttons it guards.

    An operator's tick on any of its boxes ticks them all; unticking lets the robot move out of a tight spot.

    Attributes:
        hint: The boxes' hover text: what the moves keep clear of, and when to untick.
        boxes: Every box showing it.
    """

    def __init__(self, hint: str):
        """Ticked, with no box yet (`add`)."""
        self.hint = hint
        self.boxes: list[viser.GuiCheckboxHandle] = []
        self._value = True

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
