"""A choice as a row of buttons, the chosen one filled: on the phone a dropdown opens the keyboard and is tiny."""

from __future__ import annotations

import viser

from ...colors import GREY, INK, WHITE
from ...plugin_api.context import PluginContext
from ...ui.css import group_buttons


class ChoiceRow:
    """One choice shown as a row of buttons (`add_button_group`), the chosen one filled.

    Attributes:
        value: The chosen option's label.
    """

    def __init__(self, ctx: PluginContext, gui: viser.GuiApi, label: str, options: tuple[str, ...], initial: str,
                 hint: str = ""):
        """Add the row and its style; a click chooses its option on the main thread (`ctx.defer_value`).

        Args:
            ctx: The plugin's context.
            gui: The GUI api.
            label: Shown above the row.
            options: The buttons' labels, left to right; short enough that all fit one phone-wide row.
            initial: The option chosen at start.
            hint: The row's hover text.
        """
        self.options, self.value = tuple(options), initial
        self._group = gui.add_button_group(label, list(options), hint=hint or None)
        self._style = gui.add_html(self._css())
        self._group.on_click(ctx.defer_value(f"choose {label}", self._choose))

    def _choose(self, option: str) -> None:
        """Make `option` the chosen one, filled."""
        self.value = option
        self._style.content = self._css()

    def _css(self) -> str:
        """Every button outlined, tall enough for a finger; the chosen one filled."""
        every = group_buttons(self._group._impl.uuid)
        chosen = group_buttons(self._group._impl.uuid, self.options.index(self.value))
        return (f"<style>{every} {{ height: 40px !important; background: transparent !important; "
                f"color: {INK} !important; border: 1px solid {GREY} !important; }}"
                f"{chosen} {{ background: {INK} !important; color: {WHITE} !important; }}</style>")
