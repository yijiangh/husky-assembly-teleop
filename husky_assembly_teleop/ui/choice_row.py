"""A choice as a row of buttons, the chosen one filled: every option in sight, and on the phone no tiny dropdown."""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable

import viser

from ..colors import CHOSEN, INK, NONE, WHITE
from .css import group_buttons

if TYPE_CHECKING:
    from ..plugin_api.context import PluginContext


class ChoiceRow:
    """One choice shown as a row of buttons (`add_button_group`), the chosen one filled.

    Attributes:
        options: The values to choose from, left to right.
    """

    def __init__(self, ctx: PluginContext, gui: viser.GuiApi, label: str, options: tuple[str, ...],
                 initial: str | None, hint: str = "", on_choose: Callable[[str], None] | None = None,
                 labels: tuple[str, ...] | None = None, height: int | None = 40):
        """Add the row and its style; a click chooses its option on the main thread (`ctx.defer_value`).

        Args:
            ctx: The plugin's context.
            gui: The GUI api.
            label: Shown above the row.
            options: The values to choose from, left to right; also the buttons' labels unless `labels` is given.
            initial: The option chosen at start; None (or any non-option) fills none.
            hint: The row's hover text.
            on_choose: Called with the option a click chose, after it is filled.
            labels: The buttons' labels, one per option; short enough that all fit one phone-wide row.
            height: Button height in px, tall enough for a finger by default; None keeps viser's.
        """
        self.options, self._value, self._on_choose, self._height = tuple(options), initial, on_choose, height
        self._labels = tuple(labels or options)
        self._group = gui.add_button_group(label, list(self._labels), hint=hint or None)
        self._style = gui.add_html(self._css())
        self._group.on_click(ctx.defer_value(f"choose {label}",
                                             lambda clicked: self._choose(self.options[self._labels.index(clicked)])))

    @property
    def value(self) -> str | None:
        """The chosen option, or None while none is."""
        return self._value

    @value.setter
    def value(self, option: str | None) -> None:
        """Fill `option` instead (none if None), without calling `on_choose`; cheap to set every draw."""
        if option != self._value:
            self._value = option
            self._style.content = self._css()

    def set_options(self, options: tuple[str, ...], labels: tuple[str, ...] | None = None) -> None:
        """Show other options in the same row and place, the first chosen; cheap to call every draw.

        Args:
            options: The new values, left to right.
            labels: Their buttons' labels; the options themselves if None.
        """
        options, labels = tuple(options), tuple(labels or options)
        if (options, labels) != (self.options, self._labels):
            self.options, self._labels, self._value = options, labels, options[0]
            self._group.options = labels
            self._style.content = self._css()

    def _choose(self, option: str) -> None:
        """Make the clicked `option` the chosen one and tell `on_choose`."""
        self.value = option
        if self._on_choose is not None:
            self._on_choose(option)

    def _css(self) -> str:
        """Every button outlined, `height` high; the chosen one filled."""
        every = group_buttons(self._group._impl.uuid)
        height = "" if self._height is None else f"height: {self._height}px !important; "
        css = (f"{every} {{ {height}background: transparent !important; "
               f"color: {INK} !important; border: 1px solid {NONE} !important; }}")
        if self._value in self.options:
            chosen = group_buttons(self._group._impl.uuid, self.options.index(self._value))
            css += f"{chosen} {{ background: {CHOSEN} !important; color: {WHITE} !important; }}"
        return f"<style>{css}</style>"
