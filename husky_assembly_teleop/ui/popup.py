"""
A warning window that floats beside the main panel and asks the operator to decide.

! Use it instead of viser's `add_modal`: a modal covers the page, Stop all too.
"""

from __future__ import annotations

from contextlib import contextmanager
from typing import TYPE_CHECKING, Iterator, Mapping

from .visualization import MAIN_PANEL_WIDTH

if TYPE_CHECKING:
    import viser

    from .visualization import PluginView

#: Where it floats: past the main panel, away from the side panel's buttons, so a double click cannot land on it.
POPUP_X, POPUP_Y = MAIN_PANEL_WIDTH + 30, 15
POPUP_WIDTH = 380


class Popup:
    """One window, made hidden: buttons on top, the text that changes below them (AGENTS.md, "Panels: nothing jumps").

    ! Main thread only; connect `buttons` with `ctx.defer`.

    Attributes:
        buttons: The window's buttons by label.
    """

    def __init__(self, view: PluginView, title: str, icon: str,
                 buttons: Mapping[str, tuple[int, int, int] | None] | None = None):
        """Make the window, hidden.

        Args:
            view: The plugin's view.
            title: The window's title.
            icon: Its icon.
            buttons: Each button's colour (None for the theme's) by label, top to bottom; none for a window whose
                widgets all come from `body`.
        """
        self._panel, self._gui = view.panel(), view.gui
        self._tab = self._panel.add_tab(title, icon)
        buttons = buttons or {}
        with self._tab:
            self.buttons = {label: view.gui.add_button(label, color=color, disabled=False)
                            for label, color in buttons.items()}
            self._text = view.gui.add_html("")
        self._panel.float(x=POPUP_X, y=POPUP_Y, width=POPUP_WIDTH)
        self._open = False
        self.close()

    @contextmanager
    def body(self) -> Iterator[viser.GuiApi]:
        """Add widgets of the plugin's own to the window, below its buttons and text."""
        with self._tab:
            yield self._gui

    @property
    def is_open(self) -> bool:
        """bool: Whether the window shows."""
        return self._open

    def show(self, html: str) -> None:
        """Show the window with this text, or update it; every call that changes something is a browser message."""
        if self._text.content != html:
            self._text.content = html
        if not self._open:
            self._open = True
            self._panel.expand()  # the operator may have minimized it last time
            self._panel.visible = True

    def close(self) -> None:
        """Hide the window."""
        self._open = False
        self._panel.visible = False
