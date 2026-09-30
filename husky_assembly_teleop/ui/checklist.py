"""
A selectable list: one checkbox per item in a collapsible folder, "Select all" on top.

viser 1.1 has no list or multi-select widget; this is the stand-in. Build one in
`setup`, `add` / `remove` items from intents, read `selected()` when acting on
the selection, and call `sync()` from `draw`.

! Item checkboxes have no callbacks: their values are operator settings, read
  when needed. Only "Select all" acts, through an intent.

* Rows are packed tightly, with the box before the label, by CSS (`_tight_style`).

! viser runs `on_update` also when Python assigns `.value`. "Select all" is
  assigned in `sync`, so its callback ignores events without a client, or every
  sync would untick the list.
"""

from __future__ import annotations

from typing import Hashable

import viser

from ..plugin_api.context import PluginContext
from .style import note


class CheckList:
    """Checkboxes for a changing set of items, keyed by any hashable (e.g. an id)."""

    def __init__(self, ctx: PluginContext, gui: viser.GuiApi, label: str, empty_text: str = "nothing yet"):
        """Build the folder, the "Select all" checkbox and the empty-list note.

        ! Call inside `ctx.view.ui()`, so the folder lands in the plugin's folder.

        Args:
            ctx: The owning plugin's context, for routing "Select all" to the main thread.
            gui: The GUI api from `ctx.view.ui()`.
            label: Folder title, e.g. "Samples".
            empty_text: Shown while the list is empty.
        """
        self._ctx = ctx
        self._gui = gui
        self._folder = gui.add_folder(label)
        with self._folder:
            # ? The style sits in the folder as an empty div; it takes no space.
            style = gui.add_html("")
            self._all = gui.add_checkbox("Select all", initial_value=False)
            self._empty = gui.add_html(note(empty_text))
        style.content = _tight_style(self._all._impl.uuid)
        #: Item checkboxes, in the order they were added.
        self._boxes: dict[Hashable, viser.GuiCheckboxHandle] = {}
        self._all.on_update(self._on_select_all)

    def add(self, key: Hashable, label: str, hint: str = "", selected: bool = False) -> None:
        """Append an item at the bottom of the list. Call on the main thread.

        Args:
            key: Unique item key.
            label: Checkbox text.
            hint: Hover text, e.g. the item's numbers.
            selected: Whether it starts ticked.

        Raises:
            ValueError: If `key` is already listed.
        """
        if key in self._boxes:
            raise ValueError(f"{key!r} is already listed")
        with self._folder:
            self._boxes[key] = self._gui.add_checkbox(label, initial_value=selected, hint=hint or None)

    def remove(self, key: Hashable) -> None:
        """Remove an item; unknown keys are ignored. Call on the main thread."""
        box = self._boxes.pop(key, None)
        if box is not None:
            box.remove()

    def selected(self) -> list[Hashable]:
        """The ticked items' keys, in list order."""
        return [key for key, box in self._boxes.items() if box.value]

    def is_selected(self, key: Hashable) -> bool:
        """Whether `key` is listed and ticked."""
        box = self._boxes.get(key)
        return box is not None and box.value

    def sync(self) -> None:
        """Tick "Select all" exactly when every item is ticked, and show the note when empty. Call from `draw`."""
        # * Assigning an unchanged value sends nothing and runs no callback.
        self._all.value = bool(self._boxes) and all(box.value for box in self._boxes.values())
        self._empty.visible = not self._boxes

    async def _on_select_all(self, event: viser.GuiEvent) -> None:
        """Tick or untick every item when the operator clicks "Select all". Runs on a viser thread."""
        if event.client is None:
            return  # our own assignment in `sync`, not a click
        value = bool(event.target.value)
        self._ctx.submit("select all" if value else "select none", lambda: self._set_all(value))

    def _set_all(self, value: bool) -> None:
        """Set every item checkbox to `value`. Runs as an intent."""
        for box in self._boxes.values():
            box.value = value


def _tight_style(select_all_uuid: str) -> str:
    """CSS that packs the rows of one CheckList: no gap between rows, box before a full-width label.

    ? viser has no list styling, so this targets its DOM (viser 1.1): the folder's
      container holds one div per row, `row > flex > label box > p > label[for=<uuid>]`.
      The container is found through the "Select all" label, so no other folder changes.
      If a viser update breaks this, the list only goes back to normal spacing.

    Args:
        select_all_uuid: The "Select all" checkbox's uuid.

    Returns:
        str: A <style> element, for an html widget.
    """
    rows = f'div:has(> div > div > div > p > label[for="{select_all_uuid}"])'
    # ! viser sets these paddings and widths inline, so only !important overrides them.
    return (f"<style>"
            f"{rows} > div {{ padding-bottom: 0 !important; }}"
            f"{rows} > div > div {{ flex-direction: row-reverse; justify-content: flex-end; column-gap: 0.5em; }}"
            f"{rows} > div > div > div {{ width: auto !important; flex-grow: 0 !important; "
            f"padding-right: 0 !important; }}"
            f"</style>")
