"""A checklist the operator ticks before a step's actions may run: a checkbox per item, alike on every page."""

from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Sequence

import viser

from ...ui.style import SECTION_CTRL, section

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext


class Checklist:
    """Checkboxes in a step's folder, on the laptop page and maybe the phone's; the step judges what they mean.

    Attributes:
        items: Each box's label and hint.
        boxes: The laptop page's checkboxes, once built; every other page's follow them.
    """

    def __init__(self, items: Sequence[tuple[str, str]]):
        """Not built yet."""
        self.items = tuple(items)
        self.boxes: list[viser.GuiCheckboxHandle] = []
        self._pages: list[list[viser.GuiCheckboxHandle]] = []

    def build(self, ctx: PluginContext, gui: viser.GuiApi, title: str, on_tick: Callable[[str, bool], None]) -> None:
        """Add a section headed `title` and the boxes, unticked; once per page, the laptop's first.

        An operator's tick sets the same box on every page, then calls `on_tick(label, ticked)`.
        """
        gui.add_html(section(title, SECTION_CTRL))
        boxes = [gui.add_checkbox(text, initial_value=False, hint=hint) for text, hint in self.items]
        self._pages.append(boxes)
        if not self.boxes:
            self.boxes = boxes
        for index, box in enumerate(boxes):
            box.on_update(ctx.defer_value(f"tick {box.label}", lambda ticked, index=index: self._ticked(
                index, bool(ticked), on_tick), operator_only=True))

    def _ticked(self, index: int, ticked: bool, on_tick: Callable[[str, bool], None]) -> None:
        """Set the box on every page, then call `on_tick`."""
        for boxes in self._pages:
            if boxes[index].value != ticked:
                boxes[index].value = ticked
        on_tick(self.items[index][0], ticked)

    def tick(self, labels: Sequence[str], ticked: bool) -> None:
        """Tick or untick the named boxes on every page, e.g. to show ticks kept from before."""
        for boxes in self._pages:
            for box in boxes:
                if box.label in labels and box.value != ticked:
                    box.value = ticked
