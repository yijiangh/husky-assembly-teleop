"""
One widget shown on the laptop page and on the phone page: setting an attribute writes both, only where it changed.

Wire each page's callbacks on its own handle; `BothPages` only carries what is drawn (content, value, visible, label).
"""

from __future__ import annotations

from typing import Any, Callable


class BothPages:
    """The same widget on both pages; reads give the laptop page's value.

    Args:
        handle: The laptop page's handle.
    """

    def __init__(self, handle: Any):
        """Only the laptop page's handle so far (`on_both` adds the phone's)."""
        object.__setattr__(self, "_pages", [(handle, None)])

    def add(self, handle: Any, transform: Callable[[str], str] | None = None) -> None:
        """Add another page's handle; `transform` rewrites `content` for it, e.g. `ui.phone.phone_text`."""
        self._pages.append((handle, transform))

    @property
    def handles(self) -> list[Any]:
        """list: Each page's handle, the laptop's first."""
        return [handle for handle, _ in self._pages]

    def __getattr__(self, name: str) -> Any:
        """The laptop page's value."""
        return getattr(self._pages[0][0], name)

    def __setattr__(self, name: str, value: Any) -> None:
        """Write every page's handle where it differs."""
        # * Only changed values: every assignment is a message to each browser.
        for handle, transform in self._pages:
            page_value = transform(value) if transform is not None and name == "content" else value
            if getattr(handle, name) != page_value:
                setattr(handle, name, page_value)


def on_both(existing: Any, handle: Any, transform: Callable[[str], str] | None = None) -> BothPages:
    """The laptop page's `existing` handle (or a `BothPages`) with the phone page's `handle` added."""
    both = existing if isinstance(existing, BothPages) else BothPages(existing)
    both.add(handle, transform)
    return both
