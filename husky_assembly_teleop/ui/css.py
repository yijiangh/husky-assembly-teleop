"""
CSS for viser widgets that viser offers no option for, on the laptop's page: buttons side by side, button groups.

! Each finds a widget by its uuid (`handle._impl.uuid`) in viser 1.1.1's markup; if viser's markup changes, the rules
  stop matching and the widgets fall back to viser's own layout.
"""

from __future__ import annotations

from typing import Sequence

from ..colors import INK, WHITE


def side_by_side_css(uuids: Sequence[str]) -> str:
    """CSS that puts these buttons on one row, side by side; add them one after the other, nothing between them.

    ? viser 1.1.1 draws an unlabelled folder (`add_folder(None)`) without an element of its own, so `columns_css` would
      lay out the whole container around it: this styles each button instead, found by its id (the button's uuid).

    Args:
        uuids: The buttons' uuids, `handle._impl.uuid`, left to right.

    Returns:
        str: CSS rules, without the <style> element.
    """
    # * Each button's box keeps viser's side margins (Mantine's "xs" spacing).
    width = f"calc({100 / len(uuids):g}% - 2 * var(--mantine-spacing-xs))"
    return "".join(f'div:has(> button[id="{uuid}"]) {{ display: inline-block; width: {width}; vertical-align: top; }}'
                   for uuid in uuids)


def group_buttons(uuid: str, index: int | None = None) -> str:
    """The CSS selector of a button group's buttons (`add_button_group`), or of its `index`-th button only.

    ? viser 1.1.1's button groups take no colour or size of their own. The group is found by its label; its buttons
      are the children of one flex row.

    Args:
        uuid: The button group's uuid, `handle._impl.uuid`.
        index: Which button, from 0, left to right; None for every one.

    Returns:
        str: A CSS selector.
    """
    buttons = f'div:has(> div > p > label[for="{uuid}"]) button'
    return buttons if index is None else f"{buttons}:nth-child({index + 1})"


def one_row_css(uuid: str) -> str:
    """CSS that keeps a button group's buttons on one row: a row too wide for the panel scrolls sideways, without a bar.

    * viser wraps them onto more rows instead, and the rows below then move.

    Args:
        uuid: The button group's uuid, `handle._impl.uuid`.

    Returns:
        str: CSS rules, without the <style> element.
    """
    group = f'div:has(> div > p > label[for="{uuid}"])'
    # ? The input's box (beside the label's) may then shrink below its buttons' width, so their flex row scrolls.
    row = f"{group} div:has(> button)"
    return (f"{group} > div:last-child {{ min-width: 0; }}"
            f"{row} {{ flex-wrap: nowrap !important; overflow-x: auto; scrollbar-width: none; }}"
            f"{row}::-webkit-scrollbar {{ display: none; }}")


def css_string(text: str) -> str:
    """`text` as a quoted CSS string, e.g. for `content:`; a newline becomes a line break."""
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"').replace("\n", "\\A ") + '"'


def group_tooltips_css(uuid: str, tips: Sequence[str | None]) -> str:
    """CSS that shows a tooltip under a button group while one of its buttons is hovered: `tips[i]` for the i-th.

    The box hangs off the group's row, the full row wide, since a box inside the scrolling button row would be cut
    off; viser's own `hint` covers the whole group only, so leave it out when the buttons have tips.

    Args:
        uuid: The button group's uuid, `handle._impl.uuid`.
        tips: One text per button, left to right, a newline in it for a line break; None for a button without one.

    Returns:
        str: CSS rules, without the <style> element.
    """
    group = f'div:has(> div > p > label[for="{uuid}"])'
    css = (f"{group} {{ position: relative; }}"
           f"{group}::after {{ display: none; position: absolute; left: 0; right: 0; top: 100%; z-index: 20; "
           f"padding: 6px 10px; border-radius: 4px; background: {INK}; color: {WHITE}; font-size: 12px; "
           "line-height: 1.35; white-space: pre-wrap; pointer-events: none; }")
    for index, tip in enumerate(tips):
        if tip:
            css += (f"{group}:has(button:nth-child({index + 1}):hover)::after {{ display: block; "
                    f"content: {css_string(tip)}; }}")
    return css


def button_group_color_css(uuid: str, color: str) -> str:
    """CSS that draws a button group's buttons in `color`, outline and label, instead of viser's blue.

    Args:
        uuid: The button group's uuid, `handle._impl.uuid`.
        color: A CSS colour, e.g. `colors.TEAL`.

    Returns:
        str: CSS rules, without the <style> element.
    """
    return f"{group_buttons(uuid)} {{ color: {color} !important; border-color: {color} !important; }}"
