"""
The phone page: Stop all, a header any plugin may add to, then one tab per plugin (`PluginView.phone`).

A second viser server, so the phone gets only panel text and button events, never the 3D scene. Its CSS turns
viser's phone layout (a bottom sheet, shown only below 576 px width, so a phone in portrait) into a full-screen
panel with big buttons; it finds the sheet by an empty element with the id MARKER.

! Callbacks on this page run on viser's threads, as on the desktop: only hand work to `ctx.defer`.
"""

from __future__ import annotations

from typing import Callable, Iterable, Sequence

import viser

#: Shown in the phone's browser tab.
PHONE_TITLE = "Husky Phone"

#: Id of an empty element on the phone page; the CSS finds viser's bottom sheet by it.
MARKER = "husky-phone"
#: The bottom sheet holding MARKER.
#: ! `:has(#id)` counts as an id selector: prefix specific rules with SHEET, or the sheet-wide rules below win.
SHEET = f".mantine-Paper-root:has(#{MARKER})"


def grid_css(cells: tuple[tuple[str, int, int], ...], columns: int) -> str:
    """CSS that lays out the buttons of every folder holding all of `cells` as a grid.

    ? viser has no grid container; buttons are found by their icon class (`icon-tabler-<name>`).
    ! Use each icon in one grid only: its cell rule applies wherever that icon is.

    Args:
        cells: (icon, row, column) per button, 1-based.
        columns: How many columns the grid has.

    Returns:
        str: CSS rules, without the <style> element.
    """
    def button(icon: str) -> str:
        return f"button .icon-tabler-{icon}"

    folder = "div" + "".join(f":has(> div > {button(icon)})" for icon, _, _ in cells)
    # * minmax(0, 1fr) and slim padding let the columns shrink below the buttons' natural width on a phone.
    grid = f"{folder} {{ display: grid; grid-template-columns: repeat({columns}, minmax(0, 1fr)); }}"
    # ! Give every button an explicit cell, or a later button fills an empty cell before it.
    places = "".join(f"div:has(> {button(icon)}) {{ grid-area: {row} / {column}; }}"
                     f"div:has(> {button(icon)}) > button {{ padding: 0 4px; }}" for icon, row, column in cells)
    return grid + places


def columns_css(icon: str, columns: int, wrap: bool = False) -> str:
    """CSS that lays out, row by row in `columns` columns, the buttons of every folder holding a button with `icon`.

    ! Keep only buttons in that folder: any other widget takes a cell too. And give the folder a label: viser 1.1.1
      draws an unlabelled one without an element of its own, so the grid would take its container (`side_by_side_css`).

    Args:
        icon: An icon of one of the folder's buttons, e.g. viser.Icon.MINUS.
        columns: How many columns.
        wrap: Let a long label wrap onto more lines, the button growing taller, instead of being cut off.

    Returns:
        str: CSS rules, without the <style> element.
    """
    folder = f"div:has(> div > button .icon-tabler-{icon})"
    css = f"{folder} {{ display: grid; grid-template-columns: repeat({columns}, minmax(0, 1fr)); column-gap: 8px; }}"
    if wrap:
        # ? viser's buttons are Mantine buttons: a fixed height, and a label that never wraps.
        css += (f"{folder} button {{ height: auto !important; min-height: var(--button-height); padding-top: 4px; "
                f"padding-bottom: 4px; }}"
                f"{folder} button .mantine-Button-inner {{ height: auto; }}"
                f"{folder} button .mantine-Button-label {{ white-space: normal !important; overflow: visible; "
                f"line-height: 1.25; text-align: center; }}")
    return css


def big_buttons(icons: list[str], height: int, icon_size: int, labels: bool = True, scope: str = SHEET) -> str:
    """CSS that makes the buttons with these icons `height` px high, with `icon_size` px icons.

    Args:
        icons: The buttons' icons, e.g. viser.Icon.HAND_STOP.
        height: Button height, px.
        icon_size: Icon size, px.
        labels: False for icon-only buttons: drops the empty label so the icon is centred.
        scope: A selector the buttons are inside: the phone page's sheet by default, "" for the whole page.

    Returns:
        str: CSS rules, without the <style> element.
    """
    inside = f"{scope} " if scope else ""

    def each(suffix: str = "") -> str:
        return ", ".join(f"{inside}button:has(.icon-tabler-{icon}){suffix}" for icon in icons)

    # * viser draws an icon in a 1em box, so the icon grows with its section's font size.
    css = (f"{each()} {{ height: {height}px !important; font-size: 18px !important; }}"
           f"{each(' .mantine-Button-section')} {{ font-size: {icon_size}px; }}")
    if not labels:
        css += (f"{each(' .mantine-Button-label')} {{ display: none; }}"
                f"{each(' .mantine-Button-section')} {{ margin: 0 !important; }}")
    return css


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


def slim_labels_css(uuids: Iterable[str], width_em: float = 2.5) -> str:
    """CSS that shrinks the label box of these inputs to `width_em`, so a slider with a short label takes the row.

    ? viser gives every labelled input a fixed 7.25em label box. This relies on viser 1.1.1's markup,
      `div(label box) > p > label[for=<uuid>]` (client `components/common.tsx`); if it changes, the box is wide again.

    Args:
        uuids: The inputs' uuids, `handle._impl.uuid`.
        width_em: The label box's width, em.

    Returns:
        str: CSS rules, without the <style> element.
    """
    return "".join(f'div:has(> p > label[for="{uuid}"]) {{ width: {width_em}em !important; }}' for uuid in uuids)


def group_buttons(uuid: str, index: int | None = None) -> str:
    """The CSS selector of a button group's buttons (`add_button_group`), or of its `index`-th button only.

    ? viser 1.1.1's button groups take no colour or size of their own. The group is found by its label, as in
      `slim_labels_css`; its buttons are the children of one flex row.

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
           "padding: 6px 10px; border-radius: 4px; background: #212529; color: #fff; font-size: 12px; "
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
        color: A CSS colour, e.g. `ui.style.SECTION_TOOL`.

    Returns:
        str: CSS rules, without the <style> element.
    """
    return f"{group_buttons(uuid)} {{ color: {color} !important; border-color: {color} !important; }}"


#: The phone page's CSS, in order: no 3D canvas; the sheet fills the screen; a long press must not select text, open
#: the browser menu or scroll (a scroll cancels a hold); finger-sized buttons; icons beside their label; chips
#: readable at arm's length (`phone_text`), with chip lines high enough for them; readout lines (`style.values`) too
#: wide for the phone scroll sideways, as chip lines do, rather than being cut off; a big Stop all.
PHONE_STYLE = f"""
canvas {{ display: none; }}
{SHEET} {{ top: 0 !important; left: 0 !important; width: 100% !important; max-height: 100% !important;
           user-select: none; -webkit-user-select: none; -webkit-touch-callout: none; }}
{SHEET} div[style*="width: 20em"] {{ width: 100% !important; }}
{SHEET} button {{ touch-action: none; }}
{SHEET} .mantine-Button-root {{ height: 44px !important; }}
{SHEET} .mantine-Button-section svg {{ position: static !important; transform: none !important; }}
{SHEET} .phone-text span, {SHEET} .phone-text div {{ font-size: 14px !important; }}
{SHEET} .chip-line {{ min-height: 24px; }}
{SHEET} .phone-text span[style*="width:42px"] {{ width: auto !important; min-width: 44px; margin-right: 6px; }}
{SHEET} .phone-text .values {{ overflow-x: auto; scrollbar-width: none; }}
{SHEET} .phone-text .values::-webkit-scrollbar {{ display: none; }}
{big_buttons([viser.Icon.HAND_STOP], 72, 30)}
"""


def phone_text(html: str) -> str:
    """Wrap panel HTML (chips, section bars) so it is readable at arm's length on the phone."""
    return f'<div class="phone-text">{html}</div>'


class PhonePage:
    """The phone page's fixed part: Stop all, the header folder and the tab group plugins add to."""

    def __init__(self, server: viser.ViserServer, on_stop: Callable[[viser.GuiEvent], None]):
        """Build the page.

        Args:
            server: The phone's viser server.
            on_stop: Called by Stop all, on a viser thread.
        """
        self.server = server
        gui = server.gui
        gui.configure_theme(show_logo=False, show_share_button=False)
        gui.add_html(f'<title>{PHONE_TITLE}</title><div id="{MARKER}"></div><style>{PHONE_STYLE}</style>')
        self.stop = gui.add_button("Stop all", color="red", icon=viser.Icon.HAND_STOP,
                                   hint="Soft stop: stop every arm's program, switch every base controller off, "
                                        "stop every tool, cancel every plugin task. Not an emergency stop.")
        self.stop.on_click(on_stop)
        #: Each plugin's header folder goes in here, above the tabs. Unlabelled folders draw only their children.
        self.header = gui.add_folder(None)
        #: One tab per plugin, added by `PluginView.phone`.
        self.tabs = gui.add_tab_group()
