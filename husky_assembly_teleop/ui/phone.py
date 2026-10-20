"""
The phone page: Stop all, a header any plugin may add to, then one tab per plugin (`PluginView.phone`).

A second viser server, so the phone gets only panel text and button events, never the 3D scene. Its CSS turns
viser's phone layout (a bottom sheet, shown only below 576 px width, so a phone in portrait) into a full-screen
panel with big buttons; it finds the sheet by an empty element with the id MARKER.

! Callbacks on this page run on viser's threads, as on the desktop: only hand work to `ctx.defer`.
"""

from __future__ import annotations

from typing import Callable

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


def columns_css(icon: str, columns: int) -> str:
    """CSS that lays out, row by row in `columns` columns, the buttons of every folder holding a button with `icon`.

    ! Keep only buttons in that folder: any other widget takes a cell too.

    Args:
        icon: An icon of one of the folder's buttons, e.g. viser.Icon.MINUS.
        columns: How many columns.

    Returns:
        str: CSS rules, without the <style> element.
    """
    return (f"div:has(> div > button .icon-tabler-{icon}) "
            f"{{ display: grid; grid-template-columns: repeat({columns}, minmax(0, 1fr)); column-gap: 8px; }}")


def big_buttons(icons: list[str], height: int, icon_size: int, labels: bool = True) -> str:
    """CSS that makes the phone page's buttons with these icons `height` px high, with `icon_size` px icons.

    Args:
        icons: The buttons' icons, e.g. viser.Icon.HAND_STOP.
        height: Button height, px.
        icon_size: Icon size, px.
        labels: False for icon-only buttons: drops the empty label so the icon is centred.

    Returns:
        str: CSS rules, without the <style> element.
    """
    def each(suffix: str = "") -> str:
        return ", ".join(f"{SHEET} button:has(.icon-tabler-{icon}){suffix}" for icon in icons)

    # * viser draws an icon in a 1em box, so the icon grows with its section's font size.
    css = (f"{each()} {{ height: {height}px !important; font-size: 18px !important; }}"
           f"{each(' .mantine-Button-section')} {{ font-size: {icon_size}px; }}")
    if not labels:
        css += (f"{each(' .mantine-Button-label')} {{ display: none; }}"
                f"{each(' .mantine-Button-section')} {{ margin: 0 !important; }}")
    return css


#: The phone page's CSS, in order: no 3D canvas; the sheet fills the screen; a long press must not select text, open
#: the browser menu or scroll (a scroll cancels a hold); finger-sized buttons; icons beside their label; chips
#: readable at arm's length (`phone_text`); a big Stop all.
PHONE_STYLE = f"""
canvas {{ display: none; }}
{SHEET} {{ top: 0 !important; left: 0 !important; width: 100% !important; max-height: 100% !important;
           user-select: none; -webkit-user-select: none; -webkit-touch-callout: none; }}
{SHEET} div[style*="width: 20em"] {{ width: 100% !important; }}
{SHEET} button {{ touch-action: none; }}
{SHEET} .mantine-Button-root {{ height: 44px !important; }}
{SHEET} .mantine-Button-section svg {{ position: static !important; transform: none !important; }}
{SHEET} .phone-text span, {SHEET} .phone-text div {{ font-size: 14px !important; }}
{SHEET} .phone-text span[style*="width:42px"] {{ width: auto !important; min-width: 44px; margin-right: 6px; }}
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
