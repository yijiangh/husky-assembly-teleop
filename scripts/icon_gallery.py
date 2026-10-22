"""
Every icon viser ships, drawn as viser draws it, for picking a plugin's `icon`: search, then click one to copy it.

Icons this repo already uses are marked: blue for a plugin's own icon (keep those unique), grey for other uses;
hover one to see where. viser bundles an older Tabler set than tabler.io shows, so icons there can look different
or be missing here. Run with the venv sourced, then open http://localhost:8160:

    python3 src/husky-assembly-teleop/scripts/icon_gallery.py [--port 8160]
"""

from __future__ import annotations

import argparse
import re
import time
import zipfile
from collections import defaultdict
from html import escape
from pathlib import Path

import viser
# ? Private: viser has no public list of its icons' drawings.
from viser._icons import ICONS_DIR

#: The package whose icon uses are marked.
PACKAGE = Path(__file__).resolve().parents[1] / "husky_assembly_teleop"
#: Icons per page; more makes typing in the search field lag.
PAGE = 300
SHOW = {"all": "All icons", "used": "Used here", "plugin": "Plugin icons"}
PLUGIN_COLOR, USED_COLOR = "#228be6", "#868e96"
#: Copies the clicked cell's data-copy and says so in the cell; falls back to execCommand off localhost.
COPY_JS = ("(c=>{const s=c.dataset.copy;(navigator.clipboard?navigator.clipboard.writeText(s):Promise.reject())"
           ".catch(()=>{const a=document.createElement('textarea');a.value=s;document.body.appendChild(a);"
           "a.select();document.execCommand('copy');a.remove();});const n=c.querySelector('.copied');"
           "n.textContent='copied';setTimeout(()=>n.textContent='',1200);})(this)")


def load_icons() -> dict[str, str]:
    """Each bundled icon's SVG by its code name, e.g. "ARROW_UP"."""
    with zipfile.ZipFile(ICONS_DIR / "tabler-icons.zip") as icons:
        return {name.removesuffix(".svg").upper().replace("-", "_"): icons.read(name).decode()
                for name in sorted(icons.namelist())}


def find_uses(package: Path) -> tuple[dict[str, set[str]], dict[str, str]]:
    """Where the package uses each icon, and which plugin each plugin icon belongs to; the old monitor left out.

    Returns:
        tuple[dict[str, set[str]], dict[str, str]]: Files using each icon, short; plugin name by plugin icon.
    """
    uses: dict[str, set[str]] = defaultdict(set)
    plugins: dict[str, str] = {}
    for path in sorted(package.rglob("*.py")):
        if "old" in path.relative_to(package).parts:
            continue
        text = path.read_text()
        short = str(path.relative_to(package).with_suffix(""))
        for name in re.findall(r"Icon\.([A-Z0-9_]+)", text):
            uses[name].add(short)
        plugin = re.search(r'^    name = "([a-z0-9_]+)"', text, re.M)
        for name in re.findall(r"^    icon = viser\.Icon\.([A-Z0-9_]+)", text, re.M):
            plugins[name] = plugin.group(1) if plugin else short
    return uses, plugins


def rank(name: str, words: list[str]) -> tuple[int, str]:
    """Exact names first, then names starting with the first word, then the rest; alphabetical within each."""
    if not words:
        return 2, name
    return (0 if name == "_".join(words) else 1 if name.startswith(words[0]) else 2), name


def cell(name: str, svg: str, size: int, uses: set[str], plugin: str | None) -> str:
    """One icon above its name, outlined if used here; click copies `viser.Icon.<name>`."""
    svg = svg.replace('width="24" height="24"', f'width="{size}" height="{size}"')
    color = PLUGIN_COLOR if plugin else USED_COLOR if uses else "rgba(128,128,128,.3)"
    note = f"plugin {plugin}" if plugin else f"used in {len(uses)} file{'s' * (len(uses) != 1)}" if uses else ""
    tip = f"viser.Icon.{name}" + "".join(f"\n{use}" for use in sorted(uses))
    return (f'<div data-copy="viser.Icon.{name}" title="{escape(tip)}" onclick="{COPY_JS}" '
            f'style="display:flex;flex-direction:column;align-items:center;width:{max(size * 3, 96)}px;padding:6px;'
            f'border:{2 if uses else 1}px solid {color};border-radius:6px;font-size:10px;text-align:center;'
            f'cursor:pointer">{svg}<b style="margin-top:4px;word-break:break-all">{name}</b>'
            f'<span style="color:{color}">{note}</span><span class="copied" style="color:#40c057"></span></div>')


def main() -> None:
    """Serve the gallery until stopped with Ctrl+C."""
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", type=int, default=8160)
    port = parser.parse_args().port
    svgs = load_icons()
    uses, plugins = find_uses(PACKAGE)

    server = viser.ViserServer(port=port, verbose=False)
    panel = server.gui.add_panel()
    with panel.add_tab(f"{len(svgs)} icons"):
        search = server.gui.add_text("Search", initial_value="", hint="Words in the name, in any order, e.g. arrow up")
        show = server.gui.add_dropdown("Show", options=list(SHOW.values()), initial_value=SHOW["all"])
        size = server.gui.add_slider("Size", min=16, max=64, step=4, initial_value=28)
        pages = server.gui.add_button_group("Page", ["Previous", "Next"])
        status = server.gui.add_html("")
        # * Its own scroll area, so the fields above stay in view.
        grid = server.gui.add_html("")
    page = 0

    def found() -> list[str]:
        words = search.value.upper().replace("-", " ").replace("_", " ").split()
        names = [name for name in svgs if all(word in name for word in words)]
        if show.value == SHOW["used"]:
            names = [name for name in names if name in uses]
        elif show.value == SHOW["plugin"]:
            names = [name for name in names if name in plugins]
        return sorted(names, key=lambda name: rank(name, words))

    def draw(new_search: bool = False) -> None:
        nonlocal page
        names = found()
        last = max(0, (len(names) - 1) // PAGE)
        page = 0 if new_search else min(page, last)
        shown = names[page * PAGE:(page + 1) * PAGE]
        status.content = (f"<p style='margin:4px 0'>{len(names)} found"
                          + (f", page {page + 1} of {last + 1}" if last else "")
                          + ". Click an icon to copy its name; hover it to see where it is used.</p>")
        grid.content = ("<div style='display:flex;flex-wrap:wrap;gap:6px;max-height:70vh;overflow-y:auto'>"
                        + "".join(cell(name, svgs[name], int(size.value), uses.get(name, set()), plugins.get(name))
                                  for name in shown) + "</div>")

    def turn(event: viser.GuiEvent) -> None:
        nonlocal page
        page += 1 if event.target.value == "Next" else -1
        page = max(page, 0)
        draw()

    for handle in (search, show):
        handle.on_update(lambda _event: draw(new_search=True))
    size.on_update(lambda _event: draw())
    pages.on_click(turn)
    draw()
    panel.float(x=10, y=10, width=1360, height=1000)
    print(f"Icon gallery on http://localhost:{port}, Ctrl+C to stop", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        server.stop()


if __name__ == "__main__":
    main()
