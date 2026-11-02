"""
A person's layout file: where tabs and windows go instead of the code's, and a few other matters of taste.

Placements cover folders of plugin tabs, plugins' tabs and plugins' own windows; preferences the rest, each plugin's
in its own section, which only that plugin reads.

Read once at start from `~/.config/husky_monitor/layout.json`, or the file the monitor's `layout` parameter names. No
file means the code's layout: plugins' tabs in the docked side panel at their `tab_path`, hidden ones in the strip.
A layout is a person's preference, so it is never in git. Each entry that cannot be used is reported and skipped.

    {"windows": {"Plan": {"float": [410, 15], "width": 420}},
     "plugins": {"obstacles": {"hidden": false, "tab_path": ["Tools", "Obstacles"]},
                 "console": {"preferences": {"hidden": ["runtime"]}}},
     "panels": {"Exec plots": {"dock": "right"}},
     "preferences": {"ghost_timeout": 0}}

- `windows`: a top-level folder of tabs gets a panel of its own; every tab under it goes there.
- `plugins`: a plugin's `tab_path` and `hidden`, instead of its class's, and its `preferences`: a JSON object handed to
  the plugin as `ctx.view.preferences`, which the plugin documents and checks.
- `panels`: a window a plugin made itself in `setup` (`PluginView.panel`), by its first tab's label.
- `preferences`: the monitor's own: seconds ghosts stay after the last input (0: until another plugin's show).
- A placement: `"float": [x, y]` (pixels; negative counts from the right or bottom), `"dock": "right" | "left"`,
  `"dock_below"` or `"dock_above"`: a folder in `windows`, or "side" for the side panel; with optional `"width"`,
  `"height"` (pixels) and `"minimized"`.
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from types import MappingProxyType
from typing import Any, Callable, Mapping

import viser

#: The side panel's name in a placement's `dock_below` / `dock_above`.
SIDE = "side"


def default_layout_path() -> Path:
    """The person's layout file: `$XDG_CONFIG_HOME/husky_monitor/layout.json`, by default under ~/.config."""
    config_home = os.environ.get("XDG_CONFIG_HOME") or str(Path.home() / ".config")
    return Path(config_home) / "husky_monitor" / "layout.json"


@dataclass(frozen=True)
class Placement:
    """Where a panel goes: floating at `at`, docked to `edge`, or docked below or above the panel `anchor` names.

    Attributes:
        at: (x, y) pixels for a floating panel, else None.
        edge: "right" or "left" for a docked one, else None.
        anchor: The folder in `windows`, or SIDE, it docks against; with `below`.
        below: Below the anchor, else above it.
        width: Width in pixels, or None to keep it.
        height: Height in pixels of a floating panel, or None to keep it.
        minimized: Whether it starts minimized.
    """

    at: tuple[float, float] | None = None
    edge: str | None = None
    anchor: str | None = None
    below: bool = True
    width: float | None = None
    height: float | None = None
    minimized: bool = False

    def apply(self, panel: viser.PanelHandle, anchor: viser.PanelHandle | None = None) -> None:
        """Place `panel` here; `anchor` is the panel `self.anchor` names, when it docks against one."""
        if self.at is not None:
            panel.float(x=self.at[0], y=self.at[1], width=self.width, height=self.height)
        else:
            if anchor is not None:
                (panel.dock_below if self.below else panel.dock_above)(anchor)
            elif self.edge == "left":
                panel.dock_left()
            else:
                panel.dock_right()
            if self.width is not None:
                panel.set_width(self.width)
        if self.minimized:
            panel.minimize()


@dataclass(frozen=True)
class PluginPlace:
    """A plugin's tab, instead of its class's: its `tab_path` and whether it starts in the strip; None keeps it.

    Also its `preferences`, the JSON object only that plugin reads (`ctx.view.preferences`).
    """

    tab_path: tuple[str, ...] | None = None
    hidden: bool | None = None
    preferences: Mapping[str, Any] = field(default_factory=lambda: MappingProxyType({}))


@dataclass(frozen=True)
class Preferences:
    """The monitor's own matters of taste; None keeps the code's.

    Attributes:
        ghost_timeout: Seconds a plugin's ghosts stay after the last input in it; 0 until another plugin's show.
    """

    ghost_timeout: float | None = None


@dataclass(frozen=True)
class Layout:
    """A person's layout; empty is the code's layout.

    Attributes:
        windows: Top-level folders with a panel of their own, by label, in the file's order.
        plugins: Plugins' tabs instead of their classes', by plugin name.
        panels: Placements of plugins' own windows, by their first tab's label.
        preferences: The monitor's own matters of taste; each plugin's are in `plugins`.
        source: The file read, or None.
    """

    windows: dict[str, Placement] = field(default_factory=dict)
    plugins: dict[str, PluginPlace] = field(default_factory=dict)
    panels: dict[str, Placement] = field(default_factory=dict)
    preferences: Preferences = Preferences()
    source: Path | None = None

    def plugin(self, name: str, hidden: bool, tab_path: tuple[str, ...]) -> tuple[bool, tuple[str, ...]]:
        """A plugin's `hidden` and `tab_path`, as its class gives them unless this layout moves it."""
        place = self.plugins.get(name)
        if place is None:
            return hidden, tab_path
        return (hidden if place.hidden is None else place.hidden,
                tab_path if place.tab_path is None else place.tab_path)

    def plugin_preferences(self, name: str) -> Mapping[str, Any]:
        """A plugin's own preferences from the file, unchecked (the plugin checks them); empty without any."""
        place = self.plugins.get(name)
        return place.preferences if place is not None else MappingProxyType({})


def read_layout(path: Path, warn: Callable[[str], None]) -> Layout:
    """Read a layout file; no file is the empty layout. Each entry that cannot be used is reported to `warn`, skipped.

    Args:
        path: The file.
        warn: Gets one line per problem.
    """
    if not path.is_file():
        return Layout()
    try:
        data = json.loads(path.read_text())
    except (OSError, ValueError) as error:
        warn(f"layout {path}: not read ({error}); the code's layout is used")
        return Layout()
    if not isinstance(data, dict):
        warn(f"layout {path}: not a JSON object; the code's layout is used")
        return Layout()
    for key in sorted(set(data) - {"windows", "plugins", "panels", "preferences"}):
        warn(f"layout {path}: {key!r} is no section (windows, plugins, panels, preferences); skipped")
    windows = _placements(data.get("windows", {}), f"layout {path} windows", warn)
    for label, placement in list(windows.items()):
        if placement.anchor is not None and placement.anchor != SIDE and placement.anchor not in windows:
            warn(f"layout {path} windows {label!r}: docks against {placement.anchor!r}, no folder in windows; "
                 f"docked right instead")
            windows[label] = Placement(edge="right", width=placement.width, minimized=placement.minimized)
    panels = _placements(data.get("panels", {}), f"layout {path} panels", warn)
    for label, placement in list(panels.items()):
        if placement.anchor is not None:
            warn(f"layout {path} panels {label!r}: a plugin's window floats or docks to an edge only; skipped")
            del panels[label]
    return Layout(windows, _plugins(data.get("plugins", {}), f"layout {path} plugins", warn), panels,
                  _preferences(data.get("preferences", {}), f"layout {path} preferences", warn), path)


def _placements(section: Any, where: str, warn: Callable[[str], None]) -> dict[str, Placement]:
    """The placements of a section, by label; a bad one reported and skipped."""
    if not isinstance(section, dict):
        warn(f"{where}: not a JSON object; skipped")
        return {}
    placements = {}
    for label, entry in section.items():
        try:
            placements[str(label)] = _placement(entry)
        except ValueError as error:
            warn(f"{where} {label!r}: {error}; skipped")
    return placements


def _placement(entry: Any) -> Placement:
    """One placement from its JSON object.

    Raises:
        ValueError: If it is not exactly one valid position with valid sizes.
    """
    if not isinstance(entry, dict):
        raise ValueError("not a JSON object")
    unknown = set(entry) - {"float", "dock", "dock_below", "dock_above", "width", "height", "minimized"}
    if unknown:
        raise ValueError(f"unknown keys {sorted(unknown)}")
    positions = [key for key in ("float", "dock", "dock_below", "dock_above") if key in entry]
    if len(positions) != 1:
        raise ValueError("needs exactly one of float, dock, dock_below, dock_above")
    sizes = {key: entry.get(key) for key in ("width", "height")}
    for key, value in sizes.items():
        if value is not None and (not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0):
            raise ValueError(f"{key} must be a positive number of pixels")
    minimized = entry.get("minimized", False)
    if not isinstance(minimized, bool):
        raise ValueError("minimized must be true or false")
    common = dict(width=sizes["width"], height=sizes["height"], minimized=minimized)
    position, value = positions[0], entry[positions[0]]
    if position == "float":
        if not (isinstance(value, list) and len(value) == 2
                and all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in value)):
            raise ValueError("float must be [x, y] pixels")
        return Placement(at=(float(value[0]), float(value[1])), **common)
    if position == "dock":
        if value not in ("right", "left"):
            raise ValueError('dock must be "right" or "left"')
        return Placement(edge=value, **common)
    if not isinstance(value, str) or not value:
        raise ValueError(f"{position} must name a folder in windows, or {SIDE!r}")
    return Placement(anchor=value, below=position == "dock_below", **common)


def _plugins(section: Any, where: str, warn: Callable[[str], None]) -> dict[str, PluginPlace]:
    """The plugins' places, by plugin name; a bad one reported and skipped."""
    if not isinstance(section, dict):
        warn(f"{where}: not a JSON object; skipped")
        return {}
    places = {}
    for name, entry in section.items():
        if not isinstance(entry, dict) or not entry or set(entry) - {"tab_path", "hidden", "preferences"}:
            warn(f"{where} {name!r}: needs tab_path, hidden and/or preferences, nothing else; skipped")
            continue
        tab_path, hidden, preferences = entry.get("tab_path"), entry.get("hidden"), entry.get("preferences", {})
        if tab_path is not None and not (isinstance(tab_path, list) and tab_path
                                         and all(isinstance(label, str) and label for label in tab_path)):
            warn(f"{where} {name!r}: tab_path must be a list of labels; skipped")
            continue
        if hidden is not None and not isinstance(hidden, bool):
            warn(f"{where} {name!r}: hidden must be true or false; skipped")
            continue
        if not isinstance(preferences, dict):
            warn(f"{where} {name!r}: preferences must be a JSON object; skipped")
            continue
        places[str(name)] = PluginPlace(tuple(tab_path) if tab_path is not None else None, hidden,
                                        MappingProxyType(dict(preferences)))
    return places


def _preferences(section: Any, where: str, warn: Callable[[str], None]) -> Preferences:
    """The preferences of a section; a bad entry reported and left to the code."""
    if not isinstance(section, dict):
        warn(f"{where}: not a JSON object; skipped")
        return Preferences()
    for key in sorted(set(section) - {"ghost_timeout"}):
        warn(f"{where}: unknown {key!r} (a plugin's go in plugins.<name>.preferences); skipped")
    timeout = section.get("ghost_timeout")
    if timeout is not None and (not isinstance(timeout, (int, float)) or isinstance(timeout, bool) or timeout < 0):
        warn(f"{where}: ghost_timeout must be seconds, 0 or more; skipped")
        timeout = None
    return Preferences(float(timeout) if timeout is not None else None)
