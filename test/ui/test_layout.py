"""The layout file: windows, plugins' tabs, plugins' own windows and preferences from JSON, bad entries skipped."""

from __future__ import annotations

import json

from husky_assembly_teleop.ui.layout import (Layout, Placement, PluginPlace, Preferences, default_layout_path,
                                             read_layout)


def read(tmp_path, data) -> tuple[Layout, list[str]]:
    """The layout of a file holding `data`, and what was reported."""
    path = tmp_path / "layout.json"
    path.write_text(data if isinstance(data, str) else json.dumps(data))
    warnings = []
    return read_layout(path, warnings.append), warnings


def test_no_file_is_the_code_layout(tmp_path):
    """A missing file: nothing overridden, nothing reported."""
    warnings = []
    assert read_layout(tmp_path / "none.json", warnings.append) == Layout() and warnings == []


def test_the_default_file_lives_in_the_users_config(monkeypatch, tmp_path):
    """$XDG_CONFIG_HOME/husky_monitor/layout.json, ~/.config without it: outside any checkout."""
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    assert default_layout_path() == tmp_path / "husky_monitor" / "layout.json"


def test_every_section_is_read(tmp_path):
    """A floating Plan window, a window docked below it, a plugin moved, a plugin's window docked left."""
    layout, warnings = read(tmp_path, {
        "windows": {"Plan": {"float": [410, 15], "width": 420},
                    "Tools": {"dock_below": "Plan", "minimized": True}},
        "plugins": {"obstacles": {"tab_path": ["Tools", "Obstacles"], "hidden": False}},
        "panels": {"Exec plots": {"dock": "left", "width": 500}}})
    assert warnings == []
    assert layout.windows == {"Plan": Placement(at=(410.0, 15.0), width=420),
                              "Tools": Placement(anchor="Plan", minimized=True)}
    assert layout.plugins == {"obstacles": PluginPlace(("Tools", "Obstacles"), False)}
    assert layout.panels == {"Exec plots": Placement(edge="left", width=500)}
    assert layout.plugin("obstacles", True, ("Obstacles",)) == (False, ("Tools", "Obstacles"))
    assert layout.plugin("console", False, ()) == (False, ())


def test_bad_entries_are_reported_and_skipped(tmp_path):
    """Each bad entry is one line and is left out; the good ones stay."""
    layout, warnings = read(tmp_path, {
        "windows": {"Plan": {"float": [410]}, "Base": {"dock": "up"}, "Arm": {"dock_below": "Nowhere"},
                    "Ok": {"dock": "right", "width": -1}, "Fine": {"dock": "right"}},
        "plugins": {"a": {"hidden": "yes"}, "b": {"tab_path": []}, "c": {"colour": 1}},
        "panels": {"Console": {"dock_below": "side"}},
        "extra": {}})
    assert set(layout.windows) == {"Arm", "Fine"} and layout.windows["Arm"] == Placement(edge="right")
    assert layout.plugins == {} and layout.panels == {}
    assert len(warnings) == 9


def test_a_file_that_is_no_json_object_is_the_code_layout(tmp_path):
    """Broken JSON, or a list: reported once, nothing overridden."""
    for text in ("{", "[]"):
        layout, warnings = read(tmp_path, text)
        assert layout == Layout() and len(warnings) == 1


def test_preferences_are_read_and_bad_ones_left_to_the_code(tmp_path):
    """The monitor's ghost timeout; a bad value or a plugin's setting at the top level is reported and left out."""
    layout, warnings = read(tmp_path, {"preferences": {"ghost_timeout": 0}})
    assert warnings == [] and layout.preferences == Preferences(0.0)
    layout, warnings = read(tmp_path, {"preferences": {"ghost_timeout": -1, "big_buttons": True}})
    assert layout.preferences == Preferences() and len(warnings) == 2


def test_each_plugin_gets_only_its_own_preferences_unchecked(tmp_path):
    """A plugin's `preferences` object reaches that plugin as it is; none for another; a non-object is skipped."""
    layout, warnings = read(tmp_path, {"plugins": {"console": {"preferences": {"hidden": ["runtime"]}},
                                                   "assembly_run": {"preferences": ["big_buttons"]}}})
    assert dict(layout.plugin_preferences("console")) == {"hidden": ["runtime"]}
    assert dict(layout.plugin_preferences("assembly_run")) == {} and dict(layout.plugin_preferences("health")) == {}
    assert len(warnings) == 1 and "preferences must be a JSON object" in warnings[0]
