"""The plugin registry: ordering by dependencies, and what an unknown name reports."""

from __future__ import annotations

import pytest

from husky_assembly_teleop.plugin_api import plugin as plugin_module
from husky_assembly_teleop.plugin_api.plugin import HuskyPlugin, resolve_order


def _plugin(name: str, requires: tuple[str, ...] = ()) -> type[HuskyPlugin]:
    return type(name, (HuskyPlugin,), {"name": name, "requires": requires})


@pytest.fixture
def registry(monkeypatch):
    """A registry holding arm_planner, base_planner and robot_control, the planners requiring robot_control."""
    monkeypatch.setenv("RCUTILS_COLORIZED_OUTPUT", "0")
    plugins = {
        "robot_control": _plugin("robot_control"),
        "arm_planner": _plugin("arm_planner", ("robot_control",)),
        "base_planner": _plugin("base_planner", ("robot_control",)),
    }
    monkeypatch.setattr(plugin_module, "PLUGIN_REGISTRY", plugins)
    return plugins


def test_dependencies_come_first(registry):
    """Each plugin comes after what it requires, and a shared dependency only once."""
    assert resolve_order(["arm_planner", "base_planner"]) == ["robot_control", "arm_planner", "base_planner"]


def test_unknown_name_proposes_best_match_and_lists_all(registry):
    """A misspelt name gets the closest registered name proposed, and every registered name listed."""
    with pytest.raises(KeyError) as error:
        resolve_order(["arm_planer"])
    message = str(error.value)
    assert "did you mean   arm_planner\n" in message
    assert all(name in message for name in registry)


def test_unknown_dependency_names_its_dependent(registry):
    """A misspelt `requires` entry names the plugin that requires it."""
    registry["gripper"] = _plugin("gripper", ("robot_controls",))
    with pytest.raises(KeyError) as error:
        resolve_order(["gripper"])
    assert "'robot_controls', required by 'gripper'" in str(error.value)
    assert "did you mean   robot_control\n" in str(error.value)


def test_unknown_name_is_coloured_when_forced(registry, monkeypatch):
    """With colour forced on, the proposed name is bold green."""
    monkeypatch.setenv("RCUTILS_COLORIZED_OUTPUT", "1")
    with pytest.raises(KeyError) as error:
        resolve_order(["arm_planer"])
    assert "\033[1;32marm_planner\033[0m" in str(error.value)


def test_unknown_name_with_empty_registry(monkeypatch):
    """With nothing registered, the message points to the import errors in the log."""
    monkeypatch.setattr(plugin_module, "PLUGIN_REGISTRY", {})
    with pytest.raises(KeyError, match="no plugins are registered"):
        resolve_order(["arm_planner"])
