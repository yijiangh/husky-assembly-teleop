"""
The plugin base class, and the registry that finds and orders plugins.

! Keep a plugin's derived state (plans, plot histories) on its instance, not in the monitor.
"""

from __future__ import annotations

import importlib
import pkgutil
import traceback
from typing import Callable, Iterable

from .context import PluginContext


class HuskyPlugin:
    """One self-contained feature with its own state, UI and scene nodes.

    Override only the hooks you need; work longer than one tick goes in a task started with `ctx.spawn`.

    - ! Hooks run on the main thread and must never block: a slow hook stalls every ROS callback.
    - ! After `config.max_plugin_errors` failed ticks in a row (a hook or task raised), the plugin and its
      dependents are stopped until restart.
    """

    #: Unique name, used for the scene path, GUI folder, config and logs.
    name: str = "unnamed"

    #: Plugins to set up before this one; reach them through `ctx.require`.
    requires: tuple[str, ...] = ()

    #: Whether its design is still open; loading it logs a warning.
    experimental: bool = False

    def setup(self, ctx: PluginContext) -> None:
        """Build widgets and scene nodes once, at startup, and keep the handles for `draw`.

        ! `teardown` still runs after a raise here, so it must cope with a half-finished setup.
        """

    def update(self, ctx: PluginContext) -> None:
        """Advance this plugin's state by one tick, after its intents and before its tasks resume."""

    def draw(self, ctx: PluginContext) -> None:
        """Copy state into the handles made in `setup`; do not add nodes here.

        ! Skipped while the panels are frozen: put anything that must keep running in `update`, an intent or a task.
        """

    def teardown(self, ctx: PluginContext) -> None:
        """Release what the monitor cannot (PyBullet bodies, files, hardware), once at shutdown, even if stopped.

        Tasks are cancelled before this runs; scene nodes and widgets are removed after.
        """


#: Plugin name -> class. Populated by @register at import time.
PLUGIN_REGISTRY: dict[str, type[HuskyPlugin]] = {}


def register(plugin_class: type[HuskyPlugin]) -> type[HuskyPlugin]:
    """Class decorator that registers a plugin under its `name` and returns it unchanged.

    Raises:
        ValueError: If `plugin_class.name` is missing or already taken.
    """
    name = plugin_class.name
    if name == "unnamed":
        raise ValueError(f"{plugin_class.__name__} must set a unique `name`")
    if name in PLUGIN_REGISTRY:
        raise ValueError(f"plugin name {name!r} is already registered")
    PLUGIN_REGISTRY[name] = plugin_class
    return plugin_class


def discover(log_error: Callable[[str], None]) -> None:
    """Import every module under `plugins/` so their @register calls run.

    A module that fails to import is logged and skipped.
    """
    from .. import plugins

    for module in pkgutil.iter_modules(plugins.__path__):
        try:
            importlib.import_module(f"{plugins.__name__}.{module.name}")
        except Exception:
            log_error(f"skipping plugin module {module.name!r}, which failed to "
                      f"import:\n{traceback.format_exc()}")


def load_plugins(enabled: Iterable[str],
                 log_error: Callable[[str], None]) -> list[HuskyPlugin]:
    """Create the enabled plugins, plus their dependencies, in setup order.

    Args:
        enabled: Names to load. Empty loads nothing.
        log_error: Receives a message for each module that fails to import.

    Returns:
        list[HuskyPlugin]: Constructed plugins, each after what it requires.

    Raises:
        KeyError: If a requested or required name is not registered.
        ValueError: If the dependency graph contains a cycle.
    """
    discover(log_error)
    return [PLUGIN_REGISTRY[name]() for name in resolve_order(enabled)]


def resolve_order(requested: Iterable[str]) -> list[str]:
    """Sort `requested` and its dependencies so each comes after what it requires.

    ? Order matters: a plugin sees an earlier plugin's effects this tick, a later one's next tick.

    Raises:
        KeyError: If a name is not registered, including when its module failed to import (see the log).
        ValueError: If a dependency cycle is found.
    """
    order: list[str] = []
    settled: set[str] = set()
    # Names on the current path; seeing one twice means a cycle.
    visiting: list[str] = []

    def visit(name: str) -> None:
        if name in settled:
            return
        if name in visiting:
            cycle = " -> ".join(visiting[visiting.index(name):] + [name])
            raise ValueError(f"plugin dependency cycle: {cycle}")
        if name not in PLUGIN_REGISTRY:
            raise KeyError(f"no plugin registered under the name {name!r}")

        visiting.append(name)
        for dependency in PLUGIN_REGISTRY[name].requires:
            visit(dependency)
        visiting.pop()

        settled.add(name)
        order.append(name)

    for name in requested:
        visit(name)
    return order
