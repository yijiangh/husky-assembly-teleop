"""
The plugin base class, and the registry that finds and orders plugins.

! A plugin owns its own derived state -- trajectories, preview ghosts, plot
  histories, plans in progress -- on its own instance and nowhere else. That is
  what stops the god object growing back; splitting files without giving that
  state a home just moves it. See doc/refactor_rationale.md.
"""

from __future__ import annotations

import importlib
import pkgutil
import traceback
from typing import Callable, Iterable

from .concurrency import Task
from .context import PluginContext


class HuskyPlugin:
    """One self-contained feature: its own state, its own UI, its own scene nodes.

    Two ways to write one, whichever fits:

      - **Reactive**: override `update`, called once per tick. For anything that
        mirrors state -- a live plot, a readout, a diagnostic overlay.
      - **Sequential**: override `run` as a generator that yields whenever it is
        willing to be interrupted. For anything with a shape -- a calibration
        sweep, a plan-then-execute cycle. The logic stays in the order it
        happens instead of being smeared across state flags.

    Either kind can call `ctx.spawn` for a discrete, cancellable operation
    alongside its main loop. Every hook below receives this plugin's
    PluginContext as `ctx`.

    ! Every hook runs on the ROS thread, so all of them may touch world state,
      the scene and ROS with no locking. The one thing they may not do is block:
      a slow step stalls the tick and every ROS callback behind it. Wait by
      yielding, with the helpers in concurrency.py.

    ! A hook that raises counts against the plugin. After
      `config.max_plugin_errors` consecutive ticks with a failure it is torn
      down, along with anything that requires it. A clean tick clears the count.
    """

    #: Unique name. Used for the scene path, the GUI folder label, the
    #: enabled-plugins config, dependency declarations and log messages.
    name: str = "unnamed"

    #: Plugins that must be set up before this one, and which it may reach
    #: through `ctx.require`. Declaring this is what gives the monitor a
    #: deterministic order -- which matters, because each plugin drains its own
    #: intent queue when its turn comes, so it sees an earlier plugin's effects
    #: this tick and a later one's only next tick.
    requires: tuple[str, ...] = ()

    def setup(self, ctx: PluginContext) -> None:
        """Register UI and initialise state. Called once, at startup.

        Build every widget and scene node here, keep the handles on `self`, and
        mutate them later in `draw`: viser is retained mode, so nothing needs to
        be -- or should be -- rebuilt per tick. `ctx.view` is the private UI
        slice to build into.

        A plugin whose setup raises is disabled immediately rather than started
        against half-built state, so it is fine to let a missing file or an
        unreachable service propagate from here.
        """

    def run(self, ctx: PluginContext) -> Task:
        """The plugin's main loop, resumed one step per tick.

        This default calls `update` once per tick forever, which is what a
        reactive plugin wants. Override to write sequential logic instead, and
        yield often enough that no step takes more than a few milliseconds.

        ! The loop must never end. A plugin runs for the lifetime of the
          program, so an override loops forever and waits by yielding. A
          sequential plugin that finishes its sweep goes back to waiting for the
          next trigger rather than returning:

              def run(self, ctx):
                  while True:
                      yield from wait_until(ctx, lambda: self.triggered)
                      self.triggered = False
                      yield from self._sweep(ctx)

          Returning is reported as a failure and counts against
          `max_plugin_errors`, the same as raising -- which it has to be, since
          Python closes a generator that raised, and a loop that silently stopped
          would leave the plugin loaded, still drawing, and never advancing
          again. That is the "panel froze and the log does not say why" failure.

          So guard inside the loop if a step may fail and the sequence should
          carry on; let it propagate if the plugin should be taken down.

        Yields:
            None: Once per tick.
        """
        while True:
            self.update(ctx)
            yield

    def update(self, ctx: PluginContext) -> None:
        """Advance this plugin's state by one tick. Called by the default `run`.

        The scene already reflects this tick's measurements by the time this
        runs, so kinematics and collision queries answer against current reality.
        """

    def draw(self, ctx: PluginContext) -> None:
        """Push this plugin's state into its scene nodes and widgets.

        Called once per tick after every plugin has stepped, inside the
        visualization's atomic block. Assign to handles created in `setup`; do
        not add nodes here.
        """

    def teardown(self, ctx: PluginContext) -> None:
        """Release anything that outlives the process's own cleanup.

        Called on shutdown, and when a plugin is disabled after repeated errors.
        Jobs are cancelled and scene nodes and widgets removed for you; this is
        for open files, recordings and hardware left in an odd state.
        """


#: Plugin name -> class. Populated by @register at import time.
PLUGIN_REGISTRY: dict[str, type[HuskyPlugin]] = {}


def register(plugin_class: type[HuskyPlugin]) -> type[HuskyPlugin]:
    """Class decorator that makes a plugin discoverable. Returns it unchanged.

    ! By decorator rather than by an import in the monitor. If HuskyMonitor
      imported each plugin class by name, adding a plugin would mean editing the
      monitor -- exactly the coupling the plugin system exists to remove.

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
    """Import every module under `plugins/`, running their @register calls.

    ! One unimportable plugin must not take the monitor down with it.
      Every module has to be imported to find out what it registers, and plugins
      pull in heavy optional dependencies -- compas_fab, the Drake planner
      client, tracikpy. A failure goes to `log_error` and that module is
      skipped; if it was the one defining a requested plugin, `resolve_order`
      then reports the name as unregistered.
    """
    from . import plugins

    for module in pkgutil.iter_modules(plugins.__path__):
        try:
            importlib.import_module(f"{plugins.__name__}.{module.name}")
        except Exception:
            log_error(f"skipping plugin module {module.name!r}, which failed to "
                      f"import:\n{traceback.format_exc()}")


def load_plugins(enabled: Iterable[str],
                 log_error: Callable[[str], None]) -> list[HuskyPlugin]:
    """Discover, resolve dependencies, and instantiate plugins in setup order.

    Args:
        enabled: Names to load, explicitly: a new file under `plugins/` must not
            enable itself everywhere it is installed. Empty loads nothing, which
            is a valid way to run the core alone. Dependencies are pulled in
            even when not listed.
        log_error: Where to report plugin modules that could not be imported.

    Returns:
        list[HuskyPlugin]: Constructed plugins, each after what it requires.

    Raises:
        KeyError: If a requested or required name is not registered.
        ValueError: If the dependency graph contains a cycle.
    """
    discover(log_error)
    return [PLUGIN_REGISTRY[name]() for name in resolve_order(enabled)]


def resolve_order(requested: Iterable[str]) -> list[str]:
    """Topologically sort `requested` and its dependencies into setup order.

    ? Why order is worth pinning down. Each plugin drains its own intent queue
      when its turn comes, rather than everything draining at the top of the
      tick, which makes ordering observable: a plugin sees an earlier plugin's
      intents in the same tick, and a later one's only in the next.

    Raises:
        KeyError: If a name, or something it requires, is not registered. A
            module that failed to import looks the same from here, so check the
            log for a skipped module before assuming a typo.
        ValueError: If a dependency cycle is found.
    """
    order: list[str] = []
    settled: set[str] = set()
    # Names on the current depth-first path. A name seen twice on one path is a
    # cycle, which has to be a hard error: no order satisfies it.
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
