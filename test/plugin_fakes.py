"""A plugin context for tests: ROS time ticking at 10 Hz, tasks on the running loop, other plugins, logs and a GUI.

Run the plugin's tasks inside `asyncio.run`; `step(t)` changes the world each tick (e.g. the snapshot). Widgets are
`FakeWidget`s: call a widget's `callback` to click it.
"""

from __future__ import annotations

import asyncio
from contextlib import nullcontext
from functools import partial
from types import SimpleNamespace
from typing import Any, Callable

from husky_assembly_teleop.plugin_api.concurrency import WaitTimeout
from scaffolding_assembly_core.scene import Scene


class FakeWidget(SimpleNamespace):
    """A viser widget: what was given when it was added, what the plugin wrote into it, and its callback."""

    def on_click(self, callback: Callable) -> None:
        """Keep the callback."""
        self.callback = callback

    on_update = on_click


class FakeGui:
    """viser's GUI api: every `add_*` call makes a FakeWidget, kept in order in `widgets`."""

    def __init__(self):
        """No widgets yet."""
        self.widgets: list[FakeWidget] = []

    def __getattr__(self, name: str) -> Callable[..., FakeWidget]:
        """`add_<kind>(label, value, **options)`: a widget of that kind."""
        if not name.startswith("add_"):
            raise AttributeError(name)

        def add(label: str = "", *values: Any, **options: Any) -> FakeWidget:
            widget = FakeWidget(kind=name[4:], label=label, value=values[0] if values else options.get("initial_value"),
                                content="", color=options.get("color"), options=options)
            self.widgets.append(widget)
            return widget
        return add

    def of_kind(self, kind: str) -> list[FakeWidget]:
        """The widgets of one kind, e.g. "button", in order."""
        return [widget for widget in self.widgets if widget.kind == kind]


class FakeContext:
    """The part of a `PluginContext` plugins use, without a monitor; a test reads `logs` and moves `t`."""

    def __init__(self, robots: tuple = (), plugins: dict | None = None, snapshot: Scene | None = None,
                 step: Callable[[float], None] = lambda _t: None, drive_root=None):
        """Start at ROS time 10 s.

        Args:
            robots: The configured robots, `config.robots`.
            plugins: Name -> plugin, what `require` returns.
            snapshot: The scene snapshot; tests may replace `scene.snapshot` at any time.
            step: Called with the new time at every tick.
            drive_root: `config.drive_root`.
        """
        self.t, self.step = 10.0, step
        self.config = SimpleNamespace(robots=tuple(robots), drive_root=drive_root)
        self.plugins = dict(plugins or {})
        self.scene = SimpleNamespace(snapshot=snapshot if snapshot is not None else Scene())
        self.world = SimpleNamespace(robots={})
        self.gui = FakeGui()
        self.view = SimpleNamespace(ui=lambda: nullcontext(self.gui), scene_root="/plugin")
        #: (level, message) per log line.
        self.logs: list[tuple[str, str]] = []

    def now(self) -> float:
        """ROS time, seconds."""
        return self.t

    def require(self, name: str) -> Any:
        """Another plugin, by name."""
        return self.plugins[name]

    def defer(self, label: str, run: Callable[[], Any]) -> Callable[..., Any]:
        """A widget callback that runs `run` at once (no viser thread here)."""
        return lambda *_event: run()

    def defer_value(self, label: str, run: Callable[[Any], Any]) -> Callable[[Any], Any]:
        """A widget callback handed the new value, run at once."""
        return run

    def spawn(self, label: str, work) -> asyncio.Task:
        """Start a task on the running loop."""
        return asyncio.get_running_loop().create_task(work, name=label)

    async def run_in_thread(self, function: Callable, *args: Any, **kwargs: Any) -> Any:
        """Run on the loop's default thread pool."""
        return await asyncio.get_running_loop().run_in_executor(None, partial(function, *args, **kwargs))

    async def next_tick(self) -> None:
        """Advance 0.1 s, step the world, let other tasks run."""
        self.t += 0.1
        self.step(self.t)
        await asyncio.sleep(0)

    async def wait_until(self, predicate: Callable[[], bool], timeout_s: float | None = None,
                         description: str = "condition") -> None:
        """Tick until `predicate` holds, as `PluginContext.wait_until`."""
        deadline = None if timeout_s is None else self.t + timeout_s
        while not predicate():
            if deadline is not None and self.t >= deadline:
                raise WaitTimeout(f"timed out after {timeout_s}s waiting for {description}")
            await self.next_tick()

    async def sleep(self, seconds: float) -> None:
        """Tick for `seconds`."""
        until = self.t + seconds
        await self.wait_until(lambda: self.t >= until)

    def log_info(self, message: str) -> None:
        """Keep the line."""
        self.logs.append(("info", message))

    def log_warn(self, message: str) -> None:
        """Keep the line."""
        self.logs.append(("warn", message))

    def log_error(self, message: str) -> None:
        """Keep the line."""
        self.logs.append(("error", message))

    def lines(self, level: str) -> list[str]:
        """The logged lines of one level."""
        return [message for kind, message in self.logs if kind == level]
