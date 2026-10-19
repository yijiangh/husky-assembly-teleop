"""The monitor's tick and plugin lifecycle, without ROS or viser: order, errors, soft stop and shutdown."""

from __future__ import annotations

import asyncio
from contextlib import contextmanager
from types import SimpleNamespace

import pytest
from robot_fakes import Log

import husky_assembly_teleop.monitor as monitor_module
from husky_assembly_teleop.monitor import HuskyMonitor
from husky_assembly_teleop.plugin_api.plugin import HuskyPlugin
from husky_assembly_teleop.world.measured import WorldState
from scaffolding_assembly_core.scene import Scene

#: Consecutive failing ticks after which a plugin is stopped.
MAX_ERRORS = 3


class Recorder(HuskyPlugin):
    """A plugin that writes each hook it runs into a shared list, and raises where told to."""

    def __init__(self, name: str, calls: list, requires: tuple[str, ...] = (), raises: tuple[str, ...] = ()):
        """A plugin called `name`; `raises` names the hooks that raise."""
        self.name, self.requires, self.calls, self.raises = name, requires, calls, raises

    def _hook(self, hook: str) -> None:
        self.calls.append((self.name, hook))
        if hook in self.raises:
            raise RuntimeError(f"{self.name} {hook} broke")

    def setup(self, ctx) -> None:
        """Record it."""
        self._hook("setup")

    def update(self, ctx) -> None:
        """Record it."""
        self._hook("update")

    def draw(self, ctx) -> None:
        """Record it."""
        self._hook("draw")

    def teardown(self, ctx) -> None:
        """Record it."""
        self._hook("teardown")


class FakeView:
    """A plugin's UI corner; only clearing it matters here."""

    def __init__(self):
        """Not cleared yet."""
        self.cleared = False

    def clear(self) -> None:
        """Take the UI away."""
        self.cleared = True


class FakeVisualization:
    """Stands in for ui.visualization.Visualization: records draws, the stop button and the broken banner."""

    def __init__(self, port=None, phone_port=None):
        """Nothing drawn, no stop requested."""
        self.ports, self.frozen, self.stop_requested = (8080, 8081), False, False
        self.draws, self.broken, self.views, self.stopped = 0, {}, {}, False

    def view_for(self, name: str) -> FakeView:
        """A fresh view per plugin."""
        return self.views.setdefault(name, FakeView())

    def take_stop_request(self) -> bool:
        """Whether Stop all was pressed since the last tick."""
        requested, self.stop_requested = self.stop_requested, False
        return requested

    @contextmanager
    def atomic(self):
        """One batch."""
        yield

    def draw(self, snapshot) -> None:
        """Count it."""
        self.draws += 1

    def show_broken(self, reasons: dict) -> None:
        """Keep the banner's reasons."""
        self.broken = dict(reasons)

    def stop(self) -> None:
        """The servers stop."""
        self.stopped = True


class FakeTimer:
    """Stands in for TickTimer: times nothing."""

    @contextmanager
    def part(self, _what: str):
        """Time nothing."""
        yield

    def note(self, *_args) -> None:
        """Note nothing."""

    def timed_task(self, _name: str, work):
        """The task's coroutine, untimed."""
        return work

    def stop(self) -> None:
        """Nothing to stop."""


def build(monkeypatch, *plugins: HuskyPlugin, robots=()) -> HuskyMonitor:
    """A monitor with these plugins (dependency order), built by its own `_build` without ROS or viser.

    Each robot in `robots` needs only `soft_stop()`.
    """
    monkeypatch.setattr(monitor_module, "Visualization", FakeVisualization)
    monkeypatch.setattr(monitor_module, "Kinematics", lambda *_args, **_kwargs: SimpleNamespace(update=lambda w: None))
    monkeypatch.setattr(monitor_module, "load_plugins", lambda _names, _log_error: list(plugins))
    monkeypatch.setattr(monitor_module, "take_snapshot", lambda *_args: Scene())
    monitor = HuskyMonitor.__new__(HuskyMonitor)
    log, clock = Log(), [100.0]
    monitor.get_logger, monitor.now, monitor.clock = (lambda: log), (lambda: clock[0]), clock
    monitor._config = SimpleNamespace(robots=(), enabled_plugins=(), viser_port=0, phone_port=0,
                                      max_plugin_errors=MAX_ERRORS, shutdown_grace=0.5, tick_period=0.001)
    monitor._world, monitor._loaded, monitor._object_connections, monitor._tick_index = WorldState(), {}, {}, 0
    monitor._executor = SimpleNamespace(shutdown=lambda: None)
    monitor._kinematics, monitor._scene, monitor._snapshot, monitor._viz = None, Scene(), Scene(), None
    monitor._timer, monitor._stop_reasons, monitor._undecodable, monitor._shut_down = FakeTimer(), {}, set(), False
    monitor._pump_ros = lambda: 0
    monitor._startup_started, monitor._startup_times = 0.0, {}
    for index, robot in enumerate(robots):
        monitor._world.robots[f"r{index}"] = robot
    monitor._build()
    return monitor


def ticks(monitor: HuskyMonitor, count: int = 1) -> None:
    """Run `count` ticks."""
    async def run():
        for _ in range(count):
            await monitor._tick()
    asyncio.run(run())


def test_each_tick_runs_update_then_draw_in_dependency_order(monkeypatch):
    """Setup once in order; every tick updates all plugins, then draws robots and plugins."""
    calls = []
    monitor = build(monkeypatch, Recorder("a", calls), Recorder("b", calls, requires=("a",)))
    ticks(monitor, 2)
    assert calls == [("a", "setup"), ("b", "setup"),
                     ("a", "update"), ("b", "update"), ("a", "draw"), ("b", "draw"),
                     ("a", "update"), ("b", "update"), ("a", "draw"), ("b", "draw")]
    assert monitor._viz.draws == 2


def test_intents_run_before_the_update_that_reacts_to_them(monkeypatch):
    """A click queued from a viser thread runs at the start of the plugin's next step."""
    calls = []
    monitor = build(monkeypatch, Recorder("a", calls))
    monitor._loaded["a"].ctx.submit("click", lambda: calls.append(("a", "click")))
    ticks(monitor)
    assert calls[1:3] == [("a", "click"), ("a", "update")]


def test_a_plugin_failing_tick_after_tick_is_stopped_with_its_dependents(monkeypatch):
    """After MAX_ERRORS failing ticks in a row: it and what requires it stop, the banner names both."""
    calls = []
    monitor = build(monkeypatch, Recorder("a", calls, raises=("update",)), Recorder("b", calls, requires=("a",)),
                    Recorder("c", calls))
    ticks(monitor, MAX_ERRORS - 1)
    assert not monitor._loaded["a"].stopped
    ticks(monitor, 2)
    assert monitor._loaded["a"].stopped and monitor._loaded["b"].stopped and not monitor._loaded["c"].stopped
    assert set(monitor._viz.broken) == {"a", "b"} and "depends on 'a'" in monitor._viz.broken["b"]
    calls.clear()
    ticks(monitor)
    assert calls == [("c", "update"), ("c", "draw")]


def test_failures_must_be_consecutive_to_stop_a_plugin(monkeypatch):
    """A plugin that fails every other tick keeps running."""
    calls = []
    flaky = Recorder("a", calls)
    monitor = build(monkeypatch, flaky)
    for index in range(2 * MAX_ERRORS):
        flaky.raises = ("update",) if index % 2 == 0 else ()
        ticks(monitor)
    assert not monitor._loaded["a"].stopped and monitor._loaded["a"].errors == 1


def test_a_plugin_failing_setup_stops_and_its_dependents_are_never_set_up(monkeypatch):
    """Setup raising stops the plugin at once; a dependent is stopped too, and is never set up or torn down."""
    calls = []
    monitor = build(monkeypatch, Recorder("a", calls, raises=("setup",)), Recorder("b", calls, requires=("a",)),
                    Recorder("c", calls))
    assert calls == [("a", "setup"), ("c", "setup")]
    assert monitor._loaded["a"].stopped and monitor._loaded["b"].stopped
    assert "failed in setup" in monitor._viz.broken["a"]
    asyncio.run(monitor.shutdown())
    assert ("a", "teardown") in calls and ("b", "teardown") not in calls


@pytest.mark.safety
def test_soft_stop_stops_every_robot_before_plugins_act_and_cancels_their_tasks(monkeypatch):
    """Stop all: each robot's soft stop runs before any plugin update, and every plugin task is cancelled."""
    calls = []
    robots = [SimpleNamespace(soft_stop=lambda index=index: calls.append((f"robot {index}", "soft stop")))
              for index in range(2)]
    monitor = build(monkeypatch, Recorder("a", calls), robots=robots)

    async def run():
        task = monitor._loaded["a"].ctx.spawn("move", asyncio.sleep(60.0))
        monitor._viz.stop_requested = True
        await monitor._tick()
        await asyncio.sleep(0)
        return task

    task = asyncio.run(run())
    assert calls[1:4] == [("robot 0", "soft stop"), ("robot 1", "soft stop"), ("a", "update")]
    assert task.cancelled()


def test_shutdown_tears_down_in_reverse_order_once_and_waits_for_cleanup(monkeypatch):
    """Teardown runs last plugin first, after its tasks finished cleaning up; a second shutdown does nothing."""
    calls = []
    monitor = build(monkeypatch, Recorder("a", calls), Recorder("b", calls, requires=("a",)))

    async def cleanup():
        try:
            await asyncio.sleep(60.0)
        finally:
            calls.append(("b", "task cleaned up"))

    async def run():
        monitor._loaded["b"].ctx.spawn("hold", cleanup())
        await asyncio.sleep(0)
        await monitor.shutdown()
        await monitor.shutdown()

    calls.clear()
    asyncio.run(run())
    assert calls == [("b", "task cleaned up"), ("b", "teardown"), ("a", "teardown")]
    assert all(view.cleared for view in monitor._viz.views.values()) and monitor._viz.stopped


def test_a_teardown_that_raises_does_not_stop_the_others(monkeypatch):
    """Every set-up plugin is torn down even if one teardown raises; the error is logged."""
    calls = []
    monitor = build(monkeypatch, Recorder("a", calls), Recorder("b", calls, raises=("teardown",)))
    asyncio.run(monitor.shutdown())
    assert ("a", "teardown") in calls and "failed in teardown" in monitor.get_logger().text("error")
