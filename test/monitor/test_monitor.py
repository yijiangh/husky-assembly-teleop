"""The monitor's tick and plugin lifecycle, without ROS or viser: order, errors, soft stop and shutdown."""

from __future__ import annotations

import asyncio
import math
from collections import deque
from contextlib import contextmanager
from types import SimpleNamespace

import pytest
from robot_fakes import Log

import husky_assembly_teleop.monitor as monitor_module
from husky_assembly_teleop.monitor import HuskyMonitor
from husky_assembly_teleop.plugin_api.console_log import ConsoleLog
from husky_assembly_teleop.plugin_api.plugin import HuskyPlugin
from husky_assembly_teleop.robot_interface.soft_stop import RobotStopped
from husky_assembly_teleop.world.clearance import ARM_STOP_DISTANCE, STOP_DISTANCE, WATCH_EVERY
from husky_assembly_teleop.world.measured import WorldState
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.robot import RobotObject
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
        self.ports, self.paused, self.stop_requested = (8080, 8081), False, False
        self.draws, self.broken, self.views, self.stopped = 0, {}, {}, False

    def view_for(self, name: str, *_settings: object) -> FakeView:
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


class FakeClearances:
    """Stands in for ClearanceChecker: keeps what it is asked, and hands out `found` once as the finished check."""

    def __init__(self):
        """Nothing asked or found."""
        self.asked, self.found = [], None

    def check(self, _snapshot, asked) -> bool:
        """Keep what is asked."""
        self.asked.append(asked)
        return True

    def finished(self):
        """`found`, once."""
        found, self.found = self.found, None
        return found

    def close(self) -> None:
        """Nothing to close."""


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
    monitor._ros_logger, monitor.console_log = log, ConsoleLog()
    monitor._config = SimpleNamespace(robots=(), enabled_plugins=(), viser_port=0, phone_port=0,
                                      max_plugin_errors=MAX_ERRORS, shutdown_grace=0.5, tick_period=0.001)
    monitor._world, monitor._loaded, monitor._object_connections, monitor._tick_index = WorldState(), {}, {}, 0
    monitor._executor = SimpleNamespace(shutdown=lambda: None)
    monitor._kinematics, monitor._scene, monitor._snapshot, monitor._viz = None, Scene(), Scene(), None
    monitor._timer, monitor._stop_reasons, monitor._undecodable, monitor._shut_down = FakeTimer(), {}, set(), False
    monitor._pump_ros = lambda: 0
    monitor._startup_started, monitor._startup_times = 0.0, {}
    monitor._clearances, monitor.clearance_trips = FakeClearances(), ()
    monitor._clearances_asked, monitor._watched_at = [], -math.inf
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
    """Stop all: each robot's soft stop runs before any plugin update; every task but the survivors is cancelled."""
    calls = []
    robots = [SimpleNamespace(soft_stop=lambda reason="", index=index: calls.append((f"robot {index}", "soft stop")))
              for index in range(2)]
    monitor = build(monkeypatch, Recorder("a", calls), robots=robots)

    async def run():
        ctx = monitor._loaded["a"].ctx
        move = ctx.spawn("move", asyncio.sleep(60.0))
        load = ctx.spawn("load", asyncio.sleep(60.0), survives_soft_stop=True)
        monitor._viz.stop_requested = True
        await monitor._tick()
        await asyncio.sleep(0)
        return move, not load.done()

    move, survived = asyncio.run(run())
    assert calls[1:4] == [("robot 0", "soft stop"), ("robot 1", "soft stop"), ("a", "update")]
    assert move.cancelled() and survived
    assert "cancelled 'a: move'" in monitor.get_logger().text("warning")


@pytest.mark.safety
def test_clicks_queued_before_stop_all_still_run_and_a_refused_one_is_no_failure(monkeypatch):
    """A click queued before Stop all runs after it; one whose command the stopped robot refuses ends with a warning."""
    calls = []
    monitor = build(monkeypatch, Recorder("a", calls))
    ctx = monitor._loaded["a"].ctx

    def start() -> None:
        raise RobotStopped("robot r0 is soft stopped, a path not sent; press Resume first")

    ctx.submit("start run", start)
    ctx.submit("load", lambda: calls.append(("a", "load")))
    monitor._viz.stop_requested = True
    ticks(monitor)
    assert ("a", "load") in calls and monitor._loaded["a"].errors == 0
    assert "intent 'start run' ended: robot r0 is soft stopped" in monitor.get_logger().text("warning")


@pytest.mark.safety
def test_a_refused_command_in_an_update_or_a_task_never_stops_the_plugin(monkeypatch):
    """A RobotStopped raised by update tick after tick, or ending a task, is a warning, not a failure."""

    class Refused(Recorder):
        def update(self, ctx) -> None:
            """Command a stopped robot."""
            super().update(ctx)
            raise RobotStopped("robot r0 is soft stopped, a twist not sent; press Resume first")

    async def follow() -> None:
        raise RobotStopped("robot r0 is soft stopped, a path not sent; press Resume first")

    monitor = build(monkeypatch, Refused("a", []))

    async def run():
        monitor._loaded["a"].ctx.spawn("follow", follow())
        for _ in range(MAX_ERRORS + 1):
            await monitor._tick()

    asyncio.run(run())
    warnings = monitor.get_logger().text("warning")
    assert not monitor._loaded["a"].stopped and monitor._loaded["a"].errors == 0
    assert "plugin 'a' update ended" in warnings and "task 'a: follow' ended" in warnings


@pytest.mark.safety
def test_a_robot_failing_to_stop_does_not_keep_the_others_from_stopping(monkeypatch):
    """One robot's soft stop raising is logged; the next robot still stops, tasks are cancelled, the tick goes on."""
    calls = []

    def broken(reason=""):
        raise RuntimeError("publisher destroyed")

    robots = [SimpleNamespace(soft_stop=broken),
              SimpleNamespace(soft_stop=lambda reason="": calls.append(("r1", "soft stop")))]
    monitor = build(monkeypatch, Recorder("a", calls), robots=robots)

    async def run():
        task = monitor._loaded["a"].ctx.spawn("move", asyncio.sleep(60.0))
        monitor._viz.stop_requested = True
        await monitor._tick()
        await asyncio.sleep(0)
        return task

    assert asyncio.run(run()).cancelled()
    assert ("r1", "soft stop") in calls and ("a", "draw") in calls
    assert "publisher destroyed" in monitor.get_logger().text("error")


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


def test_a_clearance_is_checked_every_tick_and_trips_on_what_was_found_or_a_lost_pose(monkeypatch):
    """Each tick asks about driven robots; a found hit or a lost base pose soft stops the robot and cancels the task."""
    calls, stops = [], []
    tracked = SimpleNamespace(mocap_id=1, state=SimpleNamespace(marker_errors=deque([(100.0, 0.0)]),
                                                                last_fix_time=100.0))
    robots = [SimpleNamespace(base=tracked, soft_stop=lambda reason="", serial=serial: stops.append(serial))
              for serial in ("r0", "r1")]
    plugin = Recorder("a", calls)
    monitor = build(monkeypatch, plugin, robots=robots)
    # * Both robots known in the snapshot, else a clearance refuses to start.
    model = SimpleNamespace(movable_joints=())
    known = Scene(robots={robot_id(name): RobotObject(robot_id(name), model, Pose(), {}) for name in ("r0", "r1")})
    monkeypatch.setattr(monitor_module, "take_snapshot", lambda *_args: known)
    monitor._snapshot = known
    ctx = monitor._loaded["a"].ctx

    async def drive(serial):
        with ctx.clearance([serial]) as clearance:
            await ctx.wait_until(lambda: False)
        return clearance

    async def run():
        found = ctx.spawn("drive r0", drive("r0"))
        await asyncio.sleep(0)
        await monitor._tick()
        (key, own, margin, assumable, _allowed), = monitor._clearances.asked[-1]
        assert own == robot_id("r0") and margin == STOP_DISTANCE and assumable == frozenset()
        monitor._clearances.found = [(key, "came within 10 cm of wall", None)]
        await monitor._tick()
        lost = ctx.spawn("drive r1", drive("r1"))
        await asyncio.sleep(0)
        lost_state = SimpleNamespace(marker_errors=deque(), tracked=False, tracking_valid=False, last_update_time=None)
        robots[1].base = SimpleNamespace(mocap_id=1, state=lost_state)
        await monitor._tick()
        return await asyncio.gather(found, lost, return_exceptions=True)

    found, lost = asyncio.run(run())
    assert isinstance(found, asyncio.CancelledError) and isinstance(lost, asyncio.CancelledError)
    assert stops == ["r0", "r1"]
    errors = monitor.get_logger().text("error")
    assert "robot r0 came within 10 cm of wall" in errors and "robot r1 cannot be checked" in errors
    trips = [(trip.serial, trip.task) for trip in monitor.clearance_trips]
    assert trips == [("r0", "a: drive r0"), ("r1", "a: drive r1")]


def test_a_check_that_finds_nothing_marks_the_clearances_it_asked_about_checked(monkeypatch):
    """Only a finished check counts: asked and not yet answered stays unchecked."""
    monitor = build(monkeypatch, Recorder("a", []), robots=[SimpleNamespace(base=SimpleNamespace(
        mocap_id=1, state=SimpleNamespace(marker_errors=deque([(100.0, 0.0)]), last_fix_time=100.0)))])
    known = Scene(robots={robot_id("r0"): RobotObject(robot_id("r0"), SimpleNamespace(movable_joints=()), Pose(), {})})
    monkeypatch.setattr(monitor_module, "take_snapshot", lambda *_args: known)
    monitor._snapshot = known
    ctx, seen = monitor._loaded["a"].ctx, []

    async def drive():
        with ctx.clearance(["r0"], refuse_first=True) as clearance:
            seen.append(clearance)
            await ctx.wait_until(lambda: False)

    async def run():
        task = ctx.spawn("drive r0", drive())
        await asyncio.sleep(0)
        await monitor._tick()
        asked = seen[0].checked
        monitor._clearances.found = []
        await monitor._tick()
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        return asked

    assert not asyncio.run(run()) and seen[0].checked


def test_a_watch_joins_a_check_every_half_second_and_only_reports(monkeypatch):
    """Asked every WATCH_EVERY; too close fills `found`, never a soft stop; a robot it cannot check is left out."""
    stops = []
    robots = [SimpleNamespace(base=SimpleNamespace(mocap_id=1, state=SimpleNamespace(
        marker_errors=deque([(100.0, 0.0)]), last_fix_time=100.0)), soft_stop=lambda reason="": stops.append(reason))]
    monitor = build(monkeypatch, Recorder("a", []), robots=robots)
    known = Scene(robots={robot_id("r0"): RobotObject(robot_id("r0"), SimpleNamespace(movable_joints=()), Pose(), {})})
    monkeypatch.setattr(monitor_module, "take_snapshot", lambda *_args: known)
    monitor._snapshot = known
    watch = monitor._loaded["a"].ctx.watch_clearance(["r0"], ARM_STOP_DISTANCE)

    async def run():
        await monitor._tick()
        (key, own, margin, _assumable, _allowed), = monitor._clearances.asked[-1]
        assert key[1] is watch and own == robot_id("r0") and margin == ARM_STOP_DISTANCE
        monitor._clearances.found = [(key, "came within 1 cm of the rack", SimpleNamespace())]
        await monitor._tick()
        found, asked = dict(watch.found), len(monitor._clearances.asked)
        monitor.clock[0] += WATCH_EVERY
        await monitor._tick()
        monitor._clearances.found = [(key, "cannot be checked: base not measured", None)]
        await monitor._tick()
        return found, asked

    found, asked = asyncio.run(run())
    assert found == {"r0": "came within 1 cm of the rack"} and asked == 1, "not asked again within WATCH_EVERY"
    assert watch.found == {} and watch.checked_at is not None and not stops


def test_log_lines_reach_the_ros_log_and_the_console_with_their_source(monkeypatch):
    """A plugin's line goes to the ROS log as "[name] text" and to the console as the plain text under its name."""
    monitor = build(monkeypatch, Recorder("a", []))
    monitor._loaded["a"].ctx.log_warn("slow")
    monitor.log_info("started")
    assert monitor.get_logger().lines[-2:] == [("warning", "[a] slow"), ("info", "started")]
    kept = [(entry.source, entry.text) for entry in monitor.console_log.entries()]
    assert kept[-2:] == [("a", "slow"), ("runtime", "started")]


def test_a_plugin_topic_is_part_of_the_source(monkeypatch):
    """A topic tags the line "<plugin>/<topic>", in the ROS log and in the console."""
    monitor = build(monkeypatch, Recorder("a", []))
    monitor._loaded["a"].ctx.log_info("changed", "chips")
    assert monitor.get_logger().lines[-1] == ("info", "[a/chips] changed")
    assert monitor.console_log.entries()[-1].source == "a/chips"


def test_the_node_logger_is_rclpys_own_until_the_console_log_exists(monkeypatch):
    """While the node is built, before the monitor set up its console log, rclpy's own logger answers."""
    rclpy_logger = object()
    monkeypatch.setattr(monitor_module.Node, "get_logger", lambda _node: rclpy_logger)
    assert HuskyMonitor.__new__(HuskyMonitor).get_logger() is rclpy_logger
