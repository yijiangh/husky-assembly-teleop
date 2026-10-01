"""
The ROS2 node that owns everything and runs the tick.

It owns the world state, kinematics, scene, viser server and robot interfaces, drives each plugin, and
is where the tick order is written down.

! No feature code here: every experiment, panel and diagnostic is a plugin.
"""

from __future__ import annotations

import asyncio
import gc
import signal
import sys
import traceback
from dataclasses import dataclass

import rclpy
from rclpy.executors import SingleThreadedExecutor, TimeoutException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from .plugin_api.concurrency import LoopWatchdog
from .config import MonitorConfig, config_from_ros_parameters
from .plugin_api.context import PluginContext
from .design_io.geometry import Geometry
from .world.kinematics import Kinematics
from .plugin_api.plugin import HuskyPlugin, load_plugins
from .robot_interface.robot import HuskyRobotInterface
from .robot_interface.connections import RosConnections
from .robot_interface.mocap import subscribe_mocap
from .world.scene import PluginScene, Scene, TrackedDescription
from .ui.visualization import Visualization
from .world.measured import TrackedObject, WorldState

#: Most ROS callbacks run in one tick; the rest wait, so a flood cannot starve the plugins.
ROS_CALLBACKS_PER_TICK = 5000


@dataclass
class _LoadedPlugin:
    """One plugin and everything the monitor tracks about it.

    Attributes:
        plugin: The plugin instance.
        ctx: Its context, kept here rather than on the plugin; plugins get it as a hook argument.
        stopped: Set once it or a dependency failed too often; no longer stepped or drawn, but loaded.
        set_up: Whether its setup was started; teardown runs only if so.
        closed: Whether its teardown has run.
        errors: Consecutive ticks in which it raised.
        last_error_tick: Index of the tick it last raised in.
    """

    plugin: HuskyPlugin
    ctx: PluginContext
    stopped: bool = False
    set_up: bool = False
    closed: bool = False
    errors: int = 0
    last_error_tick: int = -1

    @property
    def name(self) -> str:
        """str: The plugin's unique name."""
        return self.plugin.name


class HuskyMonitor(Node):
    """The monitor node. Plugins reach it only through their PluginContext.

    Threads:
      1. Main: one asyncio loop runs the tick, ROS callbacks, plugin hooks and tasks; all state lives here, unlocked.
      2. viser's: GUI callbacks may only call PluginContext.submit (via defer) and return.
      3. Workers (run_in_thread, planner executors): compute on copies such as the snapshot, never shared state.

    - ! ROS callbacks run only at the start of each tick, so a topic faster than queue depth x tick rate
      loses messages: give one whose every sample matters a deeper queue.
    - ! Construct it inside a running asyncio loop: plugin setup may start tasks.
    """

    def __init__(self):
        """Bring up state, scene, UI and plugins. `run` then starts ticking."""
        super().__init__("husky_monitor")

        self._config = config_from_ros_parameters(self)
        self._world = WorldState()
        self._loaded: dict[str, _LoadedPlugin] = {}
        # The mocap subscription of each tracked object, by object name.
        self._object_connections: dict[str, RosConnections] = {}

        # Monotonic tick counter.
        self._tick_index = 0

        # ? Used only to run ready callbacks when the tick says; see _pump_ros.
        self._executor = SingleThreadedExecutor(context=self.context)
        self._executor.add_node(self)

        self._kinematics: Kinematics | None = None
        self._scene = Scene()
        self._viz: Visualization | None = None
        self._watchdog: LoopWatchdog | None = None
        # Why each stopped plugin stopped, by name, for the banner.
        self._stop_reasons: dict[str, str] = {}
        self._shut_down = False

        try:
            self._build()
        except Exception:
            # Release what was already acquired, e.g. the viser server.
            self._release()
            raise

    def _build(self) -> None:
        """Construct the kinematics, the UI and the plugins. Called once, by __init__."""
        self._kinematics = Kinematics(self._config.robots, log_warn=self.log_warn)
        self._viz = Visualization(port=self._config.viser_port)

        for robot_config in self._config.robots:
            self._world.add_robot(HuskyRobotInterface(self, robot_config))
            self._viz.load_robot(robot_config)

        # Create plugins in dependency order
        for plugin in load_plugins(self._config.enabled_plugins, self.log_error):
            if plugin.experimental:
                self.log_warn(f"plugin {plugin.name!r} is experimental: its behaviour and API may change")
            ctx = PluginContext(
                name=plugin.name,
                monitor=self,
                view=self._viz.view_for(plugin.name),
                scene=PluginScene(self._scene, plugin.name,
                                  log_warn=lambda message, name=plugin.name: self.log_warn(f"[{name}] {message}")),
                kinematics=self._kinematics,
                dependencies=plugin.requires,
            )
            self._loaded[plugin.name] = _LoadedPlugin(plugin=plugin, ctx=ctx)

        # A plugin failing setup is stopped with its dependents; those are never set up.
        for loaded in self._loaded.values():
            if loaded.stopped:
                continue
            loaded.set_up = True
            try:
                loaded.plugin.setup(loaded.ctx)
            except Exception:
                self.log_error(f"plugin {loaded.name!r} failed in setup:\n{traceback.format_exc()}")
                self._stop(loaded, "failed in setup")

    # --- --- --- --- --- FOR PluginContext (documented there) --- --- --- --- ---

    @property
    def config(self) -> MonitorConfig:
        """MonitorConfig: Read-only run configuration."""
        return self._config

    @property
    def world(self) -> WorldState:
        """WorldState: Measured reality."""
        return self._world

    def now(self) -> float:
        """Current ROS time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def log_info(self, message: str) -> None:
        """Log `message` at info level."""
        self.get_logger().info(message)

    def log_warn(self, message: str) -> None:
        """Log `message` at warning level."""
        self.get_logger().warning(message)

    def log_error(self, message: str) -> None:
        """Log `message` at error level."""
        self.get_logger().error(message)

    def plugin(self, name: str) -> HuskyPlugin:
        """Look up a loaded plugin by name; KeyError if none is loaded."""
        try:
            return self._loaded[name].plugin
        except KeyError:
            raise KeyError(f"plugin {name!r} is not loaded") from None

    def track_object(self, name: str, mocap_id: int, geometry: Geometry | None = None,
                     touches: tuple[str, ...] = (), label: str = "") -> TrackedObject:
        """Register a tracked object, describe it in the scene, and subscribe to its mocap pose."""
        if name in self._world.tracked_objects:
            raise ValueError(f"object {name!r} is already tracked")
        obj = TrackedObject(name=name, mocap_id=mocap_id)
        connections = RosConnections(self)
        # * Stored exactly like a robot base's pose.
        subscribe_mocap(connections, mocap_id, obj, self.now)
        if geometry is not None and not geometry.collision:
            self.log_warn(f"tracked object {name!r} has no collision meshes: it is drawn but never collides")
        self._world.tracked_objects[name] = obj
        self._scene.tracked[name] = TrackedDescription(geometry=geometry, touches=tuple(touches), label=label)
        self._object_connections[name] = connections
        return obj

    def untrack_object(self, name: str) -> None:
        """Unsubscribe a tracked object and remove it from the registry."""
        connections = self._object_connections.pop(name, None)
        if connections is not None:
            connections.destroy_all()
        self._world.tracked_objects.pop(name, None)
        self._scene.tracked.pop(name, None)

    # --- --- --- --- --- RUN --- --- --- --- ---

    async def run(self) -> None:
        """Tick every `tick_period` until Ctrl-C or SIGTERM.

        ! A tick that runs late is not made up with a burst of fast ones.
        """
        loop = asyncio.get_running_loop()
        stop = asyncio.Event()
        signals = (signal.SIGINT, signal.SIGTERM)
        for number in signals:
            loop.add_signal_handler(number, stop.set)

        # Names this task in stall reports outside plugin hooks.
        asyncio.current_task().set_name("monitor tick")
        # * Everything alive now (modules, models, the viser scene) lives until shutdown: keep it out of later
        #   garbage collections, which hold the GIL on whichever thread runs them.
        gc.freeze()
        self._watchdog = LoopWatchdog(
            loop, limit=self._config.tick_period * self._config.slow_step_warn_ratio,
            repeat_after=self._config.slow_step_warn_period, log_warn=self.log_warn)
        try:
            due = loop.time()
            while not stop.is_set():
                await self._tick()
                due = max(due + self._config.tick_period, loop.time())
                await asyncio.sleep(due - loop.time())
        finally:
            # * A second Ctrl-C during shutdown now interrupts it.
            for number in signals:
                loop.remove_signal_handler(number)
            self._watchdog.stop()
            self.log_info("stopping: cancelling plugin tasks and tearing down")

    # --- --- --- --- --- TICK --- --- --- --- ---

    async def _tick(self) -> None:
        """One tick. The order below is deliberate; see the comment on each step."""
        self._tick_index += 1

        # 1. Run every ROS callback that is waiting, so the world is as fresh as it gets.
        self._pump_ros()

        # 2. Soft stop before any plugin can command new actions.
        if self._viz.take_stop_request():
            self._soft_stop()

        # 3. Fix forward kinematics for this tick, before anything reads a link pose.
        self._kinematics.update(self._world)

        # 4. Copy the whole world before any plugin runs, for planners and the 3D view.
        snapshot = self._scene.take_snapshot(self._world, self._kinematics, self._tick_index, self.now())

        # 5. Step each plugin, in dependency order: intents, then update. Each hook is timed for stall reports.
        for loaded in self._loaded.values():
            if not loaded.stopped:
                with self._watchdog.watching(f"plugin {loaded.name!r} step"):
                    self._step_plugin(loaded)

        # 6. Resume tasks waiting for this tick, in dependency order; yielding once lets each step before the draw.
        #    ? Stopped plugins too: their cancelled tasks may still be cleaning up.
        for loaded in self._loaded.values():
            loaded.ctx._wake_tick_waiters()
        await asyncio.sleep(0)

        # 7. Draw. While frozen (for text selection), only robots and the scene are drawn, not plugins.
        with self._viz.atomic():
            self._viz.draw(snapshot)
            for loaded in self._loaded.values() if not self._viz.frozen else ():
                if loaded.stopped:
                    continue
                try:
                    with self._watchdog.watching(f"plugin {loaded.name!r} draw"):
                        loaded.plugin.draw(loaded.ctx)
                except Exception:
                    self._plugin_failed(loaded, "draw")

    def _pump_ros(self) -> None:
        """Run every ROS callback that is ready: subscriptions, service answers, timers.

        A callback that raises is logged and dropped.
        """
        budget = ROS_CALLBACKS_PER_TICK - self._drain_subscriptions()
        for _ in range(max(budget, 0)):
            try:
                handler, _entity, _node = self._executor.wait_for_ready_callbacks(timeout_sec=0.0)
            except TimeoutException:
                return  # nothing more is ready
            handler()
            if handler.exception() is not None:
                self.log_error(f"ROS callback failed:\n{handler.exception()!r}")
        self.log_warn(f"more than {ROS_CALLBACKS_PER_TICK} ROS callbacks were ready in one tick; "
                      f"the rest run next tick")

    def _drain_subscriptions(self) -> int:
        """Run the callback of every queued message of every subscription, oldest first.

        ! The executor takes only one message per subscription per tick: a 100 Hz topic would be read at
          the tick rate, from the old end of its queue (measured: about 1 s behind with depth 100).

        Returns:
            int: How many messages were taken, at most ROS_CALLBACKS_PER_TICK.
        """
        taken = 0
        for subscription in self.subscriptions:
            while taken < ROS_CALLBACKS_PER_TICK:
                with subscription.handle:
                    message_and_info = subscription.handle.take_message(subscription.msg_type, subscription.raw)
                if message_and_info is None:
                    break  # this queue is empty
                taken += 1
                try:
                    subscription.callback(message_and_info[0])
                except Exception as error:
                    self.log_error(f"ROS callback of {subscription.topic_name} failed:\n{error!r}")
        return taken

    def _soft_stop(self) -> None:
        """Stop every robot and cancel every plugin task.

        In the core so it works whichever plugins are loaded.
        """
        self.log_warn("SOFT STOP: stopping every robot and cancelling every plugin task")
        for robot in self._world.robots.values():
            robot.soft_stop()
        for loaded in self._loaded.values():
            loaded.ctx._cancel_all_tasks()

    def _step_plugin(self, loaded: _LoadedPlugin) -> None:
        """Give one plugin its turn: count failed tasks, run intents, then update.

        Each part is caught on its own, so one failing still lets the others run unless the plugin got stopped.
        """
        if loaded.ctx._take_task_failures():
            self._plugin_failed(loaded, "a task", details="")
        # Intents first, so UI work is applied before the update that reacts to it.
        if not loaded.stopped:
            try:
                loaded.ctx._drain_intents()
            except Exception:
                self._plugin_failed(loaded, "intents")
        if not loaded.stopped:
            try:
                loaded.plugin.update(loaded.ctx)
            except Exception:
                self._plugin_failed(loaded, "update")

    # --- --- --- --- --- ERRORS --- --- --- --- ---

    def _plugin_failed(self, loaded: _LoadedPlugin, hook: str, details: str | None = None) -> None:
        """Log that a plugin raised in `hook`, and stop it if it keeps doing so.

        Args:
            loaded: The plugin whose work raised.
            hook: Which part was running, for the log message.
            details: Extra log text; None logs the exception being handled, "" nothing.
        """
        if details is None:
            details = traceback.format_exc()
        self.log_error(f"plugin {loaded.name!r} failed in {hook}" + (f":\n{details}" if details else ""))
        if loaded.last_error_tick == self._tick_index:
            return  # already counted this tick
        # Consecutive only if it also failed in the tick immediately before.
        consecutive = loaded.last_error_tick == self._tick_index - 1
        loaded.errors = loaded.errors + 1 if consecutive else 1
        loaded.last_error_tick = self._tick_index

        if loaded.errors >= self._config.max_plugin_errors:
            self._stop(loaded, f"failed in {hook}, {loaded.errors} ticks in a row")

    def _stop(self, loaded: _LoadedPlugin, reason: str) -> None:
        """Stop a failing plugin and every plugin that depends on it, and show the monitor as broken.

        Only their tasks are cancelled; their UI stays until shutdown. There is no recovery short of a restart.

        Args:
            loaded: The plugin that failed.
            reason: Why, for the banner and the log.
        """
        to_stop = {loaded.name: reason}
        # Plugins are in dependency order, so one pass also catches dependents of dependents.
        for other in self._loaded.values():
            if other.stopped or other.name in to_stop:
                continue
            broken = [name for name in other.plugin.requires if name in to_stop]
            if broken:
                to_stop[other.name] = f"stopped because it depends on {broken[0]!r}"

        for name, why in to_stop.items():
            stopping = self._loaded[name]
            stopping.stopped = True
            stopping.ctx._close()
            self._stop_reasons[name] = why
            self.log_error(f"STOPPED plugin {name!r}: {why}. The monitor is broken; restart it.")
        self._viz.show_broken(self._stop_reasons)

    # --- --- --- --- --- SHUTDOWN --- --- --- --- ---

    async def shutdown(self) -> None:
        """Cancel every plugin's tasks, let them clean up, then tear everything down.

        Reverse dependency order, so cleanup can still use dependencies; ROS keeps running so a final hold
        takes effect. Safe to call twice.
        """
        if self._shut_down:
            return
        self._shut_down = True

        keep_alive = asyncio.get_running_loop().create_task(self._keep_alive(), name="shutdown ticks")
        try:
            for loaded in reversed(self._loaded.values()):
                tasks = loaded.ctx._close()
                if tasks:
                    _done, pending = await asyncio.wait(tasks, timeout=self._config.shutdown_grace)
                    if pending:
                        self.log_warn(f"plugin {loaded.name!r} has {len(pending)} task(s) still "
                                      f"cleaning up after {self._config.shutdown_grace} s; "
                                      f"tearing down without them")
                self._close(loaded)
        finally:
            keep_alive.cancel()
            self._release()

    async def _keep_alive(self) -> None:
        """A reduced tick during shutdown: ROS, kinematics, the world copy and tick waiters, no hooks."""
        while True:
            self._pump_ros()
            self._tick_index += 1
            self._kinematics.update(self._world)
            self._scene.take_snapshot(self._world, self._kinematics, self._tick_index, self.now())
            for loaded in self._loaded.values():
                loaded.ctx._wake_tick_waiters()
            await asyncio.sleep(self._config.tick_period)

    def _close(self, loaded: _LoadedPlugin) -> None:
        """Run a plugin's teardown hook, if it was set up, and take its UI away. Once."""
        if loaded.closed:
            return
        loaded.closed = True
        if loaded.set_up:
            try:
                loaded.plugin.teardown(loaded.ctx)
            except Exception:
                self.log_error(f"plugin {loaded.name!r} failed in teardown:\n"
                               f"{traceback.format_exc()}")
        loaded.ctx.view.clear()
        # * Its bodies go with it, so they stop being obstacles for everyone else.
        self._scene.remove_prefix(f"{loaded.name}/")

    def _release(self) -> None:
        """Release plugins, UI and the executor, in reverse build order.

        ! Must run on Ctrl-C: viser's server thread outlives rclpy.shutdown(). Safe after a partial construction.
        """
        for loaded in reversed(self._loaded.values()):
            loaded.ctx._close()
            self._close(loaded)
        if self._viz is not None:
            self._viz.stop()
        self._executor.shutdown()


# --- --- --- --- --- MAIN --- --- --- --- ---

#: Seconds a busy thread may keep the GIL while another waits for it (Python's default is 5 ms).
#: ? Each time the main thread gives up the GIL (a ROS wait, a socket write), it can wait this long to get it
#:   back while a worker (planning, loading) runs Python: at 5 ms a 0.1 ms tick took ~390 ms, at 1 ms ~70 ms.
GIL_SWITCH_INTERVAL = 0.001


async def _run_monitor() -> None:
    """Build the monitor, tick until stopped, then shut it down cleanly."""
    monitor = HuskyMonitor()
    try:
        await monitor.run()
    finally:
        await monitor.shutdown()
        monitor.destroy_node()


def main(args: list[str] | None = None) -> None:
    """Run the monitor until interrupted, reading sys.argv when `args` is None. Configured by ROS parameters."""
    # ! No rclpy signal handler: it would shut ROS down before plugin tasks send their final commands.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    sys.setswitchinterval(GIL_SWITCH_INTERVAL)
    try:
        asyncio.run(_run_monitor())
    except KeyboardInterrupt:
        pass  # Ctrl-C during startup, or a second one during shutdown
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
