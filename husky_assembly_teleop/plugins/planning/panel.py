"""
PlannerPlugin: plan to a target on a worker thread, preview the path, commit it.

Plan states: none -> planning -> ready -> sent; a ready plan goes stale when the robot or target moves.

* Subclasses implement `make_search`, `stale_reason`, `send_plan`, `show_target` and `show_path`,
  and build their panel from `_add_path_controls`, `_add_ghosts` and the `_draw_*` helpers.
! The planning world is the subclass's, created on the worker thread; the base only closes it at teardown.
"""

from __future__ import annotations

import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from html import escape
from typing import TYPE_CHECKING, Any, Callable, Iterable, Protocol

import viser

from ...config import find_robot_serial
from async_timeout import timeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin
from ...ui.ghost import RecentUse, RobotGhost, robot_ghosts
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, chip, section
from .path import TimedPath

if TYPE_CHECKING:
    from ...config import RobotConfig
    from .search import PlanResult

#: Time slider resolution, seconds.
SLIDER_STEP = 0.05
#: Seconds to wait for the worker beyond the search's own time limit.
WAIT_MARGIN = 5.0

#: Chip text and colour of each plan state.
_STATE_CHIPS = {"none": ("no plan", NONE), "planning": ("planning…", BUSY), "ready": ("plan ready", OK),
                "stale": ("plan stale: replan", BUSY), "sent": ("sent (stub)", OK)}


class PlanningWorld(Protocol):
    """A subclass's planning world, as far as the base needs it."""

    def close(self) -> None:
        """Disconnect the world. On the worker, once no search is running."""


@dataclass
class Search:
    """One planning request, built by the subclass on the main thread (`make_search`).

    Attributes:
        serial: The robot the plan is for.
        label: Task name, e.g. "plan a200-0806".
        message: Shown in the panel while planning.
        work: Syncs the world and searches, on the worker; must stop soon after its `abort` is set.
        time_limit: Seconds to wait for `work`, margin included.
        accepted: Main thread, when a path was found: returns the panel message and whether it is a warning.
        finished: Main thread, after `work` returned, path or not.
    """

    serial: str
    label: str
    message: str
    work: Callable[[], PlanResult]
    time_limit: float
    accepted: Callable[[PlanResult], tuple[str, bool]]
    finished: Callable[[], None] = lambda: None


class PlannerPlugin(HuskyPlugin):
    """Target, plan on a worker thread, preview, commit. Subclass it; do not register it."""

    requires = ("cell",)
    #: Prefix of the worker thread's name, e.g. "base-plan".
    worker_name = "plan"

    def __init__(self):
        """Start with no robot chosen, no target and no plan."""
        #: The robot being planned for, by serial.
        self.serial: str | None = None
        #: Whether a target has been given yet; until then no target ghost is drawn.
        self._has_target = False
        #: The current plan, and the robot it is for.
        self.path: TimedPath | None = None
        self._path_serial: str | None = None
        #: The plan's state: "none", "planning", "ready", "stale" or "sent".
        self.plan_state = "none"
        #: Time shown on the slider and the ghost, seconds into the path.
        self._t = 0.0
        self._playing = False
        self._last_tick: float | None = None
        #: One line of feedback on the last action, and whether it was a failure.
        self._message, self._message_failed = "", False
        #: The running search, and the flag that ends its worker early.
        self._plan_task: asyncio.Task | None = None
        self._abort = threading.Event()
        # One worker: it owns the planning world, so searches run one after another.
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix=self.worker_name)
        #: The subclass's planning world. ! Worker thread only, from its first use on.
        self._world: PlanningWorld | None = None
        #: When the operator last used this panel; ghosts show only while it counts (`_add_ghosts`).
        self._used: RecentUse | None = None

    # --- --- --- --- --- FOR THE SUBCLASS TO SUPPLY --- --- --- --- ---

    def make_search(self, ctx: PluginContext, abort: threading.Event) -> Search:
        """Build the search from the current robot to the current target. Main thread.

        Args:
            ctx: This plugin's context. Take `ctx.scene.snapshot` here, not in `work`.
            abort: The flag `work` must watch.

        Returns:
            Search: The request.
        """
        raise NotImplementedError

    def stale_reason(self, ctx: PluginContext) -> str:
        """Why the ready plan no longer fits, e.g. "the robot moved since planning"; "" if it still does."""
        raise NotImplementedError

    def send_plan(self, ctx: PluginContext) -> tuple[bool, str]:
        """Send the ready plan to the robot.

        Returns:
            tuple[bool, str]: Whether it was taken, and the panel message.
        """
        raise NotImplementedError

    def show_target(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """Pose the chosen robot's target ghost at the target."""
        raise NotImplementedError

    def show_path(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """Pose the planned robot's path ghost at `self.path.sample(self._t)`."""
        raise NotImplementedError

    def forget_plan(self) -> None:
        """Drop anything the subclass kept about the plan (`Search.accepted`). Optional."""

    # --- --- --- --- --- SETUP HELPERS --- --- --- --- ---

    def _add_path_controls(self, ctx: PluginContext, gui: viser.GuiApi, plan_hint: str, commit_hint: str) -> None:
        """Add the "path" section: Plan/Play/Clear, the time slider and Commit, with their callbacks.

        Args:
            ctx: This plugin's context.
            gui: GUI api, inside `ctx.view.ui()`.
            plan_hint: Hover text of the Plan/Play/Clear buttons.
            commit_hint: Hover text of Commit.
        """
        gui.add_html(section("path", SECTION_CTRL))
        plan = gui.add_button_group("Path", ["Plan", "Play", "Clear"], hint=plan_hint)
        self._slider = gui.add_slider("t s", min=0.0, max=1.0, step=SLIDER_STEP, initial_value=0.0,
                                      hint="Scrub through the planned path")
        commit = gui.add_button("Commit", color="blue", icon=viser.Icon.SEND, hint=commit_hint)
        plan.on_click(ctx.defer_value("path button", lambda label: self._on_path(ctx, label)))
        self._slider.on_update(ctx.defer_value("scrub", lambda value: self._scrub(float(value))))
        commit.on_click(ctx.defer("commit", lambda: self._commit(ctx)))

    def _add_ghosts(self, ctx: PluginContext, robots: Iterable[RobotConfig]) -> None:
        """Build a target ghost and a path ghost per robot, shown only while the panel is in use.

        ! Call in setup (loading meshes is too slow for a tick). Ghosts live under "ghosts/": keep subclass
          nodes elsewhere, since removing a node removes everything below it.
        """
        self._used = RecentUse(ctx.config.ghost_timeout)
        robots = list(robots)
        self._target_ghosts = robot_ghosts(ctx, robots, "target")
        self._path_ghosts = robot_ghosts(ctx, robots, "path")

    def teardown(self, ctx: PluginContext) -> None:
        """End any search, then close the planning world on its worker."""
        self._abort.set()  # first, so the shutdown wait is short
        if self._world is not None:
            self._executor.submit(self._world.close)
        self._executor.shutdown(wait=True)

    # --- --- --- --- --- COMMANDS (intents, on the main thread) --- --- --- --- ---

    def _on_path(self, ctx: PluginContext, label: str) -> None:
        """Plan, play or pause, or clear.

        Args:
            ctx: This plugin's context.
            label: The button clicked.
        """
        self._used.touch()
        if label == "Plan":
            self._plan(ctx)
        elif label == "Play":
            if self.path is None:
                ctx.log_info("play ignored: nothing planned")
                return
            if not self._playing and self._t >= self.path.duration:
                self._t = 0.0  # play again from the start
            self._playing = not self._playing
        elif label == "Clear":
            self._clear()

    def _plan(self, ctx: PluginContext) -> None:
        """Start a search to the target, unless one is running or there is no target."""
        if not self._has_target:
            self._say("set a target first", failed=True)
            return
        if self.plan_state == "planning":
            ctx.log_info("plan ignored: already planning")
            return
        self._clear()
        # A fresh flag per search, so an old search's abort cannot end this one.
        self._abort = threading.Event()
        search = self.make_search(ctx, self._abort)
        self.plan_state = "planning"
        self._say(search.message)
        self._plan_task = ctx.spawn(search.label, self._run(search, self._abort))

    async def _run(self, search: Search, abort: threading.Event) -> None:
        """Run one search on the worker and take its result on the main thread; no path is reported, not raised."""
        try:
            async with timeout(search.time_limit):
                result = await asyncio.get_running_loop().run_in_executor(self._executor, search.work)
        except asyncio.TimeoutError:
            self.plan_state = "none"
            self._say(f"timed out after {search.time_limit:.0f}s waiting for the planner", failed=True)
            return
        finally:
            # ! Threads cannot be interrupted: always tell the worker to stop.
            abort.set()

        search.finished()
        if result.path is None:
            self.plan_state = "none"
            self._say(f"no plan: {result.reason}", failed=True)
            return
        self.path, self._path_serial, self.plan_state = result.path, search.serial, "ready"
        self._t, self._playing = 0.0, False
        self._say(*search.accepted(result))

    def _scrub(self, t: float) -> None:
        """Show the path at time `t`."""
        # ! Keep this check: draw writes the slider back, which would otherwise loop.
        if abs(t - self._t) > 1e-9:
            self._t = t
            self._used.touch()

    def _commit(self, ctx: PluginContext) -> None:
        """Send the plan, if it is still good."""
        self._used.touch()
        if self.plan_state != "ready":
            self._say(f"nothing to commit: the plan is {self.plan_state}", failed=True)
            return
        sent, message = self.send_plan(ctx)
        if sent:
            self.plan_state = "sent"
        self._say(message, failed=not sent)

    def _clear(self) -> None:
        """Forget the plan, and end a search in progress."""
        if self._plan_task is not None and not self._plan_task.done():
            self._plan_task.cancel()
            self._abort.set()  # stop the worker now, not at the task's next step
            self._say("planning cancelled")
        self._plan_task = None
        self.path, self._path_serial, self.plan_state = None, None, "none"
        self._t, self._playing = 0.0, False
        self.forget_plan()

    def _cell_step(self, ctx: PluginContext) -> tuple[Any, str, str] | None:
        """The cell plugin's selected step, and the configured robot it is for.

        Returns:
            tuple[Any, str, str] | None: The step, the robot's serial and its design name;
                None if there is none, after saying why in the panel.
        """
        cell = ctx.require("cell")
        step = cell.step
        if step is None:
            self._say("no design loaded in the cell plugin", failed=True)
            return None
        robot = cell.design.design.robots[step.action.robot]
        serial = find_robot_serial(ctx.config.robots, robot.serial or "")
        if serial is None:
            self._say(f"{step.label} is for {robot.name}, which is not configured", failed=True)
            return None
        return step, serial, robot.name

    def _say(self, message: str, failed: bool = False) -> None:
        """Set the one line of feedback under the buttons."""
        self._message, self._message_failed = message, failed

    # --- --- --- --- --- TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Advance playback, and mark a ready plan stale when `stale_reason` says so."""
        now = ctx.now()
        elapsed = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        if self.path is None:
            return
        if self._playing:
            self._t = min(self._t + elapsed, self.path.duration)
            self._playing = self._t < self.path.duration
        # * Only a ready plan goes stale; a sent plan stays sent.
        if self.plan_state == "ready":
            reason = self.stale_reason(ctx)
            if reason:
                self.plan_state = "stale"
                self._say(reason)

    # --- --- --- --- --- DRAW HELPERS --- --- --- --- ---

    def _draw_slider(self) -> None:
        """Fit the time slider to the plan and show the current time."""
        duration = 0.0 if self.path is None else self.path.duration
        self._slider.max = max(duration, SLIDER_STEP)
        self._slider.value = round(self._t, 3)

    def _draw_ghosts(self, ctx: PluginContext) -> None:
        """Show the target and path ghosts while the panel is in use; hide every other ghost."""
        in_use = self._used.active
        target = self.serial if in_use and self._has_target else None
        planned = self._path_serial if in_use and self.path is not None else None
        for serial, ghost in self._target_ghosts.items():
            self.show_target(ctx, ghost) if serial == target else ghost.hide()
        for serial, ghost in self._path_ghosts.items():
            self.show_path(ctx, ghost) if serial == planned else ghost.hide()

    def _plan_chip(self) -> str:
        """The plan state as a chip."""
        return chip(*_STATE_CHIPS[self.plan_state])

    def _message_html(self) -> str:
        """The last message, one line, red if it was a failure."""
        return (f'<div style="font-size:11px;color:{FAIL if self._message_failed else NONE};'
                f'white-space:nowrap;overflow:hidden;text-overflow:ellipsis">'
                f'{escape(self._message) or "&nbsp;"}</div>')
