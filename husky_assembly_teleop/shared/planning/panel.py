"""
Planner panels: target, Plan/Play/Clear, a time slider, Commit and ghosts.

- `PlannerPanel` is a separate plugin that drives a headless planner (`service.PlannerService`) through its API.
  Subclasses set `planner`, supply `make_request`, `target_change`, `show_target` and `show_path`, and build their
  panel from `_add_path_controls`, `_add_ghosts` and the `_draw_*` helpers.
- `PlannerPlugin` is the older panel and planner in one; it goes once both planners are on `PlannerPanel`.
! The planning world is the planner's, created on the worker thread; the base only closes it at teardown.
! Drive the planner through its public API (set target, plan, cancel, results) from the panel wherever you can,
  so those paths stay ready for other users such as an executor; only debug drawing and internal state reach inside.
"""

from __future__ import annotations

import asyncio
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from html import escape
from typing import TYPE_CHECKING, Callable, Iterable

import viser

from ...config import find_robot_serial
from async_timeout import timeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin
from ...ui.ghost import RecentUse, RobotGhost, robot_ghosts
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, block, chip, note, section
from .path import TimedPath
# * WAIT_MARGIN and with_guesses are re-exported for the planners still on PlannerPlugin.
from .service import WAIT_MARGIN, Plan, PlannerPluginApi, PlanningWorld, with_guesses  # noqa: F401

if TYPE_CHECKING:
    from ...config import RobotConfig
    from ..process import ProcessPluginApi, Step
    from .search import PlanResult

#: Time slider resolution, seconds.
SLIDER_STEP = 0.05

#: Chip text and colour of each plan state.
_STATE_CHIPS = {"none": ("no plan", NONE), "planning": ("planning…", BUSY), "ready": ("plan ready", OK),
                "stale": ("plan stale: replan", BUSY), "sent": ("sent", OK)}


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

    requires = ("process",)
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
        #: Nothing to plan for (e.g. no robots configured): the panel says why, and the hooks do nothing.
        self.idle = False

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

    def _set_up_idle(self, ctx: PluginContext, why: str) -> None:
        """Instead of the panel, say why there is nothing to plan for, and stay `idle`. Call from setup.

        * Raising instead would stop the plugin and mark the whole monitor broken, e.g. when run without robots.

        Args:
            ctx: This plugin's context.
            why: One line, e.g. "no robots configured".
        """
        self.idle = True
        with ctx.view.ui() as gui:
            gui.add_html(block(note(escape(why))))

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

    def _process_step(self, ctx: PluginContext) -> tuple[Step, str, str] | None:
        """The process plugin's selected step, and the configured robot it is for.

        Returns:
            tuple[Step, str, str] | None: The step, the robot's serial and its process name;
                None if there is none, after saying why in the panel.
        """
        process: ProcessPluginApi = ctx.require("process")
        selection = process.selection
        if selection is None:
            self._say("no process loaded in the process plugin", failed=True)
            return None
        step = selection.step
        if not step.action.robot:
            self._say(f"{step.label} is placed by hand: no robot to plan for", failed=True)
            return None
        robot = selection.loaded.process.robots[step.action.robot]
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
        if self.idle:
            return
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
        """The last message, one line, red if it was a failure; the full text on hover."""
        return (f'<div title="{escape(self._message, quote=True)}" '
                f'style="font-size:11px;color:{FAIL if self._message_failed else NONE};'
                f'white-space:nowrap;overflow:hidden;text-overflow:ellipsis">'
                f'{escape(self._message) or "&nbsp;"}</div>')


#: Chip text and colour of each state of the planner's latest plan, and of no plan.
_PLAN_CHIPS = {None: ("no plan", NONE), "planning": ("planning…", BUSY), "ready": ("plan ready", OK),
               "stale": ("plan stale: replan", BUSY), "sent": ("sent", OK), "failed": ("no plan found", FAIL),
               "cancelled": ("cancelled", NONE)}


class PlannerPanel(HuskyPlugin):
    """A planner's panel: target, Plan/Play/Clear, time slider, Commit and ghosts. Subclass it; do not register it.

    Plan, Clear and Commit go through the planner's public API (`PlannerPluginApi`); debug drawing may read the
    planner itself.
    """

    #: The planner plugin this panel drives. ! Also list it in `requires`.
    planner = ""

    def __init__(self):
        """Start with no robot chosen and no target."""
        #: The robot whose target is being set, by serial.
        self.serial: str | None = None
        #: Whether a target has been given yet; until then no target ghost is drawn.
        self._has_target = False
        #: Time shown on the slider and the ghost, seconds into the path.
        self._t = 0.0
        self._playing = False
        self._last_tick: float | None = None
        #: One line of feedback on the last action, and whether it was a failure.
        self._message, self._message_failed = "", False
        #: The planner's plan whose message the panel shows; a new or changed plan replaces the message.
        self._shown: Plan | None = None
        #: When the operator last used this panel; ghosts show only while it counts (`_add_ghosts`).
        self._used: RecentUse | None = None
        #: Nothing to plan for (e.g. no robots configured): the panel says why, and the hooks do nothing.
        self.idle = False
        self._api: PlannerPluginApi | None = None

    # --- --- --- --- --- FOR THE SUBCLASS TO SUPPLY --- --- --- --- ---

    def make_request(self, ctx: PluginContext):
        """The planner's request for the current robot and target, or None after `_say`ing why there is none."""
        raise NotImplementedError

    def target_change(self, ctx: PluginContext, plan: Plan) -> str:
        """How the panel's target differs from the plan's, e.g. "the target changed since planning"; "" if not."""
        raise NotImplementedError

    def show_target(self, ctx: PluginContext, ghost: RobotGhost) -> None:
        """Pose the chosen robot's target ghost at the target."""
        raise NotImplementedError

    def show_path(self, ctx: PluginContext, ghost: RobotGhost, plan: Plan) -> None:
        """Pose the planned robot's path ghost at `plan.path.sample(self._t)`."""
        raise NotImplementedError

    # --- --- --- --- --- SETUP HELPERS --- --- --- --- ---

    def _connect(self, ctx: PluginContext) -> None:
        """Take the planner's API. ! Call first in setup."""
        self._api = ctx.require(self.planner)

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

    def _set_up_idle(self, ctx: PluginContext, why: str) -> None:
        """Instead of the panel, say why there is nothing to plan for, and stay `idle`. Call from setup.

        * Raising instead would stop the plugin and mark the whole monitor broken, e.g. when run without robots.
        """
        self.idle = True
        with ctx.view.ui() as gui:
            gui.add_html(block(note(escape(why))))

    # --- --- --- --- --- COMMANDS (intents, on the main thread) --- --- --- --- ---

    def _on_path(self, ctx: PluginContext, label: str) -> None:
        """Plan, play or pause, or clear."""
        self._used.touch()
        plan = self._api.plan
        if label == "Plan":
            self._plan(ctx)
        elif label == "Play":
            if plan is None or plan.path is None:
                ctx.log_info("play ignored: nothing planned")
                return
            if not self._playing and self._t >= plan.path.duration:
                self._t = 0.0  # play again from the start
            self._playing = not self._playing
        elif label == "Clear":
            self._api.clear()

    def _plan(self, ctx: PluginContext) -> None:
        """Ask the planner for a plan to the target, unless one is being planned or there is no target."""
        if not self._has_target:
            self._say("set a target first", failed=True)
            return
        plan = self._api.plan
        if plan is not None and plan.state == "planning":
            ctx.log_info("plan ignored: already planning")
            return
        request = self.make_request(ctx)
        if request is not None:
            self._api.request_plan(request)

    def _scrub(self, t: float) -> None:
        """Show the path at time `t`."""
        # ! Keep this check: draw writes the slider back, which would otherwise loop.
        if abs(t - self._t) > 1e-9:
            self._t = t
            self._used.touch()

    def _commit(self, ctx: PluginContext) -> None:
        """Ask the planner to send its plan, if the panel's target is still the plan's."""
        self._used.touch()
        plan = self._api.plan
        if plan is None or plan.state != "ready":
            self._say(f"nothing to commit: the plan is {plan.state if plan else 'missing'}", failed=True)
            return
        change = self.target_change(ctx, plan)
        if change:
            self._say(f"not committed: {change}; replan", failed=True)
            return
        ctx.spawn(f"commit plan {plan.id}", self._report(self._api.commit(plan.id)))

    async def _report(self, sent: asyncio.Future[tuple[bool, str]]) -> None:
        """Show how a commit went."""
        taken, message = await sent
        self._say(message, failed=not taken)

    def _process_step(self, ctx: PluginContext) -> tuple[Step, str, str] | None:
        """The process plugin's selected step, and the configured robot it is for.

        Returns:
            tuple[Step, str, str] | None: The step, the robot's serial and its process name;
                None if there is none, after saying why in the panel.
        """
        process: ProcessPluginApi = ctx.require("process")
        selection = process.selection
        if selection is None:
            self._say("no process loaded in the process plugin", failed=True)
            return None
        step = selection.step
        if not step.action.robot:
            self._say(f"{step.label} is placed by hand: no robot to plan for", failed=True)
            return None
        robot = selection.loaded.process.robots[step.action.robot]
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
        """Take the planner's news into the message line, and advance playback."""
        if self.idle:
            return
        now = ctx.now()
        elapsed = 0.0 if self._last_tick is None else now - self._last_tick
        self._last_tick = now
        plan = self._api.plan
        if plan is not self._shown:
            if plan is not None and (self._shown is None or plan.id != self._shown.id):
                self._t, self._playing = 0.0, False
            self._shown = plan
            if plan is not None:
                self._say(plan.message, failed=plan.state == "failed" or plan.warning)
        if self._playing and plan is not None and plan.path is not None:
            self._t = min(self._t + elapsed, plan.path.duration)
            self._playing = self._t < plan.path.duration

    # --- --- --- --- --- DRAW HELPERS --- --- --- --- ---

    def _draw_slider(self) -> None:
        """Fit the time slider to the plan and show the current time."""
        plan = self._api.plan
        duration = 0.0 if plan is None or plan.path is None else plan.path.duration
        self._slider.max = max(duration, SLIDER_STEP)
        self._slider.value = round(self._t, 3)

    def _draw_ghosts(self, ctx: PluginContext) -> None:
        """Show the target and path ghosts while the panel is in use; hide every other ghost."""
        in_use = self._used.active
        plan = self._api.plan
        target = self.serial if in_use and self._has_target else None
        planned = plan.request.serial if in_use and plan is not None and plan.path is not None else None
        for serial, ghost in self._target_ghosts.items():
            if serial == target:
                self.show_target(ctx, ghost)
            else:
                ghost.hide()
        for serial, ghost in self._path_ghosts.items():
            if serial == planned:
                self.show_path(ctx, ghost, plan)
            else:
                ghost.hide()

    def _plan_chip(self) -> str:
        """The state of the planner's latest plan as a chip."""
        plan = self._api.plan
        return chip(*_PLAN_CHIPS[plan.state if plan is not None else None])

    def _message_html(self) -> str:
        """The last message, one line, red if it was a failure; the full text on hover."""
        return (f'<div title="{escape(self._message, quote=True)}" '
                f'style="font-size:11px;color:{FAIL if self._message_failed else NONE};'
                f'white-space:nowrap;overflow:hidden;text-overflow:ellipsis">'
                f'{escape(self._message) or "&nbsp;"}</div>')
