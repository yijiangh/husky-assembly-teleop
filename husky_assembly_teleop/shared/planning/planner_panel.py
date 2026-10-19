"""
Planner panels: target, Plan/Play/Clear, a time slider, Commit and ghosts.

`PlannerPanel` is a separate plugin that drives a headless planner (`planner_service.PlannerService`) through its API.
Subclasses set `planner`, supply `make_request`, `target_change`, `show_target` and `show_path`, and build their
panel from `_add_path_controls`, `_add_ghosts` and the `_draw_*` helpers.
! The planning world is the planner's, created on the worker thread; the base only closes it at teardown.
! Drive the planner through its public API (set target, plan, cancel, results) from the panel wherever you can,
  so those paths stay ready for other users such as an executor; only debug drawing and internal state reach inside.
"""

from __future__ import annotations

import asyncio
from html import escape
from typing import TYPE_CHECKING, Iterable

import viser

from ...config import find_robot_serial
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin
from ...ui.ghost import RecentUse, RobotGhost, robot_ghosts
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, block, chip, note, section
from .planner_contract import Plan, PlannerContract

if TYPE_CHECKING:
    from ...config import RobotConfig
    from ..process_contract import ProcessContract, Selection

#: Time slider resolution, seconds.
SLIDER_STEP = 0.05


#: Chip text and colour of each state of the planner's latest plan, and of no plan.
_PLAN_CHIPS = {None: ("no plan", NONE), "planning": ("planning…", BUSY), "ready": ("plan ready", OK),
               "stale": ("plan stale: replan", BUSY), "sent": ("sent", OK), "failed": ("no plan found", FAIL),
               "cancelled": ("cancelled", NONE)}


class PlannerPanel(HuskyPlugin):
    """A planner's panel: target, Plan/Play/Clear, time slider, Commit and ghosts. Subclass it; do not register it.

    Plan, Clear and Commit go through the planner's public API (`PlannerContract`); debug drawing may read the
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
        #: How the panel's target differs from the ready plan's, or "": the chip then asks for a replan.
        self._target_moved = ""
        #: When the operator last used this panel; ghosts show only while it counts (`_add_ghosts`).
        self._used: RecentUse | None = None
        #: Nothing to plan for (e.g. no robots configured): the panel says why, and the hooks do nothing.
        self.idle = False
        self._api: PlannerContract | None = None

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

    def _process_step(self, ctx: PluginContext) -> tuple[Selection, str, str] | None:
        """The process plugin's selected movement, and the configured robot it is for.

        Returns:
            tuple[Selection, str, str] | None: The selection, the robot's serial and its process name;
                None if there is none, after saying why in the panel.
        """
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        if selection is None:
            self._say("no process loaded in the process plugin", failed=True)
            return None
        label = selection.movement.id
        if not selection.action.robot:
            self._say(f"{label} is placed by hand: no robot to plan for", failed=True)
            return None
        robot = selection.process.robots[selection.action.robot]
        serial = find_robot_serial(ctx.config.robots, robot.serial or "")
        if serial is None:
            self._say(f"{label} is for {robot.name}, which is not configured", failed=True)
            return None
        return selection, serial, robot.name

    def _say(self, message: str, failed: bool = False) -> None:
        """Set the one line of feedback under the buttons."""
        self._message, self._message_failed = message, failed

    # --- --- --- --- --- TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Take the planner's news into the message line, advance playback, and note a target moved since planning."""
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
        moved = self.target_change(ctx, plan) if plan is not None and plan.state == "ready" else ""
        if moved and moved != self._target_moved:
            self._say(moved)
        self._target_moved = moved

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
        if self._target_moved:
            return chip("target moved: replan", BUSY)
        return chip(*_PLAN_CHIPS[plan.state if plan is not None else None])

    def _message_html(self) -> str:
        """The last message, one line, red if it was a failure; the full text on hover."""
        return (f'<div title="{escape(self._message, quote=True)}" '
                f'style="font-size:11px;color:{FAIL if self._message_failed else NONE};'
                f'white-space:nowrap;overflow:hidden;text-overflow:ellipsis">'
                f'{escape(self._message) or "&nbsp;"}</div>')
