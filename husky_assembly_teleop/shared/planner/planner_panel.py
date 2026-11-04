"""
Planner panels: target, Plan/Play/Clear, a time slider, Commit and ghosts.

`PlannerPanel` is a separate plugin that drives a headless planner (`planner_service.PlannerService`) through its API.
Subclasses set `planner`, supply `make_request`, `target_change`, `show_target` and `show_path`, and build their
panel from `_add_path_controls`, `_add_ghosts` and the `_draw_*` helpers. Commit's Clearance box sets whether the
motion runs under a clearance; `clearance_chips` warns while the robot already stands too close, or while the plan
keeps less than usual, so its clearance is smaller too.
! The planning world is the planner's, created on the worker thread; the base only closes it at teardown.
! Drive the planner through its public API (set target, plan, cancel, results) from the panel wherever you can,
  so those paths stay ready for other users such as an executor; only debug drawing and internal state reach inside.
"""

from __future__ import annotations

import asyncio
import re
from html import escape
from typing import TYPE_CHECKING, Callable, Iterable, Sequence

import viser

from ...config import find_robot_serial
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin
from ...ui.ghost import RecentUse, RobotGhost, robot_ghosts
from ...colors import ACTION, TEAL, WHITE, rgb
from ...ui.css import button_group_color_css, group_buttons
from ...ui.phone import columns_css
from ...ui.second_press import SECOND_PRESS_SECONDS, SecondPress, again_label, show_waiting
from ...ui.style import BUSY, FAIL, INFO, NONE, OK, SECTION_CTRL, block, chip, hover, note, section
from ...world.stand_ins import unknown_parts, unknown_robots
from ..teleop.clearance_toggle import ClearanceToggle, close_chip
from scaffolding_assembly_core.ids import robot_id
from .plan import Plan, PlannerContract

if TYPE_CHECKING:
    from scaffolding_assembly_core.scene import Scene

    from ...config import RobotConfig
    from ..process_contract import ProcessContract, Selection

#: The side panel tab every planner panel's tab goes in: `tab_path = (PLAN_TAB, label)`.
PLAN_TAB = "Plan"
#: Time slider resolution, seconds.
SLIDER_STEP = 0.05
#: The Path buttons' colour, outlines and Plan filled: the one button of the panel that stands out. Teal, as Plan in
#: the Assembly tab: it computes, nothing moves.
PATH_BUTTONS_COLOR = TEAL
#: Lays out the Run folder as Commit and Cancel side by side, for an html widget outside that folder.
COMMIT_STYLE = f"<style>{columns_css(viser.Icon.SEND, 2)}</style>"


#: Chip text and colour of each state of the planner's latest plan, and of no plan.
_PLAN_CHIPS = {None: ("no plan", NONE, "Set a target and press Plan."),
               "planning": ("planning…", BUSY, "The planner searches; the line below the buttons says how it goes."),
               "ready": ("plan ready", OK, "Play previews it; Commit sends it."),
               "stale": ("plan stale: replan", BUSY, "The plan no longer fits what is measured; the line below the "
                                                     "buttons says why. Plan again."),
               "sent": ("sent", INFO, "Sent to the robot; the panel's run chip says how it goes. Cancel stops it."),
               "failed": ("no plan found", FAIL, "The line below the buttons says why."),
               "cancelled": ("cancelled", NONE, "Planning was stopped; Plan tries again.")}


class PlannerPanel(HuskyPlugin):
    """A planner's panel: target, Plan/Play/Clear, time slider, Commit and ghosts. Subclass it; do not register it.

    Plan, Clear and Commit go through the planner's public API (`PlannerContract`); debug drawing may read the
    planner itself.
    """

    #: The planner plugin this panel drives. ! Also list it in `requires`.
    planner = ""
    #: The target ghost's kind, a key of `ui.ghost.KIND_COLORS`: blue "target"; a subclass may pick another colour.
    target_kind = "target"

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
        #: One line of feedback on the last action, and its colour: FAIL, BUSY (a question or warning), OK or NONE.
        self._message, self._message_color = "", NONE
        #: Commit's second press while other robots are not measured.
        self._confirm = CommitConfirm()
        #: Commit's Clearance box, below it; its hint comes with `_add_path_controls`.
        self.clearance = ClearanceToggle("")
        #: The planner's plan whose message the panel shows; a new or changed plan replaces the message.
        self._shown: Plan | None = None
        #: How the panel's target differs from the ready plan's, or "": the chip then asks for a replan.
        self._target_moved = ""
        #: When the operator last used this panel; ghosts show only while it counts (`_add_ghosts`).
        self._used: RecentUse | None = None
        #: The last request this panel's Plan sent: only its plan arriving counts as use, not another plugin's.
        self._requested = None
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

    def commit_warnings(self, ctx: PluginContext, plan: Plan) -> list[str]:
        """Reasons Commit asks for a second press, besides other robots not measured, e.g. "arm out of stow"."""
        return []

    def commit_unknown(self, ctx: PluginContext, plan: Plan) -> frozenset[str]:
        """Ids of the robots the plan moves that the second press lets be unknown (`commit_warnings` asks); none."""
        return frozenset()

    def commit_out_of_reach(self, ctx: PluginContext, plan: Plan) -> frozenset[str]:
        """Ids of other robots not measured that the plan's motion cannot reach: Commit lets them be unknown; none."""
        return frozenset()

    def commit_margin(self, ctx: PluginContext) -> float:
        """The margin a commit's clearance keeps, metres (`PlannerContract.stop_distance`), for the watch."""
        return self._api.stop_distance(self._chosen_plan())

    def clearance_chips(self) -> str:
        """Commit's clearance chips: the robot already too close (`start_chip`), the plan keeping less than usual."""
        return self.clearance.start_chip("Commit") + close_chip(self._api.close_note(self._chosen_plan()))

    def _chosen_plan(self) -> Plan | None:
        """The latest plan if it is for the chosen robot, else None."""
        plan = self._api.plan
        return plan if plan is not None and plan.request.serial == self.serial else None

    #: Options of the chosen robot's watch (`ctx.watch_clearance`), e.g. {"unknown_arms": ALL_ARMS}.
    watch_options: dict = {}

    # --- --- --- --- --- SETUP HELPERS --- --- --- --- ---

    def _connect(self, ctx: PluginContext) -> None:
        """Take the planner's API. ! Call first in setup."""
        self._api = ctx.require(self.planner)

    def _add_path_controls(self, ctx: PluginContext, gui: viser.GuiApi, plan_hint: str, commit_hint: str,
                           clearance_hint: str, before_plan: Callable[[viser.GuiApi], None] | None = None,
                           before_commit: Callable[[viser.GuiApi], None] | None = None) -> None:
        """Add the "path" section: Plan/Play/Clear, the time slider, Commit and its Clearance box, with their callbacks.

        Args:
            ctx: This plugin's context.
            gui: GUI api, inside `ctx.view.ui()`.
            plan_hint: Hover text of the Plan/Play/Clear buttons.
            commit_hint: Hover text of Commit.
            clearance_hint: Hover text of the Clearance box: what a committed motion keeps clear of.
            before_plan: Adds a subclass's own inputs between the section bar and Plan, e.g. the plan's margin.
            before_commit: Adds a subclass's own inputs between the time slider and Commit, e.g. the plan's slow-down.
        """
        gui.add_html(section("path", SECTION_CTRL))
        if before_plan is not None:
            before_plan(gui)
        plan = gui.add_button_group("Path", ["Plan", "Play", "Clear"], hint=plan_hint)
        # * Plan filled, so it never passes for Load's Step (blue outline) right above it.
        uuid = plan._impl.uuid
        gui.add_html(f"<style>{button_group_color_css(uuid, PATH_BUTTONS_COLOR)}"
                     f"{group_buttons(uuid, 0)} {{ background: {PATH_BUTTONS_COLOR} !important; "
                     f"color: {WHITE} !important; border-color: {PATH_BUTTONS_COLOR} !important; }}"
                     f"{group_buttons(uuid, 0)}:hover {{ filter: brightness(0.9); }}</style>")
        self._slider = gui.add_slider("Preview time", min=0.0, max=1.0, step=SLIDER_STEP, initial_value=0.0,
                                      hint="Scrub through the planned path: seconds from its start")
        if before_commit is not None:
            before_commit(gui)
        # * Cancel beside Commit: a folder of its own, laid out in two columns (found by Commit's icon).
        gui.add_html(COMMIT_STYLE)
        with gui.add_folder("Run"):
            commit = self._confirm.button = gui.add_button(
                "Commit", color=rgb(ACTION), icon=viser.Icon.SEND, hint=f"{commit_hint} {CommitConfirm.HINT}")
            cancel = gui.add_button("Cancel", color=rgb(FAIL), icon=viser.Icon.PLAYER_STOP,
                                    hint="Stop the motion Commit started: the planner stops the robot")
        # * Below the buttons it guards, as the jogs'.
        self.clearance.hint = clearance_hint
        self.clearance.add(ctx, gui, "commit")
        plan.on_click(ctx.defer_value("path button", lambda label: self._on_path(ctx, label)))
        # ! Operator drags only: draw writes the slider every tick, and viser calls back for that too, late, which
        #   would set the preview back.
        self._slider.on_update(ctx.defer_value("scrub", lambda value: self._scrub(float(value)), operator_only=True))
        commit.on_click(ctx.defer("commit", lambda: self._commit(ctx)))
        cancel.on_click(ctx.defer("cancel commit", lambda: self._cancel_commit()))

    def _add_ghosts(self, ctx: PluginContext, robots: Iterable[RobotConfig]) -> None:
        """Build a target ghost and a path ghost per robot, shown only while the panel is in use.

        ! Call in setup (loading meshes is too slow for a tick). Ghosts live under "ghosts/": keep subclass
          nodes elsewhere, since removing a node removes everything below it.
        """
        self._used = RecentUse(ctx.config.ghost_timeout)
        robots = list(robots)
        self._target_ghosts = robot_ghosts(ctx, robots, self.target_kind)
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
            self._clear_plan()

    def _plan(self, ctx: PluginContext) -> None:
        """Ask the planner for a plan to the target, unless one is being planned or there is no target."""
        if not self._has_target:
            self._say("set a target first", FAIL)
            return
        plan = self._api.plan
        if plan is not None and plan.state == "planning":
            ctx.log_info("plan ignored: already planning")
            return
        request = self.make_request(ctx)
        if request is not None:
            self._requested = request
            self._api.request_plan(request)

    def _clear_plan(self) -> None:
        """Forget the plan (`PlannerContract.clear`) and say so, if there was one; a caller may `_say` more after."""
        if self._api.plan is not None:
            self._api.clear()
            self._say("plan cleared")

    def _scrub(self, t: float) -> None:
        """Show the path at time `t`."""
        # * The same time again is not a use of the panel.
        if abs(t - self._t) > 1e-9:
            self._t = t
            self._used.touch()

    def _commit(self, ctx: PluginContext) -> None:
        """Ask the planner to send its plan, if the panel's target is still the plan's."""
        self._used.touch()
        plan = self._api.plan
        if plan is None or plan.state != "ready":
            self._say(f"nothing to commit: the plan is {plan.state if plan else 'missing'}", FAIL)
            return
        change = self.target_change(ctx, plan)
        if change:
            self._say(f"not committed: {change}; replan", FAIL)
            return
        warnings, far = self.commit_warnings(ctx, plan), self.commit_out_of_reach(ctx, plan)
        asked, unknown = self._confirm.asks(ctx, [robot_id(plan.request.serial)], "Commit", warnings, plan.id, far)
        if asked:
            self._say(asked, BUSY)
            return
        allowed = unknown | self.commit_unknown(ctx, plan)
        ctx.spawn(f"commit plan {plan.id}", self._report(self._api.commit(plan.id, allowed, self.clearance.value)))

    def _cancel_commit(self) -> None:
        """Stop the motion a commit started, and say so; a Commit waiting for its second press waits no more."""
        self._used.touch()
        self._confirm.press.forget()
        if self._api.cancel_commit():
            self._say("commit cancelled: the robot stops")
        else:
            self._say("nothing to cancel: no commit is running")

    async def _report(self, sent: asyncio.Future[tuple[bool, str]]) -> None:
        """Show how a commit went."""
        taken, message = await sent
        self._say(message, OK if taken else FAIL)

    def _process_step(self, ctx: PluginContext) -> tuple[Selection, str, str] | None:
        """The process plugin's selected movement, and the configured robot it is for.

        Returns:
            tuple[Selection, str, str] | None: The selection, the robot's serial and its process name;
                None if there is none, after saying why in the panel.
        """
        process: ProcessContract = ctx.require("process")
        selection = process.selection
        if selection is None:
            self._say("no process loaded in the process plugin", FAIL)
            return None
        label = selection.movement.id
        if not selection.action.robot:
            self._say(f"{label} is placed by hand: no robot to plan for", FAIL)
            return None
        robot = selection.process.robots[selection.action.robot]
        serial = find_robot_serial(ctx.config.robots, robot.serial or "")
        if serial is None:
            self._say(f"{label} is for {robot.name}, which is not configured", FAIL)
            return None
        return selection, serial, robot.name

    def _say(self, message: str, color: str = NONE) -> None:
        """Set the one line of feedback under the buttons: FAIL only for a failure, BUSY for a question or warning.

        * A failure (a refusal) forgets a Commit waiting for its second press: the next press asks again.
        """
        self._message, self._message_color = message, color
        if color == FAIL:
            self._confirm.press.forget()

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
            shown, self._shown = self._shown, plan
            if plan is not None and (shown is None or plan.id != shown.id):
                self._t, self._playing = 0.0, False
            arrived = plan is not None and plan.state == "ready" and (
                shown is None or plan.id != shown.id or shown.state != "ready")
            if arrived and plan.request is self._requested:
                # * This panel's plan arriving counts as use, so its ghost shows after a long search too; a plan
                #   another plugin asked for (calibration's moves) does not.
                self._used.touch()
            # * Only a new plan, state or message is news: a re-timed plan (`set_duration`) keeps the line as it is.
            if plan is not None and (shown is None or (plan.id, plan.state, plan.message)
                                     != (shown.id, shown.state, shown.message)):
                failed = plan.state == "failed"
                # * A sent plan's message is the commit's answer: green, as `_report` shows it, whichever runs last.
                self._say(plan.message, FAIL if failed else OK if plan.state == "sent" else BUSY if plan.warning
                          else NONE)
                if failed or plan.warning:
                    ctx.log_warn(plan.message)  # * the line's hover may be cut; the console keeps it whole
        if self._playing and plan is not None and plan.path is not None:
            self._t = min(self._t + elapsed, plan.path.duration)
            self._playing = self._t < plan.path.duration
        moved = self.target_change(ctx, plan) if plan is not None and plan.state == "ready" else ""
        if moved and moved != self._target_moved:
            self._say(moved)
        self._target_moved = moved
        self._confirm.draw(now)
        self.clearance.watch(ctx, self.serial, self.commit_margin(ctx), **self.watch_options)

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
            return chip("target moved: replan", BUSY, "The target changed since planning; Plan again before Commit.")
        return chip(*_PLAN_CHIPS[plan.state if plan is not None else None])

    def _message_html(self) -> str:
        """The last message, one line, in its colour; the text on hover (`hover`)."""
        return (f'<div title="{escape(hover(self._message), quote=True)}" '
                f'style="font-size:11px;color:{self._message_color};'
                f'white-space:nowrap;overflow:hidden;text-overflow:ellipsis">'
                f'{escape(self._message) or "&nbsp;"}</div>')


class CommitConfirm:
    """Commit's second press (`SecondPress`): while other robots are unknown, or the panel warns, the first only asks.

    The second press lets the robots unknown then be unknown during the motion: its clearance checks them as stand-ins,
    and soft stops for any other robot that is not measured (`ctx.clearance`, `unknown_robots`). It counts only for the
    plan and the reasons the first asked about: another robot lost, or a new plan, asks again.
    """

    #: For Commit's hover text.
    HINT = f"With other robots not measured, press it twice within {SECOND_PRESS_SECONDS:g} s."

    def __init__(self):
        """Wait for nothing yet; set `button` once it is built."""
        self.button: viser.GuiButtonHandle | None = None
        self.press = SecondPress()
        #: The button's own label and colour (None: the theme's), shown again once it stops waiting.
        self._label = ""
        self._color: tuple[int, int, int] | str | None = None
        self._kept = False

    def asks(self, ctx: PluginContext, acting: Iterable[str], label: str, warnings: Sequence[str] = (),
             subject: object = None, far: frozenset[str] = frozenset()) -> tuple[str, frozenset[str]]:
        """What to tell the operator instead of sending ("" to send now), and the robots the commit lets be unknown.

        Args:
            ctx: The plugin's context.
            acting: Ids of the robots the commit moves; not counted.
            label: The button's label, for the message.
            warnings: More reasons to ask first, e.g. "the arm 40° from its stow pose".
            subject: What is sent, e.g. the plan's id: a second press counts only for the same one.
            far: Ids of robots not measured that the motion cannot reach (`world.arm_reach.out_of_reach`): they may
                be unknown without asking.

        Returns:
            tuple[str, frozenset[str]]: The question, or ""; the ids of the other robots not measured now.
        """
        unknown = unknown_robots(ctx.scene.snapshot, set(acting))
        near = unknown - far
        reasons = [*warnings] + ([f"{len(near)} other robot{'s' if len(near) > 1 else ''} unknown"] if near else [])
        if not reasons:
            self.press.forget()
            return "", unknown
        # * The warnings without their numbers, which drift between presses (e.g. "the arm 40° from stow").
        key = repr((subject, sorted(near), [re.sub(r"[\d.]+", "#", warning) for warning in warnings]))
        if self.press.press(ctx.now(), key):
            return "", unknown
        return (f"press {label} again within {SECOND_PRESS_SECONDS:g} s to move with "
                f"{'; '.join(reasons)}"), frozenset()

    def draw(self, now: float) -> None:
        """Make the button amber and "Commit?" while it waits for its second press, as it was otherwise."""
        if self.button is None:
            return
        if not self._kept:
            self._color, self._kept = self.button.color, True
        if self.button.label != again_label(self._label):
            self._label = self.button.label  # * its owner's own label, also after the owner changed it
        show_waiting(self.button, bool(self.press.waiting(now)), self._label, self._color)


def unknown_robots_chip(snapshot: Scene, acting: Iterable[str]) -> str:
    """An amber chip counting the other robots not measured, each named on hover (`unknown_lines`); "" if none.

    Args:
        snapshot: The tick's snapshot.
        acting: Ids of the robots the panel plans for; not counted.
    """
    unknown = unknown_robots(snapshot, set(acting))
    if not unknown:
        return ""
    return chip(f"{len(unknown)} robot{'s' if len(unknown) > 1 else ''} unknown", BUSY,
                "\n".join(unknown_lines(snapshot, unknown)) + "\nHealth shows why.")


def unknown_lines(snapshot: Scene, unknown: Iterable[str]) -> list[str]:
    """One short line per robot not measured, e.g. "a200-0805: base, ur_arm unknown"."""
    return [f"{key.split('/', 1)[1]}: {unknown_parts(snapshot.robots[key])} unknown" for key in sorted(unknown)]
