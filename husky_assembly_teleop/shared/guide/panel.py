"""
The guided procedure's panel: the overview of the steps, the current step, Back / Next / Done, the procedure's controls.

- Back and Next move one step and nothing else: they never refuse, write or start work. Done leaves a step whose
  result is finished for the work that is left (`on_done`); on an unfinished one (`step.unfinished`), or one whose
  precondition has expired (`Step.expired`), it is amber: its first press says why, and a second goes on anyway.
  None of them moves while a procedure runs.
- The overview shows every step's state (`Step.state`) on a fixed line: result steps solid, precondition steps
  dashed; the current step outlines the precondition steps it needs that have expired.
- ! Nothing above the step's controls may change height while a step shows, or the buttons jump: its chips sit on
  fixed lines (the checks, the ready checks, one status line always there), and text that comes and goes (the Go
  question, the step's `question`, Done's question, why a state no chip names holds: `EXPLAINED_BELOW`) goes below
  the controls.
- `build_phone` adds the same on the phone page (`BothPages`): the overview as one row, the chips, Back / Next / Done
  and Cancel, and each step's phone controls (`Step.build_phone`). Go, Auto and Preview stay on the laptop, where the
  ghost shows the move.
"""

from __future__ import annotations

from dataclasses import replace
from html import escape
from typing import TYPE_CHECKING, Callable, Sequence

import viser

from ...ui.phone import big_buttons, phone_text
from ...ui.second_press import SECOND_PRESS_SECONDS, SecondPress, again_label
from ...colors import INK, rgb
from ...ui.style import (BUSY, CURRENT, FAIL, LEVEL_COLORS, NONE, OK, SECTION_CTRL, WHITE, block, check_chip, chip,
                         chip_line, chip_row, hover, note, section, warning)
from ...world.checks import WARN, Check
from ..planner.planner_panel import SLIDER_STEP
from .both_pages import BothPages, on_both
from .procedure import GO_AGAIN, Procedure
from .step import Step, unfinished

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext


#: The step states whose reason no chip of the step names: the line below the buttons says it.
EXPLAINED_BELOW = ("expired", "out of date", "pending")


def set_changed(handle, **values) -> None:
    """Set a widget's properties, only those that changed: every assignment is a message to each browser."""
    for name, value in values.items():
        # * `BothPages` compares each page's handle itself.
        if isinstance(handle, BothPages) or getattr(handle, name) != value:
            setattr(handle, name, value)


def state_chip(text: str, state: Check, dashed: bool, current: bool = False, needed: bool = False) -> str:
    """A chip in the colour of a step's state: solid for a result, dashed for a precondition.

    Args:
        text: The chip's text, e.g. the step's number.
        state: The step's state (`Step.state`); its label and detail on hover.
        dashed: Whether the step is a precondition step.
        current: Ring it: the current step.
        needed: Outline it, dotted: a precondition the current step needs has expired.
    """
    color = LEVEL_COLORS[state.level]
    look = (f"background:transparent;color:{color};border:1px dashed {color}" if dashed else
            f"background:{color};color:{WHITE};border:1px solid {color}")
    if current:
        look += f";box-shadow:0 0 0 2px {CURRENT}"
    if needed:
        look += f";outline:2px dotted {INK};outline-offset:1px"
    title = hover(f"{state.label}: {state.detail}", "the step's chips") if state.detail else state.label
    return (f'<span title="{escape(title, quote=True)}" style="display:inline-block;white-space:nowrap;{look};'
            f'border-radius:4px;padding:0 5px;margin:0 6px 2px 2px;font-size:11px;font-weight:600">{text}</span>')


def step_buttons_css(arrow_width: int) -> str:
    """CSS for Back and Next as narrow arrows, then Done filling the rest: Done is the button to press.

    Args:
        arrow_width: Back's and Next's width, px.
    """
    folder = f"div:has(> div > button .icon-tabler-{viser.Icon.CHEVRON_LEFT})"
    # * Each button's cell has viser's side margins on both sides: the arrow's column counts them in.
    arrow = f"calc({arrow_width}px + 2 * var(--mantine-spacing-xs))"
    css = f"{folder} {{ display: grid; grid-template-columns: {arrow} {arrow} minmax(0, 1fr); }}"
    for icon in (viser.Icon.CHEVRON_LEFT, viser.Icon.CHEVRON_RIGHT):
        # * Icon only, centred: the label stays in the code for the hint and the log.
        css += (f"{folder} button:has(.icon-tabler-{icon}) {{ width: 100%; padding: 0; }}"
                f"{folder} button:has(.icon-tabler-{icon}) .mantine-Button-label {{ display: none; }}"
                f"{folder} button:has(.icon-tabler-{icon}) .mantine-Button-section {{ margin: 0 !important; }}")
    return css


#: The step buttons' hints: Back and Next only look at another step; Done finishes this one.
BACK_HINT, NEXT_HINT = "Look at the step before; changes nothing", "Look at the next step; changes nothing"
DONE_HINT = "Keep this step's result and go to the work that is left"
#: Back, Next and Done's icons.
STEP_ICONS = [viser.Icon.CHEVRON_LEFT, viser.Icon.CHEVRON_RIGHT, viser.Icon.CIRCLE_CHECK]
#: On the phone: the step buttons big; the check chips in a box of fixed height, scrolling if a chip's value grows
#: them past it, since there they wrap onto rows that would push the controls.
PHONE_STEP_STYLE = (f"<style>{step_buttons_css(72)}{big_buttons(STEP_ICONS, 60, 26)}"
                    f".guide-chips {{height: 104px; overflow-y: auto}}</style>")


class GuidePanel:
    """Shows a guided procedure's steps one at a time, and moves between them.

    Attributes:
        steps: The steps, in order.
        procedure: The procedure the steps run.
        index: The current step.
    """

    def __init__(self, title: str, steps: Sequence[Step], procedure: Procedure, *,
                 on_done: Callable[[PluginContext, bool], int] | None = None,
                 auto_hint: str = "Go without asking; Cancel stops."):
        """Start at the first step.

        Args:
            title: The panel's title, e.g. "base calibration".
            steps: The steps, in order.
            procedure: The procedure the steps run.
            on_done: Called when Done leaves the current step, with whether it is unfinished or expired and Done was
                pressed again: keep what the step made, and return the step to go to. Without it Done goes to the
                next step.
            auto_hint: The Auto checkbox's hint: what Auto skips.
        """
        self.title = title
        self.steps = list(steps)
        self.procedure = procedure
        self.index = 0
        #: Done's second press on an unfinished or expired step, keyed by the step's index: it leaves it as it stands.
        self._press = SecondPress()
        #: The path Preview was last reset for; whether Play plays it, and the tick time it last moved on from.
        self._previewing = None
        self._playing = False
        self._played_at: float | None = None
        #: The running procedure's task and the step that started it: Go and Preview act only on that step.
        self._owned: tuple[object, int] | None = None
        #: The phone page's overview, once `build_phone` made it.
        self._phone_list_html = None
        self._on_done = on_done
        self._auto_hint = auto_hint

    @property
    def current(self) -> Step:
        """Step: The current step."""
        return self.steps[self.index]

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Build, top to bottom: the title, the overview, Back / Next / Done, the step's chips, controls, notes, text.

        ! Back, Done and Next sit above everything whose height changes from step to step, so they never move.
        The step's own widgets sit in "Step controls", one folder per step, only the current one shown; below them
        Auto, Preview, Go and Cancel for the steps that run things.
        """
        self._title_html = gui.add_html("")
        self._list_html = gui.add_html("")
        # * The step buttons side by side: the Step folder holds only them; Back and Next small arrows.
        # * The check chips in a box of fixed height, scrolling if a chip's value grows them past it: a new row of
        #   chips would push the step's controls down.
        gui.add_html(f"<style>{step_buttons_css(44)}.guide-chips {{height: 76px; overflow-y: auto}}</style>")
        with gui.add_folder("Step"):
            self._back_button = gui.add_button("Back", icon=viser.Icon.CHEVRON_LEFT, hint=BACK_HINT)
            self._next_button = gui.add_button("Next", icon=viser.Icon.CHEVRON_RIGHT, hint=NEXT_HINT)
            self._done_button = gui.add_button("Done", color=rgb(OK), icon=viser.Icon.CIRCLE_CHECK, hint=DONE_HINT)
        # * The step's checks, then the procedure's line in its own handle: what runs and `extra` change often, and
        #   a change would drop the checks' hover.
        self._status_html = gui.add_html("")
        self._line_html = gui.add_html("")
        self._folders = []
        with gui.add_folder("Step controls"):
            for index, step in enumerate(self.steps):
                folder = gui.add_folder(step.name, visible=index == 0)
                self._folders.append(folder)
                with folder:
                    step.build(ctx, gui)
            self._auto = gui.add_checkbox("Auto", initial_value=False, hint=self._auto_hint)
            self._preview = gui.add_slider("Preview time", min=0.0, max=1.0, step=SLIDER_STEP, initial_value=0.0,
                                           hint="Scrub what Go starts, seconds from its start; the ghost shows the "
                                                "robot.")
            self._play_button = gui.add_button("Play", icon=viser.Icon.PLAYER_PLAY,
                                               hint="Play what Go starts in real time, from the Preview time; "
                                                    "press again to pause.")
            self._go_button = gui.add_button("Go", color=rgb(OK), icon=viser.Icon.PLAYER_PLAY)
            self._cancel_button = gui.add_button("Cancel", color=rgb(FAIL), icon=viser.Icon.PLAYER_STOP)
            # * Below the buttons it guards, as the jogs'.
            self._go_clearance = self.procedure.clearance.add(ctx, gui, "go")
        self._notes_html = gui.add_html("")
        self._text_html = gui.add_html("")
        procedure = self.procedure
        self._auto.on_update(ctx.defer_value("auto", lambda value: setattr(procedure, "auto", bool(value)),
                                             operator_only=True))
        self._preview.on_update(ctx.defer_value("preview", lambda value: setattr(procedure, "preview", float(value)),
                                                operator_only=True))
        self._play_button.on_click(ctx.defer("play", self._play))
        procedure.confirm.button = self._go_button
        self._go_button.on_click(ctx.defer("go", lambda: self._go(ctx)))
        self._cancel_button.on_click(ctx.defer("cancel", procedure.cancel))
        self._wire_steps(ctx, self._back_button, self._done_button, self._next_button)

    def _owner(self) -> int | None:
        """The index of the step whose button started the running procedure; None while none runs.

        Noted when the panel first sees the procedure, which a step's own button starts while that step shows.
        """
        procedure = self.procedure
        if not procedure.running:
            self._owned = None
            return None
        if self._owned is None or self._owned[0] is not procedure._task:
            self._owned = (procedure._task, self.index)
        return self._owned[1]

    def _elsewhere(self) -> str:
        """The name of the step the running procedure belongs to while another shows, else ""."""
        owner = self._owner()
        return self.steps[owner].name if owner is not None and owner != self.index else ""

    def _running_chip(self) -> str:
        """An amber chip saying what the running procedure does: "waits for Go" or "runs"; "" while none runs."""
        procedure = self.procedure
        if not procedure.running:
            return ""
        if procedure.waiting:
            return chip("waits for Go", BUSY, f"Go here answers: {procedure.prompt}")
        return chip("runs", BUSY, f"{procedure.doing or 'Its procedure runs'}; Cancel stops it.")

    def _go(self, ctx: PluginContext) -> None:
        """Answer the procedure's question, only on the step that started it."""
        if elsewhere := self._elsewhere():
            ctx.log_info(f"Go ignored: it answers {elsewhere}'s question; go back to {elsewhere}")
            return
        self.procedure.press_go(ctx)

    def _play(self) -> None:
        """Play the preview, again from its start once at its end, or pause it."""
        procedure = self.procedure
        path = procedure.preview_path
        if path is None:
            return
        if not self._playing and procedure.preview >= path.duration:
            procedure.preview = 0.0
        self._playing = not self._playing

    def _wire_steps(self, ctx: PluginContext, back, done, forward) -> None:
        """Make one page's Back, Done and Next buttons act."""
        back.on_click(ctx.defer("back", lambda: self.navigate(ctx, forward=False)))
        done.on_click(ctx.defer("done", lambda: self.done(ctx)))
        forward.on_click(ctx.defer("next", lambda: self.navigate(ctx, forward=True)))

    def build_phone(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Build the same on the phone page, after `build`: chips, Back / Next / Done, each step's phone controls, Cancel.

        ! Go, Auto and Preview are not built: a move is started where its ghost can be previewed. The Go question
        shows, with where to answer it.
        """
        def notes(html: str) -> str:
            go = note("Go on the laptop, after the preview.", CURRENT) if self.procedure.prompt else ""
            return phone_text(html + go)

        gui.add_html(PHONE_STEP_STYLE)
        self._title_html = on_both(self._title_html, gui.add_html(""), phone_text)
        # * The overview as one row of numbers: nine lines would push the controls off a phone's screen.
        self._phone_list_html = gui.add_html("")
        with gui.add_folder("Step"):
            back = gui.add_button("Back", icon=viser.Icon.CHEVRON_LEFT, hint=BACK_HINT)
            forward = gui.add_button("Next", icon=viser.Icon.CHEVRON_RIGHT, hint=NEXT_HINT)
            done = gui.add_button("Done", color=rgb(OK), icon=viser.Icon.CIRCLE_CHECK, hint=DONE_HINT)
        self._status_html = on_both(self._status_html, gui.add_html(""), phone_text)
        self._line_html = on_both(self._line_html, gui.add_html(""), phone_text)
        with gui.add_folder("Step controls"):
            for index, step in enumerate(self.steps):
                folder = gui.add_folder(step.name, visible=index == self.index)
                self._folders[index] = on_both(self._folders[index], folder)
                with folder:
                    step.build_phone(ctx, gui)
            cancel = gui.add_button("Cancel", color=rgb(FAIL), icon=viser.Icon.PLAYER_STOP)
        self._notes_html = on_both(self._notes_html, gui.add_html(""), notes)
        self._text_html = on_both(self._text_html, gui.add_html(""), phone_text)
        cancel.on_click(ctx.defer("cancel", self.procedure.cancel))
        self._wire_steps(ctx, back, done, forward)
        self._back_button = on_both(self._back_button, back)
        self._done_button = on_both(self._done_button, done)
        self._next_button = on_both(self._next_button, forward)
        self._cancel_button = on_both(self._cancel_button, cancel)

    # --- --- --- --- --- NAVIGATION --- --- --- --- ---

    def _busy(self, ctx: PluginContext, button: str) -> bool:
        """Whether a working procedure keeps `button` from acting; says so. One that only waits for Go is ended.

        ! Done is never disabled: it ends a procedure of this step waiting for Go (nothing moves), and refuses only
          while it moves. Another step's procedure it leaves alone.
        """
        procedure = self.procedure
        if self._elsewhere():
            return False
        if procedure.waiting:
            ctx.log_info(f"{button}: ended what waited for Go ({procedure.prompt})")
            procedure.cancel()
            return False
        if procedure.running:
            why = f"{procedure.doing or 'a procedure'} runs; Cancel it first"
            ctx.log_info(f"{button} ignored: {why}")
            procedure.outcome = Check(f"{button.lower()} refused", WARN, why)
        return procedure.running

    def navigate(self, ctx: PluginContext, forward: bool) -> None:
        """Show the step before or after; nothing else: a running procedure goes on, still shown in the line."""
        self._owner()  # * notes the step a procedure started this tick belongs to, before leaving it
        index = min(self.index + 1, len(self.steps) - 1) if forward else max(self.index - 1, 0)
        if index != self.index:
            self.show(ctx, index)

    def done(self, ctx: PluginContext) -> None:
        """Leave the current step for the work that is left; an unfinished or expired one only on the second press."""
        if self._busy(ctx, "Done"):
            return
        why = self._done_asks(ctx, self.current.state(ctx))
        anyway = why is not None
        if anyway and not self._press.press(ctx.now(), str(self.index)):
            ctx.log_info(f"Done asks again: {self.current.name} {why}")
            return
        if anyway:
            ctx.log_warn(f"{self.current.name}: left as it stands ({why})")
        index = (self._on_done(ctx, anyway) if self._on_done is not None
                 else min(self.index + 1, len(self.steps) - 1))
        self.show(ctx, index)

    def _done_asks(self, ctx: PluginContext, state: Check) -> str | None:
        """Why Done asks once more on the current step: its state unfinished, or its precondition expired; else None."""
        if unfinished(state):
            return f"{state.label}: {state.detail}"
        expired = self.current.expired(ctx)
        return f"expired: {expired}" if expired is not None else None

    def _confirming(self, ctx: PluginContext) -> bool:
        """Whether a first Done on the current step still waits for its second."""
        return self._press.waiting(ctx.now()) == str(self.index)

    def show(self, ctx: PluginContext, index: int) -> None:
        """Make step `index` the current one."""
        self.current.leave(ctx)
        self.index = index
        self._press.forget()
        self.procedure.outcome = None
        for number, folder in enumerate(self._folders):
            folder.visible = number == index
        self.current.enter(ctx)

    # --- --- --- --- --- DRAWING --- --- --- --- ---

    def draw(self, ctx: PluginContext, detail: str = "", extra: str = "") -> None:
        """Show the title, the overview, the current step's chips and status, the controls, and its text.

        Args:
            ctx: The plugin's context.
            detail: Beside the title, e.g. the chosen robot.
            extra: Chips for the status line, e.g. live problems.
        """
        step = self.current
        states = [each.state(ctx) for each in self.steps]
        names = [each.name for each in self.steps]
        needed = {names.index(name) for name in step.needs
                  if step.need_expired(ctx, self.steps[names.index(name)]) is not None}
        owner = self._owner()  # * first: the overview marks the step whose procedure runs
        self._title_html.content = section(self.title, SECTION_CTRL,
                                           f"{detail} · step {self.index + 1}/{len(self.steps)}")
        self._list_html.content = block("".join(self._row(index, states[index], index in needed)
                                                for index in range(len(self.steps))))
        if self._phone_list_html is not None:
            # * The step whose procedure runs has a ▶ in its chip, as the laptop's overview marks it.
            set_changed(self._phone_list_html, content=phone_text(block(chip_line("".join(
                state_chip(f"▶{index + 1}" if index == owner else str(index + 1), states[index],
                           self.steps[index].precondition, index == self.index, index in needed)
                for index in range(len(self.steps)))))))
        state = states[self.index]
        asks = self._done_asks(ctx, state)
        if asks is None:
            self._press.forget()
        asked = self._confirming(ctx)
        # * A failing result is accepted as it stands; one missing or out of date is only skipped.
        anyway = "Accept as is" if state.label == "fails" else "Skip"
        procedure = self.procedure
        now = ctx.now()
        path = procedure.preview_path
        if path is None:
            self._playing = False
        elif self._playing and self._played_at is not None:
            procedure.preview = min(procedure.preview + now - self._played_at, path.duration)
            self._playing = procedure.preview < path.duration
        self._played_at = now
        # * Go is amber while it waits for its second press; the question goes with the wait.
        procedure.confirm.draw(now)
        again = procedure.outcome is not None and procedure.outcome.label == GO_AGAIN
        if again and not procedure.confirm.press.waiting(now):
            procedure.outcome, again = None, False
        # * On another step the procedure's chip names its step: Back and Next leave it running.
        elsewhere = self._elsewhere()
        if elsewhere:
            what = "waits for Go" if procedure.waiting else (procedure.doing or "runs")
            status = chip(f"{elsewhere} {what}", BUSY, f"Started in {elsewhere}: go back there for Go and Preview; "
                                                       f"Cancel stops it from any step.")
        else:
            status = chip(procedure.doing, BUSY, "The step's action runs; Cancel stops it.") if procedure.doing else ""
        if procedure.outcome is not None:
            outcome = procedure.outcome
            status += check_chip(replace(outcome, detail=hover(outcome.detail)))
        ready = step.ready(ctx)
        self._status_html.content = (
            section(step.name, CURRENT, state.label)
            # * Each line named: what the step made, whether its buttons can act now, and what happens now.
            + '<div class="guide-chips">' + chip_row("result", "".join(check_chip(check) for check in step.checks(ctx)),
                                                     wrap=True)
            + (chip_row("ready", "".join(check_chip(check) for check in ready), wrap=True) if ready else "") + "</div>")
        self._line_html.content = chip_row("now", status + extra)
        question = step.question(ctx)
        # * Why the overview says so, for a state no chip names, below the buttons: Done's question says it once asked.
        why = state.detail if state.label in EXPLAINED_BELOW and state.detail and not asked else ""
        if elsewhere and procedure.prompt:
            asking = note(escape(f"{elsewhere} waits for Go: go back to {elsewhere} to answer it."), CURRENT)
        else:
            asking = note(escape(procedure.prompt), CURRENT) if procedure.prompt else ""
        self._notes_html.content = (
            asking
            + (warning(escape(f"{step.name} {state.label}: {why.rstrip('.')}.")) if why else "")
            + (warning(escape(procedure.outcome.detail)) if again else "")
            + (warning(escape(question)) if question else "")
            + (warning(escape(f"{step.name}: {asks}. Press Done again within {SECOND_PRESS_SECONDS:g} s to "
                              f"{anyway.lower()}: it goes on as it stands."))
               if asked else ""))
        self._text_html.content = section("what to do", CURRENT) + block(f"<div>{escape(step.instructions)}</div>")
        self._draw_controls(asks is not None, again_label(anyway) if asked else "Done")
        step.draw(ctx)

    def _row(self, index: int, state: Check, needed: bool) -> str:
        """One line of the overview: the step's number and name, its state in short, why on hover; the current bold.

        The step whose procedure runs has an amber chip after its name (`_running_chip`).
        """
        step, current = self.steps[index], index == self.index
        running = self._running_chip() if index == self._owner() else ""
        color, why = LEVEL_COLORS[state.level], escape(hover(state.detail, "the step's chips"), quote=True)
        # * Room around the chip for the current step's ring and an expired precondition's outline: the row hides its
        # * overflow, so a long state's text cannot widen the panel.
        return (f'<div style="display:flex;align-items:center;white-space:nowrap;overflow:hidden;min-height:20px;'
                f'padding:3px 0 3px 3px">{state_chip(str(index + 1), state, step.precondition, current, needed)}'
                f'<span style="font-weight:{700 if current else 400};margin-right:6px">{escape(step.name)}</span>'
                f'{running}'
                f'<span style="color:{color};font-size:11px;font-weight:600;margin-right:6px">'
                f'{escape(state.label)}</span>'
                f'<span title="{why}" style="color:{NONE};font-size:11px;'
                f'overflow:hidden;text-overflow:ellipsis">{escape(state.value)}</span></div>')

    def _draw_controls(self, unfinished_step: bool, done_label: str) -> None:
        """Show Back, Done, Next, Go, Cancel, Preview, Play and Auto as what they do now: disabled while they do nothing.

        Done is never disabled, Back and Next only on the first and last step. Done is amber while the step is
        unfinished or expired (it asks once more, then labelled `done_label`), else green.
        """
        step, procedure = self.current, self.procedure
        set_changed(self._auto, visible=step.auto)
        path = procedure.preview_path
        # * Go and Preview act for the step that started the procedure only: greyed on every other.
        elsewhere = bool(self._elsewhere())
        set_changed(self._preview, visible=step.auto, disabled=path is None or elsewhere)
        set_changed(self._play_button, visible=step.auto, disabled=path is None or elsewhere,
                    label="Pause" if self._playing else "Play")
        if path is not self._previewing:
            # * Only when a new preview starts, and while it plays: the operator's own drags reach the procedure a
            #   tick later. The end first, so the value is not clamped to the last path's.
            self._previewing, self._playing = path, False
            self._preview.max = max(path.duration, SLIDER_STEP) if path is not None else 1.0
            self._preview.value = procedure.preview
        elif self._playing:
            self._preview.value = round(procedure.preview, 3)
        set_changed(self._go_button, visible=step.goes, disabled=not procedure.prompt or elsewhere,
                    label=f"Go: {procedure.go_label}" if procedure.prompt else "Go")
        set_changed(self._cancel_button, visible=step.cancels or procedure.running, disabled=not procedure.running)
        set_changed(self._go_clearance, visible=step.goes)
        # ! Never disabled while a procedure runs: Back and Next only show another step, Done ends one waiting for Go
        #   or says why not (`_busy`).
        back = (None, self.index == 0)
        forward = (None, self.index >= len(self.steps) - 1)
        done = (rgb(BUSY if unfinished_step else OK), False)
        for button, (color, disabled) in ((self._back_button, back), (self._done_button, done),
                                          (self._next_button, forward)):
            set_changed(button, color=color, disabled=disabled)
        set_changed(self._done_button, label=done_label)
