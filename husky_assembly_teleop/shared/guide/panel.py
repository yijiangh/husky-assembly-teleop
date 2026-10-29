"""
The guided procedure's panel: the overview of the steps, the current step, Back / Next / Done, the procedure's controls.

- Back and Next move one step and nothing else: they never refuse, write or start work. Done leaves a step whose
  result is finished for the work that is left (`on_done`); on an unfinished one (`step.unfinished`) its first press
  says why, and a second goes on anyway. None of them moves while a procedure runs.
- The overview shows every step's state (`Step.state`) on a fixed line: result steps solid, precondition steps
  dashed; the current step outlines the precondition steps it needs that have expired.
- ! Nothing above the step's controls may change height while a step shows, or the buttons jump: its chips sit on
  fixed lines (the checks, the ready checks, one status line always there), and text that comes and goes (the Go
  question, the step's `question`, Done's question) goes below the controls.
- `build_phone` adds the same on the phone page (`BothPages`): the overview as one row, the chips, Back / Next / Done
  and Cancel, and each step's phone controls (`Step.build_phone`). Go, Auto and Preview stay on the laptop, where the
  ghost shows the move.
"""

from __future__ import annotations

from html import escape
from typing import TYPE_CHECKING, Callable, Sequence

import viser

from ...ui.phone import big_buttons, phone_text
from ...ui.style import (BUSY, INFO, LEVEL_COLORS, NONE, SECTION_CTRL, block, check_chip, chip, chip_line, section,
                         warning)
from ...world.checks import Check
from .both_pages import BothPages, on_both
from .procedure import Procedure
from .step import Step, unfinished

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext


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
            f"background:{color};color:#fff;border:1px solid {color}")
    if current:
        look += f";box-shadow:0 0 0 2px {INFO}"
    if needed:
        look += f";outline:2px dotted {INFO};outline-offset:1px"
    title = f"{state.label}: {state.detail}" if state.detail else state.label
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
#: Seconds a first Done on an unfinished step waits for the second; then Done is plain Done again.
CONFIRM_SECONDS = 5.0
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
            on_done: Called when Done leaves the current step, with whether it is unfinished and Done was pressed
                again: keep what the step made, and return the step to go to. Without it Done goes to the next step.
            auto_hint: The Auto checkbox's hint: what Auto skips.
        """
        self.title = title
        self.steps = list(steps)
        self.procedure = procedure
        self.index = 0
        #: The step whose Done asked once already and when, so a second Done within CONFIRM_SECONDS leaves it.
        self._asked: tuple[int, float] | None = None
        #: The path Preview was last reset for.
        self._previewing = None
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
            self._done_button = gui.add_button("Done", color="green", icon=viser.Icon.CIRCLE_CHECK, hint=DONE_HINT)
        self._status_html = gui.add_html("")
        self._folders = []
        with gui.add_folder("Step controls"):
            for index, step in enumerate(self.steps):
                folder = gui.add_folder(step.name, visible=index == 0)
                self._folders.append(folder)
                with folder:
                    step.build(ctx, gui)
            self._auto = gui.add_checkbox("Auto", initial_value=False, hint=self._auto_hint)
            self._preview = gui.add_slider("Preview", min=0.0, max=1.0, step=0.01, initial_value=0.0,
                                           hint="Scrub what Go starts; the ghost shows the robot.")
            self._go_button = gui.add_button("Go", color="green", icon=viser.Icon.PLAYER_PLAY)
            self._cancel_button = gui.add_button("Cancel", color="red", icon=viser.Icon.PLAYER_STOP)
        self._notes_html = gui.add_html("")
        self._text_html = gui.add_html("")
        procedure = self.procedure
        self._auto.on_update(ctx.defer_value("auto", lambda value: setattr(procedure, "auto", bool(value)),
                                             operator_only=True))
        self._preview.on_update(ctx.defer_value("preview", lambda value: setattr(procedure, "preview", float(value)),
                                                operator_only=True))
        self._go_button.on_click(ctx.defer("go", procedure.go))
        self._cancel_button.on_click(ctx.defer("cancel", procedure.cancel))
        self._wire_steps(ctx, self._back_button, self._done_button, self._next_button)

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
            go = warning("Go on the laptop, after the preview.", INFO) if self.procedure.prompt else ""
            return phone_text(html + go)

        gui.add_html(PHONE_STEP_STYLE)
        self._title_html = on_both(self._title_html, gui.add_html(""), phone_text)
        # * The overview as one row of numbers: nine lines would push the controls off a phone's screen.
        self._phone_list_html = gui.add_html("")
        with gui.add_folder("Step"):
            back = gui.add_button("Back", icon=viser.Icon.CHEVRON_LEFT, hint=BACK_HINT)
            forward = gui.add_button("Next", icon=viser.Icon.CHEVRON_RIGHT, hint=NEXT_HINT)
            done = gui.add_button("Done", color="green", icon=viser.Icon.CIRCLE_CHECK, hint=DONE_HINT)
        self._status_html = on_both(self._status_html, gui.add_html(""), phone_text)
        with gui.add_folder("Step controls"):
            for index, step in enumerate(self.steps):
                folder = gui.add_folder(step.name, visible=index == self.index)
                self._folders[index] = on_both(self._folders[index], folder)
                with folder:
                    step.build_phone(ctx, gui)
            cancel = gui.add_button("Cancel", color="red", icon=viser.Icon.PLAYER_STOP)
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
        """Whether a procedure runs, so `button` does nothing; says so."""
        if self.procedure.running:
            ctx.log_info(f"{button} ignored: {self.procedure.doing or 'a procedure'} runs; Cancel it first")
        return self.procedure.running

    def navigate(self, ctx: PluginContext, forward: bool) -> None:
        """Go one step back or forward; nothing else."""
        if self._busy(ctx, "Next" if forward else "Back"):
            return
        index = min(self.index + 1, len(self.steps) - 1) if forward else max(self.index - 1, 0)
        if index != self.index:
            self.show(ctx, index)

    def done(self, ctx: PluginContext) -> None:
        """Leave the current step for the work that is left; an unfinished one only on the second press."""
        if self._busy(ctx, "Done"):
            return
        state = self.current.state(ctx)
        anyway = unfinished(state)
        if anyway and not self._confirming(ctx):
            self._asked = (self.index, ctx.now())
            ctx.log_info(f"Done asks again: {self.current.name} {state.label}: {state.detail}")
            return
        if anyway:
            ctx.log_warn(f"{self.current.name}: left as it stands ({state.label}: {state.detail})")
        index = (self._on_done(ctx, anyway) if self._on_done is not None
                 else min(self.index + 1, len(self.steps) - 1))
        self.show(ctx, index)

    def _confirming(self, ctx: PluginContext) -> bool:
        """Whether a first Done on the current step still waits for its second."""
        return (self._asked is not None and self._asked[0] == self.index
                and ctx.now() - self._asked[1] <= CONFIRM_SECONDS)

    def show(self, ctx: PluginContext, index: int) -> None:
        """Make step `index` the current one."""
        self.current.leave(ctx)
        self.index, self._asked = index, None
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
        needed = {names.index(name) for name in step.needs if self.steps[names.index(name)].expired(ctx) is not None}
        self._title_html.content = section(self.title, SECTION_CTRL,
                                           f"{detail} · step {self.index + 1}/{len(self.steps)}")
        self._list_html.content = block("".join(self._row(index, states[index], index in needed)
                                                for index in range(len(self.steps))))
        if self._phone_list_html is not None:
            set_changed(self._phone_list_html, content=phone_text(block(chip_line("".join(
                state_chip(str(index + 1), states[index], self.steps[index].precondition, index == self.index,
                           index in needed) for index in range(len(self.steps)))))))
        state = states[self.index]
        if not unfinished(state) or not self._confirming(ctx):
            self._asked = None
        # * A failing result is accepted as it stands; one missing or out of date is only skipped.
        anyway = "Accept as is" if state.label == "fails" else "Skip"
        procedure = self.procedure
        status = chip(procedure.doing, BUSY) if procedure.doing else ""
        if procedure.outcome is not None:
            status += check_chip(procedure.outcome)
        ready = step.ready(ctx)
        self._status_html.content = (
            section(step.name, INFO, state.label)
            + '<div class="guide-chips">' + block("".join(check_chip(check) for check in step.checks(ctx)))
            + (block("".join(check_chip(check) for check in ready)) if ready else "") + "</div>"
            + chip_line(status + extra))
        question = step.question(ctx)
        self._notes_html.content = (
            (warning(escape(procedure.prompt), INFO) if procedure.prompt else "")
            + (warning(escape(question)) if question else "")
            + (warning(escape(f"{step.name}: {state.label}: {state.detail}. {anyway} within "
                              f"{CONFIRM_SECONDS:g} s goes on as it stands."))
               if self._asked is not None else ""))
        self._text_html.content = section("what to do", INFO) + block(f"<div>{escape(step.instructions)}</div>")
        self._draw_controls(unfinished(state), anyway if self._asked is not None else "Done")
        step.draw(ctx)

    def _row(self, index: int, state: Check, needed: bool) -> str:
        """One line of the overview: the step's number and name, its state in short, why on hover; the current bold."""
        step, current = self.steps[index], index == self.index
        color = LEVEL_COLORS[state.level]
        return (f'<div style="display:flex;align-items:center;white-space:nowrap;overflow:hidden;min-height:20px">'
                f'{state_chip(str(index + 1), state, step.precondition, current, needed)}'
                f'<span style="font-weight:{700 if current else 400};margin-right:6px">{escape(step.name)}</span>'
                f'<span style="color:{color};font-size:11px;font-weight:600;margin-right:6px">'
                f'{escape(state.label)}</span>'
                f'<span title="{escape(state.detail, quote=True)}" style="color:{NONE};font-size:11px;'
                f'overflow:hidden;text-overflow:ellipsis">{escape(state.value)}</span></div>')

    def _draw_controls(self, unfinished_step: bool, done_label: str) -> None:
        """Show Back, Done, Next, Go, Cancel, Preview and Auto as what they do now: grey while they do nothing.

        Done is orange while the step is unfinished (it asks once more, labelled `done_label`), green otherwise.
        """
        step, procedure = self.current, self.procedure
        set_changed(self._auto, visible=step.auto)
        set_changed(self._preview, visible=step.auto, disabled=procedure.preview_path is None)
        if procedure.preview_path is not self._previewing:
            # * Only when a new preview starts: the operator's own drags reach the procedure a tick later.
            self._previewing = procedure.preview_path
            self._preview.value = procedure.preview
        set_changed(self._go_button, visible=step.goes, disabled=not procedure.prompt,
                    label=f"Go: {procedure.go_label}" if procedure.prompt else "Go")
        set_changed(self._cancel_button, visible=step.cancels or procedure.running, disabled=not procedure.running)
        if procedure.running:
            back = done = forward = ("gray", True)
        else:
            back = ("gray", True) if self.index == 0 else (None, False)
            forward = ("gray", True) if self.index >= len(self.steps) - 1 else (None, False)
            done = ("orange", False) if unfinished_step else ("green", False)
        for button, (color, disabled) in ((self._back_button, back), (self._done_button, done),
                                          (self._next_button, forward)):
            set_changed(button, color=color, disabled=disabled)
        set_changed(self._done_button, label=done_label)
