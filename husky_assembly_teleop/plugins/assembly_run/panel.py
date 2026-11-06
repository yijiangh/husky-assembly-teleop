"""
The assembly_run part of the Assembly tab: Mark done and Reopen, the run's settings, the run, and where the robots are.

The actions are listed, with their marks, in the process panel; selecting one there makes it current here.

! Never hide a row or a button, and keep every changing line one line tall, so nothing moves under the cursor.
"""

from __future__ import annotations

from html import escape
from typing import TYPE_CHECKING, NamedTuple

from ...plugin_api.context import PluginContext
from ...shared.runs.runs import DONE, SPIRAL_SECONDS_RANGE, SPIRAL_TURNS_RANGE, RunSettings
from ...shared.assembly_tab.layout import (MARK_BAND, MARK_ICON, REOPEN_ICON, RUN_BAND, RUN_SETTINGS_BAND,
                                           TAB_BUTTONS_CSS)
from ...ui.css import group_tooltips_css, side_by_side_css
from ...ui.second_press import again_label, show_waiting, waiting_group_css
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, block, chip, chip_line, hover, section, values
from .writes import AGAIN_MARK, AGAIN_NEW_RUN

if TYPE_CHECKING:
    from ...shared.assembly_run_contract import RunView
    from .plugin import AssemblyRunPlugin

#: The run buttons.
CONTINUE_NEWEST, CONTINUE_MATCH, NEW_RUN = "Continue newest", "Continue match", "New run"
NEW_RUN_AGAIN = again_label(NEW_RUN)
#: Mark done's own colour, shown again once it stops waiting for its second press.
MARK_COLOR = "yellow"


class PanelStatus(NamedTuple):
    """What the panel shows besides the view.

    Attributes:
        run: The run's name, or "".
        again: The press waiting for its second one (`AssemblyRunPlugin.again`), or "".
        message: The last line of feedback, and whether it is a failure.
        save_error: The last write that failed, or "".
        robots: One line per configured robot, live or where it was recorded (the plugin's `robot_lines`).
        settings_refused: How many settings requests were refused; a change puts the widgets back.
        spiral_fits: Whether the current action tightens a joint screw: only then is the Spiral box live.
        servo_fits: Whether the current action inserts a ground bar: only then is the Servo by bar box live.
    """

    run: str
    again: str
    message: tuple[str, bool]
    save_error: str
    robots: tuple[str, ...]
    settings_refused: int
    spiral_fits: bool
    servo_fits: bool


class RunPanel:
    """The panel's widgets, built once, and what `draw` writes into them. Main thread."""

    def __init__(self, ctx: PluginContext, plugin: AssemblyRunPlugin):
        """Build the panel in the Assembly tab; every click becomes one of the plugin's requests.

        Args:
            ctx: The plugin's context.
            plugin: The plugin the buttons call.
        """
        # The settings the widgets were last set to, the refusals counted then, and whether the spiral and servo fit.
        self._shown_settings: tuple[RunSettings, int, bool, bool] | None = None
        with ctx.view.ui(band=MARK_BAND) as gui:
            gui.add_html(section("mark done", SECTION_CTRL))
            # * Draws nothing: the tab's big buttons when the layout file asks (assembly_run's `big_buttons`), and
            #   Mark done and Reopen side by side once they exist.
            pair = gui.add_html("")
            self._mark = gui.add_button("Mark done", color=MARK_COLOR, icon=MARK_ICON,
                                        hint="Mark the current action done, recording every live robot where it is. "
                                             "Its robot not live: press again to record the export's end, "
                                             "assumed")
            reopen = gui.add_button("Reopen", color="gray", icon=REOPEN_ICON,
                                    hint="Put the current action back to pending")
            big = TAB_BUTTONS_CSS if ctx.view.preferences.get("big_buttons") is True else ""
            pair.content = f"<style>{big}{side_by_side_css([self._mark._impl.uuid, reopen._impl.uuid])}</style>"
            # ! Keep changing text BELOW the buttons.
            self._details = gui.add_html("")
        default = RunSettings()
        with ctx.view.ui(band=RUN_SETTINGS_BAND) as gui:
            self._split = gui.add_slider("Insert split mm", min=0.0, max=30.0, step=0.5,
                                         initial_value=default.insert_split_mm,
                                         hint="The insert's last millimetres run compliant")
            # * Greyed until the run's current action tightens a joint screw (`draw`): no action yet.
            self._spiral = gui.add_checkbox("Spiral", default.spiral, disabled=True,
                                            hint="The insert's compliant part searches in a spiral; greyed for a "
                                                 "ground bar, whose insert is rigid")
            self._spiral_seconds = gui.add_slider("Spiral s", min=SPIRAL_SECONDS_RANGE[0], max=SPIRAL_SECONDS_RANGE[1],
                                                  step=1.0, initial_value=default.spiral_seconds,
                                                  hint="Seconds the spiral takes to reach its full 15 mm radius")
            self._spiral_turns = gui.add_slider("Spiral turns", min=SPIRAL_TURNS_RANGE[0], max=SPIRAL_TURNS_RANGE[1],
                                                step=1, initial_value=default.spiral_turns,
                                                hint="Turns the spiral makes on its way out; pitch = 15 mm / turns")
            # * Greyed until the run's current action inserts a ground bar (`draw`): no step yet.
            self._servo = gui.add_checkbox("Servo by bar", default.servo_by_bar, disabled=True,
                                           hint="Servo a ground bar by its mocap fit instead of tool0 (on by "
                                                "default); greyed and unticked for any other bar, which always "
                                                "servos on tool0: it must mate with its joints")
            self._retreat_live = gui.add_checkbox("Retreat live axis", default.retreat_live_axis,
                                                  hint="The retreat backs each tool out along its own live -z as "
                                                       "the encoders read it (the export's distance); off: along "
                                                       "the export's axis")
            self._retreat_compliant = gui.add_checkbox("Retreat compliant", default.retreat_compliant,
                                                       hint="Test mode: the retreat breaks out under the compliance "
                                                            "controller instead of the stiff screw-out at the screw "
                                                            "rate (in the lab the compliant tools pushed the bar)")
        with ctx.view.ui(band=RUN_BAND) as gui:
            gui.add_html(section("run", SECTION_CTRL))
            self._status = gui.add_html("")
            # * A tip per button (`group_tooltips_css`), not viser's hint, which covers the whole row.
            self._run = gui.add_button_group("Run", [CONTINUE_NEWEST, CONTINUE_MATCH, NEW_RUN])
            tips = ("Continue the run used last; one made on another export of this process goes on in a new run",
                    "Write on in an older run made on these very files",
                    "Start a new run, every action pending: press again to confirm. Old runs are kept")
            gui.add_html(f"<style>{group_tooltips_css(self._run._impl.uuid, tips)}</style>")
            self._robots = gui.add_html("")
            # * Only a <style>: New run amber while it waits for its second press.
            self._waiting_css = gui.add_html("")

        self._mark.on_click(ctx.defer("mark done", plugin.request_mark_done))
        reopen.on_click(ctx.defer("reopen", plugin.request_reopen))
        self._run.on_click(ctx.defer_value("run button", lambda label: self._on_run(plugin, label)))
        # The turns go as a number: the plugin refuses one that is not whole, then makes it an int.
        settings = ((self._split, "insert_split_mm", float), (self._spiral, "spiral", bool),
                    (self._spiral_seconds, "spiral_seconds", float),
                    (self._spiral_turns, "spiral_turns", float), (self._servo, "servo_by_bar", bool),
                    (self._retreat_live, "retreat_live_axis", bool),
                    (self._retreat_compliant, "retreat_compliant", bool))
        # ! `operator_only`: viser also fires `on_update` when the plugin writes `widget.value` (skipped only when it
        #   is unchanged), and the Spiral and Servo by bar boxes are shown unticked while greyed: that write must not
        #   reach `request_settings`, or it would untick the stored setting. The test fake takes every call as the
        #   operator's, so no test can show this regression.
        for widget, setting, kind in settings:
            widget.on_update(ctx.defer_value(setting, lambda value, setting=setting, kind=kind:
                                             plugin.request_settings(**{setting: kind(value)}), operator_only=True))

    @staticmethod
    def _on_run(plugin: AssemblyRunPlugin, label: str) -> None:
        """Ask for what a run button says; New run reads "New run?" while it waits for its second press."""
        if label in (NEW_RUN, NEW_RUN_AGAIN):
            plugin.request_new_run()
        elif label == CONTINUE_NEWEST:
            plugin.request_continue()
        else:
            plugin.request_continue(plugin.view.match)

    def draw(self, view: RunView, status: PanelStatus) -> None:
        """Write the run into the widgets; a current action or movement the process lacks is shown, never raised on.

        Args:
            view: The run now.
            status: The rest the panel shows.
        """
        loaded = view.process
        schedule = loaded.process.schedule if loaded is not None else ()
        done = sum(value == DONE for value in view.statuses.values())
        if status.run:
            chips = chip(escape(status.run), OK, str(view.folder))
        elif view.offer and view.offer == view.match:
            chips = chip("newer run unreadable", BUSY, f"a newer run folder can't be read (see the log); run "
                                                       f"{view.match} was made on these files")
        elif view.offer:
            older = f"; run {view.match}, older, was made on these files" if view.match else ""
            chips = chip("export changed", BUSY, f"run {view.offer} was made on another export{older}")
        elif loaded is not None:
            chips = chip("no run yet", NONE, "this process has no run folder yet: a Run button starts one")
        else:
            chips = chip("no process", NONE, "no process loaded: Load one in the Process panel")
        chips += chip(f"{done}/{len(schedule)} done", OK if schedule and done == len(schedule) else NONE,
                      "actions of the process's schedule marked done in this run")
        if status.save_error:
            chips += chip("save error", FAIL, hover(status.save_error))
        self._status.content = chip_line(chips)

        # * Mark done names the action it marks, e.g. "B20_J done" beside its check icon (short: a half-width button at
        #   18 px); a press waiting for its second one says so on its button.
        action = view.current[0] if view.current is not None else None
        show_waiting(self._mark, action is not None and status.again == f"{AGAIN_MARK} {action}",
                     f"{action} done" if action is not None else "Mark done", MARK_COLOR)
        new_run = status.again == AGAIN_NEW_RUN
        self._run.options = (CONTINUE_NEWEST, CONTINUE_MATCH, NEW_RUN_AGAIN if new_run else NEW_RUN)
        css = f"<style>{waiting_group_css(self._run, 2)}</style>" if new_run else ""
        if self._waiting_css.content != css:
            self._waiting_css.content = css
        shown = (view.settings, status.settings_refused, status.spiral_fits, status.servo_fits)
        if shown != self._shown_settings:
            # * Only when the settings change, a change was refused or a fit changes; the plugin's own writes never
            #   come back as requests (`operator_only` above).
            self._shown_settings = shown
            settings = view.settings
            # * Greyed and shown unticked where they do not apply (the spiral: an action with no joint screw; the
            #   servo: a ground bar's action); the stored setting is kept.
            self._spiral.disabled, self._servo.disabled = not status.spiral_fits, not status.servo_fits
            self._split.value, self._spiral.value = settings.insert_split_mm, settings.spiral and status.spiral_fits
            self._servo.value = settings.servo_by_bar and status.servo_fits
            self._retreat_live.value, self._retreat_compliant.value = (settings.retreat_live_axis,
                                                                       settings.retreat_compliant)
            self._spiral_seconds.value, self._spiral_turns.value = settings.spiral_seconds, settings.spiral_turns

        message, failed = status.message
        self._details.content = block(values(_one_line(escape(f"current   {_where(view)}")))
                                      + _one_line(escape(message) or "&nbsp;", FAIL if failed else NONE))
        self._robots.content = block(values(*(_one_line(escape(line)) for line in status.robots)))


def _where(view: RunView) -> str:
    """The current action and movement as text; one the process lacks is said, not raised on."""
    current, loaded = view.current, view.process
    if current is None or loaded is None:
        return "—"
    action = loaded.process.actions.get(current[0])
    if action is None:
        return f"{current[0]} (not in this process)"
    movements = action.movements
    if not 0 <= current[1] < len(movements):
        return f"{current[0]} · movement {current[1] + 1} (it has {len(movements)})"
    return f"{current[0]} · {movements[current[1]].id} ({current[1] + 1}/{len(movements)})"


def _one_line(html: str, color: str | None = None) -> str:
    """Content on one line, cut with "…" when too wide; the whole line on hover."""
    tint = f"color:{color};font-size:11px;" if color else ""
    title = "" if html == "&nbsp;" else html
    return f'<div title="{title}" style="{tint}white-space:pre;overflow:hidden;text-overflow:ellipsis">{html}</div>'
