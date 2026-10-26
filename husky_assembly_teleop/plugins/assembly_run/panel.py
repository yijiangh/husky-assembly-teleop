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
from ...ui.assembly_tab import MARK_BAND, MARK_ICON, REOPEN_ICON, RUN_BAND, RUN_SETTINGS_BAND, TAB_BUTTONS_CSS
from ...ui.phone import side_by_side_css
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, SECTION_TOOL, block, chip, chip_line, section, values
from .writes import AGAIN_MARK, AGAIN_NEW_RUN

if TYPE_CHECKING:
    from ...shared.assembly_run_contract import RunView
    from .plugin import AssemblyRunPlugin

#: The run buttons.
CONTINUE_NEWEST, CONTINUE_MATCH, NEW_RUN = "Continue newest", "Continue match", "New run"


class PanelStatus(NamedTuple):
    """What the panel shows besides the view.

    Attributes:
        run: The run's name, or "".
        again: The click waiting for its second one (`AssemblyRunPlugin.again`), or "".
        message: The last line of feedback, and whether it is a failure.
        save_error: The last write that failed, or "".
        robots: One line per configured robot, live or where it was recorded (the plugin's `robot_lines`).
        settings_refused: How many settings requests were refused; a change puts the widgets back.
    """

    run: str
    again: str
    message: tuple[str, bool]
    save_error: str
    robots: tuple[str, ...]
    settings_refused: int


class RunPanel:
    """The panel's widgets, built once, and what `draw` writes into them. Main thread."""

    def __init__(self, ctx: PluginContext, plugin: AssemblyRunPlugin):
        """Build the panel in the Assembly tab; every click becomes one of the plugin's requests.

        Args:
            ctx: The plugin's context.
            plugin: The plugin the buttons call.
        """
        # The settings the widgets were last set to, and the refusals counted then.
        self._shown_settings: tuple[RunSettings, int] | None = None
        with ctx.view.ui(band=MARK_BAND) as gui:
            gui.add_html(section("action", SECTION_CTRL))
            # * Draws nothing: the tab's big buttons, and Mark done and Reopen side by side once they exist.
            pair = gui.add_html("")
            self._mark = gui.add_button("Mark done", color="yellow", icon=MARK_ICON,
                                        hint="Mark the current action done, recording every live robot where it is. "
                                             "Its robot not live: click again within 10 s to record the export's "
                                             "end, assumed")
            reopen = gui.add_button("Reopen", color="gray", icon=REOPEN_ICON,
                                    hint="Put the current action back to pending")
            pair.content = (f"<style>{TAB_BUTTONS_CSS}{side_by_side_css([self._mark._impl.uuid, reopen._impl.uuid])}"
                            f"</style>")
            # ! Keep changing text BELOW the buttons.
            self._details = gui.add_html("")
        default = RunSettings()
        with ctx.view.ui(band=RUN_SETTINGS_BAND) as gui:
            self._split = gui.add_slider("Insert split mm", min=0.0, max=30.0, step=0.5,
                                         initial_value=default.insert_split_mm,
                                         hint="The insert's last millimetres run compliant")
            self._spiral = gui.add_checkbox("Spiral", default.spiral,
                                            hint="The insert's compliant part searches in a spiral")
            self._rigid = gui.add_checkbox("Rigid insert", default.rigid_only,
                                           hint="The whole insert under joint control, the screw tightens, no "
                                                "compliance; the split and the spiral are ignored")
            self._spiral_seconds = gui.add_slider("Spiral s", min=SPIRAL_SECONDS_RANGE[0], max=SPIRAL_SECONDS_RANGE[1],
                                                  step=1.0, initial_value=default.spiral_seconds,
                                                  hint="Seconds the spiral takes to reach its full 15 mm radius")
            self._spiral_turns = gui.add_slider("Spiral turns", min=SPIRAL_TURNS_RANGE[0], max=SPIRAL_TURNS_RANGE[1],
                                                step=1, initial_value=default.spiral_turns,
                                                hint="Turns the spiral makes on its way out; pitch = 15 mm / turns")
            self._servo = gui.add_checkbox("Servo by bar", default.servo_by_bar,
                                           hint="Servo a grounded bar by its mocap fit instead of tool0")
        with ctx.view.ui(band=RUN_BAND) as gui:
            gui.add_html(section("run", SECTION_TOOL))
            self._status = gui.add_html("")
            self._run = gui.add_button_group("Run", [CONTINUE_NEWEST, CONTINUE_MATCH, NEW_RUN],
                                             hint="Continue the run used last, made on another export of this "
                                                  "process, in a new run; or write on in an older run made on these "
                                                  "very files; or start a new run, every action pending (click again "
                                                  "within 10 s). Old runs are kept")
            self._robots = gui.add_html("")

        self._mark.on_click(ctx.defer("mark done", plugin.request_mark_done))
        reopen.on_click(ctx.defer("reopen", plugin.request_reopen))
        self._run.on_click(ctx.defer_value("run button", lambda label: self._on_run(plugin, label)))
        # The turns go as a number: the plugin refuses one that is not whole, then makes it an int.
        settings = ((self._split, "insert_split_mm", float), (self._spiral, "spiral", bool),
                    (self._rigid, "rigid_only", bool), (self._spiral_seconds, "spiral_seconds", float),
                    (self._spiral_turns, "spiral_turns", float), (self._servo, "servo_by_bar", bool))
        for widget, setting, kind in settings:
            widget.on_update(ctx.defer_value(setting, lambda value, setting=setting, kind=kind:
                                             plugin.request_settings(**{setting: kind(value)})))

    @staticmethod
    def _on_run(plugin: AssemblyRunPlugin, label: str) -> None:
        """Ask for what a run button says; New run reads "New run: click again" while it waits for the second click."""
        if label.startswith(NEW_RUN):
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
        else:
            chips = chip("no run yet" if loaded is not None else "no process", NONE)
        chips += chip(f"{done}/{len(schedule)} done", OK if schedule and done == len(schedule) else NONE)
        if status.save_error:
            chips += chip("save error", FAIL, status.save_error)
        self._status.content = chip_line(chips)

        # * Mark done names the action it marks, e.g. "B20_J done" beside its check icon (short: a half-width button at
        #   18 px); a click waiting for its second one says so on its button.
        action = view.current[0] if view.current is not None else None
        if action is not None and status.again == f"{AGAIN_MARK} {action}":
            self._mark.label = "Click again"
        else:
            self._mark.label = f"{action} done" if action is not None else "Mark done"
        self._run.options = (CONTINUE_NEWEST, CONTINUE_MATCH,
                             f"{NEW_RUN}: click again" if status.again == AGAIN_NEW_RUN else NEW_RUN)
        if (view.settings, status.settings_refused) != self._shown_settings:
            # ? Only when the settings change or a change was refused: the operator's own accepted click comes back
            #   as a request for what is already set, which changes nothing.
            self._shown_settings = (view.settings, status.settings_refused)
            settings = view.settings
            self._split.value, self._spiral.value = settings.insert_split_mm, settings.spiral
            self._rigid.value, self._servo.value = settings.rigid_only, settings.servo_by_bar
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
    """Content on one line, cut with "…" when too wide."""
    tint = f"color:{color};font-size:11px;" if color else ""
    return f'<div style="{tint}white-space:pre;overflow:hidden;text-overflow:ellipsis">{html}</div>'
