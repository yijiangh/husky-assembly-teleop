"""
The assembly_run panel: the run, its actions, its buttons and prompt, its settings, and where the robots are.

! Never hide a row or a button, and keep every changing line one line tall, so nothing moves under the cursor.
"""

from __future__ import annotations

from html import escape
from typing import TYPE_CHECKING, NamedTuple

from ...plugin_api.context import PluginContext
from ...shared.runs.runs import DONE, RunSettings
from ...ui.checklist import ACTION_ROWS, action_window
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, SECTION_TOOL, block, chip, chip_line, section, values

if TYPE_CHECKING:
    from ...shared.assembly_run_contract import RunView
    from .plugin import AssemblyRunPlugin

#: Shown in place of the robot for an action no robot acts in.
BY_HAND = "by hand"
#: The run buttons.
CONTINUE_NEWEST, CONTINUE_MATCH, NEW_RUN = "Continue newest", "Continue match", "New run"


class PanelStatus(NamedTuple):
    """What the panel shows besides the view.

    Attributes:
        run: The run's name, or "".
        prompt: The question waiting for Confirm or Cancel, or "".
        message: The last line of feedback, and whether it is a failure.
        save_error: The last write that failed, or "".
        robots: One line per configured robot, live or where it was recorded (the plugin's `robot_lines`).
        settings_refused: How many settings requests were refused; a change puts the widgets back.
    """

    run: str
    prompt: str
    message: tuple[str, bool]
    save_error: str
    robots: tuple[str, ...]
    settings_refused: int


class RunPanel:
    """The panel's widgets, built once, and what `draw` writes into them. Main thread."""

    def __init__(self, ctx: PluginContext, plugin: AssemblyRunPlugin):
        """Build the panel; every click becomes one of the plugin's requests.

        Args:
            ctx: The plugin's context.
            plugin: The plugin the buttons call.
        """
        # The action shown in each row as last drawn, so a click is read against it.
        self._actions: list[str | None] = [None] * ACTION_ROWS
        # The settings the widgets were last set to, and the refusals counted then.
        self._shown_settings: tuple[RunSettings, int] | None = None
        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            self._title = gui.add_html(section("actions", SECTION_CTRL))
            self._rows = [gui.add_button("·", color="gray") for _ in range(ACTION_ROWS)]
            run = gui.add_button_group("Run", [CONTINUE_NEWEST, CONTINUE_MATCH, NEW_RUN],
                                       hint="Continue the run used last, made on another export of this process, in a "
                                            "new run; or write on in an older run made on these very files; or start "
                                            "a new run, every action pending (asks first). Old runs are kept")
            action = gui.add_button_group("Action", ["Mark done", "Reopen"],
                                          hint="Mark the current action done, recording every live robot where it "
                                               "is; or put it back to pending")
            prompt = gui.add_button_group("Prompt", ["Confirm", "Cancel"], hint="Answer the question below")
            self._prompt = gui.add_html("")
            gui.add_html(section("settings", SECTION_TOOL))
            self._transfer = gui.add_checkbox("Transfer workaround", True,
                                              hint="Run the transfer as a free move without the bar; the operator "
                                                   "places the bar at the insertion start")
            self._split = gui.add_slider("Insert split mm", min=0.0, max=30.0, step=0.5, initial_value=5.0,
                                         hint="The insert's last millimetres run compliant")
            self._spiral = gui.add_checkbox("Spiral", False, hint="The insert's compliant part searches in a spiral")
            self._servo = gui.add_checkbox("Servo by bar", False,
                                           hint="Servo a grounded bar by its mocap fit instead of tool0")
            # ! Keep changing text BELOW the buttons.
            self._details = gui.add_html("")

        for row, button in enumerate(self._rows):
            button.on_click(ctx.defer("select action", lambda row=row: self._select(plugin, row)))
        run.on_click(ctx.defer_value("run button", lambda label: self._run(plugin, label)))
        action.on_click(ctx.defer_value("action button", lambda label: plugin.request_mark_done()
                                        if label == "Mark done" else plugin.request_reopen()))
        prompt.on_click(ctx.defer_value("prompt button", lambda label: plugin.answer(label == "Confirm")))
        settings = ((self._transfer, "transfer_as_free", bool), (self._split, "insert_split_mm", float),
                    (self._spiral, "spiral", bool), (self._servo, "servo_by_bar", bool))
        for widget, setting, kind in settings:
            widget.on_update(ctx.defer_value(setting, lambda value, setting=setting, kind=kind:
                                             plugin.request_settings(**{setting: kind(value)})))

    def _select(self, plugin: AssemblyRunPlugin, row: int) -> None:
        """Make the action in a row current; empty rows do nothing."""
        if self._actions[row] is not None:
            plugin.request_set_current(self._actions[row])

    @staticmethod
    def _run(plugin: AssemblyRunPlugin, label: str) -> None:
        """Ask for what a run button says."""
        if label == NEW_RUN:
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

        current = view.current
        index = schedule.index(current[0]) if current is not None and current[0] in schedule else None
        top = action_window(index or 0, len(schedule))
        self._title.content = section("actions", SECTION_CTRL,
                                      f"{index + 1}/{len(schedule)}" if index is not None else "")
        for row, button in enumerate(self._rows):
            at = top + row
            self._actions[row] = schedule[at] if at < len(schedule) else None
            if self._actions[row] is None:
                button.label, button.color = "·", "gray"
                continue
            action = loaded.process.actions[schedule[at]]
            robot = loaded.process.robots[action.robot].name if action.robot else BY_HAND
            assumed = action.id in view.assumed
            label = f"{at + 1}. {action.id} · {robot} · {view.statuses[action.id]}"
            button.label = label + " (assumed)" if assumed else label
            button.color = ("blue" if index == at else "orange" if assumed
                            else "green" if view.statuses[action.id] == DONE else "gray")

        self._prompt.content = block(_one_line(escape(status.prompt) or "&nbsp;", BUSY if status.prompt else NONE))
        if (view.settings, status.settings_refused) != self._shown_settings:
            # ? Only when the settings change or a change was refused: the operator's own accepted click comes back
            #   as a request for what is already set, which changes nothing.
            self._shown_settings = (view.settings, status.settings_refused)
            self._transfer.value, self._split.value = view.settings.transfer_as_free, view.settings.insert_split_mm
            self._spiral.value, self._servo.value = view.settings.spiral, view.settings.servo_by_bar

        message, failed = status.message
        lines = [f"current   {_where(view)}", *status.robots]
        self._details.content = block(values(*(_one_line(escape(line)) for line in lines))
                                      + _one_line(escape(message) or "&nbsp;", FAIL if failed else NONE))


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
