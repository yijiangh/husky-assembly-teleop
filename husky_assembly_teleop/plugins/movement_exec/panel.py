"""
The movement_exec part of the Assembly tab and the phone's Exec tab: what runs, the prompt, Confirm, Cancel, Dry run.

A live line below the buttons follows the compliant part running.

! Never hide a row or a button, and keep every changing line one line tall, so nothing moves under the cursor.
"""

from __future__ import annotations

from html import escape
from typing import TYPE_CHECKING

import numpy as np
import viser

from ...plugin_api.context import PluginContext
from ...robot_interface.end_effectors import ScaffoldingV3
from ...ui.assembly_tab import CANCEL_ICON, CONFIRM_ICON, EXEC_SETTINGS_BAND, PROMPT_BAND
from ...ui.phone import big_buttons, phone_text, side_by_side_css
from ...ui.style import BUSY, FAIL, NONE, SECTION_CTRL, block, chip, chip_line, section

if TYPE_CHECKING:
    from ...shared.execution.operator import Prompt
    from ...shared.execution.stream import Recorder
    from .plugin import MovementExecPlugin

#: Cancel's hover text, on the desktop and the phone.
CANCEL_HINT = "Answer no; with no prompt open, stop the running step and hold every arm where it is"


class ExecPanel:
    """The panel's widgets, built once, and what `draw` writes into them. Main thread."""

    def __init__(self, ctx: PluginContext, plugin: MovementExecPlugin):
        """Build the panel in the Assembly tab and the phone tab; every click becomes one of the plugin's requests.

        Args:
            ctx: The plugin's context.
            plugin: The plugin the buttons call.
        """
        # The dry run and refusal count the checkbox was last set to.
        self._shown: tuple[bool, int] | None = None
        with ctx.view.ui(band=PROMPT_BAND) as gui:
            gui.add_html(section("exec", SECTION_CTRL))
            self._status = gui.add_html("")
            self._prompt = gui.add_html("")
            # * Draws nothing: Confirm and Cancel side by side, once they exist.
            pair = gui.add_html("")
            confirm = gui.add_button("Confirm", color="green", icon=CONFIRM_ICON, hint="Do what the prompt says")
            cancel = gui.add_button("Cancel", color="red", icon=CANCEL_ICON, hint=CANCEL_HINT)
            pair.content = f"<style>{side_by_side_css([confirm._impl.uuid, cancel._impl.uuid])}</style>"
            # ! Keep changing text BELOW the buttons.
            self._live = gui.add_html("")
            self._details = gui.add_html("")
            self._servo = gui.add_html("")
        with ctx.view.ui(band=EXEC_SETTINGS_BAND) as gui:
            self._dry = gui.add_checkbox("Dry run (every check, nothing sent)", False,
                                         hint="Exec runs every check and builds every trajectory, and sends nothing")
        confirm.on_click(ctx.defer("exec confirm", plugin.request_confirm))
        cancel.on_click(ctx.defer("exec cancel", plugin.request_cancel))
        self._dry.on_update(ctx.defer_value("dry run", lambda value: plugin.request_dry_run(bool(value))))
        with ctx.view.phone("Exec", viser.Icon.PLAYER_PLAY) as phone:
            phone.add_html(f"<style>{big_buttons([CONFIRM_ICON, CANCEL_ICON], 72, 30)}</style>")
            self._phone_status = phone.add_html("")
            self._phone_prompt = phone.add_html("")
            confirm = phone.add_button("Confirm", color="green", icon=CONFIRM_ICON)
            cancel = phone.add_button("Cancel", color="red", icon=CANCEL_ICON, hint=CANCEL_HINT)
            self._phone_live = phone.add_html("")
            self._phone_last = phone.add_html("")
            self._phone_servo = phone.add_html("")
        confirm.on_click(ctx.defer("exec confirm (phone)", plugin.request_confirm))
        cancel.on_click(ctx.defer("exec cancel (phone)", plugin.request_cancel))

    def draw(self, active: str, prompt: Prompt | None, dry_run: bool, last: tuple[str, bool], refused: int,
             recorder: Recorder | None = None, servo: str = "") -> None:
        """Write the state into the widgets.

        Args:
            active: "<robot> · <label>" while a request runs, else "".
            prompt: The open prompt, or None.
            dry_run: Whether the dry run is on.
            last: The last result as one line, and whether it failed.
            refused: How many dry-run changes were refused; a change puts the checkbox back.
            recorder: The samples of the compliant part running, or None.
            servo: The running servo's last reading, e.g. "servo 2/8 · tool0 L 0.31 R 0.27 mm", or how the last servo
                ended (grey once the request is over); "" for none.
        """
        chips = chip("DRY RUN", BUSY, "every check, nothing sent") if dry_run else ""
        chips += chip(escape(active), BUSY) if active else chip("idle", NONE)
        if prompt is not None:
            chips += chip("waiting for Confirm", FAIL if prompt.warn else BUSY)
        line = _one_line(escape(prompt.line) if prompt is not None else "no prompt",
                         (FAIL if prompt.warn else BUSY) if prompt is not None else NONE)
        self._status.content = chip_line(chips)
        self._prompt.content = block(line)
        self._phone_status.content = phone_text(chip_line(chips))
        self._phone_prompt.content = phone_text(block(line))
        readout = escape(live_line(recorder)) or "no compliant part running"
        live = _one_line(readout, BUSY if recorder is not None else NONE)
        self._live.content = block(live)
        self._phone_live.content = phone_text(block(live))
        if (dry_run, refused) != self._shown:
            self._shown = (dry_run, refused)
            self._dry.value = dry_run
        message, failed = last
        self._details.content = block(_one_line(escape(message) or "&nbsp;", FAIL if failed else NONE))
        # * On the phone the whole line, wrapped: a HOLD NOT SENT must be read in full next to the robot.
        self._phone_last.content = phone_text(block(f'<div style="color:{FAIL if failed else NONE}">'
                                                    f'{escape(message) or "&nbsp;"}</div>'))
        # * One line high, always there: the servo's readings come and go without moving anything.
        line = _one_line(escape(servo) or "&nbsp;", BUSY if servo and active else NONE)
        self._servo.content = block(line)
        self._phone_servo.content = phone_text(block(line))


def live_line(recorder: Recorder | None) -> str:
    """The compliant part running, from its last sample: phase · e · stalled motors · |F| per arm; "" without one."""
    if recorder is None:
        return ""
    if not recorder.samples:
        return f"{recorder.phase or 'starting'} · no sample yet"
    sample = recorder.samples[-1]
    stalled = [f"{arm} M{motor}" for arm, motors in sample["motors"].items()
               for motor, state in enumerate(motors, 1) if state == ScaffoldingV3.STALLED]
    forces = [f"{arm} {np.linalg.norm(each['wrench'][:3]):.1f} N" for arm, each in sample["arms"].items()
              if each["wrench"] is not None]
    return (f"{recorder.phase or 'starting'} · e {sample['t']:.1f} s · stalled: {', '.join(stalled) or 'none'} · "
            f"|F| {', '.join(forces) or 'not measured'}")


def _one_line(html: str, color: str) -> str:
    """Content on one line, cut with "…" when too wide; the whole line on hover."""
    return (f'<div title="{html}" style="color:{color};font-size:12px;white-space:nowrap;overflow:hidden;'
            f'text-overflow:ellipsis">{html}</div>')
