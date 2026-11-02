"""
The Exec plots window: the servo's readings pass by pass, and a compliant part's force and torque each tick.

Fed by the executor (`servo_reading`, `compliant_sample`, `status`), drawn each tick; the curves of a part stay until
the next part of its kind starts. When a part starts, its kind's folder shows, the other's hides, and the window comes
forward (`expand`), as the old monitor's windows opened; only the operator closes it. Under each servo plot a readout
gives the last pass's numbers (the legend shows the mouse's, not the last), and the chip line says which phase the
pass is in.
"""

from __future__ import annotations

from html import escape
from typing import Mapping, Sequence

import numpy as np
import viser

from ...plugin_api.trace import Signal, Trace
from ...shared.execution.servo_record import BASE_COLORS, FIRST_ARM_COLORS, SECOND_ARM_COLORS
from ...ui.style import NONE, chip, chip_line, numbers, values
from ...ui.trace_plot import TracePlot

#: Samples kept per trace: passes of a servo (far more than any), ticks of a compliant part (a 300 s hold at 20 Hz).
SERVO_SAMPLES, COMPLIANT_SAMPLES = 200, 6000
#: Sides in series order; a one-arm robot fills the first family, the other stays NaN.
SIDES = ("left", "right")
#: The servo plots, in order: signal, the readout rows under it as (label, channel count), the number width.
SERVO_PLOTS = (("tool0_pos_err", (("L", 4), ("R", 4)), 8), ("tool0_rot_err", (("L", 3), ("R", 3)), 8),
               ("base_pos_diff", (("base", 4),), 8), ("base_rot_diff", (("base", 3),), 8),
               ("bar", (("bar", 2),), 8), ("force_change", (("N", 2),), 8))


def _nothing() -> None:
    """A signal fed by `Trace.add`, never read."""
    return None


class ExecPlots:
    """The Exec plots window's traces and plots: the servo's in one folder, the compliant part's in another."""

    def __init__(self, gui: viser.GuiApi, panel: viser.PanelHandle):
        """Build the plots inside `gui` (the window's tab); `panel` is the window, brought forward as a part starts."""
        per_side = tuple(f"{side[0].upper()} {axis}" for side in SIDES for axis in ("x", "y", "z", "|d|"))
        per_axis = tuple(f"{side[0].upper()} {axis}" for side in SIDES for axis in "xyz")
        self._servo = Trace((
            Signal("tool0_pos_err", _nothing, per_side, "mm"),
            Signal("tool0_rot_err", _nothing, per_axis, "deg"),
            Signal("base_pos_diff", _nothing, ("x", "y", "z", "|d|"), "mm"),
            Signal("base_rot_diff", _nothing, ("x", "y", "z"), "deg"),
            Signal("bar", _nothing, ("mocap vs design", "FK vs design"), ("mm", "mm")),
            Signal("force_change", _nothing, ("L", "R"), "N")), SERVO_SAMPLES)
        self._compliant = Trace((Signal("force", _nothing, per_axis, "N"),
                                 Signal("torque", _nothing, per_axis, "Nm")), COMPLIANT_SAMPLES)
        self._panel = panel
        self._note = gui.add_html(chip_line(""))
        arms = FIRST_ARM_COLORS + SECOND_ARM_COLORS
        colors = {"tool0_pos_err": arms, "tool0_rot_err": FIRST_ARM_COLORS[:3] + SECOND_ARM_COLORS[:3],
                  "base_pos_diff": BASE_COLORS, "base_rot_diff": BASE_COLORS[:3], "bar": BASE_COLORS[:2],
                  "force_change": (FIRST_ARM_COLORS[3], SECOND_ARM_COLORS[3])}
        spans = {"tool0_pos_err": 0.1, "tool0_rot_err": 0.01, "base_pos_diff": 0.1, "base_rot_diff": 0.01,
                 "bar": 0.1, "force_change": 1.0}
        self._servo_folder = gui.add_folder("servo")
        #: Signal -> the readout under its plot, holding the last pass's numbers.
        self._last: dict[str, viser.GuiHtmlHandle] = {}
        with self._servo_folder:
            self._plots = []
            for signal, rows, _width in SERVO_PLOTS:
                self._plots.append(TracePlot(gui, self._servo, signal, seconds=None, min_span=spans[signal],
                                             x_label="pass", colors=colors[signal], points=True, live_legend=False))
                self._last[signal] = gui.add_html(self._readout(signal, None))
        self._compliant_folder = gui.add_folder("force and torque")
        with self._compliant_folder:
            self._plots += [
                TracePlot(gui, self._compliant, "force", seconds=None, min_span=1.0, x_label="s",
                          colors=FIRST_ARM_COLORS[:3] + SECOND_ARM_COLORS[:3]),
                TracePlot(gui, self._compliant, "torque", seconds=None, min_span=0.1, x_label="s",
                          colors=FIRST_ARM_COLORS[:3] + SECOND_ARM_COLORS[:3])]
        self._arms: tuple[str, ...] = ()
        self._label = ""
        self._no_wrench = False

    def _show(self, servo: bool) -> None:
        """Show the running kind's folder, hide the other, and bring the window forward."""
        self._servo_folder.visible, self._compliant_folder.visible = servo, not servo
        self._panel.expand()

    def start_servo(self, label: str) -> None:
        """Forget the last servo's points: a new servo part starts; its curves stay until the next."""
        self._servo.clear()
        # * x is the pass number: counted from 0 even when reading 0 never came.
        self._servo.start = 0.0
        self._label = label
        self._note.content = chip_line(chip(f"servo: {label}", NONE))
        for signal, _rows, _width in SERVO_PLOTS:
            self._last[signal].content = self._readout(signal, None)
        self._show(servo=True)

    def status(self, text: str, color: str) -> None:
        """The servo's phase as a chip beside its label, on the one chip line that is always there."""
        self._note.content = chip_line(chip(f"servo: {self._label}", NONE) + chip(escape(text), color))

    def servo_reading(self, reading: Mapping) -> None:
        """One point per pass at x = the pass: tool0 errors per side, the base's drift, the bar, the force change."""
        k = float(reading.get("iter", len(self._servo.times("tool0_pos_err"))))
        tool0, base, bar = reading.get("tool0", {}), reading.get("base", {}), reading.get("bar") or {}
        if "arm" in tool0:  # a one-arm robot's side (`arm_side`)
            tool0 = {SIDES[0]: tool0["arm"]}
        pos = [value for side in SIDES for value in _side(tool0, side, "pos_err_mm", "pos_norm_mm", 4)]
        rot = [value for side in SIDES for value in _side(tool0, side, "rot_err_deg", None, 3)]
        base_pos = [*base.get("pos_diff_mm", [np.nan] * 3), base.get("pos_norm_mm", np.nan)]
        base_rot = list(base.get("rot_diff_deg", [np.nan] * 3))
        bar_values = [_bar(bar, "mocap_vs_design"), _bar(bar, "fk_vs_design")]
        change = reading.get("force_change_N") or {}
        force = [change.get(side, np.nan) for side in SIDES]
        self._servo.add(k, pos, rot, base_pos, base_rot, bar_values, force)
        for signal, series in (("tool0_pos_err", pos), ("tool0_rot_err", rot), ("base_pos_diff", base_pos),
                               ("base_rot_diff", base_rot), ("bar", bar_values), ("force_change", force)):
            self._last[signal].content = self._readout(signal, series, int(k))

    def _readout(self, signal: str, series: Sequence[float] | None, k: int = 0) -> str:
        """The rows under a servo plot: the pass, then each family's last numbers ("—" before any, or all NaN)."""
        rows, width = next((rows, width) for name, rows, width in SERVO_PLOTS if name == signal)
        lines, start = [f"pass {k:>2}" if series is not None else "pass  —"], 0
        for label, count in rows:
            family = None if series is None else list(series[start:start + count])
            if family is not None and all(np.isnan(family)):
                family = None
            lines.append(f"{label:<4} " + numbers(family, count, width, 2))
            start += count
        return values(*lines)

    def start_compliant(self, arms: Sequence[str], label: str) -> None:
        """Forget the last compliant part's samples: a new insert, retreat or handoff starts on these arms."""
        self._compliant.clear()
        self._arms, self._label, self._no_wrench = tuple(arms), label, False
        self._note.content = chip_line(chip(f"compliant: {label}", NONE))
        self._show(servo=False)

    def compliant_sample(self, sample: Mapping) -> None:
        """One point at x = the sample's seconds: force [N] and torque [Nm] per arm; no wrench plots 0 and says so."""
        force, torque, missing = [], [], False
        for index in range(len(SIDES)):
            arm = self._arms[index] if index < len(self._arms) else None
            wrench = sample.get("arms", {}).get(arm, {}).get("wrench") if arm is not None else None
            if arm is not None and wrench is None:
                missing = True
            values_ = [np.nan] * 6 if arm is None else ([0.0] * 6 if wrench is None else list(wrench))
            force += values_[:3]
            torque += values_[3:6]
        self._compliant.add(float(sample.get("t", 0.0)), force, torque)
        if missing and not self._no_wrench:
            self._no_wrench = True
            self._note.content = chip_line(chip(f"compliant: {self._label}", NONE)
                                           + chip("no force reading: the arms report no wrench", NONE))

    def draw(self) -> None:
        """Send new samples to the browser."""
        for plot in self._plots:
            plot.draw()


def _side(tool0: Mapping, side: str, key: str, norm: str | None, count: int) -> list[float]:
    """A side's `key` values from a reading's tool0 errors, plus its `norm`, or NaNs."""
    error = tool0.get(side)
    if error is None:
        return [np.nan] * count
    values_ = list(error.get(key, []))
    if norm is not None:
        values_.append(error.get(norm, np.nan))
    return (values_ + [np.nan] * count)[:count]


def _bar(bar: Mapping, key: str) -> float:
    """A bar reading's position deviation, mm, or NaN."""
    entry = bar.get(key)
    return float(entry.get("dpos_mm", np.nan)) if isinstance(entry, Mapping) else np.nan
