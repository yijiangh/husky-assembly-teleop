"""
The Plots tab beside Assembly: the servo's readings pass by pass, and a compliant part's force and torque each tick.

Fed by the executor (`servo_reading`, `compliant_sample`), drawn each tick; the curves of a part stay until the next
part of its kind starts. Drag the tab out of the side panel for a window of its own.
"""

from __future__ import annotations

from typing import Mapping, Sequence

import numpy as np
import viser

from ...plugin_api.trace import Signal, Trace
from ...ui.style import NONE, chip, chip_line
from ...ui.trace_plot import TracePlot

#: Line colours: the first arm's x, y, z and |d| in reds, the second's in greens, the base's in blues.
FIRST_ARM = ("#e03131", "#f06595", "#c92a2a", "#ffa8a8")
SECOND_ARM = ("#2f9e44", "#69db7c", "#2b8a3e", "#b2f2bb")
BASE = ("#4263eb", "#74c0fc", "#1c7ed6", "#a5d8ff")
#: Samples kept per trace: passes of a servo (far more than any), ticks of a compliant part (a 300 s hold at 20 Hz).
SERVO_SAMPLES, COMPLIANT_SAMPLES = 200, 6000
#: Sides in series order; a one-arm robot fills the first family, the other stays NaN.
SIDES = ("left", "right")
#: Arm names in series order for the force plots, by arm name as the robot's arms are listed.


def _nothing() -> None:
    """A signal fed by `Trace.add`, never read."""
    return None


class ExecPlots:
    """The Plots tab's traces and plots."""

    def __init__(self, gui: viser.GuiApi):
        """Build the plots inside `gui` (the tab): the servo's five, then the compliant part's two."""
        per_side = tuple(f"{side[0].upper()} {axis}" for side in SIDES for axis in ("x", "y", "z", "|d|"))
        per_axis = tuple(f"{side[0].upper()} {axis}" for side in SIDES for axis in "xyz")
        self._servo = Trace((
            Signal("tool0_pos_err", _nothing, per_side, "mm"),
            Signal("tool0_rot_err", _nothing, per_axis, "deg"),
            Signal("base_pos_diff", _nothing, ("x", "y", "z", "|d|"), "mm"),
            Signal("base_rot_diff", _nothing, ("x", "y", "z"), "deg"),
            Signal("bar", _nothing, ("mocap vs design", "FK vs design"), ("mm", "mm"))), SERVO_SAMPLES)
        self._compliant = Trace((Signal("force", _nothing, per_axis, "N"),
                                 Signal("torque", _nothing, per_axis, "Nm")), COMPLIANT_SAMPLES)
        self._note = gui.add_html(chip_line(""))
        arms = FIRST_ARM + SECOND_ARM
        self._plots = [
            TracePlot(gui, self._servo, "tool0_pos_err", seconds=None, min_span=0.1, x_label="pass", colors=arms),
            TracePlot(gui, self._servo, "tool0_rot_err", seconds=None, min_span=0.01, x_label="pass",
                      colors=FIRST_ARM[:3] + SECOND_ARM[:3]),
            TracePlot(gui, self._servo, "base_pos_diff", seconds=None, min_span=0.1, x_label="pass", colors=BASE),
            TracePlot(gui, self._servo, "base_rot_diff", seconds=None, min_span=0.01, x_label="pass",
                      colors=BASE[:3]),
            TracePlot(gui, self._servo, "bar", seconds=None, min_span=0.1, x_label="pass", colors=BASE[:2]),
            TracePlot(gui, self._compliant, "force", seconds=None, min_span=1.0, x_label="s",
                      colors=FIRST_ARM[:3] + SECOND_ARM[:3]),
            TracePlot(gui, self._compliant, "torque", seconds=None, min_span=0.1, x_label="s",
                      colors=FIRST_ARM[:3] + SECOND_ARM[:3])]
        self._arms: tuple[str, ...] = ()
        self._label = ""
        self._no_wrench = False

    def start_servo(self, label: str) -> None:
        """Forget the last servo's points: a new servo part starts; its curves stay until the next."""
        self._servo.clear()
        self._label = label
        self._note.content = chip_line(chip(f"servo: {label}", NONE))

    def servo_reading(self, reading: Mapping) -> None:
        """One point per pass at x = the pass: tool0 errors per side, the base's drift, the bar's deviation."""
        k = float(reading.get("iter", len(self._servo.times("tool0_pos_err"))))
        tool0, base, bar = reading.get("tool0", {}), reading.get("base", {}), reading.get("bar") or {}
        pos = [value for side in SIDES for value in _side(tool0, side, "pos_err_mm", "pos_norm_mm", 4)]
        rot = [value for side in SIDES for value in _side(tool0, side, "rot_err_deg", None, 3)]
        base_pos = [*base.get("pos_diff_mm", [np.nan] * 3), base.get("pos_norm_mm", np.nan)]
        base_rot = list(base.get("rot_diff_deg", [np.nan] * 3))
        bar_values = [_bar(bar, "mocap_vs_design"), _bar(bar, "fk_vs_design")]
        self._servo.add(k, pos, rot, base_pos, base_rot, bar_values)

    def start_compliant(self, arms: Sequence[str], label: str) -> None:
        """Forget the last compliant part's samples: a new insert, retreat or handoff starts on these arms."""
        self._compliant.clear()
        self._arms, self._label, self._no_wrench = tuple(arms), label, False
        self._note.content = chip_line(chip(f"compliant: {label}", NONE))

    def compliant_sample(self, sample: Mapping) -> None:
        """One point at x = the sample's seconds: force [N] and torque [Nm] per arm; no wrench plots 0 and says so."""
        force, torque, missing = [], [], False
        for index in range(len(SIDES)):
            arm = self._arms[index] if index < len(self._arms) else None
            wrench = sample.get("arms", {}).get(arm, {}).get("wrench") if arm is not None else None
            if arm is not None and wrench is None:
                missing = True
            values = [np.nan] * 6 if arm is None else ([0.0] * 6 if wrench is None else list(wrench))
            force += values[:3]
            torque += values[3:6]
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
    values = list(error.get(key, []))
    if norm is not None:
        values.append(error.get(norm, np.nan))
    return (values + [np.nan] * count)[:count]


def _bar(bar: Mapping, key: str) -> float:
    """A bar reading's position deviation, mm, or NaN."""
    entry = bar.get(key)
    return float(entry.get("dpos_mm", np.nan)) if isinstance(entry, Mapping) else np.nan
