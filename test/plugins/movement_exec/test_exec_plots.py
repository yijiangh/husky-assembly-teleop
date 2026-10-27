"""The Plots tab: a servo reading adds one point per pass, a compliant part's samples one per tick."""

from __future__ import annotations

import numpy as np
from plugin_fakes import FakeGui

from husky_assembly_teleop.plugins.movement_exec.plots import ExecPlots


class FakeUplot:
    """What `gui.add_uplot` gives back: keeps the data set on it."""

    def __init__(self, **fields):
        """Keep the data and scales the plot was built with."""
        self.data, self.scales = fields.get("data"), fields.get("scales")


def _plots() -> tuple[ExecPlots, FakeGui]:
    gui = FakeGui()
    gui.add_uplot = lambda **fields: FakeUplot(**fields)
    return ExecPlots(gui), gui


def _reading(k: int) -> dict:
    """A servo reading as `ServoLoop._measure` builds it, both sides, with a bar."""
    return {"iter": k,
            "tool0": {"left": {"pos_err_mm": [1.0, 2.0, 2.0], "pos_norm_mm": 3.0, "rot_err_deg": [0.1, 0.2, 0.3]},
                      "right": {"pos_err_mm": [0.0, 0.0, 4.0], "pos_norm_mm": 4.0, "rot_err_deg": [0.0, 0.0, 0.5]}},
            "base": {"pos_diff_mm": [0.5, 0.0, 0.0], "pos_norm_mm": 0.5, "rot_diff_deg": [0.0, 0.0, 0.01]},
            "bar": {"mocap_vs_design": {"dpos_mm": 1.5, "dang_deg": 0.2}, "fk_vs_design": {"dpos_mm": 1.1}}}


def test_a_servo_reading_is_one_point_per_pass_and_a_new_servo_starts_over():
    """Two readings make rows at x = 0 and 1 with the errors by side; `start_servo` empties the trace."""
    plots, _gui = _plots()
    plots.start_servo("B20_J_transfer on a200-0806")
    plots.servo_reading(_reading(0))
    plots.servo_reading(_reading(1))
    assert plots._servo.times("tool0_pos_err").tolist() == [0.0, 1.0]
    row = plots._servo.values("tool0_pos_err")[-1]
    np.testing.assert_allclose(row, [1.0, 2.0, 2.0, 3.0, 0.0, 0.0, 4.0, 4.0])
    np.testing.assert_allclose(plots._servo.values("tool0_rot_err")[-1], [0.1, 0.2, 0.3, 0.0, 0.0, 0.5])
    np.testing.assert_allclose(plots._servo.values("base_pos_diff")[-1], [0.5, 0.0, 0.0, 0.5])
    np.testing.assert_allclose(plots._servo.values("bar")[-1], [1.5, 1.1])
    plots.draw()
    plots.start_servo("another")
    assert len(plots._servo.times("tool0_pos_err")) == 0


def test_a_one_arm_reading_leaves_the_other_family_blank():
    """A support robot's reading fills the first family; the second is NaN, never zero."""
    plots, _gui = _plots()
    reading = _reading(3)
    reading["tool0"] = {"arm": reading["tool0"]["left"]}
    plots.servo_reading(reading)
    row = plots._servo.values("tool0_pos_err")[-1]
    assert np.all(np.isnan(row)), "a side that is neither left nor right is not drawn"
    reading["tool0"] = {"left": reading["tool0"]["arm"]}
    plots.servo_reading(reading)
    row = plots._servo.values("tool0_pos_err")[-1]
    assert row[:4].tolist() == [1.0, 2.0, 2.0, 3.0] and np.all(np.isnan(row[4:]))


def test_compliant_samples_split_force_and_torque_per_arm_and_say_when_a_wrench_is_missing():
    """A recorder sample with both wrenches fills six values each; a missing wrench plots zeros and a note."""
    plots, gui = _plots()
    plots.start_compliant(["left_ur_arm", "right_ur_arm"], "B21_J_insert on a200-0806")
    sample = {"t": 0.5, "arms": {"left_ur_arm": {"wrench": [0, 1, 2, 3, 4, 5]},
                                 "right_ur_arm": {"wrench": [6, 7, 8, 9, 10, 11]}}}
    plots.compliant_sample(sample)
    assert plots._compliant.times("force").tolist() == [0.5]
    np.testing.assert_allclose(plots._compliant.values("force")[-1], [0, 1, 2, 6, 7, 8])
    np.testing.assert_allclose(plots._compliant.values("torque")[-1], [3, 4, 5, 9, 10, 11])
    assert "no force reading" not in plots._note.content
    plots.compliant_sample({"t": 0.55, "arms": {"left_ur_arm": {"wrench": None}, "right_ur_arm": {"wrench": None}}})
    np.testing.assert_allclose(plots._compliant.values("force")[-1], [0] * 6)
    assert "no force reading" in plots._note.content
    plots.draw()
    assert len(plots._plots) == 7
