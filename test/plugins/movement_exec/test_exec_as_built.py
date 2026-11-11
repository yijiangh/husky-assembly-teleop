"""The as-built record around a part: the start of a request's first part, the end of every part, and a click.

On the fake lab with a fake mocap_bars plugin whose `measure_now` fits B20's fake markers at the hint it is given.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
from test_movement_exec import Lab, execute

from husky_assembly_teleop.experiments.as_built.build import result_folder
from husky_assembly_teleop.shared.bars.layouts import bar_layouts, fake_sensed_markers, fit_bar
from husky_assembly_teleop.shared.bars.pose import bar_slide_roll
from husky_assembly_teleop.shared.mocap_bars_contract import BarMeasurement
from husky_assembly_teleop.shared.movement_exec_contract import DONE
from scaffolding_assembly_core.geometry import Pose

BAR = "bars/B20"


class FakeMocapBars:
    """The mocap_bars plugin as movement_exec reads it: B20 fitted 30 mm slid and 2° rolled from the hint it gets."""

    name = "mocap_bars"

    def __init__(self, loaded, fail: bool = False):
        """Fits B20 of `loaded`; `fail` makes `measure_now` raise."""
        self.layout, self.fail, self.hints = bar_layouts(loaded.process)[BAR], fail, []
        self.fits: dict[str, BarMeasurement] = {}

    def measure_now(self, hints=None):
        """One measurement of B20 around its hint (its seat when none)."""
        if self.fail:
            raise RuntimeError("Motive went away")
        self.hints.append(dict(hints or {}))
        hint = (hints or {}).get(BAR)
        T = hint.matrix() if hint is not None else np.eye(4)
        sensed = fake_sensed_markers(T, self.layout.points, noise_m=0.0)
        fit = fit_bar(sensed, self.layout)
        found = BarMeasurement(BAR, self.layout, fit, Pose.from_matrix(fit.T_world_from_bar), sensed, "fake",
                               10.0, False, None, None)
        self.fits = {BAR: found}
        return dict(self.fits)


def lab_with_bars(current=("B20_R", 2), fail: bool = False) -> tuple[Lab, FakeMocapBars]:
    """The executor's lab with the fake mocap_bars plugin beside it."""
    lab = Lab(current=current)
    bars = FakeMocapBars(lab.loaded, fail)
    lab.ctx.plugins["mocap_bars"] = bars
    return lab, bars


def as_built(lab: Lab) -> list[tuple[dict, dict]]:
    """Every as-built record requested, as (event data, record), in order."""
    return [(fields["data"], fields["file"]) for event, fields in lab.run.records if event == "as_built"]


@pytest.mark.safety
def test_a_motion_records_its_start_and_its_end():
    """B20_R_home: two records, before the arms move and after they arrived, each with B20 fitted at its seat."""
    lab, bars = lab_with_bars()
    result, _held = execute(lab, lab.request("B20_R_home"))
    assert result.status == DONE, result.reason
    records = as_built(lab)
    assert [data["phase"] for data, _record in records] == ["start", "end"]
    assert [data["outcome"] for data, _record in records] == [None, "ok"]
    start, end = (record for _data, record in records)
    assert start["schema_version"] == 2 and start["problem"] == "261008_2ModulesDemo_purged_process"
    assert start["robot"] == "cindy" and start["serial"] == "a200-0806"
    assert start["action"]["id"] == "B20_R" and start["action"]["kind"] == "R" and start["action"]["bar_id"] == "B20"
    assert start["step"]["movement_id"] == "B20_R_home" and start["step"]["routine"] == "free_home"
    assert start["step"]["part"]["kind"] == "motion" and start["step"]["part"]["last"]
    assert end["step"]["outcome"] == "ok" and end["step"]["status"] == "ARRIVED" and not end["step"]["dry_run"]
    # B20 was released and stands built: measured against its seat, the fake's 30 mm slide and 2° roll read back.
    bar = end["bars"]["B20"]
    assert bar["bar_fit"]["ok"] and bar["reference_kind"] == "assembled" and bar["held_by"] is None
    assert bar["milestone"] is None, "a free move home is no milestone of the action's bar"
    assert bar["deviation_vs_reference"]["dpos_mm"] == pytest.approx(30.0, abs=0.5)
    assert bar["deviation_vs_reference"]["dang_deg"] == pytest.approx(2.0, abs=0.1)
    assert bar["deviation_vs_assembled"] == bar["deviation_vs_reference"]
    assert start["instrumented_bars"][0] == {"bar": BAR, "bar_id": "B20"}
    assert [entry["bar_id"] for entry in start["instrumented_bars"]] == ["B20", "B21", "B34", "B35"]
    # The hint handed to the fit is the seat (the reference), so a fit takes the quick match.
    np.testing.assert_allclose(bars.hints[0][BAR].matrix(), lab.loaded.process.bodies[BAR].assembly_pose.matrix())
    # The robot: live, both flanges by FK against the authored end, the other robots live too.
    assert end["robot_state"]["base_tracked"] and sorted(end["robot_state"]["arms"]) == ["left", "right"]
    # * A free move's export gives joints, not flange poses: no tool0 error to take, but the FK is kept.
    assert end["tool0"] == {} and end["robot_state"]["arms"]["left"]["tool0_fk"] is not None
    assert end["robot_state"]["arms"]["left"]["joints"][0] == pytest.approx(end["robots"]["robots/a200-0806"]["joints"][
        "left_ur_arm_shoulder_pan_joint"])
    assert sorted(end["robots"]) == ["robots/a200-0804", "robots/a200-0805", "robots/a200-0806"]
    cindy = end["robots"]["robots/a200-0806"]
    assert cindy["name"] == "cindy" and cindy["source"] == "live"
    assert end["options"]["dry_run"] is False and end["options"]["transfer_as_free"]["applies"] is False
    assert end["settings"]["servo_by_bar"] is True
    lines = [line for line in lab.ctx.lines("info") if line.startswith("[as-built] B20_R_home")]
    assert lines[0].startswith("[as-built] B20_R_home start: B20 fit ok 8/8, rms 0.0 mm | vs assembled dpos 30.0 mm")
    assert lines[1].startswith("[as-built] B20_R_home end (done): B20 fit ok 8/8")
    assert np.allclose(bar["T_reference"], lab.loaded.process.bodies[BAR].assembly_pose.matrix())
    fitted = Pose(tuple(bar["T_world_from_bar"][:3]), tuple(bar["T_world_from_bar"][3:])).matrix()
    expected = lab.loaded.process.bodies[BAR].assembly_pose.matrix() @ bar_slide_roll(0.03, 2.0)
    np.testing.assert_allclose(fitted, expected, atol=1e-3)


@pytest.mark.safety
def test_a_failed_measurement_is_said_and_the_step_still_runs():
    """`measure_now` raising: one error line per record, no record, the motion done all the same."""
    lab, _bars = lab_with_bars(fail=True)
    result, _held = execute(lab, lab.request("B20_R_home"))
    assert result.status == DONE and not as_built(lab)
    errors = [line for line in lab.ctx.lines("error") if line.startswith("[as-built]")]
    assert errors == ["[as-built] start of B20_R_home NOT recorded: RuntimeError('Motive went away')",
                      "[as-built] end of B20_R_home NOT recorded: RuntimeError('Motive went away')"]


@pytest.mark.safety
def test_record_now_takes_a_manual_record_of_the_current_step_and_the_button_asks_for_it():
    """Outside a request: one record, phase manual, the run's current step named; the panel button calls it."""
    lab, _bars = lab_with_bars(current=("B20_J", 3))
    lab.run.view.folder = None
    lab.plugin.request_as_built()
    records = as_built(lab)
    assert len(records) == 1 and records[0][0]["phase"] == "manual"
    record = records[0][1]
    assert record["step"]["phase"] == "manual" and record["step"]["movement_id"] == "B20_J_transfer"
    assert record["action"]["id"] == "B20_J" and record["bars"]["B20"]["milestone"] is None
    assert record["run_id"] == "" and record["servo"] is None
    button = next(widget for widget in lab.ctx.gui.widgets if widget.label == "As-built: record now")
    assert button.kind == "button"
    lab.ctx.plugins.pop("mocap_bars")
    lab.plugin.request_as_built()
    assert len(as_built(lab)) == 1
    assert "[as-built] the mocap_bars plugin is not loaded: nothing recorded" in lab.ctx.lines("info")


def test_the_as_built_page_is_written_to_the_days_result_folder():
    """The 3D page's path: `<drive>/.../<day>-asbuilt-result/`, the folder the server serves."""
    assert result_folder(Path("/drive"), "20261104") == Path(
        "/drive/data_experiment/bar_action_live_replan_exe/20261104-asbuilt-result")
