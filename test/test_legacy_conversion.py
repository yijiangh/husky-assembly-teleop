"""The two benchmark exports convert into valid schema 2 designs, with the plan checks' known findings only.

Runs on the exports in the folder named by HUSKY_DESIGN_STUDY (holding `260814_RobArch_support_ik` and
`260920_RobArch_demo_revamp_backup`); skipped when unset. ! Slow: ~20 s per export.
"""

from __future__ import annotations

import os
from pathlib import Path

import pytest

from bar_assembly_core.design import read
from bar_assembly_core.design.plan_check import check_plan
from bar_assembly_core.legacy.conversion import convert_export

STUDY = Path(os.environ.get("HUSKY_DESIGN_STUDY", "/nonexistent"))
DATA = Path(__file__).resolve().parent.parent / "data"
#: Plan check errors that are in the exports themselves.
#: - B11: in the next hold release, the export still shows Alice at her grasp pose after her retreat.
#: - B14 (260920 only): the export's joints at B16's release put B16 295 mm from its design pose.
KNOWN_ERRORS = {
    "260814_RobArch_support_ik": {
        "B11: B3_HR_retreat -> B7_HR_open: robots/alice joints jump by 0.444",
        "B11: B12_HR_retreat -> B15_HR_open: robots/alice joints jump by 0.29"},
}
KNOWN_ERRORS["260920_RobArch_demo_revamp_backup"] = KNOWN_ERRORS["260814_RobArch_support_ik"] | {
    f"B14: B16_R_ungrasp start: bars/B16 as held by robots/cindy/{side}_ur_arm_tool0 is 295 mm and "
    f"12.3° from the geometry" for side in ("left", "right")}
UNMATED = ("joints/J11-12_male", "joints/J14-15_male", "joints/J2-3_male", "joints/J6-7_male")

pytestmark = [pytest.mark.slow,
              pytest.mark.skipif(not STUDY.is_dir(), reason="HUSKY_DESIGN_STUDY does not name a folder")]


@pytest.fixture(scope="module", params=sorted(KNOWN_ERRORS))
def converted(request, tmp_path_factory):
    """One export converted and read back (A1–A13 pass, or reading raises)."""
    export = STUDY / request.param
    if not export.is_dir():
        pytest.skip(f"{export} is missing")
    folder = convert_export(export, tmp_path_factory.mktemp("design") / request.param, DATA, report=lambda _: None)
    return request.param, read(folder)


def test_counts_and_size(converted):
    """48 actions, 184 movements (tighten merged into the insert, untighten folded into the ungrasp), 36 mates."""
    _, design = converted
    assert len(design.actions) == 48 and sum(len(action.movements) for action in design.actions.values()) == 184
    assert len(design.mates) == 36 and all(body.mount for key, body in design.bodies.items()
                                           if key.startswith("joints/"))
    size = sum(path.stat().st_size for path in (design.folder / "actions").glob("*.json"))
    assert size < 0.76e6


def test_plan_checks(converted):
    """Only the exports' own inconsistencies are errors; the four halves on fake bars are warnings."""
    name, design = converted
    report = check_plan(design)
    assert set(report.errors) == KNOWN_ERRORS[name]
    assert [line.split(" ")[2] for line in report.warnings] == list(UNMATED)
    assert all(line.startswith("B5:") for line in report.warnings)


def test_jointing_and_release(converted):
    """B10: mount, grasp, transfer, one insert that builds the bar; the release ungrasps (form B), then retreats.

    The ungrasp backs the jointing screws off; movements are named by role, and `on` changes in targets.
    """
    _, design = converted
    assert [m.id for m in design.actions["B10_J"].movements] == [
        "B10_J_load", "B10_J_mount", "B10_J_grasp", "B10_J_transfer", "B10_J_insert"]
    mount, grasp, transfer, insert = design.actions["B10_J"].movements[1:]
    assert mount.ends_on == "operator" and [h.to.split("/")[-1] for h in mount.target.attached["bars/B10"]] == [
        "left_ur_arm_tool0", "right_ur_arm_tool0"]
    assert mount.target.on == {"tools/AT3L": "joints/J3-10_male", "tools/AT3R": "joints/J7-10_male"}
    assert grasp.start.tools["tools/AT3L"].on == "joints/J3-10_male"
    assert grasp.start.tools["tools/AT3R"].on == "joints/J7-10_male"
    assert transfer.coupled and transfer.controller == "position"
    assert (insert.path, insert.coupled, insert.controller, insert.ends_on) == ("linear", True, "compliant", "tools")
    assert insert.drives == {"tools/AT3L": "tighten", "tools/AT3R": "tighten"} and "bars/B10" in insert.target.built
    assert all(abs(line.distance - 0.015) < 1e-6 for line in insert.line.values())
    ungrasp, retreat, home = design.actions["B10_R"].movements
    assert [m.id for m in (ungrasp, retreat, home)] == ["B10_R_ungrasp", "B10_R_retreat", "B10_R_home"]
    assert not ungrasp.arms and ungrasp.target.attached == {} and set(ungrasp.grip_change.values()) == {"open"}
    assert ungrasp.drives == {"tools/AT3L": "loosen", "tools/AT3R": "loosen"} and ungrasp.target.built is None
    assert retreat.path == "linear" and retreat.start.tools["tools/AT3L"].on == "joints/J3-10_male"
    assert retreat.target.on == {"tools/AT3L": None, "tools/AT3R": None}
    assert home.start.tools["tools/AT3L"].on is None
    assert not {"lm_axis", "lm_distance_mm", "retreat_axes_world", "ends_on", "planner_fills"} & {
        key for action in design.actions.values() for movement in action.movements for key in movement.notes}


def test_support_hold(converted):
    """Alice holds the built B3 from her close until her open; until Cindy's ungrasp B3 has both robots' holders."""
    _, design = converted
    to_grasp, close = design.actions["B3_H"].movements[-2:]
    assert (to_grasp.id, close.id) == ("B3_H_to_grasp", "B3_H_close")
    assert to_grasp.target.on == {"tools/alice/SupportGripper": "bars/B3"}
    assert close.start.tools["tools/alice/SupportGripper"].on == "bars/B3"
    ungrasp = design.actions["B3_R"].movements[0]
    assert "bars/B3" in ungrasp.start.built
    assert {holder.to.split("/")[1] for holder in ungrasp.start.attached["bars/B3"]} == {"cindy", "alice"}
    release = design.actions["B3_HR"].movements[0]
    assert release.target.attached.get("bars/B3") is None and {"bars/B4", "bars/B9"} <= release.start.built
