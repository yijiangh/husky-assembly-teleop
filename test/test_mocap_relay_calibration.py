"""Which base calibration the mocap relay applies: crl-husky's default, or the newest Drive session that is sane."""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from husky_assembly_teleop.mocap_relay.calibration import (SESSIONS_FOLDER, default_calibration, explain,
                                                           newest_session_file, resolve, robot_mocap_ids)

#: Alice's rigid body and serial in crl-husky's config.
ALICE_ID, ALICE = 1840, "a200-0804"


def _session(root: Path, day: str, content: dict, folder: str = "") -> Path:
    """Write a calibration session's output for Alice, e.g. on day "20261101"."""
    path = root / SESSIONS_FOLDER / day / folder / "calibrated_transformation_0804_rhino.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(content))
    return path


def _shifted(dx: float) -> dict:
    """Alice's default calibration moved by dx metres along x, as a session would write it (no mocap id)."""
    default = default_calibration(ALICE_ID)
    return {"base_mocap_from_ur_arm_base_link_inertia": [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]],
            "base_mocap_from_base_footprint": [[default.position[0] + dx, *default.position[1:]],
                                               list(default.orientation)]}


def test_robots_and_defaults():
    """Every robot's rigid body has a default; rigid bodies of no robot (the probe) have none."""
    assert robot_mocap_ids()[ALICE_ID] == ALICE
    assert default_calibration(ALICE_ID).session == "" and default_calibration(1863) is None


def test_no_drive_uses_the_default():
    """Without a Drive root, the default."""
    calibration, warnings = resolve(ALICE_ID, ALICE, None)
    assert calibration == default_calibration(ALICE_ID) and warnings == []


def test_newest_session_wins_and_subfolders_are_ignored(tmp_path):
    """The newest dated session with the robot's file; archive/ below a session never counts."""
    _session(tmp_path, "20261001", _shifted(0.002))
    newest = _session(tmp_path, "20261020", _shifted(0.003))
    _session(tmp_path, "20261030", _shifted(0.004), folder="archive")
    _session(tmp_path, "notadate", _shifted(0.004))
    assert newest_session_file(tmp_path, ALICE) == newest
    calibration, warnings = resolve(ALICE_ID, ALICE, tmp_path)
    assert calibration.session == "20261020" and warnings == []
    assert calibration.position[0] == pytest.approx(default_calibration(ALICE_ID).position[0] + 0.003)
    assert "Drive session 20261020" in calibration.describe() and "ur_arm_base_link_inertia" in calibration.describe()
    table = explain(ALICE_ID, ALICE, calibration, warnings, tmp_path)
    assert table[1].split() == ["from", "20261020/calibrated_transformation_0804_rhino.json"]
    assert table[2].split() == ["vs", "default", "3.0", "mm", "/", "0.00°"] and len(table) == 3


@pytest.mark.parametrize("content, why", [
    (_shifted(0.2), "from the default"),
    ({"base_mocap_from_base_footprint": [[0, 0], [0, 0, 0, 1]]}, "must be"),
    ({**_shifted(0.002), "mocap_id": 1850}, "for rigid body 1850"),
])
def test_insane_sessions_fall_back_to_the_default(tmp_path, content, why):
    """Far off, malformed, or for another rigid body: warned about, and the default is used."""
    _session(tmp_path, "20261020", content)
    calibration, warnings = resolve(ALICE_ID, ALICE, tmp_path)
    assert calibration == default_calibration(ALICE_ID)
    assert len(warnings) == 1 and why in warnings[0] and "using the default" in warnings[0]
    table = "\n".join(explain(ALICE_ID, ALICE, calibration, warnings, tmp_path))
    assert "crl-husky default" in table and "! refused   20261020/calibrated_transformation_0804_rhino.json" in table


def test_a_half_synced_file_falls_back(tmp_path):
    """A file that does not parse (Insync still writing it) is warned about, not fatal."""
    path = _session(tmp_path, "20261020", _shifted(0.002))
    path.write_text('{"base_mocap_from_base_footprint": [[0.1, ')
    calibration, warnings = resolve(ALICE_ID, ALICE, tmp_path)
    assert calibration == default_calibration(ALICE_ID) and "half-synced" in warnings[0]


def test_same_numbers_same_hash(tmp_path):
    """A session holding exactly the default's transforms has the default's hash, whatever else the files say."""
    default = default_calibration(ALICE_ID)
    content = {"base_mocap_from_base_footprint": [list(default.position), list(default.orientation)],
               **{key: json.loads(Path(default.source).read_text())["mocap_ids"][str(ALICE_ID)][key]
                  for key in default.measured}, "note": "anything"}
    _session(tmp_path, "20261020", content)
    calibration, _ = resolve(ALICE_ID, ALICE, tmp_path)
    assert calibration.session == "20261020" and calibration.sha256 == default.sha256
