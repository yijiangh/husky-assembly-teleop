"""Tests for shared.assembly_planning.carry_plan: a saved carry read, and planned again by a stand-in for tamp."""

from __future__ import annotations

import json
import math
import time

import pytest
from planning_fixtures import purged_process

from husky_assembly_teleop.shared.assembly_planning.carry_plan import (
    ANY_CLASS, CARRY_FILE, CARRY_FORMAT, ROADMAP_FILE, CarryReplan, read_carry)
from husky_assembly_teleop.shared.assembly_planning.route_search import Refused
from scaffolding_assembly_core.process.solutions import solved_against

NAMES = [f"joint_{index}" for index in range(12)]
ROWS = [[0.1 * index] * 12 for index in range(3)]


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process()


def write_carry(purged, root, action_id: str = "B20_J", **changes) -> None:
    """A carry file for an action's bar under `root`, as tamp writes it; `changes` replace its entries."""
    against = solved_against(purged.process, action_id)
    carry = {"format": CARRY_FORMAT, "action": action_id, "joint_names": NAMES, "path": ROWS,
             "station": [3.0, 0.5, 270.0], "base": [1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0],
             "solved_against": {"process": against.process, "action": against.action}, **changes}
    folder = root / "prm" / f"{purged.process.folder.name}__{action_id.split('_')[0]}"
    folder.mkdir(parents=True, exist_ok=True)
    (folder / CARRY_FILE).write_text(json.dumps(carry))


def test_a_saved_carry_gives_the_loading_posture_the_end_and_the_station(purged, tmp_path):
    """Read back: the first row is the loading posture, the last the end, the station in metres and radians."""
    write_carry(purged, tmp_path)
    carry = read_carry(str(tmp_path), purged.process, "B20_J")
    assert carry.start == dict(zip(NAMES, ROWS[0])) and carry.end == dict(zip(NAMES, ROWS[-1]))
    assert carry.station == pytest.approx((3.0, 0.5, math.radians(270.0))) and len(carry.rows) == 3
    assert carry.base == (1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    write_carry(purged, tmp_path, station=None)
    assert read_carry(str(tmp_path), purged.process, "B20_J").station is None


def test_a_missing_carry_is_refused_with_the_command_that_plans_it(purged, tmp_path):
    """No file: the refusal names the file and the planning command for that bar; no root: says so."""
    command = "no saved carry for B20 at .*carry.json: plan it with .*--bar B20 .*--save-carry"
    with pytest.raises(Refused, match=command):
        read_carry(str(tmp_path), purged.process, "B20_J")
    with pytest.raises(Refused, match="no roadmap root"):
        read_carry("", purged.process, "B20_J")


@pytest.mark.parametrize("changes, why", [
    ({"format": "other"}, "is not the saved carry of B20_J"),
    ({"action": "B21_J"}, "is not the saved carry of B20_J"),
    ({"path": []}, "is not the saved carry of B20_J"),
    ({"base": [1.0, 2.0]}, "is not the saved carry of B20_J"),
    ({"solved_against": {"process": "older", "action": "older"}}, "another export of the process"),
])
def test_a_carry_of_another_action_or_an_older_export_is_refused(purged, tmp_path, changes, why):
    """Another format or action, an empty path, or a process changed since: refused, never half used."""
    write_carry(purged, tmp_path, **changes)
    with pytest.raises(Refused, match=why):
        read_carry(str(tmp_path), purged.process, "B20_J")


#: Stands in for tamp's script: with class "back" it keeps the roadmap's carry (copied from `CARRY_FROM`) as the new one
#: with that class; with "vertical" it finds none, as tamp says it; with "horizontal" it runs until stopped.
SCRIPT = """
import json, pathlib, sys, time
args = sys.argv[1:]
root, bar, wanted = (args[args.index(name) + 1] for name in ("--prm-root", "--bar", "--cdfm-home-anchor"))
folder = pathlib.Path(root) / "prm" / f"{pathlib.Path(args[0]).name}__{bar}"
assert (folder / "roadmap.json.gz").is_file() and not (folder / "carry.json").exists()
print("[plan] roadmap backend:", folder, flush=True)
if wanted == "horizontal":
    time.sleep(60)
if wanted == "vertical":
    print("[plan] M1 FAILED: prm_no_start_of_class (after 1.0 s)")
    sys.exit(2)
carry = json.loads(pathlib.Path("CARRY_FROM").read_text())
carry["loading_class"] = wanted
(folder / "carry.json").write_text(json.dumps(carry))
print("[save] the roadmap's carry ->", folder / "carry.json")
"""
STATION = (-1.3, -1.9, math.pi)


def replan(purged, tmp_path, loading_class: str) -> CarryReplan:
    """B20's carry planned again by the stand-in script, on a roadmap root holding its roadmap and a saved carry."""
    root = tmp_path / "root"
    write_carry(purged, root, loading_class="horizontal")
    folder = root / "prm" / f"{purged.process.folder.name}__B20"
    (folder / ROADMAP_FILE).write_bytes(b"roadmap")
    script = tmp_path / "scripts" / "plan.py"
    script.parent.mkdir(exist_ok=True)
    script.write_text(SCRIPT.replace("CARRY_FROM", str(folder / CARRY_FILE)))
    return CarryReplan(str(root), purged.process, "B20_J", STATION, loading_class, workers=2,
                       work_root=tmp_path / "work", script=script)


def ended(job: CarryReplan) -> int:
    """The job's exit code, waited for."""
    until = time.monotonic() + 30.0
    while job.poll() is None and time.monotonic() < until:
        time.sleep(0.02)
    return job.poll()


@pytest.mark.slow
def test_a_carry_planned_again_replaces_the_saved_one_only_once_found(purged, tmp_path):
    """The script works on a copy of the roadmap; its carry is kept beside the roadmap by `adopt`, a failure is not."""
    job = replan(purged, tmp_path, "back")
    assert "--station -1.3000 -1.9000 180.00" in " ".join(job.command) and "--save-carry" in job.command
    assert job.poll() is None and not job.running, "not started yet"
    job.start()
    assert ended(job) == 0
    assert read_carry(str(tmp_path / "root"), purged.process, "B20_J").loading_class == "horizontal", "not yet"
    assert job.adopt().loading_class == "back"
    assert read_carry(str(tmp_path / "root"), purged.process, "B20_J").loading_class == "back"
    assert "[save] the roadmap's carry" in job.log_file.read_text() and job.last.startswith("[save]")

    failed = replan(purged, tmp_path, "vertical")
    failed.start()
    assert ended(failed) == 2
    with pytest.raises(Refused, match=r"no loading pose of this class .*\(prm_no_start_of_class\); log: "):
        failed.adopt()
    assert read_carry(str(tmp_path / "root"), purged.process, "B20_J").loading_class == "horizontal"


@pytest.mark.slow
def test_a_carry_being_planned_can_be_stopped_and_keeps_nothing(purged, tmp_path):
    """Stop ends the script; nothing is adopted. A bar without a roadmap, or an unknown class, is refused at once."""
    job = replan(purged, tmp_path, "horizontal")
    job.start()
    assert job.running
    job.stop()
    assert ended(job) is not None and not job.running
    with pytest.raises(Refused, match="stopped"):
        job.adopt()
    with pytest.raises(Refused, match="no roadmap of B21 at "):
        CarryReplan(str(tmp_path / "root"), purged.process, "B21_J", STATION, ANY_CLASS, script=tmp_path)
    with pytest.raises(Refused, match="no loading pose class 'sideways'"):
        CarryReplan(str(tmp_path / "root"), purged.process, "B20_J", STATION, "sideways", script=tmp_path)
