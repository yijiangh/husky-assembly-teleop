"""Tests for the file checks A1–A13 (A14 is in test_design_solutions.py): one broken design per check."""

import json
from dataclasses import replace
from pathlib import Path

import pytest
from design_fixtures import build_design, joints, with_action, with_movement, with_start, write_robot_files

from bar_assembly_core.design import (Design, DesignError, Holder, LineSpec, RobotState, Target, ToolState, read,
                                      validate, write)
from bar_assembly_core.geometry import Pose


def _problems(design: Design) -> list:
    """The problems validate reports for a design (fails if there are none)."""
    with pytest.raises(DesignError) as error:
        validate(design)
    return error.value.problems


def _reports(design: Design, check: int, text: str) -> None:
    """Assert validate reports `text` under check A<check>."""
    problems = _problems(design)
    assert any(p.startswith(f"A{check}:") and text in p for p in problems), problems


def _read_reports(folder: Path, check: int, text: str) -> None:
    """Assert reading a design folder reports `text` under check A<check>."""
    with pytest.raises(DesignError) as error:
        read(folder)
    assert any(p.startswith(f"A{check}:") and text in p for p in error.value.problems), error.value.problems


def _edit(path: Path, change) -> None:
    """Change one JSON file in place: `change(data)` edits the parsed file."""
    data = json.loads(path.read_text())
    change(data)
    path.write_text(json.dumps(data))


@pytest.fixture
def written(tmp_path: Path) -> Path:
    """The fixture design written to `tmp_path/out`."""
    write(build_design(tmp_path), tmp_path / "out")
    return tmp_path / "out"


def test_valid_design_passes(tmp_path: Path):
    """The shared design breaks no check."""
    validate(build_design(tmp_path))


def test_a1_json_format_and_schema(written: Path):
    """A file with NaN, and a file of another kind, are refused."""
    path = written / "actions" / "B1_J_joint.json"
    path.write_text(path.read_text().replace('"distance": 0.015', '"distance": NaN'))
    _read_reports(written, 1, "not UTF-8 JSON")
    _edit(written / "design.json", lambda data: data.update(format="other"))
    _read_reports(written, 1, "format is 'other'")


def test_a2_keys_and_types(written: Path):
    """Missing required keys, unknown keys outside notes, and wrong types are all reported at once."""
    def change(data):
        movement = data["movements"][0]
        movement["speed"] = 3
        del movement["start"]["tools"]
        movement["start"]["present"] = "ground/WG0"
        data["notes"] = {"free": "text", "anything": 1}
    _edit(written / "actions" / "B1_J_joint.json", change)
    with pytest.raises(DesignError) as error:
        read(written)
    problems = "\n".join(error.value.problems)
    assert "unknown keys ['speed']" in problems and "missing ['tools']" in problems
    assert "present: expected a list" in problems and "notes" not in problems


def test_a3_ids_and_labels(tmp_path: Path):
    """Bad characters, an unknown prefix, a repeated movement id and a label with '/'."""
    design = build_design(tmp_path)
    body = design.bodies["bars/B2"]
    _reports(replace(design, bodies={**design.bodies, "stuff/X": replace(body, id="stuff/X")}),
             3, "does not start with")
    _reports(replace(design, bodies={**design.bodies, "bars/B 9": replace(body, id="bars/B 9")}), 3, "invalid body id")
    first = design.actions["B1_J_joint"].movements[0].id
    _reports(with_movement(design, "B1_H_hold", 0, id=first), 3, f"movement id {first!r} is used 2 times")
    _reports(with_movement(design, "B1_H_hold", 0, label="a/b"), 3, "label 'a/b' contains '/'")


def test_a4_references_and_files(tmp_path: Path):
    """Unknown bodies, robots, tools and links; a missing URDF; a URDF mesh by absolute path or missing."""
    design = build_design(tmp_path)
    _reports(with_action(design, "B1_J_joint", bar="bars/NOPE"), 4, "unknown body 'bars/NOPE'")
    _reports(with_action(design, "B1_J_joint", robot="robots/nobody"), 4, "unknown robot 'robots/nobody'")
    attached = {"bars/B1": (Holder("robots/cindy/no_link", Pose()),)}
    _reports(with_movement(design, "B1_J_joint", 0, target=Target(attached=attached)), 4, "no link 'no_link'")
    _reports(with_movement(design, "B1_J_joint", 2, drives={"tools/Nope": "tighten"}), 4, "unknown tool")
    robot = design.robots["robots/alice"]
    missing = replace(design, robots={**design.robots, "robots/alice": replace(robot, urdf=tmp_path / "no.urdf")})
    _reports(missing, 4, "no.urdf' does not exist")
    for mesh, text in (("/abs/base.obj", "is not a path relative"), ("meshes/gone.obj", "does not exist")):
        urdf, srdf = write_robot_files(tmp_path / mesh.replace("/", "_"), mesh=mesh, write_mesh_file=False)
        moved = replace(robot, urdf=urdf, srdf=srdf)
        _reports(replace(design, robots={**design.robots, "robots/alice": moved}), 4, text)


def test_a5_schedule_and_action_files(tmp_path: Path, written: Path):
    """An action not scheduled, a scheduled action without a file, and a file whose id is not its name."""
    design = build_design(tmp_path)
    _reports(replace(design, schedule=design.schedule[:-1]), 5, "not in the schedule")
    _reports(replace(design, schedule=(*design.schedule, "B9")), 5, "'B9' has no action file")
    _edit(written / "actions" / "B1_H_hold.json", lambda data: data.update(id="B1_H_other"))
    _read_reports(written, 5, "has id 'B1_H_other', not its file name")


def test_a6_tool_mounts(tmp_path: Path):
    """A tool on no robot, and a mount link the URDF lacks."""
    design = build_design(tmp_path)
    alice = design.robots["robots/alice"]
    _reports(replace(design, robots={**design.robots, "robots/alice": replace(alice, tools={})}),
             6, "tools/Grip: mounted on 0 robots")
    moved = replace(alice, tools={"no_flange": "tools/Grip"})
    _reports(replace(design, robots={**design.robots, "robots/alice": moved}), 6, "mount link 'no_flange'")


def test_a7_poses(tmp_path: Path, written: Path):
    """A quaternion that is not of unit length, and a pose that is not 7 numbers."""
    design = build_design(tmp_path)
    bad = replace(design.bodies["bars/B2"], pose=Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.1)))
    _reports(replace(design, bodies={**design.bodies, "bars/B2": bad}), 7, "bars/B2 pose")
    _edit(written / "design.json", lambda data: data["bodies"]["bars/B2"].update(pose=[0, 0, 0, 1]))
    _read_reports(written, 7, "a pose has 7 numbers, not 4")


def test_a8_robots_and_joints(tmp_path: Path):
    """A state missing a robot or a joint, and a joint name the URDF lacks."""
    design = build_design(tmp_path)
    start = design.actions["B1_J_joint"].movements[1].start
    _reports(with_start(design, "B1_J_joint", 1, robots={"robots/cindy": start.robots["robots/cindy"]}),
             8, "robot robots/alice is not listed")
    partial = {name: value for name, value in joints(0.0).items() if name != "right_joint2"}
    robots = {**start.robots, "robots/cindy": RobotState(start.robots["robots/cindy"].base, partial)}
    _reports(with_start(design, "B1_J_joint", 1, robots=robots), 8, "missing joints ['right_joint2']")
    _reports(with_movement(design, "B1_J_joint", 2, target=Target(joints={"robots/cindy": {"elbow": 1.0}})),
             8, "no joint 'elbow'")


def test_a9_present_attached_built_poses(tmp_path: Path):
    """`present` lists bars and ground only; attached, built and poses name present bars; no pose for a built bar."""
    design = build_design(tmp_path)
    start = design.actions["B1_H_hold"].movements[0].start
    _reports(with_start(design, "B1_H_hold", 0, present=start.present | {"joints/J1_male"}), 9,
             "'joints/J1_male' is not one of bars, ground")
    _reports(with_start(design, "B1_H_hold", 0, present=start.present - {"bars/B1"}), 9, "bars/B1 is not present")
    _reports(with_start(design, "B1_H_hold", 0, poses={**start.poses, "bars/B1": Pose()}), 9,
             "bars/B1 is built or attached, so its pose is not written")
    _reports(with_start(design, "B1_H_hold", 0, attached={"bars/B1": ()}), 9, "bars/B1 has no holder")


def test_a10_tools(tmp_path: Path):
    """Every mounted tool listed, grips and drives from the vocabulary, `on` an existing body, a known kind."""
    design = build_design(tmp_path)
    start = design.actions["B1_J_joint"].movements[1].start
    _reports(with_start(design, "B1_J_joint", 1, tools={"tools/Grip": start.tools["tools/Grip"]}), 10,
             "tool tools/AT3L is not listed")
    _reports(with_start(design, "B1_J_joint", 1, tools={**start.tools, "tools/AT3L": ToolState("half", None)}), 10,
             "grip must be one of ('open', 'closed')")
    _reports(with_start(design, "B1_J_joint", 1, tools={**start.tools, "tools/AT3L": ToolState("open", "bars/B9")}),
             10, "on 'bars/B9', which is not a body")
    _reports(with_movement(design, "B1_J_joint", 2, drives={"tools/AT3L": "spin"}), 10, "not 'spin'")
    _reports(with_movement(design, "B1_J_joint", 2, drives={"tools/Grip": "tighten"}), 10, "drives nothing")
    tool = replace(design.tools["tools/Grip"], kind="laser")
    _reports(replace(design, tools={**design.tools, "tools/Grip": tool}), 10, "unknown tool kind 'laser'")


def test_a11_movement_parts(tmp_path: Path):
    """Path and controller exactly when arms move; ends_on from its values; one line per arm on a linear path."""
    design = build_design(tmp_path)
    _reports(with_movement(design, "B1_J_joint", 0, arms=("robots/cindy/left_tool0",)), 11,
             "arms move, so path must be")
    _reports(with_movement(design, "B1_J_joint", 1, controller="position"), 11, "no arm moves")
    _reports(with_movement(design, "B1_J_joint", 1, ends_on="stall"), 11, "ends_on must be one of")
    _reports(with_movement(design, "B1_J_joint", 2, line={}), 11, "a linear path needs one line per moving arm")
    _reports(with_movement(design, "B1_H_hold", 0, line={"robots/alice/left_tool0": LineSpec((0.0, 0.0, 1.0), 0.1)}),
             11, "a line needs path linear")
    line = {"robots/cindy/left_tool0": LineSpec((0.0, 0.0, -2.0), 0.015)}
    _reports(with_movement(design, "B1_J_joint", 2, line=line), 11, "is not a unit vector")


def test_a12_mounts_and_mates(tmp_path: Path):
    """Every half is mounted on a bar, only halves are; mates pair halves or a half and ground, once per half."""
    design = build_design(tmp_path)
    half = design.bodies["joints/J1_male"]
    _reports(replace(design, bodies={**design.bodies, "joints/J1_male": replace(half, mount=None)}), 12,
             "a connector half needs a mount")
    _reports(replace(design, bodies={**design.bodies, "joints/J1_male": replace(half, mount="ground/WG0")}), 12,
             "is not a bar of the design")
    bar = design.bodies["bars/B2"]
    _reports(replace(design, bodies={**design.bodies, "bars/B2": replace(bar, mount="bars/B1")}), 12,
             "only connector halves")
    _reports(replace(design, mates=design.mates | {("bars/B1", "joints/J1_male")}), 12, "pairs neither")
    _reports(replace(design, mates=design.mates | {("joints/G1_ground", "joints/J1_male")}), 12,
             "joints/J1_male is in 2 mates")


def test_a13_notes(tmp_path: Path):
    """Notes are flat: strings, numbers and booleans."""
    design = build_design(tmp_path)
    _reports(with_movement(design, "B1_J_joint", 2, notes={"axes": {"left": [0, 1, 0]}}), 13, "flat values only")
    _reports(with_action(design, "B1_H_hold", notes={"axes": [0, 1, 0]}), 13, "flat values only")


def test_all_problems_reported_at_once(tmp_path: Path):
    """Several broken checks give one error listing every one of them."""
    design = build_design(tmp_path)
    design = with_action(design, "B1_J_joint", bar="bars/NOPE")
    design = with_start(design, "B1_H_hold", 0, poses={"bars/B9": Pose()})
    body = design.bodies["bars/B2"]
    design = replace(design, bodies={**design.bodies, "bars/B2": replace(body, pose=Pose(orientation=(0, 0, 0, 2)))})
    problems = _problems(design)
    assert {p.split(":")[0] for p in problems} >= {"A4", "A7"}
    assert any("bars/NOPE" in p for p in problems) and any("bars/B9" in p for p in problems)


def test_write_refuses_an_invalid_design(tmp_path: Path):
    """The checks run on write too, before anything is written."""
    with pytest.raises(DesignError, match="A4"):
        write(with_action(build_design(tmp_path), "B1_J_joint", bar="bars/NOPE"), tmp_path / "out")
    assert not (tmp_path / "out").exists()
