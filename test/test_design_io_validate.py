"""Tests for design_io validate (T4): one broken design per format §9 rule, and all problems at once."""

from dataclasses import replace
from pathlib import Path

import pytest
from design_io_fixtures import build_design, joints, with_action, with_movement, with_start, write_robot_files

from husky_assembly_teleop.design_io import Attached, Design, DesignError, Pose, RobotState, Target, validate


def _problems(design: Design) -> list:
    """The problems validate reports for a design (fails if there are none)."""
    with pytest.raises(DesignError) as error:
        validate(design)
    return error.value.problems


def _reports(design: Design, rule: int, text: str) -> None:
    """Assert validate reports `text` under `rule`."""
    problems = _problems(design)
    assert any(p.startswith(f"rule {rule}:") and text in p for p in problems), problems


def test_valid_design_passes(tmp_path: Path):
    """The shared design breaks no rule."""
    validate(build_design(tmp_path))


def test_rule_2_ids(tmp_path: Path):
    """Bad characters, a missing prefix and repeated movement ids."""
    design = build_design(tmp_path)
    body = design.bodies["bars/B2"]
    _reports(replace(design, bodies={**design.bodies, "stuff/X": replace(body, id="stuff/X")}),
             2, "does not start with")
    _reports(replace(design, bodies={**design.bodies, "bars/B 9": replace(body, id="bars/B 9")}),
             2, "invalid body id")
    first = design.actions["B1_J_joint"].movements[0].id
    _reports(with_movement(design, "B2_H_hold", 0, id=first), 2, f"movement id {first!r} is used 2 times")


def test_rule_3_files_and_schedule(tmp_path: Path):
    """A missing URDF, and an action that is not scheduled."""
    design = build_design(tmp_path)
    robot = design.robots["robots/alice"]
    missing = replace(design, robots={**design.robots, "robots/alice": replace(robot, urdf=tmp_path / "no.urdf")})
    _reports(missing, 3, "no.urdf' does not exist")
    _reports(replace(design, schedule=("B1_J_joint",)), 3, "not in the schedule")
    _reports(replace(design, schedule=("B1_J_joint", "B2_H_hold", "B9")), 3, "'B9' has no action file")


def test_rule_4_references(tmp_path: Path):
    """Unknown bodies, robots, and links that are not in the URDF."""
    design = build_design(tmp_path)
    _reports(with_action(design, "B1_J_joint", bar="bars/NOPE"), 4, "unknown body 'bars/NOPE'")
    _reports(with_action(design, "B1_J_joint", robot="robots/nobody"), 4, "unknown robot 'robots/nobody'")
    attached = {"bars/B1": Attached("robots/cindy/no_link", Pose())}
    _reports(with_start(design, "B1_J_joint", 0, attached=attached), 4, "no link 'no_link'")


def test_rule_5_tool_mounts(tmp_path: Path):
    """A tool on no robot, and a mount link that is not in the URDF."""
    design = build_design(tmp_path)
    alice = design.robots["robots/alice"]
    _reports(replace(design, robots={**design.robots, "robots/alice": replace(alice, tools={})}),
             5, "tools/Grip: mounted on 0 robots")
    moved = replace(alice, tools={"no_flange": "tools/Grip"})
    _reports(replace(design, robots={**design.robots, "robots/alice": moved}), 5, "mount link 'no_flange'")


def test_rule_6_joints(tmp_path: Path):
    """A state missing a joint, and a joint name the URDF does not have."""
    design = build_design(tmp_path)
    base = design.actions["B1_J_joint"].movements[1].start.robots["robots/cindy"].base
    partial = {name: value for name, value in joints(0.0).items() if name != "right_joint2"}
    robots = {"robots/alice": None, "robots/cindy": RobotState(base, partial)}
    _reports(with_start(design, "B1_J_joint", 1, robots=robots), 6, "missing joints ['right_joint2']")
    target = Target(joints={"robots/cindy": {"elbow": 1.0}}, links={})
    _reports(with_movement(design, "B1_J_joint", 0, target=target), 6, "no joint 'elbow'")


def test_rule_7_states(tmp_path: Path):
    """A state without a robot, a pose for an absent body, a body both moved and attached."""
    design = build_design(tmp_path)
    start = design.actions["B1_J_joint"].movements[0].start
    only_cindy = {"robots/cindy": start.robots["robots/cindy"]}
    _reports(with_start(design, "B1_J_joint", 0, robots=only_cindy), 7, "robot robots/alice is not listed")
    _reports(with_start(design, "B1_J_joint", 0, poses={"bars/B2": Pose()}), 7, "bars/B2 has a pose")
    _reports(with_start(design, "B1_J_joint", 0, poses={"bars/B1": Pose()}), 7, "both in poses and in attached")


def test_rule_8_attached_to_absent_robot(tmp_path: Path):
    """A body held by a robot that is not in the state."""
    design = build_design(tmp_path)
    attached = {"bars/B1": Attached("robots/alice/left_tool0", Pose())}
    _reports(with_start(design, "B1_J_joint", 0, attached=attached), 8, "attached to robots/alice")


def test_rule_9_unit_quaternions(tmp_path: Path):
    """A quaternion that is not of unit length."""
    design = build_design(tmp_path)
    body = design.bodies["bars/B2"]
    bad = replace(body, pose=Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.1)))
    _reports(replace(design, bodies={**design.bodies, "bars/B2": bad}), 9, "bars/B2 pose")


def test_rule_10_arms(tmp_path: Path):
    """An arm link that ends no SRDF group, and an arm of another robot."""
    design = build_design(tmp_path)
    _reports(with_movement(design, "B1_J_joint", 0, arms=("robots/cindy/left_link2",)), 10,
             "no SRDF group of robots/cindy ends at 'left_link2'")
    _reports(with_movement(design, "B1_J_joint", 0, arms=("robots/alice/left_tool0",)), 10,
             "not a link of the acting robot robots/cindy")


def test_rule_11_robot_meshes(tmp_path: Path):
    """A URDF mesh given by absolute path, and one that does not exist."""
    design = build_design(tmp_path)
    for mesh, text in (("/abs/base.obj", "is not a path relative"), ("meshes/gone.obj", "does not exist")):
        urdf, srdf = write_robot_files(tmp_path / mesh.replace("/", "_"), mesh=mesh, write_mesh_file=False)
        robot = replace(design.robots["robots/alice"], urdf=urdf, srdf=srdf)
        _reports(replace(design, robots={**design.robots, "robots/alice": robot}), 11, text)


def test_all_problems_reported_at_once(tmp_path: Path):
    """Several broken rules give one error listing every one of them."""
    design = build_design(tmp_path)
    design = with_action(design, "B1_J_joint", bar="bars/NOPE")
    design = with_start(design, "B2_H_hold", 0, poses={"bars/B9": Pose()})
    body = design.bodies["bars/B2"]
    design = replace(design, bodies={**design.bodies, "bars/B2": replace(body, pose=Pose(orientation=(0, 0, 0, 2)))})
    problems = _problems(design)
    assert {p.split(":")[0] for p in problems} >= {"rule 4", "rule 7", "rule 9"}
    assert any("bars/NOPE" in p for p in problems) and any("bars/B9" in p for p in problems)
