"""Tests for planner results in `solutions/`: write and read, staleness and broken files (A14), the chaining rule."""

import json
from dataclasses import replace
from pathlib import Path

import pytest
from design_fixtures import build_design, joints, with_movement

from bar_assembly_core.design import Design, DesignError, Target, write
from bar_assembly_core.design.solutions import (MovementResult, Planner, Solution, Trajectory, is_stale,
                                                read_solutions, solution_warnings, solved_against, write_solution)
from bar_assembly_core.geometry import Pose

PLANNER = Planner("tamp", repo="husky_assembly_tamp", commit="abc", ik_backend="ssik", artifacts="cindy_v2",
                  settings={"seed": 3, "timeout_s": 10.0})


@pytest.fixture
def design(tmp_path: Path) -> Design:
    """The fixture design, written: a solution refers to files."""
    return write(build_design(tmp_path), tmp_path / "out")


def _solution(design: Design) -> Solution:
    """A solution of B2_J_joint: two solved movements that chain, one keyframe, one not planned."""
    names = tuple(joints(0.0))
    return Solution("B2_J_joint", solved_against(design, "B2_J_joint"), PLANNER, {
        "B2_M0_mount": MovementResult("not_planned"),
        "B2_M1_grasp": MovementResult("keyframe_only", bases={"robots/cindy": Pose((1.0, 0.0, 0.0))},
                                      start={"robots/cindy": joints(0.0)}, end={"robots/cindy": joints(0.0)}),
        "B2_M2_transfer": MovementResult(
            "solved", start={"robots/cindy": joints(0.0)}, end={"robots/cindy": joints(0.3)}, start_overridden=True,
            trajectory=Trajectory("robots/cindy", names, ((0.0,) * 4, (0.3,) * 4), (0.0, 1.5)),
            path_poses={"robots/cindy/left_tool0": (Pose(), Pose((0.0, 0.0, 0.1)))}),
        "B2_M3_insert": MovementResult("failed", reason="no IK at the target"),
    })


def test_write_then_read(design):
    """A solution written and read back is equal; it is not stale; empty fields are left out of the file."""
    solution = _solution(design)
    path = write_solution(design, solution)
    assert path == design.folder / "solutions" / "B2_J_joint.json"
    assert read_solutions(design) == {"B2_J_joint": solution}
    assert not is_stale(design, solution) and solution_warnings(design, read_solutions(design)) == []
    raw = json.loads(path.read_text())
    assert raw["movements"]["B2_M0_mount"] == {"status": "not_planned"}
    assert raw["format"] == "husky_design/solution"
    assert raw["solved_against"]["design"] == solution.solved_against.design


def test_a14_stale_after_the_action_changes(design, tmp_path):
    """Changing the action file makes its solution stale: a warning, never a read error; rewriting it unchanged not."""
    write_solution(design, _solution(design))
    rewritten = write(design, design.folder, overwrite=True)
    assert not is_stale(rewritten, read_solutions(rewritten)["B2_J_joint"]), "same content, same hash"
    changed = with_movement(rewritten, "B2_J_joint", 2, target=Target(joints={"robots/cindy": joints(0.31)}))
    changed = write(changed, design.folder, overwrite=True)
    solutions = read_solutions(changed)
    assert is_stale(changed, solutions["B2_J_joint"])
    assert any(line.startswith("A14: solutions/B2_J_joint.json is stale") for line in
               solution_warnings(changed, solutions))


def test_a14_broken_solutions_are_refused(design):
    """An unknown movement, a failure without a reason, and a file naming another action are refused."""
    solution = _solution(design)
    bad = replace(solution, movements={**solution.movements, "B9_M0": MovementResult("solved"),
                                       "B2_M3_insert": MovementResult("failed")})
    with pytest.raises(DesignError) as error:
        write_solution(design, bad)
    problems = "\n".join(error.value.problems)
    assert "A14:" in problems and "not a movement of B2_J_joint" in problems and "needs a reason" in problems
    path = write_solution(design, solution)
    path.rename(path.with_name("B1_J_joint.json"))
    with pytest.raises(DesignError, match="names action 'B2_J_joint', not its file name"):
        read_solutions(design)


def test_a14_needs_solved_against(design):
    """A solution file without `solved_against` is refused (A2)."""
    path = write_solution(design, _solution(design))
    raw = json.loads(path.read_text())
    del raw["solved_against"]
    path.write_text(json.dumps(raw))
    with pytest.raises(DesignError, match="missing \\['solved_against'\\]"):
        read_solutions(design)


def test_chaining_breaks_are_reported(design):
    """Each solved movement starts where the one before it ended, robot by robot."""
    solution = _solution(design)
    jumped = replace(solution.movements["B2_M2_transfer"], start={"robots/cindy": joints(0.1)})
    write_solution(design, replace(solution, movements={**solution.movements, "B2_M2_transfer": jumped}))
    warnings = solution_warnings(design, read_solutions(design))
    assert warnings == ["solutions: robots/cindy ends B2_M1_grasp and starts B2_M2_transfer 0.1 apart"]
