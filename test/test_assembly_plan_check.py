"""Tests for experiments.assembly_plan_check without planning: offsets, the movements picked, the homes, the CLI."""

from __future__ import annotations

import math

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, movement, purged_process, robot_config
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.experiments import assembly_plan_check
from husky_assembly_teleop.experiments.assembly_plan_check import (DATA, OLD_HOME, BaseOffsets, arm_movements,
                                                                   home_check, main, parser, perturbed, run_check,
                                                                   select_actions)
from husky_assembly_teleop.shared.planning.movement import Routine, classify_action
from husky_assembly_teleop.shared.planning.route_result import RouteResult
from husky_assembly_teleop.world.kinematics import stow_joints
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.process.solutions import MovementResult

BASE = Pose((0.947, -3.171, -0.016), tuple(Rotation.from_euler("z", 0.6).as_quat()))


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process()


def _yaw(pose: Pose) -> float:
    """A pose's turn about z, radians."""
    return Rotation.from_quat(pose.orientation).as_euler("zyx")[0]


def test_perturbed_moves_on_the_floor_and_turns_about_z():
    """Exactly (dx, dy) on the floor, the yaw turned by dyaw, the height and tilt unchanged."""
    moved = perturbed(BASE, (0.013, -0.007, math.radians(1.5)))
    assert np.allclose(np.subtract(moved.position, BASE.position), (0.013, -0.007, 0.0), atol=1e-15)
    assert _yaw(moved) - _yaw(BASE) == pytest.approx(math.radians(1.5), abs=1e-12)
    assert np.allclose(Rotation.from_quat(moved.orientation).apply((0, 0, 1)), (0, 0, 1), atol=1e-12)


def test_offsets_per_base_within_bounds_and_seeded():
    """One offset per robot and base, within ±m and ±deg; the same seed draws the same; none without perturb."""
    def draw(seed: int) -> list:
        offsets = BaseOffsets((0.02, 2.0), seed)
        bases = [Pose((x, 1.0, 0.0)) for x in (0.0, 0.5, 1.0)]
        return [offsets(robot, base) for robot in ("robots/cindy", "robots/alice") for base in bases]
    first = draw(0)
    assert first == draw(0) and first != draw(1)
    assert all(abs(dx) <= 0.02 and abs(dy) <= 0.02 and abs(dyaw) <= math.radians(2.0) for dx, dy, dyaw in first)
    assert len(set(first)) == 6
    offsets = BaseOffsets((0.02, 2.0), 0)
    assert offsets("robots/cindy", BASE) == offsets("robots/cindy", Pose((0.9472, -3.1708, -0.016), BASE.orientation))
    assert BaseOffsets(None, 0)("robots/cindy", BASE) == (0.0, 0.0, 0.0)


def test_an_offset_does_not_depend_on_the_order_drawn():
    """A stand's offset is the same drawn first, or after others (as `--only` or `--actions` would)."""
    alone = BaseOffsets((0.02, 2.0), 3)("robots/cindy", BASE)
    later = BaseOffsets((0.02, 2.0), 3)
    for x in (0.0, 0.5, 1.0):
        later("robots/alice", Pose((x, 1.0, 0.0)))
    assert later("robots/cindy", BASE) == alone


def test_the_first_18_actions_and_their_arm_movements(purged):
    """18 actions, B20_J to B35_HR; with the workaround 7 travels to load skipped and 36 arm movements planned."""
    actions = select_actions(purged, "18")
    assert len(actions) == 18 and (actions[0], actions[-1]) == ("B20_J", "B35_HR")
    assert select_actions(purged, "B20_R,B20_J") == ["B20_J", "B20_R"]
    with pytest.raises(ValueError, match="B99_J"):
        select_actions(purged, "B20_J,B99_J")
    picked = arm_movements(purged.process, actions, transfer_as_free=True)
    skipped = [movement.id for _action, movement, skip in picked if skip]
    assert len(skipped) == 7 and all(name.endswith("_J_load") for name in skipped)
    assert sum(not skip for _action, _movement, skip in picked) == 36
    assert [movement.id for _action, movement, _skip in picked[:3]] == ["B20_J_load", "B20_J_transfer", "B20_J_insert"]
    unskipped = arm_movements(purged.process, actions, transfer_as_free=False)
    assert len(unskipped) == 43 and not any(skip for _action, _movement, skip in unskipped)


def test_every_way_home_is_the_old_home(purged):
    """The 7 ways home of the 18 actions go to the old monitor's fixed home, to the 12 decimals written."""
    specs = [spec for action_id in select_actions(purged, "18")
             for spec in classify_action(purged.process, purged.process.actions[action_id])]
    homes = home_check(specs)
    assert len(homes) == 7 and max(homes.values()) <= 1e-12
    assert len(OLD_HOME) == 12


def test_command_line_defaults():
    """By default: the first 18 actions, the workaround on, no offsets, seed 0, the repo's data folder."""
    options = parser().parse_args(["/some/process"])
    assert (options.actions, options.workaround, options.perturb_base, options.seed, options.only) == \
        ("18", "on", None, 0, None)
    assert options.data == DATA and DATA.is_dir()
    assert options.write_solutions is None and not options.verbose


def test_bad_input_exits_2(purged, capsys):
    """An unknown action, movement, or a malformed offset is bad input: exit code 2, nothing planned."""
    folder = str(purged.source)
    assert main([folder, "--actions", "B99_J"]) == 2
    assert main([folder, "--actions", "B20_J", "--only", "B21_J_insert"]) == 2
    assert main([folder, "--perturb-base", "0.02"]) == 2
    # * Checked before planning: a folder inside the process, or an existing copy, plans nothing.
    assert main([folder, "--write-solutions", str(purged.source / "out")]) == 2
    assert main([folder, "--write-solutions", str(purged.source.parent)]) == 2
    assert "bad input" in capsys.readouterr().err


def test_arms_carry_across_base_drives(purged, monkeypatch):
    """A robot's arms start where its last movement left them, after a base drive too; else authored, else stowed.

    A stand-in planner records each start and ends every movement at its count; B22's transfer fails.
    """
    starts = {}

    def plan(mirror: str, _loaded, step, snapshot, _robots, **_options) -> RouteResult:
        joints = snapshot.robots[mirror].joints
        starts[step.movement.id] = dict(joints)
        if step.movement.id == "B22_J_transfer":
            return RouteResult(MovementResult("failed", "stand-in"), Routine.TRANSFER_FREE)
        end = {name: float(len(starts)) for name in joints}
        return RouteResult(MovementResult("solved", end={step.action.robot: end}), Routine.TRANSFER_FREE)

    monkeypatch.setattr(assembly_plan_check, "plan_movement", plan)
    configs = tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY))
    mirrors = {robot_id(config.serial): robot_id(config.serial) for config in configs}
    rows = run_check(purged, configs, select_actions(purged, "B20_J,B20_R,B21_J,B21_H,B21_R,B22_J"), mirrors=mirrors,
                     log=lambda _line: None)
    ends = {row.movement: index + 1 for index, row in enumerate(row for row in rows if row.route is not None)}
    cindy = stow_joints(robot_config(CINDY), robot_config(CINDY).model.movable_joints)
    assert starts["B20_J_transfer"] == cindy
    # * Cindy's base drives from B20's stand to B21's: her arms stay where B20's way home left them.
    assert set(starts["B21_J_transfer"].values()) == {float(ends["B20_R_home"])}
    alice = robot_config(ALICE)
    assert starts["B21_H_approach"] == stow_joints(alice, alice.model.movable_joints)
    authored = movement(purged, "B22_J_insert")[1].start.robots["robots/cindy"].joints
    assert starts["B22_J_insert"] == {**cindy, **authored}
