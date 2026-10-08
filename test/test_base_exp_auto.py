"""Automated base experiments: the grid, its balancing, and the clearance check against walls and robots."""

import asyncio
import math
import random
from pathlib import Path

import numpy as np
import pytest

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.design_io.geometry import box_geometry
from husky_assembly_teleop.design_io.pose import Pose
from husky_assembly_teleop.plugins.base_exp.auto import (GRID, STANDARD_SET, Clearance, Sampler, StandardSet, cell_key,
                                                         cells, check_poses, standard_set)
from husky_assembly_teleop.plugins.base_exp.plan import plan_from
from husky_assembly_teleop.world.scene import Body, RobotEntry, SceneSnapshot

DATA = Path(__file__).resolve().parent.parent / "data"


def test_every_grid_cell_makes_a_plan():
    """Every cell of every template is a valid plan starting at the robot."""
    for name in GRID:
        for settings in cells(name):
            plan, problem = plan_from("a200-0804", settings, (1.0, 2.0, 0.5))
            assert plan is not None, (settings, problem)
            assert plan.poses[0] == pytest.approx([1.0, 2.0, 0.5]) and plan.expected > 0


def test_sampler_prefers_the_template_with_fewest_successes():
    """With Straight done twice and the rest once, Straight comes last; the first cells are uncovered ones."""
    done = [cells("Straight")[0], cells("Straight")[1]] + [cells(name)[0] for name in GRID if name != "Straight"]
    sampler = Sampler(done, random.Random(0))
    order = [settings["name"] for settings in sampler.candidates(per_template=1)]
    assert order[-1] == "Straight" and sorted(order) == sorted(GRID)
    first = next(sampler.candidates(per_template=1))
    assert cell_key(first) not in {cell_key(settings) for settings in done}


def test_sampler_balances_over_many_runs():
    """Taking the first candidate and counting it as done spreads runs evenly over templates."""
    sampler = Sampler([], random.Random(1))
    for _ in range(50):
        sampler.add(next(sampler.candidates(per_template=1)))
    assert set(sampler.counts.values()) == {10}


def test_check_poses_are_spaced_but_keep_both_ends():
    """A 1 m straight is checked every 5 cm, a quarter turn every 5 degrees, ends included."""
    line = np.column_stack([np.linspace(0, 1, 101), np.zeros(101), np.zeros(101)])
    picked = check_poses(line)
    assert len(picked) == 21 and picked[0] == pytest.approx(line[0]) and picked[-1] == pytest.approx(line[-1])
    turn = np.column_stack([np.zeros(91), np.zeros(91), np.radians(np.arange(91))])
    assert len(check_poses(turn)) == 19


@pytest.mark.slow
def test_clearance_sees_walls_and_robots():
    """A path into a wall or past another robot is not clear; one into open space is."""
    alice, cindy = (robot_config_from_serial(serial, DATA) for serial in ("0804", "0806"))
    wall = Body("obstacles/boxes/wall", box_geometry((0.2, 4.0, 1.0)), Pose((3.0, 0.0, 0.5)), label="wall")
    bases = {alice.serial: Pose(), cindy.serial: Pose((0.0, -3.0, 0.0))}
    robots = {config.serial: RobotEntry(config=config, base=bases[config.serial], base_tracked=True, joints={},
                                        unmeasured=frozenset(), base_time=None, joints_time=None)
              for config in (alice, cindy)}
    snapshot = SceneSnapshot(bodies={wall.id: wall}, world_poses={wall.id: wall.placement}, robots=robots)

    def straight(x0, y0, yaw, length):
        s = np.linspace(0.0, length, 41)
        return np.column_stack([x0 + s * math.cos(yaw), y0 + s * math.sin(yaw), np.full_like(s, yaw)])

    async def check():
        clearance = Clearance()
        try:
            into_wall = await clearance.path_hit(snapshot, alice.serial, straight(0, 0, 0.0, 2.5))
            toward_cindy = await clearance.path_hit(snapshot, alice.serial, straight(0, 0, -math.pi / 2, 1.8))
            open_space = await clearance.path_hit(snapshot, alice.serial, straight(0, 0, math.pi / 2, 1.0))
            here = await clearance.pose_hit(snapshot, alice.serial, (0.0, 0.0, 0.0))
        finally:
            clearance.close()
        return into_wall, toward_cindy, open_space, here

    into_wall, toward_cindy, open_space, here = asyncio.run(check())
    assert into_wall == "wall"
    assert toward_cindy is not None and "wall" not in toward_cindy
    assert open_space is None and here is None


def test_sampler_fills_number_levels_evenly():
    """Within a template, successive runs spread over each number's levels before repeating one."""
    sampler = Sampler([], random.Random(2))
    straights = []
    while len(straights) < 8:
        settings = next(sampler.candidates(per_template=1))
        sampler.add(settings)
        if settings["name"] == "Straight":
            straights.append(settings)
    assert sorted(s["parameters"]["length"] for s in straights) == sorted(GRID["Straight"]["length"])
    assert {(s["mode"], s["linear_speed"]) for s in straights} == {("Geometric", 0.2)}


def test_every_grid_curve_is_feasible_at_the_fixed_speed():
    """No path of the grid is tighter than the speed and turn-rate caps allow."""
    from husky_assembly_teleop.plugins.base_exp.auto import SPEED
    from husky_assembly_teleop.plugins.base_exp.tracking import Tracker
    linear, turn = SPEED
    tightest = linear / math.radians(turn)
    for name in GRID:
        for settings in cells(name):
            plan, _ = plan_from("a200-0804", settings, (0.0, 0.0, 0.0))
            curvature = np.nan_to_num(Tracker(plan.path)._curvature)
            assert np.abs(curvature).max() <= 1.0 / tightest + 0.05, settings


def test_away_from_the_start_pose():
    """With 'Return every run' the robot drives back unless it stands on its start pose (position and heading)."""
    import math

    from husky_assembly_teleop.plugins.base_exp.plugin import _away
    home = (1.0, -1.5, 0.0)
    assert not _away((1.02, -1.5, math.radians(3.0)), home)
    assert _away((1.1, -1.5, 0.0), home)
    assert _away((1.0, -1.5, math.radians(10.0)), home)
    assert not _away(None, home)


def test_standard_set_is_made_of_grid_cells():
    """Every standard path is a path grid cell, once, so the analysis can find it in any run."""
    grid = {cell_key(settings) for name in GRID for settings in cells(name)}
    keys = [cell_key(settings) for settings in standard_set()]
    assert set(keys) <= grid and len(set(keys)) == len(STANDARD_SET)


def test_standard_set_hands_out_the_least_attempted_first():
    """The set goes in order; attempted paths, done or failed, move to the back until every path has one more."""
    paths = standard_set()
    chooser = StandardSet()
    assert [cell_key(s) for s in chooser.candidates()] == [cell_key(s) for s in paths]
    chooser.count(paths[0], True)
    chooser.count(paths[1], False)
    order = [cell_key(s) for s in chooser.candidates()]
    assert order[0] == cell_key(paths[2]) and order[-2:] == [cell_key(paths[0]), cell_key(paths[1])]
    assert chooser.summary == f"round 1: 2/{len(paths)} paths"


def test_standard_set_resumes_from_earlier_attempts():
    """Attempts saved before (another series of the same setup) count, so a restarted series goes on where it left."""
    paths = standard_set()
    slow = {**paths[0], "linear_speed": 0.1}
    chooser = StandardSet([*paths, slow])
    assert chooser.summary == f"round 2: 0/{len(paths)} paths"
    assert cell_key(next(chooser.candidates())) == cell_key(paths[0])


def test_earlier_runs_match_the_current_setup(tmp_path):
    """Runs of the same experiment and setup are found from their experiment.json; other setups are not."""
    import json

    from husky_assembly_teleop.plugins.base_exp.record import CONDITIONS, current_key, earlier_runs
    # * A tuple parameter reads back from JSON as a list: the keys must still match.
    tuning = {"lookahead": 0.2, "gains": (1.0, 2.0)}
    setup = {"environment": "sim", "controller": {"name": "pure_pursuit", "parameters": tuning},
             "sim_model": {"xICR": -0.1}, "nodes": ["husky_sim"]}
    other = {**setup, "controller": {"name": "pure_pursuit", "parameters": {"lookahead": 0.3}}}
    for name, described in (("a", setup), ("b", other), ("c", setup)):
        (tmp_path / "exp" / name).mkdir(parents=True)
        (tmp_path / "exp" / name / "experiment.json").write_text(
            json.dumps({**described, "conditions": dict(CONDITIONS), "outcome": "done"}))
    found = earlier_runs(tmp_path, "exp", current_key(setup))
    assert len(found) == 2
    assert earlier_runs(tmp_path, "missing", current_key(setup)) == []
