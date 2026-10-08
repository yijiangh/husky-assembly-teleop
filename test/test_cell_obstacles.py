"""The cell plugin puts every body of a step's design scene into the scene; held ones follow the real robot."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
import viser
from design_fixtures import build_design
from test_compas_fab_mirror import configs, mirror, tool0, world  # noqa: F401 (fixtures)

from bar_assembly_core.design import Action, BodySpec, Movement, State, write
from bar_assembly_core.robot import Tool
from bar_assembly_core.geometry import box_geometry
from bar_assembly_core.robot import robot_model
from bar_assembly_core.geometry import Pose
from bar_assembly_core.scene import Attachment
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugins.cell.design import CellDesign, Step, displayed_joints, load_design, scene_bodies
from husky_assembly_teleop.plugins.cell.drawing import GHOST_OPACITY, DesignDrawing, load_robot_models
from bar_assembly_core.scene import Scene
from husky_assembly_teleop.world.scene import PluginScene


@pytest.fixture
def cell(tmp_path) -> CellDesign:
    """The fixture design, written and loaded the way the plugin loads it."""
    write(build_design(tmp_path), tmp_path / "design")
    return load_design(tmp_path / "design", tmp_path)


def _by_id(cell: CellDesign, step_index: int, robots=()) -> dict:
    """The scene bodies of one step, by scene id, with `robots` configured."""
    return {body.id: body for body in scene_bodies(cell, cell.steps[step_index], robots, "cell/")}


def _real_cindy(cell: CellDesign) -> SimpleNamespace:
    """A configured robot standing in for Cindy (serial 0806): the design's URDF, with its own tool on left_tool0."""
    spec = cell.design.robots["robots/cindy"]
    tool = Tool("tools/a200-0806/left", box_geometry((0.05, 0.05, 0.05)), Pose(), "scaffolding_v3")
    return SimpleNamespace(serial="a200-0806", model=robot_model("a200-0806", spec.urdf, spec.srdf,
                                                                 {"left_tool0": tool}))


def _enabled(bodies: dict) -> set[str]:
    """The ids of the enabled ones."""
    return {body_id for body_id, body in bodies.items() if body.enabled}


def test_every_body_every_step(cell):
    """The same ids in every step, whatever stands: switching steps never adds or removes a body."""
    expected = {f"cell/{body_id}" for body_id in cell.design.bodies}
    assert all(set(_by_id(cell, index)) == expected for index in range(len(cell.steps)))


def test_held_bodies_disabled_robot_touches_dropped(cell):
    """B1_M1, no robot configured: B1 and its halves are held by cindy, so only B2 (on its rack), J1 and WG0 stand."""
    bodies = _by_id(cell, 1)
    assert _enabled(bodies) == {"cell/bars/B2", "cell/joints/J1_male", "cell/ground/WG0"}
    assert bodies["cell/ground/WG0"].touches == ("cell/joints/G1_ground",)


def test_placed_bodies_poses_and_derived_touches(cell):
    """B1_H_M0: B1 is built, so it and its halves stand at their design poses; a half may touch its bar."""
    bodies = _by_id(cell, 3)
    assert _enabled(bodies) == {f"cell/{body_id}" for body_id in cell.design.bodies}
    assert bodies["cell/joints/J1_female"].placement == cell.design.bodies["joints/J1_female"].pose
    assert bodies["cell/joints/J1_female"].touches == ("cell/bars/B1",)


def test_design_geometry_and_fixed_colour_every_step(cell):
    """The design's own geometry object and one colour per body in every step, so nothing is ever rebuilt."""
    first, later = _by_id(cell, 1), _by_id(cell, 3)
    assert first["cell/bars/B2"].geometry is cell.design.bodies["bars/B2"].geometry
    assert first["cell/bars/B2"].geometry is later["cell/bars/B2"].geometry
    assert first["cell/bars/B2"].color == later["cell/bars/B2"].color == (205 / 255, 170 / 255, 110 / 255, 1.0)
    assert first["cell/bars/B2"].label == "bars/B2"  # no label in the design: its design id
    assert later["cell/bars/B1"].label == "first bar" and later["cell/bars/B1"].enabled


def test_held_body_follows_the_configured_robot(cell):
    """With Cindy configured, B1 is held by her real link, and touches name the real robot and its tool."""
    bodies = _by_id(cell, 1, (_real_cindy(cell),))
    grasp = cell.steps[1].movement.start.attached["bars/B1"][0].grasp
    assert bodies["cell/bars/B1"].enabled
    assert bodies["cell/bars/B1"].placement == Attachment("robots/a200-0806", "left_tool0", grasp)
    assert "tools/a200-0806/left" in bodies["cell/joints/G1_ground"].touches
    assert bodies["cell/ground/WG0"].touches == ("cell/joints/G1_ground", "robots/a200-0806/wheel_link")


def test_robot_without_the_held_link_is_refused(cell):
    """The real Cindy's URDF has no `left_tool0`: retargeting the grasp is refused, not guessed."""
    real = robot_config_from_serial("0806", Path(__file__).resolve().parent.parent / "data")
    with pytest.raises(ValueError, match="left_tool0"):
        _by_id(cell, 1, (real,))


def test_scene_accepts_them(cell):
    """Every id is valid and owned by "cell/"; each has collision shapes, so nothing is warned about."""
    warnings = []
    PluginScene(Scene(), "cell", warnings.append, Scene).put_many(_by_id(cell, 2).values())
    assert warnings == []


@pytest.mark.slow
def test_arm_planner_mirror_sees_a_cell_obstacle(mirror, configs, tmp_path):  # noqa: F811
    """A design body at Alice's tool0 collides in the arm planner's compas_fab mirror, under its cell id."""
    mirror.sync(world(configs))
    design = replace(build_design(tmp_path), bodies={
        "obstacles/box": BodySpec("obstacles/box", tool0(mirror), box_geometry((0.1, 0.1, 0.1)))})
    cell = CellDesign(tmp_path, design, ())
    movement = Movement("M0", State(robots={}, present=frozenset()), ends_on="operator")
    step = Step(Action("A0", "bar_jointing", "robots/cindy", "bars/B1", (movement,)), 0, 0, movement)

    mirror.sync(world(configs, scene_bodies(cell, step, (), "cell/")))
    assert any("cell/obstacles/box" in pair for pair in mirror.collisions(full_report=True))


def test_overlay_draws_only_what_the_core_does_not(cell):
    """Standing bodies are left to the core, held ones are drawn, absent ones only with `everything`."""
    server = viser.ViserServer(port=0, verbose=False)
    try:
        drawing = DesignDrawing(server, "/design", cell.design, load_robot_models(cell.design))
        for _ in drawing.build():
            pass
        shown = lambda: {body_id for body_id, (frame, _) in drawing._bodies.items() if frame.visible}  # noqa: E731
        held, absent = cell.steps[1], cell.steps[0]
        b1 = {"bars/B1", "joints/G1_ground", "joints/J1_female"}
        drawing.show(held.movement.start, displayed_joints(held, False))
        assert shown() == b1
        drawing.show(absent.movement.start, displayed_joints(absent, False))
        assert shown() == set()
        drawing.show(absent.movement.start, displayed_joints(absent, False), everything=True)
        assert shown() == b1
        assert all(mesh.opacity == GHOST_OPACITY for mesh in drawing._bodies["bars/B1"][1])
    finally:
        server.stop()
