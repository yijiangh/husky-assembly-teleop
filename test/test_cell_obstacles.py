"""The cell plugin puts every design body into the scene, enabled where it stands; the core draws them."""

from __future__ import annotations

from dataclasses import replace

import pytest
import viser
from design_io_fixtures import build_design
from test_compas_fab_mirror import configs, mirror, tool0, world  # noqa: F401 (fixtures)

from bar_assembly_core.design_io import BodySpec, State, box_geometry, write
from husky_assembly_teleop.plugins.cell.design import CellDesign, displayed_joints, load_design, obstacles
from husky_assembly_teleop.plugins.cell.drawing import GHOST_OPACITY, DesignDrawing, load_robot_models
from husky_assembly_teleop.world.scene import PluginScene, Scene


@pytest.fixture
def cell(tmp_path) -> CellDesign:
    """The fixture design, written and loaded the way the plugin loads it."""
    write(build_design(tmp_path), tmp_path / "design")
    return load_design(tmp_path / "design", tmp_path)


def _by_id(cell: CellDesign, step_index: int) -> dict:
    """The obstacles of one step's start state, by scene id."""
    return {body.id: body for body in obstacles(cell, cell.steps[step_index].movement.start, "cell/")}


def _enabled(bodies: dict) -> set[str]:
    """The ids of the enabled ones."""
    return {body_id for body_id, body in bodies.items() if body.enabled}


def test_every_body_every_step(cell):
    """The same ids in every step, whatever stands: switching steps never adds or removes a body."""
    expected = {f"cell/{body_id}" for body_id in cell.design.bodies}
    assert all(set(_by_id(cell, index)) == expected for index in range(len(cell.steps)))


def test_held_and_absent_bodies_disabled_robot_touches_dropped(cell):
    """B1_M0: B1 is held by cindy and B2, J2 are absent, so only the joint and the ground stand."""
    bodies = _by_id(cell, 0)
    assert _enabled(bodies) == {"cell/joints/J1_male", "cell/ground/WG0"}
    assert bodies["cell/ground/WG0"].touches == ()


def test_placeholders_disabled_poses_and_touches_kept(cell):
    """B1_M1: B1's pose is a placeholder; J1 is moved; J2 may touch B1 (listed on J2's side only)."""
    bodies = _by_id(cell, 1)
    assert not bodies["cell/bars/B1"].enabled
    assert bodies["cell/bars/B2"].enabled
    assert bodies["cell/joints/J1_male"].placement.position == pytest.approx((0.1, 0.0, 0.01))
    assert bodies["cell/joints/J2_male"].touches == ("cell/bars/B1",)


def test_design_geometry_and_fixed_colour_every_step(cell):
    """The design's own geometry object and one colour per body in every step, so nothing is ever rebuilt."""
    first, later = _by_id(cell, 1), _by_id(cell, 2)
    assert first["cell/bars/B2"].geometry is cell.design.bodies["bars/B2"].geometry
    assert first["cell/bars/B2"].geometry is later["cell/bars/B2"].geometry
    assert first["cell/bars/B2"].color == later["cell/bars/B2"].color == (205 / 255, 170 / 255, 110 / 255, 1.0)
    assert first["cell/bars/B2"].label == "bars/B2"  # no label in the design: its design id
    assert later["cell/bars/B1"].label == "first bar" and later["cell/bars/B1"].enabled


def test_scene_accepts_them(cell):
    """Every id is valid and owned by "cell/"; each has collision shapes, so nothing is warned about."""
    warnings = []
    PluginScene(Scene(), "cell", warnings.append).put_many(_by_id(cell, 2).values())
    assert warnings == []


@pytest.mark.slow
def test_arm_planner_mirror_sees_a_cell_obstacle(mirror, configs, tmp_path):  # noqa: F811
    """A design body at Alice's tool0 collides in the arm planner's compas_fab mirror, under its cell id."""
    mirror.sync(world(configs))
    design = replace(build_design(tmp_path), bodies={
        "obstacles/box": BodySpec("obstacles/box", tool0(mirror), box_geometry((0.1, 0.1, 0.1)))})
    cell = CellDesign(tmp_path, design, ())
    state = State(robots={}, present=frozenset({"obstacles/box"}), poses={}, attached={})

    mirror.sync(world(configs, obstacles(cell, state, "cell/")))
    assert any("cell/obstacles/box" in pair for pair in mirror.collisions(full_report=True))


def test_overlay_draws_only_what_the_core_does_not(cell):
    """B1_M0: standing bodies are left to the core, the held bar is drawn, absent ones only with `everything`."""
    server = viser.ViserServer(port=0, verbose=False)
    try:
        drawing = DesignDrawing(server, "/design", cell.design, load_robot_models(cell.design))
        for _ in drawing.build():
            pass
        step = cell.steps[0]
        shown = lambda: {body_id for body_id, (frame, _) in drawing._bodies.items() if frame.visible}  # noqa: E731
        drawing.show(step.movement.start, displayed_joints(step, False))
        assert shown() == {"bars/B1"}
        drawing.show(step.movement.start, displayed_joints(step, False), everything=True)
        assert shown() == {"bars/B1", "bars/B2", "joints/J2_male"}
        assert all(mesh.opacity == GHOST_OPACITY for mesh in drawing._bodies["bars/B2"][1])
    finally:
        server.stop()
