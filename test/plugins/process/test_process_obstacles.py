"""The process plugin puts every body of a movement's process scene into the scene; held ones follow the real robot."""

from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import pytest
import viser
from process_fixtures import build_process, with_start

from repo_paths import DATA
from scaffolding_assembly_core.process import Action, BodySpec, Movement, Process, Target, joints_at, write
from scaffolding_assembly_core.robot import Tool
from scaffolding_assembly_core.geometry import box_geometry, cylinder_geometry
from scaffolding_assembly_core.robot import robot_model
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.scene import Attachment
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.plugins.process.scene_bodies import live_ids, scene_bodies
from husky_assembly_teleop.plugins.process.drawing import GHOST_OPACITY, ProcessDrawing, load_robot_models
from husky_assembly_teleop.plugins.process.plugin import BY_HAND_LABEL, ProcessPlugin
from scaffolding_assembly_core.scene import Scene
from husky_assembly_teleop.world.scene import PluginScene


@pytest.fixture
def process(tmp_path) -> Process:
    """The fixture process, written and process the way the plugin loads it."""
    write(build_process(tmp_path), tmp_path / "process")
    return load_process(tmp_path / "process", tmp_path)


def _movement(process: Process, index: int) -> tuple[Action, Movement]:
    """The schedule's movement at `index`, with its action."""
    return list(process.movements())[index]


def _by_id(process: Process, index: int, robots=()) -> dict:
    """The scene bodies at the start of the movement at `index`, by scene id, with `robots` configured."""
    return {body.id: body for body in scene_bodies(process, _movement(process, index)[1], robots, "process/")}


def _real_cindy(process: Process) -> SimpleNamespace:
    """A configured robot standing in for Cindy (serial 0806): the process's URDF, with its own tool on left_tool0."""
    spec = process.robots["robots/cindy"]
    tool = Tool("tools/a200-0806/left", box_geometry((0.05, 0.05, 0.05)), Pose(), "scaffolding_v3")
    return SimpleNamespace(serial="a200-0806", model=robot_model("a200-0806", spec.urdf, spec.srdf,
                                                                 {"left_tool0": tool}))


def _enabled(bodies: dict) -> set[str]:
    """The ids of the enabled ones."""
    return {body_id for body_id, body in bodies.items() if body.enabled}


def test_every_body_every_movement(process):
    """The same ids at every movement, whatever stands: changing the movement never adds or removes a body."""
    expected = {f"process/{body_id}" for body_id in process.bodies}
    assert all(set(_by_id(process, index)) == expected for index in range(len(list(process.movements()))))


def test_held_bodies_disabled_robot_touches_dropped(process):
    """B1_M1, no robot configured: B1 and its halves are held by cindy, so only B2 (on its rack), J1 and WG0 stand."""
    bodies = _by_id(process, 1)
    assert _enabled(bodies) == {"process/bars/B2", "process/joints/J1_male", "process/ground/WG0"}
    assert bodies["process/ground/WG0"].touches == ("process/joints/G1_ground",)


def test_configured_robot_may_stand_on_the_ground_where_the_plan_has_it_absent(tmp_path):
    """A step whose plan has Cindy absent: the real Cindy still stands on the floor, so her wheel may touch WG0."""
    process = build_process(tmp_path)
    step = next(index for index, (action_id, action) in enumerate(process.actions.items())
                if action.robot == "robots/alice")
    action_id = list(process.actions)[step]
    start = process.actions[action_id].movements[0].start
    process = with_start(process, action_id, 0, robots={**start.robots, "robots/cindy": None})
    write(process, tmp_path / "process")
    process = load_process(tmp_path / "process", tmp_path)
    index = next(i for i, (action, _) in enumerate(process.movements()) if action.id == action_id)
    assert _movement(process, index)[1].start.robots["robots/cindy"] is None
    ground = _by_id(process, index, (_real_cindy(process),))["process/ground/WG0"]
    assert "robots/a200-0806/wheel_link" in ground.touches


def test_placed_bodies_poses_and_derived_touches(process):
    """B1_H_M0: B1 is built, so it and its halves stand at their assembly poses; a half may touch its bar."""
    bodies = _by_id(process, 3)
    assert _enabled(bodies) == {f"process/{body_id}" for body_id in process.bodies}
    assert bodies["process/joints/J1_female"].placement == process.bodies["joints/J1_female"].assembly_pose
    assert bodies["process/joints/J1_female"].touches == ("process/bars/B1",)


def test_process_geometry_and_fixed_colour_every_movement(process):
    """The process's own geometry object and one colour per body at every movement, so nothing is ever rebuilt."""
    first, later = _by_id(process, 1), _by_id(process, 3)
    assert first["process/bars/B2"].geometry is process.bodies["bars/B2"].geometry
    assert first["process/bars/B2"].geometry is later["process/bars/B2"].geometry
    assert first["process/bars/B2"].color == later["process/bars/B2"].color == (205 / 255, 170 / 255, 110 / 255, 1.0)
    assert first["process/bars/B2"].label == "bars/B2"  # no label in the process: its process id
    assert later["process/bars/B1"].label == "first bar" and later["process/bars/B1"].enabled


def test_held_body_follows_the_configured_robot(process):
    """With Cindy configured, B1 is held by her real link, and touches name the real robot and its tool."""
    bodies = _by_id(process, 1, (_real_cindy(process),))
    grasp = _movement(process, 1)[1].start.attached["bars/B1"][0].grasp
    assert bodies["process/bars/B1"].enabled
    assert bodies["process/bars/B1"].placement == Attachment("robots/a200-0806", "left_tool0", grasp)
    assert "tools/a200-0806/left" in bodies["process/joints/G1_ground"].touches
    assert bodies["process/ground/WG0"].touches == ("process/joints/G1_ground", "robots/a200-0806/wheel_link")


def test_live_ids_both_ways(process):
    """Bodies get the prefix, Cindy and her links and tool become the real ones; unconfigured Alice is not mapped."""
    ids, back = live_ids(process, (_real_cindy(process),), "process/")
    tool = process.robots["robots/cindy"].tools["left_tool0"]
    assert ids("bars/B1") == "process/bars/B1" and back("process/bars/B1") == "bars/B1"
    assert ids("robots/cindy/left_tool0") == "robots/a200-0806/left_tool0"
    assert back("robots/a200-0806/wheel_link") == "robots/cindy/wheel_link"
    assert ids(tool) == "tools/a200-0806/left" and back("tools/a200-0806/left") == tool
    assert "robots/alice" not in ids and "tracked/probe" not in back


def test_live_ids_refuse_two_robots_on_one_serial(process):
    """Two planned robots with one serial would make the way back ambiguous: refused."""
    robots = {robot_id: replace(spec, serial="0806") for robot_id, spec in process.robots.items()}
    with pytest.raises(ValueError, match="both map to 'robots/a200-0806'"):
        live_ids(replace(process, robots=robots), (_real_cindy(process),), "process/")


def test_robot_without_the_held_link_is_refused(process):
    """The real Cindy's URDF has no `left_tool0`: retargeting the grasp is refused, not guessed."""
    real = robot_config_from_serial("0806", DATA)
    with pytest.raises(ValueError, match="left_tool0"):
        _by_id(process, 1, (real,))


def test_scene_accepts_them(process):
    """Every id is valid and owned by "process/"; each has collision shapes, so nothing is warned about."""
    warnings = []
    PluginScene(Scene(), "process", warnings.append, Scene).put_many(_by_id(process, 2).values())
    assert warnings == []


def test_overlay_draws_only_what_the_core_does_not(process):
    """Standing bodies are left to the runtime, held ones are drawn, absent ones only with `everything`."""
    server = viser.ViserServer(port=0, verbose=False)
    try:
        drawing = ProcessDrawing(server, "/process", process, load_robot_models(process))
        for _ in drawing.build():
            pass
        shown = lambda: {body_id for body_id, (frame, _) in drawing._bodies.items() if frame.visible}  # noqa: E731
        (held_action, held), (absent_action, absent) = _movement(process, 1), _movement(process, 0)
        b1 = {"bars/B1", "joints/G1_ground", "joints/J1_female"}
        drawing.show(held.start, joints_at(process, held_action.id, held.id))
        assert shown() == b1
        drawing.show(absent.start, joints_at(process, absent_action.id, absent.id))
        assert shown() == set()
        drawing.show(absent.start, joints_at(process, absent_action.id, absent.id), everything=True)
        assert shown() == b1
        assert all(mesh.opacity == GHOST_OPACITY for mesh in drawing._bodies["bars/B1"][1])
    finally:
        server.stop()


def test_bar_placed_by_hand(tmp_path):
    """A `bar_placing` action has no robot: it loads, its scene builds, and the panel says "by hand"."""
    process = build_process(tmp_path)
    last = process.actions[process.schedule[-1]].movements[-1].start
    start = replace(last, present=last.present | {"bars/B3"})
    placing = Action("B3_P", "bar_placing", "", "bars/B3", (
        Movement("B3_P_place", start, ends_on="operator", target=Target.of(built=start.built | {"bars/B3"})),))
    bar = BodySpec("bars/B3", Pose((1.0, 0.0, 0.1)), cylinder_geometry(0.01, 0.5))
    process = replace(process, schedule=(*process.schedule, placing.id), bodies={**process.bodies, bar.id: bar},
                      actions={**process.actions, placing.id: placing})
    write(process, tmp_path / "process")
    process = load_process(tmp_path / "process", tmp_path)
    action, movement = list(process.movements())[-1]
    assert action.robot == "" and "robots/cindy" in joints_at(process, action.id, movement.id, at_end=True)
    assert "process/bars/B3" in {body.id for body in scene_bodies(process, movement, (), "process/") if body.enabled}
    panel = ProcessPlugin()
    panel._take(process)
    panel.select(len(panel._movements) - 1)
    assert BY_HAND_LABEL in panel._details_html(*panel.selected)
