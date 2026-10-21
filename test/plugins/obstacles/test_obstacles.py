"""Tests for the obstacles plugin: the lab file, temporary obstacles, and editing without the UI."""

from __future__ import annotations

import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from repo_paths import DATA
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.plugins.obstacles.plugin import ObstaclesPlugin
from husky_assembly_teleop.shared.obstacles_contract import ObstaclesContract
from husky_assembly_teleop.world import lab as lab_module
from husky_assembly_teleop.world.base_collision import BaseCollisionWorld
from husky_assembly_teleop.world.clearance import STOP_DISTANCE
from husky_assembly_teleop.world.lab import (FLOOR_WITHOUT_BORDER, Shape, border_bodies, floor_body, read_lab,
                                             read_temporary, shape_id, write_lab, write_temporary)
from husky_assembly_teleop.world.scene import PluginScene
from scaffolding_assembly_core.geometry import Pose, box_geometry
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Attachment, Body, Scene, world_poses

#: The ids `obstacles` put before the lab moved into a file (2026-10-15): they must not change. The floor came later.
IDS = ["obstacles/boxes/bar_table_1", "obstacles/boxes/bar_table_2", "obstacles/boxes/border_x_max",
       "obstacles/boxes/border_x_min", "obstacles/boxes/border_y_max", "obstacles/boxes/border_y_min",
       "obstacles/boxes/building_cabinet", "obstacles/boxes/main_pc_table", "obstacles/boxes/pc_table_2",
       "obstacles/boxes/pc_table_3", "obstacles/cylinders/tripod_3", "obstacles/cylinders/tripod_ada",
       "obstacles/cylinders/tripod_net", "obstacles/trusses/truss_1", "obstacles/trusses/truss_2"]


@pytest.fixture
def files(tmp_path, monkeypatch):
    """Temporary-obstacle files in a temporary folder, and a copy of the main lab file there."""
    monkeypatch.setattr(lab_module, "TEMPORARY_FILE", tmp_path / "temporary.json")
    monkeypatch.setattr(lab_module, "OLD_TEMPORARY_FILE", tmp_path / "debug_gizmo.json")
    (tmp_path / "lab").mkdir()
    (tmp_path / "lab" / "main.json").write_text((DATA / "lab" / "main.json").read_text())
    return tmp_path


def _context(data_directory: Path, robots: tuple = ()) -> SimpleNamespace:
    """The part of a plugin context the plugin uses without its UI, with an empty scene and `robots` configured."""
    messages: list[str] = []
    scene = PluginScene(Scene(), ObstaclesPlugin.name, log_warn=messages.append, snapshot=Scene)
    return SimpleNamespace(scene=scene, config=SimpleNamespace(data_directory=data_directory, lab="", robots=robots),
                           log_info=messages.append, log_warn=messages.append, log_error=messages.append,
                           messages=messages)


def _loaded(files: Path, robots: tuple = ()) -> tuple[ObstaclesPlugin, SimpleNamespace]:
    """The plugin with its files read and its bodies put, as setup does before the UI."""
    plugin, ctx = ObstaclesPlugin(), _context(files, robots)
    plugin.load(ctx)
    plugin._put_all(ctx)
    return plugin, ctx


# --- --- --- --- --- THE LAB FILE --- --- --- --- ---

def test_main_lab_keeps_the_ids(files):
    """The lab file gives the same bodies, under the same ids, as the constants it replaced."""
    plugin, ctx = _loaded(files)
    assert sorted(ctx.scene.bodies) == sorted(IDS + ["obstacles/floor"])
    assert all(body_id.startswith("obstacles/") for body_id in ctx.scene.bodies)


def test_offers_the_lab_with_its_border(files):
    """Through its contract the plugin offers the lab as loaded, border included; an edit replaces it."""
    plugin, ctx = _loaded(files)
    obstacles: ObstaclesContract = plugin
    before = obstacles.lab
    assert before.border is not None and before.shapes
    plugin.remove(ctx, shape_id(plugin.name, before.shapes[0]))
    assert obstacles.lab is not before and len(obstacles.lab.shapes) == len(before.shapes) - 1
    assert obstacles.lab.border == before.border


def test_lab_file_round_trip(tmp_path):
    """Written and read back, a lab is the same; each obstacle is one line of the file."""
    lab = read_lab(DATA / "lab" / "main.json")
    write_lab(lab, tmp_path / "lab.json")
    assert read_lab(tmp_path / "lab.json") == lab
    lines = (tmp_path / "lab.json").read_text().splitlines()
    assert sum('"type":' in line for line in lines) == len(lab.shapes)


def test_broken_lab_file_is_refused(tmp_path):
    """Not a lab file, or two obstacles with one name: an error, never a half-read lab."""
    (tmp_path / "a.json").write_text(json.dumps({"format": "other"}))
    with pytest.raises(ValueError):
        read_lab(tmp_path / "a.json")
    box = {"type": "box", "name": "t", "center": [0, 0, 0], "size": [1, 1, 1]}
    (tmp_path / "b.json").write_text(json.dumps({"format": "husky_lab", "obstacles": [box, box]}))
    with pytest.raises(ValueError, match="share a name"):
        read_lab(tmp_path / "b.json")


def test_shapes_stay_upright_and_square():
    """A drag keeps only the yaw; a cylinder keeps x and y of its size equal."""
    shape = Shape("cylinder", "c", (0.0, 0.0, 1.0), (0.4, 0.4, 2.0))
    tilted = Pose((1.0, 2.0, 1.0), (math.sin(0.1), 0.0, math.sin(0.3), math.cos(0.3)))
    moved = shape.moved(tilted)
    assert moved.center == (1.0, 2.0, 1.0) and moved.pose.orientation[:2] == (0.0, 0.0)
    assert moved.resized((0.6, 0.2, 1.5)).size == (0.6, 0.6, 1.5)


# --- --- --- --- --- TEMPORARY OBSTACLES --- --- --- --- ---

def test_debug_gizmo_obstacles_are_taken_over(files):
    """Without a temporary file, the debug gizmo's old one is read once; the next write makes the new file."""
    (files / "debug_gizmo.json").write_text(json.dumps({"shapes": [
        {"kind": "cylinder", "number": 4, "size": [0.3, 0.3, 1.2], "position": [3.0, -1.0, 0.6],
         "orientation": [0.0, 0.0, 0.7071068, 0.7071068]}]}))
    plugin, ctx = _loaded(files)
    assert plugin.temporary[4].type == "cylinder" and plugin.temporary[4].yaw == pytest.approx(math.pi / 2)
    assert "obstacles/temporary/4" in ctx.scene.bodies
    plugin.write_temporary(ctx)
    assert plugin.write_temporary(ctx) is None and not (files / "temporary.json").exists(), "unchanged: not written"
    plugin.add(ctx, "box")
    plugin.write_temporary(ctx)
    assert set(read_temporary()) == {4, 5}, "numbers go on after the old ones"


def test_temporary_file_round_trip(files):
    """Written and read back, temporary obstacles are the same."""
    shapes = {2: Shape("box", "tmp box 2", (1.0, 2.0, 0.5), (0.5, 0.4, 1.0), yaw=0.5, source="temporary")}
    write_temporary(shapes)
    read = read_temporary()
    assert read[2].size == shapes[2].size and read[2].yaw == pytest.approx(0.5, abs=1e-4), "kept to 0.01°"


def test_broken_temporary_file_is_never_overwritten(files):
    """A file that cannot be read starts empty with a warning, and is kept."""
    (files / "temporary.json").write_text("{not json")
    plugin, ctx = _loaded(files)
    plugin.add(ctx, "box")
    plugin.write_temporary(ctx)
    assert (files / "temporary.json").read_text() == "{not json" and ctx.messages


# --- --- --- --- --- EDITING --- --- --- --- ---

def test_edit_save_and_revert(files):
    """A move makes the lab unsaved; Save writes it, Revert after another edit goes back to the file."""
    plugin, ctx = _loaded(files)
    table = "obstacles/boxes/pc_table_2"
    plugin._move(ctx, table, Pose((-1.0, 2.4, 0.365)))
    assert plugin.dirty and ctx.scene.bodies[table].placement.position == (-1.0, 2.4, 0.365)
    plugin.save(ctx)
    assert not plugin.dirty and read_lab(plugin.path) == plugin.lab
    plugin.set_typed(ctx, table, (0.0, 0.0, 0.365), 30.0, (2.0, 0.8, 0.73))
    assert plugin.dirty and ctx.scene.bodies[table].geometry.collision[0].size == (2.0, 0.8, 0.73)
    plugin.revert(ctx)
    assert not plugin.dirty and ctx.scene.bodies[table].placement.position == (-1.0, 2.4, 0.365)


def test_promote_rename_and_remove(files):
    """A temporary obstacle moves into the lab under a free name, can be renamed (a new id), and removed."""
    plugin, ctx = _loaded(files)
    temporary = plugin.add(ctx, "cylinder")
    promoted = plugin.promote(ctx, temporary)
    assert promoted == "obstacles/cylinders/cylinder_1" and temporary not in ctx.scene.bodies
    assert plugin.temporary == {} and plugin.dirty
    renamed = plugin.rename(ctx, promoted, "tripod net")
    assert renamed == "obstacles/cylinders/tripod_net_2", "the name was taken"
    assert renamed in ctx.scene.bodies and promoted not in ctx.scene.bodies
    plugin.remove(ctx, renamed)
    assert renamed not in ctx.scene.bodies and renamed not in plugin.shapes()


def test_border_follows_its_range(files):
    """New x and y ranges move the walls, under the same ids."""
    plugin, ctx = _loaded(files)
    plugin.set_border(ctx, (-2.0, 2.0), (3.0, -3.0))
    assert plugin.lab.border.y == (-3.0, 3.0)
    walls = {body.id: body for body in border_bodies("obstacles", plugin.lab.border)}
    assert set(walls) <= set(ctx.scene.bodies)
    assert ctx.scene.bodies["obstacles/boxes/border_x_max"].placement.position[0] == pytest.approx(2.1)


# --- --- --- --- --- THE FLOOR --- --- --- --- ---

def test_floor_lies_under_the_border_walls(files):
    """A slab just under z = 0 out to the walls' outer edge, never drawn, following the border."""
    plugin, ctx = _loaded(files)
    floor = ctx.scene.bodies["obstacles/floor"]
    assert floor.geometry.visual == () and floor.geometry.collision
    plugin.set_border(ctx, (-2.0, 2.0), (-1.0, 3.0))
    floor = ctx.scene.bodies["obstacles/floor"]
    (box,) = floor.geometry.collision
    size, center = box.size, floor.placement.position
    assert center[2] + size[2] / 2 == pytest.approx(0.0)
    assert (center[0] - size[0] / 2, center[0] + size[0] / 2) == pytest.approx((-2.2, 2.2))
    assert (center[1] - size[1] / 2, center[1] + size[1] / 2) == pytest.approx((-1.2, 3.2))
    (box,) = floor_body("obstacles", None, ()).geometry.collision
    assert box.size[:2] == (FLOOR_WITHOUT_BORDER, FLOOR_WITHOUT_BORDER)


def test_floor_lets_the_platform_touch_it_but_not_what_the_arms_hold(files):
    """Alice standing on the floor keeps clear of it; a box she holds 5 cm above it does not."""
    config = robot_config_from_serial("0804", DATA)
    plugin, ctx = _loaded(files, (config,))
    own = robot_id(config.serial)
    floor = ctx.scene.bodies["obstacles/floor"]
    assert f"{own}/front_left_wheel_link" in floor.touches and f"{own}/base_link" in floor.touches
    assert f"{own}/ur_arm_base_link" in floor.touches and f"{own}/ur_arm_shoulder_link" not in floor.touches

    held = Body("test/held", box_geometry((0.1, 0.1, 0.1)), Attachment(own, None, Pose((0.8, 0.0, 0.1))))
    world = BaseCollisionWorld()
    try:
        for bodies, hit in (((floor,), None), ((floor, held), "floor")):
            robots = {own: RobotObject(own, config.model, Pose(), {})}
            bodies = {body.id: body for body in bodies}
            world.sync(Scene(bodies=bodies, robots=robots, world_poses=world_poses(
                bodies, robots, lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link))))
            assert world.hit_by(own, (0.0, 0.0, 0.0), STOP_DISTANCE) == hit
    finally:
        world.close()


def test_shape_ids_follow_the_name():
    """Ids are "<plugin>/<group>/<slug of the name>"."""
    assert shape_id("obstacles", Shape("truss", "truss 1", (0, 0, 1), (0.3, 0.3, 2))) == "obstacles/trusses/truss_1"
