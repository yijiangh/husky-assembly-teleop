"""The build's placement: one rigid move of the build, the lab left; the panel's Apply and the run's request."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import purged_process
from plugin_fakes import FakeContext
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.plugins.process.placement import IDENTITY, off_floor, placed
from husky_assembly_teleop.plugins.process.plugin import ProcessPlugin
from husky_assembly_teleop.robot_interface.robot_lease import LeaseSlot, take_lease
from husky_assembly_teleop.shared.process_contract import MARK_DONE
from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.scene import Attachment

#: Turned and lifted, so every part of the transform shows.
PLACEMENT = Pose.from_arrays([1.5, -0.7, 0.2], Rotation.from_euler("xyz", [3.0, -2.0, 37.0], degrees=True).as_quat())
#: On the floor: x, y and a yaw.
ON_FLOOR = Pose.from_arrays([1.5, -0.7, 0.0], Rotation.from_euler("z", 37.0, degrees=True).as_quat())


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process().process


def _close(first: Pose, second: Pose) -> bool:
    return np.allclose(first.matrix(), second.matrix(), atol=1e-9)


def test_the_build_moves_rigidly_obstacles_included_and_the_floor_stays(purged):
    """Every build pose of the scene at a movement's end is the authored one moved, obstacles too; the floor is not.

    A body a robot holds follows that robot's base, kept on the floor, and the process's joints: off the floor plane
    the live IK takes it to its target, so it is left out here.
    """
    moved = placed(purged, PLACEMENT)
    pairs = list(zip(purged.movements(), moved.movements()))
    checked = 0
    for (_action, authored), (_same, placed_movement) in pairs[::25]:
        before = purged.scene_at(authored, at_end=True).world_poses
        after = moved.scene_at(placed_movement, at_end=True).world_poses
        held = {body_id for body_id, body in moved.scene_at(placed_movement, at_end=True).bodies.items()
                if isinstance(body.placement, Attachment)}
        for body_id, pose in before.items():
            if body_id in held:
                continue
            lab = body_id.startswith("ground/")
            assert _close(after[body_id], pose if lab else compose(PLACEMENT, pose)), body_id
            checked += not lab
    assert checked > 100
    assert any(body_id.startswith("obstacles/") for body_id in before), "the purged process has no obstacles"


def _on_floor(base: Pose) -> Pose:
    """`base` moved by PLACEMENT, projected onto the floor: its own height, turned about z by PLACEMENT's heading."""
    moved = compose(PLACEMENT, base)
    heading = Rotation.from_quat(PLACEMENT.orientation).apply([1.0, 0.0, 0.0])
    turn = Rotation.from_euler("z", np.arctan2(heading[1], heading[0]))
    return Pose.from_arrays([moved.position[0], moved.position[1], base.position[2]],
                            (turn * Rotation.from_quat(base.orientation)).as_quat())


def test_bases_targets_and_lines_move_too(purged):
    """Robot bases (start, target) stay on the floor; a target's flange poses and a line's direction move rigidly."""
    moved = placed(purged, PLACEMENT)
    rotation = Rotation.from_quat(PLACEMENT.orientation)
    for (_action, authored), (_same, movement) in zip(purged.movements(), moved.movements()):
        for robot_id, robot in authored.start.robots.items():
            if robot is not None and robot.base is not None:
                moved = movement.start.robots[robot_id].base
                assert _close(moved, _on_floor(robot.base))
                assert np.isclose(moved.position[2], robot.base.position[2])
                assert np.allclose(Rotation.from_quat(moved.orientation).apply([0.0, 0.0, 1.0]),
                                   Rotation.from_quat(robot.base.orientation).apply([0.0, 0.0, 1.0]))
        if authored.target is not None:
            for link, pose in authored.target.link_poses.items():
                assert _close(movement.target.link_poses[link], compose(PLACEMENT, pose))
            for robot_id, base in authored.target.bases.items():
                assert _close(movement.target.bases[robot_id], _on_floor(base))
        for flange, line in authored.line.items():
            np.testing.assert_allclose(movement.line[flange].direction, rotation.apply(line.direction), atol=1e-12)


def test_a_floor_placement_moves_the_bases_rigidly(purged):
    """On the floor (x, y, yaw), the projection changes nothing: bases move by the placement itself."""
    moved = placed(purged, ON_FLOOR)
    for (_action, authored), (_same, movement) in list(zip(purged.movements(), moved.movements()))[::10]:
        for robot_id, robot in authored.start.robots.items():
            if robot is not None and robot.base is not None:
                assert np.allclose(movement.start.robots[robot_id].base.matrix(),
                                   compose(ON_FLOOR, robot.base).matrix(), atol=1e-9)


def test_no_placement_is_the_authored_process_and_off_floor_is_told(purged):
    """The identity returns the process itself; a height or a tilt is off the floor, a floor move is not."""
    assert placed(purged, IDENTITY) is purged
    assert off_floor(PLACEMENT) and not off_floor(ON_FLOOR) and not off_floor(IDENTITY)


def _set_up(process) -> tuple[ProcessPlugin, FakeContext]:
    ctx = FakeContext()
    ctx.config.process_directory = None
    plugin = ProcessPlugin()
    plugin.setup(ctx)
    plugin._take(process)
    return plugin, ctx


def test_apply_places_the_build_on_the_same_movement(purged):
    """Apply: the plugin serves the placed process, the authored one kept, the same movement selected and redrawn."""
    plugin, ctx = _set_up(purged)
    plugin.select(30)
    selected = plugin._authored_index[plugin.index]
    plugin.apply_placement(ctx, ON_FLOOR)
    assert plugin.authored is purged and plugin.process is not purged and plugin.placement == ON_FLOOR
    assert plugin._authored_index[plugin.index] == selected and plugin.selection.process is plugin.process
    assert "build placed at 1.500, -0.700, 0.000 m, 0.0, 0.0, 37.0°; replan" in ctx.lines("info")
    assert "placed 1.500, -0.700" in plugin._placement_html(ctx) and "off the floor" not in plugin._placement_html(ctx)
    plugin.apply_placement(ctx, PLACEMENT)
    assert "off the floor" in plugin._placement_html(ctx)


def test_apply_asks_twice_once_an_action_is_done_and_the_run_may_place_it(purged):
    """A done mark refuses one Apply (the box goes back), the second places it; the run's request places it too."""
    plugin, ctx = _set_up(purged)
    plugin.request_marks({purged.schedule[0]: MARK_DONE})
    plugin.apply_placement(ctx, ON_FLOOR)
    assert plugin.placement == IDENTITY and plugin.process is purged and plugin._pose.pose == IDENTITY
    assert "not placed" in plugin._placement_html(ctx)
    assert "press Apply again" in ctx.lines("warn")[-1]
    ctx.t += 0.5
    plugin.apply_placement(ctx, PLACEMENT)
    assert plugin.placement == PLACEMENT and "placed anyway" in ctx.lines("warn")[-1]
    plugin.request_placement(ON_FLOOR)
    assert plugin.placement == ON_FLOOR and plugin.process is not purged


def test_apply_is_refused_while_a_step_runs_and_a_new_load_is_as_authored(purged):
    """A step holding a robot's lease refuses Apply; taking a process again puts it where it was authored."""
    plugin, ctx = _set_up(purged)
    slot = LeaseSlot("a200-0806")
    ctx.world.robots["a200-0806"] = SimpleNamespace(lease_slot=slot)
    lease = take_lease([slot], "movement_exec: exec B20_J_insert", owner="movement_exec")
    plugin.apply_placement(ctx, ON_FLOOR)
    assert plugin.placement == IDENTITY
    lease.release()
    plugin.apply_placement(ctx, ON_FLOOR)
    assert plugin.placement == ON_FLOOR
    plugin._take(purged)
    assert plugin.placement == IDENTITY and plugin.process is purged and plugin._pose.pose == IDENTITY
