"""Grasp offsets: a hold's support grasp slid and rolled about its bar, its targets with it; the panel's grasp box."""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import purged_process
from plugin_fakes import FakeContext
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.plugins.process.grasp_offset import hold_of, offset_grasps
from husky_assembly_teleop.plugins.process.placement import placed
from husky_assembly_teleop.plugins.process.plugin import ProcessPlugin
from husky_assembly_teleop.robot_interface.robot_lease import LeaseSlot, take_lease
from husky_assembly_teleop.shared.assembly_planning.movement import classify
from husky_assembly_teleop.shared.process_contract import MARK_ASSUMED, MARK_DONE
from scaffolding_assembly_core.geometry import Pose, compose

FLANGE, BAR = "robots/alice/ur_arm_tool0", "bars/B21"


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process().process


def _grasp_and_bar(process):
    """B21_H's holder grasp at its close, its flange at the grasp, and the bar there."""
    hold = process.actions["B21_H"]
    grasp = next(holder.grasp for holder in hold.movements[3].target.attached[BAR] if holder.to == FLANGE)
    flange = hold.movements[2].target.link_poses[FLANGE]
    return grasp, flange, compose(flange, grasp)


def test_the_grasp_slides_along_the_bar_and_the_bar_stays(purged):
    """20 mm along the bar's z, rolled 30°: the bar is where it was, the flange 20 mm further along its axis."""
    moved = offset_grasps(purged, {"B21_H": (20.0, 30.0)})
    _grasp, flange, bar = _grasp_and_bar(purged)
    _new_grasp, new_flange, new_bar = _grasp_and_bar(moved)
    assert np.allclose(bar.matrix(), new_bar.matrix(), atol=1e-9)
    shift = np.asarray(new_flange.position) - np.asarray(flange.position)
    assert float(shift @ bar.matrix()[:3, 2]) == pytest.approx(0.020, abs=1e-9)


def test_the_hold_s_targets_lines_and_holders_move_from_approach_to_retreat(purged):
    """Approach and grasp lose their joints, both lines turn with the roll, the holder follows over the whole span."""
    moved = offset_grasps(purged, {"B21_H": (0.0, 90.0)})
    hold, release = moved.actions["B21_H"], moved.actions["B21_HR"]
    assert all(not movement.target.joints for movement in hold.movements[:3:2])
    for authored, new in ((purged.actions["B21_H"].movements[2], hold.movements[2]),
                          (purged.actions["B21_HR"].movements[1], release.movements[1])):
        before, after = np.asarray(authored.line[FLANGE].direction), np.asarray(new.line[FLANGE].direction)
        assert after @ before == pytest.approx(0.0, abs=1e-6), "turned 90° about a bar across the line"
        assert new.line[FLANGE].distance == authored.line[FLANGE].distance
    assert release.movements[2].target == purged.actions["B21_HR"].movements[2].target, "the leave stays"
    between = moved.actions["B22_J"].movements[0].start.attached[BAR]
    authored = purged.actions["B22_J"].movements[0].start.attached[BAR]
    assert [holder.grasp for holder in between if holder.to != FLANGE] == \
        [holder.grasp for holder in authored if holder.to != FLANGE]
    assert next(holder.grasp for holder in between if holder.to == FLANGE) != \
        next(holder.grasp for holder in authored if holder.to == FLANGE)
    first = purged.schedule.index("B21_H")
    assert all(moved.actions[each] is purged.actions[each] for each in purged.schedule[:first])


def test_a_moved_approach_still_solves_near_the_authored_grasp(purged):
    """Its joints are dropped, but its IK still stays near the authored joints at the grasp, not the arm's own."""
    moved = offset_grasps(purged, {"B21_H": (5.0, 3.0)})
    hold = moved.actions["B21_H"]
    approach = classify(moved, hold, hold.movements[0])
    assert approach.goal_joints is None
    assert approach.ik_reference == purged.actions["B21_H"].movements[2].target.joints["robots/alice"]


def test_no_offset_is_the_process_itself_and_a_release_names_its_hold(purged):
    """Zero, unknown or a jointing action: nothing moves; B21_HR belongs to B21_H, B21_J to no hold."""
    assert offset_grasps(purged, {}) is purged
    assert offset_grasps(purged, {"B21_H": (0.0, 0.0), "nope": (5.0, 0.0), "B21_J": (5.0, 0.0)}) is purged
    assert hold_of(purged, "B21_HR") == "B21_H" and hold_of(purged, "B21_H") == "B21_H"
    assert hold_of(purged, "B21_J") is None


def test_an_offset_on_a_placed_build_slides_along_the_placed_bar(purged):
    """Applied after the placement, the slide follows the bar where the build stands."""
    placement = Pose.from_arrays([1.0, 0.5, 0.0], Rotation.from_euler("z", 40.0, degrees=True).as_quat())
    build = placed(purged, placement)
    _grasp, flange, bar = _grasp_and_bar(build)
    _new_grasp, new_flange, _new_bar = _grasp_and_bar(offset_grasps(build, {"B21_H": (-15.0, 0.0)}))
    shift = np.asarray(new_flange.position) - np.asarray(flange.position)
    assert np.allclose(shift, -0.015 * bar.matrix()[:3, 2], atol=1e-9)


def _set_up(process) -> tuple[ProcessPlugin, FakeContext]:
    ctx = FakeContext()
    ctx.config.process_directory = None
    plugin = ProcessPlugin()
    plugin.setup(ctx)
    plugin._take(process)
    return plugin, ctx


def _select(plugin: ProcessPlugin, action_id: str) -> None:
    plugin.select(next(index for index, (action, _movement) in enumerate(plugin._movements) if action.id == action_id))


def test_the_grasp_box_moves_the_selected_hold_and_reset_puts_it_back(purged):
    """Grey off a hold; on B21_HR Apply moves B21_H's grasp in the served process, Reset serves the authored one."""
    plugin, ctx = _set_up(purged)
    _select(plugin, "B21_J")
    plugin.draw(ctx)
    assert plugin._slide.disabled and plugin._roll.disabled and not plugin._grasp_buttons.disabled
    plugin._on_grasp(ctx, "Apply")
    plugin.draw(ctx)
    assert plugin.process is purged and "not moved" in plugin._grasp_status.content
    _select(plugin, "B21_HR")
    plugin.draw(ctx)
    assert not plugin._slide.disabled and "B21_H as authored" in plugin._grasp_status.content
    axes = plugin._grasp_axes
    assert not any(frame.visible for frame in axes.values()), "as authored: nothing extra drawn"
    plugin._slide.value, plugin._roll.value = 20.0, -10.0
    plugin.draw(ctx)
    assert "not applied" in plugin._grasp_status.content
    gripper = axes["tools/alice/SupportGripper"]
    _grasp, flange, _bar = _grasp_and_bar(purged)
    shift = np.asarray(gripper.position) - np.asarray(flange.position)
    assert gripper.visible and np.linalg.norm(shift) > 0.019, "the gripper follows the sliders before Apply"
    plugin._on_grasp(ctx, "Apply")
    assert plugin.grasp_offsets == {}, "the release selected: the gripper may hold the bar, so it asks first"
    ctx.t += 0.5
    plugin._on_grasp(ctx, "Apply")
    assert plugin.grasp_offsets == {"B21_H": (20.0, -10.0)} and plugin.process is not purged
    assert plugin.selection.action.id == "B21_HR" and plugin.authored is purged
    assert "grasp of B21_H: slid 20 mm, rolled -10° from the authored one; replan" in ctx.lines("info")
    plugin.draw(ctx)
    assert "slid 20 mm, rolled -10°" in plugin._grasp_status.content and plugin._slide.value == 20.0
    ctx.t += 6.0
    plugin._on_grasp(ctx, "Reset")
    ctx.t += 0.5
    plugin._on_grasp(ctx, "Reset")
    assert plugin.grasp_offsets == {} and plugin.process is purged and plugin._slide.value == 0.0
    _select(plugin, "B21_J")
    plugin.draw(ctx)
    assert not any(frame.visible for frame in axes.values()), "hidden off a hold"


def test_the_run_s_offsets_apply_refused_while_a_step_runs_and_a_new_load_drops_them(purged):
    """The run's request moves the grasps; a step holding a lease refuses the panel's Apply; a new load: none."""
    plugin, ctx = _set_up(purged)
    plugin.request_grasp_offsets({"B21_H": (5.0, 0.0), "B35_H": (0.0, 0.0)})
    assert plugin.grasp_offsets == {"B21_H": (5.0, 0.0)} and plugin.process is not purged
    slot = LeaseSlot("a200-0806")
    ctx.world.robots["a200-0806"] = SimpleNamespace(lease_slot=slot)
    lease = take_lease([slot], "movement_exec: exec B21_J_insert", owner="movement_exec")
    plugin.apply_grasp_offset(ctx, "B21_H", 0.0, 0.0)
    assert plugin.grasp_offsets == {"B21_H": (5.0, 0.0)}
    lease.release()
    plugin._take(purged)
    assert plugin.grasp_offsets == {} and plugin.process is purged


@pytest.mark.parametrize("marks", [{"B21_H": MARK_DONE}, {"B21_H": MARK_ASSUMED}, {"B22_J": MARK_DONE}])
def test_a_hold_already_closed_moves_only_on_a_second_press(purged, marks):
    """Marked done, its gripper holds the bar as authored: Apply asks first; the second press moves it, warned."""
    plugin, ctx = _set_up(purged)
    plugin.request_marks(marks)
    plugin.apply_grasp_offset(ctx, "B21_H", 20.0, 30.0)
    assert plugin.grasp_offsets == {} and plugin.process is purged and "Press again" in plugin._grasp_refused
    ctx.t += 0.5
    plugin.apply_grasp_offset(ctx, "B21_H", 20.0, 30.0)
    assert plugin.grasp_offsets == {"B21_H": (20.0, 30.0)}
    assert any("already closed" in line for line in ctx.lines("warn"))


def test_a_finished_close_not_marked_yet_asks_and_another_offset_asks_again(purged):
    """The guide waits on the close for Done, unmarked; a second press counts only for the offset it asked about."""
    plugin, ctx = _set_up(purged)
    close = next(index for index, (action, movement) in enumerate(plugin._movements)
                 if action.id == "B21_H" and movement.id.endswith("_close"))
    plugin.select(close)
    plugin.apply_grasp_offset(ctx, "B21_H", 5.0, 0.0)
    ctx.t += 0.5
    plugin.apply_grasp_offset(ctx, "B21_H", -100.0, 120.0)
    assert plugin.grasp_offsets == {}, "another offset than the one asked about"
    ctx.t += 0.5
    plugin.apply_grasp_offset(ctx, "B21_H", -100.0, 120.0)
    assert plugin.grasp_offsets == {"B21_H": (-100.0, 120.0)}


def test_the_run_puts_its_offsets_back_on_a_closed_hold_without_asking(purged):
    """Continue restores what the run already built with: no second press."""
    plugin, _ctx = _set_up(purged)
    plugin.request_marks({"B21_H": MARK_DONE})
    plugin.request_grasp_offsets({"B21_H": (20.0, 30.0)})
    assert plugin.grasp_offsets == {"B21_H": (20.0, 30.0)}
