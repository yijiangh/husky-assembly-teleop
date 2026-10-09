"""Tests for scenes from designs: scene_at, scene_after, id maps with retarget, and hold scenes."""

from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from design_fixtures import build_design

from bar_assembly_core.geometry import Pose, compose, invert
from bar_assembly_core.design.hold import closing_movement, hold_scenes, release_movement
from bar_assembly_core.design.scenes import seeded
from bar_assembly_core.ids import IdMap
from bar_assembly_core.scene import retarget
from bar_assembly_core.kinematics import link_pose
from bar_assembly_core.scene import Attachment


@pytest.fixture
def design(tmp_path):
    """The fixture design: Cindy builds B1 and B2; Alice holds B1 until B2 is built."""
    return build_design(tmp_path)


def _movement(design, movement_id):
    """One movement of the design, by id."""
    return next(movement for _, movement in design.movements() if movement.id == movement_id)


def test_scene_at_robots_and_held_body(design):
    """An absent robot is disabled, unknown joints are unmeasured; an attached bar and its halves follow its link."""
    scene = design.scene_at(_movement(design, "B1_M0_mount"))
    alice, cindy = scene.robots["robots/alice"], scene.robots["robots/cindy"]
    assert not alice.enabled and cindy.enabled
    assert cindy.unmeasured == frozenset(cindy.model.movable_joints) and cindy.acting_problems()
    assert not scene.bodies["bars/B1"].enabled and not scene.bodies["joints/G1_ground"].enabled

    movement = _movement(design, "B1_M1_grasp")
    scene = design.scene_at(movement)
    cindy = scene.robots["robots/cindy"]
    grasp = movement.start.attached["bars/B1"][0].grasp
    assert scene.bodies["bars/B1"].placement == Attachment("robots/cindy", "left_tool0", grasp)
    offset = compose(invert(design.bodies["bars/B1"].pose), design.bodies["joints/G1_ground"].pose)
    on_flange = Attachment("robots/cindy", "left_tool0", compose(grasp, offset))
    assert scene.bodies["joints/G1_ground"].placement == on_flange
    tool0 = link_pose(cindy.model.urdf, cindy.base, cindy.joints, "left_tool0")
    np.testing.assert_allclose(scene.world_poses["bars/B1"].position, compose(tool0, grasp).position, atol=1e-9)
    # * Derived contacts: the tool with the half it is on, halves with their bar, the pending ground mate, wheels.
    assert set(scene.bodies["joints/G1_ground"].touches) == {"tools/AT3L", "bars/B1", "ground/WG0"}
    assert set(scene.bodies["bars/B1"].touches) == {"joints/G1_ground", "joints/J1_female", "tools/AT3L"}
    assert set(scene.bodies["ground/WG0"].touches) == {"joints/G1_ground", "robots/cindy/wheel_link"}
    assert "left_link2" in cindy.model.tool_touches["left_tool0"], "the tool's mount contacts are on the model"


def test_scene_at_built_and_staged_bars(design):
    """A built bar stands at its design pose while held; a staged bar at its written pose."""
    scene = design.scene_at(_movement(design, "B1_R_M0_ungrasp"))
    assert scene.bodies["bars/B1"].placement == design.bodies["bars/B1"].pose
    assert scene.bodies["bars/B2"].placement == Pose((2.0, 0.0, 0.1)) and scene.bodies["bars/B2"].enabled


def test_scene_at_unknown_base(design):
    """A base the design leaves open makes the robot refuse to act until its owner sets one."""
    cindy = design.scene_at(_movement(design, "B1_R_M1_retreat")).robots["robots/cindy"]
    assert not cindy.base_tracked and "its base is not tracked" in cindy.acting_problems()


def test_scene_shares_only_geometry_and_models(design):
    """Editing a scene never reaches the design or another scene; geometry and models are the same objects."""
    movement = _movement(design, "B1_M1_grasp")
    first, second = design.scene_at(movement), design.scene_at(movement)
    first.robots["robots/cindy"].joints["left_joint1"] = 9.0
    first.bodies["ground/WG0"].enabled = False
    assert movement.start.robots["robots/cindy"].joints["left_joint1"] != 9.0
    assert second.robots["robots/cindy"].joints["left_joint1"] != 9.0 and second.bodies["ground/WG0"].enabled
    assert first.bodies["bars/B2"] is not second.bodies["bars/B2"]
    assert first.bodies["bars/B2"].geometry is design.bodies["bars/B2"].geometry
    assert first.robots["robots/cindy"].model is second.robots["robots/cindy"].model
    # ? A new Design object with the same robots and tools keeps the same model: mirrors don't rebuild.
    edited = replace(design, schedule=design.schedule)
    assert edited.scene_at(movement).robots["robots/cindy"].model is first.robots["robots/cindy"].model


def test_scene_after_follows_the_schedule(design):
    """After B2: the next action's start. After B1 (its hold release ends the schedule): the last target's joints."""
    after_b2 = design.scene_after("bars/B2")
    assert after_b2.robots["robots/alice"].enabled and after_b2.bodies["bars/B2"].enabled
    assert after_b2.bodies["bars/B2"].placement == design.bodies["bars/B2"].pose
    after_b1 = design.scene_after("bars/B1")
    assert set(after_b1.robots["robots/alice"].joints.values()) == {0.9}
    with pytest.raises(KeyError):
        design.scene_after("bars/none")


def test_hold_scenes(design):
    """Alice's hold of B1 is solved where her grip closes (Cindy still holds B1) and checked where she lets go."""
    assert closing_movement(design, "B1_H_hold").id == "B1_H_M1_close"
    assert release_movement(design, "B1_H_hold").id == "B1_HR_M0_open"
    solve, release = hold_scenes(design, "B1_H_hold")
    assert solve.bodies["bars/B1"].enabled and solve.bodies["bars/B2"].placement == Pose((2.0, 0.0, 0.1))
    assert isinstance(solve.bodies["bars/B1"].placement, Pose)
    assert "tools/Grip" in solve.bodies["joints/J1_female"].touches
    assert isinstance(release.bodies["bars/B2"].placement, Pose) and release.bodies["bars/B2"].placement != Pose(
        (2.0, 0.0, 0.1))
    with pytest.raises(ValueError):
        closing_movement(design, "B1_J_joint")


def test_id_map_refuses_unmapped_ids():
    """A mapped robot maps its links; anything not listed is refused."""
    ids = IdMap({"robots/cindy": "robots/a200-0806", "bars/B1": "cell/bars/B1"})
    assert ids("robots/cindy/left_tool0") == "robots/a200-0806/left_tool0" and ids("bars/B1") == "cell/bars/B1"
    assert "robots/alice/left_tool0" not in ids
    with pytest.raises(KeyError):
        ids("robots/alice")


def test_retarget_swaps_the_robot_only(design):
    """The grasp and link carry over to the mapped robot; an unmapped robot or a missing link is refused."""
    scene = design.scene_at(_movement(design, "B1_M1_grasp"))
    model = scene.robots["robots/cindy"].model
    grasp = scene.bodies["bars/B1"].placement.offset
    held = {"bars/B1": scene.bodies["bars/B1"].placement, "t/on_body": Attachment("bars/B1", None, Pose())}
    moved = retarget(held, IdMap({"robots/cindy": "robots/real"}), {"robots/real": model})
    assert moved["bars/B1"] == Attachment("robots/real", "left_tool0", grasp)
    assert moved["t/on_body"] == held["t/on_body"]
    with pytest.raises(KeyError):
        retarget(held, IdMap({}), {})
    other = SimpleNamespace(links=frozenset({"base_link"}))  # ? only `links` is read
    with pytest.raises(ValueError, match="left_tool0"):
        retarget(held, IdMap({"robots/cindy": "robots/real"}), {"robots/real": other})


def test_seeded_fills_in_what_the_design_leaves_open(design):
    """Cindy's joints are null at her mount: seeded, she can be planned for; known joints keep their value."""
    scene = design.scene_at(_movement(design, "B1_M0_mount"))
    cindy = scene.robots["robots/cindy"]
    assert cindy.acting_problems()
    seeded(scene, "robots/cindy", {"left_joint1": 0.4})
    assert not cindy.acting_problems() and cindy.joints["left_joint1"] == 0.4 and cindy.joints["left_joint2"] == 0.0
    scene = design.scene_at(_movement(design, "B1_M1_grasp"))
    before = dict(scene.robots["robots/cindy"].joints)
    seeded(scene, "robots/cindy", {"left_joint1": 0.4})
    assert scene.robots["robots/cindy"].joints == before
