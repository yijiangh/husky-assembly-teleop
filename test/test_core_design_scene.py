"""Tests for scenes from designs: scene_at, scene_after, id maps with retarget, and hold scenes."""

from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest
from design_fixtures import build_design

from bar_assembly_core.geometry import Pose, compose
from bar_assembly_core.design.hold import hold_scene, hold_scene_for, release_bar
from bar_assembly_core.ids import IdMap
from bar_assembly_core.scene import retarget
from bar_assembly_core.kinematics import ForwardKinematics
from bar_assembly_core.scene import Attachment


@pytest.fixture
def design(tmp_path):
    """The fixture design: Cindy holds B1 in its first movement; Alice holds B2 until B1 is built."""
    return build_design(tmp_path)


def _movement(design, movement_id):
    """One movement of the design, by id."""
    return next(movement for _, movement in design.movements() if movement.id == movement_id)


def test_scene_at_robots_and_held_body(design):
    """An absent robot is disabled, unknown joints are unmeasured, a held body follows its link."""
    scene = design.scene_at(_movement(design, "B1_M0_free"))
    alice, cindy = scene.robots["robots/alice"], scene.robots["robots/cindy"]
    assert not alice.enabled and cindy.enabled
    assert cindy.unmeasured == frozenset(cindy.model.movable_joints) and cindy.acting_problems()
    assert scene.bodies["bars/B1"].placement == Attachment("robots/cindy", "left_tool0", Pose((0.0, 0.0, 0.12)))
    tool0 = ForwardKinematics().link_pose(cindy.model.urdf, cindy.base, {}, "left_tool0")
    np.testing.assert_allclose(scene.world_poses["bars/B1"].position,
                               compose(tool0, Pose((0.0, 0.0, 0.12))).position, atol=1e-9)
    assert not scene.bodies["bars/B2"].enabled and scene.bodies["joints/J1_male"].enabled
    # * Touches both ways: the state's (B1-AT3L) and the design's (J2 lists B1).
    assert {"tools/AT3L", "joints/J2_male"} <= set(scene.bodies["bars/B1"].touches)
    assert "left_link2" in cindy.model.tool_touches["left_tool0"], "the design tool's touches are on the model"


def test_scene_shares_only_geometry_and_models(design):
    """Editing a scene never reaches the design or another scene; geometry and models are the same objects."""
    movement = _movement(design, "B1_M1_tighten")
    first, second = design.scene_at(movement), design.scene_at(movement)
    first.robots["robots/cindy"].joints["left_joint1"] = 9.0
    first.bodies["bars/B2"].enabled = False
    assert movement.start.robots["robots/cindy"].joints["left_joint1"] != 9.0
    assert second.robots["robots/cindy"].joints["left_joint1"] != 9.0 and second.bodies["bars/B2"].enabled
    assert first.bodies["bars/B2"] is not second.bodies["bars/B2"]
    assert first.bodies["bars/B2"].geometry is design.bodies["bars/B2"].geometry
    assert first.robots["robots/cindy"].model is second.robots["robots/cindy"].model
    # ? A new Design object with the same robots and tools keeps the same model: mirrors don't rebuild.
    edited = replace(design, schedule=design.schedule)
    assert edited.scene_at(movement).robots["robots/cindy"].model is first.robots["robots/cindy"].model
    # * The placeholder pose is kept: disabling it is the scene owner's choice.
    assert first.bodies["bars/B1"].placement == design.bodies["bars/B1"].pose and second.bodies["bars/B1"].enabled


def test_scene_after_follows_the_schedule(design):
    """After B1: the next action's start. After B2, the last bar: its last movement with the target joints."""
    after_b1 = design.scene_after("bars/B1")
    assert after_b1.robots["robots/alice"].enabled and after_b1.bodies["bars/B2"].enabled
    after_b2 = design.scene_after("bars/B2")
    target = design.actions["B2_H_hold"].movements[-1].target.joints["robots/alice"]
    assert all(after_b2.robots["robots/alice"].joints[name] == value for name, value in target.items())
    with pytest.raises(KeyError):
        design.scene_after("bars/none")


def test_hold_scene(design):
    """Alice holds B2 until B1 is built: her hold scene is the world after B1, B2 disabled; nothing else changes."""
    assert release_bar(design, "B2_H_hold") == "bars/B1"
    held = hold_scene_for(design, "B2_H_hold")
    assert not held.bodies["bars/B2"].enabled and held.bodies["joints/J2_male"].enabled
    after = design.scene_after("bars/B1")
    assert hold_scene(after, "bars/B2").bodies["bars/B2"].enabled is False and after.bodies["bars/B2"].enabled
    with pytest.raises(ValueError):
        release_bar(design, "B1_J_joint")


def test_id_map_refuses_unmapped_ids():
    """A mapped robot maps its links; anything not listed is refused."""
    ids = IdMap({"robots/cindy": "robots/a200-0806", "bars/B1": "cell/bars/B1"})
    assert ids("robots/cindy/left_tool0") == "robots/a200-0806/left_tool0" and ids("bars/B1") == "cell/bars/B1"
    assert "robots/alice/left_tool0" not in ids
    with pytest.raises(KeyError):
        ids("robots/alice")


def test_retarget_swaps_the_robot_only(design):
    """The grasp and link carry over to the mapped robot; an unmapped robot or a missing link is refused."""
    scene = design.scene_at(_movement(design, "B1_M0_free"))
    model = scene.robots["robots/cindy"].model
    held = {"bars/B1": scene.bodies["bars/B1"].placement, "t/on_body": Attachment("bars/B1", None, Pose())}
    moved = retarget(held, IdMap({"robots/cindy": "robots/real"}), {"robots/real": model})
    assert moved["bars/B1"] == Attachment("robots/real", "left_tool0", Pose((0.0, 0.0, 0.12)))
    assert moved["t/on_body"] == held["t/on_body"]
    with pytest.raises(KeyError):
        retarget(held, IdMap({}), {})
    other = SimpleNamespace(links=frozenset({"base_link"}))  # ? only `links` is read
    with pytest.raises(ValueError, match="left_tool0"):
        retarget(held, IdMap({"robots/cindy": "robots/real"}), {"robots/real": other})
