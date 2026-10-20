"""Tests for shared.assembly_planning.scene_for: a process step's planning scene on the real robots."""

from __future__ import annotations

from dataclasses import replace

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, movement, purged_process, robot_config, step

from husky_assembly_teleop.shared.assembly_planning.scene_for import RobotBelief, scene_for
from husky_assembly_teleop.shared.process_scene.scene_bodies import scene_bodies
from husky_assembly_teleop.world.kinematics import stow_joints
from scaffolding_assembly_core.geometry import Pose, box_geometry, compose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.process import RobotState
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Attachment, Body, Scene

ALICE_ID, BELLE_ID, CINDY_ID = (robot_id(f"a200-{serial}") for serial in (ALICE, BELLE, CINDY))
B21 = "process/bars/B21"
#: Snapshot bodies without a world pose: held by a robot not in the scene, or fixed to a body that is missing.
GHOSTS = (Body("obstacles/held", box_geometry((0.1, 0.1, 0.1)), Attachment("robots/ghost", "tool0", Pose())),
          Body("cell/probe", box_geometry((0.1, 0.1, 0.1)), Attachment("tracked/missing", None, Pose())))


@pytest.fixture(scope="module")
def purged():
    """The purged process."""
    return purged_process()


def _measured(loaded, movement_id: str, serial: str, planned: str) -> RobotObject:
    """A measured real robot where the plan has its planned robot at a movement's start (assumed joints if null)."""
    state = movement(loaded, movement_id)[1].start.robots[planned]
    config = robot_config(serial)
    return RobotObject(robot_id(config.serial), config.model, state.base,
                       dict(state.joints or step(loaded, movement_id).assumed_start))


def _unmeasured(serial: str) -> RobotObject:
    """A real robot whose base and joints are not measured."""
    config = robot_config(serial)
    return RobotObject(robot_id(config.serial), config.model, Pose(), {}, base_tracked=False,
                       unmeasured=frozenset(config.model.movable_joints))


def _snapshot(*robots: RobotObject) -> Scene:
    """A snapshot with these robots, an obstacle, a tracked object and a stale process body."""
    bodies = [Body("obstacles/boxes/A", box_geometry((0.2, 0.2, 0.2)), Pose((5.0, 5.0, 0.1))),
              Body("tracked/probe", box_geometry((0.01, 0.01, 0.01)), Pose((6.0, 5.0, 0.1))),
              Body(B21, box_geometry((0.01, 0.01, 0.01)), Pose((7.0, 5.0, 0.1)))]
    return Scene(bodies={body.id: body for body in bodies}, world_poses={body.id: body.placement for body in bodies},
                 robots={robot.id: robot for robot in robots})


def _alice_at(purged, movement_id: str, *serials: str, **options):
    """Alice acting at a movement: Alice measured, the other robots configured but not measured."""
    robots = (robot_config(ALICE), *(robot_config(serial) for serial in serials))
    snapshot = _snapshot(_measured(purged, movement_id, ALICE, "robots/alice"),
                         *(_unmeasured(serial) for serial in serials))
    return scene_for(purged, step(purged, movement_id), snapshot, robots, **options)


def _alice_close(purged, *serials: str, **options):
    """Alice's B21_H_close: Alice measured, the other robots not."""
    return _alice_at(purged, "B21_H_close", *serials, **options)


def _cindy_at(purged, movement_id: str, **options):
    """Cindy acting, measured where the plan has her at the start of a movement, nobody else configured."""
    snapshot = _snapshot(_measured(purged, movement_id, CINDY, "robots/cindy"))
    return scene_for(purged, step(purged, movement_id), snapshot, (robot_config(CINDY),), **options)


def _with_robot_state(current, planned: str, state: RobotState | None):
    """A step whose start state gives one planned robot another state."""
    start = current.movement.start
    return replace(current, movement=replace(current.movement, start=replace(
        start, robots={**start.robots, planned: state})))


def _cindy_with_ghosts(purged):
    """Cindy at B21_J_transfer, the snapshot also holding GHOSTS, kept by their prefixes."""
    snapshot = _snapshot(_measured(purged, "B21_J_transfer", CINDY, "robots/cindy"))
    snapshot.bodies.update((body.id, body) for body in GHOSTS)
    return scene_for(purged, step(purged, "B21_J_transfer"), snapshot, (robot_config(CINDY),),
                     keep=("obstacles/", "cell/"))


def test_held_bar_follows_the_real_robot(purged):
    """At B21_J_transfer Cindy holds B21: it is attached to the real Cindy's left flange, by the plan's grasp."""
    planning = _cindy_at(purged, "B21_J_transfer")
    grasp = movement(purged, "B21_J_transfer")[1].start.attached["bars/B21"][0].grasp
    assert planning.acting == CINDY_ID
    assert planning.scene.bodies[B21].placement == Attachment(CINDY_ID, "left_ur_arm_tool0", grasp)


def test_unmeasured_robot_stands_where_the_design_has_it(purged):
    """Alice's B21_H_close: Cindy, not measured, takes the design; B21 is built, and Cindy's tool0 holds it there."""
    planning = _alice_close(purged, CINDY)
    assert planning.sources == {CINDY_ID: "design"}
    b21 = purged.process.bodies["bars/B21"].assembly_pose
    assert planning.scene.bodies[B21].placement == b21
    cindy = planning.scene.robots[CINDY_ID]
    assert not cindy.acting_problems()
    grasp = movement(purged, "B21_H_close")[1].start.attached["bars/B21"][0]
    assert grasp.to == "robots/cindy/left_ur_arm_tool0"
    held = compose(link_pose(cindy.model.urdf, cindy.base, cindy.joints, "left_ur_arm_tool0"), grasp.grasp)
    assert np.allclose(held.position, b21.position, atol=1e-4)
    assert abs(abs(np.dot(held.orientation, b21.orientation)) - 1.0) < 1e-4


def test_a_belief_beats_the_design_and_no_design_parks(purged):
    """A belief places Cindy; Belle, not measured and absent from the design, is parked with a warning."""
    belief = Pose((1.0, 2.0, 0.0))
    joints = movement(purged, "B21_H_close")[1].start.robots["robots/cindy"].joints
    planning = _alice_close(purged, CINDY, BELLE, beliefs={CINDY_ID: RobotBelief(belief, {**joints})})
    assert planning.sources == {CINDY_ID: "belief", BELLE_ID: "parked"}
    assert planning.scene.robots[CINDY_ID].base == belief
    assert planning.scene.robots[BELLE_ID].acting_problems()
    assert any(BELLE_ID in warning and "parked" in warning for warning in planning.warnings)


def test_design_base_without_joints_stows_the_arms(purged):
    """A design that places Cindy but leaves her joints open: she stands there with her arms stowed."""
    base = movement(purged, "B21_H_close")[1].start.robots["robots/cindy"].base
    open_joints = _with_robot_state(step(purged, "B21_H_close"), "robots/cindy", RobotState(base, None))
    snapshot = _snapshot(_measured(purged, "B21_H_close", ALICE, "robots/alice"), _unmeasured(CINDY))
    planning = scene_for(purged, open_joints, snapshot, (robot_config(ALICE), robot_config(CINDY)))
    config, cindy = robot_config(CINDY), planning.scene.robots[CINDY_ID]
    assert planning.sources == {CINDY_ID: "design (stowed arms)"}
    assert cindy.base == base and cindy.joints == stow_joints(config, config.model.movable_joints)
    assert not cindy.acting_problems()


def test_bodies_held_by_a_parked_robot_are_disabled(purged):
    """Cindy holds B21 but is parked (not measured, no design base): B21 and its halves are disabled, with a warning."""
    transfer = _with_robot_state(step(purged, "B21_J_transfer"), "robots/cindy", RobotState(None, None))
    alice_acting = replace(step(purged, "B21_H_approach"), movement=transfer.movement)
    snapshot = _snapshot(_measured(purged, "B21_H_approach", ALICE, "robots/alice"), _unmeasured(CINDY))
    planning = scene_for(purged, alice_acting, snapshot, (robot_config(ALICE), robot_config(CINDY)))
    assert planning.sources == {CINDY_ID: "parked"}
    held = [f"process/{body_id}" for body_id in ("bars/B21", *purged.process.halves_of("bars/B21"))]
    assert all(planning.scene.bodies[body_id].placement.parent == CINDY_ID for body_id in held)
    assert not any(planning.scene.bodies[body_id].enabled for body_id in held)
    assert f"{B21} is held by parked {CINDY_ID}: disabled" in planning.warnings


def test_bodies_without_a_world_pose_are_removed(purged):
    """Kept snapshot bodies held by a missing robot or body have no world pose: removed, each with a warning."""
    planning = _cindy_with_ghosts(purged)
    assert not {body.id for body in GHOSTS} & set(planning.scene.bodies)
    assert {f"{body.id} has no world pose: removed" for body in GHOSTS} <= set(planning.warnings)
    assert set(planning.scene.world_poses) == set(planning.scene.bodies)


def test_unconfigured_robot_is_left_out_with_a_warning(purged):
    """Cindy stands in the design of Alice's step but is not configured: a warning names her."""
    planning = _alice_close(purged)
    assert planning.sources == {}
    assert any(warning.startswith("robots/cindy (serial 0806) is not configured") for warning in planning.warnings)


def test_acting_robot_must_be_configured_and_in_the_scene(purged):
    """Alice's step without Alice configured, or without her in the snapshot, is refused."""
    with pytest.raises(ValueError, match="robots/alice"):
        scene_for(purged, step(purged, "B21_H_close"), _snapshot(), (robot_config(CINDY),))
    with pytest.raises(ValueError, match="robots/alice"):
        scene_for(purged, step(purged, "B21_H_close"), _snapshot(), (robot_config(ALICE),))


def test_hide_held(purged):
    """hide_held disables B21 and its halves, which Cindy holds; nothing else."""
    plain = _cindy_at(purged, "B21_J_transfer").scene.bodies
    hidden = _cindy_at(purged, "B21_J_transfer", hide_held=True).scene.bodies
    turned_off = {body_id for body_id in plain if plain[body_id].enabled and not hidden[body_id].enabled}
    assert turned_off == {f"process/{body_id}" for body_id in ("bars/B21", *purged.process.halves_of("bars/B21"))}


def test_ignore_built(purged):
    """ignore_built at B22_J_insert disables B20, B21 and their halves; B22 and the ground stay."""
    plain = _cindy_at(purged, "B22_J_insert").scene.bodies
    ignored = _cindy_at(purged, "B22_J_insert", ignore_built=True).scene.bodies
    turned_off = {body_id for body_id in plain if plain[body_id].enabled and not ignored[body_id].enabled}
    process = purged.process
    assert turned_off == {f"process/{body_id}" for bar in ("bars/B20", "bars/B21")
                          for body_id in (bar, *process.halves_of(bar))}
    assert ignored["process/bars/B22"].enabled and ignored["process/ground/WG0"].enabled


@pytest.mark.parametrize("movement_id", ["B21_H_approach", "B21_H_to_grasp", "B21_R_retreat"])
def test_ignore_built_keeps_the_actions_own_bar(purged, movement_id):
    """Alice going for B21, and Cindy retreating from it: B21 is built but stays, with its halves; B20 goes."""
    planning = (_cindy_at if movement_id.startswith("B21_R") else _alice_at)(purged, movement_id, ignore_built=True)
    bodies = planning.scene.bodies
    assert all(bodies[f"process/{body_id}"].enabled for body_id in ("bars/B21", *purged.process.halves_of("bars/B21")))
    assert not bodies["process/bars/B20"].enabled


def test_end_contacts_let_the_gripper_touch_its_bar(purged):
    """Alice's B21_H_to_grasp ends on B21: with end_contacts B21 may touch her real gripper, without it may not.

    Only the bar the gripper ends on: the halves mounted on it still may not touch the gripper.
    """
    gripper = robot_config(ALICE).model.tools["ur_arm_tool0"].id
    bodies = _alice_at(purged, "B21_H_to_grasp", end_contacts=True).scene.bodies
    assert gripper in bodies[B21].touches
    assert gripper not in _alice_at(purged, "B21_H_to_grasp").scene.bodies[B21].touches
    halves = purged.process.halves_of("bars/B21")
    assert "joints/J21-22_mocap" in halves
    assert not any(gripper in bodies[f"process/{half}"].touches for half in halves)


def test_only_obstacles_come_from_the_snapshot(purged):
    """The snapshot's obstacle is kept; its tracked object and stale process body are not."""
    planning = _cindy_at(purged, "B21_J_transfer")
    bodies = planning.scene.bodies
    assert "obstacles/boxes/A" in bodies and "tracked/probe" not in bodies
    assert set(bodies) - {"obstacles/boxes/A"} == {f"process/{body_id}" for body_id in purged.process.bodies}
    assert set(planning.scene.world_poses) == set(bodies)


def test_geometry_is_the_plugins(purged):
    """Every process body has the very Geometry object the process plugin puts, so mirrors never rebuild it."""
    current = step(purged, "B21_J_transfer")
    plugin = {body.id: body.geometry
              for body in scene_bodies(purged.process, current.movement, (robot_config(CINDY),), "process/")}
    bodies = _cindy_at(purged, "B21_J_transfer").scene.bodies
    assert all(bodies[body_id].geometry is geometry for body_id, geometry in plugin.items())


@pytest.mark.slow
def test_cindy_mirror_syncs_the_planning_scene(purged):
    """A compas_fab mirror for Cindy takes the planning scene of her transfer, the bodies without a pose removed."""
    planning = _cindy_with_ghosts(purged)
    mirror = CompasFabMirror(CINDY_ID)
    try:
        mirror.sync(planning.scene)
        assert mirror.state is not None
    finally:
        mirror.close()
