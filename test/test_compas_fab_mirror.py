"""Tests for mirrors.compas_fab.CompasFabMirror: snapshots in, a compas_fab cell for one robot that follows them.

The last test compares against a design's own RobotCell (scene_refactor_plan.md §9, phase 5). It needs
a design folder: set HUSKY_DESIGN_DIRECTORY, e.g. to 260814_RobArch_support_ik; it is skipped otherwise.
"""

from __future__ import annotations

import os
from dataclasses import replace
from pathlib import Path

import numpy as np
import pybullet as p
import pytest

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.robot_interface.arm import UR_JOINT_NAMES
from bar_assembly_core.design_io.geometry import Geometry, box_geometry
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror
from bar_assembly_core.scene import Attachment, Body, RobotEntry, SceneSnapshot
from bar_assembly_core.design_io.pose import Pose

DATA = Path(__file__).resolve().parent.parent / "data"
ALICE, BELLE = "0804", "0805"


@pytest.fixture(scope="module")
def configs():
    """Alice's and Belle's stitched configs, loaded once."""
    return {ALICE: robot_config_from_serial(ALICE, DATA), BELLE: robot_config_from_serial(BELLE, DATA)}


@pytest.fixture(scope="module")
def mirror():
    """One mirror planning for Alice, shared by the tests: loading the models takes seconds."""
    mirror = CompasFabMirror(ALICE)
    yield mirror
    mirror.close()


def stow(config) -> dict[str, float]:
    """The arm's stow joints by name."""
    arm = config.arms[0]
    return {f"{arm.name}_{name}": value for name, value in zip(UR_JOINT_NAMES, arm.stow_joints)}


def world(configs, bodies=(), belle: Pose | None = None) -> SceneSnapshot:
    """Alice at the origin with the arm stowed, Belle at `belle` (default: far away), and some bodies.

    ? An attached body's world pose is not used by the mirror for Alice's own links; its grasp stands in.
    """
    robots = {ALICE: RobotEntry(configs[ALICE], Pose(), True, stow(configs[ALICE]), frozenset(), None, None),
              BELLE: RobotEntry(configs[BELLE], belle or Pose((10.0, 0.0, 0.0)), True, stow(configs[BELLE]),
                                frozenset(), None, None)}
    poses = {body.id: body.placement if isinstance(body.placement, Pose) else body.placement.grasp
             for body in bodies}
    return SceneSnapshot(bodies={body.id: body for body in bodies}, world_poses=poses, robots=robots)


def tool0(mirror: CompasFabMirror) -> Pose:
    """Where Alice's tool0 is in the mirror's world now."""
    client = mirror.client
    state = p.getLinkState(client.robot_puid, client.robot_link_puids["ur_arm_tool0"],
                           physicsClientId=client.client_id)
    return Pose.from_arrays(state[4], state[5])


def test_stowed_robot_is_clear(mirror, configs):
    """The stowed arm, its stitched gripper and the other robot far away: nothing collides."""
    mirror.sync(world(configs))
    assert mirror.collisions(full_report=True) == []


def test_box_at_the_tool_and_touches(mirror, configs):
    """A box at tool0 is reported with our ids; touching the robot allows it."""
    mirror.sync(world(configs))
    box = box_geometry((0.1, 0.1, 0.1))
    at_tool = tool0(mirror)
    mirror.sync(world(configs, (Body("t/box", box, at_tool),)))
    hits = mirror.collisions(full_report=True)
    assert hits and all(a.startswith("robots/0804/") and b == "t/box" for a, b in hits)

    mirror.sync(world(configs, (Body("t/box", box, at_tool, touches=("robots/0804",)),)))
    assert mirror.collisions() == []
    links = tuple(f"robots/0804/{a.split('/')[-1]}" for a, _ in hits)
    mirror.sync(world(configs, (Body("t/box", box, at_tool, touches=links),)))
    assert mirror.collisions() == []


def test_other_robot_is_an_obstacle(mirror, configs):
    """Belle overlapping Alice is reported as "robots/0805"."""
    mirror.sync(world(configs, belle=Pose((0.3, 0.0, 0.0))))
    hits = mirror.collisions(full_report=True)
    assert hits and {b for _, b in hits} == {"robots/0805"}


def test_stationary_contacts_are_allowed(mirror, configs):
    """A box inside Belle, far from Alice, is allowed for the snapshot and listed."""
    mirror.sync(world(configs, (Body("t/box", box_geometry((0.2, 0.2, 0.2)), Pose((10.0, 0.0, 0.3))),)))
    assert mirror.collisions() == []
    assert mirror.static_contacts == [("robots/0805", "t/box")]


def test_attached_body_moves_with_the_arm(mirror, configs):
    """A body on tool0 hits a box where tool0 is, and is clear of it once the arm moves away."""
    mirror.sync(world(configs))
    at_tool = tool0(mirror)
    held = Body("t/held", box_geometry((0.05, 0.05, 0.05)), Attachment("robots/0804", "ur_arm_tool0", Pose()),
                touches=("robots/0804",))
    # The box may touch the robot, so only the held body can hit it.
    box = Body("t/box", box_geometry((0.05, 0.05, 0.05)), at_tool, touches=("robots/0804",))
    mirror.sync(world(configs, (held, box)))
    assert mirror.collisions(full_report=True) == [("t/held", "t/box")]
    assert ("t/held", "t/box") not in mirror.collisions({"ur_arm_shoulder_lift_joint": -1.8}, full_report=True)


def test_rebuild_only_when_models_change(mirror, configs, monkeypatch):
    """A moved body only sets the state; a new geometry or robot config rebuilds the cell."""
    builds = []
    original = mirror.planner.set_robot_cell
    monkeypatch.setattr(mirror.planner, "set_robot_cell", lambda cell: builds.append(cell) or original(cell))
    box = box_geometry((0.2, 0.2, 0.2))
    mirror.sync(world(configs, (Body("t/box", box, Pose((5.0, 0.0, 0.3))),)))
    builds.clear()
    mirror.sync(world(configs, (Body("t/box", box, Pose((0.0, 0.0, 0.3))),)))
    assert builds == [] and mirror.collisions()
    away = Body("t/box", box_geometry((0.2, 0.2, 0.2)), Pose((5.0, 0.0, 0.3)))
    mirror.sync(world(configs, (away,)))
    assert len(builds) == 1 and mirror.collisions() == []
    mirror.sync(world(dict(configs, **{BELLE: replace(configs[BELLE])}), (away,)))
    assert len(builds) == 1, "an equal config is the same robot model"
    visual_only = Geometry(box.visual, ())
    mirror.sync(world(configs, (Body("t/ghost", visual_only, Pose((0.0, 0.0, 0.3))),)))
    assert mirror.collisions() == [], "a body without collision meshes never collides"


def test_disabled_body_is_hidden_without_rebuild(mirror, configs, monkeypatch):
    """Disabling and enabling a body only changes the state: it stops and starts colliding, no rebuild."""
    builds = []
    original = mirror.planner.set_robot_cell
    monkeypatch.setattr(mirror.planner, "set_robot_cell", lambda cell: builds.append(cell) or original(cell))
    box = box_geometry((0.2, 0.2, 0.2))
    mirror.sync(world(configs, (Body("t/box", box, Pose((0.0, 0.0, 0.3))),)))
    assert mirror.collisions()
    builds.clear()
    mirror.sync(world(configs, (Body("t/box", box, Pose((0.0, 0.0, 0.3)), enabled=False),)))
    assert mirror.collisions(full_report=True) == []
    mirror.sync(world(configs, (Body("t/box", box, Pose((0.0, 0.0, 0.3))),)))
    assert mirror.collisions() and builds == []


def test_search_check_agrees_with_check_collision(mirror, configs):
    """For random arm configurations near a box, Belle and a held body, the fast check and compas_fab agree."""
    arm = list(stow(configs[ALICE]))
    grasp = Attachment("robots/0804", "ur_arm_tool0", Pose((0.0, 0.0, 0.2)))
    held = Body("t/held", box_geometry((0.4, 0.05, 0.05)), grasp,
                touches=("robots/0804/ur_arm_robotiq_2f_85", "robots/0804/ur_arm_wrist_3_link"))
    box = Body("t/box", box_geometry((0.3, 0.3, 0.3)), Pose((0.8, 0.0, 0.8)))
    mirror.sync(world(configs, (held, box), belle=Pose((1.2, 0.8, 0.0))))
    assert mirror.collisions() == [], "the start must be clear: the fast check leaves static pairs out"
    check = mirror.search_check(arm)
    rng = np.random.default_rng(0)
    results = []
    for values in rng.uniform(-np.pi, np.pi, size=(60, len(arm))):
        fast = check(values)
        slow = mirror.collisions(dict(zip(arm, values)))
        assert (fast is None) == (slow == []), f"{values}: fast {fast}, compas_fab {slow}"
        results.append(fast is None)
    assert 5 < sum(results) < 55, "want both clear and colliding samples"


# --- --- --- --- --- AGAINST THE DESIGN'S OWN CELL --- --- --- --- ---

DESIGN = os.environ.get("HUSKY_DESIGN_DIRECTORY")
#: Design tool -> our stitched link or robot (Cindy's scaffolding tools, the other robots).
DESIGN_TOOLS = {"AT3L": "robots/0806/left_ur_arm_scaffolding_v3_left",
                "AT3R": "robots/0806/right_ur_arm_scaffolding_v3_right",
                "ObstacleRobotAlice": "robots/0804", "ObstacleRobotBelle": "robots/0805"}
#: Cindy's actions to compare, and one movement of each with an authored configuration.
DESIGN_ACTIONS = ("B3__J.json", "B5__J.json", "B12__J.json", "B20__J.json", "B23__J.json")


def _pose(frame) -> Pose:
    """One of our poses from a compas frame."""
    w, x, y, z = frame.quaternion
    return Pose.from_arrays(frame.point, (x, y, z, w))


def _design_snapshot(state, geometries, cindy) -> SceneSnapshot:
    """Our snapshot of a design state for Cindy: robots, visible bodies, attachments and touches."""
    from husky_assembly_teleop.config import robot_config_from_serial as config_for
    robots = {"0806": RobotEntry(cindy, _pose(state.robot_base_frame), True,
                                 dict(zip(state.robot_configuration.joint_names,
                                          state.robot_configuration.joint_values)), frozenset(), None, None)}
    for name, serial in (("ObstacleRobotAlice", ALICE), ("ObstacleRobotBelle", BELLE)):
        tool = state.tool_states[name]
        joints = dict(zip(tool.configuration.joint_names, tool.configuration.joint_values))
        robots[serial] = RobotEntry(config_for(serial, DATA), _pose(tool.frame), True, joints, frozenset(), None, None)
    bodies, poses = {}, {}
    for name, body in state.rigid_body_states.items():
        if body.is_hidden:
            continue
        touches = tuple(DESIGN_TOOLS.get(other, other) for other in body.touch_bodies) + \
            tuple(f"robots/0806/{link}" for link in body.touch_links)
        if body.attached_to_link:
            placement = Attachment("robots/0806", body.attached_to_link, _pose(body.attachment_frame))
        else:
            placement = _pose(body.frame)
        bodies[name] = Body(name, geometries[name], placement, touches)
        poses[name] = placement if isinstance(placement, Pose) else placement.grasp
    return SceneSnapshot(bodies=bodies, world_poses=poses, robots=robots)


@pytest.mark.skipif(not DESIGN, reason="set HUSKY_DESIGN_DIRECTORY to a design folder")
def test_same_result_as_the_design_cell():
    """For authored states of Cindy, the mirror agrees with the design's own RobotCell: clear, or the same pairs hit."""
    from compas.data import json_load
    from compas.geometry import Translation
    from compas_fab.backends import CollisionCheckError, PyBulletClient, PyBulletPlanner
    import rs_data_structure  # noqa: F401  registers the Action types for json_load

    from bar_assembly_core.mirrors.compas_convert import filled

    folder = Path(DESIGN)
    design_cell = json_load(str(folder / "RobotCell.json"))
    geometries = {name: Geometry.from_rigid_body(body) for name, body in design_cell.rigid_body_models.items()}
    cindy = robot_config_from_serial("0806", DATA)
    states = []
    for file in DESIGN_ACTIONS:
        for movement in json_load(str(folder / "BarActions" / file)).movements:
            if movement.start_state.robot_configuration is not None:
                state = movement.start_state.copy()
                given = state.robot_configuration
                state.robot_configuration = filled(design_cell.zero_full_configuration(),
                                                   dict(zip(given.joint_names, given.joint_values)))
                states.append((movement.movement_id, state))
    assert states

    # * One more state per action: the held bar pushed 0.3 m down, into the robot or the structure.
    pushed = []
    for label, state in states[::2]:
        state = state.copy()
        held = next(name for name, body in state.rigid_body_states.items()
                    if body.attached_to_link and name.startswith("bar_"))
        frame = state.rigid_body_states[held].attachment_frame
        state.rigid_body_states[held].attachment_frame = frame.transformed(Translation.from_vector([0, 0, -0.3]))
        pushed.append((f"{label} pushed", state))

    with PyBulletClient("direct", verbose=False) as client:
        planner = PyBulletPlanner(client)
        planner.set_robot_cell(design_cell)
        expected = {}
        for label, state in states + pushed:
            try:
                planner.check_collision(state, {"full_report": True})
                expected[label] = set()
            except CollisionCheckError as error:
                expected[label] = {frozenset(map(lambda model: _design_id(design_cell, model), pair))
                                   for pair in error.collision_pairs}

    mirror = CompasFabMirror("0806")
    try:
        for label, state in states + pushed:
            mirror.sync(_design_snapshot(state, geometries, cindy))
            ours = set(map(frozenset, mirror.collisions(full_report=True)))
            theirs = expected[label]
            assert ours == theirs, f"{label}: design {_listed(theirs)}, mirror {_listed(ours)}"
            # * The search check agrees too, moving both arms from this state.
            arms = [name for name in state.robot_configuration.joint_names if "_ur_arm_" in name]
            fast = mirror.search_check(arms)([state.robot_configuration[name] for name in arms])
            assert (fast is None) == (not theirs), f"{label}: search check {fast}, design {_listed(theirs)}"
    finally:
        mirror.close()
    assert any(expected[label] for label, _ in pushed), "no pushed state collides: the comparison shows nothing"


def _design_id(cell, model) -> str:
    """Our id of one side of a design collision pair: a link of Cindy, a tool or a rigid body."""
    for name, value in [*cell.tool_models.items(), *cell.rigid_body_models.items()]:
        if value is model:
            return DESIGN_TOOLS.get(name, name)
    return f"robots/0806/{model.name}"


def _listed(pairs: set[frozenset]) -> list[tuple[str, ...]]:
    """Pairs as sorted tuples, for a readable failure message."""
    return sorted(tuple(sorted(pair)) for pair in pairs)
