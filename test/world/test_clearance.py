"""The clearance checker: a robot too close to a scene body is found on the worker; one in the open is not."""

from __future__ import annotations

import time

import pytest
from repo_paths import DATA

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.world.base_collision import BaseCollisionWorld
from husky_assembly_teleop.world.clearance import (ALL_ARMS, SELF_DISTANCE, STOP_DISTANCE, ClearanceChecker,
                                                   assumable_joints)
from scaffolding_assembly_core.geometry import Pose, box_geometry
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Attachment, Body, Scene, world_poses
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

pytestmark = [pytest.mark.safety, pytest.mark.slow]

#: Alice and Belle.
CONFIGS = tuple(robot_config_from_serial(serial, DATA) for serial in ("0804", "0805"))
#: A wall across the floor at x = 1 m.
WALL = Body("obstacles/wall", box_geometry((0.2, 4.0, 1.0)), Pose((1.0, 0.0, 0.5)), label="wall")


def snapshot(config, x: float, unmeasured: frozenset = frozenset()) -> Scene:
    """Alice at (x, 0) facing the wall, arm at zero (`unmeasured`: joints not known), and the wall."""
    own = robot_id(config.serial)
    robots = {own: RobotObject(own, config.model, Pose((x, 0.0, 0.0)), {}, unmeasured=unmeasured)}
    bodies = {WALL.id: WALL}
    return Scene(bodies=bodies, robots=robots, world_poses=world_poses(
        bodies, robots, lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link)))


def found(checker: ClearanceChecker, scene: Scene, own: str, assumable: frozenset = frozenset(),
          allowed: frozenset = frozenset()) -> list:
    """Check `own` in `scene`, these joints and other robots allowed to be unknown, and wait for the answer."""
    assert checker.check(scene, [("drive", own, STOP_DISTANCE, assumable, allowed)])
    for _ in range(600):
        answer = checker.finished()
        if answer is not None:
            return answer
        time.sleep(0.05)
    raise TimeoutError("the clearance check did not finish")


def test_a_robot_at_the_wall_is_too_close_and_one_in_the_open_is_not():
    """At the wall the check names it; two metres back it finds nothing; a busy checker skips a snapshot."""
    config = robot_config_from_serial("0804", DATA)
    own = robot_id(config.serial)
    checker = ClearanceChecker()
    try:
        assert found(checker, snapshot(config, -2.0), own) == []
        (key, why, hit), = found(checker, snapshot(config, 0.6), own)
        assert (key, why) == ("drive", f"came within 10 cm of {WALL.label}")
        assert hit.robot == own and hit.margin == STOP_DISTANCE and hit.pairs[0] == (own, WALL.id)
        # * The contact names the link that came close (the top plate reaches into the wall here), on the wall's face.
        contact = hit.contacts[0]
        assert contact.pair[0].startswith(f"{own}/") and contact.pair[1] == WALL.id
        assert contact.depth < STOP_DISTANCE and contact.point_b[0] == pytest.approx(0.9, abs=1e-3)
        assert hit.concave == frozenset() and hit.message.startswith("clearance: ")
        assert checker.check(snapshot(config, -2.0), [("drive", own, STOP_DISTANCE, frozenset(), frozenset())])
        assert not checker.check(snapshot(config, -2.0), [("drive", own, STOP_DISTANCE, frozenset(), frozenset())])
    finally:
        checker.close()


def test_an_unknown_arm_trips_a_strict_clearance_and_counts_as_stowed_in_a_lenient_one():
    """Arm joints never measured: strict says it cannot be checked; with the arm allowed, it is checked as it stands."""
    config = robot_config_from_serial("0804", DATA)
    own = robot_id(config.serial)
    unknown = frozenset(name for name in config.model.movable_joints if name.startswith("ur_arm_"))
    checker = ClearanceChecker()
    try:
        (_key, why, hit), = found(checker, snapshot(config, -2.0, unknown), own)
        assert why == "cannot be checked: ur_arm not measured" and hit is None
        allowed = assumable_joints(config.model.movable_joints, ["ur_arm"])
        assert allowed == unknown == assumable_joints(config.model.movable_joints, ALL_ARMS) & unknown
        assert found(checker, snapshot(config, -2.0, unknown), own, allowed) == []
        (_key, why, hit), = found(checker, snapshot(config, 0.6, unknown), own, allowed)
        assert why == f"came within 10 cm of {WALL.label}" and hit is not None
        # * Another arm's joints do not cover this one's.
        (_key, why, _hit), = found(checker, snapshot(config, -2.0, unknown), own,
                                   assumable_joints(config.model.movable_joints, ["left_ur_arm"]))
        assert why.startswith("cannot be checked")
    finally:
        checker.close()


def test_another_robot_that_lost_mocap_trips_unless_allowed_then_stands_where_last_seen():
    """Belle's base is not tracked: not allowed, Alice cannot be checked; allowed, Belle stands where last seen."""
    alice, belle = CONFIGS
    own, other = robot_id(alice.serial), robot_id(belle.serial)

    def scene(x: float) -> Scene:
        robots = {own: RobotObject(own, alice.model, Pose((x, -3.0, 0.0)), {}),
                  other: RobotObject(other, belle.model, Pose((0.0, -3.0, 0.0)), {}, base_tracked=False,
                                     base_time=1.0)}
        return Scene(robots=robots, world_poses={})

    checker = ClearanceChecker()
    try:
        (_key, why, hit), = found(checker, scene(-3.0), own)
        assert why == f"cannot be checked: {other.split('/')[1]}: base not measured" and hit is None
        assert found(checker, scene(-3.0), own, allowed=frozenset({other})) == []
        (_key, why, hit), = found(checker, scene(-0.8), own, allowed=frozenset({other}))
        assert why == f"came within 10 cm of {other}" and hit is not None
    finally:
        checker.close()


def stowed(config) -> dict[str, float]:
    """Every arm of a robot at its stow pose, by joint name."""
    return {f"{arm.name}_{name}": value for arm in config.arms for name, value in zip(UR_JOINT_NAMES, arm.stow_joints)}


def alone(config, joints: dict[str, float], bodies=()) -> Scene:
    """One robot at the origin with these joints, and bodies (some held), nothing else."""
    own = robot_id(config.serial)
    robots = {own: RobotObject(own, config.model, Pose(), joints)}
    bodies = {body.id: body for body in bodies}
    return Scene(bodies=bodies, robots=robots, world_poses=world_poses(
        bodies, robots, lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link)))


@pytest.mark.parametrize("serial", ["0804", "0805", "0806"])
def test_every_robot_stowed_is_clear_of_itself(serial):
    """Stowed, no link, tool or arm comes within SELF_DISTANCE of the same robot: no false hit on any robot."""
    config = robot_config_from_serial(serial, DATA)
    world = BaseCollisionWorld()
    try:
        world.sync(alone(config, stowed(config)))
        assert world.self_hit(robot_id(config.serial), SELF_DISTANCE, "stowed") is None
    finally:
        world.close()


def test_an_arm_folded_into_itself_and_a_bar_into_its_robot_are_found_and_named():
    """Elbow folded: arm links against each other; a 1 m bar held at tool0 when stowed: the bar against the rack."""
    config = robot_config_from_serial("0804", DATA)
    own = robot_id(config.serial)
    folded = {**stowed(config), "ur_arm_elbow_joint": 2.8}
    bar = Body("bars/b", box_geometry((0.03, 0.03, 1.0)), Attachment(own, "ur_arm_tool0", Pose((0.0, 0.0, 0.5))),
               label="the bar")
    checker = ClearanceChecker()
    try:
        (_key, why, hit), = found(checker, alone(config, folded), own)
        assert why == "came within 1 cm of itself: ur_arm_upper_arm_link with ur_arm_wrist_1_link"
        assert hit.robot == own and hit.contacts[0] is not None
        (_key, why, hit), = found(checker, alone(config, stowed(config), (bar,)), own)
        assert why.startswith("came within 1 cm of itself: the bar with ") and hit.pairs[0][0] == bar.id
        short = Body("bars/s", box_geometry((0.03, 0.03, 0.05)), Attachment(own, "ur_arm_tool0", Pose()),
                     label="a short bar")
        assert found(checker, alone(config, stowed(config), (short,)), own) == [], "held at the flange: allowed"
    finally:
        checker.close()
