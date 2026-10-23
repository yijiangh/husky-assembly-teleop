"""The clearance checker: a robot too close to a scene body is found on the worker; one in the open is not."""

from __future__ import annotations

import time

import pytest
from repo_paths import DATA

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.world.clearance import ALL_ARMS, STOP_DISTANCE, ClearanceChecker, assumable_joints
from scaffolding_assembly_core.geometry import Pose, box_geometry
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Body, Scene, world_poses

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


def found(checker: ClearanceChecker, scene: Scene, own: str, assumable: frozenset = frozenset()) -> list:
    """Check `own` in `scene`, these joints allowed to be unknown, and wait for the answer."""
    assert checker.check(scene, [("drive", own, STOP_DISTANCE, assumable)])
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
        assert checker.check(snapshot(config, -2.0), [("drive", own, STOP_DISTANCE, frozenset())])
        assert not checker.check(snapshot(config, -2.0), [("drive", own, STOP_DISTANCE, frozenset())])
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
        assert why.startswith("cannot be checked: joints not known: ur_arm_") and hit is None
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


def test_another_robot_that_lost_mocap_is_checked_where_last_seen():
    """Belle's base is not tracked: she still stands where last seen, so Alice driving up to her trips."""
    alice, belle = CONFIGS
    own, other = robot_id(alice.serial), robot_id(belle.serial)

    def scene(x: float) -> Scene:
        robots = {own: RobotObject(own, alice.model, Pose((x, -3.0, 0.0)), {}),
                  other: RobotObject(other, belle.model, Pose((0.0, -3.0, 0.0)), {}, base_tracked=False,
                                     base_time=1.0)}
        return Scene(robots=robots, world_poses={})

    checker = ClearanceChecker()
    try:
        assert found(checker, scene(-3.0), own) == []
        (_key, why, hit), = found(checker, scene(-0.8), own)
        assert why == f"came within 10 cm of {other}" and hit is not None
    finally:
        checker.close()
