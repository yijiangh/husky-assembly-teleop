"""Tests for shared.planning.collision: compas_fab's FK in the world frame, swept checks, hits in process terms."""

from __future__ import annotations

from math import cos, sin

import numpy as np
import pybullet as p
import pytest
from compas_fab.robots import TargetMode
from planning_fixtures import BELLE, CINDY, robot_config
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.shared.planning.collision import (SweptReport, collisions, in_process_terms, swept_check,
                                                             world_frame_fk)
from husky_assembly_teleop.shared.planning.joints import arm_joint_names
from husky_assembly_teleop.world.kinematics import stow_joints
from scaffolding_assembly_core.geometry import Pose, box_geometry
from scaffolding_assembly_core.ids import IdMap, robot_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.mirrors.compas import planning_group, pose_from_frame
from scaffolding_assembly_core.mirrors.compas_fab import CompasFabMirror
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Body, Scene

CINDY_ID, BELLE_ID = robot_id(f"a200-{CINDY}"), robot_id(f"a200-{BELLE}")
CINDY_LEFT_TOOL = "tools/a200-0806/left_ur_arm"
LEFT = arm_joint_names("left_ur_arm")
#: Base (2, 1, 0), yaw 0.7.
OFF_ORIGIN = Pose((2.0, 1.0, 0.0), (0.0, 0.0, sin(0.35), cos(0.35)))
#: Belle on the floor next to Cindy at the origin, her gripper's tip pushed 2 cm into Cindy's stowed left tool and
#: nothing else touching (found with `branches`).
BELLE_BASE = Pose((0.8026, 0.7413, 0.0), (0.0, 0.0, sin(-0.8251 / 2), cos(-0.8251 / 2)))
BELLE_JOINTS = (2.6562, -3.3941, 1.3807, -1.1312, 2.0589, -1.5733)


@pytest.fixture(scope="module")
def mirror():
    """One mirror planning for Cindy, shared by the tests."""
    mirror = CompasFabMirror(CINDY_ID)
    yield mirror
    mirror.close()


def _cindy(base: Pose = Pose()) -> RobotObject:
    """Cindy, arms stowed."""
    config = robot_config(CINDY)
    return RobotObject(CINDY_ID, config.model, base, stow_joints(config, config.model.movable_joints))


def _scene(*robots: RobotObject, bodies: tuple[Body, ...] = ()) -> Scene:
    """A scene of some robots and bodies with world poses."""
    return Scene(bodies={body.id: body for body in bodies}, world_poses={body.id: body.placement for body in bodies},
                 robots={robot.id: robot for robot in robots})


def _belle_at_cindys_tool(mirror: CompasFabMirror) -> None:
    """Sync Cindy stowed at the origin and Belle with her gripper in Cindy's left tool."""
    config = robot_config(BELLE)
    belle = RobotObject(BELLE_ID, config.model, BELLE_BASE, dict(zip(arm_joint_names("ur_arm"), BELLE_JOINTS)))
    mirror.sync(_scene(_cindy(), belle))


def test_in_process_terms():
    """Each side of a pair in process ids where mapped; robot links follow their robot; the rest stay."""
    process_ids = IdMap({CINDY_ID: "robots/cindy", CINDY_LEFT_TOOL: "tools/cindy/AT3L",
                         "process/bars/B21": "bars/B21"})
    assert in_process_terms((f"{CINDY_ID}/left_ur_arm_wrist_3_link", "process/bars/B21"), process_ids) == (
        "robots/cindy/left_ur_arm_wrist_3_link", "bars/B21")
    pair = in_process_terms((CINDY_LEFT_TOOL, "obstacles/boxes/A"), process_ids)
    assert pair == ("tools/cindy/AT3L", "obstacles/boxes/A")


@pytest.mark.slow
def test_world_frame_fk_matches_the_urdf(mirror):
    """Off the origin, the fixed forward kinematics give the URDF's tool0; compas_fab's own are metres off."""
    cindy = _cindy(OFF_ORIGIN)
    mirror.sync(_scene(cindy))
    group = planning_group(mirror.cell, "left_ur_arm_tool0")
    expected = link_pose(cindy.model.urdf, OFF_ORIGIN, cindy.joints, "left_ur_arm_tool0")
    with world_frame_fk(mirror.planner):
        fixed = pose_from_frame(mirror.planner.forward_kinematics(mirror.state, TargetMode.ROBOT, group))
    plain = pose_from_frame(mirror.planner.forward_kinematics(mirror.state, TargetMode.ROBOT, group))
    assert np.allclose(fixed.position, expected.position, atol=1e-6)
    turn = Rotation.from_quat(fixed.orientation).inv() * Rotation.from_quat(expected.orientation)
    assert turn.magnitude() < 1e-6
    assert np.linalg.norm(np.subtract(plain.position, expected.position)) > 0.5
    assert "forward_kinematics" not in vars(mirror.planner)


@pytest.mark.slow
def test_swept_check_finds_a_box_between_clear_waypoints(mirror):
    """A box where the left tool passes midway: both waypoints clear, both segments hit, every sample counted."""
    cindy = _cindy(OFF_ORIGIN)
    start = np.array([cindy.joints[name] for name in LEFT])
    end = start + np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    middle = link_pose(cindy.model.urdf, OFF_ORIGIN, {**cindy.joints, **dict(zip(LEFT, (start + end) / 2))},
                       "left_ur_arm_tool0")
    mirror.sync(_scene(cindy))
    assert swept_check(mirror, LEFT, [start, end]) == SweptReport(True, 199)

    mirror.sync(_scene(cindy, bodies=(Body("obstacles/box", box_geometry((0.04, 0.04, 0.04)), Pose(middle.position)),)))
    assert collisions(mirror, dict(zip(LEFT, start))) == [] and collisions(mirror, dict(zip(LEFT, end))) == []
    report = swept_check(mirror, LEFT, [start, end, start])
    assert not report.ok and report.samples == 2 * 199 and report.error == ""
    assert [index for index, _pairs in report.bad_segments] == [0, 1]
    assert all(any("obstacles/box" in pair for pair in pairs) for _index, pairs in report.bad_segments)


@pytest.mark.slow
def test_swept_check_fails_closed(mirror):
    """A check that raises (an unknown joint) is a failed check, never a clear path."""
    mirror.sync(_scene(_cindy()))
    report = swept_check(mirror, ["no_such_joint"], [[0.0], [1.0]])
    assert not report.ok and "no_such_joint" in report.error


@pytest.mark.slow
def test_swept_check_refuses_waypoints_of_the_wrong_width(mirror):
    """Waypoints with more or fewer values than joint names fail the check, naming the mismatch; none is checked."""
    mirror.sync(_scene(_cindy()))
    for path in ([[0.0] * 7, [0.1] * 7], [[0.0] * 5, [0.1] * 5]):
        report = swept_check(mirror, LEFT, path)
        assert (report.ok, report.samples) == (False, 0) and "6 values" in report.error, report


@pytest.mark.slow
def test_only_the_tools_overlap(mirror):
    """The scene of the next test: Belle's gripper is inside Cindy's left tool, and nothing else of hers touches Cindy.

    Checked in PyBullet itself: only Belle's gripper link touches Cindy's left tool, and nothing of Belle touches
    Cindy's links or right tool. ! Whatever `collisions` reports must then be about Cindy's left tool.
    """
    _belle_at_cindys_tool(mirror)
    client = mirror.client
    belle, world = client.tools_puids[BELLE_ID], client.client_id
    touching = p.getClosestPoints(client.tools_puids[CINDY_LEFT_TOOL], belle, 0.0, physicsClientId=world)
    # * Index 4 of a contact is the link of Belle's body; index 12 of its joint info, that link's name.
    assert touching and {p.getJointInfo(belle, point[4], physicsClientId=world)[12] for point in touching} == {
        b"ur_arm_tool0_tool"}
    assert not p.getClosestPoints(client.robot_puid, belle, 0.0, physicsClientId=world)
    assert not p.getClosestPoints(client.tools_puids["tools/a200-0806/right_ur_arm"], belle, 0.0,
                                  physicsClientId=world)
    assert all(CINDY_LEFT_TOOL in pair for pair in collisions(mirror, full_report=True))


@pytest.mark.slow
@pytest.mark.xfail(strict=True, reason="needs core CC.6 (M2 task C)")
def test_tool_against_another_robots_tool(mirror):
    """Belle's gripper in Cindy's left tool, nothing else touching: `collisions` names Cindy's tool against Belle."""
    _belle_at_cindys_tool(mirror)
    pairs = collisions(mirror, full_report=True)
    assert any(CINDY_LEFT_TOOL in pair and any(side.startswith((BELLE_ID, "tools/a200-0805")) for side in pair)
               for pair in pairs), pairs
