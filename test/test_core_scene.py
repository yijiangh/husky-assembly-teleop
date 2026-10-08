"""Tests for the core's robots and scenes: RobotModel, RobotObject validity, world poses, forward kinematics."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from bar_assembly_core.geometry import box_geometry
from bar_assembly_core.geometry import Pose, compose
from bar_assembly_core.kinematics import ForwardKinematics
from bar_assembly_core.robot import RobotObject, robot_model
from bar_assembly_core.scene import Attachment, Body, Scene, world_poses
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.world.kinematics import Kinematics
from husky_assembly_teleop.world.scene import take_snapshot, tracked_body

DATA = Path(__file__).resolve().parent.parent / "data"
#: Robot -> (serial, the design copy's URDF variant, the one link it turns 90 degrees against the stock UR frames).
NOT_STOCK = {
    "cindy": ("0806", "mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf",
              "right_ur_arm_base"),
    "alice": ("0804", "mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Alice_Calibrated.urdf",
              "ur_arm_base_link"),
}


@pytest.fixture(scope="module")
def cindy():
    """Cindy's config, with its live model."""
    return robot_config_from_serial("0806", DATA)


def test_live_model_keeps_tools_apart(cindy):
    """The live model's URDF has no tool links; each tool is a Tool that may touch its arm's wrist links."""
    model = cindy.model
    assert model.flanges == ("left_ur_arm_tool0", "right_ur_arm_tool0") and model.stock_ur_frames
    assert not any("scaffolding" in link for link in model.links)
    assert model.tools["left_ur_arm_tool0"].id == "tools/a200-0806/left_ur_arm"
    assert model.tool_touches["left_ur_arm_tool0"][:2] == ("left_ur_arm_wrist_2_link", "left_ur_arm_wrist_3_link")
    assert len(model.movable_joints) == 12, "both arms; the wheels are SRDF-passive"


@pytest.mark.parametrize("robot", sorted(NOT_STOCK))
def test_frame_convention_and_fk_comparison(robot):
    """The design's URDF variant is not stock, yet every link but one gives the same pose: compare by FK, not file."""
    serial, urdf, turned = NOT_STOCK[robot]
    live = robot_config_from_serial(serial, DATA).model
    other = robot_model(robot, DATA / "husky_urdf" / urdf, live.srdf)
    assert live.stock_ur_frames and not other.stock_ur_frames
    assert other.links == live.links and other.flanges == live.flanges
    fk = ForwardKinematics()
    joints = dict(zip(live.movable_joints, np.random.default_rng(1).uniform(-2.0, 2.0, len(live.movable_joints))))
    differ = {}
    for link in sorted(live.links):
        a, b = fk.link_pose(live.urdf, Pose(), joints, link), fk.link_pose(other.urdf, Pose(), joints, link)
        angle = np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(a.orientation, b.orientation))))))
        if np.linalg.norm(np.subtract(a.position, b.position)) > 1e-6 or angle > 1e-4:
            differ[link] = angle
    assert list(differ) == [turned] and differ[turned] == pytest.approx(90.0, abs=1e-6)


def test_fk_matches_live_kinematics(cindy):
    """The core FK on the model's URDF gives the monitor's link poses for the same base and joints."""
    kinematics = Kinematics((cindy,), log_warn=print)
    joints = dict(kinematics.joints(cindy.serial))
    base = kinematics.base_pose(cindy.serial)
    fk = ForwardKinematics()
    for link in ("left_ur_arm_tool0", "right_ur_arm_wrist_3_link", "base_link"):
        ours, live = fk.link_pose(cindy.model.urdf, base, joints, link), kinematics.link_pose(cindy.serial, link)
        np.testing.assert_allclose(ours.position, live.position, atol=1e-9)


def test_acting_problems(cindy):
    """A robot can act only when present, tracked and with every movable joint known."""
    robot = RobotObject("robots/a200-0806", cindy.model, Pose(), {})
    assert robot.acting_problems() == []
    robot.base_tracked, robot.unmeasured, robot.enabled = False, frozenset({"left_ur_arm_elbow_joint"}), False
    problems = robot.acting_problems()
    assert len(problems) == 3 and "left_ur_arm_elbow_joint" in problems[2]
    copy = robot.copy()
    copy.joints["x"] = 1.0
    assert robot.joints == {} and copy.model is robot.model


def test_world_poses_follow_parents(cindy):
    """Held by a link: forward kinematics; fixed to a body: that body; a missing parent leaves the body out."""
    robots = {"robots/r": RobotObject("robots/r", cindy.model, Pose((1.0, 0.0, 0.0)), {})}
    grasp = Pose((0.0, 0.0, 0.1))
    bodies = {"a": Body("a", box_geometry((0.1, 0.1, 0.1)), Attachment("robots/r", "left_ur_arm_tool0", grasp)),
              "b": Body("b", box_geometry((0.1, 0.1, 0.1)), Attachment("a", None, grasp)),
              "c": Body("c", box_geometry((0.1, 0.1, 0.1)), Attachment("robots/missing", None, grasp))}
    fk = ForwardKinematics()
    poses = world_poses(bodies, robots,
                        lambda robot, link: fk.link_pose(robot.model.urdf, robot.base, robot.joints, link))
    tool0 = fk.link_pose(cindy.model.urdf, Pose((1.0, 0.0, 0.0)), {}, "left_ur_arm_tool0")
    assert set(poses) == {"a", "b"}
    np.testing.assert_allclose(poses["a"].position, compose(tool0, grasp).position)
    np.testing.assert_allclose(poses["b"].position, compose(poses["a"], grasp).position)


def test_monitor_snapshot_has_robot_objects_and_tracked_bodies(cindy):
    """take_snapshot: RobotObjects by id with only movable joints unmeasured, tracked objects as bodies.

    A body held by a tracked object follows it.
    """
    kinematics = Kinematics((cindy,), log_warn=print)
    base = SimpleNamespace(state=SimpleNamespace(tracked=False, last_fix_time=None))
    robot = SimpleNamespace(config=cindy, base=base, arms={})
    probe = SimpleNamespace(position=np.array([1.0, 2.0, 3.0]), orientation=np.array([0.0, 0.0, 0.0, 1.0]),
                            tracked=True, last_fix_time=None)
    world = SimpleNamespace(robots={cindy.serial: robot}, tracked_objects={"probe": probe})
    scene = Scene()
    scene.put(tracked_body("probe", label="the probe"))
    scene.put(tracked_body("unseen"))
    scene.put(Body("t/tip", box_geometry((0.01, 0.01, 0.01)), Attachment("tracked/probe", None, Pose((0.0, 0.0, 0.1)))))
    snapshot = take_snapshot(scene, world, kinematics, tick=1, time=0.0)

    entry = snapshot.robots["robots/a200-0806"]
    assert entry.model is cindy.model and not entry.base_tracked
    assert entry.unmeasured == frozenset(cindy.model.movable_joints), "wheels are never counted as unmeasured"
    assert snapshot.bodies["tracked/probe"].label == "the probe"
    assert snapshot.world_poses["t/tip"].position == pytest.approx((1.0, 2.0, 3.1))
    assert snapshot.label("tracked/probe") == "the probe"
    assert not snapshot.bodies["tracked/unseen"].enabled, "no fix yet"
    # * The copy is not the live scene: editing one never reaches the other.
    snapshot.bodies["t/tip"].enabled = False
    assert scene.bodies["t/tip"].enabled and snapshot.bodies["t/tip"] is not scene.bodies["t/tip"]
