"""Tests for the core's robots and scenes: RobotModel, RobotObject validity, world poses, forward kinematics."""

from __future__ import annotations

import re
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from yourdfpy import URDF

from repo_paths import DATA
from scaffolding_assembly_core.geometry import box_geometry
from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.robot import RobotObject, robot_model
from scaffolding_assembly_core.scene import Attachment, Body, Scene, world_poses
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES
from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.world.kinematics import Kinematics
from husky_assembly_teleop.world.scene import take_snapshot, tracked_body

#: Robot -> (serial, the joints old exports turned to rpy 0 0 -pi/2 (and the mount turned back to pi), the one
#: link that leaves at 90 degrees). How husky_urdf had them before 16 Oct 2026, and old process exports still do.
NOT_STOCK = {
    "cindy": ("0806", ("right_ur_arm_base_link-base_fixed_joint",), None, "right_ur_arm_base"),
    "alice": ("0804", ("ur_arm_base_link-base_link_inertia", "ur_arm_base_link-base_fixed_joint"),
              "arm_mount_joint", "ur_arm_base_link"),
}


def _turned_copy(urdf: Path, joints: tuple[str, ...], mount: str | None, out: Path) -> Path:
    """The URDF with the old turned UR base frames: `joints` at rpy 0 0 -pi/2, `mount` at 0 0 pi."""
    text = urdf.read_text()
    for name, rpy in [*((j, "0 0 -1.570796326795") for j in joints), *(((mount, "0 0 3.1415927"),) if mount else ())]:
        # ? Comments blanked first: the files keep old origins and whole transmission blocks commented out.
        live = re.sub(r"<!--.*?-->", lambda m: " " * len(m.group(0)), text, flags=re.S)
        start = live.index(f'<joint name="{name}"')
        tag = re.compile(r'rpy="[^"]*"').search(live, start, live.index("</joint>", start))
        text = text[:tag.start()] + f'rpy="{rpy}"' + text[tag.end():]
    out.write_text(text)
    return out


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
def test_frame_convention_and_fk_comparison(robot, tmp_path):
    """An old export's turned URDF is not stock, yet every link but one gives the same pose: compare by FK, not file."""
    serial, joints, mount, turned = NOT_STOCK[robot]
    live = robot_config_from_serial(serial, DATA).model
    other = robot_model(robot, _turned_copy(live.urdf, joints, mount, tmp_path / "turned.urdf"), live.srdf)
    assert live.stock_ur_frames and not other.stock_ur_frames
    assert other.links == live.links and other.flanges == live.flanges
    joints = dict(zip(live.movable_joints, np.random.default_rng(1).uniform(-2.0, 2.0, len(live.movable_joints))))
    differ = {}
    for link in sorted(live.links):
        a, b = link_pose(live.urdf, Pose(), joints, link), link_pose(other.urdf, Pose(), joints, link)
        angle = np.degrees(2 * np.arccos(min(1.0, abs(float(np.dot(a.orientation, b.orientation))))))
        if np.linalg.norm(np.subtract(a.position, b.position)) > 1e-6 or angle > 1e-4:
            differ[link] = angle
    assert list(differ) == [turned] and differ[turned] == pytest.approx(90.0, abs=1e-6)


def test_fk_matches_yourdfpy(cindy):
    """The core's numpy FK gives yourdfpy's link poses (the monitor's FK) for every link, mimic joints included."""
    urdf = URDF.load(str(cindy.model.urdf), load_meshes=False, build_collision_scene_graph=False,
                     load_collision_meshes=False)
    values = np.random.default_rng(2).uniform(-2.0, 2.0, len(urdf.actuated_joint_names))
    urdf.update_cfg(values)
    joints = dict(zip(urdf.actuated_joint_names, values))
    base = Pose((1.0, 2.0, 0.3), (0.0, 0.0, 0.6, 0.8))
    for link in sorted(cindy.model.links):
        live = compose(base, Pose.from_matrix(urdf.get_transform(frame_to=link, frame_from=urdf.base_link)))
        np.testing.assert_allclose(link_pose(cindy.model.urdf, base, joints, link).matrix(), live.matrix(), atol=1e-9)


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
    poses = world_poses(bodies, robots,
                        lambda robot, link: link_pose(robot.model.urdf, robot.base, robot.joints, link))
    tool0 = link_pose(cindy.model.urdf, Pose((1.0, 0.0, 0.0)), {}, "left_ur_arm_tool0")
    assert set(poses) == {"a", "b"}
    np.testing.assert_allclose(poses["a"].position, compose(tool0, grasp).position)
    np.testing.assert_allclose(poses["b"].position, compose(poses["a"], grasp).position)


def test_monitor_snapshot_has_robot_objects_and_tracked_bodies(cindy):
    """take_snapshot: RobotObjects by id with only movable joints unmeasured, tracked objects as bodies.

    A body held by a tracked object follows it.
    """
    kinematics = Kinematics((cindy,), log_warn=print)
    base = SimpleNamespace(mocap_id=None, state=SimpleNamespace(tracked=False, last_fix_time=None))
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


def test_lost_mocap_and_stale_joint_states_are_unknown(cindy):
    """A base whose mocap pose is lost is untracked, and an arm whose joint states went stale is unmeasured.

    Both keep their last value; the other arm, never measured, stays unmeasured too.
    """
    kinematics = Kinematics((cindy,), log_warn=print)
    left = {f"left_ur_arm_{name}": 0.5 for name in UR_JOINT_NAMES}
    kinematics.update(SimpleNamespace(robots={cindy.serial: SimpleNamespace(state=SimpleNamespace(
        base=SimpleNamespace(tracked=True, position=(1.0, 2.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0)),
        joint_positions=left))}))
    # * A valid sample at t = 10 s, captured then; the left arm reported then too.
    state = SimpleNamespace(tracked=True, tracking_valid=True, last_update_time=10.0, last_fix_time=10.0,
                            marker_errors=deque([(10.0, 0.001)]))
    arms = {"left_ur_arm": SimpleNamespace(state=SimpleNamespace(last_update_time=10.0)),
            "right_ur_arm": SimpleNamespace(state=SimpleNamespace(last_update_time=None))}
    world = SimpleNamespace(robots={cindy.serial: SimpleNamespace(
        config=cindy, base=SimpleNamespace(mocap_id=3, state=state), arms=arms)}, tracked_objects={})
    right = frozenset(name for name in cindy.model.movable_joints if name.startswith("right_ur_arm_"))

    fresh = take_snapshot(Scene(), world, kinematics, tick=1, time=10.1).robots["robots/a200-0806"]
    assert fresh.base_tracked and fresh.unmeasured == right

    # * The relay goes silent: `tracked` stays True, but the pose is lost and the joint states are old.
    old = take_snapshot(Scene(), world, kinematics, tick=2, time=15.0).robots["robots/a200-0806"]
    assert not old.base_tracked and old.base.position == pytest.approx((1.0, 2.0, 0.0)) and old.base_time == 10.0
    assert old.unmeasured == frozenset(cindy.model.movable_joints)
    assert all(old.joints[name] == 0.5 for name in left), "the last value is kept"
