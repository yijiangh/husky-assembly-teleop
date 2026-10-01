"""Tests for kinematics.Kinematics: measurements in, link poses out, checked against PyBullet."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pybullet as p
import pytest
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.config import robot_config_from_serial
from husky_assembly_teleop.world.kinematics import Kinematics
from husky_assembly_teleop.design_io.pose import Pose

DATA = Path(__file__).resolve().parent.parent / "data"


def fake_world(serial: str, position=(0.0, 0.0, 0.0), orientation=(0.0, 0.0, 0.0, 1.0),
               tracked: bool = True, joints: dict[str, float] | None = None) -> SimpleNamespace:
    """Build the smallest stand-in for a WorldState with one robot.

    ? Mirrors exactly what `update` reads: world.robots[serial].state.base.{position, orientation,
      tracked} and state.joint_positions, as on HuskyRobotInterface and RobotState.

    Args:
        serial: The robot's serial.
        position: Measured base position.
        orientation: Measured base quaternion (x, y, z, w).
        tracked: Whether the latest mocap sample was valid.
        joints: Measured joint values by name.

    Returns:
        SimpleNamespace: An object with `.robots[serial].state`.
    """
    base = SimpleNamespace(position=np.asarray(position, dtype=float),
                           orientation=np.asarray(orientation, dtype=float), tracked=tracked)
    state = SimpleNamespace(base=base, joint_positions=dict(joints or {}))
    return SimpleNamespace(robots={serial: SimpleNamespace(state=state)})


def angle_between(a: Pose, b: Pose) -> float:
    """Rotation angle between two poses' orientations, radians."""
    return (Rotation.from_quat(a.orientation).inv() * Rotation.from_quat(b.orientation)).magnitude()


@pytest.fixture(scope="module", params=["0804", "0806"])
def config(request):
    """A stitched robot config, single-arm (0804) and dual-arm (0806)."""
    return robot_config_from_serial(request.param, DATA)


def test_matches_pybullet(config):
    """Link world poses agree with PyBullet for random joints and a tilted, moved base."""
    kinematics = Kinematics((config,), log_warn=print)
    serial = config.serial
    names = kinematics.joint_names(serial)
    links = [f"{arm.name}_{suffix}" for arm in config.arms for suffix in ("tool0", "base_link_inertia")]

    client = p.connect(p.DIRECT)
    try:
        body = p.loadURDF(str(config.urdf_file), useFixedBase=False, physicsClientId=client)
        joint_index, link_index = {}, {}
        for i in range(p.getNumJoints(body, physicsClientId=client)):
            info = p.getJointInfo(body, i, physicsClientId=client)
            joint_index[info[1].decode("utf-8")] = i
            link_index[info[12].decode("utf-8")] = i

        rng = np.random.default_rng(0)
        for _ in range(5):
            position = rng.uniform(-3.0, 3.0, 3)
            orientation = Rotation.from_euler("xyz", rng.uniform(-np.pi, np.pi, 3)).as_quat()
            joints = {name: float(rng.uniform(-np.pi, np.pi)) for name in names}
            kinematics.update(fake_world(serial, position, orientation, joints=joints))

            p.resetBasePositionAndOrientation(body, position, orientation, physicsClientId=client)
            for name, value in joints.items():
                p.resetJointState(body, joint_index[name], value, physicsClientId=client)

            for link in links:
                # ? Indices 4/5 are the URDF link frame; 0/1 would be the inertial frame.
                state = p.getLinkState(body, link_index[link], computeForwardKinematics=True,
                                       physicsClientId=client)
                expected = Pose.from_arrays(state[4], state[5])
                actual = kinematics.link_pose(serial, link)
                assert np.allclose(actual.position, expected.position, atol=1e-6), link
                assert angle_between(actual, expected) < 1e-6, link
    finally:
        p.disconnect(client)


def test_starts_at_default_pose_with_joints_unmeasured(config):
    """Before any measurement: default base pose, every joint 0 and unmeasured."""
    kinematics = Kinematics((config,), log_warn=print)
    serial = config.serial
    assert kinematics.base_pose(serial) == Pose.from_arrays(config.default_position, config.default_orientation)
    assert set(kinematics.joints(serial).values()) == {0.0}
    assert kinematics.unmeasured(serial) == frozenset(kinematics.joint_names(serial))


def test_untracked_base_keeps_last_pose(config):
    """A sample marked untracked leaves the base where the last valid fix put it."""
    kinematics = Kinematics((config,), log_warn=print)
    serial = config.serial
    kinematics.update(fake_world(serial, position=(1.0, 2.0, 0.0)))
    kinematics.update(fake_world(serial, position=(9.0, 9.0, 9.0), tracked=False))
    assert kinematics.base_pose(serial).position == (1.0, 2.0, 0.0)


def test_absent_joint_keeps_last_value(config):
    """A joint missing from the next measurement keeps its value and stays measured."""
    kinematics = Kinematics((config,), log_warn=print)
    serial = config.serial
    first, second = kinematics.joint_names(serial)[:2]
    kinematics.update(fake_world(serial, joints={first: 0.5, second: -0.25}))
    kinematics.update(fake_world(serial, joints={second: 1.0}))
    assert kinematics.joints(serial)[first] == 0.5
    assert kinematics.joints(serial)[second] == 1.0
    assert first not in kinematics.unmeasured(serial) and second not in kinematics.unmeasured(serial)


def test_link_pose_follows_update(config):
    """A link pose asked for before an update is recomputed after it."""
    kinematics = Kinematics((config,), log_warn=print)
    serial = config.serial
    link = f"{config.arms[0].name}_tool0"
    before = kinematics.link_pose(serial, link)
    kinematics.update(fake_world(serial, position=(1.0, 0.0, 0.0)))
    after = kinematics.link_pose(serial, link)
    assert np.allclose(np.subtract(after.position, before.position), (1.0, 0.0, 0.0))


def test_unknown_joint_warns_once(config):
    """A joint the URDF lacks is reported once, not every tick, and nothing else changes."""
    warnings = []
    kinematics = Kinematics((config,), log_warn=warnings.append)
    serial = config.serial
    for _ in range(3):
        kinematics.update(fake_world(serial, joints={"no_such_joint": 1.0}))
    assert len(warnings) == 1 and "no_such_joint" in warnings[0]
    assert "no_such_joint" not in kinematics.joints(serial)


def test_unknown_link_and_serial_raise(config):
    """Asking for a link or robot that doesn't exist raises KeyError."""
    kinematics = Kinematics((config,), log_warn=print)
    with pytest.raises(KeyError, match="no_such_link"):
        kinematics.link_pose(config.serial, "no_such_link")
    with pytest.raises(KeyError, match="no_such_robot"):
        kinematics.base_pose("no_such_robot")
