"""Teleop jog math (husky_world.jog_tool0_target) on the calibrated URDFs.

Each test loads the robot in a headless PyBullet, stands in for the monitor,
and steps the arm like FAKE_HARDWARE does: the joints jump to each target.
husky_world imports the ROS stack, so these tests skip when it is not sourced.
"""

from types import SimpleNamespace

import numpy as np
import pybullet_planning as pp
import pytest

husky_world = pytest.importorskip('husky_assembly_teleop.husky_world')
from husky_assembly_teleop.robot_registry import robot_by_name  # noqa: E402

# A comfortable arm pose away from singularities (pan, lift, elbow, w1, w2, w3).
START_CONF = np.array([0.0, -1.9, 1.9, -1.57, -1.57, 0.0])
# One monitor tick of fake-hardware travel.
TICK_S = 0.05


@pytest.fixture(params=['Cindy', 'Alice'])
def monitor(request):
    """A stand-in monitor: the robot's URDF as the ghost, live joints at START_CONF."""
    spec = robot_by_name(request.param)
    pp.connect(use_gui=False)
    try:
        ghost = pp.load_pybullet(spec.urdf_path, fixed_base=True)
        hi = SimpleNamespace(arm_joint_pose=[START_CONF.copy() for _ in range(spec.n_arms)])
        yield SimpleNamespace(
            huskies=[SimpleNamespace(interface=hi)], selected_robot_id=0,
            goal_model=SimpleNamespace(robot=ghost), _connected_robot=lambda: spec)
    finally:
        pp.disconnect()


def _jog(monitor, axis: int, speed: float, n_ticks: int, lead_s: float = TICK_S):
    """Hold a jog button for n_ticks fake ticks of lead_s each; return the start and end tool0 poses."""
    hi = monitor.huskies[0].interface
    start = husky_world._live_tool0_in_arm_base(monitor, 0)
    travelled = 0.0
    for _ in range(n_ticks):
        conf, travelled, problem = husky_world.jog_tool0_target(
            monitor, 0, start, axis, speed, lead_s, travelled)
        assert conf is not None, problem
        hi.arm_joint_pose[0] = conf
    return start, husky_world._live_tool0_in_arm_base(monitor, 0)


@pytest.mark.parametrize('axis', [0, 1, 2])
def test_move_is_a_straight_line_along_tool0(monitor, axis):
    """-10 mm/s for 1 s: tool0 goes 10 mm back along its own axis, orientation held."""
    start, end = _jog(monitor, axis, -0.01, 20)
    moved = husky_world._pose_difference(start, end)
    expected = np.zeros(6)
    expected[axis] = -0.01
    assert np.allclose(moved[:3], expected[:3], atol=2e-4)       # within 0.2 mm
    assert np.allclose(moved[3:], 0.0, atol=np.deg2rad(0.05))     # orientation kept


@pytest.mark.parametrize('axis', [3, 4, 5])
def test_turn_is_about_tool0_origin(monitor, axis):
    """2 deg/s for 1 s: tool0 turns 2 deg about its own axis, its origin stays put."""
    start, end = _jog(monitor, axis, np.deg2rad(2.0), 20)
    moved = husky_world._pose_difference(start, end)
    expected = np.zeros(3)
    expected[axis - 3] = np.deg2rad(2.0)
    assert np.allclose(moved[3:], expected, atol=np.deg2rad(0.05))
    assert np.allclose(moved[:3], 0.0, atol=2e-4)


def test_turn_counts_past_180_deg(monitor):
    """A wrist turn keeps going past 180 deg instead of reversing (rotation vector wrap)."""
    # 20 deg/s (under the 30 deg/s joint cap) in 0.25 s steps: 210 deg in 42 steps.
    start, end = _jog(monitor, 5, np.deg2rad(20.0), 42, lead_s=0.25)
    turned = husky_world._pose_difference(start, end)[5]
    # 210 deg reads back as -150 deg about +z.
    assert np.degrees(turned) == pytest.approx(-150.0, abs=0.5)


def test_jog_stops_at_a_joint_limit(monitor):
    """The jog refuses a target past a joint limit instead of sending it."""
    hi = monitor.huskies[0].interface
    hi.arm_joint_pose[0] = START_CONF.copy()
    hi.arm_joint_pose[0][5] = np.deg2rad(359.5)   # wrist 3 right at +360 deg
    start = husky_world._live_tool0_in_arm_base(monitor, 0)
    conf, _travelled, problem = husky_world.jog_tool0_target(
        monitor, 0, start, 5, np.deg2rad(10.0), 0.2)
    assert conf is None
    assert 'limit' in problem
