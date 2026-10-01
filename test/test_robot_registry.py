"""Checks for the robot registry (pure python: no ROS, no PyBullet, no data drive)."""
import os
import subprocess
import sys

import pytest

from husky_assembly_teleop.robot_registry import (
    ROBOTS, other_robots, robot_by_domain_id, robot_by_id, robot_by_name,
    robot_by_namespace, robot_by_serial, robot_from_env, robot_name_from_id,
)


# * ------------------------------------------------------------- the table

def test_table_matches_exporter_and_husky_world_names():
    """Names verbatim from the Rhino exporter; ids as in husky_world.init."""
    assert list(ROBOTS) == ['Cindy', 'Alice', 'Belle']
    cindy, alice, belle = ROBOTS['Cindy'], ROBOTS['Alice'], ROBOTS['Belle']

    assert (cindy.robot_id, alice.robot_id, belle.robot_id) == (
        'dual-arm_husky_Cindy', 'single-arm_husky_Alice', 'single-arm_husky_Belle')
    assert (cindy.namespace, cindy.domain_id, cindy.mocap_id) == ('/a200_0806', '86', 1860)
    assert (alice.namespace, alice.domain_id, alice.mocap_id) == ('/a200_0804', '84', 1840)
    assert (belle.namespace, belle.domain_id, belle.mocap_id) == ('/a200_0805', '85', 1850)
    assert [s.obstacle_tool_name for s in ROBOTS.values()] == [
        'ObstacleRobotCindy', 'ObstacleRobotAlice', 'ObstacleRobotBelle']
    assert [s.cell_file for s in ROBOTS.values()] == [
        'RobotCell.json', 'RobotCell_Alice.json', 'RobotCell_Belle.json']

    assert cindy.dual_arm and cindy.role == 'assembly' and cindy.n_arms == 2
    assert cindy.planning_groups == ('base_left_arm_manipulator', 'base_right_arm_manipulator')
    assert cindy.side_keys == ('left', 'right')
    assert cindy.tool_names == ('AT3L', 'AT3R') and cindy.gripper_kind == 'scaffolding'
    assert cindy.ee_types == ('assembly_tool_v3_left', 'assembly_tool_v3_right')
    assert cindy.connect_gripper is False and cindy.rb_prefix == ''

    for spec in (alice, belle):
        assert not spec.dual_arm and spec.role == 'support' and spec.n_arms == 1
        assert spec.planning_groups == ('manipulator',) and spec.side_keys == ('arm',)
        assert spec.tool_names == ('SupportGripper',) and spec.gripper_kind == 'robotiq'
        assert spec.ee_types == ('robotiq_gripper',) and spec.connect_gripper is True
        assert spec.rb_prefix == 'env_'


def test_joint_names_in_side_order():
    """Cindy lists the left arm first (as the exported configurations do)."""
    cindy, alice = ROBOTS['Cindy'], ROBOTS['Alice']
    assert len(cindy.all_arm_joint_names) == 12
    assert cindy.all_arm_joint_names[0] == 'left_ur_arm_shoulder_pan_joint'
    assert cindy.all_arm_joint_names[6] == 'right_ur_arm_shoulder_pan_joint'
    assert cindy.all_arm_joint_names[-1] == 'right_ur_arm_wrist_3_joint'
    assert alice.all_arm_joint_names == [
        'ur_arm_shoulder_pan_joint', 'ur_arm_shoulder_lift_joint', 'ur_arm_elbow_joint',
        'ur_arm_wrist_1_joint', 'ur_arm_wrist_2_joint', 'ur_arm_wrist_3_joint']


def test_registry_import_stays_light():
    """Importing the registry must not pull in pybullet, compas or ROS.

    Checked in a fresh interpreter, since other tests in this session may
    already have imported those modules.
    """
    code = ('import sys, husky_assembly_teleop.robot_registry; '
            'print(sorted(m for m in ("pybullet", "pybullet_planning", "compas", '
            '"compas_fab", "rclpy", "numpy") if m in sys.modules))')
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, check=True)
    assert out.stdout.strip().splitlines()[-1] == '[]'


# * ------------------------------------------------------------- lookups

def test_lookups():
    """Look a robot up by name, id, namespace, serial or domain id; unknown keys are KeyErrors."""
    assert robot_by_name('Belle').serial == '0805'
    assert robot_by_id('single-arm_husky_Alice').name == 'Alice'
    assert robot_by_namespace('/a200_0806').name == 'Cindy'
    assert robot_by_serial('0804').name == 'Alice'
    assert robot_by_domain_id(85).name == 'Belle'
    assert robot_by_domain_id('85').name == 'Belle'
    with pytest.raises(KeyError, match='Cindy'):   # message lists the valid names
        robot_by_name('Dora')
    with pytest.raises(KeyError):
        robot_by_domain_id(99)


def test_robot_from_env():
    """ROS_DOMAIN_ID picks the robot; missing or unknown falls back to the default (flagged True)."""
    assert robot_from_env(environ={'ROS_DOMAIN_ID': '85'}) == (ROBOTS['Belle'], False)
    assert robot_from_env(environ={'ROS_DOMAIN_ID': '86'}) == (ROBOTS['Cindy'], False)
    assert robot_from_env(environ={}) == (ROBOTS['Cindy'], True)
    assert robot_from_env(environ={'ROS_DOMAIN_ID': '99'}) == (ROBOTS['Cindy'], True)
    assert robot_from_env(default_domain_id='84', environ={}) == (ROBOTS['Alice'], True)


def test_other_robots_and_name_from_id():
    """The other robots come in registry order; a robot id maps back to its short name."""
    assert [s.name for s in other_robots('Cindy')] == ['Alice', 'Belle']
    assert [s.name for s in other_robots('Alice')] == ['Cindy', 'Belle']
    assert [s.name for s in other_robots('Belle')] == ['Cindy', 'Alice']
    assert robot_name_from_id('single-arm_husky_Alice') == 'Alice'
    assert robot_name_from_id('dual-arm_husky_Cindy') == 'Cindy'
    with pytest.raises(KeyError):
        robot_name_from_id('single-arm_husky_Dora')


# * ------------------------------------------------------------- per-side helpers

def test_side_and_link_helpers():
    """Links, planning groups and flanges map to the right arm side for each robot."""
    cindy, alice = ROBOTS['Cindy'], ROBOTS['Alice']
    assert cindy.side_of_link('left_ur_arm_tool0') == 'left'
    assert cindy.side_of_link('right_ur_arm_base_link') == 'right'
    assert alice.side_of_link('ur_arm_tool0') == 'arm'
    with pytest.raises(KeyError):
        alice.side_of_link('left_ur_arm_tool0')
    assert cindy.group_for_side('right') == 'base_right_arm_manipulator'
    assert cindy.flange_for_side('left') == 'left_ur_arm_tool0'
    assert alice.group_for_side('arm') == 'manipulator'
    assert alice.flange_for_side('arm') == 'ur_arm_tool0'
    assert alice.arm_base_links == ('ur_arm_base_link',)
    with pytest.raises(KeyError):
        cindy.group_for_side('arm')


def test_base_calibration_filename():
    """The base calibration file is named after the robot's serial, with an optional suffix."""
    assert ROBOTS['Cindy'].base_calibration_filename() == 'calibrated_transformation_0806.json'
    assert (ROBOTS['Alice'].base_calibration_filename('rhino')
            == 'calibrated_transformation_0804_rhino.json')


def test_robot_description_files_exist_and_differ():
    """Alice and Belle carry their own calibrated URDF / SRDF."""
    for spec in ROBOTS.values():
        assert os.path.isfile(spec.urdf_path), spec.urdf_path
        assert os.path.isfile(spec.srdf_path), spec.srdf_path
    assert ROBOTS['Alice'].urdf_path != ROBOTS['Belle'].urdf_path
    assert ROBOTS['Alice'].srdf_path != ROBOTS['Belle'].srdf_path
    assert 'Alice' in os.path.basename(ROBOTS['Alice'].urdf_path)
    assert 'Belle' in os.path.basename(ROBOTS['Belle'].urdf_path)
