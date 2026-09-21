"""
Shared configuration loader for calibration pipeline.

This module provides common configuration and utility functions used across
all calibration scripts (0_circle_fitting.py through 3_verify_calibration.py).

Usage:
    from config_loader import load_config, HERE

    config = load_config('20250617')  # Pass the date folder name
    # or
    config = load_config()  # Uses DEFAULT_DATE_FOLDER
"""

import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))

# * The date folder is NOT chosen here anymore. Both calibration dates live in
# * husky_assembly_teleop/__init__.py so the live app and these offline scripts
# * can no longer drift apart. These scripts are launched directly
# * (e.g. "python 4_punch_validation.py"), which puts only this folder on the
# * import path, so add the repository root before importing the package.
_REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
if _REPO_ROOT not in sys.path:
    sys.path.append(_REPO_ROOT)

from husky_assembly_teleop import CALIBRATION_ANALYSIS_DATE

# Date folder every script reads/writes when no folder is passed explicitly.
# ! To switch datasets, edit CALIBRATION_ANALYSIS_DATE (and usually
# ! CALIBRATION_DATE next to it) in husky_assembly_teleop/__init__.py.
DEFAULT_DATE_FOLDER = CALIBRATION_ANALYSIS_DATE



def load_config(date_folder=None):
    """
    Load configuration from YAML file inside the specified date folder.
    
    Args:
        date_folder: Name of the date folder containing config.yaml.
                    If None, uses DEFAULT_DATE_FOLDER.
    
    Returns:
        dict: Configuration dictionary with 'date_folder' added.
    """
    if date_folder is None:
        date_folder = DEFAULT_DATE_FOLDER

    # Print the active folder so it is always obvious WHICH dataset a script
    # just analysed (the usual source of "why is this the old data?").
    print(f"[config_loader] Using calibration date folder: {date_folder}")

    config_file = os.path.join(HERE, date_folder, 'config.yaml')
    
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found: {config_file}")
    
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # Ensure date_folder is in config (override if present in yaml)
    config['date_folder'] = date_folder
    
    return config


def get_data_folder(date_folder=None):
    """Get the path to the data folder for the specified date."""
    if date_folder is None:
        date_folder = DEFAULT_DATE_FOLDER
    return os.path.join(HERE, date_folder)


def get_robot_urdf(robot_name):
    """Get the URDF path for the specified robot."""
    base_path = os.path.join(HERE, '..')
    
    if robot_name == '0806':
        urdf_path = os.path.join(
            base_path, 
            'husky_urdf', 'mt_husky_dual_ur5_e_moveit_config', 'urdf', 
            'husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf'
        )
    else:
        urdf_path = os.path.join(
            base_path,
            'husky_urdf', 'mt_husky_moveit_config', 'urdf', 
            'husky_ur5_e_no_base_joint_Alice_Calibrated.urdf'
        )
    
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    
    return urdf_path


def get_joint_names(robot_name, arm='left'):
    """Get joint names based on robot type."""
    if robot_name == '0806':
        return [
            f"{arm}_ur_arm_shoulder_pan_joint", 
            f"{arm}_ur_arm_shoulder_lift_joint",
            f"{arm}_ur_arm_elbow_joint", 
            f"{arm}_ur_arm_wrist_1_joint", 
            f"{arm}_ur_arm_wrist_2_joint", 
            f"{arm}_ur_arm_wrist_3_joint"
        ]
    else:
        return [
            "ur_arm_shoulder_pan_joint", 
            "ur_arm_shoulder_lift_joint",
            "ur_arm_elbow_joint", 
            "ur_arm_wrist_1_joint", 
            "ur_arm_wrist_2_joint", 
            "ur_arm_wrist_3_joint"
        ]


def get_tool0_link_name(robot_name, arm='left'):
    """Get tool0 link name based on robot type."""
    if robot_name == '0806':
        return f"{arm}_ur_arm_tool0"
    else:
        return "ur_arm_tool0"


def get_arm_base_link_name(robot_name, arm='left'):
    """Get the arm base link name for the specified robot."""
    if robot_name == '0806':
        return f"{arm}_ur_arm_base_link_inertia"
        # return f"{arm}_ur_arm_base_link"
    else:
        return "ur_arm_base_link_inertia"
        # return "ur_arm_base_link"


def get_shoulder_pan_joint_name(robot_name, arm='left'):
    """Get the shoulder pan joint name for parsing URDF base offset."""
    if robot_name == '0806':
        return f"{arm}_ur_arm_shoulder_pan_joint"
    else:
        return "ur_arm_shoulder_pan_joint"
