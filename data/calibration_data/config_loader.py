"""
Shared configuration loader for calibration pipeline.

This module provides common configuration and utility functions used across
all calibration scripts (0_circle_fitting.py through 3_verify_calibration.py).

The dated dataset folders (<date>/config.yaml, j0/, j1/, ...) live on the
gdrive under CALIB_DATA_DIR, NOT next to these scripts. HERE is only this
scripts folder (used for sibling scripts and the repo-relative URDF path).

Usage:
    from config_loader import load_config, CALIB_DATA_DIR

    config = load_config('20250617')  # Pass the date folder name
    # or
    config = load_config()  # Uses DEFAULT_DATE_FOLDER
"""

import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))

# * These scripts are launched directly (e.g. "python 4_punch_validation.py"),
# * which puts only this folder on the import path, so add the repository root
# * before importing the package. Run them with the venv and ROS sourced.
_REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
if _REPO_ROOT not in sys.path:
    sys.path.append(_REPO_ROOT)

from crl_husky import config_resolver

from husky_assembly_teleop.drive import drive_root

# * The Drive's experiment folder (HUSKY_DRIVE_ROOT), or "" without a Drive.
EXPERIMENT_DATA_DIRECTORY = os.path.join(str(drive_root()), 'data_experiment') if drive_root() else ''

# * Root holding the dated calibration datasets: CALIBRATION_DATA_DIR if set
# * (scripts/compare_calibration.py points it at a copy), else the Drive's.
# * Build every dataset path as os.path.join(CALIB_DATA_DIR, <date>, ...), never from HERE.
CALIB_DATA_DIR = os.environ.get('CALIBRATION_DATA_DIR') or os.path.join(EXPERIMENT_DATA_DIRECTORY, 'calibration_data')

# Shared gdrive folder where the Rhino/Grasshopper visualisation inputs are staged.
RHINO_EXPORT_DIR = os.path.join(EXPERIMENT_DATA_DIRECTORY, 'visualise_calibration_to_rhino')

# Date folder every script reads/writes when no folder is passed explicitly:
# CALIBRATION_DATE if set, else the one below.
DEFAULT_DATE_FOLDER = os.environ.get('CALIBRATION_DATE') or '20261016'



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

    config_file = os.path.join(CALIB_DATA_DIR, date_folder, 'config.yaml')
    
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
    return os.path.join(CALIB_DATA_DIR, date_folder)


def get_robot_urdf(robot_name: str) -> str:
    """Get the URDF path for the specified robot.

    * Each husky has its own kinematically calibrated URDF (Alice and Belle
    * differ by up to ~0.5 mm per link): the one crl-husky's model.yaml names,
    * as the monitor uses.

    Args:
        robot_name (str): Serial number from config.yaml, e.g. '0805'.

    Returns:
        str: Path of the robot's URDF.
    """
    model = config_resolver.get_robot_model(robot_name)
    if model is None:
        raise ValueError(f"crl-husky has no model.yaml for robot {robot_name!r}")
    urdf_path = os.path.join(_REPO_ROOT, 'data', 'husky_urdf', model['urdf'])

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
