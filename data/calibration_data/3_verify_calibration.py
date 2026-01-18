"""
Verify calibration by comparing FK-computed tool0 pose with mocap-measured flange pose.

This script validates the calibration quality by:
1. Loading calibration data (joint configs + mocap poses)
2. Computing FK to get tool0 pose from joint configurations
3. Comparing FK tool0 pose with mocap flange pose
4. Analyzing the consistency of the tool0-to-flange_mocap transformation

If calibration is correct, the transformation from tool0 (FK) to flange_mocap should be
constant across all configurations, representing the fixed offset between the two frames.
"""

import os
import json
import logging
import numpy as np
import matplotlib.pyplot as plt
import pybullet_planning as pp

from config_loader import (
    load_config, get_robot_urdf, get_joint_names, 
    get_tool0_link_name, HERE
)

# Load configuration
config = load_config()
DATE_FOLDER = config['date_folder']
DATA_BATCH = config['data_batches'][0]  # Default to first batch, can be overridden
ROBOT_NAME = config['robot_name']
ARM = config['arm']
USE_GUI = True # config['use_gui']

# File paths
DATA_FOLDER = os.path.join(HERE, DATE_FOLDER, DATA_BATCH)
CALIBRATION_FILE = os.path.join(HERE, DATE_FOLDER, f'calibrated_transformation_{ROBOT_NAME}.json')
ROBOT_URDF = get_robot_urdf(ROBOT_NAME)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger()

file_handler = logging.FileHandler(os.path.join(DATA_FOLDER, 'verification_log.txt'), mode='w')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
logger.addHandler(file_handler)


def load_calibration(calibration_file):
    """Load calibrated transformation from JSON file."""
    with open(calibration_file, 'r') as f:
        data = json.load(f)
    return data.get('base_mocap_from_base_footprint', None)


def compute_tool0_flange_offset(robot, arm_joints, tool0_link_name, calibration_data, data_folder):
    """
    Compute the offset between FK tool0 and mocap flange for each data point.
    
    Returns list of (tool0_from_flange_mocap, joint_conf) tuples.
    """
    # Load calibration - if not available, use identity
    base_mocap_from_base_footprint = calibration_data
    if base_mocap_from_base_footprint is None:
        logger.warning('No calibration data found, using identity pose')
        base_mocap_from_base_footprint = pp.unit_pose()
    
    # Find all calibration JSON files
    json_files = [f for f in os.listdir(data_folder) 
                  if f.startswith('calibration_') and f.endswith('.json')]
    
    results = []
    
    for file_name in json_files:
        logger.info('Processing: %s', file_name)
        file_path = os.path.join(data_folder, file_name)
        
        with open(file_path, 'r') as f:
            data = json.load(f)
        
        for entry in data['raw_data']:
            flange_mocap_pose = entry.get("flange_mocap_pose", [])
            base_mocap_pose = entry.get("base_mocap_pose", [])
            joint_conf = entry.get("joint_conf", [])
            
            if not flange_mocap_pose or not base_mocap_pose or not joint_conf:
                continue
            
            # Set robot pose based on mocap and calibration
            world_from_footprint = pp.multiply(base_mocap_pose, base_mocap_from_base_footprint)
            pp.set_pose(robot, world_from_footprint)
            pp.set_joint_positions(robot, arm_joints, joint_conf)
            
            # Get FK tool0 pose
            tool0_link = pp.link_from_name(robot, tool0_link_name)
            world_from_tool0 = pp.get_link_pose(robot, tool0_link)
            
            # Compute offset: tool0_from_flange_mocap
            # This should be constant if calibration is correct
            tool0_from_flange_mocap = pp.multiply(pp.invert(world_from_tool0), flange_mocap_pose)
            
            results.append({
                'tool0_from_flange_mocap': tool0_from_flange_mocap,
                'joint_conf': joint_conf,
                'world_from_tool0': world_from_tool0,
                'flange_mocap_pose': flange_mocap_pose
            })
            
            # Visualization (if GUI enabled)
            if USE_GUI:
                pp.draw_pose(world_from_tool0, length=0.1)
                pp.add_text("tool0 (FK)", position=[p + 0.01 for p in world_from_tool0[0]])
                pp.draw_pose(flange_mocap_pose, length=0.1)
                pp.add_text("flange_mocap", position=[p + 0.01 for p in flange_mocap_pose[0]])
                pp.wait_if_gui('Visualization')
                pp.remove_all_debug()
    
    return results


def analyze_results(results, data_folder, data_batch):
    """Analyze the consistency of tool0-to-flange offset across all samples."""
    
    # Extract positions and orientations
    positions = [r['tool0_from_flange_mocap'][0] for r in results]
    quaternions = [r['tool0_from_flange_mocap'][1] for r in results]
    joint_confs = [r['joint_conf'] for r in results]
    
    # Position analysis
    positions = np.array(positions)
    pos_mean = np.mean(positions, axis=0)
    pos_distances = np.linalg.norm(positions - pos_mean, axis=1) * 1000  # mm
    
    logger.info('=' * 60)
    logger.info('Position Analysis (tool0_from_flange_mocap offset)')
    logger.info('=' * 60)
    logger.info('  Mean position: [%.4f, %.4f, %.4f] m', *pos_mean)
    logger.info('  Max distance from mean: %.3f mm', np.max(pos_distances))
    logger.info('  Mean distance from mean: %.3f mm', np.mean(pos_distances))
    logger.info('  Std distance from mean: %.3f mm', np.std(pos_distances))
    
    # Orientation analysis
    rotation_matrices = [pp.matrix_from_quat(q) for q in quaternions]
    
    logger.info('=' * 60)
    logger.info('Orientation Analysis')
    logger.info('=' * 60)
    
    # Analyze each axis
    for axis_idx, axis_name in enumerate(['X', 'Y', 'Z']):
        axes = np.array([rm[:, axis_idx] for rm in rotation_matrices])
        axis_mean = np.mean(axes, axis=0)
        axis_mean = axis_mean / np.linalg.norm(axis_mean)
        
        angles = [np.rad2deg(np.arccos(np.clip(np.dot(a, axis_mean), -1, 1))) for a in axes]
        logger.info('  %s-axis max angle from mean: %.3f deg', axis_name, np.max(angles))
    
    # Plot results
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # Plot 1: Position offset vs first joint angle
    first_joint_values = [conf[0] for conf in joint_confs]
    ax1 = axes[0]
    ax1.scatter(first_joint_values, pos_distances, alpha=0.7, s=20)
    ax1.set_xlabel('First joint angle (rad)')
    ax1.set_ylabel('Position offset from mean (mm)')
    ax1.set_title('Position Consistency vs Joint Configuration')
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Orientation components vs first joint angle
    ax2 = axes[1]
    colors = ['r', 'g', 'b']
    for axis_idx, (color, axis_name) in enumerate(zip(colors, ['X', 'Y', 'Z'])):
        axis_components = [rm[0, axis_idx] for rm in rotation_matrices]  # X component of each axis
        ax2.scatter(first_joint_values, axis_components, c=color, alpha=0.5, s=20, 
                   label=f'{axis_name}-axis (x-component)')
    
    ax2.set_xlabel('First joint angle (rad)')
    ax2.set_ylabel('Rotation matrix component')
    ax2.set_title('Orientation Consistency vs Joint Configuration')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = os.path.join(data_folder, f'verification_{data_batch}.png')
    plt.savefig(output_path, dpi=150)
    logger.info('Plot saved to: %s', output_path)
    plt.show()
    plt.close()


def main():
    logger.info('=' * 60)
    logger.info('Calibration Verification')
    logger.info('=' * 60)
    logger.info('Data batch: %s', DATA_BATCH)
    logger.info('Robot: %s', ROBOT_NAME)
    
    # Initialize PyBullet
    pp.connect(use_gui=USE_GUI, shadows=True, color=[0.9, 0.9, 1.0])
    
    # Load robot
    if not os.path.exists(ROBOT_URDF):
        raise FileNotFoundError(f"URDF not found: {ROBOT_URDF}")
    
    with pp.HideOutput():
        robot = pp.load_pybullet(ROBOT_URDF, fixed_base=False, cylinder=False)
    
    # Get joint names and tool0 link name based on robot type
    joint_names = get_joint_names(ROBOT_NAME, ARM)
    tool0_link_name = get_tool0_link_name(ROBOT_NAME, ARM)
    arm_joints = pp.joints_from_names(robot, joint_names)
    
    logger.info('Joint names: %s', joint_names)
    logger.info('Tool0 link: %s', tool0_link_name)
    
    # Load calibration
    calibration_data = None
    if os.path.exists(CALIBRATION_FILE):
        calibration_data = load_calibration(CALIBRATION_FILE)
        logger.info('Loaded calibration from: %s', CALIBRATION_FILE)
    else:
        logger.warning('Calibration file not found: %s', CALIBRATION_FILE)
    
    # Compute tool0-flange offsets
    results = compute_tool0_flange_offset(robot, arm_joints, tool0_link_name, calibration_data, DATA_FOLDER)
    logger.info('Processed %d samples', len(results))
    
    if USE_GUI:
        pp.wait_if_gui('Processing complete')
    
    # Analyze results
    analyze_results(results, DATA_FOLDER, DATA_BATCH)
    
    pp.disconnect()
    
    logger.info('=' * 60)
    logger.info('Verification complete!')
    logger.info('=' * 60)


if __name__ == '__main__':
    main()
