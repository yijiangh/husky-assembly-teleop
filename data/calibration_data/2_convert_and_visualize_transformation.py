"""
Compute and visualize the calibrated transformation from mocap base frame to robot base.

This script:
1. Loads the calibration result from 1_calibration_analysis.py
2. Computes the transformation from base_mocap to base_footprint
3. Visualizes the result in PyBullet
4. Saves the calibrated transformation to a JSON file
"""

import os
import json
import numpy as np
import pybullet as p
import pybullet_planning as pp

from config_loader import load_config, get_robot_urdf, get_arm_base_link_name, HERE
from logging_utils import setup_logger

# Load configuration
config = load_config()
DATE_FOLDER = config['date_folder']
ROBOT_NAME = config['robot_name']
ARM = config['arm']
USE_GUI = config.get('use_gui', True)
ARM_BASE_LINK_NAME = get_arm_base_link_name(ROBOT_NAME, ARM)

# File paths
CALIBRATION_FILE = os.path.join(HERE, DATE_FOLDER, 'base_frame_calibration.json')
OUTPUT_FILE = os.path.join(HERE, DATE_FOLDER, f'calibrated_transformation_{ROBOT_NAME}.json')

# Configure logging with colored output
log_file = os.path.join(HERE, DATE_FOLDER, 'compute_tf_log.txt')
logger = setup_logger(log_file=log_file)


def load_calibration_data(calibration_file):
    """Load calibration data from JSON file."""
    with open(calibration_file, 'r') as f:
        data = json.load(f)

    # Reconstruct transformation matrix using dynamic key name
    key_name = f'base_mocap_from_{ARM_BASE_LINK_NAME}'
    tf = np.array(data[key_name])

    logger.info('Loaded calibration data from: %s', calibration_file)

    return tf, data


def visualize_link_poses(robot, draw_length=0.1):
    """Draw poses and labels for all robot links."""
    for link_id in range(pp.get_num_links(robot)):
        link_name = pp.get_link_name(robot, link_id)
        if link_name:
            link_pose = pp.get_link_pose(robot, link_id)
            pp.draw_pose(link_pose, length=draw_length)
            
            # Add text with small random offset to avoid overlapping
            pos = link_pose[0]
            offset = [np.random.uniform(-0.02, 0.02) for _ in range(2)] + [np.random.uniform(0.05, 0.07)]
            text_pos = [pos[i] + offset[i] for i in range(3)]
            pp.add_text(link_name, position=text_pos)


def main():
    logger.info('=' * 60)
    logger.info('Computing calibrated transformation')
    logger.info('=' * 60)
    
    # * Load calibration data
    # This transformation is: base_mocap_from_{ARM_BASE_LINK_NAME}
    # (arm base link pose expressed in the base mocap frame)
    base_mocap_from_arm_base_link_tf, calibration_data = load_calibration_data(CALIBRATION_FILE)
    CALIB_base_mocap_from_arm_base_link = pp.pose_from_tform(base_mocap_from_arm_base_link_tf)

    logger.info(f'base_mocap_from_{ARM_BASE_LINK_NAME}: %s', CALIB_base_mocap_from_arm_base_link)
    
    # Initialize PyBullet
    pp.connect(use_gui=USE_GUI, shadows=True, color=[0.9, 0.9, 1.0])
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)
    
    # Load robot
    robot_urdf = get_robot_urdf(ROBOT_NAME)
    logger.info('Loading robot from: %s', robot_urdf)
    
    with pp.HideOutput():
        robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)
    
    # Get link references
    base_footprint_link = pp.link_from_name(robot, "base_footprint")
    arm_base_link_name = get_arm_base_link_name(ROBOT_NAME, ARM)
    arm_base_link = pp.link_from_name(robot, arm_base_link_name)
    
    # Get fixed transformation from arm_base_link to base_footprint (from URDF)
    arm_base_link_from_base_footprint = pp.get_relative_pose(robot, base_footprint_link, arm_base_link)
    logger.info('arm_base_link_from_base_footprint (from URDF): %s', arm_base_link_from_base_footprint)
    
    # Compute base_mocap_from_base_footprint
    # base_mocap_from_base_footprint = base_mocap_from_arm_base_link * arm_base_link_from_base_footprint
    base_mocap_from_base_footprint = pp.multiply(CALIB_base_mocap_from_arm_base_link, arm_base_link_from_base_footprint)
    logger.info('base_mocap_from_base_footprint: %s', base_mocap_from_base_footprint)
    
    # Set robot transparency
    for link_id in range(pp.get_num_links(robot)):
        pp.set_color(robot, [1, 1, 1, 0.6], link=link_id)
    
    # Visualize all link poses (optional, can be commented out)
    # visualize_link_poses(robot)
    # pp.wait_if_gui('All link poses visualized')
    # pp.remove_all_debug()
    
    # Visualize calibration results
    # Draw base_mocap frame at origin (identity)
    base_mocap_pose = pp.Pose()  # Identity pose
    pp.draw_pose(base_mocap_pose, length=0.3)
    pp.add_text("base_mocap (origin)", position=[0.05, 0.05, 0.05])
    
    # Draw arm_base_link in base_mocap frame
    pp.draw_pose(CALIB_base_mocap_from_arm_base_link, length=0.2)
    pp.add_text(ARM_BASE_LINK_NAME, position=[p + 0.02 for p in CALIB_base_mocap_from_arm_base_link[0]])
    
    # Draw base_footprint in base_mocap frame
    pp.draw_pose(base_mocap_from_base_footprint, length=0.2)
    pp.add_text("base_footprint", position=[p + 0.02 for p in base_mocap_from_base_footprint[0]])
    
    # Set robot pose
    pp.set_pose(robot, base_mocap_from_base_footprint)

    # Compare calibrated arm_base_link with URDF arm_base_link
    URDF_base_mocap_from_arm_base_link = pp.get_link_pose(robot, arm_base_link)
    logger.info('URDF arm_base_link pose: %s', URDF_base_mocap_from_arm_base_link)
    logger.info('CALIB arm_base_link pose: %s', CALIB_base_mocap_from_arm_base_link)
    
    # Compute position and orientation difference
    pos_diff = np.array(URDF_base_mocap_from_arm_base_link[0]) - np.array(CALIB_base_mocap_from_arm_base_link[0])
    pos_diff_mm = pos_diff * 1000
    logger.info('Position difference (URDF - CALIB): %.3f, %.3f, %.3f mm', *pos_diff_mm)
    logger.info('Position difference magnitude: %.3f mm', np.linalg.norm(pos_diff_mm))
    
    # Draw URDF arm_base_link for comparison
    pp.draw_pose(URDF_base_mocap_from_arm_base_link, length=0.15)
    pp.add_text(f"{ARM_BASE_LINK_NAME} (URDF)", position=[p + 0.04 for p in URDF_base_mocap_from_arm_base_link[0]])
     
    # Save calibrated transformation
    output_data = {
        f'base_mocap_from_{ARM_BASE_LINK_NAME}': [list(v) for v in CALIB_base_mocap_from_arm_base_link],
        'base_mocap_from_base_footprint': [list(v) for v in base_mocap_from_base_footprint],
    }
    
    with open(OUTPUT_FILE, 'w') as f:
        json.dump(output_data, f, indent=4)
    
    logger.info('Calibrated transformation saved to: %s', OUTPUT_FILE)
    logger.info('=' * 60)
    logger.info('Done!')
    logger.info('=' * 60)
    
    pp.wait_if_gui('Calibration visualization')


if __name__ == '__main__':
    main()
