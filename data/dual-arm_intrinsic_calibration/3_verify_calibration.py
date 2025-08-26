"""
Dual-Arm Calibration Verification Script

This script verifies the dual-arm calibration by:
1. Loading the calibrated URDF
2. Computing forward kinematics for each data entry
3. Applying TCP offsets
4. Comparing computed vs recorded positions
5. Visualizing the results

Author: Based on dual-arm calibration work
"""

import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os, logging

# Add the parent directory to the path to import pybullet_planning
import pybullet_planning as pp

HERE = os.path.dirname(os.path.abspath(__file__))

# TCP offsets in mm (convert to meters)
LEFT_TCP_OFFSET_MM = [0.84, 0.1, 118.31]  # tool0_from_TCP for left arm
RIGHT_TCP_OFFSET_MM = [-2.07, -0.56, 118.67]  # tool0_from_TCP for right arm

# Convert to meters
TCP_OFFSETS_M = [[x/1000.0 for x in LEFT_TCP_OFFSET_MM],
                [x/1000.0 for x in RIGHT_TCP_OFFSET_MM]]

HUSKY_DUAL_UR5e_JOINT_NAMES = [["left_ur_arm_shoulder_pan_joint", 
                      "left_ur_arm_shoulder_lift_joint",
                      "left_ur_arm_elbow_joint", 
                      "left_ur_arm_wrist_1_joint", 
                      "left_ur_arm_wrist_2_joint", 
                      "left_ur_arm_wrist_3_joint" ],
                                ["right_ur_arm_shoulder_pan_joint", 
                      "right_ur_arm_shoulder_lift_joint",
                      "right_ur_arm_elbow_joint", 
                      "right_ur_arm_wrist_1_joint", 
                      "right_ur_arm_wrist_2_joint", 
                      "right_ur_arm_wrist_3_joint" ]]

def load_robot_and_data(logger, urdf_type="uncalibrated"):
    """
    Load the robot URDF and calibration data.
    
    Args:
        urdf_type (str): Type of URDF to load. Options:
            - "uncalibrated": husky_dual_ur5_e_no_base_joint.urdf
            - "arm_calibrated": husky_dual_ur5_e_no_base_joint_Arm_Calibrated.urdf
            - "all_calibrated": husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf
    
    Returns:
        tuple: (robot_id, data, urdf_type) where robot_id is the PyBullet robot ID, 
               data is the JSON data, and urdf_type is a string indicating the URDF type
    """
    # Choose URDF file based on parameter
    if urdf_type == "uncalibrated":
        urdf_filename = "husky_dual_ur5_e_no_base_joint.urdf"
        urdf_type_name = "uncalibrated"
    elif urdf_type == "arm_calibrated":
        urdf_filename = "husky_dual_ur5_e_no_base_joint_Arm_Calibrated.urdf"
        urdf_type_name = "arm_calibrated"
    elif urdf_type == "all_calibrated":
        urdf_filename = "husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf"
        urdf_type_name = "all_calibrated"
    else:
        raise ValueError(f"Invalid urdf_type: {urdf_type}. Must be 'uncalibrated', 'arm_calibrated', or 'all_calibrated'")
    
    # Load the URDF
    urdf_path = os.path.join(r"D:\0_Project\03-2025_husky_assembly\Code\husky-assembly-teleop\data", "husky_urdf", "mt_husky_dual_ur5_e_moveit_config", "urdf", urdf_filename)
    
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    
    with pp.HideOutput():
        robot_id = pp.load_pybullet(urdf_path, fixed_base=True)

    pp.set_color(robot_id, pp.apply_alpha(pp.GREY, 0.3))
    
    # Load calibration data
    json_path = os.path.join(os.path.dirname(__file__), "20250822_dual-arm-intrinsic_data.json")
    with open(json_path, 'r') as f:
        data = json.load(f)
    
    logger.info(f"Loaded {len(data)} data entries")
    return robot_id, data, urdf_type_name

def compute_forward_kinematics(robot_id, joint_names, joint_config, arm_side):
    #     Write a python script that does the following:
    # 1. Use the load_pybullet function to load the calibrated URDF for the dual arm  @husky_dual_ur5_e_no_base_joint_Calibrated.urdf 
    # 2. For each entry of the data in @20250822_dual-arm-intrinsic_data.json , compute FK (use pp.set_joint_positions on the joint conf values and then pp.get_link_pose on the tool0 link). 
    # 3. Apply the following additional TCP offset on the tool0 frame. Convert mm to m. since pybullet uses meter.
    # Left arm: tool0_from_TCP: [0.84, 0.1, 118.31] mm
    # Right arm: tool0_from_TCP: [-2.07, -0.56, 118.67] mm
    # 4. Record out the positional difference between these two points in the world coordinate frame.
    # 5. Also for each arm, get the tcp_point's position in its own arm_base_link frame, can compare that with the recorded data entry. Record the positional difference.
    # 6. Print these data, save it as a json, and also visualize in graph.
    joints = pp.joints_from_names(robot_id, joint_names)
    pp.set_joint_positions(robot_id, joints, joint_config)
    world_from_tool0 = pp.get_link_pose(robot_id, pp.link_from_name(robot_id, f"{arm_side}_ur_arm_tool0"))    
    world_from_tcp = pp.multiply(world_from_tool0, pp.Pose(point=pp.Point(*TCP_OFFSETS_M[0 if arm_side == 'left' else 1])))

    # world_from_base_link = pp.get_link_pose(robot_id, pp.link_from_name(robot_id, f"{arm_side}_ur_arm_base_link"))
    world_from_base_link = pp.get_link_pose(robot_id, pp.link_from_name(robot_id, f"{arm_side}_ur_arm_base_link_inertia"))

    base_link_from_tcp = pp.multiply(pp.invert(world_from_base_link), world_from_tcp)
    return world_from_tcp, base_link_from_tcp

def compute_position_difference(pos1, pos2):
    """
    Compute Euclidean distance between two positions.
    
    Args:
        pos1: [x, y, z] position 1
        pos2: [x, y, z] position 2
        
    Returns:
        float: Euclidean distance
    """
    return np.linalg.norm(np.array(pos1) - np.array(pos2))

def main(gui=True, urdf_type="uncalibrated"):
    """
    Main function to verify dual-arm calibration.
    
    Args:
        urdf_type (str): Type of URDF to load. Options:
            - "uncalibrated": husky_dual_ur5_e_no_base_joint.urdf
            - "arm_calibrated": husky_dual_ur5_e_no_base_joint_Arm_Calibrated.urdf
            - "all_calibrated": husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf
    """
    from logging.handlers import RotatingFileHandler

    # Set up logging to file
 
    # Configure logging
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO) 
    logger.addHandler(console_handler)

    # Create file handler with URDF type in name
    LOG_PATH = os.path.join(HERE, f"dual_arm_intrinsic_calibration_log_{urdf_type}.txt")
    file_handler = logging.FileHandler(LOG_PATH, mode='w')
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
    logger.addHandler(file_handler)

    # Example usage
    logger.info(f"=== Dual-Arm Calibration Verification ({urdf_type.upper()} URDF) ===")
    
    # Initialize PyBullet
    # p.connect(p.DIRECT)  # Use DIRECT mode for faster computation
    # p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # p.setGravity(0, 0, -9.81)
    pp.connect(use_gui=gui, shadows=True, color=[0.9, 0.9, 1.0])
    logger.info("Connected to PyBullet")
 
    # Load robot and data
    robot_id, data, urdf_type_name = load_robot_and_data(logger, urdf_type)
    # Get joint indices for each arm using HUSKY_DUAL_UR5e_JOINT_NAMES
    left_joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
    right_joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]

    # Results storage
    results = []
    
    # Load calibration results for verification
    calibration_results_path = os.path.join(os.path.dirname(__file__), "calibration_results.json")
    with open(calibration_results_path, 'r') as f:
        calibration_results = json.load(f)
    
    # Parse calibration transformation (right_base_link_inertia_from_left_base_link_inertia)
    calibration_translation_mm = calibration_results['translation']
    calibration_rotation_rpy = calibration_results['rotation_rpy']
    
    # Convert translation from mm to meters
    calibration_translation_m = [x / 1000.0 for x in calibration_translation_mm]
    
    logger.info(f"Calibration transformation:")
    logger.info(f"  Translation (mm): {calibration_translation_mm}")
    logger.info(f"  Translation (m): {calibration_translation_m}")
    logger.info(f"  Rotation RPY (rad): {calibration_rotation_rpy}")
    
    # Create calibration transformation pose
    calibration_pose = pp.Pose(
        point=pp.Point(*calibration_translation_m),
        euler=pp.Euler(*calibration_rotation_rpy)
    )
    
    # Get poses for verification
    logger.info(f"\n=== Calibration Verification ===")
    
    # Get the pose of right_base_link_inertia
    right_base_link_inertia_id = pp.link_from_name(robot_id, "right_ur_arm_base_link_inertia")
    world_from_right_base_link_inertia = pp.get_link_pose(robot_id, right_base_link_inertia_id)
    
    logger.info(f"Right base link inertia pose:")
    logger.info(f"  Position: {world_from_right_base_link_inertia[0]}")
    logger.info(f"  Orientation: {world_from_right_base_link_inertia[1]}")
    
    # Draw the right base link inertia pose
    pp.draw_pose(world_from_right_base_link_inertia, length=0.1, width=2)
    
    # Get ground truth left base link inertia pose
    left_base_link_inertia_id = pp.link_from_name(robot_id, "left_ur_arm_base_link_inertia")
    ground_truth_left_base_link_inertia = pp.get_link_pose(robot_id, left_base_link_inertia_id)
    
    logger.info(f"Ground truth left base link inertia pose:")
    logger.info(f"  Position: {ground_truth_left_base_link_inertia[0]}")
    logger.info(f"  Orientation: {ground_truth_left_base_link_inertia[1]}")
    
    # Draw the ground truth left base link inertia pose
    pp.draw_pose(ground_truth_left_base_link_inertia, length=0.05, width=2)  # Red color
    
    # Compute left base link inertia pose using calibration transformation
    world_from_left_inertia_computed = pp.multiply(world_from_right_base_link_inertia, calibration_pose)
    
    logger.info(f"Computed left base link inertia pose:")
    logger.info(f"  Position: {world_from_left_inertia_computed[0]}")
    logger.info(f"  Orientation: {world_from_left_inertia_computed[1]}")
    
    # Draw the computed left base link inertia pose
    pp.draw_pose(world_from_left_inertia_computed, length=0.1, width=2)  # Green color
    
    # Numerically compare computed vs ground truth
    position_error = compute_position_difference(
        world_from_left_inertia_computed[0], 
        ground_truth_left_base_link_inertia[0]
    )
    
    # Compute orientation error (simplified - using quaternion difference)
    orientation_error = compute_position_difference(
        world_from_left_inertia_computed[1], 
        ground_truth_left_base_link_inertia[1]
    )
    
    logger.info(f"\nCalibration verification results:")
    logger.info(f"  Position error: {position_error*1000:.2f} mm")
    logger.info(f"  Orientation error: {orientation_error:.6f}")
    
    if position_error < 0.01:  # Less than 10mm
        logger.info(f"Position error is acceptable (< 10mm)")
    else:
        logger.info(f"Position error is too large (>= 10mm)")
    
    pp.wait_if_gui()
    
    for i, entry in enumerate(data):
        logger.info(f"\nProcessing entry {i+1}/{len(data)}")
        
        # Skip entries with null configuration
        if entry['left_arm']['conf'] is None:
            logger.info(f"  Skipping entry {i+1} - left arm conf is null")
            continue
        
        left_conf = entry['left_arm']['conf']
        right_conf = entry['right_arm']['conf']
        
        # Recorded positions (convert from mm to m)
        recorded_left_pos = [x/1000.0 for x in entry['left_arm']['tcp_point_in_base_frame']]
        recorded_right_pos = [x/1000.0 for x in entry['right_arm']['tcp_point_in_base_frame']]
        
        # Compute forward kinematics
        left_world_from_tcp, left_base_from_tcp = compute_forward_kinematics(robot_id, left_joint_names, left_conf, 'left')
        right_world_from_tcp, right_base_from_tcp = compute_forward_kinematics(robot_id, right_joint_names, right_conf, 'right')
        
        # Compute differences
        # ! this is factoring in the base link transformation inaccuracy
        world_tcp_diff = compute_position_difference(left_world_from_tcp[0], right_world_from_tcp[0])

        # ! this should be near zero if we use the calibrated urdf per arm
        left_recorded_diff = compute_position_difference(left_base_from_tcp[0], recorded_left_pos)
        right_recorded_diff = compute_position_difference(right_base_from_tcp[0], recorded_right_pos)

        # # Skip this entry if left_recorded_diff or right_recorded_diff is larger than 50 mm
        # if left_recorded_diff > 0.05 or right_recorded_diff > 0.05:
        #     logger.info(f"  Skipping entry {i+1} - left or right recorded diff > 50 mm (left: {left_recorded_diff*1000:.2f} mm, right: {right_recorded_diff*1000:.2f} mm)")
        #     continue
        
        # Store results
        result = {
            'entry_index': i,
            'left_conf': left_conf,
            'right_conf': right_conf,
            'recorded_left_pos_mm': entry['left_arm']['tcp_point_in_base_frame'],
            'recorded_right_pos_mm': entry['right_arm']['tcp_point_in_base_frame'],
            'computed_left_tcp_world_m': left_world_from_tcp[0],
            'computed_right_tcp_world_m': right_world_from_tcp[0],
            'computed_left_tcp_base_m': left_base_from_tcp[0],
            'computed_right_tcp_base_m': right_base_from_tcp[0],
            'world_tcp_difference_m': world_tcp_diff,
            'left_recorded_difference_m': left_recorded_diff,
            'right_recorded_difference_m': right_recorded_diff
        }
        
        results.append(result)
        
        logger.info(f"  World TCP difference: {world_tcp_diff*1000:.2f} mm")
        logger.info(f"  Left recorded difference: {left_recorded_diff*1000:.2f} mm")
        logger.info(f"  Right recorded difference: {right_recorded_diff*1000:.2f} mm")
        pp.wait_if_gui()
    
    # Print summary statistics
    if results:
        world_diffs = [r['world_tcp_difference_m'] for r in results]
        left_diffs = [r['left_recorded_difference_m'] for r in results]
        right_diffs = [r['right_recorded_difference_m'] for r in results]
        
        logger.info(f"\n=== Summary Statistics ===")
        logger.info(f"World TCP differences (mm):")
        logger.info(f"  Mean: {np.mean(world_diffs)*1000:.2f}")
        logger.info(f"  Std: {np.std(world_diffs)*1000:.2f}")
        logger.info(f"  Min: {np.min(world_diffs)*1000:.2f}")
        logger.info(f"  Max: {np.max(world_diffs)*1000:.2f}")
        
        logger.info(f"\nLeft arm recorded differences (mm):")
        logger.info(f"  Mean: {np.mean(left_diffs)*1000:.2f}")
        logger.info(f"  Std: {np.std(left_diffs)*1000:.2f}")
        logger.info(f"  Min: {np.min(left_diffs)*1000:.2f}")
        logger.info(f"  Max: {np.max(left_diffs)*1000:.2f}")
        
        logger.info(f"\nRight arm recorded differences (mm):")
        logger.info(f"  Mean: {np.mean(right_diffs)*1000:.2f}")
        logger.info(f"  Std: {np.std(right_diffs)*1000:.2f}")
        logger.info(f"  Min: {np.min(right_diffs)*1000:.2f}")
        logger.info(f"  Max: {np.max(right_diffs)*1000:.2f}")
        
        # Save results to JSON
        output_file = os.path.join(HERE, f'calibration_verification_results_{urdf_type_name}.json')
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2)
        logger.info(f"\nResults saved to: {output_file}")
        
        # Create visualization
        create_visualization(results, logger, urdf_type_name)
        
    else:
        logger.info("No valid results to process")
            
    pp.disconnect()

def create_visualization(results, logger, urdf_type):
    """
    Create visualization of the results.
    
    Args:
        results: List of result dictionaries
        logger: Logger instance
        urdf_type: String indicating the URDF type ("calibrated" or "uncalibrated")
    """
    try:
        # Extract data for plotting
        entry_indices = [r['entry_index'] for r in results]
        world_diffs_mm = [r['world_tcp_difference_m'] * 1000 for r in results]
        left_diffs_mm = [r['left_recorded_difference_m'] * 1000 for r in results]
        right_diffs_mm = [r['right_recorded_difference_m'] * 1000 for r in results]
        
        # Create figure with subplots
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
        
        # Plot 1: World TCP differences
        ax1.plot(entry_indices, world_diffs_mm, 'bo-', label='World TCP Difference')
        ax1.set_xlabel('Entry Index')
        ax1.set_ylabel('Distance (mm)')
        ax1.set_title('World TCP Position Differences')
        ax1.grid(True)
        ax1.legend()
        
        # Plot 2: Left arm recorded differences
        ax2.plot(entry_indices, left_diffs_mm, 'ro-', label='Left Arm Difference')
        ax2.set_xlabel('Entry Index')
        ax2.set_ylabel('Distance (mm)')
        ax2.set_title('Left Arm: Computed vs Recorded')
        ax2.grid(True)
        ax2.legend()
        
        # Plot 3: Right arm recorded differences
        ax3.plot(entry_indices, right_diffs_mm, 'go-', label='Right Arm Difference')
        ax3.set_xlabel('Entry Index')
        ax3.set_ylabel('Distance (mm)')
        ax3.set_title('Right Arm: Computed vs Recorded')
        ax3.grid(True)
        ax3.legend()
        
        # Plot 4: 3D scatter plot of computed vs recorded positions
        ax4 = fig.add_subplot(2, 2, 4, projection='3d')
        
        # Extract positions
        left_computed = np.array([r['computed_left_tcp_base_m'] for r in results])
        left_recorded = np.array([r['recorded_left_pos_mm'] for r in results]) / 1000.0  # Convert to meters
        right_computed = np.array([r['computed_right_tcp_base_m'] for r in results])
        right_recorded = np.array([r['recorded_right_pos_mm'] for r in results]) / 1000.0  # Convert to meters
        
        # Plot left arm positions
        ax4.scatter(left_computed[:, 0], left_computed[:, 1], left_computed[:, 2], 
                   c='red', marker='o', label='Left Computed', s=50)
        ax4.scatter(left_recorded[:, 0], left_recorded[:, 1], left_recorded[:, 2], 
                   c='red', marker='s', label='Left Recorded', s=50, alpha=0.7)
        
        # Plot right arm positions
        ax4.scatter(right_computed[:, 0], right_computed[:, 1], right_computed[:, 2], 
                   c='blue', marker='o', label='Right Computed', s=50)
        ax4.scatter(right_recorded[:, 0], right_recorded[:, 1], right_recorded[:, 2], 
                   c='blue', marker='s', label='Right Recorded', s=50, alpha=0.7)
        
        ax4.set_xlabel('X (m)')
        ax4.set_ylabel('Y (m)')
        ax4.set_zlabel('Z (m)')
        ax4.set_title('Computed vs Recorded Positions')
        ax4.legend()
        
        plt.tight_layout()
        plt.savefig(os.path.join(HERE, f'calibration_verification_plots_{urdf_type}.png'), dpi=300, bbox_inches='tight')
        logger.info(f"Visualization saved as: calibration_verification_plots_{urdf_type}.png")
        
        # Show the plot
        # plt.show()
        
    except Exception as e:
        logger.info(f"Error creating visualization: {e}")

if __name__ == "__main__":
    # Set this to True to use calibrated URDF, False to use uncalibrated URDF
    USE_GUI = 0
    URDF_TYPE = "all_calibrated"  # Options: "uncalibrated", "arm_calibrated", "all_calibrated"
    
    main(gui=USE_GUI, urdf_type=URDF_TYPE)
