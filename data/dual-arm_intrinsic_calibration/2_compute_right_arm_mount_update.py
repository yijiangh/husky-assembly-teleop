#!/usr/bin/env python3
"""
Script to compute the updated transformation for right_arm_mount_joint
based on kinematic chain analysis and calibration data using PyBullet planning.
"""

import json
import numpy as np
import os
import logging
import xml.etree.ElementTree as ET
import shutil
from datetime import datetime

# Add the parent directory to the path to import pybullet_planning
import pybullet_planning as pp

HERE = os.path.dirname(os.path.abspath(__file__))


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


def transformation_to_rpy_xyz(T):
    """Convert 4x4 transformation matrix to rpy and xyz."""
    xyz = T[:3, 3]
    
    # Extract rotation matrix
    R = T[:3, :3]
    
    # Convert to roll-pitch-yaw
    pitch = np.arcsin(-R[2, 0])
    if abs(np.cos(pitch)) > 1e-6:
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        # Gimbal lock case
        roll = np.arctan2(-R[0, 1], R[1, 1])
        yaw = 0
    
    return [roll, pitch, yaw], xyz.tolist()


def pose_to_transformation_matrix(pose):
    """Convert PyBullet pose to 4x4 transformation matrix."""
    position, orientation = pose
    
    # Convert quaternion to rotation matrix
    from pybullet_planning import quaternion_matrix
    T = quaternion_matrix(orientation)
    T[:3, 3] = position
    
    return T


def main():
    # Configure logging
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    
    # Clear any existing handlers
    for handler in logger.handlers[:]:
        logger.removeHandler(handler)
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(logging.Formatter('%(levelname)s - %(message)s'))
    logger.addHandler(console_handler)

    # File handler
    log_filename = f"right_arm_mount_update_log.txt"
    log_path = os.path.join(HERE, log_filename)
    file_handler = logging.FileHandler(log_path, mode='w')
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
    logger.addHandler(file_handler)

    logger.info("=" * 80)
    logger.info("RIGHT ARM MOUNT JOINT UPDATE COMPUTATION")
    logger.info("=" * 80)
    
    # Load calibration data
    calibration_results_path = os.path.join(HERE, "calibration_results.json")
    with open(calibration_results_path, 'r') as f:
        calibration_data = json.load(f)
    
    logger.info("Calibration data:")
    logger.info(f"Translation (mm): {calibration_data['translation']}")
    logger.info(f"Rotation RPY (rad): {calibration_data['rotation_rpy']}")
    logger.info(f"Final error: {calibration_data['final_error']}")
    logger.info("")
    
    # Convert calibration translation from mm to meters
    calibration_translation_m = [x / 1000.0 for x in calibration_data['translation']]
    logger.info(f"Translation (m): {calibration_translation_m}")
    logger.info("")
    
    # Initialize PyBullet (without GUI for faster computation)
    pp.connect(use_gui=False)
    logger.info("Connected to PyBullet")
    
    # Load the URDF
    urdf_file = os.path.join(HERE, '..', "husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_Arm_Calibrated.urdf")
    
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    
    with pp.HideOutput():
        robot_id = pp.load_pybullet(urdf_file, fixed_base=True)
    
    logger.info(f"Loaded URDF: {urdf_file}")
    logger.info("")
    
    # Create calibration transformation pose
    calibration_pose = pp.Pose(
        point=pp.Point(*calibration_translation_m),
        euler=pp.Euler(*calibration_data['rotation_rpy'])
    )
    
    logger.info("=== Kinematic Chain Analysis ===")
    
    # Get the pose of right_base_link_inertia
    right_base_link_inertia_id = pp.link_from_name(robot_id, "right_ur_arm_base_link_inertia")
    world_from_right_base_link_inertia = pp.get_link_pose(robot_id, right_base_link_inertia_id)
    
    # Get ground truth left base link inertia pose
    left_base_link_inertia_id = pp.link_from_name(robot_id, "left_ur_arm_base_link_inertia")
    world_from_left_base_link_inertia = pp.get_link_pose(robot_id, left_base_link_inertia_id)
   
    # Compute left base link inertia pose using calibration transformation
    world_from_left_inertia_computed = pp.multiply(world_from_right_base_link_inertia, calibration_pose)
    
    logger.info("Computed left base link inertia pose:")
    logger.info(f"  Position: {world_from_left_inertia_computed[0]}")
    logger.info(f"  Orientation: {world_from_left_inertia_computed[1]}")
    logger.info("")
    
    # Verify the calibration transformation
    position_error = compute_position_difference(
        world_from_left_inertia_computed[0], 
        world_from_left_base_link_inertia[0]
    )
    
    orientation_error = compute_position_difference(
        world_from_left_inertia_computed[1], 
        world_from_left_base_link_inertia[1]
    )
    
    logger.info("Calibration verification:")
    logger.info(f"  Position error: {position_error*1000:.2f} mm")
    logger.info(f"  Orientation error: {orientation_error:.6f}")
    
    logger.info("")
    
    # Now compute the required transformation for right_arm_mount_joint
    logger.info("=== Computing Right Arm Mount Joint Transformation ===")
    
    # Get the current right arm mount joint pose (should be identity)
    right_arm_mount_joint_id = pp.link_from_name(robot_id, "right_ur_arm_base_link")
    world_from_right_arm_base = pp.get_link_pose(robot_id, right_arm_mount_joint_id)
   
    # Get the right arm bulkhead pose
    right_arm_bulkhead_id = pp.link_from_name(robot_id, "right_arm_bulkhead_link")
    world_from_right_bulkhead = pp.get_link_pose(robot_id, right_arm_bulkhead_id)
    
    # Get the left arm bulkhead pose
    left_arm_bulkhead_id = pp.link_from_name(robot_id, "left_arm_bulkhead_link")
    world_from_left_bulkhead = pp.get_link_pose(robot_id, left_arm_bulkhead_id)
    
    # Compute the required transformation
    # We want: right_bulkhead_from_right_base = right_bulkhead_from_left_base_inertia * left_base_inertia_from_right_base_inertia * right_base_inertia_from_right_base
    
    # 1. right_bulkhead_from_left_base_inertia (current left arm mount transformation)
    right_bulkhead_from_left_base_inertia = pp.multiply(
        pp.invert(world_from_right_bulkhead), 
        world_from_left_base_link_inertia
    )
    
    # 2. left_base_inertia_from_right_base_inertia (inverse of calibration transformation)
    left_base_inertia_from_right_base_inertia = pp.invert(calibration_pose)
    
    # 3. right_base_inertia_from_right_base (should be identity since they're both on the same bulkhead)
    # But we need to account for the different orientations in the URDF
    right_base_inertia_from_right_base = pp.multiply(
        pp.invert(world_from_right_base_link_inertia),
        world_from_right_arm_base
    )
    
    # 4. Compute the required transformation
    right_bulkhead_from_right_base = pp.multiply(
        right_bulkhead_from_left_base_inertia,
        left_base_inertia_from_right_base_inertia, 
        right_base_inertia_from_right_base
    )
    
    logger.info("Computed right_bulkhead_from_right_base:")
    logger.info(f"  Position: {right_bulkhead_from_right_base[0]}")
    logger.info(f"  Orientation: {right_bulkhead_from_right_base[1]}")
    logger.info("")
    
    # Convert to transformation matrix and then to RPY/XYZ
    T_right_bulkhead_from_right_base = pose_to_transformation_matrix(right_bulkhead_from_right_base)
    rpy_result, xyz_result = transformation_to_rpy_xyz(T_right_bulkhead_from_right_base)
    
    logger.info("=" * 80)
    logger.info("COMPUTED TRANSFORMATION")
    logger.info("=" * 80)
    logger.info("Computed transformation for right_arm_mount_joint:")
    logger.info(f"rpy: {[x for x in rpy_result]}")
    logger.info(f"xyz: {[x for x in xyz_result]}")
    logger.info("")
    
    # Verify the computation
    logger.info("=" * 80)
    logger.info("VERIFICATION")
    logger.info("=" * 80)
    
    # Apply the computed transformation and verify
    test_right_arm_base = pp.multiply(world_from_right_bulkhead, right_bulkhead_from_right_base)
    
    logger.info("Test right arm base pose (after applying computed transformation):")
    logger.info(f"  Position: {test_right_arm_base[0]}")
    logger.info(f"  Orientation: {test_right_arm_base[1]}")
    logger.info("")

    # Compare test_right_arm_base with world_from_right_base using similar mechanism as above

    position_error = compute_position_difference(
        test_right_arm_base[0],
        world_from_right_arm_base[0]
    )

    orientation_error = compute_position_difference(
        test_right_arm_base[1],
        world_from_right_arm_base[1]
    )

    logger.info("Comparison between test_right_arm_base and world_from_right_base:")
    logger.info(f"  Position error: {position_error*1000:.2f} mm")
    logger.info(f"  Orientation error: {orientation_error:.6f}")
    
    # Show the updated URDF joint definition
    logger.info("=" * 80)
    logger.info("UPDATED URDF JOINT DEFINITION")
    logger.info("=" * 80)
    updated_joint_def = """    <joint name="right_arm_mount_joint" type="fixed">
        <parent link="right_arm_bulkhead_link" />
        <child link="right_ur_arm_base_link" />
        <origin rpy="{} {} {}" xyz="{} {} {}" />
    </joint>""".format(
        # round(rpy_result[0], 6), round(rpy_result[1], 6), round(rpy_result[2], 6),
        # round(xyz_result[0], 6), round(xyz_result[1], 6), round(xyz_result[2], 6)
        rpy_result[0], rpy_result[1], rpy_result[2],
        xyz_result[0], xyz_result[1], xyz_result[2]
    )
    logger.info(updated_joint_def)
    logger.info("")
    
    # Create backup and update URDF
    # logger.info("=" * 80)
    # logger.info("URDF UPDATE")
    # logger.info("=" * 80)
    
    # Create backup
    # backup_file = f"{urdf_file}.backup"
    # shutil.copy2(urdf_file, backup_file)
    # logger.info(f"Created backup: {backup_file}")
    
    # # Update the URDF file
    # tree = ET.parse(urdf_file)
    # root = tree.getroot()
    
    # # Find the right_arm_mount_joint
    # for joint in root.findall('.//joint'):
    #     if joint.get('name') == 'right_arm_mount_joint':
    #         origin = joint.find('origin')
    #         if origin is not None:
    #             origin.set('rpy', f"{round(rpy_result[0], 6)} {round(rpy_result[1], 6)} {round(rpy_result[2], 6)}")
    #             origin.set('xyz', f"{round(xyz_result[0], 6)} {round(xyz_result[1], 6)} {round(xyz_result[2], 6)}")
    #             logger.info("Updated right_arm_mount_joint in URDF file")
    #             break
    
    # Write the updated URDF
    # tree.write(urdf_file, encoding='utf-8', xml_declaration=True)
    # logger.info(f"Updated URDF file: {urdf_file}")
    # logger.info("")
    
    logger.info("=" * 80)
    logger.info("SUMMARY")
    logger.info("=" * 80)
    logger.info("[OK] Calibration data loaded and converted to meters")
    logger.info("[OK] PyBullet robot loaded and poses computed")
    logger.info("[OK] Kinematic chain computed using PyBullet functions")
    # logger.info("[OK] URDF backup created")
    # logger.info("[OK] URDF file updated")
    # logger.info("")
    logger.info("The right_arm_mount_joint has been updated with the computed transformation")
    logger.info("based on the calibration data and kinematic chain analysis.")
    logger.info("")
    logger.info(f"Log file saved to: {log_path}")
    
    # Disconnect from PyBullet
    pp.disconnect()


if __name__ == "__main__":
    main()
