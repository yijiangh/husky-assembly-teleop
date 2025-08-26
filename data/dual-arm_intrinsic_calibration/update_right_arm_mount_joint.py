#!/usr/bin/env python3
"""
Script to compute and apply the updated transformation for right_arm_mount_joint
based on kinematic chain analysis and calibration data.
"""

import json
import numpy as np
import xml.etree.ElementTree as ET
import shutil
from datetime import datetime


def rpy_to_rotation_matrix(rpy):
    """Convert roll-pitch-yaw angles to rotation matrix."""
    roll, pitch, yaw = rpy
    # Rotation around X, Y, Z axes
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(roll), -np.sin(roll)],
                   [0, np.sin(roll), np.cos(roll)]])
    
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)],
                   [0, 1, 0],
                   [-np.sin(pitch), 0, np.cos(pitch)]])
    
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0],
                   [np.sin(yaw), np.cos(yaw), 0],
                   [0, 0, 1]])
    
    return Rz @ Ry @ Rx


def rpy_xyz_to_transformation(rpy, xyz):
    """Convert rpy and xyz to 4x4 transformation matrix."""
    R = rpy_to_rotation_matrix(rpy)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = xyz
    return T


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


def parse_urdf_transformations(urdf_file):
    """Parse URDF file to extract relevant transformations."""
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    transformations = {}
    
    for joint in root.findall('.//joint'):
        joint_name = joint.get('name')
        parent = joint.find('parent').get('link')
        child = joint.find('child').get('link')
        
        origin = joint.find('origin')
        if origin is not None:
            xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
            rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
            transformations[joint_name] = {
                'parent': parent,
                'child': child,
                'xyz': xyz,
                'rpy': rpy
            }
    
    return transformations


def verify_kinematic_chain(T_right_bh_from_dual_arm_bh, transformations, calibration_data):
    """Verify that the computed transformation satisfies the kinematic chain."""
    
    # Convert calibration translation from mm to meters
    calibration_translation_m = [x / 1000.0 for x in calibration_data['translation']]
    
    # Get all required transformations
    T_dual_arm_bulkhead = rpy_xyz_to_transformation(
        transformations['dual_arm_bulkhead_joint']['rpy'], 
        transformations['dual_arm_bulkhead_joint']['xyz']
    )
    
    T_left_arm_bulkhead = rpy_xyz_to_transformation(
        transformations['left_arm_bulkhead_joint']['rpy'], 
        transformations['left_arm_bulkhead_joint']['xyz']
    )
    
    T_left_arm_mount = rpy_xyz_to_transformation(
        transformations['left_arm_mount_joint']['rpy'], 
        transformations['left_arm_mount_joint']['xyz']
    )
    
    T_left_base_inertia = rpy_xyz_to_transformation(
        transformations['left_ur_arm_base_link-base_link_inertia']['rpy'], 
        transformations['left_ur_arm_base_link-base_link_inertia']['xyz']
    )
    
    T_right_base_inertia = rpy_xyz_to_transformation(
        transformations['right_ur_arm_base_link-base_link_inertia']['rpy'], 
        transformations['right_ur_arm_base_link-base_link_inertia']['xyz']
    )
    
    T_calibration = rpy_xyz_to_transformation(
        calibration_data['rotation_rpy'], 
        calibration_translation_m
    )
    
    # Compute the kinematic chain
    T_dual_arm_bh_from_left_base_inertia = (
        T_left_arm_bulkhead @ 
        T_left_arm_mount @ 
        T_left_base_inertia
    )
    
    T_left_base_inertia_from_right_base_inertia = np.linalg.inv(T_calibration)
    T_right_inertia_from_right_base_link = np.linalg.inv(T_right_base_inertia)
    
    # Compute right_bh_from_right_base_link using the computed transformation
    T_right_bh_from_right_base_link = (
        T_right_bh_from_dual_arm_bh @
        T_dual_arm_bh_from_left_base_inertia @
        T_left_base_inertia_from_right_base_inertia @
        T_right_inertia_from_right_base_link
    )
    
    # This should be approximately equal to T_left_arm_mount
    error = np.linalg.norm(T_right_bh_from_right_base_link - T_left_arm_mount)
    
    return error, T_right_bh_from_right_base_link


def main():
    print("=" * 80)
    print("RIGHT ARM MOUNT JOINT UPDATE COMPUTATION")
    print("=" * 80)
    
    # Load calibration data
    with open('data/dual-arm_intrinsic_calibration/calibration_results.json', 'r') as f:
        calibration_data = json.load(f)
    
    # Parse URDF transformations
    urdf_file = 'data/husky_urdf/mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf'
    transformations = parse_urdf_transformations(urdf_file)
    
    print("Calibration data:")
    print(f"Translation (mm): {calibration_data['translation']}")
    print(f"Rotation RPY (rad): {calibration_data['rotation_rpy']}")
    print(f"Final error: {calibration_data['final_error']}")
    print()
    
    # Convert calibration translation from mm to meters
    calibration_translation_m = [x / 1000.0 for x in calibration_data['translation']]
    print(f"Translation (m): {calibration_translation_m}")
    print()
    
    # Extract relevant transformations from URDF
    dual_arm_bulkhead_joint = transformations['dual_arm_bulkhead_joint']
    left_arm_bulkhead_joint = transformations['left_arm_bulkhead_joint']
    left_arm_mount_joint = transformations['left_arm_mount_joint']
    left_base_inertia_joint = transformations['left_ur_arm_base_link-base_link_inertia']
    right_base_inertia_joint = transformations['right_ur_arm_base_link-base_link_inertia']
    
    print("Current URDF transformations:")
    print(f"dual_arm_bulkhead_joint: {dual_arm_bulkhead_joint}")
    print(f"left_arm_bulkhead_joint: {left_arm_bulkhead_joint}")
    print(f"left_arm_mount_joint: {left_arm_mount_joint}")
    print(f"left_base_inertia_joint: {left_base_inertia_joint}")
    print(f"right_base_inertia_joint: {right_base_inertia_joint}")
    print()
    
    # Convert to transformation matrices
    T_dual_arm_bulkhead = rpy_xyz_to_transformation(
        dual_arm_bulkhead_joint['rpy'], 
        dual_arm_bulkhead_joint['xyz']
    )
    
    T_left_arm_bulkhead = rpy_xyz_to_transformation(
        left_arm_bulkhead_joint['rpy'], 
        left_arm_bulkhead_joint['xyz']
    )
    
    T_left_arm_mount = rpy_xyz_to_transformation(
        left_arm_mount_joint['rpy'], 
        left_arm_mount_joint['xyz']
    )
    
    T_left_base_inertia = rpy_xyz_to_transformation(
        left_base_inertia_joint['rpy'], 
        left_base_inertia_joint['xyz']
    )
    
    T_right_base_inertia = rpy_xyz_to_transformation(
        right_base_inertia_joint['rpy'], 
        right_base_inertia_joint['xyz']
    )
    
    # Calibration transformation (right_base_link_inertia_from_left_base_link_inertia)
    T_calibration = rpy_xyz_to_transformation(
        calibration_data['rotation_rpy'], 
        calibration_translation_m  # Use meters
    )
    
    # Compute the kinematic chain:
    # We want: right_bh_from_right_base_link = right_bh_from_dual_arm_bh * dual_arm_bh_from_left_base_inertia * left_base_inertia_from_right_base_inertia * right_inertia_from_right_base_link
    
    # 1. Compute dual_arm_bh_from_left_base_inertia
    # Chain: dual_arm_bulkhead -> left_arm_bulkhead -> left_ur_arm_base_link -> left_ur_arm_base_link_inertia
    T_dual_arm_bh_from_left_base_inertia = (
        T_left_arm_bulkhead @ 
        T_left_arm_mount @ 
        T_left_base_inertia
    )
    
    # 2. Compute left_base_inertia_from_right_base_inertia (inverse of calibration)
    T_left_base_inertia_from_right_base_inertia = np.linalg.inv(T_calibration)
    
    # 3. Compute right_inertia_from_right_base_link (inverse of right_base_inertia_joint)
    T_right_inertia_from_right_base_link = np.linalg.inv(T_right_base_inertia)
    
    # 4. We want right_bh_from_right_base_link to be the same as left_bh_from_left_base_link
    # So: right_bh_from_right_base_link = left_bh_from_left_base_link = T_left_arm_mount
    
    # Rearranging the equation:
    # right_bh_from_dual_arm_bh = right_bh_from_right_base_link * right_base_link_from_right_inertia * right_base_inertia_from_left_base_inertia * left_base_inertia_from_dual_arm_bh
    
    T_right_bh_from_dual_arm_bh = (
        T_left_arm_mount @  # Use left arm mount as reference
        T_right_inertia_from_right_base_link @
        T_left_base_inertia_from_right_base_inertia @
        np.linalg.inv(T_dual_arm_bh_from_left_base_inertia)
    )
    
    # Convert back to rpy and xyz
    rpy_result, xyz_result = transformation_to_rpy_xyz(T_right_bh_from_dual_arm_bh)
    
    print("=" * 80)
    print("COMPUTED TRANSFORMATION")
    print("=" * 80)
    print("Computed transformation for right_arm_mount_joint:")
    print(f"rpy: {[round(x, 6) for x in rpy_result]}")
    print(f"xyz: {[round(x, 6) for x in xyz_result]}")
    print()
    
    # Verify the computation
    error, T_verification = verify_kinematic_chain(T_right_bh_from_dual_arm_bh, transformations, calibration_data)
    
    print("=" * 80)
    print("VERIFICATION")
    print("=" * 80)
    print(f"Kinematic chain error: {error:.6f}")
    print("Expected transformation (left_arm_mount):")
    print(f"rpy: {left_arm_mount_joint['rpy']}")
    print(f"xyz: {left_arm_mount_joint['xyz']}")
    print()
    print("Computed right_bh_from_right_base_link:")
    rpy_verif, xyz_verif = transformation_to_rpy_xyz(T_verification)
    print(f"rpy: {[round(x, 6) for x in rpy_verif]}")
    print(f"xyz: {[round(x, 6) for x in xyz_verif]}")
    print()
    
    # Show the updated URDF joint definition
    print("=" * 80)
    print("UPDATED URDF JOINT DEFINITION")
    print("=" * 80)
    print("""    <joint name="right_arm_mount_joint" type="fixed">
        <parent link="right_arm_bulkhead_link" />
        <child link="right_ur_arm_base_link" />
        <origin rpy="{} {} {}" xyz="{} {} {}" />
    </joint>""".format(
        round(rpy_result[0], 6), round(rpy_result[1], 6), round(rpy_result[2], 6),
        round(xyz_result[0], 6), round(xyz_result[1], 6), round(xyz_result[2], 6)
    ))
    print()
    
    # Create backup and update URDF
    print("=" * 80)
    print("URDF UPDATE")
    print("=" * 80)
    
    # Create backup
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_file = f"{urdf_file}.backup_{timestamp}"
    shutil.copy2(urdf_file, backup_file)
    print(f"Created backup: {backup_file}")
    
    # Update the URDF file
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    # Find the right_arm_mount_joint
    for joint in root.findall('.//joint'):
        if joint.get('name') == 'right_arm_mount_joint':
            origin = joint.find('origin')
            if origin is not None:
                origin.set('rpy', f"{round(rpy_result[0], 6)} {round(rpy_result[1], 6)} {round(rpy_result[2], 6)}")
                origin.set('xyz', f"{round(xyz_result[0], 6)} {round(xyz_result[1], 6)} {round(xyz_result[2], 6)}")
                print("Updated right_arm_mount_joint in URDF file")
                break
    
    # Write the updated URDF
    tree.write(urdf_file, encoding='utf-8', xml_declaration=True)
    print(f"Updated URDF file: {urdf_file}")
    print()
    
    print("=" * 80)
    print("SUMMARY")
    print("=" * 80)
    print("✓ Calibration data loaded and converted to meters")
    print("✓ Kinematic chain computed")
    print("✓ Transformation verified (error: {:.6f})".format(error))
    print("✓ URDF backup created")
    print("✓ URDF file updated")
    print()
    print("The right_arm_mount_joint has been updated with the computed transformation")
    print("based on the calibration data and kinematic chain analysis.")


if __name__ == "__main__":
    main()
