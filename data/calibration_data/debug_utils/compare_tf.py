#!/usr/bin/env python3
"""
Transformation Matrix Comparison Script

Compare two 4x4 transformation matrices and compute:
- Position offsets
- Rotation angle differences
- Axis alignment deviations
"""

import os
import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def parse_format1(lines):
    """
    Parse format like:
    tf[:3, 0] = [x, y, z]
    tf[:3, 1] = [x, y, z]
    ...
    """
    matrix = np.eye(4)
    for line in lines:
        if 'tf[:3,' in line:
            # Extract column index and values
            col_idx = int(line.split('tf[:3,')[1].split(']')[0].strip())
            values_str = line.split('=')[1].strip()
            values = eval(values_str)  # Safe here since we control input
            matrix[:3, col_idx] = values
    return matrix


def parse_format2(matrix_list):
    """
    Parse format like nested list:
    [[...], [...], [...], [...]]
    """
    return np.array(matrix_list)


def rotation_matrix_to_euler(R):
    """
    Convert rotation matrix to Euler angles (ZYX convention)
    Returns angles in degrees
    """
    sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
    
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0
    
    return np.degrees([x, y, z])


def rotation_matrix_to_axis_angle(R):
    """
    Convert rotation matrix to axis-angle representation
    Returns axis (unit vector) and angle in degrees
    """
    angle = np.arccos((np.trace(R) - 1) / 2)
    
    if np.abs(angle) < 1e-6:
        return np.array([0, 0, 1]), 0.0
    
    axis = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ]) / (2 * np.sin(angle))
    
    return axis, np.degrees(angle)


def angle_between_vectors(v1, v2):
    """
    Compute angle between two vectors in degrees
    """
    v1_norm = v1 / np.linalg.norm(v1)
    v2_norm = v2 / np.linalg.norm(v2)
    cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1.0, 1.0)
    return np.degrees(np.arccos(cos_angle))


def compare_transforms(tf1, tf2):
    """
    Compare two transformation matrices and print detailed analysis
    """
    print("=" * 70)
    print("TRANSFORMATION MATRIX COMPARISON")
    print("=" * 70)
    
    # Extract rotation and translation
    R1 = tf1[:3, :3]
    R2 = tf2[:3, :3]
    t1 = tf1[:3, 3]
    t2 = tf2[:3, 3]
    
    # Position comparison
    print("\n📍 POSITION COMPARISON")
    print("-" * 70)
    print(f"Transform 1 position: [{t1[0]:9.6f}, {t1[1]:9.6f}, {t1[2]:9.6f}]")
    print(f"Transform 2 position: [{t2[0]:9.6f}, {t2[1]:9.6f}, {t2[2]:9.6f}]")
    
    position_diff = t2 - t1
    position_distance = np.linalg.norm(position_diff)
    print(f"\nPosition offset:      [{position_diff[0]:9.6f}, {position_diff[1]:9.6f}, {position_diff[2]:9.6f}]")
    print(f"Euclidean distance:   {position_distance:.6f} units")
    
    # Euler angles comparison
    print("\n🔄 ROTATION COMPARISON (Euler Angles ZYX)")
    print("-" * 70)
    euler1 = rotation_matrix_to_euler(R1)
    euler2 = rotation_matrix_to_euler(R2)
    
    print(f"Transform 1 (deg):    Roll={euler1[0]:7.3f}°, Pitch={euler1[1]:7.3f}°, Yaw={euler1[2]:7.3f}°")
    print(f"Transform 2 (deg):    Roll={euler2[0]:7.3f}°, Pitch={euler2[1]:7.3f}°, Yaw={euler2[2]:7.3f}°")
    
    euler_diff = euler2 - euler1
    print(f"\nEuler difference:     Roll={euler_diff[0]:7.3f}°, Pitch={euler_diff[1]:7.3f}°, Yaw={euler_diff[2]:7.3f}°")
    
    # Axis-angle comparison
    print("\n⚡ ROTATION COMPARISON (Axis-Angle)")
    print("-" * 70)
    
    # Compute relative rotation
    R_relative = R2 @ R1.T
    axis, angle = rotation_matrix_to_axis_angle(R_relative)
    
    print(f"Relative rotation axis:  [{axis[0]:7.4f}, {axis[1]:7.4f}, {axis[2]:7.4f}]")
    print(f"Relative rotation angle: {angle:.4f}°")
    
    # Axis alignment comparison
    print("\n📐 AXIS ALIGNMENT DEVIATIONS")
    print("-" * 70)
    
    x_axis_angle = angle_between_vectors(R1[:, 0], R2[:, 0])
    y_axis_angle = angle_between_vectors(R1[:, 1], R2[:, 1])
    z_axis_angle = angle_between_vectors(R1[:, 2], R2[:, 2])
    
    print(f"X-axis deviation: {x_axis_angle:7.4f}°")
    print(f"Y-axis deviation: {y_axis_angle:7.4f}°")
    print(f"Z-axis deviation: {z_axis_angle:7.4f}°")
    print(f"Mean deviation:   {np.mean([x_axis_angle, y_axis_angle, z_axis_angle]):7.4f}°")
    
    # Frobenius norm of rotation difference
    R_diff_norm = np.linalg.norm(R2 - R1, 'fro')
    print(f"\nRotation matrix Frobenius norm difference: {R_diff_norm:.6f}")
    
    return {
        'position_diff': position_diff,
        'position_distance': position_distance,
        'euler_diff': euler_diff,
        'relative_axis': axis,
        'relative_angle': angle,
        'axis_deviations': [x_axis_angle, y_axis_angle, z_axis_angle],
        'R1': R1,
        'R2': R2,
        't1': t1,
        't2': t2
    }


def visualize_transforms(tf1, tf2, results, save_path=None):
    """
    Create visualization of the two transformation matrices
    """
    fig = plt.figure(figsize=(15, 5))
    
    # 3D visualization of coordinate frames
    ax1 = fig.add_subplot(131, projection='3d')
    
    R1, R2 = results['R1'], results['R2']
    t1, t2 = results['t1'], results['t2']
    
    # Scale for axis vectors
    scale = 0.1
    
    # Transform 1 axes (red, green, blue)
    colors1 = ['darkred', 'darkgreen', 'darkblue']
    for i, color in enumerate(colors1):
        ax1.quiver(t1[0], t1[1], t1[2], 
                   R1[0, i]*scale, R1[1, i]*scale, R1[2, i]*scale,
                   color=color, arrow_length_ratio=0.3, linewidth=2, 
                   label=f'TF1 {"XYZ"[i]}-axis')
    
    # Transform 2 axes (lighter red, green, blue)
    colors2 = ['salmon', 'lightgreen', 'lightblue']
    for i, color in enumerate(colors2):
        ax1.quiver(t2[0], t2[1], t2[2], 
                   R2[0, i]*scale, R2[1, i]*scale, R2[2, i]*scale,
                   color=color, arrow_length_ratio=0.3, linewidth=2,
                   label=f'TF2 {"XYZ"[i]}-axis', linestyle='--')

    # Draw the unit/reference pose axes at the origin as a visual reference
    origin = np.zeros(3)
    ref_scale = 0.12
    ref_colors = ['salmon', 'lightgreen', 'lightblue'] # ['gray', 'gray', 'gray']
    for i, color in enumerate(ref_colors):
        ax1.quiver(origin[0], origin[1], origin[2],
                   (np.eye(3)[0, i])*ref_scale,
                   (np.eye(3)[1, i])*ref_scale,
                   (np.eye(3)[2, i])*ref_scale,
                   color=color, arrow_length_ratio=0.2, linewidth=1.3,
                   alpha=0.8, linestyle=':', label=f'Ref {"XYZ"[i]}-axis')
    
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('Coordinate Frames Visualization')
    ax1.legend(fontsize=8)
    
    # Set equal aspect ratio
    max_range = np.max([np.abs(t1).max(), np.abs(t2).max(), scale]) * 1.2
    ax1.set_xlim([np.mean([t1[0], t2[0]]) - max_range, np.mean([t1[0], t2[0]]) + max_range])
    ax1.set_ylim([np.mean([t1[1], t2[1]]) - max_range, np.mean([t1[1], t2[1]]) + max_range])
    ax1.set_zlim([np.mean([t1[2], t2[2]]) - max_range, np.mean([t1[2], t2[2]]) + max_range])
    
    # Bar chart for axis deviations
    ax2 = fig.add_subplot(132)
    axes_labels = ['X-axis', 'Y-axis', 'Z-axis']
    deviations = results['axis_deviations']
    bars = ax2.bar(axes_labels, deviations, color=['red', 'green', 'blue'], alpha=0.7)
    ax2.set_ylabel('Angle Deviation (degrees)')
    ax2.set_title('Axis Alignment Deviations')
    ax2.grid(axis='y', alpha=0.3)
    
    # Add value labels on bars
    for bar, dev in zip(bars, deviations):
        height = bar.get_height()
        ax2.text(bar.get_x() + bar.get_width()/2., height,
                f'{dev:.3f}°', ha='center', va='bottom')
    
    # Position offset visualization
    ax3 = fig.add_subplot(133)
    offset_labels = ['X', 'Y', 'Z']
    offsets_mm = [v * 1000 for v in results['position_diff']]
    colors = ['red', 'green', 'blue']
    bars = ax3.bar(offset_labels, offsets_mm, color=colors, alpha=0.7)
    ax3.set_ylabel('Position Offset (mm)')
    ax3.set_title('Position Differences')
    ax3.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
    ax3.grid(axis='y', alpha=0.3)

    # Add value labels on bars
    for bar, off in zip(bars, offsets_mm):
        height = bar.get_height()
        va = 'bottom' if height >= 0 else 'top'
        ax3.text(bar.get_x() + bar.get_width()/2., height,
                f'{off:.2f}', ha='center', va=va)
    
    fig.suptitle('base_mocap_from_arm_base_link: comparison between Python script and GH script', fontsize=12)
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"\nVisualization saved to '{save_path}'")
    plt.show()


# ============================================================================
# MAIN SCRIPT
# ============================================================================

# Add parent directory to path so we can import config_loader
import sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from config_loader import load_config, HERE as CALIBRATION_DATA_DIR


def load_tf_from_json(json_path):
    """Load a 4x4 transformation matrix from a calibration JSON file.

    Supports both key names: 'base_mocap_from_arm_base_link' and
    'base_frame_transformation_matrix'.
    """
    with open(json_path, 'r') as f:
        data = json.load(f)

    if 'base_mocap_from_arm_base_link' in data:
        matrix = data['base_mocap_from_arm_base_link']
    else:
        matrix = data['base_frame_transformation_matrix']

    return parse_format2(matrix)


if __name__ == "__main__":

    # ========== CONFIGURATION ==========
    config = load_config()
    DATE_FOLDER = config['date_folder']

    # ========== TRANSFORM 1: GH generated ==========
    gh_file = os.path.join(CALIBRATION_DATA_DIR, DATE_FOLDER, 'base_frame_calibration_GH.json')
    tf1 = load_tf_from_json(gh_file)
    print(f"Loaded GH-generated tf from: {gh_file}")

    # ========== TRANSFORM 2: Python generated ==========
    py_file = os.path.join(CALIBRATION_DATA_DIR, DATE_FOLDER, 'base_frame_calibration.json')
    tf2 = load_tf_from_json(py_file)
    print(f"Loaded Python-generated tf from: {py_file}")

    # ========== RUN COMPARISON ==========
    results = compare_transforms(tf1, tf2)

    # Save figure to the date folder
    save_path = os.path.join(CALIBRATION_DATA_DIR, DATE_FOLDER, 'compare_gh-py_tf.png')
    visualize_transforms(tf1, tf2, results, save_path=save_path)

    print("\nAnalysis complete!")