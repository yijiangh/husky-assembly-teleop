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
import pybullet as p

from config_loader import (
    load_config, get_robot_urdf, get_joint_names, 
    get_tool0_link_name, HERE
)

# Load configuration
config = load_config()
DATE_FOLDER = config['date_folder']
VALIDATION_DATA_BATCH = config.get('validation_data_batch', config['data_batches'][0])  # Use validation batch from config
ROBOT_NAME = config['robot_name']
ARM = config['arm']
USE_GUI = False # config['use_gui']

# File paths
VALIDATION_DATA_FOLDER = os.path.join(HERE, DATE_FOLDER, VALIDATION_DATA_BATCH)
CALIBRATION_FILE = os.path.join(HERE, DATE_FOLDER, f'calibrated_transformation_{ROBOT_NAME}.json')
ROBOT_URDF = get_robot_urdf(ROBOT_NAME)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger()

file_handler = logging.FileHandler(os.path.join(VALIDATION_DATA_FOLDER, 'verification_log.txt'), mode='w')
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
                'flange_mocap_pose': flange_mocap_pose,
                'base_mocap_pose': base_mocap_pose
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


def analyze_results(results, data_folder, data_batch, date_folder, robot_name, arm):
    """Analyze the consistency of tool0-to-flange offset across all samples."""
    
    # Extract positions and orientations
    positions = [r['tool0_from_flange_mocap'][0] for r in results]
    quaternions = [r['tool0_from_flange_mocap'][1] for r in results]
    
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
    
    # Compute angular deviations per axis from mean
    angular_deviations = {'X': [], 'Y': [], 'Z': []}
    axis_means = {}
    for axis_idx, axis_name in enumerate(['X', 'Y', 'Z']):
        axes_data = np.array([rm[:, axis_idx] for rm in rotation_matrices])
        axis_mean = np.mean(axes_data, axis=0)
        axis_mean = axis_mean / np.linalg.norm(axis_mean)
        axis_means[axis_name] = axis_mean
        angular_deviations[axis_name] = np.array([
            np.rad2deg(np.arccos(np.clip(np.dot(a, axis_mean), -1, 1))) for a in axes_data
        ])

    # Extract joint configurations and base poses for diversity plots
    joint_confs = np.array([r['joint_conf'] for r in results])
    base_positions = np.array([r['base_mocap_pose'][0] for r in results])
    base_quaternions = [r['base_mocap_pose'][1] for r in results]

    # Extract yaw angles from base quaternions
    def quat_to_yaw(q):
        """Extract yaw angle from quaternion [x, y, z, w]."""
        x, y, z, w = q
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return np.rad2deg(np.arctan2(siny_cosp, cosy_cosp))

    base_yaws = np.array([quat_to_yaw(q) for q in base_quaternions])

    # Common title info
    title_info = f'Date: {date_folder} | Robot: {robot_name} | Arm: {arm} | Batch: {data_batch} | Samples: {len(results)}'
    sample_indices = np.arange(len(results))

    # ========== Figure 1: Calibration Verification (2x2) ==========
    fig1, axes1 = plt.subplots(2, 2, figsize=(14, 10))
    fig1.suptitle(f'Calibration Verification | {title_info}', fontsize=12, fontweight='bold')

    # Plot 1: Sample index vs position offset from mean
    ax1 = axes1[0, 0]
    ax1.scatter(sample_indices, pos_distances, alpha=0.7, s=20)
    ax1.set_xlabel('Sample Index')
    ax1.set_ylabel('Position Offset from Mean (mm)')
    ax1.set_title(f'Position Offset from Mean\nMean Offset: {np.mean(pos_distances):.3f} mm')
    ax1.grid(True, alpha=0.3)

    # Plot 2: Sample index vs angular deviation per axis from mean
    ax2 = axes1[0, 1]
    colors = ['r', 'g', 'b']
    mean_angles = []
    for axis_name, color in zip(['X', 'Y', 'Z'], colors):
        ax2.plot(sample_indices, angular_deviations[axis_name], c=color, alpha=0.8, linewidth=2,
                 label=f'{axis_name}-axis')
        mean_angles.append(np.mean(angular_deviations[axis_name]))
    ax2.set_xlabel('Sample Index')
    ax2.set_ylabel('Angular Deviation from Mean (deg)')
    ax2.set_title(f'Angular Deviation per Axis from Mean\nMean: X={mean_angles[0]:.3f}°, Y={mean_angles[1]:.3f}°, Z={mean_angles[2]:.3f}°')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Plot 3: CDF for position offset with 95% cutoff
    ax3 = axes1[1, 0]
    sorted_pos = np.sort(pos_distances)
    cdf_pos = np.arange(1, len(sorted_pos) + 1) / len(sorted_pos)
    ax3.plot(sorted_pos, cdf_pos, 'b-', linewidth=2)
    pos_95 = np.percentile(pos_distances, 95)
    ax3.axhline(y=0.95, color='r', linestyle='--', label='95% cutoff')
    ax3.axvline(x=pos_95, color='r', linestyle='--')
    ax3.plot(pos_95, 0.95, 'ro', markersize=8)
    ax3.annotate(f'{pos_95:.2f} mm', xy=(pos_95, 0.95), xytext=(-40, -30),
                textcoords='offset points', fontsize=10, color='r',
                arrowprops=dict(arrowstyle='->', color='r', alpha=0.7))
    ax3.set_xlabel('Position Offset (mm)')
    ax3.set_ylabel('CDF')
    ax3.set_title(f'Position Offset CDF\n95th percentile: {pos_95:.3f} mm | Goal 95% under 5mm')
    ax3.legend()
    ax3.grid(True, alpha=0.3)

    # Plot 4: CDF for angular deviation per axis with 95% cutoff
    ax4 = axes1[1, 1]
    for axis_name, color in zip(['X', 'Y', 'Z'], colors):
        sorted_ang = np.sort(angular_deviations[axis_name])
        cdf_ang = np.arange(1, len(sorted_ang) + 1) / len(sorted_ang)
        ang_95 = np.percentile(angular_deviations[axis_name], 95)
        ax4.plot(sorted_ang, cdf_ang, color=color, linewidth=2, label=f'{axis_name}-axis (95%: {ang_95:.3f}°)')
        ax4.plot(ang_95, 0.95, 'o', color=color, markersize=6)
    ax4.axhline(y=0.95, color='gray', linestyle='--', alpha=0.7, label='95% cutoff')
    ax4.set_xlabel('Angular Deviation (deg)')
    ax4.set_ylabel('CDF')
    ax4.set_title('Angular Deviation CDF per Axis | Goal 95% under 0.3°')
    ax4.legend()
    ax4.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    output_path1 = os.path.join(data_folder, f'verification_{data_batch}.png')
    plt.savefig(output_path1, dpi=150)
    logger.info('Verification plot saved to: %s', output_path1)
    plt.show()
    plt.close(fig1)

    # ========== Figure 2: Data Diversity (1x2) ==========
    fig2, axes2 = plt.subplots(1, 2, figsize=(14, 5))
    fig2.suptitle(f'Data Diversity | {title_info}', fontsize=12, fontweight='bold')

    # Plot 5: Joint configuration diversity (box plots for 6 DOF)
    ax5 = axes2[0]
    joint_labels = [f'J{i+1}' for i in range(joint_confs.shape[1])]
    joint_data_deg = np.rad2deg(joint_confs)  # Convert to degrees for readability
    bp = ax5.boxplot(joint_data_deg, labels=joint_labels, patch_artist=True)
    joint_colors = plt.cm.viridis(np.linspace(0.2, 0.8, len(joint_labels)))
    for patch, color in zip(bp['boxes'], joint_colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    ax5.set_xlabel('Joint')
    ax5.set_ylabel('Joint Angle (deg)')
    joint_ranges = [f'{np.min(joint_data_deg[:, i]):.1f}~{np.max(joint_data_deg[:, i]):.1f}'
                   for i in range(joint_confs.shape[1])]
    ax5.set_title(f'Joint Configuration Diversity\nRanges (deg): {", ".join(joint_ranges)}')
    ax5.grid(True, alpha=0.3, axis='y')

    # Plot 6: Robot base position (X-Y scatter) with arrows for yaw orientation
    ax6 = axes2[1]
    base_x_mm = base_positions[:, 0] * 1000
    base_y_mm = base_positions[:, 1] * 1000
    # Compute arrow directions from yaw angles
    yaw_rad = np.deg2rad(base_yaws)
    arrow_dx = np.cos(yaw_rad)
    arrow_dy = np.sin(yaw_rad)
    # Plot points
    ax6.scatter(base_x_mm, base_y_mm, c='blue', alpha=0.5, s=20, zorder=2)
    # Plot arrows indicating orientation
    arrow_scale = max(np.ptp(base_x_mm), np.ptp(base_y_mm)) * 0.08  # Scale arrows relative to plot range
    ax6.quiver(base_x_mm, base_y_mm, arrow_dx * arrow_scale, arrow_dy * arrow_scale,
               angles='xy', scale_units='xy', scale=1, color='red', alpha=0.6,
               width=0.003, headwidth=3, headlength=4, zorder=3)
    ax6.set_xlabel('Base X (mm)')
    ax6.set_ylabel('Base Y (mm)')
    x_range = np.ptp(base_x_mm)
    y_range = np.ptp(base_y_mm)
    yaw_range = np.ptp(base_yaws)
    ax6.set_title(f'Base Position & Yaw Diversity\nX range: {x_range:.1f}mm, Y range: {y_range:.1f}mm, Yaw range: {yaw_range:.1f}°')
    ax6.set_aspect('equal', adjustable='datalim')
    ax6.grid(True, alpha=0.3)

    plt.tight_layout(rect=[0, 0, 1, 0.92])
    output_path2 = os.path.join(data_folder, f'diversity_{data_batch}.png')
    plt.savefig(output_path2, dpi=150)
    logger.info('Diversity plot saved to: %s', output_path2)
    plt.show()
    plt.close(fig2)


def main():
    logger.info('=' * 60)
    logger.info('Calibration Verification')
    logger.info('=' * 60)
    logger.info('Validation data batch: %s', VALIDATION_DATA_BATCH)
    logger.info('Robot: %s', ROBOT_NAME)
    
    # Initialize PyBullet
    pp.connect(use_gui=USE_GUI, shadows=True, color=[0.9, 0.9, 1.0])
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=pp.CLIENT)

    # INSERT_YOUR_CODE
    # Add a plane for ground visualization in PyBullet
    # with pp.HideOutput():
    #     plane = pp.create_plane(color=[0.85, 0.85, 0.85, 1.0])  # Light gray ground plane
    
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
    results = compute_tool0_flange_offset(robot, arm_joints, tool0_link_name, calibration_data, VALIDATION_DATA_FOLDER)
    logger.info('Processed %d samples', len(results))
    
    if USE_GUI:
        pp.wait_if_gui('Processing complete')
    
    # Analyze results
    analyze_results(results, VALIDATION_DATA_FOLDER, VALIDATION_DATA_BATCH, DATE_FOLDER, ROBOT_NAME, ARM)
    
    pp.disconnect()
    
    logger.info('=' * 60)
    logger.info('Verification complete!')
    logger.info('=' * 60)


if __name__ == '__main__':
    main()
