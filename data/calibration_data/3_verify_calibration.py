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
import numpy as np
import matplotlib.pyplot as plt
import pybullet_planning as pp
import pybullet as p

from config_loader import (
    load_config, get_robot_urdf, get_joint_names,
    get_tool0_link_name, HERE
)
from logging_utils import setup_logger

# Load configuration
config = load_config()
DATE_FOLDER = config['date_folder']
VALIDATION_DATA_BATCH = config.get('validation_data_batch', config['data_batches'][0])  # Use validation batch from config
ROBOT_NAME = config['robot_name']
ARM = config['arm']
USE_GUI = config['use_gui']

# File paths
VALIDATION_DATA_FOLDER = os.path.join(HERE, DATE_FOLDER, VALIDATION_DATA_BATCH)
CALIBRATION_FILE = os.path.join(HERE, DATE_FOLDER, f'calibrated_transformation_{ROBOT_NAME}.json')
ROBOT_URDF = get_robot_urdf(ROBOT_NAME)

# Configure logging with colored console output
logger = setup_logger(log_file=os.path.join(VALIDATION_DATA_FOLDER, 'verification_log.txt'))


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
        
        raw_entries = data['raw_data']

        # Shift mocap by -1 (wrapped): joint_conf[i] pairs with mocap[i-1]
        # At i=0 this wraps to mocap[-1] (last entry)
        for i in range(len(raw_entries)):
            # if i == 0:
            #     continue  # Skip first entry due to wrap-around
            joint_conf_entry = raw_entries[i]
            mocap_entry = raw_entries[i]
            # mocap_entry = raw_entries[i - 1]

            flange_mocap_pose = mocap_entry.get("flange_mocap_pose", [])
            base_mocap_pose = mocap_entry.get("base_mocap_pose", [])
            joint_conf = joint_conf_entry.get("joint_conf", [])

            if not flange_mocap_pose or not base_mocap_pose or not joint_conf:
                logger.warning(
                    f"Missing data in {file_name}: "
                    f"flange_mocap_pose={bool(flange_mocap_pose)}, "
                    f"base_mocap_pose={bool(base_mocap_pose)}, "
                    f"joint_conf={bool(joint_conf)}"
                )
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
            offset_norm_mm = np.linalg.norm(tool0_from_flange_mocap[0]) * 1000
            world_dist_mm = np.linalg.norm(np.array(world_from_tool0[0]) - np.array(flange_mocap_pose[0])) * 1000
            logger.info(f'  Sample {i}: tool0_from_flange_mocap pos={tool0_from_flange_mocap[0][0]:.3f}, {tool0_from_flange_mocap[0][1]:.3f}, {tool0_from_flange_mocap[0][2]:.3f} m (norm={offset_norm_mm:.1f} mm) | world dist={world_dist_mm:.1f} mm')
            if abs(offset_norm_mm - world_dist_mm) > 0.1:
                logger.warning(f'  Sample {i}: MISMATCH! tool0_from_flange norm={offset_norm_mm:.1f} mm != world dist={world_dist_mm:.1f} mm (diff={abs(offset_norm_mm - world_dist_mm):.3f} mm)')

            results.append({
                'tool0_from_flange_mocap': tool0_from_flange_mocap,
                'joint_conf': joint_conf,
                'world_from_tool0': world_from_tool0,
                'flange_mocap_pose': flange_mocap_pose,
                'base_mocap_pose': base_mocap_pose,
                'file_name': file_name
            })
            
            # Visualization (if GUI enabled)
            if USE_GUI:
                pp.draw_pose(base_mocap_pose, length=0.1)
                pp.draw_pose(world_from_tool0, length=0.1)
                pp.draw_pose(flange_mocap_pose, length=0.1)
                # pp.add_text("tool0 (FK)", position=[p + 0.01 for p in world_from_tool0[0]])
                # pp.add_text("flange_mocap", position=[p + 0.01 for p in flange_mocap_pose[0]])
                
                pp.wait_if_gui('Visualization')
                pp.remove_all_debug()

    # pp.wait_if_gui('Visualization')

    return results


def plot_tool0_vs_flange_3d(results, data_folder, data_batch):
    """
    Draw a 3D debug plot showing tool0 (FK) origins vs flange_mocap origins,
    plus a 2D scatter of per-sample distances with dividing lines between circles.
    """
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    fig = plt.figure(figsize=(20, 9))
    ax3d = fig.add_subplot(121, projection='3d')
    ax2d = fig.add_subplot(122)

    tool0_positions = np.array([r['world_from_tool0'][0] for r in results])
    flange_positions = np.array([r['flange_mocap_pose'][0] for r in results])
    file_names = [r['file_name'] for r in results]

    # Compute distances
    distances_mm = np.linalg.norm(tool0_positions - flange_positions, axis=1) * 1000

    # Normalize distances for colormap
    dist_min, dist_max = distances_mm.min(), distances_mm.max()
    if dist_max > dist_min:
        norm_distances = (distances_mm - dist_min) / (dist_max - dist_min)
    else:
        norm_distances = np.zeros_like(distances_mm)

    cmap = plt.cm.coolwarm

    # ===== Left: 3D plot =====
    for i in range(len(results)):
        t0 = tool0_positions[i]
        fl = flange_positions[i]
        color = cmap(norm_distances[i])

        ax3d.plot([t0[0], fl[0]], [t0[1], fl[1]], [t0[2], fl[2]],
                  color=color, linewidth=1.5, alpha=0.7)
        ax3d.scatter(*t0, color='blue', s=20, alpha=0.6)
        ax3d.scatter(*fl, color='red', s=20, alpha=0.6)

        mid = (t0 + fl) / 2
        ax3d.text(mid[0], mid[1], mid[2], f'{distances_mm[i]:.1f}', fontsize=6, alpha=0.8)

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=dist_min, vmax=dist_max))
    sm.set_array([])
    fig.colorbar(sm, ax=ax3d, shrink=0.6, pad=0.1, label='Distance (mm)')

    ax3d.scatter([], [], [], color='blue', s=40, label='tool0 (FK)')
    ax3d.scatter([], [], [], color='red', s=40, label='flange_mocap')
    ax3d.legend(loc='upper left')

    # Set axis limits based on data extent with padding
    all_pts = np.vstack([tool0_positions, flange_positions])
    data_center = np.mean(all_pts, axis=0)
    data_extent = np.max(np.abs(all_pts - data_center)) * 1.3  # 30% padding
    ax3d.set_xlim(data_center[0] - data_extent, data_center[0] + data_extent)
    ax3d.set_ylim(data_center[1] - data_extent, data_center[1] + data_extent)
    ax3d.set_zlim(data_center[2] - data_extent, data_center[2] + data_extent)

    ax3d.set_xlabel('X (m)')
    ax3d.set_ylabel('Y (m)')
    ax3d.set_zlabel('Z (m)')
    ax3d.set_title(f'tool0 (FK) vs flange_mocap\nDistance range: {dist_min:.1f} - {dist_max:.1f} mm')

    # ===== Right: 2D scatter of distances with circle dividers =====
    sample_indices = np.arange(len(results))
    scatter = ax2d.scatter(sample_indices, distances_mm, c=distances_mm, cmap=cmap,
                           vmin=dist_min, vmax=dist_max, s=25, alpha=0.8)
    fig.colorbar(scatter, ax=ax2d, label='Distance (mm)')

    # Draw dashed lines between different calibration files (circles)
    boundaries = []
    for i in range(1, len(file_names)):
        if file_names[i] != file_names[i - 1]:
            boundaries.append(i - 0.5)
            ax2d.axvline(x=i - 0.5, color='gray', linestyle='--', linewidth=1, alpha=0.7)

    # Label each circle region at the top of the plot
    y_top = dist_max * 1.05
    region_starts = [0] + [int(b + 0.5) for b in boundaries]
    region_ends = [int(b + 0.5) for b in boundaries] + [len(results)]
    for start, end in zip(region_starts, region_ends):
        region_name = file_names[start].replace('calibration_', '').replace('.json', '')
        mid_x = (start + end - 1) / 2
        ax2d.text(mid_x, y_top, region_name, ha='center', va='bottom', fontsize=7, alpha=0.7)

    ax2d.set_xlabel('Sample Index')
    ax2d.set_ylabel('Distance (mm)')
    ax2d.set_title(f'Per-Sample Distance\nMean: {np.mean(distances_mm):.1f} mm | '
                   f'Median: {np.median(distances_mm):.1f} mm')
    ax2d.grid(True, alpha=0.3)

    fig.suptitle(f'Debug: tool0 vs flange_mocap | {data_batch} | {len(results)} samples',
                 fontsize=13, fontweight='bold')
    plt.tight_layout(rect=[0, 0, 1, 0.95])
    output_path = os.path.join(data_folder, f'debug_tool0_vs_flange_{data_batch}.png')
    plt.savefig(output_path, dpi=150)
    logger.info('Debug 3D plot saved to: %s', output_path)
    plt.show()
    plt.close(fig)


def analyze_results(results, data_folder, data_batch, date_folder, robot_name, arm):
    """Analyze the consistency of tool0-to-flange offset across all samples."""
    
    # Extract positions and orientations
    positions = [r['tool0_from_flange_mocap'][0] for r in results]
    quaternions = [r['tool0_from_flange_mocap'][1] for r in results]
    
    # Position analysis
    positions = np.array(positions)
    pos_mean = np.mean(positions, axis=0)
    pos_distances = np.linalg.norm(positions, axis=1) * 1000  # mm (distance to origin)
    pos_distances = pos_distances - np.average(pos_distances)  # center around mean distance
    
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
    
    # Compute angular deviations from mean axis
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
        logger.info('  %s-axis max angle from mean: %.3f deg', axis_name, np.max(angular_deviations[axis_name]))

    # ========== Debug: 3D orientation axes plot ==========
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    file_names = [r['file_name'] for r in results]
    unique_files = list(dict.fromkeys(file_names))

    fig_ori = plt.figure(figsize=(12, 10))
    ax_ori = fig_ori.add_subplot(111, projection='3d')

    axis_colors = ['r', 'g', 'b']  # X=red, Y=green, Z=blue
    axis_labels = ['X', 'Y', 'Z']
    unit_axes = np.eye(3)

    # Draw thick unit axes
    for axis_idx in range(3):
        direction = unit_axes[axis_idx]
        ax_ori.quiver(0, 0, 0, direction[0], direction[1], direction[2],
                      color=axis_colors[axis_idx], linewidth=4, arrow_length_ratio=0.1,
                      alpha=1.0, label=f'Unit {axis_labels[axis_idx]}')

    # Draw each sample's axes as thin arrows, colored by file with alpha
    file_color_map = {fname: plt.cm.tab10.colors[i % 10] for i, fname in enumerate(unique_files)}
    legend_added = set()

    OUTLIER_ANGLE_THRESHOLD = 170  # degrees
    outlier_count = 0

    for sample_idx, rm in enumerate(rotation_matrices):
        fname = file_names[sample_idx]
        file_color = file_color_map[fname]

        for axis_idx in range(3):
            col = rm[:, axis_idx]
            angle = angular_deviations[axis_labels[axis_idx]][sample_idx]
            is_outlier = angle >= OUTLIER_ANGLE_THRESHOLD

            if is_outlier:
                # Highlight outlier with thick line and full opacity
                ax_ori.quiver(0, 0, 0, col[0], col[1], col[2],
                              color='magenta', linewidth=3, alpha=0.9,
                              arrow_length_ratio=0.08)
                ax_ori.text(col[0] * 1.05, col[1] * 1.05, col[2] * 1.05,
                            f'#{sample_idx} {axis_labels[axis_idx]} {angle:.0f}°',
                            fontsize=6, color='magenta', fontweight='bold')
                outlier_count += 1
            else:
                ax_ori.quiver(0, 0, 0, col[0], col[1], col[2],
                              color=axis_colors[axis_idx], linewidth=0.5, alpha=0.15,
                              arrow_length_ratio=0.05)

        # Add file to legend via invisible scatter (once per file)
        if fname not in legend_added:
            file_label = fname.replace('calibration_', '').replace('.json', '')
            ax_ori.scatter([], [], [], color=file_color, s=30, label=file_label)
            legend_added.add(fname)

    # Also scatter the tips of each axis column to see clustering
    for axis_idx in range(3):
        tips = np.array([rm[:, axis_idx] for rm in rotation_matrices])
        for file_idx, fname in enumerate(unique_files):
            mask = np.array([f == fname for f in file_names])
            file_color = file_color_map[fname]
            ax_ori.scatter(tips[mask, 0], tips[mask, 1], tips[mask, 2],
                           color=axis_colors[axis_idx], s=8, alpha=0.3,
                           edgecolors=file_color, linewidths=0.5)

    ax_ori.set_xlim([-1.2, 1.2])
    ax_ori.set_ylim([-1.2, 1.2])
    ax_ori.set_zlim([-1.2, 1.2])
    ax_ori.set_xlabel('X')
    ax_ori.set_ylabel('Y')
    ax_ori.set_zlabel('Z')
    # Add outlier legend entry
    if outlier_count > 0:
        ax_ori.quiver([], [], [], [], [], [], color='magenta', linewidth=3, label=f'Outlier (>={OUTLIER_ANGLE_THRESHOLD}°)')

    ax_ori.set_title(f'tool0_from_flange_mocap Rotation Axes\n'
                     f'Thick = unit axes | Thin = data | Magenta = outliers (>={OUTLIER_ANGLE_THRESHOLD}°, n={outlier_count})\n'
                     f'Angle range: X=[{np.min(angular_deviations["X"]):.1f}°,{np.max(angular_deviations["X"]):.1f}°] '
                     f'Y=[{np.min(angular_deviations["Y"]):.1f}°,{np.max(angular_deviations["Y"]):.1f}°] '
                     f'Z=[{np.min(angular_deviations["Z"]):.1f}°,{np.max(angular_deviations["Z"]):.1f}°]')
    ax_ori.legend(fontsize=7, loc='upper left')

    plt.tight_layout()
    output_path_ori = os.path.join(data_folder, f'debug_orientation_{data_batch}.png')
    plt.savefig(output_path_ori, dpi=150)
    logger.info('Debug orientation plot saved to: %s', output_path_ori)
    plt.show()
    plt.close(fig_ori)

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
    ax1.set_ylabel('Position Norm (mm)')
    ax1.set_title(f'tool0_from_flange_mocap Position Norm\nMean: {np.mean(pos_distances):.3f} mm')
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
    for axis_idx, (axis_name, color) in enumerate(zip(['X', 'Y', 'Z'], colors)):
        sorted_ang = np.sort(angular_deviations[axis_name])
        cdf_ang = np.arange(1, len(sorted_ang) + 1) / len(sorted_ang)
        ang_95 = np.percentile(angular_deviations[axis_name], 95)
        ax4.plot(sorted_ang, cdf_ang, color=color, linewidth=2, label=f'{axis_name}-axis (95%: {ang_95:.3f}°)')
        ax4.plot(ang_95, 0.95, 'o', color=color, markersize=6)
        ax4.annotate(f'{ang_95:.2f}°', xy=(ang_95, 0.95), xytext=(10, -15 * (axis_idx + 1)),
                     textcoords='offset points', fontsize=9, color=color,
                     arrowprops=dict(arrowstyle='->', color=color, alpha=0.7))
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

    # ========== Figure 1b: Debug 3D scatter of tool0_from_flange_mocap positions ==========
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    file_names = [r['file_name'] for r in results]
    positions_mm = positions * 1000
    pos_mean_mm = pos_mean * 1000

    fig1b = plt.figure(figsize=(12, 9))
    ax3d = fig1b.add_subplot(111, projection='3d')

    # Color each point by its file (circle)
    unique_files = list(dict.fromkeys(file_names))  # preserve order
    file_colors = plt.cm.tab10.colors
    for file_idx, fname in enumerate(unique_files):
        mask = np.array([f == fname for f in file_names])
        color = file_colors[file_idx % len(file_colors)]
        label = fname.replace('calibration_', '').replace('.json', '')
        pts = positions_mm[mask]
        ax3d.scatter(pts[:, 0], pts[:, 1], pts[:, 2],
                     c=[color], s=20, alpha=0.6, label=label)
        # Draw lines from each point to the mean
        for pt in pts:
            ax3d.plot([pt[0], pos_mean_mm[0]], [pt[1], pos_mean_mm[1]], [pt[2], pos_mean_mm[2]],
                      color=color, linewidth=0.5, alpha=0.3)

    # Draw mean as a large marker
    ax3d.scatter(*pos_mean_mm, color='black', s=200, marker='*', zorder=5, label='mean')

    ax3d.set_xlabel('X (mm)')
    ax3d.set_ylabel('Y (mm)')
    ax3d.set_zlabel('Z (mm)')
    ax3d.set_title(f'tool0_from_flange_mocap positions (in tool0 frame)\n'
                   f'Mean: [{pos_mean_mm[0]:.1f}, {pos_mean_mm[1]:.1f}, {pos_mean_mm[2]:.1f}] mm | '
                   f'Max dev: {np.max(pos_distances):.1f} mm | {len(results)} samples')
    ax3d.legend(fontsize=8, loc='upper left')

    plt.tight_layout()
    output_path1b = os.path.join(data_folder, f'debug_offset_positions_{data_batch}.png')
    plt.savefig(output_path1b, dpi=150)
    logger.info('Debug offset positions plot saved to: %s', output_path1b)
    plt.show()
    plt.close(fig1b)

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
    # Plot arrows indicating orientation (fixed scale based on plot range: 8000mm x 4000mm)
    arrow_scale = 300  # Fixed arrow length in mm
    ax6.quiver(base_x_mm, base_y_mm, arrow_dx * arrow_scale, arrow_dy * arrow_scale,
               angles='xy', scale_units='xy', scale=1, color='red', alpha=0.6,
               width=0.003, headwidth=3, headlength=4, zorder=3)
    ax6.set_xlabel('Base X (mm)')
    ax6.set_ylabel('Base Y (mm)')
    x_range = np.ptp(base_x_mm)
    y_range = np.ptp(base_y_mm)
    yaw_range = np.ptp(base_yaws)
    ax6.set_title(f'Base Position & Yaw Diversity\nX range: {x_range:.1f}mm, Y range: {y_range:.1f}mm, Yaw range: {yaw_range:.1f}°')
    ax6.set_xlim(-4000, 4000)
    ax6.set_ylim(-2000, 2000)
    ax6.set_aspect('equal')
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

    # Debug 3D plot: tool0 vs flange_mocap
    plot_tool0_vs_flange_3d(results, VALIDATION_DATA_FOLDER, VALIDATION_DATA_BATCH)

    # Analyze results
    analyze_results(results, VALIDATION_DATA_FOLDER, VALIDATION_DATA_BATCH, DATE_FOLDER, ROBOT_NAME, ARM)
    
    pp.disconnect()
    
    logger.info('=' * 60)
    logger.info('Verification complete!')
    logger.info('=' * 60)


if __name__ == '__main__':
    main()
