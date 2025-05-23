import os
import json
import numpy as np
import matplotlib.pyplot as plt
import pybullet_planning as pp
import pandas as pd
import seaborn as sns
from mpl_toolkits.mplot3d import Axes3D
import logging
import colorlog

# Configure logging
logger = logging.getLogger()
logger.setLevel(logging.INFO)

# Create console handler with color formatting
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)

color_formatter = colorlog.ColoredFormatter(
    '%(log_color)s%(asctime)s - %(levelname)s - %(message)s',
    log_colors={
        'DEBUG': 'cyan',
        'INFO': 'green',
        'WARNING': 'yellow',
        'ERROR': 'red',
        'CRITICAL': 'red,bg_white',
    }
)
console_handler.setFormatter(color_formatter)
logger.addHandler(console_handler)

# Path setup
HERE = os.path.dirname(os.path.abspath(__file__))
DATA_BATCH = '20250519_vary_pos_vary_yaw'  # Change this if needed
# DATA_BATCH = '20250519_fixed_pos_vary_yaw'
data_folder = os.path.join(HERE, DATA_BATCH)

# Create file handler for logging
file_handler = logging.FileHandler(os.path.join(data_folder, f'tool0_bar_center_analysis_log.txt'), mode='w')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
logger.addHandler(file_handler)

# Function to compute angle between two vectors in degrees
def angle_between(v1, v2):
    v1_unit = v1 / np.linalg.norm(v1)
    v2_unit = v2 / np.linalg.norm(v2)
    dot_product = np.clip(np.dot(v1_unit, v2_unit), -1.0, 1.0)
    angle_rad = np.arccos(dot_product)
    return np.degrees(angle_rad)

def main():
    # Load compiled data
    json_file_path = os.path.join(data_folder, f'compiled_bar_holding_acc_{DATA_BATCH}.json')
    logger.info(f"Loading data from: {json_file_path}")
    
    with open(json_file_path, 'r') as f:
        data = json.load(f)
    
    # Extract all tool0_from_bar_center transformations
    tool0_from_bar_centers = []
    closest_axes = []
    bar_heights = []
    
    for entry in data:
        tool0_from_bar_center = entry.get('tool0_from_bar_center')
        if tool0_from_bar_center:
            tool0_from_bar_centers.append(tool0_from_bar_center)
            closest_axes.append(entry.get('closest_axis'))
            bar_heights.append(entry['fitted_line']['point'][2])
    
    logger.info(f"Found {len(tool0_from_bar_centers)} tool0_from_bar_center transformations")
    
    # Extract positions and quaternions
    positions = [transform[0] for transform in tool0_from_bar_centers]
    quaternions = [transform[1] for transform in tool0_from_bar_centers]
    
    # Convert to numpy arrays for easier calculation
    positions_np = np.array(positions)
    
    # Calculate average position
    avg_position = np.mean(positions_np, axis=0)
    logger.info(f"Average tool0_from_bar_center position: {avg_position}")
    
    # Calculate distances from average position
    distances = np.linalg.norm(positions_np - avg_position, axis=1)
    logger.info(f"Mean distance from average position: {np.mean(distances):.6f} m")
    logger.info(f"Max distance from average position: {np.max(distances):.6f} m")
    logger.info(f"Min distance from average position: {np.min(distances):.6f} m")
    logger.info(f"Standard deviation of distances: {np.std(distances):.6f} m")
    
    # Create DataFrame for easier plotting
    df = pd.DataFrame({
        'Distance': distances,
        'Closest Axis': [str(axis) for axis in closest_axes],
        'Bar Height': bar_heights,
        'Sample Index': range(len(distances))
    })
    
    # Plot 1: Scatter plot of distances from average position
    plt.figure(figsize=(12, 6))
    sns.scatterplot(data=df, x='Sample Index', y='Distance', hue='Closest Axis')
    plt.axhline(y=np.mean(distances), color='r', linestyle='--', label=f'Mean: {np.mean(distances):.6f} m')
    plt.title('Distance of Each Sample from Average Position')
    plt.xlabel('Sample Index')
    plt.ylabel('Distance from Average Position (m)')
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_position_distances.png'))
    
    # 3D scatter plot of positions
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Color points by closest axis
    colors = {'0': 'r', '1': 'g', '2': 'b'}
    for axis in set(df['Closest Axis']):
        mask = df['Closest Axis'] == axis
        ax.scatter(
            positions_np[mask, 0], 
            positions_np[mask, 1], 
            positions_np[mask, 2],
            c=colors.get(axis, 'k'),
            label=f'Axis {axis}'
        )
    
    # Add average position
    ax.scatter(
        avg_position[0], 
        avg_position[1], 
        avg_position[2],
        c='k', marker='*', s=200,
        label='Average Position'
    )
    
    ax.set_title('3D Scatter Plot of Tool0 from Bar Center Positions')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_positions_3d.png'))
    
    # Calculate rotation matrices and extract axes
    x_axes = []
    y_axes = []
    z_axes = []
    
    for quat in quaternions:
        rot_matrix = pp.matrix_from_quat(quat)
        x_axes.append(rot_matrix[:3, 0])  # First column is x-axis
        y_axes.append(rot_matrix[:3, 1])  # Second column is y-axis
        z_axes.append(rot_matrix[:3, 2])  # Third column is z-axis
    
    # Convert to numpy arrays
    x_axes_np = np.array(x_axes)
    y_axes_np = np.array(y_axes)
    z_axes_np = np.array(z_axes)
    
    # Calculate average axes
    avg_x_axis = np.mean(x_axes_np, axis=0)
    avg_x_axis = avg_x_axis / np.linalg.norm(avg_x_axis)
    
    avg_y_axis = np.mean(y_axes_np, axis=0)
    avg_y_axis = avg_y_axis / np.linalg.norm(avg_y_axis)
    
    avg_z_axis = np.mean(z_axes_np, axis=0)
    avg_z_axis = avg_z_axis / np.linalg.norm(avg_z_axis)
    
    logger.info(f"Average X-axis: {avg_x_axis}")
    logger.info(f"Average Y-axis: {avg_y_axis}")
    logger.info(f"Average Z-axis: {avg_z_axis}")
    
    # Calculate angle deviations from average axes
    x_deviations = [angle_between(axis, avg_x_axis) for axis in x_axes_np]
    y_deviations = [angle_between(axis, avg_y_axis) for axis in y_axes_np]
    z_deviations = [angle_between(axis, avg_z_axis) for axis in z_axes_np]
    
    # Log statistics for each axis
    for axis_name, deviations in [('X', x_deviations), ('Y', y_deviations), ('Z', z_deviations)]:
        logger.info(f"{axis_name}-axis statistics:")
        logger.info(f"  Mean deviation: {np.mean(deviations):.4f} degrees")
        logger.info(f"  Max deviation: {np.max(deviations):.4f} degrees")
        logger.info(f"  Min deviation: {np.min(deviations):.4f} degrees")
        logger.info(f"  Standard deviation: {np.std(deviations):.4f} degrees")
    
    # Add deviation data to DataFrame
    df['X-Axis Deviation'] = x_deviations
    df['Y-Axis Deviation'] = y_deviations
    df['Z-Axis Deviation'] = z_deviations
    
    # Plot 2: Axis deviations from average
    plt.figure(figsize=(15, 8))
    
    plt.subplot(1, 3, 1)
    sns.scatterplot(data=df, x='Sample Index', y='X-Axis Deviation', hue='Closest Axis')
    plt.axhline(y=np.mean(x_deviations), color='r', linestyle='--', label=f'Mean: {np.mean(x_deviations):.4f}°')
    plt.title('X-Axis Angle Deviations')
    plt.xlabel('Sample Index')
    plt.ylabel('Angle Deviation (degrees)')
    plt.legend()
    
    plt.subplot(1, 3, 2)
    sns.scatterplot(data=df, x='Sample Index', y='Y-Axis Deviation', hue='Closest Axis')
    plt.axhline(y=np.mean(y_deviations), color='r', linestyle='--', label=f'Mean: {np.mean(y_deviations):.4f}°')
    plt.title('Y-Axis Angle Deviations')
    plt.xlabel('Sample Index')
    plt.ylabel('Angle Deviation (degrees)')
    plt.legend()
    
    plt.subplot(1, 3, 3)
    sns.scatterplot(data=df, x='Sample Index', y='Z-Axis Deviation', hue='Closest Axis')
    plt.axhline(y=np.mean(z_deviations), color='r', linestyle='--', label=f'Mean: {np.mean(z_deviations):.4f}°')
    plt.title('Z-Axis Angle Deviations')
    plt.xlabel('Sample Index')
    plt.ylabel('Angle Deviation (degrees)')
    plt.legend()
    
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_axis_deviations.png'))
    
    # Plot 3: Boxplot of axis deviations by closest axis
    plt.figure(figsize=(15, 8))
    
    plt.subplot(1, 3, 1)
    sns.boxplot(data=df, x='Closest Axis', y='X-Axis Deviation')
    plt.title('X-Axis Deviations by Bar Orientation')
    plt.xlabel('Bar Aligned to Global Axis')
    plt.ylabel('X-Axis Deviation (degrees)')
    
    plt.subplot(1, 3, 2)
    sns.boxplot(data=df, x='Closest Axis', y='Y-Axis Deviation')
    plt.title('Y-Axis Deviations by Bar Orientation')
    plt.xlabel('Bar Aligned to Global Axis')
    plt.ylabel('Y-Axis Deviation (degrees)')
    
    plt.subplot(1, 3, 3)
    sns.boxplot(data=df, x='Closest Axis', y='Z-Axis Deviation')
    plt.title('Z-Axis Deviations by Bar Orientation')
    plt.xlabel('Bar Aligned to Global Axis')
    plt.ylabel('Z-Axis Deviation (degrees)')
    
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_axis_deviations_by_orientation.png'))
    
    # Plot 4: Accumulated frequency graph (CDF) for position and axis deviations
    plt.figure(figsize=(15, 10))
    
    plt.subplot(2, 2, 1)
    sns.ecdfplot(data=df, x='Distance', hue='Closest Axis')
    plt.title('CDF of Position Distances')
    plt.xlabel('Distance from Average Position (m)')
    plt.ylabel('Cumulative Frequency')
    
    plt.subplot(2, 2, 2)
    sns.ecdfplot(data=df, x='X-Axis Deviation', hue='Closest Axis')
    plt.title('CDF of X-Axis Deviations')
    plt.xlabel('Angle Deviation (degrees)')
    plt.ylabel('Cumulative Frequency')
    
    plt.subplot(2, 2, 3)
    sns.ecdfplot(data=df, x='Y-Axis Deviation', hue='Closest Axis')
    plt.title('CDF of Y-Axis Deviations')
    plt.xlabel('Angle Deviation (degrees)')
    plt.ylabel('Cumulative Frequency')
    
    plt.subplot(2, 2, 4)
    sns.ecdfplot(data=df, x='Z-Axis Deviation', hue='Closest Axis')
    plt.title('CDF of Z-Axis Deviations')
    plt.xlabel('Angle Deviation (degrees)')
    plt.ylabel('Cumulative Frequency')
    
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_cdf_plots.png'))
    
    # Additional analysis: Combined bar chart of mean deviations
    plt.figure(figsize=(10, 6))
    mean_deviations = {
        'Position (mm)': np.mean(distances) * 1000,  # Convert to mm
        'X-Axis (deg)': np.mean(x_deviations),
        'Y-Axis (deg)': np.mean(y_deviations),
        'Z-Axis (deg)': np.mean(z_deviations)
    }
    
    std_deviations = {
        'Position (mm)': np.std(distances) * 1000,  # Convert to mm
        'X-Axis (deg)': np.std(x_deviations),
        'Y-Axis (deg)': np.std(y_deviations),
        'Z-Axis (deg)': np.std(z_deviations)
    }
    
    means = list(mean_deviations.values())
    stds = list(std_deviations.values())
    labels = list(mean_deviations.keys())
    
    x = np.arange(len(labels))
    width = 0.35
    
    fig, ax = plt.subplots(figsize=(12, 6))
    bars = ax.bar(x, means, width, yerr=stds, capsize=10)
    
    ax.set_ylabel('Mean Deviation')
    ax.set_title('Mean Deviations in Position and Orientation')
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    
    # Add value labels on top of bars
    for i, bar in enumerate(bars):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + stds[i]/2,
                f'{means[i]:.4f}', ha='center', va='bottom')
    
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_mean_deviations.png'))
    
    logger.info("Analysis completed. All visualizations saved.")

if __name__ == "__main__":
    main()