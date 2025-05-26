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
file_handler = logging.FileHandler(os.path.join(data_folder, f'tool0_bar_center_position_analysis_log.txt'), mode='w')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
logger.addHandler(file_handler)

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
    
    # Extract positions only
    positions = [transform[0] for transform in tool0_from_bar_centers]
    
    # Convert to numpy array for easier calculation
    positions_np = np.array(positions)
    
    # Calculate average position
    avg_position = np.mean(positions_np, axis=0)
    # Convert to mm for logging
    avg_position_mm = avg_position * 1000
    logger.info(f"Average tool0_from_bar_center position: [{avg_position_mm[0]:.2f}, {avg_position_mm[1]:.2f}, {avg_position_mm[2]:.2f}] mm")
    
    # Calculate distances from average position
    distances = np.linalg.norm(positions_np - avg_position, axis=1)
    # Convert to mm for logging and plotting
    distances_mm = distances * 1000
    
    logger.info(f"Mean distance from average position: {np.mean(distances_mm):.2f} mm")
    logger.info(f"Max distance from average position: {np.max(distances_mm):.2f} mm")
    logger.info(f"Min distance from average position: {np.min(distances_mm):.2f} mm")
    logger.info(f"Standard deviation of distances: {np.std(distances_mm):.2f} mm")
    
    # Create DataFrame for easier plotting
    df = pd.DataFrame({
        'Distance (mm)': distances_mm,
        'Closest Axis': [str(axis) for axis in closest_axes],
        'Bar Height': np.array(bar_heights) * 1000,  # Convert to mm
        'Sample Index': range(len(distances_mm))
    })
    
    # Plot 1: Scatter plot of distances from average position
    plt.figure(figsize=(12, 6))
    sns.scatterplot(data=df, x='Sample Index', y='Distance (mm)', hue='Closest Axis')
    plt.axhline(y=np.mean(distances_mm), color='r', linestyle='--', label=f'Mean: {np.mean(distances_mm):.2f} mm')
    plt.title('Distance of Each Sample from Average Position')
    plt.xlabel('Sample Index')
    plt.ylabel('Distance from Average Position (mm)')
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_position_distances.png'))
    
    # Plot 2: 3D scatter plot of positions
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Convert positions to mm for 3D plot
    positions_np_mm = positions_np * 1000
    avg_position_mm = avg_position * 1000
    
    # Color points by closest axis
    colors = {'0': 'r', '1': 'g', '2': 'b'}
    for axis in set(df['Closest Axis']):
        mask = df['Closest Axis'] == axis
        ax.scatter(
            positions_np_mm[mask, 0], 
            positions_np_mm[mask, 1], 
            positions_np_mm[mask, 2],
            c=colors.get(axis, 'k'),
            label=f'Axis {axis}'
        )
    
    # Add average position
    ax.scatter(
        avg_position_mm[0], 
        avg_position_mm[1], 
        avg_position_mm[2],
        c='k', marker='*', s=200,
        label='Average Position'
    )
    
    ax.set_title('3D Scatter Plot of Tool0 from Bar Center Positions')
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_positions_3d.png'))
    
    # Plot 3: Boxplot of position distances by closest axis
    plt.figure(figsize=(10, 6))
    sns.boxplot(data=df, x='Closest Axis', y='Distance (mm)')
    plt.title('Position Deviation by Bar Orientation')
    plt.xlabel('Bar Aligned to Global Axis')
    plt.ylabel('Distance from Average Position (mm)')
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_position_by_orientation.png'))
    
    # Plot 4: Accumulated frequency graph (CDF) for position
    plt.figure(figsize=(10, 6))
    sns.ecdfplot(data=df, x='Distance (mm)', hue='Closest Axis')
    plt.title('CDF of Position Distances')
    plt.xlabel('Distance from Average Position (mm)')
    plt.ylabel('Cumulative Frequency')
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_position_cdf.png'))
    
    # Plot 5: Scatterplot of position deviation vs bar height
    plt.figure(figsize=(10, 6))
    sns.scatterplot(data=df, x='Bar Height', y='Distance (mm)', hue='Closest Axis')
    plt.title('Position Deviation vs Bar Height')
    plt.xlabel('Bar Height (mm)')
    plt.ylabel('Distance from Average Position (mm)')
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_position_vs_height.png'))
    
    # Plot 6: Summary statistics
    plt.figure(figsize=(8, 6))
    # Position statistics by axis
    position_stats = df.groupby('Closest Axis')['Distance (mm)'].agg(['mean', 'std']).reset_index()
    
    x = np.arange(len(position_stats))
    width = 0.5
    
    fig, ax = plt.subplots(figsize=(10, 6))
    bars = ax.bar(x, position_stats['mean'], width, yerr=position_stats['std'], capsize=10)
    
    ax.set_ylabel('Mean Distance (mm)')
    ax.set_title('Mean Position Deviation by Bar Orientation')
    ax.set_xticks(x)
    ax.set_xticklabels(['Axis ' + label for label in position_stats['Closest Axis']])
    
    # Add value labels on top of bars
    for i, bar in enumerate(bars):
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + position_stats['std'].iloc[i]/2,
                f'{position_stats["mean"].iloc[i]:.2f} mm', ha='center', va='bottom')
    
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_position_stats.png'))
    
    # Calculate position components deviation
    x_distances = np.abs(positions_np[:, 0] - avg_position[0]) * 1000  # Convert to mm
    y_distances = np.abs(positions_np[:, 1] - avg_position[1]) * 1000  # Convert to mm
    z_distances = np.abs(positions_np[:, 2] - avg_position[2]) * 1000  # Convert to mm
    
    # Add to dataframe
    df['X Distance (mm)'] = x_distances
    df['Y Distance (mm)'] = y_distances
    df['Z Distance (mm)'] = z_distances
    
    # Log component statistics
    logger.info("Position component deviations:")
    logger.info(f"  X - Mean: {np.mean(x_distances):.2f} mm, Std: {np.std(x_distances):.2f} mm")
    logger.info(f"  Y - Mean: {np.mean(y_distances):.2f} mm, Std: {np.std(y_distances):.2f} mm")
    logger.info(f"  Z - Mean: {np.mean(z_distances):.2f} mm, Std: {np.std(z_distances):.2f} mm")
    
    # Plot 7: Component deviations
    plt.figure(figsize=(10, 6))
    component_data = pd.melt(df, id_vars=['Sample Index', 'Closest Axis'], 
                            value_vars=['X Distance (mm)', 'Y Distance (mm)', 'Z Distance (mm)'],
                            var_name='Component', value_name='Component Distance (mm)')
    
    sns.boxplot(data=component_data, x='Component', y='Component Distance (mm)', hue='Closest Axis')
    plt.title('Position Component Deviations')
    plt.xlabel('Position Component')
    plt.ylabel('Absolute Deviation (mm)')
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, 'tool0_bar_position_components.png'))
    
    logger.info("Position analysis completed. All visualizations saved.")

if __name__ == "__main__":
    main()