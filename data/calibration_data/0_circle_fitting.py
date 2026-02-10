# https://github.com/CristianoPizzamiglio/circle-fitting-3d
# https://scikit-spatial.readthedocs.io/en/stable/index.html

# https://leomariga.github.io/pyRANSAC-3D/

import json
import logging
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import pybullet_planning as pp
from circle_fitting_3d import Circle3D
from config_loader import load_config, HERE
from logging_utils import setup_logger

# Load configuration
config = load_config()
date_folder = config['date_folder']
data_batches = config['data_batches']
EXPORT = config['export']

# Configure logging with colored output
logger = setup_logger()


def process_data_batch(data_batch, date_folder, export=True):
    """Process a single data batch (j0 or j1) for circle fitting."""
    data_folder = os.path.join(HERE, date_folder, data_batch)
    
    # Create file handler for this batch
    file_handler = logging.FileHandler(os.path.join(data_folder, f'circle_fitting_log_{data_batch}.txt'), mode='w')
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))
    logger.addHandler(file_handler)
    
    logger.info('=' * 60)
    logger.info(f'Processing data batch: {data_batch}')
    logger.info('=' * 60)
    
    # load each json file start with the name "calibration_" in the data folder
    json_files = [f for f in os.listdir(data_folder) if f.startswith('calibration_') and f.endswith('.json')]
    
    centers = []
    normals = []
    new_data = {'takes': []}
    
    # Collect visualization data for combined plot
    all_viz_data = []
    
    for i, file_name in enumerate(json_files):
        logger.info('Processing: %s', file_name)
        file_path = os.path.join(data_folder, file_name)
    
        # Load the JSON file
        with open(file_path, 'r') as file:
            data = json.load(file)
    
        # Parse the origin of mocap_pose data
        points = []
        for entry in data['raw_data']:
            flange_mocap_pose = entry.get("flange_mocap_pose", [])
            base_mocap_pose = entry.get("base_mocap_pose", [])
            if flange_mocap_pose and base_mocap_pose:
                # ! from here on all the flange readings on in the base_mocap frame
                base_from_flange = pp.multiply(pp.invert(base_mocap_pose), flange_mocap_pose)
                origin = base_from_flange[0]  # Assuming the first element is the origin
                points.append(origin)
    
        # Fit circle to points
        start_time = time.perf_counter()
        circle_3d = Circle3D(points)
        elapsed_time = time.perf_counter() - start_time
        logger.info('  Circle fitting time: %.3f ms', elapsed_time * 1000)
    
        # Calculate projections onto circle and distances (vectorized)
        pts = np.array(points)
        center = np.array(circle_3d.center)
        normal = np.array(circle_3d.normal)
        normal = normal / np.linalg.norm(normal)
        # Project points onto the circle's plane
        diff = pts - center
        dist_to_plane = diff @ normal  # signed distances to plane
        pts_on_plane = pts - np.outer(dist_to_plane, normal)
        # Get unit vectors from center to projected points, then scale by radius
        vecs = pts_on_plane - center
        vecs_norm = np.linalg.norm(vecs, axis=1, keepdims=True)
        vecs_unit = vecs / vecs_norm
        projected_points = center + vecs_unit * circle_3d.radius
        distances = np.linalg.norm(projected_points - pts, axis=1).tolist()
    
        logger.info('  Circle fit quality - max point distance: %.3f mm', max(distances) * 1000)
    
        # Store visualization data for combined plot
        all_viz_data.append({
            'file_name': file_name,
            'points': np.array(points),
            'projected_points': np.array(projected_points),
            'distances': distances,
            'center': np.array(circle_3d.center),
            'normal': np.array(circle_3d.normal),
            'radius': circle_3d.radius,
            'max_dist': max(distances)
        })
    
        centers.append(circle_3d.center)
        normals.append(circle_3d.normal)
    
        if export:
            take_data = {
                'file_name': file_name,
                'raw_data': data['raw_data'],
                'center': list(centers[-1]),
                'normal': list(normals[-1]),
                }
            new_data['takes'].append(take_data)
    
    # Export results
    if export:
        output_file = os.path.join(data_folder, f'{data_batch}_analysis.json')
        with open(output_file, 'w') as f:
            json.dump(new_data, f, indent=4)
        logger.info('Results exported to: %s', output_file)
        logger.info('Total takes processed: %d', len(new_data['takes']))
    
    # Combined 3D visualization of all circles and projection errors
    fig = plt.figure(figsize=(14, 12))
    ax = fig.add_subplot(projection='3d')
    
    # Use different colors for each take
    colors = plt.cm.tab10(np.linspace(0, 1, len(all_viz_data)))
    
    for idx, (viz_data, color) in enumerate(zip(all_viz_data, colors)):
        points_arr = viz_data['points']
        projected_arr = viz_data['projected_points']
        distances = viz_data['distances']
        center = viz_data['center']
        normal = viz_data['normal']
        radius = viz_data['radius']
        
        # Plot data points
        ax.scatter(points_arr[:, 0] * 1000, points_arr[:, 1] * 1000, points_arr[:, 2] * 1000,
                   c=[color], s=30, alpha=0.7, label=f'Take {idx+1} points')
        
        # Plot projected points on circle
        ax.scatter(projected_arr[:, 0] * 1000, projected_arr[:, 1] * 1000, projected_arr[:, 2] * 1000,
                   c=[color], s=10, marker='x', alpha=0.5)
        
        # Draw lines from data points to projections with annotated distances
        for pt, proj, dist in zip(points_arr, projected_arr, distances):
            ax.plot([pt[0] * 1000, proj[0] * 1000],
                    [pt[1] * 1000, proj[1] * 1000],
                    [pt[2] * 1000, proj[2] * 1000],
                    color=color, linestyle='--', alpha=0.4, linewidth=0.8)
            # Annotate distance at midpoint
            mid = (pt + proj) / 2
            ax.text(mid[0] * 1000, mid[1] * 1000, mid[2] * 1000,
                    f'{dist * 1000:.2f}', fontsize=6, alpha=0.6, color=color)
        
        # Plot fitted circle
        theta = np.linspace(0, 2 * np.pi, 100)
        if abs(normal[2]) < 0.9:
            v1 = np.cross(normal, np.array([0, 0, 1]))
        else:
            v1 = np.cross(normal, np.array([1, 0, 0]))
        v1 = v1 / np.linalg.norm(v1)
        v2 = np.cross(normal, v1)
        v2 = v2 / np.linalg.norm(v2)
        
        circle_points = center + radius * (np.outer(np.cos(theta), v1) + np.outer(np.sin(theta), v2))
        ax.plot(circle_points[:, 0] * 1000, circle_points[:, 1] * 1000, circle_points[:, 2] * 1000,
                color=color, linewidth=2, label=f'Take {idx+1} circle (r={radius * 1000:.1f} mm)')
        
        # Plot circle center
        ax.scatter([center[0] * 1000], [center[1] * 1000], [center[2] * 1000],
                   c=[color], s=80, marker='*')
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title(f'{data_batch} - All Circle Fits ({len(all_viz_data)} takes)')
    ax.legend(loc='upper left', fontsize=8)
    
    plt.savefig(os.path.join(data_folder, f'{data_batch}_all_circles.png'), dpi=150)
    plt.show()
    plt.close()
    
    # 2D plot of projection errors across all circles
    fig, ax = plt.subplots(figsize=(12, 6))
    
    markers = ['o', 's', '^', 'D', 'v', '<', '>', 'p', 'h', '*']
    sample_offset = 0
    
    for idx, (viz_data, color) in enumerate(zip(all_viz_data, colors)):
        distances_mm = np.array(viz_data['distances']) * 1000
        n_samples = len(distances_mm)
        sample_indices = np.arange(sample_offset, sample_offset + n_samples)
        
        marker = markers[idx % len(markers)]
        ax.scatter(sample_indices, distances_mm, c=[color], s=40, marker=marker,
                   label=f'Take {idx+1} (max: {viz_data["max_dist"] * 1000:.2f} mm)', alpha=0.8)
        
        # Add vertical separator line between takes
        if idx > 0:
            ax.axvline(x=sample_offset - 0.5, color='gray', linestyle='--', alpha=0.5, linewidth=1)
        
        sample_offset += n_samples
    
    ax.set_xlabel('Sample index')
    ax.set_ylabel('Projection error (mm)')
    ax.set_title(f'{data_batch} - Projection Errors Across All Circles ({len(all_viz_data)} takes)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(data_folder, f'{data_batch}_projection_errors.png'), dpi=150)
    plt.show()
    plt.close()
    
    # Remove file handler after processing
    logger.removeHandler(file_handler)
    file_handler.close()
    
    return centers, normals, all_viz_data


# Process all data batches
if __name__ == '__main__':
    for data_batch in data_batches:
        logger.info(f'\n{"#" * 60}')
        logger.info(f'Starting processing for {data_batch}')
        logger.info(f'{"#" * 60}\n')
        
        process_data_batch(data_batch, date_folder, export=EXPORT)
    
    logger.info('\n' + '=' * 60)
    logger.info('All data batches processed successfully!')
    logger.info('=' * 60)
