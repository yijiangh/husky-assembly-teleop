# https://github.com/CristianoPizzamiglio/circle-fitting-3d
# https://scikit-spatial.readthedocs.io/en/stable/index.html

# https://leomariga.github.io/pyRANSAC-3D/

import json, os
import logging, datetime
from turtle import Vec2D
from circle_fitting_3d import Circle3D
from matplotlib.pylab import normal
from skspatial.objects import Line, Points, Point, Vector, Plane
from skspatial.plotting import plot_3d
import matplotlib.pyplot as plt
import numpy as np
import pybullet_planning as pp

MARKER_NAME_PAIRS = [
    ['Marker5', 'Marker6'],
    ['Marker7', 'Marker8'],
    ['Marker2', 'Marker4'],
    ['Marker1', 'Marker3']
]

data_batch = ''
EXPORT = 1

HERE = os.path.dirname(os.path.abspath(__file__))
data_folder = os.path.join(HERE, data_batch)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger()

# Create file handler
file_handler = logging.FileHandler(os.path.join(data_folder, f'bar_holding_acc_analysis_log_{data_batch}.txt'), mode='w')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))

# Add handlers to the logger
logger.addHandler(file_handler)

json_files = [f for f in os.listdir(data_folder) if f.startswith('bar_holding_acc_') and f.endswith('.json')]

new_data = []

# accumulated data, for drawing
centers = []
fitted_lines = []
base_positions = []

for i, file_name in enumerate(json_files):
    logger.info('Working on file: %s', file_name)
    file_path = os.path.join(data_folder, file_name)

    # Load the JSON file
    with open(file_path, 'r') as file:
        data = json.load(file)

    # Parse the origin of mocap_pose data
    base_mocap_origins = []
    base_mocap_quats = []
    for entry in data['raw_data']:
        # each individual take
        joint_conf = entry.get("joint_conf", [])
        footprint_pose = entry.get("footprint_base_link_pose", [])

        marker_pts = entry.get("bar_rig", [])
        center_points = []
        for maker_pair in MARKER_NAME_PAIRS:
            m1, m2 = maker_pair
            p1 = np.array(marker_pts[m1]["marker_positions"])
            p2 = np.array(marker_pts[m2]["marker_positions"])
            # should be around 0.079m
            if pp.get_distance(p1, p2) > 0.1:
                raise ValueError(f"Distance between {m1} and {m2} is too large: {pp.get_distance(p1, p2)} m")
            center = (p1 + p2) / 2
            center_points.append(center)

        line_fit = Line.best_fit(center_points)

        # Compute the direction vector of the fitted line
        line_direction = line_fit.direction / np.linalg.norm(line_fit.direction)

        # Define global axes
        global_axes = {
            0: np.array([1, 0, 0]),
            1: np.array([0, 1, 0]),
            2: np.array([0, 0, 1]),
        }

        # Find the closest global axis
        closest_axis = None
        min_angle = float('inf')
        tol = 1e-1
        for axis_name, axis_vector in global_axes.items():
            # Compute the angle between the line direction and the global axis
            dot_product = np.dot(line_direction, axis_vector)
            angle = np.arccos(np.clip(dot_product, -1.0, 1.0))  # Clip to handle numerical precision issues

            # Check if the angle is close to 0 or pi
            if np.isclose(angle, 0, atol=tol) or np.isclose(angle, np.pi, atol=tol):
                closest_axis = axis_name
                # use unclipped one when exiting
                min_angle = np.arccos(dot_product)
                break

            # Update the closest axis if the angle is smaller
            if angle < min_angle:
                closest_axis = axis_name
                min_angle = angle
        else:
            logger.error(f'No axis found that is close to the line direction. The tolerance {tol} might be too strict.')
            raise ValueError()

        display_min_angle = np.rad2deg(min_angle)
        if display_min_angle > 90:
            display_min_angle = 180 - display_min_angle
        logger.info('Closest global axis to the line: %s (angle: %.2f deg)', closest_axis, display_min_angle)

        if EXPORT:
            take_data = {
                'raw_data': entry, 
                'footprint_pose': footprint_pose, 
                'joint_conf': joint_conf, 
                'point_centers': [list(center) for center in center_points], 
                'fitted_line': {'point' : list(line_fit.point), 'direction' : list(line_fit.direction)}, 
                'closest_axis': closest_axis,
                'angle_to_closest_axis': display_min_angle, # ! this is degrees!
                }
            new_data.append(take_data)

        centers.append(center_points)
        fitted_lines.append(line_fit)
        base_positions.append(footprint_pose[0])
        # print('center_points:', center_points)
        # print('fitted_line:', line_fit.point, line_fit.direction)

    # break

if EXPORT:
    analysis_file_path = os.path.join(data_folder, f'analysis_bar_holding_acc_{data_batch}.json')
    with open(analysis_file_path, 'w') as file:
        json.dump(new_data, file, indent=4)

ax = plt.figure().add_subplot(projection='3d')

# plot all footprint point, observed centers, fitted lines
line_len = 0.5
for line_fit, marker_centers, robot_base_pos in zip(fitted_lines, centers, base_positions):
    line_fit.plot_3d(ax, t_1=-line_len, t_2=line_len, c='k')
    Points(marker_centers).plot_3d(ax, c='b', depthshade=False)
    Point(robot_base_pos).plot_3d(ax, c='g', depthshade=False)

# for o, v in zip(centers, normals):
#     Line(point=o, direction=v).plot_3d(ax, t_1=-line_len, t_2=line_len, c='r')

plt.savefig(os.path.join(data_folder, f'bar_holding_acc_{data_batch}.png'))
plt.show()