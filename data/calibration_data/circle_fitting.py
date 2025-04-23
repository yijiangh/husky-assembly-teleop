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

data_batch = 'j0'
EXPORT = False

HERE = os.path.dirname(os.path.abspath(__file__))
data_folder = os.path.join(HERE, data_batch)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger()

# Create file handler
file_handler = logging.FileHandler(os.path.join(data_folder, f'circle_fitting_log_{data_batch}.txt'), mode='w')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))

# Add handlers to the logger
logger.addHandler(file_handler)

# load each json file start with the name "calibration_" in the data folder
json_files = [f for f in os.listdir(data_folder) if f.startswith('calibration_') and f.endswith('.json')]

centers = []
normals = []
radius = []
all_points = []
max_pt_project_distance = []
new_data = {'takes': []}

all_base_mocap_pos = []
all_base_mocap_quats = []

for i, file_name in enumerate(json_files):
    logger.info('Working on file: %s', file_name)
    file_path = os.path.join(data_folder, file_name)

    # Load the JSON file
    with open(file_path, 'r') as file:
        data = json.load(file)

    # Parse the origin of mocap_pose data
    points = []
    base_mocap_origins = []
    base_mocap_quats = []
    for entry in data['raw_data']:
        flange_mocap_pose = entry.get("flange_mocap_pose", [])
        base_mocap_pose = entry.get("base_mocap_pose", [])
        if flange_mocap_pose and base_mocap_pose:
            base_from_flange = pp.multiply(pp.invert(base_mocap_pose), flange_mocap_pose)
            origin = base_from_flange[0]  # Assuming the first element is the origin
            points.append(origin)

            base_mocap_origins.append(base_mocap_pose[0])
            base_mocap_quats.append(base_mocap_pose[1])

    circle_3d = Circle3D(points)
    distances = []
    for point in points:
        point = Point(point)
        point_on_plane = circle_3d._plane.project_point(point)
        vector_to_projected_point = point_on_plane - circle_3d.center
        vector_to_projected_point = vector_to_projected_point / np.linalg.norm(vector_to_projected_point)
        projected_point = circle_3d.center + vector_to_projected_point * circle_3d.radius
        distances.append(np.linalg.norm(projected_point - point))
    max_pt_project_distance.append(max(distances))

    all_base_mocap_pos.extend(base_mocap_origins)
    all_base_mocap_quats.extend(base_mocap_quats)

    base_mocap_origins = np.array(base_mocap_origins)
    base_mocap_origin_mean = np.mean(base_mocap_origins, axis=0)
    base_origin_distances = np.linalg.norm(base_mocap_origins - base_mocap_origin_mean, axis=1)
    logger.info('max distance between base_mocap_origins and mean: %s', max(base_origin_distances))

    base_mocap_quat_mean = np.mean(base_mocap_quats, axis=0)
    base_quat_distance = np.linalg.norm(base_mocap_quats - base_mocap_quat_mean, axis=1)
    logger.info('max distance between base_mocap_quats and mean: %s', max(base_quat_distance))

    # ! use the averaged base mocap pose to transform the fitted circle back to the world frame
    world_from_base_mocap = (base_mocap_origin_mean, base_mocap_quat_mean)
    centers.append(pp.tform_point(world_from_base_mocap, circle_3d.center))
    tf = pp.tform_from_pose(world_from_base_mocap)
    world_from_normal = tf[:3, :3] @ circle_3d.normal
    normals.append(world_from_normal)
    radius.append(circle_3d.radius)
    all_points.extend(pp.apply_affine(world_from_base_mocap, points))

    if EXPORT:
        take_data = {
            'file_name': file_name,
            'raw_data': data['raw_data'], 
            'center': list(centers[-1]), 
            'radius': radius[-1], 
            'normal': list(normals[-1]), 
            # 'base_origin_distances_to_mean': list(base_origin_distances), 
            # 'base_quat_distances_to_mean': list(base_quat_distance),
            }
        new_data['takes'].append(take_data)

###################
# Compute mean of all base mocap positions and analyze deviations
all_base_mocap_pos = np.array(all_base_mocap_pos)
base_mocap_pos_mean = np.mean(all_base_mocap_pos, axis=0)
base_pos_distances = np.linalg.norm(all_base_mocap_pos - base_mocap_pos_mean, axis=1)

logger.info('Mean base mocap position: %s', base_mocap_pos_mean)
logger.info('Max distance between base_mocap_pos and mean: %.5f', max(base_pos_distances))
logger.info('Min distance between base_mocap_pos and mean: %.5f', min(base_pos_distances))
logger.info('Average distance between base_mocap_pos and mean: %.5f', np.mean(base_pos_distances))

# Calculate deviations along each axis
x_deviations = np.abs(all_base_mocap_pos[:, 0] - base_mocap_pos_mean[0])
y_deviations = np.abs(all_base_mocap_pos[:, 1] - base_mocap_pos_mean[1])
z_deviations = np.abs(all_base_mocap_pos[:, 2] - base_mocap_pos_mean[2])

# Plot position deviations by axis
fig, ax = plt.subplots(figsize=(10, 6))
indices = np.arange(len(base_pos_distances))
ax.plot(indices, x_deviations, 'r-', label='X-axis')
ax.plot(indices, y_deviations, 'g-', label='Y-axis')
ax.plot(indices, z_deviations, 'b-', label='Z-axis')
ax.plot(indices, base_pos_distances, 'k--', label='Total distance')
ax.set_xlabel('Sample index')
ax.set_ylabel('Distance from mean (m)')
ax.set_title('Base Mocap Position Deviations by Axis')
ax.legend()
plt.savefig(os.path.join(data_folder, f'{data_batch}_mocap_base_pos_deviation_by_axis.png'))

# Log the maximum deviations
logger.info('Max X-axis deviation: %.5f', max(x_deviations))
logger.info('Max Y-axis deviation: %.5f', max(y_deviations))
logger.info('Max Z-axis deviation: %.5f', max(z_deviations))

###################

all_base_mocap_quats = np.array(all_base_mocap_quats)

# Compute rotation matrices from quaternions
rotation_matrices = np.array([pp.tform_from_pose(([0,0,0],quat)) for quat in all_base_mocap_quats])

# Extract x, y, z axes from each rotation matrix
x_axes = rotation_matrices[:, :3, 0]  # First column is x-axis
y_axes = rotation_matrices[:, :3, 1]  # Second column is y-axis
z_axes = rotation_matrices[:, :3, 2]  # Third column is z-axis

# Compute mean axes
mean_x_axis = Vector(np.mean(x_axes, axis=0))
mean_y_axis = Vector(np.mean(y_axes, axis=0))
mean_z_axis = Vector(np.mean(z_axes, axis=0))

# Normalize mean axes
mean_x_axis = mean_x_axis.unit()
mean_y_axis = mean_y_axis.unit()
mean_z_axis = mean_z_axis.unit()

# Compute angle deviations
x_deviations = [np.rad2deg(Vector(axis).angle_between(mean_x_axis)) for axis in x_axes]
y_deviations = [np.rad2deg(Vector(axis).angle_between(mean_y_axis)) for axis in y_axes]
z_deviations = [np.rad2deg(Vector(axis).angle_between(mean_z_axis)) for axis in z_axes]

logger.info('Max angle deviation for x-axis: %.2f degrees', max(x_deviations))
logger.info('Max angle deviation for y-axis: %.2f degrees', max(y_deviations))
logger.info('Max angle deviation for z-axis: %.2f degrees', max(z_deviations))

# Plot the angle deviations
fig, ax = plt.subplots()
indices = list(range(len(x_deviations)))
ax.plot(indices, x_deviations, 'r-', label='X-axis')
ax.plot(indices, y_deviations, 'g-', label='Y-axis')
ax.plot(indices, z_deviations, 'b-', label='Z-axis')
ax.set_xlabel('Sample index')
ax.set_ylabel('Angle deviation (degrees)')
ax.set_title('Rotation axes deviations from mean')
ax.legend()
plt.savefig(os.path.join(data_folder, f'{data_batch}_mocap_base_axis_deviation_batch_{i}.png'))
# plt.show()

###################

line_fit = Line.best_fit(centers)
logger.info(f'{data_batch} fitted axis : {line_fit.direction}')

logger.info('max_pt_project_distance: %s', max_pt_project_distance)

angle_diff = []
for n in normals:
    v = Vector(n)
    if v.angle_between(line_fit.direction) > np.pi/2:
        line_fit.direction = -line_fit.direction
    angle_diff.append(np.rad2deg(v.angle_between(line_fit.direction)))
logger.info('angle diff between center normals and fitted line (deg): %s', list(angle_diff))

mean_normal = np.mean(normals, axis=0)
mean_angle_diff = []
for n in normals:
    v = Vector(n)
    mean_angle_diff.append(np.rad2deg(v.angle_between(mean_normal)))
logger.info('angle diff between center normals and mean normal (deg): %s', list(mean_angle_diff))

# projection on a plane with the averaged fitted normal and then compute the distance to the center
project_plane = Plane(point=centers[0], normal=mean_normal)
projected_centers = [project_plane.project_point(center) for center in centers]
# compute the max distance difference between projected centers and the mean center
mean_projected_center = np.mean(projected_centers, axis=0)
center_distances = np.linalg.norm(projected_centers - mean_projected_center, axis=1)
logger.info('max distance between centers and mean center: %s', max(center_distances))

# # compute distance between line_fit.point and each point in centers, then take the max
# distances = []
# for center in centers:
#     distances.append(np.linalg.norm(line_fit.point - center))
projected_fitted_point = project_plane.project_point(line_fit.point)
center_distances = np.linalg.norm(projected_centers - projected_fitted_point, axis=1)
logger.info('max distance between centers and line_fit.point: %s', max(center_distances))

if EXPORT:
    analysis_file_path = os.path.join(data_folder, f'{data_batch}_analysis.json')
    new_data['fitted_point'] = list(line_fit.point)
    new_data['fitted_axis'] = list(line_fit.direction)
    new_data['mean_circle_center'] = list(np.mean(centers, axis=0))
    new_data['mean_circle_normals'] = list(np.mean(normals, axis=0))
    new_data['angle_diff_fitted_axis_and_normals_per_take'] = list(angle_diff)
    new_data['max_pt_project_distance_per_take'] = list(max_pt_project_distance)
    with open(analysis_file_path, 'w') as file:
        json.dump(new_data, file, indent=4)

ax = plt.figure().add_subplot(projection='3d')

line_len = 0.5
line_fit.plot_3d(ax, t_1=-line_len, t_2=line_len, c='k'),
Points(all_points).plot_3d(ax, c='b', depthshade=False),
Points(centers).plot_3d(ax, c='g', depthshade=False),
for o, v in zip(centers, normals):
    Line(point=o, direction=v).plot_3d(ax, t_1=-line_len, t_2=line_len, c='r')

plt.savefig(os.path.join(data_folder, f'{data_batch}.png'))
plt.show()