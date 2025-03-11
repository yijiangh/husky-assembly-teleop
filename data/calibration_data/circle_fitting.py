# https://github.com/CristianoPizzamiglio/circle-fitting-3d
# https://scikit-spatial.readthedocs.io/en/stable/index.html

# https://leomariga.github.io/pyRANSAC-3D/

import json, os
import logging, datetime
from circle_fitting_3d import Circle3D
from skspatial.objects import Line, Points, Point
from skspatial.plotting import plot_3d
import matplotlib.pyplot as plt
import numpy as np

data_batch = 'j0'
EXPORT = True

HERE = os.path.dirname(os.path.abspath(__file__))
data_folder = os.path.join(HERE, data_batch)

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger()

# Create file handler
file_handler = logging.FileHandler(os.path.join(data_folder, f'circle_fitting_log_{data_batch}.txt'))
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
        mocap_pose = entry.get("flange_mocap_pose", [])
        base_mocap_pose = entry.get("base_mocap_pose", [])
        if mocap_pose:
            origin = mocap_pose[0]  # Assuming the first element is the origin
            points.append(origin)
        if base_mocap_pose:
            base_mocap_origins.append(base_mocap_pose[0])
            base_mocap_quats.append(base_mocap_pose[1])

    circle_3d = Circle3D(points)
    centers.append(circle_3d.center)
    normals.append(circle_3d.normal)
    radius.append(circle_3d.radius)
    all_points.extend(points)

    distances = []
    for point in points:
        point = Point(point)
        point_on_plane = circle_3d._plane.project_point(point)
        vector_to_projected_point = point_on_plane - circle_3d.center
        vector_to_projected_point = vector_to_projected_point / np.linalg.norm(vector_to_projected_point)
        projected_point = circle_3d.center + vector_to_projected_point * circle_3d.radius
        distances.append(np.linalg.norm(projected_point - point))
    max_pt_project_distance.append(max(distances))

    base_mocap_origins = np.array(base_mocap_origins)
    mean = np.mean(base_mocap_origins, axis=0)
    base_origin_distances = np.linalg.norm(base_mocap_origins - mean, axis=1)
    logger.info('max distance between base_mocap_origins and mean: %s', max(base_origin_distances))

    base_mocap_quats = np.array(base_mocap_quats)
    mean = np.mean(base_mocap_quats, axis=0)
    base_quat_distance = np.linalg.norm(base_mocap_quats - mean, axis=1)
    logger.info('max distance between base_mocap_quats and mean: %s', max(base_quat_distance))

    if EXPORT:
        take_data = {
            'file_name': file_name,
            'raw_data': data['raw_data'], 
            'center': list(circle_3d.center), 
            'radius': circle_3d.radius, 
            'normal': list(circle_3d.normal), 
            'base_origin_distances_to_mean': list(base_origin_distances), 
            'base_quat_distances_to_mean': list(base_quat_distance),
            }
        new_data['takes'].append(take_data)

line_fit = Line.best_fit(centers)
logger.info(f'{data_batch} fitted axis : {line_fit.direction}')

logger.info('max_pt_project_distance: %s', max_pt_project_distance)

angle_diff = []
for n in normals:
    angle_diff.append(np.rad2deg(n.angle_between(line_fit.direction)))
logger.info('angle diff between center normals and fitted line (deg): %s', angle_diff)

mean_center = np.mean(normals, axis=0)
mean_angle_diff = []
for n in normals:
    mean_angle_diff.append(np.rad2deg(n.angle_between(mean_center)))
logger.info('angle diff between center normals and mean normal (deg): %s', mean_angle_diff)

# compute the max distance difference between centers and the mean center
mean_center = np.mean(centers, axis=0)
center_distances = np.linalg.norm(centers - mean_center, axis=1)
logger.info('max distance between centers and mean center: %s', max(center_distances))

# compute distance between line_fit.point and each point in centers, then take the max
distances = []
for center in centers:
    distances.append(np.linalg.norm(line_fit.point - center))
logger.info('max distance between centers and line_fit.point: %s', max(distances))

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

line_fit.plot_3d(ax, t_1=-7, t_2=7, c='k'),
Points(all_points).plot_3d(ax, c='b', depthshade=False),
Points(centers).plot_3d(ax, c='g', depthshade=False),
for o, v in zip(centers, normals):
    Line(point=o, direction=v).plot_3d(ax, t_1=-7, t_2=7, c='r')

plt.show()
plt.savefig(os.path.join(data_folder, f'{data_batch}.png'))