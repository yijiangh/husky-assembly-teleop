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

# Configure logging
# timestamp = datetime.now().strftime("%Y%m%d_%H%M")
logging.basicConfig(filename=f'circle_fitting_log_{data_batch}.txt', level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

HERE = os.path.dirname(os.path.abspath(__file__))

data_folder = os.path.join(HERE, data_batch)

# load each json file start with the name "calibration_" in the data folder
json_files = [f for f in os.listdir(data_folder) if f.startswith('calibration_') and f.endswith('.json')]

centers = []
normals = []
radius = []
all_points = []
max_pt_project_distance = []
new_data = {'takes': []}

for i, file_name in enumerate(json_files):
    logging.info('Working on file: %s', file_name)
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
    logging.info('max distance between base_mocap_origins and mean: %s', max(base_origin_distances))

    base_mocap_quats = np.array(base_mocap_quats)
    mean = np.mean(base_mocap_quats, axis=0)
    base_quat_distance = np.linalg.norm(base_mocap_quats - mean, axis=1)
    logging.info('max distance between base_mocap_quats and mean: %s', max(base_quat_distance))

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
logging.info(f'{data_batch} axis fix : {line_fit.direction}')

angle_diff = []
for n in normals:
    angle_diff.append(np.rad2deg(n.angle_between(line_fit.direction)))
logging.info('angle diff between fitted line and center normals (deg): %s', angle_diff)
logging.info('max_pt_project_distance: %s', max_pt_project_distance)

if EXPORT:
    new_data['fitted_axis'] = list(line_fit.direction)
    new_data['angle_diff_fitted_axis_and_normals_per_take'] = list(angle_diff)
    new_data['max_pt_project_distance_per_take'] = list(max_pt_project_distance)
    with open(file_path, 'w') as file:
        json.dump(new_data, file, indent=4)

ax = plt.figure().add_subplot(projection='3d')

line_fit.plot_3d(ax, t_1=-7, t_2=7, c='k'),
Points(all_points).plot_3d(ax, c='b', depthshade=False),
for o, v in zip(centers, normals):
    Line(point=o, direction=v).plot_3d(ax, t_1=-7, t_2=7, c='r')
plt.show()
plt.savefig(f'{data_batch}.png')