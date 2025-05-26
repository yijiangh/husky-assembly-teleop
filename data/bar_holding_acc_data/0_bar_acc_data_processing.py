# https://github.com/CristianoPizzamiglio/circle-fitting-3d
# https://scikit-spatial.readthedocs.io/en/stable/index.html

# https://leomariga.github.io/pyRANSAC-3D/

import json, os
import logging, datetime
from skspatial.objects import Line, Points, Point
import matplotlib.pyplot as plt
import numpy as np
import pybullet_planning as pp

from compute_robot_com import *
import colorlog

MARKER_NAME_PAIRS = [
    ['5', '6'],
    ['7', '8'],
    ['2', '4'],
    ['1', '3']
]

# DATA_BATCH = '20250519_fixed_pos_vary_yaw'
DATA_BATCH = '20250519_vary_pos_vary_yaw'
EXPORT = 1
viewer = 0

HERE = os.path.dirname(os.path.abspath(__file__))
data_folder = os.path.join(HERE, DATA_BATCH)

# Configure logging
# Configure colorized logging
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

# Create file handler
file_handler = logging.FileHandler(os.path.join(data_folder, f'bar_holding_acc_processing_log_{DATA_BATCH}.txt'), mode='w')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))

# Add handlers to the logger
logger.addHandler(file_handler)

json_files = [f for f in os.listdir(data_folder) if f.startswith('bar_holding_acc_') and f.endswith('.json')]

pp.connect(use_gui=viewer, shadows=True, color=[0.9, 0.9, 1.0])
robot_urdf = os.path.join(HERE, '..', 'husky_urdf', 'mt_husky_moveit_config', 'urdf', 'husky_ur5_e_no_base_joint.urdf')

# Check if file exists
if not os.path.exists(robot_urdf):
    logger.error(f"Robot URDF file not found: {robot_urdf}")
    raise FileNotFoundError(f"Robot URDF file not found: {robot_urdf}")

# Create a ground plane for contact detection
p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, 0)

with pp.HideOutput():
    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)

polygon_id = None  # Track support polygon visualization

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
            p1 = np.array(marker_pts[m1]["pos"])
            p2 = np.array(marker_pts[m2]["pos"])
            # should be around 0.079m
            marker_dist = pp.get_distance(p1, p2)
            if marker_dist > 0.083 or marker_dist < 0.081:
                logger.warning(f"Distance between {m1} and {m2} is too small or large: {pp.get_distance(p1, p2)} m")

            err1 = np.array(marker_pts[m1]["error"])
            err2 = np.array(marker_pts[m2]["error"])
            # Check if marker errors are too large
            if np.linalg.norm(err1) > 0.002:
                logger.warning(f"Error for marker {m1} is too large: {np.linalg.norm(err1):.6f} m")
            if np.linalg.norm(err2) > 0.002:
                logger.warning(f"Error for marker {m2} is too large: {np.linalg.norm(err2):.6f} m")
            
            center = (p1 + p2) / 2
            center_points.append(center)

        line_fit = Line.best_fit(center_points)

        # Compute the direction vector of the fitted line
        line_direction = line_fit.direction / np.linalg.norm(line_fit.direction)

        # Get target bar pose
        goal_bar_pos, goal_bar_quat = entry.get("world_from_bar_pose", [])
        observed_bar_pos = line_fit.point

        # Construct rotation matrix using line_direction as the z-axis
        # First, find a non-parallel vector to create x and y axes
        if abs(line_direction[2]) < abs(line_direction[0]) and abs(line_direction[2]) < abs(line_direction[1]):
            temp_vector = np.array([0, 0, 1])
        elif abs(line_direction[0]) < abs(line_direction[1]):
            temp_vector = np.array([1, 0, 0])
        else:
            temp_vector = np.array([0, 1, 0])

        # Calculate x-axis (perpendicular to line_direction)
        x_axis = np.cross(temp_vector, line_direction)
        x_axis = x_axis / np.linalg.norm(x_axis)

        # Calculate y-axis to complete the right-handed coordinate system
        y_axis = np.cross(line_direction, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)

        # Construct rotation matrix [x_axis, y_axis, z_axis]
        rotation_matrix = np.column_stack((x_axis, y_axis, line_direction))

        # Convert rotation matrix to quaternion
        observed_bar_quat = pp.quat_from_matrix(rotation_matrix)

        observed_tf = pp.tform_from_pose((observed_bar_pos, observed_bar_quat))
        observed_z = observed_tf[:, 2]
        # Compare observed_z with line_direction
        # Calculate the angle between observed_z and line_direction
        dot_product = np.dot(observed_z[:3], line_direction)
        # Handle numerical precision by clipping
        angle = np.arccos(np.clip(dot_product, -1.0, 1.0))
        # If the vectors are pointing in opposite directions, we need to adjust
        if angle > np.pi/2:
            angle = np.pi - angle
        # Log error if the angle deviation is larger than 1e-5 rad
        if angle > 1e-5:
            logger.error(f"Angle deviation between observed_z and line_direction too large: {angle:.8f} rad")
        else:
            logger.info(f"Observed_z and line_direction are aligned. Angle: {angle:.8f} rad")

        # Log the observed bar position and orientation
        logger.info(f"Observed bar position: {observed_bar_pos}")
        logger.info(f"Observed bar quaternion: {observed_bar_quat}")

        # Calculate average of center points
        average_center = np.mean(center_points, axis=0)

        # Calculate distance between average center and observed bar position
        center_error = np.linalg.norm(average_center - observed_bar_pos)

        # Log the distance
        logger.info(f"Distance between average center and fitted line point: {center_error:.6f} m")

        # Log warning if the error is too large
        if center_error > 0.001:
            logger.error(f"Error between average center and fitted line point is too large: {center_error:.6f} m")

        # compute the distance betwen the observed bar position and the goal bar position
        bar_pos_error = np.linalg.norm(observed_bar_pos - goal_bar_pos)

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

        # Extract rotation matrix from goal bar quaternion
        goal_bar_rotation = pp.matrix_from_quat(goal_bar_quat)

        # Extract the axis from the rotation matrix (we assume Z axis of the bar is the main axis)
        goal_bar_axis = goal_bar_rotation[:, 2]  # Third column is the Z axis direction
        goal_bar_axis = goal_bar_axis / np.linalg.norm(goal_bar_axis)  # Normalize

        # Find the closest global axis to the goal bar's orientation
        goal_closest_axis = None
        goal_min_angle = float('inf')
        for axis_name, axis_vector in global_axes.items():
            # Compute the angle between the goal bar direction and the global axis
            dot_product = np.dot(goal_bar_axis, axis_vector)
            angle = np.arccos(np.clip(dot_product, -1.0, 1.0))
            
            # Check if the angle is close to 0 or pi
            if np.isclose(angle, 0, atol=tol) or np.isclose(angle, np.pi, atol=tol):
                goal_closest_axis = axis_name
                goal_min_angle = np.arccos(dot_product)
                break
            
            # Update the closest axis if the angle is smaller
            if angle < goal_min_angle:
                goal_closest_axis = axis_name
                goal_min_angle = angle

        # Log the goal bar axis information
        goal_display_angle = np.rad2deg(goal_min_angle)
        if goal_display_angle > 90:
            goal_display_angle = 180 - goal_display_angle
        logger.info('Closest global axis to the goal bar quat: %s (angle: %.2f deg)', 
                    goal_closest_axis, goal_display_angle)

        # Check if the observed axis matches the goal axis
        if goal_closest_axis != closest_axis:
            logger.error(f"Observed bar axis ({closest_axis}) differs from goal bar axis ({goal_closest_axis})")

        display_min_angle = np.rad2deg(min_angle)
        if display_min_angle > 90:
            display_min_angle = 180 - display_min_angle
        logger.info('Closest global axis to the line: %s (angle: %.2f deg)', closest_axis, display_min_angle)

        # * compute robot com data
        pp.set_pose(robot, footprint_pose)

        arm_joints = pp.joints_from_names(robot, HUSKY_UR5e_JOINT_NAMES)
        pp.set_joint_positions(robot, arm_joints, joint_conf)

        # Get the tool0 link pose
        world_from_tool0 = pp.get_link_pose(robot, pp.link_from_name(robot, 'ur_arm_tool0'))
        observed_bar_pose = (observed_bar_pos, observed_bar_quat)
        tool0_from_bar_center = pp.multiply(pp.invert(world_from_tool0), observed_bar_pose)
        # this is the TOOL0_FROM_GRIPPER_TCP

        # Compute the position error vector in world frame
        pos_error_world = observed_bar_pos - goal_bar_pos
        logger.info(f"Position error in world frame: {pos_error_world}")

        # Get the rotation matrix from world to tool0
        tool0_rotation_matrix = pp.matrix_from_quat(world_from_tool0[1])
        tool0_rotation_matrix_transpose = tool0_rotation_matrix.T  # Transpose for inverse rotation

        # Transform the position error vector to tool0 frame
        pos_error_tool0 = np.dot(tool0_rotation_matrix_transpose, pos_error_world)
        logger.info(f"Position error in tool0 frame: {pos_error_tool0}")

        # Compute center of mass for just the UR5e arm
        arm_com = compute_ur5e_com(robot)
        # print(f"UR5e Arm Center of Mass: {arm_com}")

        # Compute center of mass for the entire robot
        robot_com = compute_robot_com(robot)
        # print(f"Entire Robot Center of Mass: {robot_com}")

        # Get wheel contact points
        contact_points = get_wheel_contact_points(robot)
        # print(f"Wheel contact points: {contact_points}")

        # Sort contact points to form a consistent polygon (for export)
        center = compute_support_polygon_center(contact_points)

        def angle_from_center(point):
            return np.arctan2(point[1] - center[1], point[0] - center[0])

        sorted_contact_points = sorted(contact_points, key=angle_from_center)

        # Compute the center of the support polygon
        support_polygon_center = compute_support_polygon_center(contact_points)
        # print(f"Support Polygon Center: {support_polygon_center}")

        # Compute projected distance between robot COM and support polygon center
        distance_com_to_polygon = compute_projected_distance(robot_com, support_polygon_center)
        # print(f"Projected Distance from Robot COM to Support Polygon Center: {distance_com_to_polygon} m")

        # Export data if requested
        if EXPORT:
            take_data = {
                'raw_data': entry, 
                'footprint_pose': footprint_pose, 
                'joint_conf': joint_conf, 
                'point_centers': [list(center) for center in center_points], 
                'tool0_from_bar_center' : tool0_from_bar_center,
                'fitted_line': {'point' : list(line_fit.point), 'direction' : list(line_fit.direction)}, 
                'closest_axis': closest_axis,
                'angle_to_closest_axis': display_min_angle, # ! this is degrees!
                'bar_pos_error' : float(bar_pos_error),
                'bar_pos_error_vector_tool0': pos_error_tool0.tolist(),
                'ur5e_com': arm_com.tolist(),
                'robot_com': robot_com.tolist(),
                'support_polygon_vertices': [point.tolist() for point in sorted_contact_points],
                'support_polygon_center': support_polygon_center.tolist(),
                'distance_com_to_polygon_center': float(distance_com_to_polygon)
                }
            new_data.append(take_data)

        centers.append(center_points)
        fitted_lines.append(line_fit)
        base_positions.append(footprint_pose[0])
        # print('center_points:', center_points)
        # print('fitted_line:', line_fit.point, line_fit.direction)

        if viewer:
            # Remove previous polygon if it exists
            # if polygon_id is not None:
            #     p.removeBody(polygon_id)

            # Draw new support polygon
            polygon_id = draw_support_polygon(contact_points)

            # Visualize support polygon center with a green sphere
            polygon_center_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[0, 0.8, 0, 0.7])
            p.createMultiBody(baseVisualShapeIndex=polygon_center_visual, basePosition=support_polygon_center)

            # Visualize arm CoM with a small red sphere
            arm_com_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 0.7])
            p.createMultiBody(baseVisualShapeIndex=arm_com_visual, basePosition=arm_com)

            # Visualize entire robot CoM with a small blue sphere
            robot_com_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.05, rgbaColor=[0, 0, 1, 0.7])
            p.createMultiBody(baseVisualShapeIndex=robot_com_visual, basePosition=robot_com)

            # Draw a line from robot COM to support polygon center to visualize distance
            p.addUserDebugLine(
                [robot_com[0], robot_com[1], 0.02],  # Slightly above ground
                [support_polygon_center[0], support_polygon_center[1], 0.02],
                lineColorRGB=[1, 0.5, 0],  # Orange
                lineWidth=4.0
            )

            # Visualize wheel contact points
            for point in contact_points:
                wheel_contact_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 1, 0, 0.7])
                p.createMultiBody(baseVisualShapeIndex=wheel_contact_visual, basePosition=point)

            pp.draw_pose(footprint_pose)
            # pp.wait_if_gui()

if viewer:
    # move the robot away to see the coms
    pp.set_pose(robot, pp.unit_pose())

if EXPORT:
    compiled_file_path = os.path.join(data_folder, f'compiled_bar_holding_acc_{DATA_BATCH}.json')
    with open(compiled_file_path, 'w') as file:
        json.dump(new_data, file, indent=4)
    logger.info('Exported data to %s', compiled_file_path)

ax = plt.figure().add_subplot(projection='3d')

# plot all footprint point, observed centers, fitted lines
line_len = 0.5
for line_fit, marker_centers, robot_base_pos in zip(fitted_lines, centers, base_positions):
    line_fit.plot_3d(ax, t_1=-line_len, t_2=line_len, c='k')
    Points(marker_centers).plot_3d(ax, c='b', depthshade=False)
    Point(robot_base_pos).plot_3d(ax, c='g', depthshade=False)

if EXPORT:
    plt.savefig(os.path.join(data_folder, f'bar_holding_acc_{DATA_BATCH}.png'))
    logger.info('Exported plot to %s', os.path.join(data_folder, f'bar_holding_acc_{DATA_BATCH}.png'))

plt.show()