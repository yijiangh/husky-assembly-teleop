import os, logging
import json
import numpy as np
from skspatial.objects import Line, Point, Vector
import pybullet_planning as pp
import matplotlib.pyplot as plt
# from pybullet_mocap.common import load_robot

HERE = os.path.dirname(os.path.abspath(__file__))
j0_data_file_path = os.path.join(HERE, 'j0', 'j0_analysis.json')
j1_data_file_path = os.path.join(HERE, 'j1', 'j1_analysis.json')

# parse the json file
with open(j0_data_file_path, 'r') as file:
    j0_data = json.load(file)
with open(j1_data_file_path, 'r') as file:
    j1_data = json.load(file)

j0_point = np.array(j0_data['fitted_point'])
j0_axis = np.array(j0_data['fitted_axis'])

j1_point = np.array(j1_data['mean_circle_center'])
j1_axis = np.array(j1_data['mean_circle_normals'])

world_from_base_mocap = j1_data['takes'][0]["raw_data"][10]["base_mocap_pose"]

line_j0 = Line(point=j0_point, direction=j0_axis)
line_j1 = Line(point=j1_point, direction=j1_axis)
intersect_point = line_j0.intersect_line(line_j1, check_coplanar=False)

# use numpy to compute compute angle between j0_axis and j1_axis
angle = np.rad2deg(Vector(j0_axis).angle_between(Vector(j1_axis)))
print(f'angle between j0_axis and j1_axis: {angle}')

# project intersection point onto both lines and check the distance
proj_j0 = line_j0.project_point(intersect_point)
proj_j1 = line_j1.project_point(intersect_point)
dist_j0 = np.linalg.norm(proj_j0 - intersect_point)
dist_j1 = np.linalg.norm(proj_j1 - intersect_point)
print(f'distance between intersection point and projected point on j0_axis: {dist_j0}')
print(f'distance between intersection point and projected point on j1_axis: {dist_j1}')

# plot
ax = plt.figure().add_subplot(projection='3d')
line_j0.plot_3d(ax, t_1=0, t_2=0.2, c='b')
line_j1.plot_3d(ax, t_1=0, t_2=0.2, c='g')
intersect_point.plot_3d(ax, c='k')

# move intersect_point along j0_axis for a distance of 0.163
arm_base_link_origin = intersect_point - 0.163 * line_j0.direction
base_link_x_axis = Line(point=arm_base_link_origin, direction=j1_axis)
base_link_z_axis = Line(point=arm_base_link_origin, direction=j0_axis)
y_axis = np.cross(base_link_z_axis.direction, base_link_x_axis.direction)
y_axis = y_axis / np.linalg.norm(y_axis)
base_link_y_axis = Line(point=arm_base_link_origin, direction=y_axis)

base_link_x_axis.plot_3d(ax, t_1=0, t_2=0.2, c='r')
base_link_y_axis.plot_3d(ax, t_1=0, t_2=0.2, c='g')
base_link_z_axis.plot_3d(ax, t_1=0, t_2=0.2, c='b')
Point(arm_base_link_origin).plot_3d(ax, c='k')
# plt.show()

# construct a 4x4 transformation matrix
transformation_matrix = np.zeros((4, 4))
transformation_matrix[:3, 0] = base_link_x_axis.direction
transformation_matrix[:3, 1] = base_link_y_axis.direction
transformation_matrix[:3, 2] = base_link_z_axis.direction
transformation_matrix[:3, 3] = arm_base_link_origin
world_from_arm_base_link = pp.pose_from_tform(transformation_matrix)

pp.connect(use_gui=True, shadows=True, color=[0.9, 0.9, 1.0])
robot_urdf = os.path.join('/home/yijiangh/ros2_ws/src/husky-asembly-teleop/data','husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint.urdf')
with pp.HideOutput():
    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)
pp.set_pose(robot, world_from_base_mocap)

base_base_link = pp.link_from_name(robot, "base_footprint")
arm_base_link = pp.link_from_name(robot, "ur_arm_base_link")
# this is fixed
arm_base_link_from_base_footprint = pp.get_relative_pose(robot, base_base_link, arm_base_link)

base_mocap_from_arm_base = pp.multiply(pp.invert(world_from_base_mocap), world_from_arm_base_link)
base_mocap_from_base_footprint = pp.multiply(base_mocap_from_arm_base, arm_base_link_from_base_footprint)

# sensed data
pp.draw_pose(world_from_base_mocap)
pp.draw_pose(world_from_arm_base_link)

world_from_arm_base = pp.multiply(world_from_base_mocap, base_mocap_from_arm_base)
world_from_base_footprint = pp.multiply(world_from_base_mocap, base_mocap_from_base_footprint)
pp.draw_pose(world_from_arm_base, length=1.2)
pp.draw_pose(world_from_base_footprint, length=1.2)

# save this to a json file
output_file_path = os.path.join(HERE, 'calibrated_transformation_0804.json')
with open(output_file_path, 'w') as file:
    json.dump({
        'base_mocap_from_arm_base': [list(v) for v in base_mocap_from_arm_base],
        'base_mocap_from_base_footprint': [list(v) for v in base_mocap_from_base_footprint],
        }, 
        file, indent=4)

pp.wait_if_gui()
