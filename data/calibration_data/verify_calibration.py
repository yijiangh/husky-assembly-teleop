from cProfile import label
import os, logging
import json
import numpy as np
from skspatial.objects import Line, Point, Vector
import pybullet_planning as pp
import matplotlib.pyplot as plt
# from pybullet_mocap.common import load_robot

HUSKY_UR5e_JOINT_NAMES = ["ur_arm_shoulder_pan_joint", 
                      "ur_arm_shoulder_lift_joint",
                      "ur_arm_elbow_joint", 
                      "ur_arm_wrist_1_joint", 
                      "ur_arm_wrist_2_joint", 
                      "ur_arm_wrist_3_joint" ]

HERE = os.path.dirname(os.path.abspath(__file__))
# data_batch = 'verification'
data_batch = 'j0'
# data_batch = 'j1'

subfolder = ''
# subfolder = '20250311'
data_folder = os.path.join(HERE, subfolder, data_batch)

viewer = 0

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger()

# Create file handler
file_handler = logging.FileHandler(os.path.join(data_folder, f'log.txt'), mode='w')
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter('%(asctime)s - %(levelname)s - %(message)s'))

# Add handlers to the logger
logger.addHandler(file_handler)

pp.connect(use_gui=viewer, shadows=True, color=[0.9, 0.9, 1.0])
# robot_urdf = os.path.join('/home/yijiangh/ros2_ws/src/husky-asembly-teleop/data','husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint.urdf')
# robot_urdf = os.path.join('/home/yijiangh/ros2_ws/src/pybullet_mocap/data', 'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint.urdf')
robot_urdf = os.path.join(r'D:\0_Project\03-2025_husky_assembly\Code\husky-asembly-teleop\data',r'husky_urdf\mt_husky_moveit_config\urdf\husky_ur5_e_no_base_joint.urdf')
with pp.HideOutput():
    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)

calib_file_path = os.path.join(HERE, subfolder, 'calibrated_transformation_0804.json')
with open(calib_file_path, 'r') as file:
    data = json.load(file)
# base_mocap_from_base_footprint = data['base_mocap_from_base_footprint']
base_mocap_from_base_footprint = pp.unit_pose()

pp.draw_pose(pp.unit_pose())

json_files = [f for f in os.listdir(data_folder) if f.startswith('calibration_') and f.endswith('.json')]
tool0_from_flange_mocap_batches = []
world_from_diff_tfs = []
joint_confs = []
delta_x_vecs = []
for i, file_name in enumerate(json_files):
    logger.info('Working on file: %s', file_name)
    file_path = os.path.join(data_folder, file_name)

    with open(file_path, 'r') as file:
        data = json.load(file)

    for entry in data['raw_data']:
        flange_mocap_pose = entry.get("flange_mocap_pose", [])
        base_mocap_pose = entry.get("base_mocap_pose", [])
        conf = entry.get("joint_conf", [])

        # compute arm FK based on the base_mocap_pose and the calibrated offset
        world_from_footprint = pp.multiply(base_mocap_pose, base_mocap_from_base_footprint)
        pp.set_pose(robot, world_from_footprint)

        arm_joints = pp.joints_from_names(robot, HUSKY_UR5e_JOINT_NAMES)
        pp.set_joint_positions(robot, arm_joints, conf)

        world_from_tool0 = pp.get_link_pose(robot, pp.link_from_name(robot, 'ur_arm_tool0'))
        tool0_from_flange_mocap = pp.multiply(pp.invert(world_from_tool0), flange_mocap_pose)

        base_mocap_from_tool0 = pp.multiply(pp.invert(base_mocap_pose), world_from_tool0)
        base_mocap_from_flange_mocap = pp.multiply(pp.invert(base_mocap_pose), flange_mocap_pose)

        # decompose the quaternion of base_mocap_from_tool0 and get their x axis
        axis = 1
        x_axis_tool0 = pp.tform_from_pose(base_mocap_from_tool0)[:, axis]
        x_axis_flange = pp.tform_from_pose(base_mocap_from_flange_mocap)[:, axis]
        vec_diff = x_axis_tool0 + x_axis_flange
        # np.rad2deg(np.arccos(np.dot(x_axis_tool0, x_axis_flange)))
        delta_x_vecs.append(vec_diff)

        tool0_from_flange_mocap_batches.append(tool0_from_flange_mocap) 
        joint_confs.append(conf)

        pp.draw_pose(world_from_tool0)
        pp.draw_pose(flange_mocap_pose)
        pp.draw_pose(tool0_from_flange_mocap)

        pp.draw_pose(base_mocap_from_tool0)
        pp.draw_pose(base_mocap_from_flange_mocap)
        # pp.camera_focus_on_point(np.array(base_mocap_from_tool0[0]))

        pp.wait_if_gui('viz')
        pp.remove_all_debug()

pp.wait_if_gui()

fig = plt.figure()
ax = plt.subplot(111)

origins = [pose[0] for pose in tool0_from_flange_mocap_batches]
origin_mean = np.mean(origins, axis=0)
# compute distance to mean, scale to mm
distances = [1e3 * np.linalg.norm(origin - origin_mean) for origin in origins]
logger.info('Max origin distance to mean: %f', max(distances))

# plot distance in a line plot
# ax.plot(distances, label='origin to mean distance (mm)')

tfs = [pp.matrix_from_quat(pose[1]) for pose in tool0_from_flange_mocap_batches]
# for i in range(3):
#     x_axes = [tf[:3,i] for tf in tfs]
#     # compute angle between each axis and the mean
#     x_axis_mean = np.mean(x_axes, axis=0)
#     x_axis_mean = x_axis_mean / np.linalg.norm(x_axis_mean)
#     x_angles = []
#     for j in range(len(tfs)-1):
#         angle = np.rad2deg(np.arccos(np.dot(x_axes[j], x_axis_mean)))
#         x_angles.append(angle)

#     logger.info(f'Axis {i}')
#     logger.info('Max axis angle to mean vec: %f', np.mean(x_angles))
#     # Extract first joint values for x-axis
#     first_joint_values = [conf[0] for conf in joint_confs[:-1]]  # -1 to match length of x_angles
#     ax.scatter(first_joint_values, x_angles, label=f'axis {i} to mean angle (deg)')

first_joint_values = [conf[0] for conf in joint_confs]  # -1 to match length of x_angles
for i in range(3):
    axis_delta_x_vecs = [tf[:3,0][i] for tf in tfs]  # -1 to match length of x_angles
    ax.scatter(first_joint_values, axis_delta_x_vecs, label=f'diff vec axis {i} to mean angle (deg)')

# for i in range(6):
#     ax.plot([conf[i] for conf in joint_confs], label=f'joint value {i}', linewidth=0.2)

# Shrink current axis by 20%
box = ax.get_position()
ax.set_position([box.x0, box.y0, box.width * 0.8, box.height])

# Put a legend to the right of the current axis
ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
# plt.legend()

plt.savefig(os.path.join(data_folder, f'verification_{data_batch}.png'))
plt.show()

pp.disconnect()