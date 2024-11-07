import sys, os, argparse
import time
import random
import socket, json
import struct
from threading import Thread
# from plyer import notification

import numpy as np
import pybullet_planning as pp

from pybullet_mocap.optitrack.NatNetClient import NatNetClient
from pybullet_mocap.optitrack.Utils import print_configuration
from pybullet_mocap import DATA_DIRECTORY
from tracikpy import TracIKSolver
# import ikfast_ur5e

from compas_robots import RobotModel
from compas_fab.robots import RobotSemantics
from compas_fab.robots import Robot as RobotClass

LOCAL_SERVER = False
# ! Emre might need to set it to 180
CLIENT_IP = '192.168.0.7' # Set to your own IP
LOCAL_SERVER_IP = 'localhost' # '127.0.0.1'
MOCAP_IP = '192.168.0.117'

HUSKY_IP = '192.168.0.113'
HUSKY_UDP_PORT = 65432
HUSKY_TCP_PORT = 54321

HERE = os.path.dirname(__file__)

yup_tform = np.eye(4)
yup_tform[:3,0] = [0, 1, 0]
yup_tform[:3,1] = [0, 0, 1]
yup_tform[:3,2] = [1, 0, 0]
zup_from_yup = pp.pose_from_tform(yup_tform)

# <link name="bar_tcp"/>
# <joint name="tool0-bar_tcp_fixed_joint" type="fixed">
#   <origin rpy="0 0 3.141592653589793" xyz="0 0 0.138"/>
#   <parent link="robotiq_85_mount"/>
#   <child link="bar_tcp"/>
# </joint>
TOOL0_FROM_GRIPPER_TCP = pp.Pose(point=(0, 0, 0.138), euler=pp.Euler(yaw=np.pi))

HUSKYU_JOINT_NAMES = [
                      "ur_arm_shoulder_pan_joint", 
                      "ur_arm_shoulder_lift_joint",
                      "ur_arm_elbow_joint", 
                      "ur_arm_wrist_1_joint", 
                      "ur_arm_wrist_2_joint", 
                      "ur_arm_wrist_3_joint" ]
WHEEL_JOINT_NAMES = [
                      "front_right_wheel", 
                      "rear_right_wheel",
                      "front_left_wheel", 
                      "rear_left_wheel" ]
JOINT_JUMP_THRESHOLD = np.pi/3
POS_STEP_SIZE = 0.01
ORI_STEP_SIZE = np.pi/18

name_from_mocap_id = {
    1028 : 'husky0804',
    1011 : 'bar',
    1030 : 'foundation_bar',
    # 1029 : 'greybox',
    # 1032 : 'ur_shoulder_link',
}

# goal registar for optitrack to overwrite
rigid_body_poses = {}

###################

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame( new_id, position, rotation ):
    global rigid_body_poses
    rigid_body_poses[new_id] = (position, rotation)
    # print( "Received frame for rigid body", new_id )
    # print( "Received frame for rigid body", new_id," ",position," ",rotation )

def send_base_arm_trajectory_command(socket_server, joint_names, joint_positions, time_steps):
    # check if jointPositions and timeSteps have the same size
    if (len(joint_positions) != len(time_steps)):
        print("Error: jointPositions and timeSteps have different sizes")
        return

    traj = [] # create an empty array
    for i in range(len(joint_positions)):
        if (len(joint_positions[i]) != 8):
            print("Error: jointPositions[" + str(i) + "] has length " + str(len(joint_positions[i])) + " instead of 8")
            return
        traj_point = {}
        traj_point["xVel"] = joint_positions[i][0]
        traj_point["angVel"] = joint_positions[i][1]
        traj_point["q1"] = joint_positions[i][2]
        traj_point["q2"] = joint_positions[i][3]
        traj_point["q3"] = joint_positions[i][4]
        traj_point["q4"] = joint_positions[i][5]
        traj_point["q5"] = joint_positions[i][6]
        traj_point["q6"] = joint_positions[i][7]
        traj_point["time_from_start"] = time_steps[i]
        traj.append(traj_point)

    j = {}
    j["trajectory"] = traj
    j["joint_names"] = joint_names

    j_file = json.dumps(j)
    encoded_json = j_file.encode('utf-8')
    msg = struct.pack('>I', len(encoded_json)) + encoded_json
    print("***************************")
    print("Sending goal trajectory with pts = " + str(len(joint_positions)) + " and duration = " + str(time_steps[-1]))
    print('Data size = %d' % len(encoded_json))

    try:
        socket_server.sendall(msg)
    except socket.error as e:
        print("error while sending: %s" %e)

########################

def send_gripper_command(socket_server, gripper_pos):
    assert gripper_pos >= 0 and gripper_pos <= 255
    j = {}
    j["gripper_pos"] = gripper_pos

    j_file = json.dumps(j)
    encoded_json = j_file.encode('utf-8')
    msg = struct.pack('>I', len(encoded_json)) + encoded_json
    print("***************************")
    print("Sending goal gripper pose = " + str(gripper_pos))
    print('Data size = %d' % len(encoded_json))

    try:
        socket_server.sendall(msg)
    except socket.error as e:
        print("error while sending: %s" %e)

########################

def load_robot(load_calib_tip=False):
    robot_urdf = os.path.join(DATA_DIRECTORY,'husky_urdf/mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint.urdf')
    robot_srdf = os.path.join(DATA_DIRECTORY, 'husky_urdf/mt_husky_moveit_config/config/husky.srdf')

    if load_calib_tip:
        gripper_obj = os.path.join(DATA_DIRECTORY,'calibration_tip.stl')
        gripper_scale = 1
    else:
        gripper_obj = os.path.join(DATA_DIRECTORY,'husky_urdf/robotiq_85/meshes/static/robotiq_85_close_20mm.obj')
        gripper_scale = 1

    # gripper_obj = os.path.join(DATA_DIRECTORY,'husky_urdf/robotiq_85/meshes/static/robotiq_85_open.obj')
    # robot_urdf = os.path.join(HERE,'robotiq_85/urdf/robotiq_85_gripper_simple.urdf')
    # robot_urdf = os.path.join(HERE,'mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e.urdf')
    # print(robot_urdf)
    assert os.path.exists(robot_urdf)
    assert os.path.exists(gripper_obj)

    move_group = 'manipulator'
    robot_model = RobotModel.from_urdf_file(robot_urdf)
    robot_semantics = RobotSemantics.from_srdf_file(robot_srdf, robot_model)
    # cp_robot = RobotClass(robot_model, semantics=robot_semantics)

    robot = pp.load_pybullet(robot_urdf, fixed_base=False, cylinder=False)

    # ik_solver = lambda x: [0,0,0,0,0,0]
    # if not ik_from_arm_base:
        # ik_solver = TracIKSolver(robot_urdf, "world_link", "ur_arm_tool0")
    # else:
    ik_solver = TracIKSolver(robot_urdf, "ur_arm_base_link", "ur_arm_tool0")
    # pp.camgera_focus_on_body(robot)

    # get disabled collision pairs from SRDF
    disabled_self_collision_link_names = robot_semantics.disabled_collisions
    disabled_collisions = get_disabled_collisions(robot, disabled_self_collision_link_names) 

    tool0_pose = pp.get_link_pose(robot, pp.link_from_name(robot, 'ur_arm_tool0'))
    # pp.draw_pose(tool0_pose)
    ee = pp.create_obj(gripper_obj, scale=gripper_scale) 
    pp.set_pose(ee, pp.multiply(tool0_pose, pp.Pose(euler=pp.Euler(yaw=-np.pi/2))))
    
    ee_attachment = pp.create_attachment(robot, pp.link_from_name(robot, 'ur_arm_tool0'), ee)

    # tool0_from_ee = pp.Pose(euler=pp.Euler(yaw=-np.pi/2), point=[0,0,0.138])
    # tcp_pose = pp.multiply(tool0_pose, tool0_from_ee)
    # tcp_pose = pp.get_link_pose(robot, pp.link_from_name(robot, 'central_tcp'))
    # pp.draw_pose(tcp_pose)

    return robot, ee_attachment, ik_solver, disabled_collisions

def get_disabled_collisions(robot, disabled_self_collision_link_names):
    """get robot's link-link tuples disabled from collision checking

    Returns
    -------
    set of int-tuples
        int for link index in pybullet
    """
    return {tuple(pp.link_from_name(robot, link)
                  for link in pair if pp.has_link(robot, link))
                  for pair in disabled_self_collision_link_names}

def get_custom_limits(robot, custom_limits=None):
    """[summary]

    Returns
    -------
    [type]
        {joint index : (lower limit, upper limit)}
    """
    custom_limits = custom_limits or {}
    limits = {pp.joint_from_name(robot, joint): limits
              for joint, limits in custom_limits.items()}
    return limits

def check_path(joints, path, collision_fn=None, jump_threshold=None, diagnosis=False):
    """return False if path is not valid
    """
    joint_jump_thresholds = jump_threshold or [JOINT_JUMP_THRESHOLD for jt in joints]
    for jt1, jt2 in zip(path[:-1], path[1:]):
        delta_j = np.abs(np.array(jt1) - np.array(jt2))
        if any(delta_j > np.array(joint_jump_thresholds)):
            return False
    if collision_fn is not None:
        for q in path:
            if collision_fn(q, diagnosis):
                return False
    return True

ORTHOGONAL_GROUND = True
from itertools import cycle
from pybullet_planning import multiply, Pose, Euler, Point
def get_grasp_pose(direction, angle, offset=1e-3):
    # tool0_from_object
    #direction = Pose(euler=Euler(roll=np.pi / 2, pitch=direction))
    return multiply(Pose(point=Point(z=offset)),
                    Pose(euler=Euler(yaw=angle)),
                    direction
                    # Pose(point=Point(z=translation)),
                    # Pose(euler=Euler(roll=(1-reverse) * np.pi)
                    )

def plan_pickup_motion(robot, ik_solver, bar_body, attachments, obstacles, 
                       debug=False, ik_from_arm_base=True, disabled_collisions=None):
    # plan a transit motion from init conf to pick_approach conf  
    custom_limits = get_custom_limits(robot, {})
    resolutions = np.ones(6) * 0.05
    disabled_collisions = disabled_collisions or {}
    extra_disabled_collisions = [
        ((robot, pp.link_from_name(robot, 'ur_arm_wrist_3_link')), 
         (attachments[0].child, pp.BASE_LINK)), 
         # pp.link_from_name(ee_body, 'robotiq_85_base_link'))),
        ]

    # joints = pp.get_movable_joints(robot)
    movable_joints = pp.joints_from_names(robot, HUSKYU_JOINT_NAMES)
    tool_link = pp.link_from_name(robot, 'ur_arm_tool0')
    gripper_tcp_from_tool0 = pp.invert(TOOL0_FROM_GRIPPER_TCP)

    sample_fn = pp.get_sample_fn(robot, movable_joints, custom_limits=custom_limits)
    distance_fn = pp.get_distance_fn(robot, movable_joints) #, weights=weights)
    extend_fn = pp.get_extend_fn(robot, movable_joints, resolutions=resolutions)
    transit_collision_fn = pp.get_collision_fn(robot, movable_joints, obstacles=obstacles,
                                                attachments=attachments, 
                                                self_collisions=1,
                                                disabled_collisions=disabled_collisions, extra_disabled_collisions=extra_disabled_collisions,
                                                custom_limits=custom_limits, 
                                                max_distance=0)
    extra_disabled_collisions += [
        ((bar_body, pp.BASE_LINK), 
         (attachments[0].child, pp.BASE_LINK)),
    ]
    approach_collision_fn = pp.get_collision_fn(robot, movable_joints, obstacles=obstacles,
                                                attachments=attachments, 
                                                self_collisions=1,
                                                disabled_collisions=disabled_collisions, extra_disabled_collisions=extra_disabled_collisions,
                                                custom_limits=custom_limits, 
                                                max_distance=0)

    # See: https://pybullet-planning.readthedocs.io/en/latest/reference/generated/pybullet_planning.primitives.grasp_gen.get_side_cylinder_grasps.html#pybullet_planning.primitives.grasp_gen.get_side_cylinder_grasps
    center, (_, height) = pp.approximate_as_cylinder(bar_body)
    grasp_gen = pp.get_side_cylinder_grasps(bar_body, safety_margin_length=height/2-0.05)

    world_from_object = pp.get_pose(bar_body)
    # * sample grasp and IK, and plan for approach motion
    grasp_attempts = 50
    attach_conf = None
    path = None
    start_conf = pp.get_joint_positions(robot, movable_joints)
    with pp.WorldSaver():
        with pp.LockRenderer(1):
            for g_id in range(grasp_attempts):
                print('Grasp attempt #{}/{}'.format(g_id, grasp_attempts))
                gripper_from_object = next(grasp_gen)
                world_from_gripper_tcp = pp.multiply(world_from_object, pp.invert(gripper_from_object))
                world_from_tool0 = pp.multiply(world_from_gripper_tcp, gripper_tcp_from_tool0)

                # pp.draw_pose(world_from_gripper_tcp)
                # pp.draw_pose(world_from_tool0)

                world_from_arm_base = pp.get_link_pose(robot, pp.link_from_name(robot, "ur_arm_base_link"))
                arm_base_from_tool0 = pp.multiply(pp.invert(world_from_arm_base), world_from_tool0)
                # arm_base_from_tcp_pose = pp.multiply(pp.invert(world_from_arm_base), world_from_gripper_tcp)
                # pp.draw_pose(pp.multiply(world_from_arm_base, arm_base_from_tcp_pose))

                attach_conf = ik_solver.ik(pp.tform_from_pose(arm_base_from_tool0))
                if attach_conf is not None and not approach_collision_fn(attach_conf, diagnosis=debug):
                    # print("solved conf: ", conf)
                    # print("grasp: ", gripper_from_object)

                    # * plan pregrasp motion
                    # move world_from_tool0 in the minus z direction for 0.1m
                    tool0_from_pregrasp = pp.Pose(point=[0,0,-0.1])
                    arm_base_from_pregrasp = pp.multiply(arm_base_from_tool0, tool0_from_pregrasp)

                    approach_path = []
                    pregrasp_poses = list(pp.interpolate_poses(arm_base_from_tool0, arm_base_from_pregrasp, pos_step_size=POS_STEP_SIZE, ori_step_size=ORI_STEP_SIZE))
                    prev_conf = attach_conf
                    for fpose in pregrasp_poses:
                        # pp.draw_pose(fpose)
                        attach_conf = ik_solver.ik(pp.tform_from_pose(fpose), qinit=prev_conf)
                        if attach_conf is None or approach_collision_fn(attach_conf, diagnosis=debug):
                            notify('ik can\'t find an ik solution for approaching')
                            break
                        else:
                            approach_path.append(attach_conf)

                    if len(approach_path) != len(pregrasp_poses) or \
                        not check_path(movable_joints, approach_path, jump_threshold=JOINT_JUMP_THRESHOLD):
                        continue
                    else:
                        print('Pregrasp path found: {} pts'.format(len(approach_path)))
                        # * plan transit motion from current conf to pregrasp conf
                        end_conf = approach_path[-1]
                        # print('start conf: ', start_conf)
                        transit_path = None

                        if pp.check_initial_end(start_conf, end_conf, transit_collision_fn, diagnosis=debug):
                            transit_path = pp.solve_motion_plan(start_conf, end_conf, 
                                                        distance_fn, sample_fn, extend_fn,
                                                        transit_collision_fn,
                                                        algorithm='birrt', 
                                                        max_time=10, 
                                                        max_iterations=20, 
                                                        smooth=20, diagnosis=debug,
                                                        coarse_waypoints=False,
                                                        ) 
                        else:
                            notify('initial and end confs for transit motion are not valid')

                        if transit_path is None:
                            # notify('transit path not found')
                            # return approach_path[::-1]
                            # return None
                            # path = approach_path[::-1]
                            continue
                        else:
                            notify('transit path found: transit {} pts'.format(len(transit_path)))
                        path = transit_path + approach_path[::-1]
                        break
            else:
                notify("no ik solution after {} grasp attempts".format(grasp_attempts))

    return path

def plan_transit_motion(robot, end_conf, attachments, obstacles, debug=False, disabled_collisions=None):
    custom_limits = get_custom_limits(robot, {})
    resolutions = np.ones(6) * 0.05
    disabled_collisions = disabled_collisions or {}
    extra_disabled_collisions = [
        ((robot, pp.link_from_name(robot, 'ur_arm_wrist_3_link')), 
         (attachments[0].child, pp.BASE_LINK)), 
        ]

    movable_joints = pp.joints_from_names(robot, HUSKYU_JOINT_NAMES)
    sample_fn = pp.get_sample_fn(robot, movable_joints, custom_limits=custom_limits)
    distance_fn = pp.get_distance_fn(robot, movable_joints) #, weights=weights)
    extend_fn = pp.get_extend_fn(robot, movable_joints, resolutions=resolutions)

    transit_collision_fn = pp.get_collision_fn(robot, movable_joints, obstacles=obstacles,
                                                attachments=attachments, 
                                                self_collisions=1,
                                                disabled_collisions=disabled_collisions, extra_disabled_collisions=extra_disabled_collisions,
                                                custom_limits=custom_limits, 
                                                max_distance=0)

    transit_path = None
    with pp.WorldSaver():
        with pp.LockRenderer(True):
            # * plan transit motion from current conf to pregrasp conf
            start_conf = pp.get_joint_positions(robot, movable_joints)
            # print('start conf: ', start_conf)

            # new_collision_fn = lambda q, diagnosis=False: collision_fn(q, diagnosis=True)
            if pp.check_initial_end(start_conf, end_conf, transit_collision_fn, diagnosis=debug):
                transit_path = pp.solve_motion_plan(start_conf, end_conf, 
                                            distance_fn, sample_fn, extend_fn,
                                            transit_collision_fn,
                                            algorithm='birrt', 
                                            max_time=10, 
                                            max_iterations=20, 
                                            smooth=20, diagnosis=debug,
                                            coarse_waypoints=False,
                                            ) 
            else:
                notify('initial and end conf not valid')
            if transit_path is None:
                notify('transit path not found')
            else:
                notify('transit path found: transit {} pts'.format(len(transit_path)))

    return transit_path

def notify(msg):
    print(msg)
    # notification.notify(
    #     title='husky_assembly',
    #     message=msg,
    #     app_icon=None,  # e.g. 'C:\\icon_32x32.ico'
    #     timeout=2,  # seconds
    # )

def align_joint_conf_by_joint_names(source_joint_names, target_conf, target_joint_names):
    return [target_conf[target_joint_names.index(joint_name)] for joint_name in source_joint_names]

def save_joint_state_to_json():
    global arm_joint_state
    file_path = os.path.join(HERE, 'arm_joint_state.json')
    with open(file_path, 'w') as f:
        json.dump(arm_joint_state, f, indent=4)
    notify('Arm joint state saved to {}'.format(file_path))

def read_saved_joint_state_from_json():
    file_path = os.path.join(HERE, 'arm_joint_state.json')
    if not os.path.exists(file_path):
        notify('no saved arm joint state found at {}'.format(file_path))
        return None
    with open(file_path, 'r') as f:
        saved_arm_joint_state = json.load(f)
    notify('Saved arm joint state read from {}'.format(file_path))
    return saved_arm_joint_state

def find_time_interval(t, t_list):
    n = len(t_list)

    if t < t_list[0]:
        raise ValueError("t not in t_list")

    for i in range(1, n):
        if t < t_list[i]:
            return t_list[i-1], i-1
    
    return t_list[-1], n-1