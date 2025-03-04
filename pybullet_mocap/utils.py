import sys, os, argparse
import socket, json

import numpy as np
import pybullet_planning as pp

from pybullet_mocap import DATA_DIRECTORY
from pybullet_planning import multiply, Pose, Euler, Point
from tracikpy import TracIKSolver
# import ikfast_ur5e

from compas_robots import RobotModel
from compas_fab.robots import RobotSemantics
from compas_fab.robots import Robot as RobotClass

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

HUSKY_JOINT_NAMES = [
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
RETRACTION_LENGTH = 0.1

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

def get_grasp_pose(direction, angle, offset=1e-3):
    # tool0_from_object
    #direction = Pose(euler=Euler(roll=np.pi / 2, pitch=direction))
    return multiply(Pose(point=Point(z=offset)),
                    Pose(euler=Euler(yaw=angle)),
                    direction
                    # Pose(point=Point(z=translation)),
                    # Pose(euler=Euler(roll=(1-reverse) * np.pi)
                    )

def plan_transfer_motion(robot, ik_solver, bar_body, attachments, obstacles, 
                       debug=False, disabled_collisions=None):
    # plan a transit motion from init conf to pick_approach conf  
    custom_limits = get_custom_limits(robot, {})
    resolutions = np.ones(6) * 0.05
    disabled_collisions = disabled_collisions or {}
    extra_disabled_collisions = [
        ((robot, pp.link_from_name(robot, 'ur_arm_wrist_3_link')), 
         (attachments[0].child, pp.BASE_LINK)), 
         # pp.link_from_name(ee_body, 'robotiq_85_base_link'))),
        ]

    movable_joints = pp.joints_from_names(robot, HUSKY_JOINT_NAMES)
    tool_link = pp.link_from_name(robot, 'ur_arm_tool0')
    gripper_tcp_from_tool0 = pp.invert(TOOL0_FROM_GRIPPER_TCP)

    sample_fn = pp.get_sample_fn(robot, movable_joints, custom_limits=custom_limits)
    distance_fn = pp.get_distance_fn(robot, movable_joints) #, weights=weights)
    extend_fn = pp.get_extend_fn(robot, movable_joints, resolutions=resolutions)
    # extra_disabled_collisions += [
    #     ((bar_body, pp.BASE_LINK), 
    #      (attachments[0].child, pp.BASE_LINK)),
    # ]

    # Assuming the bar body is already at the target pose, managed by the monitor side
    # See: https://pybullet-planning.readthedocs.io/en/latest/reference/generated/pybullet_planning.primitives.grasp_gen.get_side_cylinder_grasps.html#pybullet_planning.primitives.grasp_gen.get_side_cylinder_grasps
    center, (_, height) = pp.approximate_as_cylinder(bar_body)
    grasp_gen = pp.get_side_cylinder_grasps(bar_body, safety_margin_length=height/2-0.05)

    # debug = True

    world_from_object = pp.get_pose(bar_body)
    # * sample grasp and IK, and plan for approach motion
    grasp_attempts = 50
    detach_conf = None
    path = None
    grasp = None
    start_conf = pp.get_joint_positions(robot, movable_joints)
    with pp.WorldSaver():
        with pp.LockRenderer(0):
            for g_id in range(grasp_attempts):
                print('Grasp attempt #{}/{}'.format(g_id, grasp_attempts))
                gripper_from_object = next(grasp_gen)
                tool0_from_object = pp.multiply(pp.invert(gripper_tcp_from_tool0), gripper_from_object)
                # world_from_gripper_tcp = pp.multiply(world_from_object, pp.invert(gripper_from_object))
                world_from_tool0 = pp.multiply(world_from_object, pp.invert(tool0_from_object))

                world_from_arm_base = pp.get_link_pose(robot, pp.link_from_name(robot, "ur_arm_base_link"))
                arm_base_from_tool0 = pp.multiply(pp.invert(world_from_arm_base), world_from_tool0)

                pp.draw_pose(world_from_tool0)
                pp.draw_pose(world_from_arm_base)
                # pp.wait_if_gui()

                detach_conf = ik_solver.ik(pp.tform_from_pose(arm_base_from_tool0))
                if detach_conf is not None:
                    pp.set_joint_positions(robot, movable_joints, detach_conf)
                    # element_attachment = pp.create_attachment(robot, tool_link, bar_body)
                    element_attachment = pp.Attachment(robot, tool_link, tool0_from_object, bar_body)

                    transfer_collision_fn = pp.get_collision_fn(robot, movable_joints, obstacles=obstacles,
                                                                attachments=attachments + [element_attachment] , 
                                                                self_collisions=1,
                                                                disabled_collisions=disabled_collisions, extra_disabled_collisions=extra_disabled_collisions,
                                                                custom_limits=custom_limits, 
                                                                max_distance=0)

                    transit_path = None
                    if pp.check_initial_end(start_conf, detach_conf, transfer_collision_fn, diagnosis=debug):
                        transit_path = pp.solve_motion_plan(start_conf, detach_conf, 
                                                        distance_fn, sample_fn, extend_fn,
                                                        transfer_collision_fn,
                                                        algorithm='birrt', 
                                                        max_time=10, 
                                                        max_iterations=20, 
                                                        smooth=20, diagnosis=debug,
                                                        coarse_waypoints=False,
                                                        ) 
                    else:
                        notify('initial and end confs for transit motion are not valid')

                    if transit_path is None:
                        continue
                    else:
                        notify('transit path found: transit {} pts'.format(len(transit_path)))

                    path = transit_path
                    grasp = tool0_from_object
                    break
            else:
                notify("no ik solution after {} grasp attempts".format(grasp_attempts))

    return path, grasp

def plan_transit_motion(robot, end_conf, attachments, obstacles, debug=False, disabled_collisions=None):
    custom_limits = get_custom_limits(robot, {})
    resolutions = np.ones(6) * 0.05
    disabled_collisions = disabled_collisions or {}
    extra_disabled_collisions = [
        ((robot, pp.link_from_name(robot, 'ur_arm_wrist_3_link')), 
         (attachments[0].child, pp.BASE_LINK)), 
        ]

    movable_joints = pp.joints_from_names(robot, HUSKY_JOINT_NAMES)
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

def plan_retract_to_home_motion(robot, ik_solver, bar_body, attachments, obstacles, 
                             debug=False, disabled_collisions=None):
    # plan a linear retract motion along negative z-axis, then a transit motion to home conf
    from pybullet_mocap.husky_robot import UR5e_HOME_STATE

    # * plan retract motion
    custom_limits = get_custom_limits(robot, {})
    resolutions = np.ones(6) * 0.05
    disabled_collisions = disabled_collisions or {}
    extra_disabled_collisions = [
        ((robot, pp.link_from_name(robot, 'ur_arm_wrist_3_link')), 
         (attachments[0].child, pp.BASE_LINK)), 
         # pp.link_from_name(ee_body, 'robotiq_85_base_link'))),
        ((bar_body, pp.BASE_LINK), 
         (attachments[0].child, pp.BASE_LINK)),
    ]

    movable_joints = pp.joints_from_names(robot, HUSKY_JOINT_NAMES)
    tool_link = pp.link_from_name(robot, 'ur_arm_tool0')

    sample_fn = pp.get_sample_fn(robot, movable_joints, custom_limits=custom_limits)
    distance_fn = pp.get_distance_fn(robot, movable_joints) #, weights=weights)
    extend_fn = pp.get_extend_fn(robot, movable_joints, resolutions=resolutions)

    retreat_collision_fn = pp.get_collision_fn(robot, movable_joints, obstacles=obstacles,
                                                attachments=attachments, 
                                                self_collisions=1,
                                                disabled_collisions=disabled_collisions, 
                                                extra_disabled_collisions=extra_disabled_collisions,
                                                custom_limits=custom_limits, 
                                                max_distance=0)

    # ! assuming current robot pose is at grasp
    arm_base_from_tool0 = pp.get_relative_pose(robot, tool_link, pp.link_from_name(robot, 'ur_arm_base_link'))
    current_conf = pp.get_joint_positions(robot, movable_joints)

    tool0_from_pregrasp = pp.Pose(point=[0,0,-RETRACTION_LENGTH])
    arm_base_from_pregrasp = pp.multiply(arm_base_from_tool0, tool0_from_pregrasp)

    retreat_path = [current_conf]
    pregrasp_poses = list(pp.interpolate_poses(arm_base_from_tool0, arm_base_from_pregrasp, 
                                               pos_step_size=POS_STEP_SIZE, ori_step_size=ORI_STEP_SIZE))

    debug = True
    for fpose in pregrasp_poses[1:]:
        retreat_conf = ik_solver.ik(pp.tform_from_pose(fpose), qinit=retreat_path[-1])
        if retreat_conf is None or retreat_collision_fn(retreat_conf, diagnosis=debug):
            notify('ik can\'t find an ik solution for retreat conf')
            break
        else:
            retreat_path.append(retreat_conf)

    if len(retreat_path) != len(pregrasp_poses) or \
        not check_path(movable_joints, retreat_path, jump_threshold=JOINT_JUMP_THRESHOLD):
        return None

    # * plan transit motion
    pp.set_joint_positions(robot, movable_joints, retreat_path[-1])
    transit_path = plan_transit_motion(robot, UR5e_HOME_STATE, attachments, obstacles, debug=debug, disabled_collisions=disabled_collisions)
    if transit_path is None:
        return None

    return retreat_path + transit_path    

############################

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