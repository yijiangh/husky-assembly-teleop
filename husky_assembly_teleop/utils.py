import sys, os, argparse
import socket, json

import numpy as np
import pybullet as p
import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY
from pybullet_planning import multiply, Pose, Euler, Point
from tracikpy import TracIKSolver
# import ikfast_ur5e

HERE = os.path.dirname(__file__)

yup_tform = np.eye(4)
yup_tform[:3,0] = [0, 1, 0]
yup_tform[:3,1] = [0, 0, 1]
yup_tform[:3,2] = [1, 0, 0]
zup_from_yup = pp.pose_from_tform(yup_tform)


# Mocap (y-up) -> z-up axis convention helpers.
# 'rhino' (preferred): keep mocap_x as new_x, so new = (mocap_x, -mocap_z, mocap_y).
#                      Same convention as the Rhino model.
# 'rotated' (legacy):  new = (mocap_z, mocap_x, mocap_y). Used to be hardcoded in monitor.
# Quat (xyzw) axis components transform the same way as a 3-vector.
MOCAP_AXIS_CONVENTIONS = ('rhino', 'rotated')


def mocap_pos_y_up_to_z_up(pos, convention='rhino'):
    if convention == 'rhino':
        return [pos[0], -pos[2], pos[1]]
    if convention == 'rotated':
        return [pos[2], pos[0], pos[1]]
    raise ValueError(f"unknown mocap axis convention {convention!r}")


def mocap_quat_y_up_to_z_up(quat, convention='rhino'):
    qx, qy, qz, qw = quat
    if convention == 'rhino':
        return [qx, -qz, qy, qw]
    if convention == 'rotated':
        return [qz, qx, qy, qw]
    raise ValueError(f"unknown mocap axis convention {convention!r}")

# <link name="bar_tcp"/>
# <joint name="tool0-bar_tcp_fixed_joint" type="fixed">
#   <origin rpy="0 0 3.141592653589793" xyz="0 0 0.152"/>
#   <parent link="robotiq_85_mount"/>
#   <child link="bar_tcp"/>
# </joint>
TOOL0_FROM_GRIPPER_TCP = pp.Pose(point=(0, 0, 0.152 + 0.012), euler=pp.Euler(yaw=np.pi))

UR5E_JOINT_NAMES = [
                      "ur_arm_shoulder_pan_joint", 
                      "ur_arm_shoulder_lift_joint",
                      "ur_arm_elbow_joint", 
                      "ur_arm_wrist_1_joint", 
                      "ur_arm_wrist_2_joint", 
                      "ur_arm_wrist_3_joint" ]

HUSKY_DUAL_UR5e_JOINT_NAMES = [["left_ur_arm_shoulder_pan_joint",
                      "left_ur_arm_shoulder_lift_joint",
                      "left_ur_arm_elbow_joint",
                      "left_ur_arm_wrist_1_joint",
                      "left_ur_arm_wrist_2_joint",
                      "left_ur_arm_wrist_3_joint" ],
                                ["right_ur_arm_shoulder_pan_joint",
                      "right_ur_arm_shoulder_lift_joint",
                      "right_ur_arm_elbow_joint",
                      "right_ur_arm_wrist_1_joint",
                      "right_ur_arm_wrist_2_joint",
                      "right_ur_arm_wrist_3_joint" ]]

# Fixed dual-arm "home" configuration used by Plan M4 (return-to-home after
# bar placement). Order matches HUSKY_DUAL_UR5e_JOINT_NAMES[0] + [1].
HUSKY_DUAL_ARM_HOME_CONF_12 = np.array([
    -1.381079037103113, -0.08674286382411818, -2.8050931738052864,
    -1.7444565873683324, 0.23963370629882144, 1.4217452086745808,
     1.3946926052686688, -3.0267499888085663,  2.8043950421044888,
    -1.727003294848389, -0.40561451816348215, -1.2402309664671707,
])

WHEEL_JOINT_NAMES = [
                      "front_right_wheel", 
                      "rear_right_wheel",
                      "front_left_wheel", 
                      "rear_left_wheel" ]
JOINT_JUMP_THRESHOLD = np.pi/3
POS_STEP_SIZE = 0.01
ORI_STEP_SIZE = np.pi/18
RETRACTION_LENGTH = 0.1 

from compas.geometry import Frame, Transformation
def pose_from_frame(frame, scale=1.0):
    return ([v*scale for v in frame.point], frame.quaternion.xyzw)

def frame_from_pose(pose, scale=1.0):
    point, (x, y, z, w) = pose
    return Frame.from_quaternion([w, x, y, z], point=[v*scale for v in point])


def vec12_from_conf(conf):
    """Extract a 12-vec (left||right arm joint values) from a compas
    Configuration. Joint order matches HUSKY_DUAL_UR5e_JOINT_NAMES[0] + [1].
    """
    names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    return np.asarray([float(conf[n]) for n in names], dtype=float)


def conf_from_12vec(vec12):
    """Build a compas Configuration from a 12-vec (left||right arm joints)."""
    from compas_robots import Configuration
    names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    vals = [float(v) for v in vec12]
    if len(vals) != 12:
        raise ValueError(f"vec12 must be length 12, got {len(vals)}")
    return Configuration.from_revolute_values(vals, joint_names=names)


def joint_trajectory_from_path(path_12):
    """Wrap a list / array of 12-vecs into a compas_fab JointTrajectory.

    The trajectory uses HUSKY_DUAL_UR5e_JOINT_NAMES (12 names, left then
    right). One JointTrajectoryPoint per waypoint.
    """
    from compas_fab.robots import JointTrajectory, JointTrajectoryPoint
    from compas_robots.model import Joint
    names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    types = [Joint.REVOLUTE] * len(names)
    points = []
    for i, q in enumerate(path_12):
        q = list(map(float, q))
        if len(q) != 12:
            raise ValueError(f"path_12[{i}] must be length 12, got {len(q)}")
        points.append(JointTrajectoryPoint(joint_values=q, joint_types=types, joint_names=names))
    return JointTrajectory(trajectory_points=points, joint_names=names)


def path_12_from_joint_trajectory(jt):
    """Inverse of joint_trajectory_from_path: extract a list of 12-vec
    numpy arrays from a JointTrajectory whose joint_names are
    HUSKY_DUAL_UR5e_JOINT_NAMES[0]+[1] (order preserved).
    """
    names = list(HUSKY_DUAL_UR5e_JOINT_NAMES[0]) + list(HUSKY_DUAL_UR5e_JOINT_NAMES[1])
    return [
        np.asarray([float(p.joint_values[p.joint_names.index(n)]) for n in names],
                   dtype=float)
        for p in jt.points
    ]

def pose_from_transformation(tf, scale=1.0):
    frame = Frame.from_transformation(tf)
    return pose_from_frame(frame, scale)

def transformation_from_pose(pose, scale=1.0):
    frame = frame_from_pose(pose, scale)
    return Transformation.from_frame(frame)

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

def get_arm_ik_for_grasp_bar(robot, ik_solver, world_from_tool0, attachments, obstacles, hint_conf=None):
    IK_ATTEMPTS = 10
    
    # use correct joint names for dual arm husky
    joint_names = UR5E_JOINT_NAMES
    arm_prefix = ""
    if ik_solver.base_link.startswith("left_"):
        arm_prefix = "left_"
        joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
    if ik_solver.base_link.startswith("right_"):
        arm_prefix = "right_"
        joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
    
    custom_limits = get_custom_limits(robot, {})
    # disabled_collisions = disabled_collisions or {}
    extra_disabled_collisions = [
        ((robot, pp.link_from_name(robot, arm_prefix + 'ur_arm_wrist_3_link')), 
         (attachments[0].child, pp.BASE_LINK)), 
         # pp.link_from_name(ee_body, 'robotiq_85_base_link'))),
        ]

    movable_joints = pp.joints_from_names(robot, joint_names)
    sample_fn = pp.get_sample_fn(robot, movable_joints, custom_limits=custom_limits)
    collision_fn = pp.get_collision_fn(robot, movable_joints, obstacles=obstacles,
                                                attachments=attachments, 
                                                self_collisions=1,
                                                disabled_collisions={}, 
                                                extra_disabled_collisions=extra_disabled_collisions,
                                                custom_limits=custom_limits, 
                                                max_distance=0)

    # * the robot base pose should be udpated by the main loop in monitor according to mocap observation before the planning starts
    world_from_arm_base = pp.get_link_pose(robot, pp.link_from_name(robot, ik_solver.base_link))
    if hint_conf is None:
        start_conf = pp.get_joint_positions(robot, movable_joints)
    else:
        start_conf = hint_conf
    conf = None
    
    diagnose = 0
    with pp.WorldSaver():
        with pp.LockRenderer(not diagnose):
            for i in range(IK_ATTEMPTS):
                if i == 0:
                    qinit = start_conf
                else:
                    qinit = sample_fn()

                arm_base_from_tool0 = pp.multiply(pp.invert(world_from_arm_base), world_from_tool0)
                conf = ik_solver.ik(pp.tform_from_pose(arm_base_from_tool0), qinit=qinit)

                if conf is not None and not collision_fn(conf, diagnosis=diagnose):
                    break
            else:
                notify("no ik solution after {} attempts".format(IK_ATTEMPTS))
                return None
    return conf

def plan_transit_motion(robot, end_conf, attachments, obstacles, debug=False,
                        disabled_collisions=None, dual_arm_index=None,
                        joint_resolution=0.05, max_time=10,
                        max_iterations=20, ee_types=None,
                        cfab_collision_fn=None):
    # cfab_collision_fn (optional): when provided, REPLACES pp's collision
    # predicate. Signature: (conf, **kwargs) -> bool (True == colliding).
    # Adapted to support dual-arm (composite) planning.
    #
    # ee_types: optional list of strings, one per attachment (parallels
    # ``attachments``). When an entry is ``"assembly_tool_v3_left"`` or
    # ``"assembly_tool_v3_right"``, the corresponding arm's wrist_2_link is
    # added to the disabled-collisions list for that EE — the tool body
    # extends past wrist_3 into the wrist_2 swept volume on the husky URDF,
    # so without this disable the planner rejects otherwise-valid poses.
    joint_names = UR5E_JOINT_NAMES
    arm_prefix = ""

    def _is_assembly_tool_v3(ee_type):
        return isinstance(ee_type, str) and ee_type.startswith("assembly_tool_v3")

    # Dual-arm mode: plan in the composite joint space of both arms
    if dual_arm_index == "both":
        joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0] + HUSKY_DUAL_UR5e_JOINT_NAMES[1]
        # Expect attachments to be a list of two (one for each arm)
        if not (isinstance(attachments, list) and len(attachments) == 2):
            raise ValueError("In dual-arm mode, attachments must be a list of two (left, right)")
        extra_disabled_collisions = [
            # Left arm
            ((robot, pp.link_from_name(robot, 'left_ur_arm_wrist_3_link')),
             (attachments[0].child, pp.BASE_LINK)),
            # Right arm
            ((robot, pp.link_from_name(robot, 'right_ur_arm_wrist_3_link')),
             (attachments[1].child, pp.BASE_LINK)),
        ]
        # Extra wrist_2 disable when an assembly_tool_v3_* is mounted.
        if ee_types and len(ee_types) >= 2:
            for side_prefix, ee_type, attach in (
                ("left_", ee_types[0], attachments[0]),
                ("right_", ee_types[1], attachments[1]),
            ):
                if _is_assembly_tool_v3(ee_type):
                    for wrist_link in ('ur_arm_wrist_2_link', 'ur_arm_wrist_1_link'):
                        extra_disabled_collisions.append(
                            ((robot, pp.link_from_name(robot, side_prefix + wrist_link)),
                             (attach.child, pp.BASE_LINK))
                        )
        # Combine both attachments for collision checking
        all_attachments = attachments
    else:
        if dual_arm_index==0:
            arm_prefix = "left_"
            joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[0]
        if dual_arm_index==1:
            arm_prefix = "right_"
            joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[1]
        extra_disabled_collisions = [
            ((robot, pp.link_from_name(robot, arm_prefix + 'ur_arm_wrist_3_link')),
             (attachments[0].child, pp.BASE_LINK)),
        ]
        # Extra wrist_2 disable when an assembly_tool_v3_* is mounted on this arm.
        if ee_types and len(ee_types) >= 1 and _is_assembly_tool_v3(ee_types[0]):
            for wrist_link in ('ur_arm_wrist_2_link',):
                extra_disabled_collisions.append(
                    ((robot, pp.link_from_name(robot, arm_prefix + wrist_link)),
                     (attachments[0].child, pp.BASE_LINK))
                )
        all_attachments = attachments if isinstance(attachments, list) else [attachments]
    
    custom_limits = get_custom_limits(robot, {})
    resolutions = np.ones(len(joint_names)) * float(joint_resolution)
    disabled_collisions = disabled_collisions or {}

    movable_joints = pp.joints_from_names(robot, joint_names)
    sample_fn = pp.get_sample_fn(robot, movable_joints, custom_limits=custom_limits)
    distance_fn = pp.get_distance_fn(robot, movable_joints) #, weights=weights)
    extend_fn = pp.get_extend_fn(robot, movable_joints, resolutions=resolutions)

    moving_links = pp.get_moving_links(robot, movable_joints)
    all_links = list(range(pp.get_num_links(robot)))
    non_moving_links = [link for link in all_links if link not in moving_links]

    # TODO make this only check collision above 0.001 m collision depth for scaffolding tool collision with wrist_1
    transit_collision_fn = pp.get_collision_fn(robot, movable_joints, obstacles=obstacles,
                                                attachments=all_attachments,
                                                self_collisions=1,
                                                disabled_collisions=disabled_collisions, extra_disabled_collisions=extra_disabled_collisions,
                                                custom_limits=custom_limits,
                                                max_distance=0.000)

    if cfab_collision_fn is not None:
        # Override pp's collision predicate with cfab's PyBulletCheckCollision.
        # pp.check_initial_end passes `diagnosis` as a positional arg; the
        # extend/sample paths pass it as a kwarg. Accept both.
        def _adapted_collision_fn(q, *_args, **_kw):
            return bool(cfab_collision_fn(np.asarray(q, dtype=float)))
        transit_collision_fn = _adapted_collision_fn

    transit_path = None
    with pp.WorldSaver():
        with pp.LockRenderer(not debug):
            # * plan transit motion from current conf to pregrasp conf
            start_conf = pp.get_joint_positions(robot, movable_joints)
            # print('start conf: ', start_conf)

            if pp.check_initial_end(start_conf, end_conf, transit_collision_fn, diagnosis=True):
                transit_path = pp.solve_motion_plan(start_conf, end_conf, 
                                            distance_fn, sample_fn, extend_fn,
                                            transit_collision_fn,
                                            algorithm='birrt', 
                                            max_time=max_time,
                                            max_iterations=max_iterations,
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

def plan_transfer_motion(robot, ik_solver, transfer_element, attachments, obstacles, 
                       grasp=None,
                       debug=False, disabled_collisions=None):
    # plan a transit motion from init conf to pick_approach conf  
    # if grasp is not None, use a pre-computed grasp pose instead of sampling a new one

    custom_limits = get_custom_limits(robot, {})
    resolutions = np.ones(6) * 0.05
    disabled_collisions = disabled_collisions or {}
    extra_disabled_collisions = [
        ((robot, pp.link_from_name(robot, 'ur_arm_wrist_3_link')), 
         (attachments[0].child, pp.BASE_LINK)), 
         # pp.link_from_name(ee_body, 'robotiq_85_base_link'))),
        ]

    movable_joints = pp.joints_from_names(robot, UR5E_JOINT_NAMES)
    tool_link = pp.link_from_name(robot, 'ur_arm_tool0')
    bar_body = transfer_element.body

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
    # ! we enforce the grasp is at the center of the bar now
    grasp_gen = pp.get_side_cylinder_grasps(bar_body, safety_margin_length=height/2-0.0)

    debug = 0

    world_from_object = transfer_element.goal_pose
    # * sample grasp and IK, and plan for approach motion
    grasp_attempts = 50 if grasp is None else 1
    linear_path_num = 5

    detach_conf = None
    free_path = None
    linear_path = None
    start_conf = pp.get_joint_positions(robot, movable_joints)
    with pp.WorldSaver():
        with pp.LockRenderer(not debug):
            for g_id in range(grasp_attempts):
                if grasp is None:
                    print('Grasp attempt #{}/{}'.format(g_id, grasp_attempts))
                    gripper_from_object = next(grasp_gen)
                    tool0_from_object = pp.multiply(TOOL0_FROM_GRIPPER_TCP, gripper_from_object)
                else:
                    print('Reusing grasp.')
                    tool0_from_object = grasp

                # world_from_gripper_tcp = pp.multiply(world_from_object, pp.invert(gripper_from_object))
                world_from_tool0 = pp.multiply(world_from_object, pp.invert(tool0_from_object))

                # the arm base link pose has been updated already by the main loop in monitor before the planning starts,
                # but will not change during planning, so we should wait until the robot settles down and then start planning
                world_from_arm_base = pp.get_link_pose(robot, pp.link_from_name(robot, ik_solver.base_link))
                arm_base_from_tool0 = pp.multiply(pp.invert(world_from_arm_base), world_from_tool0)

                # pp.draw_pose(world_from_tool0)
                # pp.draw_pose(world_from_arm_base)
                # pp.wait_if_gui()

                detach_conf = ik_solver.ik(pp.tform_from_pose(arm_base_from_tool0), qinit=start_conf)

                # TODO should compute a last-chunk linear trajectory based on bar workspace trajectory
                # ideally use the neighboring bar's contact normal to derive a non-blocking direction cone
                # For now, I will just approx the linear movement with the last 20 traj points of the transfer motion

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

                    transfer_path = None
                    if pp.check_initial_end(start_conf, detach_conf, transfer_collision_fn, diagnosis=debug):
                        transfer_path = pp.solve_motion_plan(start_conf, detach_conf, 
                                                        distance_fn, sample_fn, extend_fn,
                                                        transfer_collision_fn,
                                                        algorithm='birrt', 
                                                        max_time=10, 
                                                        max_iterations=20, 
                                                        smooth=20, diagnosis=False,
                                                        coarse_waypoints=False,
                                                        ) 
                    else:
                        notify('initial and end confs for transfer motion are not valid')

                    if transfer_path is None:
                        continue

                    pp.set_joint_positions(robot, movable_joints, transfer_path[-1])
                    # pp.wait_if_gui()
                    if plan_retract_to_home_motion(robot, ik_solver, bar_body, attachments, obstacles, plan_transit_home=False) is None:
                        # retry if no retraction is found
                        notify('transfer path rejected due to invalid retraction path')
                        continue
                    notify('transfer path found: {} pts'.format(len(transfer_path)))

                    if len(transfer_path) <= linear_path_num:
                        free_path = []
                        linear_path = transfer_path
                    else:
                        free_path = transfer_path[:-linear_path_num]
                        linear_path = transfer_path[-linear_path_num:]

                    grasp = tool0_from_object
                    break
            else:
                notify("no ik solution after {} grasp attempts".format(grasp_attempts))

    return free_path, linear_path, grasp

def plan_retract_to_home_motion(robot, ik_solver, bar_body, attachments, obstacles, 
                             debug=False, disabled_collisions=None, plan_transit_home=True):
    # plan a linear retract motion along negative z-axis, then a transit motion to home conf
    from husky_assembly_teleop.husky_robot import UR5e_HOME_STATE

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

    movable_joints = pp.joints_from_names(robot, UR5E_JOINT_NAMES)
    tool_link = pp.link_from_name(robot, 'ur_arm_tool0')

    retreat_collision_fn = pp.get_collision_fn(robot, movable_joints, obstacles=obstacles,
                                                attachments=attachments, 
                                                self_collisions=1,
                                                disabled_collisions=disabled_collisions, 
                                                extra_disabled_collisions=extra_disabled_collisions,
                                                custom_limits=custom_limits, 
                                                max_distance=0)

    # ! assuming current robot pose is at grasp
    arm_base_from_tool0 = pp.get_relative_pose(robot, tool_link, pp.link_from_name(robot, ik_solver.base_link))
    current_conf = pp.get_joint_positions(robot, movable_joints)

    tool0_from_pregrasp = pp.Pose(point=[0,0,-RETRACTION_LENGTH])
    arm_base_from_pregrasp = pp.multiply(arm_base_from_tool0, tool0_from_pregrasp)

    retreat_path = [current_conf]
    pregrasp_poses = list(pp.interpolate_poses(arm_base_from_tool0, arm_base_from_pregrasp, 
                                               pos_step_size=POS_STEP_SIZE, ori_step_size=ORI_STEP_SIZE))

    debug = 0
    for i, fpose in enumerate(pregrasp_poses[1:]):
        retreat_conf = ik_solver.ik(pp.tform_from_pose(fpose), qinit=retreat_path[-1])
        if retreat_conf is None or retreat_collision_fn(retreat_conf, diagnosis=debug):
            notify('ik can\'t find an ik solution for retreat conf at pose #{}/{}'.format(i+1, len(pregrasp_poses)))
            break
        else:
            retreat_path.append(retreat_conf)

    if len(retreat_path) != len(pregrasp_poses) or \
        not check_path(movable_joints, retreat_path, jump_threshold=JOINT_JUMP_THRESHOLD):
        return None

    # * plan transit motion
    transit_path = []
    if plan_transit_home:
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

def draw_joint_axes(robot, dual_arm=False, selected_arm_index=0, axis_length=0.1, line_width=3.0):
    """
    Draw the rotational axis of each revolute joint for the selected arm.

    Parameters
    ----------
    robot : int
        PyBullet body ID of the robot
    dual_arm : bool
        Whether the robot has dual arms
    selected_arm_index : int
        Which arm to visualize (0 for left/single, 1 for right)
    axis_length : float
        Length of the axis line to draw (in meters)
    line_width : float
        Width of the debug line

    Returns
    -------
    list of int
        List of debug line IDs that were created
    """
    # Get the appropriate joint names based on robot configuration
    if dual_arm:
        joint_names = HUSKY_DUAL_UR5e_JOINT_NAMES[selected_arm_index]
    else:
        joint_names = UR5E_JOINT_NAMES

    # Get joint indices
    joint_indices = pp.joints_from_names(robot, joint_names)

    # Store debug line IDs for potential cleanup
    debug_line_ids = []

    # Color scheme: RGB colors for different joints
    colors = [
        [1.0, 0.0, 0.0],  # Red - shoulder pan
        [0.0, 1.0, 0.0],  # Green - shoulder lift
        [0.0, 0.0, 1.0],  # Blue - elbow
        [1.0, 1.0, 0.0],  # Yellow - wrist 1
        [1.0, 0.0, 1.0],  # Magenta - wrist 2
        [0.0, 1.0, 1.0],  # Cyan - wrist 3
    ]

    for i, (joint_name, joint_idx) in enumerate(zip(joint_names, joint_indices)):
        # Get joint info from PyBullet
        joint_info = p.getJointInfo(robot, joint_idx)
        joint_type = joint_info[2]  # Joint type (0=revolute, 1=prismatic, 4=fixed)
        joint_axis = joint_info[13]  # Joint axis in local frame
        parent_frame_pos = joint_info[14]  # Position of joint frame in parent frame
        parent_frame_orn = joint_info[15]  # Orientation of joint frame in parent frame
        parent_index = joint_info[16]  # Parent link index

        # Only process revolute joints
        if joint_type != p.JOINT_REVOLUTE:
            notify(f"Warning: Joint {joint_name} is not a revolute joint (type={joint_type})")
            continue

        # Get the parent link's world pose
        if parent_index == -1:
            # Joint is attached to base link
            parent_world_pos, parent_world_orn = pp.get_pose(robot)
        else:
            parent_world_pos, parent_world_orn = pp.get_link_pose(robot, parent_index)

        # Convert parent link's world orientation to rotation matrix
        parent_rot_matrix = np.array(p.getMatrixFromQuaternion(parent_world_orn)).reshape(3, 3)

        # Joint frame position in world coordinates
        joint_frame_pos_world = np.array(parent_world_pos) + parent_rot_matrix @ np.array(parent_frame_pos)

        # Joint frame orientation in world coordinates
        joint_frame_orn_world = p.multiplyTransforms(
            parent_world_pos, parent_world_orn,
            parent_frame_pos, parent_frame_orn
        )[1]

        # Convert joint frame orientation to rotation matrix
        joint_rot_matrix = np.array(p.getMatrixFromQuaternion(joint_frame_orn_world)).reshape(3, 3)

        # Transform joint axis to world coordinates
        joint_axis_world = joint_rot_matrix @ np.array(joint_axis)
        joint_axis_world = joint_axis_world / np.linalg.norm(joint_axis_world)  # Normalize

        # Calculate start and end points of the axis line
        axis_start = joint_frame_pos_world - joint_axis_world * (axis_length / 2)
        axis_end = joint_frame_pos_world + joint_axis_world * (axis_length / 2)

        # Draw the axis line
        line_id = p.addUserDebugLine(
            lineFromXYZ=axis_start.tolist(),
            lineToXYZ=axis_end.tolist(),
            lineColorRGB=colors[i % len(colors)],
            lineWidth=line_width,
            physicsClientId=pp.CLIENT
        )
        debug_line_ids.append(line_id)

        # Add text label at the joint
        text_id = p.addUserDebugText(
            text=joint_name.split('_')[-1],  # Just show the joint type (e.g., "shoulder_pan_joint" -> "joint")
            textPosition=(joint_frame_pos_world + joint_axis_world * (axis_length / 2 + 0.02)).tolist(),
            textColorRGB=colors[i % len(colors)],
            textSize=1.2,
            physicsClientId=pp.CLIENT
        )
        debug_line_ids.append(text_id)

        notify(f"Drew axis for {joint_name} at position {joint_frame_pos_world} with axis {joint_axis_world}")

    return debug_line_ids
