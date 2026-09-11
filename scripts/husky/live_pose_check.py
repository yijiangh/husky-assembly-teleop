#!/usr/bin/env python3
"""Explain an "already in collision" verdict on the LIVE arm pose, offline.

* The open-loop engine refuses to plan an approach when the arms' current
* configuration collides in its PyBullet world -- and sometimes the real arms
* are visibly clear. This script reads the same joints (RTDE, or a saved JSON,
* or a trajectory sample as a stand-in), rebuilds the same world, and then
* asks the questions the engine's one-line verdict cannot answer:
*
*   1. does the verdict depend on how OPEN the model's fingers are?
*      (the engine checks with the jaws fully open; the real jaws may not be)
*   2. does it depend on the part being a footprint BOX rather than its mesh?
*      (a leg's box is 30 mm wide where the leg body is 22 mm)
*   3. WHERE is the overlap -- how far the finger tip sits below the box top,
*      and how far inside it sideways -- so a layout or table-height error
*      shows up as a number, not a feeling.
*
* It also writes the joints, and PyBullet's tool0 poses for both arms, to a
* JSON that `live_pose_in_planner.py` (planner venv) loads to run the SAME
* pose through the planner's own model: a mismatch there is a robot-model
* disagreement (base offset, arm mount calibration), not a collision question.

Run (ros2 venv + install sourced), on the robot:

    python3 src/husky-assembly-teleop/scripts/husky/live_pose_check.py \
        --layout-json <layout.json> --out /tmp/live_q.json

or without the robot, taking sample 0 of a trajectory as the "live" pose:

    python3 ... --layout-json <layout.json> --traj <open-loop.json> --swap-arms --sample 0
"""

import argparse
import json
import os
import sys

import numpy as np
import pybullet as p
import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY
from husky_assembly_teleop.common import HUSKY_DUAL_UR5e_JOINT_NAMES, load_robot
from husky_assembly_teleop.open_loop_approach import (OBSTACLE_LABELS,
                                                      build_collision_fn,
                                                      build_obstacles,
                                                      describe_collision)
from husky_assembly_teleop.open_loop_engine import (GRIPPER_CLOSE, GRIPPER_OPEN,
                                                    GRIPPER_VIZ_FACTORS,
                                                    load_viz_grippers)
from husky_assembly_teleop.open_loop_parts import load_part_bodies
from husky_assembly_teleop.open_loop_traj import ARM_SIDES
from husky_assembly_teleop.pickup_calib import load_layout

JOINTS_12 = HUSKY_DUAL_UR5e_JOINT_NAMES[0] + HUSKY_DUAL_UR5e_JOINT_NAMES[1]
# The monitor's "pre-open" (husky_world.open_gripper_full): what the real jaws
# are most likely sitting at if nobody opened them fully.
MONITOR_PRE_OPEN = 0.426


def read_live(left_ip: str, right_ip: str, freq: float) -> tuple:
    """Read both arms' joints and TCP poses once over RTDE, left arm first.

    Args:
        left_ip (str): Left arm controller IP.
        right_ip (str): Right arm controller IP.
        freq (float): RTDE receive frequency [Hz].

    Returns:
        tuple: (q12, tcp) -- 12 joint values [rad] left then right, and per
        arm the controller's own TCP pose [x y z rx ry rz] in the UR base
        frame, with whatever TCP the pendant currently has set.
    """
    from rtde_receive import RTDEReceiveInterface
    q, tcp = [], []
    for ip in (left_ip, right_ip):
        rtde = RTDEReceiveInterface(ip, freq)
        q.extend(rtde.getActualQ())
        tcp.append(list(rtde.getActualTCPPose()))
        rtde.disconnect()
    return np.asarray(q, dtype=float), tcp


def ur_fk_check(robot, side: str, tool0_link: int, tcp_pose, tcp_offset_z: float):
    """Compare the UR controller's own TCP position with PyBullet's FK.

    ! This is the referee the other checks lack: the controller computes its
    ! TCP from its OWN calibrated kinematics. If PyBullet's tool0 (plus the
    ! pendant's TCP offset) lands somewhere else at the SAME joints, the URDF
    ! the engine plans with is not the robot -- and no collision shape can be
    ! trusted before that is fixed. RTDE reports in the UR "base" frame, which
    ! is the URDF's base_link turned half a turn about z.

    Args:
        robot (int): PyBullet robot body.
        side (str): 'left' or 'right'.
        tool0_link (int): That arm's tool0 link index.
        tcp_pose: RTDE ``getActualTCPPose()`` for that arm.
        tcp_offset_z (float): The pendant's TCP offset along tool z [m].
    """
    try:
        base = pp.link_from_name(robot, f'{side}_ur_arm_base_link')
    except Exception:
        print(f'  {side}: no {side}_ur_arm_base_link in the URDF, FK check skipped')
        return
    world_from_base = pp.get_link_pose(robot, base)
    world_from_tool0 = pp.get_link_pose(robot, tool0_link)
    world_from_tcp = pp.multiply(world_from_tool0, pp.Pose(point=(0, 0, tcp_offset_z)))
    base_from_tcp = pp.multiply(pp.invert(world_from_base), world_from_tcp)
    # UR 'base' = base_link rotated 180 deg about z (ROS-Industrial convention).
    pb_xyz = np.asarray(base_from_tcp[0]) * np.array([-1.0, -1.0, 1.0])
    ur_xyz = np.asarray(tcp_pose[:3], dtype=float)
    print(f'  {side}: UR says TCP at {np.round(ur_xyz, 4).tolist()}, PyBullet FK says '
          f'{np.round(pb_xyz, 4).tolist()} (tool0 + {1000 * tcp_offset_z:.1f} mm) '
          f'-> |diff| {1000 * np.linalg.norm(ur_xyz - pb_xyz):.1f} mm')


def set_grippers(grippers, angle: float):
    """Put both viz grippers at one knuckle angle.

    Args:
        grippers (list): From `load_viz_grippers`.
        angle (float): Knuckle angle [rad], 0 = fully open.
    """
    for _body, attachment, joints in grippers:
        attachment.assign()
        pp.set_joint_positions(_body, joints,
                               [factor * angle for factor in GRIPPER_VIZ_FACTORS])


def overlap_report(grippers, part_bodies: dict, layout: dict):
    """Where each finger tip stands relative to every part box, in mm.

    Args:
        grippers (list): From `load_viz_grippers`.
        part_bodies (dict): Part name -> box body id (the engine's obstacles).
        layout (dict): The layout, for the table height.
    """
    top_z = float(layout['table']['top_z'])
    for side, (body, _att, _joints) in zip(ARM_SIDES, grippers):
        for tip_name in ('robotiq_85_left_finger_tip_link',
                         'robotiq_85_right_finger_tip_link'):
            tip = pp.link_from_name(body, tip_name)
            lo, hi = (np.asarray(v) for v in p.getAABB(body, tip))
            print(f'  {side} arm, {tip_name.split("_85_")[1]}: lowest point '
                  f'{1000 * (lo[2] - top_z):+.1f} mm above the table top')
            for name, part in part_bodies.items():
                plo, phi = (np.asarray(v) for v in p.getAABB(part))
                # Overlap per axis of the two boxes; negative = clear by that much.
                over = np.minimum(hi, phi) - np.maximum(lo, plo)
                if (over > 0).all():
                    print(f'      overlaps {name}\'s box by x {1000 * over[0]:.1f} / '
                          f'y {1000 * over[1]:.1f} / z {1000 * over[2]:.1f} mm '
                          f'(tip bottom is {1000 * (lo[2] - phi[2]):+.1f} mm above '
                          f'the box top)')


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--layout-json', required=True)
    cli.add_argument('--left-ip', default='192.168.131.40')
    cli.add_argument('--right-ip', default='192.168.131.41')
    cli.add_argument('--freq', type=float, default=125.0)
    cli.add_argument('--q-json', help='use the joints saved by an earlier run instead of RTDE')
    cli.add_argument('--traj', help='use a trajectory sample as the "live" pose instead of RTDE')
    cli.add_argument('--swap-arms', action='store_true')
    cli.add_argument('--sample', type=int, default=0)
    cli.add_argument('--out', default='/tmp/live_q.json',
                     help='where to save the joints + tool0 poses for live_pose_in_planner.py')
    cli.add_argument('--tcp-offset-z', type=float, default=0.1632,
                     help="the pendant's TCP z offset [m], for the UR-vs-PyBullet FK check "
                          "(default: the gripper TCP the engine sets, 0.152 + coupling)")
    args = cli.parse_args()

    tcp_poses = None
    if args.traj:
        from husky_assembly_teleop.open_loop_traj import load_open_loop_traj
        traj = load_open_loop_traj(args.traj, swap_arms=args.swap_arms)
        q12 = np.asarray(traj.q12[args.sample], dtype=float)
        source = f'{os.path.basename(args.traj)} sample {args.sample}'
    elif args.q_json:
        with open(args.q_json) as f:
            saved = json.load(f)
        q12 = np.asarray(saved['q12'], dtype=float)
        tcp_poses = saved.get('ur_tcp_pose')          # kept from the RTDE read, if any
        source = args.q_json
    else:
        q12, tcp_poses = read_live(args.left_ip, args.right_ip, args.freq)
        source = f'RTDE {args.left_ip} / {args.right_ip}'
    print(f'joints from {source}:')
    for side, block in zip(ARM_SIDES, (q12[:6], q12[6:])):
        print(f'  {side}: {np.round(block, 4).tolist()}')

    layout = load_layout(args.layout_json)
    pp.connect(use_gui=False)
    p.setAdditionalSearchPath(os.path.join(DATA_DIRECTORY, 'husky_urdf'))
    with pp.HideOutput():
        robot = load_robot(dual_arm=True)
    joints = pp.joints_from_names(robot, JOINTS_12)
    tool0 = [pp.link_from_name(robot, n) for n in ('left_ur_arm_tool0', 'right_ur_arm_tool0')]
    grippers = load_viz_grippers(robot, tool0)
    attachments = [attachment for _body, attachment, _joints in grippers]
    obstacles = build_obstacles(layout=layout)
    boxes = {label.split(' ', 1)[1]: body for body, label in OBSTACLE_LABELS.items()
             if label.startswith('part ')}
    pp.set_joint_positions(robot, joints, [float(v) for v in q12])
    tool0_poses = [pp.get_link_pose(robot, link) for link in tool0]
    for side, pose in zip(ARM_SIDES, tool0_poses):
        print(f'  PyBullet tool0 {side}: xyz {np.round(pose[0], 4).tolist()} '
              f'quat(xyzw) {np.round(pose[1], 4).tolist()}')
    if tcp_poses is not None:
        print('\n=== the UR controller\'s own FK vs PyBullet\'s (same joints) ===')
        for side, link, tcp_pose in zip(ARM_SIDES, tool0, tcp_poses):
            ur_fk_check(robot, side, link, tcp_pose, args.tcp_offset_z)

    # 1. The engine's own verdict, then the same check at other finger angles.
    fn = build_collision_fn(robot, attachments, obstacles)
    print(f'\n=== the engine\'s check, part boxes as obstacles ===')
    for label, angle in (('jaws fully open (what the engine assumes)', GRIPPER_OPEN),
                         ('jaws at the monitor pre-open 0.426', MONITOR_PRE_OPEN),
                         ('jaws fully closed', GRIPPER_CLOSE)):
        set_grippers(grippers, angle)
        verdict = fn(q12)
        print(f'--- {label}: {"IN COLLISION" if verdict else "clear"}')
        if verdict:
            describe_collision(robot, attachments, obstacles, q12)
    set_grippers(grippers, GRIPPER_OPEN)
    print('\n=== where the finger tips stand (jaws fully open) ===')
    overlap_report(grippers, boxes, layout)

    # 2. The same pose against the parts' TRUE meshes instead of their boxes.
    print('\n=== the same check with the true part meshes instead of the boxes ===')
    meshes = load_part_bodies(layout)
    for name, body in meshes.items():
        OBSTACLE_LABELS[body] = f'mesh {name}'
    with_meshes = [obstacles[0], obstacles[1]] + list(meshes.values())
    fn_mesh = build_collision_fn(robot, attachments, with_meshes)
    for label, angle in (('jaws fully open', GRIPPER_OPEN),
                         ('jaws at 0.426', MONITOR_PRE_OPEN),
                         ('jaws fully closed', GRIPPER_CLOSE)):
        set_grippers(grippers, angle)
        verdict = fn_mesh(q12)
        print(f'--- {label}: {"IN COLLISION" if verdict else "clear"}')
        if verdict:
            describe_collision(robot, attachments, with_meshes, q12)

    with open(args.out, 'w') as f:
        json.dump({'source': source, 'q12': q12.tolist(),
                   'joint_names': JOINTS_12,
                   'ur_tcp_pose': tcp_poses,
                   'tool0_pybullet': {side: {'xyz': list(map(float, pose[0])),
                                             'quat_xyzw': list(map(float, pose[1]))}
                                      for side, pose in zip(ARM_SIDES, tool0_poses)}},
                  f, indent=1)
    print(f'\nsaved joints + tool0 poses -> {args.out}  '
          f'(next: live_pose_in_planner.py in the planner venv)')
    pp.disconnect()


if __name__ == '__main__':
    sys.exit(main())
