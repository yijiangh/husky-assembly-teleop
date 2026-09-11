#!/usr/bin/env python3
"""Run a LIVE arm pose through the planner's own model: view it, collision-check it.

* Second half of `live_pose_check.py`. That script saved the arms' joints
* (RTDE) and PyBullet's tool0 poses; this one rebuilds the planner's scene from
* the plan the trajectory came from (same layout, same parts on the table,
* same rai robot model with its certified collision proxies), puts the arms
* at those joints, and answers two questions:
*
*   1. does the PLANNER's model think this pose collides? Its proxies are the
*      geometry every witness was certified against; MuJoCo mirrors them by
*      FK, so this is the same answer the sim would give.
*   2. do the two robot models even agree where the flanges are? PyBullet's
*      tool0 (saved in the JSON) is compared with rai's at the same joints. A
*      difference of millimetres here is a base/arm-mount calibration
*      disagreement between the two URDFs -- a cause worth knowing before
*      blaming any collision shape.

Run in the PLANNER venv (any working directory):

    source ~/Code/fixtureless-assembly/.venv/bin/activate
    python3 /home/su/ros2_ws/src/husky-assembly-teleop/scripts/husky/live_pose_in_planner.py \
        --plan experiments/real-stool-husky-current/plan.json --q-json /tmp/live_q.json [--view]
"""

import argparse
import json
import os
import sys

import numpy as np

# ! Two things must be true to run this, and neither is the ros2 setup:
# !   1. the PLANNER venv (~/Code/fixtureless-assembly/.venv) is active -- rai
# !      ("robotic") lives only there;
# !   2. the planner checkout is importable -- found here from --planner-root,
# !      so the working directory no longer matters.
DEFAULT_PLANNER_ROOT = os.path.expanduser('~/Code/fixtureless-assembly')

# Teleop arm side -> planner robot name (env.make_husky_duo_env: right = a1, left = a2).
ROBOT_OF_SIDE = {'left': 'a2', 'right': 'a1'}
UR_JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')


def main():
    cli = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    cli.add_argument('--plan', required=True, help='plan.json the trajectory was solved from')
    cli.add_argument('--q-json', default='/tmp/live_q.json')
    cli.add_argument('--view', action='store_true', help='open the rai viewer on the pose')
    cli.add_argument('--planner-root', default=DEFAULT_PLANNER_ROOT,
                     help='the fixtureless-assembly checkout (default: %(default)s)')
    args = cli.parse_args()

    root = os.path.abspath(os.path.expanduser(args.planner_root))
    sys.path.insert(0, root)
    if not os.path.isabs(args.plan):                    # plan paths are relative to the checkout
        args.plan = os.path.join(root, args.plan)
    try:
        import robotic                                   # noqa: F401  (rai)
    except ImportError:
        raise SystemExit('rai ("robotic") is not importable: activate the PLANNER venv first:\n'
                         f'    source {root}/.venv/bin/activate')
    os.chdir(root)                                       # the planner resolves its .g/mesh paths from here

    from plan_io import read_plan
    from problem import _quat_to_R
    from replay import build_scene

    with open(args.q_json) as f:
        saved = json.load(f)
    q12 = np.asarray(saved['q12'], dtype=float)
    print(f'joints from {saved.get("source")}')

    cfg, _plan = read_plan(args.plan)
    C, _assembly = build_scene(cfg)

    # Set the twelve arm joints BY NAME: the planner's joint order is its own.
    names = list(C.getJointNames())
    full = np.asarray(C.getJointState(), dtype=float)
    for k, side in enumerate(('left', 'right')):
        robot = ROBOT_OF_SIDE[side]
        for j, joint in enumerate(UR_JOINTS):
            name = f'{robot}_ur_{joint}'
            if name not in names:
                raise SystemExit(f'planner model has no joint {name}; joints are {names}')
            full[names.index(name)] = q12[6 * k + j]
    C.setJointState(full)

    # 1. Joint limits, then collisions, with the planner's own proxies.
    lo, hi = (np.asarray(x, dtype=float) for x in C.getJointLimits())
    outside = [(names[i], full[i], lo[i], hi[i]) for i in range(len(full))
               if names[i].startswith(('a1_ur_', 'a2_ur_')) and not lo[i] <= full[i] <= hi[i]]
    if outside:
        print('JOINT LIMITS (planner model):')
        for name, v, l, h in outside:
            print(f'  {name} = {v:.4f} outside [{l:.4f}, {h:.4f}]')
    C.computeCollisions()
    pairs = [(a, b, d) for a, b, d in C.getCollisions() if d < 0.0]
    print(f'planner collision check: total penetration '
          f'{1000 * C.getCollisionsTotalPenetration():.2f} mm, '
          f'{len(pairs)} penetrating pair(s)')
    for a, b, d in sorted(pairs, key=lambda x: x[2])[:12]:
        print(f'  {a} ~ {b}: {1000 * d:+.2f} mm')

    # 2. Flange positions: the two robot models at the same joints.
    print('tool0 (flange) positions, PyBullet vs planner, same joints:')
    for side in ('left', 'right'):
        frame = C.getFrame(f'{ROBOT_OF_SIDE[side]}_ur_tool0')
        rai_xyz = np.asarray(frame.getPosition(), dtype=float)
        pb_xyz = np.asarray(saved['tool0_pybullet'][side]['xyz'], dtype=float)
        print(f'  {side}: planner {np.round(rai_xyz, 4).tolist()}  pybullet '
              f'{np.round(pb_xyz, 4).tolist()}  |diff| {1000 * np.linalg.norm(rai_xyz - pb_xyz):.1f} mm')
        gc = C.getFrame(f'{ROBOT_OF_SIDE[side]}_ur_gripper_center')
        print(f'         planner gripper_center {np.round(gc.getPosition(), 4).tolist()}')
    if args.view:
        C.view(True, 'live pose in the planner model (close to exit)')


if __name__ == '__main__':
    sys.exit(main())
