"""Export assembly-tool meshes from a cfab RobotCell to OBJ files.

The cfab RobotCell stores the per-arm tool body as a compas RigidBody
under ``rigid_body_models[<name>]`` (typical names:
``AssemblyLeftArmToolBody`` / ``AssemblyRightArmToolBody``). The mesh's
native unit is mm (``native_scale=0.001``), matching the legacy
``assembly_tool_v3_<side>_mm.obj`` convention used by the pp-side
visualization in ``husky_assembly_teleop.common.create_end_effector``.

This script writes one OBJ per requested side to DATA_DIRECTORY, named
``assembly_tool_v3_<side>_mm_from_cell.obj``. ``common.create_end_effector``
prefers this `_from_cell` file when present and falls back to the legacy
`_mm.obj` so the planning + visualization meshes stay in sync.

Usage:
    cd /home/yijiangh/Code/ros2_ws
    source venv/bin/activate
    source install/setup.bash
    python src/husky-assembly-teleop/scripts/export_tool_mesh_from_cell.py
        [--problem-name <name>]   # defaults to DESIGN_PROBLEM_NAME
        [--side left|right|both]  # defaults to both
        [--collision]             # use collision_meshes (default: visual_meshes)
"""

import argparse
import os
import sys

import pybullet_planning as pp

from husky_assembly_teleop import DATA_DIRECTORY, DESIGN_PROBLEM_NAME
from husky_assembly_teleop.cfab_session import CfabSession


_TOOL_RB_NAMES = {
    'left': 'AssemblyLeftArmToolBody',
    'right': 'AssemblyRightArmToolBody',
}


def export_one(robot_cell, side: str, output_path: str, use_collision: bool):
    name = _TOOL_RB_NAMES[side]
    rb = robot_cell.rigid_body_models.get(name)
    if rb is None:
        sys.exit(f"rigid body {name!r} not found in robot_cell.rigid_body_models")
    meshes = rb.collision_meshes if use_collision else rb.visual_meshes
    if not meshes:
        sys.exit(f"{name!r} has no {'collision' if use_collision else 'visual'}_meshes")
    if len(meshes) > 1:
        print(f"WARN: {name!r} has {len(meshes)} meshes; exporting only the first.")
    meshes[0].to_obj(output_path)
    print(f"wrote {output_path}  ({meshes[0].number_of_vertices()} verts, "
          f"{meshes[0].number_of_faces()} faces, native_scale={rb.native_scale})")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--problem-name', default=DESIGN_PROBLEM_NAME)
    parser.add_argument('--side', choices=['left', 'right', 'both'], default='both')
    parser.add_argument('--collision', action='store_true',
                        help='Export collision_meshes (default: visual_meshes)')
    args = parser.parse_args()

    pp.connect(use_gui=False)
    sess = CfabSession(args.problem_name, connection_type='direct', enable_debug_gui=False)
    rc = sess.robot_cell

    sides = ['left', 'right'] if args.side == 'both' else [args.side]
    for side in sides:
        out = os.path.join(DATA_DIRECTORY, f'assembly_tool_v3_{side}_mm_from_cell.obj')
        export_one(rc, side, out, args.collision)


if __name__ == '__main__':
    main()
