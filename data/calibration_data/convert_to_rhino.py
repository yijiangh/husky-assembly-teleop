"""Convert a base-mocap calibration file to the 'rhino' axis convention.

The pose value `base_mocap_from_base_footprint` is corrected by applying a
-90° rotation about its child-frame (base_footprint) Z axis. Position is
unchanged.

Empirical reason: under the legacy 'rotated' y-up->z-up convention the
robot at identity faces mocap_z; under 'rhino' it faces mocap_x — that's
a -90° rotation about world Z (and equivalently about base_footprint's
local Z for a floor-standing robot). The calibration captured under the
old convention encodes the wrong base yaw if used as-is, so we right-mul
the stored quaternion by q_z(-90°):

    calib_rhino.pos  = calib_rotated.pos
    calib_rhino.quat = calib_rotated.quat ⊗ q_z(-90°)

Other top-level pose keys (debug/reference, e.g.
`base_mocap_from_<arm>_arm_base_link_inertia`) are passed through
unchanged — only `base_mocap_from_base_footprint` is loaded at runtime
(see common.py).

Usage:
    python convert_to_rhino.py <input.json> [-o <output.json>]
If `-o` is omitted, output is `<input>_rhino.json` next to the input.
"""

import argparse
import json
import math
import os
import sys


# q_z(-90°) as (qx, qy, qz, qw)
_SQRT2_OVER_2 = math.sqrt(2.0) / 2.0
Q_Z_NEG_90 = (0.0, 0.0, -_SQRT2_OVER_2, _SQRT2_OVER_2)


def quat_mul(a, b):
    """Hamilton quaternion product a ⊗ b, both as (qx, qy, qz, qw)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def rotate_pose_about_child_z(pose, q_delta):
    """Right-multiply pose.quat by q_delta. Position unchanged.

    pose is ``[[px, py, pz], [qx, qy, qz, qw]]``.
    """
    pos, quat = pose
    return [list(pos), quat_mul(tuple(quat), q_delta)]


def convert(input_path: str, output_path: str | None = None) -> str:
    if not os.path.exists(input_path):
        sys.exit(f"input not found: {input_path}")

    with open(input_path, 'r') as f:
        data = json.load(f)

    if output_path is None:
        root, ext = os.path.splitext(input_path)
        output_path = f"{root}_rhino{ext}"

    out = dict(data)

    # Apply the -90° z fix to the runtime-loaded calibration only.
    key = 'base_mocap_from_base_footprint'
    if key in out:
        out[key] = rotate_pose_about_child_z(out[key], Q_Z_NEG_90)
        print(f"applied -90° z fix to '{key}'")
    else:
        print(f"WARN: '{key}' not found in input")

    out['mocap_axis_convention'] = 'rhino'
    out['_source_calibration_file'] = os.path.basename(input_path)
    out['_conversion_note'] = (
        "Empirical fix for rotated->rhino: base_mocap_from_base_footprint "
        "quat is right-multiplied by q_z(-90°). Position unchanged. Other "
        "top-level poses (e.g. base_mocap_from_<arm>_arm_base_link_inertia) "
        "are passed through unchanged — they are reference-only and not "
        "loaded at runtime."
    )

    with open(output_path, 'w') as f:
        json.dump(out, f, indent=4)

    print(f"wrote {output_path}")
    return output_path


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='path to existing calibrated_transformation_*.json')
    parser.add_argument('-o', '--output', default=None,
                        help='output path (default: <input>_rhino.json)')
    args = parser.parse_args()
    convert(args.input, args.output)
