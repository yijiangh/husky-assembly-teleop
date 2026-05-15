"""Convert a base-mocap calibration file to the 'rhino' axis convention.

The calibration `base_mocap_from_base_footprint` (and the per-arm
`base_mocap_from_*_arm_base_link*`) is a *relative* pose between two
physical frames (mocap RB local ↔ robot link). Both frames are physical
and do not depend on our world axis convention, so the (pos, quat) values
are mathematically invariant under the y-up→z-up convention switch
(rotated ↔ rhino).

This script therefore performs an identity copy of all pose values; the
only added information is a top-level marker
``"mocap_axis_convention": "rhino"`` so the runtime can pick the right
file by `HuskyMonitor.MOCAP_AXIS_CONVENTION`.

Math (with P = R_z(-90°) = rhino<-rotated change-of-basis):
    world_from_mocap_H = P · world_from_mocap_R
    world_from_BF_H    = P · world_from_BF_R
    calib              = inv(world_from_mocap) · world_from_BF
    => calib_H = inv(P·wm_R) · (P·BF_R)
              = inv(wm_R) · inv(P) · P · BF_R
              = calib_R                          ← identity

Usage:
    python convert_to_rhino.py <input.json> [-o <output.json>]
If `-o` is omitted, output is `<input>_rhino.json` next to the input.
"""

import argparse
import json
import os
import sys


def convert(input_path: str, output_path: str | None = None) -> str:
    if not os.path.exists(input_path):
        sys.exit(f"input not found: {input_path}")

    with open(input_path, 'r') as f:
        data = json.load(f)

    if output_path is None:
        root, ext = os.path.splitext(input_path)
        output_path = f"{root}_rhino{ext}"

    out = dict(data)
    out['mocap_axis_convention'] = 'rhino'
    out['_source_calibration_file'] = os.path.basename(input_path)
    out['_conversion_note'] = (
        'Identity copy: base_mocap_from_* poses are invariant under '
        'rotated↔rhino y-up->z-up convention switch.'
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
