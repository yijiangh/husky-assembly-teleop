"""Export mocap camera poses (in the Mocap origin frame) to CSV + JSON for Rhino8.

The camera name / position / orientation come from the NatNet "camera description"
(NatNetClient.__unpack_camera_description, Type 5), saved per-take by
husky_monitor.get_mocap_camera_inventory() into each take JSON under
data/mocap_experiments/<date>/<session>/takes/*.json as `mocap_camera_inventory`.
NOTE: calibration trajectory files (data/calibration_data/<date>/) do NOT carry
camera inventory -- only mocap_experiments takes do.

That saved data is RAW MOCAP (y-up) meters: position=(x,y,z), orientation
quaternion=(qx,qy,qz,qw). This script applies the same y-up -> z-up convention the
rest of the repo uses (utils.mocap_pos/quat_y_up_to_z_up, default 'rhino') so the
cameras land in the Rhino model frame, then writes (into OUTPUT_DIR):

  <out>/<stem>_cameras.json  metadata + list of {name, position[xyz], orientation[xyzw]}
  <out>/<stem>_cameras.csv   name,x,y,z,qx,qy,qz,qw   (one row per camera)

Positions stay in METERS here; scale to your Rhino doc units inside the importer
(import_mocap_cameras_rhino.py UNIT_SCALE, default 100 for a centimeter document).

Usage:
    python export_mocap_cameras.py <take.json> [--out DIR] [--convention rhino|rotated|raw]
    python export_mocap_cameras.py --latest        # newest take in the repo
"""

import argparse
import csv
import glob
import json
import os

# Repo take files live here; only used by --latest. Edit if your repo moved.
MOCAP_EXPERIMENTS_DIR = (
    "/home/su/ros2_ws/src/husky-assembly-teleop/data/mocap_experiments"
)

# Default output folder (csv + json land next to the Rhino importer script).
OUTPUT_DIR = (
    "/home/su/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/"
    "2025-03 Husky Assembly/data_experiment/visualise_mocap_camera"
)


# Inline copies of utils.mocap_*_y_up_to_z_up (kept dependency-free so this stays
# a standalone script; husky_assembly_teleop/utils.py is the source of truth).
def _pos_y_up_to_z_up(pos, convention):
    if convention == "raw":
        return [pos[0], pos[1], pos[2]]
    if convention == "rhino":
        return [pos[0], -pos[2], pos[1]]
    if convention == "rotated":
        return [pos[2], pos[0], pos[1]]
    raise ValueError(f"unknown convention {convention!r}")


def _quat_y_up_to_z_up(quat, convention):
    qx, qy, qz, qw = quat
    if convention == "raw":
        return [qx, qy, qz, qw]
    if convention == "rhino":
        return [qx, -qz, qy, qw]
    if convention == "rotated":
        return [qz, qx, qy, qw]
    raise ValueError(f"unknown convention {convention!r}")


def _find_latest_take():
    pattern = os.path.join(MOCAP_EXPERIMENTS_DIR, "*", "*", "takes", "*.json")
    takes = sorted(glob.glob(pattern))
    if not takes:
        raise FileNotFoundError(f"no take JSON found under {MOCAP_EXPERIMENTS_DIR}")
    return takes[-1]


def export(take_path, out_dir=None, convention="rhino"):
    with open(take_path) as f:
        take = json.load(f)

    inventory = take.get("mocap_camera_inventory")
    if not inventory or not inventory.get("cameras"):
        raise ValueError(
            f"no mocap_camera_inventory.cameras in {take_path} "
            f"(calibration trajectory files do not carry camera inventory; "
            f"use a take under {MOCAP_EXPERIMENTS_DIR})"
        )

    cameras = []
    for cam in inventory["cameras"]:
        cameras.append(
            {
                "name": cam["name"],
                "position": _pos_y_up_to_z_up(cam["position"], convention),
                "orientation": _quat_y_up_to_z_up(cam["orientation"], convention),
            }
        )

    stem = take.get("take_stem") or os.path.splitext(os.path.basename(take_path))[0]
    out_dir = out_dir or OUTPUT_DIR
    os.makedirs(out_dir, exist_ok=True)
    json_path = os.path.join(out_dir, f"{stem}_cameras.json")
    csv_path = os.path.join(out_dir, f"{stem}_cameras.csv")

    payload = {
        "source_take": os.path.abspath(take_path),
        "frame": "mocap_origin",
        "axis_convention": convention,
        "position_units": "meters",
        "orientation": "quaternion_xyzw",
        "camera_count": len(cameras),
        "cameras": cameras,
    }
    with open(json_path, "w") as f:
        json.dump(payload, f, indent=2)

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["name", "x", "y", "z", "qx", "qy", "qz", "qw"])
        for cam in cameras:
            writer.writerow([cam["name"], *cam["position"], *cam["orientation"]])

    print(f"wrote {len(cameras)} cameras (convention={convention}):")
    print(f"  {json_path}")
    print(f"  {csv_path}")
    return json_path, csv_path


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("take", nargs="?", help="take JSON path (omit with --latest)")
    ap.add_argument("--latest", action="store_true", help="use newest take in the repo")
    ap.add_argument("--out", help=f"output directory (default: {OUTPUT_DIR})")
    ap.add_argument("--convention", default="rhino", choices=("rhino", "rotated", "raw"))
    args = ap.parse_args()

    take_path = _find_latest_take() if args.latest else args.take
    if not take_path:
        ap.error("provide a take JSON path or --latest")
    export(take_path, args.out, args.convention)


if __name__ == "__main__":
    main()
