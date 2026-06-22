"""View the husky+UR5e ARM skeleton at one calibration point (Grasshopper or terminal).

Reads {date}_{batch}_robot_skeleton.json (written by export_robot_skeleton.py) and shows
the arm pose for the selected take (curve) + point. Pure stdlib (json/math) so it runs in
a Rhino8 GhPython (CPython) component AND in a plain terminal; no pybullet/ROS needed.

All geometry is in the mocap WORLD frame (origin = Motive origin), in millimetres -- the
same frame as the step-1 outlier/circle viewer, so the two overlay directly.

RUN_MODE:
  "grasshopper" -> assigns geometry to these GhPython OUTPUT params (name them on the
     component): joints (Point3d list), bones (Line list), tool0_plane (Plane), info (str).
     Wire two integer sliders to INPUT params named TAKE and POINT to scrub.
  "terminal"    -> prints the selected take/point, joint_conf and the link xyz.

The take/point indices match the step-1 outlier CSV (take_idx + pt_idx).
"""

import json
import os
import shutil

# ============================ user controls ============================
RUN_MODE = "grasshopper"   # "grasshopper" -> GH outputs; "terminal" -> print here
TAKE_IDX = 0               # which take/curve (GH input param TAKE overrides this)
POINT_IDX = 0              # which point in the take (GH input param POINT overrides this)
SHOW_TOOL0_FRAME = True    # output a Plane at tool0 (grasshopper mode)

_EXPORT_DIR = (
    "/home/su/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/"
    "2025-03 Husky Assembly/data_experiment/visualise_calibration_to_rhino"
)
SKELETON_JSON = _EXPORT_DIR + r"/20260615_j0_robot_skeleton.json"
# =======================================================================


def load_skeleton(path):
    with open(path) as f:
        return json.load(f)


def clamp(i, n):
    """Clamp index into [0, n-1] (sliders may overshoot)."""
    return max(0, min(int(i), n - 1)) if n else 0


def select(skel, take_idx, point_idx):
    """Return (take, point, take_idx, point_idx) with indices clamped to valid range."""
    takes = skel['takes']
    ti = clamp(take_idx, len(takes))
    take = takes[ti]
    pi = clamp(point_idx, len(take['points']))
    return take, take['points'][pi], ti, pi


def info_text(skel, take, point, ti, pi):
    jc = ", ".join(f"{v:.3f}" for v in point['joint_conf'])
    return (f"{take['traj_label']} #{point['pt_idx']}  "
            f"(take {ti}/{len(skel['takes']) - 1}, point {pi}/{len(take['points']) - 1})\n"
            f"joints[rad]: {jc}")


# ============================ terminal mode ============================
def print_terminal(skel, take_idx, point_idx):
    take, point, ti, pi = select(skel, take_idx, point_idx)
    print("=" * 64)
    print(f"robot skeleton  {skel['date']} {skel['batch']} arm={skel['arm']} ({skel['frame']})")
    print(f"  {len(skel['takes'])} takes; selected take {ti} ({take['traj_label']}) "
          f"has {len(take['points'])} points")
    print("=" * 64)
    print(info_text(skel, take, point, ti, pi))
    print("link positions (mm):")
    for name, xyz in zip(skel['link_order'], point['link_xyz_mm']):
        print(f"  {name:34s} ({xyz[0]:9.1f}, {xyz[1]:9.1f}, {xyz[2]:9.1f})")
    print("done")


# ============================ grasshopper mode ============================
def build_grasshopper(skel, take_idx, point_idx):
    """Return (joints, bones, tool0_plane, info) using Rhino.Geometry."""
    import Rhino.Geometry as rg

    take, point, ti, pi = select(skel, take_idx, point_idx)
    pts = [rg.Point3d(x, y, z) for x, y, z in point['link_xyz_mm']]

    # bones: lines between consecutive links (skip near-zero segments)
    bones = []
    for a, b in zip(pts[:-1], pts[1:]):
        if a.DistanceTo(b) > 1e-6:
            bones.append(rg.Line(a, b))

    tool0_plane = None
    if SHOW_TOOL0_FRAME:
        qx, qy, qz, qw = point['tool0_quat']        # pybullet order [x,y,z,w]
        try:
            q = rg.Quaternion(qw, qx, qy, qz)        # Rhino order (w,x,y,z)
            ok, pln = q.GetRotation()
            if ok:
                pln.Origin = pts[-1]
                tool0_plane = pln
        except Exception:
            tool0_plane = None

    return pts, bones, tool0_plane, info_text(skel, take, point, ti, pi)


def _resolve_index(name, default):
    """Use the GH input param `name` if the component provides it, else the constant."""
    val = globals().get(name, None)
    try:
        return int(val) if val is not None else default
    except (TypeError, ValueError):
        return default


def sync_self_to_drive():
    """Keep a copy of this script in _EXPORT_DIR. No-op from a GH buffer (virtual __file__),
    the Drive copy itself, or if the dir isn't mounted."""
    try:
        src = os.path.abspath(__file__)
    except NameError:
        return
    if not os.path.isfile(src):          # GH 'rhinocode:' virtual path -> not a real file
        return
    dest = os.path.join(_EXPORT_DIR, os.path.basename(src))
    if not os.path.isdir(_EXPORT_DIR) or os.path.abspath(dest) == src:
        return
    try:
        shutil.copy2(src, dest)
        print(f"[sync] copied script -> {dest}")
    except OSError as e:
        print(f"[sync] skip copy: {e}")


def main():
    sync_self_to_drive()
    skel = load_skeleton(SKELETON_JSON)
    take_idx = _resolve_index('TAKE', TAKE_IDX)
    point_idx = _resolve_index('POINT', POINT_IDX)

    if RUN_MODE == "terminal":
        print_terminal(skel, take_idx, point_idx)
    elif RUN_MODE == "grasshopper":
        joints, bones, tool0_plane, info = build_grasshopper(skel, take_idx, point_idx)
        # expose as module globals so GhPython picks them up as outputs
        globals().update(joints=joints, bones=bones, tool0_plane=tool0_plane, info=info)
        print(info)
    else:
        raise ValueError(f"RUN_MODE must be 'grasshopper' or 'terminal', got {RUN_MODE!r}")


main()
