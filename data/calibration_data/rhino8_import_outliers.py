"""View / debug calibration circles + outlier points (step 1), from raw analysis data.

Reads `{batch}_analysis.json` (staged into the Drive folder by
visualize_calibration_outliers_rhino.py). That file holds, per take, the UNCHANGED raw_data
(base_mocap_pose, flange_mocap_pose, joint_conf, ...) PLUS the base-frame circle fit
(center, normal). We REUSE that fit (no re-fitting) and re-express everything in the
**Motive world frame** (origin = Motive origin) using light pure-Python pose math, so it
runs in a Rhino8 GhPython (CPython) component AND a plain terminal — no numpy/pybullet.

Frame: points come from `flange_mocap_pose`, the circle from the stored base-frame fit;
both are mapped into Motive world via a reference base pose (first sample's `base_mocap_pose`)
-- "base-fit moved to Motive". Nothing uses tool0_fk_pose / tool0_fk_from_mocap.

RUN_MODE:
  "grasshopper" -> assign DataTree outputs (one branch {i} per curve) to GhPython OUTPUT params:
     circles, points, point_labels, circle_labels, colors (one per branch), origin (Point3d).
     Points are grouped under their curve. Wire a Text Tag (3D) for on-point labels and a
     colored preview keyed by branch to color points to match their curve.
  "terminal" -> print circles then outlier points grouped per curve {i}.

Coordinates are in mm. The take/point indices match the calibration collection (pt_idx =
raw_data index). Overlays the step-2 robot skeleton at the same Motive origin.
"""

import json
import math
import os
import re
import shutil

# ============================ user controls ============================
RUN_MODE = "grasshopper"    # "grasshopper" -> GH DataTree outputs; "terminal" -> print here
DATE = "20260615"           # which dated subfolder in the Drive folder
BATCH = "j0"                # which batch's analysis file (j0 / j1)
THRESHOLD = "mean"          # "mean" (per-curve mean error) or a number in mm -> outlier cutoff
POINT_LABELS = True         # build a label per outlier point
LABEL_TEXT_SIZE = 12.0      # text height for the GH Text Tag (passed through as `text_size`)
# Optional label fields (jX_tY and the point #index are always shown). Date/time come from the
# take file name, e.g. calibration_20260615_1301_... -> date 20260615, time 1301.
SHOW_DATE = False
SHOW_TIME = False
SHOW_ERROR_DISTANCE = True  # append "  <err>mm" to outlier point labels

_EXPORT_DIR = (
    "/home/su/Insync/yijiang94817@gmail.com/Google Drive - Shared with me/"
    "2025-03 Husky Assembly/data_experiment/visualise_calibration_to_rhino"
)
ANALYSIS_JSON = _EXPORT_DIR + "/" + DATE + "/" + BATCH + "_analysis.json"
# =======================================================================


# ---------------------------- pose math (pure python, no numpy) ----------------------------
# pose = [[px,py,pz], [qx,qy,qz,qw]]  (pybullet quaternion order x,y,z,w)
def _sub(a, b): return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
def _add(a, b): return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
def _scale(a, s): return [a[0] * s, a[1] * s, a[2] * s]
def _dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
def _norm(a): return math.sqrt(_dot(a, a))
def _unit(a):
    n = _norm(a)
    return _scale(a, 1.0 / n) if n > 1e-12 else a


def qrot(q, v):
    """Rotate vector v by quaternion q=[x,y,z,w]."""
    u, s = [q[0], q[1], q[2]], q[3]
    t = _scale(_cross(u, v), 2.0)
    return _add(_add(v, _scale(t, s)), _cross(u, t))


def transform_point(pose, v):
    return _add(qrot(pose[1], v), pose[0])


def pose_inv(pose):
    q = pose[1]
    qi = [-q[0], -q[1], -q[2], q[3]]
    return [_scale(qrot(qi, pose[0]), -1.0), qi]


# ---------------------------- labels ----------------------------
_TAKE_RE = re.compile(r"calibration_(\d+)_(\d+)_", re.IGNORECASE)


def _date_time(file_name):
    m = _TAKE_RE.search(file_name or "")
    return (m.group(1), m.group(2)) if m else ("", "")


def label_prefix(file_name, traj_label):
    """'jX_tY' with optional date/time prefix (shared by points and curves)."""
    short = traj_label.replace("_traj", "_t")
    date, time = _date_time(file_name)
    parts = []
    if SHOW_DATE and date:
        parts.append(date)
    if SHOW_TIME and time:
        parts.append(time)
    parts.append(short)
    return " ".join(parts)


def point_label(file_name, traj_label, pt_idx, err_mm):
    text = f"{label_prefix(file_name, traj_label)} #{pt_idx}"
    if SHOW_ERROR_DISTANCE:
        text += f"  {err_mm:.2f}mm"
    return text


def traj_label(file_name, take_idx):
    m = re.search(r"_J(\d+)_traj(\d+)", file_name or "")
    return f"j{m.group(1)}_traj{m.group(2)}" if m else f"take{take_idx}"


# ---------------------------- core ----------------------------
def compute_curves(analysis):
    """Return a list of curve dicts (one per valid take), Motive-world frame, mm.

    Each: {branch, label, file_name, center_mm, normal, radius_mm, points:[{pt_idx,xyz_mm,err_mm}]}
    points are the outliers only (err above the per-curve cutoff).
    """
    curves = []
    branch = 0
    for take_idx, take in enumerate(analysis["takes"]):
        fname = take["file_name"]
        label = traj_label(fname, take_idx)

        # rebuild base-frame points + keep raw index (= pt_idx), exactly like 0_circle_fitting
        pts_base, raw_idx, base_ref = [], [], None
        for ri, e in enumerate(take["raw_data"]):
            flange, base = e.get("flange_mocap_pose"), e.get("base_mocap_pose")
            if flange and base:
                if base_ref is None:
                    base_ref = base
                pts_base.append(transform_point(pose_inv(base), flange[0]))
                raw_idx.append(ri)
        if len(pts_base) < 3:
            continue

        # move into Motive world via the reference base pose
        pts_w = [transform_point(base_ref, p) for p in pts_base]
        center_w = transform_point(base_ref, take["center"])
        normal_w = _unit(qrot(base_ref[1], take["normal"]))

        # radius + per-point distance-to-circle (mm), reusing the stored fit
        errs, in_plane = [], []
        for p in pts_w:
            d = _sub(p, center_w)
            perp = _dot(d, normal_w)
            r_ip = _norm(_sub(d, _scale(normal_w, perp)))
            in_plane.append(r_ip)
        radius = sum(in_plane) / len(in_plane)
        for r_ip, p in zip(in_plane, pts_w):
            d = _sub(p, center_w)
            perp = _dot(d, normal_w)
            errs.append(math.sqrt(perp * perp + (r_ip - radius) ** 2))
        errs_mm = [e * 1000.0 for e in errs]

        cutoff = (sum(errs_mm) / len(errs_mm)) if str(THRESHOLD).lower() == "mean" else float(THRESHOLD)

        points = []
        for ri, p, e_mm in zip(raw_idx, pts_w, errs_mm):
            if e_mm > cutoff:
                points.append({"pt_idx": ri, "xyz_mm": _scale(p, 1000.0), "err_mm": e_mm})

        curves.append({
            "branch": branch, "label": label, "file_name": fname,
            "center_mm": _scale(center_w, 1000.0), "normal": normal_w,
            "radius_mm": radius * 1000.0, "cutoff_mm": cutoff, "points": points,
        })
        branch += 1
    return curves


def load_analysis(path):
    with open(path) as f:
        return json.load(f)


# ---------------------------- terminal mode ----------------------------
def print_terminal(curves):
    print("=" * 64)
    print(f"calibration viewer (terminal)  {DATE} {BATCH}  frame=mocap_world_mm")
    print(f"  threshold: {THRESHOLD}")
    print("=" * 64)
    print(f"\n[circles] {len(curves)} curves")
    for c in curves:
        cm = c["center_mm"]
        print(f"  {{{c['branch']}}} {label_prefix(c['file_name'], c['label']):>16}  "
              f"r={c['radius_mm']:8.2f} mm  center=({cm[0]:8.1f}, {cm[1]:8.1f}, {cm[2]:8.1f})")
    total = sum(len(c["points"]) for c in curves)
    print(f"\n[outliers] {total} points (grouped per curve)")
    for c in curves:
        pts = c["points"]
        items = ", ".join(f"#{p['pt_idx']} {p['err_mm']:.2f}mm" for p in pts)
        print(f"  {{{c['branch']}}} {c['label'].replace('_traj', '_t')} "
              f"(cutoff {c['cutoff_mm']:.2f}mm, {len(pts)} pts): {items}")
    print("\ndone")


# ---------------------------- grasshopper mode ----------------------------
def build_grasshopper(curves):
    """Return dict of GhPython outputs as DataTrees keyed per-curve branch {i}."""
    import Rhino.Geometry as rg
    import System.Drawing as sd
    from Grasshopper import DataTree
    from Grasshopper.Kernel.Data import GH_Path

    palette = [
        sd.Color.Red, sd.Color.DodgerBlue, sd.Color.ForestGreen, sd.Color.Orange,
        sd.Color.Magenta, sd.Color.Teal, sd.Color.Goldenrod, sd.Color.MediumPurple,
        sd.Color.Brown, sd.Color.DeepPink,
    ]

    circles = DataTree[object]()
    points = DataTree[object]()
    point_labels = DataTree[object]()
    circle_labels = DataTree[object]()
    colors = DataTree[object]()

    for c in curves:
        path = GH_Path(c["branch"])
        col = palette[c["branch"] % len(palette)]
        center = rg.Point3d(*c["center_mm"])
        normal = rg.Vector3d(*c["normal"])
        circles.Add(rg.Circle(rg.Plane(center, normal), c["radius_mm"]), path)
        circle_labels.Add(label_prefix(c["file_name"], c["label"]), path)
        colors.Add(col, path)
        for p in c["points"]:
            points.Add(rg.Point3d(*p["xyz_mm"]), path)
            if POINT_LABELS:
                point_labels.Add(
                    point_label(c["file_name"], c["label"], p["pt_idx"], p["err_mm"]), path)

    print(f"{len(curves)} curves, {sum(len(c['points']) for c in curves)} outlier points")
    return {
        "circles": circles, "points": points, "point_labels": point_labels,
        "circle_labels": circle_labels, "colors": colors,
        "origin": rg.Point3d(0, 0, 0), "text_size": LABEL_TEXT_SIZE,
    }


# ---------------------------- self-copy ----------------------------
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
    curves = compute_curves(load_analysis(ANALYSIS_JSON))
    if RUN_MODE == "terminal":
        print_terminal(curves)
    elif RUN_MODE == "grasshopper":
        globals().update(build_grasshopper(curves))
    else:
        raise ValueError(f"RUN_MODE must be 'grasshopper' or 'terminal', got {RUN_MODE!r}")


main()
