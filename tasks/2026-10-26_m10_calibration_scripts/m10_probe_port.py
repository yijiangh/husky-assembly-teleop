"""Scratch check (planner research only): re-compute the old calibration math in plain numpy on stored sessions."""
import json, os, sys
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from husky_assembly_teleop.robot_interface.frames import fixed_transform

ROOT = Path(os.environ["HUSKY_DRIVE_ROOT"]) / "data_experiment/calibration_data"
URDF_NEW = Path("/home/yijiangh/Code/ros2_ws_next/dev/jg-sync/data/husky_urdf")
URDF_OLD = Path(sys.argv[1])
SESS = {"20261016": ("0806", "left_ur_arm", "mt_husky_dual_ur5_e_moveit_config/urdf/husky_dual_ur5_e_no_base_joint_All_Calibrated.urdf"),
        "20261013": ("0805", "ur_arm", "mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Belle_Calibrated.urdf"),
        "20260623": ("0804", "ur_arm", "mt_husky_moveit_config/urdf/husky_ur5_e_no_base_joint_Alice_Calibrated.urdf")}

def T(pose):  # [[x,y,z],[qx,qy,qz,qw]] -> 4x4
    M = np.eye(4); M[:3, :3] = Rot.from_quat(pose[1]).as_matrix(); M[:3, 3] = pose[0]; return M

def circle(points):
    P = np.unique(np.asarray(points, float), axis=0)
    m = P.mean(axis=0)
    U, S, _ = np.linalg.svd((P - m).T)
    n = U[:, 2]
    n_unit = n / np.linalg.norm(n)
    Q = P - np.outer((P - m) @ n / np.linalg.norm(n), n_unit)
    qc = Q.mean(axis=0); Qc = Q - qc
    u = (Qc[1] - Qc[0]) / np.linalg.norm(Qc[1] - Qc[0]); v = np.cross(u, n_unit)
    xy = Qc @ np.column_stack((u, v, n_unit))
    A = np.hstack((2 * xy[:, :2], np.ones((len(xy), 1)))); b = (xy[:, :2] ** 2).sum(axis=1)
    c = np.linalg.lstsq(A, b, rcond=None)[0]
    center = np.column_stack((u, v, n_unit)) @ np.array([c[0], c[1], 0.0]) + qc
    return center, n_unit, np.sqrt(c[2] + c[0] ** 2 + c[1] ** 2)

def line(centers, normals):
    N = np.mean(normals, axis=0); N /= np.linalg.norm(N)
    ca = np.mean(centers, axis=0)
    proj = [c - ((c - ca) @ N) * N for c in centers]
    return np.mean(proj, axis=0), N

def motion_axis(takes, j):
    axes = []
    for t in takes:
        e = [x for x in t["raw_data"] if x.get("flange_mocap_pose") and x.get("base_mocap_pose")]
        pts = np.array([(np.linalg.inv(T(x["base_mocap_pose"])) @ T(x["flange_mocap_pose"]))[:3, 3] for x in e])
        ang = np.array([x["joint_conf"][j] for x in e]); c = np.array(t["center"]); n = np.array(t["normal"]) / np.linalg.norm(t["normal"])
        votes = sum(np.sign(np.cross(pts[i] - c, pts[i + 1] - c) @ n) * np.sign(ang[i + 1] - ang[i]) for i in range(len(pts) - 1))
        axes.append(n if votes > 0 else -n)
    a = np.mean(axes, axis=0); return a / np.linalg.norm(a)

for date, (digits, arm, urdf_rel) in SESS.items():
    d = ROOT / date
    j0 = json.load(open(d / "j0/j0_analysis.json")); j1 = json.load(open(d / "j1/j1_analysis.json"))
    worst_c = worst_n = 0.0; signs = []
    for data in (j0, j1):
        for t in data["takes"]:
            pts = [(np.linalg.inv(T(x["base_mocap_pose"])) @ T(x["flange_mocap_pose"]))[:3, 3] for x in t["raw_data"] if x.get("flange_mocap_pose") and x.get("base_mocap_pose")]
            c, n, r = circle(pts)
            worst_c = max(worst_c, np.linalg.norm(c - t["center"])); worst_n = max(worst_n, min(np.linalg.norm(n - t["normal"]), np.linalg.norm(n + t["normal"])))
        ns = np.array([t["normal"] for t in data["takes"]]); signs.append(np.sign(ns @ ns[0]).astype(int).tolist())
    print(f"{date} circle refit vs stored: centre {worst_c:.2e} m, normal {worst_n:.2e}; stored normal signs vs first: {signs}")
    p0, v0 = line([t["center"] for t in j0["takes"]], [t["normal"] for t in j0["takes"]])
    p1, v1 = line([t["center"] for t in j1["takes"]], [t["normal"] for t in j1["takes"]])
    m0, m1 = motion_axis(j0["takes"], 0), motion_axis(j1["takes"], 1)
    if v0 @ m0 < 0: v0 = -v0
    want = -m1 if digits == "0806" else m1
    if v1 @ want < 0: v1 = -v1
    k = np.cross(v0, v1); k /= np.linalg.norm(k)
    # plane0 (p0, v1) ∩ plane1 (p1, k): line direction v1 x k; point closest to p1 on it
    dirn = np.cross(v1, k); dirn /= np.linalg.norm(dirn)
    # solve for a point on both planes (least norm offset from p1)
    Amat = np.vstack([v1, k]); bvec = np.array([v1 @ p0, k @ p1])
    x0 = p1 + np.linalg.lstsq(Amat, bvec - Amat @ p1, rcond=None)[0]
    p01 = x0 + ((p1 - x0) @ dirn) * dirn
    urdf_text = (URDF_NEW / urdf_rel).read_text()
    import re
    jn = f"{arm}_shoulder_pan_joint"
    mobj = re.search(r'<joint name="%s" type="revolute">.*?<origin[^>]*xyz="([^"]+)"' % jn, urdf_text, re.S)
    offset = float(mobj.group(1).split()[2])
    z = v0; y = v1 if digits == "0806" else -v1
    x = np.cross(y, z); x /= np.linalg.norm(x); z = np.cross(x, y); z /= np.linalg.norm(z)
    C = np.eye(4); C[:3, 0], C[:3, 1], C[:3, 2], C[:3, 3] = x, y, z, p01 - offset * z
    stored = np.array(json.load(open(d / "base_frame_calibration.json"))[f"base_mocap_from_{arm}_base_link_inertia"])
    print(f"   base offset {offset}; arm base frame vs stored: pos {np.linalg.norm(C[:3,3]-stored[:3,3]):.2e} m, rot {np.abs(C[:3,:3]-stored[:3,:3]).max():.2e}")
    out = json.load(open(d / f"calibrated_transformation_{digits}_rhino.json"))
    for name, root in (("new 4d0c854", URDF_NEW), ("old aa6a261", URDF_OLD)):
        A = fixed_transform(root / urdf_rel, "base_footprint", f"{arm}_base_link_inertia")
        Am = A.matrix() if callable(A.matrix) else A.matrix
        F = stored @ np.linalg.inv(Am)
        Fs = T(out["base_mocap_from_base_footprint"])
        ang = np.degrees(Rot.from_matrix(Fs[:3, :3].T @ F[:3, :3]).magnitude())
        print(f"   footprint (stored arm base · A⁻¹, {name}) vs stored file: pos {np.linalg.norm(F[:3,3]-Fs[:3,3]):.2e} m, angle {ang:.2e} deg")
