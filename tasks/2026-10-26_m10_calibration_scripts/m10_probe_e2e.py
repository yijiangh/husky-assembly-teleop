"""Scratch check: full chain raw records -> K and C, against the stored session outputs."""
import json, os, sys, re
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as Rot
sys.path.insert(0, os.path.dirname(__file__))
import importlib.util
spec = importlib.util.spec_from_file_location("probe", os.path.join(os.path.dirname(__file__), "m10_probe_port.py"))
from husky_assembly_teleop.robot_interface.frames import fixed_transform
src = open(os.path.join(os.path.dirname(__file__), "m10_probe_port.py")).read().split("for date, (digits")[0]
exec(src)
for date, (digits, arm, urdf_rel) in SESS.items():
    d = ROOT / date
    batches = {}
    for b in ("j0", "j1"):
        data = json.load(open(d / b / f"{b}_analysis.json"))
        cs, ns, takes = [], [], []
        for t in data["takes"]:
            pts = [(np.linalg.inv(T(x["base_mocap_pose"])) @ T(x["flange_mocap_pose"]))[:3, 3] for x in t["raw_data"] if x.get("flange_mocap_pose") and x.get("base_mocap_pose")]
            c, n, r = circle(pts); cs.append(c); ns.append(n); takes.append({**t, "center": list(c), "normal": list(n)})
        ns = [n if n @ ns[0] > 0 else -n for n in ns]
        batches[b] = (cs, ns, takes)
    p0, v0 = line(*batches["j0"][:2]); p1, v1 = line(*batches["j1"][:2])
    a0, a1 = motion_axis(batches["j0"][2], 0), motion_axis(batches["j1"][2], 1)
    v0 = v0 if v0 @ a0 > 0 else -v0
    y = -a1 / np.linalg.norm(a1); y = (v1 if v1 @ y > 0 else -v1)
    k = np.cross(v0, y); k /= np.linalg.norm(k); dirn = np.cross(y, k); dirn /= np.linalg.norm(dirn)
    Amat = np.vstack([y, k]); x0 = p1 + np.linalg.lstsq(Amat, np.array([y @ p0, k @ p1]) - Amat @ p1, rcond=None)[0]
    p01 = x0 + ((p1 - x0) @ dirn) * dirn
    text = (URDF_NEW / urdf_rel).read_text()
    offset = float(re.search(r'<joint name="%s_shoulder_pan_joint" type="revolute">.*?<origin[^>]*xyz="([^"]+)"' % arm, text, re.S).group(1).split()[2])
    z = v0; x = np.cross(y, z); x /= np.linalg.norm(x); z = np.cross(x, y); z /= np.linalg.norm(z)
    K = np.eye(4); K[:3, 0], K[:3, 1], K[:3, 2], K[:3, 3] = x, y, z, p01 - offset * z
    stored = np.array(json.load(open(d / "base_frame_calibration.json"))[f"base_mocap_from_{arm}_base_link_inertia"])
    A = fixed_transform(URDF_NEW / urdf_rel, "base_footprint", f"{arm}_base_link_inertia").matrix()
    C = K @ np.linalg.inv(A); Cs = T(json.load(open(d / f"calibrated_transformation_{digits}_rhino.json"))["base_mocap_from_base_footprint"])
    ang = lambda M1, M2: np.degrees(Rot.from_matrix(M1[:3, :3].T @ M2[:3, :3]).magnitude())
    print(f"{date}: K re-fit vs stored: {np.linalg.norm(K[:3,3]-stored[:3,3])*1e6:.3f} um, {ang(K, stored):.2e} deg | C vs stored file: {np.linalg.norm(C[:3,3]-Cs[:3,3])*1e6:.3f} um, {ang(C, Cs):.2e} deg")
