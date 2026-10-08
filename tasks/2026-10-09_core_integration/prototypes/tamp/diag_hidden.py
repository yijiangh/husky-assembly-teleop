"""Where do hidden (absent) bodies sit in the mirror's PyBullet world, and does tamp's obstacle list include them?"""
import sys
import numpy as np
import pybullet_planning as pp
from bar_assembly_core.design import read
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror
from husky_assembly_tamp.motion_planner import api
d = read(sys.argv[1]); m = [mv for a, mv in d.movements() if mv.id == "B10_J_M4_tool_tighten_joint"][0]
mirror = CompasFabMirror("robots/cindy"); mirror.sync(d.scene_at(m))
hidden = {k for k, s in mirror.state.rigid_body_states.items() if s.is_hidden}
with mirror.lend() as planner:
    obstacles = api._collect_obstacle_puids(planner, exclude={"bars/B10"})
    names = {p: k for k, ps in planner.client.rigid_bodies_puids.items() for p in ps}
    hid = [p for p in obstacles if names[p] in hidden]
    poses = np.array([pp.get_pose(p)[0] for p in hid])
    base = np.array(mirror.state.robot_base_frame.point)
    print(f"tamp obstacle list: {len(obstacles)} PyBullet bodies, {len(hid)} of them hidden (absent) bodies")
    print("hidden bodies sit at (first 3):", poses[:3].round(3).tolist(), "robot base at", base.round(3).tolist())
    print("hidden bodies within 2 m of the robot base:", int((np.linalg.norm(poses[:, :2] - base[:2], axis=1) < 2.0).sum()))
mirror.close()
