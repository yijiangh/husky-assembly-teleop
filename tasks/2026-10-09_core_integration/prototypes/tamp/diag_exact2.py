"""Exact clearance between Cindy's tools and the bar they carry (trimesh, no hulls), at the insert start."""
import sys
import numpy as np
import trimesh
from bar_assembly_core.design import read
from bar_assembly_core.geometry import shape_mesh
from bar_assembly_core.kinematics import link_pose
design = read(sys.argv[1]); CINDY = "robots/cindy"
movements = {m.id: m for _, m in design.movements()}
worst = []
for action_id in design.schedule:
    action = design.actions[action_id]
    if action.type != "bar_jointing":
        continue
    m = [mv for mv in action.movements if mv.drives][0]  # the insert
    scene = design.scene_at(m); robot = scene.robots[CINDY]; bar = action.bar
    bm = shape_mesh(design.bodies[bar].geometry.collision[0])
    bar_mesh = trimesh.Trimesh(bm.vertices, bm.faces, process=False).apply_transform(scene.world_poses[bar].matrix())
    pq = trimesh.proximity.ProximityQuery(bar_mesh)
    for flange, tool_id in sorted(design.robots[CINDY].tools.items()):
        tm = shape_mesh(design.tools[tool_id].geometry.collision[0])
        fl = link_pose(robot.model.urdf, robot.base, robot.joints, flange).matrix()
        tool = trimesh.Trimesh(tm.vertices, tm.faces, process=False).apply_transform(fl)
        pts = np.vstack([tool.vertices, trimesh.sample.sample_surface_even(tool, 20000, seed=0)[0]])
        d = np.abs(pq.signed_distance(pts)).min()
        hull_d = np.abs(trimesh.proximity.ProximityQuery(bar_mesh).signed_distance(tool.convex_hull.sample(20000))).min()
        inside = (trimesh.proximity.ProximityQuery(tool.convex_hull).signed_distance(bar_mesh.vertices) > 0).sum()
        worst.append((d, action.id, tool_id, inside))
d = np.array([w[0] for w in worst])
print(f"{len(worst)} tool/bar pairs at the 20 insert starts: exact tool-surface to bar clearance min {d.min()*1000:.2f} mm, "
      f"median {np.median(d)*1000:.2f} mm; bar vertices inside the tool's convex hull: min {min(w[3] for w in worst)}, max {max(w[3] for w in worst)}")
