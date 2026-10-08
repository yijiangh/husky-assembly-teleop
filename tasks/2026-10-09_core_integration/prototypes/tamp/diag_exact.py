"""Is a Cindy tool really clear of the bar it carries? Exact distances in a bare PyBullet world.

At the start of each listed movement: tool AT3L/AT3R mesh at its flange, bar at its world pose. The bar is a static
concave trimesh (exact). The tool is tried three ways: one convex hull (what compas_fab builds), VHACD convex parts,
and the exact triangle mesh (concave). Also: trimesh vertex test (bar vertices inside the tool hull).
"""
import sys
import tempfile
from pathlib import Path

import numpy as np
import pybullet as p
import trimesh

from bar_assembly_core.design import read
from bar_assembly_core.geometry import shape_mesh
from bar_assembly_core.kinematics import link_pose

design = read(sys.argv[1])
movements = {m.id: m for _, m in design.movements()}
CINDY = "robots/cindy"
tmp = Path(tempfile.mkdtemp())


def obj(path, mesh):
    trimesh.Trimesh(mesh.vertices, mesh.faces, process=False).export(path)
    return str(path)


def vhacd_parts(src):
    out = str(src) + ".vhacd.obj"
    p.vhacd(src, out, str(tmp / "log.txt"), resolution=1000000, maxNumVerticesPerCH=64)
    vertices, objects = [], []
    for line in Path(out).read_text().splitlines():
        if line.startswith("o "):
            objects.append([])
        elif line.startswith("v "):
            vertices.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            objects[-1].append([int(x.split("/")[0]) - 1 for x in line.split()[1:4]])
    paths = []
    for i, faces in enumerate(objects):
        used = sorted({k for f in faces for k in f})
        index = {old: new for new, old in enumerate(used)}
        part = trimesh.Trimesh([vertices[k] for k in used], [[index[k] for k in f] for f in faces], process=False)
        path = str(src) + f".part{i}.obj"
        part.export(path)
        paths.append(path)
    return paths


p.connect(p.DIRECT)
for movement_id in sys.argv[2:]:
    m = movements[movement_id]
    scene = design.scene_at(m)
    robot = scene.robots[CINDY]
    bar_id = [a for a in m.start.attached] or [b for b in ("bars/" + movement_id.split("_")[0],)]
    bar_id = bar_id[0]
    bar_pose = scene.world_poses[bar_id]
    bar_mesh = shape_mesh(design.bodies[bar_id].geometry.collision[0])
    bar_path = obj(tmp / "bar.obj", bar_mesh)
    bar_col = p.createCollisionShape(p.GEOM_MESH, fileName=bar_path, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
    bar = p.createMultiBody(0, bar_col, basePosition=bar_pose.position, baseOrientation=bar_pose.orientation)
    for flange, tool_id in sorted(design.robots[CINDY].tools.items()):
        tool = design.tools[tool_id]
        fl = link_pose(robot.model.urdf, robot.base, robot.joints, flange)
        mesh = shape_mesh(tool.geometry.collision[0])
        src = obj(tmp / f"{tool_id.split('/')[-1]}.obj", mesh)
        out = {}
        hull = p.createCollisionShape(p.GEOM_MESH, fileName=src)
        exact = p.createCollisionShape(p.GEOM_MESH, fileName=src, flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
        parts = vhacd_parts(src)
        compound = p.createCollisionShapeArray([p.GEOM_MESH] * len(parts), fileNames=parts, meshScales=[[1, 1, 1]] * len(parts))
        for label, shape in (("hull", hull), (f"vhacd x{len(parts)}", compound), ("exact", exact)):
            body = p.createMultiBody(0, shape, basePosition=fl.position, baseOrientation=fl.orientation)
            pts = p.getClosestPoints(body, bar, 0.05)
            out[label] = min((c[8] for c in pts), default=None)
            p.removeBody(body)
        print(f"{movement_id:30s} {tool_id:12s} vs {bar_id}: signed distance (m) "
              + ", ".join(f"{k} {v:+.4f}" if v is not None else f"{k} >0.05" for k, v in out.items()))
    p.removeBody(bar)
