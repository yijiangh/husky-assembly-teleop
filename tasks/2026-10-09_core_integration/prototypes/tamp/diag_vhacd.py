"""Do the bar<->tool contacts go away when tools are loaded as several convex parts instead of one hull?

Monkeypatches bar_assembly_core.mirrors.compas_fab.tool_model (scratchpad only): each tool's collision mesh is split
by PyBullet's VHACD into convex parts, and the ToolModel gets one collision mesh per part.
"""
import sys
import tempfile
from pathlib import Path

import numpy as np
import pybullet as p
import trimesh
from compas.datastructures import Mesh
from compas.geometry import Transformation

import bar_assembly_core.mirrors.compas_fab as cfm
from bar_assembly_core.design import read
from bar_assembly_core.geometry import shape_mesh
from bar_assembly_core.mirrors.compas import frame_from_pose, joined_mesh
from compas_robots import ToolModel

CACHE = {}


def obj_objects(path):
    """Each `o` object of an OBJ file as (vertices, faces), faces re-indexed per object."""
    vertices, objects = [], []
    for line in Path(path).read_text().splitlines():
        if line.startswith("o "):
            objects.append([])
        elif line.startswith("v "):
            vertices.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("f "):
            objects[-1].append([int(x.split("/")[0]) - 1 for x in line.split()[1:4]])
    out = []
    for faces in objects:
        used = sorted({i for face in faces for i in face})
        index = {old: new for new, old in enumerate(used)}
        out.append(([vertices[i] for i in used], [[index[i] for i in face] for face in faces]))
    return out


def convex_parts(tool):
    """VHACD parts of a tool's collision shapes, as compas meshes."""
    meshes = [shape_mesh(shape) for shape in tool.geometry.collision]
    key = id(tool.geometry)
    if key in CACHE:
        return CACHE[key]
    parts = []
    with tempfile.TemporaryDirectory() as tmp:
        for i, mesh in enumerate(meshes):
            src, out = Path(tmp, f"in{i}.obj"), Path(tmp, f"out{i}.obj")
            trimesh.Trimesh(mesh.vertices, mesh.faces).export(src)
            cid = p.connect(p.DIRECT)
            p.vhacd(str(src), str(out), str(Path(tmp, "log.txt")), resolution=1000000, maxNumVerticesPerCH=64, physicsClientId=cid)
            p.disconnect(cid)
            parts += [Mesh.from_vertices_and_faces(v, f) for v, f in obj_objects(out)]
    CACHE[key] = parts
    return parts


def tool_model_parts(tool, name):
    model = ToolModel(None, frame_from_pose(tool.tcp), name=name)
    model.add_link("attached_tool_link", visual_meshes=[joined_mesh(tool.geometry.visual or tool.geometry.collision)],
                   collision_meshes=convex_parts(tool))
    model._rebuild_tree()
    model._create(model.root, Transformation())
    return model


design = read(sys.argv[1])
movements = {m.id: m for _, m in design.movements()}
checks = ["B10_J_M4_tool_tighten_joint", "B10_R_M2_LM_retreat", "B19_J_M4_tool_tighten_joint",
          "B21_J_M4_tool_tighten_joint", "B19_R_M2_LM_retreat"]
for label, factory in (("one hull per tool (core today)", cfm.tool_model), ("VHACD parts", tool_model_parts)):
    cfm.tool_model = factory
    mirror = cfm.CompasFabMirror("robots/cindy")
    for movement_id in checks:
        if movement_id not in movements:
            continue
        mirror.sync(design.scene_at(movements[movement_id]))
        print(f"{label:32s} {movement_id:32s} {mirror.collisions(full_report=True)}")
    if factory is tool_model_parts:
        print("parts per tool:", {k: len(v) for k, v in CACHE.items()})
    mirror.close()
