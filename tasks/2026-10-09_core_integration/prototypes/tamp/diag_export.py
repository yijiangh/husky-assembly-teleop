"""PRM-style export: can the mirror's compas_fab cell + state go to another process as JSON (compas json_dump)?
And does pybullet_planning's LockRenderer work inside lend()?"""
import sys, time, os
import pybullet_planning as pp
from compas.data import json_dumps, json_loads
from compas_fab.backends import PyBulletClient, PyBulletPlanner, CollisionCheckError
from bar_assembly_core.design import read
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror
d = read(sys.argv[1]); m = [mv for a, mv in d.movements() if mv.id == "B10_R_M3_free_home"][0]
mirror = CompasFabMirror("robots/cindy"); mirror.sync(d.scene_at(m))
with mirror.lend():
    try:
        with pp.LockRenderer():
            pass
        print("LockRenderer inside lend(): OK")
    except Exception as exc:
        print(f"LockRenderer inside lend(): {type(exc).__name__}: {exc}")
t = time.time()
cell_json, state_json = json_dumps(mirror.cell), json_dumps(mirror.state)
print(f"json_dumps cell {len(cell_json)/1e6:.1f} MB, state {len(state_json)/1e3:.0f} kB in {time.time()-t:.1f}s")
t = time.time()
cell, state = json_loads(cell_json), json_loads(state_json)
client = PyBulletClient("direct", verbose=False); client.__enter__()
planner = PyBulletPlanner(client); planner.set_robot_cell(cell); planner.set_robot_cell_state(state)
print(f"rebuilt in a fresh client in {time.time()-t:.1f}s")
def check(pl, st):
    try:
        pl.check_collision(st, {"full_report": True}); return []
    except CollisionCheckError as e:
        return sorted((a.name, b.name) for a, b in e.collision_pairs)
print("mirror collisions:", mirror.collisions(full_report=True), "| rebuilt cell collisions:", check(planner, state))
