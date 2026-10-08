"""Why does R_M3 (home, free) fail? Collisions along the straight joint path, derived contacts vs old-style contacts."""
import os
import sys
from collections import Counter

import numpy as np

os.environ.setdefault("HUSKY_IK_BACKEND", "gradient")
from bar_assembly_core.design import read  # noqa: E402
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror  # noqa: E402
from husky_assembly_tamp.motion_planner import api  # noqa: E402

design = read(sys.argv[1])
action_id = sys.argv[2] if len(sys.argv) > 2 else "B10_R_release"
CINDY = "robots/cindy"
R = design.actions[action_id]
R3 = R.movements[-1]
mirror = CompasFabMirror(CINDY)
scene = design.scene_at(R3)
mirror.sync(scene)
left, right = api._arm_joint_names(mirror.cell)
names = left + right
start = np.array([scene.robots[CINDY].joints[n] for n in names])
goal = np.array([R3.target.joints[CINDY][n] for n in names])
print(R3.id, "start->goal max |d| rad:", np.abs(goal - start).max().round(3))
print("tools at start:", {k: v for k, v in R3.start.tools.items()})
hits = Counter()
free = 0
for t in np.linspace(0, 1, 41):
    q = dict(zip(names, start + t * (goal - start)))
    pairs = mirror.collisions(q, full_report=True)
    free += not pairs
    hits.update(pairs)
print(f"straight joint path: {free}/41 samples collision-free; pairs hit (samples): {hits.most_common(8)}")

# Old-style: allow every carried/structure bar to touch Cindy's tools (old export listed them as touches)
patched = scene.copy()
for body in patched.bodies.values():
    if body.id.startswith("bars/"):
        body.touches = tuple(sorted(set(body.touches) | {"tools/AT3L", "tools/AT3R"}))
mirror.sync(patched)
hits2 = Counter()
free2 = 0
for t in np.linspace(0, 1, 41):
    q = dict(zip(names, start + t * (goal - start)))
    pairs = mirror.collisions(q, full_report=True)
    free2 += not pairs
    hits2.update(pairs)
print(f"with bars<->tools allowed: {free2}/41 free; pairs: {hits2.most_common(8)}")
with mirror.lend() as planner:
    path, info = api.plan_free_dual_arm(planner, mirror.state, list(goal), max_time=60, max_iterations=200)
print("plan_free_dual_arm with bars<->tools allowed:", None if path is None else len(path), info.get("failure_reason"))
mirror.close()
