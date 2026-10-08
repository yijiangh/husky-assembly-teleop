"""Under Python 3.9 with Rhino's pins: import the mirrors, build a CompasFabMirror for Cindy, sync two scenes, lend."""
import sys, time
from dataclasses import replace
from pathlib import Path
sys.path.insert(0, "/home/jakob/ra/workspace_design_core/src/husky-assembly-teleop")
sys.path.insert(0, "/home/jakob/ra/workspace_design_core/src/husky-assembly-teleop/external/compas_fab/src")
import compas, compas_robots, pybullet_planning
print("python", sys.version.split()[0], "compas", compas.__version__, "pybullet_planning", getattr(pybullet_planning, "__version__", "?"))
from bar_assembly_core.design import read
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror
import compas_fab; print("compas_fab", compas_fab.__file__)
design = read(Path(sys.argv[1]) / "converted" / "260814_RobArch_support_ik")
insert = [m for a, m in design.movements() if a.bar == "bars/B10" and m.drives][0]
scene = design.scene_at(insert)
mirror = CompasFabMirror("robots/cindy", log=print)
t = time.perf_counter(); mirror.sync(scene); print(f"first sync {time.perf_counter()-t:.1f}s")
print("collisions at insert start:", mirror.collisions()[:6])
retreat = [m for a, m in design.movements() if a.bar == "bars/B10" and m.path == "linear" and not m.coupled][0]
t = time.perf_counter(); mirror.sync(design.scene_at(retreat)); print(f"second sync (state only) {time.perf_counter()-t:.2f}s")
print("collisions at retreat start:", mirror.collisions()[:6])
with mirror.lend() as planner:
    print("lent planner", type(planner).__name__, "tools", sorted(planner.client.robot_cell.tool_models)[:4],
          "bodies", sorted(planner.client.robot_cell.rigid_body_models)[:3])
# A state for a scene whose acting robot has null joints: the mirror refuses it.
load = [m for a, m in design.movements() if a.bar == "bars/B10"][0]
try:
    mirror.sync(design.scene_at(load))
except ValueError as e:
    print("refused:", str(e)[:120])
