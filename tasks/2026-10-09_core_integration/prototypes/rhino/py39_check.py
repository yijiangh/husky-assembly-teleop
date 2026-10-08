"""Run the pure core under Python 3.9 with Rhino's pinned numpy/scipy: read, validate, plan-check, scenes, FK."""
import sys, time
from pathlib import Path
sys.path.insert(0, "/home/jakob/ra/workspace_design_core/src/husky-assembly-teleop")
SCR = Path(sys.argv[1])
t = time.perf_counter()
from bar_assembly_core import design as D
from bar_assembly_core.design import plan_check, relations, hold, solutions, scenes
from bar_assembly_core import kinematics
print("import", round(time.perf_counter() - t, 2), "s; python", sys.version.split()[0])
assert "compas" not in sys.modules and "husky_assembly_teleop" not in sys.modules
for name in ("260814_RobArch_support_ik", "260920_RobArch_demo_revamp_backup"):
    folder = SCR / "converted" / name
    t = time.perf_counter(); design = D.read(folder); t_read = time.perf_counter() - t
    t = time.perf_counter(); report = plan_check.check_plan(design); t_check = time.perf_counter() - t
    steps = list(design.movements())
    t = time.perf_counter(); sc = [design.scene_at(m) for _, m in steps]; t_scene = (time.perf_counter() - t) / len(steps)
    holds = [a for a in design.schedule if design.actions[a].type == "bar_holding"]
    hs = hold.hold_scene_for(design, holds[0]) if holds else None
    bar = design.actions[design.schedule[0]].bar
    after = design.scene_after(bar)
    robot = design.robots["robots/cindy"]
    s = sc[5]; r = s.robots["robots/cindy"]
    t = time.perf_counter()
    from bar_assembly_core.urdf import urdf_links
    links = sorted(urdf_links(robot.urdf))
    poses = [kinematics.link_pose(robot.urdf, r.base, r.joints, l) for l in links]
    t_fk = time.perf_counter() - t
    h = D.content_hash(folder / "design.json")
    print(f"{name}: read {t_read:.2f}s, check_plan {t_check:.2f}s ({len(report.errors)} errors, {len(report.warnings)} warnings), "
          f"scene_at {t_scene*1000:.1f} ms/movement over {len(steps)}, FK all {len(links)} links {t_fk*1000:.1f} ms, "
          f"hold scene of {holds[0] if holds else None}: robots enabled {[k for k,v in hs.robots.items() if v.enabled] if hs else None}, hash {h[:12]}")
    for e in report.errors[:6]: print("   E", e[:160])
    ws = {}
    for w in report.warnings: ws.setdefault(w[:3], []).append(w)
    for k, v in ws.items(): print("   W", k, len(v), "e.g.", v[0][:150])
