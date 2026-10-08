"""Collisions the core's derived contacts report at the keyframe states Rhino's IK checks (insert start = approach,
insert end = assembled, retreat start, retreat end), Cindy's mirror, on both converted designs."""
import sys, collections
from dataclasses import replace
from pathlib import Path
sys.path.insert(0, "/home/jakob/ra/workspace_design_core/src/husky-assembly-teleop")
sys.path.insert(0, "/home/jakob/ra/workspace_design_core/src/husky-assembly-teleop/external/compas_fab/src")
from bar_assembly_core.design import read
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror
for name in ("260814_RobArch_support_ik", "260920_RobArch_demo_revamp_backup"):
    design = read(Path(sys.argv[1]) / "converted" / name)
    mirror = CompasFabMirror("robots/cindy")
    kinds = collections.Counter(); bars_hit = collections.defaultdict(set); checked = 0; examples = collections.defaultdict(list)
    for action, m in design.movements():
        if action.robot != "robots/cindy" or not m.arms or m.path != "linear":
            continue
        scene = design.scene_at(m)
        if scene.robots["robots/cindy"].acting_problems():
            continue
        mirror.sync(scene)
        for when, joints in (("start", None), ("end", (m.target.joints or {}).get("robots/cindy") if m.target else None)):
            if when == "end" and not joints:
                continue
            checked += 1
            for a, b in mirror.collisions(joints):
                key = tuple(sorted((a.split("/")[0] + ("/" + a.split("/")[1] if a.startswith("tools") else ""),
                                    b.split("/")[0])))
                kinds[key] += 1; bars_hit[key].add(action.bar)
                if 'AT3L' not in a+b and len(examples[key]) < 3: examples[key].append(f'{m.id} {when}: {a} x {b}')
    print(name, f"{checked} keyframe states checked")
    for k, v in kinds.most_common():
        print("  ", k, v, "bars:", len(bars_hit[k]), examples[k])
