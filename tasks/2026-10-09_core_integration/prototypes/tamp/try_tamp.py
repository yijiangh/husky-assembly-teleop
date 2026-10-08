"""
Feed tamp from a schema 2 design through the core: read -> scene_at -> CompasFabMirror -> lend() -> tamp.

Run in the teleop venv:
    PYTHONPATH=<teleop root>[:<tamp worktree>] python try_tamp.py <design folder> [--tamp-label main]

Steps (each prints OK / FAIL with the reason):
  1. read + plan_check
  2. B10_J_M4 (insert): scene_at, mirror.sync, full collision report at the authored start
  3. same, after allowing bar<->tool contacts (what the old export allowed)
  4. tamp names/groups seen through the mirror's cell (resolve_arm_groups, _arm_joint_names, tool_models)
  5. tamp motion API on the mirror's planner via lend():
       J_M4 insert   -> plan_constrained_dual_arm_linear(goal_conf = target joints)
       R_M2 retreat  -> plan_dual_arm_linear_independent(goal_conf = target joints)
       R_M3 home     -> plan_free_dual_arm(goal_conf = target joints, max_time short)
       J_M3 transfer -> plan_constrained_dual_arm(derive_start=True) (start joints are null in the design)
  6. keyframe IK (gradient backend): approach (J_M3 target), assembled (J_M4 target), retreat (R_M2 target)
  7. write solutions/B10_J_joint.json and B10_R_release.json with the core's writer, read them back, chain check
  8. ssik artifact identity: baked FK of asset/ssik/<arm>_ik.py vs the design URDF's FK
"""

from __future__ import annotations

import argparse
import ast
import dataclasses
import os
import sys
import time
import traceback
from pathlib import Path

import numpy as np

T0 = time.time()
os.environ.setdefault("HUSKY_IK_BACKEND", "gradient")  # ssik is not installed in the teleop venv


def say(tag, msg):
    print(f"[{time.time() - T0:6.1f}s] {tag:5s} {msg}", flush=True)


parser = argparse.ArgumentParser()
parser.add_argument("design")
parser.add_argument("--tamp-label", default="installed")
parser.add_argument("--free-time", type=float, default=5.0)
parser.add_argument("--cdfm-time", type=float, default=20.0)
parser.add_argument("--no-patch", action="store_true", help="J_M3 with derived contacts only")
parser.add_argument("--skip", default="", help="comma list of steps to skip")
args = parser.parse_args()
skip = set(filter(None, args.skip.split(",")))

import husky_assembly_tamp  # noqa: E402

say("INFO", f"tamp from {Path(husky_assembly_tamp.__file__).parent.parent} ({args.tamp_label})")

from bar_assembly_core.design import read  # noqa: E402
from bar_assembly_core.design.plan_check import check_plan  # noqa: E402
from bar_assembly_core.design.solutions import (MovementResult, Planner, Solution, Trajectory,  # noqa: E402
                                                read_solutions, solution_warnings, solved_against, write_solution)
from bar_assembly_core.design.carry import assumed_start_all  # noqa: E402
from bar_assembly_core.mirrors.compas_fab import CompasFabMirror  # noqa: E402
from bar_assembly_core.mirrors.compas import frame_from_pose  # noqa: E402
from bar_assembly_core.kinematics import link_pose  # noqa: E402

# --- 1. read
design = read(args.design)
say("OK", f"read {design.folder.name}: {len(design.actions)} actions, {sum(1 for _ in design.movements())} movements")
try:
    report = check_plan(design, geometry=False)
    say("INFO", f"plan_check (no geometry): {len(report.errors)} errors, {len(report.warnings)} warnings")
except Exception as exc:  # noqa: BLE001
    say("FAIL", f"plan_check: {exc!r}")

CINDY = "robots/cindy"
J, R = design.actions["B10_J_joint"], design.actions["B10_R_release"]
mv = {m.id: m for m in (*J.movements, *R.movements)}
M3, M4 = mv["B10_J_M3_CDFM_transfer_to_approach"], mv["B10_J_M4_tool_tighten_joint"]
R2, R3 = mv["B10_R_M2_LM_retreat"], mv["B10_R_M3_free_home"]
for m in (M3, M4, R2, R3):
    rs = m.start.robots[CINDY]
    say("INFO", f"{m.id}: arms={len(m.arms)} path={m.path} coupled={m.coupled} controller={m.controller} "
                f"ends_on={m.ends_on} drives={bool(m.drives)} line={bool(m.line)} "
                f"start.joints={'null' if rs.joints is None else len(rs.joints)} "
                f"target.joints={'-' if m.target is None else len(m.target.joints.get(CINDY, {}))} "
                f"target.links={'-' if m.target is None else len(m.target.links)}")

mirror = CompasFabMirror(CINDY, log=lambda line: say("INFO", line))


def sync(scene):
    t = time.time()
    mirror.sync(scene)
    return time.time() - t


def joints12(cell):
    from husky_assembly_tamp.motion_planner.api import _arm_joint_names
    left, right = _arm_joint_names(cell)
    return left + right


# --- 2. insert start: sync and full collision report
scene_m4 = design.scene_at(M4)
dt = sync(scene_m4)
say("OK", f"mirror.sync(scene_at({M4.id})) in {dt:.1f}s; cell tools={sorted(mirror.cell.tool_models)}; "
          f"{len(mirror.cell.rigid_body_models)} rigid bodies, e.g. {sorted(mirror.cell.rigid_body_models)[:3]}")
hidden = sorted(k for k, s in mirror.state.rigid_body_states.items() if s.is_hidden)
bar_state = mirror.state.rigid_body_states["bars/B10"]
say("INFO", f"bars/B10 attached_to_link={bar_state.attached_to_link} touch_bodies={bar_state.touch_bodies} "
            f"touch_links={bar_state.touch_links}; hidden bodies: {len(hidden)}")
half = mirror.state.rigid_body_states["joints/J3-10_male"]
say("INFO", f"joints/J3-10_male attached_to_link={half.attached_to_link} touch_bodies={half.touch_bodies}")
for key, ts in mirror.state.tool_states.items():
    say("INFO", f"tool {key}: group={ts.attached_to_group} touch_links={ts.touch_links} hidden={ts.is_hidden}")
pairs = mirror.collisions(full_report=True)
say("INFO" if pairs else "OK", f"collisions at the authored insert start ({len(pairs)} pairs): {pairs}")

# --- 3. what if the bar may touch the tools that hold it (old export: touch_bodies ['AT3L', 'AT3R'])
patched = scene_m4.copy()
body = patched.bodies["bars/B10"]
body.touches = tuple(sorted(set(body.touches) | {"tools/AT3L", "tools/AT3R"}))
sync(patched)
pairs2 = mirror.collisions(full_report=True)
say("INFO", f"collisions with bar<->own tools allowed ({len(pairs2)} pairs): {pairs2}")

# --- 4. tamp's view of the mirror's cell
try:
    from husky_assembly_tamp.keyframe.dual_arm_ik import resolve_arm_groups, _require_loaded_cell
    from husky_assembly_tamp.motion_planner.api import LEFT_GROUP, RIGHT_GROUP
    with mirror.lend() as planner:
        _require_loaded_cell(planner)
        groups = resolve_arm_groups(planner.client.robot_cell)
        names = joints12(planner.client.robot_cell)
    say("OK", f"tamp sees groups {groups} (api hardcodes {LEFT_GROUP}, {RIGHT_GROUP}); mirror groups "
              f"{mirror._groups}; 12 joints {names[0]} .. {names[-1]}")
except Exception as exc:  # noqa: BLE001
    say("FAIL", f"tamp cell checks: {exc!r}")
    traceback.print_exc()

results = {}  # movement id -> MovementResult
JN = None


def conf_of(joints, names):
    return [float(joints[n]) for n in names]


def run_api(label, scene, call):
    """Sync a scene, lend the planner, run one tamp call; returns (result, seconds)."""
    try:
        sync(scene)
    except Exception as exc:  # noqa: BLE001
        say("FAIL", f"{label}: mirror.sync refused: {exc}")
        return None, 0.0
    t = time.time()
    try:
        with mirror.lend() as planner:
            out = call(planner)
        return out, time.time() - t
    except Exception as exc:  # noqa: BLE001
        say("FAIL", f"{label}: {type(exc).__name__}: {exc}")
        traceback.print_exc()
        return None, time.time() - t


def path_rows(path, names):
    return tuple(tuple(float(v) for v in q) for q in path)


def jt_rows(jt, names):
    out = []
    for point in jt.points:
        values = dict(zip(point.joint_names, point.joint_values))
        out.append(tuple(float(values[n]) for n in names))
    return tuple(out)


# --- 5. motion API
from husky_assembly_tamp.motion_planner import api  # noqa: E402

JN = joints12(mirror.cell)
base = M4.start.robots[CINDY].base

if "m4" not in skip:
    goal = conf_of(M4.target.joints[CINDY], JN)
    # with the bar<->tool contacts allowed (step 3), otherwise the start is in collision for any check
    jt, dt = run_api("J_M4 insert", patched, lambda pl: api.plan_constrained_dual_arm_linear(
        pl, mirror.state, active_bar_id="bars/B10", goal_conf=goal))
    if jt is not None:
        rows = jt_rows(jt, JN)
        err = max(abs(a - b) for a, b in zip(rows[-1], goal))
        say("OK", f"J_M4 insert: plan_constrained_dual_arm_linear -> {len(rows)} waypoints in {dt:.1f}s; "
                  f"end vs target joints max |d| = {err:.2e} rad")
        results[M4.id] = MovementResult("solved", start={CINDY: dict(zip(JN, rows[0]))},
                                        end={CINDY: dict(zip(JN, rows[-1]))},
                                        trajectory=Trajectory(CINDY, tuple(JN), rows))
    else:
        say("FAIL", f"J_M4 insert: no trajectory ({dt:.1f}s)")
        results[M4.id] = MovementResult("failed", reason="plan_constrained_dual_arm_linear returned None")

    # unpatched: the derived contacts only
    jt0, dt0 = run_api("J_M4 insert (derived contacts only)", scene_m4, lambda pl: api.plan_constrained_dual_arm_linear(
        pl, mirror.state, active_bar_id="bars/B10", goal_conf=goal))
    say("INFO", f"J_M4 with derived contacts only: {'solved' if jt0 is not None else 'failed'} "
                f"(linear planners skip CC3-5 by default, so the hull contact is not seen)")

if "r2" not in skip:
    scene = design.scene_at(R2)
    goal = conf_of(R2.target.joints[CINDY], JN)
    jt, dt = run_api("R_M2 retreat", scene, lambda pl: api.plan_dual_arm_linear_independent(
        pl, mirror.state, goal_conf=goal))
    pairs_r2 = mirror.collisions(full_report=True)
    say("INFO", f"collisions at R_M2 start (ungrasped, tools still on the halves): {pairs_r2}")
    if jt is not None:
        rows = jt_rows(jt, JN)
        say("OK", f"R_M2 retreat: plan_dual_arm_linear_independent -> {len(rows)} waypoints in {dt:.1f}s")
        results[R2.id] = MovementResult("solved", start={CINDY: dict(zip(JN, rows[0]))},
                                        end={CINDY: dict(zip(JN, rows[-1]))},
                                        trajectory=Trajectory(CINDY, tuple(JN), rows))
    else:
        results[R2.id] = MovementResult("failed", reason="plan_dual_arm_linear_independent returned None")

if "r3" not in skip:
    scene = design.scene_at(R3)
    goal = conf_of(R3.target.joints[CINDY], JN)
    out, dt = run_api("R_M3 home", scene, lambda pl: api.plan_free_dual_arm(
        pl, mirror.state, goal, max_time=args.free_time, max_iterations=200))
    if out is not None:
        path, info = out
        if path is not None:
            rows = path_rows(path, JN)
            say("OK", f"R_M3 home: plan_free_dual_arm -> {len(rows)} waypoints in {dt:.1f}s")
            results[R3.id] = MovementResult("solved", start={CINDY: dict(zip(JN, rows[0]))},
                                            end={CINDY: dict(zip(JN, rows[-1]))},
                                            trajectory=Trajectory(CINDY, tuple(JN), rows))
        else:
            say("FAIL", f"R_M3 home: {info.get('failure_reason')} in {dt:.1f}s")
            results[R3.id] = MovementResult("failed", reason=str(info.get("failure_reason")))

if "m3" not in skip:
    # The design leaves J_M3's start joints null (the loading pose is the planner's to choose).
    scene = design.scene_at(M3)
    try:
        sync(scene)
        say("FAIL", "J_M3: mirror accepted null start joints (unexpected)")
    except ValueError as exc:
        say("INFO", f"J_M3: mirror.sync refuses the design's null start joints: {exc}")
    # Fill with the core's assumed joints (display/seed only) and let tamp derive the start.
    assumed, source = assumed_start_all(design)[(J.id, M3.id)]
    say("INFO", f"J_M3 assumed start joints from {source!r}: {None if assumed is None else len(assumed)} joints")
    seeded = scene.copy()
    robot = seeded.robots[CINDY]
    robot.joints = dict(assumed or M3.target.joints[CINDY])
    robot.unmeasured = frozenset()
    if not args.no_patch:
        seeded.bodies["bars/B10"].touches = tuple(sorted(set(seeded.bodies["bars/B10"].touches)
                                                         | {"tools/AT3L", "tools/AT3R"}))
    goal = conf_of(M3.target.joints[CINDY], JN)
    out, dt = run_api("J_M3 transfer", seeded, lambda pl: api.plan_constrained_dual_arm(
        pl, mirror.state, active_bar_id="bars/B10", goal_conf=goal, derive_start=True,
        max_time=args.cdfm_time, max_attempts=1, random_seed=0))
    if out is not None:
        path, info = out
        if path is not None:
            rows = path_rows(path, JN)
            say("OK", f"J_M3 transfer: plan_constrained_dual_arm(derive_start) -> {len(rows)} waypoints in {dt:.1f}s; "
                      f"path_poses={len(info.get('path_poses') or [])}")
            results[M3.id] = MovementResult("solved", start={CINDY: dict(zip(JN, rows[0]))}, start_overridden=False,
                                            end={CINDY: dict(zip(JN, rows[-1]))},
                                            trajectory=Trajectory(CINDY, tuple(JN), rows))
        else:
            say("FAIL", f"J_M3 transfer: {info.get('failure_reason')} in {dt:.1f}s")
            results[M3.id] = MovementResult("failed", reason=str(info.get("failure_reason")))

# --- 6. keyframe IK: approach, assembled, retreat, at the design's base
if "kf" not in skip:
    os.environ.setdefault("HUSKY_IK_BACKEND", "gradient")
    try:
        from types import SimpleNamespace
        from husky_assembly_tamp.keyframe import config as kf_config
        from husky_assembly_tamp.keyframe.ik_keyframe import solve_keyframe_chain
        say("INFO", f"keyframe IK backend: {kf_config.IK_BACKEND}")
        LEFT, RIGHT = f"{CINDY}/left_ur_arm_tool0", f"{CINDY}/right_ur_arm_tool0"
        chain = []
        for role, movement in (("approach", M3), ("assembled", M4), ("retreat", R2)):
            scene = design.scene_at(movement)
            robot = scene.robots[CINDY]
            if robot.unmeasured:  # the chain re-seeds from the previous solve; the first is a cold solve
                robot.joints, robot.unmeasured = dict(M3.target.joints[CINDY]), frozenset()
                robot.joints = {k: 0.0 for k in robot.joints}
            scene.bodies["bars/B10"].touches = tuple(sorted(set(scene.bodies["bars/B10"].touches)
                                                            | {"tools/AT3L", "tools/AT3R"}))
            sync(scene)
            chain.append((role, SimpleNamespace(
                start_state=mirror.state.copy(),
                target_ee_frames={"left": frame_from_pose(movement.target.links[LEFT]),
                                  "right": frame_from_pose(movement.target.links[RIGHT])})))
        # ! One cell for all three: the mirror's last sync (retreat) is the cell; the states differ only in
        #   attachments/touches, which compas_fab takes from each start_state.
        base_mm = base.matrix()
        base_mm[:3, 3] *= 1000.0
        t = time.time()
        with mirror.lend() as planner:
            solved = solve_keyframe_chain(planner, chain, base_mm, check_collision=True,
                                          home_conf_12=conf_of(R3.target.joints[CINDY], JN))
        dt = time.time() - t
        if solved is None:
            say("FAIL", f"keyframe chain failed in {dt:.1f}s")
        else:
            errs = {}
            for role, movement in (("approach", M3), ("assembled", M4), ("retreat", R2)):
                cfg = solved[role].robot_configuration
                design_j = movement.target.joints[CINDY]
                d = np.array([float(cfg[n]) - design_j[n] for n in JN])
                wrapped = (d + np.pi) % (2 * np.pi) - np.pi
                errs[role] = float(np.abs(d).max())
                errs[role + " (mod 2pi)"] = float(np.abs(wrapped).max())
                errs[role + " #joints off by 2pi"] = int(np.sum(np.abs(np.abs(d) - 2 * np.pi) < 0.05))
            say("OK", f"keyframe chain solved in {dt:.1f}s; max |joint - design target| per keyframe: "
                      + ", ".join(f"{k} {v:.3f}" for k, v in errs.items()))
    except Exception as exc:  # noqa: BLE001
        say("FAIL", f"keyframe IK: {type(exc).__name__}: {exc}")
        traceback.print_exc()

# --- 7. solutions
if "sol" not in skip:
    planner_block = Planner(name="husky_assembly_tamp", repo="husky_assembly_tamp", commit=args.tamp_label,
                            ik_backend="gradient", artifacts="robots/cindy/robot.urdf",
                            settings={"free_max_time": args.free_time})
    # Movements without an arm motion: tamp has nothing to plan; record them as not_planned
    try:
        for action in (J, R):
            movements = {}
            for movement in action.movements:
                movements[movement.id] = results.get(movement.id, MovementResult("not_planned"))
            path = write_solution(design, Solution(action.id, solved_against(design, action.id), planner_block,
                                                   movements))
            say("OK", f"wrote {path.relative_to(design.folder)} ({path.stat().st_size / 1e3:.0f} kB): "
                      + ", ".join(f"{k.split('_', 2)[2]}={v.status}" for k, v in movements.items()))
        solutions = read_solutions(design)
        warnings = solution_warnings(design, solutions)
        say("INFO", f"read back {len(solutions)} solutions; warnings: {warnings}")
    except Exception as exc:  # noqa: BLE001
        say("FAIL", f"solutions: {type(exc).__name__}: {exc}")
        traceback.print_exc()

# --- 8. ssik artifact identity vs the design URDF
if "ssik" not in skip:
    tamp_root = Path(husky_assembly_tamp.__file__).parent.parent
    urdf = design.robots[CINDY].urdf
    for side in ("left", "right"):
        artifact = tamp_root / "asset" / "ssik" / f"{side}_ur_arm_ik.py"
        if not artifact.exists():
            say("INFO", f"ssik: no artifact at {artifact}")
            continue
        consts = {}
        for node in ast.parse(artifact.read_text()).body:
            if isinstance(node, ast.Assign) and getattr(node.targets[0], "id", "") in (
                    "_JOINT_AXES", "_JOINT_T_LEFTS", "_JOINT_T_RIGHTS", "_JOINT_NAMES"):
                consts[node.targets[0].id] = eval(compile(ast.Expression(node.value), str(artifact), "eval"),
                                                  {"np": np})

        def fk(q):
            from scipy.spatial.transform import Rotation
            acc = np.eye(4)
            for axis, left, right, value in zip(consts["_JOINT_AXES"], consts["_JOINT_T_LEFTS"],
                                                consts["_JOINT_T_RIGHTS"], q):
                rot = np.eye(4)
                rot[:3, :3] = Rotation.from_rotvec(np.asarray(axis) * value).as_matrix()
                acc = acc @ left @ rot @ right
            return acc

        rng = np.random.default_rng(0)
        worst_p = worst_r = 0.0
        for _ in range(20):
            q = rng.uniform(-np.pi, np.pi, 6)
            joints = dict(zip(consts["_JOINT_NAMES"], q))
            from bar_assembly_core.geometry import Pose
            b = link_pose(urdf, Pose(), joints, f"{side}_ur_arm_base_link").matrix()
            e = link_pose(urdf, Pose(), joints, f"{side}_ur_arm_tool0").matrix()
            rel = np.linalg.inv(b) @ e
            art = fk(q)
            worst_p = max(worst_p, float(np.abs(rel[:3, 3] - art[:3, 3]).max()))
            worst_r = max(worst_r, float(np.abs(rel[:3, :3] - art[:3, :3]).max()))
        say("OK" if worst_p < 1e-6 else "INFO",
            f"ssik {side} artifact vs design URDF FK over 20 random q: max |dp| = {worst_p:.2e} m, "
            f"max |dR| = {worst_r:.2e}")

mirror.close()
say("DONE", "")
