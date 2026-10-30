"""
What the servo part says and records around its loop.

The feedback line, the gate after a grounded insert, the panel's one line per reading, the old monitor's console lines
(what a pass aims at, the plan, the load, how the loop ended), the record's IK entry and the servo file's name: pure
helpers for `servo_part.py` (same thread, same rules).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import TYPE_CHECKING, Mapping

from scaffolding_assembly_core.ids import robot_id

from ...shared.execution.servo import (AFTER_INSERT, BAR_TOL, GROUNDED_END_TOL, MAX_PASSES, TOOL0_TOL, force_vectors,
                                       insert_end_gap)
from ...shared.movement_exec_contract import CANCELLED, DONE, SERVO_TARGET

if TYPE_CHECKING:
    from ...shared.assembly_planning.ik import IkResult
    from ...shared.assembly_run_contract import RunView
    from ...shared.execution.servo import ServoPlan
    from .sequences import Job

#: How a side reads on the panel line and in the prompt ("arm" stays as it is).
SIDE = {"left": "L", "right": "R"}
#: (servo folder, movement id) -> the last record number `servo_file` named, for records the writer has not written yet.
NAMED: dict[tuple[str, str], int] = {}


def say_feedback(job: Job, plan: ServoPlan) -> None:
    """Log what the loop steers on (W:2243-2252); "Servo by bar" ticked but not usable for a grounded bar warns."""
    ctx, settings = job.ctx, job.run.view.settings
    if plan.by_bar:
        ctx.log_info(f"[servo] feedback: the MoCap bar {plan.bar} (converges at {BAR_TOL[0] * 1e3:.1f} mm / "
                     f"{math.degrees(BAR_TOL[1]):.1f} deg).")
        return
    # * Only a grounded bar held in the tools servos by bar; for any other the box is no promise, so no warning.
    if settings.servo_by_bar and plan.held and plan.grounded:
        ctx.log_warn(f"[servo] 'Servo by bar' is ticked, but this run aims tool0: {plan.why_tool0}.")
    if plan.layout is not None and plan.holder is not None:
        ctx.log_info(f"[servo] feedback: tool0 ({TOOL0_TOL * 1e3:.1f} mm); the MoCap bar {plan.bar} is recorded "
                     f"alongside.")
    else:
        why = f"{plan.bar} carries no MoCap markers" if plan.held else "no bar held"
        ctx.log_info(f"[servo] feedback: tool0 ({TOOL0_TOL * 1e3:.1f} mm); no MoCap bar recorded ({why}).")


def insert_end_problem(job: Job, plan: ServoPlan) -> str:
    """After a grounded insert: why the servo may not run, "" if it may (W:5994-6024); warns when the guard is off.

    ! The insert's end is the request's insert plan's last row, nothing else: the authored end joints were solved at
      the design base, so at the live base they stand millimetres off. A servo Exec on its own is refused.
    """
    if plan.kind != AFTER_INSERT:
        return ""
    request, ctx = job.request, job.ctx
    route = request.route
    if route is None or not route.ok or request.route_movement != plan.primary:
        return "no insert plan in this request: Exec the insert and its servo together"
    trajectory = route.result.trajectory
    names, end_row = list(trajectory.joint_names), list(trajectory.positions[-1])
    robot = ctx.scene.snapshot.robots.get(robot_id(request.serial))
    if robot is None:
        return f"{request.serial} is not in the scene"
    gap = insert_end_gap(robot, names, end_row, plan.arms)
    if gap > GROUNDED_END_TOL:  # (W:6004-6007)
        return (f"the flanges stopped {gap * 1e3:.1f} mm from the end of the insert (limit "
                f"{GROUNDED_END_TOL * 1e3:.0f} mm): no servo after it")
    if force_vectors(job.robot, plan.arms) is None:  # (W:6013-6015)
        ctx.log_warn(f"{job.who}: no force reading: the servo's load guard is OFF. Watch the arms and use Cancel if "
                     f"the bar pushes on the floor.")
    return ""


def sides(values: dict[str, float], spec: str) -> str:
    """Side -> number as "L 0.31 R 0.27"."""
    return " ".join(f"{SIDE.get(side, side)} {value:{spec}}" for side, value in values.items())


def line(k: int, reading: dict, plan: ServoPlan) -> str:
    """The panel's one line for a reading: tool0 or the bar, and the force change after an insert (spec §2.5)."""
    bar = reading.get("bar")
    if plan.by_bar and bar is not None and bar["usable"]:
        deviation = bar["mocap_vs_design"]
        text = (f"servo {k}/{MAX_PASSES} · bar {plan.bar.rsplit('/', 1)[-1]} MoCap {deviation['dpos_mm']:.2f} mm / "
                f"{deviation['dang_deg']:.2f}° · FK {bar['fk_vs_design']['dpos_mm']:.2f} mm")
    else:
        text = f"servo {k}/{MAX_PASSES} · tool0 " + sides(
            {side: err["pos_norm_mm"] for side, err in reading["tool0"].items()}, ".2f") + " mm"
    if plan.max_force_change is not None and reading.get("force_change_N"):
        text += f" · force change {sides(reading['force_change_N'], '.1f')} N"
    return text


def aim_text(plan: ServoPlan, engaged: bool, bar: Mapping | None) -> str:
    """What the next pass aims at, for the pass banner: the MoCap bar once engaged, else tool0 and why."""
    short, target = plan.bar.rsplit("/", 1)[-1] if plan.bar else "", SERVO_TARGET[plan.kind]
    if engaged and bar is not None and bar.get("fit") is not None:
        fit = bar["fit"]
        return (f"the MoCap bar {short} onto {target} (fit {fit['n_matched']}/{fit['n_design']}, rms "
                f"{(fit['rms_m'] or 0.0) * 1e3:.1f} mm)")
    if plan.by_bar:
        why = bar["reason"] if bar is not None else "not measured"
        return f"tool0 at the authored targets of {target} (the MoCap bar {short} is not usable: {why})"
    return f"tool0 at the authored targets of {target}"


def load_line(k: int, reading: Mapping, wrench0: Mapping) -> str:
    """The old load line (W:2327-2333): each arm's force and torque, its change since reading 0, the force change."""
    parts = []
    for side, wrench in reading["wrench"].items():
        change = wrench["force_N"] - wrench0[side]["force_N"]
        parts.append(f"{SIDE.get(side, side)}={wrench['force_N']:.1f} N / {wrench['torque_Nm']:.2f} Nm "
                     f"({change:+.1f} N vs iter 0)")
    text = f"[servo {k}] load " + " | ".join(parts)
    if reading.get("force_change_N"):
        text += f" | force change since iter 0 {sides(reading['force_change_N'], '.1f')} N"
    return text


def planned_line(k: int, planner: str, move: Mapping, flanges_move: str) -> str:
    """The old "First move planned" line (W:2493-2495), and its later-pass form."""
    head = "[servo] First move planned" if k == 1 else f"[servo {k}] move planned"
    return (f"{head}: {move['rows']} waypoints, travels {move['excursion_deg']:.1f} deg (ends {move['net_deg']:.1f} "
            f"deg from where it starts), over {move['traj_time']:.0f} s; flanges move {flanges_move} ({planner})")


def end_line(result: str, status: str, reason: str, k: int) -> str:
    """How the loop ended, in the old monitor's words (W:2444, 2617-2652)."""
    if result == CANCELLED:
        return "### SERVOING cancelled by user."
    if status == "not converged":
        return f"### SERVOING ran all {MAX_PASSES} iterations without converging."
    if result == DONE and status == "converged":
        return f"### SERVOING converged at iter {k} {reason.removeprefix('converged ')}"
    if status == "on target":
        return f"### SERVOING stopped at iter 1: {reason}"
    where = f"at iter {k}" if k else "before the first pass"
    return f"### SERVOING stopped {where}: {reason}"


def ik_record(ik: IkResult | None) -> dict | None:
    """The pass's IK as the record keeps it (spec §2.9)."""
    if ik is None:
        return None
    return {"backend": ik.backend, "rank": ik.rank, "combinations": ik.combinations, "max_change": ik.max_change}


def servo_file(view: RunView, plan: ServoPlan) -> str | None:
    """The servo record's path in the run, numbered as `runs.record_movement` will number it; None without a run.

    The number is one past the highest on disk or named here before (a record of this movement may still be queued
    on the writer). ? A queued record the writer then fails to write leaves the next name one too high.
    """
    folder = getattr(view, "folder", None)
    if folder is None:
        return None
    where = Path(folder) / "servo" / plan.action
    taken = [int(number) for number in (path.name[len(plan.target) + 1:-len(".json")]
                                        for path in where.glob(f"{plan.target}.*.json")) if number.isdigit()]
    key = (str(where), plan.target)
    NAMED[key] = max(taken + [NAMED.get(key, 0)], default=0) + 1
    return f"servo/{plan.action}/{plan.target}.{NAMED[key]}.json"
