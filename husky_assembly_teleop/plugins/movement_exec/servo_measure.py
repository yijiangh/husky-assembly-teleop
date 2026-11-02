"""
What the servo part says and records around its loop.

The feedback line, the gate after a ground bar's insert, the panel's one line per reading, the record's IK entry and the
servo file's name: pure helpers for `servo_part.py` (same thread, same rules).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import TYPE_CHECKING

from scaffolding_assembly_core.ids import robot_id

from ...shared.execution.servo import (AFTER_INSERT, BAR_TOL, GROUNDED_END_TOL, MAX_PASSES, TOOL0_TOL, force_vectors,
                                       insert_end_gap)
from ...shared.runs.runs import reserve_record

if TYPE_CHECKING:
    from ...shared.assembly_planning.ik import IkResult
    from ...shared.assembly_run_contract import RunView
    from ...shared.execution.servo import ServoPlan
    from .sequences import Job

#: How a side reads on the panel line and in the prompt ("arm" stays as it is).
SIDE = {"left": "L", "right": "R"}


def say_feedback(job: Job, plan: ServoPlan) -> None:
    """Log what the loop steers on; "Servo by bar" ticked but not usable for a ground bar warns."""
    ctx, settings = job.ctx, job.run.view.settings
    if plan.by_bar:
        ctx.log_info(f"[servo] feedback: the mocap bar {plan.bar} (converges at {BAR_TOL[0] * 1e3:.1f} mm / "
                     f"{math.degrees(BAR_TOL[1]):.1f} deg).")
        return
    # * Only a ground bar held in the tools servos by bar; for any other the box is no promise, so no warning.
    if settings.servo_by_bar and plan.held and plan.grounded:
        ctx.log_warn(f"[servo] 'Servo by bar' is ticked, but this run aims tool0: {plan.why_tool0}.")
    if plan.layout is not None and plan.holder is not None:
        ctx.log_info(f"[servo] feedback: tool0 ({TOOL0_TOL * 1e3:.1f} mm); the mocap bar {plan.bar} is recorded "
                     f"alongside.")
    else:
        why = f"{plan.bar} carries no mocap markers" if plan.held else "no bar held"
        ctx.log_info(f"[servo] feedback: tool0 ({TOOL0_TOL * 1e3:.1f} mm); no mocap bar recorded ({why}).")


def insert_end_problem(job: Job, plan: ServoPlan) -> str:
    """After a ground bar's insert: why the servo may not run, "" if it may; warns when the guard is off.

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
    if gap > GROUNDED_END_TOL:
        return (f"the flanges stopped {gap * 1e3:.1f} mm from the end of the insert (limit "
                f"{GROUNDED_END_TOL * 1e3:.0f} mm): no servo after it")
    if force_vectors(job.robot, plan.arms) is None:
        ctx.log_warn(f"{job.who}: no force reading: the servo's load guard is OFF. Watch the arms and use Cancel if "
                     f"the bar pushes on the floor.")
    return ""


def sides(values: dict[str, float], spec: str) -> str:
    """Side -> number as "L 0.31 R 0.27"."""
    return " ".join(f"{SIDE.get(side, side)} {value:{spec}}" for side, value in values.items())


def line(k: int, reading: dict, plan: ServoPlan) -> str:
    """The panel's one line for a reading: tool0 or the bar, and the force change after an insert."""
    bar = reading.get("bar")
    if plan.by_bar and bar is not None and bar["usable"]:
        deviation = bar["mocap_vs_design"]
        text = (f"servo {k}/{MAX_PASSES} · bar {plan.bar.rsplit('/', 1)[-1]} mocap {deviation['dpos_mm']:.2f} mm / "
                f"{deviation['dang_deg']:.2f}° · FK {bar['fk_vs_design']['dpos_mm']:.2f} mm")
    else:
        text = f"servo {k}/{MAX_PASSES} · tool0 " + sides(
            {side: err["pos_norm_mm"] for side, err in reading["tool0"].items()}, ".2f") + " mm"
    if plan.max_force_change is not None and reading.get("force_change_N"):
        text += f" · force change {sides(reading['force_change_N'], '.1f')} N"
    return text


def ik_record(ik: IkResult | None) -> dict | None:
    """The pass's IK as the record keeps it."""
    if ik is None:
        return None
    return {"backend": ik.backend, "rank": ik.rank, "combinations": ik.combinations, "max_change": ik.max_change}


def servo_file(view: RunView, plan: ServoPlan) -> tuple[Path, str] | None:
    """Take the servo record's file in the run (`runs.reserve_record`); None without a run.

    Returns:
        tuple[Path, str] | None: The file, and its path in the run.
    """
    folder = getattr(view, "folder", None)
    if folder is None:
        return None
    path = reserve_record(Path(folder), plan.action, plan.target, "servo")
    return path, path.relative_to(folder).as_posix()
