"""
The as-built record: where every MoCap bar is at the start and the end of every part a step runs, on the Drive.

One JSON line per record, appended to `<drive>/data_experiment/bar_action_live_replan_exe/<YYYYMMDD>-asbuilt/
as_built_<problem>.jsonl` (the old monitor's file: one per lab day and process, every robot's monitor appending), read
back by `scripts/as_built_page.py`. A record stays raw: the markers as Motive sent them, the fit, where the bar should
be, the robot's base, joints, tool0 and load; every derived number is computed offline, so the page and any later
analysis cannot disagree. Pure: numpy and the core only.

- `as_built_path`, `day_of`: where the file of a day lives.
- `milestone_of`: which moment of a bar's life a record is (before the insert, after the retreat, ...).
- `reference_of`: where a bar should be at that moment, by the process (never from FK).
- `build_record`, `with_saved_robots`, `servo_block`, `bar_lines`: one record from plain pieces, and its log lines.
- `append_record`, `read_records`: write one line, read them all.
"""

from __future__ import annotations

import json
from datetime import datetime
from pathlib import Path
from typing import TYPE_CHECKING, Any, Mapping

import numpy as np

from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.process.plan_check import end_state
from scaffolding_assembly_core.process.relations import placement
from scaffolding_assembly_core.process.types import Holder

from ...drive import drive_folder
from ..assembly_planning.movement import Routine

if TYPE_CHECKING:
    from scaffolding_assembly_core.process import Process

    from .runs import RobotRecord

SCHEMA_VERSION = 2
#: Where the day folders sit, under the Drive root (beside the old monitor's `<day>-servoing/`).
AS_BUILT_FOLDER = Path("data_experiment/bar_action_live_replan_exe")
DIR_SUFFIX = "-asbuilt"
FILE_PREFIX = "as_built_"
#: The record kind other plugins pass to `request_record`.
KIND = "as_built"
#: When a record was taken: just before a step's first part ran, right after a part, or on a click.
PHASES = ("start", "end", "manual")
#: The moments of a bar's life the page puts first (`milestone_of`).
MILESTONES = ("before_insert", "after_insert", "after_fix", "before_retreat", "after_retreat", "hold_approach",
              "hold_at_grasp", "hold_grasped", "hold_released", "hold_retreated", "bystander")
#: The page's letter for each action type.
ACTION_KIND = {"bar_jointing": "J", "bar_release": "R", "bar_holding": "H", "bar_holding_release": "HR",
               "bar_placing": "P"}
#: Part kinds, as `movement_exec_contract` names them (kept here as words, so this module stays pure).
MOTION_PART, INSERT_PART, RETREAT_PART, OPEN_PART, HANDOFF_PART, BY_HAND_PART, SERVO_PART = (
    "motion", "insert", "retreat", "robotiq_open", "handoff", "operator", "servo")


def as_built_path(drive_root: Path | None, problem: str, day: str) -> Path:
    """The as-built file of one day and one process: `<drive>/.../<day>-asbuilt/as_built_<problem>.jsonl`.

    Raises:
        FileNotFoundError: No Drive root, or its runs folder is not synced (`drive_folder`).
    """
    return drive_folder(drive_root, AS_BUILT_FOLDER) / f"{day}{DIR_SUFFIX}" / f"{FILE_PREFIX}{problem}.jsonl"


def day_of(stamp: str) -> str:
    """The day of an ISO stamp as the folder names it: "2026-10-30T14:15:01" -> "20261030"."""
    return stamp[:10].replace("-", "")


def json_safe(value: Any) -> Any:
    """A copy of `value` that `json.dumps` accepts: numpy, tuples, Poses and `to_json` objects as plain types."""
    if hasattr(value, "to_json") and callable(value.to_json):
        return json_safe(value.to_json())
    if isinstance(value, Pose):
        return [*map(float, value.position), *map(float, value.orientation)]
    if isinstance(value, dict):
        return {(k.item() if isinstance(k, np.generic) else k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, Path):
        return str(value)
    return value


def milestone_of(action_type: str | None, routine: Routine | None, part_kind: str | None, phase: str,
                 is_active: bool, last_part: bool, after_insert: bool) -> str | None:
    """Which moment of a bar's life a record is, for the page to put first.

    The action's own bar gets a milestone at the moments below; every other bar measured then is a "bystander".

    | action | routine, part | phase | milestone |
    |---|---|---|---|
    | J | the transfer's last part (its servo, or the mount prompt) | end | before_insert |
    | J | the insert's last part (the insert, or its servo) | end | after_insert |
    | J | a step by hand after the insert (a ground bar: fix the foundation) | end | after_fix |
    | R | the retreat | start / end | before_retreat / after_retreat |
    | H | the approach's last part (its servo) | end | hold_approach |
    | H | the linear move in / the handoff | end | hold_at_grasp / hold_grasped |
    | HR | the gripper open / the linear move out | end | hold_released / hold_retreated |

    Args:
        action_type: The action's type ("bar_jointing", ...), None outside an action.
        routine: The part's movement's routine.
        part_kind: The part's kind.
        phase: One of PHASES.
        is_active: The bar is the one the action is about.
        last_part: The part is the last of its step.
        after_insert: The step comes after the action's insert.
    """
    if not is_active:
        return "bystander"
    if phase == "manual" or action_type is None:
        return None
    kind = ACTION_KIND.get(action_type)
    if kind == "R" and routine is Routine.RETREAT_LINEAR and part_kind == RETREAT_PART:
        return "before_retreat" if phase == "start" else "after_retreat"
    if phase != "end":
        return None
    if kind == "J":
        if routine in (Routine.TRANSFER_FREE, Routine.TRANSFER_HELD) and last_part:
            return "before_insert"
        if routine is Routine.INSERT and last_part:
            return "after_insert"
        if routine is Routine.OPERATOR and part_kind == BY_HAND_PART and after_insert:
            return "after_fix"
    elif kind == "H":
        if routine is Routine.SUPPORT_APPROACH and last_part:
            return "hold_approach"
        if routine is Routine.SUPPORT_LINEAR and part_kind == MOTION_PART:
            return "hold_at_grasp"
        if routine is Routine.TOOL_ONLY and part_kind == HANDOFF_PART:
            return "hold_grasped"
    elif kind == "HR":
        if routine is Routine.TOOL_ONLY and part_kind == OPEN_PART:
            return "hold_released"
        if routine is Routine.SUPPORT_LINEAR and part_kind == MOTION_PART:
            return "hold_retreated"
    return None


def reference_of(process: Process, action_id: str | None, movement_index: int | None, bar: str, phase: str,
                 servo_bar: tuple[str, Pose, tuple[str, Pose]] | None = None) -> tuple:
    """Where a bar should be at a record, by the process: never from FK (the old `_as_built_bar_reference`).

    In order: the servo's own target when a servo aims this bar; the assembly pose when nothing is loaded; the
    movement's start state for "start" and "manual", the state it ends in for "end"; a resting bar's pose; a held bar's
    authored flange pose (the movement's own target at its end, else the nearest earlier movement's) composed with the
    grasp; none authored (the bar mounted by hand): no reference.

    Args:
        process: The process.
        action_id: The action, or None.
        movement_index: The part's movement in the action, or None.
        bar: The bar's body id.
        phase: One of PHASES.
        servo_bar: (bar, the design bar pose the servo aims at, (holder flange, grasp)) of a running servo, or None.

    Returns:
        tuple: (T_reference 4x4 | None, kind, held_by link id | None, T_link_from_bar 4x4 | None), kind one of
        "servo_target", "assembled", "resting", "held_authored:<movement id>", "held_no_authored_pose".
    """
    if servo_bar is not None and servo_bar[0] == bar:
        holder, grasp = servo_bar[2]
        return servo_bar[1].matrix(), "servo_target", holder, grasp.matrix()
    assembled = (process.bodies[bar].assembly_pose.matrix(), "assembled", None, None)
    action = process.actions.get(action_id) if action_id else None
    if action is None or movement_index is None or not 0 <= movement_index < len(action.movements):
        return assembled
    movement = action.movements[movement_index]
    state = end_state(movement) if phase == "end" else movement.start
    if bar not in state.present:
        return assembled
    where = placement(process, state, bar)
    if isinstance(where, Pose):
        resting = bar not in state.built and bar in state.poses
        return where.matrix(), "resting" if resting else "assembled", None, None
    holder: Holder = where
    T_grasp = holder.grasp.matrix()
    # * The flange's authored pose: this movement's own target at its end, else the nearest earlier one that has it.
    candidates = [movement] if phase == "end" else []
    candidates += list(reversed(action.movements[:movement_index]))
    for source in candidates:
        own = source.target.robots.get(action.robot) if source.target is not None else None
        pose = own.link_poses.get(holder.to) if own is not None else None
        if pose is not None:
            return compose(pose, holder.grasp).matrix(), f"held_authored:{source.id}", holder.to, T_grasp
    return None, "held_no_authored_pose", holder.to, T_grasp


def build_record(*, problem: str, run_id: str, robot: str, serial: str, action: Mapping | None,
                 step: Mapping | None, instrumented_bars: list, bars: Mapping, robot_state: Mapping,
                 tool0: Mapping, load: Mapping | None, servo: Mapping | None, options: Mapping,
                 robots: Mapping, settings: Mapping, now: datetime | None = None) -> dict:
    """One as-built record from plain pieces, ready for `append_record`.

    Args:
        problem: The process folder's name.
        run_id: The run folder's name, "" without a run.
        robot: The recording robot's process name ("cindy").
        serial: Its serial.
        action: {index, id, type, kind, bar, bar_id, robot}, or None outside an action.
        step: {index, movement_id, routine, part, step, phase, outcome, status, reason, dry_run}; None on a click
            with nothing loaded (the phase is then "manual").
        instrumented_bars: Every bar that carries markers: [{bar, bar_id, mocap_id}].
        bars: Bar id -> its measurement entry (the fit, the sample, the reference, the deviations, the milestone).
        robot_state: The recording robot's base, joints, tool0 FK and authored targets.
        tool0: Side -> tool0 error against the part's authored target (`servo.tool0_errors`), {} without targets.
        load: {arms {side: {force_N, torque_Nm, wrench}}, zero {at, reason} | None}, or None without wrenches.
        servo: The servo part's summary (`servo_block`), or None.
        options: The settings and tick boxes in force: what the page lists per step.
        robots: Live id -> {name, serial, base, joints, source, at, after_action} of every live robot.
        settings: The run's settings.
        now: The time stamp (now by default).

    Raises:
        ValueError: An unknown phase.
    """
    phase = (step or {}).get("phase", "manual")
    if phase not in PHASES:
        raise ValueError(f"unknown as-built phase {phase!r}; expected one of {PHASES}")
    now = now or datetime.now()
    return json_safe({
        "schema_version": SCHEMA_VERSION, "stamp": now.isoformat(timespec="seconds"), "stamp_epoch": now.timestamp(),
        "problem": problem, "run_id": run_id, "robot": robot, "serial": serial, "action": action,
        "step": step if step is not None else {"phase": "manual", "outcome": None},
        "instrumented_bars": list(instrumented_bars), "bars": bars, "robot_state": robot_state, "tool0": tool0,
        "load": load, "servo": servo, "options": options, "robots": robots, "settings": settings})


def with_saved_robots(record: dict, saved: Mapping[str, RobotRecord]) -> dict:
    """The record with the robots it lacks filled from the run's saved ends (source "assumed"), for the page."""
    robots = dict(record.get("robots") or {})
    for live, end in saved.items():
        if live in robots:
            continue
        robots[live] = json_safe({"name": live.rsplit("/", 1)[-1], "serial": live.rsplit("/", 1)[-1],
                                  "base": end.base, "joints": dict(end.joints), "source": "assumed", "at": end.at,
                                  "after_action": end.after_action})
    return {**record, "robots": robots}


def servo_block(payload: Mapping, file: str | None) -> dict | None:
    """The page's servo numbers from a servo record (`servo_record.servo_payload`): its last reading and outcome."""
    readings = payload.get("servoing_data") or []
    if not readings:
        return None
    last = readings[-1]
    arms = {side: {"pos_norm_mm": error.get("pos_norm_mm"), "rot_max_deg": max(map(abs, error.get("rot_err_deg", [0])))}
            for side, error in last.get("tool0", {}).items()}
    bar = last.get("bar") or {}
    settings = payload.get("settings", {})
    outcome = payload.get("outcome", {})
    return {"file": file, "movement_id": payload.get("movement_id"), "feedback": payload.get("servo_feedback"),
            "planner_mode": payload.get("planner_mode"), "iterations": len(payload.get("moves") or []),
            "arms": arms, "worst": max((arm["pos_norm_mm"] for arm in arms.values() if arm["pos_norm_mm"] is not None),
                                       default=None),
            "bar": {key: bar.get(key) for key in ("mocap_vs_design", "fk_vs_design", "mocap_vs_fk")} if bar else None,
            "engaged": payload.get("servo_feedback") == "mocap_bar", "by_bar": settings.get("by_bar"),
            "why_tool0": settings.get("why_tool0"), "status": outcome.get("status"), "reason": outcome.get("reason")}


def bar_lines(record: Mapping) -> list[str]:
    """One line per measured bar for the `[as-built]` log, in the old words; "no bar measured" without any."""
    lines = []
    for bar_id, entry in (record.get("bars") or {}).items():
        fit = entry.get("bar_fit") or {}
        if fit.get("ok"):
            fit_text = f"fit ok {fit['n_matched']}/{fit['n_design']}, rms {(fit.get('rms_m') or 0.0) * 1e3:.1f} mm"
        else:
            fit_text = f"fit FAILED ({fit.get('reason', 'no fit')})"
        deviation = entry.get("deviation_vs_reference")
        reference = entry.get("reference_kind")
        if deviation is not None:
            reference_text = (f"vs {reference} dpos {deviation['dpos_mm']:.1f} mm, dang "
                              f"{deviation['dang_deg']:.3f} deg")
        else:
            reference_text = f"no reference ({reference})"
        milestone = entry.get("milestone")
        lines.append(f"{bar_id}{f' [{milestone}]' if milestone else ''} {fit_text} | {reference_text}")
    return lines or ["no bar measured"]


def append_record(path: Path, record: Mapping) -> Path:
    """Append one record as one line of the as-built file; the day folder is made when missing."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    line = json.dumps(json_safe(record))
    with open(path, "a") as file:
        file.write(line + "\n")
        file.flush()
    return path


def read_records(path: Path) -> list[dict]:
    """Every record of an as-built file, in file order; empty when the file does not exist.

    ! A last line cut short (a crash mid-write, the Drive caught while it syncs) is dropped with a printed note; a
    broken line anywhere else means the file was edited or damaged, and raises.

    Raises:
        ValueError: A line other than the last cannot be read.
    """
    path = Path(path)
    if not path.exists():
        return []
    lines = [line for line in path.read_text().split("\n") if line.strip()]
    records = []
    for number, line in enumerate(lines, start=1):
        try:
            records.append(json.loads(line))
        except json.JSONDecodeError as error:
            if number == len(lines):
                print(f"[as-built] {path.name}: its last line is cut short (line {number}); it is skipped.")
                break
            raise ValueError(f"{path}: line {number} is not a JSON record ({error})") from error
    return records
