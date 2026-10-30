"""
The as-built records of one lab day shaped for the 3D assembly page (the old `4_assembly_viewer.py` on the new records).

`collect_assembly` reads the day's file (`shared/runs/as_built.py`) and the process it was made on, and gives the page
(`page.py` + the unchanged `4_assembly_viewer.js`) the same scene data as before: a slider over the schedule's actions,
each record's bars (placement at the start, middle and end against where the bar should be, rotation, the plates and
the bow, the markers), the robot's tool0 error, the servo's last pass, the wrist load, the tick boxes in force, every
bar's history and the verdict on its balls. The numbers the old page took from the servo files and the monitor log
come from the record itself now (`servo`, `options`).
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from scaffolding_assembly_core.process import read

from ...shared.runs.as_built import (ACTION_KIND, AS_BUILT_FOLDER, DIR_SUFFIX, FILE_PREFIX, as_built_path,
                                     read_records)
from ...world.bar_fit import BarFit
from ...world.bar_pose import bar_ends, pose_deviation
from ...world.marker_fit import plate_geometry
from ...world.marker_layouts import BarLayout, Sensed
from .page import RobotModels, flatten, matrix_from_pose

#: Records of one action further apart than this, or from another run, are a redo ("run 2").
REDO_GAP_S = 600.0
#: The tick boxes that change what a step does, as the page names them (the old panel's words).
OPTION_BOXES = {"free_move": "Transfer as a free move (workaround)", "servo_bar": "Servo feedback: MoCap bar markers",
                "spiral": "Insert compliant part: SPIRAL search"}
#: The verdict on a bar's balls (`ball_findings`), set from the first runs (2026-10-20, B20).
BALL_PAIR_TOL_MM = 0.6
PLATE_OFFSET_TOL_MM = 0.5
PLATE_STEADY_MM = 0.5
BOW_CHANGE_TOL_MM = 0.5
FIT_RMS_TOL_MM = 1.0
MIN_RECORDS_FOR_DAY = 3
#: The measurements the page colours and labels by, in the card's order.
METRICS = [
    {"key": "placement", "label": "placement error", "unit": "mm", "high": 5.0,
     "help": "the worst of the distances at the start, the middle and the end of the bar, between the MoCap bar "
             "and where it should be at that moment (the step target)",
     "formula": "max over start / middle / end of | MoCap - target |"},
    {"key": "rotation", "label": "rotation error", "unit": "deg", "high": 0.25,
     "help": "the angle between the MoCap bar's centre line and the target's",
     "formula": "arccos | z_MoCap . z_target |"},
    {"key": "tool0", "label": "tool0 vs target", "unit": "mm", "high": 1.5,
     "help": "where the robot believes its flange is (mocap base + joints) against the authored target of the "
             "step, the worse arm; the part of the error the robot can see",
     "formula": "max over arms of | FK(base, joints) - target |"},
    {"key": "servo", "label": "servo residual", "unit": "mm", "high": 1.5,
     "help": "the servo's last iteration in this step (tool0 vs its aim), the worse arm",
     "formula": "last iteration: max over arms of pos_norm_mm"},
    {"key": "pair_length", "label": "plate length error", "unit": "mm", "high": 2.0,
     "help": "the worst | M1-M2 distance - design | over the bar's MoCap plates; large = a ball mislabelled, "
             "hidden or loose (not bending)",
     "formula": "max over plates of | |M1 - M2| - 48.3 mm |"},
    {"key": "spacing", "label": "spacing", "unit": "mm", "high": 2.0,
     "help": "the worst | distance between neighbouring plates (the midpoints of their two balls) - design |; the "
             "same in every record = plates mounted off the table, changing = a ball moved",
     "formula": "max over neighbouring plates of | |mid_a - mid_b| - design |"},
    {"key": "bend", "label": "bending", "unit": "mm", "high": 1.0,
     "help": "how far the inner MoCap joints sit off the straight line through the outer two, across the bar: "
             "the bow of the bar (Motive noise ~0.5 mm)",
     "formula": "max over inner joints of | shift_xy - line through the outer joints' shifts |"},
    {"key": "fit_residual", "label": "fit residual", "unit": "mm", "high": 1.5,
     "help": "the worst distance between a sensed ball and its design ball once the bar is fitted",
     "formula": "max over matched balls of | sensed - fitted design |"},
    {"key": "load", "label": "load imbalance", "unit": "N", "high": 2.0,
     "help": "the gap between the two wrists' force magnitudes (Cindy only)",
     "formula": "| |F_left| - |F_right| |"},
]
EXTRA_FORMULAS = [
    ("MoCap vs FK", "MoCap bar against tool0 FK composed with the grasp: the part of the error the robot cannot "
                    "see (base calibration, arm, TCP, the bar in the jaws)"),
    ("reference", "the step target: the seat for a resting bar; for a held bar the authored flange target of "
                  "the step's end composed with the grasp (never FK)"),
]
#: Short names for the records' pills.
MILESTONE_LABEL = {
    "before_insert": "before insert", "after_insert": "after insert", "after_fix": "after fix",
    "before_retreat": "before retreat", "after_retreat": "after retreat", "hold_approach": "approach",
    "hold_at_grasp": "at grasp", "hold_grasped": "grasped", "hold_released": "released",
    "hold_retreated": "backed off",
}


def finite(value: Any) -> Any:
    """A copy of `value` with NaN and infinities turned into None: `JSON.parse` refuses a bare NaN."""
    if isinstance(value, dict):
        return {k: finite(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [finite(v) for v in value]
    if isinstance(value, np.ndarray):
        return finite(value.tolist())
    if isinstance(value, np.generic):
        return finite(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


# * ================================================================ the day's files

def as_built_days(drive_root: Path) -> list[str]:
    """Every day with an as-built folder on the Drive, oldest first."""
    folder = drive_root / AS_BUILT_FOLDER
    return sorted(path.name[:-len(DIR_SUFFIX)] for path in folder.glob(f"*{DIR_SUFFIX}") if path.is_dir()) \
        if folder.is_dir() else []


def as_built_problems(drive_root: Path, day: str) -> list[str]:
    """The problems a day holds records of, by its files, newest first."""
    folder = drive_root / AS_BUILT_FOLDER / f"{day}{DIR_SUFFIX}"
    files = sorted(folder.glob(f"{FILE_PREFIX}*.jsonl"), key=lambda path: path.stat().st_mtime, reverse=True)
    return [path.name[len(FILE_PREFIX):-len(".jsonl")] for path in files]


# * ================================================================ the process

def timeline(process) -> list[dict]:
    """Which bars exist at each scheduled action, and who holds them.

    A bar exists from its J action on; Cindy (the acting robot) holds it between its J and its R; a support robot holds
    it between its H and its HR.
    """
    actions = [process.actions[action_id] for action_id in process.schedule]
    kinds = [ACTION_KIND.get(action.type, "?") for action in actions]
    robots = [process.robots[action.robot].id.rsplit("/", 1)[-1].capitalize() if action.robot in process.robots
              else "" for action in actions]
    j_at = {action.bar: k for k, (action, kind) in enumerate(zip(actions, kinds)) if kind in ("J", "P")}
    r_at = {action.bar: k for k, (action, kind) in enumerate(zip(actions, kinds)) if kind == "R"}
    out = []
    for k, (action, kind) in enumerate(zip(actions, kinds)):
        exists = [bar for bar, at in j_at.items() if at <= k]
        held = {bar: robots[j_at[bar]] for bar in exists if j_at[bar] < k < r_at.get(bar, len(actions))}
        for h, (other, other_kind) in enumerate(zip(actions, kinds)):
            if other_kind == "H" and h <= k:
                release = next((hr for hr, (later, later_kind) in enumerate(zip(actions, kinds)) if hr > h
                                and later_kind == "HR" and later.bar == other.bar and later.robot == other.robot),
                               len(actions))
                if k < release:
                    held[other.bar] = robots[h]
        out.append({"index": k, "action_id": action.id, "kind": kind, "bar_id": _short(action.bar),
                    "robot": robots[k], "exists": [_short(bar) for bar in exists],
                    "held": {_short(bar): robot for bar, robot in held.items()}})
    return out


def assembled_bars(process) -> dict[str, dict]:
    """Every bar's assembled (seat) pose and length: {bar id: {T, length, ends}}."""
    out = {}
    for body_id, body in process.bodies.items():
        if not body_id.startswith("bars/"):
            continue
        cylinder = next((shape for shape in body.geometry.collision if hasattr(shape, "height")), None)
        if cylinder is None:
            print(f"[assembly] ! {body_id}: no cylinder in its collision shapes; it is not drawn.")
            continue
        T = body.assembly_pose.matrix()
        out[_short(body_id)] = {"T": T, "length": float(cylinder.height),
                                "ends": bar_ends(T, float(cylinder.height)).tolist()}
    return out


def _short(bar: str) -> str:
    """A bar's name without its folder."""
    return bar.rsplit("/", 1)[-1]


# * ================================================================ numbers of one record

def _mat(value) -> np.ndarray | None:
    """A recorded 4x4 as an array, or None."""
    return None if value is None else np.asarray(value, dtype=float)


def _three_places(T_a: np.ndarray, T_b: np.ndarray, length: float) -> dict[str, float]:
    """The distances between two bars at the start, the middle and the end, mm: {start, middle, end, worst, ...}."""
    a, b = bar_ends(T_a, length), bar_ends(T_b, length)
    start, end = float(np.linalg.norm(a[0] - b[0])), float(np.linalg.norm(a[1] - b[1]))
    middle = float(np.linalg.norm(a.mean(axis=0) - b.mean(axis=0)))
    values = [start, middle, end]
    return {"start": start * 1e3, "middle": middle * 1e3, "end": end * 1e3, "worst": max(values) * 1e3,
            "mean": float(np.mean(values)) * 1e3, "spread": (max(values) - min(values)) * 1e3}


def _deviation(T_a: np.ndarray | None, T_b: np.ndarray | None) -> dict | None:
    """How far one pose is from another, as plain numbers; None when a pose is missing."""
    if T_a is None or T_b is None:
        return None
    deviation = pose_deviation(T_a, T_b)
    return {"dpos_mm": deviation.dpos_mm, "dang_deg": deviation.dang_deg,
            "dpos_bar_mm": np.asarray(deviation.dpos_bar_mm).tolist(),
            "rotvec_bar_deg": np.asarray(deviation.rotvec_bar_deg).tolist()}


def axis_angle_deg(T_a: np.ndarray, T_b: np.ndarray) -> float:
    """The angle between two bars' centre lines, whichever way each points, 0 to 90 degrees."""
    za, zb = np.asarray(T_a, dtype=float)[:3, 2], np.asarray(T_b, dtype=float)[:3, 2]
    cos = abs(float(za @ zb)) / (np.linalg.norm(za) * np.linalg.norm(zb))
    return float(np.degrees(np.arccos(min(1.0, cos))))


def _side_of_link(link: str | None) -> str:
    """The arm side a held link belongs to: left, right, or arm on a one-arm robot."""
    for side in ("left", "right"):
        if side in (link or ""):
            return side
    return "arm"


def _fit_of(entry: Mapping) -> tuple[BarFit, Sensed | None, BarLayout]:
    """The recorded fit, sample and layout back as objects, for the plate geometry."""
    fit_json, sensed_json, layout_json = entry.get("bar_fit") or {}, entry.get("sensed"), entry.get("layout") or {}
    fit = BarFit(ok=bool(fit_json.get("ok")), reason=fit_json.get("reason", ""), bar_name=fit_json.get("bar_name", ""),
                 T_world_from_bar=_mat(fit_json.get("T_world_from_bar")),
                 correspondences={int(k): int(v) for k, v in (fit_json.get("correspondences") or {}).items()},
                 sensed_ids=list(fit_json.get("sensed_ids") or []),
                 residuals_m={int(k): float(v) for k, v in (fit_json.get("residuals_m") or {}).items()},
                 rms_m=fit_json.get("rms_m"), max_residual_m=fit_json.get("max_residual_m"),
                 n_matched=fit_json.get("n_matched", 0), n_design=fit_json.get("n_design", 0),
                 unmatched_sensed=list(fit_json.get("unmatched_sensed") or []),
                 missing_design=list(fit_json.get("missing_design") or []), ambiguous=bool(fit_json.get("ambiguous")),
                 tracked=bool(fit_json.get("tracked")))
    sensed = None
    if sensed_json:
        residuals = sensed_json.get("residuals")
        sensed = Sensed(np.asarray(sensed_json.get("points") or np.zeros((0, 3)), dtype=float).reshape(-1, 3),
                        tuple(sensed_json.get("ids") or ()), None if residuals is None else np.asarray(residuals),
                        float(sensed_json.get("age") or 0.0), sensed_json.get("source") or "",
                        int(sensed_json.get("dropped") or 0), int(sensed_json.get("occluded") or 0))
    layout = BarLayout(entry.get("bar") or "", tuple(layout_json.get("names") or ()),
                       np.asarray(layout_json.get("points") or np.zeros((0, 3)), dtype=float).reshape(-1, 3),
                       float(layout_json.get("length") or 0.0))
    return fit, sensed, layout


def marker_match(fit: BarFit, sensed: Sensed | None, layout: BarLayout, T_reference: np.ndarray | None) -> tuple:
    """Every sensed marker with what it matched, and every design marker where it should be (the page's balls).

    Returns:
        tuple: (sensed [{id, pos, state, name, residual_mm}], authored [{name, pos, seen}]); a sensed marker's state
        is "matched" or "stray" (the fit used it, matched to nothing).
    """
    rows = []
    if sensed is not None:
        design_of = {s: d for s, d in fit.correspondences.items()}
        for s, point in enumerate(sensed.points):
            d = design_of.get(s)
            residual = fit.residuals_m.get(s)
            rows.append({"id": sensed.ids[s] if s < len(sensed.ids) else s, "pos": [float(v) for v in point],
                         "state": "matched" if d is not None else "stray",
                         "name": layout.names[d] if d is not None and d < len(layout.names) else None,
                         "residual_mm": None if residual is None else float(residual) * 1e3})
    authored = []
    if T_reference is not None and len(layout.points):
        placed = (T_reference[:3, :3] @ layout.points.T).T + T_reference[:3, 3]
        missing = set(fit.missing_design)
        authored = [{"name": name, "pos": placed[d].tolist(), "seen": d not in missing}
                    for d, name in enumerate(layout.names)]
    return rows, authored


def shape_bar(entry: Mapping, record: Mapping, seat: Mapping) -> dict:
    """One measured bar of a record, with every number the page shows."""
    fit, sensed, layout = _fit_of(entry)
    length = seat["length"]
    T_fit = fit.T_world_from_bar if fit.ok else None
    T_ref, T_seat = _mat(entry.get("T_reference")), seat["T"]
    names = list(layout.names)
    out = {
        "bar": entry.get("bar_id"), "body": f"bar_{entry.get('bar_id')}", "ok": bool(fit.ok), "reason": fit.reason,
        "source": entry.get("marker_source"), "age_s": entry.get("marker_age_s"),
        "milestone": entry.get("milestone"), "held_by": entry.get("held_by"),
        "reference_kind": entry.get("reference_kind"),
        "fitted": bar_ends(T_fit, length).tolist() if T_fit is not None else None,
        "reference": bar_ends(T_ref, length).tolist() if T_ref is not None else None,
        "seat": seat["ends"],
        "n_matched": fit.n_matched, "n_design": fit.n_design,
        "fit_rms": None if fit.rms_m is None else fit.rms_m * 1e3,
        "fit_residual": None if fit.max_residual_m is None else fit.max_residual_m * 1e3,
        "ambiguous": fit.ambiguous, "tracked": fit.tracked,
        "missing": [names[int(d)] for d in fit.missing_design if int(d) < len(names)],
        "strays": len(fit.unmatched_sensed),
    }
    out["markers_sensed"], out["markers_authored"] = marker_match(fit, sensed, layout,
                                                                  T_ref if T_ref is not None else T_seat)
    if T_fit is None:
        return out
    # * Where the bar is against where it should be now (the reference) and against its seat.
    if T_ref is not None:
        tips = _three_places(T_fit, T_ref, length)
        out.update({f"tip_{k}": v for k, v in tips.items() if k in ("start", "middle", "end")})
        out["placement"] = tips["worst"]
        out["rotation"] = axis_angle_deg(T_fit, T_ref)
        out["vs_reference"] = _deviation(T_fit, T_ref)
    seat_tips = _three_places(T_fit, T_seat, length)
    out.update({f"seat_{k}": v for k, v in seat_tips.items() if k in ("start", "middle", "end")})
    out["placement_seat"] = seat_tips["worst"]
    out["rotation_seat"] = axis_angle_deg(T_fit, T_seat)
    # * The two frames the page draws as axes, and the measured bar's turn from the drawn reference.
    T_axes = T_ref if T_ref is not None else T_seat
    out["frame_fit"], out["frame_ref"] = flatten(T_fit), flatten(T_axes)
    out["turn_ref_deg"] = _deviation(T_fit, T_axes)["rotvec_bar_deg"]
    # * The plates: length, shift across the bar, bending.
    if sensed is not None and len(layout.points):
        geometry = plate_geometry(fit, sensed, layout).to_json()
        out["pairs"] = {"joints": geometry["joints"], "spacing": geometry["spacing"]}
        out["bend"], out["pair_length"] = geometry["bend_mm"], geometry["worst_length_err_mm"]
        gaps = [abs(gap["err_mm"]) for gap in geometry["spacing"]]
        out["spacing"] = max(gaps) if gaps else None
    # * Where the ROBOT believes the bar is (tool0 FK with the grasp), when an arm holds it.
    grasp = _mat(entry.get("T_link_from_bar"))
    side = _side_of_link(entry.get("held_by"))
    fk_pose = (((record.get("robot_state") or {}).get("arms") or {}).get(side) or {}).get("tool0_fk")
    if grasp is not None and fk_pose:
        T_fk = matrix_from_pose(fk_pose) @ grasp
        out["fk"] = {"side": side, "fk_vs_reference": _deviation(T_fk, T_ref),
                     "mocap_vs_reference": _deviation(T_fit, T_ref), "mocap_vs_fk": _deviation(T_fit, T_fk),
                     "line": bar_ends(T_fk, length).tolist()}
    return out


def tool0_numbers(record: Mapping) -> dict:
    """Per arm: where tool0 is by FK against the step's authored target, and where FK puts the flange."""
    out = {}
    for side, arm in (((record.get("robot_state") or {}).get("arms")) or {}).items():
        fk = arm.get("tool0_fk")
        out[side] = {"fk_vs_target": _deviation(matrix_from_pose(fk), matrix_from_pose(arm.get("tool0_target_end"))),
                     "pos": matrix_from_pose(fk)[:3, 3].tolist() if fk else None}
    return out


def load_numbers(record: Mapping) -> dict | None:
    """The wrists' force magnitudes and their gap (a two-arm robot), and what they are relative to."""
    arms = ((record.get("load") or {}).get("arms")) or {}
    if "left" not in arms or "right" not in arms:
        return None
    zero = (record.get("load") or {}).get("zero")
    return {"left_N": arms["left"]["force_N"], "right_N": arms["right"]["force_N"],
            "imbalance": abs(arms["left"]["force_N"] - arms["right"]["force_N"]),
            "zero": None if not zero else {"at": str(zero.get("at")), "note": zero.get("reason")}}


def options_of(record: Mapping) -> list[dict]:
    """Each tick box that could change the step: how it was set and whether the step used it (the record says)."""
    options, servo = record.get("options") or {}, record.get("servo")
    step = record.get("step") or {}
    routine, part_kind = step.get("routine"), (step.get("part") or {}).get("kind")
    is_transfer, is_insert = routine in ("transfer_free", "transfer_held"), routine == "insert"
    free = options.get("transfer_as_free") or {}
    out = [{"key": "free_move", "box": free.get("box") if is_transfer else None, "applies": is_transfer,
            "used": free.get("used") if is_transfer else None,
            "why": ("planned from the live arms without the bar; the bar is mounted by hand at the insertion start"
                    if free.get("used") else "the transfer carried a bar already in the tools") if is_transfer else "",
            "shown": is_transfer}]
    by_bar = options.get("servo_by_bar") or {}
    runs_servo = part_kind == "servo" or servo is not None
    used = bool(servo and servo.get("engaged"))
    if used:
        why = "the servo aimed the bar's MoCap fit at its Rhino pose"
    elif runs_servo and (record.get("action") or {}).get("kind") == "H":
        why = "a support robot's servo aims its flange (tool0) only"
    elif runs_servo:
        why = "the servo aimed the flanges (tool0) at their Rhino targets" + (f": {by_bar['why']}" if by_bar.get("why")
                                                                              else "")
    else:
        why = "no servo runs in this step"
    out.append({"key": "servo_bar", "box": by_bar.get("box"), "applies": runs_servo,
                "used": used if runs_servo else False, "why": why, "shown": runs_servo or bool(by_bar.get("box"))})
    spiral = options.get("spiral") or {}
    mode = options.get("insert_mode")
    if is_insert and spiral.get("used"):
        entry = {"box": True, "applies": True, "used": True,
                 "why": "the compliant part circled the bar about the insertion line while the joint tightened"}
    elif is_insert and mode and mode != "rigid":
        one_mate = "" if spiral.get("fits") else " (a bar with one mating joint never spirals)"
        entry = {"box": bool(spiral.get("box")), "applies": True, "used": False,
                 "why": "the compliant part drove a straight line" + one_mate}
    else:
        why = ("this insert ran rigid all the way (a grounded bar): the box does not act on it" if is_insert
               else "the box acts on a screwed bar's insert only")
        entry = {"box": spiral.get("box"), "applies": False, "used": False, "why": why}
    out.append({"key": "spiral", **entry, "shown": is_insert or bool(entry["box"])})
    for option in out:
        option["label"] = OPTION_BOXES[option["key"]]
    return out


# * ================================================================ a bar through the day

def add_history(history: dict, bar: Mapping, entry: Mapping) -> bool:
    """Add one measurement of a bar to its history, unless the bar was not placed yet (then only counted)."""
    slot = history.setdefault(bar["bar"], {"entries": [], "skipped": 0})
    # ! A cancelled step's milestone does not count: the bar may still lie on its loading spot.
    if not slot["entries"] and (bar.get("milestone") in (None, "bystander")
                                or entry.get("outcome") not in (None, "ok")):
        slot["skipped"] += 1
        return False
    slot["entries"].append(entry)
    return True


def ball_check(pairs_of_day: list) -> dict:
    """One bar's balls over a day: what stays the same between records and what wanders (per plate)."""
    by_joint: dict[str, list] = {}
    for pairs in pairs_of_day:
        for joint in (pairs or {}).get("joints") or []:
            by_joint.setdefault(joint["joint"], []).append(joint)
    plates = {}
    for joint, rows in by_joint.items():
        errs = [row["length_err_mm"] for row in rows if row["length_err_mm"] is not None]
        shifts = np.asarray([row["shift_mm"] for row in rows if row["shift_mm"] is not None],
                            dtype=float).reshape(-1, 3)
        bows = np.asarray([row["bow_mm"] for row in rows if row.get("bow_mm") is not None], dtype=float).reshape(-1, 2)
        plates[joint] = {
            "n": len(rows), "pair_err_min": min(errs) if errs else None, "pair_err_max": max(errs) if errs else None,
            "shift_mean": shifts.mean(axis=0).tolist() if len(shifts) else None,
            "shift_range": float(np.ptp(shifts, axis=0).max()) if len(shifts) else None,
            "bow_mean": bows.mean(axis=0).tolist() if len(bows) else None,
            "bow_range": float(np.ptp(bows, axis=0).max()) if len(bows) else None}
    ranges = [p["pair_err_max"] - p["pair_err_min"] for p in plates.values() if p["pair_err_min"] is not None]
    n = len(pairs_of_day)
    return {"n_records": n, "noise_mm": max(ranges) / 2 if n >= 2 and ranges else None, "plates": plates}


def _plate_name(joint: str) -> str:
    """A MoCap joint's short name: "joints/J20-22_mocap" (or the old "joint_J20-22_mocap") -> "J20-22"."""
    return joint.rsplit("/", 1)[-1].replace("joint_", "").replace("_mocap", "")


def ball_findings(bar: Mapping, day: Mapping | None = None) -> list[str]:
    """The verdict on one measured bar, in plain words: a bad ball, a plate off the table, bending, a doubtful fit."""
    if not bar.get("ok") or not bar.get("pairs"):
        return []
    lines = []
    joints = bar["pairs"]["joints"]
    plates = (day or {}).get("plates") or {}
    steady = day is not None and day["n_records"] >= MIN_RECORDS_FOR_DAY
    # * 1. A ball off its spot: its plate's two balls are not 48.3 mm apart (bending cannot change that).
    for j in joints:
        name = _plate_name(j["joint"])
        if j["length_mm"] is None:
            lines.append(f"plate {name}: only {j['n_seen']} of its 2 balls seen (hidden or not streamed)")
            continue
        if abs(j["length_err_mm"]) <= BALL_PAIR_TOL_MM:
            continue
        p = plates.get(j["joint"]) or {}
        always = (steady and p.get("pair_err_min") is not None
                  and min(abs(p["pair_err_min"]), abs(p["pair_err_max"])) > BALL_PAIR_TOL_MM
                  and p["pair_err_min"] * p["pair_err_max"] > 0)
        design = j["length_mm"] - j["length_err_mm"]
        lines.append(f"bad ball on plate {name}: its two balls are {j['length_mm']:.1f} mm apart, not {design:.1f}"
                     + (" -- in every record this day: a ball mounted off its spot or a wrong table entry" if always
                        else ": a ball loose, half hidden or mislabelled"))
    # * 2. Bending: an inner plate's bow away from its usual one (needs the day).
    if steady:
        for j in joints:
            p = plates.get(j["joint"]) or {}
            if j.get("bow_mm") is None or p.get("bow_mean") is None:
                continue
            change = float(np.linalg.norm(np.asarray(j["bow_mm"]) - np.asarray(p["bow_mean"])))
            if change > BOW_CHANGE_TOL_MM:
                lines.append(f"bending at plate {_plate_name(j['joint'])}: its bow is {change:.1f} mm away from its "
                             f"usual this day (the bar bends under a load here, or a ball moved)")
    # * 3. Constant offsets: plates off the design table the same way in every record (needs the day).
    offsets = []
    if steady:
        for joint, p in plates.items():
            if p["shift_mean"] is None or p["shift_range"] is None:
                continue
            size = float(np.linalg.norm(p["shift_mean"]))
            if size > PLATE_OFFSET_TOL_MM and p["shift_range"] < PLATE_STEADY_MM:
                offsets.append(f"{_plate_name(joint)} {size:.1f}")
    if offsets:
        lines.append(f"plates off the design table by the same amount in every record (mm): {', '.join(offsets)} -- "
                     f"mounted off their table spots, a constant offset, not bending")
    # * 4. The fit itself: an assignment that fits as well, or a large mismatch nothing above explains.
    if bar.get("ambiguous"):
        lines.append("doubtful fit: another assignment of the balls fits as well")
    rms = bar.get("fit_rms")
    if rms is not None and rms > FIT_RMS_TOL_MM:
        lines.append(f"typical ball mismatch {rms:.1f} mm (above {FIT_RMS_TOL_MM:.1f}): "
                     + ("mostly the constant plate offsets above" if offsets
                        else "nothing above explains it: the fit itself is doubtful"))
    return lines or ["balls good, bar straight, fit tight"]


# * ================================================================ the robots

def record_robots(record: Mapping, models: RobotModels, process) -> list[dict]:
    """The robots of one record: every one it measured live, the others as the run last saved them."""
    out = []
    for live_id, robot in (record.get("robots") or {}).items():
        name = robot.get("name") or live_id.rsplit("/", 1)[-1]
        spec = process.robots.get(f"robots/{name}")
        if spec is None or not robot.get("base"):
            continue
        urdf = spec.urdf if Path(spec.urdf).is_absolute() else Path(process.folder or ".") / spec.urdf
        out.append(models.pose(name.capitalize(), Path(urdf), robot["base"], robot.get("joints") or {},
                               live=robot.get("source") == "live", source=robot.get("source") or "saved"))
    return out


# * ================================================================ the whole day

def collect_assembly(day: str, problem: str, drive_root: Path, process_folder: Path | None = None,
                     with_robots: bool = True) -> dict:
    """Read one day's as-built records and shape them for the page.

    Args:
        day: YYYYMMDD.
        problem: The process folder's name (the records' `problem`).
        drive_root: The Drive root the records are under.
        process_folder: The process folder (the seats, the schedule, the robots); default
            `<drive>/data_design_study/<problem>`.
        with_robots: Draw the robots (loads the URDF meshes).

    Returns:
        dict: The page's data without the environment and the page text: {problem, day, source_file, assembled,
        actions [...records...], history, ball_check, robot_models}.
    """
    path = as_built_path(drive_root, problem, day)
    records = sorted(read_records(path), key=lambda record: record.get("stamp_epoch") or 0.0)
    if not records:
        raise SystemExit(f"{path} holds no record.")
    process_folder = Path(process_folder) if process_folder else drive_root / "data_design_study" / problem
    process = read(process_folder)
    seats = assembled_bars(process)
    models = RobotModels() if with_robots else None
    actions = timeline(process)
    print(f"[assembly] {len(records)} records from {path}; process {problem}, {len(actions)} actions, "
          f"{len(seats)} bars.")
    history: dict = {}
    pairs_of_day: dict = {}
    placed: set = set()
    starts: dict = {}
    start_records: dict = {}
    last_of_action: dict = {}
    index_of = {action["action_id"]: action["index"] for action in actions}
    for record in records:
        action = record.get("action") or {}
        k = index_of.get(action.get("id"))
        if k is None:
            continue  # a click with nothing loaded: not on the slider
        step = record.get("step") or {}
        act = actions[k]
        key = (record.get("run_id"), k, step.get("index"))
        epoch = record.get("stamp_epoch") or 0.0
        if step.get("phase") == "start":
            starts[key] = epoch
        bars = []
        for entry in (record.get("bars") or {}).values():
            seat = seats.get(entry.get("bar_id"))
            if seat is None:
                continue
            bar = shape_bar(entry, record, seat)
            # * A bar is drawn from its first milestone on (mounted, then placed), in a step that ended well; before
            #   that it lies on its loading spot, metres away: only counted in its history.
            if bar["milestone"] not in (None, "bystander") and step.get("outcome") in (None, "ok"):
                placed.add(bar["bar"])
            if bar["bar"] not in placed:
                if bar["ok"]:
                    history.setdefault(bar["bar"], {"entries": [], "skipped": 0})["skipped"] += 1
                continue
            bars.append(bar)
        servo = dict(record["servo"]) if record.get("servo") else None
        if servo and servo.get("movement_id") != step.get("movement_id"):
            servo["note"] = (f"saved as {servo['movement_id']} (the servo after a transfer or an insert is named "
                             "after the step it aims at)")
        prev = last_of_action.get(k)
        redo = prev is not None and (prev[0] != record.get("run_id") or epoch - prev[1] > REDO_GAP_S)
        last_of_action[k] = (record.get("run_id"), epoch)
        tool0 = tool0_numbers(record)
        worst_tool0 = max((v["fk_vs_target"]["dpos_mm"] for v in tool0.values() if v["fk_vs_target"]), default=None)
        shaped = {
            "stamp": record.get("stamp"), "time": (record.get("stamp") or "T")[11:19], "run_id": record.get("run_id"),
            "robot": (record.get("robot") or "").capitalize(), "redo": redo, "note": None,
            "step": {"index": step.get("index"), "movement_id": step.get("movement_id"), "kind": step.get("routine"),
                     "phase": step.get("phase"), "outcome": step.get("outcome")},
            "milestone": next((b["milestone"] for b in bars if b["milestone"] not in (None, "bystander")), None),
            "bars": bars, "tool0": tool0, "tool0_worst": worst_tool0, "servo": servo,
            "load": load_numbers(record), "fk_note": (record.get("robot_state") or {}).get("fk_note"),
            "robots": record_robots(record, models, process) if models is not None else [],
        }
        if step.get("phase") == "manual":
            shaped["label"] = "recorded by hand"
        else:
            shaped["label"] = (MILESTONE_LABEL.get(shaped["milestone"])
                               or f"{(step.get('movement_id') or 'click').split('_', 2)[-1]} · {step.get('phase')}")
        shaped["options"] = options_of(record)
        if step.get("phase") == "start":
            start_records[key] = shaped
        elif step.get("phase") == "end" and key in start_records:
            start_records.pop(key)["options"] = shaped["options"]
        act.setdefault("records", []).append(shaped)
        for bar in bars:
            if not bar["ok"]:
                continue
            kept = add_history(history, bar, {
                "action": k, "record": len(act["records"]) - 1, "time": shaped["time"], "kind": act["kind"],
                "robot": act["robot"], "milestone": bar["milestone"], "placement_seat": bar.get("placement_seat"),
                "seat_start": bar.get("seat_start"), "seat_middle": bar.get("seat_middle"),
                "seat_end": bar.get("seat_end"), "rotation_seat": bar.get("rotation_seat"), "bend": bar.get("bend"),
                "outcome": step.get("outcome"), "fitted": bar.get("fitted")})
            if kept and bar.get("pairs"):
                pairs_of_day.setdefault(bar["bar"], []).append(bar["pairs"])
    # * The verdict on every measured bar, against what its balls did over the whole day.
    checks = {bar_id: ball_check(pairs) for bar_id, pairs in pairs_of_day.items()}
    for action in actions:
        action.setdefault("records", [])
        for record in action["records"]:
            for bar in record["bars"]:
                bar["findings"] = ball_findings(bar, checks.get(bar["bar"]))
        ok = [r for r in action["records"] if r["step"]["outcome"] in (None, "ok")] or action["records"]
        values = [b.get("placement") for b in (ok[-1]["bars"] if ok else []) if b.get("placement") is not None]
        action["worst"] = max(values) if values else None
    return {
        "problem": problem, "day": day, "source_file": str(path),
        "assembled": {bar_id: {"start": s["ends"][0], "end": s["ends"][1], "length": s["length"]}
                      for bar_id, s in seats.items()},
        "actions": actions, "history": history, "ball_check": checks,
        "robot_models": models.payload() if models is not None else {}, "monitor_log": None,
    }


# * The step controls and their style, put into the shared page template (`page.write_page`).
PAGE_CSS = """#steps { left:364px; right:14px; bottom:14px; width:auto; padding:9px 12px; }
#steps-head { display:flex; align-items:center; gap:8px; }
#steps-head button { width:auto; margin:0; padding:1px 8px; }
#action { flex:1; accent-color:#1763b8; }
#action-label { margin:5px 0 2px; font-size:12px; }
#options { margin:2px 0 4px; font-size:11px; line-height:1.55; }
#options .opt { display:block; padding-left:8px; }
#options .opt b { display:inline-block; min-width:66px; white-space:nowrap; }
#options .opt.used b { color:#1a7f37; }
#options .opt.unused b { color:#6b7686; }
#options .opt.note { color:#9a6700; padding-left:0; }
#records { display:flex; flex-wrap:wrap; gap:4px; margin:4px 0; }
.rec { width:auto; margin:0; padding:2px 8px; font-size:11px; border-radius:11px; }
.rec.on { background:#1763b8; color:#fff; border-color:#1763b8; }
.rec.cancelled { text-decoration:line-through; color:#b45309; }
.rec.on.cancelled { color:#fff; }
.rec small { color:var(--dim); font-size:10px; font-variant-numeric:tabular-nums; }
.rec.on small { color:#dbe7f7; }
.try { align-self:center; margin:0 2px 0 10px; font-size:10px; color:var(--dim); letter-spacing:.04em;
  text-transform:uppercase; white-space:nowrap; }
.try:first-child { margin-left:0; }
#steps-foot { display:flex; align-items:center; gap:10px; margin-top:4px; font-size:11px; color:var(--dim); }
#steps-foot label { display:flex; align-items:center; gap:4px; cursor:pointer; white-space:nowrap; }
#strip-by { width:auto; padding:1px 4px; font-size:11px; }
#timeline { display:flex; gap:1px; height:14px; margin-top:6px; }
#timeline .cell { flex:1; min-width:3px; border-radius:2px; cursor:pointer; }
#timeline .cell.cur { outline:2px solid #1d2430; outline-offset:1px; }
"""
PAGE_CONTROLS = """    <div class="card" id="steps">
      <div id="steps-head"><span class="muted">action</span>
        <button id="prev" title="previous action (left arrow)">&#9664;</button>
        <input type="range" id="action" min="0" max="0" value="0">
        <button id="next" title="next action (right arrow)">&#9654;</button></div>
      <div id="action-label"></div>
      <div id="records"></div>
      <div id="options"></div>
      <div id="steps-foot"><label><input type="checkbox" id="all-records"> all records</label>
        <span>[ and ] step through them</span><span style="margin-left:auto">strip by</span>
        <select id="strip-by"></select></div>
      <div id="timeline"></div>
    </div>
"""
