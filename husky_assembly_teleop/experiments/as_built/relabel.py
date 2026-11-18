"""Read a bar back from the Motive asset it was streamed under by mistake.

When a bar's asset was given another bar's number in Motive (B25's balls streamed as 1039, B39's number), the records
hold its balls under the wrong bar, fitted against the wrong pattern. `relabel_assets` refits them against the right
bar's pattern and puts them under its name, the way the monitor would have recorded them; the file stays as it is.
"""

from __future__ import annotations

from typing import Mapping

import numpy as np

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.process import Process

from ...shared.assembly_planning.movement import Routine, routine_or_none
from ...shared.bars.layouts import bar_layouts, fit_bar
from ...shared.bars.pose import pose_deviation
from ...shared.runs.as_built import json_safe, milestone_of, reference_of
from ...world.marker_match import Sensed


def parse_assets(texts: list[str] | None) -> dict[str, int]:
    """Command-line `B25=1039` pairs as {"B25": 1039}.

    Raises:
        ValueError: A pair is not `<bar>=<asset number>`.
    """
    out = {}
    for text in texts or []:
        bar, _, asset = text.partition("=")
        if not bar or not asset.strip().isdigit():
            raise ValueError(f"{text!r}: give a bar and the asset it was streamed as, e.g. B25=1039")
        out[bar.strip()] = int(asset)
    return out


def relabel_assets(records: list[dict], assets: Mapping[str, int], process: Process) -> int:
    """Refit, in place, the balls streamed under each given asset as the bar named for it.

    Args:
        records: The day's records, as read from the file.
        assets: Bar id -> the Motive asset its balls were streamed as, e.g. {"B25": 1039}.
        process: The process, for the bars' patterns, targets and milestones.

    Returns:
        int: How many bar entries were moved.
    """
    layouts = bar_layouts(process)
    moved = 0
    for record in records:
        bars = record.get("bars") or {}
        for bar_id, asset in assets.items():
            key = next((k for k, entry in bars.items() if entry.get("mocap_id") == asset and k != bar_id), None)
            if key is None or f"bars/{bar_id}" not in layouts:
                continue
            bars[bar_id] = _refit(bars.pop(key), f"bars/{bar_id}", layouts[f"bars/{bar_id}"], process, record)
            moved += 1
    return moved


def _refit(entry: Mapping, bar: str, layout, process: Process, record: Mapping) -> dict:
    """One record's entry of `bar`, from another bar's entry holding its balls (as `as_built_record` builds it)."""
    sensed_json = entry.get("sensed") or {}
    action_json = record.get("action") or {}
    step = record.get("step") or {}
    part = step.get("part") or {}
    action_id = action_json.get("id")
    movement_index = step.get("index")
    phase = step.get("phase") or "manual"
    T_reference, kind, held_by, T_grasp = reference_of(process, action_id, movement_index, bar, phase)
    T_assembled = process.bodies[bar].assembly_pose.matrix()
    fit = None
    if sensed_json.get("points"):
        residuals = sensed_json.get("residuals")
        sensed = Sensed(points=np.asarray(sensed_json["points"], dtype=float), ids=tuple(sensed_json.get("ids") or ()),
                        residuals=None if residuals is None else np.asarray(residuals, dtype=float),
                        age=float(sensed_json.get("age") or 0.0), source=sensed_json.get("source") or "asset",
                        dropped=int(sensed_json.get("dropped") or 0), occluded=int(sensed_json.get("occluded") or 0),
                        notes=tuple(sensed_json.get("notes") or ()))
        fit = fit_bar(sensed, layout, hint=T_reference if T_reference is not None else T_assembled)
    ok = fit is not None and fit.ok and fit.T_world_from_bar is not None
    action = process.actions.get(action_id) if action_id else None
    movement = action.movements[movement_index] if action is not None and movement_index is not None else None
    insert_index = next((k for k, each in enumerate(action.movements)
                         if routine_or_none(action, each) is Routine.INSERT), None) if action is not None else None
    milestone = milestone_of(action.type if action is not None else None,
                             routine_or_none(action, movement) if movement is not None else None,
                             part.get("kind"), phase, action is not None and action.bar == bar,
                             bool(part.get("last")),
                             insert_index is not None and movement_index is not None and movement_index > insert_index)
    out = dict(entry)
    out.update({
        "bar": bar, "bar_id": bar.rsplit("/", 1)[-1],
        "bar_fit": json_safe(fit) if fit is not None else entry.get("bar_fit"),
        "T_world_from_bar": json_safe(Pose.from_matrix(fit.T_world_from_bar)) if ok else None,
        "layout": {"names": list(layout.names), "points": np.asarray(layout.points).tolist(), "length": layout.length},
        "planned": json_safe(process.bodies[bar].assembly_pose), "deviation": None,
        "held_by": held_by, "T_link_from_bar": json_safe(T_grasp), "reference_kind": kind,
        "T_reference": json_safe(T_reference), "T_assembled": json_safe(T_assembled),
        "deviation_vs_reference": (json_safe(pose_deviation(fit.T_world_from_bar, T_reference))
                                   if ok and T_reference is not None else None),
        "deviation_vs_assembled": json_safe(pose_deviation(fit.T_world_from_bar, T_assembled)) if ok else None,
        "milestone": milestone, "relabelled_from": entry.get("bar_id")})
    return out
