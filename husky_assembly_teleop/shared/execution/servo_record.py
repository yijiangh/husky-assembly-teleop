"""
The servo's record: the JSON payload of one run (spec §2.9) and its four-panel PNG (W:2685-2816).

Pure: no ROS, no context. `plot_servo` runs on assembly_run's writer thread, so it draws with matplotlib's `Figure` +
`FigureCanvasAgg`, never pyplot: pyplot's figure manager starts a GUI loop of its own (W:2727-2731).
"""

from __future__ import annotations

from math import degrees
from pathlib import Path
from typing import Mapping, Sequence

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from .servo import (APPROACH, BAR_GATE, BAR_MAX_RMS, BAR_TOL, CONFIRM_TRAVEL, GROUNDED_END_TOL, MARKER_MAX_AGE,
                    MAX_PASSES, MIN_TIME, SETTLE, SPEED, TOOL0_TOL, ServoPlan)

# * Line colours (RGB 0-255), per family x, y, z, |d|, light -> dark so |d| reads as the bold summary line: the first
#   arm reds, the second greens, the base blues (W:116-118).
LEFT_ARM_RGB = ((240, 130, 120), (225, 70, 55), (150, 20, 20), (90, 0, 0))
RIGHT_ARM_RGB = ((150, 220, 140), (70, 180, 80), (20, 130, 40), (0, 70, 20))
BASE_RGB = ((150, 190, 230), (70, 130, 200), (30, 80, 160), (10, 40, 100))

#: Where the bar's frame sits in the record, and where its deviations are evaluated (spec D3).
BAR_FRAME = {"origin": "middle", "measured_at": "start (T·Trans(0,0,−L/2))"}


def servo_payload(plan: ServoPlan, *, robot: str, serial: str, readings: Sequence[Mapping], moves: Sequence[Mapping],
                  status: str, reason: str, engaged: bool, dry_run: bool) -> dict:
    """The JSON record of one servo run: the old monitor's keys (W:2719-2722) and the new ones (spec §2.9).

    Args:
        plan: The servo's plan.
        robot: The acting robot's id, e.g. "robots/a200-0806".
        serial: Its serial.
        readings: One dict per reading: ``iter``, ``tool0``, ``base``, ``wrench``, ``force_change_N``, ``track_mm``,
            ``bar`` (`bar_reading`) when measured.
        moves: One dict per pass: ``planner``, ``attempts``, ``rows``, ``ik``, ``correction_mm``, ``excursion_deg``,
            ``net_deg``, ``traj_time``, ``duration``, ``prompted``, ``outcome``.
        status: How the run ended (the part's result).
        reason: Why, in plain words.
        engaged: The loop steered on the MoCap bar (a usable fit came), so the record is a bar-mode one (W:2669-2672).
        dry_run: The moves were built, never sent.

    Returns:
        dict: Plain lists, dicts, numbers and strings.
    """
    bars = [reading["bar"] for reading in readings if reading.get("bar") is not None]
    if plan.kind == APPROACH:
        planner_mode = "single_arm_joint"
    else:
        planner_mode = "transfer+mocap_bar" if engaged else "transfer"
    return {
        "planner_mode": planner_mode, "movement_id": plan.target,
        "servo_feedback": "mocap_bar" if engaged else "tool0",
        "bar_name": bars[0]["bar"] if bars else None,
        "servoing_data": list(readings),
        "robot": robot, "serial": serial, "action": plan.action, "kind": plan.kind,
        "bar_frame": {**BAR_FRAME, "length_m": None if plan.layout is None else plan.layout.length},
        "settings": {
            "max_passes": MAX_PASSES, "tool0_tol_mm": TOOL0_TOL * 1e3,
            "bar_tol_mm_deg": [BAR_TOL[0] * 1e3, degrees(BAR_TOL[1])],
            "bar_gate_mm_deg": [BAR_GATE[0] * 1e3, degrees(BAR_GATE[1])],
            "bar_max_rms_mm": BAR_MAX_RMS * 1e3, "max_force_change_N": plan.max_force_change,
            "grounded_end_tol_mm": GROUNDED_END_TOL * 1e3, "confirm_travel_deg": degrees(CONFIRM_TRAVEL),
            "speed_deg_s": degrees(SPEED), "min_time_s": MIN_TIME, "settle_s": SETTLE,
            "marker_max_age_s": MARKER_MAX_AGE, "by_bar": plan.by_bar, "why_tool0": plan.why_tool0,
            "linear_only": plan.linear_only},
        "moves": list(moves),
        "outcome": {"status": status, "reason": reason, "passes": len(moves)},
        "dry_run": dry_run,
    }


def plot_servo(payload: Mapping, path: Path) -> None:
    """Draw a run's four panels into a PNG: tool0 position and orientation error, base drift in both (W:2725-2816).

    Args:
        payload: The record (`servo_payload`), with at least one reading.
        path: The PNG to write.
    """
    data = payload["servoing_data"]
    iters = [reading["iter"] for reading in data]
    axis_names = ("x", "y", "z")
    # The arms the record has: left / right on Cindy, one "arm" on a support robot.
    sides = list(data[0]["tool0"].keys())
    side_rgb = dict(zip(sides, (LEFT_ARM_RGB, RIGHT_ARM_RGB)))
    bar_records = [reading for reading in data if reading.get("bar") is not None]
    bar_iters = [reading["iter"] for reading in bar_records]

    def arm_color(side: str, index: int) -> tuple:
        return tuple(value / 255.0 for value in side_rgb[side][index])

    def base_color(index: int) -> tuple:
        return tuple(value / 255.0 for value in BASE_RGB[index])

    def bar_curve(key: str, field: str) -> list:
        # One number per bar reading; NaN (a gap in the line) where there was no MoCap fit.
        return [reading["bar"][key][field] if reading["bar"].get(key) is not None else np.nan
                for reading in bar_records]

    fig = Figure(figsize=(13, 9))
    FigureCanvasAgg(fig)
    axes = fig.subplots(2, 2)

    # * Tool0 position error, per axis + euclidean norm, every arm (mm); the held bar vs its target: MoCap dashed,
    #   FK dotted.
    for side in sides:
        for index, axis_name in enumerate(axis_names):
            axes[0, 0].plot(iters, [reading["tool0"][side]["pos_err_mm"][index] for reading in data], "o-",
                            color=arm_color(side, index), label=f"{side.capitalize()} {axis_name}")
        axes[0, 0].plot(iters, [reading["tool0"][side]["pos_norm_mm"] for reading in data], "o-", linewidth=2.2,
                        color=arm_color(side, 3), label=f"{side.capitalize()} |d|")
    if bar_records:
        axes[0, 0].plot(bar_iters, bar_curve("mocap_vs_design", "dpos_mm"), "o--", color="k",
                        label="bar dpos (MoCap vs design)")
        axes[0, 0].plot(bar_iters, bar_curve("fk_vs_design", "dpos_mm"), "o:", color="0.45",
                        label="bar dpos (FK vs design)")
    axes[0, 0].set_title("Tool0 position error (per axis + |d|)")
    axes[0, 0].set_xlabel("iteration")
    axes[0, 0].set_ylabel("mm")
    axes[0, 0].legend(fontsize=7, ncol=2)
    axes[0, 0].grid(True)

    # * Tool0 orientation error, per axis, every arm (deg).
    for side in sides:
        for index, axis_name in enumerate(axis_names):
            axes[0, 1].plot(iters, [reading["tool0"][side]["rot_err_deg"][index] for reading in data], "o-",
                            color=arm_color(side, index), label=f"{side.capitalize()} {axis_name}")
    if bar_records:
        axes[0, 1].plot(bar_iters, bar_curve("mocap_vs_design", "dang_deg"), "o--", color="k",
                        label="bar dang (MoCap vs design)")
        axes[0, 1].plot(bar_iters, bar_curve("fk_vs_design", "dang_deg"), "o:", color="0.45",
                        label="bar dang (FK vs design)")
    axes[0, 1].set_title("Tool0 orientation error (per axis)")
    axes[0, 1].set_xlabel("iteration")
    axes[0, 1].set_ylabel("deg")
    axes[0, 1].legend(fontsize=7, ncol=2)
    axes[0, 1].grid(True)

    # * Mobile-base position drift vs the run's start, per axis + euclidean norm (mm).
    for index, axis_name in enumerate(axis_names):
        axes[1, 0].plot(iters, [reading["base"]["pos_diff_mm"][index] for reading in data], "o-",
                        color=base_color(index), label=axis_name)
    axes[1, 0].plot(iters, [reading["base"]["pos_norm_mm"] for reading in data], "o-", linewidth=2.2,
                    color=base_color(3), label="|d|")
    axes[1, 0].set_title("Base position drift vs run start (per axis + |d|)")
    axes[1, 0].set_xlabel("iteration")
    axes[1, 0].set_ylabel("mm")
    axes[1, 0].legend(fontsize=8)
    axes[1, 0].grid(True)

    # * Mobile-base orientation drift vs the run's start, per axis (deg).
    for index, axis_name in enumerate(axis_names):
        axes[1, 1].plot(iters, [reading["base"]["rot_diff_deg"][index] for reading in data], "o-",
                        color=base_color(index), label=axis_name)
    axes[1, 1].set_title("Base orientation drift vs run start (per axis)")
    axes[1, 1].set_xlabel("iteration")
    axes[1, 1].set_ylabel("deg")
    axes[1, 1].legend(fontsize=8)
    axes[1, 1].grid(True)

    fig.tight_layout()
    fig.savefig(path)
