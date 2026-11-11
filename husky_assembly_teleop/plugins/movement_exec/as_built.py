"""
The as-built record of one moment: every MoCap bar measured, where each should be, the robot, its load, the options.

`as_built_record` gathers it on the main thread (inside the exec task, between parts, or on the "record now"
click) from the mocap_bars plugin's fits and the step's context; `shared/runs/as_built.py` holds the pure pieces and the
file. The record is handed to assembly_run (`request_record`, kind "as_built"), which appends it to the day's file on
the Drive.
"""

from __future__ import annotations

from dataclasses import asdict
from typing import TYPE_CHECKING, Any, Mapping

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.robot import FLANGE_SUFFIX

from ...shared.runs.as_built import ACTION_KIND, build_record, milestone_of, reference_of
from ...shared.assembly_planning.joints import arm_joint_names, arm_side
from ...shared.assembly_planning.movement import Routine, routine_or_none
from ...shared.assembly_run_contract import is_live, live_record
from ...shared.bars.layouts import bar_layouts
from ...shared.bars.pose import pose_deviation
from ...shared.execution.servo import flange_poses, tool0_errors, wrench_magnitudes
from ...shared.execution.spiral import spiral_fits
from ...shared.movement_exec_contract import DONE, INSERT_PART, SERVO_INDEX, SERVO_PART
from ...shared.runs.runs import stamp

#: The transfer's two routines: the workaround (a free move, the bar mounted after) and the held one.
TRANSFER_ROUTINES = (Routine.TRANSFER_FREE, Routine.TRANSFER_HELD)

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.robot import HuskyRobotInterface
    from ...shared.assembly_planning.process_steps import LoadedProcess
    from ...shared.assembly_run_contract import RunView
    from ...shared.mocap_bars_contract import MocapBarsContract
    from ...shared.movement_exec_contract import ExecRequest
    from .sequences import PartRun


def as_built_record(ctx: PluginContext, bars_plugin: MocapBarsContract, loaded: LoadedProcess, view: RunView, *,
                    action_id: str | None, movement_index: int | None, serial: str, robot: HuskyRobotInterface,
                    phase: str, request: ExecRequest | None = None, run: PartRun | None = None,
                    dry_run: bool = False, ft_zero: Mapping[str, Mapping] | None = None) -> dict | None:
    """One as-built record now, or None when the process has no bar with markers.

    Args:
        ctx: The plugin's context (the clock, the snapshot).
        bars_plugin: The mocap_bars plugin: its `measure_now` fits every bar from the newest marker cloud.
        loaded: The loaded process.
        view: The run, for its folder, settings and the other robots' saved ends.
        action_id: The action, or None.
        movement_index: The part's movement in the action, or None.
        serial: The recording robot.
        robot: Its interface, for the wrenches.
        phase: "start", "end" or "manual".
        request: The request being run, or None on a click.
        run: The part just run (for "end": how it ended), the part about to run (for "start"), or None.
        dry_run: Whether the request sends nothing.
        ft_zero: Serial -> {at, reason} of the force sensors' last zero.
    """
    process = loaded.process
    layouts = bar_layouts(process)
    if not layouts:
        return None
    action = process.actions.get(action_id) if action_id else None
    part = run.part if run is not None else None
    servo_bar = None
    if run is not None and run.servo_plan is not None and run.servo_plan.bar is not None \
            and run.servo_plan.design_bar is not None:
        plan = run.servo_plan
        servo_bar = (plan.bar, plan.design_bar, plan.holder)
    # * Where every bar should be, first: it is the fit's hint too (~1 ms a bar instead of a 20 ms search).
    references = {bar: reference_of(process, action_id, movement_index, bar, phase, servo_bar) for bar in layouts}
    hints = {bar: Pose.from_matrix(T) if T is not None else process.bodies[bar].assembly_pose
             for bar, (T, _kind, _held, _grasp) in references.items()}
    measurements = bars_plugin.measure_now(hints)
    insert_index = next((k for k, movement in enumerate(action.movements)
                         if routine_or_none(action, movement) is Routine.INSERT), None) if action is not None else None
    last_part = request is not None and part is not None and part is request.parts[-1]
    after_insert = (insert_index is not None and movement_index is not None and movement_index > insert_index)
    bars = {}
    for bar, found in measurements.items():
        T_reference, kind, held_by, T_grasp = references[bar]
        T_assembled = process.bodies[bar].assembly_pose.matrix()
        fit = found.fit
        ok = fit.ok and fit.T_world_from_bar is not None
        entry = found.to_json()
        entry.update({
            "bar_id": bar.rsplit("/", 1)[-1], "bar_fit": entry.pop("fit"), "T_world_from_bar": entry.pop("pose"),
            "marker_source": found.source, "marker_age_s": found.sensed.age if found.sensed is not None else None,
            "held_by": held_by, "T_link_from_bar": T_grasp, "reference_kind": kind, "T_reference": T_reference,
            "T_assembled": T_assembled,
            "deviation_vs_reference": (pose_deviation(fit.T_world_from_bar, T_reference)
                                       if ok and T_reference is not None else None),
            "deviation_vs_assembled": pose_deviation(fit.T_world_from_bar, T_assembled) if ok else None,
            "milestone": milestone_of(action.type if action is not None else None,
                                      part.routine if part is not None else None,
                                      part.kind if part is not None else None, phase,
                                      action is not None and bar == action.bar, last_part, after_insert)})
        bars[entry["bar_id"]] = entry
    robot_name = request.robot_name if request is not None else _process_name(process, serial)
    snapshot = ctx.scene.snapshot
    live = snapshot.robots.get(robot_id(serial))
    movement = action.movements[movement_index] if action is not None and movement_index is not None else None
    targets = _targets(action, movement, run)
    arms = list(robot.arms)
    flanges = [f"{action.robot if action is not None else 'robots/' + robot_name}/{arm}{FLANGE_SUFFIX}"
               for arm in arms]
    fk = flange_poses(live, flanges) if live is not None and live.base_tracked else {}
    robot_state = {
        "base_tracked": live is not None and live.base_tracked,
        "base_pose": live.base if live is not None and live.base_tracked else None,
        "arms": {arm_side(arm): {"arm": arm, "flange": flange, "joint_names": arm_joint_names(arm),
                                 "joints": [live.joints.get(name) if live is not None else None
                                            for name in arm_joint_names(arm)],
                                 "tool0_fk": fk.get(flange), "tool0_target_end": targets.get(flange)}
                 for arm, flange in zip(arms, flanges)},
        "fk_note": None if fk else "the base is not tracked by mocap, so tool0 is not placed in the world"}
    tool0 = tool0_errors(fk, {flange: targets[flange] for flange in fk if flange in targets}) if targets else {}
    wrenches = wrench_magnitudes(robot, arms)
    load = None if wrenches is None else {
        "arms": {side: {**wrench, "wrench": robot.arms[arm].state.wrench}
                 for (side, wrench), arm in zip(wrenches.items(), arms)},
        "zero": dict((ft_zero or {}).get(serial) or {}) or None}
    robots = {live_id: {"name": _process_name(process, live_id.rsplit("/", 1)[-1]),
                        "serial": live_id.rsplit("/", 1)[-1], **asdict(live_record(each, stamp()))}
              for live_id, each in snapshot.robots.items() if is_live(each, ctx.now())}
    step = None
    if part is not None and request is not None:
        outcome = None
        if phase == "end":
            outcome = "ok" if run.result == DONE else run.result
        step = {"index": request.step.primary if part.index == SERVO_INDEX else part.index,
                "movement_id": part.movement, "routine": part.routine.value if part.routine is not None else None,
                "part": {"index": part.index, "kind": part.kind, "also": list(part.also), "tools": list(part.tools),
                         "text": part.text, "last": last_part},
                "step": {"primary": request.step.primary, "indices": list(request.step.indices)},
                "phase": phase, "outcome": outcome, "status": run.status, "reason": run.reason, "dry_run": dry_run}
    elif phase == "manual" and movement is not None:
        step = {"index": movement_index, "movement_id": movement.id,
                "routine": (routine_or_none(action, movement) or Routine.OPERATOR).value, "part": None, "step": None,
                "phase": "manual", "outcome": None, "status": "", "reason": "recorded by a click", "dry_run": False}
    schedule = list(process.schedule)
    return build_record(
        problem=loaded.source.name, run_id=folder.name if (folder := getattr(view, "folder", None)) else "",
        robot=robot_name,
        serial=serial,
        action=None if action is None else {"index": schedule.index(action.id) if action.id in schedule else None,
                                            "id": action.id, "type": action.type,
                                            "kind": ACTION_KIND.get(action.type), "bar": action.bar,
                                            "bar_id": action.bar.rsplit("/", 1)[-1], "robot": action.robot},
        step=step, instrumented_bars=[{"bar": bar, "bar_id": bar.rsplit("/", 1)[-1]} for bar in layouts],
        bars=bars, robot_state=robot_state, tool0=tool0, load=load, servo=run.servo if run is not None else None,
        options=_options(request, run, view, dry_run), robots=robots, settings=asdict(view.settings))


def _process_name(process, serial: str) -> str:
    """The process's name for a robot by its serial ("cindy" for a200-0806); the serial when none matches."""
    for spec in process.robots.values():
        if spec.serial and serial.endswith(spec.serial):
            return spec.id.rsplit("/", 1)[-1]
    return serial


def _targets(action, movement, run: PartRun | None) -> dict[str, Pose]:
    """Flange id -> the authored pose the part aims at: the servo's targets, else the movement's target link poses."""
    if run is not None and run.servo_plan is not None:
        return dict(run.servo_plan.targets)
    if action is None or movement is None or movement.target is None:
        return {}
    own = movement.target.robots.get(action.robot)
    return dict(own.link_poses) if own is not None else {}


def _options(request: ExecRequest | None, run: PartRun | None, view: RunView, dry_run: bool) -> dict[str, Any]:
    """The settings and tick boxes in force at the record: what the page lists per step."""
    settings = view.settings
    part = run.part if run is not None else None
    plan = (run.command or {}).get("plan") if run is not None and isinstance(run.command, dict) else None
    plan = plan if isinstance(plan, dict) else {}
    insert_mode = None
    if part is not None and part.kind == INSERT_PART:
        if plan.get("grounded") or not part.tools:
            insert_mode = "rigid"
        else:
            insert_mode = "compliant-two-mate" if len(plan.get("mating") or part.tools) >= 2 else "compliant-one-mate"
    servo = run.servo if run is not None else None
    servo_plan = run.servo_plan if run is not None else None
    return {
        "transfer_as_free": {"box": request.transfer_as_free() if request is not None else None,
                             "applies": part is not None and part.routine in TRANSFER_ROUTINES,
                             "used": part is not None and part.routine is Routine.TRANSFER_FREE},
        "servo_by_bar": {"box": settings.servo_by_bar,
                         "applies": part is not None and part.kind == SERVO_PART,
                         "used": bool(servo and servo.get("engaged")),
                         "why": servo_plan.why_tool0 if servo_plan is not None else None},
        "spiral": {"box": settings.spiral, "fits": part is not None and part.kind == INSERT_PART
                   and spiral_fits(part.tools), "applies": part is not None and part.kind == INSERT_PART,
                   "used": bool(plan.get("spiral"))},
        "insert_mode": insert_mode, "dry_run": dry_run,
        "ignore_built": request.ignore_built() if request is not None else None,
        "obstacles_collide": request.obstacles_collide() if request is not None else None,
        "insert_split_mm": settings.insert_split_mm, "spiral_seconds": settings.spiral_seconds,
        "spiral_turns": settings.spiral_turns}
