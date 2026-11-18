"""
The servo's numbers: what a step's servo aims at, each reading's errors, the mocap bar's correction, the stop tests.

Pure: any thread, no ROS.

- `servo_kind` / `servo_plan`: whether a step's primary movement gets a servo, and what it aims at (`ServoPlan`).
- `flange_poses`, `tool0_errors`, `base_drift`, `wrench_magnitudes`, `force_vectors`, `tracking_gaps`: one reading.
- `bar_reading`, `bar_on_target`, `bar_targets`: the held bar by mocap, and the flange targets corrected by it.
- `held_targets`: the targets one rigid move of the held bar can reach, when steering by tool0.
- `excursion`, `recommended_time`: how far a pass travels, and how long it should take.
- `stop_after_move`: whether the loop is done, in the old order.
- `carried_insert_goals`, `insert_end_gap`: the correction carried into the insert, and the gate after it.

! Metres and radians throughout, the thresholds and `excursion` included; the old mm and degrees only in texts and in
  the record keys that say so (`pos_err_mm`, `dang_deg`).
"""

from __future__ import annotations

from dataclasses import dataclass
from math import degrees, radians
from typing import TYPE_CHECKING, Iterable, Mapping, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose
from scaffolding_assembly_core.ids import split_link_id
from scaffolding_assembly_core.kinematics import link_pose
from scaffolding_assembly_core.robot import FLANGE_SUFFIX

from ..bars.pose import apply_authored_move, bar_servo_correction, bar_start, pose_deviation
from ..assembly_planning.joints import arm_of_flange, arm_side
from ..assembly_planning.movement import Routine, classify, routine_or_none
from .force_sensors import force_magnitude

if TYPE_CHECKING:
    from scaffolding_assembly_core.process import Action
    from scaffolding_assembly_core.robot import RobotObject

    from ...robot_interface.robot import HuskyRobotInterface
    from ...world.marker_match import BarFit
    from ..bars.layouts import BarLayout
    from ..assembly_planning.process_steps import LoadedProcess
    from ..runs.runs import RunSettings

# * ================================================================ thresholds

#: Passes at most, and the tool0 position under which the loop is converged or has nothing left to plan.
MAX_PASSES = 8
TOOL0_TOL = 0.0002
#: The mocap bar is on its target within this of the design: metres, radians.
BAR_TOL = (0.0005, radians(0.1))
#: ! A mocap bar further than this from the design is a wrong match (another bar, a stray marker), never steered on.
BAR_GATE = (0.030, radians(3.0))
#: A fit whose markers sit further than this (rms) from the design markers is not trusted.
BAR_MAX_RMS = 0.003
#: After a ground bar's insert: the loop stops when a flange's force vector changed by more than this, newtons.
MAX_FORCE_CHANGE = 20.0
#: A ground bar's insert counts as finished, so the servo after it may run, when both flanges stand within this of its
#: planned end, metres (`insert_end_gap`).
GROUNDED_END_TOL = 0.003
#: A flange further than this from its target after the best rigid move of the held bar gets a warning, metres: what
#: is left is the flanges' relation differing from the design's, which no held move can change.
HELD_LEFTOVER_WARN = 0.0005
#: A later pass travelling further than this (its excursion) asks the operator first, radians.
CONFIRM_TRAVEL = radians(5.0)
#: Joint speed a move is timed for, radians per second, and the shortest move, seconds.
SPEED = radians(3.0)
MIN_TIME = 3.0
#: Wait after an arm move before measuring: the base settles in the mocap stream, seconds.
SETTLE = 2.0
#: A marker sample older than this is not fitted, seconds.
MARKER_MAX_AGE = 0.5

#: The servo kinds: after a transfer (onto the insert's start), after a support robot's approach, after the insert of
#: a ground bar.
TRANSFER, APPROACH, AFTER_INSERT = "transfer", "approach", "after_insert"
#: How `stop_after_move` ends the loop.
#: ! The same strings as `movement_exec_contract`'s DONE and FAILED, which imports `servo_kind` from here.
DONE, FAILED = "done", "failed"


@dataclass(frozen=True)
class ServoPlan:
    """What one step's servo aims at, decided once per run (`servo_plan`).

    Attributes:
        kind: TRANSFER, APPROACH or AFTER_INSERT.
        action: The action's id.
        primary: Index of the step's primary movement in the action.
        target: Id of the movement whose target link poses are the authored flange targets F* (the primary).
        step_index: Index into `loaded.steps` of the movement whose planning scene the servo plans in: the insert
            after a transfer, else the primary itself.
        flanges: The moving arms' flange link ids, e.g. "robots/cindy/left_ur_arm_tool0", in the movement's order.
        arms: Their arm names, e.g. "left_ur_arm", same order.
        targets: Flange id -> its authored world target F*.
        held: The bar is in the tools (two arms): TRANSFER and AFTER_INSERT.
        bar: The held bar's body id, or None.
        holder: The bar's first holder as (flange id, grasp: the bar pose in that flange's frame), or None.
        layout: The bar's marker layout, or None when it carries no markers (or no bar is held).
        design_bar: The bar pose the servo aims at, B* = F*_holder . grasp, or None.
        grounded: The bar stands on the ground at its insert.
        by_bar: Steer on the mocap bar once a usable fit comes; else tool0 on `targets` throughout.
        why_tool0: Why not by the bar, in plain words; empty when `by_bar`.
        linear_only: Straight lines only: after a ground bar's insert the bar stands on the floor.
        max_force_change: The load guard's limit, newtons (after a ground bar's insert), or None: no guard.
    """

    kind: str
    action: str
    primary: int
    target: str
    step_index: int
    flanges: tuple[str, ...]
    arms: tuple[str, ...]
    targets: dict[str, Pose]
    held: bool
    bar: str | None
    holder: tuple[str, Pose] | None
    layout: BarLayout | None
    design_bar: Pose | None
    grounded: bool
    by_bar: bool
    why_tool0: str
    linear_only: bool
    max_force_change: float | None


# * ================================================================ which servo, and what it aims at

def servo_kind(loaded: LoadedProcess, action_id: str, primary: int, transfer_as_free: bool) -> str | None:
    """Which servo a step's primary movement gets, by its routine; None when it gets none.

    Args:
        loaded: The loaded process.
        action_id: The action.
        primary: Index of the step's primary movement in the action.
        transfer_as_free: The transfer workaround (both transfer routines give TRANSFER).

    Returns:
        str | None: TRANSFER after a transfer, APPROACH after a support robot's approach, AFTER_INSERT after the
        insert of a ground bar (whether or not "servo by bar" is on); else None.
    """
    action = loaded.process.actions[action_id]
    spec = classify(loaded.process, action, action.movements[primary], transfer_as_free)
    if spec.routine in (Routine.TRANSFER_FREE, Routine.TRANSFER_HELD):
        return TRANSFER
    if spec.routine is Routine.SUPPORT_APPROACH:
        return APPROACH
    if spec.routine is Routine.INSERT and spec.drives.grounded:
        return AFTER_INSERT
    return None


def servo_plan(loaded: LoadedProcess, action_id: str, primary: int, settings: RunSettings,
               layouts: Mapping[str, BarLayout]) -> ServoPlan | str:
    """What the servo after a step's primary movement aims at, or why there is none, in plain words.

    * The targets are the primary's own authored link poses: after a transfer the insert's start, after an insert its
      end, after an approach the approach pose. The planning scene is the insert's after a transfer (the bar is then
      in the tools, as at the insert's start), else the primary's.

    Args:
        loaded: The loaded process.
        action_id: The action.
        primary: Index of the step's primary movement in the action.
        settings: The run's settings (`servo_by_bar`).
        layouts: Bar id -> marker layout (`shared/bars/layouts.bar_layouts`).

    Returns:
        ServoPlan | str: The plan, or why there is none: no servo after this movement, no target pose for an arm, no
        insert after the transfer, the bar not held by a moving arm.
    """
    process = loaded.process
    action = process.actions[action_id]
    movement = action.movements[primary]
    # Both transfer routines give TRANSFER, so the workaround flag does not matter here.
    kind = servo_kind(loaded, action_id, primary, transfer_as_free=False)
    if kind is None:
        return f"{movement.id}: no servo after it"
    spec = classify(process, action, movement)
    own = movement.target.robots.get(action.robot) if movement.target is not None else None
    link_poses = own.link_poses if own is not None else {}
    missing = [part.flange for part in spec.arms if part.flange not in link_poses]
    if missing:
        return f"{movement.id} gives no target pose for {', '.join(missing)}"
    targets = {part.flange: link_poses[part.flange] for part in spec.arms}
    # * After a transfer: the insert's planning scene, and whether its bar stands on the ground.
    insert = primary
    if kind == TRANSFER:
        insert = next((k for k in range(primary + 1, len(action.movements))
                       if routine_or_none(action, action.movements[k]) is Routine.INSERT), None)
        if insert is None:
            return f"no insert after {movement.id} in {action_id}"
    grounded = (spec if insert == primary else classify(process, action, action.movements[insert])).drives.grounded
    held = kind in (TRANSFER, AFTER_INSERT)
    bar = holder = layout = design_bar = None
    if held:
        bar = action.bar
        holders = movement.start.attached.get(bar, ())
        if not holders or holders[0].to not in targets:
            return f"{bar} is not held by a moving arm at the start of {movement.id}"
        holder = (holders[0].to, holders[0].grasp)
        layout = layouts.get(bar)
        design_bar = compose(targets[holder[0]], holder[1])  # B* = F*_h . G
    # * Steer on the mocap bar only for a ground bar held in the tools, with markers to fit.
    if not settings.servo_by_bar:
        why_tool0 = "'Servo by bar' is off"
    elif not held:
        why_tool0 = "this loop plans free moves (no bar held)"
    elif not grounded:
        why_tool0 = f"{movement.id} does not hold a ground bar (a normal bar must mate with its joints)"
    elif layout is None:
        why_tool0 = f"{bar} carries no mocap markers"
    else:
        why_tool0 = ""
    return ServoPlan(kind, action_id, primary, movement.id, loaded.index_of(action_id, insert),
                     tuple(part.flange for part in spec.arms), tuple(part.arm for part in spec.arms), targets, held,
                     bar, holder, layout, design_bar, grounded, by_bar=not why_tool0, why_tool0=why_tool0,
                     linear_only=kind == AFTER_INSERT,
                     max_force_change=MAX_FORCE_CHANGE if kind == AFTER_INSERT else None)


# * ================================================================ one reading

def flange_side(flange: str) -> str:
    """The side of a flange id's arm: "left", "right", or "arm" on a one-arm robot (`joints.arm_side`)."""
    return arm_side(arm_of_flange(split_link_id(flange)[1]))


def flange_poses(robot: RobotObject, flanges: Iterable[str]) -> dict[str, Pose]:
    """Flange id -> its world pose at the robot's measured base and joints: F_s = B . FK_s(q_s).

    Only the link name of a flange id ("robots/cindy/left_ur_arm_tool0") is read, so a process id names the live
    robot's link.
    """
    return {flange: link_pose(robot.model.urdf, robot.base, robot.joints, split_link_id(flange)[1])
            for flange in flanges}


def per_axis_angles_deg(qa: Sequence[float], qb: Sequence[float]) -> list[float]:
    """Angle between the x, y and z axes of two (x, y, z, w) quaternions, [x, y, z] in degrees."""
    rot_a = Rotation.from_quat(qa).as_matrix()
    rot_b = Rotation.from_quat(qb).as_matrix()
    angles = []
    for axis in range(3):
        va = rot_a[:, axis] / np.linalg.norm(rot_a[:, axis])
        vb = rot_b[:, axis] / np.linalg.norm(rot_b[:, axis])
        angles.append(degrees(np.arccos(min(1.0, max(-1.0, float(np.dot(va, vb)))))))
    return angles


def tool0_errors(flanges: Mapping[str, Pose], targets: Mapping[str, Pose]) -> dict[str, dict]:
    """Each target flange's error, by side, as the record keeps it.

    Args:
        flanges: Flange id -> its live world pose (`flange_poses`).
        targets: Flange id -> its authored target F* (`ServoPlan.targets`).

    Returns:
        dict[str, dict]: Side -> ``pos_err_mm`` [dx, dy, dz], ``pos_norm_mm``, ``rot_err_deg`` [x, y, z]
        (`per_axis_angles_deg`), ``observed_pose`` (position, orientation) of the live flange.
    """
    out = {}
    for flange, target in targets.items():
        observed = flanges[flange]
        pos_err_mm = (np.asarray(observed.position) - np.asarray(target.position)) * 1e3
        out[flange_side(flange)] = {
            "pos_err_mm": pos_err_mm.tolist(), "pos_norm_mm": float(np.linalg.norm(pos_err_mm)),
            "rot_err_deg": per_axis_angles_deg(observed.orientation, target.orientation),
            "observed_pose": (observed.position, observed.orientation)}
    return out


def base_drift(base: Pose, start: Pose) -> dict:
    """The base's drift from the servo's start: ``pos_diff_mm``, ``pos_norm_mm``, ``rot_diff_deg``."""
    pos_diff_mm = (np.asarray(base.position) - np.asarray(start.position)) * 1e3
    return {"pos_diff_mm": pos_diff_mm.tolist(), "pos_norm_mm": float(np.linalg.norm(pos_diff_mm)),
            "rot_diff_deg": per_axis_angles_deg(base.orientation, start.orientation)}


def _wrenches(robot: HuskyRobotInterface, arms: Iterable[str]) -> dict[str, np.ndarray] | None:
    """Side -> the arm's latest (6,) wrench, or None while any arm has none."""
    out = {}
    for arm in arms:
        wrench = robot.arms[arm].state.wrench
        if wrench is None:
            return None
        out[arm_side(arm)] = np.asarray(wrench, dtype=float)
    return out


def wrench_magnitudes(robot: HuskyRobotInterface, arms: Iterable[str]) -> dict[str, dict] | None:
    """Side -> ``force_N``, ``torque_Nm`` at the flange right now; None while any arm has no F/T reading.

    Magnitudes, not components: the sensor reads in its own wrist frame, which turns with the arm.
    """
    wrenches = _wrenches(robot, arms)
    if wrenches is None:
        return None
    return {side: {"force_N": force_magnitude(wrench), "torque_Nm": float(np.linalg.norm(wrench[3:6]))}
            for side, wrench in wrenches.items()}


def force_vectors(robot: HuskyRobotInterface, arms: Iterable[str]) -> dict[str, np.ndarray] | None:
    """Side -> the (3,) force at the flange in its sensor frame, newtons; None while any arm has none.

    ! The load guard watches how this vector CHANGES: the sensors are not zeroed with the bar held, and pressing the
      bar onto the floor makes the magnitude smaller, not larger. None leaves the guard off.
    """
    wrenches = _wrenches(robot, arms)
    return None if wrenches is None else {side: wrench[:3].copy() for side, wrench in wrenches.items()}


def tracking_gaps(robot: RobotObject, names: Sequence[str], end_row: Sequence[float],
                  arms: Iterable[str]) -> dict[str, float]:
    """Side -> how far the flange stands from where a path's last row puts it, metres.

    * Both at the live base: purely the arms' tracking error. A small gap with a large tool0-vs-target error means
      the base moved between planning and measuring.

    Args:
        robot: The live robot.
        names: The joint names of the path's columns.
        end_row: The last row sent, radians.
        arms: The arm names.
    """
    planned = {**robot.joints, **dict(zip(names, end_row))}
    gaps = {}
    for arm in arms:
        link = f"{arm}{FLANGE_SUFFIX}"
        live = link_pose(robot.model.urdf, robot.base, robot.joints, link)
        end = link_pose(robot.model.urdf, robot.base, planned, link)
        gaps[arm_side(arm)] = float(np.linalg.norm(np.subtract(live.position, end.position)))
    return gaps


# * ================================================================ the move between passes

def excursion(rows: Sequence[Sequence[float]]) -> tuple[int, float, float]:
    """(waypoints, net, excursion) of a joint path, radians; (0, 0.0, 0.0) for an empty one.

    ! Judge a path by its excursion (the furthest any joint gets from its first-row value over EVERY row), not its
      net travel (first row to last): a path that swings out and comes back has a net travel of 0.
    """
    path = np.asarray(rows, dtype=float)
    if path.size == 0:
        return 0, 0.0, 0.0
    offsets = np.abs(path - path[0])
    return len(path), float(np.max(offsets[-1])), float(np.max(offsets))


def recommended_time(travel: float, max_time: float) -> float:
    """Seconds the next move should take: its excursion (radians) at SPEED, within MIN_TIME..max_time."""
    return float(min(max(travel / SPEED, MIN_TIME), float(max_time)))


# * ================================================================ the held bar by mocap

def deviation_text(deviation: Mapping | None) -> str:
    """A bar deviation (`PoseDeviation.to_json`) as "1.23 mm/0.045°", or "n/a" without a fit."""
    return "n/a" if deviation is None else f"{deviation['dpos_mm']:.2f} mm/{deviation['dang_deg']:.3f}°"


def _deviation(T_meas: np.ndarray, T_design: np.ndarray, length: float) -> dict:
    """A bar pose's deviation from another, both evaluated at the bar START."""
    return pose_deviation(bar_start(T_meas, length), bar_start(T_design, length)).to_json()


def bar_reading(plan: ServoPlan, flanges: Mapping[str, Pose], fit: BarFit | None, source: str | None,
                design: Pose) -> dict:
    """The held bar as mocap and FK see it, against the pose the servo aims at; JSON-ready.

    Args:
        plan: The servo's plan: its holder and layout (the bar's length).
        flanges: Flange id -> live world pose of this reading (`flange_poses`).
        fit: The fit of the bar's markers around the FK bar, or None when there was no marker sample.
        source: Where the markers came from ("cloud": the relay's raw markers; "fake" for a desk rehearsal); None
            when unknown.
        design: The bar pose the servo aims at, B* (`ServoPlan.design_bar`).

    Returns:
        dict: ``bar``, ``grounded``, ``source``, ``fit`` (`BarFit.to_json`), ``T_design``, ``T_fk``, ``T_mocap``,
        ``T_corr`` (4x4 lists, the process bar frame in world; T_corr carries the mocap bar onto the design),
        ``fk_vs_design``, ``mocap_vs_design``, ``mocap_vs_fk`` (`PoseDeviation.to_json`, at the bar START),
        ``usable`` (the loop may steer on it) and ``reason`` (why not; '' when usable). The mocap fields are None
        without a good fit.
    """
    holder, grasp = plan.holder
    length = plan.layout.length
    T_design = design.matrix()
    T_fk = compose(flanges[holder], grasp).matrix()  # B_fk = F_h . G
    T_mocap = T_corr = mocap_vs_design = mocap_vs_fk = None
    if fit is not None and fit.ok:
        T_mocap = np.asarray(fit.T_world_from_bar, dtype=float)
        mocap_vs_design = _deviation(T_mocap, T_design, length)
        mocap_vs_fk = _deviation(T_mocap, T_fk, length)
        T_corr = bar_servo_correction(T_mocap, T_design)  # T_corr = B* . inv(B^)
    # * Steer on it only when it can be trusted, the checks in the old order.
    gate_mm, gate_deg = BAR_GATE[0] * 1e3, degrees(BAR_GATE[1])
    if fit is None:
        reason = "the bar could not be measured (no marker sample)"
    elif not fit.ok:
        reason = f"fit failed: {fit.reason}"
    elif source is None:
        reason = "where the markers came from is unknown; not steered on"
    elif source == "fake":
        reason = "fake markers (desk rehearsal; Motive does not stream the bar), never steered on"
    elif fit.rms_m is None or fit.rms_m >= BAR_MAX_RMS:
        reason = f"fit rms {(fit.rms_m or 0.0) * 1e3:.1f} mm (not under {BAR_MAX_RMS * 1e3:.0f} mm)"
    elif mocap_vs_design["dpos_mm"] > gate_mm or mocap_vs_design["dang_deg"] > gate_deg:
        reason = (f"mocap-vs-design {deviation_text(mocap_vs_design)} is beyond the {gate_mm:.0f} mm / "
                  f"{gate_deg:.0f} deg gate (a wrong match, not an as-built offset)")
    else:
        reason = ""
    return {"bar": plan.bar, "grounded": plan.grounded, "source": source,
            "fit": None if fit is None else fit.to_json(),
            "T_design": T_design.tolist(), "T_fk": T_fk.tolist(),
            "T_mocap": None if T_mocap is None else T_mocap.tolist(),
            "T_corr": None if T_corr is None else T_corr.tolist(),
            "fk_vs_design": _deviation(T_fk, T_design, length),
            "mocap_vs_design": mocap_vs_design, "mocap_vs_fk": mocap_vs_fk,
            "usable": not reason, "reason": reason}


def bar_on_target(bar: Mapping) -> bool:
    """Whether a `bar_reading` is usable and has the mocap bar within BAR_TOL of the design.

    True before any move: nothing to correct; after a move: converged.
    """
    if not bar["usable"]:
        return False
    deviation = bar["mocap_vs_design"]
    return deviation["dpos_mm"] < BAR_TOL[0] * 1e3 and deviation["dang_deg"] < degrees(BAR_TOL[1])


def bar_targets(T_corr: np.ndarray, flanges: Mapping[str, Pose]) -> dict[str, Pose]:
    """Flange id -> its target carrying the mocap bar onto the design: T_corr . F_s.

    One rigid move for both flanges (`T_corr` and `flanges` of the SAME reading), so the grasp actually held is kept.
    """
    T_corr = np.asarray(T_corr, dtype=float)
    return {flange: Pose.from_matrix(T_corr @ pose.matrix()) for flange, pose in flanges.items()}


def held_targets(targets: Mapping[str, Pose], flanges: Mapping[str, Pose]) -> dict[str, Pose]:
    """Flange id -> where the one rigid move that best fits the two targets takes it.

    ! Use this to aim a held move by tool0: it follows one flange's target and keeps the flanges' relation, so aiming
      at the authored targets corrects that flange's angle by swinging the far end of the bar.
    - The positions come first: the line between the flanges lands on the line between the targets, middle on middle,
      so a difference in span is shared by both ends.
    - The turn about that line, which positions cannot see, is the mean of both flanges' angle errors.

    Args:
        targets: The two flanges' authored targets.
        flanges: The two flanges' poses, of the same reading.
    """
    first, second = flanges
    live = {flange: np.asarray(flanges[flange].position, dtype=float) for flange in flanges}
    aimed = {flange: np.asarray(targets[flange].position, dtype=float) for flange in flanges}
    span, aim = live[second] - live[first], aimed[second] - aimed[first]
    span, aim = span / np.linalg.norm(span), aim / np.linalg.norm(aim)
    cross = np.cross(span, aim)
    size = float(np.linalg.norm(cross))
    align = Rotation.from_rotvec(cross / size * np.arctan2(size, span @ aim) if size > 0.0 else np.zeros(3))
    rotation = {flange: Rotation.from_quat(flanges[flange].orientation) for flange in flanges}
    errors = [Rotation.from_quat(targets[flange].orientation) * (align * rotation[flange]).inv() for flange in flanges]
    roll = np.mean([error.as_rotvec() @ aim for error in errors])
    turn = Rotation.from_rotvec(aim * roll) * align
    middle, aimed_middle = (live[first] + live[second]) / 2.0, (aimed[first] + aimed[second]) / 2.0
    return {flange: Pose(tuple(map(float, aimed_middle + turn.apply(live[flange] - middle))),
                         tuple(map(float, (turn * rotation[flange]).as_quat()))) for flange in flanges}


# * ================================================================ the stop tests

def stop_after_move(reading: Mapping, plan: ServoPlan, engaged: bool) -> tuple[str, str] | None:
    """Whether the loop stops after a move and its reading, the tests in the old order.

    1. The load guard (after a ground bar's insert): a flange's force changed by more than the limit: FAILED.
    2. Steering on the bar (`engaged`): its fit lost: FAILED; the mocap bar within BAR_TOL of the design: DONE.
    3. Else tool0: every flange within TOOL0_TOL of its authored target: DONE.

    Args:
        reading: This pass's reading: ``tool0`` (`tool0_errors`), ``force_change_N`` (side -> newtons) and ``bar``
            (`bar_reading`) when it has them.
        plan: The servo's plan.
        engaged: A usable bar reading has set the targets.

    Returns:
        tuple[str, str] | None: (DONE or FAILED, why in plain words), or None: run another pass.
    """
    change = reading.get("force_change_N")
    if plan.max_force_change is not None and change:
        worst = max(change.values())
        if worst > plan.max_force_change:
            return FAILED, (f"the force on an arm changed by {worst:.1f} N since the first reading (limit "
                            f"{plan.max_force_change:.0f} N); the bar may be pushed against something")
    if plan.by_bar and engaged:
        bar = reading.get("bar")
        if bar is None or not bar["usable"]:
            return FAILED, (f"the mocap bar fit is lost ({'not measured' if bar is None else bar['reason']}); "
                            f"the arms stay at the last bar-corrected pose")
        if bar_on_target(bar):
            deviation = bar["mocap_vs_design"]
            return DONE, (f"converged on the mocap bar: dpos {deviation['dpos_mm']:.2f} mm, dang "
                          f"{deviation['dang_deg']:.3f} deg (< {BAR_TOL[0] * 1e3:.1f} mm / "
                          f"{degrees(BAR_TOL[1]):.1f} deg)")
        return None
    if max(side["pos_norm_mm"] for side in reading["tool0"].values()) < TOOL0_TOL * 1e3:
        return DONE, (f"converged on tool0 (< {TOOL0_TOL * 1e3:.1f} mm)"
                      + (": the mocap bar fit was never usable, so the loop aimed tool0" if plan.by_bar else ""))
    return None


# * ================================================================ carried into the insert, and the gate after it

def authored_start(action: Action, insert: int, flange: str) -> Pose:
    """A flange's authored pose at the insert's start: its target in the nearest earlier movement that gives one.

    Raises:
        ValueError: If no earlier movement of the action gives one.
    """
    for earlier in reversed(action.movements[:insert]):
        own = earlier.target.robots.get(action.robot) if earlier.target is not None else None
        if own is not None and flange in own.link_poses:
            return own.link_poses[flange]
    raise ValueError(f"no authored start for {flange} before {action.movements[insert].id}")


def carried_insert_goals(loaded: LoadedProcess, action_id: str, insert: int, robot: RobotObject) -> dict[str, Pose]:
    """Flange id -> the insert's goal after a mocap-bar servo: the authored move from the live flange.

    G_s = F*end_s . inv(F*start_s) . F_s (`shared/bars/pose.apply_authored_move`): F*start is the flange's target in the
    last earlier movement that gives one (the transfer), F*end the insert's, F_s the live flange.

    Args:
        loaded: The loaded process.
        action_id: The action.
        insert: Index of the insert in the action.
        robot: The live robot at planning time.

    Raises:
        ValueError: If no earlier movement of the action gives a flange its authored start.
    """
    action = loaded.process.actions[action_id]
    movement = action.movements[insert]
    ends = movement.target.robots[action.robot].link_poses
    live = flange_poses(robot, movement.arms)
    return {flange: Pose.from_matrix(apply_authored_move(authored_start(action, insert, flange).matrix(),
                                                         ends[flange].matrix(), live[flange].matrix()))
            for flange in movement.arms}


def insert_end_gap(robot: RobotObject, names: Sequence[str], end_row: Sequence[float], arms: Iterable[str]) -> float:
    """How far the worst flange stopped from the insert plan's last row (`tracking_gaps`), metres.

    * The URDF's tool0 at the live joints, not the measured TCP (the insert's own `end_gap_mm` record).
    ! The servo after a ground bar's insert runs only when this is within GROUNDED_END_TOL: judged at the flanges,
      because a joint tolerance cannot tell a 15 mm insert that never ran from one that did.
    """
    return max(tracking_gaps(robot, names, end_row, arms).values())
