"""
The calibration's checks and their limits: plain functions of what was measured, each returning `Check`s.

The steps (`steps/`) judge their results and readiness with them; a result fails while one is BAD.
! A step's chips must not appear or vanish: the functions behind them return the same labels whatever was measured,
  PENDING while there is nothing to judge yet. Only `take_checks` varies; it serves the take list, not a step's chips.
"""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from scaffolding_assembly_core.geometry import Pose, compose, invert

from ....robot_interface.mocap_definitions import (FRAME_ERROR, FRAME_HELD, FRAME_NONE, FRAME_REFIT_PENDING,
                                                   FRAME_UNCHECKED, FRAME_UNTRUSTED)
from ....ui.style import few, hover
from ....world.checks import BAD, GOOD, INFO, PENDING, WARN, Check
from ....world.mocap import MARKER_ERROR_LIMIT, MARKER_ERROR_WARN
from .fit import Circle, Solution
from .pivot import PIVOT_MIN_POSES, Pivot

if TYPE_CHECKING:
    from ....robot_interface.mocap_relay import FloorCalibrationState

#: Takes per joint: Solve needs TAKES_NEEDED; with fewer than TAKES_WANTED the axes rest on few circles (amber).
TAKES_NEEDED, TAKES_WANTED = 2, 4
#: Seconds of driving that judge the rigid bodies' marker errors.
DRIVE_SECONDS = 5.0
#: Seconds Setup's checklist and a marker error measurement hold for; then they expire for the steps that need them.
PRECONDITION_HOLDS = 3600.0
#: Setup's checklist; each box holds for PRECONDITION_HOLDS after its tick, TOOL_BOX only for the tool model ticked for.
#: SAFETY_BOXES must also hold for a robot move, or it is refused (a Go prompt confirms them again).
TOOL_BOX = "Tool on flange"
SAFETY_BOXES = ("Area clear", "E-stop at hand")
CHECKLIST_BOXES = (TOOL_BOX, *SAFETY_BOXES, "Cameras calibrated")
#: A take's circle: largest point error, metres, and how consistently the joint turned one way (`Circle.turn`).
CIRCLE_ERROR = 1.5e-3
TURN_MIN = 0.9
#: What leaves a take out of Solve, in short (`left_out_reason`).
LEFT_OUT_LIMITS = f"limits {CIRCLE_ERROR * 1e3:g} mm off the circle, {TURN_MIN:.0%} one way"
#: Validation: the tool body's spread in tool0 under the new calibration, 95th percentile: position amber and red
#: (metres), and each axis' angle (degrees). The old pipeline's targets.
VALIDATION_WARN = 2e-3
VALIDATION_DISTANCE = 5e-3
VALIDATION_ANGLE = 0.3
#: Validation needs this many samples, at this many base spots; a spot is new this far from the others (metres,
#: degrees).
VALIDATION_SAMPLES = 15
VALIDATION_SPOTS = 3
SPOT_DISTANCE, SPOT_ANGLE = 0.5, 30.0
#: Collision checks and planned moves hold while the base stays this close to where it stood: metres, radians.
CHECKED_POSITION = 0.02
CHECKED_ANGLE = np.radians(2.0)
#: Punch validation: takes needed, base yaw range wanted (degrees), the tips' largest distance from their mean, amber
#: and red (metres), and the plausible TCP offset length (metres; a slipped digit is metres off).
PUNCH_TAKES = 4
PUNCH_YAW_RANGE = 90.0
PUNCH_WARN, PUNCH_BAD = 2e-3, 5e-3
PUNCH_OFFSET = (0.05, 0.3)
#: Tip offset: poses wanted; their tilt spread (RMS), green from and red below (degrees, `Pivot.tilt`); each pose's tip
#: from the fitted point, amber and red (metres); how far the base may move and turn between poses (metres,
#: degrees: 0.05° moves a point a metre away by about 1 mm).
TIP_POSES = 4
TIP_TILT, TIP_TILT_BAD = 15.0, 5.0
TIP_WARN, TIP_BAD = 0.5e-3, 1.5e-3
TIP_BASE_MOVED, TIP_BASE_TURNED = 1e-3, 0.05
#: The end effector kind of the tool body's mount: the collision checks need its model on the flange.
MARKER_KIND = "calibration_marker"


def expiry(what: str, since: float, now: float) -> str | None:
    """Why a precondition met at Unix time `since` has expired at `now`: over PRECONDITION_HOLDS; None if it holds."""
    age = now - since
    if age <= PRECONDITION_HOLDS:
        return None
    return f"{what} {time.strftime('%H:%M', time.localtime(since))}, {age / 60:.0f} min ago (over " \
           f"{PRECONDITION_HOLDS / 60:.0f} min)"


def tool_check(serial: str, tools: dict[str, str | None], arm: str, wanted: str | None = MARKER_KIND) -> Check:
    """Judge the arm's tool model: the collision checks see the tool set at launch, which must be `wanted`.

    Args:
        serial: The robot's serial, e.g. "a200-0805".
        tools: Each arm's end effector kind, in arm order; None for a bare arm.
        arm: The arm being calibrated.
        wanted: The kind it must be: the marker for the takes, None (bare) for the punch, which has no model.
    """
    if tools[arm] == wanted:
        return Check("tool model", GOOD, f"{wanted} on the flange" if wanted else "bare arm, as for the punch")
    kinds = ",".join((wanted or "none") if other == arm else kind or "none" for other, kind in tools.items())
    return Check("tool model", BAD, f"the monitor models {tools[arm] or 'a bare arm'} on the flange, not "
                 f"{wanted or 'a bare arm (the punch has no model)'}: restart it with -p tools:=\"['"
                 f"{serial.split('-')[-1]}:{kinds}']\"")


def sample_marker_error(errors: dict | None) -> float | None:
    """The largest marker error a sample stored with it ({"base": m, "tool": m}), metres; None if it stored none."""
    values = [float(value) for value in (errors or {}).values() if value is not None]
    return max(values) if values else None


def marker_error_level(error: float | None) -> int:
    """A sample's marker error judged as in Marker error: red over MARKER_ERROR_LIMIT, amber over MARKER_ERROR_WARN."""
    if error is None:
        return PENDING
    return BAD if error > MARKER_ERROR_LIMIT else WARN if error > MARKER_ERROR_WARN else GOOD


def marker_error_detail(errors: dict | None) -> str:
    """A sample's stored marker errors in words, e.g. "base 0.12 mm, tool 1.40 mm"; "marker error unknown" without."""
    known = {body: value for body, value in (errors or {}).items() if value is not None}
    if not known:
        return "marker error unknown"
    return ", ".join(f"{body} {value * 1e3:.2f} mm" for body, value in known.items())


def marker_checks(largest: Sequence[float] | None, seconds: float, expired: str | None = None) -> list[Check]:
    """Judge both bodies' marker errors while driving: amber over MARKER_ERROR_WARN, red over MARKER_ERROR_LIMIT.

    Args:
        largest: (base, tool) largest marker error while driving, metres; None before any.
        seconds: How long the base was driven, seconds.
        expired: Why a full measurement no longer counts (too old), or None: amber, as Record and Validate refuse.
    """
    if seconds < DRIVE_SECONDS:
        # * Red while driving: only a red check makes the step fail, so Done asks first and Accept as is takes a
        # * short drive. No countdown: Marker error's readout counts the seconds.
        drive = Check("drive", BAD, f"drive around for {DRIVE_SECONDS:g} s in all, turning both ways")
    elif expired is not None:
        drive = Check("drive", WARN, f"{expired}: Recheck before Record or Validate")
    else:
        drive = Check("drive", GOOD, f"driven {DRIVE_SECONDS:g} s or more")
    checks = [drive]
    # * No numbers: the largest error rises while driving, which would drop the chips' hover; Marker error's readout
    # * shows them.
    for column, name in enumerate(("base", "tool")):
        if largest is None:
            checks.append(Check(f"{name} markers", PENDING, "not measured yet: drive around"))
            continue
        worst = float(largest[column])
        level = GOOD if worst <= MARKER_ERROR_WARN else WARN if worst <= MARKER_ERROR_LIMIT else BAD
        detail = {GOOD: f"largest marker error while driving within {MARKER_ERROR_WARN * 1e3:g} mm",
                  WARN: f"largest marker error while driving over {MARKER_ERROR_WARN * 1e3:g} mm: usable, but its "
                        f"markers are worth a look",
                  BAD: f"largest marker error while driving over {MARKER_ERROR_LIMIT * 1e3:g} mm: look at its markers "
                       f"and Recheck, or Refine the {name} body in Motive"}[level]
        checks.append(Check(f"{name} markers", level, detail))
    return checks


def frame_check(frame: FloorCalibrationState | None, anchors_missing: Sequence[str] = ()) -> Check:
    """Judge the lab frame the relay holds: green held; amber unchecked or a refit pending; red untrusted, error.

    No floor calibration still compares takes in one frame (Motive's own): grey, as before the relay announced one.

    Args:
        frame: The floor calibration the relay announced (`MocapRelayState.frame`), or None before it announced one.
        anchors_missing: The floor calibration's anchors the relay does not see.
    """
    if frame is None:
        return Check("frame", PENDING, "the relay announced no floor calibration yet: is it running in this domain?",
                     "?")
    missing = f"; anchors missing: {few(anchors_missing)}" if anchors_missing else ""
    if frame.status == FRAME_HELD:
        refit = f", refit used: {frame.refit}" if frame.refit else ""
        held_by = frame.decides or "anchors or truss cameras"
        return Check("frame", GOOD, hover(f"held by the {held_by}: {frame.floor_calibration}{refit}{missing}",
                                          "the mocap probe"), frame.status)
    if frame.status == FRAME_UNCHECKED:
        why = "applied, not checked yet: no fit of the anchors or truss cameras has decided"
    elif frame.status == FRAME_REFIT_PENDING:
        why = f"Motive's frame moved ({frame.pending}): poses are off until Use refit in the mocap probe"
    elif frame.status == FRAME_ERROR:
        why = f"the relay could not read its floor calibration: {frame.error}"
    elif frame.status == FRAME_NONE:
        why = ("no floor calibration applied: poses are Motive's own, fine for comparing takes; make and apply one "
               "with the mocap probe")
    elif frame.status == FRAME_UNTRUSTED:
        why = f"not trusted: the anchors and truss cameras do not agree ({frame.pending}); check them in Motive"
    else:
        why = f"the relay says {frame.status}"
    level = PENDING if frame.status == FRAME_NONE else WARN if frame.status in (FRAME_UNCHECKED,
                                                                                FRAME_REFIT_PENDING) else BAD
    return Check("frame", level, hover(why + missing, "the mocap probe"), frame.status)


def take_count_check(counts: Sequence[int], what: str) -> Check:
    """Judge the takes per joint: red under TAKES_NEEDED, amber under TAKES_WANTED.

    Args:
        counts: (J0, J1) takes.
        what: What was counted, e.g. "clear".
    """
    level = BAD if min(counts) < TAKES_NEEDED else WARN if min(counts) < TAKES_WANTED else GOOD
    return Check("takes", level, f"{counts[0]} J0 and {counts[1]} J1 takes {what}; {TAKES_NEEDED} each needed, "
                                 f"{TAKES_WANTED} or more wanted", f"J0 {counts[0]} · J1 {counts[1]}")


def validation_level(distance: float) -> str:
    """One validation sample's level by its distance from the mean in tool0, metres: as the position check judges."""
    return GOOD if distance <= VALIDATION_WARN else WARN if distance <= VALIDATION_DISTANCE else BAD


def take_checks(label: str, circle: Circle) -> list[Check]:
    """Judge one recorded take's circle."""
    error, turn = float(circle.errors.max()), abs(circle.turn)
    # * Amber, not red: a bad take is left out of Solve; record it again or go on with the others.
    checks = [Check(f"{label} fit", GOOD if error <= CIRCLE_ERROR else WARN,
                    f"points within {error * 1e3:.2f} mm of the circle (limit {CIRCLE_ERROR * 1e3:g} mm)",
                    f"{error * 1e3:.2f} mm off")]
    if turn < TURN_MIN:
        checks.append(Check(f"{label} sweep", WARN, f"the joint turned one way in only {turn:.0%} of the steps",
                            f"{turn:.0%} one way"))
    return checks


def left_out_reason(circle: Circle) -> str | None:
    """Why Solve leaves a take out, in short, e.g. "fit 1.62 mm off"; None if Solve uses it (`take_checks`)."""
    return ", ".join(f"{check.label.strip()} {check.value}" for check in take_checks("", circle)
                     if check.level != GOOD) or None


def left_out_check(circles: Sequence[tuple[int, Circle]]) -> Check:
    """Judge the recorded takes' circles together: the takes left out of Solve, and why (`take_checks`).

    Args:
        circles: Each recorded take's number and circle.
    """
    if not circles:
        return Check("left out", PENDING, "no take recorded yet")
    why = [f"{number} ({reason})" for number, circle in circles if (reason := left_out_reason(circle))]
    if not why:
        return Check("left out", GOOD, "none: every take fits its circle and swept one way", "0")
    return Check("left out", WARN, hover(f"to record again: take {few(why, 2)}; {LEFT_OUT_LIMITS}", "the take chips"),
                 str(len(why)))


def pose_change(before: Pose, after: Pose) -> tuple[float, float]:
    """How far `after` is from `before`: metres and degrees."""
    change = compose(invert(before), after)
    return float(np.linalg.norm(change.position)), float(np.degrees(Rotation.from_quat(change.orientation).magnitude()))


def base_moved(before: Pose, after: Pose) -> bool:
    """Whether the base moved past CHECKED_POSITION or CHECKED_ANGLE: checks and plans made at `before` are stale."""
    metres, degrees = pose_change(before, after)
    return metres > CHECKED_POSITION or np.radians(degrees) > CHECKED_ANGLE


def new_spot(before: Pose, after: Pose) -> bool:
    """Whether the base stands at another spot than `before`, as `base_spots` counts (SPOT_DISTANCE, SPOT_ANGLE)."""
    metres, degrees = pose_change(before, after)
    return metres > SPOT_DISTANCE or degrees > SPOT_ANGLE


#: The chips `solve_checks` returns, pending before a solution.
SOLVE_LABELS = ("change vs applied", "J0 axis", "J1 axis", "axes", "relay limits")


def solve_checks(solution: Solution, applied: Pose | None, sanity: str | None) -> list[Check]:
    """Judge a solution: its fits, its axes, and whether the relay would take it.

    Args:
        solution: The solved calibration.
        applied: The calibration the relay applies now, or None if it applies none.
        sanity: Why the relay warns about it (`calibration_sessions.sanity_problem`), or None.
    """
    # * The change and the axes are blue, facts: no limit judges them.
    if applied is None:
        checks = [Check("change vs applied", PENDING, "the relay applies no calibration to compare with")]
    else:
        metres, degrees = pose_change(applied, solution.footprint)
        checks = [Check("change vs applied", INFO, f"{metres * 1e3:.2f} mm, {degrees:.3f}° from the calibration the "
                        f"relay applies now: what Apply will change", f"{metres * 1e3:.1f} mm")]
    for name, axis in (("J0", solution.j0), ("J1", solution.j1)):
        detail = (f"normals within {axis.normal_angles.max():.3f}°, centres within "
                  f"{axis.center_distances.max() * 1e3:.2f} mm of the line")
        checks.append(Check(f"{name} axis", INFO, detail))
    checks.append(Check("axes", GOOD if solution.skew < 1e-3 else WARN,
                        f"{solution.axes_angle:.3f}° apart, {solution.skew * 1e3:.2f} mm skew",
                        f"{solution.skew * 1e3:.1f} mm skew"))
    checks.append(Check("relay limits", WARN if sanity else GOOD, f"{sanity}: the relay applies it with a warning"
                        if sanity else "within the relay's sanity limits"))
    return checks


def mount_checks(correction: Pose) -> list[Check]:
    """Judge an arm's measured mount against the URDF's (`fit.mount_correction`): amber past a millimetre or 0.1°."""
    metres, degrees = pose_change(Pose(), correction)
    level = GOOD if metres <= 1e-3 and degrees <= 0.1 else WARN
    return [Check("mount", level, f"measured mount {metres * 1e3:.2f} mm, {degrees:.3f}° from the URDF's; the relay is "
                                  f"not changed", f"{metres * 1e3:.1f} mm")]


def yaw_degrees(pose: Pose) -> float:
    """A pose's heading about world z, degrees."""
    return float(np.degrees(Rotation.from_quat(pose.orientation).as_euler("ZYX")[0]))


def yaw_range(poses: Sequence[Pose]) -> float:
    """How far apart the poses' headings are, degrees: 360 minus the largest gap between them."""
    if len(poses) < 2:
        return 0.0
    yaws = np.sort(np.mod([yaw_degrees(pose) for pose in poses], 360.0))
    return float(360.0 - np.max(np.diff(np.append(yaws, yaws[0] + 360.0))))


def spot_groups(bases: Sequence[Pose]) -> list[list[int]]:
    """The samples' indices by base spot, spots in the order of their first sample.

    A pose far from every earlier spot (`new_spot`) starts one; else it joins the first spot it is near, also one
    driven back to later.
    """
    spots: list[Pose] = []
    groups: list[list[int]] = []
    for index, pose in enumerate(bases):
        near = next((group for spot, group in zip(spots, groups) if not new_spot(spot, pose)), None)
        if near is None:
            spots.append(pose)
            groups.append([index])
        else:
            near.append(index)
    return groups


def base_spots(bases: Sequence[Pose]) -> int:
    """How many base spots the samples were taken at (`spot_groups`)."""
    return len(spot_groups(bases))


def validation_checks(distances: np.ndarray | None, angles: np.ndarray | None,
                      bases: Sequence[Pose]) -> list[Check]:
    """Judge the tool body's spread in tool0 under the new calibration, and how much of the room it covers.

    Args:
        distances: (n,) each sample's distance from the mean position, metres; None if not judged (no samples, or
            no solution): position and orientation are then pending.
        angles: (n, 3) each sample's axis angles from the mean axes, degrees; None with `distances`.
        bases: Each sample's base body pose in the world.
    """
    count, spots = len(bases), base_spots(bases)
    checks = [Check("samples", GOOD if count >= VALIDATION_SAMPLES else BAD,
                    f"{count} samples; {VALIDATION_SAMPLES} needed", f"{count}/{VALIDATION_SAMPLES}"),
              Check("spots", GOOD if spots >= VALIDATION_SPOTS else BAD,
                    f"{spots} base spots far enough apart ({SPOT_DISTANCE:g} m or {SPOT_ANGLE:g}°), "
                    f"{yaw_range(bases):.0f}° of headings; {VALIDATION_SPOTS} needed", f"{spots}/{VALIDATION_SPOTS}")]
    if distances is None or not len(distances):
        return checks + [Check("position", PENDING, "no samples yet"), Check("orientation", PENDING, "no samples yet")]
    p95, worst = float(np.percentile(distances, 95)), float(np.percentile(angles, 95, axis=0).max())
    position = GOOD if p95 <= VALIDATION_WARN else WARN if p95 <= VALIDATION_DISTANCE else BAD
    return checks + [
        Check("position", position, f"95 % within {p95 * 1e3:.2f} mm, all within {distances.max() * 1e3:.2f} mm "
                                    f"(amber {VALIDATION_WARN * 1e3:g}, red {VALIDATION_DISTANCE * 1e3:g} mm)",
              f"{p95 * 1e3:.1f} mm"),
        Check("orientation", GOOD if worst <= VALIDATION_ANGLE else WARN,
              f"95 % of each axis within {worst:.3f}° (target {VALIDATION_ANGLE:g}°)", f"{worst:.2f}°")]


def tcp_offset_check(offset: np.ndarray | None) -> Check:
    """Judge the TCP offset, tool0 to the punch tip, metres: red outside PUNCH_OFFSET; pending without one."""
    if offset is None:
        return Check("TCP offset", PENDING, "no tip offset solved yet")
    length = float(np.linalg.norm(offset))
    return Check("TCP offset", GOOD if PUNCH_OFFSET[0] <= length <= PUNCH_OFFSET[1] else BAD,
                 f"{length * 1e3:.1f} mm from tool0 (expected {PUNCH_OFFSET[0] * 1e3:g}–{PUNCH_OFFSET[1] * 1e3:g} mm: "
                 f"from Tip offset, or entered in mm)", f"{length * 1e3:.0f} mm")


#: The tip offset's tilt chip (`Pivot.tilt`).
TILT = "tilt spread (RMS)"


def tip_checks(count: int, pivot: Pivot | None, bases: Sequence[Pose]) -> list[Check]:
    """Judge a tip offset's poses: enough, tilted about two axes, tips together, the base still between them.

    Args:
        count: The poses taken.
        pivot: Their fit, or None with too few.
        bases: Each pose's base_footprint in the world.
    """
    poses = Check("poses", GOOD if count >= TIP_POSES else BAD, f"{count} of {TIP_POSES} poses", f"{count}/{TIP_POSES}")
    moves = [pose_change(bases[0], base) for base in bases[1:]]
    moved, turned = (max(move[0] for move in moves), max(move[1] for move in moves)) if moves else (0.0, 0.0)
    still = f"{moved * 1e3:.2f} mm and {turned:.3f}° from the first pose"
    if moved > TIP_BASE_MOVED or turned > TIP_BASE_TURNED:
        base = Check("base", BAD, f"moved {still} (red over {TIP_BASE_MOVED * 1e3:g} mm or {TIP_BASE_TURNED:g}°): "
                                  f"the poses are in the robot's frame; clear them and start again")
    else:
        base = Check("base", GOOD, f"within {still}")
    if pivot is None:
        needed = f"{PIVOT_MIN_POSES} poses needed"
        return [poses, Check(TILT, PENDING, needed), Check("fit", PENDING, needed), base, tcp_offset_check(None)]
    tilt, worst = np.degrees(pivot.tilt), float(pivot.residuals.max())
    return [poses,
            Check(TILT, GOOD if tilt >= TIP_TILT else WARN if tilt >= TIP_TILT_BAD else BAD,
                  f"how far the poses turn about the axis they turn least about: want {TIP_TILT:g}°, red under "
                  f"{TIP_TILT_BAD:g}°; tilt the tip about two different axes, ±30° about each gives about 18°",
                  f"{tilt:.0f}°"),
            Check("fit", GOOD if worst <= TIP_WARN else WARN if worst <= TIP_BAD else BAD,
                  f"every tip within {worst * 1e3:.2f} mm of the fitted point (amber {TIP_WARN * 1e3:g}, red "
                  f"{TIP_BAD * 1e3:g} mm)", f"{worst * 1e3:.2f} mm"),
            base, tcp_offset_check(pivot.offset)]


def punch_checks(tips: np.ndarray, bases: Sequence[Pose]) -> list[Check]:
    """Judge a punch validation's takes, which should all put the tip on the target; the spread pending until two.

    Args:
        tips: (n, 3) each take's tip in the world, metres.
        bases: Each take's base_footprint in the world.
    """
    checks = [Check("takes", GOOD if len(tips) >= PUNCH_TAKES else BAD, f"{len(tips)} of {PUNCH_TAKES} takes",
                    f"{len(tips)}/{PUNCH_TAKES}")]
    if len(tips) < 2:
        return checks + [Check("tip spread", PENDING, "two takes needed"),
                         Check("headings", PENDING, "two takes needed")]
    spread = float(np.linalg.norm(tips - tips.mean(axis=0), axis=1).max())
    turned = yaw_range(bases)
    return checks + [
        Check("tip spread", GOOD if spread <= PUNCH_WARN else WARN if spread <= PUNCH_BAD else BAD,
              f"every tip within {spread * 1e3:.2f} mm of their mean (amber {PUNCH_WARN * 1e3:g}, red "
              f"{PUNCH_BAD * 1e3:g} mm)", f"{spread * 1e3:.1f} mm"),
        Check("headings", GOOD if turned >= PUNCH_YAW_RANGE else WARN,
              f"base headings over {turned:.0f}° (want {PUNCH_YAW_RANGE:g}°)", f"{turned:.0f}°")]


def punch_as_taken_check(tips: np.ndarray) -> Check:
    """Judge the punch tips end to end, as taken: amber over PUNCH_WARN, never red (the verdict is `punch_checks`).

    Args:
        tips: (n, 3) each take's tip in the world under the calibration the relay applied and the TCP offset used.
    """
    if len(tips) < 2:
        return Check("as taken", PENDING, "two takes needed")
    spread = float(np.linalg.norm(tips - tips.mean(axis=0), axis=1).max())
    return Check("as taken", GOOD if spread <= PUNCH_WARN else WARN,
                 f"end to end, under the calibration the relay applied and the TCP offset used at each take: every "
                 f"tip within {spread * 1e3:.2f} mm of their mean", f"{spread * 1e3:.1f} mm")
