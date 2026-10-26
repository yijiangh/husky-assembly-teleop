"""
The compliant Robotiq close: a support robot's gripper closes on a bar another robot holds, its arm compliant meanwhile.

The arm springs around where it stands (the anchor: its measured TCP) with a zero target wrench, so the closing fingers
pull it up to MAX_DRIFT instead of pulling on the other robot's tools (W:6637-6869).

- An anchor further than ANCHOR_TOL from the URDF's tool0 of the live joints means the frames are wrong: the gripper
  then closes with the arm stiff (STIFF).
- Once the switch to compliance is asked, every ending stops fingers still closing (a cancel too) and brings the arm
  back under joint control.
- ! Main thread, from a plugin task that holds the robot's lease.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import numpy as np

from scaffolding_assembly_core.geometry import Pose

from ...robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER
from ...robot_interface.soft_stop import MotionRefused
from .controllers import leave_compliance, switch_arms
from .precheck import part_problem
from .stream import (CLOSED, DRY_RUN, FAILED, FK_TCP_TOL, REFUSED, STIFF, PartOutcome, Recorder, fk_tcp_gap,
                     gap_record)
from .tools import ROBOTIQ_CLOSE, ROBOTIQ_EFFORT, robotiq_move

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.arm import ArmInterface
    from ...robot_interface.end_effectors import RobotiqGripper
    from ...robot_interface.robot import HuskyRobotInterface

#: How far the measured TCP may move from the anchor, metres, before the hold ends (W:6641).
MAX_DRIFT = 0.01
#: The anchor and the URDF's tool0 of the live joints must agree this well, metres and radians (W:6645-6646).
ANCHOR_TOL = FK_TCP_TOL
#: Seconds the F/T sensor gets to zero, before the arm goes compliant (W:6648).
FT_SETTLE = 0.5
#: Seconds the gripper has to answer the close (W:6652).
TIMEOUT = 30.0


def handoff_text(arm: str) -> str:
    """The handoff's Confirm question, after "<robot> (<serial>): <movement> — "."""
    return (f"CLOSE + compliant handoff on {arm}: the arm goes COMPLIANT before the close (gives way ≤ "
            f"{MAX_DRIFT * 1e3:.0f} mm); jaws {ROBOTIQ_CLOSE}, effort {ROBOTIQ_EFFORT}, ≤ {TIMEOUT:.0f} s; then joint "
            f"control. Watch Cindy's tools are not pulled; hand on the e-stop.")


async def run_handoff(ctx: PluginContext, robot: HuskyRobotInterface, arm: str, gripper: RobotiqGripper, *,
                      dry_run: bool = False, recorder: Recorder | None = None) -> PartOutcome:
    """Close `gripper` on a bar another robot holds, its arm compliant around where it stands until the gripper answers.

    Joint control confirmed, F/T sensor zeroed, anchor checked against the URDF, compliance, the anchor and a zero
    wrench, then the close; each tick the anchor and a zero wrench again until the gripper answers.

    Args:
        ctx: The plugin's context.
        robot: The support robot.
        arm: The name of its arm carrying `gripper`, e.g. "ur_arm".
        gripper: The Robotiq on that arm.
        dry_run: Run the checks and send nothing.
        recorder: Gets the samples (e.g. for a live readout), made at the start; None: its own.

    Returns:
        PartOutcome: CLOSED (the gripper reported success: stalled on the bar or at its goal); STIFF (closed with the
            arm stiff: the anchor check failed); REFUSED (nothing sent); FAILED (a switch or a target refused: nothing
            closed; the arm moved over MAX_DRIFT, the gripper reported no success or did not answer within TIMEOUT,
            or the arm may be left compliant); DRY_RUN. The caller logs the ending.

    Raises:
        asyncio.CancelledError: When cancelled; fingers still closing were stopped and the arm brought back first.
    """
    label = f"{robot.config.serial} {arm}: handoff"
    interface = robot.arms[arm]
    record: dict = {"arm": arm, "gripper": {"target": ROBOTIQ_CLOSE, "effort": ROBOTIQ_EFFORT}}
    started = ctx.now()

    def ended(status: str, reason: str) -> PartOutcome:
        """The outcome, its record completed."""
        seconds = ctx.now() - started
        return PartOutcome(status, reason, seconds, {"status": status, "reason": reason, "seconds": seconds, **record})

    problem = _problem(ctx, robot, arm, gripper)
    if problem:
        return ended(REFUSED, f"{problem}; nothing sent")
    if dry_run:
        distance, angle = fk_tcp_gap(robot, arm)
        stiff = "" if _anchor_ok(distance, angle) else "; the anchor check fails: a real run closes with the arm STIFF"
        ctx.log_warn(f"DRY RUN {label}: passed every check (anchor {distance * 1e3:.1f} mm, "
                     f"{math.degrees(angle):.2f}° from the URDF's tool0){stiff}, NOT SENT")
        return ended(DRY_RUN, f"every check passed{stiff}; nothing sent (dry run)")

    joint = await switch_arms(ctx, [interface], SCALED_JOINT_TRAJECTORY_CONTROLLER)
    if not joint.ok:
        return ended(FAILED, f"{joint.reason}; nothing closed")
    # * Zeroed while the fingers are open and touch nothing; never with a bar in them (it would tare its weight).
    before = _force(interface)
    zeroed = interface.zero_ft_sensor()
    await ctx.sleep(FT_SETTLE)
    after = _force(interface)
    record["ft_zero"] = {"sent": zeroed, "force_before_N": before, "force_after_N": after}
    (ctx.log_info if zeroed else ctx.log_warn)(
        f"{label}: F/T sensor {'zeroed' if zeroed else 'NOT zeroed (service not available)'}: |F| "
        f"{_newtons(before)} before, {_newtons(after)} {FT_SETTLE:.1f} s later")

    # ! The compliant arm pulls straight to the anchor: it must be where the arm really is.
    state = interface.state
    anchor = Pose.from_arrays(state.tcp_position, state.tcp_orientation)
    distance, angle = fk_tcp_gap(robot, arm)
    record["fk_tcp_gap"] = gap_record((distance, angle))
    record["anchor"] = [np.round(anchor.position, 6).tolist(), np.round(anchor.orientation, 6).tolist()]
    gap = (f"anchor {distance * 1e3:.1f} mm, {math.degrees(angle):.2f}° from the URDF's tool0 (limit "
           f"{ANCHOR_TOL[0] * 1e3:.0f} mm, {math.degrees(ANCHOR_TOL[1]):.0f}°)")
    if not _anchor_ok(distance, angle):
        ctx.log_error(f"{label}: {gap}: no handoff, the gripper closes with the arm STIFF")
        stiff = await robotiq_move(ctx, gripper, ROBOTIQ_CLOSE, effort=ROBOTIQ_EFFORT)
        record["stiff_close"] = stiff.as_record()
        return ended(STIFF, f"{gap}: closed with the arm stiff: {stiff.reason}")
    ctx.log_info(f"{label}: {gap}")

    recorder = Recorder(robot, started) if recorder is None else recorder
    recorder.phase = "close"
    try:
        status, reason = await _close(ctx, interface, gripper, anchor, recorder, record, label)
    finally:
        try:
            # * A6: fingers still closing stop where they are (nothing happens once they stopped); then the arm
            #   stiffens.
            gripper.stop()
        finally:
            back = ctx.now()
            restored = await leave_compliance(ctx, [interface], label=label)
            record.setdefault("switch", {}).update(back_s=ctx.now() - back, back_ok=restored)
    record["gripper"].update(position=gripper.state.position, result_ok=gripper.state.last_result_ok)
    record.update(recorder.as_record())
    if status == CLOSED and not restored:
        status, reason = FAILED, (f"{reason}; but {arm} may be left under {CARTESIAN_COMPLIANCE_CONTROLLER}: switch it "
                                  f"in robot_control before anything else")
    return ended(status, reason)


async def _close(ctx: PluginContext, arm: ArmInterface, gripper: RobotiqGripper, anchor: Pose, recorder: Recorder,
                 record: dict, label: str) -> tuple[str, str]:
    """Switch the arm to compliance, hold it at `anchor` and close the gripper until it answers.

    Returns:
        tuple[str, str]: CLOSED or FAILED, and why; `record` gets the switch and the largest drift.
    """
    switched = await switch_arms(ctx, [arm], CARTESIAN_COMPLIANCE_CONTROLLER)
    record["switch"] = {"to_s": switched.seconds, "ok": switched.ok}
    if not switched.ok:
        return FAILED, f"{switched.reason}; nothing closed"
    # ! The anchor and a zero wrench go out before the close goal: robot_control may have left a wrench.
    refused = _hold_anchor(arm, anchor)
    if refused:
        return FAILED, f"{refused}; nothing closed"
    recorder.tick(ctx.now())
    try:
        gripper.move(ROBOTIQ_CLOSE, ROBOTIQ_EFFORT)
    except MotionRefused as stopped:
        return FAILED, f"{gripper.namespace}: {stopped}; nothing closed"
    if not gripper.state.moving:
        return FAILED, f"{gripper.namespace}: the close goal was not sent (its log says why); nothing closed"
    closing = ctx.now()
    ctx.log_info(f"{label}: closing to {ROBOTIQ_CLOSE:.3f} rad, effort {ROBOTIQ_EFFORT:.2f}, the arm compliant around "
                 f"its anchor with zero wrench until the gripper answers (≤ {TIMEOUT:.0f} s); the hold ends if the arm "
                 f"moves over {MAX_DRIFT * 1e3:.0f} mm")
    record["drift_max_mm"] = 0.0
    while True:
        await ctx.next_tick()
        drift = float(np.linalg.norm(np.subtract(arm.state.tcp_position, anchor.position)))
        record["drift_max_mm"] = max(record["drift_max_mm"], drift * 1e3)
        if drift > MAX_DRIFT:
            return FAILED, (f"the arm moved {drift * 1e3:.1f} mm from its anchor (limit {MAX_DRIFT * 1e3:.0f} mm); "
                            f"check the grip")
        refused = _hold_anchor(arm, anchor)
        if refused:
            return FAILED, f"{refused}; check the grip"
        recorder.tick(ctx.now())
        if not gripper.state.moving:
            where = "unknown" if gripper.state.position is None else f"{gripper.state.position:.3f} rad"
            if gripper.state.last_result_ok:
                return CLOSED, f"the gripper closed (stalled on the bar or at its goal) at {where}"
            return FAILED, f"the gripper did not report success (at {where}); check the grip"
        if ctx.now() - closing > TIMEOUT:
            return FAILED, f"no answer from the gripper within {TIMEOUT:.0f} s of the close; check the grip"


def _problem(ctx: PluginContext, robot: HuskyRobotInterface, arm: str, gripper: RobotiqGripper) -> str:
    """Why the handoff may not start; "" if it may.

    Every part's checks but the frames (the anchor is checked after the F/T zero instead), and the gripper ready and
    idle.
    """
    problem = part_problem(ctx, robot, [arm], {}, frames=False)
    if problem:
        return problem
    if not gripper.server_is_ready():
        return f"{gripper.namespace}: the gripper's action server is not available"
    if gripper.state.moving:
        return f"{gripper.namespace}: an earlier gripper command still runs; wait for it, or Stop"
    return ""


def _anchor_ok(distance: float, angle: float) -> bool:
    """Whether the measured TCP is within ANCHOR_TOL of the URDF's tool0."""
    return distance <= ANCHOR_TOL[0] and angle <= ANCHOR_TOL[1]


def _hold_anchor(arm: ArmInterface, anchor: Pose) -> str:
    """Send the anchor as the compliance target, then a zero target wrench; why either was refused, or ""."""
    name = arm.config.name
    try:
        if not arm.send_cartesian_target(anchor.position, anchor.orientation):
            return f"{name} refused the anchor target (its log says why)"
        if not arm.send_target_wrench((0.0, 0.0, 0.0)):
            return f"{name} refused the zero target wrench (its log says why)"
    except MotionRefused as refused:
        return f"{name}: {refused}"
    return ""


def _force(arm: ArmInterface) -> float | None:
    """The arm's measured |F|, newtons; None before its first F/T reading."""
    wrench = arm.state.wrench
    return None if wrench is None else round(float(np.linalg.norm(wrench[:3])), 2)


def _newtons(force: float | None) -> str:
    """A force for the log."""
    return "unknown" if force is None else f"{force:.1f} N"
