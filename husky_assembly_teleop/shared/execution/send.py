"""
Send one motion to a robot's arms and watch it until it arrives, stops short, times out or is cancelled.

`motion_command` times a planned path (`retime`) and splits it per arm by joint name; `send_motion` sends it, both arms
of a two-arm robot together, then waits until the time is up and every arm is at its end.

- Every ending but arriving holds every arm of the robot: a timeout, a stop short of the end, something else moving
  (`watch`), the caller's `until`, a cancel (re-raised). A hold an arm refuses (stale joints) makes the outcome
  HOLD_NOT_SENT, said loudly.
- A dry run builds each arm's trajectory, so every check the arm runs before sending runs, and sends nothing.
- A send the robot refuses by raising (soft stopped, or no lease: `MotionRefused`) is REFUSED.
- ! Main thread, from a plugin task.
"""

from __future__ import annotations

import asyncio
import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Callable, Sequence

import numpy as np

from ...plugin_api.concurrency import WaitTimeout
from ...robot_interface.arm import JOINT_STATE_MAX_AGE
from ...robot_interface.soft_stop import MotionRefused
from ..assembly_planning.joints import arm_joint_names
from .retime import retime

if TYPE_CHECKING:
    from ...plugin_api.context import PluginContext
    from ...robot_interface.robot import HuskyRobotInterface

#: Every joint this close to its end, radians, counts as arrived (as robot_control's moves).
ARRIVED_TOLERANCE = END_TOLERANCE = math.radians(0.5)
#: Seconds past the duration before an unfinished motion is held as timed out.
ARRIVE_MARGIN = 5.0
#: Seconds past the duration after which arms that stopped short count as stopped.
STILL_SETTLE = 1.0
#: A motion moving a joint further than this, radians, asks to confirm with a warning.
LARGE_TRAVEL = math.pi / 3

#: How a motion ended.
ARRIVED, REFUSED, TIMEOUT, STOPPED, INTERRUPTED, ENDED, HOLD_NOT_SENT, DRY_RUN = (
    "ARRIVED", "REFUSED", "TIMEOUT", "STOPPED", "INTERRUPTED", "ENDED", "HOLD_NOT_SENT", "DRY_RUN")


@dataclass(frozen=True)
class ArmMotion:
    """One arm's part of a motion.

    Attributes:
        arm: The arm's name, e.g. "left_ur_arm".
        positions: Joint values per sample (M x 6), UR_JOINT_NAMES order.
        velocities: Their velocities (M x 6), the last row 0.
    """

    arm: str
    positions: np.ndarray
    velocities: np.ndarray


@dataclass(frozen=True)
class MotionCommand:
    """A planned path timed for a robot's arms, ready to send.

    Attributes:
        serial: The robot it is for.
        arms: One motion per moving arm, in the robot's arm order.
        duration: Seconds.
        minimum: The shortest duration the arms' limits allow.
        requested: The duration asked for.
        travel: The furthest any joint travels along the path, radians.
    """

    serial: str
    arms: tuple[ArmMotion, ...]
    duration: float
    minimum: float
    requested: float
    travel: float

    def as_record(self) -> dict:
        """The command as JSON values: what is sent, sample by sample."""
        return {"serial": self.serial, "arms": [motion.arm for motion in self.arms], "duration": self.duration,
                "minimum": self.minimum, "requested": self.requested, "travel": self.travel,
                "samples": len(self.arms[0].positions),
                "positions": {motion.arm: motion.positions.tolist() for motion in self.arms},
                "velocities": {motion.arm: motion.velocities.tolist() for motion in self.arms}}


@dataclass(frozen=True)
class SendOutcome:
    """How a motion ended.

    Attributes:
        status: ARRIVED, REFUSED (nothing sent), TIMEOUT, STOPPED, INTERRUPTED (something else moved), ENDED (the
            caller's `until` came true), HOLD_NOT_SENT (one of those, and an arm refused its hold: stop it with the
            pendant or the e-stop) or DRY_RUN.
        reason: One line saying what happened.
        seconds: From sending to the end of the wait.
        end: Arm name -> its measured joints at the end, or None if nothing was sent.
        end_error: Arm name -> the largest gap between its joints and its end, radians.
        holds: Arm name -> whether its hold went out, every arm of the robot, when it was held.
    """

    status: str
    reason: str = ""
    seconds: float = 0.0
    end: dict[str, list[float]] | None = None
    end_error: dict[str, float] = field(default_factory=dict)
    holds: dict[str, bool] = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        """bool: Arrived, or (dry run) would have been sent."""
        return self.status in (ARRIVED, DRY_RUN)

    def as_record(self) -> dict:
        """The outcome as JSON values."""
        return {"status": self.status, "reason": self.reason, "seconds": self.seconds, "end": self.end,
                "end_error": dict(self.end_error), "holds": dict(self.holds)}


@dataclass
class SendProgress:
    """How far a send got, filled in as it goes: what the record of a send cut short by a cancel says.

    Attributes:
        seconds: From sending to the cancel.
        end: Arm name -> its measured joints at the cancel.
        end_age: Arm name -> how old those joints were at the cancel, seconds; None if none ever came.
        holds: Arm name -> whether its hold went out, every arm of the robot.
    """

    seconds: float = 0.0
    end: dict[str, list[float] | None] | None = None
    end_age: dict[str, float | None] = field(default_factory=dict)
    holds: dict[str, bool] = field(default_factory=dict)

    @property
    def end_stale(self) -> bool:
        """bool: Whether any arm's `end` joints were stale (over JOINT_STATE_MAX_AGE): not where it really stopped."""
        return any(age is None or age > JOINT_STATE_MAX_AGE for age in self.end_age.values())

    @property
    def hold_not_sent(self) -> list[str]:
        """list[str]: The arms that refused their hold."""
        return [arm for arm, sent in self.holds.items() if not sent]


def motion_command(serial: str, arm_names: Sequence[str], joint_names: Sequence[str],
                   rows: Sequence[Sequence[float]], traj_time: float) -> MotionCommand:
    """Time a planned path for a robot's arms: every column together, then split per arm by joint name.

    Args:
        serial: The robot.
        arm_names: Its configured arms, in its arm order.
        joint_names: The path's columns.
        rows: The path (N x len(joint_names)), radians; the first row is where the arms are.
        traj_time: Seconds asked for; the arms' limits may need longer (`retime`).

    Returns:
        MotionCommand: The command.

    Raises:
        ValueError: If a joint belongs to none of the arms, an arm is only partly in the path, or the path can't be
            timed (`RetimeError`).
    """
    names = list(joint_names)
    rows = np.asarray(rows, dtype=float)
    if len(set(names)) != len(names) or rows.ndim != 2 or rows.shape[1] != len(names):
        raise ValueError(f"the path's {rows.shape} values do not match its {len(names)} distinct joint names")
    moving, columns = [], []
    for arm in arm_names:
        own = arm_joint_names(arm)
        given = [name in names for name in own]
        if any(given) and not all(given):
            raise ValueError(f"{serial} {arm} is only partly in the path: {[n for n, g in zip(own, given) if g]}")
        if all(given):
            moving.append(arm)
            columns += [names.index(name) for name in own]
    stray = sorted(set(names) - {name for arm in moving for name in arm_joint_names(arm)})
    if stray or not moving:
        raise ValueError(f"the path's joints {stray or names} belong to no arm of {serial} ({', '.join(arm_names)})")
    ordered = rows[:, columns]
    timed = retime(ordered, traj_time)
    travel = float(np.max(np.sum(np.abs(np.diff(ordered, axis=0)), axis=0)))
    arms = tuple(ArmMotion(arm, timed.positions[:, 6 * k:6 * k + 6], timed.velocities[:, 6 * k:6 * k + 6])
                 for k, arm in enumerate(moving))
    return MotionCommand(serial, arms, timed.duration, timed.minimum, float(traj_time), travel)


async def send_motion(ctx: PluginContext, robot: HuskyRobotInterface, command: MotionCommand, *,
                      dry_run: bool = False, watch: Callable[[], str] | None = None,
                      progress: SendProgress | None = None, on_sent: Callable[[], None] | None = None,
                      until: Callable[[], bool] | None = None) -> SendOutcome:
    """Send a motion and wait until every arm is at its end once the duration is up; hold every arm otherwise.

    Args:
        ctx: The plugin's context, for its ticks and log.
        robot: The robot to move; its serial must be the command's.
        command: What to send.
        dry_run: Build every trajectory (all the arm's checks) and send nothing.
        watch: Checked every tick while the arms move: why the motion must stop (e.g. another robot moves), or "".
        progress: Filled in when a cancel cuts the send short (where the arms were, the holds), or None.
        on_sent: Called once the trajectory went out, before the wait, e.g. to answer a commit.
        until: Checked every tick while the arms move: True ends the motion early, every arm held (ENDED).

    Returns:
        SendOutcome: ARRIVED, REFUSED (nothing sent), TIMEOUT, STOPPED, INTERRUPTED or ENDED (all held),
            HOLD_NOT_SENT, or DRY_RUN.

    Raises:
        asyncio.CancelledError: When cancelled; every arm was held first (`progress` says whether each hold went out).
    """
    serial = command.serial
    if robot.config.serial != serial:
        return SendOutcome(REFUSED, f"the command is for {serial}, not {robot.config.serial}")
    missing = [motion.arm for motion in command.arms if motion.arm not in robot.arms]
    if missing:
        return SendOutcome(REFUSED, f"{serial} has no arm {', '.join(missing)}")
    arms = {motion.arm: robot.arms[motion.arm] for motion in command.arms}
    try:
        if dry_run:
            return _dry_run(ctx, command, arms)
        if len(command.arms) == 2:
            sent = robot.send_multi_arm_trajectory({motion.arm: motion.positions for motion in command.arms},
                                                   command.duration,
                                                   {motion.arm: motion.velocities for motion in command.arms})
        elif len(command.arms) == 1:
            motion = command.arms[0]
            sent = arms[motion.arm].send_joint_trajectory(motion.positions, command.duration, motion.velocities)
        else:
            return SendOutcome(REFUSED, f"{len(command.arms)} arms in one command; at most two move together")
    except MotionRefused as refused:
        return SendOutcome(REFUSED, f"{serial} refused the trajectory: {refused}; nothing sent")
    if not sent:
        return SendOutcome(REFUSED, f"{serial} refused the trajectory (the log says why); nothing sent")
    started = ctx.now()
    ctx.log_info(f"{serial}: moving {', '.join(arms)}, {command.duration:.1f} s")
    if on_sent is not None:
        on_sent()
    ends = {motion.arm: motion.positions[-1] for motion in command.arms}
    lost, interrupted, ended = False, "", False

    def errors() -> dict[str, float]:
        """Each moving arm's largest gap to its end, radians (infinite without joints)."""
        return {name: _gap(arm.joint_vector(), ends[name]) for name, arm in arms.items()}

    def finished() -> bool:
        """Arrived, stopped short once the time is up, interrupted or ended; says once when joint states stop coming."""
        nonlocal lost, interrupted, ended
        now = ctx.now()
        interrupted = watch() if watch is not None else ""
        ended = not interrupted and until is not None and until()
        if interrupted or ended:
            return True
        stale = [name for name, arm in arms.items()
                 if arm.state.last_update_time is None or now - arm.state.last_update_time > JOINT_STATE_MAX_AGE]
        if stale and not lost:
            lost = True
            ctx.log_error(f"lost {serial} joint states: the arm runs the trajectory it has; e-stop if needed")
        if now - started < command.duration:
            return False
        if max(errors().values()) < ARRIVED_TOLERANCE:
            return True
        # ? Still by the last motion seen, on fresh joint states: `is_executing` is not set again once a slow cosine
        #   start cleared it, and lost joint states must end in the timeout, not here.
        return now - started > command.duration + STILL_SETTLE and not stale and all(
            arm.state.last_motion_time is not None and now - arm.state.last_motion_time > STILL_SETTLE
            for arm in arms.values())

    def held(status: str, reason: str) -> SendOutcome:
        """Hold every arm, then the outcome: HOLD_NOT_SENT, said loudly, if an arm refused its hold."""
        holds = _hold(ctx, robot, reason)
        refused = [name for name, sent in holds.items() if not sent]
        if refused:
            status, reason = HOLD_NOT_SENT, _not_held(serial, refused, reason, ctx.log_error)
        else:
            reason += "; every arm held"
        return _outcome(status, reason, ctx.now() - started, arms, errors(), holds)

    try:
        await ctx.wait_until(finished, timeout_s=command.duration + ARRIVE_MARGIN, description=f"{serial} to arrive")
    except WaitTimeout:
        return held(TIMEOUT, f"not at the end {command.duration + ARRIVE_MARGIN:.0f} s after sending")
    except asyncio.CancelledError:
        holds = _hold(ctx, robot, "cancelled")
        if progress is not None:
            progress.seconds, progress.holds = ctx.now() - started, holds
            progress.end = _outcome(STOPPED, "", 0.0, arms, {}).end
            progress.end_age = {name: None if arm.state.last_update_time is None
                                else ctx.now() - arm.state.last_update_time for name, arm in arms.items()}
        refused = [name for name, sent in holds.items() if not sent]
        if refused:
            _not_held(serial, refused, "cancelled", ctx.log_error)
        raise
    if interrupted:
        return held(INTERRUPTED, interrupted)
    if ended:
        return held(ENDED, "ended by the caller")
    gaps = errors()
    if max(gaps.values()) < ARRIVED_TOLERANCE:
        return _outcome(ARRIVED, f"arrived, end within {math.degrees(max(gaps.values())):.2f}°",
                        ctx.now() - started, arms, gaps)
    return held(STOPPED, f"stopped {math.degrees(max(gaps.values())):.1f}° short of the end")


def _dry_run(ctx: PluginContext, command: MotionCommand, arms: dict) -> SendOutcome:
    """Build each arm's trajectory, every check included, and say so loudly; send nothing."""
    for motion in command.arms:
        if arms[motion.arm].build_trajectory(motion.positions, command.duration, motion.velocities) is None:
            return SendOutcome(REFUSED, f"{command.serial} {motion.arm} refused the trajectory (the log says why)")
    rule = "#" * 78
    for motion in command.arms:
        ctx.log_warn(f"\n{rule}\n##  DRY RUN {command.serial} {motion.arm}: passed every check, NOT SENT\n"
                     f"##  {len(motion.positions)} samples, {command.duration:.1f} s\n{rule}")
    return SendOutcome(DRY_RUN, "every check passed; nothing sent (dry run)")


def _hold(ctx: PluginContext, robot: HuskyRobotInterface, why: str) -> dict[str, bool]:
    """Hold every arm of the robot where it is; each arm logs whether its hold went out.

    Returns:
        dict[str, bool]: Arm name -> whether its hold was sent.
    """
    ctx.log_warn(f"{robot.config.serial}: {why}: holding every arm")
    return {name: bool(arm.hold()) for name, arm in robot.arms.items()}


def _not_held(serial: str, arms: Sequence[str], why: str, log: Callable[[str], None] | None = None) -> str:
    """The loud line for arms that refused their hold, logged as an error if `log` is given."""
    line = f"HOLD NOT SENT on {', '.join(arms)}: stop {serial} with the teach pendant or the e-stop ({why})"
    if log is not None:
        log(line)
    return line


def _gap(here: np.ndarray | None, end: np.ndarray) -> float:
    """The largest gap between an arm's joints and its end, radians; infinite without joints."""
    return math.inf if here is None else float(np.max(np.abs(here - end)))


def _outcome(status: str, reason: str, seconds: float, arms: dict, gaps: dict[str, float],
             holds: dict[str, bool] | None = None) -> SendOutcome:
    """An outcome with each moving arm's measured joints."""
    end = {}
    for name, arm in arms.items():
        here = arm.joint_vector()
        end[name] = None if here is None else here.tolist()
    return SendOutcome(status, reason, seconds, end, gaps, dict(holds or {}))
