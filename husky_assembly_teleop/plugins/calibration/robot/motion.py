"""
The calibration's arm motions: planned moves previewed before Go, sweep steps, and samples once the tool is still.

! Every motion runs under the robot's lease, waited for inside it (`wait_holding`): the arm never moves on after
  the lease is given back. A sweep step also needs the caller's clearance.
"""

from __future__ import annotations

import asyncio
import math
from typing import TYPE_CHECKING

import numpy as np

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id

from ....plugin_api.concurrency import WaitTimeout
from ....robot_interface.arm import MOVING_WITHIN, joint_move
from ....robot_interface.mocap_state import mocap_problem
from ....shared.arm_planner_contract import ArmPlannerContract, ArmPlanRequest
from ....shared.execution.moving import arm_moving
from ....shared.guide.procedure import Procedure
from ....shared.planner.timed_path import TimedPath
from ..analysis.checks import base_moved
from .sampling import SAMPLE_TICKS, STILL_SPREAD, STILL_TICKS, Reading, Sample, Sampler, mean_pose

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext
    from ....robot_interface.arm import ArmInterface
    from ....world.measured import TrackedObject
    from ..run import Run

#: An arm has arrived once no joint is further than this from the target, radians.
ARRIVED = math.radians(0.5)
#: Seconds a move may take beyond its planned duration, and a planned move beyond twice its preview's.
MOVE_MARGIN = 5.0
PLAN_MARGIN = 30.0
#: Seconds a waypoint may take to settle and give SAMPLE_TICKS good readings.
SAMPLE_TIMEOUT = 20.0
#: Shortest sweep step, seconds: a 12° step takes about this at the joint move speed limit. The default minimum is
#: for moves sent by hand; sweep steps are single-joint and collision-checked.
STEP_MIN_DURATION = 1.0
#: The arm stands still once its joints move less than this over STILL_TICKS, radians.
JOINTS_STILL = 1e-3


def sweep_path(waypoints: np.ndarray, tick_period: float) -> TimedPath:
    """A sweep as a take runs it, for its preview: a stop at each waypoint, and between two a `step_to` move.

    Each stop is its shortest: the arm still for MOVING_WITHIN (`arrived`), then SAMPLE_TICKS readings. Readings
    that come in moving or untracked make the real stop longer.

    Args:
        waypoints: (n, 6) joint values, radians.
        tick_period: The monitor's tick, seconds: a sample takes SAMPLE_TICKS of them.
    """
    waypoints = np.asarray(waypoints, dtype=float)
    stop = MOVING_WITHIN + SAMPLE_TICKS * tick_period
    times, states = [0.0, stop], [waypoints[0], waypoints[0]]
    for start, end in zip(waypoints, waypoints[1:]):
        # * The move's own points, evenly spaced in time: the ghost speeds up and slows to rest as the arm does.
        positions, _velocities, duration = joint_move(start, end, min_duration=STEP_MIN_DURATION)
        times += list(times[-1] + np.linspace(0.0, duration, len(positions))[1:])
        states += list(positions[1:])
        times.append(times[-1] + stop)
        states.append(end)
    return TimedPath(np.array(times), np.array(states))


class NoPlan(RuntimeError):
    """The arm planner found no move to a pose."""


def stopped(arm: ArmInterface, now: float) -> bool:
    """Whether the arm stands still for MOVING_WITHIN (`arm_moving`); stale joint states never count as still.

    ! Not `is_executing` alone: it clears during a slow move (the pendant's speed slider).
    """
    return not arm_moving(arm, "the arm", now, MOVING_WITHIN)


def arrived(arm: ArmInterface, joints: np.ndarray, now: float) -> bool:
    """Whether the arm stands still at `joints`; still, since a short move is within ARRIVED all the way."""
    here = arm.joint_vector()
    return stopped(arm, now) and here is not None and float(np.max(np.abs(here - joints))) <= ARRIVED


async def wait_rested(ctx: PluginContext, arm: ArmInterface) -> None:
    """Wait until the arm last moved MOVING_WITHIN ago: before that it refuses a trajectory."""
    await ctx.wait_until(lambda: arm.state.last_motion_time is None
                         or ctx.now() - arm.state.last_motion_time >= MOVING_WITHIN,
                         timeout_s=MOVE_MARGIN, description="the arm to rest")


async def wait_holding(ctx: PluginContext, arm: ArmInterface, done, timeout_s: float, description: str) -> None:
    """Wait until `done()` while the arm moves; hold the arm if the wait times out or is cancelled, then raise.

    ! Wait inside the lease the move was sent under, so the arm never moves on once the lease is given back.
    """
    try:
        await ctx.wait_until(done, timeout_s=timeout_s, description=description)
    except (WaitTimeout, asyncio.CancelledError):
        arm.hold()
        raise


class ArmMotion:
    """Moves the run's arm and samples it; the procedure shows what it does and asks for Go."""

    def __init__(self, procedure: Procedure):
        """Act for `procedure`."""
        self.procedure = procedure

    async def move_to(self, ctx: PluginContext, run: Run, joints: np.ndarray, what: str) -> None:
        """Plan a collision-free move to `joints` with the arm planner, wait for Go (`optional_go`), send it and wait.

        Other robots not measured make Go ask twice; the move then checks them as stand-ins. A base driven while Go
        waits (`base_moved`) has the move planned again from where it stands. Go's Clearance box says whether the move
        runs under the planner's clearance; the prompt says when that is below 5 mm (a plan keeping less than 1 cm).

        Returns once the arm arrived and the planner gave the robot's lease back.

        Raises:
            NoPlan: If the planner finds no move.
            RuntimeError: If the move was not sent.
        """
        arm = run.robot.arms[run.arm]
        here = arm.joint_vector()
        if here is not None and np.max(np.abs(here - joints)) <= ARRIVED:
            return
        planner: ArmPlannerContract = ctx.require("arm_planner")
        again = ""
        while True:
            self.procedure.doing = f"{again}planning the move to {what}"
            planned_at = ctx.kinematics.base_pose(run.serial)
            plan = await planner.request_plan(ArmPlanRequest(run.serial, run.arm, tuple(float(v) for v in joints)))
            if plan.state != "ready":
                raise NoPlan(f"no move to {what}: {plan.message}")
            # * A plan keeping less than usual has a smaller clearance too (`close_note`): the prompt says so.
            close = planner.close_note(plan) if self.procedure.clearance.value else ""
            unknown = await self.procedure.optional_go(
                ctx, plan.path, f"{again}Move to {what}: {plan.message}{f'; {close}' if close else ''}. Scrub Preview "
                                f"to see the way, check the robot's surroundings, then Go.", "move",
                [robot_id(run.serial)])
            if not base_moved(planned_at, ctx.kinematics.base_pose(run.serial)):
                break
            again = "The base moved: "
        try:
            sent, message = await planner.commit(plan.id, unknown, self.procedure.clearance.value)
        except asyncio.CancelledError:
            # * Cancelled before the planner answered: it may already be leasing the robot or checking the start.
            planner.cancel_commit(run.serial)
            raise
        if not sent:
            raise RuntimeError(f"move to {what} not sent: {message}")
        self.procedure.doing = f"moving to {what}"
        try:
            # ? The plan's duration is its preview's; the trajectory sent stops at every waypoint and takes longer.
            await wait_holding(ctx, arm, lambda: arrived(arm, joints, ctx.now()),
                               2 * plan.path.duration + PLAN_MARGIN, f"the arm to reach {what}")
        except asyncio.CancelledError:
            # * Cancel stops the planner's motion too, which then gives the robot back.
            planner.cancel_commit(run.serial)
            raise
        lease_slot = run.robot.lease_slot
        await ctx.wait_until(lambda: lease_slot.lease is None, timeout_s=MOVE_MARGIN,
                             description="the arm planner to give the robot back")

    async def step_to(self, ctx: PluginContext, run: Run, joints: np.ndarray) -> None:
        """Send one joint move under the held lease once the arm rests, and wait to arrive."""
        arm = run.robot.arms[run.arm]
        if arrived(arm, joints, ctx.now()):
            return
        await wait_rested(ctx, arm)
        duration = arm.send_joint_move(joints, min_duration=STEP_MIN_DURATION)
        if duration is None:
            refusal = arm.last_refusal
            raise RuntimeError(f"the arm refused the move: {refusal.reason if refusal else 'see the log'}")
        await wait_holding(ctx, arm, lambda: arrived(arm, joints, ctx.now()), duration + MOVE_MARGIN,
                           "the arm to arrive")

    async def sample(self, ctx: PluginContext, run: Run, tool: TrackedObject, what: str) -> Sample:
        """Wait until the tool stands still on the base with good mocap, then average SAMPLE_TICKS readings.

        If that takes over SAMPLE_TIMEOUT, say why and wait for Go to try again; Cancel ends the procedure.
        """
        base, arm = run.robot.base, run.robot.arms[run.arm]
        sampler = Sampler()
        why = ""

        def reading_ok() -> bool:
            nonlocal why
            now = ctx.now()
            problem = mocap_problem(base.mocap_id, base.state, now) or mocap_problem(tool.mocap_id, tool, now)
            joints = arm.joint_vector()
            if problem or joints is None:
                why = problem or "no joint state"
                sampler.clear()
                return False
            sampler.add(Reading(Pose.from_arrays(base.state.position, base.state.orientation),
                                Pose.from_arrays(tool.position, tool.orientation), joints,
                                base.state.marker_error, tool.marker_error))
            if not sampler.still():
                spread = sampler.spread(STILL_TICKS)
                why = "the tool still moves on the base" + (f" ({spread * 1e3:.2f} mm)" if spread is not None else "")
                return False
            return len(sampler.readings) >= SAMPLE_TICKS

        while True:
            try:
                await ctx.wait_until(reading_ok, timeout_s=SAMPLE_TIMEOUT, description="a still, tracked tool and base")
                break
            except WaitTimeout:
                await self.procedure.ask_go(ctx, f"{what}: no good sample in {SAMPLE_TIMEOUT:g} s: {why}. Go tries "
                                                 f"again, Cancel stops.")
                sampler.clear()
                self.procedure.doing = f"sampling {what}"
        applied = run.applied()
        calibration = Pose(applied.position, applied.orientation) if applied is not None else Pose()
        return sampler.sample(calibration, applied.sha256 if applied is not None else "", ctx.now())

    async def still_arm(self, ctx: PluginContext, run: Run, what: str,
                        with_base: bool = True) -> tuple[Pose | None, np.ndarray]:
        """Wait until the arm, and the base with good mocap if `with_base`, stand still; average SAMPLE_TICKS readings.

        If that takes over SAMPLE_TIMEOUT, say why and wait for Go to try again; Cancel ends the procedure.

        Returns:
            tuple[Pose | None, np.ndarray]: base_footprint in the world (None without the base), and the arm's joints.
        """
        base, arm = run.robot.base, run.robot.arms[run.arm]
        bases, joints, why = [], [], ""
        parts = "the base and arm" if with_base else "the arm"

        def still() -> bool:
            nonlocal why
            problem = mocap_problem(base.mocap_id, base.state, ctx.now()) if with_base else None
            here = arm.joint_vector()
            if problem or here is None:
                why = problem or "no joint state"
                bases.clear()
                joints.clear()
                return False
            if with_base:
                bases.append(ctx.kinematics.base_pose(run.serial))
            joints.append(here)
            del bases[:-SAMPLE_TICKS], joints[:-SAMPLE_TICKS]
            spread = max(np.ptp([pose.position for pose in bases[-STILL_TICKS:]], axis=0)) if with_base else 0.0
            moving = np.ptp(joints[-STILL_TICKS:], axis=0).max()
            why = f"{parts} still move{'s' if not with_base else ''}"
            return len(joints) >= SAMPLE_TICKS and spread <= STILL_SPREAD and moving <= JOINTS_STILL

        self.procedure.doing = f"{what}: waiting for {parts} to stand still"
        while True:
            try:
                await ctx.wait_until(still, timeout_s=SAMPLE_TIMEOUT, description=f"{parts} to stand still")
                return (mean_pose(bases) if with_base else None), np.mean(joints, axis=0)
            except WaitTimeout:
                await self.procedure.ask_go(ctx, f"{what}: no still reading in {SAMPLE_TIMEOUT:g} s: {why}. Go tries "
                                                 f"again, Cancel stops.")
