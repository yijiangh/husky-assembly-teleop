"""
The SERVO part: the arms brought onto the authored targets pass by pass, the robot leased.

It runs after a transfer, a support robot's approach or a grounded insert (W:2373-2668 two arms, W:6128-6193 one
arm). Each pass: the checks of every part again (process and action unchanged, others still, the robot live), a plan
on the motion planner (`request_servo_step`), the operator's Confirm on the first pass and on any later one
travelling over CONFIRM_TRAVEL, the move, a settle, a reading. With "Servo by bar" the loop steers on the MoCap bar
once a usable fit comes (`bar_reading`), else on tool0. Done = converged, already on target, or nothing left to plan;
every other end leaves the step pending (spec A1, A2).

! Main thread, inside the plugin's task. A cancel during a move leaves every arm held (`send_motion`); `run_servo`'s
  `finally` still untracks the bar and writes the record, then the cancel goes on.
"""

from __future__ import annotations

import asyncio
import math
from typing import TYPE_CHECKING, Any

import numpy as np
from async_timeout import timeout

from scaffolding_assembly_core.geometry import compose
from scaffolding_assembly_core.ids import robot_id

from ...shared.assembly_planning.joints import arm_joint_names
from ...shared.assembly_planning.tamp_route import HELD
from ...shared.assembly_run_contract import is_live
from ...shared.execution.basis import basis_of
from ...shared.execution.send import ARRIVED, SendProgress, send_motion
from ...shared.execution.send import DRY_RUN as SENT_DRY, HOLD_NOT_SENT as SENT_NOT_HELD, REFUSED as SENT_REFUSED
from ...shared.execution.servo import (BAR_TOL, CONFIRM_TRAVEL, MARKER_MAX_AGE, MAX_PASSES, SETTLE, TOOL0_TOL,
                                       TRANSFER, ServoPlan, bar_on_target, bar_reading, bar_targets, base_drift,
                                       deviation_text, excursion, flange_poses, flange_side, force_vectors,
                                       recommended_time, servo_plan, stop_after_move, tool0_errors, tracking_gaps,
                                       wrench_magnitudes)
from ...shared.execution.servo_record import plot_servo, servo_payload
from ...shared.motion_planner_contract import MotionPlanRequest, ServoAsk
from ...shared.movement_exec_contract import (CANCELLED, DONE, DRY_RUN, FAILED, HOLD_NOT_SENT, MAX_TRAJ_TIME,
                                              REFUSED, SERVO_TARGET, Carried)
from ...shared.planner.planner_search import WAIT_MARGIN
from ...world.freeze import WorldMoved
from ...world.marker_layouts import bar_asset_id, bar_layouts, fit_bar, sensed_from_raw
from .sequences import Job, PartRun, confirmed, live_command_of, others_basis, others_moving, world_problem
from .servo_measure import SIDE, ik_record, insert_end_problem, line, say_feedback, servo_file, sides

if TYPE_CHECKING:
    from scaffolding_assembly_core.robot import RobotObject

    from ...shared.assembly_planning.route_result import RouteResult
    from ...shared.motion_planner_contract import MotionPlannerContract
    from ...world.measured import TrackedObject

#: Seconds a pass's plan may take: tamp's held search once, and the planner queue's margin.
PLAN_TIME = HELD["max_time"] + 60.0 + WAIT_MARGIN
#: Seconds to wait for the bar's first marker sample once it is tracked.
FIRST_SAMPLE = 1.0
NOT_LIVE = "is not live (base tracked and fresh within 1 s, joints within 0.5 s)"


async def run_servo(job: Job, run: PartRun) -> None:
    """Run the step's servo (spec §2.7); `run` says how it ended and names its record (`PartRun.servo_file`).

    Args:
        job: The request being run.
        run: The servo part, filled in as it goes.

    Raises:
        asyncio.CancelledError: When cancelled; a move has held every arm. The record is written first.
    """
    request = job.request
    plan = servo_plan(request.loaded, request.action, request.step.primary, job.run.view.settings,
                      bar_layouts(request.loaded.process))
    if isinstance(plan, str):
        run.ended(REFUSED, "refused", plan)
        return
    say_feedback(job, plan)
    problem = insert_end_problem(job, plan)
    if problem:
        run.ended(REFUSED, "refused", problem)
        return
    loop = ServoLoop(job, run, plan)
    try:
        await loop.run()
    except BaseException as error:
        # * The record below says how it ended; the plugin's own record of the part refines the reason after.
        if not run.result:
            cancelled = isinstance(error, asyncio.CancelledError)
            what = "cancelled" if cancelled else f"raised {error!r}"
            if run.progress is not None and run.progress.holds:
                where = f"during pass {loop.passes}'s move; every arm held"
            else:
                where = f"before the move of pass {loop.passes}; nothing sent" if loop.passes else \
                    "before the first pass; nothing sent"
            run.ended(CANCELLED if cancelled else FAILED, "cancelled" if cancelled else "raised", f"{what} {where}")
        raise
    finally:
        loop.close()


class ServoLoop:
    """One servo's state across its passes: the targets, the readings and the moves, the bar tracked.

    Attributes:
        targets: Flange id -> the pose the next pass aims at: authored, or bar-corrected once `engaged`.
        engaged: A usable MoCap reading has set the targets (spec D4).
        readings: One dict per reading (reading 0 before any move), as the record keeps them.
        moves: One dict per pass that moved (or was built in a dry run).
        passes: The pass running, 0 before the first.
    """

    def __init__(self, job: Job, run: PartRun, plan: ServoPlan):
        """Nothing read yet; the targets are the authored ones."""
        self.job, self.part, self.plan, self.ctx = job, run, plan, job.ctx
        self.serial = job.request.serial
        self.targets = dict(plan.targets)
        self.engaged = self.warned = False
        self.readings: list[dict] = []
        self.moves: list[dict] = []
        self.passes = 0
        self.force0: dict[str, np.ndarray] | None = None
        self.start_base = None
        self.tracked: TrackedObject | None = None
        self.name = f"servo_{plan.bar.rsplit('/', 1)[-1]}" if plan.bar else ""

    async def run(self) -> None:
        """Track the bar, read once, then up to MAX_PASSES passes; ends through `run.ended`."""
        job, run, plan = self.job, self.part, self.plan
        await self._track()
        robot = self._live()
        if robot is None:
            run.ended(REFUSED, "refused", f"{job.who} {NOT_LIVE}")
            return
        self.start_base = robot.base
        bar = self._measure(robot, 0, None, None).get("bar")
        if plan.by_bar and bar is not None and bar_on_target(bar):  # (W:2404-2411)
            run.ended(DONE, "on target", f"the MoCap bar is already on its target "
                                         f"({deviation_text(bar['mocap_vs_design'])}, under {BAR_TOL[0] * 1e3:.1f} mm "
                                         f"/ {math.degrees(BAR_TOL[1]):.1f} deg)")
            return
        for k in range(1, MAX_PASSES + 1):
            self.passes = k
            ended = await self._pass(k)
            if ended is not None:
                run.ended(*ended)
                return
        run.ended(FAILED, "not converged", f"ran all {MAX_PASSES} passes without converging")  # (W:2637-2639)

    async def _pass(self, k: int) -> tuple[str, str, str] | None:
        """Plan, confirm, move and read once; (result, status, reason) when the loop ends, None to go on."""
        job, run, plan, ctx = self.job, self.part, self.plan, self.ctx
        request, view, loaded = job.request, job.run.view, job.request.loaded
        problem = world_problem(job, motion=False)
        if problem:
            return REFUSED, "refused", problem
        robot = self._live()
        if robot is None:
            return REFUSED, "refused", f"{job.who} {NOT_LIVE}"
        snapshot = ctx.scene.snapshot
        names = tuple(name for arm in plan.arms for name in arm_joint_names(arm))
        as_free, ignore_built = request.transfer_as_free(), request.ignore_built()
        obstacles = request.obstacles_collide()
        # * The basis names the primary's step (what is current); the plan is made in the insert's scene after a
        #   transfer (`plan.step_index`), where the bar is in the tools.
        basis = basis_of(snapshot, loaded, loaded.steps[loaded.index_of(request.action, plan.primary)], self.serial,
                         names, robot.joints, as_free, view.statuses, ignore_built, ctx.now(),
                         obstacles_collide=obstacles)
        ask = ServoAsk(dict(self.targets), TOOL0_TOL, plan.linear_only, plan.held)
        planner: MotionPlannerContract = ctx.require("motion_planner")
        async with timeout(PLAN_TIME):
            step = await planner.request_servo_step(MotionPlanRequest(
                loaded, loaded.steps[plan.step_index], snapshot, ctx.config.robots, beliefs=view.beliefs,
                transfer_as_free=as_free, ignore_built=ignore_built, obstacles_collide=obstacles, servo=ask,
                label=f"servo {k}/{MAX_PASSES} {plan.target}"))
        route = step.route
        correction = {flange_side(flange): metres * 1e3 for flange, metres in step.correction.items()}
        flanges_move = ", ".join(f"{SIDE.get(side, side)} {mm:.2f} mm" for side, mm in correction.items())
        if step.nothing_left:  # (W:2428-2441)
            return DONE, "nothing left", (f"nothing left to plan (correction {flanges_move}, under "
                                          f"{TOOL0_TOL * 1e3:.1f} mm){self._bar_note()}")
        if not step.ok:  # (W:2388-2390, W:2462-2470)
            return FAILED, "no plan", f"pass {k}: {route.result.reason}"
        trajectory = route.result.trajectory
        joint_names, rows = list(trajectory.joint_names), np.array(trajectory.positions, dtype=float)
        waypoints, net, travel = excursion(rows)
        traj_time = recommended_time(travel, MAX_TRAJ_TIME)  # (spec A8)
        command = live_command_of(job, joint_names, rows, traj_time)
        if isinstance(command, str):
            return REFUSED, "refused", command
        run.command = command.as_record()
        warn = travel > CONFIRM_TRAVEL
        move = {"planner": route.planner, "attempts": route.attempts, "rows": waypoints, "ik": ik_record(route.ik),
                "correction_mm": correction, "excursion_deg": math.degrees(travel), "net_deg": math.degrees(net),
                "traj_time": traj_time, "duration": None, "prompted": k == 1 or warn, "outcome": None}
        if move["prompted"]:  # (W:2484-2516, spec Q4)
            text = self._prompt(k, route, move, flanges_move)
            if not await confirmed(job, run, text, warn=warn):
                return CANCELLED, "cancelled", f"stopped at pass {k}: cancelled at the prompt; nothing sent"
        # ! Checked again after the plan and Confirm, against THIS pass's basis (a servo-only Exec carries none in the
        #   request); the command is rebuilt from the joints now and sent in this same tick.
        command = world_problem(job, motion=True, basis=basis) or live_command_of(job, joint_names, rows, traj_time)
        if isinstance(command, str):
            return REFUSED, "refused", command
        run.command = command.as_record()
        try:
            others = others_basis(job)
        except WorldMoved as unmeasured:
            return REFUSED, "refused", f"can't tell whether the other robots stand still: {unmeasured}"
        run.progress = SendProgress()
        self.moves.append(move)
        with ctx.freeze(others) as run.freeze:
            outcome = await send_motion(ctx, job.robot, command, dry_run=job.dry_run, watch=lambda: others_moving(job),
                                        progress=run.progress)
        run.outcome = outcome.as_record()
        move.update(duration=outcome.seconds, outcome=outcome.status)
        if outcome.status == SENT_DRY:
            return DRY_RUN, outcome.status, f"pass {k} built: {outcome.reason}"
        if outcome.status != ARRIVED:  # (W:2535)
            result = {SENT_REFUSED: REFUSED, SENT_NOT_HELD: HOLD_NOT_SENT}.get(outcome.status, FAILED)
            return result, outcome.status, outcome.reason
        job.moved_at[self.serial] = ctx.now()
        run.end = {name: value for arm, joints in (outcome.end or {}).items() if joints is not None
                   for name, value in zip(arm_joint_names(arm), joints)}
        await ctx.sleep(SETTLE)  # (W:2535-2545)
        robot = self._live()
        if robot is None:
            return FAILED, "not live", f"{job.who} {NOT_LIVE} after pass {k}: no reading"
        run.base = robot.base
        reading = self._measure(robot, k, joint_names, rows[-1])
        stop = stop_after_move(reading, plan, self.engaged)  # (W:2603-2639)
        if stop is not None:
            return stop[0], "converged" if stop[0] == DONE else "stopped", stop[1]
        return None

    def _prompt(self, k: int, route: RouteResult, move: dict, flanges_move: str) -> str:
        """The question before a move: the first pass, or a later one travelling over CONFIRM_TRAVEL (spec §2.7)."""
        plan = self.plan
        path = (f"travels {move['excursion_deg']:.1f}° (ends {move['net_deg']:.1f}° from where it starts), "
                f"{move['traj_time']:.0f} s; flanges move {flanges_move}")
        if k == 1:
            aim = f"the MoCap bar {plan.bar.rsplit('/', 1)[-1]}" if self.engaged else "tool0"
            return (f"{plan.target} — SERVO 1/{MAX_PASSES} onto {SERVO_TARGET[plan.kind]} ({aim}): {route.planner}, "
                    f"{move['rows']} rows, {path}. Confirm to move; Cancel stops and records.")
        return (f"{plan.target} — SAFEGUARD servo {k}/{MAX_PASSES}: this path {path} ({route.planner}, "
                f"{move['rows']} rows). Confirm to move, or Cancel to stop and record.")

    def _bar_note(self) -> str:
        """How far the MoCap bar still is from its target, when steering on it (W:2434-2437); else ''."""
        bar = self.readings[-1].get("bar") if self.readings else None
        if not (self.plan.by_bar and self.engaged and bar is not None and bar["mocap_vs_design"] is not None):
            return ""
        return f"; the MoCap bar is {deviation_text(bar['mocap_vs_design'])} from its target"

    def _live(self) -> RobotObject | None:
        """The acting robot in the snapshot, measured now; None when not."""
        robot = self.ctx.scene.snapshot.robots.get(robot_id(self.serial))
        return robot if robot is not None and is_live(robot, self.ctx.now()) else None

    # --- --- --- --- --- READINGS --- --- --- --- ---

    def _measure(self, robot: RobotObject, k: int, names: list[str] | None, end_row: np.ndarray | None) -> dict:
        """Reading `k`: tool0 against the authored targets, base drift, forces, tracking, and the bar (W:2282-2357).

        In bar mode a usable reading sets the next pass's targets (`bar_targets`); the first unusable one before
        that is said once.
        """
        job, plan, ctx = self.job, self.plan, self.ctx
        flanges = flange_poses(robot, plan.flanges)
        reading: dict[str, Any] = {"iter": k, "tool0": tool0_errors(flanges, plan.targets),  # (W:2395-2396)
                                   "base": base_drift(robot.base, self.start_base),
                                   "wrench": wrench_magnitudes(job.robot, plan.arms)}
        force = force_vectors(job.robot, plan.arms)
        if force is not None:  # (W:2297-2302)
            if self.force0 is None:
                self.force0 = force
            reading["force_change_N"] = {side: float(np.linalg.norm(force[side] - self.force0[side]))
                                         for side in force}
        if end_row is not None:  # (W:1565-1606)
            gaps = tracking_gaps(robot, names, end_row, plan.arms)
            reading["track_mm"] = {side: gap * 1e3 for side, gap in gaps.items()}
        tool0 = " ".join(f"{SIDE.get(side, side)}={err['pos_norm_mm']:.2f}" for side, err in reading["tool0"].items())
        track = f" | track {sides(reading['track_mm'], '.2f')} mm" if "track_mm" in reading else ""
        ctx.log_info(f"[servo {k}] tool0 pos {tool0} mm{track}")
        if plan.layout is not None and plan.holder is not None:
            reading["bar"] = self._bar(flanges, k)
        self.readings.append(reading)
        job.show(line(k, reading, plan))
        job.measured(reading)
        return reading

    def _bar(self, flanges: dict, k: int) -> dict:
        """The held bar as MoCap and FK see it; in bar mode, the targets after it (W:2322-2357)."""
        plan, ctx = self.plan, self.ctx
        holder, grasp = plan.holder
        fit = source = None
        if self.tracked is not None and self.tracked.markers is not None:
            sensed = sensed_from_raw(self.tracked.markers, ctx.now())
            # The fit is matched around the FK bar first (W:1820).
            fit = fit_bar(sensed, plan.layout, max_age=MARKER_MAX_AGE, hint=compose(flanges[holder], grasp).matrix())
            source = sensed.source
        bar = bar_reading(plan, flanges, fit, source, plan.design_bar)
        ctx.log_info(f"[servo {k}] bar {bar['bar']}: MoCap-vs-design {deviation_text(bar['mocap_vs_design'])}, "
                     f"FK-vs-design {deviation_text(bar['fk_vs_design'])}, MoCap-vs-FK "
                     f"{deviation_text(bar['mocap_vs_fk'])}"
                     + ("" if bar["usable"] else f" (not usable: {bar['reason']})"))
        if not plan.by_bar:
            return bar
        if bar["usable"]:
            # * One rigid correction for both flanges: the live flanges moved by what carries the MoCap bar onto its
            #   target (W:2341-2348).
            self.targets, self.engaged = bar_targets(np.asarray(bar["T_corr"]), flanges), True
        elif not self.engaged and not self.warned:  # (W:2349-2357)
            self.warned = True
            ctx.log_warn(f"[servo {k}] the MoCap bar is not usable ({bar['reason']}); the loop aims tool0 at the "
                         f"authored target until a fit is usable.")
        return bar

    # --- --- --- --- --- THE BAR'S MARKERS, AND THE END --- --- --- --- ---

    async def _track(self) -> None:
        """Track the bar's own Motive asset (spec A5) and wait up to FIRST_SAMPLE for its first markers."""
        plan, ctx = self.plan, self.ctx
        if plan.layout is None or plan.holder is None:
            return
        mocap_id = bar_asset_id(plan.bar)
        if mocap_id is None:
            ctx.log_warn(f"[servo] {plan.bar} has no Motive asset id: the bar is not measured")
            return
        ctx.untrack_object(self.name)  # a run that crashed may have left it
        self.tracked = ctx.track_object(self.name, mocap_id, label=f"servo {plan.bar} markers ({mocap_id})")
        until = ctx.now() + FIRST_SAMPLE
        while self.tracked.markers is None and ctx.now() < until:
            await ctx.sleep(0.1)
        if self.tracked.markers is None:
            ctx.log_warn(f"[servo] no marker sample of {plan.bar} (asset {mocap_id}) within {FIRST_SAMPLE:.0f} s: is "
                         f"Motive streaming it?")

    def close(self) -> None:
        """Untrack the bar, say where it ended, carry the correction, and record the run (W:2651-2668)."""
        job, run, plan, ctx = self.job, self.part, self.plan, self.ctx
        if self.tracked is not None:
            ctx.untrack_object(self.name)
        last = next((reading["bar"] for reading in reversed(self.readings) if reading.get("bar") is not None), None)
        if last is not None:  # (W:2651-2657)
            ctx.log_info(f"### bar pose at termination ({last['bar']}): FK-vs-design "
                         f"{deviation_text(last['fk_vs_design'])}, MoCap-vs-design "
                         f"{deviation_text(last['mocap_vs_design'])}, MoCap-vs-FK "
                         f"{deviation_text(last['mocap_vs_fk'])}")
        if plan.by_bar and self.engaged and plan.kind == TRANSFER:  # (W:2658-2662)
            insert = job.request.loaded.steps[plan.step_index]
            run.carry = Carried(job.request.loaded, plan.action, insert.movement_index, self.serial, ctx.now())
            ctx.log_info(f"[servo] the MoCap-bar correction is carried into the plan of {insert.movement.id}.")
        if self.moves or len(self.readings) > 1 or last is not None:  # (W:2664-2668; a move cut short counts too)
            payload = servo_payload(plan, robot=robot_id(self.serial), serial=self.serial, readings=self.readings,
                                    moves=self.moves, status=run.result, reason=run.reason, engaged=self.engaged,
                                    dry_run=job.dry_run)
            run.servo_file = servo_file(job.run.view, plan)
            job.run.request_record("servo", action=plan.action, movement=plan.target,
                                   data={"status": run.result, "passes": len(self.moves)}, file=payload, kind="servo",
                                   figure=plot_servo)
