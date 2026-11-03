"""
The arm moved by hand in the steps that put a tip on a target (Tip offset, Punch): free drive, and tool steps.

One task per shown step owns the arm. It holds the robot's lease only while free drive is on or a tool step moves,
and does nothing while a procedure runs or a robot move is refused (`move_refusal`: Area clear or E-stop at hand
not confirmed within the hour), so a take sees a still arm. While Clearance is ticked, a tool step goes out only once
the robot is found ARM_STOP_DISTANCE clear, and soft stops it closer.
Ending free drive holds the arm on the joint controller. What went wrong is shown as the procedure's outcome until free
drive runs again or a tool step goes out.
"""

from __future__ import annotations

import asyncio
from contextlib import nullcontext
from typing import TYPE_CHECKING, Callable

import numpy as np
import viser

from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id

from ....plugin_api.concurrency import WaitTimeout
from ....robot_interface.arm import FREE_DRIVE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER
from ....robot_interface.soft_stop import MotionRefused
from ....robot_interface.tool_step import robot_step_in_tool0
from ....shared.guide.procedure import Procedure
from ....shared.teleop.clearance_toggle import ClearanceToggle
from ....shared.teleop.dpad import HOLD_CALLBACK_HZ, HOLD_TIMEOUT
from ....shared.teleop.tool_pad import ROBOT_FRAME, TOOL_FRAME, build_tool_step_inputs
from ....world.checks import BAD, WARN, Check
from ....world.clearance import ARM_STOP_DISTANCE, Clearance
from ....world.stand_ins import unknown_robots
from .motion import MOVE_MARGIN, stopped, wait_holding, wait_rested

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext
    from ....robot_interface.arm import ArmInterface
    from ..run import Run

#: Seconds a controller switch, or a clearance's first check, may take.
SWITCH_TIMEOUT = 3.0
#: Clicks kept while a tool step moves; more are dropped.
QUEUED_STEPS = 5


class HandMoves:
    """Free drive and tool steps of the run's arm, from the widgets of every step that has them (`build_inputs`).

    Free drive runs while Free drive (hold) is held, or while Keep free drive on is ticked.

    Attributes:
        free_drive: Whether Keep free drive on is ticked; unticked when free drive ends otherwise (a procedure, Stop
            all, a refusal).
        clearance: Clearance, below every tool pad: while ticked, a tool step keeps the robot ARM_STOP_DISTANCE from
            anything, itself included.
    """

    def __init__(self, procedure: Procedure):
        """Nothing wanted; off while `procedure` runs."""
        self.procedure = procedure
        self.free_drive = False
        self.clearance = ClearanceToggle(
            f"A tool step goes out only with the robot {ARM_STOP_DISTANCE * 100:g} cm clear of anything, itself "
            f"included, and soft stops it closer. Untick to step a tip onto its target; free drive is never checked.")
        #: `ctx.now()` until which the hold button counts as held: HOLD_TIMEOUT after its last report.
        self._held_until = float("-inf")
        #: Queued tool pad clicks: the step, its pivot in tool0, and the frame it is in (`tool_pad.FRAMES`).
        self._steps: list[tuple[Pose, np.ndarray, str]] = []
        self._task: asyncio.Task | None = None
        self._boxes: list[viser.GuiCheckboxHandle] = []
        #: The context and run while a step with these widgets shows; None otherwise.
        self._moving: tuple[PluginContext, Run] | None = None
        #: Why the robot may not move now, or None (`CalibrationStep.move_refusal`).
        self._refusal: Callable[[], str | None] = lambda: None
        #: The outcome these hand moves showed last, cleared once the arm moves by hand again (`_report`).
        self._reported: Check | None = None

    def build_inputs(self, ctx: PluginContext, gui: viser.GuiApi, pivot: Callable[[], np.ndarray],
                     phone: bool = False) -> None:
        """Add Free drive (hold), Keep free drive on and the tool pad (`phone`); tool steps turn about `pivot()`."""
        hold = gui.add_button("Free drive (hold)", icon=viser.Icon.HAND_GRAB,
                              hint="The arm can be pushed by hand while you hold this; released, the joint controller "
                                   "holds it. Off while a procedure runs.")
        hold.on_hold(ctx.defer("free drive hold", lambda: self.hold_free_drive(ctx.now())),
                     callback_hz=HOLD_CALLBACK_HZ)
        free = gui.add_checkbox("Keep free drive on", initial_value=False,
                                hint="Free drive without holding the button, until unticked; only with someone at the "
                                     "arm. Off while a procedure runs.")
        free.on_update(ctx.defer_value("free drive", self.want_free_drive, operator_only=True))
        build_tool_step_inputs(ctx, gui, "calibration", lambda step, frame: self.add_step(step, pivot(), frame),
                               self.clearance, phone)
        self._boxes.append(free)

    def draw(self) -> None:
        """Show in every Keep free drive on checkbox whether it is ticked."""
        for box in self._boxes:
            if box.value != self.free_drive:
                box.value = self.free_drive

    def start(self, ctx: PluginContext, run: Run, refusal: Callable[[], str | None] = lambda: None) -> None:
        """Let the widgets move the run's arm while `refusal()` is None."""
        self._moving, self._refusal = (ctx, run), refusal
        self._spawn()

    def stop(self) -> None:
        """Stop moving the arm: free drive ends, a tool step under way is held, queued steps are dropped."""
        self._moving = None
        self.free_drive, self._held_until, self._steps = False, float("-inf"), []
        if self._task is not None:
            self._task.cancel()
            self._task = None

    def _why_not(self) -> str | None:
        """Why the arm may not move by hand now, or None."""
        if self.procedure.running:
            return f"{self.procedure.doing or 'a procedure'} runs"
        return self._refusal() if self._moving is not None else "no step that moves the arm by hand shows"

    def _report(self, check: Check) -> None:
        """Show what went wrong as the procedure's outcome, until the arm moves by hand again."""
        self.procedure.outcome = self._reported = check

    def _moved(self) -> None:
        """The arm moves by hand again: clear what these hand moves showed, not another source's outcome."""
        if self._reported is not None and self.procedure.outcome is self._reported:
            self.procedure.outcome = None
        self._reported = None

    def _freeing(self, now: float) -> bool:
        """Whether free drive is wanted now: the button held, or Keep free drive on ticked."""
        return self.free_drive or now <= self._held_until

    def hold_free_drive(self, now: float) -> None:
        """A report of Free drive (hold) held: free drive for HOLD_TIMEOUT more; drops the queued tool steps."""
        why = self._why_not()
        if why is not None:
            self._report(Check("free drive refused", BAD, why))
            return
        self._held_until, self._steps = now + HOLD_TIMEOUT, []
        self._spawn()

    def want_free_drive(self, on: bool) -> None:
        """Keep free drive on ticked or unticked; ticking drops the queued tool steps, meant from the pose before."""
        why = self._why_not() if on else None
        if why is not None:
            self._report(Check("free drive refused", BAD, why))
            on = False
        self.free_drive = on
        if on:
            self._steps = []
        self._spawn()

    def add_step(self, step: Pose, pivot: np.ndarray, frame: str = TOOL_FRAME) -> None:
        """A tool pad click: queue the step, given in `frame`, turning about `pivot` in tool0."""
        why = self._why_not() or ("free drive is on" if self.free_drive else None) or (
            f"{QUEUED_STEPS} steps are waiting already" if len(self._steps) >= QUEUED_STEPS else None)
        if why is not None:
            self._report(Check("tool step not sent", BAD, why))
            return
        self._steps.append((step, np.asarray(pivot, dtype=float), frame))
        self._spawn()

    def _spawn(self) -> None:
        """Start the task again if something ended it (a soft stop cancels it)."""
        if self._moving is not None and (self._task is None or self._task.done()):
            ctx, run = self._moving
            self._task = ctx.spawn(f"hand moves {run.serial}", self._move(ctx, run))

    async def _move(self, ctx: PluginContext, run: Run) -> None:
        """Keep free drive on while wanted, else send the queued tool steps one by one."""
        arm = run.robot.arms[run.arm]
        try:
            while True:
                await ctx.next_tick()
                why = self._why_not()
                if why is not None:
                    # * A procedure starting ends them as documented; only a move turning refused is news.
                    if (self._freeing(ctx.now()) or self._steps) and not self.procedure.running:
                        self._report(Check("arm moves by hand ended", WARN, why))
                    self.free_drive, self._held_until, self._steps = False, float("-inf"), []
                    continue
                try:
                    if self._freeing(ctx.now()):
                        await self._free_drive(ctx, run, arm)
                    elif self._steps:
                        await self._step(ctx, run, arm, *self._steps.pop(0))
                except (MotionRefused, WaitTimeout) as refused:
                    self.free_drive, self._held_until, self._steps = False, float("-inf"), []
                    self._report(Check("arm not moved", BAD, str(refused)))
        finally:
            self.free_drive = False
            _end_free_drive(arm)

    async def _free_drive(self, ctx: PluginContext, run: Run, arm: ArmInterface) -> None:
        """Switch to free drive and keep it on while held or kept on, and allowed; hold the arm after."""
        with ctx.lease_robots([run.serial], "calibration free drive"):
            if not arm.controllers.switch(FREE_DRIVE_CONTROLLER):
                self.free_drive, self._held_until = False, float("-inf")
                self._report(Check("free drive refused", BAD, arm.controllers.state.switch_error or "see the log"))
                return
            try:
                while self._freeing(ctx.now()) and self._why_not() is None:
                    # * Stop all, a failed switch or another panel's switch: free drive is over.
                    if arm.controllers.state.requested != FREE_DRIVE_CONTROLLER:
                        self.free_drive, self._held_until = False, float("-inf")
                        self._report(Check("free drive ended", WARN, arm.controllers.state.switch_error
                                           or "another controller was started"))
                        return
                    if arm.controllers.is_active(FREE_DRIVE_CONTROLLER):
                        self._moved()
                    arm.request_free_drive()
                    await ctx.next_tick()
            finally:
                _end_free_drive(arm)

    async def _step(self, ctx: PluginContext, run: Run, arm: ArmInterface, step: Pose, pivot: np.ndarray,
                    frame: str) -> None:
        """Send one tool step on the joint controller once the arm rests, and keep the lease until it has stopped.

        While Clearance is ticked the step goes out only once its clearance found the robot clear; too close at the
        first check, nothing is sent and the queued steps are dropped.
        """
        # * As robot_control's jogs: other robots not measured at the click are checked as stand-ins.
        others = [name for name in run.robot.arms if name != run.arm]
        guard = (ctx.clearance([run.serial], ARM_STOP_DISTANCE, unknown_arms=others, refuse_first=True,
                               unknown_robots=unknown_robots(ctx.scene.snapshot, {robot_id(run.serial)}))
                 if self.clearance.value else nullcontext())
        clearance = None
        try:
            with ctx.lease_robots([run.serial], "calibration tool step"), guard as clearance:
                await self._send_step(ctx, arm, step, pivot, frame, clearance)
        except asyncio.CancelledError:
            if clearance is not None and clearance.refused:
                self._steps = []
                self._report(Check("tool step refused", BAD, f"{clearance.problem}: untick Clearance to step closer"))
            raise

    async def _send_step(self, ctx: PluginContext, arm: ArmInterface, step: Pose, pivot: np.ndarray, frame: str,
                         clearance: Clearance | None) -> None:
        """`_step` under its lease and `clearance` (None while unticked); a robot-frame step goes out in tool0's."""
        controllers = arm.controllers
        if not controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER):
            if not controllers.switch(SCALED_JOINT_TRAJECTORY_CONTROLLER):
                self._report(Check("tool step not sent", BAD, controllers.state.switch_error or "see the log"))
                return
            await ctx.wait_until(lambda: controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER),
                                 timeout_s=SWITCH_TIMEOUT, description="the joint controller")
        await wait_rested(ctx, arm)
        if clearance is not None:
            await ctx.wait_until(lambda: clearance.checked, timeout_s=SWITCH_TIMEOUT,
                                 description="the clearance's first check")
        if frame == ROBOT_FRAME and (here := arm.joint_vector()) is not None and arm.urdf is not None:
            step = robot_step_in_tool0(arm.urdf, arm.config.name, here, step)
        duration = arm.send_tool_step(step, pivot)
        if duration is None:
            refusal = arm.last_refusal
            self._report(Check("tool step refused", BAD, refusal.reason if refusal else "see the log"))
            self._steps = []
            return
        self._moved()
        await wait_holding(ctx, arm, lambda: stopped(arm, ctx.now()), duration + MOVE_MARGIN,
                           "the tool step to end")


def _end_free_drive(arm: ArmInterface) -> None:
    """Hold the arm if free drive runs: that switches back to the joint controller and needs no lease."""
    if arm.controllers.is_active(FREE_DRIVE_CONTROLLER):
        arm.hold()
