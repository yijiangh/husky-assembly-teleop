"""
The D-pad of the steps that drive (Marker error, Validate, Punch): drives the run's base while a button is held.

The lease, and a clearance while Clearance is ticked, are held only while a button is held, so the arm planner can move
the arm in between; the D-pad stays off while a procedure works (it drives while one only waits for Go) or a robot move
is refused (`move_refusal`: Area clear or E-stop at hand). A hold starting too close is refused, nothing sent; a
clearance trip or Stop all ends the drive task; the next press starts it again, and a refused press says why.
"""

from __future__ import annotations

import asyncio
from contextlib import nullcontext
from typing import TYPE_CHECKING, Callable

import viser

from scaffolding_assembly_core.ids import robot_id

from ....robot_interface.base import PLATFORM_VELOCITY_CONTROLLER
from ....robot_interface.soft_stop import MotionRefused
from ....shared.guide.procedure import GO_TIMEOUT, Procedure
from ....shared.planner.planner_service import PlannerService
from ....shared.teleop.clearance_toggle import ClearanceToggle
from ....shared.teleop.dpad import UNGUARDED_COLOR, Hold, color_dpad
from ....world.checks import BAD, WARN, Check
from ....world.clearance import ALL_ARMS, STOP_DISTANCE
from ....world.stand_ins import unknown_robots

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext
    from ..run import Run


class Driving:
    """Drives one run's base from the D-pad buttons of every step that has them.

    Attributes:
        on_driven: Called on every tick the base was sent a twist, e.g. to judge marker errors while moving; None
            for nothing.
        clearance: Clearance, below every D-pad: while ticked, a hold soft stops the base within STOP_DISTANCE of
            anything, and a hold starting that close is refused.
        buttons: Every D-pad's buttons, amber while Clearance is unticked (`draw`).
    """

    def __init__(self, procedure: Procedure):
        """Not driving; the D-pad stays off while `procedure` runs."""
        self.procedure = procedure
        self.on_driven: Callable[[PluginContext], None] | None = None
        self.clearance = ClearanceToggle(f"Soft stop the base within {STOP_DISTANCE * 100:g} cm of anything; a hold "
                                         f"starting that close is refused, and arms not measured count as last seen "
                                         f"or stowed. Untick to drive out of a tight spot; every D-pad here has the "
                                         f"same box.")
        self.buttons: list[viser.GuiButtonHandle] = []
        self._held: Hold | None = None
        self._task: asyncio.Task | None = None
        #: What the D-pad drives while a step with one shows: the context and run; None otherwise.
        self._driving: tuple[PluginContext, Run] | None = None
        #: Why the robot may not move now, or None (`CalibrationStep.move_refusal`).
        self._refusal: Callable[[], str | None] = lambda: None

    def on_hold(self, hold: Hold) -> None:
        """A D-pad button is held (`build_velocity_inputs`): drive, starting the task again if something ended it."""
        self._held = hold
        if self._driving is not None:
            self._spawn(*self._driving)

    def draw(self) -> None:
        """Colour the D-pads amber while Clearance is unticked."""
        color_dpad(self.buttons, None if self.clearance.value else UNGUARDED_COLOR)

    def start(self, ctx: PluginContext, run: Run, refusal: Callable[[], str | None] = lambda: None) -> None:
        """Let the D-pad drive the run's base while `refusal()` is None."""
        self._driving, self._refusal = (ctx, run), refusal
        self._spawn(ctx, run)

    def _spawn(self, ctx: PluginContext, run: Run) -> None:
        if self._task is None or self._task.done():
            self._task = ctx.spawn(f"drive {run.serial}", self._drive(ctx, run))

    def stop(self) -> None:
        """Stop driving; the base gets a zero twist."""
        self._driving = None
        if self._task is not None:
            self._task.cancel()
            self._task = None

    def _holding(self, ctx: PluginContext) -> bool:
        return self._held is not None and not self._held.released(ctx.now())

    async def _drive(self, ctx: PluginContext, run: Run) -> None:
        """Drive the base while a button is held; save the run after each hold."""
        base, procedure = run.robot.base, self.procedure
        try:
            while True:
                await ctx.next_tick()
                if not self._holding(ctx):
                    continue
                refusal = self._refusal()
                # * A procedure that only waits for Go moves nothing: the base may be driven meanwhile.
                busy = procedure.running and not procedure.waiting
                if busy or refusal is not None:
                    procedure.outcome = (Check("D-pad off", WARN, f"{procedure.doing or 'a procedure'} runs")
                                         if busy else Check("D-pad refused", BAD, refusal))
                    await ctx.wait_until(lambda: not self._holding(ctx), timeout_s=GO_TIMEOUT,
                                         description="D-pad up")
                    continue
                try:
                    # * As robot_control's D-pad: other robots not measured at the press are checked as stand-ins.
                    others = unknown_robots(ctx.scene.snapshot, {robot_id(run.serial)})
                    # * Not against itself: driving moves no arm, so nothing comes closer to the rest of the robot.
                    #   Arms not measured count as last seen, or stowed. A start too close is refused, nothing sent.
                    guard = (ctx.clearance([run.serial], unknown_arms=ALL_ARMS, unknown_robots=others,
                                           refuse_first=True, check_self=False)
                             if self.clearance.value else nullcontext())
                    with ctx.lease_robots([run.serial], "calibration drive"), guard as clearance:
                        try:
                            await PlannerService.first_check(ctx, clearance)
                        except asyncio.CancelledError:
                            if refusal := PlannerService.start_refusal(clearance):
                                procedure.outcome = Check("D-pad refused", BAD, refusal)
                            raise
                        if not base.controllers.is_active(PLATFORM_VELOCITY_CONTROLLER):
                            base.controllers.switch(PLATFORM_VELOCITY_CONTROLLER)
                        while self._holding(ctx):
                            if base.send_twist(*self._held.twist()) and self.on_driven is not None:
                                self.on_driven(ctx)
                            await ctx.next_tick()
                        base.send_twist(0.0, 0.0, require_controller=False)
                except MotionRefused as refused:
                    procedure.outcome = Check("D-pad refused", BAD, str(refused))
                    await ctx.wait_until(lambda: not self._holding(ctx), timeout_s=GO_TIMEOUT,
                                         description="D-pad up")
                run.save()
        finally:
            base.send_twist(0.0, 0.0, require_controller=False)
            run.save()
