"""
Example 7: lease the robot you move, and freeze what you rely on standing still.

Wiggle turns one arm's wrist 15 deg and back, as a task:

  1. `ctx.lease_robots([serial])`: every motion command needs the robot's lease, and now only this task has it.
     Jogging the robot from Control, or another plugin's commit, raises NotLeased. The other robots stay free.
  2. `take_basis(...)`: measure what must stand still: the other robots, this robot's base and its other arms.
  3. `ctx.freeze(basis)`, optional: every tick the monitor compares them with the basis. If one moves past the
     tolerance, or cannot be measured any more, it cancels this task and logs why. That also catches what this
     monitor did not send: another monitor, a joystick, a tool's buttons, a pushed base.

! Neither the lease nor the freeze stops a moving arm: on cancel, hold it yourself (`except` below).
! Run it on a simulation first: it really moves the arm.

Run with:  -p plugins:="['example_lease_and_freeze']"
"""

from __future__ import annotations

import asyncio

import numpy as np
import viser

from ...plugin_api.concurrency import WaitTimeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...robot_interface.arm import MOVING_WITHIN, ArmInterface
from ...robot_interface.soft_stop import MotionRefused
from ...shared.execution.send import ARRIVED_TOLERANCE
from ...ui.style import BUSY, FAIL, NONE, OK, REFUSAL_SHOWN, SECTION_CTRL, chip, chip_line, section
from ...world.freeze import take_basis

#: The wiggle: wrist 3 turns this far, radians, then back.
WIGGLE = np.radians([0.0, 0.0, 0.0, 0.0, 0.0, 15.0])
#: Seconds a move may take beyond its planned duration before the wiggle gives up.
MOVE_MARGIN = 5.0


@register
class ExampleLeaseAndFreezePlugin(HuskyPlugin):
    """Wiggle one arm under its robot's lease and a freeze of everything else; shows who holds each robot."""

    name = "example_lease_and_freeze"

    def __init__(self):
        """Start idle."""
        self._task: asyncio.Task | None = None
        #: How the last wiggle ended: a short chip label, and the whole reason. It stays until the next wiggle.
        self.outcome = ("", "")
        #: Why the last click did nothing: label, reason and time; its chip goes after REFUSAL_SHOWN.
        self._refused: tuple[str, str, float] | None = None

    @property
    def running(self) -> bool:
        """bool: Whether a wiggle is running right now."""
        return self._task is not None and not self._task.done()

    def setup(self, ctx: PluginContext) -> None:
        """Build the arm choice, the buttons and the status.

        Args:
            ctx: This plugin's context.
        """
        arms = [f"{serial} {name}" for serial, robot in ctx.world.robots.items() for name in robot.arms]
        with ctx.view.ui() as gui:
            self._status: viser.GuiHtmlHandle = gui.add_html("")
            gui.add_html(section("wiggle", SECTION_CTRL))
            self._arm = gui.add_dropdown("Arm", arms or ["no arms"], disabled=not arms)
            buttons = gui.add_button_group("Wiggle", ["Wiggle", "Cancel"])
        buttons.on_click(ctx.defer_value("wiggle button", lambda clicked: self._on_button(ctx, clicked)))

    def _on_button(self, ctx: PluginContext, clicked: str) -> None:
        """Start or cancel a wiggle, or say why the click does nothing now."""
        if clicked == "Wiggle" and not self.running and not self._arm.disabled:
            serial, arm_name = self._arm.value.split(" ", 1)
            self._task = ctx.spawn(f"wiggle {serial} {arm_name}", self._wiggle(ctx, serial, arm_name))
        elif clicked == "Cancel" and self.running:
            self._task.cancel()
        else:
            # ! Every click that does nothing says why.
            why = "a wiggle is running" if self.running else "no arm" if self._arm.disabled else "nothing to cancel"
            self._refused = (f"{clicked.lower()} ignored", why, ctx.now())
            ctx.log_info(f"{clicked} ignored: {why}")

    async def _wiggle(self, ctx: PluginContext, serial: str, arm_name: str) -> None:
        """Turn the wrist out and back, under the robot's lease and a freeze of everything else.

        Args:
            ctx: This plugin's context.
            serial: The robot to move.
            arm_name: Its arm to move.
        """
        robot = ctx.world.robots[serial]
        arm = robot.arms[arm_name]
        self.outcome, self._refused = ("running", ""), None
        freeze = None
        try:
            with ctx.lease_robots([serial], f"wiggle {serial} {arm_name}"):
                # * Measure once the lease is held, so nothing in this monitor moves the robot in between.
                basis = take_basis(ctx.world, ctx.now(),
                                   robots=[other for other in ctx.world.robots if other != serial], bases=[serial],
                                   arms=[(serial, name) for name in robot.arms if name != arm_name])
                with ctx.freeze(basis) as freeze:
                    start = arm.joint_vector()
                    if start is None:
                        self.outcome = ("not started", f"{serial} {arm_name}: no joint state yet")
                        return
                    for target in (start + WIGGLE, start):
                        duration = arm.send_joint_move(target)
                        if duration is None:
                            refusal = arm.last_refusal
                            self.outcome = ("move refused", refusal.reason if refusal else "see the log")
                            return
                        # ! Wait on the joints, not `is_executing`: it clears during a slow move (the pendant's speed
                        #   slider), and the lease would go back while the arm still moves.
                        await ctx.wait_until(lambda: _settled(ctx, arm, target), timeout_s=duration + MOVE_MARGIN,
                                             description=f"{serial} {arm_name} to arrive")
            self.outcome = ("done", "")
        except MotionRefused as refused:
            # Expected: another holder has the robot's lease, or something moved before the start.
            self.outcome, self._refused = ("", ""), (refused.label, str(refused), ctx.now())
        except WaitTimeout as timeout:
            self.outcome = ("timed out", str(timeout))
            ctx.log_warn(str(timeout))
            arm.hold()
        except asyncio.CancelledError:
            # * Cancel, Stop all, or the freeze; the freeze says if it was the freeze.
            self.outcome = ("freeze broken", freeze.problem) if freeze is not None and freeze.problem \
                else ("cancelled", "")
            arm.hold()  # ! Holding needs no lease, and giving the lease back never stops an arm by itself.
            raise

    def draw(self, ctx: PluginContext) -> None:
        """Show who holds each robot, and how the last wiggle ended.

        Args:
            ctx: This plugin's context.
        """
        chips = ""
        for serial, robot in ctx.world.robots.items():
            lease = robot.lease_slot.lease
            chips += chip(f"{serial} {'held' if lease else 'free'}", BUSY if lease else NONE,
                          lease.holder if lease else "")
        label, reason = self.outcome
        if label:
            chips += chip(label, OK if label in ("running", "done") else FAIL, reason)
        if self._refused is not None and ctx.now() - self._refused[2] <= REFUSAL_SHOWN:
            chips += chip(self._refused[0], FAIL, self._refused[1])
        self._status.content = chip_line(chips)


def _settled(ctx: PluginContext, arm: ArmInterface, target: np.ndarray) -> bool:
    """Whether `arm` is at `target` (ARRIVED_TOLERANCE) and has not moved for MOVING_WITHIN, so it takes a new move."""
    here, moved = arm.joint_vector(), arm.state.last_motion_time
    return (here is not None and float(np.max(np.abs(here - target))) < ARRIVED_TOLERANCE
            and (moved is None or ctx.now() - moved >= MOVING_WITHIN))
