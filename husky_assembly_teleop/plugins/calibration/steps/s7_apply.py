"""Apply: give the solution the name the relay reads, ask the relay to reload, and see it apply the new calibration."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING

import viser

from ....colors import rgb
from ....robot_interface.mocap_definitions import OUT_OF_DATE
from ....plugin_api.concurrency import WaitTimeout
from ....world.checks import BAD, GOOD, PENDING, WARN, Check
from ....shared.execution.moving import robot_moving
from ....ui.style import BUSY, indented
from ..storage.session_files import approve_calibration
from .base import CalibrationStep, relay_sha, relay_up_check

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext

#: Seconds the relay gets to announce the reloaded calibration.
RELOAD_TIMEOUT = 5.0


class Apply(CalibrationStep):
    """The relay applying the current solution's output; needs the validation verdict passing or accepted.

    A mount check has nothing to apply: it is finished once Solve wrote it (`Run.finished`).
    """

    name = "Apply"
    instructions = "Press Apply: approve and reload while the robot stands still."
    on_laptop = "Apply on the laptop."

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Apply; the readout below it."""
        button = gui.add_button("Apply: approve and reload", color=rgb(BUSY), icon=viser.Icon.REFRESH,
                                hint="Gives the calibration the name the relay reads and has the relay reload this "
                                     "robot's. Refused while the robot moves.")
        # * The readout last: its lines come and go, and must not move the buttons.
        self._html = gui.add_html("")
        button.on_click(ctx.defer("reload", lambda: self.start(ctx, "reload", self._reload(ctx))))

    async def _reload(self, ctx: PluginContext) -> None:
        """Approve the solution's output, have the relay reload this robot's calibration, and wait to see it applied.

        ! Refused while the robot moves: the reload changes the pose it and its follower drive from.
        """
        run, procedure = self.run, self.cal.procedure
        if why := robot_moving(run.serial, run.robot, ctx.now()):
            raise RuntimeError(f"{why}: reload only while the robot stands still")
        solved = run.current_solved()
        if solved is None or solved.output_sha is None:
            raise RuntimeError("the solution changed while waiting: solve again")
        if solved.output != run.output_file(approved=True):
            # * From here on the relay applies it, also at its next start.
            run.solved = replace(solved, output=approve_calibration(run.session, run.digits))
            ctx.log_info(f"calibration approved: {run.solved.output}")
        future = run.relay.reload_base_calibration(run.robot.config.mocap_id) if run.relay is not None else None
        if future is None:
            raise RuntimeError("the mocap relay does not answer: is it running?")
        procedure.doing = "asking the relay to reload"
        response = await ctx.ros(future)
        if not response.success:
            raise RuntimeError(f"the relay did not reload: {response.message}")
        procedure.doing = "waiting for the relay to announce it"
        try:
            await ctx.wait_until(lambda: relay_sha(run) == solved.output_sha, timeout_s=RELOAD_TIMEOUT,
                                 description="the relay to apply the new calibration")
        except WaitTimeout:
            entry = run.applied()
            raise RuntimeError(f"the relay applies {entry.source if entry else 'no calibration'} instead after "
                               f"{RELOAD_TIMEOUT:g} s; its log says why") from None
        run.applied_sha = solved.output_sha
        run.save()

    def none_yet(self, ctx: PluginContext) -> str | None:
        """Nothing applied yet (a mount check: not written yet)."""
        run = self.run
        if run.mount_check:
            return None if run.finished else "Solve writes the mount check"
        return "not applied yet" if run.applied_sha is None else None

    def out_of_date(self, ctx: PluginContext) -> str | None:
        """An earlier solution applied."""
        return self._earlier_applied() if not self.run.finished else None

    def _earlier_applied(self) -> str:
        """An earlier solution applied, and what to do: Apply again, after Solve once the takes changed since."""
        then = "Apply again" if self.run.current_solved() is not None else "Solve, then Apply again"
        return f"an earlier solution was applied: {then}"

    def checks(self, ctx: PluginContext) -> list[Check]:
        """What the relay applies now, live; whether this run applied is the step's state.

        The chip only warns: a relay restarted, not announced yet or applying another calibration changes nothing
        this run did, nor does the relay finding its calibration built for an earlier body definition. A mount check has
        nothing to apply.
        """
        run = self.run
        solved = run.current_solved()
        if run.mount_check:
            return [Check("mount check", GOOD, f"written to {solved.output.name if solved and solved.output else '-'}; "
                                               f"nothing to apply: only the robot's first arm calibrates the base")]
        relay, now = run.relay, relay_sha(run)
        applies = f"the relay applies {now[:12]}" if now else "the relay publishes the base uncalibrated"
        entry = run.applied()
        if relay is None or relay.state.calibrations_time is None:
            live = Check("relay now", PENDING, "the mocap relay announced no calibrations yet")
        elif entry is not None and entry.definition_status == OUT_OF_DATE:
            live = Check("relay now", WARN, f"{applies}, {entry.definition_detail}")
        elif run.applied_sha is None:
            live = Check("relay now", PENDING, f"{applies}; this run applied nothing yet")
        elif now == run.applied_sha:
            live = Check("relay now", GOOD, "the relay applies what this run applied")
        else:
            live = Check("relay now", WARN, f"{applies} now: restarted, or another calibration applied since this "
                                            f"run's")
        return [live]

    def ready(self, ctx: PluginContext) -> list[Check]:
        """Something to apply, the validation verdict passing or accepted (`Run.validation_good`), and the relay up."""
        run = self.run
        if run.mount_check:
            return [Check("apply", BAD, "nothing to apply: this arm only checks its mount")]
        solved = run.current_solved()
        if run.validation_passes():
            validation = Check("validation", GOOD, "the verdict passes")
        elif run.validation_good():
            validation = Check("validation", WARN, "the verdict fails, accepted in Validate")
        elif not run.current_validation_files():
            validation = Check("validation", BAD, "no verdict yet: sample poses in Validate")
        elif solved is None:
            validation = Check("validation", BAD, "the verdict is pending: Solve first")
        else:
            validation = Check("validation", BAD, "the verdict fails: sample again, or accept it with Done pressed "
                                                  "twice in Validate")
        return [Check("solution", GOOD, f"wrote {solved.output.name}") if solved is not None and solved.output_sha
                else Check("solution", BAD, "solve first"), validation, relay_up_check(run), same_drive_check(run)]

    def draw(self, ctx: PluginContext) -> None:
        """The new calibration's hash and the one the relay applies."""
        solved = self.run.current_solved()
        new = solved.output_sha if solved is not None else None
        self._html.content = indented(f"new calibration: {(new or '-')[:12]}",
                                      f"relay applies: {(relay_sha(self.run) or '-')[:12]}")


def same_drive_check(run) -> Check:
    """Whether the relay reads sessions from the Drive this run writes in: else it never finds what Apply approves."""
    relay = run.relay
    if relay is None or relay.state.calibrations_time is None:
        return Check("relay's Drive", PENDING, "the relay announced nothing yet")
    theirs, ours = relay.state.drive_root, run.drive_root
    if ours is None:
        return Check("relay's Drive", BAD, "this monitor has no Drive (HUSKY_DRIVE_ROOT or drive_root): nothing to "
                                           "write the calibration to")
    if theirs is None:
        return Check("relay's Drive", BAD, f"the relay reads no Drive, only crl-husky's defaults: start it with "
                                           f"drive_root:={ours}")
    if Path(theirs).resolve() != Path(ours).resolve():
        return Check("relay's Drive", BAD, f"the relay reads {theirs}, this monitor writes to {ours}: start both "
                                           f"with the same HUSKY_DRIVE_ROOT")
    return Check("relay's Drive", GOOD, f"the relay reads this monitor's Drive, {ours}")
