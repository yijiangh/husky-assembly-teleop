"""What the calibration's steps share, and their base class."""

from __future__ import annotations

import time
from dataclasses import replace
from typing import TYPE_CHECKING, Callable, Coroutine

from ....shared.collisions_contract import CollisionsContract
from ....shared.guide.both_pages import on_both
from ....shared.guide.procedure import Procedure
from ....shared.guide.step import Step, in_short, red_checks
from ....shared.teleop.dpad import build_velocity_inputs
from ....ui.phone import phone_text
from ....ui.style import hover
from ....world.checks import BAD, GOOD, PENDING, WARN, Check
from ....world.tcp import tcp_check, tool0_by_kinematics
from ..analysis.checks import CAMERAS_BOX, CHECKLIST_BOXES, SAFETY_BOXES, frame_check, tool_check
from ..robot.driving import Driving
from ..robot.hand_moves import HandMoves
from ..robot.motion import ArmMotion
from ..views.results import ResultsPanel
from ..views.takes_view import TakesView

if TYPE_CHECKING:
    import numpy as np
    import viser

    from ....plugin_api.context import PluginContext
    from ....world.measured import TrackedObject
    from ..run import Run

#: The tool body's name in `world.tracked_objects`.
TOOL_NAME = "calibration_tool"
#: A step's states in the overview (`CalibrationStep.state`): a result's; a calculation still waiting, and Tip offset's
#: fit not made the TCP offset (amber, no work left); a precondition's.
NONE_YET, OUT_OF_DATE, FAILS, ACCEPTED, PASSES = "none yet", "out of date", "fails", "accepted", "passes"
PENDING_STATE, NOT_IN_USE = "pending", "not in use"
EXPIRED, HOLDS = "expired", "holds"
#: The states that leave work at a result step (`CalibrationPlugin.work_left`).
WORK = (NONE_YET, OUT_OF_DATE, FAILS, PENDING_STATE)
#: The ready chip saying whether Area clear and E-stop at hand hold, which every robot move needs.
SAFETY = "area, e-stop"
#: The chip of the arm's reported TCP against tool0 by the kinematics (`Calibration.tcp_check`).
PENDANT_TCP = "pendant TCP"


def tool_body_check(cal: Calibration) -> Check:
    """Whether a tool body is tracked: sampling needs it."""
    if cal.tool is None:
        return Check("tool body", BAD, "no tool body: set it in Setup")
    return Check("tool body", GOOD, f"rigid body {cal.tool.mocap_id}")


def punch_tool_check(run: Run) -> Check:
    """Whether the monitor runs the arm bare, as with the punch on its flange (the punch has no model)."""
    return tool_check(run.serial, {arm.name: arm.end_effector for arm in run.robot.config.arms}, run.arm, wanted=None)


def relay_sha(run: Run) -> str | None:
    """The hash of the calibration the relay applies to the run's robot now; None if it applies none."""
    applied = run.applied()
    return applied.sha256 if applied is not None else None


def relay_up_check(run: Run, label: str = "relay up") -> Check:
    """Whether the relay announced the calibrations it applies, and which one the run's robot gets (source and hash)."""
    relay = run.relay
    if relay is None or relay.state.calibrations_time is None:
        return Check(label, BAD, "the mocap relay announced no calibrations yet: is it running in this domain?")
    applied = run.applied()
    return Check(label, GOOD, f"applies {applied.source} ({applied.sha256[:12]})" if applied is not None else
                 "publishes the base uncalibrated")


def lab_frame_check(run: Run) -> Check:
    """Whether the relay holds the lab frame (`frame_check`): what world poses stored across a step are compared in."""
    relay = run.relay
    return frame_check(relay.state.frame, relay.state.anchors_missing) if relay is not None else frame_check(None)


class Calibration:
    """What the steps share: the run, the tracked tool body, and the services they act through. The plugin owns it.

    Attributes:
        run: The chosen arm's run; replaced when another arm or run is chosen; None without an arm with sweeps.
        tool: The tracked tool body, or None.
        procedure: The one procedure running.
        motion: Moves and samples the arm.
        driving: The D-pad.
        hand: Free drive and the tool pad.
        takes: Draws the takes.
        results: The Live and Results side panel.
        hits: Where the takes' collisions are listed.
        steps: The steps by name, once the plugin built them (`Step.needs`).
        switch_run: Make a run the chosen one (`CalibrationPlugin.switch_run`).
        show_work_left: Make the step with work left the current one (`CalibrationPlugin.show_work_left`); its name.
    """

    def __init__(self, hits: CollisionsContract, switch_run: Callable[[PluginContext, Run], None],
                 show_work_left: Callable[[PluginContext], str]):
        """Nothing chosen yet."""
        self.run: Run | None = None
        self.tool: TrackedObject | None = None
        self.procedure = Procedure()
        self.motion = ArmMotion(self.procedure)
        self.driving = Driving(self.procedure)
        self.hand = HandMoves(self.procedure)
        self.takes = TakesView()
        self.results = ResultsPanel(lambda: self.run, lambda: self.tool.marker_error if self.tool else None)
        self.hits = hits
        self.steps: dict[str, CalibrationStep] = {}
        self.switch_run = switch_run
        self.show_work_left = show_work_left
        #: The last TCP verdict per arm, kept while the arm moves (`tcp_check`).
        self._tcps: dict[tuple[str, str], Check] = {}

    def tcp_check(self, ctx: PluginContext, run: Run) -> Check:
        """Whether the TCP the arm reports is tool0 by the kinematics: the pendant's TCP zero; judged while still."""
        key = (run.serial, run.arm)
        check = tcp_check(run.robot.arms[run.arm], tool0_by_kinematics(ctx.kinematics, run.serial, run.arm), ctx.now())
        if check is not None:
            # * One label at every level, apart from Punch's TCP offset: the pendant's TCP must be zero.
            self._tcps[key] = replace(check, label=PENDANT_TCP)
        return self._tcps.get(key, Check(PENDANT_TCP, PENDING, "judged once the arm stands still"))

    @staticmethod
    def wall_now() -> float:
        """The Unix time preconditions are stamped and judged by (`Run.ticks`, `Run.marker_time`); tests replace it."""
        return time.time()

    def track_tool(self, ctx: PluginContext, mocap_id: int) -> None:
        """Track the tool body with this Motive id (none for 0); the run measures with it from now on.

        The same id keeps the tracked body: a new one has no pose until its first message.
        """
        if mocap_id != (self.tool.mocap_id if self.tool is not None else 0):
            ctx.untrack_object(TOOL_NAME)
            self.tool = ctx.track_object(TOOL_NAME, mocap_id, label="calibration tool") if mocap_id > 0 else None
        if self.run is not None:
            self.run.tool_body = mocap_id


class CalibrationStep(Step):
    """A step of the calibration: it reaches the run and the shared services through `cal`.

    - Its state (`state`) comes from `none_yet`, `out_of_date` and its checks; a precondition step's also from
      `expired`.
    - Its actions warn while a precondition it `needs` has expired, and ask once more at Go (`confirm`); a robot move
      is refused only while Area clear or E-stop at hand were not confirmed within the hour, and Go confirms them.
    - ! `run` is None only before an arm is chosen, and only Setup is shown then.
    """

    #: Whether the step's folder has the D-pad; Free drive and the tool pad (`build_movers` adds them).
    drives = False
    moves_by_hand = False
    #: Whether the step's actions move the robot: its ready checks show whether Area clear and E-stop at hand hold.
    moves_robot = False
    #: Whether Go moves the arm along a planned path: Clearance may refuse it, and Go confirms expired safety boxes.
    go_moves = False
    #: The bodies of "base" and "tool" whose live chip the step shows while one is tracked badly.
    watches: tuple[str, ...] = ()
    #: Whether the step measures with mocap: it needs Setup's Cameras calibrated (`need_expired`).
    uses_mocap = True
    #: The steps in `needs` whose expiry refuses the step's actions instead of asking first.
    refused_without: tuple[str, ...] = ()
    #: The step's readout, below its buttons; on the phone too once built there.
    _html = None

    def __init__(self, cal: Calibration):
        """A step of `cal`."""
        self.cal = cal
        #: The expired preconditions the operator went on past during the running procedure (`confirm`).
        self._gone_past: set[str] = set()

    @property
    def run(self) -> Run:
        """Run: The chosen arm's run."""
        return self.cal.run

    def build_movers(self, ctx: PluginContext, gui: viser.GuiApi, phone: bool = False) -> None:
        """Add the D-pad if the step drives, and Free drive and the tool pad if it moves the arm by hand (`phone`)."""
        if self.drives:
            driving = self.cal.driving
            _speed, buttons = build_velocity_inputs(ctx, gui, f"calibration {self.name}", driving.on_hold,
                                                    driving.clearance)
            driving.buttons += buttons
        if self.moves_by_hand:
            self.cal.hand.build_inputs(ctx, gui, lambda: self.run.tip_in_tool0(), phone)

    def build_phone(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """On the phone: the movers as on the laptop, its robot buttons, what stays on the laptop, the readout."""
        self.build_movers(ctx, gui, phone=True)
        self.build_robot_buttons(ctx, gui)
        super().build_phone(ctx, gui)
        if self._html is not None:
            self._html = on_both(self._html, gui.add_html(""), phone_text)

    def build_robot_buttons(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Add the step's buttons used next to the robot, on both pages (its `build` calls it too); none by default."""

    def enter(self, ctx: PluginContext) -> None:
        """Let the D-pad, Free drive and the tool pad move the robot in a step that has them (`move_refusal`)."""
        run = self.cal.run
        if self.drives and run is not None:
            self.cal.driving.start(ctx, run, lambda: self.move_refusal(ctx))
        else:
            self.cal.driving.stop()
        if self.moves_by_hand and run is not None:
            self.cal.hand.start(ctx, run, lambda: self.move_refusal(ctx))
        else:
            self.cal.hand.stop()

    def run_changed(self, ctx: PluginContext) -> None:
        """Another run was chosen: show its values in the step's widgets."""

    # --- --- --- --- --- STATE --- --- --- --- ---

    def none_yet(self, ctx: PluginContext) -> str | None:
        """Why the step has no result yet, e.g. "no take recorded"; None once it has one."""
        return None

    def out_of_date(self, ctx: PluginContext) -> str | None:
        """Why the step's result is out of date: built from data that changed since; None while it is current."""
        return None

    def judged_under(self, ctx: PluginContext):
        """What the verdict is judged under besides its own inputs, for its acceptance stamp; None for nothing."""
        return None

    def state(self, ctx: PluginContext) -> Check:
        """The step's state: none yet, out of date, fails, accepted, passes; a precondition's expired or holds.

        A failing result can be accepted, and a missing one of a result step (skipped); one out of date stays so.
        """
        if self.cal.run is None:
            return Check(NONE_YET, PENDING, "choose an arm in Setup")
        if (why := self.none_yet(ctx)) is not None:
            if not self.precondition and self.run.accepted(self.name, self.judged_under(ctx)):
                return Check(ACCEPTED, WARN, f"skipped: {why}", "skipped")
            return Check(NONE_YET, PENDING, why)
        if (why := self.out_of_date(ctx)) is not None:
            return Check(OUT_OF_DATE, BAD, why)
        red = red_checks(self.checks(ctx))
        # * The first red check, as `Step.state` says it: the step's chips have the rest.
        more = f" (+{len(red) - 1} more: see the step's chips)" if len(red) > 1 else ""
        why = hover(f"{red[0].label}: {red[0].detail}{more}", "the step's chips") if red else ""
        accepted = bool(red) and self.run.accepted(self.name, self.judged_under(ctx))
        if red and not accepted:
            return Check(FAILS, BAD, why, in_short(red))
        if self.precondition and (expired := self.expired(ctx)) is not None:
            return Check(EXPIRED, WARN, expired)
        if accepted:
            return Check(ACCEPTED, WARN, why, in_short(red))
        return Check(HOLDS, GOOD) if self.precondition else Check(PASSES, GOOD)

    def done(self, ctx: PluginContext) -> None:
        """Done leaves the step, after any acceptance: keep what it made; nothing by default."""

    # --- --- --- --- --- ACTING --- --- --- --- ---

    def expired(self, ctx: PluginContext) -> str | None:
        """Why this step's precondition has expired for the steps that `need` it; None while it holds or it has none.

        ! A step with a precondition overrides it, and shows it expired amber in its own checks, never red.
        """
        return None

    def need_expired(self, ctx: PluginContext, needed: Step) -> str | None:
        """Why `needed` has expired for this step; of Setup's checklist only the boxes this step needs.

        - The safety boxes have their own chip where the step moves the robot, and Go asks about them there.
        - Cameras calibrated only where the step measures with mocap (`uses_mocap`): Tip offset uses the joints alone.
        """
        if needed.name != "Setup":
            return needed.expired(ctx)
        boxes = [box for box in CHECKLIST_BOXES if not (self.moves_robot and box in SAFETY_BOXES)
                 and not (box == CAMERAS_BOX and not self.uses_mocap)]
        return self.run.checklist_expired(self.cal.wall_now(), boxes)

    def needs_checks(self, ctx: PluginContext) -> list[Check]:
        """A ready chip per step in `needs`, amber once expired (red if `refused_without`); then the safety boxes'."""
        checks = []
        for name in self.needs:
            why = self.need_expired(ctx, self.cal.steps[name])
            if why is None:
                checks.append(Check(name, GOOD, "holds"))
            elif name in self.refused_without:
                checks.append(Check(name, BAD, hover(f"expired: {why}: back to {name}", f"the {name} step")))
            else:
                checks.append(Check(name, WARN, hover(f"expired: {why}: back to {name}, or Go on past it when asked",
                                                      f"the {name} step")))
        if self.moves_robot:
            why = self.run.safety_expired(self.cal.wall_now())
            checks.append(Check(SAFETY, BAD, hover(f"{why}: {self._unconfirmed()}", "Setup's checklist")) if why else
                          Check(SAFETY, GOOD, "Area clear and E-stop at hand confirmed within the hour"))
        return checks

    def _unconfirmed(self) -> str:
        """What the step's moves do while Area clear or E-stop at hand were not confirmed within the hour."""
        hand = (["the D-pad"] if self.drives else []) + (["Free drive", "the tool pad"] if self.moves_by_hand else [])
        parts = (["Go asks you to confirm them"] if self.go_moves else []) + (
            [f"{' and '.join([', '.join(hand[:-1]), hand[-1]] if len(hand) > 1 else hand)} "
             f"refuse{'s' if len(hand) == 1 else ''} until they are ticked in Setup"] if hand else [])
        return "; ".join(parts) or "nothing moves until they are ticked in Setup"

    def refusal(self, ctx: PluginContext) -> str | None:
        """Why the step's actions cannot run now: its red ready checks but the safety boxes', which Go confirms."""
        return "; ".join(f"{check.label}: {check.detail}" for check in red_checks(self.ready(ctx))
                         if check.label != SAFETY) or None

    def move_refusal(self, ctx: PluginContext) -> str | None:
        """Why the D-pad, Free drive and the tool pad may not move the robot now: Area clear or E-stop at hand."""
        why = self.run.safety_expired(self.cal.wall_now())
        return f"{why}: tick them again in Setup" if why else None

    def busy(self, refused: str) -> bool:
        """Whether a procedure runs, so an action is refused: shown as `refused` (e.g. "arm not changed") and why."""
        procedure = self.cal.procedure
        if procedure.running:
            procedure.outcome = Check(refused, WARN, f"{procedure.doing or 'a procedure'} runs: Cancel it first")
        return procedure.running

    def preview_joints(self, ctx: PluginContext) -> np.ndarray | None:
        """The arm joints the ghost shows while no procedure previews a path; None for no ghost."""
        return None

    def start(self, ctx: PluginContext, label: str, work: Coroutine, moves: bool = False) -> None:
        """Run `work` as the procedure once `confirm`ed, unless one works or the step's actions are refused: say why.

        One that only waits for Go is replaced (`Procedure.start`): e.g. Record take with another take chosen.

        Args:
            ctx: The plugin's context.
            label: What it is, e.g. "take".
            work: The procedure's coroutine.
            moves: Whether it moves the robot: Go must confirm Area clear and E-stop at hand first if they expired.
        """
        procedure, name = self.cal.procedure, f"{label} {self.run.serial} {self.run.arm}" if self.run else label
        if not procedure.waiting and self.busy(f"{label} not started"):
            work.close()
            return
        refusal = self.refusal(ctx) if self.cal.run is not None else "choose an arm in Setup"
        if refusal is None:
            procedure.start(ctx, label, self._confirmed(ctx, work, moves), name)
            return
        work.close()
        ctx.log_info(f"{label} not started: {refusal}")
        procedure.outcome = Check(f"{label} not started", BAD, hover(refusal))

    async def _confirmed(self, ctx: PluginContext, work: Coroutine, moves: bool) -> None:
        """Run `work` once `confirm`ed."""
        self._gone_past = set()
        try:
            await self.confirm(ctx, moves)
        except BaseException:
            work.close()
            raise
        await work

    async def confirm(self, ctx: PluginContext, moves: bool) -> None:
        """Before acting, ask at Go: to confirm Area clear and E-stop at hand, and to go on past expired preconditions.

        - The safety boxes only if they expired and the action `moves` the robot; Go ticks them again.
        - Each expired precondition in `needs` is asked about once per procedure.
        - ! Auto skips neither question.
        """
        run, procedure = self.run, self.cal.procedure
        if moves and (why := run.safety_expired(self.cal.wall_now())) is not None:
            await procedure.ask_go(ctx, f"{why}. Is the area clear, and does someone hold the e-stop? Go confirms "
                                        f"both.", "confirm")
            for label in SAFETY_BOXES:
                run.tick(label, True, self.cal.wall_now())
            run.save(start=True)
        expired = {name: why for name in self.needs if name not in self.refused_without + tuple(self._gone_past)
                   and (why := self.need_expired(ctx, self.cal.steps[name])) is not None}
        if expired:
            await procedure.ask_go(ctx, f"Expired: {'; '.join(f'{name}: {why}' for name, why in expired.items())}. "
                                        f"Go acts anyway, or Cancel and go back to {next(iter(expired))}.",
                                   "act anyway")
            self._gone_past.update(expired)
