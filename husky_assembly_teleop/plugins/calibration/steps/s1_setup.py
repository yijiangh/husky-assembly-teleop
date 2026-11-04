"""Setup: the robot arm, a new or resumed run, the tool body, and the operator's checklist."""

from __future__ import annotations

from typing import TYPE_CHECKING

import viser

from ....shared.guide.both_pages import on_both
from ....shared.guide.checklist import Checklist
from ....ui.phone import phone_text
from ....ui.style import SECTION_CTRL, hover, indented, note, section
from ....world.calibration_sessions import sessions_folder
from ....world.checks import BAD, GOOD, PENDING, WARN, Check
from ....world.mocap import mocap_check
from ..analysis.checks import CHECKLIST_BOXES, PRECONDITION_HOLDS, MARKER_KIND, TOOL_BOX, tool_check
from ..storage.progress import Progress, runs
from ..run import Run
from .base import EXPIRED, HOLDS, NONE_YET, CalibrationStep, lab_frame_check, relay_up_check

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext
    from .base import Calibration

#: The Run dropdown's first entry.
NEW_RUN = "New run"
#: Why the tool body's mocap is grey for a finished run.
PUNCH_ONLY = "not needed: the run is finished; Tip offset and Punch use the punch, not the tool body"
#: Each checklist box's hint, by label.
HINTS = {
    TOOL_BOX: "The tool on this arm's flange is the one the tool model chip names: the tool body for the takes; the "
              "punch, on an arm the monitor runs bare, for Tip offset and Punch. Holds only for that tool model.",
    "Area clear": "Nothing and nobody within the arm's reach, all the way round.",
    "E-stop at hand": "Someone holds the robot's e-stop during every motion.",
    "Cameras calibrated": "Motive's camera calibration is Exceptional and under a week old; no camera moved since."}


class Setup(CalibrationStep):
    """Choose the arm and its run; the relay, mocap, arm and Drive ready; the operator's checklist ticked.

    - Its precondition, the checklist, holds while each box was ticked within PRECONDITION_HOLDS, Tool on flange for
      this tool model (`Run.checklist_expired`); needs nothing. A tick starts the run file, which keeps the tick times.
    - Its state is the checklist's alone: the other checks are live conditions, chips only.
    - A finished run is resumed for Tip offset and Punch, with the punch on the flange: the tool body's mocap is grey
      and the tool model must be the bare arm.
    """

    name = "Setup"
    precondition = True
    instructions = "Choose the arm. Mount the tool body on its flange, clear the area, and tick the checklist."

    def __init__(self, cal: Calibration, choices: list[str]):
        """Offer `choices`, "<serial> <arm>" of every arm with sweeps."""
        super().__init__(cal)
        self.choices = choices
        #: The Run dropdown's runs by label.
        self._runs: dict[str, Progress] = {}
        self._checklist = Checklist([(label, HINTS[label]) for label in CHECKLIST_BOXES])
        #: What the Run list was last filled for (`_listing`).
        self._listed = None
        #: The Drive problem last logged, so each is logged once.
        self._drive_logged = ""

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """Arm and Run dropdowns and the tool body; the checklist; then the record: On stand and the notes."""
        gui.add_html(note("The arm whose first two joints sweep; its tool body on the flange. One arm per "
                          "session."))
        self._arm_choice = gui.add_dropdown("Arm", self.choices or ["no robot with sweeps"], disabled=not self.choices)
        self._run_choice = gui.add_dropdown("Run", [NEW_RUN], hint="Resume a run of this arm (newest first), or start "
                                                                   "a new one: its file starts with the first thing "
                                                                   "recorded.")
        refresh = gui.add_button("Refresh runs", icon=viser.Icon.REFRESH,
                                 hint="Look again in the sessions folder (see the Drive check), e.g. for runs "
                                      "another monitor wrote.")
        self._run_html = gui.add_html("")
        self._tool_id = gui.add_number("Tool body", initial_value=0, step=1,
                                       hint="Motive rigid body on the flange (the calibration marker's by default).")
        self._checklist.build(ctx, gui, "before you start", self._on_tick)
        gui.add_html(section("for the record", SECTION_CTRL, "kept in the run file for people; nothing uses them"))
        self._on_stand = gui.add_checkbox("On stand", initial_value=False,
                                          hint="Ticked: the base stands on its stand, wheels off the floor; unticked: "
                                               "on its wheels.")
        self._notes = gui.add_text("Notes", "", multiline=True, hint="Anything worth knowing about this run later.")
        self._arm_choice.on_update(ctx.defer_value("choose arm", lambda value: self.choose(ctx, value),
                                                   operator_only=True))
        self._run_choice.on_update(ctx.defer_value("choose run", lambda label: self._on_run_choice(ctx, label),
                                                   operator_only=True))
        refresh.on_click(ctx.defer("refresh runs", lambda: self.refresh_runs()))
        self._tool_id.on_update(ctx.defer_value("tool body", lambda value: self._on_tool_body(ctx, int(value)),
                                                operator_only=True))
        self._on_stand.on_update(ctx.defer_value("on stand", lambda value: self._note(on_stand=bool(value)),
                                                 operator_only=True))
        self._notes.on_update(ctx.defer_value("run notes", lambda value: self._note(notes=str(value)),
                                              operator_only=True))

    def build_phone(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """On the phone: the run line and the checklist, ticked on either page; arm, run and tool body on the laptop."""
        gui.add_html(note("Arm, run and tool body: on the laptop."))
        self._run_html = on_both(self._run_html, gui.add_html(""), phone_text)
        self._checklist.build(ctx, gui, "before you start", self._on_tick)

    def run_changed(self, ctx: PluginContext) -> None:
        """Show the run's arm, tool body, On stand, notes and the boxes that hold, and list the arm's runs."""
        run = self.run
        self._arm_choice.value = f"{run.serial} {run.arm}"
        self._tool_id.value = run.tool_body
        self._on_stand.value, self._notes.value = run.on_stand, run.notes
        self._show_ticks()
        self.refresh_runs()

    # --- --- --- --- --- CHECKLIST --- --- --- --- ---

    def _on_tick(self, label: str, ticked: bool) -> None:
        """Keep when the operator ticked the box (or forget it), and save: a tick starts the run file."""
        self.run.tick(label, ticked, self.cal.wall_now())
        try:
            self.run.save(start=True)
        except OSError as error:
            self.cal.procedure.outcome = Check("run not started", BAD, str(error))

    def _show_ticks(self) -> None:
        """Tick the boxes that hold, untick the others."""
        now = self.cal.wall_now()
        holds = {label: self.run.box_expired(label, now) is None for label in CHECKLIST_BOXES}
        for ticked in (True, False):
            self._checklist.tick([label for label, held in holds.items() if held == ticked], ticked)

    def expired(self, ctx: PluginContext) -> str | None:
        """Why the checklist has expired for the later steps: each box that does not hold, and why; None if it holds."""
        return self.run.checklist_expired(self.cal.wall_now())

    def state(self, ctx: PluginContext) -> Check:
        """The checklist's state: none yet until every box was ticked once, then expired or holds."""
        never = [] if self.cal.run is None else [label for label in CHECKLIST_BOXES if label not in self.run.ticks]
        if self.cal.run is None or never:
            return Check(NONE_YET, PENDING, f"tick {', '.join(never) or 'the checklist'}")
        why = self.expired(ctx)
        return Check(EXPIRED, WARN, why) if why else Check(HOLDS, GOOD, "every box ticked within the hour")

    def _checklist_check(self) -> Check:
        """Green while it holds; amber once every box was ticked but some no longer hold; red while one never was."""
        run = self.run
        why = run.checklist_expired(self.cal.wall_now())
        if why is None:
            return Check("checklist", GOOD, f"all ticked, within the last {PRECONDITION_HOLDS / 60:.0f} min")
        never = [label for label in CHECKLIST_BOXES if label not in run.ticks]
        if never:
            return Check("checklist", BAD, "; ".join(never))
        return Check("checklist", WARN, f"{why}: tick again once true")

    # --- --- --- --- --- CHOICES --- --- --- --- ---

    def choose(self, ctx: PluginContext, value: str) -> None:
        """Pick the robot and arm: a new run of it."""
        if self.busy("arm not changed"):
            self._arm_choice.value = f"{self.run.serial} {self.run.arm}"
            return
        serial, arm = value.split(" ", 1)
        self.cal.switch_run(ctx, Run.new(ctx.world.robots[serial], arm, ctx.config.data_directory,
                                         ctx.config.drive_root, ctx.world.mocap_relay))

    def refresh_runs(self) -> None:
        """List the chosen arm's runs in the Run dropdown, newest first, the current one selected."""
        run = self.cal.run
        if run is not None:
            run.refresh_sessions()
        try:
            found = runs(run.sessions(), run.serial, run.arm) if run is not None else []
        except FileNotFoundError:
            found = []
        self._listed = self._listing()
        self._runs = {self._run_label(progress): progress for progress in found}
        current = next((label for label, progress in self._runs.items()
                        if run.progress is not None and progress.file == run.progress.file), NEW_RUN)
        self._run_choice.options = [NEW_RUN, *self._runs]
        self._run_choice.value = current

    def _listing(self) -> tuple | None:
        """What the Run list shows: the run file, and the sessions folder by what the relay reads (`Run.sessions`)."""
        run = self.cal.run
        return None if run is None else (run.progress.file if run.progress else None, run.simulated(), run.mixed())

    def _run_label(self, progress: Progress) -> str:
        stamp = progress.file.stem[-6:]
        state = "finished" if progress.applied else "in progress"
        return f"{progress.session.name} {stamp[:2]}:{stamp[2:4]} · {len(progress.takes)} takes · {state}"

    def _on_run_choice(self, ctx: PluginContext, label: str) -> None:
        """Resume the chosen run and go to its work left, or start afresh: a new run, its file not yet started."""
        procedure, run = self.cal.procedure, self.run
        if self.busy("run not changed"):
            self.refresh_runs()
            return
        if label == NEW_RUN:
            fresh = Run.new(run.robot, run.arm, ctx.config.data_directory, ctx.config.drive_root, run.relay)
            # * The same robot as it stands: the tool body and On stand carry over, the notes do not.
            fresh.tool_body, fresh.on_stand = run.tool_body, run.on_stand
            self.cal.switch_run(ctx, fresh)
            procedure.outcome = Check("new run", GOOD, "its file starts with the first thing recorded")
            return
        progress = self._runs.get(label)
        if progress is None:
            return
        try:
            # * The file as it is now: the list's copy dates from its last refresh, and resuming that would lose newer
            #   results at the next save.
            resumed = Run.resume(run.robot, Progress.load(progress.file), ctx.config.data_directory,
                                 ctx.config.drive_root, run.relay)
        except (OSError, ValueError) as error:
            procedure.outcome = Check("resume failed", BAD, str(error))
            ctx.log_warn(f"resume of {progress.file} failed: {error}")
            self.refresh_runs()
            return
        self.cal.switch_run(ctx, resumed)
        at = self.cal.show_work_left(ctx)
        procedure.outcome = Check("resumed", GOOD, f"{progress.file.name}: at {at}, the first step with work left")

    def _note(self, **fields) -> None:
        """Keep On stand or the notes in the run, and save."""
        for name, value in fields.items():
            setattr(self.run, name, value)
        self.run.save()

    def _on_tool_body(self, ctx: PluginContext, mocap_id: int) -> None:
        """Track another tool body: takes measured with the one before are out of date until it is back."""
        if self.busy("tool body not changed"):
            self._tool_id.value = self.run.tool_body
            return
        self.cal.track_tool(ctx, mocap_id)
        self.run.save()

    # --- --- --- --- --- CHECKS AND DRAWING --- --- --- --- ---

    def checks(self, ctx: PluginContext) -> list[Check]:
        """Live: sweeps, relay, lab frame, both bodies' mocap, arm, tool model, Drive, soft stop; the checklist."""
        run = self.cal.run
        if run is None:
            return [Check("arm", BAD, "no robot with sweeps for one of its arms")]
        robot, now, tool = run.robot, ctx.now(), self.cal.tool
        checks = [Check("sweeps", BAD, "; ".join(run.problems)) if run.problems else
                  Check("sweeps", GOOD, f"{len(run.sweeps.sweeps)} takes for this arm"), relay_up_check(run),
                  lab_frame_check(run)]
        checks.append(mocap_check("base mocap", robot.base.mocap_id, robot.base.state, now))
        if run.finished:
            checks.append(Check("tool mocap", PENDING, PUNCH_ONLY))
        else:
            checks.append(mocap_check("tool mocap", tool.mocap_id, tool, now) if tool
                          else Check("tool mocap", BAD, "no tool body"))
        joints = robot.arms[run.arm].joint_vector()
        checks.append(Check("arm", GOOD if joints is not None else BAD,
                            "joints measured" if joints is not None else "no joint state"))
        checks.append(tool_check(robot.config.serial, {a.name: a.end_effector for a in robot.config.arms}, run.arm,
                                 wanted=None if run.finished else MARKER_KIND))
        try:
            run.sessions()
            self._drive_logged = ""
            # * Below the Drive root: the root's local path is long (Insync), and the same for every session.
            checks.append(Check("Drive", GOOD, f"sessions in the Drive's {sessions_folder(run.simulated()).as_posix()}"
                                + (" (the relay reads a simulation)" if run.simulated() else "")))
        except FileNotFoundError as error:
            if str(error) != self._drive_logged:
                self._drive_logged = str(error)
                ctx.log_warn(f"calibration Drive: {error}")
            checks.append(Check("Drive", BAD, hover(str(error))))
        checks.append(Check("soft stop", BAD, "the robot is soft stopped: Resume it") if robot.base.soft_stopped
                      else Check("soft stop", GOOD, "not soft stopped"))
        checks.append(self._checklist_check())
        return checks

    def draw(self, ctx: PluginContext) -> None:
        """The run file's name, the boxes that hold ticked; the Run list again once what it shows changed."""
        run = self.cal.run
        if run is not None:
            self._show_ticks()
        progress = run.progress if run is not None else None
        if self._listed != self._listing():
            self.refresh_runs()
        # * Always one line: it must not move the widgets below.
        self._run_html.content = indented(f"run file {progress.file.name}" if progress is not None else
                                          "run file: starts with the first thing recorded")
