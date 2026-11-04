"""Record: each clear take, planned to its start, then swept with a sample at every waypoint once the tool is still."""

from __future__ import annotations

import time
from contextlib import nullcontext
from typing import TYPE_CHECKING

import numpy as np
import viser

from scaffolding_assembly_core.ids import robot_id

from ....world.checks import BAD, GOOD, PENDING, WARN, Check
from ....world.freeze import take_basis
from ....ui.style import few, indented
from ..analysis.checks import take_count_check
from ..storage.session_files import write_take
from ..robot.motion import sweep_path
from ..robot.sweeps import nearest_take, shortest_order
from ..views.takes_view import take_index, take_label
from .base import CalibrationStep, tool_body_check

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext

#: Clearance kept by arm motions during takes and validation, metres (driving keeps the default STOP_DISTANCE). Above
#: what the arm covers while the check lags a tick or two: 20°/s at 1 m reach is 3.5 cm in 0.1 s.
ARM_CLEARANCE = 0.05


class Record(CalibrationStep):
    """The takes: recorded one at a time, or all on Auto in the shortest order from where the arm stands.

    Takes are current while the tool body is the same and the base body's definition had no data change (a refine, a
    moved pivot). Needs Setup and Marker error (asks first once expired) and Sweeps (refused without a collision check
    where the base stands).
    """

    name = "Record"
    instructions = ("Press Record take, then Go: move and Go: sweep once each preview looks clear. Record every clear "
                    "take. Keep the tool body mounted until Validate is done.")
    goes = auto = cancels = moves_robot = go_moves = True
    watches = ("base", "tool")
    needs = ("Setup", "Marker error", "Sweeps")
    refused_without = ("Sweeps",)
    on_laptop = "Record takes on the laptop: each move waits for Go there, after its preview."

    def __init__(self, cal):
        """No take suggested yet."""
        super().__init__(cal)
        #: The pending take to record next (`_next_take`), and what the Take dropdown was last filled for.
        self._suggested: int | None = None
        self._filled_for: tuple | None = None

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """The take to record, and Record take; the take chips below them."""
        # * Only takes checked clear: a colliding one cannot be chosen.
        self._take_choice = gui.add_dropdown("Take", ["-"])
        record = gui.add_button("Record take", icon=viser.Icon.PLAYER_RECORD)
        # * The readout last: its lines come and go, and must not move the buttons.
        self._html = gui.add_html("")
        record.on_click(ctx.defer("record take", lambda: self._on_record(ctx)))

    def run_changed(self, ctx: PluginContext) -> None:
        """Fill the Take dropdown afresh."""
        self._suggested, self._filled_for = None, None

    def enter(self, ctx: PluginContext) -> None:
        """Suggest the take to record next."""
        super().enter(ctx)
        ctx.spawn("next take", self._suggest(ctx), survives_soft_stop=True)

    def leave(self, ctx: PluginContext) -> None:
        """Hide the takes."""
        self.cal.takes.hide(ctx)

    async def _suggest(self, ctx: PluginContext) -> None:
        self._suggested = await self._next_take(ctx)

    async def _next_take(self, ctx: PluginContext) -> int | None:
        """The pending clear take to record next: the first of the shortest order from where the arm stands."""
        run = self.run
        pending = run.pending()
        if not pending:
            return None
        here = run.robot.arms[run.arm].joint_vector()
        at = here if here is not None else run.stow
        # * A worker thread: with a dozen takes pending the search takes a fifth of a second.
        order = await ctx.run_in_thread(shortest_order, run.sweeps.sweeps, pending, at, run.stow, *run.limits)
        return order[0][0]

    def _on_record(self, ctx: PluginContext) -> None:
        index = take_index(str(self._take_choice.value))
        if index is None:
            self.cal.procedure.outcome = Check("no take", WARN, "check collisions in Sweeps first")
            return
        self.start(ctx, "take", self._record_takes(ctx, index), moves=True)

    async def _record_takes(self, ctx: PluginContext, index: int) -> None:
        """Record a take; on Auto, then every pending clear take, in the shortest order (`_next_take`).

        Before each next take its actions are judged again, and asked about as at the first (`confirm`).
        """
        while index is not None:
            await self._record_take(ctx, index)
            if not self.cal.procedure.auto:
                return
            if (refusal := self.refusal(ctx)) is not None:
                raise RuntimeError(refusal)
            await self.confirm(ctx, moves=True)
            index = await self._next_take(ctx)

    async def _record_take(self, ctx: PluginContext, index: int) -> None:
        """Move to a take's start, sweep it under the lease, sample every waypoint, save it and fit its circle."""
        run, procedure, motion, takes = self.run, self.cal.procedure, self.cal.motion, self.cal.takes
        if index not in run.usable():
            raise RuntimeError(f"take {index + 1} is not checked clear in Sweeps")
        sweep = run.sweeps.sweeps[index]
        here = run.robot.arms[run.arm].joint_vector()
        if here is not None:
            # * The same take, turned by whole turns and maybe run backwards to start nearest the arm.
            nearest = nearest_take(sweep, here, run.stow, *run.limits)
            takes.backwards, sweep = (nearest.start - nearest.end) * (sweep.start - sweep.end) < 0, nearest
        waypoints = sweep.waypoints()
        session = run.session_folder()
        await motion.move_to(ctx, run, waypoints[0], f"the start of take {index + 1}")
        # * The sweep too is looked at before it runs: Preview turns the ghost round the circle.
        unknown = await procedure.optional_go(ctx, sweep_path(waypoints, ctx.config.tick_period),
                                              f"Sweep take {index + 1}: scrub or play Preview to see the arm's way "
                                              f"round, then Go.", "sweep", [robot_id(run.serial)])
        others = [(run.serial, name) for name in run.robot.arms if name != run.arm]
        samples = []
        takes.in_progress = (index, samples)
        try:
            with ctx.lease_robots([run.serial], f"calibration take {index + 1}"), \
                    (ctx.clearance([run.serial], ARM_CLEARANCE, unknown_robots=unknown)
                     if procedure.clearance.value else nullcontext()):
                # * Judged again under the lease: the base may have been driven while Go was waited for, and the
                #   sweeps are checked clear only where it stood then (Sweeps).
                if (refusal := self.refusal(ctx)) is not None:
                    if (moved := run.collision_base_moved(ctx.kinematics.base_pose(run.serial))) is not None:
                        ctx.log_info(f"take {index + 1} refused under the lease; the base is {moved} from where the "
                                     f"collisions were checked")
                    raise RuntimeError(refusal)
                with ctx.freeze(take_basis(ctx.world, ctx.now(), bases=[run.serial], arms=others)):
                    for number, joints in enumerate(waypoints, start=1):
                        procedure.doing = f"take {index + 1}: waypoint {number} of {len(waypoints)}"
                        await motion.step_to(ctx, run, joints)
                        samples.append(await motion.sample(ctx, run, self.cal.tool,
                                                           f"take {index + 1}, waypoint {number}"))
        finally:
            takes.in_progress, takes.backwards = None, False
        name = f"{time.strftime('%H%M%S')}_{run.digits}_{run.arm}_J{sweep.joint}_take{index + 1}"
        recorded = run.keep_take(index, write_take(session / f"j{sweep.joint}", name,
                                                   [sample.record() for sample in samples]))
        if not recorded.good:
            ctx.log_info(f"take {index + 1} left out of Solve: its file went to archive/")
        run.save(start=True)
        self._suggested = await self._next_take(ctx)

    def none_yet(self, ctx: PluginContext) -> str | None:
        """No take recorded yet."""
        return "no take recorded yet" if not self.run.takes else None

    def out_of_date(self, ctx: PluginContext) -> str | None:
        """Too few good takes (or none) once the old ones are left out: its old takes check says which."""
        takes, old = self.checks(ctx)
        return old.detail if takes.level == BAD and old.level != GOOD else None

    def checks(self, ctx: PluginContext) -> list[Check]:
        """Enough good takes per joint, and the old ones to record again; the take chips below show each circle."""
        run = self.run
        counts = [sum(run.sweeps.sweeps[index].joint == joint for index in run.good_takes()) for joint in (0, 1)]
        old = sorted(set(run.takes) - set(run.current_takes()))
        # * Grey only before the first take: once takes are in, too few good ones is red.
        takes = (take_count_check(counts, "recorded well (amber ones are left out)") if run.takes else
                 Check("takes", PENDING, "no take recorded yet", "J0 0 · J1 0"))
        return [takes,
                Check("old takes", WARN, f"takes {few([str(i + 1) for i in old], 5)} were recorded with another "
                                         f"tool body or before a base refine: record them again", str(len(old)))
                if old else Check("old takes", GOOD, "none", "0")]

    def ready(self, ctx: PluginContext) -> list[Check]:
        """A tool body; its preconditions: the checklist, marker errors, a clear take checked here; the safety boxes."""
        return [tool_body_check(self.cal), *self.needs_checks(ctx)]

    def _fill_choices(self) -> None:
        """Fill the Take dropdown with the clear takes, at the suggested one, whenever they change."""
        run = self.run
        usable, pending = run.usable(), run.pending()
        key = (tuple(usable), tuple(pending), self._suggested)
        if key == self._filled_for:
            return
        self._filled_for = key
        labels = [take_label(index, sweep) for index, sweep in enumerate(run.sweeps.sweeps)]
        current = take_index(str(self._take_choice.value))
        choice = (self._suggested if self._suggested in pending else current if current in pending
                  else min(pending, default=None))
        options = [labels[index] for index in usable] or ["no clear take: check collisions in Sweeps"]
        self._take_choice.options = options
        self._take_choice.value = labels[choice] if choice is not None else options[0]

    def preview_joints(self, ctx: PluginContext) -> np.ndarray | None:
        """The chosen take's start; none while a take records: the arm itself is there."""
        index = take_index(str(self._take_choice.value))
        if index is None or self.cal.takes.in_progress is not None:
            return None
        return self.run.sweeps.sweeps[index].waypoints()[0]

    def draw(self, ctx: PluginContext) -> None:
        """The clear takes' chips, their marker errors and how many are recorded; the takes by what was measured."""
        run = self.run
        self._fill_choices()
        usable, current = run.usable(), run.current_takes()
        self._html.content = (self.cal.takes.chips(run, usable, by_collisions=False)
                              + self.cal.takes.marker_chips(run, usable)
                              + indented(f"recorded {sum(index in current for index in usable)}/{len(usable)}"))
        self.cal.takes.draw(ctx, run, by_collisions=False)
