"""Validate: the tool body in tool0 under the new calibration, at the validation poses from several base spots."""

from __future__ import annotations

from html import escape
from typing import TYPE_CHECKING

import numpy as np
import viser

from scaffolding_assembly_core.geometry import compose

from ....shared.arm_planner_contract import ArmPlannerContract
from ....ui.style import FAIL, LEVEL_COLORS, NONE, chip
from ....world.checks import BAD, GOOD, PENDING, WARN, Check
from ..analysis.checks import base_moved, new_spot, sample_marker_error, validation_level
from ..analysis.old_outputs import write_validation_outputs
from ..robot.motion import NoPlan
from ..robot.sweeps import nearest_pose
from ..views.takes_view import chip_row, marker_chip
from ..views.validation_view import LEVEL_RGB, PENDING_RGB, ValidationView
from .base import PASSES, PENDING_STATE, CalibrationStep, tool_body_check

if TYPE_CHECKING:
    from ....plugin_api.context import PluginContext


class Validate(CalibrationStep):
    """The validation verdict: the samples, a file each and grouped by base spot, judged under the current solution.

    Samples are current while the tool body is the same and neither body's definition had a data change, whatever the
    solution. Needs Setup and Marker error (asks first once expired).
    """

    name = "Validate"
    instructions = ("Drive to a spot with the D-pad, then Sample poses here: the arm visits each validation pose. Do "
                    "it at 3 spots or more, turned differently. Under a good calibration the tool body stays put in "
                    "tool0.")
    drives = goes = auto = cancels = moves_robot = True
    needs = ("Setup", "Marker error")
    on_laptop = "Sample poses here on the laptop: each move waits for Go there, after its preview."

    def __init__(self, cal):
        """No spot sampled yet."""
        super().__init__(cal)
        #: The current spot: each pose's state ("pending", "collides", "skipped", "sampled") and why, and the index of
        #: each sampled pose's sample among all samples.
        self._pose_states: list[tuple[str, str]] = []
        self._pose_samples: dict[int, int] = {}
        self.scene = ValidationView()

    def build(self, ctx: PluginContext, gui: viser.GuiApi) -> None:
        """The D-pad, Sample poses here and Clear samples; the readout below them."""
        self.build_movers(ctx, gui)
        button = gui.add_button("Sample poses here", icon=viser.Icon.PLAYER_PLAY,
                                hint="Visit every validation pose where the base stands, and sample each.")
        clear = gui.add_button("Clear all spots", icon=viser.Icon.TRASH,
                               hint="Start the validation again: every spot's file goes to validation/archive/.")
        # * The readout last: its lines come and go, and must not move the buttons.
        self._html = gui.add_html("")
        button.on_click(ctx.defer("validate", lambda: self.start(ctx, "validation", self._validate(ctx), moves=True)))
        clear.on_click(ctx.defer("clear validation", lambda: self._clear(ctx)))

    def run_changed(self, ctx: PluginContext) -> None:
        """Forget the spot shown."""
        self._pose_states, self._pose_samples = [], {}

    def leave(self, ctx: PluginContext) -> None:
        """Hide the validation dots, targets and spots."""
        self.scene.hide(ctx)

    async def _validate(self, ctx: PluginContext) -> None:
        """Visit the validation poses clear where the base stands, nearest first, and sample each; a file per sample.

        - Each sample is written as it is taken: whatever ends the validation, nothing sampled is lost.
        - The poses are checked for collisions where the base stands, and again once it moved (`base_moved`).
        - Cancel while the arm moves or samples holds the arm: Go goes on, a second Cancel ends. The D-pad drives
          meanwhile; at another spot (`new_spot`) every pose is sampled afresh.
        """
        run, procedure = self.run, self.cal.procedure
        arm, count = run.robot.arms[run.arm], len(run.validation_poses)
        spot = checked = None
        before = len(run.validation_files)
        while True:
            here = ctx.kinematics.base_pose(run.serial)
            if spot is None or new_spot(spot, here):
                spot, checked = here, None
                self._pose_states, self._pose_samples = [("pending", "")] * count, {}
            if checked is None or base_moved(checked, here):
                await self._check_poses(ctx)
                checked = here
            pending = [index for index, (state, _) in enumerate(self._pose_states) if state == "pending"]
            if not pending:
                break
            joints = arm.joint_vector()
            if joints is None:
                raise RuntimeError("no joint state of the arm")
            index, goal = nearest_pose(run.validation_poses, pending, joints, *run.limits, run.stow)
            if not await procedure.pausable(self._visit(ctx, index, goal)):
                left = len([state for state, _ in self._pose_states if state == "pending"])
                await procedure.ask_go(ctx, f"Stopped with {left} of {count} poses left here. Go goes on (drive "
                                            f"first for another spot); Cancel ends and keeps the samples.",
                                       "go on")
        if len(run.validation_files) == before:
            raise RuntimeError("no validation pose reachable here: drive elsewhere")

        # * The validation outputs read the files in place: current spots archived before come back first.
        run.tidy()
        run.save(start=True)
        run.write_analysis()
        solved = run.current_solved()
        if run.validation().spread is not None and not run.mount_check:
            ctx.spawn("calibration validation outputs", self.cal.results.write_and_show(
                ctx, run, write_validation_outputs, run.session, run.digits, run.arm, run.urdf,
                solved.solution.footprint, run.current_validation_files()), survives_soft_stop=True)

    async def _check_poses(self, ctx: PluginContext) -> None:
        """Check the poses not sampled here for collisions where the base stands: pending if clear, else collides."""
        run, procedure = self.run, self.cal.procedure
        planner: ArmPlannerContract = ctx.require("arm_planner")
        count = len(run.validation_poses)
        for index, pose in enumerate(run.validation_poses):
            if self._pose_states[index][0] == "sampled":
                continue
            procedure.doing = f"checking pose {index + 1} of {count} for collisions"
            try:
                hit = await planner.check_path(run.serial, run.arm, [pose])
                why = "" if hit is None else hit.message
            except (ValueError, RuntimeError) as error:
                why = str(error)
            self._pose_states[index] = ("collides", why) if why else ("pending", "")

    async def _visit(self, ctx: PluginContext, index: int, goal: np.ndarray) -> None:
        """Move to validation pose `index` (as `goal`) and sample it; skipped if the planner finds no move."""
        run, procedure, motion = self.run, self.cal.procedure, self.cal.motion
        what = f"validation pose {index + 1} of {len(run.validation_poses)}"
        try:
            await motion.move_to(ctx, run, goal, what)
        except NoPlan as error:
            self._pose_states[index] = ("skipped", str(error))
            return
        with ctx.lease_robots([run.serial], f"calibration {what}"):
            procedure.doing = f"sampling {what}"
            sample = await motion.sample(ctx, run, self.cal.tool, what)
        run.add_validation_sample(sample, index + 1)
        run.save()
        self._pose_samples[index] = len(run.validation().bases) - 1
        self._pose_states[index] = ("sampled", "")

    def _clear(self, ctx: PluginContext) -> None:
        """Start the validation again; the files go to archive/."""
        if self.busy("spots not cleared"):
            return
        self.run.clear_validation()
        self._pose_states, self._pose_samples = [], {}
        self.run.save()

    def none_yet(self, ctx: PluginContext) -> str | None:
        """No spot sampled yet."""
        return "no spot sampled yet" if not self.run.validation_files else None

    def out_of_date(self, ctx: PluginContext) -> str | None:
        """Every spot sampled with another tool body or before a refine."""
        run = self.run
        return ("sampled with another tool body or before a refine" if not run.current_validation_files()
                else None)

    def state(self, ctx: PluginContext) -> Check:
        """As any result's; pending while no red check but the verdict waits for a current solution."""
        state = super().state(ctx)
        if state.label == PASSES and self.run.current_solved() is None:
            return Check(PENDING_STATE, PENDING, "judged under the current solution: Solve first")
        return state

    def checks(self, ctx: PluginContext) -> list[Check]:
        """Enough samples at enough spots, the tool body's spread in tool0 under the solution; the out of date ones."""
        run = self.run
        old = len(run.validation_files) - len(run.current_validation_files())
        return run.validation_verdict() + [
            Check("out of date", WARN, f"{old} samples were taken with another tool body or before a refine: they no "
                                       f"longer count, sample again", str(old))
            if old else Check("out of date", GOOD, "none", "0")]

    def ready(self, ctx: PluginContext) -> list[Check]:
        """Validation poses, a tool body; the checklist and marker errors, the safety boxes; no solution needed."""
        run = self.run
        return [Check("poses", GOOD, f"{len(run.validation_poses)} validation poses") if run.validation_poses else
                Check("poses", BAD, f"data/calibration/sweeps.yaml lists no validation poses for {run.arm}"),
                tool_body_check(self.cal), *self.needs_checks(ctx)]

    def draw(self, ctx: PluginContext) -> None:
        """A chip per spot, per pose at this spot and its marker error; in the scene the samples, targets and spots."""
        run = self.run
        validation = run.validation()
        looks = self._pose_looks(validation)
        poses = "".join(chip(str(index + 1), color, detail) for index, (color, _rgb, detail) in enumerate(looks))
        markers = "".join(marker_chip(str(index + 1), [validation.marker_errors[self._pose_samples[index]]])
                          if index in self._pose_samples and self._pose_samples[index] < len(validation.marker_errors)
                          else chip(f"{index + 1} · -", NONE, "not sampled here yet")
                          for index in range(len(run.validation_poses)))
        self._html.content = (chip_row("spots", self._spot_chips(validation)) + chip_row("poses here", poses)
                              + chip_row("markers", markers))
        self.scene.draw_dots(ctx, validation)
        self.scene.draw_targets(ctx, run, [rgb for _color, rgb, _detail in looks])
        footprint = self.run.footprint()
        self.scene.draw_spots(ctx, [compose(spot, footprint) for spot in validation.spots])

    def _spot_chips(self, validation) -> str:
        """A chip per spot, "spot 2 · 6" (its samples), by its worst sample's distance from the mean; the next last."""
        distances = validation.spread[0] if validation.spread is not None else None
        chips = ""
        for number, members in enumerate(validation.spot_samples, start=1):
            size = len(members)
            if distances is None:
                color, detail = NONE, f"{size} samples; judged once there is a current solution"
            else:
                spread = distances[members]
                color = LEVEL_COLORS[max(validation_level(d) for d in spread)]
                detail = f"{size} samples, all within {spread.max() * 1e3:.1f} mm of the mean"
            worst = [sample_marker_error(validation.marker_errors[index]) for index in members]
            known = [error for error in worst if error is not None]
            detail += f"; marker errors up to {max(known) * 1e3:.2f} mm" if known else "; marker errors unknown"
            chips += chip(escape(f"spot {number} · {size}"), color, detail)
        if not self.cal.procedure.running:
            # * The spot to come: always one, so the row never starts empty.
            chips += chip(escape(f"spot {len(validation.spot_sizes) + 1} · 0"), NONE,
                          "next: drive somewhere new (0.5 m or 30° from the others), then Sample poses here")
        return chips

    def _pose_looks(self, validation) -> list[tuple[str, tuple[int, int, int], str]]:
        """Each validation pose's chip colour, scene colour and hover text at this spot; grey until visited."""
        distances = validation.spread[0] if validation.spread is not None else None
        states = self._pose_states or [("pending", "to sample")] * len(self.run.validation_poses)
        looks = []
        for index, (state, why) in enumerate(states):
            sample = self._pose_samples.get(index)
            if state == "sampled" and distances is not None and sample is not None and sample < len(distances):
                level = validation_level(distances[sample])
                looks.append((LEVEL_COLORS[level], LEVEL_RGB[level],
                              f"{distances[sample] * 1e3:.2f} mm from the mean"))
            elif state in ("collides", "skipped"):
                looks.append((FAIL, LEVEL_RGB[BAD], why or state))
            else:
                looks.append((NONE, PENDING_RGB, why or state))
        return looks
