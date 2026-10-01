"""
The cell plugin: loads an authored design and steps through its cell states.

! EXPERIMENTAL: a first cut. How designs load, and what planning plugins
  read from it, may still change.

Viewer and provider only: it draws one movement's robot cell state at a time,
and planning plugins that declare `requires = ("cell",)` read the selected
movement from it. It owns no planner and no PyBullet body.

* Reads schema 1 design folders (doc/design_format.md) through `design_io`. For an export in
  the old compas_fab format the first Load asks: Convert writes it into `<export>_design` next to
  it (or switches to that copy if it is up to date) and loads it; Load again reads the old
  export with compas and converts it in memory, writing nothing.
* Drawing and stepping use only the `design_io.Design` (drawing.py): no compas_fab cells or
  states. Planners build those in their own mirrors.
* Planned states use the design's tools; the live robots keep the monitor's. When the two
  disagree for a configured robot, the panel warns (tasks/2026-10-01_design_io_library.md §10).
! The authored state is never modified: what is drawn is posed from it (`displayed_joints`).
! Reading the robots' meshes takes a moment, so loading runs on a worker thread, and the
  viser nodes are added a robot per tick by a task. The worker only returns a result; the
  task hands it over on the main thread.
"""

from __future__ import annotations

import asyncio
import traceback
from html import escape
from pathlib import Path

from typing import TYPE_CHECKING

from ...design_conversion import convert_export, converted_folder, is_old_export, is_up_to_date
from ...plugin_api.concurrency import timeout
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, block, chip, note, section, values
from ...design_io.timing import Stopwatch
from .design import CellDesign, Step, displayed_joints, load_design, load_old_export
from .drawing import DesignDrawing, load_robot_models

if TYPE_CHECKING:
    from ...config import RobotConfig

#: Give up on a load after this many seconds.
LOAD_TIMEOUT = 120.0

#: Give up on converting an old export after this many seconds (it takes about 15).
CONVERT_TIMEOUT = 300.0


def _load_in_background(folder: Path, report, old_export_data: Path | None = None
                        ) -> tuple[CellDesign, dict, Stopwatch]:
    """Load a design and read its robots' URDFs with their meshes. Runs on the worker thread.

    Args:
        folder: Design folder, or an old export.
        report: Called with a progress string.
        old_export_data: For an old export, the monitor's data directory (robot files); None for a design.

    Returns:
        tuple: The design, each robot's yourdfpy model by robot id, and the time each step took.
    """
    watch = Stopwatch()
    design = (load_design(folder, report, watch) if old_export_data is None
              else load_old_export(folder, old_export_data, report, watch))
    report("reading the robots' URDFs and meshes")
    models = load_robot_models(design.design)
    watch.lap("robot URDFs and meshes")
    return design, models, watch


@register
class CellPlugin(HuskyPlugin):
    """Shows one authored robot cell state at a time and steps through the schedule."""

    name = "cell"
    experimental = True

    def __init__(self):
        """Start with nothing loaded."""
        #: The loaded design, or None. Replaced on load, never edited.
        self.design: CellDesign | None = None

        #: Index of the selected step in `design.steps`.
        self.index = 0

        #: Show the robot at the movement's target configuration instead of its start.
        self.at_target = False

        #: Goes up whenever the design or selected step changes, so dependents can poll it.
        self.revision = 0

        self._load_task: asyncio.Task | None = None
        self._build_task: asyncio.Task | None = None
        # Written by the worker thread, read by draw; plain assignment, so no lock.
        self._progress = ""
        self._error = ""
        # Where the design's tools differ from the monitor's configured ones, one line per robot.
        self._tool_mismatch: list[str] = []
        #: An old-format export the operator loaded, waiting for Convert or a second Load; None otherwise.
        self._pending_export: Path | None = None
        # What the panel asks about it; worked out once, as it reads the export's files.
        self._question = ""

        #: Robot id -> its yourdfpy model, for the drawing.
        self._models: dict = {}
        #: The design's viser nodes, built after a load; None until built.
        self._drawing: DesignDrawing | None = None
        self._visible = True
        self._ghost = False
        # True when the drawing is out of date and draw must re-pose it.
        self._stale = True

    # --- --- --- --- --- WHAT OTHER PLUGINS READ --- --- --- --- ---

    @property
    def step(self) -> Step | None:
        """Step | None: The selected movement, or None before a design is loaded.

        ! Its start state and target are the authored ones: copy before modifying.
        """
        if self.design is None or not self.design.steps:
            return None
        return self.design.steps[self.index]

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, and load the configured design if there is one.

        Args:
            ctx: This plugin's context.
        """
        folder = ctx.config.design_directory
        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            self._folder = gui.add_text("Folder", initial_value=str(folder or ""),
                                        hint="Design folder with design.json (doc/design_format.md), or an "
                                             "export in the old compas_fab format to convert")
            load = gui.add_button_group("Design", ["Load", "Convert"],
                                        hint="Load the folder. For an old-format export, Convert writes it as a "
                                             "design into <export>_design next to it (or switches to that copy) "
                                             "and loads it; Load again opens the export as it is")
            gui.add_html(section("step", SECTION_CTRL))
            self._slider = gui.add_slider("Step", min=0, max=1, step=1, initial_value=0)
            go = gui.add_button_group("Go", ["◀ action", "◀", "▶", "action ▶"],
                                      hint="Previous / next movement, or jump to the previous / next action")
            show = gui.add_button_group("Show", ["Start", "Target"],
                                        hint="The robot at the movement's start state, or at its target configuration")
            view = gui.add_button_group("View", ["Hide", "Ghost"],
                                        hint="Hide / show the drawn state; ghost makes it see-through")
            # ! Keep text that changes per step BELOW the buttons, so they never move mid-click.
            self._details = gui.add_html("")
            self._error_text = gui.add_html("")

        # ! Route every viser callback through defer so it runs on the main thread.
        load.on_click(ctx.defer_value("design button", lambda label: self._on_design(ctx, label)))
        self._slider.on_update(ctx.defer_value("select step", lambda value: self.select(int(value))))
        go.on_click(ctx.defer_value("step button", self._on_go))
        show.on_click(ctx.defer_value("show", lambda label: self._set_at_target(label == "Target")))
        view.on_click(ctx.defer_value("view", self._on_view))

        if folder is not None:
            self.load(ctx, folder)

    # --- --- --- --- --- COMMANDS --- --- --- --- ---
    # Run on the main thread, via intents.

    def load(self, ctx: PluginContext, folder: Path) -> None:
        """Start loading a design folder, replacing the current design when done.

        Args:
            ctx: This plugin's context.
            folder: The design folder.
        """
        if self._load_task is not None and not self._load_task.done():
            ctx.log_info("load ignored: a design is already loading")
            return
        self._load_task = ctx.spawn(f"load {folder.name}", self._load(ctx, folder))

    def _on_design(self, ctx: PluginContext, label: str) -> None:
        """Load the folder in the text field; or convert the old export waiting for an answer.

        Args:
            ctx: This plugin's context.
            label: The button clicked.
        """
        if label == "Load":
            self.load(ctx, Path(self._folder.value.strip()))
        elif self._pending_export is None:
            ctx.log_info("convert ignored: load an export in the old compas_fab format first")
        elif self._load_task is not None and not self._load_task.done():
            ctx.log_info("convert ignored: a design is already loading")
        else:
            export, copy = self._pending_export, converted_folder(self._pending_export)
            self._pending_export = None
            if is_up_to_date(export):
                # * Already converted: switch to the copy.
                self._folder.value = str(copy)
                self._load_task = ctx.spawn(f"load {copy.name}", self._load(ctx, copy))
            else:
                self._load_task = ctx.spawn(f"convert {export.name}", self._convert(ctx, export))

    def select(self, index: int) -> None:
        """Select a step. Clamped to the design; ignored before one is loaded.

        Args:
            index: Index into `design.steps`.
        """
        if self.design is None or not self.design.steps:
            return
        index = max(0, min(int(index), len(self.design.steps) - 1))
        # ! Keep this check: draw writes the slider back, which would otherwise loop.
        if index == self.index:
            return
        self.index = index
        self._changed()

    def _set_at_target(self, at_target: bool) -> None:
        """Show the start state or the target configuration.

        Args:
            at_target: True for the target.
        """
        if at_target != self.at_target:
            self.at_target = at_target
            self._stale = True

    def _on_go(self, label: str) -> None:
        """Step to the previous or next movement or action.

        Args:
            label: The button clicked.
        """
        step = self.step
        if step is None:
            return
        if label == "◀":
            self.select(self.index - 1)
        elif label == "▶":
            self.select(self.index + 1)
        elif label == "◀ action":
            # First movement of this action, or of the previous one if already there.
            action = step.action_index - (1 if step.movement_index == 0 else 0)
            self.select(self.design.first_step_of(max(action, 0)))
        elif label == "action ▶" and step.action_index + 1 < self.design.action_count:
            self.select(self.design.first_step_of(step.action_index + 1))

    def _on_view(self, label: str) -> None:
        """Toggle hiding or ghosting the drawn state.

        Args:
            label: The button clicked.
        """
        if label == "Hide":
            self._visible = not self._visible
        else:
            self._ghost = not self._ghost
            if self._drawing is not None:
                self._drawing.set_ghost(self._ghost)
        self._stale = True

    def _changed(self) -> None:
        """Note that the design or the selection changed."""
        self.revision += 1
        self._stale = True

    # --- --- --- --- --- TASKS --- --- --- --- ---

    async def _load(self, ctx: PluginContext, folder: Path) -> None:
        """Load a design on the worker thread, then swap it in.

        ! Failures are shown in the panel and log, not raised: a bad folder is
          the operator's mistake and must not count as a plugin failure.

        Args:
            ctx: This plugin's context.
            folder: The design folder.
        """
        folder = Path(folder).expanduser()
        old = is_old_export(folder)
        # * An old-format export: the first Load asks (Convert, or Load again to open it as it is).
        if old and self._pending_export != folder:
            self._pending_export, self._question, self._error = folder, _convert_question(folder), ""
            ctx.log_warn(self._question)
            return
        self._pending_export = None
        self._error, self._progress = "", f"loading {folder}"
        try:
            # A thread cannot be interrupted: on timeout or cancel its result is dropped.
            async with timeout(LOAD_TIMEOUT):
                design, models, watch = await ctx.run_in_thread(
                    _load_in_background, folder, lambda text: setattr(self, "_progress", text),
                    ctx.config.data_directory if old else None)
        except asyncio.TimeoutError:
            self._error = f"timed out after {LOAD_TIMEOUT}s waiting for the design to load"
            ctx.log_error(self._error)
            return
        except Exception as failure:
            self._error = f"{type(failure).__name__}: {failure}"
            ctx.log_error(f"could not load {folder}:\n{traceback.format_exc()}")
            return
        finally:
            self._progress = ""

        # * Swap in on the main thread; `update` builds the new nodes on demand.
        if self._build_task is not None:
            self._build_task.cancel()
        if self._drawing is not None:
            self._drawing.remove()
            self._drawing = None
        self.design, self._models, self.index = design, models, 0
        self._changed()
        ctx.log_info(f"loaded {folder} ({'old compas_fab export' if design.is_old_export else 'design'}) "
                     f"in {watch.summary()}; {design.action_count} actions, {len(design.steps)} movements")
        # ? Not counted here: adding the viser nodes, over the next ticks (`_build` logs it).
        self._tool_mismatch = tool_mismatches(design, ctx.config.robots)
        for line in self._tool_mismatch:
            ctx.log_warn(f"design tools differ from the configured ones: {line}")

    async def _convert(self, ctx: PluginContext, export: Path) -> None:
        """Convert an old-format export into `<export>_design` on the worker thread, then load it.

        ! Failures go to the panel and log, as for loading.

        Args:
            ctx: This plugin's context.
            export: The export folder.
        """
        destination = converted_folder(export)
        self._error, self._progress = "", f"converting {export.name}"
        watch = Stopwatch()
        try:
            async with timeout(CONVERT_TIMEOUT):
                await ctx.run_in_thread(convert_export, export, destination, ctx.config.data_directory,
                                        lambda text: setattr(self, "_progress", text), watch)
        except asyncio.TimeoutError:
            self._error = f"timed out after {CONVERT_TIMEOUT}s converting {export}"
            ctx.log_error(self._error)
            return
        except Exception as failure:
            self._error = f"could not convert: {type(failure).__name__}: {failure}"
            ctx.log_error(f"could not convert {export}:\n{traceback.format_exc()}")
            return
        finally:
            self._progress = ""
        ctx.log_info(f"converted {export} into {destination} in {watch.summary()}")
        self._pending_export = None
        self._folder.value = str(destination)
        await self._load(ctx, destination)

    async def _build(self, ctx: PluginContext) -> None:
        """Add the design's viser nodes, a robot per tick.

        Args:
            ctx: This plugin's context.
        """
        self._progress = "drawing the design"
        watch = Stopwatch()
        drawing = DesignDrawing(ctx.view, f"{ctx.view.scene_root}/design", self.design.design, self._models)
        drawing.set_ghost(self._ghost)
        try:
            for _ in drawing.build():
                await ctx.next_tick()
        except BaseException:
            drawing.remove()  # cancelled by a new load, or failed: leave nothing half-built
            raise
        finally:
            self._progress = ""
        self._drawing = drawing
        self._stale = True
        ctx.log_info(f"drew the design in {watch.summary()} (spread over ticks)")

    # --- --- --- --- --- TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Start building the design's viser nodes, if a design is loaded and they are not built yet.

        Args:
            ctx: This plugin's context.
        """
        if self.design is None or self._drawing is not None:
            return
        if self._build_task is None or self._build_task.done():
            self._build_task = ctx.spawn("draw the design", self._build(ctx))

    def draw(self, ctx: PluginContext) -> None:
        """Pose the drawn cell from the selected state, and fill the panel.

        Args:
            ctx: This plugin's context.
        """
        self._status.content = self._status_html()
        step = self.step
        # Error text sits at the bottom so its height moves no buttons.
        self._error_text.content = block(values(escape(self._error))) if self._error else ""
        if self._pending_export is not None:
            # The question sits below the buttons, so asking moves none of them.
            self._details.content = note(escape(self._question))
        elif step is None:
            self._details.content = note("no design loaded")
        if step is None:
            return
        self._slider.max = max(len(self.design.steps) - 1, 1)
        self._slider.value = self.index
        if self._pending_export is None:
            self._details.content = self._details_html(step)

        drawing = self._drawing
        if not self._stale or drawing is None:
            return
        self._stale = False
        drawing.visible = self._visible
        if not self._visible:
            return
        try:
            drawing.show(step.movement.start, displayed_joints(step, self.at_target))
        except Exception as failure:
            # ! A state that does not fit its design is a data problem: report it, do not raise.
            self._error = f"cannot draw {step.label}: {type(failure).__name__}: {failure}"
            ctx.log_error(f"{self._error}\n{traceback.format_exc()}")
        else:
            self._error = ""

    # --- --- --- --- --- PANEL --- --- --- --- ---
    # ! Keep every row one line tall (`_one_line`) and the chip set fixed, so
    #   nothing below shifts between steps.

    def _status_html(self) -> str:
        """One line of chips: load progress or design, plus any error.

        Returns:
            str: HTML.
        """
        if self._progress:
            chips = chip(escape(self._progress), BUSY)
        elif self._pending_export is not None:
            chips = chip("old export: Convert, or Load again", BUSY, self._question)
        elif self.design is not None:
            chips = chip(escape(self.design.folder.name), OK, str(self.design.folder))
            if self.design.is_old_export:
                chips += chip("old format", BUSY, "an export in the old compas_fab format, converted in memory; "
                                                  "Load it again to be offered the conversion")
            chips += chip(f"{self.design.action_count} actions · {len(self.design.steps)} movements", NONE)
            if self._tool_mismatch:
                chips += chip("tools differ", BUSY, "; ".join(self._tool_mismatch))
        else:
            chips = chip("no design", NONE)
        if self._error:
            chips += chip("error", FAIL, self._error)
        return block(_one_line(chips))

    def _details_html(self, step: Step) -> str:
        """Two rows of chips and three of text describing the selected step.

        Args:
            step: The selected step.

        Returns:
            str: HTML.
        """
        movement, action = step.movement, step.action
        robot = self.design.design.robots[action.robot]
        which = chip(f"{self.index + 1}/{len(self.design.steps)}", OK)
        which += chip(escape(robot.name), SECTION_CTRL, action.robot)
        which += chip(movement.type + (" coupled" if movement.coupled else ""), NONE, movement.label)

        # ! Say so when the drawn start is not authored: it is assumed from where the robot last was, or zero.
        target = movement.target.joints.get(action.robot) if movement.target is not None else None
        if movement.start.robots[action.robot].joints is not None:
            carries = chip("start conf", OK)
        elif step.assumed_start is not None:
            carries = chip(f"start conf from {escape(step.assumed_from)}", BUSY,
                           "not authored: assumed from where the robot last was")
        else:
            carries = chip("no start conf: zero", FAIL)
        if not self.at_target:
            carries += chip("showing start", NONE)
        else:
            carries += chip("showing target" if target else "no target: start", OK if target else BUSY)
        carries += chip(movement.controller, NONE)

        moves = [arm.split("/")[-1] for arm in movement.arms] or [tool.split("/")[-1] for tool in movement.tools]
        lines = [f"action   {step.action_index + 1}/{self.design.action_count}  {action.id}",
                 f"movement {step.movement_index + 1}/{len(action.movements)}  {movement.id}",
                 f"moves    {', '.join(moves) or '-'}"]
        text = values(*(_one_line(escape(line)) for line in lines))
        return block(_one_line(which) + _one_line(carries) + text)


def tool_mismatches(design: CellDesign, robots: tuple[RobotConfig, ...]) -> list[str]:
    """Where the design's tools differ from the configured ones, for robots known to both by serial.

    ? Planned states use the design's tools, the measured robot the configured ones; another kind
      on a flange means the plan was made for another tool than the one mounted.

    Args:
        design: The loaded design.
        robots: The configured robots, `ctx.config.robots`.

    Returns:
        list[str]: One line per robot that differs, e.g. "robots/cindy (a200-0806): left_ur_arm
            scaffolding_v3 in the design, robotiq configured".
    """
    by_digits = {robot.serial[-4:]: robot for robot in robots}
    lines = []
    for spec in design.design.robots.values():
        config = by_digits.get(spec.serial or "")
        if config is None:
            continue
        planned = {link[:-len("_tool0")]: design.design.tools[tool].kind
                   for link, tool in spec.tools.items() if link.endswith("_tool0")}
        mounted = {arm.name: arm.end_effector for arm in config.arms}
        differences = [f"{arm} {planned.get(arm) or 'none'} in the design, {mounted.get(arm) or 'none'} configured"
                       for arm in sorted(set(planned) | set(mounted)) if planned.get(arm) != mounted.get(arm)]
        if differences:
            lines.append(f"{spec.id} ({config.serial}): " + "; ".join(differences))
    return lines


def _convert_question(export: Path) -> str:
    """What the panel asks when an old-format export is loaded.

    Args:
        export: The export folder.

    Returns:
        str: The question, naming the converted copy's folder.
    """
    copy = converted_folder(export)
    if is_up_to_date(export):
        convert = f"a converted copy exists: Convert switches to {copy}"
    else:
        convert = f"Convert writes it as a design into {copy} and loads that"
    return (f"{export.name} is an export in the old compas_fab format; {convert}. "
            f"Load again opens the old export as it is.")


def _one_line(html: str) -> str:
    """Keep content on one line, cutting anything too wide with "…".

    Uses `pre` so padding spaces still align the columns.

    Args:
        html: The row's content.

    Returns:
        str: HTML.
    """
    return f'<div style="white-space:pre;overflow:hidden;text-overflow:ellipsis">{html}</div>'
