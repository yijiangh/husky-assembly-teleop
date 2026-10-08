"""
The cell plugin: loads an authored design and draws one movement's cell state at a time.

It serves the selected step to plugins that declare `requires = ("cell",)`.

Every body of the selected movement's design scene (`Design.scene_at`) goes into the scene as "cell/<body id>"
(`design.scene_bodies`): the core draws it and planners avoid it. A body the plan holds follows the configured real
robot. The planned robots, and the bodies they hold, are drawn as an overlay (`drawing.DesignDrawing`), with the other
bodies on request. The plugin only reads designs: it never edits one.

The panel lists a window of the schedule's actions around the selected one (`action_window`); click a row to jump.

Design folders are given relative to the Drive root (`drive.py`), e.g. "data_design_study/<design>", or absolute.
Loading an old compas_fab export converts it into `<export>_design` (reused while up to date) and loads that.
The panel warns when the design's tools differ from the configured ones.

! EXPERIMENTAL: how designs load, and what planners read, may still change.
! Loading a relative folder needs the Drive root (HUSKY_DRIVE_ROOT or `drive_root`).
! Loading runs on a worker thread that only returns a result; viser nodes are added on the main thread.
"""

from __future__ import annotations

import asyncio
import traceback
from html import escape
from pathlib import Path

from typing import TYPE_CHECKING

from async_timeout import timeout
from ...drive import drive_folder
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...ui.style import BUSY, FAIL, NONE, OK, SECTION_CTRL, block, chip, note, section, values
from bar_assembly_core.design_io.timing import Stopwatch
from .design import CellDesign, Step, displayed_joints, load_design, scene_bodies
from .drawing import DesignDrawing, load_robot_models

if TYPE_CHECKING:
    from ...config import RobotConfig

#: Give up on a load after this many seconds (converting an old export takes about 15).
LOAD_TIMEOUT = 300.0

#: Rows in the action list: a window of the schedule, the selected action in the middle row.
ACTION_ROWS = 7


def action_window(current: int, count: int, rows: int = ACTION_ROWS) -> int:
    """The first action shown in the list: `current` in the middle row, except near either end of the schedule.

    Args:
        current: Index of the selected action in the schedule.
        count: How many actions the schedule has.
        rows: How many rows the list has.

    Returns:
        int: Schedule index of the top row.
    """
    return max(0, min(current - rows // 2, count - rows))


def _load_in_background(folder: Path, data_directory: Path, report) -> tuple[CellDesign, dict, Stopwatch]:
    """Load a design, converting an old export first, and read its robots' URDFs with their meshes.

    Runs on the worker thread.

    Args:
        folder: Design folder, or an old export.
        data_directory: The monitor's data directory, for converting an old export.
        report: Called with a progress string.

    Returns:
        tuple: The design, each robot's yourdfpy model by robot id, and the stopwatch.
    """
    watch = Stopwatch()
    design = load_design(folder, data_directory, report, watch)
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

        #: Show the robot at the movement's target instead of its start.
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

        #: Robot id -> its yourdfpy model, for the drawing.
        self._models: dict = {}
        #: The design's viser nodes, built after a load; None until built.
        self._drawing: DesignDrawing | None = None
        self._visible = True
        self._ghost = False
        # Overlay the bodies that do not stand in the state (absent, placeholder) too.
        self._everything = False
        # True when the drawing is out of date and draw must re-pose it.
        self._stale = True
        # Schedule index of the action list's top row, as last drawn; clicks on a row are read against it.
        self._window_start = 0
        # The `revision` the scene's obstacles were last put for.
        self._scene_revision = -1

    # --- --- --- --- --- WHAT OTHER PLUGINS READ --- --- --- --- ---

    @property
    def step(self) -> Step | None:
        """Step | None: The selected movement, or None before a design is loaded. ! Copy before modifying."""
        if self.design is None or not self.design.steps:
            return None
        return self.design.steps[self.index]

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, with the configured design folder filled in; nothing loads until Load is clicked."""
        folder = ctx.config.design_directory
        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            self._folder = gui.add_text("Folder", initial_value=str(folder or ""),
                                        hint="Design folder relative to the Drive root, e.g. "
                                             "data_design_study/<design>, or absolute: with design.json "
                                             "(doc/design_format.md), or an export in the old compas_fab format, "
                                             "converted into <export>_design next to it")
            load = gui.add_button("Load", hint="Load the folder; an old export is converted first (about 15 s) "
                                               "unless its <export>_design copy is up to date")
            self._actions_title = gui.add_html(section("actions", SECTION_CTRL))
            # * A window of the schedule that follows the selection: click a row to jump to that action.
            self._action_rows = [gui.add_button("·", color="gray") for _ in range(ACTION_ROWS)]
            gui.add_html(section("step", SECTION_CTRL))
            self._slider = gui.add_slider("Step", min=0, max=1, step=1, initial_value=0)
            go = gui.add_button_group("Go", ["◀ action", "◀", "▶", "action ▶"],
                                      hint="Previous / next movement, or jump to the previous / next action")
            show = gui.add_button_group("Show", ["Start", "Target"],
                                        hint="The robot at the movement's start state, or at its target configuration")
            view = gui.add_button_group("View", ["Hide", "Ghost", "All"],
                                        hint="Hide / show the overlay (robots, tools, held bodies; standing bodies "
                                             "are scene bodies, always drawn); ghost makes it see-through; all also "
                                             "overlays absent and placeholder bodies, see-through")
            # ! Keep changing text BELOW the buttons, so they never move mid-click.
            self._details = gui.add_html("")
            self._error_text = gui.add_html("")

        # ! Route every viser callback through defer so it runs on the main thread.
        load.on_click(ctx.defer("load design", lambda: self.load(ctx, Path(self._folder.value.strip()))))
        self._slider.on_update(ctx.defer_value("select step", lambda value: self.select(int(value))))
        go.on_click(ctx.defer_value("step button", self._on_go))
        show.on_click(ctx.defer_value("show", lambda label: self._set_at_target(label == "Target")))
        view.on_click(ctx.defer_value("view", self._on_view))
        for row, button in enumerate(self._action_rows):
            button.on_click(ctx.defer("select action", lambda row=row: self._on_action_row(row)))

    # --- --- --- --- --- COMMANDS (intents, on the main thread) --- --- --- --- ---

    def load(self, ctx: PluginContext, folder: Path) -> None:
        """Start loading a design folder, replacing the current design when done.

        Args:
            ctx: This plugin's context.
            folder: The design folder (relative to the Drive root, or absolute), or an old export to convert first.
        """
        if self._load_task is not None and not self._load_task.done():
            ctx.log_info("load ignored: a design is already loading")
            return
        try:
            folder = drive_folder(ctx.config.drive_root, folder)
        except FileNotFoundError as error:
            self._error = str(error)
            return
        self._load_task = ctx.spawn(f"load {folder.name}", self._load(ctx, folder))

    def select(self, index: int) -> None:
        """Select a step by index into `design.steps`, clamped; ignored before a design is loaded."""
        if self.design is None or not self.design.steps:
            return
        index = max(0, min(int(index), len(self.design.steps) - 1))
        # ! Keep this check: draw writes the slider back, which would otherwise loop.
        if index == self.index:
            return
        self.index = index
        self._changed()

    def _set_at_target(self, at_target: bool) -> None:
        """Show the start state, or the target."""
        if at_target != self.at_target:
            self.at_target = at_target
            self._stale = True

    def _on_go(self, label: str) -> None:
        """Step to the previous or next movement or action."""
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

    def _on_action_row(self, row: int) -> None:
        """Jump to the first movement of the action shown in a row of the list; empty rows do nothing."""
        if self.design is None:
            return
        action = self._window_start + row
        if action < self.design.action_count:
            self.select(self.design.first_step_of(action))

    def _on_view(self, label: str) -> None:
        """Toggle hiding or ghosting the overlay, or overlaying every body."""
        if label == "Hide":
            self._visible = not self._visible
        elif label == "All":
            self._everything = not self._everything
        else:
            self._ghost = not self._ghost  # `update` applies it, also to a drawing still being built
        self._stale = True

    def _changed(self) -> None:
        """Note that the design or the selection changed."""
        self.revision += 1
        self._stale = True

    # --- --- --- --- --- TASKS --- --- --- --- ---

    async def _load(self, ctx: PluginContext, folder: Path) -> None:
        """Load a design on the worker thread, then swap it in; failures go to the panel and log, not raised."""
        folder = Path(folder).expanduser()
        self._error, self._progress = "", f"loading {folder}"
        try:
            async with timeout(LOAD_TIMEOUT):
                design, models, watch = await ctx.run_in_thread(
                    _load_in_background, folder, ctx.config.data_directory,
                    lambda text: setattr(self, "_progress", text))
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
        # The converted copy, for an old export.
        root = ctx.config.drive_root
        self._folder.value = str(design.source.relative_to(root) if root and design.source.is_relative_to(root)
                                 else design.source)
        self._changed()
        ctx.log_info(f"loaded {design.source} in {watch.summary()}; "
                     f"{design.action_count} actions, {len(design.steps)} movements")
        self._tool_mismatch = tool_mismatches(design, ctx.config.robots)
        for line in self._tool_mismatch:
            ctx.log_warn(f"design tools differ from the configured ones: {line}")

    async def _build(self, ctx: PluginContext) -> None:
        """Add the design's viser nodes, spread over ticks."""
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
        """Put the selected step's obstacles into the scene, and start building the design's viser nodes."""
        if self._scene_revision != self.revision:
            self._scene_revision = self.revision
            self._put_obstacles(ctx)
        if self._drawing is not None:
            self._drawing.set_ghost(self._ghost)
            return
        if self.design is None:
            return
        if self._build_task is None or self._build_task.done():
            self._build_task = ctx.spawn("draw the design", self._build(ctx))

    def _put_obstacles(self, ctx: PluginContext) -> None:
        """Replace the scene's cell bodies with the selected step's design scene; held bodies follow the real robot.

        ! Runs only when `revision` changes: `scene_at` is too slow for every tick.
        """
        ctx.scene.remove_prefix(f"{self.name}/")
        step = self.step
        if step is None:
            return
        # ? Same ids and geometry objects every step: mirrors only move and switch bodies, never rebuild.
        try:
            bodies = scene_bodies(self.design, step, ctx.config.robots, f"{self.name}/")
        except ValueError as refused:
            # ! A grasp the real robot can't take is a data problem: report it, put the bodies without the robots.
            self._error = f"held bodies left where planned: {refused}"
            ctx.log_error(self._error)
            bodies = scene_bodies(self.design, step, (), f"{self.name}/")
        ctx.scene.put_many(bodies)

    def draw(self, ctx: PluginContext) -> None:
        """Pose the drawn cell from the selected state, and fill the panel."""
        self._status.content = self._status_html()
        step = self.step
        self._error_text.content = block(values(escape(self._error))) if self._error else ""
        self._draw_action_list(step)
        if step is None:
            self._details.content = note("no design loaded")
            return
        self._slider.max = max(len(self.design.steps) - 1, 1)
        self._slider.value = self.index
        self._details.content = self._details_html(step)

        drawing = self._drawing
        if not self._stale or drawing is None:
            return
        self._stale = False
        drawing.visible = self._visible
        if not self._visible:
            return
        try:
            drawing.show(step.movement.start, displayed_joints(step, self.at_target), self._everything)
        except Exception as failure:
            # ! A state that does not fit its design is a data problem: report it, do not raise.
            self._error = f"cannot draw {step.label}: {type(failure).__name__}: {failure}"
            ctx.log_error(f"{self._error}\n{traceback.format_exc()}")
        else:
            self._error = ""

    # --- --- --- --- --- PANEL --- --- --- --- ---

    def _draw_action_list(self, step: Step | None) -> None:
        """Label the action rows from the window around the selected action, and highlight the selected one.

        ? viser sends only props that changed, so assigning every row each draw is cheap.
        """
        count = self.design.action_count if self.design is not None else 0
        current = step.action_index if step is not None else 0
        self._window_start = action_window(current, count)
        self._actions_title.content = section("actions", SECTION_CTRL, f"{current + 1}/{count}" if count else "")
        for row, button in enumerate(self._action_rows):
            index = self._window_start + row
            if index >= count:
                # ! Never hide a row: that would shift the buttons below it.
                button.label, button.color = "·", "gray"
                continue
            action = self.design.design.actions[self.design.design.schedule[index]]
            robot = self.design.design.robots[action.robot].name
            button.label = f"{index + 1}. {action.id} · {robot} · {len(action.movements)} mv"
            button.color = "blue" if index == current else "gray"
    # ! Keep every row one line tall (`_one_line`) and the chip set fixed, so nothing shifts between steps.

    def _status_html(self) -> str:
        """One line of chips: load progress or design, plus any error."""
        if self._progress:
            chips = chip(escape(self._progress), BUSY)
        elif self.design is not None:
            chips = chip(escape(self.design.source.name), OK, str(self.design.source))
            chips += chip(f"{self.design.action_count} actions · {len(self.design.steps)} movements", NONE)
            if self._tool_mismatch:
                chips += chip("tools differ", BUSY, "; ".join(self._tool_mismatch))
        else:
            chips = chip("no design", NONE)
        if self._error:
            chips += chip("error", FAIL, self._error)
        return block(_one_line(chips))

    def _details_html(self, step: Step) -> str:
        """Two rows of chips and three of text describing the selected step."""
        movement, action = step.movement, step.action
        robot = self.design.design.robots[action.robot]
        which = chip(f"{self.index + 1}/{len(self.design.steps)}", OK)
        which += chip(escape(robot.name), SECTION_CTRL, action.robot)
        which += chip(movement.type + (" coupled" if movement.coupled else ""), NONE, movement.label)

        # ! Say so when the drawn start is not authored.
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
    """Where the design's tools differ from the configured (mounted) ones, for robots known to both by serial.

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


def _one_line(html: str) -> str:
    """Keep content on one line, cutting anything too wide with "…"; `pre` keeps padded columns aligned."""
    return f'<div style="white-space:pre;overflow:hidden;text-overflow:ellipsis">{html}</div>'
