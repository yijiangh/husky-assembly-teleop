"""
The cell plugin: loads an authored process and draws one movement's cell state at a time.

It serves the selected step to plugins that declare `requires = ("cell",)`.

Every body of the selected movement's process scene (`Process.scene_at`) goes into the scene as "cell/<body id>"
(`process.scene_bodies`): the core draws it and planners avoid it. A body the plan holds follows the configured real
robot. The planned robots, and the bodies they hold, are drawn as an overlay (`drawing.ProcessDrawing`), with the other
bodies on request. The plugin only reads processes: it never edits one.

The panel lists a window of the schedule's actions around the selected one (`action_window`); click a row to jump.

Process folders are given relative to the Drive root (`drive.py`), e.g. "data_design_study/<process>", or absolute.
Loading an old compas_fab export converts it into `<export>_process` (reused while up to date) and loads that.
The panel warns when the process's tools differ from the configured ones.

! EXPERIMENTAL: how processes load, and what planners read, may still change.
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
from scaffolding_assembly_core.legacy.timing import Stopwatch
from .process import CellProcess, Step, displayed_joints, load_process, scene_bodies
from .drawing import ProcessDrawing, load_robot_models

if TYPE_CHECKING:
    from ...config import RobotConfig

#: Give up on a load after this many seconds (converting an old export takes about 15).
LOAD_TIMEOUT = 300.0

#: Rows in the action list: a window of the schedule, the selected action in the middle row.
ACTION_ROWS = 7

#: Shown in place of the robot for a `bar_placing` action, which no robot acts in.
BY_HAND_LABEL = "by hand"


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


def _load_in_background(folder: Path, data_directory: Path, report) -> tuple[CellProcess, dict, Stopwatch]:
    """Load a process, converting an old export first, and read its robots' URDFs with their meshes.

    Runs on the worker thread.

    Args:
        folder: Process folder, or an old export.
        data_directory: The monitor's data directory, for converting an old export.
        report: Called with a progress string.

    Returns:
        tuple: The process, each robot's yourdfpy model by robot id, and the stopwatch.
    """
    watch = Stopwatch()
    process = load_process(folder, data_directory, report, watch)
    report("reading the robots' URDFs and meshes")
    models = load_robot_models(process.process)
    watch.lap("robot URDFs and meshes")
    return process, models, watch


@register
class CellPlugin(HuskyPlugin):
    """Shows one authored robot cell state at a time and steps through the schedule."""

    name = "cell"
    experimental = True

    def __init__(self):
        """Start with nothing loaded."""
        #: The loaded process, or None. Replaced on load, never edited.
        self.process: CellProcess | None = None

        #: Index of the selected step in `process.steps`.
        self.index = 0

        #: Show the robot at the movement's target instead of its start.
        self.at_target = False

        #: Goes up whenever the process or selected step changes, so dependents can poll it.
        self.revision = 0

        self._load_task: asyncio.Task | None = None
        self._build_task: asyncio.Task | None = None
        # Written by the worker thread, read by draw; plain assignment, so no lock.
        self._progress = ""
        self._error = ""
        # Where the process's tools differ from the monitor's configured ones, one line per robot.
        self._tool_mismatch: list[str] = []

        #: Robot id -> its yourdfpy model, for the drawing.
        self._models: dict = {}
        #: The process's viser nodes, built after a load; None until built.
        self._drawing: ProcessDrawing | None = None
        self._visible = True
        self._ghost = False
        # Overlay the bodies that do not stand in the state (absent ones) too.
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
        """Step | None: The selected movement, or None before a process is loaded. ! Copy before modifying."""
        if self.process is None or not self.process.steps:
            return None
        return self.process.steps[self.index]

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel, with the configured process folder filled in; nothing loads until Load is clicked."""
        folder = ctx.config.process_directory
        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            self._folder = gui.add_text("Folder", initial_value=str(folder or ""),
                                        hint="Process folder relative to the Drive root, e.g. "
                                             "data_design_study/<process>, or absolute: with process.json "
                                             "(doc/process_format.md), or an export in the old compas_fab format, "
                                             "converted into <export>_process next to it")
            load = gui.add_button("Load", hint="Load the folder; an old export is converted first (about 15 s) "
                                               "unless its <export>_process copy is up to date")
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
                                             "overlays absent bodies, see-through")
            # ! Keep changing text BELOW the buttons, so they never move mid-click.
            self._details = gui.add_html("")
            self._error_text = gui.add_html("")

        # ! Route every viser callback through defer so it runs on the main thread.
        load.on_click(ctx.defer("load process", lambda: self.load(ctx, Path(self._folder.value.strip()))))
        self._slider.on_update(ctx.defer_value("select step", lambda value: self.select(int(value))))
        go.on_click(ctx.defer_value("step button", self._on_go))
        show.on_click(ctx.defer_value("show", lambda label: self._set_at_target(label == "Target")))
        view.on_click(ctx.defer_value("view", self._on_view))
        for row, button in enumerate(self._action_rows):
            button.on_click(ctx.defer("select action", lambda row=row: self._on_action_row(row)))

    # --- --- --- --- --- COMMANDS (intents, on the main thread) --- --- --- --- ---

    def load(self, ctx: PluginContext, folder: Path) -> None:
        """Start loading a process folder, replacing the current process when done.

        Args:
            ctx: This plugin's context.
            folder: The process folder (relative to the Drive root, or absolute), or an old export to convert first.
        """
        if self._load_task is not None and not self._load_task.done():
            ctx.log_info("load ignored: a process is already loading")
            return
        try:
            folder = drive_folder(ctx.config.drive_root, folder)
        except FileNotFoundError as error:
            self._error = str(error)
            return
        self._load_task = ctx.spawn(f"load {folder.name}", self._load(ctx, folder))

    def select(self, index: int) -> None:
        """Select a step by index into `process.steps`, clamped; ignored before a process is loaded."""
        if self.process is None or not self.process.steps:
            return
        index = max(0, min(int(index), len(self.process.steps) - 1))
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
            self.select(self.process.first_step_of(max(action, 0)))
        elif label == "action ▶" and step.action_index + 1 < self.process.action_count:
            self.select(self.process.first_step_of(step.action_index + 1))

    def _on_action_row(self, row: int) -> None:
        """Jump to the first movement of the action shown in a row of the list; empty rows do nothing."""
        if self.process is None:
            return
        action = self._window_start + row
        if action < self.process.action_count:
            self.select(self.process.first_step_of(action))

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
        """Note that the process or the selection changed."""
        self.revision += 1
        self._stale = True

    # --- --- --- --- --- TASKS --- --- --- --- ---

    async def _load(self, ctx: PluginContext, folder: Path) -> None:
        """Load a process on the worker thread, then swap it in; failures go to the panel and log, not raised."""
        folder = Path(folder).expanduser()
        self._error, self._progress = "", f"loading {folder}"
        try:
            async with timeout(LOAD_TIMEOUT):
                process, models, watch = await ctx.run_in_thread(
                    _load_in_background, folder, ctx.config.data_directory,
                    lambda text: setattr(self, "_progress", text))
        except asyncio.TimeoutError:
            self._error = f"timed out after {LOAD_TIMEOUT}s waiting for the process to load"
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
        self.process, self._models, self.index = process, models, 0
        # The converted copy, for an old export.
        root = ctx.config.drive_root
        self._folder.value = str(process.source.relative_to(root) if root and process.source.is_relative_to(root)
                                 else process.source)
        self._changed()
        ctx.log_info(f"loaded {process.source} in {watch.summary()}; "
                     f"{process.action_count} actions, {len(process.steps)} movements")
        self._tool_mismatch = tool_mismatches(process, ctx.config.robots)
        for line in self._tool_mismatch:
            ctx.log_warn(f"process tools differ from the configured ones: {line}")

    async def _build(self, ctx: PluginContext) -> None:
        """Add the process's viser nodes, spread over ticks."""
        self._progress = "drawing the process"
        watch = Stopwatch()
        drawing = ProcessDrawing(ctx.view, f"{ctx.view.scene_root}/process", self.process.process, self._models)
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
        ctx.log_info(f"drew the process in {watch.summary()} (spread over ticks)")

    # --- --- --- --- --- TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Put the selected step's obstacles into the scene, and start building the process's viser nodes."""
        if self._scene_revision != self.revision:
            self._scene_revision = self.revision
            self._put_obstacles(ctx)
        if self._drawing is not None:
            self._drawing.set_ghost(self._ghost)
            return
        if self.process is None:
            return
        if self._build_task is None or self._build_task.done():
            self._build_task = ctx.spawn("draw the process", self._build(ctx))

    def _put_obstacles(self, ctx: PluginContext) -> None:
        """Replace the scene's cell bodies with the selected step's process scene; held bodies follow the real robot.

        ! Runs only when `revision` changes: `scene_at` is too slow for every tick.
        """
        ctx.scene.remove_prefix(f"{self.name}/")
        step = self.step
        if step is None:
            return
        # ? Same ids and geometry objects every step: mirrors only move and switch bodies, never rebuild.
        try:
            bodies = scene_bodies(self.process, step, ctx.config.robots, f"{self.name}/")
        except ValueError as refused:
            # ! A grasp the real robot can't take is a data problem: report it, put the bodies without the robots.
            self._error = f"held bodies left where planned: {refused}"
            ctx.log_error(self._error)
            bodies = scene_bodies(self.process, step, (), f"{self.name}/")
        ctx.scene.put_many(bodies)

    def draw(self, ctx: PluginContext) -> None:
        """Pose the drawn cell from the selected state, and fill the panel."""
        self._status.content = self._status_html()
        step = self.step
        self._error_text.content = block(values(escape(self._error))) if self._error else ""
        self._draw_action_list(step)
        if step is None:
            self._details.content = note("no process loaded")
            return
        self._slider.max = max(len(self.process.steps) - 1, 1)
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
            # ! A state that does not fit its process is a data problem: report it, do not raise.
            self._error = f"cannot draw {step.label}: {type(failure).__name__}: {failure}"
            ctx.log_error(f"{self._error}\n{traceback.format_exc()}")
        else:
            self._error = ""

    # --- --- --- --- --- PANEL --- --- --- --- ---

    def _draw_action_list(self, step: Step | None) -> None:
        """Label the action rows from the window around the selected action, and highlight the selected one.

        ? viser sends only props that changed, so assigning every row each draw is cheap.
        """
        count = self.process.action_count if self.process is not None else 0
        current = step.action_index if step is not None else 0
        self._window_start = action_window(current, count)
        self._actions_title.content = section("actions", SECTION_CTRL, f"{current + 1}/{count}" if count else "")
        for row, button in enumerate(self._action_rows):
            index = self._window_start + row
            if index >= count:
                # ! Never hide a row: that would shift the buttons below it.
                button.label, button.color = "·", "gray"
                continue
            action = self.process.process.actions[self.process.process.schedule[index]]
            robot = self.process.process.robots[action.robot].name if action.robot else BY_HAND_LABEL
            button.label = f"{index + 1}. {action.id} · {robot} · {len(action.movements)} mv"
            button.color = "blue" if index == current else "gray"
    # ! Keep every row one line tall (`_one_line`) and the chip set fixed, so nothing shifts between steps.

    def _status_html(self) -> str:
        """One line of chips: load progress or process, plus any error."""
        if self._progress:
            chips = chip(escape(self._progress), BUSY)
        elif self.process is not None:
            chips = chip(escape(self.process.source.name), OK, str(self.process.source))
            chips += chip(f"{self.process.action_count} actions · {len(self.process.steps)} movements", NONE)
            if self._tool_mismatch:
                chips += chip("tools differ", BUSY, "; ".join(self._tool_mismatch))
        else:
            chips = chip("no process", NONE)
        if self._error:
            chips += chip("error", FAIL, self._error)
        return block(_one_line(chips))

    def _details_html(self, step: Step) -> str:
        """Two rows of chips and three of text describing the selected step."""
        movement, action = step.movement, step.action
        which = chip(f"{self.index + 1}/{len(self.process.steps)}", OK)
        if action.robot:
            which += chip(escape(self.process.process.robots[action.robot].name), SECTION_CTRL, action.robot)
        else:
            which += chip(BY_HAND_LABEL, SECTION_CTRL, "a bar_placing action: the operator places the bar")
        which += chip(escape(movement_kind(movement)), NONE, movement.label)

        # ! Say so when the drawn start is not authored.
        target = movement.target.joints.get(action.robot) if movement.target is not None else None
        start = movement.start.robots.get(action.robot)
        if not action.robot:
            carries = chip("no robot acts", NONE)
        elif start is not None and start.joints is not None:
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
        carries += chip(movement.controller or "no arm moves", NONE)
        carries += chip(f"ends on {movement.ends_on}", NONE)

        moves = [arm.split("/")[-1] for arm in movement.arms] + [
            f"{tool.split('/')[-1]} grip {grip}" for tool, grip in movement.grip_change.items()] + [
            f"{tool.split('/')[-1]} {direction}" for tool, direction in movement.drives.items()]
        lines = [f"action   {step.action_index + 1}/{self.process.action_count}  {action.id}",
                 f"movement {step.movement_index + 1}/{len(action.movements)}  {movement.id}",
                 f"moves    {', '.join(moves) or '-'}"]
        text = values(*(_one_line(escape(line)) for line in lines))
        return block(_one_line(which) + _one_line(carries) + text)


def movement_kind(movement) -> str:
    """A movement's kind from its parts, e.g. "linear coupled + tools", "tools", "manual"."""
    if movement.ends_on == "operator":
        return "manual"
    arms = f"{movement.path}{' coupled' if movement.coupled else ''}" if movement.arms else ""
    tools = "tools" if movement.has_tool_part else ""
    return " + ".join(part for part in (arms, tools) if part)


def tool_mismatches(process: CellProcess, robots: tuple[RobotConfig, ...]) -> list[str]:
    """Where the process's tools differ from the configured (mounted) ones, for robots known to both by serial.

    Args:
        process: The loaded process.
        robots: The configured robots, `ctx.config.robots`.

    Returns:
        list[str]: One line per robot that differs, e.g. "robots/cindy (a200-0806): left_ur_arm
            scaffolding_v3 in the process, robotiq configured".
    """
    by_digits = {robot.serial[-4:]: robot for robot in robots}
    lines = []
    for spec in process.process.robots.values():
        config = by_digits.get(spec.serial or "")
        if config is None:
            continue
        planned = {link[:-len("_tool0")]: process.process.tools[tool].kind
                   for link, tool in spec.tools.items() if link.endswith("_tool0")}
        mounted = {arm.name: arm.end_effector for arm in config.arms}
        differences = [f"{arm} {planned.get(arm) or 'none'} in the process, {mounted.get(arm) or 'none'} configured"
                       for arm in sorted(set(planned) | set(mounted)) if planned.get(arm) != mounted.get(arm)]
        if differences:
            lines.append(f"{spec.id} ({config.serial}): " + "; ".join(differences))
    return lines


def _one_line(html: str) -> str:
    """Keep content on one line, cutting anything too wide with "…"; `pre` keeps padded columns aligned."""
    return f'<div style="white-space:pre;overflow:hidden;text-overflow:ellipsis">{html}</div>'
