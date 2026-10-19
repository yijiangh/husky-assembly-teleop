"""
The process plugin: loads an authored process and draws the state at one movement at a time.

It serves the selected movement, and the maps between process ids and live scene ids, to plugins that declare
`requires = ("process",)`: they read `selection` (`shared.process_contract.ProcessContract`).

Every body of the selected movement's process scene (`Process.scene_at`) goes into the scene as "process/<body id>"
(`scene_bodies.py`): the core draws it and planners avoid it. A body the plan holds follows the configured real
robot. The planned robots, and the bodies they hold, are drawn as an overlay (`drawing.ProcessDrawing`), with the other
bodies on request. The plugin only reads processes: it never edits one.

The panel lists a window of the schedule's actions around the selected one (`action_window`); click a row to jump.

Process folders are given relative to the Drive root (`drive.py`), e.g. "data_design_study/<process>", or absolute.
Loading an old compas_fab export converts it into `<export>_process` (reused while up to date) and loads that.
The panel warns where a process robot or its tools differ from the configured real robot (`world/model_check.py`).

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
from ...world.model_check import model_differences
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.legacy.timing import Stopwatch
from scaffolding_assembly_core.process import Action, Movement, Process, assumed_start_all, joints_at
from scaffolding_assembly_core.process.scenes import process_model
from ...shared.process_contract import Selection
from .drawing import ProcessDrawing, load_robot_models
from .loading import load_process
from .scene_bodies import live_ids, scene_bodies

if TYPE_CHECKING:
    from scaffolding_assembly_core.ids import IdMap
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


def _load_in_background(folder: Path, data_directory: Path, report) -> tuple[Process, dict, Stopwatch]:
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
    models = load_robot_models(process)
    watch.lap("robot URDFs and meshes")
    return process, models, watch


@register
class ProcessPlugin(HuskyPlugin):
    """Shows the authored state at one movement at a time and steps through the schedule."""

    name = "process"
    experimental = True

    def __init__(self):
        """Start with nothing loaded."""
        #: The loaded process, or None. Replaced on load, never edited.
        self.process: Process | None = None
        #: Every (action, movement) of the schedule, in order (`Process.movements`); `index` points into it.
        self._movements: tuple[tuple[Action, Movement], ...] = ()
        #: Schedule index of each movement's action, and the first movement of each action with any.
        self._action_of: tuple[int, ...] = ()
        self._first_of: dict[int, int] = {}
        #: The assumed start joints of every movement (`assumed_start_all`), for drawing and the panel.
        self._assumed: dict = {}

        #: Index of the selected movement in `_movements`.
        self.index = 0

        #: Show the robot at the movement's target instead of its start.
        self.at_target = False

        #: Goes up whenever the process or selected movement changes; the scene bodies are put again then.
        self.revision = 0

        #: Process id -> live scene id ("bars/B1" -> "process/bars/B1", planned robot -> real robot), as the scene
        #: bodies were put; None until they are.
        self.ids: IdMap | None = None
        #: Live scene id -> process id: the way back, e.g. for a body a planner reports.
        self.process_ids: IdMap | None = None

        self._load_task: asyncio.Task | None = None
        self._build_task: asyncio.Task | None = None
        # Written by the worker thread, read by draw; plain assignment, so no lock.
        self._progress = ""
        self._error = ""
        # Where the process's robots differ from the configured ones, one line per robot (`robot_differences`).
        self._robot_differences: list[str] = []

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
        # What `selection` last returned, kept while nothing in it changes.
        self._selection: Selection | None = None

    # --- --- --- --- --- WHAT OTHER PLUGINS READ --- --- --- --- ---

    @property
    def selection(self) -> Selection | None:
        """Selection | None: The selected movement (`ProcessContract`); a new object whenever it changes."""
        selected = self.selected
        if selected is None:
            return None
        last = self._selection
        if (last is None or last.process is not self.process or last.index != self.index or last.ids is not self.ids
                or last.process_ids is not self.process_ids):
            self._selection = Selection(self.process, self.index, *selected, self.ids, self.process_ids)
        return self._selection

    @property
    def selected(self) -> tuple[Action, Movement] | None:
        """tuple[Action, Movement] | None: The selected movement and its action, or None before a process is loaded."""
        return self._movements[self.index] if self._movements else None

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
        """Select a movement by index into `_movements`, clamped; ignored before a process is loaded."""
        if not self._movements:
            return
        index = max(0, min(int(index), len(self._movements) - 1))
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
        """Go to the previous or next movement or action."""
        if not self._movements:
            return
        current = self._action_of[self.index]
        if label == "◀":
            self.select(self.index - 1)
        elif label == "▶":
            self.select(self.index + 1)
        elif label == "◀ action":
            # First movement of this action, or of the previous action with movements if already there.
            earlier = [first for action, first in self._first_of.items() if first < self.index and action <= current]
            self.select(max(earlier) if earlier else 0)
        elif label == "action ▶":
            later = [first for first in self._first_of.values() if first > self.index]
            if later:
                self.select(min(later))

    def _on_action_row(self, row: int) -> None:
        """Jump to the first movement of the action shown in a row of the list; empty rows and actions do nothing."""
        first = self._first_of.get(self._window_start + row)
        if first is not None:
            self.select(first)

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
                differences = await ctx.run_in_thread(robot_differences, process, ctx.config.robots)
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
        self._take(process)
        self._models = models
        # The converted copy, for an old export.
        root = ctx.config.drive_root
        source = process.folder
        self._folder.value = str(source.relative_to(root) if root and source.is_relative_to(root) else source)
        self._changed()
        ctx.log_info(f"loaded {source} in {watch.summary()}; "
                     f"{len(process.schedule)} actions, {len(self._movements)} movements")
        self._robot_differences = differences
        for line in differences:
            ctx.log_warn(f"a process robot differs from the configured one: {line}")

    def _take(self, process: Process) -> None:
        """Make `process` the loaded one, with its movement list and assumed starts, and select its first movement."""
        schedule_index = {action_id: index for index, action_id in enumerate(process.schedule)}
        self.process, self.index = process, 0
        self._movements = tuple(process.movements())
        self._action_of = tuple(schedule_index[action.id] for action, _ in self._movements)
        self._first_of = {}
        for index, action in enumerate(self._action_of):
            self._first_of.setdefault(action, index)
        self._assumed = assumed_start_all(process)

    async def _build(self, ctx: PluginContext) -> None:
        """Add the process's viser nodes, spread over ticks."""
        self._progress = "drawing the process"
        watch = Stopwatch()
        drawing = ProcessDrawing(ctx.view, f"{ctx.view.scene_root}/overlay", self.process, self._models)
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
        """Put the selected movement's obstacles into the scene, and start building the process's viser nodes."""
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
        """Replace this plugin's scene bodies with the selected movement's process scene; held bodies follow the real robot.

        ! Runs only when `revision` changes: `scene_at` is too slow for every tick.
        """
        prefix = f"{self.name}/"
        ctx.scene.remove_prefix(prefix)
        self.ids = self.process_ids = None
        selected = self.selected
        if selected is None:
            return
        movement = selected[1]
        # ? Same ids and geometry objects every movement: mirrors only move and switch bodies, never rebuild.
        robots = ctx.config.robots
        try:
            bodies = scene_bodies(self.process, movement, robots, prefix)
        except ValueError as refused:
            # ! A grasp the real robot can't take is a data problem: report it, put the bodies without the robots.
            self._error = f"held bodies left where planned: {refused}"
            ctx.log_error(self._error)
            robots = ()
            bodies = scene_bodies(self.process, movement, robots, prefix)
        ctx.scene.put_many(bodies)
        self.ids, self.process_ids = live_ids(self.process, robots, prefix)

    def draw(self, ctx: PluginContext) -> None:
        """Pose the overlay from the selected state, and fill the panel."""
        self._status.content = self._status_html()
        selected = self.selected
        self._error_text.content = block(values(escape(self._error))) if self._error else ""
        self._draw_action_list()
        if selected is None:
            self._details.content = note("no process loaded")
            return
        action, movement = selected
        self._slider.max = max(len(self._movements) - 1, 1)
        self._slider.value = self.index
        self._details.content = self._details_html(action, movement)

        drawing = self._drawing
        if not self._stale or drawing is None:
            return
        self._stale = False
        drawing.visible = self._visible
        if not self._visible:
            return
        try:
            joints = joints_at(self.process, action.id, movement.id, self.at_target, self._assumed)
            drawing.show(movement.start, joints, self._everything)
        except Exception as failure:
            # ! A state that does not fit its process is a data problem: report it, do not raise.
            self._error = f"cannot draw {movement.id}: {type(failure).__name__}: {failure}"
            ctx.log_error(f"{self._error}\n{traceback.format_exc()}")
        else:
            self._error = ""

    # --- --- --- --- --- PANEL --- --- --- --- ---

    def _draw_action_list(self) -> None:
        """Label the action rows from the window around the selected action, and highlight the selected one.

        ? viser sends only props that changed, so assigning every row each draw is cheap.
        """
        count = len(self.process.schedule) if self.process is not None else 0
        current = self._action_of[self.index] if self._movements else 0
        self._window_start = action_window(current, count)
        self._actions_title.content = section("actions", SECTION_CTRL, f"{current + 1}/{count}" if count else "")
        for row, button in enumerate(self._action_rows):
            index = self._window_start + row
            if index >= count:
                # ! Never hide a row: that would shift the buttons below it.
                button.label, button.color = "·", "gray"
                continue
            action = self.process.actions[self.process.schedule[index]]
            robot = self.process.robots[action.robot].name if action.robot else BY_HAND_LABEL
            button.label = f"{index + 1}. {action.id} · {robot} · {len(action.movements)} mv"
            button.color = "blue" if index == current else "gray"
    # ! Keep every row one line tall (`_one_line`) and the chip set fixed, so nothing shifts between steps.

    def _status_html(self) -> str:
        """One line of chips: load progress or process, plus any error."""
        if self._progress:
            chips = chip(escape(self._progress), BUSY)
        elif self.process is not None:
            chips = chip(escape(self.process.folder.name), OK, str(self.process.folder))
            chips += chip(f"{len(self.process.schedule)} actions · {len(self._movements)} movements", NONE)
            if self._robot_differences:
                chips += chip("robots differ", BUSY, "; ".join(self._robot_differences))
        else:
            chips = chip("no process", NONE)
        if self._error:
            chips += chip("error", FAIL, self._error)
        return block(_one_line(chips))

    def _details_html(self, action: Action, movement: Movement) -> str:
        """Two rows of chips and three of text describing the selected movement."""
        which = chip(f"{self.index + 1}/{len(self._movements)}", OK)
        if action.robot:
            which += chip(escape(self.process.robots[action.robot].name), SECTION_CTRL, action.robot)
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
        elif self._assumed.get((action.id, movement.id), (None, ""))[0] is not None:
            carries = chip(f"start conf from {escape(self._assumed[(action.id, movement.id)][1])}", BUSY,
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
        lines = [f"action   {self._action_of[self.index] + 1}/{len(self.process.schedule)}  {action.id}",
                 f"movement {action.movements.index(movement) + 1}/{len(action.movements)}  {movement.id}",
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


def robot_differences(process: Process, robots: tuple[RobotConfig, ...]) -> list[str]:
    """Where each process robot differs from the configured real robot it stands for (`live_ids`). Any thread.

    Args:
        process: The loaded process.
        robots: The configured robots, `ctx.config.robots`.

    Returns:
        list[str]: One line per robot that differs, e.g. "robots/cindy (a200-0806): left_ur_arm_tool0:
            scaffolding_v3 in the process, robotiq configured".
    """
    ids, _ = live_ids(process, robots, "")
    configured = {robot_id(config.serial): config for config in robots}
    lines = []
    for planned_id in process.robots:
        config = configured.get(ids.get(planned_id))
        if config is None:
            continue  # stands for no configured robot
        differences = model_differences(process_model(process, planned_id), config.model)
        if differences:
            lines.append(f"{planned_id} ({config.serial}): " + "; ".join(differences))
    return lines


def _one_line(html: str) -> str:
    """Keep content on one line, cutting anything too wide with "…"; `pre` keeps padded columns aligned."""
    return f'<div style="white-space:pre;overflow:hidden;text-overflow:ellipsis">{html}</div>'
