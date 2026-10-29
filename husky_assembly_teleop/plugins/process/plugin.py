"""
The process plugin: loads an authored process and draws the state at one movement at a time.

It serves the selected movement, and the maps between process ids and live scene ids, to plugins that declare
`requires = ("process",)`: they read `selection` and may select a movement with `request_select`
(`shared.process_contract.ProcessContract`).

Every body of the selected movement's process scene (`Process.scene_at`) goes into the scene as "process/<body id>"
(`scene_bodies.py`): the runtime draws it and planners avoid it. A body the plan holds follows the configured real
robot. The planned robots, and the bodies they hold, are drawn as an overlay (`drawing.ProcessDrawing`), with the other
bodies on request. The plugin only reads processes: it never edits one.

The "Transfer workaround" box (on at start, this session only) runs Cindy's transfer as a free move with empty tools
to the insert start, where the operator places the bar. While it is on, the panel lists and draws each jointing action
in that order (`steps.as_run`); `process` stays the authored process, and `Selection.index` an authored index.

The "Rhino obstacles collide" box (off at start) decides whether the process's obstacles ("obstacles/<name>")
count in every collision check; while off they are disabled scene bodies, drawn see-through by the overlay.

The panel is the top of the side panel's Assembly tab (`ui/assembly_tab.py`), above the steps planned and run from
it. It lists a window of the schedule's actions around the selected one (`action_window`); click a row to jump. The
selected action's movements are buttons coloured by kind, the selected one ringed: click one to select it.
Each row shows the mark the run gives its action (`request_marks`: pending, done, done with its end assumed). After a
Live IK or a plan, the action's later movements are drawn with the joints found, on the live base they were found at,
and the panel turns to the target of the movement they are for (`request_carry`); the drawing only.

Process folders are given relative to the Drive root (`drive.py`), e.g. "data_design_study/<process>", or absolute.
Loading an old compas_fab export converts it into `<export>_process` (reused while up to date) and loads that.
The panel warns where a process robot or its tools differ from the configured real robot (`world/model_check.py`).

! Loading a relative folder needs the Drive root (HUSKY_DRIVE_ROOT or `drive_root`).
! Loading runs on a worker thread that only returns a result; viser nodes are added on the main thread.
"""

from __future__ import annotations

import asyncio
import traceback
from dataclasses import replace
from html import escape
from pathlib import Path

from typing import TYPE_CHECKING, Mapping

from async_timeout import timeout
from ...drive import drive_folder
from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...ui.assembly_tab import ASSEMBLY_ICON, ASSEMBLY_TAB, PROCESS_BAND
from ...ui.phone import group_buttons, group_tooltips_css, one_row_css
from ...ui.style import (BUSY, FAIL, NONE, OK, REFUSAL_SHOWN, RING, SECTION_CTRL, block, chip, chip_line, section,
                         swatch_chip, values)
from ...world.model_check import model_differences
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.legacy.timing import Stopwatch
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.process import Action, Movement, Process, State, assumed_start_all, joints_at
from scaffolding_assembly_core.process.scenes import process_model
from ...shared.process_contract import MARK_ASSUMED, MARK_DONE, Carry, Selection
from ...shared.assembly_planning.labels import Kind, MovementLabel, label_movement
from ...shared.assembly_planning.movement import mating_drives
from ...shared.assembly_planning.steps import as_run
from ...shared.movement_exec_contract import step_running
from ...shared.process_scene.scene_bodies import live_ids, process_obstacles, scene_bodies
from ...ui.checklist import ACTION_ROWS, action_window
from .drawing import ProcessDrawing, load_robot_models
from .loading import load_process

if TYPE_CHECKING:
    from scaffolding_assembly_core.ids import IdMap
    from ...config import RobotConfig

#: Give up on a load after this many seconds (converting an old export takes about 15).
LOAD_TIMEOUT = 300.0

#: Shown in place of the robot for a `bar_placing` action, which no robot acts in.
BY_HAND_LABEL = "by hand"

#: The movement buttons' only button before a load.
NO_MOVEMENT = "—"

#: Whether the transfer workaround is on when the monitor starts: Cindy's transfer runs as a free move with empty tools.
TRANSFER_AS_FREE = True

#: Whether the process's obstacles from Rhino count in collision checks when the monitor starts.
#: ! Off: they were never measured (the cabinet stands 0.38 m off); the `obstacles` plugin's measured ones are the room.
OBSTACLES_COLLIDE = False

#: Scene ids of the `obstacles` plugin's measured obstacles; not the process's own "process/obstacles/…".
MEASURED_PREFIX = "obstacles/"


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
    tab_path = (ASSEMBLY_TAB,)
    # * The Assembly tab's icon: this plugin makes the tab, since the other three require it.
    icon = ASSEMBLY_ICON

    def __init__(self):
        """Start with nothing loaded."""
        #: The loaded process as authored, or None. Replaced on load, never edited.
        self.process: Process | None = None
        #: Whether the transfer workaround is on (`ProcessContract.transfer_as_free`).
        self.transfer_as_free = TRANSFER_AS_FREE
        #: The process as the panel shows it: with the workaround, jointing actions in run order (`as_run`).
        self._shown: Process | None = None
        #: Every (action, movement) of the shown schedule, in order (`Process.movements`); `index` points into it.
        self._movements: tuple[tuple[Action, Movement], ...] = ()
        #: Schedule index of each movement's action, and the first movement of each action with any.
        self._action_of: tuple[int, ...] = ()
        self._first_of: dict[int, int] = {}
        #: The assumed start joints of every shown movement (`assumed_start_all`), for drawing and the panel.
        self._assumed: dict = {}
        #: Each shown movement's index in the authored `process.movements()`; and the way back, where an authored
        #: movement not shown (the workaround's travel to load) goes to its action's first shown one.
        self._authored_index: tuple[int, ...] = ()
        self._shown_index: dict[int, int] = {}

        #: Index of the selected movement in `_movements`.
        self.index = 0

        #: Show the robot at the movement's target instead of its start.
        self.at_target = False

        #: Whether the process's obstacles count in collision checks (`ProcessContract.obstacles_collide`).
        self.obstacles_collide = OBSTACLES_COLLIDE
        # Why the last "Transfer workaround" change was refused, and when (ROS time); "" after a change it made.
        self._transfer_refused, self._transfer_refused_at = "", 0.0
        # The loaded process's obstacles from Rhino, by process id (`process_obstacles`).
        self._obstacles: frozenset[str] = frozenset()

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
        #: Draw the overlay see-through; on at start (View Ghost toggles it), so the real robots show through it.
        self._ghost = True
        #: Draw every bar's pink line and name at its assembly pose (View's "Rhino bar lines" box); off at start.
        self._bar_lines = False
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
        #: Each action's mark in its row, by action id (`request_marks`); and the joints carried forward
        #: (`request_carry`). Both cleared when a process loads.
        self._marks: dict[str, str] = {}
        self._carry: Carry | None = None

    # --- --- --- --- --- WHAT OTHER PLUGINS READ --- --- --- --- ---

    @property
    def selection(self) -> Selection | None:
        """Selection | None: The selected movement (`ProcessContract`); a new object whenever it changes."""
        selected = self.selected
        if selected is None:
            return None
        action, movement = selected
        index = self._authored_index[self.index]
        last = self._selection
        # * A flip of the workaround changes the shown action, though the authored index may stay.
        if (last is None or last.process is not self.process or last.index != index or last.action is not action
                or last.movement is not movement or last.ids is not self.ids
                or last.process_ids is not self.process_ids):
            self._selection = Selection(self.process, index, action, movement, self.ids, self.process_ids)
        return self._selection

    @property
    def selected(self) -> tuple[Action, Movement] | None:
        """tuple[Action, Movement] | None: The selected movement and its action, or None before a process is loaded."""
        return self._movements[self.index] if self._movements else None

    # --- --- --- --- --- SETUP --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel at the top of the Assembly tab, the folder filled in; nothing loads before Load."""
        folder = ctx.config.process_directory
        # * First in the Assembly tab: pick the action and movement here, then plan and run them below.
        with ctx.view.ui(band=PROCESS_BAND) as gui:
            gui.add_html(section("process", SECTION_CTRL))
            self._status = gui.add_html("")
            self._folder = gui.add_text("Folder", initial_value=str(folder or ""),
                                        hint="Process folder relative to the Drive root, e.g. "
                                             "data_design_study/<process>, or absolute: with process.json "
                                             "(doc/process_format.md), or an export in the old compas_fab format, "
                                             "converted into <export>_process next to it")
            load = gui.add_button("Load", hint="Load the folder; an old export is converted first (about 15 s) "
                                               "unless its <export>_process copy is up to date")
            self._transfer = gui.add_checkbox("Transfer workaround", initial_value=TRANSFER_AS_FREE,
                                              hint="Run Cindy's transfer as a free move with empty tools to the insert "
                                                   "start; you place the bar there. Lists and draws each jointing "
                                                   "action in that order. Replan after a change.")
            self._collide = gui.add_checkbox("Rhino obstacles collide", initial_value=OBSTACLES_COLLIDE,
                                             hint="Count the process's obstacles from Rhino (e.g. the cabinet) in "
                                                  "every collision check: base and arm planners, clearances, "
                                                  "assembly planning. Off: drawn see-through only; the measured "
                                                  "obstacles (obstacles plugin) still count. Replan after a change.")
            # ! Always one line, so nothing below it moves.
            self._collide_status = gui.add_html("")
            self._actions_title = gui.add_html(section("actions", SECTION_CTRL))
            # * A window of the schedule that follows the selection: click a row to jump to that action.
            self._action_rows = [gui.add_button("·", color="gray") for _ in range(ACTION_ROWS)]
            gui.add_html(section("step", SECTION_CTRL))
            self._slider = gui.add_slider("Step", min=0, max=1, step=1, initial_value=0)
            go = gui.add_button_group("Go", ["◀ action", "◀ mv", "mv ▶", "action ▶"],
                                      hint="Previous / next movement, or jump to the previous / next action")
            show = gui.add_button_group("Show", ["Start", "Target"],
                                        hint="The robot at the movement's start state, or at its target configuration")
            view = gui.add_button_group("View", ["Hide", "Ghost", "All"],
                                        hint="Hide / show the overlay (robots, tools, held bodies, and the Rhino "
                                             "obstacles while they don't collide; other standing bodies are scene "
                                             "bodies, always drawn); ghost (on at start) makes it see-through; all "
                                             "also overlays absent bodies, see-through")
            bar_lines = gui.add_checkbox("Rhino bar lines", initial_value=False,
                                         hint="Every bar's centre line and name, pink, where the design puts it "
                                              "once built: a ruler of the Rhino poses.")
            # ! The selected movement's details change with it, so they keep a fixed height: the movement buttons
            #   and the Assembly tab's buttons are below them.
            self._which = gui.add_html("")
            # * The selected action's movements by their old codes, in their kinds' colours, the selected one ringed;
            #   hovering another shows its old name and what it does (`_movement_css`). No viser hint: it would sit
            #   over those.
            self._moves = gui.add_button_group("Movement", [NO_MOVEMENT])
            self._moves_style = gui.add_html("")
            self._details = gui.add_html("")
            self._error_text = gui.add_html("")

        # ! Route every viser callback through defer so it runs on the main thread.
        load.on_click(ctx.defer("load process", lambda: self.load(ctx, Path(self._folder.value.strip()))))
        # ! Operator clicks only: draw writes the boxes back, and viser calls back for that too, which could undo a
        #   click queued in the same tick.
        self._transfer.on_update(ctx.defer_value("Transfer workaround",
                                                 lambda value: self.set_transfer_as_free(ctx, bool(value)),
                                                 operator_only=True))
        self._collide.on_update(ctx.defer_value("Rhino obstacles collide",
                                                lambda value: self.set_obstacles_collide(ctx, bool(value)),
                                                operator_only=True))
        # ! Operator drags only: draw writes the slider back, and viser calls back for that too, late, which could
        #   select an older movement after quick clicks on ◀ mv / mv ▶.
        self._slider.on_update(ctx.defer_value("select step", lambda value: self.select(int(value)),
                                               operator_only=True))
        go.on_click(ctx.defer_value("step button", self._on_go))
        self._moves.on_click(ctx.defer_value("select movement", lambda code: self._on_movement(str(code))))
        show.on_click(ctx.defer_value("show", lambda label: self._set_at_target(label == "Target")))
        view.on_click(ctx.defer_value("view", self._on_view))
        bar_lines.on_update(ctx.defer_value("Rhino bar lines", lambda value: setattr(self, "_bar_lines", bool(value))))
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
        self._load_task = ctx.spawn(f"load {folder.name}", self._load(ctx, folder), survives_soft_stop=True)

    def request_select(self, index: int) -> None:
        """Select a movement for another plugin (`ProcessContract.request_select`): `select` its shown movement.

        Args:
            index: The movement's index into the authored `process.movements()`; clamped, ignored while no process
                is loaded. One not shown selects its action's first shown movement.
        """
        if self._movements:
            self.select(self._shown_index[max(0, min(int(index), len(self._shown_index) - 1))])

    def request_marks(self, marks: Mapping[str, str]) -> None:
        """Show each action's mark in its row (`ProcessContract.request_marks`), replacing the marks given before.

        Args:
            marks: Action id -> MARK_PENDING, MARK_DONE or MARK_ASSUMED.
        """
        self._marks = dict(marks)

    def request_carry(self, carry: Carry | None) -> None:
        """Draw an action's later movements with joints found live (`ProcessContract.request_carry`); None: as authored.

        Args:
            carry: The joints, the robot and from which movement on; or None.
        """
        self._carry = carry
        # * Found for the selected movement: show its target, the goal they are, as the old monitor's goal ghost did.
        selected = self.selected
        found_for = (carry.action_id, carry.movement_id) if carry is not None else None
        if selected is not None and found_for == (selected[0].id, selected[1].id):
            self.at_target = True
        self._stale = True

    def select(self, index: int) -> None:
        """Select a movement by index into `_movements`, clamped; ignored before a process is loaded."""
        if not self._movements:
            return
        index = max(0, min(int(index), len(self._movements) - 1))
        # * The same movement again changes nothing: the scene bodies are not put again.
        if index == self.index:
            return
        self.index = index
        self._changed()

    def set_transfer_as_free(self, ctx: PluginContext, on: bool) -> None:
        """Turn the transfer workaround on or off: the list is built again, on the same authored movement if shown.

        Refused while movement_exec has a robot's lease for a step (`step_running`): the step being run must not
        change under it; `draw` puts the box back, and a chip under the boxes says why for REFUSAL_SHOWN s.

        Args:
            ctx: This plugin's context.
            on: Whether the workaround is on.
        """
        # A repeated value (e.g. two clicks queued in one tick) changes nothing.
        if on == self.transfer_as_free:
            return
        refusal = step_running(ctx.world.robots)
        if refusal:
            self._transfer_refused, self._transfer_refused_at = refusal, ctx.now()
            ctx.log_warn(f"Transfer workaround not changed: {refusal}")
            return
        self._transfer_refused = ""
        self.transfer_as_free = on
        if self.process is not None:
            authored = self._authored_index[self.index] if self._movements else 0
            self._show()
            self.index = self._shown_index.get(authored, 0)
        self._changed()
        ctx.log_info("Transfer workaround on: jointing actions shown and run transit, place, grasp, insert" if on
                     else "Transfer workaround off: jointing actions as authored")

    def set_obstacles_collide(self, ctx: PluginContext, collide: bool) -> None:
        """Count the process's obstacles in every collision check, or leave them out; the scene bodies are put again.

        Args:
            ctx: This plugin's context.
            collide: Whether they count.
        """
        # A repeated value (e.g. two clicks queued in one tick) changes nothing: the bodies are not put again.
        if collide == self.obstacles_collide:
            return
        self.obstacles_collide = collide
        self._changed()
        ctx.log_info("Rhino obstacles now count in every collision check" if collide
                     else "Rhino obstacles are drawn only: left out of every collision check; replan")

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
        if label == "◀ mv":
            self.select(self.index - 1)
        elif label == "mv ▶":
            self.select(self.index + 1)
        elif label == "◀ action":
            # First movement of this action, or of the previous action with movements if already there.
            earlier = [first for action, first in self._first_of.items() if first < self.index and action <= current]
            self.select(max(earlier) if earlier else 0)
        elif label == "action ▶":
            later = [first for first in self._first_of.values() if first > self.index]
            if later:
                self.select(min(later))

    def _on_movement(self, code: str) -> None:
        """Select the movement of the selected action with this code (a movement button); any other does nothing.

        ? Read against the action selected when it runs: a click in the same tick as a jump to another action selects
          that action's movement with the code, if it has one.
        """
        if not self._movements:
            return
        codes = [label.code for label in self._labels(self._movements[self.index][0])]
        if code in codes:
            self.select(self._first_of[self._action_of[self.index]] + codes.index(code))

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
        """Make `process` the loaded one, with its movement list and assumed starts, and select its first movement.

        The old process's marks and carried joints go with it.
        """
        self.process, self.index = process, 0
        self._marks, self._carry = {}, None
        self._show()
        self._obstacles = process_obstacles(process)

    def _show(self) -> None:
        """Build the shown process (`as_run`), its movement list, assumed starts and the maps to authored indices."""
        process = self.process
        self._shown = shown = as_run(process, self.transfer_as_free)
        schedule_index = {action_id: index for index, action_id in enumerate(process.schedule)}
        self._movements = tuple(shown.movements())
        self._action_of = tuple(schedule_index[action.id] for action, _ in self._movements)
        self._first_of = {}
        for index, action in enumerate(self._action_of):
            self._first_of.setdefault(action, index)
        self._assumed = assumed_start_all(shown)
        authored = {(action.id, movement.id): index for index, (action, movement) in enumerate(process.movements())}
        self._authored_index = tuple(authored[(action.id, movement.id)] for action, movement in self._movements)
        listed = {index: shown_index for shown_index, index in enumerate(self._authored_index)}
        self._shown_index = {index: listed.get(index, self._first_of[schedule_index[action_id]])
                             for (action_id, _), index in authored.items()}

    async def _build(self, ctx: PluginContext) -> None:
        """Add the process's viser nodes, spread over ticks."""
        self._progress = "drawing the process"
        watch = Stopwatch()
        drawing = ProcessDrawing(ctx.view, f"{ctx.view.scene_root}/overlay", self.process, self._models)
        drawing.set_ghost(self._ghost)
        drawing.set_bar_lines(self._bar_lines)
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
            self._drawing.set_bar_lines(self._bar_lines)
            return
        if self.process is None:
            return
        if self._build_task is None or self._build_task.done():
            self._build_task = ctx.spawn("draw the process", self._build(ctx), survives_soft_stop=True)

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
            bodies = scene_bodies(self.process, movement, robots, prefix, self.obstacles_collide)
        except ValueError as refused:
            # ! A grasp the real robot can't take is a data problem: report it, put the bodies without the robots.
            self._error = f"held bodies left where planned: {refused}"
            ctx.log_error(self._error)
            robots = ()
            bodies = scene_bodies(self.process, movement, robots, prefix, self.obstacles_collide)
        ctx.scene.put_many(bodies)
        self.ids, self.process_ids = live_ids(self.process, robots, prefix)

    def draw(self, ctx: PluginContext) -> None:
        """Pose the overlay from the selected state, and fill the panel."""
        self._status.content = self._status_html()
        self._transfer.value = self.transfer_as_free
        self._collide.value = self.obstacles_collide
        self._collide_status.content = self._collide_html(ctx)
        selected = self.selected
        # * One line, even empty: the full error on hover.
        error = " ".join(self._error.split())
        self._error_text.content = block(values(_one_line(escape(error) or "&nbsp;", error)))
        self._draw_action_list()
        if selected is None:
            self._which.content, self._details.content = _empty_details()
            self._draw_movements([], 0)
            return
        action, movement = selected
        self._slider.max = max(len(self._movements) - 1, 1)
        self._slider.value = self.index
        labels = self._labels(action)
        self._which.content, self._details.content = self._details_html(action, movement, labels)
        self._draw_movements(labels, action.movements.index(movement))

        drawing = self._drawing
        if not self._stale or drawing is None:
            return
        self._stale = False
        drawing.visible = self._visible
        if not self._visible:
            return
        try:
            joints = joints_at(self._shown, action.id, movement.id, self.at_target, self._assumed)
            carry, state = self._carry, movement.start
            if self._carry_applies(action, movement):
                joints[carry.robot] = {**joints.get(carry.robot, {}), **carry.joints}
                state = _with_base(state, carry.robot, carry.base)
            # * Left out of collision checks, the Rhino obstacles are drawn by the overlay, not the runtime.
            drawing.show(state, joints, self._everything, frozenset() if self.obstacles_collide else self._obstacles)
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
            action = self._shown.actions[self.process.schedule[index]]
            robot = self.process.robots[action.robot].name if action.robot else BY_HAND_LABEL
            mark = self._marks.get(action.id)
            label = f"{index + 1}. {action.id} · {robot} · {len(action.movements)} mv"
            button.label = label + (f" · {'done (assumed)' if mark == MARK_ASSUMED else mark}" if mark else "")
            button.color = ("blue" if index == current else "orange" if mark == MARK_ASSUMED
                            else "green" if mark == MARK_DONE else "gray")
    # ! Keep every row one line tall (`_one_line`) and the chip set fixed, so nothing shifts between steps.

    def _carry_applies(self, action: Action, movement: Movement) -> bool:
        """Whether the carried joints show at this movement of `action`.

        They do at the carry's movement shown at its target, and at every later movement of the action shown at its
        start, or at its target when that names no joints for the carry's robot: a later movement's own target joints
        stay, on the base they were authored for.

        Args:
            action: The action as shown (`as_run`).
            movement: One of its movements.

        Returns:
            bool: Whether to draw the carry's robot at its joints here.
        """
        carry = self._carry
        if carry is None or carry.action_id != action.id:
            return False
        shown = [each.id for each in action.movements]
        if carry.movement_id not in shown:
            return False
        at, number = shown.index(carry.movement_id), action.movements.index(movement)
        if number == at:
            return self.at_target
        own = movement.target.joints.get(carry.robot) if movement.target is not None else None
        return number > at and not (self.at_target and own)

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
        return chip_line(chips)

    def _collide_html(self, ctx: PluginContext) -> str:
        """One line of chips under the boxes: a refused workaround change, the Rhino obstacles, an unchecked room.

        Chips: a "Transfer workaround" change refused in the last REFUSAL_SHOWN s, first so it is never scrolled out
        of sight, its reason on hover; whether the Rhino obstacles collide; a warning when nothing in the room is
        checked.
        """
        chips = ""
        if self._transfer_refused and ctx.now() - self._transfer_refused_at < REFUSAL_SHOWN:
            chips += chip("workaround refused", FAIL, f"Transfer workaround not changed: {self._transfer_refused}")
        if self.obstacles_collide:
            chips += chip("Rhino obstacles collide", OK)
        else:
            chips += chip("Rhino obstacles: drawn only", NONE, "left out of every collision check")
            if not any(body_id.startswith(MEASURED_PREFIX) for body_id in ctx.scene.snapshot.bodies):
                chips += chip("room unchecked", FAIL, "no measured obstacles in the scene: add 'obstacles' to the "
                                                      "plugins, or nothing in the room is checked")
        return chip_line(chips)

    def _labels(self, action: Action) -> list[MovementLabel]:
        """The labels of an action's movements, in order, as shown (`label_movement`).

        Args:
            action: An action of the shown process.
        """
        return [label_movement(self.process, action, each, self.transfer_as_free) for each in action.movements]

    def _draw_movements(self, labels: list[MovementLabel], current: int) -> None:
        """Label the movement buttons with the action's movements, coloured by kind, the `current`-th ringed.

        Args:
            labels: The action's movement labels, in order; none before a load.
            current: Index of the selected movement in `labels`.
        """
        self._moves.options = tuple(label.code for label in labels) or (NO_MOVEMENT,)
        kinds = [label.kinds[0] for label in labels]
        # * The selected movement's old name and sentence are in the details below: no tip for it.
        tips = [None if number == current else f"{label.old_name or 'not in the old monitor'}\n{label.text}"
                for number, label in enumerate(labels)]
        self._moves_style.content = f"<style>{_movement_css(self._moves._impl.uuid, kinds, current, tips)}</style>"

    def _details_html(self, action: Action, movement: Movement, labels: list[MovementLabel]) -> tuple[str, str]:
        """The selected movement's details, split around the movement buttons.

        Args:
            action: The selected action, as shown.
            movement: The selected movement.
            labels: The action's movement labels, in order.

        Returns:
            tuple[str, str]: Above the buttons, a row of chips; below them, a row of chips, four lines of text and a
                two-line description.
        """
        number = action.movements.index(movement)
        label = labels[number]
        which = chip(f"{self.index + 1}/{len(self._movements)}", OK)
        if action.robot:
            which += chip(escape(self.process.robots[action.robot].name), SECTION_CTRL, action.robot)
        else:
            which += chip(BY_HAND_LABEL, SECTION_CTRL, "a bar_placing action: the operator places the bar")
        # * Its kinds in the action flowchart's colours: FM, LM, CDFM, CDLM, Manual, TM (dashed: runs through).
        which += "".join(_kind_chip(kind, f"{kind.name}; Rhino: {movement.label}") for kind in label.kinds)

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
        if self._carry_applies(action, movement):
            carry = self._carry
            found = "joints and base" if carry.base is not None else "joints"
            title = f"{escape(carry.robot)} drawn at the {found} found for {escape(carry.movement_id)}'s end"
            carries += chip(f"joints from {escape(carry.source)}", BUSY, title)
        carries += chip(movement.controller or "no arm moves", NONE)
        carries += chip(f"ends on {movement.ends_on}", NONE)

        # * A drive the monitor never sends (e.g. a tighten on a ground half) says so.
        dropped = mating_drives(self.process, movement).dropped if movement.drives else {}
        moves = [arm.split("/")[-1] for arm in movement.arms] + [
            f"{tool.split('/')[-1]} grip {grip}" for tool, grip in movement.grip_change.items()] + [
            f"{tool.split('/')[-1]} {direction}{' (not sent)' if tool in dropped else ''}"
            for tool, direction in movement.drives.items()]
        about = f" · {action.label}" if action.label else ""
        lines = [f"action   {self._action_of[self.index] + 1}/{len(self.process.schedule)}  {action.id}{about}",
                 f"movement {number + 1}/{len(action.movements)}  {movement.id}",
                 f"old name {label.old_name or '- (not in the old monitor)'}",
                 f"moves    {', '.join(moves) or '-'}"]
        text = values(*(_one_line(escape(line)) for line in lines))
        return block(_one_line(which)), block(_one_line(carries) + text + _description(label.text))


def _kind_chip(kind: Kind, title: str) -> str:
    """A chip in a flowchart kind's colours, labelled with its abbreviation.

    Args:
        kind: The movement kind.
        title: Hover text.
    """
    return swatch_chip(escape(kind.short), kind.fill, kind.border, title, kind.fill_to, kind.dashed)


def _movement_css(uuid: str, kinds: list[Kind], current: int, tips: list[str | None]) -> str:
    """CSS for the movement buttons: one row of tall buttons in their kinds' colours, one ringed, the others with a tip.

    The row is `one_row_css`; the tip shows under the row while a button is hovered (`group_tooltips_css`).

    Args:
        uuid: The button group's uuid.
        kinds: Each button's movement kind, left to right; empty for the one button before a load.
        current: Index of the selected movement's button.
        tips: Each button's tooltip text, None for none.

    Returns:
        str: CSS rules, without the <style> element.
    """
    css = (one_row_css(uuid)
           + f"{group_buttons(uuid)} {{ height: 30px !important; font-size: 13px !important; font-weight: 700; }}"
           f"{group_buttons(uuid)}:hover {{ filter: brightness(0.93); }}")
    for index, kind in enumerate(kinds):
        fill = f"linear-gradient(90deg,{kind.fill},{kind.fill_to})" if kind.fill_to else kind.fill
        ring = f" box-shadow: inset 0 0 0 2px {RING};" if index == current else ""
        # * Dark text on the light fills, in either theme, as `swatch_chip`.
        css += (f"{group_buttons(uuid, index)} {{ background: {fill} !important; color: #212529 !important; "
                f"border: 1px {'dashed' if kind.dashed else 'solid'} {kind.border} !important;{ring} }}")
    return css + group_tooltips_css(uuid, tips)


def _with_base(state: State, robot: str, base: Pose | None) -> State:
    """`state` with `robot`'s base at `base`, e.g. where joints carried forward were found; as it is for None.

    Args:
        state: A process state.
        robot: The process robot id.
        base: Its base pose in the world, or None.
    """
    robot_state = state.robots.get(robot)
    if base is None or robot_state is None:
        return state
    return replace(state, robots={**state.robots, robot: replace(robot_state, base=base)})


def _description(text: str) -> str:
    """Two lines of wrapped text, cut with "…" past them so the panel never changes height; all of it on hover."""
    return (f'<div title="{escape(text, quote=True)}" style="font-size:11px;line-height:1.5;height:3em;'
            f'overflow:hidden;display:-webkit-box;-webkit-line-clamp:2;-webkit-box-orient:vertical;'
            f'margin-top:2px">{escape(text)}</div>')


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


def _one_line(html: str, title: str = "") -> str:
    """Keep content on one line, cutting anything too wide with "…"; `pre` keeps padded columns aligned.

    Args:
        html: The content.
        title: Hover text, e.g. the whole line; none if empty.
    """
    hover = f' title="{escape(title, quote=True)}"' if title else ""
    return f'<div{hover} style="white-space:pre;overflow:hidden;text-overflow:ellipsis">{html}</div>'


def _empty_details() -> tuple[str, str]:
    """The details before a load: the same rows as `_details_html`, empty, so the first Load moves nothing below."""
    def hidden(html: str) -> str:
        return f'<span style="visibility:hidden">{html}</span>'

    swatch, plain = hidden(swatch_chip("-", NONE, NONE)), hidden(chip("-", NONE))
    return (block(_one_line(chip("no process loaded", NONE) + swatch)),
            block(_one_line(plain) + values(*[_one_line("&nbsp;")] * 4) + _description("")))
