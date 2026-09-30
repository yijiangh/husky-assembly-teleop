"""
The cell plugin: loads an authored design and steps through its cell states.

! EXPERIMENTAL: a first cut. How designs load, and what planning plugins
  read from it, may still change.

Viewer and provider only: it draws one movement's robot cell state at a time,
and planning plugins that declare `requires = ("cell",)` read the selected
movement from it. It owns no planner and no PyBullet body.

! The authored state is never modified: what is drawn is a copy (`displayed_state`).
! Loading is slow (a RobotCell file is ~350 MB), so it runs on a worker thread,
  and meshes are added to viser a model per tick by a task. The worker only
  returns a result; the task hands it over on the main thread.
"""

from __future__ import annotations

import asyncio
import traceback
from html import escape
from pathlib import Path

from compas_fab.robots import RobotCell

from ...concurrency import timeout
from ...context import PluginContext
from ...plugin import HuskyPlugin, register
from ...ui_style import BUSY, FAIL, NONE, OK, SECTION_CTRL, block, chip, note, section, values
from .design import Design, Step, displayed_state, load_design
from .drawing import CellDrawing, CellMeshes, prepare_cell_meshes

#: Give up on a load after this many seconds.
LOAD_TIMEOUT = 120.0

#: Opacity of the drawn state when "Ghost" is on.
GHOST_OPACITY = 0.4


def _load_in_background(folder: Path, report) -> tuple[Design, dict[str, CellMeshes]]:
    """Load a design and prepare its meshes. Runs on the worker thread.

    Args:
        folder: Design folder.
        report: Called with a progress string.

    Returns:
        tuple[Design, dict[str, CellMeshes]]: The design, and each cell's meshes by robot id.
    """
    design = load_design(folder, report)
    meshes = {}
    for robot_id, cell in design.cells.items():
        report(f"preparing meshes of {robot_id}")
        meshes[robot_id] = prepare_cell_meshes(cell)
    return design, meshes


@register
class CellPlugin(HuskyPlugin):
    """Shows one authored robot cell state at a time and steps through the schedule."""

    name = "cell"
    experimental = True

    def __init__(self):
        """Start with nothing loaded."""
        #: The loaded design, or None. Replaced on load, never edited.
        self.design: Design | None = None

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

        self._meshes: dict[str, CellMeshes] = {}
        #: Robot id -> its cell's viser nodes, built when first needed.
        self._drawings: dict[str, CellDrawing] = {}
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

    @property
    def cell(self) -> RobotCell | None:
        """RobotCell | None: The selected step's robot cell, or None before a design is loaded."""
        step = self.step
        return None if step is None else self.design.cell_for(step)

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
                                        hint="Design folder with ActionSchedule.json, BarActions/ "
                                             "and RobotCell*.json")
            load = gui.add_button_group("Design", ["Load"])
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
        load.on_click(ctx.defer("load design", lambda: self.load(ctx, Path(self._folder.value.strip()))))
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
            action = step.action.index - (1 if step.movement_index == 0 else 0)
            self.select(self.design.first_step_of(max(action, 0)))
        elif label == "action ▶" and step.action.index + 1 < len(self.design.actions):
            self.select(self.design.first_step_of(step.action.index + 1))

    def _on_view(self, label: str) -> None:
        """Toggle hiding or ghosting the drawn state.

        Args:
            label: The button clicked.
        """
        if label == "Hide":
            self._visible = not self._visible
        else:
            self._ghost = not self._ghost
            for drawing in self._drawings.values():
                drawing.set_opacity(GHOST_OPACITY if self._ghost else None)
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
        self._error, self._progress = "", f"loading {folder}"
        try:
            # A thread cannot be interrupted: on timeout or cancel its result is dropped.
            async with timeout(LOAD_TIMEOUT):
                design, meshes = await ctx.run_in_thread(
                    _load_in_background, folder, lambda text: setattr(self, "_progress", text))
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
        for drawing in self._drawings.values():
            drawing.remove()
        self._drawings.clear()
        self.design, self._meshes, self.index = design, meshes, 0
        self._changed()
        ctx.log_info(f"loaded {folder}: {len(design.actions)} actions, {len(design.steps)} movements, "
                     f"cells for {', '.join(design.cells)}")

    async def _build(self, ctx: PluginContext, robot_id: str) -> None:
        """Add one cell's meshes to the scene, a model per tick.

        Args:
            ctx: This plugin's context.
            robot_id: Whose cell.
        """
        self._progress = f"drawing the cell of {robot_id}"
        drawing = CellDrawing(ctx.view.scene, f"{ctx.view.scene_root}/{robot_id}")
        if self._ghost:
            drawing.set_opacity(GHOST_OPACITY)
        try:
            for _ in drawing.build(self._meshes[robot_id]):
                await ctx.next_tick()
        except BaseException:
            drawing.remove()  # cancelled by a new load, or failed: leave nothing half-built
            raise
        finally:
            self._progress = ""
        self._drawings[robot_id] = drawing
        self._stale = True

    # --- --- --- --- --- TICK --- --- --- --- ---

    def update(self, ctx: PluginContext) -> None:
        """Start building the selected step's cell, if it has not been drawn yet.

        Args:
            ctx: This plugin's context.
        """
        step = self.step
        if step is None or step.action.robot_id in self._drawings:
            return
        if self._build_task is None or self._build_task.done():
            self._build_task = ctx.spawn(f"draw {step.action.robot_id}", self._build(ctx, step.action.robot_id))

    def draw(self, ctx: PluginContext) -> None:
        """Pose the drawn cell from the selected state, and fill the panel.

        Args:
            ctx: This plugin's context.
        """
        self._status.content = self._status_html()
        step = self.step
        # Error text sits at the bottom so its height moves no buttons.
        self._error_text.content = block(values(escape(self._error))) if self._error else ""
        if step is None:
            self._details.content = note("no design loaded")
            return
        self._slider.max = max(len(self.design.steps) - 1, 1)
        self._slider.value = self.index
        self._details.content = self._details_html(step)

        drawing = self._drawings.get(step.action.robot_id)
        if not self._stale or drawing is None:
            return
        self._stale = False
        # Show only the acting robot's cell; it already includes the other robots.
        for robot_id, other in self._drawings.items():
            other.visible = self._visible and robot_id == step.action.robot_id
        if not self._visible:
            return
        try:
            cell = self.design.cell_for(step)
            drawing.show(cell, displayed_state(cell, step, self.at_target))
        except Exception as failure:
            # ! A state that does not fit its cell is a data problem: report it, do not raise.
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
        elif self.design is not None:
            chips = chip(escape(self.design.folder.name), OK, str(self.design.folder))
            chips += chip(f"{len(self.design.actions)} actions · {len(self.design.steps)} movements", NONE)
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
        movement = step.movement
        action = step.action
        which = chip(f"{self.index + 1}/{len(self.design.steps)}", OK)
        which += chip(escape(action.robot), SECTION_CTRL, action.robot_id)
        which += chip(type(movement).__name__, NONE, movement.tag)

        # ! Say so when the drawn start is not authored: it is assumed from where the robot last was, or zero.
        has_start = movement.start_state.robot_configuration is not None
        has_target = movement.target_configuration is not None
        if has_start:
            carries = chip("start conf", OK)
        elif step.assumed_start is not None:
            carries = chip(f"start conf from {escape(step.assumed_from)}", BUSY,
                           "not authored: assumed from where the robot last was")
        else:
            carries = chip("no start conf: zero", FAIL)
        if not self.at_target:
            carries += chip("showing start", NONE)
        else:
            carries += chip("showing target" if has_target else "no target: start", OK if has_target else BUSY)
        carries += chip("trajectory" if movement.trajectory is not None else "no trajectory",
                        OK if movement.trajectory is not None else NONE)

        lines = [f"action   {action.index + 1}/{len(self.design.actions)}  {action.action_id}",
                 f"movement {step.movement_index + 1}/{len(action.action.movements)}  {movement.movement_id}",
                 f"control  {getattr(movement, 'controller', '')}"]
        text = values(*(_one_line(escape(line)) for line in lines))
        return block(_one_line(which) + _one_line(carries) + text)


def _one_line(html: str) -> str:
    """Keep content on one line, cutting anything too wide with "…".

    Uses `pre` so padding spaces still align the columns.

    Args:
        html: The row's content.

    Returns:
        str: HTML.
    """
    return f'<div style="white-space:pre;overflow:hidden;text-overflow:ellipsis">{html}</div>'
