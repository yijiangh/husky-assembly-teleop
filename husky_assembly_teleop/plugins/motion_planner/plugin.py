"""
Plan process movements for other plugins: a request in (`MotionPlanRequest`), an asyncio future out, one at a time.

Other plugins declare `requires = ("motion_planner",)` and type it as `MotionPlannerContract`
(`shared/motion_planner_contract.py`). Every request runs on one worker thread (`work.py`), which owns a compas_fab
mirror per robot (`MirrorPool`, made on its first request: loading the models takes up to a minute). The panel shows
what runs and the last answer, cancels everything, and opens PyBullet's own window on a robot's planning world; the
collisions panel shows what the last answer collided with. What the planners print on the worker (tamp's `[ssik]`
lines, PyBullet's) is logged under source "motion_planner/tamp". While a free search runs (tamp's dual-arm one or the
one-arm one), its two trees are drawn as the arms' tool0 traces, with the gap between their closest nodes and, for
tamp's, the configurations checked since the iteration began, every TREE_PERIOD s (`SearchTree`); a legend explains.

! EXPERIMENTAL: requests and answers may still change.
! The worker is the only thread that touches the mirrors, the PyBullet window's included.
"""

from __future__ import annotations

import asyncio
import sys
import threading
import time
from concurrent.futures import Future, ThreadPoolExecutor
from html import escape
from typing import TYPE_CHECKING, Any, Callable

import numpy as np
import viser
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.scene import Scene

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...shared.assembly_planning.mirrors import MirrorPool
from ...shared.assembly_planning.route_result import GoalCheck, RouteResult
from ...shared.assembly_planning.search_tree import SearchTree, TreeView
from ...shared.assembly_planning.servo_route import ServoStep
from ...shared.assembly_planning.stdout_lines import captured_lines
from ...shared.motion_planner_contract import CollisionReport, MotionPlanRequest
from ...shared.planner.planner_search import with_guesses
from ...ui.pybullet_window import add_pybullet_window_toggle
from ...ui.style import BUSY, FAIL, NONE, OK, block, chip, chip_line, values
from ...ui.visualization import PLAN_TAB
from .work import run_collisions, run_ik, run_plan, run_servo_step

if TYPE_CHECKING:
    from ...shared.collisions_contract import CollisionsContract
    from ...world.collision_hit import CollisionHit

#: The work a request runs on the worker: (pool, request, abort, log) -> its answer.
Work = Callable[[MirrorPool, MotionPlanRequest, threading.Event, Callable[[str], None]], Any]
#: Seconds between redraws of a free search's trees.
TREE_PERIOD = 2.0
#: Colours of the tree grown from the start, the one grown from the goal, and the gap between them, RGB 0-255.
TREE_COLORS = ((0, 170, 220), (210, 70, 190))
GAP_COLOR = (250, 190, 0)
#: Thickness of the trees' lines and of the gap's, pixels on screen.
TREE_LINE, GAP_LINE = 1.5, 4.0
#: Colours of a checked configuration's tool0s, clear and collided, RGB 0-255, and their size, metres.
CHECK_COLOR, HIT_COLOR = (90, 90, 90), (230, 40, 40)
CHECK_SIZE = 0.012


def describe(answer: RouteResult | GoalCheck | CollisionReport | ServoStep) -> tuple[str, bool]:
    """One line about an answer, and whether it failed.

    Args:
        answer: What a request gave.

    Returns:
        tuple[str, bool]: The line, and True for a failure.
    """
    if isinstance(answer, ServoStep):
        correction = ", ".join(f"{flange.rsplit('/', 1)[-1]} {metres * 1e3:.2f} mm"
                               for flange, metres in answer.correction.items())
        if answer.nothing_left:
            return f"servo: nothing left to plan ({correction})", False
        line, failed = describe(answer.route)
        return f"servo {line}" + (f"; correction {correction}" if correction else ""), failed
    if isinstance(answer, RouteResult):
        routine = answer.routine.value if answer.routine is not None else "unclassified"
        if answer.ok:
            return (f"{routine} solved by {answer.planner}: {answer.waypoints} waypoints, {answer.attempts} tries, "
                    f"{answer.seconds:.1f} s"), False
        return f"{routine} {answer.result.status}: {answer.result.reason}", answer.result.status == "failed"
    if isinstance(answer, GoalCheck):
        if not answer.ok:
            return f"no goal: {answer.reason}", True
        gap = answer.max_from_authored
        return (f"goal found ({answer.ik.backend}, rank {answer.ik.rank} of {answer.ik.combinations}), "
                f"max |goal - authored| {'-' if gap is None else f'{gap:.4f}'} rad, {answer.seconds:.1f} s"), False
    if answer.reason:
        return f"no collision check: {answer.reason}", True
    pairs = "; ".join(f"{a} with {b}" for a, b in answer.pairs)
    return (f"collides: {pairs}" if pairs else "clear"), bool(pairs)


def hits_of(label: str, answer: RouteResult | GoalCheck | CollisionReport | ServoStep) -> dict[str, CollisionHit]:
    """An answer's collision hits by label for the collisions panel, e.g. "plan B20_J_transfer: goal"."""
    if isinstance(answer, ServoStep):
        answer = answer.route
    found = answer.hits if isinstance(answer, (RouteResult, GoalCheck)) else (answer.hit,) if answer.hit else ()
    hits = {}
    for hit in found:
        name = f"{label}: {hit.where}"
        hits[name if name not in hits else f"{name} ({len(hits) + 1})"] = hit
    return hits


def draw_trees(ctx: PluginContext, view: TreeView) -> list:
    """Draw a free search's trees as each moving arm's tool0 traces, and the gap between their closest nodes.

    Returns:
        list: The scene nodes made, to remove before the next drawing.
    """
    scene, root = ctx.view.scene, f"{ctx.view.scene_root}/tree"
    points, parents, nodes = view.points, view.parents, []
    for tree, color in enumerate(TREE_COLORS):
        children = np.flatnonzero((view.trees == tree) & (parents >= 0))
        if len(children):
            # * (nodes, 2, arms, 3) -> one segment per arm and edge.
            segments = np.stack([points[parents[children]], points[children]], axis=1).transpose(0, 2, 1, 3)
            nodes.append(scene.add_line_segments(f"{root}/{tree}", segments.reshape(-1, 2, 3), color,
                                                 thickness=TREE_LINE, thickness_units="screen"))
    gap = view.gap
    if gap is not None:
        distance, start, goal = gap
        nodes.append(scene.add_line_segments(f"{root}/gap", np.stack([points[start], points[goal]], axis=1),
                                             GAP_COLOR, thickness=GAP_LINE, thickness_units="screen"))
        nodes.append(scene.add_label(f"{root}/gap_label", f"trees {distance:.2f} rad apart",
                                     position=tuple((points[start][0] + points[goal][0]) / 2)))
    return nodes


def tree_legend() -> str:
    """The Search tree legend: what the drawing and the progress line show."""
    def mark(color: tuple[int, int, int], dot: bool = False, thick: bool = False) -> str:
        size = "width:8px;height:8px;border-radius:50%" if dot else f"width:16px;height:{4 if thick else 2}px"
        return (f'<span style="display:inline-block;{size};background:rgb{color};vertical-align:middle;'
                f'margin-right:5px"></span>')

    def row(head: str, text: str) -> str:
        return f'<div style="margin:1px 0">{head}{text}</div>'

    def term(name: str, text: str) -> str:
        return row(f'<b style="font-family:monospace">{name}</b> ', text)

    return block(
        '<div style="font-size:11px;line-height:1.4">'
        f'<div style="color:{NONE}">Each node is drawn at every moving arm\'s tool0, every {TREE_PERIOD:.0f} s.</div>'
        + row(mark(TREE_COLORS[0]), "tree grown from the start")
        + row(mark(TREE_COLORS[1]), "tree grown from the goal")
        + row(mark(GAP_COLOR, thick=True), "the two trees' closest nodes, labelled with their distance")
        + row(mark(CHECK_COLOR, dot=True) + mark(HIT_COLOR, dot=True),
              "configurations checked in the iteration under way: clear, collided (tamp's dual-arm search only)")
        + '<div style="margin-top:4px">The line, e.g. <span style="font-family:monospace">try 1/3 · run 2/3 · it '
          '31/50 · gap 0.84 rad · 812+640 nodes · 22 checks</span></div>'
        + term("try", "a new search, up to Tries in Advanced planning")
        + term("run", "birrt's own restart within a try: both trees start again")
        + term("it", "the run's iteration: one random sample, both trees grown towards it")
        + term("gap", "joint distance between the closest nodes, all moving joints; a path is found when the trees "
                      "meet")
        + term("nodes", "start tree + goal tree; the bigger one shows only as far as the smaller one's size")
        + term("checks", "configurations checked so far in this iteration")
        + f'<div style="color:{NONE};margin-top:4px">A trace flattens 6 or 12 joints: lines close in 3D can be far '
          'apart in joints. A found path takes the drawing away; after a failure it stays until the next search.'
          '</div></div>')


def draw_checks(ctx: PluginContext, points: np.ndarray, collided: np.ndarray) -> list:
    """Draw the configurations checked since the iteration began as dots at each moving arm's tool0, red if collided.

    Returns:
        list: The scene nodes made, to remove before the next drawing.
    """
    if not len(points):
        return []
    colors = np.where(np.repeat(collided, points.shape[1])[:, None], HIT_COLOR, CHECK_COLOR).astype(np.uint8)
    return [ctx.view.scene.add_point_cloud(f"{ctx.view.scene_root}/tree/checks", points.reshape(-1, 3), colors,
                                           point_size=CHECK_SIZE, point_shape="circle")]


@register
class MotionPlannerPlugin(HuskyPlugin):
    """Plans, solves goals and checks collisions for other plugins on one worker thread."""

    name = "motion_planner"
    experimental = True
    tab_path = (PLAN_TAB, "Motion")
    icon = viser.Icon.ROUTE
    requires = ("collisions",)

    def __init__(self):
        """Start idle, with no mirror made."""
        # ! One worker, the only thread touching the pool: requests run one after another.
        self._executor = ThreadPoolExecutor(1, "motion-plan")
        self._pool = MirrorPool()
        # Every request not yet pruned: its job on the worker, the flag that stops it, its label.
        self._jobs: list[tuple[Future, threading.Event, str]] = []
        # Written by the worker, read by draw: plain assignments, so no lock.
        self._running = ""
        self._worlds: tuple[str, ...] = ()
        # The last answer's line, and whether it failed.
        self._last, self._last_failed = "", False
        # The last answer's hits by label, and the ones the collisions panel shows.
        self._hits: dict[str, CollisionHit] = {}
        self._reported: dict[str, CollisionHit] | None = None
        # Gets the planners' lines, and what they print (tamp's, PyBullet's) under source "motion_planner/tamp"; the
        # ROS logger is safe to call from the worker. ! Neither may print: the worker's stdout is captured.
        self._log: Callable[[str], None] = print
        self._log_tamp: Callable[[str], None] = lambda line: print(line, file=sys.stderr)
        # The robot whose world the PyBullet window shows when it opens.
        self._serial: str | None = None
        # The free search's trees, written by the worker; what draw last drew of them, when, and the progress line.
        self._tree = SearchTree(lambda line: self._log(line))
        self._tree_drawn: tuple[TreeView | None, bool] = (None, True)
        self._tree_at = 0.0
        self._tree_nodes: list = []
        self._check_nodes: list = []
        self._progress = ""

    # --- --- --- --- --- WHAT OTHER PLUGINS CALL (MotionPlannerContract) --- --- --- --- ---

    @property
    def busy(self) -> bool:
        """bool: Whether a request is running or queued."""
        return any(not job.done() for job, _abort, _label in self._jobs)

    @property
    def running(self) -> str:
        """str: The label of the request running now, or ""."""
        return self._running

    @property
    def progress(self) -> str:
        """str: The running free search's progress as of the last redraw of its trees; "" when none runs."""
        return self._progress if self._running else ""

    def request_plan(self, request: MotionPlanRequest) -> asyncio.Future[RouteResult]:
        """Plan the movement from the snapshot's start (`plan_movement`). Main thread."""
        return self._submit(run_plan, request, "plan")

    def request_ik(self, request: MotionPlanRequest) -> asyncio.Future[GoalCheck]:
        """Solve the movement's goal by IK at the live base (`check_goal`). Main thread."""
        return self._submit(run_ik, request, "goal of")

    def request_collisions(self, request: MotionPlanRequest) -> asyncio.Future[CollisionReport]:
        """Check what the acting robot collides with in the movement's planning scene. Main thread."""
        return self._submit(run_collisions, request, "collisions of")

    def request_servo_step(self, request: MotionPlanRequest) -> asyncio.Future[ServoStep]:
        """Plan one servo pass, `request.servo`, from the snapshot's live joints (`plan_servo_step`). Main thread."""
        return self._submit(run_servo_step, request, "servo step")

    def request_cancel(self) -> None:
        """Stop every request: queued ones answer "cancelled" unrun, the running one stops at its next try."""
        for _job, abort, _label in self._jobs:
            abort.set()

    # --- --- --- --- --- QUEUE --- --- --- --- ---

    def _submit(self, work: Work, request: MotionPlanRequest, what: str) -> asyncio.Future:
        """Queue a request on the worker; cancelling its future (a caller's timeout) stops it.

        Args:
            work: What runs on the worker.
            request: The request.
            what: Names the work in the default label, e.g. "plan".

        Returns:
            asyncio.Future: Its answer.
        """
        label = request.label or f"{what} {request.step.label}"
        abort = threading.Event()
        job = self._executor.submit(self._work, work, request, abort, label)
        self._jobs.append((job, abort, label))
        answer = asyncio.wrap_future(job)
        answer.add_done_callback(lambda done: abort.set() if done.cancelled() else self._finished(label, done))
        return answer

    def _work(self, work: Work, request: MotionPlanRequest, abort: threading.Event, label: str) -> Any:
        """Run one request on the worker, its label shown while it runs; what the planners print goes to `_log_tamp`.

        The whole request runs with stdout captured (`captured_lines`), the first request's model loading included.
        """
        self._running = label
        try:
            with captured_lines(self._log_tamp):
                return work(self._pool, request, abort, self._log)
        finally:
            self._running = ""
            self._worlds = self._pool.serials

    def _finished(self, label: str, done: asyncio.Future) -> None:
        """Log how a request ended; keep its line for the panel and its hits for the collisions panel. Main thread."""
        error, hits = done.exception(), {}
        if error is not None:
            line, failed = f"raised {error!r}", True  # ? a bug: the work answers its failures
        else:
            (line, failed), hits = describe(done.result()), hits_of(label, done.result())
        self._last, self._last_failed, self._hits = f"{label}: {line}", failed, hits
        self._log(self._last)

    # --- --- --- --- --- HOOKS --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel: status, Cancel, and the PyBullet window on a chosen robot's world."""
        self._log = self._pool.log = ctx.log_info
        self._pool.tree = self._tree
        self._log_tamp = lambda line: ctx.log_info(line, "tamp")
        serials = [config.serial for config in ctx.config.robots if config.arms]
        self._serial = serials[0] if serials else None
        with ctx.view.ui() as gui:
            self._status = gui.add_html("")
            cancel = gui.add_button("Cancel", hint="Cancel every request: queued ones answer 'cancelled' without "
                                                   "running, the running one stops at its next try")
            if serials:
                robot = gui.add_dropdown("Window", options=serials, initial_value=self._serial,
                                         hint="The robot whose planning world the PyBullet window shows when it "
                                              "opens; then it follows the robot being planned for")
                # * Guessed like the arm planner's, so an untracked robot still shows.
                add_pybullet_window_toggle(ctx, gui, self._executor, self._window)
                robot.on_update(ctx.defer_value("window robot", lambda serial: setattr(self, "_serial", str(serial))))
            self._show_tree = gui.add_checkbox("Search tree", True,
                                               hint="While a free search runs, draw its two trees as the arms' tool0 "
                                                    "traces every 2 s; the legend below says what each mark means")
            # ! Keep changing content BELOW the buttons, so nothing moves under the cursor.
            self._details = gui.add_html("")
            self._search = gui.add_html("")
            # * Last, so opening it moves nothing.
            with gui.add_folder("Search tree legend", expand_by_default=False):
                gui.add_html(tree_legend())
        cancel.on_click(ctx.defer("cancel", self.request_cancel))

    def _window(self, gui: bool, snapshot: Scene) -> None:
        """Open or close the PyBullet window on the chosen robot's world, guessed where not measured. Worker thread."""
        try:
            self._pool.set_gui(gui, with_guesses(snapshot, robot_id(self._serial))[0], self._serial)
        finally:
            self._worlds = self._pool.serials  # ? opening the window can load a robot's world

    def update(self, ctx: PluginContext) -> None:
        """Forget finished requests; show the last answer's collisions, the first ticked, or take older ones away."""
        self._jobs = [entry for entry in self._jobs if not entry[0].done()]
        hits = self._hits
        if hits is not self._reported:
            self._reported = hits
            collisions: CollisionsContract = ctx.require("collisions")
            if hits:
                collisions.replace(self.name, hits, shown=list(hits)[:1])
            else:
                collisions.clear(self.name)

    def draw(self, ctx: PluginContext) -> None:
        """Show what runs, how many wait, the robots with a loaded world, and the last answer."""
        running = self._running
        queued = sum(not job.done() for job, _abort, _label in self._jobs) - (1 if running else 0)
        chips = chip(escape(f"planning {running}"), BUSY) if running else chip("idle", NONE)
        if queued > 0:
            chips += chip(f"{queued} queued", BUSY)
        chips += chip(f"worlds: {escape(', '.join(self._worlds)) or 'none'}", OK if self._worlds else NONE,
                      "robots whose planning world is loaded; the first request for a robot loads its models")
        self._status.content = chip_line(chips)
        color = FAIL if self._last_failed else NONE
        self._details.content = block(f'<div title="{escape(self._last, quote=True)}" style="color:{color}">'
                                      + values(escape(self._last) or "no answer yet") + "</div>")
        self._draw_tree(ctx)
        self._search.content = block(values(escape(self._progress) or "no free search yet"))

    def _draw_tree(self, ctx: PluginContext) -> None:
        """Redraw the free search's checks, trees and progress line, every TREE_PERIOD s while it runs."""
        view, show = self._tree.view, bool(self._show_tree.value)
        drawn, was_shown = self._tree_drawn
        if view is drawn and show == was_shown and (view is None or view.over):
            return
        # * A new search, its end, or the box changed shows at once; new checks and iterations wait for the period.
        now = time.monotonic()
        at_once = show != was_shown or view is None or drawn is None or view.label != drawn.label or view.over
        if not at_once and now - self._tree_at < TREE_PERIOD:
            return
        self._tree_at = now
        points, collided = self._tree.probes()
        checks = f" · {len(collided)} checks" if len(collided) else ""
        self._progress = "" if view is None else view.line() + checks
        for node in self._check_nodes:
            node.remove()
        self._check_nodes = draw_checks(ctx, points, collided) if show and view is not None and not view.over else []
        if view is drawn and show == was_shown:
            return
        self._tree_drawn = (view, show)
        for node in self._tree_nodes:
            node.remove()
        self._tree_nodes = draw_trees(ctx, view) if show and view is not None and not view.found else []

    def teardown(self, ctx: PluginContext) -> None:
        """Stop every request, then close the mirrors on the worker once it is free."""
        self.request_cancel()  # first, so the wait is short
        self._executor.submit(self._pool.close)
        self._executor.shutdown(wait=True)
