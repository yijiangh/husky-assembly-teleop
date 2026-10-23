"""
Plan process movements for other plugins: a request in (`MotionPlanRequest`), an asyncio future out, one at a time.

Other plugins declare `requires = ("motion_planner",)` and type it as `MotionPlannerContract`
(`shared/motion_planner_contract.py`). Every request runs on one worker thread (`work.py`), which owns a compas_fab
mirror per robot (`MirrorPool`, made on its first request: loading the models takes up to a minute). The panel shows
what runs and the last answer, cancels everything, and opens PyBullet's own window on a robot's planning world.

! EXPERIMENTAL: requests and answers may still change.
! The worker is the only thread that touches the mirrors, the PyBullet window's included.
"""

from __future__ import annotations

import asyncio
import threading
from concurrent.futures import Future, ThreadPoolExecutor
from html import escape
from typing import Any, Callable

import viser
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.scene import Scene

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin, register
from ...shared.assembly_planning.mirrors import MirrorPool
from ...shared.assembly_planning.route_result import GoalCheck, RouteResult
from ...shared.motion_planner_contract import CollisionReport, MotionPlanRequest
from ...shared.planner.planner_panel import PLAN_TAB
from ...shared.planner.planner_search import with_guesses
from ...ui.pybullet_window import add_pybullet_window_toggle
from ...ui.style import BUSY, FAIL, NONE, OK, block, chip, values
from .work import run_collisions, run_ik, run_plan

#: The work a request runs on the worker: (pool, request, abort, log) -> its answer.
Work = Callable[[MirrorPool, MotionPlanRequest, threading.Event, Callable[[str], None]], Any]


def describe(answer: RouteResult | GoalCheck | CollisionReport) -> tuple[str, bool]:
    """One line about an answer, and whether it failed.

    Args:
        answer: What a request gave.

    Returns:
        tuple[str, bool]: The line, and True for a failure.
    """
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


@register
class MotionPlannerPlugin(HuskyPlugin):
    """Plans, solves goals and checks collisions for other plugins on one worker thread."""

    name = "motion_planner"
    experimental = True
    tab_path = (PLAN_TAB, "Motion")
    icon = viser.Icon.ROUTE

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
        # Gets the planners' lines; the ROS logger is safe to call from the worker.
        self._log: Callable[[str], None] = print
        # The robot whose world the PyBullet window shows when it opens.
        self._serial: str | None = None

    # --- --- --- --- --- WHAT OTHER PLUGINS CALL (MotionPlannerContract) --- --- --- --- ---

    @property
    def busy(self) -> bool:
        """bool: Whether a request is running or queued."""
        return any(not job.done() for job, _abort, _label in self._jobs)

    @property
    def running(self) -> str:
        """str: The label of the request running now, or ""."""
        return self._running

    def request_plan(self, request: MotionPlanRequest) -> asyncio.Future[RouteResult]:
        """Plan the movement from the snapshot's start (`plan_movement`). Main thread."""
        return self._submit(run_plan, request, "plan")

    def request_ik(self, request: MotionPlanRequest) -> asyncio.Future[GoalCheck]:
        """Solve the movement's goal by IK at the live base (`check_goal`). Main thread."""
        return self._submit(run_ik, request, "goal of")

    def request_collisions(self, request: MotionPlanRequest) -> asyncio.Future[CollisionReport]:
        """Check what the acting robot collides with in the movement's planning scene. Main thread."""
        return self._submit(run_collisions, request, "collisions of")

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
        """Run one request on the worker, its label shown while it runs."""
        self._running = label
        try:
            return work(self._pool, request, abort, self._log)
        finally:
            self._running = ""
            self._worlds = self._pool.serials

    def _finished(self, label: str, done: asyncio.Future) -> None:
        """Log how a request ended and keep its line for the panel. Main thread."""
        error = done.exception()
        if error is not None:
            line, failed = f"raised {error!r}", True  # ? a bug: the work answers its failures
        else:
            line, failed = describe(done.result())
        self._last, self._last_failed = f"{label}: {line}", failed
        self._log(self._last)

    # --- --- --- --- --- HOOKS --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Build the panel: status, Cancel, and the PyBullet window on a chosen robot's world."""
        self._log = self._pool.log = ctx.log_info
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
            # ! Keep changing content BELOW the buttons, so nothing moves under the cursor.
            self._details = gui.add_html("")
        cancel.on_click(ctx.defer("cancel", self.request_cancel))

    def _window(self, gui: bool, snapshot: Scene) -> None:
        """Open or close the PyBullet window on the chosen robot's world, guessed where not measured. Worker thread."""
        try:
            self._pool.set_gui(gui, with_guesses(snapshot, robot_id(self._serial))[0], self._serial)
        finally:
            self._worlds = self._pool.serials  # ? opening the window can load a robot's world

    def update(self, ctx: PluginContext) -> None:
        """Forget finished requests."""
        self._jobs = [entry for entry in self._jobs if not entry[0].done()]

    def draw(self, ctx: PluginContext) -> None:
        """Show what runs, how many wait, the robots with a loaded world, and the last answer."""
        running = self._running
        queued = sum(not job.done() for job, _abort, _label in self._jobs) - (1 if running else 0)
        chips = chip(escape(f"planning {running}"), BUSY) if running else chip("idle", NONE)
        if queued > 0:
            chips += chip(f"{queued} queued", BUSY)
        chips += chip(f"worlds: {escape(', '.join(self._worlds)) or 'none'}", OK if self._worlds else NONE,
                      "robots whose planning world is loaded; the first request for a robot loads its models")
        self._status.content = block(chips)
        color = FAIL if self._last_failed else NONE
        self._details.content = block(f'<div title="{escape(self._last, quote=True)}" style="color:{color}">'
                                      + values(escape(self._last) or "no answer yet") + "</div>")

    def teardown(self, ctx: PluginContext) -> None:
        """Stop every request, then close the mirrors on the worker once it is free."""
        self.request_cancel()  # first, so the wait is short
        self._executor.submit(self._pool.close)
        self._executor.shutdown(wait=True)
