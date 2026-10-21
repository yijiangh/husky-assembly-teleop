"""
PlannerService: the base class of a headless planner plugin, which plans, cancels and commits on request.

* A planner plugin subclasses it and supplies `make_search`, `stale_reason` and `execute`; it offers others its
  `PlannerContract` (`plan.py`), and its panel is a separate plugin (`planner_panel.PlannerPanel`).
* Each commit is one task that holds the robot's lease around the whole motion: `execute` sends the plan and watches
  it until it is over, so a freeze or clearance it opens covers the motion too.
! Requests run in the planner's own context (`ctx.submit`), so soft stop, teardown and its errors apply to them, and
  every future resolves: a cancelled search gives a plan in state "cancelled", a commit to a soft-stopped robot
  answers that nothing was sent.
"""

from __future__ import annotations

import asyncio
import itertools
import threading
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from typing import Callable, Generic

from async_timeout import timeout

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin
from ...robot_interface.soft_stop import MotionRefused
from .plan import Plan, R
from .planner_search import PlanningWorld, Search


class PlannerService(HuskyPlugin, Generic[R]):
    """Plans on one worker thread, keeps the latest plan, and commits it. Subclass it; do not register it."""

    #: Prefix of the worker thread's name, e.g. "base-plan".
    worker_name = "plan"
    #: Whether a commit to a robot still running an earlier commit replaces that motion, or is refused.
    replaces_running = False

    def __init__(self):
        """No plan yet; the worker thread starts with the first search."""
        self._plan: Plan[R] | None = None
        self._ids = itertools.count(1)
        #: The running search's task, and the flag that ends its worker early.
        self._task: asyncio.Task | None = None
        self._abort = threading.Event()
        # One worker: it owns the planning world, so searches run one after another.
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix=self.worker_name)
        #: The subclass's planning world. ! Worker thread only, from its first use on.
        self._world: PlanningWorld | None = None
        self._ctx: PluginContext | None = None
        #: The task of the last commit to each robot, by serial, and the ones a newer commit is replacing.
        self._executions: dict[str, asyncio.Task] = {}
        self._replaced: set[asyncio.Task] = set()

    # --- --- --- --- --- FOR THE SUBCLASS TO SUPPLY --- --- --- --- ---

    def make_search(self, ctx: PluginContext, request: R, abort: threading.Event) -> Search:
        """Build the search for `request` from the robot as it is now. Main thread.

        Args:
            ctx: This plugin's context. Take `ctx.scene.snapshot` here, not in `work`.
            request: What to plan.
            abort: The flag `work` must watch.
        """
        raise NotImplementedError

    def stale_reason(self, ctx: PluginContext, plan: Plan[R]) -> str:
        """Why a ready plan no longer fits the robot, e.g. "the arm moved since planning"; "" if it still does."""
        raise NotImplementedError

    async def execute(self, ctx: PluginContext, plan: Plan[R], answer: Callable[[bool, str], None]) -> None:
        """Send a ready plan to its robot and watch the motion until it is over; a task holding the robot's lease.

        - Call `answer(True, message)` once the robot took the plan, or `answer(False, why)` and return if it did not.
          Raising MotionRefused before answering answers that nothing was sent.
        - ! Cancelled by Stop all, the plugin stopping, or a newer commit (`replaced`): stop the motion unless replaced.
        """
        raise NotImplementedError

    def forget(self, plan: Plan[R]) -> None:
        """Drop anything kept about `plan` besides the plan itself (`Search.accepted`). Optional."""

    # --- --- --- --- --- API (PlannerContract) --- --- --- --- ---

    @property
    def plan(self) -> Plan[R] | None:
        """Plan | None: The latest plan, replaced on every change; None before the first request or after `clear`."""
        return self._plan

    def request_plan(self, request: R) -> asyncio.Future[Plan[R]]:
        """Plan for `request`, ending a search in progress; resolves once the plan is ready, failed or cancelled."""
        future = asyncio.get_running_loop().create_future()
        self._ctx.submit(f"plan {request.serial}", lambda: self._start(request, future))
        return future

    def cancel(self) -> None:
        """End a search in progress; its plan becomes "cancelled". Nothing else changes."""
        if self._task is not None and not self._task.done():
            self._abort.set()  # stop the worker now, not at the task's next step
            self._task.cancel()

    def clear(self) -> None:
        """End a search in progress and forget the plan. ! A sent plan's motion goes on: stop it on the robot."""
        self.cancel()
        if self._plan is not None:
            self.forget(self._plan)
        self._plan = None

    def commit(self, plan_id: int) -> asyncio.Future[tuple[bool, str]]:
        """Send plan `plan_id` if it is still the latest and ready; resolves with (sent, message) once sent.

        The motion is then watched until it is over, holding the robot's lease.
        """
        future = asyncio.get_running_loop().create_future()
        self._ctx.submit(f"commit plan {plan_id}", lambda: self._commit(plan_id, future))
        return future

    def replaced(self) -> bool:
        """Whether a newer commit is replacing the running `execute`: called while it is being cancelled."""
        return asyncio.current_task() in self._replaced

    # --- --- --- --- --- HOOKS --- --- --- --- ---

    def setup(self, ctx: PluginContext) -> None:
        """Keep the context requests run in. ! Subclasses call this first."""
        self._ctx = ctx

    def update(self, ctx: PluginContext) -> None:
        """Mark a ready plan stale once `stale_reason` says so. ! Subclasses call this."""
        plan = self._plan
        if plan is not None and plan.state == "ready":
            reason = self.stale_reason(ctx, plan)
            if reason:
                self._plan = replace(plan, state="stale", message=reason, warning=False)

    def teardown(self, ctx: PluginContext) -> None:
        """End any search, then close the planning world on its worker."""
        self._abort.set()  # first, so the shutdown wait is short
        if self._world is not None:
            self._executor.submit(self._world.close)
        self._executor.shutdown(wait=True)

    # --- --- --- --- --- INSIDE --- --- --- --- ---

    def _start(self, request: R, future: asyncio.Future[Plan[R]]) -> None:
        """Start a search for `request`, ending the one in progress. Main thread, as an intent."""
        self.cancel()
        # A fresh flag per search, so an old search's abort cannot end this one.
        self._abort = abort = threading.Event()
        search = self.make_search(self._ctx, request, abort)
        if self._plan is not None:
            self.forget(self._plan)
        plan = self._plan = Plan(next(self._ids), request, "planning", message=search.message)
        self._task = self._ctx.spawn(search.label, self._run(plan, search, abort, future))

    async def _run(self, plan: Plan[R], search: Search, abort: threading.Event,
                   future: asyncio.Future[Plan[R]]) -> None:
        """Run one search on the worker and settle its plan on the main thread; every way out resolves `future`."""
        try:
            async with timeout(search.time_limit):
                result = await asyncio.get_running_loop().run_in_executor(self._executor, search.work)
        except asyncio.TimeoutError:
            self._settle(future, replace(plan, state="failed", message=f"timed out after {search.time_limit:.0f}s "
                                                                       f"waiting for the planner"))
            return
        except asyncio.CancelledError:
            self._settle(future, replace(plan, state="cancelled", message="planning cancelled"))
            raise
        except Exception as error:
            # * Raised on: the task's failure counts against this plugin, and the plan says why.
            self._settle(future, replace(plan, state="failed", message=f"the planner failed: {error}"))
            raise
        finally:
            # ! Threads cannot be interrupted: always tell the worker to stop.
            abort.set()
        search.finished()
        if result.path is None:
            self._settle(future, replace(plan, state="failed", message=f"no plan: {result.reason}", hit=result.hit))
            return
        message, warning = search.accepted(result)
        self._settle(future, replace(plan, state="ready", path=result.path, message=message, warning=warning))

    def _settle(self, future: asyncio.Future[Plan[R]], plan: Plan[R]) -> None:
        """Make `plan` the latest, unless a newer request or `clear` replaced it, and resolve its future."""
        if self._plan is not None and self._plan.id == plan.id:
            self._plan = plan
        if not future.done():
            future.set_result(plan)

    def _commit(self, plan_id: int, future: asyncio.Future[tuple[bool, str]]) -> None:
        """Start executing plan `plan_id` if it is the latest and ready; otherwise answer why not."""
        plan = self._plan
        if plan is None or plan.id != plan_id:
            future.set_result((False, "nothing sent: that plan was replaced or cleared"))
            return
        if plan.state != "ready":
            future.set_result((False, f"nothing sent: the plan is {plan.state}"
                                      + (f" ({plan.message})" if plan.message else "")))
            return
        serial = plan.request.serial
        running = self._executions.get(serial)
        if running is not None and running.done():
            running = None
        if running is not None and not self.replaces_running:
            future.set_result((False, f"nothing sent: an earlier commit still moves {serial}; wait, or Hold it"))
            return
        if running is not None:
            self._replaced.add(running)
            running.cancel()
        self._executions[serial] = self._ctx.spawn(f"plan {plan.id} on {serial}", self._execute(plan, running, future))

    async def _execute(self, plan: Plan[R], replacing: asyncio.Task | None,
                       future: asyncio.Future[tuple[bool, str]]) -> None:
        """Lease the plan's robot once the commit it replaces has ended, and `execute` the plan; always answers."""
        def answer(sent: bool, message: str) -> None:
            if future.done():
                return
            if sent and self._plan is not None and self._plan.id == plan.id:
                self._plan = replace(plan, state="sent", message=message, warning=False)
            future.set_result((sent, message))

        try:
            if replacing is not None:
                await asyncio.gather(replacing, return_exceptions=True)  # it gives the lease back as it ends
            with self._ctx.lease_robots([plan.request.serial], f"commit plan {plan.id}"):
                await self.execute(self._ctx, plan, answer)
        except MotionRefused as refused:
            answer(False, f"nothing sent: {refused}")
        finally:
            self._replaced.discard(asyncio.current_task())
            answer(False, "nothing sent: the commit ended before sending")
