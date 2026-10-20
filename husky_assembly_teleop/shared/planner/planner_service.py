"""
PlannerService: the base class of a headless planner plugin, which plans, cancels and commits on request.

* A planner plugin subclasses it and supplies `make_search`, `stale_reason` and `send`; it offers others its
  `PlannerContract` (`plan.py`), and its panel is a separate plugin (`planner_panel.PlannerPanel`).
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
from typing import Generic

from async_timeout import timeout

from ...plugin_api.context import PluginContext
from ...plugin_api.plugin import HuskyPlugin
from ...robot_interface.soft_stop import RobotStopped
from .plan import Plan, R
from .planner_search import PlanningWorld, Search


class PlannerService(HuskyPlugin, Generic[R]):
    """Plans on one worker thread, keeps the latest plan, and commits it. Subclass it; do not register it."""

    #: Prefix of the worker thread's name, e.g. "base-plan".
    worker_name = "plan"

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

    def send(self, ctx: PluginContext, plan: Plan[R]) -> tuple[bool, str]:
        """Send a ready plan to the robot. Returns whether it was taken, and a message."""
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
        """Send plan `plan_id` to the robot if it is still the latest and ready; resolves with (sent, message)."""
        future = asyncio.get_running_loop().create_future()
        self._ctx.submit(f"commit plan {plan_id}", lambda: future.set_result(self._commit(plan_id)))
        return future

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
            self._settle(future, replace(plan, state="failed", message=f"no plan: {result.reason}"))
            return
        message, warning = search.accepted(result)
        self._settle(future, replace(plan, state="ready", path=result.path, message=message, warning=warning))

    def _settle(self, future: asyncio.Future[Plan[R]], plan: Plan[R]) -> None:
        """Make `plan` the latest, unless a newer request or `clear` replaced it, and resolve its future."""
        if self._plan is not None and self._plan.id == plan.id:
            self._plan = plan
        if not future.done():
            future.set_result(plan)

    def _commit(self, plan_id: int) -> tuple[bool, str]:
        """Send plan `plan_id` if it is the latest and ready."""
        plan = self._plan
        if plan is None or plan.id != plan_id:
            return False, "nothing sent: that plan was replaced or cleared"
        if plan.state != "ready":
            return False, f"nothing sent: the plan is {plan.state}" + (f" ({plan.message})" if plan.message else "")
        try:
            sent, message = self.send(self._ctx, plan)
        except RobotStopped as stopped:
            return False, f"nothing sent: {stopped}"
        if sent:
            self._plan = replace(plan, state="sent", message=message, warning=False)
        return sent, message
