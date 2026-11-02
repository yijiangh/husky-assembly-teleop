"""The planner panel's search: by a `work` function on a thread, or a `run` coroutine awaited under a time limit."""

from __future__ import annotations

import asyncio
import threading

import numpy as np

from husky_assembly_teleop.plugins.assembly_steps.step_panel_base import PlannerPlugin, Search
from husky_assembly_teleop.shared.planner.planner_search import PlanResult
from husky_assembly_teleop.shared.planner.timed_path import TimedPath


class Panel(PlannerPlugin):
    """The planner panel with nothing of its own."""

    def forget_plan(self) -> None:
        """Nothing kept."""


PATH = TimedPath(np.array([0.0, 1.0]), np.zeros((2, 6)))


def search(work=None, run=None, time_limit: float = 5.0) -> Search:
    """A search by `work` or by `run`."""
    return Search("a200-0806", "plan", "planning", work, time_limit, lambda result: ("found", False), run=run)


def test_a_search_can_be_awaited_on_the_main_thread():
    """A `run` result makes the plan ready; a `run` that never answers times out; `work` is as before."""
    async def found():
        return PlanResult(PATH)

    async def never():
        await asyncio.get_running_loop().create_future()

    panel = Panel()
    asyncio.run(panel._run(search(run=found), threading.Event()))
    assert panel.plan_state == "ready" and panel.path is PATH and panel._message == "found"

    panel = Panel()
    abort = threading.Event()
    panel.plan_state = "planning"
    asyncio.run(panel._run(search(run=never, time_limit=0.05), abort))
    assert panel.plan_state == "none" and panel._message.startswith("timed out") and abort.is_set()

    panel = Panel()
    asyncio.run(panel._run(search(work=lambda: PlanResult(PATH)), threading.Event()))
    assert panel.plan_state == "ready" and panel.path is PATH
    panel._executor.shutdown()
