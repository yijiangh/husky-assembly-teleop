"""The planner panel's search lifecycle: a search on the worker ends ready, with no plan, timed out, or stopped."""

from __future__ import annotations

import asyncio
import threading
import time
from types import SimpleNamespace

import pytest

from husky_assembly_teleop.shared.planning.panel import PlannerPlugin, Search
from husky_assembly_teleop.shared.planning.search import PlanResult


def panel() -> PlannerPlugin:
    """A planner panel with a target set and no UI."""
    plugin = PlannerPlugin()
    plugin._has_target, plugin._used = True, SimpleNamespace(touch=lambda: None)
    return plugin


def search(work, time_limit: float = 2.0, finished=None) -> Search:
    """A search for robot 0804 that runs `work` on the worker."""
    return Search(serial="0804", label="plan 0804", message="planning", work=work, time_limit=time_limit,
                  accepted=lambda result: (f"found in {result.seconds:.1f} s", False),
                  finished=finished or (lambda: None))


def run(plugin: PlannerPlugin, the_search: Search, cancel_after: float | None = None,
        abort: threading.Event | None = None) -> threading.Event:
    """Run `the_search` the way Plan does, optionally cancelling it after `cancel_after` s; return its abort flag."""
    abort = abort or threading.Event()

    async def go():
        task = asyncio.get_running_loop().create_task(plugin._run(the_search, abort))
        plugin.plan_state = "planning"
        if cancel_after is not None:
            await asyncio.sleep(cancel_after)
            task.cancel()
        try:
            await task
        except (asyncio.CancelledError, RuntimeError):
            pass

    try:
        asyncio.run(go())
    finally:
        plugin._executor.shutdown(wait=True)
    return abort


def test_a_found_path_makes_the_plan_ready():
    """Ready, for the search's robot, with the accepted message; `finished` ran and the worker was told to stop."""
    plugin, done = panel(), []
    path = SimpleNamespace(duration=3.0)
    abort = run(plugin, search(lambda: PlanResult(path, seconds=1.25), finished=lambda: done.append(True)))
    assert plugin.plan_state == "ready" and plugin.path is path and plugin._path_serial == "0804"
    assert plugin._message == "found in 1.2 s" and done and abort.is_set()


def test_no_path_says_why():
    """No plan: back to "none", and the reason is shown as a failure."""
    plugin = panel()
    run(plugin, search(lambda: PlanResult(None, reason="target in collision")))
    assert plugin.plan_state == "none" and plugin.path is None
    assert plugin._message == "no plan: target in collision" and plugin._message_failed


def test_a_search_over_its_time_limit_is_abandoned_and_told_to_stop():
    """The panel gives up after `time_limit`, and the worker sees its abort flag."""
    plugin, abort, seen = panel(), threading.Event(), []

    def slow():
        seen.append(abort.wait(2.0))
        return PlanResult(None)

    started = time.monotonic()
    run(plugin, search(slow, time_limit=0.05), abort=abort)
    assert plugin.plan_state == "none" and "timed out" in plugin._message
    assert seen == [True] and time.monotonic() - started < 1.5


@pytest.mark.xfail(strict=True, reason="_run resets plan_state only on timeout or a result (review: Fix first)")
def test_a_search_that_raises_leaves_the_panel_ready_to_plan_again():
    """An exception in the worker must not leave the plan "planning" for good."""
    plugin = panel()

    def broken():
        raise RuntimeError("mirror sync failed")

    run(plugin, search(broken))
    assert plugin.plan_state == "none"


@pytest.mark.xfail(strict=True, reason="_run resets plan_state only on timeout or a result (review: Fix first)")
def test_a_search_cancelled_by_soft_stop_leaves_the_panel_ready_to_plan_again():
    """Soft stop cancels the task; the next Plan must not say "already planning"."""
    plugin = panel()
    run(plugin, search(lambda: time.sleep(0.2) or PlanResult(None)), cancel_after=0.02)
    assert plugin.plan_state == "none"


@pytest.mark.safety
@pytest.mark.parametrize("state", ["none", "planning", "stale", "sent"])
def test_only_a_ready_plan_can_be_committed(state):
    """Commit sends nothing unless the plan is ready, and says what state it is in."""
    plugin, sent = panel(), []
    plugin.plan_state, plugin.send_plan = state, lambda ctx: sent.append(True) or (True, "sent")
    plugin._commit(None)
    assert not sent and plugin._message == f"nothing to commit: the plan is {state}" and plugin._message_failed
    plugin._executor.shutdown()


@pytest.mark.safety
@pytest.mark.parametrize("accepted, state", [(True, "sent"), (False, "ready")])
def test_commit_marks_the_plan_sent_only_when_it_went_out(accepted, state):
    """A refused send keeps the plan ready and shows the refusal as a failure."""
    plugin = panel()
    plugin.plan_state, plugin.send_plan = "ready", lambda ctx: (accepted, "the answer")
    plugin._commit(None)
    assert plugin.plan_state == state and plugin._message == "the answer" and plugin._message_failed is not accepted
    plugin._executor.shutdown()
