"""The motion_planner plugin: requests run in order on one worker, answer as futures, stop when cancelled or timed out.

The slow test asks the real service about B20's transfer on the purged process.
"""

from __future__ import annotations

import asyncio
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
from async_timeout import timeout
from planning_fixtures import ALICE, BELLE, CINDY, DATA, design_scene, purged_process, robot_config, step
from process_fixtures import build_process

from husky_assembly_teleop.plugins.motion_planner import plugin as motion
from husky_assembly_teleop.plugins.motion_planner.plugin import MotionPlannerPlugin
from husky_assembly_teleop.shared.assembly_planning.ik import ssik_unavailable
from husky_assembly_teleop.shared.assembly_planning.movement import Routine
from husky_assembly_teleop.shared.assembly_planning.route_result import RouteResult
from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.shared.assembly_planning.process_steps import loaded_process
from husky_assembly_teleop.shared.motion_planner_contract import MotionPlanRequest
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.process import write
from scaffolding_assembly_core.process.solutions import MovementResult
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

SOLVED = RouteResult(MovementResult("solved"), Routine.FREE_HOME, planner="fake")


class Pool:
    """A mirror pool that fails to make a mirror, counting what it is asked."""

    serials = ()

    def __init__(self):
        """Nothing asked yet."""
        self.asked, self.closed = [], 0

    def mirror(self, serial):
        """Count the request, then fail."""
        self.asked.append(serial)
        raise RuntimeError("no mirror here")

    def close(self):
        """Count the close."""
        self.closed += 1


def planner(pool: Pool | None = None) -> MotionPlannerPlugin:
    """A plugin not set up, quiet, with a fake pool."""
    plugin = MotionPlannerPlugin()
    plugin._log, plugin._pool = (lambda _line: None), pool or Pool()
    return plugin


def request(label: str) -> MotionPlanRequest:
    """A request that only names its movement; the real work refuses it before reading more."""
    return MotionPlanRequest(None, SimpleNamespace(label=label, movement=SimpleNamespace(id=label)), Scene(), (),
                             label=label)


async def idle(plugin: MotionPlannerPlugin) -> None:
    """Wait until the worker has finished everything queued so far."""
    await asyncio.wrap_future(plugin._executor.submit(lambda: None))


def test_requests_run_in_order_one_at_a_time(monkeypatch):
    """Requests run in the order sent; the planner is busy until the last answer."""
    order = []

    def work(_pool, request, _abort, _log):
        order.append(request.label)
        return SOLVED

    monkeypatch.setattr(motion, "run_plan", work)

    async def scenario():
        plugin = planner()
        answers = [plugin.request_plan(request(f"plan {index}")) for index in range(3)]
        assert plugin.busy
        results = await asyncio.gather(*answers)
        await asyncio.sleep(0)
        assert not plugin.busy and plugin.running == ""
        assert plugin._last == "plan 2: free_home solved by fake: 0 waypoints, 0 tries, 0.0 s"
        plugin.teardown(None)
        return results

    assert asyncio.run(scenario()) == [SOLVED] * 3 and order == ["plan 0", "plan 1", "plan 2"]


def test_cancel_answers_queued_requests_unrun(monkeypatch):
    """Cancel stops the running request; the queued ones answer "cancelled" without touching a mirror."""
    started = threading.Event()

    def blocking(_pool, _request, abort, _log):
        started.set()
        stopped = abort.wait(5.0)
        return RouteResult(MovementResult("failed", "stopped by its abort" if stopped else "ran out its time"), None)

    monkeypatch.setattr(motion, "run_plan", blocking)

    async def scenario():
        pool = Pool()
        plugin = planner(pool)
        first = plugin.request_plan(request("first"))
        queued = [plugin.request_ik(request("ik")), plugin.request_collisions(request("collisions"))]
        await asyncio.get_running_loop().run_in_executor(None, started.wait, 5.0)
        assert plugin.running == "first"
        plugin._status, plugin._details = SimpleNamespace(content=""), SimpleNamespace(content="")
        plugin.draw(None)
        assert plugin._status.content.count('class="chip-line"') == 1 and "2 queued" in plugin._status.content
        plugin.request_cancel()
        answers = await asyncio.gather(first, *queued)
        plugin.teardown(None)
        return pool, answers

    pool, (first, ik, collisions) = asyncio.run(scenario())
    assert first.result.reason == "stopped by its abort" and ik.reason == collisions.reason == "cancelled"
    assert pool.asked == []


def test_a_callers_timeout_stops_the_search(monkeypatch):
    """A caller giving up (its timeout cancels the future) sets the request's abort, so the worker stops."""
    stopped = []

    def slow(_pool, _request, abort, _log):
        stopped.append(abort.wait(5.0))
        return SOLVED

    monkeypatch.setattr(motion, "run_plan", slow)

    async def scenario():
        plugin = planner()
        answer = plugin.request_plan(request("slow"))
        with pytest.raises(asyncio.TimeoutError):
            async with timeout(0.05):
                await answer
        await idle(plugin)
        plugin.teardown(None)

    asyncio.run(scenario())
    assert stopped == [True]


def test_errors_come_back_as_failed_answers(tmp_path):
    """The real work, failing inside: every kind of request answers with the error, none raises."""
    process = write(build_process(tmp_path), tmp_path / "p")
    loaded = loaded_process(load_process(Path(process.folder), DATA))
    cindy = robot_config(CINDY)
    live = robot_id(cindy.serial)
    snapshot = Scene(robots={live: RobotObject(live, cindy.model, Pose(), {})})
    asked = MotionPlanRequest(loaded, loaded.steps[2], snapshot, (cindy,))

    async def scenario():
        plugin = planner()
        answers = await asyncio.gather(plugin.request_plan(asked), plugin.request_ik(asked),
                                       plugin.request_collisions(asked))
        plugin.teardown(None)
        return answers

    plan, ik, collisions = asyncio.run(scenario())
    reason = "the planner raised RuntimeError('no mirror here')"
    assert plan.result.status == "failed" and (plan.result.reason, ik.reason, collisions.reason) == (reason,) * 3


def test_teardown_closes_the_pool_once():
    """Teardown closes the mirrors on the worker, once, and waits for it."""
    pool = Pool()
    planner(pool).teardown(None)
    assert pool.closed == 1


@pytest.mark.slow
def test_the_service_on_the_purged_process():
    """B20's transfer at its design base: the authored goal by ssik, the free transfer planned, the start checked.

    Teardown closes the mirror.
    """
    if ssik_unavailable() is not None:
        pytest.skip(ssik_unavailable())
    purged = purged_process()
    configs = tuple(robot_config(serial) for serial in (ALICE, BELLE, CINDY))
    asked = MotionPlanRequest(purged, step(purged, "B20_J_transfer"), design_scene(purged, "B20_J_transfer"), configs,
                              transfer_as_free=True)

    async def scenario():
        plugin = MotionPlannerPlugin()
        plugin._log = lambda _line: None
        answers = (await plugin.request_ik(asked), await plugin.request_plan(asked),
                   await plugin.request_collisions(asked))
        worlds = plugin._worlds
        plugin.teardown(None)
        return plugin, worlds, answers

    plugin, worlds, (check, plan, report) = asyncio.run(scenario())
    assert check.ok and check.ik.backend == "ssik" and check.max_from_authored <= 1e-6, check.reason
    assert plan.ok and plan.routine is Routine.TRANSFER_FREE and plan.planner == "tamp.plan_free_dual_arm"
    assert report.reason == "" and report.pairs == ()
    assert worlds == ("a200-0806",) and plugin._pool.serials == ()
