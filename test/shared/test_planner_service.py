"""The headless planner service and its panel: every request settles, commits go through the API, the panel follows."""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest
from robot_fakes import lease_test_robots, wait_ticks_until

from husky_assembly_teleop.shared.planner.planner_panel import (COMMIT_CONFIRM_SECONDS, CommitConfirm, PlannerPanel,
                                                                unknown_robots_chip)
from husky_assembly_teleop.shared.planner.timed_path import TimedPath
from husky_assembly_teleop.shared.planner.planner_search import PlanResult
from husky_assembly_teleop.shared.planner.plan import Plan, PlannerContract
from husky_assembly_teleop.shared.planner.planner_search import Search
from husky_assembly_teleop.robot_interface.robot_lease import Lease, LeaseSlot, leased
from husky_assembly_teleop.robot_interface.soft_stop import RobotStopped
from husky_assembly_teleop.shared.planner.planner_service import PlannerService
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

PATH = TimedPath(times=np.array([0.0, 1.0]), points=np.array([[0.0], [1.0]]))


@dataclass(frozen=True)
class Request:
    """A request with a goal to tell requests apart."""

    serial: str
    goal: float = 1.0


class Ctx:
    """A plugin context whose intents run at once and whose tasks are plain asyncio tasks."""

    def __init__(self):
        """No log yet; no robots (`world`, `scene`)."""
        self.log = []
        self.world = SimpleNamespace(robots={})
        self.scene = SimpleNamespace(snapshot=Scene())

    def submit(self, _label, run):
        """Run the intent now."""
        run()

    def spawn(self, _label, work):
        """Start the task."""
        return asyncio.get_running_loop().create_task(work)

    lease_robots = staticmethod(lease_test_robots)

    def log_info(self, text):
        """Keep the line."""
        self.log.append(text)


class Planner(PlannerService):
    """A planner whose search is `work`, with a stale reason and a send outcome set by the test."""

    name = "test_planner"

    def __init__(self, work=lambda abort: PlanResult(PATH), time_limit=5.0):
        """Plan with `work(abort)`; send succeeds; nothing is stale."""
        super().__init__()
        self.work, self.time_limit = work, time_limit
        self.stale, self.taken, self.sent, self.forgotten = "", True, [], []

    def make_search(self, ctx, request, abort):
        """Run `work` on the worker."""
        return Search(label=f"plan {request.serial}", message="planning…", work=lambda: self.work(abort),
                      time_limit=self.time_limit, accepted=lambda result: ("found", False))

    def stale_reason(self, ctx, plan):
        """The test's stale reason."""
        return self.stale

    async def execute(self, ctx, plan, answer):
        """Keep the plan and answer as the test says; the motion is over at once."""
        self.sent.append(plan.id)
        answer(self.taken, "sent" if self.taken else "refused")

    def forget(self, plan):
        """Keep what was forgotten."""
        self.forgotten.append(plan.id)


def run(planner: Planner, scenario):
    """Set the planner up on a fake context and run `scenario(planner, ctx)` in an event loop."""
    ctx = Ctx()
    planner.setup(ctx)

    async def main():
        try:
            return await scenario(planner, ctx)
        finally:
            planner.teardown(ctx)

    return asyncio.run(main())


def test_the_service_offers_the_planner_api():
    """A planner is a PlannerContract, so users can rely on that interface."""
    assert isinstance(Planner(), PlannerContract)


def test_a_found_path_makes_a_ready_plan():
    """The future and `plan` give the same ready plan, with its path and message."""
    async def scenario(planner, ctx):
        plan = await planner.request_plan(Request("alice"))
        return plan, planner.plan

    plan, latest = run(Planner(), scenario)
    assert plan is latest and plan.state == "ready" and plan.path is PATH and plan.message == "found"
    assert plan.request == Request("alice") and plan.id == 1


@pytest.mark.parametrize("work, time_limit, state, message", [
    (lambda abort: PlanResult(None, "blocked"), 5.0, "failed", "no plan: blocked"),
    (lambda abort: (abort.wait(2.0), PlanResult(None))[1], 0.05, "failed", "timed out"),
])
def test_no_path_or_a_timeout_fails_the_plan(work, time_limit, state, message):
    """No path and a timeout both settle the plan as failed, with why."""
    async def scenario(planner, ctx):
        return await planner.request_plan(Request("alice"))

    plan = run(Planner(work, time_limit), scenario)
    assert plan.state == state and message in plan.message


@pytest.mark.parametrize("how", ["cancel", "soft stop"])
def test_a_cancelled_search_settles_as_cancelled_and_stops_the_worker(how):
    """Cancel and a soft stop (which cancels the task) both resolve the future; the worker sees its abort flag."""
    seen = []

    def slow(abort):
        seen.append(abort.wait(2.0))
        return PlanResult(None)

    async def scenario(planner, ctx):
        future = planner.request_plan(Request("alice"))
        await asyncio.sleep(0.02)
        planner.cancel() if how == "cancel" else planner._task.cancel()
        plan = await asyncio.wait_for(future, 1.0)
        await asyncio.sleep(0.05)
        return plan, planner.plan

    plan, latest = run(Planner(slow), scenario)
    assert plan.state == "cancelled" and latest is plan
    if how == "cancel":
        assert seen == [True]


def test_a_new_request_replaces_the_running_one():
    """The first search ends as cancelled, and only the second becomes the planner's plan."""
    def first_slow(abort):
        return PlanResult(None) if abort.wait(2.0) else PlanResult(PATH)

    async def scenario(planner, ctx):
        first = planner.request_plan(Request("alice", goal=1.0))
        await asyncio.sleep(0.02)
        planner.work = lambda abort: PlanResult(PATH)
        second = planner.request_plan(Request("alice", goal=2.0))
        return await first, await second, planner.plan

    first, second, latest = run(Planner(first_slow), scenario)
    assert first.state == "cancelled" and second.state == "ready" and latest is second
    assert second.id == first.id + 1 and second.request.goal == 2.0


def test_a_raising_search_fails_the_plan_and_the_task():
    """The plan says why; the task still raises, so it counts against the plugin."""
    def broken(abort):
        raise RuntimeError("world out of sync")

    async def scenario(planner, ctx):
        future = planner.request_plan(Request("alice"))
        plan = await future
        with pytest.raises(RuntimeError):
            await planner._task
        return plan

    plan = run(Planner(broken), scenario)
    assert plan.state == "failed" and "world out of sync" in plan.message


@pytest.mark.safety
def test_commit_sends_only_the_latest_ready_plan():
    """Commit refuses a replaced plan and a stale one, and sends the latest ready one, which becomes sent."""
    async def scenario(planner, ctx):
        old = await planner.request_plan(Request("alice"))
        new = await planner.request_plan(Request("alice"))
        refused_old = await planner.commit(old.id)
        planner.stale = "the arm moved since planning"
        planner.update(ctx)
        refused_stale = await planner.commit(new.id)
        stale = planner.plan
        planner.stale = ""
        newest = await planner.request_plan(Request("alice"))
        sent = await planner.commit(newest.id)
        return refused_old, refused_stale, stale, sent, planner.plan

    planner = Planner()
    refused_old, refused_stale, stale, sent, latest = run(planner, scenario)
    assert refused_old == (False, "nothing sent: that plan was replaced or cleared")
    assert not refused_stale[0] and "stale" in refused_stale[1] and stale.state == "stale"
    assert sent == (True, "sent") and latest.state == "sent" and planner.sent == [latest.id]


@pytest.mark.safety
def test_a_commit_to_a_soft_stopped_robot_answers_nothing_sent():
    """A send the stopped robot refuses still resolves the commit, and the plan stays ready for after Resume."""
    async def scenario(planner, ctx):
        plan = await planner.request_plan(Request("alice"))
        return await planner.commit(plan.id), planner.plan

    async def refused(ctx, plan, answer):
        raise RobotStopped("robot alice is soft stopped, a path not sent; press Resume first")

    planner = Planner()
    planner.execute = refused
    committed, latest = run(planner, scenario)
    assert committed == (False, "nothing sent: robot alice is soft stopped, a path not sent; press Resume first")
    assert latest.state == "ready"


@pytest.mark.safety
def test_a_commit_holds_the_robots_lease_from_the_send_until_the_motion_ends():
    """The commit answers once sent; the lease is held around the send and while the motion runs, then given back."""
    guard, running, seen = LeaseSlot("alice"), [True], []

    async def execute(ctx, plan, answer):
        seen.append(guard.problem())
        answer(True, "sent")
        await wait_ticks_until(lambda: not running[0])

    async def scenario(planner, ctx):
        ctx.lease_robots = lambda _serials, label: leased([guard], label)
        plan = await planner.request_plan(Request("alice"))
        committed = await planner.commit(plan.id)
        held = guard.lease.holder if guard.lease else None
        running[0] = False
        for _ in range(3):
            await asyncio.sleep(0)
        return plan, committed, held

    planner = Planner()
    planner.execute = execute
    plan, committed, held = run(planner, scenario)
    assert committed == (True, "sent") and seen == [None] and planner.plan.state == "sent"
    assert held == f"commit plan {plan.id}" and guard.lease is None


@pytest.mark.safety
def test_a_commit_hands_execute_the_robots_the_operator_let_be_unknown():
    """`commit(plan_id, unknown_robots)` is what `unknown_allowed` gives `execute`; none by default."""
    seen = []

    async def execute(ctx, plan, answer):
        seen.append(planner.unknown_allowed(plan))
        answer(True, "sent")

    async def scenario(planner, ctx):
        first = await planner.request_plan(Request("alice"))
        await planner.commit(first.id, {"robots/b"})
        second = await planner.request_plan(Request("alice"))
        await planner.commit(second.id)

    planner = Planner()
    planner.execute = execute
    run(planner, scenario)
    assert seen == [{"robots/b"}, frozenset()]


@pytest.mark.safety
def test_a_commit_to_a_robot_leased_elsewhere_sends_nothing():
    """Another plugin has the robot's lease: nothing is sent, the answer names the holder, and the plan stays ready."""
    guard = LeaseSlot("alice")
    Lease("robot_control: teleop", (guard,))

    async def scenario(planner, ctx):
        ctx.lease_robots = lambda _serials, label: leased([guard], label)
        plan = await planner.request_plan(Request("alice"))
        return await planner.commit(plan.id), planner.plan

    planner = Planner()
    committed, latest = run(planner, scenario)
    assert not committed[0] and "robot alice is leased by robot_control: teleop" in committed[1]
    assert planner.sent == [] and latest.state == "ready" and guard.lease.holder == "robot_control: teleop"


@pytest.mark.safety
@pytest.mark.parametrize("replaces", [False, True])
def test_a_commit_to_a_robot_still_moving_is_refused_or_replaces_the_motion(replaces):
    """Refused while the earlier commit still moves the robot, or that commit is cancelled as replaced."""
    ended = []

    async def execute(ctx, plan, answer):
        answer(True, f"sent {plan.id}")
        try:
            await wait_ticks_until(lambda: False)
        except asyncio.CancelledError:
            ended.append((plan.id, planner.replaced()))
            raise

    async def scenario(planner, ctx):
        first = await planner.request_plan(Request("alice"))
        await planner.commit(first.id)
        second = await planner.request_plan(Request("alice", goal=2.0))
        return await planner.commit(second.id)

    planner = Planner()
    planner.replaces_running, planner.execute = replaces, execute
    committed = run(planner, scenario)
    if replaces:
        assert committed == (True, "sent 2") and ended[0] == (1, True)
    else:
        assert committed == (False, "nothing sent: an earlier commit still moves alice; wait, or Hold it")


def test_clear_forgets_the_plan():
    """After clear there is no plan, the planner forgot it, and committing it sends nothing."""
    async def scenario(planner, ctx):
        plan = await planner.request_plan(Request("alice"))
        planner.clear()
        return plan, planner.plan, await planner.commit(plan.id)

    planner = Planner()
    plan, latest, committed = run(planner, scenario)
    assert latest is None and plan.id in planner.forgotten and not committed[0] and planner.sent == []


class Panel(PlannerPanel):
    """A panel with a target value and a target change set by the test."""

    name = "test_panel"
    planner = "test_planner"

    def __init__(self):
        """Target 1.0 for alice, unchanged."""
        super().__init__()
        self.serial, self._has_target, self.goal, self.change = "alice", True, 1.0, ""
        self._used = SimpleNamespace(touch=lambda: None, active=True)

    def make_request(self, ctx):
        """The current target."""
        return Request(self.serial, self.goal)

    def target_change(self, ctx, plan):
        """The test's target change."""
        return self.change


@pytest.mark.safety
def test_the_panel_plans_and_commits_through_the_api():
    """Plan sends the panel's request; Commit refuses when the target moved, else goes through the planner."""
    async def scenario(planner, ctx):
        panel = Panel()
        panel._api = planner
        ctx.now = lambda: 0.0
        panel._on_path(ctx, "Plan")
        await asyncio.sleep(0.05)
        panel.update(ctx)
        found = panel._message
        panel.change = "the target changed since planning"
        panel._commit(ctx)
        refused = panel._message
        panel.change = ""
        panel._commit(ctx)
        await asyncio.sleep(0.01)
        return found, refused, panel._message, planner.plan

    planner = Planner()
    found, refused, committed, plan = run(planner, scenario)
    assert found == "found" and "target changed" in refused and committed == "sent"
    assert plan.state == "sent" and plan.request == Request("alice", 1.0) and planner.sent == [plan.id]


@pytest.mark.safety
def test_a_refused_send_leaves_the_plan_ready():
    """The robot refusing keeps the plan ready, so it can be committed again once the robot is ready."""
    async def scenario(planner, ctx):
        plan = await planner.request_plan(Request("alice"))
        planner.taken = False
        return await planner.commit(plan.id), planner.plan

    (sent, message), latest = run(Planner(), scenario)
    assert not sent and message == "refused" and latest.state == "ready"


def test_the_panel_asks_for_a_replan_once_its_target_moves():
    """A ready plan whose target the panel moved away from shows "target moved" and says how, once."""
    async def scenario(planner, ctx):
        panel = Panel()
        panel._api = planner
        ctx.now = lambda: 0.0
        await planner.request_plan(Request("alice"))
        panel.update(ctx)
        before = panel._plan_chip()
        panel.change = "the target changed since planning"
        panel.update(ctx)
        return before, panel._plan_chip(), panel._message

    before, after, message = run(Planner(), scenario)
    assert "plan ready" in before and "target moved" in after and message == "the target changed since planning"


@pytest.mark.safety
def test_cancel_commit_stops_the_running_motion():
    """Cancel ends the running commit, whose execute then stops the robot; with nothing running it says so."""
    stopped = []

    async def execute(ctx, plan, answer):
        answer(True, "sent")
        try:
            await wait_ticks_until(lambda: False)
        except asyncio.CancelledError:
            stopped.append(plan.id)  # * where a planner stops its robot
            raise

    async def scenario(planner, ctx):
        idle = planner.cancel_commit()
        plan = await planner.request_plan(Request("alice"))
        await planner.commit(plan.id)
        cancelled = planner.cancel_commit()
        for _ in range(3):
            await asyncio.sleep(0)
        return idle, cancelled, plan, planner.cancel_commit()

    planner = Planner()
    planner.execute = execute
    idle, cancelled, plan, again = run(planner, scenario)
    assert not idle and cancelled and stopped == [plan.id] and not again


def test_the_unknown_robots_chip_counts_the_others_and_hides_at_zero():
    """No chip while every other robot is live; else a red count, each robot named on hover; the planned one never."""
    model = SimpleNamespace(movable_joints=())
    live, lost = (RobotObject(f"robots/{name}", model, Pose(), {}, base_time=1.0) for name in ("a", "b"))
    lost.base_tracked = False
    acting = RobotObject("robots/me", model, Pose(), {}, base_tracked=False)
    assert unknown_robots_chip(Scene(robots={robot.id: robot for robot in (live, acting)}), ["robots/me"]) == ""
    found = unknown_robots_chip(Scene(robots={robot.id: robot for robot in (live, lost, acting)}), ["robots/me"])
    assert ">1 robot unknown<" in found and 'title="b: base unknown\nHealth shows why."' in found and "me" not in found


def test_commit_asks_for_a_second_press_within_two_seconds_while_a_robot_is_unknown():
    """Robot b lost mocap: the first press asks and turns Commit amber; a second within 2 s sends, letting b be unknown.

    Later it asks again. The robot the commit moves is not counted.
    """
    model = SimpleNamespace(movable_joints=())
    lost = RobotObject("robots/b", model, Pose(), {}, base_tracked=False, base_time=1.0)
    ctx = SimpleNamespace(scene=SimpleNamespace(snapshot=Scene(robots={lost.id: lost})), t=10.0)
    ctx.now = lambda: ctx.t
    confirm = CommitConfirm()
    confirm.button = SimpleNamespace(color="blue")
    asked, allowed = confirm.asks(ctx, ["robots/me"], "Commit")
    assert asked == "press Commit again within 2 s to move with 1 other robot unknown"
    assert allowed == frozenset()
    confirm.draw(ctx.t)
    assert confirm.button.color == "orange" and COMMIT_CONFIRM_SECONDS == 2.0
    ctx.t += 1.5
    assert confirm.asks(ctx, ["robots/me"], "Commit") == ("", {"robots/b"}), "the second press lets b be unknown"
    confirm.draw(ctx.t)
    assert confirm.button.color == "blue"
    assert confirm.asks(ctx, ["robots/me"], "Commit")[0]
    ctx.t += 2.5
    assert confirm.asks(ctx, ["robots/me"], "Commit")[0], "too late: it asks again"
    assert confirm.asks(ctx, ["robots/b"], "Commit") == ("", frozenset()), "the robot it moves is not counted"


def test_commit_asks_twice_for_a_panels_own_warning_too():
    """With every robot measured, a warning such as an arm out of stow still asks first; the second press sends."""
    ctx = SimpleNamespace(scene=SimpleNamespace(snapshot=Scene()), t=10.0)
    ctx.now = lambda: ctx.t
    confirm = CommitConfirm()
    assert confirm.asks(ctx, ["robots/me"], "Commit") == ("", frozenset())
    asked, _allowed = confirm.asks(ctx, ["robots/me"], "Commit", ["the arm 40° from stow"])
    assert asked == "press Commit again within 2 s to move with the arm 40° from stow"
    ctx.t += 1.0
    assert confirm.asks(ctx, ["robots/me"], "Commit", ["the arm 40° from stow"]) == ("", frozenset())


def test_a_re_timed_plan_keeps_the_message_line_and_a_new_message_replaces_it():
    """The same plan re-timed (a new object, same id, state and message) is not news: a question below stays."""
    plan = Plan(1, Request("alice", 1.0), "ready", PATH, message="found, warned", warning=True)
    api, warned = SimpleNamespace(plan=plan), []
    panel = Panel()
    panel._api = api
    ctx = SimpleNamespace(now=lambda: 0.0, log_warn=warned.append)
    panel.update(ctx)
    panel._say("press Commit again within 2 s to move with the arm 40° from stow")
    api.plan = Plan(1, plan.request, "ready", PATH, message="found, warned", warning=True)
    panel.update(ctx)
    assert panel._message.startswith("press Commit again") and warned == ["found, warned"]
    api.plan = Plan(1, plan.request, "sent", PATH, message="sent to alice")
    panel.update(ctx)
    assert panel._message == "sent to alice"
