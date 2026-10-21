"""The arm planner as a headless planner (staleness, send) and its panel driving it through the API."""

from __future__ import annotations

import asyncio
import math
from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest
from robot_fakes import lease_test_robots, wait_ticks_until

from husky_assembly_teleop.plugin_api.concurrency import WaitTimeout
from husky_assembly_teleop.plugins.arm_planner import planner_plugin as arm_plugin
from husky_assembly_teleop.plugins.arm_planner.panel_plugin import ArmPlannerPanel
from husky_assembly_teleop.plugins.arm_planner.planner import ArmPath, arm_joint_names
from husky_assembly_teleop.plugins.arm_planner.planner_plugin import ArmPlannerPlugin
from husky_assembly_teleop.shared.arm_planner_contract import ArmPlanRequest
from husky_assembly_teleop.shared.planner.planner_search import PlanResult
from husky_assembly_teleop.shared.planner.plan import Plan, PlannerContract
from scaffolding_assembly_core.ids import robot_id

SERIAL, ARM = "0806", "left_ur_arm"
NAMES = arm_joint_names(ARM)
START = np.array([0.0, -1.5, 1.5, 0.0, 0.0, 0.0])
GOAL = START + np.array([0.4, 0.0, 0.0, 0.0, 0.0, 0.0])
PATH = ArmPath.along(NAMES, np.array([START, GOAL]))
REQUEST = ArmPlanRequest(SERIAL, ARM, tuple(GOAL))


def pose(x: float, y: float, yaw: float) -> SimpleNamespace:
    """A world pose on the floor; orientation (x, y, z, w)."""
    return SimpleNamespace(position=(x, y, 0.0), orientation=(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


class FakeCtx:
    """A plugin context: the arm at `joints`, the base at `base` (x, y, yaw), an arm that records its path.

    Mocap tracks the base until `clock` passes 0.3 s; intents run at once.
    """

    def __init__(self):
        """The arm at START, the base at the origin with a live mocap pose, and an arm that accepts."""
        self.joints, self.base, self.sent, self.clock = dict(zip(NAMES, START)), (0.0, 0.0, 0.0), [], 0.0
        arm = SimpleNamespace(send_joint_trajectory=lambda positions, duration, velocities: self.sent.append(positions)
                              or True, state=SimpleNamespace(is_executing=False))
        mocap = SimpleNamespace(marker_errors=deque([(0.0, 0.001)]), last_update_time=0.0, last_fix_time=0.0,
                                tracking_valid=True, tracked=True)
        self.world = SimpleNamespace(robots={SERIAL: SimpleNamespace(
            arms={ARM: arm}, base=SimpleNamespace(mocap_id=5, state=mocap))})
        robot = SimpleNamespace(enabled=False, acting_problems=lambda: [], base=pose(0.0, 0.0, 0.0), base_tracked=True)
        self.scene = SimpleNamespace(snapshot=SimpleNamespace(robots={robot_id(SERIAL): robot}))
        self.kinematics = SimpleNamespace(joints=lambda _serial: self.joints,
                                          base_pose=lambda _serial: pose(*self.base))

    def submit(self, _label, run):
        """Run the intent now."""
        run()

    def spawn(self, _label, work):
        """Start the task."""
        return asyncio.get_running_loop().create_task(work)

    def now(self):
        """The clock, set by the test."""
        return self.clock

    lease_robots = staticmethod(lease_test_robots)
    wait_until = staticmethod(wait_ticks_until)

    def log_info(self, _text):
        """Ignore the line."""

    log_warn = log_info


def make_planner(ctx: FakeCtx) -> ArmPlannerPlugin:
    """A planner whose world and search are fakes that find PATH."""
    planner = ArmPlannerPlugin()
    planner._world = SimpleNamespace(sync=lambda snapshot, serial: None, close=lambda: None)
    planner.setup(ctx)
    return planner


@pytest.fixture
def found_path(monkeypatch):
    """Every search finds PATH."""
    monkeypatch.setattr(arm_plugin, "plan_arm", lambda world, mirror, arm, goal, abort: PlanResult(PATH, seconds=0.1))


def run(planner: ArmPlannerPlugin, ctx: FakeCtx, scenario):
    """Run `scenario()` in an event loop, then tear the planner down."""
    async def main():
        try:
            return await scenario()
        finally:
            planner.teardown(ctx)

    return asyncio.run(main())


def ready_plan() -> Plan:
    """A ready plan for REQUEST."""
    return Plan(1, REQUEST, "ready", path=PATH)


def test_the_planner_is_an_arm_planner_api():
    """Other plugins can rely on the planner API (`ArmPlannerContract` is it, typed with the arm request)."""
    assert isinstance(ArmPlannerPlugin(), PlannerContract)


def plan_from(ctx: FakeCtx, planner: ArmPlannerPlugin) -> Plan:
    """A ready plan made from where `ctx` is now."""
    async def scenario():
        return await planner.request_plan(REQUEST)

    plan = run(planner, ctx, scenario)
    assert plan.state == "ready" and planner.stale_reason(ctx, plan) == ""
    return plan


@pytest.mark.safety
def test_a_plan_is_stale_once_the_arm_or_base_moves(found_path):
    """The arm past 2 degrees, the base past 5 cm or a turn past 3 degrees since planning mark the plan."""
    ctx = FakeCtx()
    planner = make_planner(ctx)
    plan = plan_from(ctx, planner)
    ctx.joints = {**ctx.joints, NAMES[0]: math.radians(3.0)}
    assert planner.stale_reason(ctx, plan) == "the arm moved since planning"
    ctx.joints = dict(zip(NAMES, START))
    ctx.base = (0.1, 0.0, 0.0)
    assert planner.stale_reason(ctx, plan) == "the base moved since planning"
    ctx.base = (0.0, 0.0, math.radians(2.0))
    assert planner.stale_reason(ctx, plan) == ""
    ctx.base = (0.0, 0.0, math.radians(-4.0))
    assert planner.stale_reason(ctx, plan) == "the base moved since planning"


@pytest.mark.safety
def test_a_plan_from_a_tracked_base_is_stale_once_mocap_loses_it(found_path):
    """The last fix stays put while an untracked base moves, so losing the pose marks the plan.

    A plan made from a guessed base is not judged by tracking.
    """
    ctx = FakeCtx()
    planner = make_planner(ctx)
    plan = plan_from(ctx, planner)
    ctx.clock = 0.5
    assert planner.stale_reason(ctx, plan).startswith("mocap lost the base since planning")
    ctx.clock = 0.0
    ctx.scene.snapshot.robots[robot_id(SERIAL)].base_tracked = False
    other = make_planner(ctx)
    guessed = plan_from(ctx, other)
    ctx.clock = 0.5
    assert other.stale_reason(ctx, guessed) == ""


def test_the_start_base_belongs_to_the_plan_it_was_made_for(found_path):
    """A replaced or cleared plan leaves no base behind, so another plan is not judged by it."""
    ctx = FakeCtx()
    planner = make_planner(ctx)

    async def scenario():
        plan = await planner.request_plan(REQUEST)
        planner.clear()
        return plan

    plan = run(planner, ctx, scenario)
    ctx.base = (1.0, 0.0, 0.0)
    assert planner._start_base is None and planner.stale_reason(ctx, plan) == ""


@pytest.mark.safety
def test_execute_sends_the_path_and_waits_until_the_arm_stands_still():
    """The plan's path goes to the request's arm, the commit is answered, and the motion is watched to its end."""
    ctx, planner, answers = FakeCtx(), ArmPlannerPlugin(), []
    arm = ctx.world.robots[SERIAL].arms[ARM]
    arm.state.is_executing = True

    async def scenario():
        task = asyncio.create_task(planner.execute(ctx, ready_plan(), lambda *answer: answers.append(answer)))
        await asyncio.sleep(0)
        moving = not task.done()
        arm.state.is_executing = False
        await task
        return moving

    assert asyncio.run(scenario()) and answers == [(True, f"sent to {SERIAL} {ARM}")]
    assert np.allclose(ctx.sent[0][0], START) and np.allclose(ctx.sent[0][-1], GOAL)


@pytest.mark.safety
def test_execute_reports_an_arm_that_refuses_the_path():
    """An arm that refuses the trajectory answers the commit that nothing was sent."""
    ctx, planner, answers = FakeCtx(), ArmPlannerPlugin(), []
    ctx.world.robots[SERIAL].arms[ARM].send_joint_trajectory = lambda *args: False
    asyncio.run(planner.execute(ctx, ready_plan(), lambda *answer: answers.append(answer)))
    (sent, message), = answers
    assert not sent and "refused" in message


@pytest.mark.safety
@pytest.mark.parametrize("how", ["cancel", "late"])
def test_an_arm_path_cancelled_or_late_is_held(how):
    """Cancelled (Stop all, plugin stop) or still moving past its duration: the arm is held."""
    ctx, planner, held = FakeCtx(), ArmPlannerPlugin(), []
    arm = ctx.world.robots[SERIAL].arms[ARM]
    arm.state.is_executing, arm.hold = True, lambda: held.append(how)

    async def late(*_args, description="", **_kwargs):
        raise WaitTimeout(f"timed out waiting for {description}")

    if how == "late":
        ctx.wait_until = late

    async def scenario():
        task = asyncio.create_task(planner.execute(ctx, ready_plan(), lambda *_answer: None))
        await asyncio.sleep(0)
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)

    asyncio.run(scenario())
    assert held == [how]


def make_panel(planner: ArmPlannerPlugin) -> ArmPlannerPanel:
    """A panel on `planner` with six sliders at the goal, as the operator would have set them."""
    panel = ArmPlannerPanel()
    panel._api = planner
    panel.serial, panel.arm, panel._has_target, panel._load_now = SERIAL, ARM, True, False
    panel._sliders = [SimpleNamespace(value=math.degrees(value)) for value in GOAL]
    panel._used = SimpleNamespace(touch=lambda: None, active=True)
    panel._collisions = FakeCollisions()
    return panel


class FakeCollisions:
    """The collisions panel: keeps what each source shows, and which labels are ticked."""

    def __init__(self):
        """Nothing shown."""
        self.shown = {}

    def replace(self, source, hits, shown=()):
        """Keep the source's hits and ticked labels."""
        self.shown[source] = (dict(hits), tuple(shown))

    def clear(self, source):
        """Drop the source."""
        self.shown.pop(source, None)


def test_target_change_compares_the_sliders_and_arm_with_the_plan():
    """Sliders at the planned goal are no change; a slider moved by 3 degrees, or another arm, is."""
    panel = make_panel(ArmPlannerPlugin())
    assert panel.target_change(None, ready_plan()) == ""
    panel._sliders[0].value += 3.0
    assert panel.target_change(None, ready_plan()) == "the target changed since planning"
    panel._sliders[0].value -= 3.0
    panel.arm = "right_ur_arm"
    assert "arm changed" in panel.target_change(None, ready_plan())


def test_make_request_takes_the_sliders():
    """The request carries the chosen robot and arm and the slider values in radians."""
    request = make_panel(ArmPlannerPlugin()).make_request(None)
    assert request == ArmPlanRequest(SERIAL, ARM, pytest.approx(tuple(GOAL)))


@pytest.mark.safety
def test_the_panel_plans_and_commits_through_the_api(found_path):
    """Plan asks the planner for the slider target; Commit refuses a moved target, else the arm gets the path."""
    ctx = FakeCtx()
    planner = make_planner(ctx)
    panel = make_panel(planner)

    async def scenario():
        panel._on_path(ctx, "Plan")
        await asyncio.sleep(0.05)
        panel.update(ctx)
        found = panel._message
        panel._sliders[2].value += 5.0
        panel._commit(ctx)
        refused = panel._message
        panel._sliders[2].value -= 5.0
        panel._commit(ctx)
        await asyncio.sleep(0.01)
        return found, refused, panel._message

    found, refused, committed = run(planner, ctx, scenario)
    assert found.startswith(f"RRT: {PATH.duration:.1f} s, 0 rests on the way") and "target changed" in refused
    assert committed == f"sent to {SERIAL} {ARM}" and planner.plan.state == "sent"
    assert planner.plan.request == ArmPlanRequest(SERIAL, ARM, pytest.approx(tuple(GOAL))) and len(ctx.sent) == 1


def test_a_plan_refused_for_a_collision_shows_its_hit_until_the_next_plan(monkeypatch):
    """A target in collision lists its hit in the collisions panel, ticked; the next plan found removes it."""
    hit = SimpleNamespace(where="target", message="target is in collision: a with b")
    results = iter([PlanResult(None, hit.message, hit=hit), PlanResult(PATH, seconds=0.1)])
    monkeypatch.setattr(arm_plugin, "plan_arm", lambda world, mirror, arm, goal, abort: next(results))
    ctx = FakeCtx()
    planner = make_planner(ctx)
    panel = make_panel(planner)

    async def scenario():
        shown = []
        for _ in range(2):
            panel._on_path(ctx, "Plan")
            await asyncio.sleep(0.05)
            panel.update(ctx)
            shown.append(dict(panel._collisions.shown))
        return shown

    refused, found = run(planner, ctx, scenario)
    label = f"{SERIAL} {ARM} target"
    assert planner.plan.state == "ready" and refused == {"arm_planner_panel": ({label: hit}, (label,))}
    assert found == {}


@pytest.mark.safety
def test_a_plan_warns_about_other_robots_it_could_not_check(found_path):
    """Another robot present with an untracked base is left out of the check: the plan says so, as a warning."""
    ctx = FakeCtx()
    blind = SimpleNamespace(enabled=True, acting_problems=lambda: ["its base is not tracked"])
    absent = SimpleNamespace(enabled=False, acting_problems=lambda: ["it is not in the scene"])
    ctx.scene.snapshot.robots.update({robot_id("a200-0805"): blind, robot_id("a200-0804"): absent})
    planner = make_planner(ctx)

    async def scenario():
        return await planner.request_plan(REQUEST)

    plan = run(planner, ctx, scenario)
    assert plan.state == "ready" and plan.warning
    assert "not checked against a200-0805: its base is not tracked" in plan.message
    assert "a200-0804" not in plan.message


def test_a_plan_with_every_robot_checked_has_no_such_warning(found_path):
    """Robots present and measured, or absent, need no warning."""
    ctx = FakeCtx()
    ctx.scene.snapshot.robots[robot_id("a200-0805")] = SimpleNamespace(enabled=True, acting_problems=lambda: [])
    planner = make_planner(ctx)

    async def scenario():
        return await planner.request_plan(REQUEST)

    plan = run(planner, ctx, scenario)
    assert plan.state == "ready" and not plan.warning and "not checked" not in plan.message
