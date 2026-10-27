"""The arm planner as a headless planner (staleness, send) and its panel driving it through the API."""

from __future__ import annotations

import asyncio
import math
from collections import deque
from types import SimpleNamespace

import numpy as np
import pytest
from plugin_fakes import ArmDriver, FakeContext, fake_arm, fake_node, fake_robot
from robot_fakes import lease_test_robots, wait_ticks_until

from husky_assembly_teleop.plugins.arm_planner import planner_plugin as arm_plugin
from husky_assembly_teleop.plugins.arm_planner.panel_plugin import ArmPlannerPanel
from husky_assembly_teleop.plugins.arm_planner.planner import ArmPath
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.plugins.arm_planner.planner_plugin import ArmPlannerPlugin
from husky_assembly_teleop.shared.arm_planner_contract import ArmPlanRequest
from husky_assembly_teleop.shared.execution.send import ARRIVED_TOLERANCE
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
                              or True, state=SimpleNamespace(last_update_time=0.0, last_motion_time=0.0),
                              joint_vector=lambda: START, hold=lambda: True)
        mocap = SimpleNamespace(marker_errors=deque([(0.0, 0.001)]), last_update_time=0.0, last_fix_time=0.0,
                                tracking_valid=True, tracked=True)
        self.world = SimpleNamespace(robots={SERIAL: SimpleNamespace(
            config=SimpleNamespace(serial=SERIAL), arms={ARM: arm}, base=SimpleNamespace(mocap_id=5, state=mocap),
            lease=None)})
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


def _driven(speed: float = 1.0, start: np.ndarray = START) -> tuple[FakeContext, object]:
    """A context over robot SERIAL with a real arm ARM at `start`, run by an ArmDriver at `speed` (1: as timed)."""
    driver = None
    ctx = FakeContext(name="arm_planner", step=lambda t: driver.tick(t))
    node = fake_node(ctx.now, ctx.logs)
    arm = fake_arm(node, ARM, start)
    ctx.world.robots[SERIAL] = robot = fake_robot(node, SERIAL, [arm])
    driver = ArmDriver(arm, robot, speed=speed)
    return ctx, arm


def _commit(ctx: FakeContext, answers: list, cancel_after: int | None = None) -> np.ndarray:
    """Execute the ready plan under the robot's lease, cancelled after `cancel_after` ticks if given; then let 2 s pass.

    Returns:
        np.ndarray: The arm's joints when the commit ended (and the lease went back).
    """
    async def commit():
        with ctx.lease_robots([SERIAL], "commit plan 1"):
            await ArmPlannerPlugin().execute(ctx, ready_plan(), lambda *answer: answers.append(answer))

    async def scenario():
        task = asyncio.create_task(commit())
        if cancel_after is not None:
            for _ in range(cancel_after):
                await ctx.next_tick()
            task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        ended = ctx.world.robots[SERIAL].arms[ARM].joint_vector().copy()
        # * A hold brakes over HOLD_DURATION from one tick further on: at 10 % speed that takes 21 ticks.
        for _ in range(25):
            await ctx.next_tick()
        return ended

    return asyncio.run(scenario())


def _held(arm) -> bool:
    """Whether the last trajectory the arm got is a hold (a single point)."""
    return len(arm._trajectory.messages[-1].points) == 1


@pytest.mark.safety
def test_execute_sends_the_path_and_waits_until_the_arm_is_at_its_end():
    """The plan's path goes to the request's arm, the commit is answered once sent, and it ends with the arm there."""
    (ctx, arm), answers = _driven(), []
    ended = _commit(ctx, answers)
    assert answers == [(True, f"sent to {SERIAL} {ARM}")] and len(arm._trajectory.messages) == 1
    assert np.allclose(ended, GOAL, atol=1e-3)


@pytest.mark.safety
def test_execute_reports_an_arm_that_refuses_the_path():
    """An arm that refuses the trajectory (it is not at the path's start) answers the commit that nothing was sent."""
    (ctx, arm), answers = _driven(start=START + np.radians([10.0, 0, 0, 0, 0, 0])), []
    _commit(ctx, answers)
    (sent, message), = answers
    assert not sent and "refused" in message and not arm._trajectory.messages


@pytest.mark.safety
def test_a_cancelled_commit_holds_the_arm():
    """Cancelled (Stop all, plugin stop, a new commit) while the arm moves: it is held before the lease goes back."""
    (ctx, arm), answers = _driven(), []
    ended = _commit(ctx, answers, cancel_after=5)
    assert _held(arm) and not np.allclose(ended, GOAL, atol=1e-2)
    assert np.allclose(arm.joint_vector(), ended, atol=1e-6)


@pytest.mark.safety
@pytest.mark.parametrize("speed, held", [(0.1, True), (0.25, False)])
def test_a_slow_arm_keeps_the_commit_until_it_is_at_its_end_or_held(speed, held):
    """The pendant's speed slider under 100 %: the commit, and so the lease, lasts until the arm arrives or is held.

    At 10 % it is late, held, and moves no more.
    ! `is_executing` clears during such a move: waiting on it gave the lease back while the arm still moved.
    """
    (ctx, arm), answers = _driven(speed), []
    ended = _commit(ctx, answers)
    assert answers == [(True, f"sent to {SERIAL} {ARM}")] and _held(arm) == held
    if held:
        assert np.allclose(arm.joint_vector(), ended, atol=1e-6)
    else:
        assert np.max(np.abs(ended - GOAL)) < ARRIVED_TOLERANCE


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
