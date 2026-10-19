"""The arm planner as a headless planner (staleness, send) and its panel driving it through the API."""

from __future__ import annotations

import asyncio
import math
from types import SimpleNamespace

import numpy as np
import pytest

from husky_assembly_teleop.plugins.arm_planner import plugin as arm_plugin
from husky_assembly_teleop.plugins.arm_planner.panel import ArmPlannerPanel
from husky_assembly_teleop.plugins.arm_planner.planner import ArmPath, arm_joint_names
from husky_assembly_teleop.plugins.arm_planner.plugin import ArmPlannerPlugin
from husky_assembly_teleop.shared.planning.arm import ArmPlanRequest
from husky_assembly_teleop.shared.planning.search import PlanResult
from husky_assembly_teleop.shared.planning.service import Plan, PlannerPluginApi
from scaffolding_assembly_core.ids import robot_id

SERIAL, ARM = "0806", "left_ur_arm"
NAMES = arm_joint_names(ARM)
START = np.array([0.0, -1.5, 1.5, 0.0, 0.0, 0.0])
GOAL = START + np.array([0.4, 0.0, 0.0, 0.0, 0.0, 0.0])
PATH = ArmPath.at_preview_speed(NAMES, np.array([START, GOAL]))
REQUEST = ArmPlanRequest(SERIAL, ARM, tuple(GOAL))


class FakeCtx:
    """A plugin context: the arm at `joints`, the base at `base`, intents run at once, an arm that records its path."""

    def __init__(self):
        """The arm at START, the base at the origin, and an arm that accepts."""
        self.joints, self.base, self.sent = dict(zip(NAMES, START)), (0.0, 0.0, 0.0), []
        arm = SimpleNamespace(send_joint_trajectory=lambda positions, duration, velocities: self.sent.append(positions)
                              or True)
        self.world = SimpleNamespace(robots={SERIAL: SimpleNamespace(arms={ARM: arm})})
        robot = SimpleNamespace(enabled=False, acting_problems=lambda: [], base=SimpleNamespace(position=(0, 0, 0)))
        self.scene = SimpleNamespace(snapshot=SimpleNamespace(robots={robot_id(SERIAL): robot}))
        self.kinematics = SimpleNamespace(joints=lambda _serial: self.joints,
                                          base_pose=lambda _serial: SimpleNamespace(position=self.base))

    def submit(self, _label, run):
        """Run the intent now."""
        run()

    def spawn(self, _label, work):
        """Start the task."""
        return asyncio.get_running_loop().create_task(work)

    def now(self):
        """A fixed clock."""
        return 0.0

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
    """Other plugins can rely on the planner API (`ArmPlannerPluginApi` is it, typed with the arm request)."""
    assert isinstance(ArmPlannerPlugin(), PlannerPluginApi)


def test_a_plan_is_stale_once_the_arm_or_base_moves(found_path):
    """The arm moving past 2 degrees or the base past 5 cm since planning marks the plan; the start is fine."""
    ctx = FakeCtx()
    planner = make_planner(ctx)

    async def scenario():
        plan = await planner.request_plan(REQUEST)
        return plan, planner.stale_reason(ctx, plan)

    plan, at_start = run(planner, ctx, scenario)
    assert plan.state == "ready" and at_start == ""
    ctx.joints = {**ctx.joints, NAMES[0]: math.radians(3.0)}
    assert planner.stale_reason(ctx, plan) == "the arm moved since planning"
    ctx.joints = dict(zip(NAMES, START))
    ctx.base = (0.1, 0.0, 0.0)
    assert planner.stale_reason(ctx, plan) == "the base moved since planning"


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
def test_send_gives_the_path_to_the_arm():
    """Send puts the plan's path onto the request's arm; an arm that refuses is reported."""
    ctx = FakeCtx()
    planner = ArmPlannerPlugin()
    assert planner.send(ctx, ready_plan()) == (True, f"sent to {SERIAL} {ARM}")
    assert np.allclose(ctx.sent[0][0], START) and np.allclose(ctx.sent[0][-1], GOAL)
    ctx.world.robots[SERIAL].arms[ARM].send_joint_trajectory = lambda *args: False
    sent, message = planner.send(ctx, ready_plan())
    assert not sent and "refused" in message


def make_panel(planner: ArmPlannerPlugin) -> ArmPlannerPanel:
    """A panel on `planner` with six sliders at the goal, as the operator would have set them."""
    panel = ArmPlannerPanel()
    panel._api = planner
    panel.serial, panel.arm, panel._has_target, panel._load_now = SERIAL, ARM, True, False
    panel._sliders = [SimpleNamespace(value=math.degrees(value)) for value in GOAL]
    panel._used = SimpleNamespace(touch=lambda: None, active=True)
    return panel


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
    assert found.startswith("RRT: 2 waypoints") and "target changed" in refused
    assert committed == f"sent to {SERIAL} {ARM}" and planner.plan.state == "sent"
    assert planner.plan.request == ArmPlanRequest(SERIAL, ARM, pytest.approx(tuple(GOAL))) and len(ctx.sent) == 1
