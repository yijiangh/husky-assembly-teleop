"""The base planner's panel: its request, Plan and Commit through the planner's API, and the target checks."""

import asyncio
from dataclasses import asdict
from types import SimpleNamespace

import numpy as np
import pytest
from robot_fakes import lease_test_robots

from husky_assembly_teleop.plugins.base_planner.panel_plugin import NO_TASK, BasePlannerPanel
from husky_assembly_teleop.plugins.base_planner.planner_plugin import BasePlannerPlugin
from husky_assembly_teleop.shared.base_planner_contract import BasePlanRequest
from husky_assembly_teleop.shared.planner.timed_path import TimedPath
from husky_assembly_teleop.shared.planner.planner_search import PlanResult
from husky_assembly_teleop.shared.planner.plan import Plan
from husky_assembly_teleop.shared.planner.planner_search import Search
from husky_assembly_teleop.shared.planner.planner_service import PlannerService

SERIAL = "alice"
GOAL = (1.0, 2.0, 0.5)
PATH = TimedPath(times=np.array([0.0, 1.0]), points=np.array([[0.0, 0.0, 0.0], GOAL]))


class Planner(PlannerService):
    """Answers from `results`, else finds PATH at once; sends every plan and keeps what it was asked."""

    name = "base_planner"

    def __init__(self):
        """Nothing asked or sent yet."""
        super().__init__()
        self.requests, self.sent, self.results = [], [], []

    def make_search(self, ctx, request, abort):
        """Answer the next of `results`, else find PATH."""
        self.requests.append(request)
        result = self.results.pop(0) if self.results else PlanResult(PATH)
        return Search(label="plan", message="planning…", work=lambda: result, time_limit=5.0,
                      accepted=lambda result: ("found", False))

    def stale_reason(self, ctx, plan):
        """Never stale."""
        return ""

    async def execute(self, ctx, plan, answer):
        """Keep the plan; the motion is over at once."""
        self.sent.append(plan.id)
        answer(True, "sent")


def panel_for(api) -> BasePlannerPanel:
    """A panel without its GUI: SERIAL chosen, the target at GOAL, task "passage", Record on, driving `api`."""
    panel = BasePlannerPanel()
    panel._api, panel.serial, panel._has_target = api, SERIAL, True
    panel._used = SimpleNamespace(touch=lambda: None, active=True)
    panel._target = SimpleNamespace(pose=GOAL)
    panel._algorithm = SimpleNamespace(value=panel._algorithms[1].label)
    panel._debug_on, panel._task = SimpleNamespace(value=True), SimpleNamespace(value="passage")
    panel._recording, panel._experiment = SimpleNamespace(value=True), SimpleNamespace(value="trial")
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


def test_request_carries_the_panels_inputs():
    """Robot, target, algorithm with its settings, debug, task and recording go into the request."""
    panel = panel_for(None)
    request = panel.make_request(None)
    assert request == BasePlanRequest(SERIAL, GOAL, panel._algorithms[1].label,
                                      tuple(asdict(panel._algorithms[1].settings()).items()), debug=True,
                                      task="passage", record=True, experiment="trial")
    panel._task = SimpleNamespace(value=NO_TASK)
    assert panel.make_request(None).task is None


def test_request_settings_reach_the_planner_unchanged():
    """The planner turns the panel's settings back into the settings the panel's algorithm shows."""
    panel = panel_for(None)
    algorithm = panel._algorithms[1]
    algorithm._widgets = tuple(SimpleNamespace(value=value) for value in ((-0.2, 0.05), "ideal", 0.3, False, 0.1))
    _, settings = BasePlannerPlugin()._algorithm(panel.make_request(None))
    assert settings == algorithm.settings() and settings.lead == 0.3 and not settings.arcs


@pytest.mark.parametrize("target, change", [(GOAL, ""), ((1.1, 2.0, 0.5), "target changed"),
                                            ((1.0, 2.0, 0.6), "target changed")])
def test_target_change_compares_the_target_with_the_plans_goal(target, change):
    """10 cm or about 6° from the plan's goal is a change; the same pose is none."""
    panel = panel_for(None)
    panel._target = SimpleNamespace(pose=target)
    said = panel.target_change(None, Plan(1, BasePlanRequest(SERIAL, GOAL), "ready", PATH))
    assert change in said if change else said == ""


@pytest.mark.safety
def test_the_panel_plans_and_commits_through_the_api():
    """Plan sends the panel's request; Commit refuses when the target moved, else the planner sends the plan."""
    planner, ctx = Planner(), SimpleNamespace(submit=lambda _label, run: run(), now=lambda: 0.0,
                                              spawn=lambda _label, work: asyncio.get_running_loop().create_task(work),
                                              log_info=lambda _text: None, lease_robots=lease_test_robots,
                                              world=SimpleNamespace(robots={}))
    planner.setup(ctx)

    async def scenario():
        panel = panel_for(planner)
        panel._on_path(ctx, "Plan")
        await asyncio.sleep(0.05)
        panel.update(ctx)
        found = panel._message
        panel._target = SimpleNamespace(pose=(2.0, 2.0, 0.5))
        panel._commit(ctx)
        refused = panel._message
        panel._target = SimpleNamespace(pose=GOAL)
        panel._commit(ctx)
        await asyncio.sleep(0.01)
        return found, refused, panel._message

    try:
        found, refused, committed = asyncio.run(scenario())
    finally:
        planner.teardown(ctx)
    assert found == "found" and "target changed" in refused and committed == "sent"
    assert planner.requests == [panel_for(None).make_request(None)] and planner.sent == [planner.plan.id]
    assert planner.plan.state == "sent"


def test_a_plan_refused_for_a_collision_shows_its_hit_until_the_next_plan():
    """A target in collision lists its hit in the collisions panel, ticked; the next plan found removes it."""
    hit = SimpleNamespace(where="target")
    planner, ctx = Planner(), SimpleNamespace(submit=lambda _label, run: run(), now=lambda: 0.0,
                                              spawn=lambda _label, work: asyncio.get_running_loop().create_task(work),
                                              log_info=lambda _text: None, world=SimpleNamespace(robots={}))
    planner.results = [PlanResult(None, "target is in collision with the wall", hit=hit)]
    planner.setup(ctx)

    async def scenario():
        panel, shown = panel_for(planner), []
        for _ in range(2):
            panel._on_path(ctx, "Plan")
            await asyncio.sleep(0.05)
            panel.update(ctx)
            shown.append(dict(panel._collisions.shown))
        return shown

    try:
        refused, found = asyncio.run(scenario())
    finally:
        planner.teardown(ctx)
    label = f"{SERIAL} target"
    assert refused == {"base_planner_panel": ({label: hit}, (label,))} and found == {}


@pytest.mark.parametrize("park", [(1.0, -2.0, 0.5), None])
def test_park_loads_the_robots_parked_pose(park):
    """Load → Park targets the robot's parked pose, or says it has none."""
    panel, targets, said = BasePlannerPanel(), [], []
    panel.serial, panel._used = SERIAL, SimpleNamespace(touch=lambda: None)
    panel._target = SimpleNamespace(set=lambda *pose: targets.append(pose))
    panel._say = lambda message, failed=False: said.append(failed)
    ctx = SimpleNamespace(config=SimpleNamespace(robots=[SimpleNamespace(serial=SERIAL, park_pose=park)]))
    panel._load_target(ctx, "Park")
    assert targets == ([park] if park else []) and said == [park is None]
