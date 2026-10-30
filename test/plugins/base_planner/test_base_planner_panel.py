"""The base planner's panel: its request, Plan and Commit through the planner's API, and the target checks."""

import asyncio
from dataclasses import asdict, replace
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
import pytest
from plugin_fakes import FakeGui
from robot_fakes import lease_test_robots

from husky_assembly_teleop.plugins.base_planner.panel_plugin import NO_TASK, BasePlannerPanel
from husky_assembly_teleop.plugins.base_planner.planner_plugin import BasePlannerPlugin
from husky_assembly_teleop.shared.base_planner_contract import BasePlanRequest
from husky_assembly_teleop.ui.ghost import KIND_COLORS
from husky_assembly_teleop.ui.style import FAIL, NONE
from husky_assembly_teleop.shared.planner.timed_path import TimedPath
from husky_assembly_teleop.shared.planner.planner_search import PlanResult
from husky_assembly_teleop.shared.planner.plan import Plan
from husky_assembly_teleop.shared.planner.planner_search import Search
from husky_assembly_teleop.shared.planner.planner_service import PlannerService
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

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
    """A panel without its GUI: SERIAL chosen, target at GOAL, margin 7 cm, task "passage", Record on, driving `api`."""
    panel = BasePlannerPanel()
    panel._api, panel.serial, panel._has_target = api, SERIAL, True
    panel._used = SimpleNamespace(touch=lambda: None, active=True)
    panel._target, panel._margin = SimpleNamespace(pose=GOAL), SimpleNamespace(value=0.07)
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


def test_target_ghost_hides_once_the_robot_stands_at_the_target():
    """Away from the target the ghost shows there; within 5 cm / 3° of it, it hides instead of covering the robot."""
    assert KIND_COLORS[BasePlannerPanel.target_kind] != KIND_COLORS["target"], "orange, apart from the process blue"
    panel = panel_for(None)
    shown = []
    ghost = SimpleNamespace(show=lambda pose, joints: shown.append(pose), hide=lambda: shown.append(None))
    pose = SimpleNamespace(position=(0.0, 0.0, 0.1))
    ctx = SimpleNamespace(kinematics=SimpleNamespace(base_pose=lambda serial: pose, joints=lambda serial: {}))
    with patch("husky_assembly_teleop.plugins.base_planner.panel_plugin.floor_pose", return_value=(0.0, 0.0, 0.0)):
        panel.show_target(ctx, ghost)
    with patch("husky_assembly_teleop.plugins.base_planner.panel_plugin.floor_pose",
               return_value=(GOAL[0] + 0.03, GOAL[1], GOAL[2])):
        panel.show_target(ctx, ghost)
    assert shown[0] is not None and tuple(shown[0].position[:2]) == GOAL[:2] and shown[1] is None


def test_request_carries_the_panels_inputs():
    """Robot, target, algorithm settings plus the panel's margin, debug, task and recording are in the request."""
    panel = panel_for(None)
    request = panel.make_request(None)
    settings = {**asdict(panel._algorithms[1].settings()), "margin": 0.07}
    assert request == BasePlanRequest(SERIAL, GOAL, panel._algorithms[1].label, tuple(settings.items()), debug=True,
                                      task="passage", record=True, experiment="trial")
    panel._task = SimpleNamespace(value=NO_TASK)
    assert panel.make_request(None).task is None


def test_request_settings_reach_the_planner_unchanged():
    """The planner turns the panel's settings back into the settings the panel's algorithm shows."""
    panel = panel_for(None)
    algorithm = panel._algorithms[1]
    algorithm._widgets = tuple(SimpleNamespace(value=value) for value in (0.08, "ideal", 0.3, False))
    _, settings = BasePlannerPlugin()._algorithm(panel.make_request(None))
    assert settings == replace(algorithm.settings(), margin=0.07) and settings.lead == 0.3 and not settings.arcs


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
                                              world=SimpleNamespace(robots={}), config=SimpleNamespace(robots=()),
                                              scene=SimpleNamespace(snapshot=Scene()))
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


@pytest.mark.safety
def test_commits_second_press_lets_the_robots_own_arms_be_unknown():
    """The robot's arms not measured: the first Commit only asks; the second sends, letting the robot be unknown."""
    model = SimpleNamespace(movable_joints=("ur_arm_shoulder_pan_joint",))
    robot = RobotObject(robot_id(SERIAL), model, Pose(), {}, unmeasured=frozenset(model.movable_joints))
    planner, ctx = Planner(), SimpleNamespace(submit=lambda _label, run: run(), now=lambda: 0.0,
                                              spawn=lambda _label, work: asyncio.get_running_loop().create_task(work),
                                              log_info=lambda _text: None, lease_robots=lease_test_robots,
                                              world=SimpleNamespace(robots={}), config=SimpleNamespace(robots=()),
                                              scene=SimpleNamespace(snapshot=Scene(robots={robot.id: robot})))
    planner.setup(ctx)

    async def scenario():
        panel = panel_for(planner)
        panel._on_path(ctx, "Plan")
        await asyncio.sleep(0.05)
        panel._commit(ctx)
        asked, sent_after_one = panel._message, list(planner.sent)
        panel._commit(ctx)
        await asyncio.sleep(0.01)
        return asked, sent_after_one

    try:
        asked, sent_after_one = asyncio.run(scenario())
    finally:
        planner.teardown(ctx)
    assert "press Commit again" in asked and "ur_arm not measured" in asked and sent_after_one == []
    assert planner.sent == [planner.plan.id] and planner.unknown_allowed(planner.plan) == {robot.id}


def test_a_plan_refused_for_a_collision_shows_its_hit_until_the_next_plan():
    """A target in collision lists its hit in the collisions panel, ticked; the next plan found removes it."""
    hit = SimpleNamespace(where="target")
    planner, ctx = Planner(), SimpleNamespace(submit=lambda _label, run: run(), now=lambda: 0.0,
                                              spawn=lambda _label, work: asyncio.get_running_loop().create_task(work),
                                              log_info=lambda _text: None, log_warn=lambda _text: None,
                                              world=SimpleNamespace(robots={}))
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
    panel._say = lambda message, color=NONE: said.append(color == FAIL)
    ctx = SimpleNamespace(config=SimpleNamespace(robots=[SimpleNamespace(serial=SERIAL, park_pose=park)]))
    panel._load_target(ctx, "Park")
    assert targets == ([park] if park else []) and said == [park is None]


def test_the_preview_slider_follows_only_the_operator():
    """Draw writes the Preview time slider every tick, and viser calls back for that too: only a drag scrubs."""
    asked = {}

    def defer_value(label, run, operator_only=False):
        asked[label] = operator_only
        return run

    ctx = SimpleNamespace(defer_value=defer_value, defer=lambda _label, run, operator_only=False: run)
    BasePlannerPanel()._add_path_controls(ctx, FakeGui(), "plan hint", "commit hint")
    assert asked["scrub"] is True


@pytest.mark.parametrize("pan, warned", [(0.03, False), (0.7, True)])
def test_an_arm_out_of_stow_asks_commit_twice_and_shows_an_amber_chip(pan, warned):
    """Within 5 degrees of stow nothing is said; further out, Commit names the arm and the chip line warns."""
    joints = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
              "wrist_3_joint")
    robot = RobotObject(robot_id(SERIAL), SimpleNamespace(movable_joints=tuple(f"ur_arm_{name}" for name in joints)),
                        Pose(), {"ur_arm_shoulder_pan_joint": pan})
    ctx = SimpleNamespace(config=SimpleNamespace(robots=[SimpleNamespace(serial=SERIAL, arms=[
                              SimpleNamespace(name="ur_arm", stow_joints=(0.0,) * 6)])]),
                          scene=SimpleNamespace(snapshot=Scene(robots={robot.id: robot})))
    panel = panel_for(None)
    said = panel.commit_warnings(ctx, Plan(1, BasePlanRequest(SERIAL, GOAL), "ready", PATH))
    assert said == ([f"the arm {np.degrees(pan):.0f}° from stow"] if warned else [])


@pytest.mark.parametrize("tracked", [True, False])
def test_the_robots_own_arms_not_measured_ask_commit_twice(tracked):
    """Arms with no joint states: Commit names them and how they are checked; not with the base untracked (refused)."""
    model = SimpleNamespace(movable_joints=("ur_arm_shoulder_pan_joint",))
    robot = RobotObject(robot_id(SERIAL), model, Pose(), {}, base_tracked=tracked,
                        unmeasured=frozenset(model.movable_joints))
    ctx = SimpleNamespace(config=SimpleNamespace(robots=[SimpleNamespace(serial=SERIAL, arms=[])]),
                          scene=SimpleNamespace(snapshot=Scene(robots={robot.id: robot})))
    panel, plan = panel_for(None), Plan(1, BasePlanRequest(SERIAL, GOAL), "ready", PATH)
    said = panel.commit_warnings(ctx, plan)
    assert said == (["ur_arm not measured: checked where last seen, else stowed"] if tracked else [])
    assert panel.commit_unknown(ctx, plan) == ({robot.id} if tracked else set()), "the second press lets them be"
