"""The step panel on the purged process: walking the steps, planning, Exec, and the goal check.

The quick tests plan through a fake planner and Exec through a fake executor; the goal check's come from the old
goal_check plugin. The slow test walks B20_J and B21_H with the real motion planner and executor, on fake robots.
"""

from __future__ import annotations

import asyncio
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, design_scene, purged_process, robot_config, step
from robot_lab import FakeRun, RobotLab

from husky_assembly_teleop.plugins.assembly_steps.panel import StepsPanel
from husky_assembly_teleop.plugins.assembly_steps.plugin import AssemblyStepsPlugin
from husky_assembly_teleop.plugins.motion_planner.plugin import MotionPlannerPlugin
from husky_assembly_teleop.plugins.movement_exec.plugin import MovementExecPlugin
from husky_assembly_teleop.shared.movement_exec_contract import CANCELLED, DONE, DRY_RUN, ExecResult, serial_of
from husky_assembly_teleop.shared.assembly_planning.ik import IkResult
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.assembly_planning.movement import Routine
from husky_assembly_teleop.shared.assembly_planning.route_result import GoalCheck, RouteResult, goal_line
from husky_assembly_teleop.shared.assembly_planning.scene_for import RobotBelief
from husky_assembly_teleop.ui.ghost import RecentUse
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.process.solutions import MovementResult, Trajectory
from scaffolding_assembly_core.urdf import UR_JOINT_NAMES

NAMES = [f"{arm}_{joint}" for arm in ("left_ur_arm", "right_ur_arm") for joint in UR_JOINT_NAMES]
RAW = dict(zip(NAMES, [0.1 * index for index in range(12)]))
AUTHORED = {**RAW, "right_ur_arm_elbow_joint": RAW["right_ur_arm_elbow_joint"] + 0.0015}
BASE = Pose((1.0, -2.5, 0.0), (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)))


def cindy_check(ok: bool = True) -> GoalCheck:
    """A check of Cindy's transfer, IK 0.0015 rad off the authored goal at one joint."""
    pair = ("robots/a200-0806/base_link", "process/bars/B20")
    ik = IkResult(dict(RAW), 0.0015, 1, 64, "ssik", ("left", "right"), pair)
    return GoalCheck("B20_J_transfer", Routine.TRANSFER_FREE, ok, "" if ok else "no IK", "robots/a200-0806", BASE,
                     dict(RAW), ik, dict(RAW), AUTHORED, {"robots/a200-0804": "live"}, ("a warning",), 1.5)


class FakePlanner:
    """The motion planner: plans move each acting arm's first joint by 0.1 rad; goals are `check`."""

    def __init__(self, lab: RobotLab):
        """No request yet."""
        self.lab, self.plans, self.goals, self.check = lab, [], [], None

    def request_plan(self, request):
        """Answer at once from the snapshot's start."""
        self.plans.append(request)
        serial = serial_of(request.loaded, request.step.action.id, self.lab.configs)
        robot = request.snapshot.robots[robot_id(serial)]
        names = [name for arm in self.lab.robots[serial].arms for name in arm_joint_names(arm)]
        start = np.array([robot.joints[name] for name in names])
        goal = start.copy()
        goal[::6] += 0.1
        trajectory = Trajectory(request.step.action.robot, tuple(names), (tuple(start), tuple((start + goal) / 2),
                                                                          tuple(goal)))
        return _answered(RouteResult(MovementResult("solved", trajectory=trajectory), Routine.FREE_HOME,
                                     planner="fake", waypoints=3))

    def request_ik(self, request):
        """Answer with `check`."""
        self.goals.append(request)
        return _answered(self.check)


class FakeExecutor:
    """movement_exec: keeps each request; the test resolves its future."""

    active = ""

    def __init__(self):
        """No request yet."""
        self.requests, self.futures = [], []

    def request_execute(self, request):
        """Keep the request; answer later."""
        self.requests.append(request)
        self.futures.append(asyncio.get_running_loop().create_future())
        return self.futures[-1]


def _answered(value):
    """A future already holding `value`."""
    future = asyncio.get_running_loop().create_future()
    future.set_result(value)
    return future


class Steps(RobotLab):
    """The robots live, the run at `current`, the step panel on a fake planner and executor.

    The process plugin is only its "Rhino obstacles collide" box, ticked; a test flips `process.obstacles_collide`.
    """

    def __init__(self, current, transfer_as_free: bool = True):
        """Build everything and take the current step."""
        super().__init__()
        self.loaded = purged_process()
        self.run = FakeRun(self.loaded, current, transfer_as_free)
        self.planner, self.executor = FakePlanner(self), FakeExecutor()
        self.process = SimpleNamespace(obstacles_collide=True)
        self.ctx.plugins = {"assembly_run": self.run, "motion_planner": self.planner, "movement_exec": self.executor,
                            "process": self.process}
        self.ctx.kinematics = SimpleNamespace(base_pose=lambda serial: self.bases[serial],
                                              joints=lambda serial: {})
        self.plugin = AssemblyStepsPlugin()
        self.plugin._ctx = self.ctx
        self.plugin._panel = StepsPanel(self.ctx, self.plugin)
        self.plugin._used = RecentUse(0.0)
        self.plugin._target_ghosts, self.plugin._path_ghosts = {}, {}
        self.update()

    def update(self) -> None:
        """One tick of the plugin: update and draw."""
        self.plugin.update(self.ctx)
        self.plugin.draw(self.ctx)

    def html(self) -> str:
        """Every line the panel shows."""
        return " ".join(widget.content for widget in self.ctx.gui.of_kind("html"))

    async def plan(self) -> None:
        """Plan, and wait for the answer."""
        self.plugin._on_path(self.ctx, "Plan")
        if self.plugin._plan_task is not None:
            await self.plugin._plan_task


def test_steps_follow_the_run_and_walk_with_prev_and_next():
    """B20_J (workaround): place and grasp after the transfer, the load skipped; Next, Prev; B20_R's ungrasp.

    The insert is refused: not supported yet.
    """
    lab = Steps(("B20_J", 3))
    assert lab.plugin.position == (1, 2) and lab.plugin.traj_time == 10.0
    assert lab.plugin._panel._status.content.count('class="chip-line"') == 1
    assert "then: B20_J_place_bar, B20_J_grasp · skipped: B20_J_load" in lab.html()
    assert "moves left_ur_arm, right_ur_arm; then waits for B20_J_place_bar by hand; then tightens" in lab.html()
    lab.plugin.go(lab.ctx, 1)
    lab.update()
    assert lab.run.selected == [("B20_J", 4)] and lab.plugin.position == (2, 2)
    assert "refuses B20_J_insert: the insert is not supported yet" in lab.html()
    lab.plugin.go(lab.ctx, 1)
    assert "no step there" in lab.plugin._message
    lab.plugin.go(lab.ctx, -1)
    assert lab.run.selected[-1] == ("B20_J", 3)
    lab = Steps(("B20_R", 1))
    assert "with: B20_R_ungrasp" in lab.html()


def test_toggling_the_workaround_moves_to_the_steps_primary():
    """B20_J at its travel to load without the workaround; turning it on makes the transfer current."""
    lab = Steps(("B20_J", 0), transfer_as_free=False)
    lab.plugin.set_workaround(lab.ctx, True)
    lab.update()
    assert lab.run.selected == [("B20_J", 3)]


def test_a_refused_workaround_change_puts_the_box_back():
    """The run refuses the change (e.g. a step runs): the box shows the run's value again, and says so."""
    lab = Steps(("B20_J", 3))
    lab.run.request_settings = lambda **changes: None
    box, = [widget for widget in lab.ctx.gui.of_kind("checkbox") if widget.label == "Transfer workaround"]
    box.value = False
    lab.plugin.set_workaround(lab.ctx, False)
    lab.update()
    assert box.value is True and "Transfer workaround refused" in lab.html()


def test_a_plan_request_carries_the_run_and_the_checked_goal():
    """Alice's approach: the run's beliefs and workaround, Ignore built bars, and the goal checked at this base.

    A base moved 3 mm since the check solves the goal again.
    """
    lab = Steps(("B21_H", 0))
    lab.run.view.beliefs = {"robots/a200-0805": RobotBelief(Pose())}
    lab.plugin.set_ignore_built(True)
    goal = dict(zip(arm_joint_names("ur_arm"), [1.0, -1.0, 1.0, -1.0, 1.0, 0.0]))
    lab.planner.check = GoalCheck("B21_H_approach", Routine.SUPPORT_APPROACH, True, base=lab.bases["a200-0804"],
                                  goal_joints=goal, ik=IkResult(goal, 0.0, 1, 8, "ssik", ("alice",), None))

    async def scenario():
        await lab.plugin.checked_goal(lab.ctx)
        await lab.plan()
        lab.bases["a200-0804"] = Pose((0.003, 0.0, 0.0))
        lab.tick(lab.ctx.t)
        await lab.plan()

    asyncio.run(scenario())
    first, second = lab.planner.plans
    assert first.beliefs == lab.run.view.beliefs and first.transfer_as_free and first.ignore_built
    assert first.goal_joints == goal and second.goal_joints is None
    assert lab.plugin.traj_time == 15.0 and lab.plugin.plan_state == "ready"
    assert [event for event, _fields in lab.run.records] == ["check_goal", "plan", "plan"]


def test_goal_and_plan_carry_the_rhino_obstacles_box_and_a_goal_checked_otherwise_is_not_reused():
    """Alice's approach, the box unticked: the goal check and the plan carry it, and the plan reuses the goal.

    Ticked, the goal checked unticked is neither shown nor reused; unticked again, it is.
    """
    lab = Steps(("B21_H", 0))
    lab.process.obstacles_collide = False
    goal = dict(zip(arm_joint_names("ur_arm"), [1.0, -1.0, 1.0, -1.0, 1.0, 0.0]))
    lab.planner.check = GoalCheck("B21_H_approach", Routine.SUPPORT_APPROACH, True, base=lab.bases["a200-0804"],
                                  goal_joints=goal, ik=IkResult(goal, 0.0, 1, 8, "ssik", ("alice",), None))
    shown = []

    async def scenario():
        await lab.plugin.checked_goal(lab.ctx)
        await lab.plan()
        lab.process.obstacles_collide = True
        shown.append(lab.plugin.goal)
        await lab.plan()
        lab.process.obstacles_collide = False
        shown.append(lab.plugin.goal)
        await lab.plan()

    asyncio.run(scenario())
    check, = lab.planner.goals
    first, ticked, unticked = lab.planner.plans
    assert not check.obstacles_collide and not first.obstacles_collide and not lab.plugin._basis.obstacles_collide
    assert ticked.obstacles_collide and not unticked.obstacles_collide
    assert shown == [None, lab.planner.check]
    assert first.goal_joints == goal and ticked.goal_joints is None and unticked.goal_joints == goal
    assert [(event, fields["data"]["obstacles_collide"]) for event, fields in lab.run.records] == [
        ("check_goal", False), ("plan", False), ("plan", True), ("plan", False)]


@pytest.mark.safety
def test_the_basis_keeps_the_box_the_plan_was_asked_with():
    """Ticked while the plan is on its way: the plan was asked unticked, so it is stale as soon as it is ready."""
    lab = Steps(("B20_R", 2))
    lab.process.obstacles_collide = False
    answers, pending = [], []

    def held_back(request):
        """The fake planner's answer, kept back until the test gives it."""
        answers.append(FakePlanner.request_plan(lab.planner, request).result())
        pending.append(asyncio.get_running_loop().create_future())
        return pending[-1]

    lab.planner.request_plan = held_back

    async def scenario():
        lab.plugin._on_path(lab.ctx, "Plan")
        while not pending:
            await asyncio.sleep(0)
        lab.process.obstacles_collide = True
        pending[0].set_result(answers[0])
        await lab.plugin._plan_task

    asyncio.run(scenario())
    assert not lab.planner.plans[0].obstacles_collide and not lab.plugin._basis.obstacles_collide
    lab.update()
    assert lab.plugin.plan_state == "stale" and "the Rhino obstacles were switched on since planning" in lab.html()


@pytest.mark.safety
def test_switching_the_rhino_obstacles_makes_the_plan_stale_on_the_panel():
    """A plan made with the box unticked goes stale once it is ticked, the reason on the panel."""
    lab = Steps(("B20_R", 2))
    lab.process.obstacles_collide = False
    asyncio.run(lab.plan())
    assert lab.plugin.plan_state == "ready" and not lab.plugin._basis.obstacles_collide
    lab.process.obstacles_collide = True
    lab.update()
    assert lab.plugin.plan_state == "stale" and "the Rhino obstacles were switched on since planning" in lab.html()


@pytest.mark.safety
def test_no_plan_is_asked_while_the_robot_is_not_live():
    """Cindy's joints not fresh: Plan asks nothing and says why."""
    lab = Steps(("B20_R", 2))
    lab.live = False
    lab.tick(lab.ctx.t)
    asyncio.run(lab.plan())
    assert lab.planner.plans == [] and lab.plugin.plan_state == "none" and "is not live" in lab.plugin._message


@pytest.mark.safety
def test_a_plan_goes_stale_on_the_panel():
    """Cindy's base moving 3 mm after planning makes the plan stale, the reason on the panel."""
    lab = Steps(("B20_R", 2))
    asyncio.run(lab.plan())
    assert lab.plugin.plan_state == "ready" and not math.isnan(lab.plugin.minimum)
    lab.bases["a200-0806"] = Pose((3.003, 1.0, 0.0))
    lab.tick(lab.ctx.t)
    lab.update()
    assert lab.plugin.plan_state == "stale" and "base moved 3.0 mm" in lab.html()


def test_exec_moves_on_to_the_next_step():
    """Belle's open: Exec asks without a plan; once done, the next step (to grasp) is current."""
    lab = Steps(("B35_H", 1))

    async def scenario():
        lab.plugin._commit(lab.ctx)
        request, = lab.executor.requests
        assert [part.kind for part in request.parts] == ["robotiq_open"] and request.route is None
        lab.executor.futures[0].set_result(ExecResult(DONE, "every part done", (1,)))
        await asyncio.sleep(0)

    asyncio.run(scenario())
    assert lab.run.selected == [("B35_H", 2)]


def test_the_last_step_offers_mark_done():
    """Belle's close, the action's last step, done: Mark B35_H done is offered and marks it in the run."""
    lab = Steps(("B35_H", 3))

    async def scenario():
        lab.plugin._commit(lab.ctx)
        lab.executor.futures[0].set_result(ExecResult(DONE, "every part done", (3,)))
        await asyncio.sleep(0)

    asyncio.run(scenario())
    lab.update()
    assert lab.run.selected == [] and lab.plugin.offer == "B35_H"
    assert any(widget.label == "Mark B35_H done" for widget in lab.ctx.gui.of_kind("button"))
    lab.plugin.mark_done(lab.ctx)
    assert lab.run.marked == ["B35_H"]


@pytest.mark.safety
def test_exec_needs_a_ready_plan_and_uses_it_once():
    """No plan: nothing to Exec. A plan goes with Exec once; a dry run gives it back, a real run clears it."""
    lab = Steps(("B20_R", 2))

    async def scenario():
        lab.plugin._commit(lab.ctx)
        assert lab.executor.requests == [] and "Plan first" in lab.plugin._message
        await lab.plan()
        lab.plugin._commit(lab.ctx)
        request, = lab.executor.requests
        assert request.route is lab.plugin.route and request.basis.serial == "a200-0806"
        assert request.route_movement == 2 and request.traj_time == lab.plugin.traj_time
        assert lab.plugin.plan_state == "sent"
        lab.plugin._commit(lab.ctx)
        assert len(lab.executor.requests) == 1 and "Exec refused" in lab.plugin._message
        lab.executor.futures[0].set_result(ExecResult(DRY_RUN, "dry run: every check passed, nothing sent"))
        await asyncio.sleep(0)
        assert lab.run.selected == [] and lab.plugin.plan_state == "ready" and lab.plugin.path is not None
        lab.plugin._commit(lab.ctx)
        lab.executor.futures[1].set_result(ExecResult(CANCELLED, "B20_R_home: cancelled (operator)"))
        await asyncio.sleep(0)

    asyncio.run(scenario())
    assert lab.run.selected == [] and lab.plugin.path is None and lab.plugin.plan_state == "none"


# --- --- --- --- --- the goal check (from the goal_check plugin) --- --- --- --- ---

def test_the_record_is_json():
    """A check's record goes through JSON as it is, the raw gap included."""
    record = json.loads(json.dumps(cindy_check().as_record(), allow_nan=False))
    assert record["routine"] == "transfer_free" and record["base"][:3] == [1.0, -2.5, 0.0]
    assert record["ik"]["colliding"] == ["robots/a200-0806/base_link", "process/bars/B20"]
    assert abs(record["max_from_authored"] - 0.0015) < 1e-12


def test_the_log_line():
    """The line names the movement, the gap, each arm's goal and the base; a failure its reason."""
    line = goal_line(cindy_check())
    assert line.startswith("B20_J_transfer at the live base: max |goal - authored goal| = 0.0015 rad; goal (rad) left "
                           "[+0.0000, +0.1000, +0.2000, +0.3000, +0.4000, +0.5000] right [+0.6000, ")
    assert line.endswith("; base 1.000 -2.500 m 90.0 deg")
    assert goal_line(cindy_check(ok=False)) == "B20_J_transfer: no goal at the live base: no IK"


def test_check_goal_asks_with_the_run_and_records_the_answer():
    """The request carries the run's beliefs, workaround and the tick's snapshot; the answer is logged and recorded.

    It shows as the target ghost while its step is current.
    """
    lab = Steps(("B20_J", 3))
    lab.run.view.beliefs = {"robots/a200-0805": RobotBelief(Pose())}
    lab.planner.check = cindy_check()
    answer = asyncio.run(lab.plugin.checked_goal(lab.ctx))
    request, = lab.planner.goals
    assert request.loaded is lab.loaded and request.step.movement.id == "B20_J_transfer"
    assert request.snapshot is lab.ctx.scene.snapshot and request.beliefs == lab.run.view.beliefs
    assert request.transfer_as_free and request.label == "check goal B20_J_transfer"
    (event, fields), = lab.run.records
    assert event == "check_goal" and (fields["action"], fields["movement"]) == ("B20_J", "B20_J_transfer")
    assert fields["data"] == {**answer.as_record(), "obstacles_collide": True}
    assert goal_line(answer) in lab.ctx.lines("info")
    shown = []
    ghost = SimpleNamespace(show=lambda base, joints, parts=None: shown.append((base, dict(joints))),
                            hide=lambda: shown.append(None))
    lab.plugin.show_target(lab.ctx, ghost)
    assert shown == [(lab.bases["a200-0806"], RAW)]
    lab.plugin.go(lab.ctx, 1)
    lab.update()
    lab.plugin.show_target(lab.ctx, ghost)
    assert shown[-1] is None or shown[-1][1] != RAW


def test_check_goal_refuses_routines_without_an_ik_goal():
    """Check goal on the way home (joints, not IK) says so and asks nothing."""
    lab = Steps(("B20_R", 2))
    lab.plugin.check_goal(lab.ctx)
    assert "only a transfer or an approach" in lab.plugin._message and lab.planner.goals == []


# --- --- --- --- --- the slow walk: the real motion planner and executor --- --- --- --- ---

class Walk(RobotLab):
    """The robots where the design has them at a movement's start, the run there, the real planner and executor.

    Robots absent from the design stand far away.
    """

    def __init__(self, movement: str):
        """Build everything at `movement`'s start."""
        loaded = purged_process()
        scene, at = design_scene(loaded, movement), step(loaded, movement)
        acting = serial_of(loaded, at.action.id, tuple(robot_config(digits) for digits in (ALICE, BELLE, CINDY)))
        joints = {}
        for serial in ("a200-0804", "a200-0805", "a200-0806"):
            robot = scene.robots.get(robot_id(serial))
            if robot is not None:
                arms = ("left_ur_arm", "right_ur_arm") if serial == "a200-0806" else ("ur_arm",)
                joints[serial] = {arm: [robot.joints[name] for name in arm_joint_names(arm)] for arm in arms}
        if at.assumed_from == "own target":
            # * No authored start: the acting robot comes with its arms stowed (as the M3 check plans it).
            config = next(config for config in (robot_config(digits) for digits in (ALICE, BELLE, CINDY))
                          if config.serial == acting)
            joints[acting] = {arm.name: list(arm.stow_joints) for arm in config.arms}
        super().__init__(joints)
        for number, serial in enumerate(self.robots):
            robot = scene.robots.get(robot_id(serial))
            self.bases[serial] = robot.base if robot is not None else Pose((20.0 + 3.0 * number, 20.0, 0.0))
        self.tick(self.ctx.t)
        self.loaded = loaded
        self.run = FakeRun(loaded, (at.action.id, at.movement_index))
        self.planner = MotionPlannerPlugin()
        self.planner._log = self.planner._pool.log = lambda _line: None
        self.executor = MovementExecPlugin()
        # * The Rhino obstacles count, as the walk was planned before their box.
        self.ctx.plugins = {"assembly_run": self.run, "motion_planner": self.planner, "movement_exec": self.executor,
                            "process": SimpleNamespace(obstacles_collide=True)}
        self.executor.setup(self.ctx)
        self.ctx.kinematics = SimpleNamespace(base_pose=lambda serial: self.bases[serial], joints=lambda serial: {})
        self.steps = AssemblyStepsPlugin()
        self.steps._ctx = self.ctx
        self.steps._panel = StepsPanel(self.ctx, self.steps)
        self.steps._used = RecentUse(0.0)
        self.steps._target_ghosts, self.steps._path_ghosts = {}, {}
        self.steps.update(self.ctx)

    async def settle(self, seconds: float = 2.5) -> None:
        """Let ROS time pass, the robots still (a lease needs every arm still for 2 s)."""
        until = self.ctx.t + seconds
        while self.ctx.t < until:
            await self.ctx.next_tick()

    async def run_step(self, prompts: int) -> None:
        """Plan the step's motion if it has one, Exec, Confirm every prompt; the step panel follows the run after."""
        steps = self.steps
        if any(part.kind == "motion" for part in steps.parts()):
            steps._on_path(self.ctx, "Plan")
            await steps._plan_task
            assert steps.plan_state == "ready", steps._message
        steps._commit(self.ctx)
        for _ in range(prompts):
            while self.executor.prompt is None:
                assert self.executor.active, f"no prompt: {self.executor._last}"
                await asyncio.sleep(0)
            prompt = self.executor.prompt
            while self.ctx.t - prompt.opened_at < 0.6:
                await self.ctx.next_tick()
            self.executor.request_confirm()
            while self.executor.prompt is prompt:
                await asyncio.sleep(0)
        while steps._executing is not None:
            await asyncio.sleep(0.01)
        await self.settle()
        steps.update(self.ctx)


@pytest.mark.slow
def test_walking_b21_h_and_b20_j_with_the_real_planner_and_executor():
    """B21_H and B20_J's transfer step, planned for real, run on fake robots that follow, every part recorded.

    Alice's B21_H: approach (planned), open, to grasp (planned), close, then Mark done offered. Cindy's B20_J transfer
    step with the workaround: the free transfer (planned), placing the bar by hand, the grasp; then the insert,
    refused as not supported yet. Cindy ends where the transfer's plan ends.
    """
    walk = Walk("B21_H_approach")

    async def b21():
        for prompts in (1, 1, 1, 1):
            await walk.run_step(prompts)
        return walk.steps.offer

    try:
        offer = asyncio.run(b21())
    finally:
        walk.planner.teardown(None)
    files = walk.run.files()
    assert offer == "B21_H", walk.steps._message
    assert [file["movement"] for file in files] == ["B21_H_approach", "B21_H_open", "B21_H_to_grasp", "B21_H_close"]
    assert [file["outcome"]["status"] for file in files] == ["ARRIVED", "reached", "ARRIVED", "reached"]
    assert files[2]["plan"]["planner"].startswith("single_arm.plan_linear")

    walk = Walk("B20_J_transfer")

    async def b20():
        await walk.run_step(3)
        return walk.steps.position, walk.steps.parts()

    try:
        position, parts = asyncio.run(b20())
    finally:
        walk.planner.teardown(None)
    files = walk.run.files()
    assert [file["movement"] for file in files] == ["B20_J_transfer", "B20_J_place_bar", "B20_J_grasp"]
    assert [file["outcome"]["status"] for file in files] == ["ARRIVED", "confirmed", "stalled"]
    assert files[0]["plan"]["planner"] == "tamp.plan_free_dual_arm"
    assert position == (2, 2) and parts[0].kind == "refused" and "not supported yet" in parts[0].text
    end = files[0]["plan"]["rows"][-1]
    names = files[0]["plan"]["joint_names"]
    measured = {name: value for arm in walk.cindy.arms.values()
                for name, value in zip(arm_joint_names(arm.config.name), arm.joint_vector())}
    assert max(abs(measured[name] - value) for name, value in zip(names, end)) < math.radians(0.5)


# --- --- --- --- --- "Ignore built bars" against the setting now (review M2) --- --- --- --- ---

class Executed(Steps):
    """The step panel with the real executor."""

    def __init__(self, current):
        """Build everything; plan B20_R's way home ignoring the built bars, the arms then left still."""
        super().__init__(current)
        self.executor = MovementExecPlugin()
        self.ctx.plugins["movement_exec"] = self.executor
        self.executor.setup(self.ctx)

    async def planned_ignoring_built(self) -> None:
        """Plan with Ignore built bars ticked, then let 2.5 s pass (a lease needs still arms)."""
        self.plugin.set_ignore_built(True)
        await self.plan()
        assert self.plugin.plan_state == "ready" and self.plugin._basis.ignore_built
        for _ in range(25):
            await self.ctx.next_tick()
        self.update()

    async def planned_without_rhino_obstacles(self) -> None:
        """Plan with the Rhino obstacles box unticked, then let 2.5 s pass (a lease needs still arms)."""
        self.process.obstacles_collide = False
        await self.plan()
        assert self.plugin.plan_state == "ready" and not self.plugin._basis.obstacles_collide
        for _ in range(25):
            await self.ctx.next_tick()
        self.update()

    async def confirm(self) -> None:
        """Confirm the prompt a little over 0.5 s after it opened."""
        while self.executor.prompt is None and self.executor.active:
            await asyncio.sleep(0)
        if self.executor.prompt is not None:
            opened = self.executor.prompt.opened_at
            while self.ctx.t - opened < 0.6:
                await self.ctx.next_tick()
            self.executor.request_confirm()
        while self.executor.active:
            await self.ctx.next_tick()


@pytest.mark.safety
def test_unticking_ignore_built_bars_with_exec_sends_nothing():
    """Unticked in the same tick as Exec (both intents before the update): the plan made ignoring them is not sent."""
    lab = Executed(("B20_R", 2))

    async def scenario():
        await lab.planned_ignoring_built()
        lab.plugin.set_ignore_built(False)
        lab.plugin._commit(lab.ctx)
        await lab.confirm()

    asyncio.run(scenario())
    assert not lab.cindy._multi_arm.messages and "ignore built bars" in lab.plugin._message
    assert lab.plugin.plan_state == "stale"


@pytest.mark.safety
def test_unticking_ignore_built_bars_at_the_prompt_is_refused_after_confirm():
    """The prompt says the built bars are ignored; unticked while it waits: refused after Confirm, nothing sent."""
    lab = Executed(("B20_R", 2))
    prompts = []

    async def scenario():
        await lab.planned_ignoring_built()
        lab.plugin._commit(lab.ctx)
        while lab.executor.prompt is None:
            await asyncio.sleep(0)
        prompts.append(lab.executor.prompt.line)
        lab.plugin.set_ignore_built(False)
        await lab.confirm()

    asyncio.run(scenario())
    assert "(built bars ignored)" in prompts[0]
    assert not lab.cindy._multi_arm.messages and "ignore built bars" in lab.executor._last


@pytest.mark.safety
def test_switching_the_rhino_obstacles_with_exec_sends_nothing():
    """Ticked in the same tick as Exec (both intents before the update): the plan made without them is not sent."""
    lab = Executed(("B20_R", 2))

    async def scenario():
        await lab.planned_without_rhino_obstacles()
        lab.process.obstacles_collide = True
        lab.plugin._commit(lab.ctx)
        await lab.confirm()

    asyncio.run(scenario())
    assert not lab.cindy._multi_arm.messages and "Rhino obstacles were switched on" in lab.plugin._message
    assert lab.plugin.plan_state == "stale"


@pytest.mark.safety
def test_switching_the_rhino_obstacles_at_the_prompt_is_refused_after_confirm():
    """The prompt says the Rhino obstacles are not checked; ticked while it waits: refused after Confirm, none sent."""
    lab = Executed(("B20_R", 2))
    prompts = []

    async def scenario():
        await lab.planned_without_rhino_obstacles()
        lab.plugin._commit(lab.ctx)
        # * Stop waiting once the Exec ends: refused before its prompt, there is none to wait for.
        while lab.executor.prompt is None and lab.executor.active:
            await asyncio.sleep(0)
        if lab.executor.prompt is not None:
            prompts.append(lab.executor.prompt.line)
        lab.process.obstacles_collide = True
        await lab.confirm()

    asyncio.run(scenario())
    assert prompts and "(Rhino obstacles not checked)" in prompts[0]
    assert not lab.cindy._multi_arm.messages and "Rhino obstacles were switched on" in lab.executor._last


# --- --- --- --- --- the reused goal's base, and a plan given back stale (review L4, L5) --- --- --- --- ---

@pytest.mark.safety
def test_a_plan_on_a_reused_goal_goes_stale_once_the_base_leaves_where_the_goal_was_solved():
    """A plan reusing a checked goal goes stale 2 mm from where the goal was solved, though 1.5 mm from the plan's base.

    Goal checked at the base; 1.5 mm off, the plan reuses it; 1.5 mm further: stale, and Exec refused.
    """
    lab = Steps(("B21_H", 0))
    base = lab.bases["a200-0804"]
    goal = dict(zip(arm_joint_names("ur_arm"), [1.0, -1.0, 1.0, -1.0, 1.0, 0.0]))
    lab.planner.check = GoalCheck("B21_H_approach", Routine.SUPPORT_APPROACH, True, base=base, goal_joints=goal,
                                  ik=IkResult(goal, 0.0, 1, 8, "ssik", ("alice",), None))

    async def scenario():
        await lab.plugin.checked_goal(lab.ctx)
        lab.bases["a200-0804"] = Pose((0.0015, 0.0, 0.0))
        lab.tick(lab.ctx.t)
        await lab.plan()
        assert lab.planner.plans[-1].goal_joints == goal and lab.plugin.plan_state == "ready"
        lab.bases["a200-0804"] = Pose((0.003, 0.0, 0.0))
        lab.tick(lab.ctx.t)
        lab.plugin._commit(lab.ctx)

    asyncio.run(scenario())
    assert lab.executor.requests == [] and lab.plugin.plan_state == "stale"
    assert "from where its goal was solved" in lab.plugin._message


@pytest.mark.safety
def test_a_plan_given_back_is_checked_for_staleness_first():
    """A dry run's plan comes back stale, not ready, when the base moved meanwhile."""
    lab = Steps(("B20_R", 2))

    async def scenario():
        await lab.plan()
        lab.plugin._commit(lab.ctx)
        lab.bases["a200-0806"] = Pose((3.003, 1.0, 0.0))
        lab.tick(lab.ctx.t)
        lab.executor.futures[0].set_result(ExecResult(DRY_RUN, "dry run: every check passed, nothing sent"))
        await asyncio.sleep(0)

    asyncio.run(scenario())
    assert lab.plugin.plan_state == "stale" and "base moved 3.0 mm" in lab.plugin._message
