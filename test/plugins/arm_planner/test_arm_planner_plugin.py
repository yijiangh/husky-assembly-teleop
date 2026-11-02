"""The arm planner as a headless planner (staleness, send) and its panel driving it through the API."""

from __future__ import annotations

import asyncio
import math
from collections import deque
from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
import pytest
from planning_fixtures import ALICE, BELLE, CINDY, robot_config
from plugin_fakes import ArmDriver, FakeContext, fake_arm, fake_node, fake_robot
from robot_fakes import lease_test_robots, wait_ticks_until

from husky_assembly_teleop.plugins.arm_planner import planner_plugin as arm_plugin
from husky_assembly_teleop.plugins.arm_planner.panel_plugin import ArmPlannerPanel
from husky_assembly_teleop.plugins.arm_planner.planner import ArmPath
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.plugins.arm_planner.planner_plugin import ArmPlannerPlugin
from husky_assembly_teleop.shared.arm_planner_contract import MAX_SLOWDOWN, ArmPlanRequest
from husky_assembly_teleop.shared.execution.send import ARRIVED_TOLERANCE
from husky_assembly_teleop.shared.planner.planner_search import PlanResult
from husky_assembly_teleop.shared.planner.plan import Plan, PlannerContract
from husky_assembly_teleop.ui.style import NONE, OK
from husky_assembly_teleop.world.checks import BAD, GOOD, PENDING, WARN, Check
from husky_assembly_teleop.world.clearance import ARM_STOP_DISTANCE, ClearanceRefused
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene

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
        robot = RobotObject(robot_id(SERIAL), SimpleNamespace(movable_joints=()), pose(0.0, 0.0, 0.0), {})
        self.scene = SimpleNamespace(snapshot=Scene(robots={robot.id: robot}))
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

    @contextmanager
    def clearance(self, _serials, _margin, unknown_arms=(), unknown_robots=()):
        """Keep no clearance: nothing here could trip one."""
        yield

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
    monkeypatch.setattr(arm_plugin, "plan_arm",
                        lambda world, mirror, arm, goal, abort: PlanResult(PATH, seconds=0.1))


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


def _driven(speed: float = 1.0, start: np.ndarray = START, **measured) -> tuple[FakeContext, object]:
    """A context over robot SERIAL with a real arm ARM at `start`, run by an ArmDriver at `speed` (1: as timed).

    The snapshot has SERIAL measured, or as `measured` says (`base_tracked`, `unmeasured`).
    """
    driver = None
    model = robot_config(CINDY).model
    ctx = FakeContext(name="arm_planner", step=lambda t: driver.tick(t), snapshot=Scene(robots={
        robot_id(SERIAL): RobotObject(robot_id(SERIAL), model, Pose(), {}, **measured)}))
    node = fake_node(ctx.now, ctx.logs)
    arm = fake_arm(node, ARM, start)
    ctx.world.robots[SERIAL] = robot = fake_robot(node, SERIAL, [arm])
    driver = ArmDriver(arm, robot, speed=speed)
    return ctx, arm


def _commit(ctx: FakeContext, answers: list, cancel_after: int | None = None, clearance: bool = True,
            planner: ArmPlannerPlugin | None = None) -> np.ndarray:
    """Execute the ready plan under the robot's lease, cancelled after `cancel_after` ticks if given; then let 2 s pass.

    Returns:
        np.ndarray: The arm's joints when the commit ended (and the lease went back).
    """
    planner = planner or ArmPlannerPlugin()

    async def commit():
        with ctx.lease_robots([SERIAL], "commit plan 1"):
            planner.clearance = clearance
            await planner.execute(ctx, ready_plan(), lambda *answer: answers.append(answer))

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
    """The plan's path goes to the arm under a 1 cm clearance, answered once sent; it ends with the arm there."""
    (ctx, arm), answers, planner = _driven(), [], ArmPlannerPlugin()
    ended = _commit(ctx, answers, planner=planner)
    assert answers == [(True, f"sent to {SERIAL} {ARM}")] and len(arm._trajectory.messages) == 1
    assert np.allclose(ended, GOAL, atol=1e-3)
    assert (_chip(planner).label, _chip(planner).level) == ("done", GOOD)
    clearance, = ctx.clearances
    assert (clearance.serials, clearance.margin) == ((SERIAL,), ARM_STOP_DISTANCE) and ARM_STOP_DISTANCE == 0.01


@pytest.mark.safety
def test_with_clearance_unticked_the_path_goes_out_without_one():
    """Clearance is on by default; off, the same path is sent with no clearance kept."""
    assert ArmPlannerPlugin().clearance
    ctx, answers = _driven()[0], []
    ended = _commit(ctx, answers, clearance=False)
    assert answers == [(True, f"sent to {SERIAL} {ARM}")] and not ctx.clearances
    assert np.allclose(ended, GOAL, atol=1e-3)


@pytest.mark.safety
@pytest.mark.parametrize("clearance", [True, False])
@pytest.mark.parametrize("measured, why", [
    ({"unmeasured": frozenset(arm_joint_names("right_ur_arm"))}, "right_ur_arm not measured"),
    ({"base_tracked": False}, "base not measured"),
])
def test_an_arm_moves_only_with_the_base_and_every_arm_measured(measured, why, clearance):
    """The other arm unknown or the base untracked: nothing is sent, Clearance on or off."""
    (ctx, arm), answers, planner = _driven(**measured), [], ArmPlannerPlugin()
    _commit(ctx, answers, clearance=clearance, planner=planner)
    assert answers == [(False, f"nothing sent: {SERIAL}: {why}")] and not arm._trajectory.messages
    assert (_chip(planner).label, _chip(planner).level) == ("refused", BAD)


@pytest.mark.safety
def test_execute_reports_an_arm_that_refuses_the_path():
    """An arm that refuses the trajectory (it is not at the path's start) answers the commit that nothing was sent."""
    (ctx, arm), answers, planner = _driven(start=START + np.radians([10.0, 0, 0, 0, 0, 0])), [], ArmPlannerPlugin()
    _commit(ctx, answers, planner=planner)
    (sent, message), = answers
    assert not sent and "refused" in message and not arm._trajectory.messages
    assert (_chip(planner).label, _chip(planner).level) == ("refused", BAD)


@pytest.mark.safety
def test_a_cancelled_commit_holds_the_arm():
    """Cancelled (Stop all, plugin stop, a new commit) while the arm moves: it is held before the lease goes back.

    The run chip says "cancelled" in grey (`PlannerService.cancelled_check`).
    """
    (ctx, arm), answers, planner = _driven(), [], ArmPlannerPlugin()
    ended = _commit(ctx, answers, cancel_after=5, planner=planner)
    assert _held(arm) and not np.allclose(ended, GOAL, atol=1e-2)
    assert np.allclose(arm.joint_vector(), ended, atol=1e-6)
    assert (_chip(planner).label, _chip(planner).level) == ("cancelled", PENDING)


@pytest.mark.safety
def test_a_replaced_commit_leaves_no_run_chip():
    """Cancelled for a newer commit: its chip goes, the newer commit's comes."""
    ctx, answers, planner = _driven()[0], [], ArmPlannerPlugin()
    planner._motions[(SERIAL, ARM)] = Check("done", GOOD)
    planner.replaced = lambda: True
    _commit(ctx, answers, cancel_after=5, planner=planner)
    assert answers == [(True, f"sent to {SERIAL} {ARM}")] and (SERIAL, ARM) not in planner._motions


@pytest.mark.safety
def test_a_clearance_that_cannot_check_replaces_the_last_run_chip(monkeypatch):
    """Refused on entering the clearance: nothing sent, the last run's "done" gives way to a red "refused"."""
    (ctx, arm), answers, planner = _driven(), [], ArmPlannerPlugin()
    planner._motions[(SERIAL, ARM)] = Check("done", GOOD)

    @contextmanager
    def refused(*_args, **_options):
        raise ClearanceRefused("robot 0806 cannot be checked against the others: belle not measured")
        yield

    monkeypatch.setattr(ctx, "clearance", refused)
    _commit(ctx, answers, planner=planner)
    assert not arm._trajectory.messages and (_chip(planner).label, _chip(planner).level) == ("refused", BAD)
    assert "belle not measured" in _chip(planner).detail


@pytest.mark.safety
def test_a_commit_that_fails_after_sending_is_not_left_moving(monkeypatch):
    """An error after the send ends the amber "moving" in a red "failed"."""
    ctx, answers, planner = _driven()[0], [], ArmPlannerPlugin()

    async def fails(_ctx, _robot, _command, on_sent):
        on_sent()
        raise RuntimeError("joint states garbled")

    monkeypatch.setattr(arm_plugin, "send_motion", fails)
    _commit(ctx, answers, planner=planner)
    assert answers == [(True, f"sent to {SERIAL} {ARM}")]
    assert (_chip(planner).label, _chip(planner).level) == ("failed", BAD)


@pytest.mark.safety
def test_a_commit_whose_lease_is_refused_shows_refused():
    """Another plugin holds the robot: `execute` never runs, and the run chip still says nothing went out."""
    (ctx, arm), planner = _driven(), ArmPlannerPlugin()
    planner.setup(ctx)
    planner._motions[(SERIAL, ARM)] = Check("done", GOOD)

    async def scenario():
        future = asyncio.get_running_loop().create_future()
        with ctx.lease_robots([SERIAL], "teleop"):
            await planner._execute(ready_plan(), None, future)
        return future.result()

    sent, message = asyncio.run(scenario())
    assert not sent and message.startswith("nothing sent") and not arm._trajectory.messages
    assert (_chip(planner).label, _chip(planner).level) == ("refused", BAD)


@pytest.mark.safety
@pytest.mark.parametrize("speed, held", [(0.1, True), (0.25, False)])
def test_a_slow_arm_keeps_the_commit_until_it_is_at_its_end_or_held(speed, held):
    """The pendant's speed slider under 100 %: the commit, and so the lease, lasts until the arm arrives or is held.

    At 10 % it is late, held, and moves no more.
    ! `is_executing` clears during such a move: waiting on it gave the lease back while the arm still moved.
    """
    (ctx, arm), answers, planner = _driven(speed), [], ArmPlannerPlugin()
    ended = _commit(ctx, answers, planner=planner)
    assert answers == [(True, f"sent to {SERIAL} {ARM}")] and _held(arm) == held
    assert _chip(planner).level == (BAD if held else GOOD)
    if held:
        assert np.allclose(arm.joint_vector(), ended, atol=1e-6)
    else:
        assert np.max(np.abs(ended - GOAL)) < ARRIVED_TOLERANCE


def _chip(planner: ArmPlannerPlugin) -> Check:
    """The run chip of SERIAL ARM."""
    return planner._motions[(SERIAL, ARM)]


def make_panel(planner: ArmPlannerPlugin) -> ArmPlannerPanel:
    """A panel on `planner` with six sliders at the goal, as the operator would have set them."""
    panel = ArmPlannerPanel()
    panel._api = panel._planner = planner
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


def test_set_duration_re_times_the_ready_plan_within_its_limits_and_keeps_its_id():
    """No replan: the path at the asked seconds, clamped to 1 to MAX_SLOWDOWN times its fastest; stale checks go on."""
    planner = ArmPlannerPlugin()
    plan = ready_plan()
    planner._plan, planner._start_base = plan, (plan.path, (0.0, 0.0, 0.0), False)
    fastest = plan.path.duration
    planner.set_duration(plan.id, 2 * fastest)
    assert planner.plan.id == plan.id and planner.plan.path.duration == pytest.approx(2 * fastest)
    assert planner._start_base[0] is planner.plan.path
    planner.set_duration(plan.id, 100 * fastest)
    assert planner.plan.path.slowdown == MAX_SLOWDOWN
    planner.set_duration(plan.id, 0.0)
    assert planner.plan.path.duration == pytest.approx(fastest)
    planner.set_duration(plan.id + 1, 3 * fastest)
    assert planner.plan.path.slowdown == 1.0, "another plan's id changes nothing"


def test_the_duration_slider_re_times_the_plan_and_new_plans_take_its_slowdown():
    """Moving it stretches the plan and keeps the preview's place along it; the next plan comes at the same slowdown."""
    planner = ArmPlannerPlugin()
    panel = make_panel(planner)
    planner._plan = ready_plan()
    panel._t = 1.0
    panel._set_duration(3 * planner.plan.path.fastest)
    assert panel._slowdown == pytest.approx(3.0) and panel._t == pytest.approx(3.0)
    planner._plan = Plan(2, REQUEST, "ready", path=PATH)
    panel.update(SimpleNamespace(now=lambda: 0.0, log_warn=lambda _text: None))
    assert planner.plan.id == 2 and planner.plan.path.duration == pytest.approx(3 * PATH.duration)
    planner._plan = Plan(3, REQUEST, "planning")
    panel._set_duration(1.0)
    assert panel._message == "nothing to re-time: plan first" and panel._slowdown == pytest.approx(3.0)


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
    # * The run chip stands in for the plan chip's "sent" while the arm moves.
    assert (_chip(planner).label, _chip(planner).level) == ("moving", WARN)
    assert planner.plan.request == ArmPlanRequest(SERIAL, ARM, pytest.approx(tuple(GOAL))) and len(ctx.sent) == 1


def test_the_message_line_is_green_once_sent_and_says_when_the_plan_is_cleared(found_path):
    """The commit's answer is green whichever of the two news lands last; Clear replaces the old line in grey."""
    ctx = FakeCtx()
    planner = make_planner(ctx)
    panel = make_panel(planner)

    async def scenario():
        panel._on_path(ctx, "Plan")
        await asyncio.sleep(0.05)
        panel.update(ctx)
        panel._commit(ctx)
        await asyncio.sleep(0.01)
        reported = panel._message_color
        panel.update(ctx)
        sent = panel._message, panel._message_color
        panel._on_path(ctx, "Clear")
        panel.update(ctx)
        return reported, sent, (panel._message, panel._message_color)

    reported, sent, cleared = run(planner, ctx, scenario)
    assert reported == OK and sent == (f"sent to {SERIAL} {ARM}", OK)
    assert cleared == ("plan cleared", NONE)


def test_loading_a_step_for_another_arm_says_so_past_the_next_tick():
    """Load › Step for the other arm clears the plan; its own line, not "plan cleared", is what stays."""
    planner = ArmPlannerPlugin()
    panel = make_panel(planner)
    panel._arms = {SERIAL: [ARM, "right_ur_arm"]}
    planner._plan = ready_plan()
    panel.update(FakeCtx())
    given = dict(zip(arm_joint_names("right_ur_arm"), GOAL))
    movement = SimpleNamespace(id="M3", target=SimpleNamespace(joints={"cindy": given}))
    selection = SimpleNamespace(movement=movement, action=SimpleNamespace(robot="cindy"))
    panel._process_step = lambda _ctx: (selection, SERIAL, "Cindy")
    panel._load_target(None, "Step")
    panel.update(FakeCtx())
    assert planner.plan is None and panel.arm == "right_ur_arm"
    assert panel._message == "target from the target of M3 (right_ur_arm)"


def test_the_status_line_shows_the_chosen_arms_run_chip():
    """Each arm keeps its own run chip: another robot's or arm's run never shows, or ends, the chosen one's."""
    planner = ArmPlannerPlugin()
    panel = make_panel(planner)
    planner._motions = {(SERIAL, ARM): Check("moving", WARN, "left moves"),
                        (SERIAL, "right_ur_arm"): Check("done", GOOD, "right arrived")}
    robots = [RobotObject(robot_id(serial), SimpleNamespace(movable_joints=()), pose(0.0, 0.0, 0.0), {})
              for serial in (SERIAL, "0804")]
    ctx = SimpleNamespace(scene=SimpleNamespace(snapshot=Scene(robots={robot.id: robot for robot in robots})))
    assert "moving" in panel._status_html(ctx) and "done" not in panel._status_html(ctx)
    panel.arm = "right_ur_arm"
    assert "done" in panel._status_html(ctx) and "moving" not in panel._status_html(ctx)
    panel.serial = "0804"
    assert "done" not in panel._status_html(ctx) and "moving" not in panel._status_html(ctx)


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


def _with_others(ctx: FakeCtx, *others: RobotObject) -> None:
    """Give `ctx` a real snapshot: the planned robot (not configured here, so a stand-in model) and `others`."""
    acting = RobotObject(robot_id(SERIAL), robot_config(CINDY).model, Pose(), {})
    ctx.scene.snapshot = Scene(robots={robot.id: robot for robot in (acting, *others)})


@pytest.mark.safety
def test_a_plan_warns_about_other_robots_that_stand_in(found_path):
    """Belle lost mocap: she is checked where last seen, and the plan says so, as a warning. An absent robot is not."""
    ctx = FakeCtx()
    belle, alice = robot_config(BELLE), robot_config(ALICE)
    _with_others(ctx, RobotObject(robot_id(belle.serial), belle.model, Pose((3.0, 0.0, 0.0)), {}, base_tracked=False,
                                  base_time=1.0, joints_time=1.0),
                 RobotObject(robot_id(alice.serial), alice.model, Pose(), {}, enabled=False))
    planner, warned = make_planner(ctx), []
    ctx.log_warn = warned.append

    async def scenario():
        return await planner.request_plan(REQUEST)

    plan = run(planner, ctx, scenario)
    assert plan.state == "ready" and plan.warning and "(1 other robot not measured: see Health)" in plan.message
    assert any("a200-0805: base not measured, checked where last seen" in line
               for line in warned)
    assert not any("a200-0804" in line for line in warned)


def test_a_plan_with_every_robot_measured_has_no_such_warning(found_path):
    """Robots present and measured, or absent, need no warning."""
    ctx = FakeCtx()
    belle = robot_config(BELLE)
    _with_others(ctx, RobotObject(robot_id(belle.serial), belle.model, Pose((3.0, 0.0, 0.0)), {}))
    planner = make_planner(ctx)

    async def scenario():
        return await planner.request_plan(REQUEST)

    plan = run(planner, ctx, scenario)
    assert plan.state == "ready" and not plan.warning and "not measured" not in plan.message
