"""The tool steps on fake tools: a Robotiq move watched to its answer, a scaffolding grasp watched to its stalls."""

from __future__ import annotations

import asyncio

import pytest
from plugin_fakes import (FakeContext, RobotiqDriver, ScaffoldingDriver, fake_node, fake_robotiq, fake_scaffolding,
                          lease_to_test)

from husky_assembly_teleop.robot_interface.end_effectors import ScaffoldingV3
from husky_assembly_teleop.shared.execution.tools import (DRY_RUN, FELL_SHORT, REACHED, REFUSED, ROBOTIQ_CLOSE,
                                                          ROBOTIQ_OPEN, STALLED, TIMEOUT, robotiq_move,
                                                          scaffolding_grasp)

pytestmark = pytest.mark.safety


def run(ctx: FakeContext, work) -> object:
    """Run one tool step to its end."""
    async def scenario():
        return await work
    return asyncio.run(scenario())


class Gripper:
    """A Robotiq at 0.0 (open) and its driver, ticking with the context."""

    def __init__(self, **driver):
        """Build them; `driver` options go to the RobotiqDriver."""
        self.ctx = FakeContext(step=lambda t: self.driver.tick(t))
        self.gripper = lease_to_test(fake_robotiq(fake_node(self.ctx.now, self.ctx.logs)))
        self.driver = RobotiqDriver(self.gripper, **driver)

    def goals(self) -> list[float]:
        """Every position sent, in order."""
        return [goal.command.position for goal in self.gripper._action.goals]


def test_a_robotiq_move_reaches_its_target():
    """Open to 0.426: one goal, answered, at the target."""
    lab = Gripper()
    outcome = run(lab.ctx, robotiq_move(lab.ctx, lab.gripper, ROBOTIQ_OPEN))
    assert outcome.status == REACHED and outcome.ok and lab.goals() == [ROBOTIQ_OPEN]
    assert outcome.detail == {"target": ROBOTIQ_OPEN, "position": ROBOTIQ_OPEN, "result_ok": True}


def test_a_robotiq_move_that_falls_short_is_not_ok():
    """Stopped 0.2 rad short (on something in the jaws): FELL_SHORT, though the gripper reports success."""
    lab = Gripper(reach=lambda target: target - 0.2)
    outcome = run(lab.ctx, robotiq_move(lab.ctx, lab.gripper, ROBOTIQ_CLOSE))
    assert outcome.status == FELL_SHORT and not outcome.ok and "0.600" in outcome.reason


def test_a_robotiq_not_ready_or_busy_is_refused():
    """No action server, or an earlier goal still running: nothing sent."""
    lab = Gripper()
    lab.gripper._action.ready = False
    assert run(lab.ctx, robotiq_move(lab.ctx, lab.gripper, ROBOTIQ_OPEN)).status == REFUSED
    lab.gripper._action.ready, lab.gripper.state.moving = True, True
    outcome = run(lab.ctx, robotiq_move(lab.ctx, lab.gripper, ROBOTIQ_OPEN))
    assert outcome.status == REFUSED and "still runs" in outcome.reason and lab.goals() == []
    lab.gripper.state.moving = False
    assert run(lab.ctx, robotiq_move(lab.ctx, lab.gripper, ROBOTIQ_OPEN, dry_run=True)).status == DRY_RUN
    assert lab.goals() == []


def test_a_robotiq_that_never_answers_times_out_and_is_stopped():
    """No answer in 5 s: TIMEOUT, then a stop: the goal is cancelled once accepted (`RobotiqGripper.stop`)."""
    lab = Gripper(answer=False)
    outcome = run(lab.ctx, robotiq_move(lab.ctx, lab.gripper, ROBOTIQ_CLOSE))
    assert outcome.status == TIMEOUT and lab.goals() == [ROBOTIQ_CLOSE] and lab.gripper._cancel_on_accept


def test_a_cancelled_robotiq_move_is_stopped():
    """Cancelling the wait stops the fingers (the running goal cancelled, no new goal), then raises."""
    lab = Gripper(delay=10.0)

    async def scenario():
        task = asyncio.ensure_future(robotiq_move(lab.ctx, lab.gripper, ROBOTIQ_CLOSE))
        while lab.ctx.t < 10.5:
            await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    assert lab.goals() == [ROBOTIQ_CLOSE] and lab.gripper._action.goals[0].cancelled


class Tools:
    """Cindy's two scaffolding tools, each with its firmware, reporting every tick."""

    def __init__(self, left: dict | None = None, right: dict | None = None):
        """Build them; each dict goes to that tool's ScaffoldingDriver."""
        self.ctx = FakeContext(step=self.tick)
        node = fake_node(self.ctx.now, self.ctx.logs)
        self.tools = {"left_ur_arm": lease_to_test(fake_scaffolding(node, "/a200_0806/left_gripper")),
                      "right_ur_arm": lease_to_test(fake_scaffolding(node, "/a200_0806/right_gripper"))}
        self.drivers = [ScaffoldingDriver(self.tools["left_ur_arm"], **(left or {})),
                        ScaffoldingDriver(self.tools["right_ur_arm"], **(right or {}))]
        self.tick(self.ctx.t)

    def tick(self, t: float) -> None:
        """Every tool runs and reports."""
        for driver in self.drivers:
            driver.tick(t)

    def commands(self, arm: str) -> list[tuple[int, int]]:
        """(motor, direction) of every command sent to one tool."""
        return [(command.motor, command.direction) for command in self.tools[arm]._command.messages]


STOP, CLOSE = (0, 0), (ScaffoldingV3.GRIPPER_MOTOR, 1)


def test_a_grasp_clears_a_stall_closes_and_stops():
    """A stall left over: STOP, the stall clears, close, both STALLED: clamped; then STOP again."""
    lab = Tools(left={"motor": ScaffoldingV3.STALLED}, right={"stall_after": 2.0})
    outcome = run(lab.ctx, scaffolding_grasp(lab.ctx, lab.tools))
    assert outcome.status == STALLED and outcome.ok
    assert outcome.detail == {"left_ur_arm": "STALLED", "right_ur_arm": "STALLED"}
    for arm in lab.tools:
        assert lab.commands(arm) == [STOP, CLOSE, STOP]


def test_a_grasp_asks_again_right_before_it_tightens():
    """Something starts moving while the leftover stall clears: refused, nothing tightened, every tool stopped."""
    lab = Tools(left={"motor": ScaffoldingV3.STALLED})
    asked = []

    def ready() -> str:
        """Busy once the stall-clearing wait has taken a tick."""
        asked.append(lab.ctx.t)
        return "a200-0804 ur_arm is running a trajectory" if lab.ctx.t > 10.0 else ""

    outcome = run(lab.ctx, scaffolding_grasp(lab.ctx, lab.tools, ready=ready))
    assert outcome.status == REFUSED and outcome.reason == ("a200-0804 ur_arm is running a trajectory; nothing "
                                                            "tightened")
    assert len(asked) == 1 and asked[0] > 10.0
    for arm in lab.tools:
        assert lab.commands(arm) == [STOP, STOP]


def test_a_tool_that_never_stalls_times_out_and_is_stopped():
    """The right screw never stalls: TIMEOUT after 30 s, "check the grip", every tool stopped."""
    lab = Tools(right={"stall_after": None})
    outcome = run(lab.ctx, scaffolding_grasp(lab.ctx, lab.tools))
    assert outcome.status == TIMEOUT and "check the grip" in outcome.reason and outcome.seconds >= 30.0
    assert lab.commands("right_ur_arm") == [STOP, CLOSE, STOP]


def test_a_stall_that_does_not_clear_tightens_nothing():
    """A tool still STALLED after STOP (its firmware ignores it): nothing tightened, every tool stopped."""
    lab = Tools()
    lab.drivers[1].tick = lambda t: lab.tools["right_ur_arm"]._on_status(type("Status", (), dict(
        state_m1="STALLED", state_m2="IDLE", current=0, pwm_pct=0, buttons_held=(), buttons_pressed=()))())
    outcome = run(lab.ctx, scaffolding_grasp(lab.ctx, lab.tools))
    assert outcome.status == TIMEOUT and "nothing tightened" in outcome.reason
    assert lab.commands("left_ur_arm") == [STOP, STOP]


def test_a_cancelled_grasp_stops_every_tool():
    """Cancelling while the screws tighten stops both, then raises."""
    lab = Tools(left={"stall_after": None}, right={"stall_after": None})

    async def scenario():
        task = asyncio.ensure_future(scaffolding_grasp(lab.ctx, lab.tools))
        while lab.ctx.t < 11.0:
            await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    for arm in lab.tools:
        assert lab.commands(arm) == [STOP, CLOSE, STOP]


def test_a_soft_stopped_robot_moves_no_tool():
    """Soft stopped (until Resume): a Robotiq move and a grasp are REFUSED, nothing moves; the screws are stopped."""
    lab = Gripper()
    lab.gripper._latch.stopped = True
    outcome = run(lab.ctx, robotiq_move(lab.ctx, lab.gripper, ROBOTIQ_CLOSE))
    assert outcome.status == REFUSED and "soft stopped" in outcome.reason and lab.goals() == []
    lab = Tools()
    for tool in lab.tools.values():
        tool._latch.stopped = True
    outcome = run(lab.ctx, scaffolding_grasp(lab.ctx, lab.tools))
    assert outcome.status == REFUSED and "soft stopped" in outcome.reason and "nothing tightened" in outcome.reason
    for arm in lab.tools:
        assert lab.commands(arm) == [STOP, STOP]


def test_a_silent_driver_is_refused():
    """A tool with no status in the last second: refused, nothing sent; a dry run checks the same and sends nothing."""
    lab = Tools(right={"silent": True})
    outcome = run(lab.ctx, scaffolding_grasp(lab.ctx, lab.tools))
    assert outcome.status == REFUSED and "right_ur_arm" in outcome.reason and lab.commands("left_ur_arm") == []
    lab = Tools()
    assert run(lab.ctx, scaffolding_grasp(lab.ctx, lab.tools, dry_run=True)).status == DRY_RUN
    assert lab.commands("left_ur_arm") == []
