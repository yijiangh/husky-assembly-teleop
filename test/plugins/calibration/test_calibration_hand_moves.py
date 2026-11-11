"""Free drive and tool steps in Tip offset and Punch: kept on only while wanted, off for procedures, under the lease."""

from __future__ import annotations

import asyncio
from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np
import pytest

from husky_assembly_teleop.plugin_api.concurrency import WaitTimeout
from husky_assembly_teleop.plugins.calibration.robot.hand_moves import QUEUED_STEPS, HandMoves
from husky_assembly_teleop.robot_interface.arm import FREE_DRIVE_CONTROLLER, SCALED_JOINT_TRAJECTORY_CONTROLLER
from husky_assembly_teleop.robot_interface.robot_lease import NotLeased
from husky_assembly_teleop.shared.guide.procedure import Procedure
from husky_assembly_teleop.world.checks import BAD, WARN, Check
from husky_assembly_teleop.world.clearance import ARM_STOP_DISTANCE
from scaffolding_assembly_core.geometry import Pose

pytestmark = pytest.mark.safety

STEP, PIVOT = Pose((0.0, 0.0, 1e-3)), np.array([0.0, 0.0, 0.11])


class Controllers:
    """Switches at once, and keeps every switch."""

    def __init__(self, active: str):
        """`active` runs, as the monitor's own choice."""
        self.state = SimpleNamespace(active=active, requested=active, switch_error=None)
        self.switches: list[tuple[str, bool]] = []

    def is_active(self, name: str) -> bool:
        """Whether `name` runs."""
        return self.state.active == name

    def switch(self, name: str, unguarded: bool = False) -> bool:
        """Run `name` now; with `rejects` set, the manager rejects the next switch after it went out."""
        self.switches.append((name, unguarded))
        if getattr(self, "rejects", False):
            self.rejects = False
            self.state.requested, self.state.switch_error = None, f"switch to {name!r} failed: rejected"
            return True
        self.state.active = self.state.requested = name
        self.state.switch_error = None
        return True


class Arm:
    """Counts free drive requests and holds, and keeps the tool steps sent; a step runs until `stops` is set."""

    def __init__(self, active: str = SCALED_JOINT_TRAJECTORY_CONTROLLER, stops: bool = True):
        """At rest under `active`; its steps end at once if `stops`."""
        self.controllers = Controllers(active)
        self.state = SimpleNamespace(last_motion_time=None, last_slight_motion_time=None, last_update_time=None,
                                     is_executing=False)
        self.last_refusal, self.requests, self.holds, self.steps, self.stops = None, 0, 0, [], stops
        self.refuse = self.stale = False

    def request_free_drive(self) -> None:
        """Count it."""
        self.requests += 1

    def hold(self) -> None:
        """Count it; in free drive, back to the joint controller as the real arm does."""
        self.holds += 1
        if self.controllers.is_active(FREE_DRIVE_CONTROLLER):
            self.controllers.switch(SCALED_JOINT_TRAJECTORY_CONTROLLER, unguarded=True)

    def send_tool_step(self, step: Pose, pivot) -> float | None:
        """Keep it (it takes half a second), or refuse it if `refuse`."""
        if self.refuse:
            self.last_refusal = SimpleNamespace(reason="not moving: too near a singularity")
            return None
        self.steps.append((step, np.asarray(pivot)))
        self.state.is_executing = not self.stops
        return 0.5


def setting(arm: Arm, leased: bool = True):
    """A clock, a context that leases the robot if `leased` and a run of `arm`."""
    clock = SimpleNamespace(now=0.0)

    @contextmanager
    def lease_robots(*_args):
        if not leased:
            raise NotLeased("0805 is leased by robot_control")
        yield None

    async def next_tick():
        clock.now += 0.05
        if not arm.stale:
            arm.state.last_update_time = clock.now  # * joint states keep coming
        await asyncio.sleep(0)

    async def sleep(seconds):
        clock.now += seconds

    def spawn(_label, work):
        work.close()
        return SimpleNamespace(done=lambda: True, cancel=lambda: None)

    async def wait_until(predicate, timeout_s=None, description=""):
        start = clock.now
        while not predicate():
            if timeout_s is not None and clock.now - start > timeout_s:
                raise WaitTimeout(description)
            await next_tick()

    @contextmanager
    def clearance(*_args, **_kwargs):
        yield SimpleNamespace(checked=True, refused=False, problem="")  # * found clear at once

    ctx = SimpleNamespace(now=lambda: clock.now, next_tick=next_tick, wait_until=wait_until, sleep=sleep,
                          lease_robots=lease_robots, spawn=spawn, clearance=clearance,
                          scene=SimpleNamespace(snapshot=SimpleNamespace(robots={})))
    run = SimpleNamespace(serial="a200-0805", arm="ur_arm", robot=SimpleNamespace(arms={"ur_arm": arm}))
    return ctx, run


def play(hand: HandMoves, ctx, run, scenario, refusal=lambda: None) -> None:
    """Run the hand moves' task while `scenario(next_tick)` acts, then cancel it as leaving the step does.

    The step shown moves by hand, and is ready while `refusal()` is None.
    """
    hand.start(ctx, run, refusal)

    async def main():
        task = asyncio.get_running_loop().create_task(hand._move(ctx, run))
        await scenario(ctx.next_tick)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(main())


def running(procedure: Procedure) -> None:
    """Make a procedure run, as a take does."""
    procedure._task = SimpleNamespace(done=lambda: False)


def test_free_drive_is_kept_on_while_wanted_and_ends_for_a_procedure():
    """Ticked: free drive runs and is asked for every tick. A take starts: free drive ends and the joints hold."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm)

    async def scenario(tick):
        hand.want_free_drive(True)
        for _ in range(5):
            await tick()
        assert arm.controllers.is_active(FREE_DRIVE_CONTROLLER) and arm.requests >= 3
        running(procedure)
        for _ in range(3):
            await tick()
        assert not hand.free_drive and arm.controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER)

    play(hand, ctx, run, scenario)
    assert arm.controllers.switches == [(FREE_DRIVE_CONTROLLER, False), (SCALED_JOINT_TRAJECTORY_CONTROLLER, True)]


def test_another_controller_started_elsewhere_ends_free_drive_and_says_so():
    """Stop all or another panel's switch: free drive is no longer wanted, and nothing switches back."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm)

    async def scenario(tick):
        hand.want_free_drive(True)
        for _ in range(3):
            await tick()
        arm.controllers.state.active = arm.controllers.state.requested = None
        for _ in range(3):
            await tick()

    play(hand, ctx, run, scenario)
    assert not hand.free_drive and procedure.outcome.label == "free drive ended"
    assert arm.controllers.switches == [(FREE_DRIVE_CONTROLLER, False)]


def test_a_tool_step_from_free_drive_switches_to_the_joint_controller_first():
    """The step goes out on the joint controller, turning about the pivot it was clicked with."""
    procedure, arm = Procedure(), Arm(FREE_DRIVE_CONTROLLER)
    hand = HandMoves(procedure)
    ctx, run = setting(arm)

    async def scenario(tick):
        hand.add_step(STEP, PIVOT)
        hand.add_step(STEP, PIVOT)
        for _ in range(30):
            await tick()

    play(hand, ctx, run, scenario)
    assert arm.controllers.switches[0] == (SCALED_JOINT_TRAJECTORY_CONTROLLER, False)
    assert len(arm.steps) == 2 and arm.steps[0][0] == STEP and np.allclose(arm.steps[0][1], PIVOT)
    assert procedure.outcome is None


@pytest.mark.parametrize("why", ["free drive", "procedure", "queue"])
def test_a_tool_step_is_not_queued_while_it_could_not_go(why):
    """In free drive, during a procedure, or with the queue full, a click says why and queues nothing."""
    procedure = Procedure()
    hand = HandMoves(procedure)
    if why == "free drive":
        hand.free_drive = True
    elif why == "procedure":
        running(procedure)
    else:
        hand._steps = [(STEP, PIVOT)] * QUEUED_STEPS
    queued = len(hand._steps)
    hand.add_step(STEP, PIVOT)
    assert len(hand._steps) == queued and procedure.outcome.label == "tool step not sent"


def test_without_the_lease_nothing_moves_and_free_drive_is_no_longer_wanted():
    """Another plugin holds the robot: the refusal is shown, and free drive must be ticked again."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm, leased=False)

    async def scenario(tick):
        hand.want_free_drive(True)
        for _ in range(3):
            await tick()

    play(hand, ctx, run, scenario)
    assert not hand.free_drive and arm.controllers.switches == [] and arm.requests == 0
    assert procedure.outcome == Check("arm not moved", BAD, "0805 is leased by robot_control")


def test_a_step_that_is_not_ready_refuses_free_drive_and_tool_steps():
    """With the step's checklist unticked, ticking free drive or a tool pad click says why and moves nothing."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm)
    hand.start(ctx, run, lambda: "checklist: E-stop at hand")
    hand.want_free_drive(True)
    assert not hand.free_drive and procedure.outcome == Check("free drive refused", BAD, "checklist: E-stop at hand")
    hand.add_step(STEP, PIVOT)
    assert hand._steps == [] and procedure.outcome.label == "tool step not sent"


def test_free_drive_ends_once_the_step_is_no_longer_ready():
    """Unticking the checklist mid free drive ends it, holds the arm and says why."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm)
    ready = SimpleNamespace(why=None)

    async def scenario(tick):
        hand.want_free_drive(True)
        for _ in range(3):
            await tick()
        ready.why = "checklist: E-stop at hand"
        for _ in range(3):
            await tick()

    play(hand, ctx, run, scenario, lambda: ready.why)
    assert not hand.free_drive and arm.controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER) and arm.holds == 1
    assert procedure.outcome == Check("arm moves by hand ended", WARN, "checklist: E-stop at hand")


def test_ticking_free_drive_drops_the_tool_steps_still_waiting():
    """Steps clicked before free drive were meant from the pose before: they never run after it."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm)
    hand.start(ctx, run)
    hand.add_step(STEP, PIVOT)
    hand.want_free_drive(True)
    assert hand._steps == []


def test_a_tool_step_keeps_the_lease_until_the_arm_stops_and_holds_it_if_it_never_does():
    """A step outlasting its duration (a slow speed slider) keeps the lease; past the margin the arm is held.

    The refusal is shown then.
    """
    procedure, arm = Procedure(), Arm(stops=False)
    hand = HandMoves(procedure)
    ctx, run = setting(arm)
    leased = []
    lease = ctx.lease_robots

    def watched(*args):
        leased.append(True)
        return lease(*args)

    ctx.lease_robots = watched

    async def scenario(tick):
        hand.add_step(STEP, PIVOT)
        for _ in range(40):
            await tick()
        assert arm.holds == 0 and arm.steps  # * still moving, still leased
        for _ in range(100):
            await tick()

    play(hand, ctx, run, scenario)
    assert arm.holds == 1 and procedure.outcome.label == "arm not moved" and leased == [True]


def test_a_slow_tool_step_keeps_the_lease_while_the_joints_still_move():
    """`is_executing` clears during a slow step (the pendant's speed slider): the lease stays until the joints rest."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm)
    held = []
    lease = ctx.lease_robots

    @contextmanager
    def watched(*args):
        with lease(*args):
            held.append(True)
            try:
                yield None
            finally:
                held.append(False)

    ctx.lease_robots = watched

    async def scenario(tick):
        hand.add_step(STEP, PIVOT)
        for _ in range(40):
            arm.state.last_slight_motion_time = ctx.now()
            await tick()
        assert arm.steps and held == [True]  # * not executing, but still creeping: still leased
        for _ in range(40):
            await tick()

    play(hand, ctx, run, scenario)
    assert held == [True, False] and arm.holds == 0


def test_a_tool_step_never_counts_as_stopped_while_the_joint_states_are_stale():
    """No joint states for a while after the step: the arm cannot be told still, so it is held, never handed back."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm)

    async def scenario(tick):
        hand.add_step(STEP, PIVOT)
        arm.stale, arm.state.last_update_time = True, ctx.now() - 10.0
        for _ in range(140):
            await tick()

    play(hand, ctx, run, scenario)
    assert arm.holds == 1 and procedure.outcome.label == "arm not moved"


def test_leaving_the_step_mid_tool_step_holds_the_arm():
    """A step change cancels the task: the arm moving under the step is held, not left to run on unleased."""
    procedure, arm = Procedure(), Arm(stops=False)
    hand = HandMoves(procedure)
    ctx, run = setting(arm)

    async def scenario(tick):
        hand.add_step(STEP, PIVOT)
        for _ in range(5):
            await tick()

    play(hand, ctx, run, scenario)
    assert arm.steps and arm.holds == 1


def test_a_refused_tool_step_is_shown_and_drops_the_waiting_ones():
    """Near a singularity the arm refuses: the reason is shown and the queue cleared, nothing else is sent."""
    procedure, arm = Procedure(), Arm()
    arm.refuse = True
    hand = HandMoves(procedure)
    ctx, run = setting(arm)

    async def scenario(tick):
        hand.add_step(STEP, PIVOT)
        hand.add_step(STEP, PIVOT)
        for _ in range(10):
            await tick()

    play(hand, ctx, run, scenario)
    assert arm.steps == [] and hand._steps == []
    assert procedure.outcome == Check("tool step refused", BAD, "not moving: too near a singularity")


def test_free_drive_runs_while_the_button_is_held_and_ends_on_release():
    """Hold reports keep free drive on; without one for HOLD_TIMEOUT it ends and the joint controller holds the arm."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm)

    async def scenario(tick):
        for _ in range(6):
            hand.hold_free_drive(ctx.now())
            await tick()
        assert arm.controllers.is_active(FREE_DRIVE_CONTROLLER) and arm.requests >= 3 and not hand.free_drive
        for _ in range(10):
            await tick()
        assert arm.controllers.is_active(SCALED_JOINT_TRAJECTORY_CONTROLLER) and arm.holds == 1

    play(hand, ctx, run, scenario)
    assert procedure.outcome is None


def test_holding_free_drive_while_the_step_is_not_ready_is_refused():
    """With Setup's checklist unticked, the button says why and the arm stays stiff."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm)
    hand.start(ctx, run, lambda: "Setup checklist: tick it in Setup: E-stop at hand")
    hand.hold_free_drive(ctx.now())
    assert procedure.outcome.label == "free drive refused" and not hand._freeing(ctx.now())


def test_a_rejected_switch_is_shown_until_free_drive_runs_again():
    """The manager rejects the switch: "free drive ended" shows; the next hold that gets free drive clears it."""
    procedure, arm = Procedure(), Arm()
    arm.controllers.rejects = True
    hand = HandMoves(procedure)
    ctx, run = setting(arm)

    async def scenario(tick):
        hand.hold_free_drive(ctx.now())
        for _ in range(3):
            await tick()
        assert procedure.outcome.label == "free drive ended" and "rejected" in procedure.outcome.detail
        for _ in range(10):
            await tick()
        assert procedure.outcome is not None  # * shown until the arm moves by hand again
        for _ in range(4):
            hand.hold_free_drive(ctx.now())
            await tick()
        assert arm.controllers.is_active(FREE_DRIVE_CONTROLLER)

    play(hand, ctx, run, scenario)
    assert procedure.outcome is None


def test_moving_by_hand_leaves_another_outcome_alone():
    """An outcome the hand moves did not show, e.g. a failed take, stays when free drive runs."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm)
    failed = Check("take failed", BAD, "the arm refused the move")
    procedure.outcome = failed

    async def scenario(tick):
        for _ in range(4):
            hand.hold_free_drive(ctx.now())
            await tick()

    play(hand, ctx, run, scenario)
    assert procedure.outcome is failed


@pytest.mark.parametrize("ticked", [True, False])
def test_a_tool_step_keeps_a_clearance_only_while_clearance_is_ticked(ticked):
    """Ticked: the step goes out under a 1 cm clearance that refuses a start too close; unticked: under none."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    hand.clearance.value = ticked
    ctx, run = setting(arm)
    asked = []
    clearance = ctx.clearance

    def watched(*args, **kwargs):
        asked.append((args, kwargs))
        return clearance(*args, **kwargs)

    ctx.clearance = watched

    async def scenario(tick):
        hand.add_step(STEP, PIVOT)
        for _ in range(40):
            await tick()

    play(hand, ctx, run, scenario)
    assert len(arm.steps) == 1
    expected = [((["a200-0805"], ARM_STOP_DISTANCE),
                 {"unknown_arms": [], "refuse_first": True, "unknown_robots": frozenset()})]
    assert asked == (expected if ticked else [])


def test_a_tool_step_too_close_at_its_first_check_sends_nothing_and_says_why():
    """The clearance refuses the start (the monitor cancels the task): no step, the queue dropped, the reason shown."""
    procedure, arm = Procedure(), Arm()
    hand = HandMoves(procedure)
    ctx, run = setting(arm)
    wait_until = ctx.wait_until

    @contextmanager
    def refusing(*_args, **_kwargs):
        yield SimpleNamespace(checked=False, refused=True, problem="came within 1 cm of the punch target")

    async def cancelled_at_the_check(predicate, timeout_s=None, description=""):
        if "clearance" in description:
            raise asyncio.CancelledError  # * as the monitor cancels a task whose clearance refused its start
        await wait_until(predicate, timeout_s, description)

    ctx.clearance, ctx.wait_until = refusing, cancelled_at_the_check

    async def scenario(tick):
        hand.add_step(STEP, PIVOT)
        hand.add_step(STEP, PIVOT)
        for _ in range(10):
            await tick()

    play(hand, ctx, run, scenario)
    assert arm.steps == [] and hand._steps == []
    assert procedure.outcome == Check("tool step refused", BAD,
                                      "came within 1 cm of the punch target: untick Clearance to step closer")
