"""The calibration's arm motions: a move that does not arrive in time holds the arm before its lease is given back."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import numpy as np
import pytest

from scaffolding_assembly_core.geometry import Pose

from husky_assembly_teleop.plugin_api.concurrency import WaitTimeout
from husky_assembly_teleop.plugins.calibration.robot.motion import ArmMotion, sweep_path
from husky_assembly_teleop.shared.guide.procedure import Procedure

pytestmark = pytest.mark.safety


class StuckArm:
    """Takes every move and never arrives; counts holds."""

    def __init__(self):
        """At zero, at rest."""
        self.state = SimpleNamespace(is_executing=False, last_motion_time=None, last_slight_motion_time=None,
                                     last_update_time=None)
        self.holds, self.last_refusal = 0, None

    def joint_vector(self) -> np.ndarray:
        """Never moves."""
        return np.zeros(6)

    def send_joint_move(self, _joints, min_duration: float) -> float:
        """Taken; runs for ever."""
        self.state.is_executing = True
        return min_duration

    def hold(self) -> None:
        """Count it."""
        self.holds += 1


def test_a_sweep_step_that_never_arrives_holds_the_arm_and_fails():
    """step_to times out after the move's duration and margin: the arm is held, then the take fails."""
    clock = SimpleNamespace(now=0.0)

    async def wait_until(predicate, timeout_s=None, description=""):
        start = clock.now
        while not predicate():
            if timeout_s is not None and clock.now - start > timeout_s:
                raise WaitTimeout(description)
            clock.now += 0.05
            await asyncio.sleep(0)

    arm = StuckArm()
    ctx = SimpleNamespace(now=lambda: clock.now, wait_until=wait_until)
    run = SimpleNamespace(arm="ur_arm", robot=SimpleNamespace(arms={"ur_arm": arm}))
    with pytest.raises(WaitTimeout):
        asyncio.run(ArmMotion(Procedure()).step_to(ctx, run, np.full(6, 0.1)))
    assert arm.holds == 1


def test_a_move_after_go_pressed_twice_lets_the_other_robots_not_measured_be_unknown():
    """With Alice not measured, Go asks twice; the commit then lets her be unknown, as the planner panel's does."""
    alice = SimpleNamespace(enabled=True, acting_problems=lambda: ["base not measured"])
    committed = []

    async def request_plan(_request):
        return SimpleNamespace(state="ready", message="clear", id=7, path=SimpleNamespace(duration=1.0))

    async def commit(plan_id, unknown_robots=(), clearance=True):
        committed.append((plan_id, frozenset(unknown_robots)))
        return False, "stopped here by the test"

    async def wait_until(predicate, timeout_s=None, description=""):
        while not predicate():
            await asyncio.sleep(0)

    planner = SimpleNamespace(request_plan=request_plan, commit=commit, close_note=lambda _plan: "")
    clock = SimpleNamespace(now=0.0)
    ctx = SimpleNamespace(now=lambda: clock.now, wait_until=wait_until, require={"arm_planner": planner}.__getitem__,
                          kinematics=SimpleNamespace(base_pose=lambda _serial: Pose()),
                          scene=SimpleNamespace(snapshot=SimpleNamespace(robots={"robots/a200-0804": alice})))
    run = SimpleNamespace(serial="a200-0805", arm="ur_arm", robot=SimpleNamespace(arms={"ur_arm": StuckArm()}))
    procedure = Procedure()

    async def scenario():
        move = asyncio.get_running_loop().create_task(
            ArmMotion(procedure).move_to(ctx, run, np.full(6, 0.1), "the start of take 1"))
        await asyncio.sleep(0)
        procedure.press_go(ctx)
        await asyncio.sleep(0)
        assert committed == [] and procedure.outcome.label == "press Go again"
        clock.now = 1.0  # * after a double click's time
        procedure.press_go(ctx)
        with pytest.raises(RuntimeError, match="stopped here by the test"):
            await move

    asyncio.run(scenario())
    assert committed == [(7, frozenset({"robots/a200-0804"}))]


def test_a_sweeps_preview_stops_at_each_waypoint_and_moves_each_step_as_the_arm_does():
    """1.5 s still at each waypoint (0.5 s at rest, 20 readings); each step slows to rest, 12° in the 1 s minimum."""
    waypoints = np.zeros((3, 6))
    waypoints[1, 0], waypoints[2, 0] = np.radians(12.0), np.radians(72.0)
    path = sweep_path(waypoints, tick_period=0.05)
    stop, long_step = 1.5, np.pi / 2 * 60.0 / 20.0  # * 60° at 20°/s peak
    assert path.duration == pytest.approx(3 * stop + 1.0 + long_step)
    np.testing.assert_allclose(path.sample(stop / 2), waypoints[0])
    np.testing.assert_allclose(path.sample(stop + 1.0 + stop / 2), waypoints[1])
    # * 0.2 s into the 1 s step: (1 - cos(0.2 pi)) / 2 of the way, not a straight 20 %.
    assert np.degrees(path.sample(stop + 0.2)[0]) == pytest.approx(12.0 * (1 - np.cos(0.2 * np.pi)) / 2)


@pytest.mark.safety
@pytest.mark.parametrize("answered", [True, False])
def test_cancel_during_a_planned_move_stops_the_planners_motion_on_that_robot(answered):
    """Cancel while the arm moves, or before the planner said it sent: the planner's commit on that robot is cancelled.

    Once moving, the arm is held too.
    """
    cancelled = []

    async def request_plan(_request):
        return SimpleNamespace(state="ready", message="clear", id=7, path=SimpleNamespace(duration=1.0))

    async def commit(_plan_id, unknown_robots=(), clearance=True):
        if not answered:
            await asyncio.get_running_loop().create_future()  # * still leasing or checking the start
        return True, "sent"

    async def wait_until(predicate, timeout_s=None, description=""):
        while not predicate():
            await asyncio.sleep(0)

    planner = SimpleNamespace(request_plan=request_plan, commit=commit, cancel_commit=cancelled.append,
                              close_note=lambda _plan: "")
    ctx = SimpleNamespace(now=lambda: 0.0, wait_until=wait_until, require={"arm_planner": planner}.__getitem__,
                          kinematics=SimpleNamespace(base_pose=lambda _serial: Pose()),
                          scene=SimpleNamespace(snapshot=SimpleNamespace(robots={})))
    arm = StuckArm()
    run = SimpleNamespace(serial="a200-0805", arm="ur_arm", robot=SimpleNamespace(arms={"ur_arm": arm}))
    procedure = Procedure()
    procedure.auto = True  # * every other robot measured: no Go

    async def scenario():
        move = asyncio.get_running_loop().create_task(
            ArmMotion(procedure).move_to(ctx, run, np.full(6, 0.1), "the start of take 1"))
        for _ in range(5):
            await asyncio.sleep(0)
        assert procedure.doing.startswith("moving" if answered else "planning")
        move.cancel()
        with pytest.raises(asyncio.CancelledError):
            await move

    asyncio.run(scenario())
    assert arm.holds == int(answered) and cancelled == ["a200-0805"]


@pytest.mark.safety
@pytest.mark.parametrize("ticked", [True, False])
def test_gos_clearance_box_says_whether_a_move_keeps_the_planners_clearance(ticked):
    """The planner's commit gets Go's Clearance: ticked, the move keeps its clearance; unticked, nothing stops it."""
    committed = []

    async def request_plan(_request):
        return SimpleNamespace(state="ready", message="clear", id=7, path=SimpleNamespace(duration=1.0))

    async def commit(_plan_id, unknown_robots=(), clearance=True):
        committed.append(clearance)
        return False, "stopped here by the test"

    planner = SimpleNamespace(request_plan=request_plan, commit=commit, close_note=lambda _plan: "")
    ctx = SimpleNamespace(now=lambda: 0.0, require={"arm_planner": planner}.__getitem__,
                          kinematics=SimpleNamespace(base_pose=lambda _serial: Pose()),
                          scene=SimpleNamespace(snapshot=SimpleNamespace(robots={})))
    run = SimpleNamespace(serial="a200-0805", arm="ur_arm", robot=SimpleNamespace(arms={"ur_arm": StuckArm()}))
    procedure = Procedure()
    procedure.auto, procedure.clearance.value = True, ticked  # * every other robot measured: no Go
    with pytest.raises(RuntimeError, match="stopped here by the test"):
        asyncio.run(ArmMotion(procedure).move_to(ctx, run, np.full(6, 0.1), "the start of take 1"))
    assert committed == [ticked]
