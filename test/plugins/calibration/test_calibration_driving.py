"""The calibration's D-pad: a press starts the drive task again after a clearance trip or Stop all ended it."""

from __future__ import annotations

import asyncio
from contextlib import contextmanager
from types import SimpleNamespace

import pytest

from husky_assembly_teleop.plugins.calibration.robot.driving import Driving
from husky_assembly_teleop.robot_interface.soft_stop import RobotStopped
from husky_assembly_teleop.shared.guide.procedure import Procedure
from husky_assembly_teleop.shared.teleop.dpad import UNGUARDED_COLOR, Hold
from husky_assembly_teleop.world.checks import BAD, Check

pytestmark = pytest.mark.safety


def hold(now: float) -> Hold:
    """Forward, reported at `now`."""
    return Hold(1.0, 0.0, SimpleNamespace(value=0.5), now)


def test_a_press_starts_the_ended_drive_task_again_while_a_step_drives():
    """An ended task (clearance trip, Stop all) is spawned again by the next press; not once the step is left."""
    spawned = []

    def spawn(name, work):
        work.close()
        spawned.append(name)
        return SimpleNamespace(done=lambda: True, cancel=lambda: None)

    ctx = SimpleNamespace(spawn=spawn)
    run = SimpleNamespace(serial="a200-0805")
    driving = Driving(Procedure())
    driving.on_hold(hold(0.0))
    assert spawned == []
    driving.start(ctx, run)
    driving.on_hold(hold(0.1))
    assert spawned == ["drive a200-0805"] * 2
    driving.stop()
    driving.on_hold(hold(0.2))
    assert len(spawned) == 2


def test_a_soft_stopped_robot_refuses_the_press_and_the_task_goes_on():
    """While soft stopped, a press says why it did nothing; the task waits for the release and keeps running."""
    clock = SimpleNamespace(now=0.0)
    procedure = Procedure()
    driving = Driving(procedure)
    sent = []

    def send_twist(linear, angular, require_controller=True):
        if linear or angular:
            raise RobotStopped("a200-0805 is soft stopped: Resume it")
        sent.append((linear, angular))
        return True

    base = SimpleNamespace(send_twist=send_twist, controllers=SimpleNamespace(is_active=lambda _name: True))
    run = SimpleNamespace(serial="a200-0805", robot=SimpleNamespace(base=base), save=lambda: None)

    @contextmanager
    def held(*_args, **_kwargs):
        yield None

    async def next_tick():
        clock.now += 0.05
        await asyncio.sleep(0)

    async def wait_until(predicate, timeout_s=None, description=""):
        while not predicate():
            await next_tick()

    ctx = SimpleNamespace(now=lambda: clock.now, next_tick=next_tick, wait_until=wait_until, lease_robots=held,
                          clearance=held, scene=SimpleNamespace(snapshot=SimpleNamespace(robots={})))

    async def scenario():
        task = asyncio.get_running_loop().create_task(driving._drive(ctx, run))
        driving.on_hold(hold(clock.now))
        for _ in range(20):
            await next_tick()
        refused = Check("D-pad refused", BAD, "a200-0805 is soft stopped: Resume it")
        assert not task.done() and procedure.outcome == refused
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task
        assert sent[-1] == (0.0, 0.0)

    asyncio.run(scenario())


def test_a_press_while_the_step_is_not_ready_drives_nothing_and_says_why():
    """While a robot move is refused (Area clear not confirmed), the D-pad sends no twist and shows why."""
    clock = SimpleNamespace(now=0.0)
    procedure = Procedure()
    driving = Driving(procedure)
    sent = []
    base = SimpleNamespace(send_twist=lambda *twist, **_kwargs: sent.append(twist) or True,
                           controllers=SimpleNamespace(is_active=lambda _name: True))
    run = SimpleNamespace(serial="a200-0805", robot=SimpleNamespace(base=base), save=lambda: None)

    async def next_tick():
        clock.now += 0.05
        await asyncio.sleep(0)

    async def wait_until(predicate, timeout_s=None, description=""):
        while not predicate():
            await next_tick()

    def spawn(_name, work):
        work.close()
        return SimpleNamespace(done=lambda: True, cancel=lambda: None)

    ctx = SimpleNamespace(now=lambda: clock.now, next_tick=next_tick, wait_until=wait_until, spawn=spawn)
    driving.start(ctx, run, lambda: "Area clear not ticked")

    async def scenario():
        task = asyncio.get_running_loop().create_task(driving._drive(ctx, run))
        driving.on_hold(hold(clock.now))
        for _ in range(3):
            await next_tick()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    assert procedure.outcome == Check("D-pad refused", BAD, "Area clear not ticked")
    assert all(twist[:2] == (0.0, 0.0) for twist in sent)


@pytest.mark.parametrize("ticked", [True, False])
def test_a_hold_drives_under_a_clearance_only_while_clearance_is_ticked(ticked):
    """Ticked: the hold drives inside the base's clearance, others not measured as stand-ins; unticked: none, amber."""
    clock = SimpleNamespace(now=0.0)
    driving = Driving(Procedure())
    driving.clearance.value = ticked
    driving.buttons = [SimpleNamespace(color=None)]
    sent, guarded = [], []
    base = SimpleNamespace(send_twist=lambda *twist, **_kwargs: sent.append(twist) or True,
                           controllers=SimpleNamespace(is_active=lambda _name: True))
    run = SimpleNamespace(serial="a200-0805", robot=SimpleNamespace(base=base), save=lambda: None)

    @contextmanager
    def lease(*_args):
        yield None

    @contextmanager
    def clearance(serials, unknown_robots=(), check_self=True):
        guarded.append((serials, unknown_robots, check_self))
        yield None

    async def next_tick():
        clock.now += 0.05
        await asyncio.sleep(0)

    alice = SimpleNamespace(enabled=True, acting_problems=lambda: ["base not measured"])
    ctx = SimpleNamespace(now=lambda: clock.now, next_tick=next_tick, lease_robots=lease, clearance=clearance,
                          scene=SimpleNamespace(snapshot=SimpleNamespace(robots={"robots/a200-0804": alice})))

    async def scenario():
        task = asyncio.get_running_loop().create_task(driving._drive(ctx, run))
        driving.on_hold(hold(clock.now))
        for _ in range(3):
            await next_tick()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    expected = [(["a200-0805"], frozenset({"robots/a200-0804"}), False)]  # * Alice, not measured, as a stand-in
    assert (0.15, 0.0) in sent and guarded == (expected if ticked else [])
    driving.draw()
    assert driving.buttons[0].color == (None if ticked else UNGUARDED_COLOR)


@pytest.mark.parametrize("waits", [True, False])
def test_the_dpad_drives_while_a_procedure_only_waits_for_go_and_not_while_it_works(waits):
    """Waiting for Go nothing moves, so the base may be driven, e.g. to another validation spot; working, it may not."""
    clock = SimpleNamespace(now=0.0)
    procedure = Procedure()
    procedure._task = SimpleNamespace(done=lambda: False)  # * a procedure runs
    procedure.prompt = "move to validation pose 3?" if waits else ""
    driving, sent = Driving(procedure), []
    base = SimpleNamespace(send_twist=lambda *twist, **_kwargs: sent.append(twist) or True,
                           controllers=SimpleNamespace(is_active=lambda _name: True))
    run = SimpleNamespace(serial="a200-0805", robot=SimpleNamespace(base=base), save=lambda: None)

    @contextmanager
    def held(*_args, **_kwargs):
        yield None

    async def next_tick():
        clock.now += 0.05
        await asyncio.sleep(0)

    async def wait_until(predicate, timeout_s=None, description=""):
        while not predicate():
            await next_tick()

    ctx = SimpleNamespace(now=lambda: clock.now, next_tick=next_tick, wait_until=wait_until, lease_robots=held,
                          clearance=held, scene=SimpleNamespace(snapshot=SimpleNamespace(robots={})))

    async def scenario():
        task = asyncio.get_running_loop().create_task(driving._drive(ctx, run))
        driving.on_hold(hold(clock.now))
        for _ in range(3):
            await next_tick()
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    assert ((0.15, 0.0) in sent) is waits
    assert waits or procedure.outcome.label == "D-pad off"
