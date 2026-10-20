"""Leases, freezes and clearances in a plugin's context: leases carry its name, freezes and clearances cancel tasks."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import numpy as np
import pytest
from robot_fakes import Log, fake_arm

from husky_assembly_teleop.plugin_api.context import PluginContext
from husky_assembly_teleop.robot_interface.robot_lease import LeaseSlot, NotLeased
from husky_assembly_teleop.world.freeze import WorldMoved, take_basis
from husky_assembly_teleop.world.measured import WorldState

pytestmark = pytest.mark.safety

NOW = 100.0


def _context() -> tuple[PluginContext, WorldState, Log]:
    """A context for plugin "executor" over robot 0805 with one arm, at ROS time NOW, logging into the Log."""
    world, log = WorldState(), Log()
    world.robots["0805"] = SimpleNamespace(arms={"ur_arm": fake_arm(now=NOW)}, lease_slot=LeaseSlot("0805"))
    monitor = SimpleNamespace(world=world, now=lambda: NOW, log_warn=log.warning, log_info=log.info,
                              log_error=log.error, timer=SimpleNamespace(timed_task=lambda _name, work: work))
    return PluginContext("executor", monitor, view=None, scene=None, kinematics=None), world, log


def _turn(world: WorldState, degrees: float) -> None:
    """Turn the arm's first joint by `degrees`."""
    world.robots["0805"].arms["ur_arm"].state.joint_positions["ur_arm_shoulder_pan_joint"] += np.radians(degrees)


def test_a_plugins_lease_names_the_plugin():
    """The holder is "<plugin>: <label>", the plugin holds it inside the block, and a second lease is refused."""
    ctx, world, _log = _context()
    guard = world.robots["0805"].lease_slot
    with ctx.lease_robots(["0805"], "B20 home") as lease:
        assert lease.holder == "executor: B20 home" and guard.problem() is None
        with pytest.raises(NotLeased, match="robot 0805 is leased by executor: B20 home"):
            with ctx.lease_robots(["0805"], "again"):
                pass
    assert guard.lease is None


def test_a_lease_taken_across_ticks_is_given_back_when_the_plugin_stops():
    """`take_lease` keeps the robot until release; closing the plugin gives it back."""
    ctx, world, _log = _context()
    lease = ctx.take_lease(["0805"], "teleop")
    assert lease.holder == "executor: teleop" and world.robots["0805"].lease_slot.lease is lease
    ctx._close()
    assert world.robots["0805"].lease_slot.lease is None


def test_a_broken_freeze_cancels_its_task_and_says_why():
    """Within the tolerance the task runs on; past it, the next check cancels the task and logs the reason."""
    ctx, world, log = _context()
    seen = {}

    async def motion() -> None:
        with ctx.freeze(take_basis(world, NOW, arms=[("0805", "ur_arm")])) as freeze:
            seen["freeze"] = freeze
            while True:
                await ctx.next_tick()

    async def scenario() -> asyncio.Task:
        task = ctx.spawn("motion", motion())
        await asyncio.sleep(0)
        _turn(world, 0.5)
        ctx._check_freezes()
        ctx._wake_tick_waiters()
        await asyncio.sleep(0)
        assert not task.done()
        _turn(world, 1.0)
        ctx._check_freezes()
        await asyncio.gather(task, return_exceptions=True)
        return task

    task = asyncio.run(scenario())
    assert task.cancelled() and seen["freeze"].problem == "robot 0805 arm ur_arm moved 1.5 deg"
    assert "'executor: motion': robot 0805 arm ur_arm moved 1.5 deg; cancelling it" in log.text("warning")
    assert not ctx._freezes


def test_a_freeze_already_broken_refuses_at_once():
    """Moved between measuring and watching: WorldMoved before the block runs."""
    ctx, world, _log = _context()

    async def motion() -> None:
        basis = take_basis(world, NOW, arms=[("0805", "ur_arm")])
        _turn(world, 5.0)
        with ctx.freeze(basis):
            raise AssertionError("must not run")

    async def scenario() -> None:
        with pytest.raises(WorldMoved, match="moved 5.0 deg"):
            await ctx.spawn("motion", motion())

    asyncio.run(scenario())


def test_a_freeze_needs_a_task_of_its_plugin():
    """Outside a task there is nothing to cancel."""
    ctx, world, _log = _context()
    with pytest.raises(RuntimeError, match="only inside a task"):
        with ctx.freeze(take_basis(world, NOW, arms=[("0805", "ur_arm")])):
            pass


def test_a_tripped_clearance_soft_stops_the_robot_and_cancels_its_task():
    """The robot is soft stopped, the task cancelled with the reason kept and logged; a late trip changes nothing."""
    ctx, world, log = _context()
    stops = []
    world.robots["0805"].soft_stop = lambda: stops.append("0805")
    seen = {}

    async def drive() -> None:
        with ctx.clearance(["0805"]) as clearance:
            seen["clearance"] = clearance
            while True:
                await ctx.next_tick()

    async def scenario() -> asyncio.Task:
        task = ctx.spawn("drive", drive())
        await asyncio.sleep(0)
        clearance, = ctx._clearances
        ctx._trip_clearance(clearance, "0805", "came within 10 cm of wall")
        await asyncio.gather(task, return_exceptions=True)
        ctx._trip_clearance(clearance, "0805", "came within 10 cm of the rack")
        return task

    task = asyncio.run(scenario())
    assert task.cancelled() and stops == ["0805"] and not ctx._clearances
    assert seen["clearance"].problem == "robot 0805 came within 10 cm of wall"
    assert "'executor: drive': robot 0805 came within 10 cm of wall; soft stopped it" in log.text("error")


def test_a_clearance_needs_a_task_of_its_plugin():
    """Outside a task there is nothing to cancel."""
    ctx, _world, _log = _context()
    with pytest.raises(RuntimeError, match="clearance\\(\\) runs only inside a task"):
        with ctx.clearance(["0805"]):
            pass
