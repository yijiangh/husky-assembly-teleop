"""Leases, freezes and clearances in a plugin's context: leases carry its name, freezes and clearances cancel tasks."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import numpy as np
import pytest
from robot_fakes import Log, fake_arm

from husky_assembly_teleop.plugin_api.context import PluginContext
from husky_assembly_teleop.robot_interface.robot_lease import LeaseSlot, NotLeased
from husky_assembly_teleop.world.clearance import ALL_ARMS, ClearanceRefused
from husky_assembly_teleop.world.freeze import WorldMoved, take_basis
from scaffolding_assembly_core.geometry import Pose
from scaffolding_assembly_core.ids import robot_id
from scaffolding_assembly_core.robot import RobotObject
from scaffolding_assembly_core.scene import Scene
from husky_assembly_teleop.world.measured import WorldState

pytestmark = pytest.mark.safety

NOW = 100.0


#: Robot 0805's arm joints, for its model in the snapshot.
ARM_JOINTS = ("ur_arm_shoulder_pan_joint", "ur_arm_elbow_joint")


def _context(unmeasured: frozenset = frozenset()) -> tuple[PluginContext, WorldState, Log]:
    """A context for plugin "executor" over robot 0805 with one arm, at ROS time NOW, logging into the Log.

    The snapshot has 0805 tracked, with `unmeasured` joints not known.
    """
    world, log = WorldState(), Log()
    world.robots["0805"] = SimpleNamespace(arms={"ur_arm": fake_arm(now=NOW)}, lease_slot=LeaseSlot("0805"), lease=None)
    # * The monitor's log functions also take the source, here the plugin's name.
    monitor = SimpleNamespace(world=world, now=lambda: NOW, log_warn=lambda text, _source: log.warning(text),
                              log_info=lambda text, _source: log.info(text),
                              log_error=lambda text, _source: log.error(text),
                              timer=SimpleNamespace(timed_task=lambda _name, work: work), clearance_trips=(),
                              config=SimpleNamespace(robots=()))
    monitor.clearance_tripped = lambda trip: setattr(monitor, "clearance_trips", (*monitor.clearance_trips, trip))
    robot = RobotObject(robot_id("0805"), SimpleNamespace(movable_joints=ARM_JOINTS), Pose(), {},
                        unmeasured=unmeasured)
    scene = SimpleNamespace(snapshot=Scene(robots={robot.id: robot}))
    return PluginContext("executor", monitor, view=None, scene=scene, kinematics=None), world, log


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
    world.robots["0805"].soft_stop = lambda reason="": stops.append(("0805", reason))
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
    assert task.cancelled() and stops == [("0805", "robot 0805 came within 10 cm of wall")] and not ctx._clearances
    assert seen["clearance"].problem == "robot 0805 came within 10 cm of wall"
    assert "'executor: drive': robot 0805 came within 10 cm of wall; soft stopped it" in log.text("error")
    trip, = ctx.clearance_trips
    assert (trip.serial, trip.task, trip.problem, trip.hit) == (
        "0805", "executor: drive", "robot 0805 came within 10 cm of wall", None)


def test_a_clearance_needs_a_task_of_its_plugin():
    """Outside a task there is nothing to cancel."""
    ctx, _world, _log = _context()
    with pytest.raises(RuntimeError, match="clearance\\(\\) runs only inside a task"):
        with ctx.clearance(["0805"]):
            pass


def test_a_clearance_refuses_to_start_on_a_robot_it_cannot_check():
    """Arm joints not known: a strict clearance raises before its block runs; one allowing the arm starts."""
    ctx, _world, _log = _context(unmeasured=frozenset({"ur_arm_elbow_joint"}))
    ran = []

    async def drive(**allowed) -> None:
        with ctx.clearance(["0805"], **allowed):
            ran.append(allowed)

    async def scenario():
        strict = ctx.spawn("strict", drive())
        lenient = ctx.spawn("lenient", drive(unknown_arms=ALL_ARMS))
        return await asyncio.gather(strict, lenient, return_exceptions=True)

    strict, lenient = asyncio.run(scenario())
    assert isinstance(strict, ClearanceRefused) and "joints not known: ur_arm_elbow_joint" in str(strict)
    assert lenient is None and ran == [{"unknown_arms": ALL_ARMS}]


def test_another_robot_not_measured_refuses_a_clearance_unless_allowed():
    """Robot 0804 was never seen: refused; allowed, the clearance starts, logs that it stands at its default pose."""
    ctx, _world, log = _context()
    other = RobotObject(robot_id("0804"), SimpleNamespace(movable_joints=ARM_JOINTS), Pose(), {}, base_tracked=False)
    ctx.scene.snapshot.robots[other.id] = other
    kept = []

    async def drive(**allowed) -> None:
        with ctx.clearance(["0805"], **allowed) as clearance:
            kept.extend(clearance.stand_ins)

    async def scenario():
        refused = await asyncio.gather(ctx.spawn("strict", drive()), return_exceptions=True)
        await ctx.spawn("allowed", drive(unknown_robots={other.id}))
        return refused

    (refused,) = asyncio.run(scenario())
    assert isinstance(refused, ClearanceRefused)
    assert str(refused) == ("robot 0805 cannot be checked against the others: robots/0804 is not measured "
                            "(its base is not tracked)")
    line, = kept
    assert line == "robots/0804 is not measured (its base is not tracked): checked at its default pose or stowed"
    assert f"clearance on 0805: {line}" in log.text("warning")
