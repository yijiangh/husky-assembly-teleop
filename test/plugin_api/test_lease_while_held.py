"""No plugin gets a robot lease while any robot is held for another plugin's automatic motion (`_lease_slots`)."""

from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest
from robot_fakes import Log

from husky_assembly_teleop.plugin_api.context import PluginContext
from husky_assembly_teleop.robot_interface.robot_lease import LeaseSlot, NotLeased
from husky_assembly_teleop.shared.execution.interlock import give_back, take
from husky_assembly_teleop.world.measured import WorldState

pytestmark = pytest.mark.safety

NOW = 100.0


def _world() -> WorldState:
    """Alice and Cindy, free: no lease, nothing held, nothing measured."""
    world = WorldState()
    for serial in ("0804", "0806"):
        world.robots[serial] = SimpleNamespace(lease_slot=LeaseSlot(serial), lease=None, arms={},
                                               base=SimpleNamespace(state=SimpleNamespace(
                                                   follower=None, joint_states_update_time=None)))
    return world


def _context(name: str, world: WorldState) -> PluginContext:
    """Plugin `name`'s context over `world` at ROS time NOW."""
    log = Log()
    monitor = SimpleNamespace(world=world, now=lambda: NOW, log_warn=log.warning, log_info=log.info,
                              log_error=log.error, timer=SimpleNamespace(timed_task=lambda _name, work: work))
    return PluginContext(name, monitor, view=None, scene=None, kinematics=None)


def test_no_plugin_gets_a_lease_on_any_robot_while_one_is_held():
    """Cindy held by movement_exec: every other plugin's lease is refused, for Alice and Cindy alike; none is taken.

    robot_control's teleop lease, a planner's commit lease, an experiment's and the example plugin's: each refusal
    names the holder. Given back, they are granted.
    """
    world = _world()
    hold = take(world, "0806", "movement_exec", "B20_R_home", lambda: None, NOW)
    for name in ("robot_control", "base_planner", "base_exp", "example_lease_and_freeze"):
        ctx = _context(name, world)
        for serial in ("0804", "0806"):
            with pytest.raises(NotLeased, match="robot 0806 is held by movement_exec \\(B20_R_home\\) for an "
                                                "automatic motion, one motion at a time"):
                ctx.take_lease([serial], "teleop")
            with pytest.raises(NotLeased, match="held by movement_exec"):
                with ctx.lease_robots([serial], "commit plan 1"):
                    pass
    assert all(robot.lease_slot.lease is None for robot in world.robots.values())
    give_back(world, hold)
    lease = _context("robot_control", world).take_lease(["0804"], "teleop")
    assert lease.holder == "robot_control: teleop"


def test_the_holder_leases_the_robot_it_holds():
    """movement_exec, holding Cindy, takes her robot lease (and could take another's): the check is for others only."""
    world = _world()
    take(world, "0806", "movement_exec", "B20_R_home", lambda: None, NOW)
    lease = _context("movement_exec", world).take_lease(["0806"], "exec B20_R_home")
    assert world.robots["0806"].lease_slot.lease is lease and lease.holder == "movement_exec: exec B20_R_home"


def test_a_task_of_another_plugin_moves_nothing_while_a_robot_is_held():
    """The example plugin's pattern (`with ctx.lease_robots(...)` in a task) ends refused, not failed: nothing sent."""
    world = _world()
    take(world, "0806", "movement_exec", "B20_R_home", lambda: None, NOW)
    ctx = _context("example_lease_and_freeze", world)
    sent = []

    async def wiggle():
        with ctx.lease_robots(["0804"], "wiggle 0804 ur_arm"):
            sent.append("trajectory")

    async def scenario():
        task = ctx.spawn("wiggle", wiggle())
        await asyncio.gather(task, return_exceptions=True)
        return task

    task = asyncio.run(scenario())
    assert sent == [] and isinstance(task.exception(), NotLeased) and ctx._take_task_failures() == 0
