"""Robot leases: all robots or none, given back however the block ends, held by the task that took them."""

from __future__ import annotations

import asyncio

import pytest

from husky_assembly_teleop.robot_interface.robot_lease import LeaseSlot, NotLeased, leased, take_lease

pytestmark = pytest.mark.safety


def test_a_free_robot_takes_no_commands():
    """Nobody holds its lease: commands are refused until someone leases it."""
    guard = LeaseSlot("0804")
    with pytest.raises(NotLeased, match="robot 0804 is not leased; take its lease first, a twist not sent"):
        guard.check("a twist")


def test_a_lease_takes_every_robot_or_none():
    """One robot already leased: the whole lease is refused, naming the holder, and the free robot stays free."""
    first, second = LeaseSlot("0804"), LeaseSlot("0805")
    with leased([first], "planner"):
        with pytest.raises(NotLeased, match="robot 0804 is leased by planner"):
            with leased([second, first], "executor"):
                pass
        assert second.lease is None and first.lease.holder == "planner"
    assert first.lease is None


def test_the_lease_is_given_back_when_the_block_raises():
    """An error inside the block still gives the robot back."""
    guard = LeaseSlot("0804")
    with pytest.raises(ValueError):
        with leased([guard], "executor"):
            raise ValueError("planning failed")
    assert guard.lease is None


def test_a_lease_taken_across_ticks_holds_only_inside_use():
    """`take_lease` holds the robots until release; commands pass only inside `use`."""
    guard = LeaseSlot("0804")
    lease = take_lease([guard], "teleop")
    assert guard.problem() == "robot 0804 is leased by teleop" and lease.active
    with lease.use():
        assert guard.problem() is None
    lease.release()
    assert not lease.active and guard.lease is None


def test_only_the_holding_task_and_tasks_it_spawns_may_command_the_robot():
    """A task spawned in the block holds the lease; one spawned before does not; after release, nobody holds it."""
    guard = LeaseSlot("0804")

    async def problem_now() -> str | None:
        return guard.problem()

    async def scenario() -> None:
        outside = asyncio.get_running_loop().create_future()
        stranger = asyncio.create_task(_wait_then(outside, problem_now))
        with leased([guard], "executor"):
            assert guard.problem() is None
            assert await asyncio.create_task(problem_now()) is None
            outside.set_result(None)
            assert await stranger == "robot 0804 is leased by executor"
            child = asyncio.get_running_loop().create_future()
            late_child = asyncio.create_task(_wait_then(child, problem_now))
        with leased([guard], "calibration"):
            child.set_result(None)
            assert await late_child == "robot 0804 is leased by calibration"

    asyncio.run(scenario())


async def _wait_then(gate: asyncio.Future, then):
    """Wait for `gate`, then return what `then()` returns."""
    await gate
    return await then()
