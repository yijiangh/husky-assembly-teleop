"""The demo's other parts: the action's steps, a path from the live joints, the start check, the grips, the hold."""

from __future__ import annotations

import asyncio
import math
from pathlib import Path

import numpy as np
import pytest
from planning_fixtures import DATA
from robot_lab import LEFT, RIGHT
from test_unscrew_retreat import ARMS, INSERTED, NAMES, START, Lab

from husky_assembly_teleop.drive import drive_root
from husky_assembly_teleop.plugins.insertion_demo.parts import (AT_START, demo_steps, live_rows, off_start,
                                                                run_compliant_hold, run_grip, tool0_at)
from husky_assembly_teleop.plugins.process.loading import load_process
from husky_assembly_teleop.robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER as COMPLIANCE
from husky_assembly_teleop.robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER as JOINT
from husky_assembly_teleop.robot_interface.end_effectors import ScaffoldingV3
from husky_assembly_teleop.shared.assembly_planning.process_steps import loaded_process
from husky_assembly_teleop.shared.execution.basis import STALE_ARM

pytestmark = pytest.mark.safety

#: The demo's process on the Drive: B6, a diagonal bar, into the jig on Cindy's base.
DEMO = "data_design_study/261104_SRD_insertion/process"


def demo_process():
    """The demo's process from the Drive; skips the test without it."""
    root = drive_root()
    if root is None or not (root / DEMO).is_dir():
        pytest.skip(f"needs HUSKY_DRIVE_ROOT with {DEMO}")
    return loaded_process(load_process(Path(root / DEMO), DATA))


# --- --- --- --- --- the process --- --- --- --- ---

def test_the_demo_steps_are_the_transfer_before_the_insert():
    """B6_J: the transfer (index 3) and the insert (index 4) of the schedule's sixth action; B6_R has no insert."""
    loaded = demo_process()
    transfer, insert = demo_steps(loaded, "B6_J")
    assert loaded.steps[transfer].movement.id == "B6_J_transfer"
    assert loaded.steps[insert].movement.id == "B6_J_insert"
    assert demo_steps(loaded, "B6_R") == "B6_R has no insert"


# --- --- --- --- --- where the arms are --- --- --- --- ---

def test_a_path_starts_from_the_live_joints_unless_they_are_too_far():
    """Row 0 becomes the measured joints; over STALE_ARM away the path is refused."""
    lab = Lab()
    rows = np.linspace(INSERTED + 0.01, START, 5)
    names, live = live_rows(lab.robot, NAMES, rows)
    np.testing.assert_allclose(live[0], INSERTED, atol=1e-12)
    np.testing.assert_allclose(live[1:], rows[1:])
    refused = live_rows(lab.robot, NAMES, np.linspace(INSERTED + 2 * STALE_ARM, START, 5))
    assert isinstance(refused, str) and "from the plan's start" in refused


def test_the_arms_stand_at_the_start_only_within_the_tolerance():
    """At the insert's start: ""; inserted (24 mm further): the arm named with its gap; a tiny gap passes."""
    lab = Lab()
    at_start = dict(zip(NAMES, START))
    off = off_start(lab.robot, at_start, ARMS)
    assert "left_ur_arm's tool0 is" in off and "press" not in off
    assert off_start(lab.robot, dict(zip(NAMES, INSERTED)), ARMS) == ""
    nearly = dict(zip(NAMES, INSERTED + 0.0005))
    assert off_start(lab.robot, nearly, ARMS) == ""
    assert AT_START == (0.005, pytest.approx(math.radians(1.0)))


def test_tool0_at_the_authored_joints_is_in_the_world_at_the_live_base():
    """The ends of the back-out: the same poses the lab builds for them; None without a base."""
    lab = Lab()
    ends = tool0_at(lab.robot, dict(zip(NAMES, START)), ARMS)
    for name, pose in lab.ends().items():
        np.testing.assert_allclose(ends[name].position, pose.position, atol=1e-12)
    lab.robot.base.state.position = None
    assert tool0_at(lab.robot, dict(zip(NAMES, START)), ARMS) is None


# --- --- --- --- --- the gripper screws --- --- --- --- ---

@pytest.mark.parametrize("close, verb", [(True, "close"), (False, "open")])
def test_a_grip_clears_runs_the_gripper_screws_until_they_stall_and_stops(close, verb):
    """STOP, then M1 one way on both tools, stalled after a second: stopped, said so."""
    lab = Lab(stall_after=1.0)
    ok, text = asyncio.run(run_grip(lab.ctx, lab.tools, close))
    assert ok and text.startswith(f"gripper screws {verb}d: stalled after left_ur_arm 1.")
    assert lab.steps() == ["STOP", f"{'TIGHTEN' if close else 'LOOSEN'} M1"]
    assert lab.events()[-2:] == ["left_ur_arm: STOP", "right_ur_arm: STOP"]


def test_a_grip_that_never_stalls_stops_the_motors_after_its_timeout():
    """No stall within the timeout: the motors are stopped and the grip reports it."""
    lab = Lab(stall_after=None)
    ok, text = asyncio.run(run_grip(lab.ctx, lab.tools, True, timeout_s=2.0))
    assert not ok and "not every gripper screw stalled within 2 s" in text
    assert lab.events()[-2:] == ["left_ur_arm: STOP", "right_ur_arm: STOP"]


# --- --- --- --- --- the compliant hold --- --- --- --- ---

def test_the_hold_zeroes_goes_compliant_and_comes_back_on_rigid():
    """Zero F/T, compliance, zero wrench; Rigid: hold and back under joint control."""
    lab = Lab()
    arms = list(lab.robot.arms.values())
    released = []

    async def scenario():
        task = asyncio.ensure_future(run_compliant_hold(lab.ctx, arms, lambda: bool(released), label="test"))
        await lab.ctx.sleep(1.0)
        assert lab.running() == [COMPLIANCE, COMPLIANCE]
        released.append(True)
        return await task

    assert asyncio.run(scenario()) == ""
    assert lab.steps() == ["zero F/T", "switch to compliance", "zero wrench", "hold", "switch to JTC"]
    assert lab.running() == [JOINT, JOINT]


def test_a_cancelled_hold_only_holds_and_leaves_the_arms_compliant():
    """Stop all while compliant: a hold each, no switch back; the log says to switch in robot_control."""
    lab = Lab()
    arms = list(lab.robot.arms.values())

    async def scenario():
        task = asyncio.ensure_future(run_compliant_hold(lab.ctx, arms, lambda: False, label="test"))
        await lab.ctx.sleep(1.0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    assert lab.running() == [COMPLIANCE, COMPLIANCE] and "switch to JTC" not in lab.steps()
    assert any("switch them back in robot_control" in line for line in lab.ctx.lines("warn"))


def test_the_lab_arms_start_where_the_test_says():
    """The fixture: the arms measured at INSERTED, both tools stalled, as after an insert."""
    lab = Lab()
    np.testing.assert_allclose(lab.robot.arms["left_ur_arm"].joint_vector(), np.array(LEFT) + np.array(
        [0.0, -0.03, 0.06, -0.03, 0.0, 0.0]))
    np.testing.assert_allclose(lab.robot.arms["right_ur_arm"].joint_vector(), np.array(RIGHT) + np.array(
        [0.0, -0.03, 0.06, -0.03, 0.0, 0.0]))
    assert all(tool.state.joint_motor == ScaffoldingV3.STALLED for tool in lab.tools.values())
