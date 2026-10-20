"""Sending a motion to fake arms that follow it: arrived, refused, timed out, stopped short, cancelled, a dry run."""

from __future__ import annotations

import asyncio

import numpy as np
import pytest
from plugin_fakes import ArmDriver, FakeContext, fake_arm, fake_node, fake_robot

from husky_assembly_teleop.shared.execution.send import (ARRIVED, ARRIVE_MARGIN, DRY_RUN, HOLD_NOT_SENT, REFUSED,
                                                         STOPPED, TIMEOUT, motion_command, send_motion)
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names

pytestmark = pytest.mark.safety

START = np.array([0.0, -1.5, 1.5, 0.0, 0.5, 0.0])
CINDY_ARMS = ("left_ur_arm", "right_ur_arm")


class Lab:
    """Cindy (two arms) and Alice (one), at START, each arm driven by an `ArmDriver`."""

    def __init__(self, **driver):
        """Build the robots; `driver` options go to every arm's driver (follow, silent_from, speed)."""
        self.ctx = FakeContext(step=self.tick)
        node = fake_node(self.ctx.now, self.ctx.logs)
        self.cindy = fake_robot(node, "a200-0806", [fake_arm(node, name, START) for name in CINDY_ARMS])
        self.alice = fake_robot(node, "a200-0804", [fake_arm(node, "ur_arm", START)])
        self.drivers = [ArmDriver(arm, robot, **driver) for robot in (self.cindy, self.alice)
                        for arm in robot.arms.values()]

    def tick(self, t: float) -> None:
        """Every arm moves and reports."""
        for driver in self.drivers:
            driver.tick(t)

    def run(self, robot, command, **options):
        """Send and wait, inside an event loop."""
        async def scenario():
            return await send_motion(self.ctx, robot, command, **options)
        return asyncio.run(scenario())

    def holds(self, robot) -> list[int]:
        """How many single-point (hold) trajectories each arm of `robot` got."""
        return [sum(len(message.points) == 1 for message in arm._trajectory.messages) for arm in robot.arms.values()]


def cindy_path(arms=CINDY_ARMS) -> tuple[list[str], np.ndarray]:
    """A three-row path for Cindy's arms from START, its columns in tamp's order (left arm, then right)."""
    names = [name for arm in arms for name in arm_joint_names(arm)]
    rows = np.tile(START, (3, len(arms)))
    rows[1, 0] += 0.3
    rows[2, 0] += 0.3
    rows[2, 2] -= 0.2
    return names, rows


def test_both_arms_arrive_through_one_multi_arm_message_with_velocities():
    """A two-arm command goes out once, with each arm's velocities, and arrives within 0.5°."""
    lab = Lab()
    names, rows = cindy_path()
    command = motion_command("a200-0806", CINDY_ARMS, names, rows, 3.0)
    outcome = lab.run(lab.cindy, command)
    assert outcome.status == ARRIVED and outcome.ok and max(outcome.end_error.values()) < np.radians(0.5)
    message, = lab.cindy._multi_arm.messages
    assert np.allclose([point.velocities for point in message.trajectory1.points], command.arms[0].velocities)
    assert not any(arm._trajectory.messages for arm in lab.cindy.arms.values())
    assert outcome.seconds >= command.duration and np.allclose(outcome.end["left_ur_arm"], rows[-1, :6], atol=1e-6)


def test_one_arm_goes_alone_and_columns_are_matched_by_name():
    """Alice's path, its columns shuffled, is put into UR order and sent on her arm's own trajectory topic."""
    lab = Lab()
    names = arm_joint_names("ur_arm")
    target = START + [0.2, 0.0, -0.1, 0.0, 0.0, 0.1]
    order = [5, 3, 1, 0, 2, 4]
    command = motion_command("a200-0804", ("ur_arm",), [names[i] for i in order],
                             np.array([START, target])[:, order], 1.0)
    assert np.allclose(command.arms[0].positions[-1], target)
    assert lab.run(lab.alice, command).status == ARRIVED
    assert len(lab.alice.arms["ur_arm"]._trajectory.messages) == 1


@pytest.mark.parametrize("arms, names, why", [
    (CINDY_ARMS, [*arm_joint_names("left_ur_arm"), "base_joint"], "belong to no arm"),
    (CINDY_ARMS, arm_joint_names("left_ur_arm")[:4], "only partly"),
    (("ur_arm",), arm_joint_names("left_ur_arm"), "belong to no arm"),
])
def test_joints_that_are_not_the_robots_arms_are_refused(arms, names, why):
    """A joint of no configured arm, or an arm only partly named, never makes a command."""
    rows = np.zeros((2, len(names)))
    rows[1, 0] = 0.1
    with pytest.raises(ValueError, match=why):
        motion_command("a200-0806", arms, names, rows, 1.0)


def test_a_refused_trajectory_sends_nothing():
    """With the trajectory controller off, the arms refuse; nothing is published, nothing held."""
    lab = Lab()
    for arm in lab.cindy.arms.values():
        arm.state.controllers.active = None
    names, rows = cindy_path()
    outcome = lab.run(lab.cindy, motion_command("a200-0806", CINDY_ARMS, names, rows, 1.0))
    assert outcome.status == REFUSED and not outcome.ok and outcome.end is None
    assert not lab.cindy._multi_arm.messages and lab.holds(lab.cindy) == [0, 0]


def test_a_command_for_another_robot_is_refused():
    """Cindy's command sent to Alice sends nothing."""
    lab = Lab()
    names, rows = cindy_path()
    outcome = lab.run(lab.alice, motion_command("a200-0806", CINDY_ARMS, names, rows, 1.0))
    assert outcome.status == REFUSED and "not a200-0804" in outcome.reason
    assert not lab.alice.arms["ur_arm"]._trajectory.messages


def test_too_slow_times_out_and_holds_every_arm():
    """Arms running at half speed are still moving duration + 5 s after sending: TIMEOUT, every arm held."""
    lab = Lab(speed=0.3)
    names, rows = cindy_path()
    command = motion_command("a200-0806", CINDY_ARMS, names, rows, 8.0)
    outcome = lab.run(lab.cindy, command)
    assert outcome.status == TIMEOUT and outcome.seconds >= command.duration + ARRIVE_MARGIN - 1e-6
    assert lab.holds(lab.cindy) == [1, 1]


def test_stopped_half_way_holds():
    """Arms that stop at half the path are STOPPED a second after the time is up, with what remains, and held."""
    lab = Lab(follow=0.5)
    names, rows = cindy_path()
    outcome = lab.run(lab.cindy, motion_command("a200-0806", CINDY_ARMS, names, rows, 2.0))
    assert outcome.status == STOPPED and "short of the end" in outcome.reason
    assert max(outcome.end_error.values()) > np.radians(5.0) and lab.holds(lab.cindy) == [1, 1]


def test_lost_joint_states_are_said_once_then_the_unsent_hold_is_the_outcome():
    """Joint states stopping mid-motion: one error line, a timeout, then the unsent holds are the outcome.

    The arms refuse to hold on stale joints: HOLD_NOT_SENT, naming the arms and the pendant / e-stop, never "held".
    """
    lab = Lab(silent_from=10.5)
    names, rows = cindy_path()
    outcome = lab.run(lab.cindy, motion_command("a200-0806", CINDY_ARMS, names, rows, 2.0))
    lost = [line for line in lab.ctx.lines("error") if line.startswith("lost a200-0806 joint states")]
    assert outcome.status == HOLD_NOT_SENT and not outcome.ok and len(lost) == 1
    assert outcome.reason.startswith("HOLD NOT SENT on left_ur_arm, right_ur_arm") and "e-stop" in outcome.reason
    assert "not at the end" in outcome.reason and "; held" not in outcome.reason
    assert sum("HOLD NOT SENT" in line for line in lab.ctx.lines("error")) >= 2


def test_a_cancel_holds_every_arm_and_is_raised():
    """Cancelling the waiting task holds both arms in the same step, then raises CancelledError."""
    lab = Lab()
    names, rows = cindy_path()
    command = motion_command("a200-0806", CINDY_ARMS, names, rows, 4.0)

    async def scenario():
        task = asyncio.ensure_future(send_motion(lab.ctx, lab.cindy, command))
        while lab.ctx.t < 11.0:
            await asyncio.sleep(0)
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    assert lab.holds(lab.cindy) == [1, 1]


def test_a_dry_run_runs_every_check_and_publishes_nothing():
    """A dry run builds each trajectory (an arm off its start refuses) and sends nothing either way."""
    lab = Lab()
    names, rows = cindy_path()
    command = motion_command("a200-0806", CINDY_ARMS, names, rows, 2.0)
    outcome = lab.run(lab.cindy, command, dry_run=True)
    assert outcome.status == DRY_RUN and outcome.ok
    banners = [line for line in lab.ctx.lines("warn") if "DRY RUN a200-0806" in line]
    assert len(banners) == 2 and all("NOT SENT" in line for line in banners)
    lab.cindy.arms["right_ur_arm"].state.joint_positions["right_ur_arm_elbow_joint"] += 0.2
    assert lab.run(lab.cindy, command, dry_run=True).status == REFUSED
    assert not lab.cindy._multi_arm.messages and not any(arm._trajectory.messages for arm in lab.cindy.arms.values())


def test_the_record_is_json():
    """A command and an outcome go through JSON."""
    import json
    lab = Lab()
    names, rows = cindy_path()
    command = motion_command("a200-0806", CINDY_ARMS, names, rows, 2.0)
    outcome = lab.run(lab.cindy, command)
    record = json.loads(json.dumps({"command": command.as_record(), "outcome": outcome.as_record()}))
    assert record["command"]["arms"] == list(CINDY_ARMS) and record["outcome"]["status"] == ARRIVED
    assert command.travel == pytest.approx(0.3)
