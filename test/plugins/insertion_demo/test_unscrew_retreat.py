"""The removal with the bar held, on a fake Cindy with her real frames and both tools: its steps and its endings."""

from __future__ import annotations

import asyncio

import numpy as np
import pytest
from planning_fixtures import CINDY, robot_config
from plugin_fakes import (ArmDriver, FakeContext, MocapDriver, ScaffoldingDriver, fake_arm, fake_node, fake_robot,
                          fake_scaffolding, lease_to_test, with_urdf_frames)
from robot_lab import LEFT, RIGHT
from scipy.spatial.transform import Rotation

from husky_assembly_teleop.plugins.insertion_demo.parts import run_unscrew_retreat
from husky_assembly_teleop.robot_interface.arm import CARTESIAN_COMPLIANCE_CONTROLLER as COMPLIANCE
from husky_assembly_teleop.robot_interface.arm import SCALED_JOINT_TRAJECTORY_CONTROLLER as JOINT
from husky_assembly_teleop.robot_interface.end_effectors import ScaffoldingV3
from husky_assembly_teleop.shared.assembly_planning.joints import arm_joint_names
from husky_assembly_teleop.shared.execution.stream import ARRIVAL, FAILED, REFUSED, RETREATED, arm_base_in_world
from husky_assembly_teleop.world.tcp import tool0_in_base_link
from scaffolding_assembly_core.geometry import Pose, compose

pytestmark = pytest.mark.safety

ARMS = {"left_ur_arm": LEFT, "right_ur_arm": RIGHT}
NAMES = arm_joint_names("left_ur_arm") + arm_joint_names("right_ur_arm")
#: The insert the arms made: the elbow closed, shoulder and wrist keeping tool0's turn; about 24 mm, nearly straight.
DELTA = np.array([0.0, -0.03, 0.06, -0.03, 0.0, 0.0] * 2)
#: Where the insert started (the approach), and where the arms stand now, inserted.
START = np.array(LEFT + RIGHT)
INSERTED = START + DELTA
#: Where Cindy's base (the mocap-tracked frame) stands.
BASE = Pose((3.0, 1.0, 0.0), tuple(Rotation.from_euler("z", 30.0, degrees=True).as_quat()))
MOCAP_ID = 6
LOOSEN_S, SECONDS = 1.5, 2.0


class Lab:
    """Cindy inserted (at INSERTED) under joint control, her tools clamped on the bar, both screws stalled; leased.

    Mocap sees the base every tick. Every command sent goes into `ctx.logs` as ("event", "<arm>: <what>"), in order
    with the log lines; `targets` keeps each arm's Cartesian targets, holds left out.
    """

    def __init__(self, lag: float = 0.5, at: np.ndarray = INSERTED, **tool):
        """Build Cindy at joints `at` and her tools; `tool` goes to both ScaffoldingDrivers (the screw never stalls)."""
        self.ctx = FakeContext(step=self.tick)
        node = fake_node(self.ctx.now, self.ctx.logs)
        config = robot_config(CINDY)
        self.urdf = config.model.urdf
        self.tools = {name: fake_scaffolding(node, f"/a200_0806/{name.split('_')[0]}_gripper") for name in ARMS}
        arms = [fake_arm(node, name, np.asarray(at)[6 * k:6 * k + 6], end_effector=self.tools[name])
                for k, name in enumerate(ARMS)]
        for arm in arms:
            arm.state.wrench = np.zeros(6)
        self.robot = with_urdf_frames(lease_to_test(fake_robot(node, "a200-0806", arms)), config)
        self.mocap = MocapDriver(self.robot, BASE, MOCAP_ID)
        self.drivers = [ArmDriver(arm, self.robot, lag=lag) for arm in arms]
        options = {"motor": ScaffoldingV3.STALLED, "screw": ScaffoldingV3.STALLED, "screw_stall_after": None, **tool}
        self.drivers += [ScaffoldingDriver(each, **options) for each in self.tools.values()]
        self.targets: dict[str, list] = {name: [] for name in ARMS}
        self.tick(self.ctx.t)
        for name, arm in self.robot.arms.items():
            self._spy(name, arm)

    def tick(self, t: float) -> None:
        """Mocap sees the base, every arm and tool moves and reports."""
        self.mocap.tick(t)
        for driver in self.drivers:
            driver.tick(t)

    def event(self, what: str) -> None:
        """Keep a command sent, in order with the log lines."""
        self.ctx.logs.append(("event", what))

    def _spy(self, name: str, arm) -> None:
        """Note every command sent to one arm and its tool."""
        tool, publish, switch = self.tools[name], self.tools[name]._command.publish, arm.controllers.switch

        def command(message) -> None:
            publish(message)
            verb = "STOP" if message.direction == 0 else "TIGHTEN" if message.direction > 0 else "LOOSEN"
            self.event(f"{name}: {verb}" + ("" if message.direction == 0 else f" M{message.motor}"))

        def switched(controller: str, unguarded: bool = False) -> bool:
            self.event(f"{name}: switch to {'compliance' if controller == COMPLIANCE else 'JTC'}")
            return switch(controller, unguarded)

        def target(sent) -> None:
            self.event(f"{name}: {'hold' if sent.hold else 'target'}")
            if not sent.hold:
                self.targets[name].append(sent)

        tool._command.publish, arm.controllers.switch = command, switched
        arm._zero_ft.call_async = lambda _request: self.event(f"{name}: zero F/T")
        arm.targets_sent.attach(target)
        arm.wrenches_sent.attach(lambda _wrench: self.event(f"{name}: zero wrench"))
        arm.trajectories_sent.attach(lambda sent: self.event(f"{name}: {'hold' if sent.hold else 'trajectory'}"))

    def events(self) -> list[str]:
        """Every command sent, in order."""
        return [line for kind, line in self.ctx.logs if kind == "event"]

    def steps(self) -> list[str]:
        """Each kind of command, in the order it was first sent."""
        return list(dict.fromkeys(event.split(": ", 1)[-1] for event in self.events()))

    def ends(self) -> dict[str, Pose]:
        """Each tool0 at the insert's start (START), in the world."""
        joints = dict(zip(NAMES, START))
        return {name: compose(arm_base_in_world(self.robot, name), tool0_in_base_link(self.urdf, name, joints))
                for name in ARMS}

    def work(self, loosen_s: float = LOOSEN_S, seconds: float = SECONDS):
        """The removal's coroutine."""
        return run_unscrew_retreat(self.ctx, self.robot, self.tools, self.ends(), loosen_s=loosen_s, seconds=seconds)

    def run(self, **options):
        """Run the removal to its end."""
        return asyncio.run(self.work(**options))

    def world_targets(self, name: str) -> list[Pose]:
        """An arm's targets back in the world, through the base (which stands still)."""
        base = arm_base_in_world(self.robot, name)
        return [compose(base, Pose.from_arrays(sent.position, sent.orientation)) for sent in self.targets[name]]

    def running(self) -> list[str | None]:
        """What each arm's controller manager runs."""
        return [self.robot.arms[arm].controllers.running for arm in ARMS]

    def nothing_sent(self) -> bool:
        """No trajectory, target, wrench, switch or tool command went out."""
        arms = self.robot.arms.values()
        return not (self.robot._multi_arm.messages or any(
            arm._trajectory.messages or arm._target_frame.messages or arm._target_wrench.messages
            or arm.controllers.switches for arm in arms) or any(tool._command.messages for tool in self.tools.values()))


def test_the_removal_clears_zeroes_goes_compliant_loosens_stops_backs_out_and_comes_back():
    """STOP, zero F/T, compliance, zero wrench, LOOSEN M2 for the slider's seconds, STOP, targets, hold, JTC."""
    lab = Lab()
    outcome = lab.run()
    assert outcome.status == RETREATED, outcome.reason
    assert lab.steps() == ["STOP", "zero F/T", "switch to compliance", "zero wrench", "LOOSEN M2", "target", "hold",
                           "switch to JTC"]
    loosen = next(phase for phase in outcome.record["phases"] if phase["name"] == "loosen")
    assert LOOSEN_S - 0.05 <= loosen["t1"] - loosen["t0"] <= LOOSEN_S + 0.15
    # * The motors are stopped before the first target goes out, and the screws were loosening in between.
    events = lab.events()
    assert events.index("right_ur_arm: STOP", events.index("left_ur_arm: LOOSEN M2")) < events.index(
        "left_ur_arm: target")
    assert lab.running() == [JOINT, JOINT]
    for name, end in lab.ends().items():
        np.testing.assert_allclose(lab.world_targets(name)[-1].position, end.position, atol=1e-9)
        assert outcome.record["end_error_mm"][name]["remaining_mm"] <= ARRIVAL * 1e3
    assert all(each["phase"] in ("loosen", "back-out") for each in outcome.record["stream"])


def test_tools_that_never_arrive_still_end_retreated_but_say_so():
    """Arms that do not follow leave every tool0 where it was: RETREATED all the same, short, with a warning."""
    lab = Lab(lag=0.0)
    outcome = lab.run()
    assert outcome.status == RETREATED and "not to the insert's start" in outcome.reason
    assert lab.running() == [JOINT, JOINT]


def test_a_cancel_during_the_back_out_stops_the_tools_and_holds_without_switching():
    """Cancelled after a few targets: STOP both tools, hold each arm, no switch back; then CancelledError."""
    lab = Lab()
    cut = []

    async def scenario():
        task = asyncio.ensure_future(lab.work())
        while sum(event.endswith(": target") for event in lab.events()) < 6 and not task.done():
            await asyncio.sleep(0)
        cut.append(len(lab.events()))
        task.cancel()
        with pytest.raises(asyncio.CancelledError):
            await task

    asyncio.run(scenario())
    after = [event for event in lab.events()[cut[0]:] if not event.endswith(": target")]
    assert after == ["left_ur_arm: STOP", "right_ur_arm: STOP", "left_ur_arm: zero wrench", "left_ur_arm: hold",
                     "right_ur_arm: zero wrench", "right_ur_arm: hold"]
    assert lab.running() == [COMPLIANCE, COMPLIANCE]


@pytest.mark.parametrize("case, why", [
    ("compliant", "is not under scaled_joint_trajectory_controller"),
    ("silent", "no tool status"),
])
def test_a_refused_removal_sends_nothing(case, why):
    """An arm not under joint control, or a tool that does not report: REFUSED before anything goes out."""
    lab = Lab(**({"silent": True} if case == "silent" else {}))
    if case == "compliant":
        lab.robot.arms["left_ur_arm"].state.controllers.active = COMPLIANCE
    outcome = lab.run()
    assert outcome.status == REFUSED and why in outcome.reason and lab.nothing_sent()


def test_a_screw_that_refuses_to_loosen_ends_failed_and_the_arms_come_back():
    """A joint screw still stalled after STOP (its firmware stuck): nothing more; the arms are back under JTC."""
    lab = Lab()
    stuck = lab.drivers[-1]
    stuck.tick = lambda t: None  # its firmware stops answering: no STOP clears it, its status goes stale
    outcome = lab.run()
    assert outcome.status in (REFUSED, FAILED) and lab.running() == [JOINT, JOINT]
    assert not any(event.endswith(": target") for event in lab.events())
